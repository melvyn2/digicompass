use core::cell::UnsafeCell;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, Ordering};

use critical_section::RestoreState;

use defmt::{Encoder, global_logger};
use embedded_hal::delay::DelayNs;
use embedded_io::ReadReady;
use rp2040_hal::Timer;
use rp2040_hal::usb::UsbBus;

use usb_device::UsbError;
use usb_device::bus::UsbBusAllocator;
use usb_device::device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid};

use usbd_serial::{SerialPort, USB_CLASS_CDC};

#[global_logger]
struct GlobalSerialLogger;

unsafe impl defmt::Logger for GlobalSerialLogger {
    fn acquire() {
        SERIAL_LOGGER.acquire()
    }

    unsafe fn flush() {
        unsafe { SERIAL_LOGGER.flush() }
    }

    unsafe fn release() {
        unsafe { SERIAL_LOGGER.release() }
    }

    unsafe fn write(b: &[u8]) {
        unsafe { SERIAL_LOGGER.write_encoded(b) }
    }
}

pub static SERIAL_LOGGER: SerialLogger = SerialLogger::new();

pub struct SerialLogger {
    setup: AtomicBool,
    taken: AtomicBool,
    cs_restore: UnsafeCell<RestoreState>,
    encoder: UnsafeCell<Encoder>,
    serial_dev: UnsafeCell<MaybeUninit<(SerialPort<'static, UsbBus>, UsbDevice<'static, UsbBus>)>>,
}

impl SerialLogger {
    const fn new() -> Self {
        Self {
            setup: AtomicBool::new(false),
            taken: AtomicBool::new(false),
            cs_restore: UnsafeCell::new(RestoreState::invalid()),
            encoder: UnsafeCell::new(Encoder::new()),
            serial_dev: UnsafeCell::new(MaybeUninit::uninit()),
        }
    }

    fn try_setup(
        &'static self,
        bus: &'static UsbBusAllocator<UsbBus>,
        delay: &mut Timer,
        timeout_ms: u32,
    ) -> bool {
        critical_section::with(|_| {
            if self.setup.load(Ordering::Acquire) {
                panic!("Tried to setup up serial logger twice");
            }

            let mut serial = SerialPort::new(bus);
            let mut usb_dev = UsbDeviceBuilder::new(bus, UsbVidPid(0x239A, 0x815E))
                .strings(&[StringDescriptors::default()
                    .manufacturer("Donebog, Inc.")
                    .product("Digicompass Debug Port")
                    .serial_number("00001")])
                .unwrap()
                .device_class(USB_CLASS_CDC)
                .build();

            // Wait for connection if requested
            let mut ready = false;
            for _ in 0..(timeout_ms / 5) {
                if serial.dtr() {
                    ready = true;
                    break;
                }
                usb_dev.poll(&mut [&mut serial]);
                delay.delay_ms(5);
            }

            if ready {
                // SAFETY: ditto
                unsafe {
                    self.serial_dev.as_mut_unchecked().write((serial, usb_dev));
                    self.write_all(b"setup finished\r\n");
                }
                self.setup.store(true, Ordering::Relaxed);
            }
            ready
        })
    }

    fn acquire(&self) {
        // if !self.setup.load(Ordering::Acquire) {
        //     return;
        // }

        // SAFETY: token is released in corresponding `release()` call
        let restore = unsafe { critical_section::acquire() };

        // Unfortunately compare_exchange is not available, but critical section
        // prevents any writes for happening between load-store
        critical_section::with(|_| {
            if self.taken.load(Ordering::Relaxed) {
                panic!("serial logger taken reentrantly")
            }
            self.taken.store(true, Ordering::Relaxed);
        });

        // SAFETY: Lock acquired here, fine to write
        unsafe {
            self.cs_restore.replace(restore);
            self.encoder
                .as_mut_unchecked()
                .start_frame(|b| self.write_all(b));
        }
    }

    /// SAFETY: Must be called in between acquire-release
    pub unsafe fn write_all(&self, mut b: &[u8]) {
        if !self.setup.load(Ordering::Acquire) {
            return;
        }

        // SAFETY: self is locked by caller so we have exclusive access,
        // and is setup so the data is init
        let (port, dev) = unsafe { self.serial_dev.as_mut_unchecked().assume_init_mut() };

        while !b.is_empty() {
            match port.write(b) {
                Ok(len) => b = &b[len..],
                Err(UsbError::WouldBlock) => {
                    dev.poll(&mut [port]);
                }
                Err(e) => panic!("{e:?}"),
            }
        }
    }

    /// SAFETY: Must be called in between acquire-release
    unsafe fn write_encoded(&self, bytes: &[u8]) {
        if !self.setup.load(Ordering::Acquire) {
            return;
        }

        unsafe {
            self.encoder
                .as_mut_unchecked()
                .write(bytes, |b| self.write_all(b))
        }
    }

    /// SAFETY: Must be called in between acquire-release
    unsafe fn flush(&self) {
        if !self.setup.load(Ordering::Acquire) {
            return;
        }
        // SAFETY: self is locked by caller so we have exclusive access,
        // and is setup so the data is init
        let (port, dev) = unsafe { self.serial_dev.as_mut_unchecked().assume_init_mut() };
        loop {
            match port.flush() {
                Ok(()) => return,
                Err(UsbError::WouldBlock) => {
                    dev.poll(&mut [port]);
                }
                Err(e) => panic!("{e:?}"),
            }
        }
    }

    /// SAFETY: Must be paired with exactly one previous acquire call
    unsafe fn release(&self) {
        if !self.setup.load(Ordering::Acquire) {
            return;
        }

        if !self.taken.load(Ordering::Relaxed) {
            panic!("serial logger release out of context")
        }

        // SAFETY: in lock & critical section, exclusive access to self
        unsafe {
            let encoder: &mut Encoder = self.encoder.as_mut_unchecked();
            encoder.end_frame(|b| self.write_all(b));
            // let restore = self.cs_restore.get().read();
            self.taken.store(false, Ordering::Release);
            // SAFETY: caller's responsibility to pair this with a previous acquire
            // critical_section::release(restore);
        }
        // unsafe { self.write_all(b"release\r\n") };
    }
}

// SAFETY: Manually lock internals
unsafe impl Sync for SerialLogger {}

/// Ensure that `USBCTRL_IRQ` is unmasked after setup, or the port may hang
#[inline]
pub fn try_setup_logger(
    bus: &'static UsbBusAllocator<UsbBus>,
    delay: &mut Timer,
    timeout_ms: u32,
) -> bool {
    SERIAL_LOGGER.try_setup(bus, delay, timeout_ms)
}

/// Used to check if the logger can be used during a panic
pub fn logger_taken() -> bool {
    SERIAL_LOGGER.taken.load(Ordering::Acquire)
}

pub fn poll_from_interrupt() {
    if !SERIAL_LOGGER.setup.load(Ordering::Acquire) {
        return;
    }

    if SERIAL_LOGGER.taken.load(Ordering::Acquire) {
        panic!("USB interrupt while logger is locked! (Should be in critical section)");
    }

    // SAFETY: Ensured that data is init and not in use. refs only last until the end of the
    // function, and do not conflict with other usages
    critical_section::with(|_| {
        let (port, dev) = unsafe {
            SERIAL_LOGGER
                .serial_dev
                .as_mut_unchecked()
                .assume_init_mut()
        };
        dev.poll(&mut [port]);

        // discard incoming data
        let mut buf = [0u8; 128];
        while port.read_ready().unwrap() {
            match port.read(&mut buf) {
                Err(UsbError::WouldBlock) => break,
                Ok(_) => {}
                Err(e) => panic!("{e:?}"),
            }
        }
    });
}
