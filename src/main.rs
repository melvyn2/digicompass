#![no_std]
#![no_main]
#![feature(unsafe_cell_access)]
#![feature(string_from_utf8_lossy_owned)]
#![feature(core_float_math)]
#![feature(iter_advance_by)]
#![feature(trivial_bounds)]

extern crate alloc;

use core::cell::OnceCell;
use core::fmt::Write;
use core::mem::{MaybeUninit, discriminant};
use core::panic::PanicInfo;
use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

use adafruit_feather_rp2040_adalogger::hal::clocks::{Clock, init_clocks_and_plls};
use adafruit_feather_rp2040_adalogger::hal::{Sio, pac, watchdog::Watchdog};
use adafruit_feather_rp2040_adalogger::pac::I2C1;
use adafruit_feather_rp2040_adalogger::{Pins, XOSC_CRYSTAL_FREQ};

use bno080::interface::I2cInterface as I2cBno080;
use bno080::wrapper::BNO080;

use cortex_m::asm::wfi;
use cortex_m::interrupt::disable as disable_interrupts;
use cortex_m::peripheral::NVIC;
use cortex_m::prelude::_embedded_hal_adc_OneShot;

use embedded_alloc::LlffHeap;

use embedded_graphics::Drawable;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::geometry::Point;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::mono_font::ascii::FONT_4X6;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::text::{Alignment, Baseline, Text, TextStyleBuilder};

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, PinState};

use embedded_hal_bus::spi::ExclusiveDevice;

use embedded_sdmmc::{SdCard, SdCardError, TimeSource, Timestamp, VolumeIdx, VolumeManager};

use fugit::{MicrosDurationU32, RateExtU32, TimerInstantU64};

use rp2040_hal::adc::AdcPin;
use rp2040_hal::gpio::bank0::{Gpio2, Gpio3, Gpio26, Gpio27};
use rp2040_hal::gpio::{FunctionI2c, FunctionSioInput, Interrupt, Pin, PullNone, PullUp};
use rp2040_hal::pac::interrupt;
use rp2040_hal::pio::PIOExt;
use rp2040_hal::rom_data::reset_to_usb_boot;
use rp2040_hal::timer::Alarm;
use rp2040_hal::usb::UsbBus;
use rp2040_hal::{Adc, I2C, Spi, Timer, halt};

use sh1106::displayrotation::DisplayRotation;
use sh1106::interface::I2cInterface as I2cSh1106;
use sh1106::mode::GraphicsMode;

use usb_device::bus::{PollResult, UsbBus as UsbBusTrait, UsbBusAllocator};

use ws2812_pio::Ws2812Direct;

use crate::data::parse_and_calc;
use crate::gps::GpsMtk;

mod data;
mod gps;
mod leds;
mod locations;
mod ui;
mod usbserial_defmt;

use crate::leds::{leds_off, set_leds_from_direction_distance, startup_ring};
use crate::locations::LocParseError;
use crate::locations::load::load_tree;
use crate::ui::draw_ui;
use crate::usbserial_defmt::{logger_taken, poll_from_interrupt, try_setup_logger};

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    disable_interrupts();

    let mut panic_s = heapless::String::<512>::new();
    if write!(&mut panic_s, "{info}").is_err() {
        let _ = panic_s.push_str("panicked, and also failed to format the panic :(");
    }

    if !logger_taken() {
        defmt::println!("{}", panic_s.as_str());
        defmt::flush();
    }

    // SAFETY: There is only ever one thread running, and it will only run
    // this section and halt now. No risk of other mutable references ever coming back in scope
    #[allow(static_mut_refs)]
    if let Some(d) = unsafe { DISPLAY.as_mut() } {
        // There are 10 lines of 32 characters
        // make space for the 10 newlines in this string
        let mut panic_s_wrapped = heapless::String::<522>::new();
        let mut line_len = 0;
        for c in panic_s.chars() {
            if c == '\n' {
                line_len = 0;
            } else if line_len == 32 {
                let _ = panic_s_wrapped.push('\n');
                let _ = panic_s_wrapped.push(c);
                line_len = 1;
            } else {
                let _ = panic_s_wrapped.push(c);
                line_len += 1;
            }
        }

        d.clear();
        let _ = Text::with_text_style(
            panic_s_wrapped.as_str(),
            Point::new(0, 0),
            MonoTextStyleBuilder::new()
                .font(&FONT_4X6)
                .text_color(BinaryColor::On)
                .build(),
            TextStyleBuilder::new()
                .baseline(Baseline::Top)
                .alignment(Alignment::Left)
                .build(),
        )
        .draw(d);
        let _ = d.flush();
    }

    halt();
}

#[global_allocator]
static ALLOCATOR: LlffHeap = LlffHeap::empty();

/// Call only once
unsafe fn init_heap() {
    // Heap size usages:
    // Mainly for locations, which take 32 bytes (!!) per entry
    // Let's take an entire 64KiB bank
    const HEAP_SIZE: usize = 4096 * 16;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();
    unsafe { ALLOCATOR.init((&raw mut HEAP) as usize, HEAP_SIZE) }
}

// Pinky promise to only use this on this main thread
struct SingleThread<T>(T);
unsafe impl<T> Sync for SingleThread<T> {}

/// Interrupt handlers should do no work except to set these bits, which the main loop will
/// then handle (except for USB interrupts, because USB polling must not miss its deadline)
pub struct InterruptFlags {
    /// 50ms has passed since the last frame, start rendering
    deadline_alarm: AtomicBool,
    /// GPS data is ready to be read
    gps_rx: AtomicBool,
    /// IMU data is ready to be read
    imu_rx: AtomicBool,
}
static INTERRUPT_FLAGS: InterruptFlags = InterruptFlags {
    deadline_alarm: AtomicBool::new(false),
    gps_rx: AtomicBool::new(false),
    imu_rx: AtomicBool::new(false),
};

static USB_BUS_ALLOCATOR: SingleThread<OnceCell<UsbBusAllocator<UsbBus>>> =
    SingleThread(OnceCell::new());

// static needed for panic handler
#[allow(clippy::type_complexity)]
#[rustfmt::skip]
static mut DISPLAY: Option<GraphicsMode<I2cSh1106<I2C<
    I2C1,
    (Pin<Gpio2, FunctionI2c, PullUp>, Pin<Gpio3, FunctionI2c, PullUp>),
>>>> = None;

fn wait_for_usb_bus(bus: &mut UsbBus, delay: &mut Timer, timeout_ms: u32) -> bool {
    bus.enable();
    let mut ready = false;
    for _ in 0..(timeout_ms / 10) {
        if discriminant(&bus.poll()) == discriminant(&PollResult::Reset) {
            ready = true;
            break;
        }
        delay.delay_ms(10)
    }

    ready
}

pub struct FakeTimesource {
    year_since_1970: AtomicU8,
    zero_indexed_month: AtomicU8,
    zero_indexed_day: AtomicU8,
    hours: AtomicU8,
    minutes: AtomicU8,
    seconds: AtomicU8,
}
impl FakeTimesource {
    pub fn set(&self, timestamp: Timestamp) {
        self.year_since_1970
            .store(timestamp.year_since_1970, Ordering::Relaxed);
        self.zero_indexed_month
            .store(timestamp.zero_indexed_month, Ordering::Relaxed);
        self.zero_indexed_day
            .store(timestamp.zero_indexed_day, Ordering::Relaxed);
        self.hours.store(timestamp.hours, Ordering::Relaxed);
        self.minutes.store(timestamp.minutes, Ordering::Relaxed);
        self.seconds.store(timestamp.seconds, Ordering::Relaxed);
    }
}
pub static FAKE_TIMESOURCE: FakeTimesource = FakeTimesource {
    year_since_1970: AtomicU8::new(0),
    zero_indexed_month: AtomicU8::new(0),
    zero_indexed_day: AtomicU8::new(0),
    hours: AtomicU8::new(0),
    minutes: AtomicU8::new(0),
    seconds: AtomicU8::new(0),
};

#[derive(Default)]
pub struct FakeTimesourceReader();
impl TimeSource for FakeTimesourceReader {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: FAKE_TIMESOURCE.year_since_1970.load(Ordering::Relaxed),
            zero_indexed_month: FAKE_TIMESOURCE.zero_indexed_month.load(Ordering::Relaxed),
            zero_indexed_day: FAKE_TIMESOURCE.zero_indexed_day.load(Ordering::Relaxed),
            hours: FAKE_TIMESOURCE.hours.load(Ordering::Relaxed),
            minutes: FAKE_TIMESOURCE.minutes.load(Ordering::Relaxed),
            seconds: FAKE_TIMESOURCE.seconds.load(Ordering::Relaxed),
        }
    }
}

// From Timer::get_counter()
fn get_ticks() -> TimerInstantU64<1_000_000> {
    // Safety: Only used for reading current timer value
    let timer = unsafe { &*pac::TIMER::PTR };
    let mut hi0 = timer.timerawh().read().bits();
    let timestamp = loop {
        let low = timer.timerawl().read().bits();
        let hi1 = timer.timerawh().read().bits();
        if hi0 == hi1 {
            break (u64::from(hi0) << 32) | u64::from(low);
        }
        hi0 = hi1;
    };
    TimerInstantU64::from_ticks(timestamp)
}
defmt::timestamp!("{:ms}", get_ticks().duration_since_epoch().to_millis());

pub fn bat_raw(
    adc: &mut Adc,
    delay: &mut Timer,
    vbat_pin: &mut AdcPin<Pin<Gpio26, FunctionSioInput, PullNone>>,
    vref_pin: Pin<Gpio27, FunctionSioInput, PullNone>,
) -> (Pin<Gpio27, FunctionSioInput, PullNone>, (u16, (u16, u16))) {
    let vbat = adc.read(vbat_pin).unwrap();

    let mut vhigh_pin = AdcPin::new(vref_pin.into_pull_up_input()).unwrap();
    delay.delay_ms(5);
    let vhigh = adc.read(&mut vhigh_pin).unwrap();

    let mut vlow_pin = AdcPin::new(vhigh_pin.release().into_pull_down_input()).unwrap();
    delay.delay_ms(5);
    let vlow = adc.read(&mut vlow_pin).unwrap();

    let vref_pin = vlow_pin.release().into_floating_input();

    (vref_pin, (vbat, (vlow, vhigh)))
}

//noinspection ALL
#[rp2040_hal::entry]
fn main() -> ! {
    unsafe {
        init_heap();
    }

    // Init base peripherals
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let mut clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Open USB bus early to allow computer detection
    let mut usb_bus = UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    );

    // Setup ADC pins to measure battery
    let mut vbat_pin = AdcPin::new(pins.a0.into_floating_input()).unwrap();
    let mut vref_pin = pins.a1.into_floating_input();

    let mut bat_measure = {
        let res = bat_raw(&mut adc, &mut timer, &mut vbat_pin, vref_pin);
        vref_pin = res.0;
        res.1
    };

    // Init LED ring
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812Direct::new_sk6812(
        pins.d12.into_function(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
    );
    leds_off(&mut ws);

    // Init SD card
    let volume_manager = {
        let spi_cs = pins.sd_cs.into_push_pull_output();
        let spi_sck = pins.sd_clk.into_function();
        let spi_mosi = pins.sd_mosi.into_function();
        let spi_miso = pins.sd_miso.into_function();
        let spi_bus = Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sck)).init(
            &mut pac.RESETS,
            clocks.peripheral_clock.freq(),
            400.kHz(), // card initialization happens at low baud rate
            embedded_hal::spi::MODE_0,
        );
        let spi = ExclusiveDevice::new(spi_bus, spi_cs, timer).unwrap();
        let sdcard = SdCard::new(spi, timer);
        match sdcard.num_bytes() {
            Ok(_) => Some(VolumeManager::new(sdcard, FakeTimesourceReader::default())),
            Err(SdCardError::CardNotFound) => None,
            Err(e) => panic!("{e:?}"),
        }
    };

    // Init display
    let mut display_rst = pins.d5.into_push_pull_output_in_state(PinState::High);
    timer.delay_ms(50);

    let disp_i2c = I2C::i2c1(
        pac.I2C1,
        pins.sda.reconfigure(),
        pins.scl.reconfigure(),
        400.kHz(),
        &mut pac.RESETS,
        125_000_000.Hz(),
    );
    let mut display: GraphicsMode<_> = sh1106::Builder::new()
        .with_rotation(DisplayRotation::Rotate180)
        .connect_i2c(disp_i2c)
        .into();

    display.init().unwrap();
    DrawTarget::clear(&mut display, BinaryColor::On).unwrap();
    display.flush().unwrap();

    let display = unsafe {
        DISPLAY = Some(display);
        // SAFETY: this reference is the only one in existence until a panic occurs,
        // at which point this one will no longer exist
        #[allow(static_mut_refs)]
        DISPLAY.as_mut().unwrap()
    };

    // Optionally init usb-serial logger
    if wait_for_usb_bus(&mut usb_bus, &mut timer, 250) {
        draw_ui(
            display,
            "5s timeout",
            "Waiting for logger",
            None,
            None,
            None,
            None,
        );
        USB_BUS_ALLOCATOR
            .0
            .set(UsbBusAllocator::new(usb_bus))
            .map_err(|_| ())
            .unwrap();
        let usb_alloc_ref = USB_BUS_ALLOCATOR.0.get().unwrap();
        if try_setup_logger(usb_alloc_ref, &mut timer, 5000) {
            unsafe {
                NVIC::unmask(pac::Interrupt::USBCTRL_IRQ);
            }
        }
    } else {
        // Return used resources
        (pac.USBCTRL_REGS, pac.USBCTRL_DPRAM, clocks.usb_clock) = usb_bus.free(&mut pac.RESETS)
    }

    defmt::trace!("defmt enabled");
    defmt::flush();

    // Load the locations tree from the SD card, if present
    draw_ui(display, "Loading", "Locations list", None, None, None, None);
    let tree = volume_manager.as_ref().and_then(|vm| {
        defmt::debug!("sdcard size is {:?}", vm.device(|d| d.num_bytes()));
        let vol = vm.open_raw_volume(VolumeIdx(0)).unwrap();
        let dir = vm.open_root_dir(vol).unwrap();
        match load_tree(vm, dir) {
            Ok(t) => Some(t),
            Err(LocParseError::SourceMissing) => None,
            Err(e) => panic!("{e:?}"),
        }
    });
    defmt::flush();

    defmt::trace!("Attempting to initalize IMU");
    let imu_i2c = I2C::i2c0(
        pac.I2C0,
        pins.d24.reconfigure(),
        pins.d25.reconfigure(),
        400.kHz(),
        &mut pac.RESETS,
        125_000_000.Hz(),
    );
    let mut imu = BNO080::new_with_interface(I2cBno080::default(imu_i2c));
    imu.init(&mut timer).unwrap();
    imu.enable_rotation_vector(50).unwrap();
    imu.enable_shake_detection(50).unwrap();
    let mut imu_int = pins.sck.into_pull_up_input();
    imu_int.set_interrupt_enabled(Interrupt::EdgeLow, true);
    unsafe { NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0) };
    defmt::info!("Initialized IMU");

    draw_ui(display, "GPS", "connecting", None, None, Some(false), None);
    let mut gps_en = pins.d4.into_push_pull_output_in_state(PinState::High);
    let mut gps = GpsMtk::new(
        pac.UART0,
        (pins.tx.into_function(), pins.rx.into_function()),
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
    )
    .unwrap();
    // while gps.stale != 0 {
    //     gps.update().unwrap();
    // }
    gps.uart.enable_rx_interrupt();
    unsafe { NVIC::unmask(pac::Interrupt::UART0_IRQ) };

    // defmt::info!(
    //     "Got first GPS fix {} {}",
    //     gps.pos[0].get::<degree>(),
    //     gps.pos[1].get::<degree>()
    // );
    defmt::flush();

    // Greet user
    draw_ui(
        display,
        "DIGICOMPASS",
        "Donebog, Inc.",
        None,
        None,
        None,
        None,
    );
    startup_ring(&mut ws, &mut timer);

    // Now the main loop
    let mut deadline_alarm = timer.alarm_0().unwrap();
    deadline_alarm.enable_interrupt();
    unsafe { NVIC::unmask(pac::Interrupt::TIMER_IRQ_0) }

    // Once awake stay for at least 30 sec
    const WAKE_TICKS: u32 = 30 * 20;
    let mut ticks_until_sleep = WAKE_TICKS;
    // let mut orientation_quat = UnitQuaternion::identity();
    let mut prev_imu_timestamp = timer.get_counter().ticks();

    // Measure battery every 60 sec
    const BAT_MEASURE_TIME: u32 = 60 * 20;
    let mut frames_until_bat = BAT_MEASURE_TIME;

    let mut name_cache = None;

    defmt::info!("Starting main loop");
    defmt::flush();

    loop {
        let timestamp = timer.get_counter().ticks();

        if INTERRUPT_FLAGS.gps_rx.load(Ordering::Relaxed) || gps.uart.uart_is_readable() {
            INTERRUPT_FLAGS.gps_rx.store(false, Ordering::Relaxed);
            gps.update().unwrap();
        }

        if INTERRUPT_FLAGS.imu_rx.load(Ordering::Relaxed) || imu_int.is_low().unwrap() {
            INTERRUPT_FLAGS.imu_rx.store(false, Ordering::Relaxed);
            imu.handle_all_messages(&mut timer, 10);

            // defmt::trace!("IMU update");

            // infalliable
            if imu.take_shaken().unwrap().is_some() {
                // Wake up devices if currently sleeping
                if ticks_until_sleep == 0 {
                    // display_rst.set_high().unwrap();
                    gps.test().unwrap();
                    imu.enable_rotation_vector(50).unwrap();
                    // TODO check if GPS ready

                    // Start frame in 50ms from now (for device init)
                    deadline_alarm.clear_interrupt();
                    deadline_alarm
                        .schedule(MicrosDurationU32::millis(50))
                        .unwrap();
                }
                // Reset sleep timer
                ticks_until_sleep = WAKE_TICKS;
            } else if ticks_until_sleep != 0 {
                // ticks_until_sleep =
                // ticks_until_sleep.saturating_sub((timestamp - prev_imu_timestamp) as u32);
                // Enter sleep now if timer finishes
                if ticks_until_sleep == 0 {
                    // Stop rendering
                    // Infallible
                    deadline_alarm.cancel().unwrap();
                    INTERRUPT_FLAGS
                        .deadline_alarm
                        .store(false, Ordering::Relaxed);

                    // Turn off devices
                    // display_rst.set_low().unwrap();
                    gps.enter_standby();
                    imu.enable_rotation_vector(0).unwrap();
                    INTERRUPT_FLAGS.gps_rx.store(false, Ordering::Relaxed);
                }
            }
            // orientation_quat = next_quat;
            prev_imu_timestamp = timestamp;
        }

        if INTERRUPT_FLAGS.deadline_alarm.load(Ordering::Relaxed) || deadline_alarm.finished() {
            // defmt::trace!("Start frame render");
            defmt::flush();
            INTERRUPT_FLAGS
                .deadline_alarm
                .store(false, Ordering::Relaxed);
            deadline_alarm.clear_interrupt();
            deadline_alarm
                .schedule(MicrosDurationU32::millis(50))
                .unwrap();

            if frames_until_bat == 0 {
                bat_measure = {
                    let res = bat_raw(&mut adc, &mut timer, &mut vbat_pin, vref_pin);
                    vref_pin = res.0;
                    res.1
                };
                frames_until_bat = BAT_MEASURE_TIME;
            } else {
                frames_until_bat -= 1;
            }

            let data = parse_and_calc(
                // Anglef32::new::<radian>(orientation_quat.euler_angles().2),
                // infallible
                imu.rotation_quaternion().unwrap(),
                gps.pos,
                gps.hgt,
                gps.date,
                bat_measure,
                volume_manager.as_ref().zip(tree.as_ref()),
                name_cache,
            )
            .unwrap();

            set_leds_from_direction_distance(&mut ws, data.bearing_rad, data.dist_m);

            // make cross blink when not ok
            let show_gps_ok = gps.stale == 0 || ((timestamp / 1_000_000) % 2 == 1);

            draw_ui(
                display,
                data.name.as_str(),
                data.dist.as_str(),
                Some(data.bat),
                Some(data.true_heading_rad),
                Some(show_gps_ok),
                Some(gps.date),
            );
            name_cache = data.name_cache_tag.map(|nct| (nct, data.name));
        }

        wfi()
    }
}

#[interrupt]
fn USBCTRL_IRQ() {
    poll_from_interrupt();
}

#[interrupt]
fn TIMER_IRQ_0() {
    INTERRUPT_FLAGS
        .deadline_alarm
        .store(true, Ordering::Relaxed);
}

#[interrupt]
fn UART0_IRQ() {
    INTERRUPT_FLAGS.gps_rx.store(true, Ordering::Relaxed);
}

#[interrupt]
fn IO_IRQ_BANK0() {
    // The only GPIO interrupt is for the IMU
    INTERRUPT_FLAGS.imu_rx.store(true, Ordering::Relaxed);
}
