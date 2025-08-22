use alloc::string::{FromUtf8Error, String, ToString};
use alloc::vec;
use alloc::vec::Vec;

use core::num::{ParseFloatError, ParseIntError};
use core::str::Utf8Error;

use cortex_m::asm::delay;

use defmt::Format;

use embedded_io::Write;

use fugit::HertzU32;

use nb::Error as NbError;

use rp2040_hal::pac::RESETS;
use rp2040_hal::uart::{
    DataBits, Enabled, ReadError, ReadErrorType, StopBits, UartConfig, UartDevice, UartPeripheral,
    ValidUartPinout,
};

use thiserror::Error;

use uom::ConstZero;
use uom::si::angle::{degree, minute};
use uom::si::f32::Length;
use uom::si::f64::{Angle, Time};
use uom::si::length::meter;
use uom::si::time::hour;

use world_magnetic_model::time::error::ComponentRange;
use world_magnetic_model::time::{Date, Month};

#[derive(Debug, Format, Error)]
pub enum GpsError {
    #[error("invalid start of NMEA sentence: {0} instead of !")]
    InvalidStart(char),
    #[error("NMEA sentence exceeded 82-byte limit")]
    TooLong,
    #[error("NMEA checksum out of hex range")]
    InvalidChecksum,
    #[error("NMEA checksum does not match data")]
    WrongChecksum,
    #[error("invalid end of NMEA sentence")]
    InvalidEnd,
    #[error("received unexpected command: {0}")]
    UnexpectedCommand(String),
    #[error("received wrong ACK parameter: expected {0} got {1}")]
    UnexpectedParameter(u16, u16),
    #[error("received malformed response")]
    MalformedResponse,
    #[error("did not receive ACK within 10 commands")]
    NoAck,
    #[error("command {0} failed with status {1}")]
    AckFail(u16, u8),
    #[error("failed to read UART: {0:?}")]
    ReadError(ReadErrorType),
    #[error("timed out reading response")]
    ReadTimeout,
}

impl From<ReadErrorType> for GpsError {
    fn from(value: ReadErrorType) -> Self {
        Self::ReadError(value)
    }
}

impl From<Utf8Error> for GpsError {
    fn from(_value: Utf8Error) -> Self {
        Self::MalformedResponse
    }
}

impl From<FromUtf8Error> for GpsError {
    fn from(_value: FromUtf8Error) -> Self {
        Self::MalformedResponse
    }
}

impl From<ParseIntError> for GpsError {
    fn from(_value: ParseIntError) -> Self {
        Self::MalformedResponse
    }
}

impl From<ParseFloatError> for GpsError {
    fn from(_value: ParseFloatError) -> Self {
        Self::MalformedResponse
    }
}

impl From<ComponentRange> for GpsError {
    fn from(_value: ComponentRange) -> Self {
        Self::MalformedResponse
    }
}

pub struct GpsMtk<D: UartDevice, P: ValidUartPinout<D>> {
    pub uart: UartPeripheral<Enabled, D, P>,
    /// Number of updates since last fix
    pub stale: u8,
    /// Latitude, longitude
    pub pos: [Angle; 2],
    /// Height above the ellipsoid
    pub hgt: Length,
    pub date: Date,
    pub time: Time,
}

impl<D: UartDevice, P: ValidUartPinout<D>> GpsMtk<D, P> {
    pub fn new(
        uart: D,
        pins: P,
        resets: &mut RESETS,
        pclk_freq: HertzU32,
        // delay: &mut Timer,
    ) -> Result<Self, GpsError> {
        let mut fast_link = UartPeripheral::new(uart, pins, resets)
            .enable(
                UartConfig::new(HertzU32::Hz(57600), DataBits::Eight, None, StopBits::One),
                pclk_freq,
            )
            .unwrap();
        let mut ret = Self {
            uart: fast_link,
            stale: u8::MAX,
            pos: [Angle::ZERO; 2],
            hgt: Length::ZERO,
            date: Date::MIN,
            time: Time::ZERO,
        };

        defmt::trace!("Testing fast link");
        if ret.discard_until_end().is_err() || defmt::dbg!(ret.read_sentence_blocking()).is_err() {
            defmt::debug!("GPS fast link failed, trying slow link");

            let (uart, pins) = ret.uart.free();
            let slow_link = UartPeripheral::new(uart, pins, resets)
                .enable(
                    UartConfig::new(HertzU32::Hz(9600), DataBits::Eight, None, StopBits::One),
                    pclk_freq,
                )
                .unwrap();
            let mut slow_self = Self {
                uart: slow_link,
                stale: u8::MAX,
                pos: [Angle::ZERO; 2],
                hgt: Length::ZERO,
                date: Date::MIN,
                time: Time::ZERO,
            };

            slow_self.discard_until_end()?;
            defmt::trace!("Slow link worked, upgrading");
            slow_self.write_sentence_noack(b"PMTK251,57600");

            let (uart, pins) = slow_self.uart.free();
            fast_link = UartPeripheral::new(uart, pins, resets)
                .enable(
                    UartConfig::new(HertzU32::Hz(57600), DataBits::Eight, None, StopBits::One),
                    pclk_freq,
                )
                .unwrap();
            ret = Self {
                uart: fast_link,
                stale: u8::MAX,
                pos: [Angle::ZERO; 2],
                hgt: Length::ZERO,
                date: Date::MIN,
                time: Time::ZERO,
            };
            // delay.delay_ms(100);
            defmt::trace!("Testing fast link again");
            ret.discard_until_end()?;
            ret.read_sentence_blocking()?;
        }

        defmt::info!("Connected to MTKGPS on fast link");
        // Fast link works, finish init
        // Hot start
        // ret.write_sentence(b"PMTK101")?;
        // Max update rate (100ms/10Hz)
        ret.write_sentence(b"PMTK220,500")?;
        // Enable SBAS (some error correction thing)
        // seems unsupported
        // ret.write_sentence(b"PMTK313,1")?;
        // Enable AIC (anti-jam)
        ret.write_sentence(b"PMTK286,1")?;
        // Set periodic normal mode (??)
        ret.write_sentence(b"PMTK225,0")?;
        // Enable RMC and GGA
        ret.write_sentence(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")?;
        defmt::info!("Finished GPS init");
        Ok(ret)
    }

    fn discard_until_end(&mut self) -> Result<(), GpsError> {
        let mut cb = [0];
        for _ in 0..200 {
            match self.read_blocking_timeout(&mut cb) {
                Ok(_) => {
                    if cb[0] == b'\n' {
                        return Ok(());
                    }
                }
                Err(GpsError::ReadTimeout) => return Ok(()),
                Err(GpsError::ReadError(ReadErrorType::Overrun)) => continue,
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }

    fn parse_time_lat_long(
        time_s: &str,
        lat_s: &str,
        ns_s: &str,
        long_s: &str,
        ew_s: &str,
    ) -> Result<(Time, Angle, Angle), GpsError> {
        let time = Time::new::<hour>(time_s[..2].parse()?)
            + Time::new::<uom::si::time::minute>(time_s[2..4].parse()?)
            + Time::new::<uom::si::time::second>(time_s[4..].parse()?);

        let ns = match ns_s {
            "N" => 1.0,
            "S" => -1.0,
            _ => return Err(GpsError::MalformedResponse),
        };
        let lat = (Angle::new::<degree>(lat_s[..2].parse()?)
            + Angle::new::<minute>(lat_s[2..].parse()?))
            * ns;

        let ew = match ew_s {
            "E" => 1.0,
            "W" => -1.0,
            _ => return Err(GpsError::MalformedResponse),
        };
        let long = (Angle::new::<degree>(long_s[..3].parse()?)
            + Angle::new::<minute>(long_s[3..].parse()?))
            * ew;

        Ok((time, lat, long))
    }

    pub fn update(&mut self) -> Result<(), GpsError> {
        // defmt::trace!("Attempting GPS update");
        let sentence = match self.read_sentence_blocking() {
            Ok(s) => String::from_utf8(s)?,
            // Ignore one overrun
            Err(GpsError::ReadError(ReadErrorType::Overrun) | GpsError::TooLong) => {
                self.discard_until_end()?;
                String::from_utf8(self.read_sentence_blocking()?)?
            }
            Err(e) => return Err(e),
        };
        let mut splits = sentence.split(',');
        use GpsError::MalformedResponse as MR;
        // Skip first two chars = sender
        match splits.next().unwrap() {
            "GPGGA" => {
                // Ex. GPGGA,053103.294,,,,,0,0,,,M,,M,,
                let time_s = splits.next().ok_or(MR)?;
                let lat_s = splits.next().ok_or(MR)?;
                let ns_s = splits.next().ok_or(MR)?;
                let long_s = splits.next().ok_or(MR)?;
                let ew_s = splits.next().ok_or(MR)?;
                let fix_s = splits.next().ok_or(MR)?;
                let _sats_s = splits.next().ok_or(MR)?;
                let _hdop_s = splits.next().ok_or(MR)?;
                let alt_s = splits.next().ok_or(MR)?;
                let alt_unit_s = splits.next().ok_or(MR)?;
                let geosep_s = splits.next().ok_or(MR)?;
                let geosep_units_s = splits.next().ok_or(MR)?;
                // let _dgps_age_s = splits.next().ok_or(MR)?;

                if fix_s == "0" {
                    self.stale = self.stale.saturating_add(1);
                    return Ok(());
                }

                let (time, lat, long) =
                    Self::parse_time_lat_long(time_s, lat_s, ns_s, long_s, ew_s)?;

                let hgt = Length::new::<meter>(alt_s.parse::<f32>()? + geosep_s.parse::<f32>()?);
                if alt_unit_s != "M" || geosep_units_s != "M" {
                    return Err(MR);
                }

                self.pos = [lat, long];
                self.hgt = hgt;
                self.time = time;
                self.stale = 0;

                Ok(())
            }
            "GPRMC" => {
                // Ex. GPRMC,235944.097,V,,,,,0.00,0.00,050180,,,N
                let time_s = splits.next().ok_or(MR)?;
                let status_s = splits.next().ok_or(MR)?;
                let lat_s = splits.next().ok_or(MR)?;
                let ns_s = splits.next().ok_or(MR)?;
                let long_s = splits.next().ok_or(MR)?;
                let ew_s = splits.next().ok_or(MR)?;
                let _gnd_speed_s = splits.next().ok_or(MR)?;
                let _course_s = splits.next().ok_or(MR)?;
                let date_s = splits.next().ok_or(MR)?;
                // let _mag_var_s = splits.next().ok_or(MR)?;
                // splits.advance_by(2).map_err(|_| MR)?;
                // let _mode_s = splits.next().ok_or(MR)?;

                if status_s == "V" {
                    self.stale = self.stale.saturating_add(1);
                    return Ok(());
                }

                let (time, lat, long) =
                    Self::parse_time_lat_long(time_s, lat_s, ns_s, long_s, ew_s)?;

                let date = Date::from_calendar_date(
                    2000 + date_s[4..].parse::<i32>()?,
                    Month::try_from(date_s[2..4].parse::<u8>()?)?,
                    date_s[..2].parse()?,
                )?;

                self.pos = [lat, long];
                self.time = time;
                self.date = date;
                self.stale = 0;

                Ok(())
            }
            other => Err(GpsError::UnexpectedCommand(other.to_string())),
        }
    }

    pub fn enter_standby(&mut self) {
        self.write_sentence_noack(b"PMTK161,0");
    }

    /// Check if link is working. Can be used to wakeup from standby.
    pub fn test(&mut self) -> Result<(), GpsError> {
        self.write_sentence(b"PMTK000")
    }

    /// Takes command like `b"PMTK000"`, writes `b"$PMTK000*32\r\n"`, and blocks for and checks ACK
    fn write_sentence(&mut self, cmd: &[u8]) -> Result<(), GpsError> {
        self.write_sentence_noack(cmd);
        if cmd.starts_with(b"PMTK") {
            let cmd_num = str::from_utf8(&cmd[4..7])?.parse::<u16>()?;
            // discard up to 9 commands
            for _ in 0..20 {
                let ack = self.read_sentence_blocking()?;
                let mut splits = ack.split(|&c| c == b',');
                // Infallible, split always returns at least 1
                let cmd = splits.next().unwrap();
                if cmd != b"PMTK001" {
                    defmt::debug!("discarding non-ack {}", String::from_utf8_lossy_owned(ack));
                    continue;
                }
                let ack_num = splits
                    .next()
                    .ok_or(GpsError::MalformedResponse)
                    .and_then(|t| Ok(str::from_utf8(t)?.parse::<u16>()?))?;
                if ack_num != cmd_num {
                    return Err(GpsError::UnexpectedParameter(cmd_num, ack_num));
                }
                let status_num = splits
                    .next()
                    .ok_or(GpsError::MalformedResponse)
                    .and_then(|t| Ok(str::from_utf8(t)?.parse::<u8>()?))?;
                if status_num != 3 {
                    return Err(GpsError::AckFail(cmd_num, status_num));
                }
                defmt::debug!("GPS command {} succeeded", cmd_num);
                return Ok(());
                // ignore startup messages
                // if cmd == b"CDACK" || cmd == b"PMTK010" || cmd == b"PMTK011" {
                //     continue;
                // } else if cmd != b"PMTK001" {
                //     return Err(GpsError::UnexpectedCommand(
                //         String::from_utf8_lossy(cmd).to_string(),
                //     ));
                // }
                // break splits;
            }
            // if splits.next() != Some(cmd_num) {
            //     return Err(GpsError::UnexpectedParameter);
            // }
            // if splits.next() != Some(b"0") {
            //     return Err(GpsError::AckFail);
            // }
        }
        Err(GpsError::NoAck)
    }

    /// skips waiting for ack
    fn write_sentence_noack(&mut self, cmd: &[u8]) {
        // add 6 for: $, *, checksum byte 1, checksum byte 2, \r, \n
        let checksum = cmd.iter().fold(0, |cs, ch| cs ^ ch);

        let checksum_first = (checksum & 0xF0) >> 4;
        let checksum_first_byte = match checksum_first {
            0x0..=0x9 => b'0' + checksum_first,
            0xA..=0xF => b'A' - 0xA + checksum_first,
            _ => unreachable!(),
        };

        let checksum_second = checksum & 0x0F;
        // Manually calculate because char::from_digit does lowercase ASCII
        let checksum_second_byte = match checksum_second {
            0x0..=0x9 => b'0' + checksum_second,
            0xA..=0xF => b'A' - 0xA + checksum_second,
            _ => unreachable!(),
        };

        self.uart.write_full_blocking(b"$");
        self.uart.write_full_blocking(cmd);
        self.uart.write_full_blocking(&[
            b'*',
            checksum_first_byte,
            checksum_second_byte,
            b'\r',
            b'\n',
        ]);

        // Infallible
        self.uart.flush().unwrap();
    }

    fn read_blocking_timeout(&mut self, buffer: &mut [u8]) -> Result<(), GpsError> {
        let mut attempts = 0;
        let mut offset = 0;

        // delay 1000 cycles per attempt, 10_000 times is 10/125 of a second
        while offset != buffer.len() && attempts <= 100_000 {
            match self.uart.read_raw(&mut buffer[offset..]) {
                Ok(bytes_read) => {
                    attempts = 0;
                    offset += bytes_read;
                }
                Err(e) => match e {
                    NbError::WouldBlock
                    | NbError::Other(ReadError {
                        err_type: ReadErrorType::Break | ReadErrorType::Framing,
                        ..
                    }) => {
                        attempts += 1;
                        delay(1_000)
                    }
                    NbError::Other(inner) => return Err(inner.err_type.into()),
                },
            }
        }

        if attempts == 0 {
            Ok(())
        } else {
            Err(GpsError::ReadTimeout)
        }
    }

    fn read_sentence_maybe(&mut self) -> Result<Option<Vec<u8>>, GpsError> {
        if !self.uart.uart_is_readable() {
            return Ok(None);
        }
        self.read_sentence_blocking().map(Some)
    }

    /// At 115200 baud, this should take maximum (1/115200) * 8 * 82 = 5.7 ms to read
    fn read_sentence_blocking(&mut self) -> Result<Vec<u8>, GpsError> {
        let mut char_buf = [0];
        self.read_blocking_timeout(&mut char_buf)?;
        if char_buf[0] != b'$' {
            return Err(GpsError::InvalidStart(char_buf[0] as char));
        }
        let mut checksum = 0;
        let mut components = vec![];
        let mut ended = false;
        // NMEA max len is 82, minus 6 bytes of header/trailer
        for _ in 0..76 {
            self.read_blocking_timeout(&mut char_buf)?;
            if char_buf[0] == b'*' {
                ended = true;
                break;
            }
            checksum ^= char_buf[0];
            components.push(char_buf[0]);
        }
        if !ended {
            return Err(GpsError::TooLong);
        }

        // read the rest
        let mut char_buf = [0; 4];
        self.read_blocking_timeout(&mut char_buf)?;
        let received_checksum = (char::from(char_buf[0])
            .to_digit(16)
            .ok_or(GpsError::InvalidChecksum)?
            << 4)
            + (char::from(char_buf[1])
                .to_digit(16)
                .ok_or(GpsError::InvalidChecksum)?);

        if received_checksum as u8 != checksum {
            defmt::error!(
                "Wrong checksum in sentence: ${}*{:02}\\r\\n",
                String::from_utf8_lossy_owned(components),
                checksum
            );
            return Err(GpsError::WrongChecksum);
        }
        if &char_buf[2..] != b"\r\n" {
            return Err(GpsError::InvalidEnd);
        }
        Ok(components)
    }
}
