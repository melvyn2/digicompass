use core::f32::consts::PI;
use embedded_hal::delay::DelayNs;
use libm::{floorf, fmaxf, fminf};
use rp2040_hal::Timer;
use rp2040_hal::gpio::AnyPin;
use rp2040_hal::pio::{PIOExt, StateMachineIndex};
use smart_leds::{RGBW, SmartLedsWrite, White};
use ws2812_pio::Ws2812Direct;

const ALL_OFF: [RGBW<u8>; 24] = [RGBW {
    r: 0,
    g: 0,
    b: 0,
    a: White(0),
}; 24];

pub fn leds_off<P: PIOExt, SM: StateMachineIndex, I: AnyPin<Function = P::PinFunction>>(
    ws: &mut Ws2812Direct<P, SM, I, RGBW<u8>>,
) {
    ws.write(ALL_OFF).unwrap();
}

fn distance_color(dist_m: u32) -> RGBW<u8> {
    // Map distance to 0..=6
    const DIST_SCALE: f32 = 12500.0;
    const HUE_MAX: f32 = 6.0;
    const DIST_OFFSET: f32 = DIST_SCALE / HUE_MAX;

    let hue = HUE_MAX - (DIST_SCALE / (dist_m as f32 + DIST_OFFSET));

    // hue to rgb as from https://commons.wikimedia.org/wiki/File:HSV-RGB-comparison.svg

    // assumes h in 0..=6
    // offsets are
    fn color_channel(h: f32) -> u8 {
        (if h < 3.0 {
            fminf(h, 1.0)
        } else {
            fmaxf(4.0 - h, 0.0)
        } * 255.0) as u8
    }

    let r = color_channel((hue + 2.0) % 6.0);
    let g = color_channel(hue);
    let b = color_channel((hue + 4.0) % 6.0);

    RGBW {
        r,
        g,
        b,
        a: White(0),
    }
}

/// Direction is angle along ring in radians, distance is meters from target
pub fn set_leds_from_direction_distance<
    P: PIOExt,
    SM: StateMachineIndex,
    I: AnyPin<Function = P::PinFunction>,
>(
    ws: &mut Ws2812Direct<P, SM, I, RGBW<u8>>,
    dir: f32,
    dist_m: u32,
) {
    let color = distance_color(dist_m);

    let mut out = ALL_OFF;

    // Find the index and mix factors of the two involved LEDs
    let dir_24 = (dir * 12.0) / PI;
    let low = floorf(dir_24) as usize;
    let ifac = dir_24 - floorf(dir_24);
    let fac = 1.0 - ifac;

    // Curve brightness
    let ifac_c = ifac * ifac;
    let fac_c = fac * fac;

    let low_color = RGBW {
        r: (color.r as f32 * fac_c) as u8,
        g: (color.g as f32 * fac_c) as u8,
        b: (color.b as f32 * fac_c) as u8,
        a: White((color.a.0 as f32 * fac_c) as u8),
    };

    let high_color = RGBW {
        r: (color.r as f32 * ifac_c) as u8,
        g: (color.g as f32 * ifac_c) as u8,
        b: (color.b as f32 * ifac_c) as u8,
        a: White((color.a.0 as f32 * ifac_c) as u8),
    };

    out[low % 24] = low_color;
    out[(low + 1) % 24] = high_color;

    ws.write(out).unwrap();
}

pub fn startup_ring<P: PIOExt, SM: StateMachineIndex, I: AnyPin<Function = P::PinFunction>>(
    ws: &mut Ws2812Direct<P, SM, I, RGBW<u8>>,
    timer: &mut Timer,
) {
    const BRIGHTNESS: u8 = 32;
    for l in 0..24 {
        let mut data = ALL_OFF.clone();
        data[l] = match l % 4 {
            0 => RGBW {
                r: BRIGHTNESS,
                g: 0,
                b: 0,
                a: White(0),
            },
            1 => RGBW {
                r: 0,
                g: BRIGHTNESS,
                b: 0,
                a: White(0),
            },
            2 => RGBW {
                r: 0,
                g: 0,
                b: BRIGHTNESS,
                a: White(0),
            },
            _ => RGBW {
                r: 0,
                g: 0,
                b: 0,
                a: White(BRIGHTNESS),
            },
        };
        // Infallible
        ws.write(data).unwrap();
        timer.delay_ms(85);
    }
    leds_off(ws);
}
