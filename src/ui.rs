use alloc::format;
use core::f32::consts::PI;

use adafruit_feather_rp2040_adalogger::pac::I2C1;

use embedded_graphics::Drawable;
use embedded_graphics::geometry::{Point, Size};
use embedded_graphics::image::{Image, ImageRaw};
use embedded_graphics::mono_font::ascii::{FONT_6X10, FONT_7X13_BOLD};
use embedded_graphics::mono_font::{MonoTextStyle, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::primitives::{
    Line, Primitive, PrimitiveStyleBuilder, Rectangle, RoundedRectangle,
};
use embedded_graphics::text::{Alignment, Baseline, Text, TextStyleBuilder};
use libm::{cosf, roundf, sinf};

use rp2040_hal::I2C;

use sh1106::interface::I2cInterface;
use sh1106::mode::GraphicsMode;
use world_magnetic_model::time::Date;

pub const BIG_FONT: MonoTextStyle<BinaryColor> = MonoTextStyleBuilder::new()
    .font(&FONT_7X13_BOLD)
    .text_color(BinaryColor::On)
    .build();
pub const BASE_FONT: MonoTextStyle<BinaryColor> = MonoTextStyleBuilder::new()
    .font(&FONT_6X10)
    .text_color(BinaryColor::On)
    .build();

/// bat: 0 to 24
fn draw_bat<T>(display: &mut GraphicsMode<I2cInterface<I2C<I2C1, T>>>, bat: u8) {
    let stroke = PrimitiveStyleBuilder::new()
        .stroke_width(1)
        .stroke_color(BinaryColor::On)
        .build();

    RoundedRectangle::with_equal_corners(
        Rectangle::new(Point::new(3, 3), Size::new(29, 9)),
        Size::new(4, 4),
    )
    .into_styled(stroke)
    .draw(display)
    .unwrap();

    if bat > 0 {
        let filled = PrimitiveStyleBuilder::new()
            // .stroke_width(1)
            .fill_color(BinaryColor::On)
            .build();

        RoundedRectangle::with_equal_corners(
            Rectangle::new(Point::new(5, 5), Size::new(bat as u32, 5)),
            Size::new(2, 2),
        )
        .into_styled(filled)
        .draw(display)
        .unwrap();
    }

    // Battery nub
    display.set_pixel(33, 6, 1);
    display.set_pixel(33, 7, 1);
    display.set_pixel(33, 8, 1);
}

/// hdg is radians east of north
fn draw_hdg<T>(display: &mut GraphicsMode<I2cInterface<I2C<I2C1, T>>>, hdg: f32) {
    // Has 24x24 top-right area, center is 116,12

    let p = |r: u8, theta: f32| -> Point {
        let x = 112 + roundf(cosf(theta + hdg) * (r as f32)) as i32;
        let y = 16 - roundf(sinf(theta + hdg) * (r as f32)) as i32;
        Point::new(x, y)
    };

    let stroke = PrimitiveStyleBuilder::new()
        .stroke_width(1)
        .stroke_color(BinaryColor::On)
        .build();

    Line::new(p(12, 13.0 * PI / 24.0), p(16, 1.668848))
        .into_styled(stroke)
        .draw(display)
        .unwrap();
    Line::new(p(16, 1.668848), p(12, 11.0 * PI / 24.0))
        .into_styled(stroke)
        .draw(display)
        .unwrap();
    Line::new(p(12, 11.0 * PI / 24.0), p(16, 1.472744))
        .into_styled(stroke)
        .draw(display)
        .unwrap();

    const ARROW_SIZE: u8 = 9;
    Line::new(p(ARROW_SIZE, PI / 2.0), p(ARROW_SIZE, 4.0 * PI / 3.0))
        .into_styled(stroke)
        .draw(display)
        .unwrap();
    Line::new(p(ARROW_SIZE, PI / 2.0), p(ARROW_SIZE, 5.0 * PI / 3.0))
        .into_styled(stroke)
        .draw(display)
        .unwrap();
    Line::new(
        p(ARROW_SIZE / 3, 3.0 * PI / 2.0),
        p(ARROW_SIZE, 4.0 * PI / 3.0),
    )
    .into_styled(stroke)
    .draw(display)
    .unwrap();
    Line::new(
        p(ARROW_SIZE / 3, 3.0 * PI / 2.0),
        p(ARROW_SIZE, 5.0 * PI / 3.0),
    )
    .into_styled(stroke)
    .draw(display)
    .unwrap();
}

fn draw_sat<T>(display: &mut GraphicsMode<I2cInterface<I2C<I2C1, T>>>, show_x: bool) {
    #[rustfmt::skip]
    const SAT_BITMAP: Image<ImageRaw<BinaryColor>> = Image::new(
        &ImageRaw::<BinaryColor>::new(&[
            0b0000_0000, 0b0000_0000,
            0b0000_0000, 0b0000_0000,
            0b0000_0000, 0b0000_0000,
            0b0000_0000, 0b0000_0000,
            0b0000_0000, 0b0000_0000,
            0b0000_0000, 0b1000_0000,
            0b0000_0000, 0b0101_1000,
            0b0000_0000, 0b0010_0100,
            0b0000_0000, 0b0100_0100,
            0b0000_0011, 0b0000_1000,
            0b0000_0100, 0b1001_0100,
            0b0000_0010, 0b0100_0010,
            0b0000_0001, 0b0010_0000,
            0b0000_0010, 0b1010_0000,
            0b0000_0100, 0b0100_0000,
            0b0000_0000, 0b0000_0000,
        ], 16),
        Point::new(0, 48)
    );
    #[rustfmt::skip]
    const CROSS_BITMAP: Image<ImageRaw<BinaryColor>> = Image::new(
        &ImageRaw::<BinaryColor>::new(&[
            0b0000_0000,
            0b0100_0100,
            0b0010_1000,
            0b0001_0000,
            0b0010_1000,
            0b0100_0100,
        ], 8),
        Point::new(0, 48)
    );

    SAT_BITMAP.draw(display).unwrap();
    if show_x {
        CROSS_BITMAP.draw(display).unwrap();
    }
}

fn draw_date<T>(display: &mut GraphicsMode<I2cInterface<I2C<I2C1, T>>>, date: Date) {
    Text::with_text_style(
        format!("{} {} {}", date.month(), date.day(), date.year()).as_str(),
        Point::new(64, 53),
        BASE_FONT,
        TextStyleBuilder::new()
            .baseline(Baseline::Top)
            .alignment(Alignment::Center)
            .build(),
    )
    .draw(display)
    .unwrap();
}

pub fn draw_ui<T>(
    display: &mut GraphicsMode<I2cInterface<I2C<I2C1, T>>>,
    dist: &str,
    loc: &str,
    bat: Option<u8>,
    hdg: Option<f32>,
    satellite_ok: Option<bool>,
    date: Option<Date>,
) {
    display.clear();

    Text::with_text_style(
        dist,
        Point::new(64, 32),
        BIG_FONT,
        TextStyleBuilder::new()
            .baseline(Baseline::Bottom)
            .alignment(Alignment::Center)
            .build(),
    )
    .draw(display)
    .unwrap();

    Text::with_text_style(
        loc,
        Point::new(64, 32),
        BASE_FONT,
        TextStyleBuilder::new()
            .baseline(Baseline::Top)
            .alignment(Alignment::Center)
            .build(),
    )
    .draw(display)
    .unwrap();

    if let Some(bat) = bat {
        draw_bat(display, bat);
    }
    if let Some(hdg) = hdg {
        draw_hdg(display, hdg);
    }
    if let Some(satellite_ok) = satellite_ok {
        draw_sat(display, !satellite_ok);
    }
    if let Some(date) = date {
        draw_date(display, date)
    }

    display.flush().unwrap();
}
