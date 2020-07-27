/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_main]
#![no_std]

use cortex_m_rt as rt;
use p_hal::stm32 as pac;
use rt::entry;
use stm32f4xx_hal as p_hal;

use panic_rtt_core::{self, rprintln, rtt_init_print};

// use embedded_hal::digital::v2::OutputPin;
// use embedded_hal::digital::v2::ToggleableOutputPin;

use stm32f401ccu6_bsp::peripherals;
// use embedded_hal::blocking::delay::{DelayMs};

use embedded_graphics::{
    image::{Image, ImageRaw},
    pixelcolor::BinaryColor,
    prelude::*,
};

use ssd1306::{prelude::*, Builder};

#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("-- > MAIN --");

    let (_user_led, _delay_source, i2c1_port, _spi1_port, _csn_pin) =
        peripherals::setup_peripherals();

    let mut disp: GraphicsMode<_> =
        Builder::new().connect_i2c(i2c1_port).into();

    disp.init().unwrap();

    let raw: ImageRaw<BinaryColor> =
        ImageRaw::new(include_bytes!("./rust.raw"), 64, 64);
    let im = Image::new(&raw, Point::new(32, 0));
    im.draw(&mut disp).unwrap();
    disp.flush().unwrap();
    
    loop {

    }
}
