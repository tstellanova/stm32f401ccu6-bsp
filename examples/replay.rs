/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_main]
#![no_std]

use cortex_m_rt as rt;
use rt::entry;

use panic_rtt_core::{self, rprintln, rtt_init_print};


use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use stm32f401ccu6_bsp::peripherals;

use embedded_graphics::{
    image::{Image, ImageRaw},
    pixelcolor::BinaryColor,
    prelude::*,
};

use ssd1351::{ 
  prelude:: *, 
  builder::Builder
};


#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("--> MAIN --");

    let (mut user_led, mut delay_source, _i2c1_port, spi1_port, csn_pin, dc_pin) =
        peripherals::setup_peripherals();

    let _ = user_led.set_high();
    rprintln!("create display...");

    let mut disp = Builder::new().connect_spi(spi1_port, dc_pin);
    //let mut disp: GraphicsMode<_> =
     //   Builder::new().connect_spi(spi1_port, csn_pin).into();

    rprintln!("init display...");
    //disp.init().unwrap();

    rprintln!("load image...");
    let raw: ImageRaw<BinaryColor> =
        ImageRaw::new(include_bytes!("./rust.raw"), 64);
    let mut x_pos = 0;
    let mut im = Image::new(&raw, Point::new(x_pos, 0));

    rprintln!("start loop...");
    loop {
        let _ = user_led.toggle();
        im.draw(&mut disp).unwrap();
        im = im.translate(Point::new(8, 0));
        x_pos += 8;
        if x_pos >= 128 {
            //wrap to left
            im = im.translate(Point::new(-x_pos, 0));
            x_pos = 0;
            disp.clear();
        }
        disp.flush().unwrap();
        delay_source.delay_ms(50u8);
    }
}
