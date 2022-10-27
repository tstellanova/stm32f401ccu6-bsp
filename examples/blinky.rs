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
use stm32f401ccu6_bsp::peripherals;


static G_IRQ1_PIN: Mutex<RefCell<Option<Irq1PinType>>> = Mutex::new(RefCell::new(None));


#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("--> MAIN --");

    let (mut user_led,
        mut delay_source,
        _i2c1_port,
        _spi1_port,
        _csn_pin,
        mut _timeout_timer,
        _irq_pin,
    ) = peripherals::setup_peripherals();

    let _ = user_led.set_high();

    rprintln!("start loop...");
    loop {
        let _ = user_led.toggle();
        delay_source.delay_ms(500u16);
    }
}
