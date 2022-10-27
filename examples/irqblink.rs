/*
Copyright (c) 2022 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)

Example of
*/

#![no_main]
#![no_std]

extern crate cortex_m;
extern crate stm32f401ccu6_bsp;

use cortex_m_rt as rt;
use rt::{entry};

use panic_rtt_core::{self, rprintln, rtt_init_print};

use embedded_hal::blocking::delay::DelayMs;

use stm32f401ccu6_bsp::peripherals::{self, Irq1PinType};
use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::interrupt::Mutex;
use core::cell::RefCell;

// use stm32f4xx_hal::gpio::ExtiPin;

use stm32f4xx_hal::{
    pac::interrupt,
    gpio::ExtiPin,
};

static G_IRQ1_PIN: Mutex<RefCell<Option<Irq1PinType>>> = Mutex::new(RefCell::new(None));
static G_FLIPPIN_STATE:AtomicBool  = AtomicBool::new(false);

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
        irq_pin,
    ) = peripherals::setup_peripherals();

    let _ = user_led.set_low();
    cortex_m::interrupt::free(|cs| {
        G_IRQ1_PIN.borrow(cs).replace(Some(irq_pin));
    });

    rprintln!("start loop...");
    loop {
        let cur_val:bool = G_FLIPPIN_STATE.load(Ordering::Relaxed);
        if cur_val {
            user_led.set_high();
        }
        else {
            user_led.set_low();
        }
        // let _ = user_led.toggle();
        delay_source.delay_ms(500u16);
    }
}

#[interrupt]
fn EXTI15_10() {
    // Interrupt Service Routine Code
    let cur_val:bool = G_FLIPPIN_STATE.load(Ordering::Relaxed);
    G_FLIPPIN_STATE.store(!cur_val, Ordering::Relaxed);

    cortex_m::interrupt::free(|cs| {
        // Obtain access to Global Button Peripheral and Clear Interrupt Pending Flag
        let mut irq_pin = G_IRQ1_PIN.borrow(cs).borrow_mut();
        irq_pin.as_mut().unwrap().clear_interrupt_pending_bit();
    });
}
