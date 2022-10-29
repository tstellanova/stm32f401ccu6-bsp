/*
Copyright (c) 2022 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_main]
#![no_std]

use cortex_m_rt as rt;
use rt::entry;
use nb;

use panic_rtt_core::{self, rprintln, rtt_init_print};
use core::cell::{RefCell};
use cortex_m::interrupt::Mutex;
use core::sync::atomic::{AtomicBool, Ordering};
use stm32f4xx_hal::{
    pac::interrupt,
    gpio::ExtiPin,
};

use embedded_hal::blocking::delay::DelayMs;
use dw1000::{ hl::DW1000, mac,  RxConfig,
              ranging::{self, Message as _RangingMessage}
};
// use stm32f401ccu6_bsp::peripherals;
use stm32f401ccu6_bsp::peripherals::{self, Spi1PortType, ChipSelectPinType, Irq1PinType};

static G_IRQ1_PIN: Mutex<RefCell<Option<Irq1PinType>>> = Mutex::new(RefCell::new(None));
static G_FLIPPIN_STATE:AtomicBool  = AtomicBool::new(false);



/**
This "beacon" is acting as dwave mobile tag.
It sends pings to any nearby base stations.
The base station responds with a range timing request,
and the beacon responds to that.
In this way, the base station can measure the range to multiple beacons.
*/
#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("--> MAIN --");

    let (mut user_led,
        mut delay_source,
        _i2c1_port,
        spi1_port,
        csn_pin,
        mut _timeout_timer,
        irq_pin
    ) =   peripherals::setup_peripherals();

    let _ = user_led.set_high();

    cortex_m::interrupt::free(|cs| {
        G_IRQ1_PIN.borrow(cs).replace(Some(irq_pin));
    });


    let mut dw1000 =
        DW1000::new(spi1_port, csn_pin)
            .init(&mut delay_source)
            .expect("Failed to initialize DW1000");

    dw1000
        .enable_tx_interrupts()
        .expect("Failed to enable TX interrupts");
    dw1000
        .enable_rx_interrupts()
        .expect("Failed to enable RX interrupts");

    // TODO these are bogus delay values: need to be calibrated
    dw1000
        // .set_antenna_delay(16456, 16300)
        .set_antenna_delay(17000u16, 17000u16)
        .expect("Failed to set antenna delay");

    dw1000
        .set_address(
            mac::PanId(0x0d57),       // hardcoded network id
            mac::ShortAddress(0xB0DA), // random device address
        )
        .expect("Failed to set address");

    let mut buffer = [0; 1024];

    loop {
        let _ = user_led.set_high();
        rprintln!("beacon loop");
        delay_source.delay_ms(250u32);

        let _ = user_led.set_low();
        rprintln!("send ping...");
        // send a ping to any nearby base stations
        let mut sending = ranging::Ping::new(&mut dw1000)
            .expect("Failed to initiate ping")
            .send(dw1000)
            .expect("Failed to initiate ping transmission");
        nb::block!({
            // while !G_FLIPPIN_STATE.load(Ordering::Relaxed) { //limits scope to just irq_pin
            //     cortex_m::asm::wfi(); // wait for any interrupt
            // }
            // G_FLIPPIN_STATE.store(false, Ordering::Relaxed);
            sending.wait_transmit()
            }).expect("Failed to send ping");
        dw1000 = sending.finish_sending().expect("Failed to finish sending");
        rprintln!("ping sent");

        // wait to receive a ranging request from a base station
        let _ = user_led.set_high();
        rprintln!("start recv range req");
        let mut receiving = dw1000
            .receive(RxConfig::default())
            .expect("Failed to receive message");
        //TODO this gets stuck waiting for a range resp that never comes -- need to retry ping sooner
        let result = nb::block!({
            while !G_FLIPPIN_STATE.load(Ordering::Relaxed) { //limits scope to just irq_pin
                cortex_m::asm::wfi(); // wait for any interrupt
            }
            G_FLIPPIN_STATE.store(false, Ordering::Relaxed);
            receiving.wait_receive(&mut buffer)
        });
        dw1000 = receiving.finish_receiving().expect("Failed to finish receiving");
        rprintln!("received range req");

        let message = match result {
            Ok(message) => message,
            Err(e) => {
                rprintln!("msg err: {:?}", &e);
                continue;
            }
        };

        // Decode the ranging request and respond with a ranging response
        rprintln!("decode range req");
        let request =
            match ranging::Request::decode::<Spi1PortType, ChipSelectPinType>(&message) {
                Ok(Some(request)) => request,
                Ok(None) | Err(_) => {
                    rprintln!("ignoring nonreq");
                    continue;
                }
            };

        delay_source.delay_ms(10u32);

        // Send ranging response
        let _ = user_led.set_low();
        rprintln!("send range resp");
        let mut sending = ranging::Response::new(&mut dw1000, &request)
            .expect("Failed to initiate response")
            .send(dw1000)
            .expect("Failed to initiate response transmission");
        nb::block!({
            // while !G_FLIPPIN_STATE.load(Ordering::Relaxed) { //limits scope to just irq_pin
            //     cortex_m::asm::wfi(); // wait for any interrupt
            // }
            // G_FLIPPIN_STATE.store(false, Ordering::Relaxed);
            sending.wait_transmit()
        }).expect("Failed to send data");
        //rprintln!("finish range resp...");
        dw1000 = sending.finish_sending().expect("Failed to finish sending");

    }
}

#[interrupt]
fn EXTI15_10() {
    // Interrupt Service Routine Code
    G_FLIPPIN_STATE.store(true, Ordering::Relaxed);

    cortex_m::interrupt::free(|cs| {
        // Obtain access to Global Button Peripheral and Clear Interrupt Pending Flag
        let mut irq_pin = G_IRQ1_PIN.borrow(cs).borrow_mut();
        irq_pin.as_mut().unwrap().clear_interrupt_pending_bit();
    });
}