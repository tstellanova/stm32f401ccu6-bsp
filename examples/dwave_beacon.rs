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
use core::cell::{Cell, RefCell};
use cortex_m::interrupt::Mutex;
use embedded_hal::blocking::delay::DelayMs;
use dw1000::{ hl::DW1000, mac,  RxConfig,
              ranging::{self, Message as _RangingMessage}
};
// use stm32f401ccu6_bsp::peripherals;
use stm32f401ccu6_bsp::peripherals::{self, Spi1PortType, ChipSelectPinType, Irq1PinType};

static G_IRQ1_PIN: Mutex<RefCell<Option<Irq1PinType>>> = Mutex::new(RefCell::new(None));



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

    rprintln!("start loop...");
    loop {
        let _ = user_led.toggle();

        //rprintln!("send ping...");
        // send a ping to any nearby base stations
        let mut sending = ranging::Ping::new(&mut dw1000)
            .expect("Failed to initiate ping")
            .send(dw1000)
            .expect("Failed to initiate ping transmission");
        rprintln!("start ping send...");
        timeout_timer.start(100_000u32);
        nb::block!({
                irq_pin.wait_for_interrupts(&mut gpiote, &mut timeout_timer);
                sending.wait_transmit()
            })
            .expect("Failed to send ping");

        //nb::block!(sending.wait_transmit()).expect("Failed to send data");
        //rprintln!("wait ping finish...");
        dw1000 = sending.finish_sending().expect("Failed to finish sending");

        // wait to receive a ranging request from a base station
        //rprintln!("prep receiving");
        let mut receiving = dw1000
            .receive(RxConfig::default())
            .expect("Failed to receive message");

        //rprintln!("start recv...");
        let result = nb::block!(receiving.wait_receive(&mut buffer));
        //rprintln!("finish recv...");
        dw1000 = receiving
            .finish_receiving()
            .expect("Failed to finish receiving");

        //rprintln!("check msg...");
        let message = match result {
            Ok(message) => message,
            Err(e) => {
                rprintln!("msg err: {:?}", &e);
                continue;
            }
        };

        // Decode the ranging request and respond with a ranging response
        //rprintln!("decode range req");
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
        rprintln!("setup range resp");
        let mut sending = ranging::Response::new(&mut dw1000, &request)
            .expect("Failed to initiate response")
            .send(dw1000)
            .expect("Failed to initiate response transmission");

        //rprintln!("start send range resp");
        nb::block!(sending.wait_transmit()).expect("Failed to send data");
        //rprintln!("finish range resp...");
        dw1000 = sending.finish_sending().expect("Failed to finish sending");

        delay_source.delay_ms(250u32);
    }
}

#[interrupt]
fn EXTI15_10() {
    // Interrupt Service Routine Code
}