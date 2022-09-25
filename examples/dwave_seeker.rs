/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_main]
#![no_std]

use cortex_m_rt as rt;
use rt::entry;
use nb;

use panic_rtt_core::{self, rprintln, rtt_init_print};

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use stm32f401ccu6_bsp::peripherals;
use dw1000::{hl::SendTime, hl::DW1000, mac, RxConfig, TxConfig,
             ranging::{self, Message as _RangingMessage},};
use stm32f401ccu6_bsp::peripherals::{Spi1PortType, ChipSelectPinType};


#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("--> MAIN --");
    let mut avg_range:i64 = 0;
    let mut range_count:i64 = 0;

    let (mut user_led,
        mut delay_source,
        _i2c1_port,
        spi1_port,
        csn_pin) =
        peripherals::setup_peripherals();

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

    // TODO bogus delays -- needs calibration
    dw1000
        .set_antenna_delay(16456, 16300)
        // .set_antenna_delay(17000u16, 17000u16)
        .expect("Failed to set antenna delay");

    dw1000
        .set_address(
            mac::PanId(0x0d57),       // hardcoded network id
            mac::ShortAddress(0xFADE), // random device address
        )
        .expect("Failed to set address");

    let mut buffer1 = [0; 1024];
    let mut buffer2 = [0; 1024];

    rprintln!("start loop...");
    loop {
        let _ = user_led.toggle();

        let mut receiving = dw1000
            .receive(RxConfig::default())
            .expect("Failed to receive message");

        let message = nb::block!(receiving.wait_receive(&mut buffer1));
        dw1000 = receiving
            .finish_receiving()
            .expect("Failed to finish receiving");

        let message = match message {
            Ok(message) => message,
            Err(e) => {
                rprintln!("msg err: {:?}", &e);
                continue;
            }
        };

        // Assume this message was a ping and try to decode it
        let ping = match ranging::Ping::decode::<Spi1PortType, ChipSelectPinType>(&message) {
            Ok(Some(ping)) => ping,
            Ok(None) => {
                rprintln!("Failed to decode ping");
                continue;
            }
            Err(e) => {
                rprintln!("Ping decode error: {:?}", &e);
                continue;
            }
        };

        let (pinger_pan_id, pinger_addr) = match ping.source {
            Some(mac::Address::Short(pan_id, addr)) => (pan_id, addr),
            _ => continue,
        };
        rprintln!("ping < {:04x}:{:04x}", pinger_pan_id.0, pinger_addr.0);

        //wait for pinger to switch over to listening for range request
        delay_source.delay_ms(10u32);

        let mut sending = ranging::Request::new(&mut dw1000, &ping)
            .expect("Failed range req initiate")
            .send(dw1000)
            .expect("Failed to initiate request xmit");

        nb::block!(sending.wait_transmit()).expect("Failed to send data");
        dw1000 = sending.finish_sending().expect("Failed to finish sending");
        let mut receiving = dw1000
            .receive(RxConfig::default())
            .expect("Failed to receive message");

        let result = nb::block!(receiving.wait_receive(&mut buffer2));
        dw1000 = receiving
            .finish_receiving()
            .expect("Failed to finish receiving");

        let message = match result {
            Ok(message) => message,
            Err(e) => {
                rprintln!("msg err: {:?}", &e);
                continue;
            },
        };

        let response =
            match ranging::Response::decode::<Spi1PortType, ChipSelectPinType>(&message) {
                Ok(Some(response)) => response,
                Ok(None) => {
                    rprintln!( "bad frame!"); //:\n{:?}", &message.frame);
                    delay_source.delay_ms(10u32);
                    continue;
                }
                Err(e) => {
                    rprintln!("decode err: {:?}", &e);
                    continue;
                }
            };

        // If this is not a PAN ID and short address, it doesn't come from a compatible node.
        // Ignore it.
        let (pan_id, addr) = match response.source {
            Some(mac::Address::Short(pan_id, addr)) => (pan_id, addr),
            _ => continue,
        };

        //rprintln!("compute distance...");
        // Ranging response received. Compute distance.
        match ranging::compute_distance_mm(&response) {
            Ok(distance_mm) => {
                range_count += 1;
                let incr_avg = ((distance_mm as i64) - avg_range)/range_count;
                avg_range = ((range_count * avg_range + incr_avg) / range_count) as i64;
                rprintln!("{:04x}:{:04x} range {} mm / avg {} mm", pan_id.0, addr.0, distance_mm, avg_range);
            }
            Err(e) => {
                rprintln!("range err: {:?}", &e);
            }
        }
        rprintln!("...");
        delay_source.delay_ms(250u32);
    }
}
