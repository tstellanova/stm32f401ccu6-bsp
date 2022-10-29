/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_main]
#![no_std]


use cortex_m_rt as rt;
use core::cell::{RefCell};
use core::ops::DerefMut;
use cortex_m::interrupt::Mutex;
use core::sync::atomic::{AtomicBool, Ordering};
use stm32f4xx_hal as p_hal;
use p_hal::{
    pac::interrupt,
    gpio::ExtiPin,
    timer::{CounterHz, Event}
};
use p_hal::pac as pac;
use fugit::{RateExtU32};

use rt::entry;
use nb;

use panic_rtt_core::{self, rprintln, rtt_init_print};

use embedded_hal::blocking::delay::DelayMs;
use dw1000::{hl::DW1000, mac, RxConfig,
             ranging::{self, Message as _RangingMessage}
};
use stm32f401ccu6_bsp::peripherals::{self, Spi1PortType, ChipSelectPinType, Irq1PinType};


static G_IRQ1_PIN: Mutex<RefCell<Option<Irq1PinType>>> = Mutex::new(RefCell::new(None));
static G_DW_TRIGGER:AtomicBool  = AtomicBool::new(false);
static TIMER_TIM2: Mutex<RefCell<Option<CounterHz<pac::TIM2>>>> = Mutex::new(RefCell::new(None));
static G_TIM2_TRIGGER:AtomicBool  = AtomicBool::new(false);



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
        csn_pin,
        timeout_timer,
        irq_pin
    ) =   peripherals::setup_peripherals();

    let _ = user_led.set_high();

    cortex_m::interrupt::free(|cs| {
        TIMER_TIM2.borrow(cs).replace(Some(timeout_timer));
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

    loop {
        let _ = user_led.set_high();
        // rprintln!("seeker loop");
        delay_source.delay_ms(100u32);

        // rprintln!("wait pingreq...");
        let mut receiving = dw1000.receive(RxConfig::default()).expect("recv1 startfail");
        if !wait_for_trigger_or_timeout(4) {
            dw1000 = receiving.finish_receiving().expect("recv1 timeout finish");
            continue;
        }
        let result = nb::block!(  receiving.wait_receive(&mut buffer1) );
        dw1000 = receiving.finish_receiving().expect("recv1 finish");

        let message = match result {
            Ok(message) => message,
            Err(e) => {
                rprintln!("msg err: {:?}", &e);
                delay_source.delay_ms(250u32);
                continue;
            }
        };

        // Assume this message was a ping and try to decode it
        // rprintln!("recvd pingreq");
        let ping = match ranging::Ping::decode::<Spi1PortType, ChipSelectPinType>(&message) {
            Ok(Some(ping)) => ping,
            Ok(None) => {
                rprintln!("empty ping");
                continue;
            }
            Err(e) => {
                rprintln!("Ping decode error: {:?}", &e);
                continue;
            }
        };

        // rprintln!("decode ping...");
        let (pinger_pan_id, pinger_addr) = match ping.source {
            Some(mac::Address::Short(pan_id, addr)) => (pan_id, addr),
            _ => {
                rprintln!("strange ping");
                continue
            },
        };
        rprintln!("ping < {:04x}:{:04x}", pinger_pan_id.0, pinger_addr.0);

        //wait for pinger to switch over to listening for range request
        delay_source.delay_ms(25u32);

        let _ = user_led.set_low();
        let mut sending = ranging::Request::new(&mut dw1000, &ping)
            .expect("ranging new")
            .send(dw1000)
            .expect("ranging send");

        //rprintln!("wait_transmit range_req...");
        nb::block!({
            sending.wait_transmit()
        }).expect("ranging xmit");
        dw1000 = sending.finish_sending().expect("ranging xmit finish");

        // receive ranging response
        let _ = user_led.set_high();
        let mut receiving = dw1000.receive(RxConfig::default()).expect("resp rcv");
        // rprintln!("wait_receive range resp ...");
        if !wait_for_trigger_or_timeout(2) {
            dw1000 = receiving.finish_receiving().expect("resp rcv timeout finish");
            continue;
        }
        let result = nb::block!(  receiving.wait_receive(&mut buffer2) );
        dw1000 = receiving.finish_receiving().expect("resp rcv finish");

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

        // rprintln!("compute distance...");
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
        // delay_source.delay_ms(250u32);
    }
}

fn wait_for_trigger_or_timeout(rate_hz: u32) -> bool  {
    let mut success: bool = false;

    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut tim2) = TIMER_TIM2.borrow(cs).borrow_mut().deref_mut() {
            tim2.clear_interrupt(Event::Update);
            let _start_rc = tim2.start(rate_hz.Hz());
            tim2.listen(Event::Update);
        }
    });

    pac::NVIC::unpend(pac::Interrupt::TIM2);
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIM2);
    }

    loop {
        //wait for either an exti pin interrupt or timer interrupt
        cortex_m::asm::wfi();
        if G_DW_TRIGGER.load(Ordering::Relaxed) {
            // the exti pin triggered
            G_DW_TRIGGER.store(false, Ordering::Relaxed);
            success = true;
            break;
        }
        else if G_TIM2_TRIGGER.load(Ordering::Relaxed) {
            // TIM2 expired trigger
            G_TIM2_TRIGGER.store(false, Ordering::Relaxed);

            // timeout before the exti pin triggered
            success = false;
            break;
        }
    }
    pac::NVIC::mask(pac::Interrupt::TIM2);

    return success;

}


#[interrupt]
fn EXTI15_10() {
    G_DW_TRIGGER.store(true, Ordering::Relaxed);

    cortex_m::interrupt::free(|cs| {
        // Obtain access to Global Button Peripheral and Clear Interrupt Pending Flag
        let mut irq_pin = G_IRQ1_PIN.borrow(cs).borrow_mut();
        irq_pin.as_mut().unwrap().clear_interrupt_pending_bit();
    });
}


#[interrupt]
fn TIM2() {
    G_TIM2_TRIGGER.store(true, Ordering::Relaxed);

    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut tim2) = TIMER_TIM2.borrow(cs).borrow_mut().deref_mut() {
            tim2.clear_interrupt(Event::Update);
        }
    });
}