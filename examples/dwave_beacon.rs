/*
Copyright (c) 2022 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_main]
#![no_std]

extern crate stm32f4xx_hal;
extern crate embedded_timeout_macros;

use cortex_m_rt as rt;
use rt::entry;
use nb;

use panic_rtt_core::{self, rprintln, rtt_init_print};
use core::cell::{RefCell};
use core::ops::DerefMut;
use cortex_m::interrupt::Mutex;
use core::sync::atomic::{AtomicBool, Ordering};
use stm32f4xx_hal as p_hal;
use p_hal::{
    pac::interrupt,
    gpio::ExtiPin,
    timer::{CounterHz, Event},
};
use p_hal::pac as pac;
use fugit::{RateExtU32};

use embedded_hal::blocking::delay::DelayMs;
use dw1000::{ hl::DW1000, mac,  RxConfig,
              ranging::{self, Message as _RangingMessage}
};
use stm32f401ccu6_bsp::peripherals::{self, Spi1PortType, ChipSelectPinType, Irq1PinType};

static G_IRQ1_PIN: Mutex<RefCell<Option<Irq1PinType>>> = Mutex::new(RefCell::new(None));
static G_DW_TRIGGER:AtomicBool  = AtomicBool::new(false);
static TIMER_TIM2: Mutex<RefCell<Option<CounterHz<pac::TIM2>>>> = Mutex::new(RefCell::new(None));
static G_TIM2_TRIGGER:AtomicBool  = AtomicBool::new(false);



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

        let _ = user_led.set_low();
        // rprintln!("send ping...");
        // send a ping to any nearby base stations
        let mut sending = ranging::Ping::new(&mut dw1000)
            .expect("ping new")
            .send(dw1000)
            .expect("ping send");
        nb::block!({ sending.wait_transmit() }).expect("ping xmit");
        dw1000 = sending.finish_sending().expect("ping xmit finish");
        rprintln!("ping sent");

        // wait to receive a ranging request from a base station
        let _ = user_led.set_high();
        // rprintln!("start recv range req");
        let mut receiving = dw1000.receive(RxConfig::default()).expect("recv1 start");
        if !wait_for_trigger_or_timeout(8) {
            rprintln!("recv1 timeout");
            dw1000 = receiving.finish_receiving().expect("recv1 timeout finish");
            delay_source.delay_ms(100u32);
            continue;
        }
        if !wait_for_trigger_or_timeout(4) {
            rprintln!("recv2 timeout");
            dw1000 = receiving.finish_receiving().expect("recv2 timeout finish");
            delay_source.delay_ms(100u32);
            continue;
        }
        rprintln!("recv2 go");
        let result = nb::block!(  receiving.wait_receive(&mut buffer)  );
        dw1000 = receiving.finish_receiving().expect("recv1 finish");
        rprintln!("recv2 done");

        let message = match result {
            Ok(message) => message,
            Err(e) => {
                rprintln!("msg err: {:?}", &e);
                continue;
            }
        };

        // Decode the ranging request and respond with a ranging response
        // rprintln!("decode range req");
        let request =
            match ranging::Request::decode::<Spi1PortType, ChipSelectPinType>(&message) {
                Ok(Some(request)) => request,
                Ok(None) | Err(_) => {
                    rprintln!("ignoring nonreq");
                    continue;
                }
            };

        delay_source.delay_ms(25u32);

        // Send ranging response
        let _ = user_led.set_low();
        // rprintln!("send range resp");
        let mut sending = ranging::Response::new(&mut dw1000, &request)
            .expect("ranging new")
            .send(dw1000)
            .expect("ranging send");
        nb::block!({ sending.wait_transmit() }).expect("ranging xmit");
        dw1000 = sending.finish_sending().expect("ranging xmit finish");
        rprintln!("sent range");
        delay_source.delay_ms(150u32);

    }
}

fn start_timeout_timer(rate_hz: u32) {
    pac::NVIC::mask(pac::Interrupt::TIM2);
    pac::NVIC::unpend(pac::Interrupt::TIM2);

    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut tim2) = TIMER_TIM2.borrow(cs).borrow_mut().deref_mut() {
            tim2.clear_interrupt(Event::Update);
            let _start_rc = tim2.start(rate_hz.Hz());
            tim2.listen(Event::Update);
        }
    });

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIM2);
    }
}

fn clear_timeout_timer() {
    pac::NVIC::mask(pac::Interrupt::TIM2);
    pac::NVIC::unpend(pac::Interrupt::TIM2);

    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut tim2) = TIMER_TIM2.borrow(cs).borrow_mut().deref_mut() {
            tim2.unlisten(Event::Update);
            tim2.clear_interrupt(Event::Update);
        }
    });
}

fn wait_for_trigger_or_timeout(rate_hz: u32) -> bool  {
    let triggered: bool;
    start_timeout_timer(rate_hz);

    loop {
        if G_DW_TRIGGER.load(Ordering::Relaxed) {
            // the exti pin triggered
            G_DW_TRIGGER.store(false, Ordering::Relaxed);
            triggered = true;
            break;
        }
        else if G_TIM2_TRIGGER.load(Ordering::Relaxed) {
            // TIM2 expired trigger
            G_TIM2_TRIGGER.store(false, Ordering::Relaxed);

            // timeout before the exti pin triggered
            triggered = false;
            break;
        }
        //wait for either an exti pin interrupt or timer interrupt
        cortex_m::asm::wfi();
    }

    clear_timeout_timer();

    return triggered;
}

#[interrupt]
fn EXTI15_10() {
    // Interrupt Service Routine Code
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