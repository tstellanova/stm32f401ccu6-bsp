/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_main]
#![no_std]


extern crate dw1000;

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
use dw1000::{hl::{DW1000}, mac, RxConfig, ranging::{self, Message as _RangingMessage} };
use stm32f401ccu6_bsp::peripherals::{self, Spi1PortType, ChipSelectPinType, Irq1PinType};


static G_IRQ1_PIN: Mutex<RefCell<Option<Irq1PinType>>> = Mutex::new(RefCell::new(None));
static G_DW_TRIGGER:AtomicBool  = AtomicBool::new(false);
static TIMER_TIM2: Mutex<RefCell<Option<CounterHz<pac::TIM2>>>> = Mutex::new(RefCell::new(None));
static G_TIM2_TRIGGER:AtomicBool  = AtomicBool::new(false);



// fn receive_one_message<'b>(dw1000: &DW1000<Spi1PortType, ChipSelectPinType, dw1000::Ready>, buffer: &'b mut [u8])
//     -> Result<dw1000::Message<'b>, dw1000::hl::Error<Spi1PortType, ChipSelectPinType>>{
//     // rprintln!("wait for message...");
//     let mut receiving = dw1000.receive(RxConfig::default()).expect("rcv1 startfail");
//     if !wait_for_trigger_or_timeout(1) {
//         receiving.finish_receiving().expect("rcv1 timeout finish");
//         return Err(Error::FrameWaitTimeout);
//     }
//     let result = nb::block!(  receiving.wait_receive(buffer) );
//     receiving.finish_receiving().expect("rcv1 finish");
//
//     return result;
// }

#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("--> MAIN --");
    let mut avg_range:i64 = 0;
    // let mut range_count:i64 = 0;
    let mut total_range_count:u32 = 0;
    let mut success_range_count:i64 = 0;

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
        .set_antenna_delay(16456u16, 16300u16)
        // .set_antenna_delay(17000u16, 17000u16)
        .expect("Failed to set antenna delay");

    dw1000
        .set_address(
            mac::PanId(0x0d57),       // hardcoded network id
            mac::ShortAddress(0xFADE), // random device address
        )
        .expect("Failed to set address");

    // let mut buffer2 = [0u8; 1024];
    let mut loop_count = 0u32;
    let mut buffer1 = [0u8; 1024];
    let mut ping_recvd = false;

    loop {
        let _ = user_led.set_high();
        rprintln!("seeker loop {}", loop_count);
        loop_count +=1;
        if !ping_recvd {
            delay_source.delay_ms(250u32);
        }

        //rprintln!("wait for msg...");
        let mut receiving = dw1000.receive(RxConfig::default()).expect("rcv1 startfail");
        if !wait_for_trigger_or_timeout(2) {
            dw1000 = receiving.finish_receiving().expect("rcv1 timeout finish");
            continue;
        }
        let result = nb::block!(  receiving.wait_receive(&mut buffer1) );
        dw1000 = receiving.finish_receiving().expect("rcv1 finish");

        let message = match result {
            Ok(message) => message,
            Err(e) => {
                rprintln!("msg err: {:?}", &e);
                continue ;
            }
        };

        // Assume this message was a ping and try to decode it
        if !ping_recvd {
            match ranging::Ping::decode::<Spi1PortType, ChipSelectPinType>(&message) {
                Ok(Some(ping)) => {
                    // rprintln!("decode ping...");
                    let (_pinger_pan_id, _pinger_addr) = match ping.source {
                        Some(mac::Address::Short(pan_id, addr)) => (pan_id, addr),
                        _ => {
                            rprintln!("strange ping");
                            continue;
                        },
                    };
                    // rprintln!("ping < {:04x}:{:04x}", _pinger_pan_id.0, _pinger_addr.0);

                    // Wait for a moment, to give the anchor a chance to start listening
                    // for the ranging request reply
                    delay_source.delay_ms(10u32);

                    // send ranging request
                    let _ = user_led.set_low();
                    let mut sending = ranging::Request::new(&mut dw1000, &ping)
                        .expect("ranging new")
                        .send(dw1000)
                        .expect("ranging send");

                    //rprintln!("wait_transmit range_req...");
                    nb::block!({ sending.wait_transmit()}).expect("ranging xmit");
                    dw1000 = sending.finish_sending().expect("ranging xmit finish");
                    // rprintln!("sent range req");

                    // delay_source.delay_ms(100u32);
                    ping_recvd = true;
                    continue; // get the next message
                },
                Ok(None) => {
                    rprintln!("not a ping");
                    // try another decoding
                },
                Err(e) => {
                    rprintln!("Ping decode error: {:?}", &e);
                    continue;
                },
            };
        }
        else {
            match ranging::Response::decode::<Spi1PortType, ChipSelectPinType>(&message) {
                Ok(Some(response)) => {
                    // rprintln!("decode range resp..");
                    ping_recvd = false;
                    // If this is not a PAN ID and short address, it doesn't come from a compatible node.
                    // Ignore it.
                    let (pan_id, addr) = match response.source {
                        Some(mac::Address::Short(pan_id, addr)) => (pan_id, addr),
                        _ => {
                            rprintln!("weird range_resp");
                            continue
                        },
                    };

                    // rprintln!("compute distance...");
                    total_range_count += 1;

                    // Ranging response received. Compute distance.
                    match ranging::compute_distance_mm(&response) {
                        Ok(distance_mm) => {
                            success_range_count += 1;
                            let incr_avg = ((distance_mm as i64) - avg_range) / success_range_count;
                            avg_range = ((success_range_count * avg_range + incr_avg) / success_range_count) as i64;
                            rprintln!("{:04x}:{:04x} range {} mm / avg {} mm", pan_id.0, addr.0, distance_mm, avg_range);
                        }
                        Err(e) => {
                            rprintln!("range ({}/{}) err: {:?}", success_range_count, total_range_count, &e);
                            // rprintln!("ping: {:?}", &ping);
                            rprintln!("resp: {:?}", &response);
                        }
                    }
                    delay_source.delay_ms(100u32);
                    continue;
                },
                Ok(None) => {
                    rprintln!( "not a rangeresp!");
                    // try another decoding
                },
                Err(e) => {
                    rprintln!("decode rangeresp err: {:?}", &e);
                    continue;
                }
            };
        }

        rprintln!("weird msg");
        ping_recvd = false;

        delay_source.delay_ms(250u32);
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
    G_DW_TRIGGER.store(false, Ordering::Relaxed);

    loop {
        //wait for either an exti pin interrupt or timer interrupt
        cortex_m::asm::wfi();
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
    }
    clear_timeout_timer();

    return triggered;
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