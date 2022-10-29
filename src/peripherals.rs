use stm32f4xx_hal as p_hal;
use stm32f4xx_hal::pac as pac;
// use p_hal::prelude::*;
use p_hal::syscfg::SysCfgExt;
use p_hal::gpio::{ GpioExt, Edge, ExtiPin};
use p_hal::rcc::RccExt;
use p_hal::timer::{TimerExt, SysTimerExt,  delay::SysDelay};
use fugit::{RateExtU32};
use p_hal::timer::counter::{CounterHz};

pub fn setup_peripherals() -> (
    // LED output pin
    UserLed1Type,
    DelaySourceType,
    I2c1PortType,
    Spi1PortType,
    ChipSelectPinType,
    TimeoutTimerType,
    Irq1PinType,
) {
    let mut dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut syscfg = dp.SYSCFG.constrain();

    // Set up the system clock
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(25u32.MHz()) //f401cb  board has 25 MHz crystal for HSE
        .sysclk(84u32.MHz()) //f401cb supports 84 MHz sysclk max
        .pclk1(48u32.MHz())
        .pclk2(48u32.MHz())
        .freeze();

    // let hclk = clocks.hclk();
    // let rng_clk = clocks.pll48clk().unwrap_or(0u32.hz());
    // let pclk1 = clocks.pclk1();
    // d_println!(get_debug_log(), "hclk: {} /16: {} pclk1: {} rng_clk: {}", hclk.0, hclk.0 / 16, pclk1.0, rng_clk.0);

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    // let gpiod = dp.GPIOD.split();

    let user_led1 = gpioc.pc13.into_push_pull_output(); //f401CxUx

    // let user_led1 = gpiod.pd12.into_push_pull_output(); //f4discovery

    // setup i2c1
    // NOTE: stm32f401CxUx board lacks external pull-ups on i2c pins
    // NOTE: eg f407 discovery board already has external pull-ups
    // NOTE: sensor breakout boards may have their own pull-ups: check carefully
    let i2c1_port = {
        let scl = gpiob
            .pb8
            .into_alternate::<4>()
            .internal_pull_up(true)
            .set_open_drain();

        let sda = gpiob
            .pb9
            .into_alternate::<4>()
            //.into_alternate_af4()
            .internal_pull_up(true)
            .set_open_drain();

        p_hal::i2c::I2c1::new(dp.I2C1, (scl,sda), 400.kHz(), &clocks)
    };

    let spi1_port = {
        // SPI1 port setup
        let sck = gpioa.pa5.into_alternate::<5>();
        let miso = gpioa.pa6.into_alternate::<5>();
        let mosi = gpioa.pa7.into_alternate::<5>();

        p_hal::spi::Spi1::new(dp.SPI1,
            (sck, miso, mosi),
            embedded_hal::spi::MODE_0,
            3.MHz(),
            &clocks
        )

    };

    // SPI chip select CS
    let spi_csn = gpioa.pa15.into_open_drain_output();

    // HINTN interrupt pin
    // let hintn = gpiob.pb0.into_pull_up_input();

    let delay_source =  cortex_m::peripheral::SYST::delay(cp.SYST, &clocks);
    let timeout_timer = dp.TIM2.counter_hz(&clocks);

    // let timeout_timer =
    //     p_hal::timer::Timer::new(Timer2, &clocks).counter_hz();

    //
    let mut irq_pin = gpiob.pb10.into_pull_down_input(); //f401CxUx
    irq_pin.make_interrupt_source(&mut syscfg);
    irq_pin.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
    irq_pin.enable_interrupt(&mut dp.EXTI);

    pac::NVIC::unpend(p_hal::pac::Interrupt::TIM2);
    pac::NVIC::unpend(irq_pin.interrupt());
    unsafe {
        pac::NVIC::unmask(irq_pin.interrupt());
    };


    (user_led1,
     delay_source,
     i2c1_port,
     spi1_port,
     spi_csn,
     timeout_timer,
     irq_pin
    )
}

pub type I2c1PortType = p_hal::i2c::I2c<
    pac::I2C1,
    (
        p_hal::gpio::gpiob::PB8<p_hal::gpio::Alternate<4, p_hal::gpio::OpenDrain>>,
        p_hal::gpio::gpiob::PB9<p_hal::gpio::Alternate<4, p_hal::gpio::OpenDrain>>

    )>;


pub type Spi1PortType = p_hal::spi::Spi<
    pac::SPI1,
    (
        // AF5
        p_hal::gpio::gpioa::PA5<p_hal::gpio::Alternate<5>>,//SCLK
        p_hal::gpio::gpioa::PA6<p_hal::gpio::Alternate<5>>, //MISO
        p_hal::gpio::gpioa::PA7<p_hal::gpio::Alternate<5>>, //MOSI
    ),
>;

pub type ChipSelectPinType =
    p_hal::gpio::gpioa::PA15<p_hal::gpio::Output<p_hal::gpio::OpenDrain>>; //CSN

pub type UserLed1Type =
    p_hal::gpio::gpioc::PC13<p_hal::gpio::Output<p_hal::gpio::PushPull>>;

pub type DelaySourceType = SysDelay;

pub type TimeoutTimerType = CounterHz<pac::TIM2>;

pub type Irq1PinType = stm32f4xx_hal::gpio::Pin<'B', 10>;

