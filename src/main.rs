#![no_std]
#![no_main]

use core::cell::RefCell;
use critical_section::{with, Mutex};


use hal::{
    adc::{AdcConfig, AdcPin, Attenuation, ADC, ADC1},
    clock::{ClockControl, CpuClock},
    gpio::{Analog, GpioPin, Output, PushPull, Unknown, IO},
    i2c::I2C,
    interrupt::{self, Priority},
    mcpwm::{
        operator::{PwmPin, PwmPinConfig},
        timer::PwmWorkingMode,
        PeripheralClockConfig, MCPWM,
    },
    peripherals::{self, Peripherals, MCPWM0, MCPWM1, SPI2, SPI3, TIMG0},
    prelude::*,
    spi::{FullDuplexMode, Spi, SpiMode},
    timer::{Timer, Timer0, TimerGroup},
    Delay, Rtc,
    Uart,
    systimer::{Alarm, Periodic, SystemTimer, Target},
};


use esp_backtrace as _;
use esp_println::println;


mod peripheral_traits;
mod peripheral_adapter;
mod led;

type ActualLed = led::Led<
    GpioPin<Output<PushPull>, 21>,
    GpioPin<Output<PushPull>, 19>,
    GpioPin<Output<PushPull>, 20>,
>;
type Global<T> = Mutex<RefCell<Option<T>>>;
//pub static GL_LED: Global<ActualLed> = Mutex::new(RefCell::new(None));

pub static mut GL_LED: Option<ActualLed> = None;

pub static GL_ADC1: Global<ADC<'_, ADC1>> = Mutex::new(RefCell::new(None));
pub static GL_DELAY: Global<Delay> = Mutex::new(RefCell::new(None));

static mut ALARM0: Option<Alarm<Periodic, 0>> = None;
static ALARM1: Mutex<RefCell<Option<Alarm<Target, 1>>>> = Mutex::new(RefCell::new(None));
static ALARM2: Mutex<RefCell<Option<Alarm<Target, 2>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::configure(
        system.clock_control,
        CpuClock::Clock240MHz,
    )
    .freeze();

    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt = timer_group0.wdt;
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc.rwdt.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    /******** Initialize LEDs ********/
    let led = led::Led::new(
        io.pins.gpio21.into_push_pull_output(),
        io.pins.gpio19.into_push_pull_output(),
        io.pins.gpio20.into_push_pull_output(),
    );
    unsafe{
        GL_LED = Some(led);
    }
//    with(|cs| GL_LED.borrow(cs).replace(Some(led)));

//    let mut red_led = io.pins.gpio21.into_push_pull_output();

    let syst = SystemTimer::new(peripherals.SYSTIMER);

    println!("SYSTIMER Current value = {}", SystemTimer::now());


    let alarm0 = syst.alarm0.into_periodic();
    alarm0.set_period(10u32.kHz());
    alarm0.interrupt_enable(true);

    let alarm1 = syst.alarm1;
    alarm1.set_target(SystemTimer::now() + (SystemTimer::TICKS_PER_SECOND * 2));
    alarm1.interrupt_enable(true);

    let alarm2 = syst.alarm2;
    alarm2.set_target(SystemTimer::now() + (SystemTimer::TICKS_PER_SECOND * 3));
    alarm2.interrupt_enable(true);

    unsafe{
        ALARM0 = Some(alarm0);
    }

    critical_section::with(|cs| {
        ALARM1.borrow_ref_mut(cs).replace(alarm1);
        ALARM2.borrow_ref_mut(cs).replace(alarm2);
    });

    interrupt::enable(
        peripherals::Interrupt::SYSTIMER_TARGET0,
        Priority::Priority1,
    )
    .unwrap();
    interrupt::enable(
        peripherals::Interrupt::SYSTIMER_TARGET1,
        Priority::Priority2,
    )
    .unwrap();
    interrupt::enable(
        peripherals::Interrupt::SYSTIMER_TARGET2,
        Priority::Priority3,
    )
    .unwrap();

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let mut delay = Delay::new(&clocks);

    loop {
//        red_led.set_low().unwrap();
//        red_led.set_high().unwrap();
//        with(|cs| {
//            GL_LED.borrow(cs).borrow_mut().as_mut().unwrap().red_on();
//            GL_LED.borrow(cs).borrow_mut().as_mut().unwrap().red_off();
//        });
    }
}

#[interrupt]
fn SYSTIMER_TARGET0() {
    unsafe{
        GL_LED.as_mut().unwrap().red_on();
    }
    
    unsafe{
        ALARM0.as_mut().unwrap().clear_interrupt();
    }

    unsafe{
        GL_LED.as_mut().unwrap().red_off();
    }
}

#[interrupt]
fn SYSTIMER_TARGET1() {
    println!("Interrupt lvl2 (alarm1)");
    critical_section::with(|cs| {
        ALARM1
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}

#[interrupt]
fn SYSTIMER_TARGET2() {
    println!("Interrupt lvl2 (alarm2)");
    critical_section::with(|cs| {
        ALARM2
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}