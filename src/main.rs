#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_println::println;
use hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rtc,
    gpio::IO,
    Delay,
};


#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();
    println!("Hello world!");

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut led_g = io.pins.gpio19.into_push_pull_output();
    let mut led_b = io.pins.gpio20.into_push_pull_output();
    let mut led_r = io.pins.gpio21.into_push_pull_output();

    let mut delay = Delay::new(&clocks);

    led_r.set_low().unwrap();
    led_g.set_low().unwrap();
    led_b.set_low().unwrap();

    loop {
        led_r.toggle().unwrap();
        led_g.toggle().unwrap();
        led_b.toggle().unwrap();
        delay.delay_ms(500u32);
    }
}
