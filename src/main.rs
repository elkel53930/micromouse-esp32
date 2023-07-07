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
    adc::{AdcConfig, Attenuation, ADC, ADC1},
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

    /* Blink LEDs */
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut led_g = io.pins.gpio19.into_push_pull_output();
    let mut led_b = io.pins.gpio20.into_push_pull_output();
    let mut led_r = io.pins.gpio21.into_push_pull_output();

    led_r.set_low().unwrap();
    led_g.set_low().unwrap();
    led_b.set_low().unwrap();

    /* Wall sensors ADC */
    let analog = peripherals.SENS.split();
    let mut adc1_config = AdcConfig::new();

    let mut pin =
        adc1_config.enable_pin(io.pins.gpio2.into_analog(), Attenuation::Attenuation11dB);

    let mut adc1 = ADC::<ADC1>::adc(analog.adc1, adc1_config).unwrap();

    /* Wall sensor LED */
    /*
        SEL1 SEL0    LED     PIN
          0    0    RS(D4)  GPIO4
          0    1    LS(D1)  GPIO1
          1    0    RF(D3)  GPIO3
          1    1    LF(D2)  GPIO2
    */
    let mut led_ena = io.pins.gpio14.into_push_pull_output();
    let mut led_sel0 = io.pins.gpio15.into_push_pull_output();
    let mut led_sel1 = io.pins.gpio16.into_push_pull_output();    

    led_ena.set_low().unwrap();
    led_sel1.set_high().unwrap();
    led_sel0.set_high().unwrap();

    

    /* Delay */
    let mut delay = Delay::new(&clocks);

    loop {
        led_ena.set_high().unwrap();
        delay.delay_us(10u32);
        let rs_value: u16 = nb::block!(adc1.read(&mut pin)).unwrap();
        led_ena.set_low().unwrap();
        println!("PIN3 ADC reading = {}", rs_value);

        led_r.toggle().unwrap();
        led_g.toggle().unwrap();
        led_b.toggle().unwrap();
        delay.delay_ms(1u32);
    }
}
