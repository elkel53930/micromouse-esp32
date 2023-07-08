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
    adc::{AdcConfig, Attenuation, ADC, ADC1}, peripheral,
    spi::{Spi, SpiMode},
};

use esp_backtrace as _;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-S3, this includes the RTC WDT, and
    // the TIMG WDT.
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt = timer_group0.wdt;

    wdt.disable();
    rtc.rwdt.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Encoder
    let sclk = io.pins.gpio11;
    let miso = io.pins.gpio12;
    let mosi = io.pins.gpio13;
    let mut cs_r = io.pins.gpio46.into_push_pull_output();
    let mut cs_l = io.pins.gpio10.into_push_pull_output();
    cs_r.set_high().unwrap();
    cs_l.set_high().unwrap();

    let mut spi = Spi::new_no_cs(
        peripherals.SPI2,
        sclk,
        mosi,
        miso,
        100u32.kHz(),
        SpiMode::Mode1,
        &mut system.peripheral_clock_control,
        &clocks,
    );


    /* IMU */
    let sclk_imu = io.pins.gpio8;
    let miso_imu = io.pins.gpio6;
    let mosi_imu = io.pins.gpio7;
    let mut cs_imu = io.pins.gpio9.into_push_pull_output();

    cs_imu.set_high().unwrap();

    let mut spi_imu = Spi::new_no_cs(
        peripherals.SPI3,
        sclk_imu,
        mosi_imu,
        miso_imu,
        100u32.kHz(),
        SpiMode::Mode3,
        &mut system.peripheral_clock_control,
        &clocks,
    );


    let mut delay = Delay::new(&clocks);

    loop {
        delay.delay_ms(250u32);
        // Encoder
        let mut data = [0x7f, 0xfe];
        cs_r.set_low().unwrap();
        delay.delay_ms(1u32);
        spi.transfer(&mut data).unwrap();
        delay.delay_ms(1u32);
        cs_r.set_high().unwrap();
        let enc = (data[0] as u16)*256 + (data[1] as u16) & 0x3fff;
        println!("enc {}", enc);

        // IMU
//        let mut data = [0xa6, 0xff, 0xff]; // Gyro yaw
        let mut data = [0x8f, 0xff]; // who am i
        cs_imu.set_low().unwrap();
        delay.delay_ms(1u32);
        spi_imu.transfer(&mut data).unwrap();
        delay.delay_ms(1u32);
        cs_imu.set_high().unwrap();
//        let enc = (data[0] as u16)*256 + (data[1] as u16) & 0x3fff;
//        println!("{}", enc);
        println!("imu {:x?}", data);

    }
}
