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
    spi::{Spi, SpiMode},
};


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
    let mut spi_imu = Spi::new(
        peripherals.SPI3,
        io.pins.gpio8,
        io.pins.gpio7,
        io.pins.gpio6,
        io.pins.gpio9,
        100u32.kHz(),
        SpiMode::Mode3,
        &mut system.peripheral_clock_control,
        &clocks,
    );


    let mut delay = Delay::new(&clocks);

    // Enable access to registers
    let mut data = [0x01, 0x80];
    spi_imu.transfer(&mut data).unwrap();

    // IMU : Set Gyro to high-performance mode
    let mut data = [0x11, 0xac];
    spi_imu.transfer(&mut data).unwrap();

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
        let mut data = [0xa6, 0xff, 0xff]; // Gyro yaw
        spi_imu.transfer(&mut data).unwrap();
        println!("imu {}", concatenate_u8_to_i16(data[2], data[1]));

    }
}

fn concatenate_u8_to_i16(a: u8, b: u8) -> i16 {
    let concatenated = ((a as i16) << 8) | (b as i16);
    concatenated
}