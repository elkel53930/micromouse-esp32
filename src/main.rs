#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_println::println;
use hal::{
    adc::{AdcConfig, AdcPin, Attenuation, ADC, ADC1},
    clock::ClockControl,
    gpio::{Analog, GpioPin, Output, PushPull, IO},
    peripherals,
    peripherals::Peripherals,
    prelude::*,
    spi::{FullDuplexMode, Spi, SpiMode},
    timer::TimerGroup,
    Rtc,
    Delay,
};

use core::cell::RefCell;
use critical_section::{with, Mutex};

mod encoder;
mod imu;
mod peripheral_adapter;
mod peripheral_traits;
use peripheral_adapter::GlobalDelay;
mod wall_sensors;
use wall_sensors::WALL_SENSOR::{LF, LS, RF, RS};

type Global<T> = Mutex<RefCell<Option<T>>>;
type ActualImu<'a> =
    imu::Imu<Spi<'a, peripherals::SPI3, FullDuplexMode>, GpioPin<Output<PushPull>, 9>, GlobalDelay>;
type ActualEncoder<'a> = encoder::Encoder<
    Spi<'a, peripherals::SPI2, FullDuplexMode>,
    GpioPin<Output<PushPull>, 46>,
    GpioPin<Output<PushPull>, 10>,
>;
type ActualWallSensors = wall_sensors::WallSensors<
    GpioPin<Output<PushPull>, 14>,
    GpioPin<Output<PushPull>, 15>,
    GpioPin<Output<PushPull>, 16>,
    AdcPin<GpioPin<Analog, 1>, ADC1>,
    AdcPin<GpioPin<Analog, 2>, ADC1>,
    AdcPin<GpioPin<Analog, 3>, ADC1>,
    AdcPin<GpioPin<Analog, 4>, ADC1>,
>;

static GL_ADC1: Global<ADC<'_, ADC1>> = Mutex::new(RefCell::new(None));
static GL_DELAY: Global<Delay> = Mutex::new(RefCell::new(None));
static GL_IMU: Global<ActualImu<'_>> = Mutex::new(RefCell::new(None));
static GL_ENCODER: Global<ActualEncoder<'_>> = Mutex::new(RefCell::new(None));
static GL_WALL_SENSORS: Global<ActualWallSensors> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    with(|cs| {
        GL_DELAY.borrow(cs).replace(Some(Delay::new(&clocks)));
    });

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
    let cs_r = io.pins.gpio46.into_push_pull_output();
    let cs_l = io.pins.gpio10.into_push_pull_output();
    let spi = Spi::new_no_cs(
        peripherals.SPI2,
        io.pins.gpio11,
        io.pins.gpio13,
        io.pins.gpio12,
        100u32.kHz(),
        SpiMode::Mode1,
        &mut system.peripheral_clock_control,
        &clocks,
    );
    let mut encoder = encoder::Encoder::new(spi, cs_r, cs_l);

    with(|cs| {
        GL_ENCODER.borrow(cs).replace(Some(encoder));
    });

    // IMU
    let spi_imu = Spi::new_no_cs(
        peripherals.SPI3,
        io.pins.gpio8,
        io.pins.gpio7,
        io.pins.gpio6,
        1000u32.kHz(),
        SpiMode::Mode3,
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let mut imu = imu::Imu::new(
        spi_imu,
        io.pins.gpio9.into_push_pull_output(),
        GlobalDelay::new(),
    );

    with(|cs| GL_IMU.borrow(cs).replace(Some(imu)));

    /* Wall sensors ADC */
    let analog = peripherals.SENS.split();
    let mut adc1_config = AdcConfig::new();

    let pin_ls = adc1_config.enable_pin(io.pins.gpio1.into_analog(), Attenuation::Attenuation11dB);
    let pin_lf = adc1_config.enable_pin(io.pins.gpio2.into_analog(), Attenuation::Attenuation11dB);
    let pin_rf = adc1_config.enable_pin(io.pins.gpio3.into_analog(), Attenuation::Attenuation11dB);
    let pin_rs = adc1_config.enable_pin(io.pins.gpio4.into_analog(), Attenuation::Attenuation11dB);
    let led_ena = io.pins.gpio14.into_push_pull_output();
    let led_sel0 = io.pins.gpio15.into_push_pull_output();
    let led_sel1 = io.pins.gpio16.into_push_pull_output();
    let adc1 = ADC::<ADC1>::adc(analog.adc1, adc1_config).unwrap();

    with(|cs| {
        GL_ADC1.borrow(cs).replace(Some(adc1));
    });

    // Wall sensors
    let mut wall_sensors =
        wall_sensors::WallSensors::new(pin_lf, pin_ls, pin_rs, pin_rf, led_ena, led_sel0, led_sel1);

    with(|cs| {
        GL_WALL_SENSORS.borrow(cs).replace(Some(wall_sensors));
    });

    loop {
        // Display wall sensor values
        let lf = with(|cs| {
            GL_WALL_SENSORS
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .read(LF)
        });
        let ls = with(|cs| {
            GL_WALL_SENSORS
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .read(LS)
        });
        let rs = with(|cs| {
            GL_WALL_SENSORS
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .read(RS)
        });
        let rf = with(|cs| {
            GL_WALL_SENSORS
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .read(RF)
        });
        println!("LF {:04} LS {:04} RS {:04} RF {:04}", lf, ls, rs, rf);

        // Encoder
        let encr = with(|cs| {
            GL_ENCODER
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .read_r()
        });
        let encl = with(|cs| {
            GL_ENCODER
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .read_l()
        });
        println!("enc {}, {}", encr, encl);

        // IMU
        let gyro = with(|cs| GL_IMU.borrow(cs).borrow_mut().as_mut().unwrap().read());
        let who_am_i = with(|cs| GL_IMU.borrow(cs).borrow_mut().as_mut().unwrap().who_am_i());
        println!("imu {}, who {:x?}", gyro, who_am_i);

        with(|cs| {
            GL_DELAY
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .delay_ms(250u32);
        });
    }
}
