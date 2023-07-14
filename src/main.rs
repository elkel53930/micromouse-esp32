#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_println::println;
use hal::{
    adc::{AdcConfig, AdcPin, Attenuation, ADC, ADC1},
    clock::ClockControl,
    gpio::{Analog, GpioPin, Output, PushPull, IO},
    peripherals::{self, Peripherals, TIMG0},
    prelude::*,
    spi::{FullDuplexMode, Spi, SpiMode},
    timer::{Timer, Timer0, TimerGroup},
    Rtc,
    Delay,
    interrupt::{self, Priority},
};

use core::cell::RefCell;
use critical_section::{with, Mutex};

mod encoder;
mod imu;
mod peripheral_adapter;
mod peripheral_traits;
use peripheral_adapter::GlobalDelay;
mod wall_sensors;
mod led;
use wall_sensors::WALL_SENSOR::{LF, LS, RF, RS};

type Global<T> = Mutex<RefCell<Option<T>>>;
type ActualImu<'a> =
    imu::Imu<Spi<'a, peripherals::SPI3, FullDuplexMode>, GpioPin<Output<PushPull>, 9>, GlobalDelay>;
type ActualEncoder<'a> = encoder::Encoder<
    Spi<'a, peripherals::SPI2, FullDuplexMode>,
    GpioPin<Output<PushPull>, 46>,
    GpioPin<Output<PushPull>, 10>>;
type ActualWallSensors = wall_sensors::WallSensors<
    GpioPin<Output<PushPull>, 14>,
    GpioPin<Output<PushPull>, 15>,
    GpioPin<Output<PushPull>, 16>,
    AdcPin<GpioPin<Analog, 1>, ADC1>,
    AdcPin<GpioPin<Analog, 2>, ADC1>,
    AdcPin<GpioPin<Analog, 3>, ADC1>,
    AdcPin<GpioPin<Analog, 4>, ADC1>>;
type ActualLed = led::Led<
    GpioPin<Output<PushPull>, 21>,
    GpioPin<Output<PushPull>, 19>,
    GpioPin<Output<PushPull>, 20>,>;

static GL_ADC1: Global<ADC<'_, ADC1>> = Mutex::new(RefCell::new(None));
static GL_DELAY: Global<Delay> = Mutex::new(RefCell::new(None));
static GL_IMU: Global<ActualImu<'_>> = Mutex::new(RefCell::new(None));
static GL_ENCODER: Global<ActualEncoder<'_>> = Mutex::new(RefCell::new(None));
static GL_WALL_SENSORS: Global<ActualWallSensors> = Mutex::new(RefCell::new(None));
static GL_LED: Global<ActualLed> = Mutex::new(RefCell::new(None));
static GL_TIMER00: Global<Timer<Timer0<TIMG0>>> = Mutex::new(RefCell::new(None));

const TIMER_INTERVAL: u64 = 1u64; // ms

struct InterruptContext{
    step: u32,
}

static INTERRUPT_CONTEXT: Global<InterruptContext> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the TIMG watchdog timer.
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut timer00 = timer_group0.timer0;
    let mut wdt0 = timer_group0.wdt;

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt0.disable();
    rtc.rwdt.disable();

    with(|cs| {
        GL_DELAY.borrow(cs).replace(Some(Delay::new(&clocks)));
    });

    // Set timer interrupt
    with(|cs| {
        INTERRUPT_CONTEXT.borrow(cs).replace(Some(InterruptContext{step: 0}));
    });
    interrupt::enable(peripherals::Interrupt::TG0_T0_LEVEL, Priority::Priority2).unwrap();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // LEDs
    let led = led::Led::new(
        io.pins.gpio21.into_push_pull_output(),
        io.pins.gpio19.into_push_pull_output(),
        io.pins.gpio20.into_push_pull_output(),
    );
    with(|cs| {
        GL_LED.borrow(cs).replace(Some(led));
    });

    // Encoder
    let cs_r = io.pins.gpio46.into_push_pull_output();
    let cs_l = io.pins.gpio10.into_push_pull_output();
    let spi = Spi::new_no_cs(
        peripherals.SPI2,
        io.pins.gpio11,
        io.pins.gpio13,
        io.pins.gpio12,
        5000u32.kHz(),
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
        5000u32.kHz(),
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

    let mut led_step = 0;

    // Start timer
    timer00.start(TIMER_INTERVAL.millis());
    timer00.listen();
    with(|cs|   {
        GL_TIMER00.borrow(cs).replace(Some(timer00));
    });

    loop{}

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


#[interrupt]
fn TG0_T0_LEVEL() {
    critical_section::with(|cs| {
        let mut timer = GL_TIMER00.borrow_ref_mut(cs);
        let timer = timer.as_mut().unwrap();

        if timer.is_interrupt_set() {
            GL_LED.borrow(cs).borrow_mut().as_mut().unwrap().set(true, true, true);
            GL_LED.borrow(cs).borrow_mut().as_mut().unwrap().set(false, true, true);
            timer.clear_interrupt();
            timer.start(TIMER_INTERVAL.millis());
            GL_LED.borrow(cs).borrow_mut().as_mut().unwrap().set(false, false, true);
            let step = INTERRUPT_CONTEXT.borrow(cs).borrow_mut().as_mut().unwrap().step;
            INTERRUPT_CONTEXT.borrow(cs).borrow_mut().as_mut().unwrap().step = (step + 1) % 2;
            GL_LED.borrow(cs).borrow_mut().as_mut().unwrap().set(false, false, false);
        }
    });
}
