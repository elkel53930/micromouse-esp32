#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_println::println;
use hal::{
    adc::{AdcConfig, AdcPin, Attenuation, ADC, ADC1},
    clock::ClockControl,
    gpio::{Analog, GpioPin, Output, PushPull, Unknown, IO},
    interrupt::{self, Priority},
    mcpwm::{
        operator::{PwmPin, PwmPinConfig},
        timer::PwmWorkingMode,
        PeripheralClockConfig, MCPWM,
    },
    peripherals::{self, Peripherals, MCPWM0, MCPWM1, TIMG0},
    prelude::*,
    spi::{FullDuplexMode, Spi, SpiMode},
    timer::{Timer, Timer0, TimerGroup},
    Delay, Rtc,
};

use core::cell::RefCell;
use critical_section::{with, Mutex};

mod encoder;
mod imu;
mod peripheral_adapter;
mod peripheral_traits;
use peripheral_adapter::GlobalDelay;
mod led;
mod wall_sensors;
use wall_sensors::WallSensor::{LF, LS, RF, RS};
mod motor;

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
type ActualLed = led::Led<
    GpioPin<Output<PushPull>, 21>,
    GpioPin<Output<PushPull>, 19>,
    GpioPin<Output<PushPull>, 20>,
>;

type ActualMotorR<'a> =
    motor::Motor<PwmPin<'a, GpioPin<Unknown, 36>, MCPWM0, 0, true>, GpioPin<Output<PushPull>, 37>>;
type ActualMotorL<'a> =
    motor::Motor<PwmPin<'a, GpioPin<Unknown, 34>, MCPWM1, 0, true>, GpioPin<Output<PushPull>, 35>>;

pub static GL_ADC1: Global<ADC<'_, ADC1>> = Mutex::new(RefCell::new(None));
pub static GL_DELAY: Global<Delay> = Mutex::new(RefCell::new(None));
pub static GL_IMU: Global<ActualImu<'_>> = Mutex::new(RefCell::new(None));
pub static GL_ENCODER: Global<ActualEncoder<'_>> = Mutex::new(RefCell::new(None));
pub static GL_WALL_SENSORS: Global<ActualWallSensors> = Mutex::new(RefCell::new(None));
pub static GL_LED: Global<ActualLed> = Mutex::new(RefCell::new(None));
pub static GL_TIMER00: Global<Timer<Timer0<TIMG0>>> = Mutex::new(RefCell::new(None));
pub static GL_MOTOR_R: Global<ActualMotorR<'_>> = Mutex::new(RefCell::new(None));
pub static GL_MOTOR_L: Global<ActualMotorL<'_>> = Mutex::new(RefCell::new(None));

const TIMER_INTERVAL: u64 = 1u64; // ms

struct InterruptContext {
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

    with(|cs| GL_DELAY.borrow(cs).replace(Some(Delay::new(&clocks))));

    // Set timer interrupt
    with(|cs| {
        INTERRUPT_CONTEXT
            .borrow(cs)
            .replace(Some(InterruptContext { step: 0 }))
    });
    interrupt::enable(peripherals::Interrupt::TG0_T0_LEVEL, Priority::Priority2).unwrap();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    /******** Initialize LEDs ********/
    let led = led::Led::new(
        io.pins.gpio21.into_push_pull_output(),
        io.pins.gpio19.into_push_pull_output(),
        io.pins.gpio20.into_push_pull_output(),
    );
    with(|cs| GL_LED.borrow(cs).replace(Some(led)));

    /******** Encoders ********/
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
    let encoder = encoder::Encoder::new(spi, cs_r, cs_l);

    with(|cs| GL_ENCODER.borrow(cs).replace(Some(encoder)));

    /******** Initialize IMU ********/
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

    let imu = imu::Imu::new(
        spi_imu,
        io.pins.gpio9.into_push_pull_output(),
        GlobalDelay::new(),
    );

    with(|cs| GL_IMU.borrow(cs).replace(Some(imu)));

    /******** Initialize Wall sensors ********/
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

    with(|cs| GL_ADC1.borrow(cs).replace(Some(adc1)));

    let wall_sensors =
        wall_sensors::WallSensors::new(pin_lf, pin_ls, pin_rs, pin_rf, led_ena, led_sel0, led_sel1);

    with(|cs| GL_WALL_SENSORS.borrow(cs).replace(Some(wall_sensors)));

    /******** Initialize Motors ********/
    let pwm_r = io.pins.gpio36;
    let mut cwccw_r = io.pins.gpio37.into_push_pull_output();

    cwccw_r.set_high().unwrap();

    let clock_cfg = PeripheralClockConfig::with_frequency(&clocks, 40u32.MHz()).unwrap();
    let mut mcpwm0 = MCPWM::new(
        peripherals.MCPWM0,
        clock_cfg,
        &mut system.peripheral_clock_control,
    );

    mcpwm0.operator0.set_timer(&mcpwm0.timer0);
    let mut pwm_r = mcpwm0
        .operator0
        .with_pin_a(pwm_r, PwmPinConfig::UP_ACTIVE_HIGH);

    // Initialize MCPWM1
    let pwm_l = io.pins.gpio34;
    let mut cwccw_l = io.pins.gpio35.into_push_pull_output();

    cwccw_l.set_high().unwrap();

    let mut mcpwm1 = MCPWM::new(
        peripherals.MCPWM1,
        clock_cfg,
        &mut system.peripheral_clock_control,
    );

    mcpwm1.operator0.set_timer(&mcpwm1.timer0);
    let mut pwm_l = mcpwm1
        .operator0
        .with_pin_a(pwm_l, PwmPinConfig::UP_ACTIVE_HIGH);

    // start timer with timestamp values in the range of 0..=99 and a frequency of 200 kHz
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(99, PwmWorkingMode::Increase, 200u32.kHz())
        .unwrap();

    // Start timers
    mcpwm0.timer0.start(timer_clock_cfg);
    mcpwm1.timer0.start(timer_clock_cfg);

    // 0% duty
    pwm_r.set_timestamp(0);
    pwm_l.set_timestamp(0);

    with(|cs| {
        GL_MOTOR_R
            .borrow(cs)
            .replace(Some(motor::Motor::new(pwm_r, cwccw_r)));
        GL_MOTOR_L
            .borrow(cs)
            .replace(Some(motor::Motor::new(pwm_l, cwccw_l)));
    });

    /******** Start interrupt timer ********/
    timer00.start(TIMER_INTERVAL.millis());
    timer00.listen();
    with(|cs| {
        GL_TIMER00.borrow(cs).replace(Some(timer00));
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

#[interrupt]
#[allow(non_snake_case)]
fn TG0_T0_LEVEL() {
    critical_section::with(|cs| {
        let mut timer = GL_TIMER00.borrow_ref_mut(cs);
        let timer = timer.as_mut().unwrap();

        if timer.is_interrupt_set() {
            GL_LED
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .set(true, true, true);
            GL_LED
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .set(false, true, true);
            timer.clear_interrupt();
            timer.start(TIMER_INTERVAL.millis());
            GL_LED
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .set(false, false, true);
            let step = INTERRUPT_CONTEXT
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .step;
            INTERRUPT_CONTEXT
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .step = (step + 1) % 2;
            GL_LED
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .set(false, false, false);
        }
    });
}
