#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_println::println;
use hal::{
    adc::{AdcConfig, AdcPin, Attenuation, ADC, ADC1},
    clock::ClockControl,
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
    Delay, Rtc, Uart,
};

use core::cell::RefCell;
use critical_section::{with, Mutex};

mod peripheral_adapter;
mod peripheral_traits;
use peripheral_adapter::GlobalDelay;
mod peripheral;
use peripheral::{encoder, fram, imu, led, motor};
mod wall_sensors;
use wall_sensors::WallSensor::{LF, LS, RF, RS};
mod console;
mod log;
mod uart_read_line;

type ActualImu<'a> =
    imu::Imu<Spi<'a, SPI3, FullDuplexMode>, GpioPin<Output<PushPull>, 9>, GlobalDelay>;
type ActualEncoder<'a> = encoder::Encoder<
    Spi<'a, SPI2, FullDuplexMode>,
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

pub static mut GL_ADC1: Option<ADC<'_, ADC1>> = None;
pub static mut GL_DELAY: Option<Delay> = None;
pub static mut GL_IMU: Option<ActualImu<'_>> = None;
pub static mut GL_ENCODER: Option<ActualEncoder<'_>> = None;
pub static mut GL_WALL_SENSORS: Option<ActualWallSensors> = None;
pub static mut GL_LED: Option<ActualLed> = None;
pub static mut GL_TIMER00: Option<Timer<Timer0<TIMG0>>> = None;
pub static mut GL_MOTOR_R: Option<ActualMotorR<'_>> = None;
pub static mut GL_MOTOR_L: Option<ActualMotorL<'_>> = None;

const TIMER_INTERVAL: u64 = 1u64; // ms

struct InterruptContext {
    step: u32,
}

static INTERRUPT_CONTEXT: Option<InterruptContext> = None;

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

    unsafe {
        GL_DELAY = Some(Delay::new(&clocks));
    }

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    /******** Initialize I2C and Log ********/
    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio18,
        io.pins.gpio17,
        10u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );
    log::init_logger(i2c);

    //    for i in 0..1000 {
    //        println!("reset {}", i);
    //        log!("reset {}", i);
    //    }

    //    let mut data = [0u8; 24];
    //    for i in 0..300 {
    //        log::read_log_chunk(7200 - data.len() as u16 * i, &mut data);
    //        esp_println::print!("{}", core::str::from_utf8(&data).unwrap());
    //    }
    //
    //    let mut data = [0u8; 10];
    //    log::read_log(0, &mut data);
    //    esp_println::println!("start: {}", core::str::from_utf8(&data).unwrap());
    //    esp_println::println!("start: {:?}", data);
    //
    //    log::read_log(8180, &mut data);
    //    esp_println::println!("end: {}", core::str::from_utf8(&data).unwrap());
    //    esp_println::println!("end: {:?}", data);

    /******** Initialize LEDs ********/
    let led = led::Led::new(
        io.pins.gpio21.into_push_pull_output(),
        io.pins.gpio19.into_push_pull_output(),
        io.pins.gpio20.into_push_pull_output(),
    );

    unsafe {
        GL_LED = Some(led);
    }

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

    unsafe {
        GL_ENCODER = Some(encoder);
    }

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

    unsafe {
        GL_IMU = Some(imu);
    }

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

    unsafe {
        GL_ADC1 = Some(adc1);
    }

    let wall_sensors =
        wall_sensors::WallSensors::new(pin_lf, pin_ls, pin_rs, pin_rf, led_ena, led_sel0, led_sel1);

    unsafe {
        GL_WALL_SENSORS = Some(wall_sensors);
    }

    /******** Initialize Motors ********/
    let clock_cfg = PeripheralClockConfig::with_frequency(&clocks, 40u32.MHz()).unwrap();

    // Initialize MCPWM0
    let pwm_r = io.pins.gpio36;
    let mut cwccw_r = io.pins.gpio37.into_push_pull_output();

    cwccw_r.set_low().unwrap();

    let mut mcpwm0 = MCPWM::new(
        peripherals.MCPWM0,
        clock_cfg,
        &mut system.peripheral_clock_control,
    );

    mcpwm0.operator0.set_timer(&mcpwm0.timer0);
    let mut pwm_r = mcpwm0
        .operator0
        .with_pin_a(pwm_r, PwmPinConfig::UP_ACTIVE_HIGH);

    // start timer with timestamp values in the range of 0..=99 and a frequency of 200 kHz
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(99, PwmWorkingMode::Increase, 20u32.kHz())
        .unwrap();

    // Start timers
    mcpwm0.timer0.start(timer_clock_cfg);

    // 0% duty
    pwm_r.set_timestamp(0);

    // Initialize MCPWM1
    let pwm_l = io.pins.gpio34;
    let mut cwccw_l = io.pins.gpio35.into_push_pull_output();

    cwccw_l.set_low().unwrap();

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
        .timer_clock_with_frequency(99, PwmWorkingMode::Increase, 20u32.kHz())
        .unwrap();

    // Start timers
    mcpwm1.timer0.start(timer_clock_cfg);

    // 0% duty
    pwm_l.set_timestamp(0);

    // Motor sleep
    let mut motor_sleep = io.pins.gpio38.into_push_pull_output();
    motor_sleep.set_high().unwrap(); // Enable

    unsafe {
        GL_MOTOR_R = Some(motor::Motor::new(pwm_r, cwccw_r));
        GL_MOTOR_L = Some(motor::Motor::new(pwm_l, cwccw_l));
        GL_MOTOR_R.as_mut().unwrap().set_duty(0);
        GL_MOTOR_L.as_mut().unwrap().set_duty(0);
    }

    /******** Initialize serial port ********/
    let mut serial0 = Uart::new(peripherals.UART0, &mut system.peripheral_clock_control);

    /******** Initialize Console ********/
    let mut console = console::Console::new();

    /******** Main loop ********/
    console.run(&mut serial0);

    let raising = 100u32;
    loop {
        // Display wall sensor values
        unsafe{GL_WALL_SENSORS.as_mut().unwrap().enable(LF);}
        unsafe {GL_DELAY.as_mut().unwrap().delay_us(raising);}
        let lf = unsafe { GL_WALL_SENSORS.as_mut().unwrap().read(LF) };
        unsafe{GL_WALL_SENSORS.as_mut().unwrap().disable();}

        unsafe{GL_WALL_SENSORS.as_mut().unwrap().enable(LS);}
        unsafe {GL_DELAY.as_mut().unwrap().delay_us(raising);}
        let ls = unsafe { GL_WALL_SENSORS.as_mut().unwrap().read(LS) };
        unsafe{GL_WALL_SENSORS.as_mut().unwrap().disable();}

        unsafe{GL_WALL_SENSORS.as_mut().unwrap().enable(RS);}
        unsafe {GL_DELAY.as_mut().unwrap().delay_us(raising);}
        let rs = unsafe { GL_WALL_SENSORS.as_mut().unwrap().read(RS) };
        unsafe{GL_WALL_SENSORS.as_mut().unwrap().disable();}

        unsafe{GL_WALL_SENSORS.as_mut().unwrap().enable(RF);}
        unsafe {GL_DELAY.as_mut().unwrap().delay_us(raising);}
        let rf = unsafe { GL_WALL_SENSORS.as_mut().unwrap().read(RF) };
        unsafe{GL_WALL_SENSORS.as_mut().unwrap().disable();}

        println!("LF {:04} LS {:04} RS {:04} RF {:04}", lf, ls, rs, rf);

        unsafe {
            GL_DELAY.as_mut().unwrap().delay_ms(1u32);
        }

        continue;

        // Display encoder values
        let encr = unsafe { GL_ENCODER.as_mut().unwrap().read_r() };
        let encl = unsafe { GL_ENCODER.as_mut().unwrap().read_l() };
        println!("enc {}, {}", encr, encl);

        // IMU
        let gyro = unsafe { GL_IMU.as_mut().unwrap().read() };

        let who_am_i = unsafe { GL_IMU.as_mut().unwrap().who_am_i() };

        println!("imu {}, who {:x?}", gyro, who_am_i);

        unsafe {
            GL_DELAY.as_mut().unwrap().delay_ms(1000u32);
        }
    }
}
