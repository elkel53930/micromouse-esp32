#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_println::println;

use embedded_hal::adc::{
    Channel,
    OneShot,
};

use esp32s3_hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rtc,
    gpio::IO,
    adc::{AdcConfig, Attenuation, ADC, ADC1, RegisterAccess, AdcPin},
};

struct Sensors<SENADC, ANALOG1, ANALOG2> {
    adc: SENADC,
    pin1: ANALOG1,
    pin2: ANALOG2,
}

impl<SENADC, PIN1, PIN2, ADCI> Sensors<SENADC, AdcPin<PIN1, ADCI>, AdcPin<PIN2, ADCI>>
    where
        SENADC: OneShot<ADCI, u16, AdcPin<PIN1, ADCI>>,
        PIN1: Channel<ADCI, ID = u8>,
        PIN2: Channel<ADCI, ID = u8>,
        ADCI: RegisterAccess, {
    
    pub fn new( adc: SENADC,
                pin1: AdcPin<PIN1, ADCI>,
                pin2: AdcPin<PIN2, ADCI>) -> Self  {
        Sensors {adc: adc, pin1: pin1, pin2: pin2}
    }

    pub fn read(&mut self) -> (u16, u16) {
        let v1 = match nb::block!(self.adc.read(&mut self.pin1)){
            Ok(value) => value,
            Err(_) => 0,
        };

        let v2 = match nb::block!(self.adc.read(&mut self.pin2)){
// |                                            ---- ^^^^^^^^^^^^^^ expected `&mut AdcPin<PIN1, ADCI>`, found `&mut AdcPin<PIN2, ADCI>`
// |                                            |
// |                                            arguments to this method are incorrect
            Ok(value) => value,
            Err(_) => 0,
        };
        (v1, v2)
    }
}


#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

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

    let analog = peripherals.SENS.split();

    let mut adc1_config = AdcConfig::new();

    let mut pin1 = adc1_config.enable_pin(io.pins.gpio1.into_analog(), Attenuation::Attenuation11dB);
    let mut pin2 = adc1_config.enable_pin(io.pins.gpio2.into_analog(), Attenuation::Attenuation11dB);
    let mut adc1 = ADC::<ADC1>::adc(analog.adc1, adc1_config).unwrap();

    // These two lines can work.
    let value1: u16 = nb::block!(adc1.read(&mut pin1)).unwrap();
    let value2: u16 = nb::block!(adc1.read(&mut pin2)).unwrap();

    let mut sensors = Sensors::new(adc1, pin1, pin2);
    let (value1, value2) = sensors.read();
    println!("{}, {}",value1, value2);

    loop {}
}