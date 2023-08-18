use crate::peripheral_traits;

use esp_backtrace as _;
use hal::{
    adc::{AdcPin, ADC1},
    gpio::{Analog, GpioPin, Unknown},
    i2c::I2C,
    mcpwm::operator::PwmPin,
    peripherals::{self, I2C0, MCPWM0, MCPWM1},
    prelude::*,
    spi::{FullDuplexMode, Spi},
    Delay,
};

use crate::{GL_ADC1, GL_DELAY};

// SPI interfaces
impl peripheral_traits::SpiInterface for Spi<'_, peripherals::SPI3, FullDuplexMode> {
    fn communicate(&mut self, data: &mut [u8]) -> Result<(), ()> {
        self.transfer(data).unwrap();
        Ok(())
    }
}

impl peripheral_traits::SpiInterface for Spi<'_, peripherals::SPI2, FullDuplexMode> {
    fn communicate(&mut self, data: &mut [u8]) -> Result<(), ()> {
        self.transfer(data).unwrap();
        Ok(())
    }
}

// GPIO pins
impl<CS: _embedded_hal_digital_v2_OutputPin> peripheral_traits::CsPin for CS {
    fn assert(&mut self) {
        self.set_low().map_err(|_| ()).unwrap();
    }

    fn negate(&mut self) {
        self.set_high().map_err(|_| ()).unwrap();
    }
}

impl<PIN: _embedded_hal_digital_v2_OutputPin> peripheral_traits::OutputPin for PIN {
    fn low(&mut self) {
        self.set_low().map_err(|_| ()).unwrap();
    }

    fn high(&mut self) {
        self.set_high().map_err(|_| ()).unwrap();
    }
}

// AD converters
// Implement trait peripheral_traits::AdConverter for ADC1 and GPIO1
impl peripheral_traits::AdConverter for AdcPin<GpioPin<Analog, 1>, ADC1> {
    fn read(&mut self) -> u16 {
        unsafe { nb::block!(GL_ADC1.as_mut().unwrap().read(self)).unwrap() }
    }
}

// Implement trait peripheral_traits::AdConverter for ADC1 and GPIO2
impl peripheral_traits::AdConverter for AdcPin<GpioPin<Analog, 2>, ADC1> {
    fn read(&mut self) -> u16 {
        unsafe { nb::block!(GL_ADC1.as_mut().unwrap().read(self)).unwrap() }
    }
}

// Implement trait peripheral_traits::AdConverter for ADC1 and GPIO3
impl peripheral_traits::AdConverter for AdcPin<GpioPin<Analog, 3>, ADC1> {
    fn read(&mut self) -> u16 {
        unsafe { nb::block!(GL_ADC1.as_mut().unwrap().read(self)).unwrap() }
    }
}

// Implement trait peripheral_traits::AdConverter for ADC1 and GPIO4
impl peripheral_traits::AdConverter for AdcPin<GpioPin<Analog, 4>, ADC1> {
    fn read(&mut self) -> u16 {
        unsafe { nb::block!(GL_ADC1.as_mut().unwrap().read(self)).unwrap() }
    }
}

// Delay
impl peripheral_traits::Delay for Delay {
    fn ms(&mut self, ms: u32) {
        self.delay_ms(ms);
    }

    fn us(&mut self, us: u32) {
        self.delay_us(us);
    }
}

pub struct GlobalDelay;

impl GlobalDelay {
    pub fn new() -> Self {
        GlobalDelay
    }
}

impl peripheral_traits::Delay for GlobalDelay {
    fn ms(&mut self, ms: u32) {
        unsafe { GL_DELAY.as_mut().unwrap().delay_ms(ms) }
    }

    fn us(&mut self, us: u32) {
        unsafe { GL_DELAY.as_mut().unwrap().delay_us(us) }
    }
}

// Motor PWM
impl peripheral_traits::Pwm for PwmPin<'_, GpioPin<Unknown, 36>, MCPWM0, 0, true> {
    fn set_duty(&mut self, duty: u16) {
        self.set_timestamp(duty);
    }
}

impl peripheral_traits::Pwm for PwmPin<'_, GpioPin<Unknown, 34>, MCPWM1, 0, true> {
    fn set_duty(&mut self, duty: u16) {
        self.set_timestamp(duty);
    }
}

// I2C interface
impl peripheral_traits::I2cInterface for I2C<'_, I2C0> {
    fn write_to(&mut self, address: u8, data: &[u8]) -> Result<(), ()> {
        self.write(address, data).unwrap();
        Ok(())
    }

    fn read_from(&mut self, address: u8, data: &mut [u8]) -> Result<(), ()> {
        self.read(address, data).unwrap();
        Ok(())
    }
}
