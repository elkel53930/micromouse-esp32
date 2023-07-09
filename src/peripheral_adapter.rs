use crate::peripheral_traits;
use critical_section::with;

use esp_backtrace as _;
use hal::{
    adc::{AdcPin, ADC1},
    gpio::{Analog, GpioPin},
    peripherals,
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

impl<PIN: _embedded_hal_digital_v2_OutputPin> peripheral_traits::GpioPin for PIN {
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
        with(|cs| nb::block!(GL_ADC1.borrow(cs).borrow_mut().as_mut().unwrap().read(self)).unwrap())
    }
}

// Implement trait peripheral_traits::AdConverter for ADC1 and GPIO2
impl peripheral_traits::AdConverter for AdcPin<GpioPin<Analog, 2>, ADC1> {
    fn read(&mut self) -> u16 {
        with(|cs| nb::block!(GL_ADC1.borrow(cs).borrow_mut().as_mut().unwrap().read(self)).unwrap())
    }
}

// Implement trait peripheral_traits::AdConverter for ADC1 and GPIO3
impl peripheral_traits::AdConverter for AdcPin<GpioPin<Analog, 3>, ADC1> {
    fn read(&mut self) -> u16 {
        with(|cs| nb::block!(GL_ADC1.borrow(cs).borrow_mut().as_mut().unwrap().read(self)).unwrap())
    }
}

// Implement trait peripheral_traits::AdConverter for ADC1 and GPIO4
impl peripheral_traits::AdConverter for AdcPin<GpioPin<Analog, 4>, ADC1> {
    fn read(&mut self) -> u16 {
        with(|cs| nb::block!(GL_ADC1.borrow(cs).borrow_mut().as_mut().unwrap().read(self)).unwrap())
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
        with(|cs| {
            GL_DELAY
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .delay_ms(ms);
        });
    }

    fn us(&mut self, us: u32) {
        with(|cs| {
            GL_DELAY
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .delay_us(us);
        });
    }
}
