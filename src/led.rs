use esp_backtrace as _;

use crate::peripheral_traits::GpioPin;

pub struct Led<R: GpioPin, G: GpioPin, B: GpioPin> {
    red: R,
    green: G,
    blue: B,
}

impl<R, G, B> Led<R, G, B>
where
    R: GpioPin,
    G: GpioPin,
    B: GpioPin,
{
    pub fn new(red: R, green: G, blue: B) -> Self {
        let mut led = Led {
            red: red,
            green: green,
            blue: blue,
        };

        led.red.high();
        led.green.high();
        led.blue.high();

        led
    }

    pub fn red_on(&mut self) {
        self.red.low();
    }

    pub fn red_off(&mut self) {
        self.red.high();
    }

    pub fn green_on(&mut self) {
        self.green.low();
    }

    pub fn green_off(&mut self) {
        self.green.high();
    }

    pub fn blue_on(&mut self) {
        self.blue.low();
    }

    pub fn blue_off(&mut self) {
        self.blue.high();
    }

    pub fn set(&mut self, r: bool, g: bool, b: bool) {
        if r {
            self.red_on();
        } else {
            self.red_off();
        }
        if g {
            self.green_on();
        } else {
            self.green_off();
        }
        if b {
            self.blue_on();
        } else {
            self.blue_off();
        }
    }
}
