use esp_backtrace as _;

use crate::peripheral_traits::OutputPin;

pub struct Led<R: OutputPin, G: OutputPin, B: OutputPin> {
    red: R,
    green: G,
    blue: B,

    is_red_on: bool,
    is_green_on: bool,
    is_blue_on: bool,
}

impl<R, G, B> Led<R, G, B>
where
    R: OutputPin,
    G: OutputPin,
    B: OutputPin,
{
    pub fn new(red: R, green: G, blue: B) -> Self {
        let mut led = Led {
            red: red,
            green: green,
            blue: blue,
            is_red_on: false,
            is_green_on: false,
            is_blue_on: false,
        };

        led.red.high();
        led.green.high();
        led.blue.high();

        led
    }

    pub fn red_on(&mut self) {
        self.red.low();
        self.is_red_on = true;
    }

    pub fn red_off(&mut self) {
        self.red.high();
        self.is_red_on = false;
    }

    pub fn red_toggle(&mut self) {
        if self.is_red_on {
            self.red_off();
        } else {
            self.red_on();
        }
    }

    pub fn green_on(&mut self) {
        self.green.low();
        self.is_green_on = true;
    }

    pub fn green_off(&mut self) {
        self.green.high();
        self.is_green_on = false;
    }

    pub fn green_toggle(&mut self) {
        if self.is_green_on {
            self.green_off();
        } else {
            self.green_on();
        }
    }

    pub fn blue_on(&mut self) {
        self.blue.low();
        self.is_blue_on = true;
    }

    pub fn blue_off(&mut self) {
        self.blue.high();
        self.is_blue_on = false;
    }

    pub fn blue_toggle(&mut self) {
        if self.is_blue_on {
            self.blue_off();
        } else {
            self.blue_on();
        }
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
