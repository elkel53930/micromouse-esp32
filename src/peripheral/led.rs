use esp_backtrace as _;

use crate::peripheral_traits::OutputPin;

pub enum LED {
    Red = 0,
    Green = 1,
    Blue = 2,
}

pub struct Led<R: OutputPin, G: OutputPin, B: OutputPin> {
    red: R,
    green: G,
    blue: B,

    is_red_on: bool,
    is_green_on: bool,
    is_blue_on: bool,

    red_step: usize,
    green_step: usize,
    blue_step: usize,
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
            red_step: 0,
            green_step: 0,
            blue_step: 0,
        };

        led.red.high();
        led.green.high();
        led.blue.high();

        led
    }

    pub fn on(&mut self, led: LED) {
        match led {
            LED::Red => self.red_on(),
            LED::Green => self.green_on(),
            LED::Blue => self.blue_on(),
        }
    }

    pub fn off(&mut self, led: LED) {
        match led {
            LED::Red => self.red_off(),
            LED::Green => self.green_off(),
            LED::Blue => self.blue_off(),
        }
    }

    pub fn toggle(&mut self, led: LED) {
        match led {
            LED::Red => self.red_toggle(),
            LED::Green => self.green_toggle(),
            LED::Blue => self.blue_toggle(),
        }
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

    fn pattern_internal(&mut self, pattern: &str, step: usize, led: LED) -> usize {
        let pattern = pattern.as_bytes();
        let on = b'1';
        let off = b'0';
        let mut step = step;

        if pattern.len() == 0 {
            return 0;
        }

        if pattern.len() <= step {
            step = 0;
        }

        match pattern[step] {
            x if x == on => self.on(led),
            x if x == off => self.off(led),
            _ => {}
        }
        step + 1
    }

    pub fn pattern(&mut self, r_pattern: &str, g_pattern: &str, b_pattern: &str) {
        self.red_step = self.pattern_internal(r_pattern, self.red_step, LED::Red);
        self.green_step = self.pattern_internal(g_pattern, self.green_step, LED::Green);
        self.blue_step = self.pattern_internal(b_pattern, self.blue_step, LED::Blue);
    }
}
