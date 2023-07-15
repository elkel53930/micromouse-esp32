use esp_backtrace as _;

use crate::peripheral_traits;

pub struct Motor<PWM: peripheral_traits::Pwm, CWCCW: peripheral_traits::OutputPin> {
    pwm: PWM,
    cwccw: CWCCW,
}

impl<PWM, CWCCW> Motor<PWM, CWCCW>
where
    PWM: peripheral_traits::Pwm,
    CWCCW: peripheral_traits::OutputPin,
{
    pub fn new(pwm: PWM, cwccw: CWCCW) -> Self {
        let mut motor = Motor {
            pwm: pwm,
            cwccw: cwccw,
        };

        motor.pwm.set_duty(0);
        motor.cwccw.high();

        motor
    }

    // TODO: set_duty() should be updated.
    // THe duty logic changes depending on the CWCCW.
    // Saturation treatment for the duty ratio is required.
    pub fn set_duty(&mut self, duty: i16) {
        let mut duty = duty;
        if duty < 0 {
            self.cwccw.low();
            duty = -duty;
        } else {
            self.cwccw.high();
        }
        self.pwm.set_duty(duty as u16);
    }

}
