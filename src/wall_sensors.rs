use esp_backtrace as _;

// import periferal_traits
use crate::peripheral_traits::{AdConverter, OutputPin};

pub struct WallSensors<Ena, Sel0, Sel1, AdcLS, AdcLF, AdcRF, AdcRS>
where
    Ena: OutputPin,
    Sel0: OutputPin,
    Sel1: OutputPin,
    AdcLS: AdConverter,
    AdcLF: AdConverter,
    AdcRF: AdConverter,
    AdcRS: AdConverter,
{
    adc_ls: AdcLS,
    adc_lf: AdcLF,
    adc_rf: AdcRF,
    adc_rs: AdcRS,
    led_ena: Ena,
    led_sel0: Sel0,
    led_sel1: Sel1,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WallSensor {
    LS = 0,
    LF = 1,
    RF = 2,
    RS = 3,
}

/* Wall sensor LED */
/*
    SEL0 SEL1   LED     PIN
      1    0   LS(D1)  GPIO1
      1    1   LF(D2)  GPIO2
      0    1   RF(D3)  GPIO3
      0    0   RS(D4)  GPIO4
*/
const SEL: [(bool, bool); 4] = [(true, false), (true, true), (false, true), (false, false)];

impl<Ena, Sel0, Sel1, AdcLF, AdcLS, AdcRF, AdcRS>
    WallSensors<Ena, Sel0, Sel1, AdcLS, AdcLF, AdcRF, AdcRS>
where
    Ena: OutputPin,
    Sel0: OutputPin,
    Sel1: OutputPin,
    AdcLF: AdConverter,
    AdcLS: AdConverter,
    AdcRF: AdConverter,
    AdcRS: AdConverter,
{
    pub fn new(
        adc_lf: AdcLF,
        adc_ls: AdcLS,
        adc_rs: AdcRS,
        adc_rf: AdcRF,
        led_ena: Ena,
        led_sel0: Sel0,
        led_sel1: Sel1,
    ) -> Self {
        WallSensors {
            adc_ls: adc_ls,
            adc_lf: adc_lf,
            adc_rf: adc_rf,
            adc_rs: adc_rs,
            led_ena: led_ena,
            led_sel0: led_sel0,
            led_sel1: led_sel1,
        }
    }

    pub fn enable(&mut self, sen: WallSensor) {
        if SEL[sen as usize].0 {
            self.led_sel0.high();
        } else {
            self.led_sel0.low();
        }
        if SEL[sen as usize].1 {
            self.led_sel1.high();
        } else {
            self.led_sel1.low();
        }
        self.led_ena.high();
    }

    pub fn disable(&mut self) {
        self.led_ena.low();
    }

    pub fn read(&mut self, sel: WallSensor) -> u16 {
        match sel {
            WallSensor::LS => self.adc_ls.read(),
            WallSensor::LF => self.adc_lf.read(),
            WallSensor::RF => self.adc_rf.read(),
            WallSensor::RS => self.adc_rs.read(),
        }
    }
}
