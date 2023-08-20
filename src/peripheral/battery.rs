use esp_backtrace as _;

use crate::peripheral_traits::AdConverter;

pub struct Battery<AdcBatt>
where
    AdcBatt: AdConverter,
{
    adc_batt: AdcBatt,
}

impl<AdcBatt> Battery<AdcBatt>
where
    AdcBatt: AdConverter,
{
    pub fn new(adc_batt: AdcBatt) -> Self {
        Self { adc_batt }
    }

    pub fn read_mv(&mut self) -> f32 {
        let raw = self.adc_batt.read();
        raw as f32 / 492.0
    }
}
