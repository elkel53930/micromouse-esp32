pub trait SpiInterface {
    fn communicate(&mut self, data: &mut [u8]) -> Result<(), ()>;
}

pub trait CsPin {
    // Assert CS pin. If negative logic, make the pin low.
    fn assert(&mut self);

    // Negate CS pin. If negative logic, make the pin high.
    fn negate(&mut self);
}

pub trait GpioPin {
    // Set pin high
    fn high(&mut self);

    // Set pin low
    fn low(&mut self);
}

pub trait AdConverter {
    fn read(&mut self) -> u16;
}

pub trait Delay {
    fn ms(&mut self, ms: u32);
    fn us(&mut self, us: u32);
}
