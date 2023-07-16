pub trait SpiInterface {
    fn communicate(&mut self, data: &mut [u8]) -> Result<(), ()>;
}

pub trait CsPin {
    // Assert CS pin. If negative logic, make the pin low.
    fn assert(&mut self);

    // Negate CS pin. If negative logic, make the pin high.
    fn negate(&mut self);
}

pub trait OutputPin {
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

pub trait Pwm {
    fn set_duty(&mut self, duty: u16);
}

pub trait I2cInterface {
    fn write_to(&mut self, address: u8, data: &[u8]) -> Result<(), ()>;
    fn read_from(&mut self, address: u8, data: &mut [u8]) -> Result<(), ()>;
}

pub trait RandomAccessMemory<Address> {
    fn write(&mut self, address: Address, data: &[u8]) -> Result<(), ()>;
    fn read(&mut self, address: Address, data: &mut [u8]) -> Result<(), ()>;
    fn capacity(&mut self) -> u32;
}
