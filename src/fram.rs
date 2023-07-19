use esp_backtrace as _;

use crate::peripheral_traits;

pub struct Fram<I2C: peripheral_traits::I2cInterface> {
    i2c: I2C,
    buffer: [u8; 254],
}

const I2C_ADDRESS: u8 = 0x50;

impl<I2C> Fram<I2C>
where
    I2C: peripheral_traits::I2cInterface,
{
    pub fn new(i2c: I2C) -> Self {
        Fram {
            i2c: i2c,
            buffer: [0; 254],
        }
    }
}

impl<I2C> peripheral_traits::RandomAccessMemory<u16> for Fram<I2C>
where
    I2C: peripheral_traits::I2cInterface,
{
    fn write(&mut self, address: u16, data: &[u8]) -> Result<(), ()> {
        if data.len() > self.buffer.len() - 2 {
            return Err(());
        }
        self.buffer[0] = (address >> 8) as u8;
        self.buffer[1] = address as u8;
        self.buffer[2..2 + data.len()].copy_from_slice(data);
        self.i2c
            .write_to(I2C_ADDRESS, &self.buffer[0..2 + data.len()])
    }

    fn read(&mut self, address: u16, data: &mut [u8]) -> Result<(), ()> {
        self.buffer[0] = (address >> 8) as u8;
        self.buffer[1] = address as u8;
        self.i2c.write_to(I2C_ADDRESS, &self.buffer[0..2]).unwrap();
        self.i2c.read_from(I2C_ADDRESS, data)
    }

    fn size(&mut self) -> u32 {
        8192
    }
}
