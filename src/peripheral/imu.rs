use esp_backtrace as _;

use crate::peripheral_traits as p;
use crate::peripheral_traits::{CsPin, SpiInterface};

pub struct Imu<Spi: SpiInterface, Cs: CsPin, Delay: p::Delay> {
    spi: Spi,
    cs: Cs,
    delay: Delay,
}

const CS_DELAY: u32 = 1;

impl<Spi, Cs, Delay> Imu<Spi, Cs, Delay>
where
    Spi: SpiInterface,
    Cs: CsPin,
    Delay: p::Delay,
{
    pub fn new(spi: Spi, cs: Cs, delay: Delay) -> Self {
        let mut imu = Imu {
            spi: spi,
            cs: cs,
            delay: delay,
        };

        // Enable access to registers
        imu.cs.assert();
        imu.delay.us(CS_DELAY);
        let mut data = [0x01, 0x80];
        imu.spi.communicate(&mut data).unwrap();
        imu.delay.us(CS_DELAY);
        imu.cs.negate();

        // IMU : Set Gyro to high-performance mode
        imu.cs.assert();
        imu.delay.us(CS_DELAY);
        let mut data = [0x11, 0xac];
        imu.spi.communicate(&mut data).unwrap();
        imu.delay.us(CS_DELAY);
        imu.cs.negate();

        imu
    }

    pub fn read(&mut self) -> i16 {
        self.cs.assert();
        self.delay.us(CS_DELAY);
        let mut data = [0xa6, 0xff, 0xff]; // Gyro yaw
        self.spi.communicate(&mut data).unwrap();
        self.delay.us(CS_DELAY);
        self.cs.negate();
        ((data[2] as i16) << 8) | (data[1] as i16)
    }

    pub fn who_am_i(&mut self) -> [u8; 2] {
        self.cs.assert();
        self.delay.us(CS_DELAY);
        let mut data = [0x8f, 0xff];
        self.spi.communicate(&mut data).unwrap();
        self.delay.us(CS_DELAY);
        self.cs.negate();
        data
    }
}
