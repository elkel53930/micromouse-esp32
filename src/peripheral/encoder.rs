use esp_backtrace as _;

use crate::peripheral_traits::{CsPin, SpiInterface};

pub struct Encoder<Spi: SpiInterface, CsR: CsPin, CsL: CsPin> {
    spi: Spi,
    cs_r: CsR,
    cs_l: CsL,
}

impl<Spi, CsR, CsL> Encoder<Spi, CsR, CsL>
where
    Spi: SpiInterface,
    CsR: CsPin,
    CsL: CsPin,
{
    pub fn new(spi: Spi, cs_r: CsR, cs_l: CsL) -> Self {
        Encoder {
            spi: spi,
            cs_r: cs_r,
            cs_l: cs_l,
        }
    }

    fn concat(&self, msb: u8, lsb: u8) -> u16 {
        (msb as u16) * 256 + (lsb as u16) & 0x3fff
    }

    pub fn read_r(&mut self) -> u16 {
        let mut data = [0x7f, 0xfe];
        self.cs_r.assert();
        self.spi.communicate(&mut data).unwrap();
        self.cs_r.negate();

        self.concat(data[0], data[1])
    }

    pub fn read_l(&mut self) -> u16 {
        let mut data = [0x7f, 0xfe];
        self.cs_l.assert();
        self.spi.communicate(&mut data).unwrap();
        self.cs_l.negate();

        self.concat(data[0], data[1])
    }
}
