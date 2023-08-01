use crate::peripheral_traits;
use crate::peripheral_traits::RandomAccessMemory;
use core::fmt::{self, Write};
use esp_backtrace as _;

use crate::fram;
use core::cell::RefCell;
use critical_section::{with, Mutex};
use hal::{i2c::I2C, peripherals::I2C0};

const LOG_HEADER_SIZE: u16 = 2;

type Global<T> = Mutex<RefCell<Option<T>>>;
static GL_FRAM_WRITER: Global<FramLogger<fram::Fram<I2C<'_, I2C0>>>> =
    Mutex::new(RefCell::new(None));

pub fn init_logger(i2c: I2C<'static, I2C0>) {
    let fram = fram::Fram::new(i2c);
    let fram_writer = FramLogger::new(fram);
    with(|cs| {
        GL_FRAM_WRITER.borrow(cs).replace(Some(fram_writer));
    });
}

pub fn reset_cursor() {
    with(|cs| {
        GL_FRAM_WRITER
            .borrow(cs)
            .borrow_mut()
            .as_mut()
            .unwrap()
            .set_cursor(LOG_HEADER_SIZE);
    });
}

pub fn read_log(address: u16, data: &mut [u8]) {
    with(|cs| {
        GL_FRAM_WRITER
            .borrow(cs)
            .borrow_mut()
            .as_mut()
            .unwrap()
            .memory()
            .read(address + LOG_HEADER_SIZE, data)
            .unwrap();
    });
}

#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => ($crate::log::_print(format_args!($($arg)*)));
}

#[macro_export]
macro_rules! log {
    ($fmt:expr) => (print!(concat!($fmt, "\n")));
    ($fmt:expr, $($arg:tt)*) => (print!(concat!($fmt, "\n"), $($arg)*));
}

pub fn _print(args: fmt::Arguments) {
    let mut writer = LogWriter {};
    writer.write_fmt(args).unwrap();
}

const BUFFER_SIZE: usize = 250;

pub struct LogWriter {}

impl Write for LogWriter {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        with(|cs| {
            GL_FRAM_WRITER
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .write_str(s)
        })
    }
}

pub struct FramLogger<Memory: peripheral_traits::RandomAccessMemory<u16>> {
    memory: Memory,
    fram_cursor: u16,
    buffer: [u8; BUFFER_SIZE],
    buffer_cursor: usize,
}

impl<Memory> FramLogger<Memory>
where
    Memory: peripheral_traits::RandomAccessMemory<u16>,
{
    pub fn new(memory: Memory) -> Self {
        let mut logger = FramLogger {
            memory: memory,
            fram_cursor: 0, // 0..FRAM size. basically, loop from LOG_HEADER_SIZE to FRAM size
            buffer: [0; BUFFER_SIZE],
            buffer_cursor: 0,
        };

        let mut buf: [u8; 2] = [0; 2];

        logger.memory.read(0x0000, &mut buf).unwrap();
        logger.fram_cursor = (buf[1] as u16) * 256 + buf[0] as u16;

        logger
    }

    fn flush(&mut self) {
        let mut write_from: usize = 0;
        let write_to: usize = self.buffer_cursor as usize;

        // If the buffer is full, rotate to the first.
        // TODO: this rotation code is not tested.
        if self.buffer_cursor + self.fram_cursor as usize >= self.memory.size() as usize {
            let remaining_size = self.memory.size() as usize - self.fram_cursor as usize;
            self.memory.write(self.fram_cursor, &self.buffer[0..remaining_size]).unwrap();
            self.fram_cursor = LOG_HEADER_SIZE;
            write_from = remaining_size;
        }

        self.memory.write(self.fram_cursor, &self.buffer[write_from..write_to]).unwrap();
        self.fram_cursor += self.buffer_cursor as u16;
        self.memory
            .write(0x0000, &self.fram_cursor.to_le_bytes())
            .unwrap();
        self.buffer_cursor = 0;
    }

    fn write_byte(&mut self, byte: u8) {
        self.buffer[self.buffer_cursor] = byte;
        self.buffer_cursor += 1;
        if self.buffer_cursor == BUFFER_SIZE || byte == b'\n' {
            self.flush();
        }
    }

    pub fn write_str(&mut self, s: &str) -> fmt::Result {
        for c in s.bytes() {
            self.write_byte(c);
        }
        Ok(())
    }

    pub fn set_cursor(&mut self, cursor: u16) {
        self.fram_cursor = cursor;
        self.memory
            .write(0x0000, &self.fram_cursor.to_le_bytes())
            .unwrap();
    }

    pub fn memory(&mut self) -> &mut Memory {
        &mut self.memory
    }

    /* 
     case 1
         HHab cdef ghij klmn opqr fram_size = 20
                        ^ cur = 13
         read_recent_log(10 ,data)
         The result will be "bcdefghijk".

     case 2
         HHab cdef ghij klmn opqr fram_size = 20
                ^ cur = 7
         read_recent_log(10 ,data)
         The result will be "mnopqrabcd".
              
     */
    pub fn read_recent_log(&mut self, len: u16, data: &mut [u8]) -> Result<(), ()> {
        if len as usize > data.len() {
            return Err(());
        }

        let total_log_capacity = self.memory.size() as u16 - LOG_HEADER_SIZE;

        if len > total_log_capacity {
            return Err(());
        }

        if self.fram_cursor > len + LOG_HEADER_SIZE + 1 {
            // case 2
            let tail_size = (len - LOG_HEADER_SIZE - 1) as usize;
            let adrs = self.memory.size() as u16 - tail_size as u16;
            self.memory.read(adrs, &mut data[0..tail_size]);
            self.memory.read(adrs, &mut data[tail_size..data.len()]);
        } else {
            // case 1
            let adrs = self.fram_cursor - len - 1;
            self.memory.read(adrs, data);
        }
        return Ok(());
    }

    // Convert relative address to absolute address
    fn rlt_to_abs_internal(&self, size: u16, cursor: u16, rlt_adrs: u16) -> u16 {
        let rlt_adrs = rlt_adrs % size;
    
        if cursor > rlt_adrs {
            cursor - rlt_adrs - 1
        } else {
            let tail_size = rlt_adrs - cursor;
            size - tail_size - 1
        }
    }
    
    const LOG_HEADER_SIZE: u16 = 2;
    
    fn rlt_to_abs(&self, rlt_adrs: u16) -> u16 {
        let size = self.memory.size() as u16 - LOG_HEADER_SIZE;
        let cursor = self.fram_cursor - LOG_HEADER_SIZE;
        self.rlt_to_abs_internal(size, cursor, rlt_adrs) + LOG_HEADER_SIZE
    }
    

    pub fn read_log(&mut self, from: u16, data: &mut [u8]) -> Result<(), ()> {
        for i in range(data.len()){

        }
        Ok(())
    }
}
