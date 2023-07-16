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
static GL_FRAM_WRITER: Global<FramWriter<fram::Fram<I2C<'_, I2C0>>>> =
    Mutex::new(RefCell::new(None));

pub fn init_logger(i2c: I2C<'static, I2C0>) {
    let fram = fram::Fram::new(i2c);
    let fram_writer = FramWriter::new(fram);
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

pub struct FramWriter<Memory: peripheral_traits::RandomAccessMemory<u16>> {
    memory: Memory,
    cursor: u16,
    buffer: [u8; BUFFER_SIZE],
    buffer_cursor: usize,
}

impl<Memory> FramWriter<Memory>
where
    Memory: peripheral_traits::RandomAccessMemory<u16>,
{
    pub fn new(memory: Memory) -> Self {
        let mut logger = FramWriter {
            memory: memory,
            cursor: 0,
            buffer: [0; BUFFER_SIZE],
            buffer_cursor: 0,
        };

        let mut buf: [u8; 2] = [0; 2];

        logger.memory.read(0x0000, &mut buf).unwrap();
        logger.cursor = (buf[1] as u16) * 256 + buf[0] as u16;

        logger
    }

    fn flush(&mut self) {
        // TODO: log rotation
        self.memory.write(self.cursor, &self.buffer).unwrap();
        self.cursor += self.buffer_cursor as u16;
        self.memory
            .write(0x0000, &self.cursor.to_le_bytes())
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
        self.cursor = cursor;
        self.memory
            .write(0x0000, &self.cursor.to_le_bytes())
            .unwrap();
    }

    pub fn memory(&mut self) -> &mut Memory {
        &mut self.memory
    }
}
