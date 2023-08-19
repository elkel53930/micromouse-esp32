use crate::peripheral_traits;
use crate::peripheral_traits::RandomAccessMemory;
use core::fmt::{self, Write};
use esp_backtrace as _;

use crate::fram;
use core::cell::RefCell;
use critical_section::{with, Mutex};
use hal::{i2c::I2C, peripherals::I2C0};

const LOG_HEADER_SIZE: u16 = 2;

static mut GL_FRAM_WRITER: Option<FramLogger<fram::Fram<I2C<'_, I2C0>>>> = None;

pub fn init_logger(i2c: I2C<'static, I2C0>) {
    let fram = fram::Fram::new(i2c);
    let fram_writer = FramLogger::new(fram);
    unsafe {
        GL_FRAM_WRITER = Some(fram_writer);
    }
}

pub fn is_initialized() {
    unsafe { GL_FRAM_WRITER.is_some() }
}

pub fn reset_cursor() {
    unsafe {
        GL_FRAM_WRITER.as_mut().unwrap().set_cursor(LOG_HEADER_SIZE);
    }
}

pub fn read_recent_log(data: &mut [u8]) {
    unsafe {
        GL_FRAM_WRITER
            .as_mut()
            .unwrap()
            .read_recent_log(data)
            .unwrap();
    }
}

pub fn read_log_chunk(from: u16, data: &mut [u8]) {
    unsafe {
        GL_FRAM_WRITER
            .as_mut()
            .unwrap()
            .read_log_chunk(from, data)
            .unwrap();
    }
}

pub fn read_log(adrs: u16, data: &mut [u8]) {
    unsafe {
        GL_FRAM_WRITER
            .as_mut()
            .unwrap()
            .read_log(adrs, data)
            .unwrap();
    }
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
        unsafe { GL_FRAM_WRITER.as_mut().unwrap().write_str(s) }
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
            self.memory
                .write(self.fram_cursor, &self.buffer[0..remaining_size])
                .unwrap();
            self.fram_cursor = LOG_HEADER_SIZE;
            write_from = remaining_size;
        }

        self.memory
            .write(self.fram_cursor, &self.buffer[write_from..write_to])
            .unwrap();
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
    pub fn read_recent_log(&mut self, data: &mut [u8]) -> Result<(), ()> {
        let len = data.len() as u16;

        let total_log_capacity = self.memory.size() as u16 - LOG_HEADER_SIZE;

        if len > total_log_capacity {
            return Err(());
        }

        if self.fram_cursor > len + LOG_HEADER_SIZE + 1 {
            // case 1
            let adrs = self.fram_cursor - len - 1;
            self.memory.read(adrs, data).unwrap();
        } else {
            // case 2
            let tail_size = (len - self.fram_cursor + LOG_HEADER_SIZE + 1) as usize;
            let adrs = self.memory.size() as u16 - tail_size as u16;
            let len = data.len();
            self.memory.read(adrs, &mut data[0..tail_size]).unwrap();
            self.memory
                .read(LOG_HEADER_SIZE, &mut data[tail_size..len])
                .unwrap();
        }
        return Ok(());
    }

    pub fn read_log_chunk(&mut self, from: u16, data: &mut [u8]) -> Result<(), ()> {
        //        esp_println::println!("read_log_chunk({:?}, {:?})", from, data);

        let cursor = self.fram_cursor - LOG_HEADER_SIZE;
        let total_size = self.memory.size() as u16 - LOG_HEADER_SIZE;
        let adjusted_from = from % total_size;
        let start_address = if cursor >= adjusted_from {
            cursor - adjusted_from
        } else {
            total_size - (adjusted_from - cursor)
        };

        let data_len = data.len() as u16;

        //        esp_println::println!("cursor: {:?}", cursor);
        //        esp_println::println!("total_size: {:?}", total_size);
        //        esp_println::println!("adjusted_from: {:?}", adjusted_from);
        //        esp_println::println!("start_address: {:?}", start_address);
        //        esp_println::println!("data_len: {:?}", data_len);

        // Reading case where it wraps around the ring buffer
        if start_address + data_len > total_size {
            let part1_size = total_size.wrapping_sub(start_address);
            let actual_part1_size = core::cmp::min(part1_size, data_len);
            let part2_size = data_len.wrapping_sub(actual_part1_size);

            //            esp_println::println!("part1_size: {:?}", part1_size);
            //            esp_println::println!("actual_part1_size: {:?}", actual_part1_size);
            //            esp_println::println!("part2_size: {:?}", part2_size);

            // Read data up to the end of the buffer
            self.read_log(start_address, &mut data[0..actual_part1_size as usize])?;

            // Read the remaining data from the beginning of the buffer
            if part2_size > 0 {
                self.read_log(0, &mut data[actual_part1_size as usize..])?;
            }
        } else {
            // Reading case where it doesn't wrap around the buffer
            self.read_log(start_address, data)?;
        }

        Ok(())
    }

    /*
        Ignore headers and read logs.
        The adrs specified to this function is different from the address of MEMORY.
        It is offset only by the header size.
        The header is written at address 0 of memory,
        but if you specify 0 for adrs and use this function,
        the header is skipped and the first data of the log body is read.
    */
    pub fn read_log(&mut self, adrs: u16, data: &mut [u8]) -> Result<(), ()> {
        let total_size = self.memory.size() as u16 - LOG_HEADER_SIZE;
        let len = data.len() as u16;

        // If the requested size is larger than the total size of the log, return an error.
        if len > total_size {
            return Err(());
        }

        let read_size = if adrs + len > total_size {
            total_size - adrs
        } else {
            len
        };

        self.memory
            .read(adrs + LOG_HEADER_SIZE, &mut data[0..read_size as usize])
    }
}
