use crate::log;
use embedded_hal::serial::Read;
use crate::uart_read_line::read_line;

const NUM_OF_COMMAND: usize = 1;
pub struct Console<'a> {
    commands: [&'a dyn ConsoleCommand; NUM_OF_COMMAND],
}

impl<'a> Console<'a> {
    pub fn new() -> Console<'a> {
        let commands: [&'a dyn ConsoleCommand; NUM_OF_COMMAND] = [
            &CmdMot {}
        ];

        Console { commands }
    }

    pub fn run<T>(&mut self, uart: &mut T)
    where
        T: Read<u8>,{
        loop {
            let mut buf = [0u8; 256];
            let mut args = [""; 16];
            let mut arg_num = 0;
            let mut len = 0;
            esp_println::print!("> ");
            read_line(uart, &mut buf);

            while buf[len] != 0 {
                len += 1;
            }

            if len == 0 {
                continue;
            }

            let mut i = 0;
            let mut arg_start = 0;
            while i < len {
                if buf[i] == ' ' as u8 {
                    args[arg_num] = core::str::from_utf8(&buf[arg_start..i]).unwrap();
                    arg_num += 1;
                    arg_start = i + 1;
                }
                i += 1;
            }
            args[arg_num] = core::str::from_utf8(&buf[arg_start..i]).unwrap();
            arg_num += 1;

            if arg_num == 0 {
                continue;
            }

            let mut found = false;
            for cmd in self.commands.iter_mut() {
                if cmd.name() == args[0] {
                    cmd.execute(&args[1..arg_num], arg_num - 1);
                    found = true;
                    break;
                }
            }
            if !found {
                esp_println::println!("Command not found: {}", args[0]);
            }
        }
    }
}

pub trait ConsoleCommand {
    fn execute(&self, args: &[&str], arg_num: usize);
    fn hint(&self);
    fn name(&self) -> &str;
}

/* showlog command */
pub struct CmdShowlog {}

impl ConsoleCommand for CmdShowlog {
    fn execute(&self, args: &[&str], arg_num: usize) {
        if arg_num == 1 || arg_num == 0 {
            let offset;

            if args.len() == 0 {
                offset = 256;
            } else {
                match args[0].parse::<u16>() {
                    Ok(num) => offset = num,
                    Err(_) => {
                        esp_println::println!("Failed to parse the argument as a number");
                        return;
                    }
                }
            }
            
            let mut data = [0u8; 24];
            let num_of_part = offset / data.len() as u16;
            let tail = offset % data.len() as u16;
            for i in 0..num_of_part {
                log::read_log_chunk(offset - data.len() as u16 * i, &mut data);
                esp_println::print!("{}", core::str::from_utf8(&data).unwrap());
            }
            if tail != 0 {
                log::read_log_chunk(tail, &mut data[0..tail as usize]);
                data[tail as usize] = 0;
                esp_println::print!("{}", core::str::from_utf8(&data[0..(tail-1) as usize]).unwrap());
            }
            esp_println::println!("");
        } else {
            esp_println::println!("Invalid number of arguments. expected 0 or 1, got {}", args.len());
            self.hint();
        }
    }

    fn hint(&self) {
        esp_println::println!("Usage: showlog [bytes]");
    }

    fn name(&self) -> &str {
        "showlog"
    }
}
