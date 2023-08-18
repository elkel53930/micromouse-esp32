use crate::uart_read_line::read_line;
use crate::{log, GL_BATTERY, GL_MOTOR_L, GL_MOTOR_R};
use embedded_hal::serial::Read;

use core::str::FromStr;

const NUM_OF_COMMAND: usize = 3;
pub struct Console<'a> {
    commands: [&'a dyn ConsoleCommand; NUM_OF_COMMAND],
}

impl<'a> Console<'a> {
    pub fn new() -> Console<'a> {
        let commands: [&'a dyn ConsoleCommand; NUM_OF_COMMAND] =
            [&CmdShowlog {}, &CmdMot {}, &CmtBatt {}];

        Console { commands }
    }

    pub fn run<T>(&mut self, uart: &mut T)
    where
        T: Read<u8>,
    {
        esp_println::println!("Welcome to ExtraICE console!");
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
                    match cmd.execute(&args[1..arg_num], arg_num - 1) {
                        Ok(_) => {}
                        Err(e) => {
                            esp_println::println!("Error: {}", e);
                            cmd.hint();
                        }
                    }
                    found = true;
                    break;
                }
            }
            if !found {
                esp_println::println!("Command not found: '{}'", args[0]);
            }
        }
    }
}

pub trait ConsoleCommand {
    fn execute(&self, args: &[&str], arg_num: usize) -> Result<(), &'static str>;
    fn hint(&self);
    fn name(&self) -> &str;
}

/* showlog command */
pub struct CmdShowlog {}

fn parse_or_error<T: FromStr>(arg: &str) -> Result<T, &'static str> {
    arg.parse::<T>().map_err(|_| "Argument parse error")
}

impl ConsoleCommand for CmdShowlog {
    fn execute(&self, args: &[&str], arg_num: usize) -> Result<(), &'static str> {
        if arg_num == 1 || arg_num == 0 {
            let offset;

            if args.len() == 0 {
                offset = 256;
            } else {
                offset = parse_or_error(args[0])?;
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
                esp_println::print!(
                    "{}",
                    core::str::from_utf8(&data[0..(tail - 1) as usize]).unwrap()
                );
            }
            esp_println::println!("");
        } else {
            return Err("invalid number of arguments");
        }
        Ok(())
    }

    fn hint(&self) {
        esp_println::println!("Usage: showlog [bytes]");
    }

    fn name(&self) -> &str {
        "showlog"
    }
}

/* mot command */
pub struct CmdMot {}

impl ConsoleCommand for CmdMot {
    fn execute(&self, args: &[&str], arg_num: usize) -> Result<(), &'static str> {
        if arg_num == 2 {
            let lspeed = parse_or_error(args[0])?;
            let rspeed = parse_or_error(args[1])?;
            unsafe {
                GL_MOTOR_L.as_mut().unwrap().set_duty(lspeed);
                GL_MOTOR_R.as_mut().unwrap().set_duty(rspeed);
            }
        } else if arg_num == 0 {
            unsafe {
                GL_MOTOR_L.as_mut().unwrap().set_duty(0);
                GL_MOTOR_R.as_mut().unwrap().set_duty(0);
            }
        } else {
            return Err("invalid number of arguments");
        }
        Ok(())
    }

    fn hint(&self) {
        esp_println::println!("Usage: mot [lspeed] [rspeed]");
        esp_println::println!(" lspeed -100 - 100");
        esp_println::println!(" rspeed -100 - 100");
    }

    fn name(&self) -> &str {
        "mot"
    }
}

/* batt command */
pub struct CmtBatt {}

impl ConsoleCommand for CmtBatt {
    fn execute(&self, args: &[&str], arg_num: usize) -> Result<(), &'static str> {
        if arg_num == 0 {
            let batt = unsafe { GL_BATTERY.as_mut().unwrap().read_mv() };
            esp_println::println!("Battery: {:.3}V", batt);
        } else if arg_num == 0 {
            unsafe {
                GL_MOTOR_L.as_mut().unwrap().set_duty(0);
                GL_MOTOR_R.as_mut().unwrap().set_duty(0);
            }
        } else {
            return Err("invalid number of arguments");
        }
        Ok(())
    }

    fn hint(&self) {
        esp_println::println!("Usage: batt");
    }

    fn name(&self) -> &str {
        "batt"
    }
}
