use crate::read_uart::read_line;
use crate::{log, GL_BATTERY, GL_INTERRUPT_CONTEXT, GL_MOTOR_L, GL_MOTOR_R, GL_SENSOR_DATA};
use embedded_hal::serial::Read;

use core::str::FromStr;

const NUM_OF_COMMAND: usize = 4;
pub struct Console<'a, UART> {
    commands: [&'a dyn ConsoleCommand<UART>; NUM_OF_COMMAND],
}

impl<'a, UART> Console<'a, UART>
where
    UART: Read<u8>,
{
    pub fn new() -> Console<'a, UART> {
        let commands: [&'a dyn ConsoleCommand<UART>; NUM_OF_COMMAND] =
            [&CmdLog {}, &CmdMot {}, &CmtBatt {}, &CmdSen {}];

        Console { commands }
    }

    pub fn run(&mut self, uart: &mut UART) {
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
                    match cmd.execute(&args[1..arg_num], arg_num - 1, uart) {
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

pub trait ConsoleCommand<UART>
where
    UART: Read<u8>,
{
    fn execute(&self, args: &[&str], arg_num: usize, uart: &mut UART) -> Result<(), &'static str>;
    fn hint(&self);
    fn name(&self) -> &str;
}

/* log command */
pub struct CmdLog {}

fn parse_or_error<UART: FromStr>(arg: &str) -> Result<UART, &'static str> {
    arg.parse::<UART>().map_err(|_| "Argument parse error")
}

impl<UART> ConsoleCommand<UART> for CmdLog
where
    UART: Read<u8>,
{
    fn execute(&self, args: &[&str], arg_num: usize, _: &mut UART) -> Result<(), &'static str> {
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
        esp_println::println!("Usage: log [bytes]");
    }

    fn name(&self) -> &str {
        "log"
    }
}

/* mot command */
pub struct CmdMot {}

impl<UART> ConsoleCommand<UART> for CmdMot
where
    UART: Read<u8>,
{
    fn execute(&self, args: &[&str], arg_num: usize, _: &mut UART) -> Result<(), &'static str> {
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
        esp_println::println!(" lspeed -99 - 99");
        esp_println::println!(" rspeed -99 - 99");
        esp_println::println!(" To stop motors, call without arguments");
    }

    fn name(&self) -> &str {
        "mot"
    }
}

/* batt command */
pub struct CmtBatt {}

impl<UART> ConsoleCommand<UART> for CmtBatt
where
    UART: Read<u8>,
{
    fn execute(&self, args: &[&str], arg_num: usize, _: &mut UART) -> Result<(), &'static str> {
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

/* sen command */
pub struct CmdSen {}

/* show all sensor's values */
impl<UART> ConsoleCommand<UART> for CmdSen
where
    UART: Read<u8>,
{
    fn execute(&self, args: &[&str], arg_num: usize, uart: &mut UART) -> Result<(), &'static str> {
        if arg_num == 0 {
            let front_original_state =
                unsafe { GL_INTERRUPT_CONTEXT.as_mut().unwrap().front_sensors };
            let side_original_state =
                unsafe { GL_INTERRUPT_CONTEXT.as_mut().unwrap().side_sensors };
            unsafe {
                GL_INTERRUPT_CONTEXT.as_mut().unwrap().front_sensors = true;
                GL_INTERRUPT_CONTEXT.as_mut().unwrap().side_sensors = true;
            }
            loop {
                let sensors = unsafe { GL_SENSOR_DATA.as_mut().unwrap().clone() };
                esp_println::println!(
                    "WSEN:{:?}, BATT:{:.3}, ENCODERS:{:?}, GYRO:{:4}",
                    sensors.wall_sensors,
                    sensors.battery,
                    sensors.encoders,
                    sensors.gyro_yaw
                );
                if let Ok(byte) = uart.read() {
                    break;
                }
            }
            unsafe {
                GL_INTERRUPT_CONTEXT.as_mut().unwrap().front_sensors = front_original_state;
                GL_INTERRUPT_CONTEXT.as_mut().unwrap().side_sensors = side_original_state;
            }
        } else {
            return Err("invalid number of arguments");
        }
        Ok(())
    }

    fn hint(&self) {
        esp_println::println!("Usage: sen");
    }

    fn name(&self) -> &str {
        "sen"
    }
}
