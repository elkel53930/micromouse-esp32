use esp_println::println;

pub trait ConsoleCommand {
    fn execute(&mut self, args: &[&str]);
    fn hint(&self);
    fn name(&self) -> &str;
}

pub struct CmdShowlog {}

impl ConsoleCommand for CmdShowlog {
    fn show_log(&mut self, len: u16) {

    }

    fn execute(&mut self, args: &[&str]) {

        if args.len() == 1 || args.len() == 0 {
            
        } else {
            println!("Invalid number of arguments. expected 0 or 1, got {}", args.len());
        }
    }

    fn hint(&self) {
        println!("Usage: showlog \{\}");
    }

    fn name(&self) -> &str {
        "showlog"
    }
}
