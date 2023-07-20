use embedded_hal::serial::Read;


pub fn read_line<T>(uart:&mut T, buffer: &mut [u8])
    where T: Read<u8>{
    let mut i = 0;
    loop {
        if 
        let Ok(byte) = uart.read(){
            if byte == b'\n' || byte == b'\r' {
                break;
            }
            buffer[i] = byte;
        } else {
            continue;
        }
        i += 1;
        // if the buffet is full, break
        if i == buffer.len() {
            break;
        }
    }
}
