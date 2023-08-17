# micromouse-esp32

A micromouse firmware for ESP32-S3.

This project is still in very early stages.

# Requirements

'esp-hal' is required.

For details, see `Cargo.toml`.

# Environment setup

See [The Rust on ESP Book](https://esp-rs.github.io/book/) for instructions on how to set up the development environment.

# Write & Run

## Write & Run with espflash

```bash
cargo espflash {your-tty-device} --monitor
```
## Monitor with picocom

```bash
picocom {your-tty-device} -b 115200 --imap lfcrlf --echo
```

# Note

T.B.D.