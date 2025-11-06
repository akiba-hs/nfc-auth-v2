# Akiba NFC Authentication (door)

## Build and flash

From this guide: https://github.com/esp-rs/esp-idf-template?tab=readme-ov-file#prerequisites

Install Rust (https://rustup.rs)

Install packages for building esp-idf:
```
apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

Install tools:
```
cargo install ldproxy
cargo install espup
cargo install espflash
```

Install Rust fork for ESP targets:
```
espup install
```

Setup environment to use Rust fork (need to run every time when building the project) and build:
```
. ~/export-esp.sh
cargo build --release
```

Connect to ESP32 board, flash, and connect to UART for logs:
```
espflash flash target/xtensa-esp32-espidf/release/akiba-nfc-auth-v2 -M -B 115200
```