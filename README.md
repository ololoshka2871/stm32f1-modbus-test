# README
Тестер Modbus-RTU для stm32f103c8t6

## Tools
* Rust nightly
* [Rtic](https://rtic.rs/1/book/en/)
* [libremodbus](https://github.com/nucleron/libremodbus)

## Build
```
$ rustup target add thumbv7em-none-eabi
$ cargo build
```

## Memory Map

#### Holding registers
Address | Data Type | Size (cells) | Access | Description 
-------:|:----------|:-------------|:-------|:------------------------------
0x0000  | Uint16    | 1            | R0     | Device ID code
0x0010  | Uint32    | 2            | R0     | Device HW version
0x0012  | Uint32    | 2            | R0     | MCU ID code
0x0020  | Uint32    | 2            | R0     | Device SW version
0x0100  | Uint16    | 1            | RW     | Test value

## Input registers
Address | Data Type | Size (cells) | Description 
-------:|:----------|:-------------|:---------------------
0x0000  | Uint16    | 1            | Seconds counter
0x0100  | Uint16    | 1            | Test value output
