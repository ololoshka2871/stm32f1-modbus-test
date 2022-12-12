# README
ШИМ контроллер на 20 каналов с интерфейсом ModbusRTU.
16 основных каналов при помощи микросхемы pca9685, и 4 дополнительных канала средствами STM32.

## Tools
* Rust nightly
* Rtic
* free-modbus
* pwm-pca9685

## Build
```
$ rustup toolchain install nightly
$ rustup target add thumbv7em-none-eabi
$ git clone --recurcive https://github.com/ololoshka2871/pwm-20ch
$ cd pwm-20ch
$ rustup override set nightly
$ cargo build
```

# [cargo make](https://sagiegurari.github.io/cargo-make/)
1. openocd - start Openocd server
2. flash - flash firmware, (start openocd server first!)

# Connection
![](img/mcu_conn.png)

### pca9685's 16 channels 
* CH 0-3, 8-11 - "прямые"
* CH 4-7, 12-15 - "инверсные"

### MCU's channels
* CH1 (PA8) - "инверсный"
* CH2 (PA9) - "инверсный"
* CH3 (PA10) - "прямой"
* CH4 (PA11) - "прямой"

## Подключение к "снаряду"

### 16 канальный режим
![](img/16_ch.png)

### 20 канальный режим
![](img/20_ch.png)

## Memory Map

#### Holding registers
Address | Data Type | Size (cells) | Access | Description 
-------:|:----------|:-------------|:-------|:------------------------------
0x0000  | Uint16    | 1            | R0     | Device ID code
0x0010  | Uint32    | 2            | R0     | Device HW version
0x0012  | Uint32    | 2            | R0     | MCU ID code
0x0020  | Uint32    | 2            | R0     | Device SW version
0x0100  | Uint16    | 1            | RW     | C0 Duty (0-4095)
0x0101  | Uint16    | 1            | RW     | C1 Duty (0-4095)
0x0102  | Uint16    | 1            | RW     | C2 Duty (0-4095)
0x0103  | Uint16    | 1            | RW     | C3 Duty (0-4095)
0x0104  | Uint16    | 1            | RW     | C4 Duty (0-4095)
0x0105  | Uint16    | 1            | RW     | C5 Duty (0-4095)
0x0106  | Uint16    | 1            | RW     | C6 Duty (0-4095)
0x0107  | Uint16    | 1            | RW     | C7 Duty (0-4095)
0x0108  | Uint16    | 1            | RW     | C8 Duty (0-4095)
0x0109  | Uint16    | 1            | RW     | C9 Duty (0-4095)
0x010A  | Uint16    | 1            | RW     | C10 Duty (0-4095)
0x010B  | Uint16    | 1            | RW     | C11 Duty (0-4095)
0x010C  | Uint16    | 1            | RW     | C12 Duty (0-4095)
0x010D  | Uint16    | 1            | RW     | C13 Duty (0-4095)
0x010E  | Uint16    | 1            | RW     | C14 Duty (0-4095)
0x010F  | Uint16    | 1            | RW     | C15 Duty (0-4095)
0x0110  | Uint16    | 1            | RW     | MC1 Duty (0-4095)
0x0111  | Uint16    | 1            | RW     | MC2 Duty (0-4095)
0x0112  | Uint16    | 1            | RW     | MC3 Duty (0-4095)
0x0113  | Uint16    | 1            | RW     | MC4 Duty (0-4095)
0x0200  | Uint16    | 1            | RW     | PWM frequency (Hz) (50 - 10000)

## Input registers
Address | Data Type | Size (cells) | Description 
-------:|:----------|:-------------|:---------------------
0x0000  | Float32   | 2            | Total output load (%)
0x0100  | Float32   | 2            | Total load 0-10 (%)
0x0102  | Float32   | 2            | Total load 11-20 (%)
0x0104  | Float32   | 2            | Total load 21-30 (%)
0x0106  | Float32   | 2            | Total load 31-40 (%)
0x0108  | Float32   | 2            | Total load 41-50 (%)
0x010A  | Float32   | 2            | Total load 51-60 (%)
0x010C  | Float32   | 2            | Total load 61-70 (%)
0x010E  | Float32   | 2            | Total load 71-80 (%)
0x0110  | Float32   | 2            | Total load 81-90 (%)
0x0112  | Float32   | 2            | Total load 91-100 (%)