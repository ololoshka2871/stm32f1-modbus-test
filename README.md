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
* CH 1-8 - "прямые"
* CH 9-16 - "инверсные"

### MCU's channels
* CH1 (PA7) - "инверсный"
* CH2 (PA8) - "инверсный"
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
0x0100  | Uint16    | 1            | RW     | Channel 1 Duty (0-4095)
0x0101  | Uint16    | 1            | RW     | Channel 2 Duty (0-4095)
0x0102  | Uint16    | 1            | RW     | Channel 3 Duty (0-4095)
0x0103  | Uint16    | 1            | RW     | Channel 4 Duty (0-4095)
0x0104  | Uint16    | 1            | RW     | Channel 5 Duty (0-4095)
0x0105  | Uint16    | 1            | RW     | Channel 6 Duty (0-4095)
0x0106  | Uint16    | 1            | RW     | Channel 7 Duty (0-4095)
0x0107  | Uint16    | 1            | RW     | Channel 8 Duty (0-4095)
0x0108  | Uint16    | 1            | RW     | Channel 9 Duty (0-4095)
0x0109  | Uint16    | 1            | RW     | Channel 10 Duty (0-4095)
0x010A  | Uint16    | 1            | RW     | Channel 11 Duty (0-4095)
0x010B  | Uint16    | 1            | RW     | Channel 12 Duty (0-4095)
0x010C  | Uint16    | 1            | RW     | Channel 13 Duty (0-4095)
0x010D  | Uint16    | 1            | RW     | Channel 14 Duty (0-4095)
0x010E  | Uint16    | 1            | RW     | Channel 15 Duty (0-4095)
0x010F  | Uint16    | 1            | RW     | Channel 16 Duty (0-4095)
0x0110  | Uint16    | 1            | RW     | Channel 17 Duty (0-4095)
0x0111  | Uint16    | 1            | RW     | Channel 18 Duty (0-4095)
0x0112  | Uint16    | 1            | RW     | Channel 19 Duty (0-4095)
0x0113  | Uint16    | 1            | RW     | Channel 20 Duty (0-4095)
0x0200  | Uint16    | 1            | RW     | PWM frequency (Hz) (50 - 10000)

## Input registers
Address | Data Type | Size (cells) | Description 
-------:|:----------|:-------------|:---------------------
0x0000  | Uint16    | 1            | Total output load (%)
0x0100  | Uint16    | 1            | Total load 0-10 (%)
0x0101  | Uint16    | 1            | Total load 11-20 (%)
0x0102  | Uint16    | 1            | Total load 21-30 (%)
0x0103  | Uint16    | 1            | Total load 31-40 (%)
0x0104  | Uint16    | 1            | Total load 41-50 (%)
0x0105  | Uint16    | 1            | Total load 51-60 (%)
0x0106  | Uint16    | 1            | Total load 61-70 (%)
0x0107  | Uint16    | 1            | Total load 71-80 (%)
0x0108  | Uint16    | 1            | Total load 81-90 (%)
0x0109  | Uint16    | 1            | Total load 91-100 (%)