use core::convert::Infallible;

use cortex_m::prelude::{_embedded_hal_serial_Read, _embedded_hal_serial_Write};
use stm32f1xx_hal::device::{USART1, USART2};
use stm32f1xx_hal::{rcc::Clocks, serial::Event};

pub const WAIT_BITS_AFTER_TX_DONE: u32 = 10;

pub struct Serial<SER, PIN> {
    serial: SER,
    re_de: PIN,
    clocks: Clocks,
}

impl<SER, PIN> Serial<SER, PIN> {
    pub fn new(serial: SER, re_de: PIN, clocks: Clocks) -> Self {
        Self {
            serial,
            re_de,
            clocks,
        }
    }
}

impl<PIN, P1, P2> libremodbus_rs::SerialInterface
    for Serial<stm32f1xx_hal::serial::Serial<USART2, (P1, P2)>, PIN>
where
    PIN: embedded_hal::digital::v2::OutputPin<Error = Infallible>,
{
    fn configure(&mut self, boud: u32, data_bits: u8, parity: libremodbus_rs::Parity) -> bool {
        use libremodbus_rs::Parity as MbParity;
        use stm32f1xx_hal::serial::{Parity, WordLength};

        if self
            .serial
            .reconfigure(
                stm32f1xx_hal::serial::Config::default()
                    .baudrate(stm32f1xx_hal::time::Bps(boud))
                    .wordlength(match data_bits {
                        8 => WordLength::Bits8,
                        9 => WordLength::Bits9,
                        _ => unreachable!(),
                    })
                    .parity(match parity {
                        MbParity::None => Parity::ParityNone,
                        MbParity::Even => Parity::ParityEven,
                        MbParity::ODD => Parity::ParityOdd,
                    }),
                self.clocks,
            )
            .is_err()
        {
            return false;
        }
        unsafe { (*USART2::ptr()).cr3.modify(|_, w| w.rtse().set_bit()) };

        true
    }

    fn close(&mut self) {
        self.serial.unlisten(Event::Rxne);
        loop {
            cortex_m::asm::bkpt();
        }
    }

    fn enable(&mut self, rx_enable: bool, tx_enabel: bool) {
        if rx_enable {
            self.serial.listen(Event::Rxne);
        } else {
            self.serial.unlisten(Event::Rxne);
        }

        if tx_enabel {
            self.serial.listen(Event::Txe);
            let _ = self.re_de.set_high();
        } else {
            self.serial.unlisten(Event::Txe);
        }
    }

    fn get_byte(&mut self) -> Option<u8> {
        match self.serial.read() {
            Ok(b) => Some(b),
            Err(_) => None,
        }
    }

    fn put_byte(&mut self, data: u8) -> bool {
        self.serial.write(data).is_ok()
    }

    fn deassert_re_de(&mut self) {
        let _ = self.re_de.set_low();
    }
}

impl<PIN, P1, P2> libremodbus_rs::SerialInterface
    for Serial<stm32f1xx_hal::serial::Serial<USART1, (P1, P2)>, PIN>
where
    PIN: embedded_hal::digital::v2::OutputPin<Error = Infallible>,
{
    fn configure(&mut self, boud: u32, data_bits: u8, parity: libremodbus_rs::Parity) -> bool {
        use libremodbus_rs::Parity as MbParity;
        use stm32f1xx_hal::serial::{Parity, WordLength};

        if self
            .serial
            .reconfigure(
                stm32f1xx_hal::serial::Config::default()
                    .baudrate(stm32f1xx_hal::time::Bps(boud))
                    .wordlength(match data_bits {
                        8 => WordLength::Bits8,
                        9 => WordLength::Bits9,
                        _ => unreachable!(),
                    })
                    .parity(match parity {
                        MbParity::None => Parity::ParityNone,
                        MbParity::Even => Parity::ParityEven,
                        MbParity::ODD => Parity::ParityOdd,
                    }),
                self.clocks,
            )
            .is_err()
        {
            return false;
        }
        unsafe { (*USART1::ptr()).cr3.modify(|_, w| w.rtse().set_bit()) };

        true
    }

    fn close(&mut self) {
        self.serial.unlisten(Event::Rxne);
        loop {
            cortex_m::asm::bkpt();
        }
    }

    fn enable(&mut self, rx_enable: bool, tx_enabel: bool) {
        if rx_enable {
            self.serial.listen(Event::Rxne);
        } else {
            self.serial.unlisten(Event::Rxne);
        }

        if tx_enabel {
            self.serial.listen(Event::Txe);
            let _ = self.re_de.set_high();
        } else {
            self.serial.unlisten(Event::Txe);
        }
    }

    fn get_byte(&mut self) -> Option<u8> {
        match self.serial.read() {
            Ok(b) => Some(b),
            Err(_) => None,
        }
    }

    fn put_byte(&mut self, data: u8) -> bool {
        self.serial.write(data).is_ok()
    }

    fn deassert_re_de(&mut self) {
        let _ = self.re_de.set_low();
    }
}
