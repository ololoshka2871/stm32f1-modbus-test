#![no_main]
#![no_std]

use panic_abort as _;
use rtic::app;

use stm32f1xx_hal::afio::AfioExt;
use stm32f1xx_hal::flash::FlashExt;
use stm32f1xx_hal::gpio::{Alternate, Floating, GpioExt, Input, Output, PushPull, PA15, PB6, PB7};

use stm32f1xx_hal::pac::{TIM2, USART1};
use stm32f1xx_hal::timer::CounterUs;

use libremodbus_rs::MBInterface;

use systick_monotonic::Systick;

use stm32f1_modbus_test::*;

//-----------------------------------------------------------------------------

pub const MCU_XTAL_HZ: u32 = 8_000_000;
pub const MODBUS_ADDR: u8 = 1;

//-----------------------------------------------------------------------------

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [RTCALARM])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        rtu: libremodbus_rs::Rtu,
    }

    #[local]
    struct Local {
        data: &'static mut DataStorage,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<{ config::SYSTICK_RATE_HZ }>;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        use stm32f1xx_hal::prelude::_fugit_RateExtU32;
        use stm32f1xx_hal::prelude::_stm32_hal_rcc_RccExt;
        use stm32f1xx_hal::prelude::_stm32_hal_time_U32Ext;
        use stm32f1xx_hal::prelude::_stm32f4xx_hal_timer_TimerExt;

        static mut UART1: Option<
            Serial<
                stm32f1xx_hal::serial::Serial<USART1, 
                    (PB6<Alternate<PushPull>>, PB7<Input<Floating>>)>,
                PA15<Output<PushPull>>,
            >,
        > = None;

        static mut MODBUS_TIMER: Option<Timer<CounterUs<TIM2>>> = None;
        static mut DATA_STORAGE: Option<DataStorage> = None;

        //---------------------------------------------------------------------

        let mut flash = ctx.device.FLASH.constrain();

        let mut gpioa = ctx.device.GPIOA.split();
        let mut gpiob = ctx.device.GPIOB.split();

        let mut afio = ctx.device.AFIO.constrain();

        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(MCU_XTAL_HZ.Hz())
            .sysclk(32u32.MHz())
            .freeze(&mut flash.acr);

        let mono = Systick::new(ctx.core.SYST, clocks.sysclk().to_Hz());

        //---------------------------------------------------------------------

        // fixme
        let tx = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
        let rx = gpiob.pb7;

        // disable jtag
        let (pa15, _pb3, _pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        let re_de = pa15
            .into_push_pull_output_with_state(&mut gpioa.crh, stm32f1xx_hal::gpio::PinState::Low);

        let mut timer = ctx.device.TIM2.counter_us(&clocks);
        timer.listen(stm32f1xx_hal::timer::Event::Update);

        ctx.device
            .DBGMCU
            .cr
            .modify(|_, w| w.dbg_tim2_stop().set_bit());

        unsafe {
            UART1.replace(Serial::new(
                stm32f1xx_hal::serial::Serial::usart1(
                    ctx.device.USART1,
                    (tx, rx),
                    &mut afio.mapr,
                    stm32f1xx_hal::serial::Config::default().baudrate(9600.bps()),
                    clocks,
                ),
                re_de,
                clocks,
            ));

            MODBUS_TIMER.replace(Timer::new(timer));
            DATA_STORAGE.replace(DataStorage::new());
        }

        let rtu = unsafe {
            libremodbus_rs::Rtu::init(
                MODBUS_ADDR,
                UART1.as_mut().unwrap_unchecked(),
                config::RS485_BOUD_RATE,
                MODBUS_TIMER.as_mut().unwrap_unchecked(),
                DATA_STORAGE.as_mut().unwrap_unchecked(),
            )
        };

        let _led = gpioa.pa5.into_push_pull_output_with_state(&mut gpioa.crl, stm32f1xx_hal::gpio::PinState::Low);

        //---------------------------------------------------------------------

        (
            Shared { rtu },
            Local {
                data: unsafe { DATA_STORAGE.as_mut().unwrap_unchecked() },
            },
            init::Monotonics(mono),
        )
    }

    //-------------------------------------------------------------------------

    #[idle(shared = [rtu])]
    fn idle(mut ctx: idle::Context) -> ! {
        assert!(ctx.shared.rtu.lock(|rtu| rtu.enable()));
        loop {
            cortex_m::asm::wfi();
        }
    }

    //-------------------------------------------------------------------------

    #[task(binds = USART1, shared = [rtu], priority = 3)]
    fn usart1_isr(mut ctx: usart1_isr::Context) {
        use libremodbus_rs::REDEControl;
        use libremodbus_rs::SerialEvent;
        use systick_monotonic::*;

        let do_poll = ctx.shared.rtu.lock(|rtu| {
            let sr = unsafe { (*USART1::ptr()).sr.read() };
            let cr = unsafe { (*USART1::ptr()).cr1.read() };

            if sr.txe().bit_is_set() && cr.txeie().bit_is_set() {
                let res = rtu.on_tx();
                if rtu.is_tx_finished() {
                    re_de_finaliser::spawn_after(
                        (libm::ceilf(
                            WAIT_BITS_AFTER_TX_DONE as f32 * 1_000.0
                                / config::RS485_BOUD_RATE as f32,
                        ) as u64)
                            .millis(),
                    )
                    .unwrap();
                }
                return res;
            }

            if sr.rxne().bit_is_set() && cr.rxneie().bit_is_set() {
                return rtu.on_rx();
            }
            false
        });

        if do_poll {
            modbus_pooler::spawn().unwrap();
        }
    }

    #[task(binds = TIM2, shared = [rtu], priority = 3)]
    fn tim2(mut ctx: tim2::Context) {
        use libremodbus_rs::MBTimerEvent;

        let do_poll = ctx.shared.rtu.lock(|rtu| {
            let res = rtu.on_timer();
            unsafe { (*TIM2::ptr()).sr.modify(|_, w| w.uif().clear_bit()) };
            res
        });

        if do_poll {
            modbus_pooler::spawn().unwrap();
        }
    }

    #[task(shared = [rtu], local = [data])]
    fn modbus_pooler(mut ctx: modbus_pooler::Context) {
        ctx.shared.rtu.lock(|rtu| rtu.pool());
    }

    #[task(shared = [rtu])]
    fn re_de_finaliser(mut ctx: re_de_finaliser::Context) {
        use libremodbus_rs::REDEControl;

        ctx.shared.rtu.lock(|rtu| rtu.deassert_re_de());
    }
}
