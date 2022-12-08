#![no_main]
#![no_std]
#![feature(array_zip)]
#![feature(macro_metavar_expr)]

mod config;
mod pwm_ctrl_ext;
mod support;

use panic_abort as _;
use rtic::app;

use stm32f1xx_hal::afio::AfioExt;
use stm32f1xx_hal::flash::FlashExt;
use stm32f1xx_hal::gpio::{
    Alternate, Floating, GpioExt, Input, Output, PushPull, PA1, PA2, PA3, PA5,
};
use stm32f1xx_hal::pac::{TIM2, USART2};
use stm32f1xx_hal::serial::{Config, Serial};
use stm32f1xx_hal::timer::CounterUs;

use libremodbus_rs::MBInterface;

use systick_monotonic::Systick;

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
        led: PA5<Output<PushPull>>,
        data: &'static mut support::DataStorage,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<{ config::SYSTICK_RATE_HZ }>;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        use stm32f1xx_hal::prelude::_fugit_RateExtU32;
        use stm32f1xx_hal::prelude::_stm32_hal_rcc_RccExt;
        use stm32f1xx_hal::prelude::_stm32_hal_time_U32Ext;
        use stm32f1xx_hal::prelude::_stm32f4xx_hal_timer_TimerExt;

        static mut UART2: Option<
            support::Serial<
                Serial<USART2, (PA2<Alternate<PushPull>>, PA3<Input<Floating>>)>,
                PA1<Output<PushPull>>,
            >,
        > = None;

        static mut MODBUS_TIMER: Option<support::Timer<CounterUs<TIM2>>> = None;
        static mut DATA_STORAGE: Option<support::DataStorage> = None;

        let mut flash = ctx.device.FLASH.constrain();

        let mut gpioa = ctx.device.GPIOA.split();
        /*
        let mut gpiob = ctx.device.GPIOB.split();
        let mut gpioc = ctx.device.GPIOC.split();
        */
        let mut afio = ctx.device.AFIO.constrain();
        /*
        let (pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);
        */

        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(32u32.MHz()).freeze(&mut flash.acr);

        let mono = Systick::new(ctx.core.SYST, clocks.sysclk().to_Hz());

        //---------------------------------------------------------------------

        let tx = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
        let rx = gpioa.pa3;
        let re_de = gpioa
            .pa1
            .into_push_pull_output_with_state(&mut gpioa.crl, stm32f1xx_hal::gpio::PinState::Low);

        let mut timer = ctx.device.TIM2.counter_us(&clocks);
        timer.listen(stm32f1xx_hal::timer::Event::Update);

        ctx.device
            .DBGMCU
            .cr
            .modify(|_, w| w.dbg_tim2_stop().set_bit());

        unsafe {
            UART2.replace(support::Serial::new(
                Serial::usart2(
                    ctx.device.USART2,
                    (tx, rx),
                    &mut afio.mapr,
                    Config::default().baudrate(9600.bps()),
                    clocks,
                ),
                re_de,
                clocks,
            ));

            MODBUS_TIMER.replace(support::Timer::new(timer));
            DATA_STORAGE.replace(support::DataStorage::new());
        }

        let rtu = unsafe {
            libremodbus_rs::Rtu::init(
                config::MODBUS_ADDR,
                UART2.as_mut().unwrap_unchecked(),
                config::RS485_BOUD_RATE,
                MODBUS_TIMER.as_mut().unwrap_unchecked(),
                DATA_STORAGE.as_mut().unwrap_unchecked(),
            )
        };

        //---------------------------------------------------------------------

        let led = gpioa
            .pa5
            .into_push_pull_output_with_state(&mut gpioa.crl, stm32f1xx_hal::gpio::PinState::Low);

        (
            Shared { rtu },
            Local {
                led,
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

    #[task(binds = USART2, shared = [rtu], local = [led], priority = 3)]
    fn usart2_tx(mut ctx: usart2_tx::Context) {
        use libremodbus_rs::REDEControl;
        use libremodbus_rs::SerialEvent;
        use systick_monotonic::*;

        //let _ = ctx.local.led.set_high();
        let do_poll = ctx.shared.rtu.lock(|rtu| {
            let sr = unsafe { (*USART2::ptr()).sr.read() };
            let cr = unsafe { (*USART2::ptr()).cr1.read() };

            if sr.txe().bit_is_set() && cr.txeie().bit_is_set() {
                let res = rtu.on_tx();
                if rtu.is_tx_finished() {
                    re_de_finaliser::spawn_after(
                        (libm::ceilf(
                            support::WAIT_BITS_AFTER_TX_DONE as f32 * 1_000.0
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
        //let _ = ctx.local.led.set_low();

        if do_poll {
            modbus_pooler::spawn().unwrap();
        }
    }

    #[task(binds = TIM2, shared = [rtu], local = [/*led*/], priority = 3)]
    fn tim2(mut ctx: tim2::Context) {
        use libremodbus_rs::MBTimerEvent;

        //let _ = ctx.local.led.set_high();
        let do_poll = ctx.shared.rtu.lock(|rtu| {
            let res = rtu.on_timer();
            unsafe { (*TIM2::ptr()).sr.modify(|_, w| w.uif().clear_bit()) };
            res
        });
        //let _ = ctx.local.led.set_low();

        if do_poll {
            modbus_pooler::spawn().unwrap();
        }
    }

    #[task(shared= [rtu], local = [data])]
    fn modbus_pooler(mut ctx: modbus_pooler::Context) {
        use pwm_ctrl_ext::PWMCtrlExt;

        ctx.shared.rtu.lock(|rtu| rtu.pool());

        let target_pwm_values = ctx.local.data.process();
        update_pca9685_channels(&target_pwm_values.0[0..15]);
        update_pwm_channels(&target_pwm_values.0[16..19]);

        ctx.shared.rtu.lock(|rtu| rtu.pool());
    }

    #[task(shared = [rtu])]
    fn re_de_finaliser(mut ctx: re_de_finaliser::Context) {
        use libremodbus_rs::REDEControl;

        ctx.shared.rtu.lock(|rtu| rtu.deassert_re_de());
    }
}

pub fn update_pca9685_channels(_targets: &[u32]) {}

pub fn update_pwm_channels(_targets: &[u32]) {}

impl pwm_ctrl_ext::PWMCtrlExt<20> for support::DataStorage {
    fn process(&mut self) -> pwm_ctrl_ext::PWMValues<20> {
        pwm_ctrl_ext::PWMValues([0u32; 20])
    }
}
