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

    #[idle(shared = [rtu], local = [data])]
    fn idle(mut ctx: idle::Context) -> ! {
        use libremodbus_rs::MBInterface;
        use pwm_ctrl_ext::PWMCtrlExt;

        if !ctx.shared.rtu.lock(|rtu| rtu.enable()) {
            panic!("Failed to enable modbus");
        }
        loop {
            ctx.shared.rtu.lock(|rtu| rtu.pool());

            let target_pwm_values = ctx.local.data.process();
            update_pca9685_channels(&target_pwm_values.0[0..15]);
            update_pwm_channels(&target_pwm_values.0[16..19]);

            cortex_m::asm::wfi();
        }
    }

    //-------------------------------------------------------------------------

    #[task(binds = USART2, shared = [rtu], local = [led])]
    fn usart2_tx(mut ctx: usart2_tx::Context) {
        use libremodbus_rs::SerialEvent;

        let _ = ctx.local.led.set_high();
        ctx.shared.rtu.lock(|rtu| {
            let sr = unsafe { (*USART2::ptr()).sr.read() };
            let cr = unsafe { (*USART2::ptr()).cr1.read() };

            if sr.txe().bit_is_set() && cr.txeie().bit_is_set() {
                rtu.on_tx();
            }

            if sr.rxne().bit_is_set() && cr.rxneie().bit_is_set() {
                rtu.on_rx();
            }
        });
        let _ = ctx.local.led.set_low();
    }

    #[task(binds = TIM2, shared = [rtu], local = [/*led*/])]
    fn tim2(mut ctx: tim2::Context) {
        use libremodbus_rs::MBTimerEvent;

        //let _ = ctx.local.led.set_high();
        ctx.shared.rtu.lock(|rtu| {
            rtu.on_timer();
            unsafe { (*TIM2::ptr()).sr.modify(|_, w| w.uif().clear_bit()) };
        });
        //let _ = ctx.local.led.set_low();
    }
}

pub fn update_pca9685_channels(_targets: &[u32]) {}

pub fn update_pwm_channels(_targets: &[u32]) {}

impl pwm_ctrl_ext::PWMCtrlExt<20> for support::DataStorage {
    fn process(&mut self) -> pwm_ctrl_ext::PWMValues<20> {
        pwm_ctrl_ext::PWMValues([0u32; 20])
    }
}
