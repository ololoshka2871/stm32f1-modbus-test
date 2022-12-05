#![no_main]
#![no_std]
#![feature(array_zip)]
#![feature(macro_metavar_expr)]

mod config;
mod support;

use panic_abort as _;
use rtic::app;

use stm32f1xx_hal::afio::AfioExt;
use stm32f1xx_hal::flash::FlashExt;
use stm32f1xx_hal::gpio::{
    Alternate, Floating, GpioExt, Input, Output, PushPull, PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7,
    PA9, PB3, PB4, PB5, PB6, PC13, PC14, PC15,
};
use stm32f1xx_hal::pac::{Interrupt, TIM1, TIM2, TIM4, USART2};
use stm32f1xx_hal::serial::{Config, Serial};
use stm32f1xx_hal::time::Hertz;
use stm32f1xx_hal::timer::Counter;
use stm32f1xx_hal::timer::{PwmChannel, Timer};

use systick_monotonic::Systick;

//-----------------------------------------------------------------------------

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [RTCALARM])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        rtu: libremodbus_rs::Rtu<
            Serial<USART2, (PA2<Alternate<PushPull>>, PA3<Input<Floating>>)>,
            Counter<TIM2, 1000000>,
        >,
    }

    #[local]
    struct Local {}

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<{ config::SYSTICK_RATE_HZ }>;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        use stm32f1xx_hal::prelude::_fugit_RateExtU32;
        use stm32f1xx_hal::prelude::_stm32_hal_rcc_RccExt;
        use stm32f1xx_hal::prelude::_stm32_hal_time_U32Ext;
        use stm32f1xx_hal::prelude::_stm32f4xx_hal_timer_TimerExt;

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
        let clocks = rcc.cfgr.sysclk(16u32.MHz()).freeze(&mut flash.acr);

        let mono = Systick::new(ctx.core.SYST, clocks.sysclk().to_Hz());

        //---------------------------------------------------------------------

        let tx = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
        let rx = gpioa.pa3;
        let uart2 = Serial::usart2(
            ctx.device.USART2,
            (tx, rx),
            &mut afio.mapr,
            Config::default().baudrate(config::RS485_BOUD_RATE.bps()),
            clocks,
        );

        let mb_timer = ctx.device.TIM2.counter_us(&clocks);
        let rtu = libremodbus_rs::Rtu::init(
            config::MODBUS_ADDR,
            uart2,
            config::RS485_BOUD_RATE,
            mb_timer,
        );

        //---------------------------------------------------------------------

        (Shared { rtu }, Local {}, init::Monotonics(mono))
    }

    //-------------------------------------------------------------------------

    #[idle(shared = [rtu])]
    fn idle(mut ctx: idle::Context) -> ! {
        use libremodbus_rs::MBInterface;

        ctx.shared.rtu.lock(|rtu| rtu.enable());
        loop {
            ctx.shared.rtu.lock(|rtu| rtu.pool());

            cortex_m::asm::wfi();
        }
    }
}
