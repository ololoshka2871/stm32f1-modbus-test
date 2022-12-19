#![no_main]
#![no_std]
#![feature(array_zip)]
#![feature(macro_metavar_expr)]

mod config;
mod pwm;
mod support;

use panic_abort as _;
use rtic::app;

use stm32f1xx_hal::afio::AfioExt;
use stm32f1xx_hal::flash::FlashExt;
use stm32f1xx_hal::gpio::{
    Alternate, Floating, GpioExt, Input, OpenDrain, Output, PushPull, PA1, PA10, PA11, PA2, PA3,
    PA5, PA8, PA9, PB6, PB7,
};
use stm32f1xx_hal::i2c::BlockingI2c;
use stm32f1xx_hal::pac::{I2C1, TIM1, TIM2, USART2};
use stm32f1xx_hal::serial::{Config, Serial};
use stm32f1xx_hal::time::Hertz;
use stm32f1xx_hal::timer::{Ch, Channel, CounterUs, PwmHz, Tim1NoRemap};

use libremodbus_rs::MBInterface;

use systick_monotonic::Systick;

use pwm::{NativeCh, PCA9685Ch, PWMChannelId, PWMValues, Position};

use pwm_pca9685::Channel as PWMChannel;
use pwm_pca9685::{Address, Pca9685};

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
        pwm: [&'static mut dyn PWMChannelId; 20],
        pac9685_channels: &'static mut [PCA9685Ch; 16],

        native_pwm: PwmHz<
            TIM1,
            Tim1NoRemap,
            (Ch<0>, Ch<1>, Ch<2>, Ch<3>),
            (
                PA8<Alternate<PushPull>>,
                PA9<Alternate<PushPull>>,
                PA10<Alternate<PushPull>>,
                PA11<Alternate<PushPull>>,
            ),
        >,
        pac9685: Pca9685<BlockingI2c<I2C1, (PB6<Alternate<OpenDrain>>, PB7<Alternate<OpenDrain>>)>>,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<{ config::SYSTICK_RATE_HZ }>;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        use stm32f1xx_hal::prelude::_fugit_RateExtU32;
        use stm32f1xx_hal::prelude::_stm32_hal_rcc_RccExt;
        use stm32f1xx_hal::prelude::_stm32_hal_time_U32Ext;
        use stm32f1xx_hal::prelude::_stm32f4xx_hal_timer_PwmExt;
        use stm32f1xx_hal::prelude::_stm32f4xx_hal_timer_TimerExt;
        use stm32f1xx_hal::timer::Tim1NoRemap;

        static mut UART2: Option<
            support::Serial<
                Serial<USART2, (PA2<Alternate<PushPull>>, PA3<Input<Floating>>)>,
                PA1<Output<PushPull>>,
            >,
        > = None;

        static mut MODBUS_TIMER: Option<support::Timer<CounterUs<TIM2>>> = None;
        static mut DATA_STORAGE: Option<support::DataStorage> = None;

        static mut NATIVE_PWM_CHANNELS: Option<[NativeCh; 4]> = None;
        static mut PCA9685_PWM_CHANNELS: Option<[PCA9685Ch; 16]> = None;

        //---------------------------------------------------------------------

        let mut flash = ctx.device.FLASH.constrain();

        let mut gpioa = ctx.device.GPIOA.split();
        let mut gpiob = ctx.device.GPIOB.split();
        /*
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
        let p0r = gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh);
        let p1r = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let p2l = gpioa.pa10.into_alternate_push_pull(&mut gpioa.crh);
        let p3l = gpioa.pa11.into_alternate_push_pull(&mut gpioa.crh);

        let tim1 = unsafe { &*TIM1::ptr() };
        let mut native_pwm = ctx.device.TIM1.pwm_hz::<Tim1NoRemap, _, _>(
            (p0r, p1r, p2l, p3l),
            &mut afio.mapr,
            config::MAX_PWM_FREQ.kHz(),
            &clocks,
        );

        // revers polarity for channels 1 and 2
        tim1.ccer.modify(|_, w| w.cc1p().set_bit().cc2p().set_bit());

        // enable channels
        native_pwm.set_duty(Channel::C1, native_pwm.get_duty(Channel::C1));
        native_pwm.enable(Channel::C1);
        native_pwm.set_duty(Channel::C2, native_pwm.get_duty(Channel::C2));
        native_pwm.enable(Channel::C2);
        native_pwm.set_duty(Channel::C3, 0);
        native_pwm.enable(Channel::C3);
        native_pwm.set_duty(Channel::C4, 0);
        native_pwm.enable(Channel::C4);

        let pwm: [&'static mut dyn PWMChannelId; 20] = unsafe {
            PCA9685_PWM_CHANNELS.replace([
                PCA9685Ch::new(PWMChannel::C0, Position::LeftAligned),
                PCA9685Ch::new(PWMChannel::C1, Position::LeftAligned),
                PCA9685Ch::new(PWMChannel::C2, Position::LeftAligned),
                PCA9685Ch::new(PWMChannel::C3, Position::LeftAligned),
                PCA9685Ch::new(PWMChannel::C4, Position::RightAligend),
                PCA9685Ch::new(PWMChannel::C5, Position::RightAligend),
                PCA9685Ch::new(PWMChannel::C6, Position::RightAligend),
                PCA9685Ch::new(PWMChannel::C7, Position::RightAligend),
                PCA9685Ch::new(PWMChannel::C8, Position::LeftAligned),
                PCA9685Ch::new(PWMChannel::C9, Position::LeftAligned),
                PCA9685Ch::new(PWMChannel::C10, Position::LeftAligned),
                PCA9685Ch::new(PWMChannel::C11, Position::LeftAligned),
                PCA9685Ch::new(PWMChannel::C12, Position::RightAligend),
                PCA9685Ch::new(PWMChannel::C13, Position::RightAligend),
                PCA9685Ch::new(PWMChannel::C14, Position::RightAligend),
                PCA9685Ch::new(PWMChannel::C15, Position::RightAligend),
            ]);

            NATIVE_PWM_CHANNELS.replace([
                NativeCh::new(16, Position::RightAligend),
                NativeCh::new(17, Position::RightAligend),
                NativeCh::new(18, Position::LeftAligned),
                NativeCh::new(19, Position::LeftAligned),
            ]);

            [
                // +
                &mut NATIVE_PWM_CHANNELS.as_mut().unwrap_unchecked()[2],
                &mut PCA9685_PWM_CHANNELS.as_mut().unwrap_unchecked()[0],
                &mut PCA9685_PWM_CHANNELS.as_mut().unwrap_unchecked()[1],
                &mut PCA9685_PWM_CHANNELS.as_mut().unwrap_unchecked()[2],
                &mut PCA9685_PWM_CHANNELS.as_mut().unwrap_unchecked()[3],
                // -
                &mut NATIVE_PWM_CHANNELS.as_mut().unwrap_unchecked()[0],
                &mut PCA9685_PWM_CHANNELS.as_mut().unwrap_unchecked()[4],
                &mut PCA9685_PWM_CHANNELS.as_mut().unwrap_unchecked()[5],
                &mut PCA9685_PWM_CHANNELS.as_mut().unwrap_unchecked()[6],
                &mut PCA9685_PWM_CHANNELS.as_mut().unwrap_unchecked()[7],
                // +
                &mut NATIVE_PWM_CHANNELS.as_mut().unwrap_unchecked()[3],
                &mut PCA9685_PWM_CHANNELS.as_mut().unwrap_unchecked()[8],
                &mut PCA9685_PWM_CHANNELS.as_mut().unwrap_unchecked()[9],
                &mut PCA9685_PWM_CHANNELS.as_mut().unwrap_unchecked()[10],
                &mut PCA9685_PWM_CHANNELS.as_mut().unwrap_unchecked()[11],
                // -
                &mut NATIVE_PWM_CHANNELS.as_mut().unwrap_unchecked()[1],
                &mut PCA9685_PWM_CHANNELS.as_mut().unwrap_unchecked()[12],
                &mut PCA9685_PWM_CHANNELS.as_mut().unwrap_unchecked()[13],
                &mut PCA9685_PWM_CHANNELS.as_mut().unwrap_unchecked()[14],
                &mut PCA9685_PWM_CHANNELS.as_mut().unwrap_unchecked()[15],
            ]
        };

        let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
        let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);

        let i2c = BlockingI2c::i2c1(
            ctx.device.I2C1,
            (scl, sda),
            &mut afio.mapr,
            stm32f1xx_hal::i2c::Mode::Fast {
                frequency: 400.kHz(),
                duty_cycle: stm32f1xx_hal::i2c::DutyCycle::Ratio16to9,
            },
            clocks,
            1000,
            10,
            1000,
            1000,
        );

        let mut pac9685 = Pca9685::new(i2c, Address::default()).unwrap();

        pac9685
            .set_prescale(pac9685_prescaler(config::MAX_PWM_FREQ.Hz()))
            .unwrap();
        for ch in unsafe { PCA9685_PWM_CHANNELS.as_mut().unwrap_unchecked() }.iter_mut() {
            ch.prepare(&mut pac9685).unwrap();
        }
        pac9685.enable().unwrap();

        //---------------------------------------------------------------------

        let led = gpioa
            .pa5
            .into_push_pull_output_with_state(&mut gpioa.crl, stm32f1xx_hal::gpio::PinState::Low);

        (
            Shared { rtu },
            Local {
                led,
                data: unsafe { DATA_STORAGE.as_mut().unwrap_unchecked() },
                pwm,
                pac9685_channels: unsafe { PCA9685_PWM_CHANNELS.as_mut().unwrap_unchecked() },
                native_pwm,
                pac9685,
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

    #[task(shared = [rtu], local = [data, pwm, pac9685_channels, native_pwm, pac9685])]
    fn modbus_pooler(mut ctx: modbus_pooler::Context) {
        use pwm::PWMCtrlExt;

        ctx.shared.rtu.lock(|rtu| rtu.pool());

        if let (Some(target_pwm_values), target_pwm_freq) = ctx
            .local
            .data
            .process(unsafe { core::mem::transmute(ctx.local.pwm) })
        {
            if let Some(target_pwm_freq) = target_pwm_freq {
                ctx.local
                    .pac9685
                    .set_prescale(pac9685_prescaler(target_pwm_freq))
                    .unwrap();
                ctx.local.native_pwm.set_period(target_pwm_freq);
            }

            //-----------------------------------------------------------------

            for channel in ctx.local.pac9685_channels.iter_mut() {
                channel
                    .configure(ctx.local.pac9685, target_pwm_values.values[channel.id()])
                    .unwrap();
            }

            //-----------------------------------------------------------------

            let native_pwm = ctx.local.native_pwm;
            let mut set_native_channel_duty =
                move |channel: Channel,
                      target_pwm_values: &PWMValues<20>,
                      cnannel_id,
                      is_inverted: bool| {
                    native_pwm.set_duty(
                        channel,
                        target_pwm_values.as_range(
                            cnannel_id,
                            config::MAX_PWM_VAL,
                            native_pwm.get_max_duty(),
                            is_inverted,
                        ),
                    );
                };

            set_native_channel_duty(Channel::C1, &target_pwm_values, 16, true);
            set_native_channel_duty(Channel::C2, &target_pwm_values, 17, true);
            set_native_channel_duty(Channel::C3, &target_pwm_values, 18, false);
            set_native_channel_duty(Channel::C4, &target_pwm_values, 19, false);
        }

        ctx.shared.rtu.lock(|rtu| rtu.pool());
    }

    #[task(shared = [rtu])]
    fn re_de_finaliser(mut ctx: re_de_finaliser::Context) {
        use libremodbus_rs::REDEControl;

        ctx.shared.rtu.lock(|rtu| rtu.deassert_re_de());
    }
}

fn pac9685_prescaler(freq: Hertz) -> u8 {
    const PCA9685_INTERNAL_CLK_HZ: f32 = 25_000_000.0;

    libm::roundf(PCA9685_INTERNAL_CLK_HZ / (4096 * freq.to_Hz()) as f32) as u8 - 1
}
