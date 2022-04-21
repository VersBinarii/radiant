#![no_main]
#![no_std]

use radiant as _;

#[rtic::app(device = stm32f4xx_hal::pac)]
mod app {
    use display_interface_spi::SPIInterface;
    use embedded_graphics::{
        mono_font::{ascii::FONT_6X10, MonoTextStyle},
        pixelcolor::{BinaryColor, Rgb565},
        prelude::*,
        text::{Alignment, Text},
    };
    use ili9341::{DisplaySize240x320, Ili9341, ModeState, Orientation};
    use radiant::dummy_pin::DummyOutputPin;
    use stm32f4xx_hal::{
        gpio::{Alternate, Edge, NoPin, Output, Pin, PushPull},
        pac::{EXTI, SPI1, SPI5, TIM1},
        prelude::*,
        spi::{Mode, NoMiso, Phase, Polarity, Spi},
        timer::{Channel, Delay, SysEvent},
    };
    use xpt2046::{self, Xpt2046, Xpt2046Exti};
    pub struct MyIrq<const C: char, const N: u8>(Pin<C, N>);
    impl Xpt2046Exti for MyIrq<'A', 2> {
        type Exti = EXTI;
        fn clear_interrupt(&mut self) {
            self.0.clear_interrupt_pending_bit();
        }

        fn disable_interrupt(&mut self, exti: &mut Self::Exti) {
            self.0.disable_interrupt(exti);
        }

        fn enable_interrupt(&mut self, exti: &mut Self::Exti) {
            self.0.enable_interrupt(exti);
        }

        fn is_high(&self) -> bool {
            self.0.is_high()
        }

        fn is_low(&self) -> bool {
            self.0.is_low()
        }
    }
    type TouchSpi = Spi<
        SPI1,
        (
            Pin<'A', 5, Alternate<5>>,
            Pin<'A', 6, Alternate<5>>,
            Pin<'A', 7, Alternate<5>>,
        ),
    >;
    type Lcd = Ili9341<
        SPIInterface<
            Spi<SPI5, (Pin<'B', 0, Alternate<6>>, NoPin, Pin<'A', 10, Alternate<6>>)>,
            Pin<'B', 1, Output>,
            Pin<'B', 2, Output>,
        >,
        DummyOutputPin,
    >;
    #[shared]
    struct Shared {
        xpt_drv: Xpt2046<TouchSpi, Pin<'A', 4, Output<PushPull>>, MyIrq<'A', 2>>,
        exti: EXTI,
    }

    #[local]
    struct Local {
        blue_led: Pin<'C', 13, Output<PushPull>>,
        delay: Delay<TIM1, 1000000>,
        lcd: Lcd,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut dp = ctx.device;
        let cp = ctx.core;

        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(100.MHz()).freeze();

        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();

        /*
         * We need a backlight constrol
         * TODO: We will need to be able to change the brighntess later
         */
        let backlight_pins = gpioa.pa3.into_alternate();
        let mut pwm = dp.TIM2.pwm_hz(backlight_pins, 10.kHz(), &clocks);
        pwm.enable(Channel::C4);
        pwm.set_duty(Channel::C4, pwm.get_max_duty() / 2);

        /*
         * Indicator that stuff is running
         */
        let blue_led = gpioc.pc13.into_push_pull_output();
        let mut systic = cp.SYST.counter_hz(&clocks);
        systic.start(1.kHz()).unwrap();
        systic.listen(SysEvent::Update);

        /*
         *  The ILI9341 driver
         */
        let lcd_clk = gpiob.pb0.into_alternate();
        let lcd_miso = NoMiso {};
        let lcd_mosi = gpioa.pa10.into_alternate().internal_pull_up(true);
        let lcd_dc = gpiob.pb1.into_push_pull_output();
        let lcd_cs = gpiob.pb2.into_push_pull_output();
        let mode = Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        };
        let lcd_spi = dp
            .SPI5
            .spi((lcd_clk, lcd_miso, lcd_mosi), mode, 8.MHz(), &clocks);
        let spi_iface = SPIInterface::new(lcd_spi, lcd_dc, lcd_cs);
        let dummy_reset = DummyOutputPin::default();
        let mut delay = dp.TIM1.delay_us(&clocks);
        let mut lcd = Ili9341::new(
            spi_iface,
            dummy_reset,
            &mut delay,
            Orientation::PortraitFlipped,
            DisplaySize240x320,
        )
        .unwrap();

        let _ = lcd.clear(Rgb565::BLUE);
        // Create a new character style
        let style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);

        // Create a text at position (20, 30) and draw it using the previously defined style
        Text::with_alignment(
            "First line\nSecond line",
            Point::new(20, 30),
            style,
            Alignment::Center,
        )
        .draw(&mut lcd)
        .unwrap();

        // Touch interface
        let mut touch_irq = gpioa.pa2.into_pull_up_input();
        let mut syscfg = dp.SYSCFG.constrain();
        touch_irq.make_interrupt_source(&mut syscfg);
        touch_irq.trigger_on_edge(&mut dp.EXTI, Edge::Falling);
        touch_irq.enable_interrupt(&mut dp.EXTI);
        let touch_cs = gpioa.pa4.into_push_pull_output();
        let touch_clk = gpioa.pa5.into_alternate();
        let touch_mosi = gpioa.pa7.into_alternate().internal_pull_up(true);
        let touch_miso = gpioa.pa6.into_alternate();
        let touch_spi = Spi::new(
            dp.SPI1,
            (touch_clk, touch_miso, touch_mosi),
            mode,
            4.MHz(),
            &clocks,
        );

        let mut xpt_drv = Xpt2046::new(
            touch_spi,
            touch_cs,
            MyIrq(touch_irq),
            xpt2046::Orientation::PortraitFlipped,
        );
        xpt_drv.init(&mut delay);

        (
            Shared {
                xpt_drv,
                exti: dp.EXTI,
            },
            Local {
                blue_led,
                delay,
                lcd,
            },
            init::Monotonics(),
        )
    }

    #[idle(local = [delay, lcd], shared = [xpt_drv, exti])]
    fn idle(ctx: idle::Context) -> ! {
        let mut xpt_drv = ctx.shared.xpt_drv;
        let mut exti = ctx.shared.exti;
        let delay = ctx.local.delay;
        let lcd = ctx.local.lcd;
        xpt_drv.lock(|xpt| exti.lock(|e| xpt.calibrate(lcd, delay, e)));
        loop {
            xpt_drv.lock(|xpt| {
                if xpt.is_touched() {
                    let p = xpt.get_touch_point();
                    defmt::println!("x:{} y:{}", p.x, p.y);
                    Pixel(p, Rgb565::RED).draw(lcd);
                    //xpt.clear_touch();
                }
            });
        }
    }

    #[task(binds = EXTI2, local = [], shared = [xpt_drv, exti])]
    fn exti2(ctx: exti2::Context) {
        let xpt = ctx.shared.xpt_drv;
        let exti = ctx.shared.exti;
        (xpt, exti).lock(|xpt, exti| xpt.exti_irq_handle(exti))
    }

    #[task(binds = SysTick, local = [counter:u16 = 0, blue_led], shared = [xpt_drv, exti])]
    fn systick(ctx: systick::Context) {
        let xpt = ctx.shared.xpt_drv;
        let exti = ctx.shared.exti;
        (xpt, exti).lock(|xpt, exti| xpt.run(exti));

        let counter = ctx.local.counter;
        *counter += 1;

        if *counter == 100 {
            ctx.local.blue_led.toggle();
            *counter = 0;
        }
    }
}
