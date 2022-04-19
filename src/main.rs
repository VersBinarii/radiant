#![no_main]
#![no_std]

use radiant as _;

#[rtic::app(device = stm32f4xx_hal::pac)]
mod app {
    use display_interface_spi::SPIInterface;
    use embedded_graphics::{
        mono_font::{ascii::FONT_6X10, MonoTextStyle},
        pixelcolor::Rgb565,
        prelude::*,
        text::{Alignment, Text},
    };
    use ili9341::{DisplaySize240x320, Ili9341, Orientation};
    use radiant::{
        dummy_pin::DummyOutputPin,
        xpt2046::{self, Xpt2046},
    };
    use stm32f4xx_hal::{
        gpio::{Alternate, Input, Output, Pin, PushPull},
        pac::{EXTI, SPI1},
        prelude::*,
        spi::{Mode, NoMiso, Phase, Polarity, Spi},
        timer::{Channel, SysEvent},
    };

    type TouchSpi = Spi<
        SPI1,
        (
            Pin<'A', 5, Alternate<5>>,
            Pin<'A', 6, Alternate<5>>,
            Pin<'A', 7, Alternate<5>>,
        ),
    >;
    #[shared]
    struct Shared {
        xpt_drv: Xpt2046<TouchSpi, Pin<'A', 4, Output<PushPull>>>,
        touch_irq: Pin<'A', 2, Input>,
        exti: EXTI,
    }

    #[local]
    struct Local {
        blue_led: Pin<'C', 13, Output<PushPull>>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let dp = ctx.device;
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
        let style = MonoTextStyle::new(&FONT_6X10, Rgb565::RED);

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
        let touch_irq = gpioa.pa2.into_pull_up_input();
        let touch_cs = gpioa.pa4.into_push_pull_output();
        let touch_clk = gpioa.pa5.into_alternate();
        let touch_mosi = gpioa.pa7.into_alternate().internal_pull_up(true);
        let touch_miso = gpioa.pa6.into_alternate();
        let touch_spi = Spi::new(
            dp.SPI1,
            (touch_clk, touch_miso, touch_mosi),
            mode,
            2.MHz(),
            &clocks,
        );

        let xpt_drv = Xpt2046::new(touch_spi, touch_cs, xpt2046::Orientation::PortraitFlipped);

        (
            Shared {
                xpt_drv,
                touch_irq,
                exti: dp.EXTI,
            },
            Local { blue_led },
            init::Monotonics(),
        )
    }

    #[idle(local = [], shared = [xpt_drv, touch_irq, exti])]
    fn idle(ctx: idle::Context) -> ! {
        let mut xpt = ctx.shared.xpt_drv;
        let mut touch_irq = ctx.shared.touch_irq;
        let mut exti = ctx.shared.exti;
        loop {
            xpt.lock(|xpt| {
                touch_irq.lock(|tirq| {
                    let mut should_release = false;
                    if tirq.is_high() {
                        should_release = true;
                    }
                    xpt.run(should_release, || exti.lock(|e| tirq.enable_interrupt(e)))
                })
            })
        }
    }

    #[task(binds = EXTI2, local = [], shared = [xpt_drv, touch_irq, exti])]
    fn exti2(ctx: exti2::Context) {
        let xpt = ctx.shared.xpt_drv;
        let touch_irq = ctx.shared.touch_irq;
        let exti = ctx.shared.exti;
        (xpt, touch_irq, exti).lock(|xpt, tirq, exti| {
            xpt.exti_irq_handle(|| {
                tirq.disable_interrupt(exti);
            })
        })
    }

    #[task(binds = SysTick, local = [counter:u16 = 0, blue_led])]
    fn systick(ctx: systick::Context) {
        let counter = ctx.local.counter;
        *counter += 1;

        if *counter == 100 {
            ctx.local.blue_led.toggle();
            *counter = 0;
        }
    }
}
