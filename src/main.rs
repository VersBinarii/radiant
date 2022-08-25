#![no_main]
#![no_std]

use radiant as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers=[OTG_FS_WKUP])]
mod app {
    use display_interface_spi::SPIInterface;
    use embedded_graphics::{pixelcolor::Rgb565, prelude::*};
    use ili9341::{DisplaySize240x320, Ili9341, Orientation};
    use radiant::{
        dummy_pin::{DummyOutputPin, MyIrq},
        gui::{Button, Gui, GuiElement},
    };
    use stm32f4xx_hal::{
        gpio::{Alternate, Edge, NoPin, Output, Pin, PushPull},
        pac::{EXTI, SPI1, SPI5, TIM1},
        prelude::*,
        spi::{Mode, NoMiso, Phase, Polarity, Spi},
        timer::{Channel, Delay, SysEvent},
    };
    use xpt2046::{self, Xpt2046};

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
        gui: Gui,
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
        let button_up = Button::new("+", Point::new(10, 10), Size::new(45, 45), Rgb565::MAGENTA);
        let button_down = Button::new("-", Point::new(60, 10), Size::new(45, 45), Rgb565::MAGENTA);

        (
            Shared {
                xpt_drv,
                exti: dp.EXTI,
                gui: Gui {
                    button_up,
                    button_down,
                },
            },
            Local {
                blue_led,
                delay,
                lcd,
            },
            init::Monotonics(),
        )
    }

    #[idle(local = [delay], shared = [xpt_drv, exti,gui])]
    fn maintask(ctx: maintask::Context) -> ! {
        let mut xpt_drv = ctx.shared.xpt_drv;
        let _exti = ctx.shared.exti;
        let _delay = ctx.local.delay;
        let mut gui = ctx.shared.gui;

        render::spawn().ok();
        loop {
            xpt_drv.lock(|xpt| {
                if xpt.is_touched() {
                    let p = xpt.get_touch_point();
                    gui.lock(|g| {
                        g.button_up.update(&p);
                        g.button_down.update(&p);
                        if g.button_up.is_pressed() {
                            defmt::println!("up pressed");
                        }
                        if g.button_down.is_pressed() {
                            defmt::println!("down pressed");
                        }
                    });
                    render::spawn().ok();
                }
            });
        }
    }

    #[task(priority=3, local = [lcd], shared = [gui])]
    fn render(mut ctx: render::Context) {
        let lcd = ctx.local.lcd;
        ctx.shared.gui.lock(|g| {
            g.draw(lcd).expect("Error while rendering Gui");
        })
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
