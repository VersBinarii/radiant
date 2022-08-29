#![no_main]
#![no_std]

use radiant as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers=[OTG_FS_WKUP, SDIO])]
mod app {
    use embedded_graphics::{pixelcolor::Rgb565, prelude::*};
    use radiant::{
        gui::{Button, Gui, GuiElement},
        peripherals::{
            init_preipherals, BlueLedPin, Lcd, TouchDriver, TriakControlPin, ZeroCrossDetectPin,
        },
    };
    use stm32f4xx_hal::{
        pac::{EXTI, TIM1},
        prelude::*,
        timer::{Delay, SysEvent},
    };

    #[shared]
    struct Shared {
        xpt_drv: TouchDriver,
        exti: EXTI,
        gui: Gui,
    }

    #[local]
    struct Local {
        blue_led: BlueLedPin,
        delay: Delay<TIM1, 1000000>,
        lcd: Lcd,
        zero_crossing: ZeroCrossDetectPin,
        triak_ctrl: TriakControlPin,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let cp = ctx.core;

        let (clocks, blue_led, zero_crossing, triak_ctrl, mut lcd, mut xpt_drv, mut delay, exti) =
            init_preipherals(ctx.device).unwrap();
        /*
         * Indicator that stuff is running
         */
        let mut systic = cp.SYST.counter_hz(&clocks);
        systic.start(1.kHz()).unwrap();
        systic.listen(SysEvent::Update);

        let _ = lcd.clear(Rgb565::BLUE);

        xpt_drv.init(&mut delay);
        let button_up = Button::new("+", Point::new(10, 10), Size::new(45, 45), Rgb565::MAGENTA);
        let button_down = Button::new("-", Point::new(60, 10), Size::new(45, 45), Rgb565::MAGENTA);

        (
            Shared {
                xpt_drv,
                exti,
                gui: Gui {
                    button_up,
                    button_down,
                },
            },
            Local {
                blue_led,
                delay,
                lcd,
                zero_crossing,
                triak_ctrl,
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

    #[task(binds = EXTI15_10, local = [zero_crossing, triak_ctrl], shared = [])]
    fn zero_crossing(ctx: zero_crossing::Context) {}

    #[task(local = [], shared = [])]
    fn triak_control(ctx: triak_control::Context) {}

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
