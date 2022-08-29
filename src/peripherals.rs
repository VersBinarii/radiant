use crate::dummy_pin::{DummyOutputPin, MyIrq};
use display_interface_spi::SPIInterface;
use ili9341::{DisplaySize240x320, Ili9341, Orientation};
use stm32f4xx_hal::{
    gpio::{Alternate, Edge, Input, NoPin, Output, Pin, PushPull},
    pac::{Peripherals, EXTI, SPI1, SPI5, TIM1},
    prelude::*,
    rcc::Clocks,
    spi::{Mode, NoMiso, Phase, Polarity, Spi},
    timer::{Channel, Delay},
};
use xpt2046::{self, Xpt2046};

/*
 * Touch driver interface
 * Controlled through the SPI 1 instance
 */
pub type TouchSpi = Spi<
    SPI1,
    (
        Pin<'A', 5, Alternate<5>>,
        Pin<'A', 6, Alternate<5>>,
        Pin<'A', 7, Alternate<5>>,
    ),
>;
pub type TouchDriver = Xpt2046<TouchSpi, Pin<'A', 4, Output<PushPull>>, MyIrq<'A', 2>>;

/*
 * LCD driver interface
 * Controlled through the SPI 5 instance
 */
pub type Lcd = Ili9341<
    SPIInterface<
        Spi<SPI5, (Pin<'B', 0, Alternate<6>>, NoPin, Pin<'A', 10, Alternate<6>>)>,
        Pin<'B', 1, Output>,
        Pin<'B', 2, Output>,
    >,
    DummyOutputPin,
>;

/*
 * Blue system led indicating system status
 */
pub type BlueLedPin = Pin<'C', 13, Output<PushPull>>;

/*
 *  Pin used for detection of the AC zeto crossing.
 *  Its connected to the LTV-814 opto coupler directly.
 *  Goes low whenever AC crosses the 0V.
 */
pub type ZeroCrossDetectPin = Pin<'B', 14, Input>;

/*
 * Used for driving the Triaks.
 * Connected directly to the 4N35 opto
 */
pub type TriakControlPin = Pin<'B', 15, Output<PushPull>>;

/*
 * TODO: Define error
 */
#[allow(clippy::type_complexity, clippy::result_unit_err)]
pub fn init_preipherals(
    mut dp: Peripherals,
) -> Result<
    (
        Clocks,
        BlueLedPin,
        ZeroCrossDetectPin,
        TriakControlPin,
        Lcd,
        TouchDriver,
        Delay<TIM1, 1000000>,
        EXTI,
    ),
    (),
> {
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(100.MHz()).freeze();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    /*
     * We need a backlight control
     * TODO: We will need to be able to change the brighntess later
     */
    let backlight_pins = gpioa.pa3.into_alternate();
    let mut pwm = dp.TIM2.pwm_hz(backlight_pins, 10.kHz(), &clocks);
    pwm.enable(Channel::C4);
    pwm.set_duty(Channel::C4, pwm.get_max_duty() / 2);

    let blue_led = gpioc.pc13.into_push_pull_output();
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
    let lcd = Ili9341::new(
        spi_iface,
        dummy_reset,
        &mut delay,
        Orientation::PortraitFlipped,
        DisplaySize240x320,
    )
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

    let xpt_drv = Xpt2046::new(
        touch_spi,
        touch_cs,
        MyIrq(touch_irq),
        xpt2046::Orientation::PortraitFlipped,
    );

    let mut zero_crossing = gpiob.pb14.into_pull_up_input();
    zero_crossing.make_interrupt_source(&mut syscfg);
    zero_crossing.trigger_on_edge(&mut dp.EXTI, Edge::Falling);
    zero_crossing.enable_interrupt(&mut dp.EXTI);
    let triak_ctrl = gpiob.pb15.into_push_pull_output();

    Ok((
        clocks,
        blue_led,
        zero_crossing,
        triak_ctrl,
        lcd,
        xpt_drv,
        delay,
        dp.EXTI,
    ))
}
