use embedded_hal::digital::{blocking::OutputPin, ErrorType, PinState};
use stm32f4xx_hal::{
    gpio::{ExtiPin, Pin},
    pac::EXTI,
};
use xpt2046::Xpt2046Exti;

#[derive(Default)]
pub struct DummyOutputPin;

impl ErrorType for DummyOutputPin {
    type Error = ();
}
impl OutputPin for DummyOutputPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_state(&mut self, _state: PinState) -> Result<(), Self::Error> {
        Ok(())
    }
}

pub struct MyIrq<const C: char, const N: u8>(pub Pin<C, N>);

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
