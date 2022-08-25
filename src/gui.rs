use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::Point,
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
    text::{Alignment, Text},
};
use heapless::String;
use ili9341::DisplayError;

pub trait GuiElement {
    fn contains_point(&self, point: Point) -> bool;
    fn is_pressed(&self) -> bool;
}

#[derive(Debug)]
pub struct Button {
    point: Point,
    size: Size,
    color: Rgb565,
    is_pressed: bool,
    pub name: String<16>,
}

impl Button {
    pub fn new<S: Into<String<16>>>(name: S, point: Point, size: Size, color: Rgb565) -> Self {
        Self {
            point,
            size,
            color,
            is_pressed: false,
            name: name.into(),
        }
    }

    fn contains_point(&self, p: &Point) -> bool {
        p.x > self.point.x
            && p.x < self.point.x + self.size.width as i32
            && p.y > self.point.y
            && p.y < self.point.y + self.size.height as i32
    }

    fn is_pressed(&self) -> bool {
        self.is_pressed
    }

    pub fn update(&mut self, point: &Point) {
        self.is_pressed = self.contains_point(point)
    }

    pub fn draw<D>(&self, g: &mut D) -> Result<(), DisplayError>
    where
        D: DrawTarget<Error = DisplayError, Color = Rgb565>,
    {
        let style = PrimitiveStyleBuilder::new()
            .stroke_color(Rgb565::RED)
            .stroke_width(3)
            .fill_color(self.color)
            .build();

        Rectangle::new(self.point, self.size)
            .into_styled(style)
            .draw(g)?;

        let style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
        let _ = Text::with_alignment(
            self.name.as_str(),
            Point::new(
                self.point.x + (self.size.width / 2) as i32,
                self.point.y + (self.size.height / 2) as i32,
            ),
            style,
            Alignment::Center,
        )
        .draw(g)?;
        Ok(())
    }
}

impl GuiElement for Button {
    fn contains_point(&self, p: Point) -> bool {
        self.contains_point(&p)
    }
    fn is_pressed(&self) -> bool {
        self.is_pressed()
    }
}

pub struct Gui {
    pub button_up: Button,
    pub button_down: Button,
}

impl Gui {
    pub fn draw<D>(&self, g: &mut D) -> Result<(), DisplayError>
    where
        D: DrawTarget<Error = DisplayError, Color = Rgb565>,
    {
        self.button_up.draw(g)?;
        self.button_down.draw(g)?;
        Ok(())
    }
}
