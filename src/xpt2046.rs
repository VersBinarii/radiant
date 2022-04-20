use core::{fmt::Debug, ops::RemAssign};
use embedded_graphics::{
    pixelcolor::Rgb565,
    primitives::{Line, Primitive, PrimitiveStyle},
    Drawable,
};
use embedded_graphics_core::{draw_target::DrawTarget, geometry::Point, pixelcolor::RgbColor};
use embedded_hal::{
    delay::blocking::DelayUs, digital::blocking::OutputPin, spi::blocking::Transfer,
};

const CHANNEL_SETTING_X: u8 = 0b10010000;
const CHANNEL_SETTING_Y: u8 = 0b11010000;

#[derive(Debug)]
pub enum BusError<SPIError, CSError> {
    Spi(SPIError),
    Pin(CSError),
}

#[derive(Debug)]
pub enum Error<E> {
    /// SPI bus error
    Bus(E),
}

#[cfg(feature = "with_defmt")]
impl<E> Format for Error<E> {
    fn format(&self, fmt: Formatter) {
        match self {
            Error::Bus(_) => write!(fmt, "Bus error"),
        }
    }
}
const MAX_SAMPLES: usize = 32;
const TX_BUFF_LEN: usize = 5;

#[derive(Debug, Clone)]
pub struct CalibrationPoint {
    a: Point,
    b: Point,
    c: Point,
}

impl CalibrationPoint {
    pub fn delta(&self) -> i32 {
        (self.a[0] - self.c[0]) * (self.b[1] - self.c[1])
            - (self.b[0] - self.c[0]) * (self.a[1] - self.c[1])
    }
}

pub struct CalibrationData {
    pub alpha_x: f32,
    pub beta_x: f32,
    pub delta_x: f32,
    pub alpha_y: f32,
    pub beta_y: f32,
    pub delta_y: f32,
}

pub enum Orientation {
    Portrait,
    PortraitFlipped,
    Landscape,
    LandscapeFlipped,
}

impl Orientation {
    pub fn calibration_point(&self) -> CalibrationPoint {
        match self {
            Orientation::Portrait | &Orientation::PortraitFlipped => CalibrationPoint {
                a: Point::new(10, 10),
                b: Point::new(80, 210),
                c: Point::new(200, 170),
            },
            Orientation::Landscape | Orientation::LandscapeFlipped => CalibrationPoint {
                a: Point::new(20, 25),
                b: Point::new(160, 220),
                c: Point::new(300, 110),
            },
        }
    }

    pub fn calibration_data(&self) -> CalibrationData {
        match self {
            Orientation::Portrait => CalibrationData {
                alpha_x: -0.0009337,
                beta_x: -0.0636839,
                delta_x: 250.342,
                alpha_y: -0.0889775,
                beta_y: -0.00118110,
                delta_y: 356.538,
            },
            Orientation::PortraitFlipped => CalibrationData {
                alpha_x: 0.0006100,
                beta_x: 0.0647828,
                delta_x: -13.634,
                alpha_y: 0.0890609,
                beta_y: 0.0001381,
                delta_y: -35.73,
            },
            Orientation::Landscape => CalibrationData {
                alpha_x: -0.0885542,
                beta_x: 0.0016532,
                delta_x: 349.800,
                alpha_y: 0.0007309,
                beta_y: 0.06543699,
                delta_y: -15.290,
            },
            Orientation::LandscapeFlipped => CalibrationData {
                alpha_x: 0.0902216,
                beta_x: 0.0006510,
                delta_x: -38.657,
                alpha_y: -0.0010005,
                beta_y: -0.0667030,
                delta_y: 258.08,
            },
        }
    }
}

#[derive(PartialEq, Debug, defmt::Format)]
pub enum TouchScreenState {
    IDLE,
    PRESAMPLING,
    TOUCHED,
    RELEASED,
}

pub enum TouchScreenOperationMode {
    NORMAL,
    CALIBRATION,
}

pub struct TouchSamples {
    sample_timer: u32,
    samples: [Point; MAX_SAMPLES],
    counter: usize,
}

impl core::default::Default for TouchSamples {
    fn default() -> Self {
        Self {
            sample_timer: 0,
            counter: 0,
            samples: [Point::default(); MAX_SAMPLES],
        }
    }
}

impl TouchSamples {
    pub fn average(&self) -> Point {
        let mut x = 0;
        let mut y = 0;

        for point in self.samples {
            x += point.x;
            y += point.y;
        }
        x /= MAX_SAMPLES as i32;
        y /= MAX_SAMPLES as i32;
        Point::new(x, y)
    }
}

pub struct Xpt2046<SPI, CS> {
    spi: SPI,
    cs: CS,
    tx_buff: [u8; TX_BUFF_LEN],
    rx_buff: [u8; TX_BUFF_LEN],
    screen_state: TouchScreenState,
    ts: TouchSamples,
    calibration_point: CalibrationPoint,
    calibration_data: CalibrationData,
    operation_mode: TouchScreenOperationMode,
}

impl<SPI, CS> Xpt2046<SPI, CS>
where
    SPI: Transfer<u8>,
    CS: OutputPin,
{
    pub fn new(spi: SPI, cs: CS, orientation: Orientation) -> Self {
        Self {
            spi,
            cs,
            tx_buff: [0; TX_BUFF_LEN],
            rx_buff: [0; TX_BUFF_LEN],
            screen_state: TouchScreenState::IDLE,
            ts: TouchSamples::default(),
            calibration_point: orientation.calibration_point(),
            calibration_data: orientation.calibration_data(),
            operation_mode: TouchScreenOperationMode::NORMAL,
        }
    }
}

impl<SPI, CS, SPIError, CSError> Xpt2046<SPI, CS>
where
    SPI: Transfer<u8, Error = SPIError>,
    CS: OutputPin<Error = CSError>,
    SPIError: core::fmt::Debug,
    CSError: core::fmt::Debug,
{
    fn spi_read(&mut self) -> Result<(), Error<BusError<SPIError, CSError>>> {
        self.cs
            .set_low()
            .map_err(|e| Error::Bus(BusError::Pin(e)))?;
        self.spi
            .transfer(&mut self.rx_buff, &self.tx_buff)
            .map_err(|e| Error::Bus(BusError::Spi(e)))?;
        self.cs
            .set_high()
            .map_err(|e| Error::Bus(BusError::Pin(e)))?;
        Ok(())
    }

    pub fn read_xy(&mut self) -> Result<Point, Error<BusError<SPIError, CSError>>> {
        self.spi_read()?;

        let x = (self.rx_buff[1] as i32) << 8 | self.rx_buff[2] as i32;
        let y = (self.rx_buff[3] as i32) << 8 | self.rx_buff[4] as i32;
        Ok(Point::new(x, y))
    }

    pub fn read_touch_point(&mut self) -> Result<Point, Error<BusError<SPIError, CSError>>> {
        let raw_point = self.read_xy()?;

        let (x, y) = match self.operation_mode {
            TouchScreenOperationMode::NORMAL => {
                let x = self.calibration_data.alpha_x * raw_point.x as f32
                    + self.calibration_data.beta_x * raw_point.y as f32
                    + self.calibration_data.delta_x;
                let y = self.calibration_data.alpha_y * raw_point.x as f32
                    + self.calibration_data.beta_y * raw_point.y as f32
                    + self.calibration_data.delta_y;

                (x as i32, y as i32)
            }
            TouchScreenOperationMode::CALIBRATION => {
                /*
                 * We're running calibration so just return raw
                 * point measurements without compensation
                 */
                (raw_point.x, raw_point.y)
            }
        };
        Ok(Point::new(x, y))
    }

    pub fn is_touched(&self) -> bool {
        self.screen_state == TouchScreenState::TOUCHED
    }

    /*
     * Sometimes the TOUCHED state needs to be cleared
     */
    pub fn clear_touch(&mut self) {
        self.screen_state = TouchScreenState::PRESAMPLING;
    }

    pub fn get_touch_point(&self) -> Point {
        self.ts.average()
    }

    pub fn init<D: DelayUs>(&mut self, delay: &mut D) {
        self.tx_buff[0] = 0x80;
        self.cs.set_high().unwrap();
        self.spi_read().unwrap();
        delay.delay_ms(1).unwrap();

        // Load the tx_buffer with the channels config
        // for all subsequent reads
        self.tx_buff = [
            CHANNEL_SETTING_X >> 3,
            CHANNEL_SETTING_X << 5,
            CHANNEL_SETTING_Y >> 3,
            CHANNEL_SETTING_Y << 5,
            0,
        ];
    }

    pub fn run(&mut self, is_released: bool, f: &mut impl FnMut()) {
        match self.screen_state {
            TouchScreenState::IDLE => {}
            TouchScreenState::PRESAMPLING => {
                if is_released {
                    self.screen_state = TouchScreenState::RELEASED
                }
                let point_sample = self.read_touch_point().unwrap();
                self.ts.samples[self.ts.counter] = point_sample;
                self.ts.counter += 1;
                if self.ts.counter + 1 == MAX_SAMPLES {
                    self.ts.counter = 0;
                    self.screen_state = TouchScreenState::TOUCHED;
                }
            }
            TouchScreenState::TOUCHED => {
                let point_sample = self.read_touch_point().unwrap();
                self.ts.samples[self.ts.counter] = point_sample;
                self.ts.counter += 1;
                /*
                 * Wrap around the counter if the screen
                 * is touched for longer time
                 */
                self.ts.counter.rem_assign(MAX_SAMPLES - 1);
                if is_released {
                    self.screen_state = TouchScreenState::RELEASED
                }
            }
            TouchScreenState::RELEASED => {
                self.screen_state = TouchScreenState::IDLE;
                self.ts.counter = 0;
                /*
                 * The PENIRQ should be re-enabled in here
                 * as we finished sending any data to the touch controller
                 */
                f()
            }
        }
    }

    pub fn exti_irq_handle(&mut self, mut f: impl FnMut()) {
        /*
         * Disable the PENIRQ so that it wont be false triggering our handler
         * as per XPT2046 Touch Screen Controller datasheet page 25
         *
         * It is recommended that the processor mask the interrupt PENIRQ
         * is associated with whenever the processor sends
         * a control byte to the XPT2046. This prevents false triggering of
         * interrupts when the PENIRQ output is disabled in the cases
         * discussed in this section.
         */
        f();
        self.screen_state = TouchScreenState::PRESAMPLING;
    }

    pub fn calibrate<DT, DELAY>(&mut self, dt: &mut DT, delay: &mut DELAY, f: &mut impl FnMut())
    where
        DT: DrawTarget<Color = Rgb565>,
        DELAY: DelayUs,
    {
        let mut calibration_count = 0;
        let mut new_a = Point::zero();
        let mut new_b = Point::zero();
        let mut new_c = Point::zero();
        let old_cp = self.calibration_point.clone();
        // Prepare the screen for points
        let _ = dt.clear(Rgb565::BLACK);

        // Set correct state to fetch raw data from touch controller
        self.operation_mode = TouchScreenOperationMode::CALIBRATION;
        while calibration_count < 4 {
            defmt::println!("Calibrating step: {}", calibration_count);
            // We must run our state machine to capture user input
            self.run(true, f);
            match calibration_count {
                0 => {
                    calibration_draw_point(dt, old_cp.a);
                    if self.screen_state == TouchScreenState::TOUCHED {
                        new_a = self.get_touch_point();
                        let _ = delay.delay_ms(200);
                        calibration_count += 1;
                    }
                }

                1 => {
                    calibration_draw_point(dt, old_cp.b);
                    if self.screen_state == TouchScreenState::TOUCHED {
                        new_b = self.get_touch_point();
                        let _ = delay.delay_ms(200);
                        calibration_count += 1;
                    }
                }
                2 => {
                    calibration_draw_point(dt, old_cp.c);
                    if self.screen_state == TouchScreenState::TOUCHED {
                        new_c = self.get_touch_point();
                        let _ = delay.delay_ms(200);
                        calibration_count += 1;
                    }
                }

                3 => {
                    // Create new calibration point from the captured samples
                    let new_calibration_point = CalibrationPoint {
                        a: new_a,
                        b: new_b,
                        c: new_c,
                    };
                    self.calibration_point = new_calibration_point;
                    // and then re-caculate calibration
                    //calibration_calculate (*this);
                    let new_calibration_data =
                        calculate_calibration(&old_cp, &self.calibration_point);
                    self.calibration_data = new_calibration_data;
                    calibration_count += 1;
                }
                _ => {}
            }
        }
        self.operation_mode = TouchScreenOperationMode::NORMAL;
    }
}

fn calibration_draw_point<DT: DrawTarget<Color = Rgb565>>(dt: &mut DT, p: Point) {
    let _ = Line::new(Point::new(p.x - 4, p.y), Point::new(p.x + 4, p.y))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
        .draw(dt);
    let _ = Line::new(Point::new(p.x, p.y - 4), Point::new(p.x, p.y + 4))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
        .draw(dt);
}

fn calculate_calibration(old_cp: &CalibrationPoint, new_cp: &CalibrationPoint) -> CalibrationData {
    let delta = new_cp.delta() as f32;
    let alpha_x = ((old_cp.a[0] - old_cp.c[0]) * (new_cp.b[1] - new_cp.c[1])
        - (old_cp.b[0] - old_cp.c[0]) * (new_cp.a[1] - new_cp.c[1])) as f32
        / delta;

    let beta_x = ((new_cp.a[0] - new_cp.c[0]) * (old_cp.b[0] - old_cp.c[0])
        - (new_cp.b[0] - new_cp.c[0]) * (old_cp.a[0] - old_cp.c[0])) as f32
        / delta;

    let delta_x = ((old_cp.a[0]) * (new_cp.b[0] * new_cp.c[1] - new_cp.c[0] * new_cp.b[1])
        - (old_cp.b[0]) * (new_cp.a[0] * new_cp.c[1] - new_cp.c[0] * new_cp.a[1])
        + (old_cp.c[0]) * (new_cp.a[0] * new_cp.b[1] - new_cp.b[0] * new_cp.a[1]))
        as f32
        / delta;

    let alpha_y = ((old_cp.a[1] - old_cp.c[1]) * (new_cp.b[1] - new_cp.c[1])
        - (old_cp.b[1] - old_cp.c[1]) * (new_cp.a[1] - new_cp.c[1])) as f32
        / delta;

    let beta_y = ((new_cp.a[0] - new_cp.c[0]) * (old_cp.b[1] - old_cp.c[1])
        - (new_cp.b[0] - new_cp.c[0]) * (old_cp.a[1] - old_cp.c[1])) as f32
        / delta;

    let delta_y = ((old_cp.a[1]) * (new_cp.b[0] * new_cp.c[1] - new_cp.c[0] * new_cp.b[1])
        - (old_cp.b[1]) * (new_cp.a[0] * new_cp.c[1] - new_cp.c[0] * new_cp.a[1])
        + (old_cp.c[1]) * (new_cp.a[0] * new_cp.b[1] - new_cp.b[0] * new_cp.a[1]))
        as f32
        / delta;
    CalibrationData {
        alpha_x,
        beta_x,
        delta_x,
        alpha_y,
        beta_y,
        delta_y,
    }
}
