use core::ops::RemAssign;

use embedded_graphics_core::geometry::Point;
use embedded_hal::{
    digital::blocking::{InputPin, OutputPin},
    spi::blocking::Transfer,
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

pub struct CalibrationPoint {
    a: [i16; 2],
    b: [i16; 2],
    c: [i16; 2],
}

impl CalibrationPoint {
    pub fn delta(&self) -> i16 {
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
                a: [10, 10],
                b: [80, 210],
                c: [200, 170],
            },
            Orientation::Landscape | Orientation::LandscapeFlipped => CalibrationPoint {
                a: [20, 25],
                b: [160, 220],
                c: [300, 110],
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

#[derive(PartialEq, Debug)]
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
    counter: u8,
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
        let tx_buff = [
            CHANNEL_SETTING_X >> 3,
            CHANNEL_SETTING_X << 5,
            CHANNEL_SETTING_Y >> 3,
            CHANNEL_SETTING_Y << 5,
            0,
        ];
        Self {
            spi,
            cs,
            tx_buff,
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

    pub fn run(&mut self, is_released: bool, mut f: impl FnMut()) {
        match self.screen_state {
            TouchScreenState::IDLE => {}
            TouchScreenState::PRESAMPLING => {
                if is_released {
                    self.screen_state = TouchScreenState::RELEASED
                }
                let point_sample = self.read_touch_point().unwrap();
                self.ts.samples[self.ts.counter as usize] = point_sample;
                self.ts.counter += 1;
                if self.ts.counter as usize == MAX_SAMPLES {
                    self.ts.counter = 0;
                    self.screen_state = TouchScreenState::TOUCHED;
                }
            }
            TouchScreenState::TOUCHED => {
                let point_sample = self.read_touch_point().unwrap();
                self.ts.samples[self.ts.counter as usize] = point_sample;
                self.ts.counter += 1;
                (self.ts.counter as usize).rem_assign(MAX_SAMPLES);
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
}
