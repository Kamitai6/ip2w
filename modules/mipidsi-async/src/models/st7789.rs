//! ST7789 display driver.

use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_hal::delay::DelayNs;

use crate::{
    dcs::{self, InterfaceExt, SetAddressMode},
    interface::Interface,
    options::ModelOptions,
};

use super::{InitError, Model};

/// ST7789 display driver.
///
/// Common display sizes:
/// - 240x240
/// - 240x280
/// - 240x320
#[derive(Debug, Clone, Copy, Default)]
pub struct ST7789;

impl Model for ST7789 {
    type ColorFormat = Rgb565;

    const FRAMEBUFFER_SIZE: (u16, u16) = (240, 320);

    fn init<DI, DELAY>(
        &mut self,
        di: &mut DI,
        delay: &mut DELAY,
        options: &ModelOptions,
    ) -> Result<SetAddressMode, InitError<DI::Error>>
    where
        DI: Interface,
        DELAY: DelayNs,
    {
        let madctl = SetAddressMode::from(options);

        delay.delay_us(150_000);

        di.write_command(dcs::ExitSleepMode)?;
        delay.delay_us(10_000);

        // Memory access control
        di.write_command(madctl)?;

        // Set inversion mode
        di.write_command(dcs::SetInvertMode::new(options.invert_colors))?;

        // Set pixel format to 16-bit (RGB565)
        di.write_command(dcs::SetPixelFormat::new(dcs::PixelFormat::Bpp16))?;
        delay.delay_us(10_000);

        // Enter normal mode
        di.write_command(dcs::EnterNormalMode)?;
        delay.delay_us(10_000);

        // Display on
        di.write_command(dcs::SetDisplayOn)?;

        // DISPON requires some time otherwise we risk SPI data issues
        delay.delay_us(120_000);

        Ok(madctl)
    }
}