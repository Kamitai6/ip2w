//! ST7735s display driver.

use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_hal::delay::DelayNs;

use crate::{
    dcs::{self, InterfaceExt, SetAddressMode},
    interface::Interface,
    options::ModelOptions,
};

use super::{InitError, Model};

/// ST7735s display driver.
///
/// Common display sizes:
/// - 128x160
/// - 80x160
#[derive(Debug, Clone, Copy, Default)]
pub struct ST7735s;

impl Model for ST7735s {
    type ColorFormat = Rgb565;

    const FRAMEBUFFER_SIZE: (u16, u16) = (132, 162);

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

        delay.delay_us(200_000);

        // Exit sleep mode
        di.write_command(dcs::ExitSleepMode)?;
        delay.delay_us(120_000);

        // Set inversion mode
        di.write_command(dcs::SetInvertMode::new(options.invert_colors))?;

        // Frame rate control
        di.write_raw(0xB1, &[0x05, 0x3A, 0x3A])?;
        di.write_raw(0xB2, &[0x05, 0x3A, 0x3A])?;
        di.write_raw(0xB3, &[0x05, 0x3A, 0x3A, 0x05, 0x3A, 0x3A])?;

        // Inversion control
        di.write_raw(0xB4, &[0b0000_0011])?;

        // Power control
        di.write_raw(0xC0, &[0x62, 0x02, 0x04])?; // power control 1
        di.write_raw(0xC1, &[0xC0])?; // power control 2
        di.write_raw(0xC2, &[0x0D, 0x00])?; // power control 3
        di.write_raw(0xC3, &[0x8D, 0x6A])?; // power control 4
        di.write_raw(0xC4, &[0x8D, 0xEE])?; // power control 5
        di.write_raw(0xC5, &[0x0E])?; // VCOM control 1

        // Gamma settings
        di.write_raw(
            0xE0,
            &[
                0x10, 0x0E, 0x02, 0x03, 0x0E, 0x07, 0x02, 0x07,
                0x0A, 0x12, 0x27, 0x37, 0x00, 0x0D, 0x0E, 0x10,
            ],
        )?; // GAMMA +Polarity
        di.write_raw(
            0xE1,
            &[
                0x10, 0x0E, 0x03, 0x03, 0x0F, 0x06, 0x02, 0x08,
                0x0A, 0x13, 0x26, 0x36, 0x00, 0x0D, 0x0E, 0x10,
            ],
        )?; // GAMMA -Polarity

        // Set pixel format to 16-bit (RGB565)
        di.write_command(dcs::SetPixelFormat::new(dcs::PixelFormat::Bpp16))?;

        // Memory access control
        di.write_command(madctl)?;

        // Display on
        di.write_command(dcs::SetDisplayOn)?;

        Ok(madctl)
    }
}