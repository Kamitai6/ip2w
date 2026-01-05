//! GC9107 display driver.

use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_hal::delay::DelayNs;

use crate::{
    dcs::{self, InterfaceExt, SetAddressMode},
    interface::Interface,
    options::ModelOptions,
};

use super::{InitError, Model};

/// GC9107 display driver.
///
/// Common display sizes:
/// - 128x128
/// - 128x160
#[derive(Debug, Clone, Copy, Default)]
pub struct GC9107;

impl Model for GC9107 {
    type ColorFormat = Rgb565;

    const FRAMEBUFFER_SIZE: (u16, u16) = (128, 160);

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
        // Initial delay
        delay.delay_ms(200);

        // Inter register enable 1
        di.send_command(0xFE, &[])?;
        delay.delay_ms(5);

        // Inter register enable 2
        di.send_command(0xEF, &[])?;
        delay.delay_ms(5);

        // Undocumented registers (from reference implementation)
        di.send_command(0xB0, &[0xC0])?;
        di.send_command(0xB2, &[0x2F])?;
        di.send_command(0xB3, &[0x03])?;
        di.send_command(0xB6, &[0x19])?;
        di.send_command(0xB7, &[0x01])?;

        // Memory access control (MADCTL)
        let madctl = SetAddressMode::from(options);
        di.write_command(madctl)?;

        // More undocumented registers
        di.send_command(0xAC, &[0xCB])?;
        di.send_command(0xAB, &[0x0E])?;
        di.send_command(0xB4, &[0x04])?;
        di.send_command(0xA8, &[0x19])?;

        // Set pixel format to 16-bit (RGB565)
        di.write_command(dcs::SetPixelFormat::new(dcs::PixelFormat::Bpp16))?;

        // More undocumented registers
        di.send_command(0xB8, &[0x08])?;
        di.send_command(0xE8, &[0x24])?;
        di.send_command(0xE9, &[0x48])?;
        di.send_command(0xEA, &[0x22])?;
        di.send_command(0xC6, &[0x30])?;
        di.send_command(0xC7, &[0x18])?;

        // Gamma settings (positive)
        di.send_command(
            0xF0,
            &[
                0x01, 0x2B, 0x23, 0x3C, 0xB7, 0x12, 0x17, 0x60, 
                0x00, 0x06, 0x0C, 0x17, 0x12, 0x1F,
            ],
        )?;

        // Gamma settings (negative)
        di.send_command(
            0xF1,
            &[
                0x05, 0x2E, 0x2D, 0x44, 0xD6, 0x15, 0x17, 0xA0, 
                0x02, 0x0D, 0x0D, 0x1A, 0x18, 0x1F,
            ],
        )?;

        // Set inversion mode
        di.write_command(dcs::SetInvertMode::new(options.invert_colors))?;

        // Exit sleep mode
        di.write_command(dcs::ExitSleepMode)?;
        delay.delay_ms(120);

        // Display on
        di.write_command(dcs::SetDisplayOn)?;

        Ok(madctl)
    }
}