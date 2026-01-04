//! Display model definitions.
//!
//! This module contains the [`Model`] trait and implementations for various
//! display controllers.

use embedded_graphics_core::pixelcolor::{Rgb565, RgbColor};
use embedded_hal::delay::DelayNs;

use crate::{
    dcs::{self, InterfaceExt, SetAddressMode},
    interface::Interface,
    options::{ModelOptions, Rotation, TearingEffect},
};

/// Display model trait.
///
/// This trait defines the interface for display controller models.
/// Each model implementation provides initialization sequences and
/// configuration specific to that controller.
pub trait Model {
    /// The color format used by this model.
    type ColorFormat: RgbColor;

    /// The framebuffer size in pixels (width, height).
    const FRAMEBUFFER_SIZE: (u16, u16);

    /// Duration of the active low reset pulse in microseconds.
    const RESET_DURATION_US: u32 = 10;

    /// Delay after reset in microseconds.
    const RESET_DELAY_US: u32 = 120_000;

    /// Initialize the display.
    ///
    /// Returns the MADCTL value set during initialization.
    fn init<DI, DELAY>(
        &mut self,
        di: &mut DI,
        delay: &mut DELAY,
        options: &ModelOptions,
    ) -> Result<SetAddressMode, InitError<DI::Error>>
    where
        DI: Interface,
        DELAY: DelayNs;

    /// Update the address window.
    fn update_address_window<DI>(
        di: &mut DI,
        _rotation: Rotation,
        sx: u16,
        sy: u16,
        ex: u16,
        ey: u16,
    ) -> Result<(), DI::Error>
    where
        DI: Interface,
    {
        di.write_command(dcs::SetColumnAddress::new(sx, ex))?;
        di.write_command(dcs::SetPageAddress::new(sy, ey))
    }

    /// Enter sleep mode.
    fn sleep<DI, DELAY>(di: &mut DI, delay: &mut DELAY) -> Result<(), DI::Error>
    where
        DI: Interface,
        DELAY: DelayNs,
    {
        di.write_command(dcs::EnterSleepMode)?;
        delay.delay_us(120_000);
        Ok(())
    }

    /// Exit sleep mode.
    fn wake<DI, DELAY>(di: &mut DI, delay: &mut DELAY) -> Result<(), DI::Error>
    where
        DI: Interface,
        DELAY: DelayNs,
    {
        di.write_command(dcs::ExitSleepMode)?;
        delay.delay_us(120_000);
        Ok(())
    }

    /// Send WriteMemoryStart command before writing pixels.
    fn write_memory_start<DI>(di: &mut DI) -> Result<(), DI::Error>
    where
        DI: Interface,
    {
        di.write_command(dcs::WriteMemoryStart)
    }

    /// Software reset.
    fn software_reset<DI, DELAY>(di: &mut DI, delay: &mut DELAY) -> Result<(), DI::Error>
    where
        DI: Interface,
        DELAY: DelayNs,
    {
        di.write_command(dcs::SoftReset)?;
        delay.delay_us(150_000);
        Ok(())
    }

    /// Update options (called when orientation etc. changes).
    fn update_options<DI>(&self, di: &mut DI, options: &ModelOptions) -> Result<(), DI::Error>
    where
        DI: Interface,
    {
        let madctl = SetAddressMode::from(options);
        di.write_command(madctl)
    }

    /// Configure tearing effect output.
    fn set_tearing_effect<DI>(
        di: &mut DI,
        tearing_effect: TearingEffect,
        _options: &ModelOptions,
    ) -> Result<(), DI::Error>
    where
        DI: Interface,
    {
        di.write_command(dcs::SetTearingEffect::new(tearing_effect))
    }

    /// Set vertical scroll region.
    fn set_vertical_scroll_region<DI>(
        di: &mut DI,
        top_fixed_area: u16,
        bottom_fixed_area: u16,
    ) -> Result<(), DI::Error>
    where
        DI: Interface,
    {
        let rows = Self::FRAMEBUFFER_SIZE.1;
        let scroll_area = rows.saturating_sub(top_fixed_area).saturating_sub(bottom_fixed_area);
        di.write_command(dcs::SetScrollArea::new(top_fixed_area, scroll_area, bottom_fixed_area))
    }

    /// Set vertical scroll offset.
    fn set_vertical_scroll_offset<DI>(di: &mut DI, offset: u16) -> Result<(), DI::Error>
    where
        DI: Interface,
    {
        di.write_command(dcs::SetScrollStart::new(offset))
    }
}

/// Initialization error.
#[derive(Debug)]
pub enum InitError<E> {
    /// Interface error.
    Interface(E),
    /// Invalid configuration.
    InvalidConfiguration,
}

impl<E> From<E> for InitError<E> {
    fn from(e: E) -> Self {
        Self::Interface(e)
    }
}

// ============================================================================
// ST7789
// ============================================================================

/// ST7789 display driver.
///
/// Common display sizes:
/// - 240x240 (square)
/// - 240x320 (rectangle)
/// - 135x240 (T-Display)
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
        // Software reset
        Self::software_reset(di, delay)?;

        // Exit sleep mode
        di.write_command(dcs::ExitSleepMode)?;
        delay.delay_us(120_000);

        // Set color mode to 16-bit (RGB565)
        di.write_command(dcs::SetPixelFormat::new(dcs::PixelFormat::Bpp16))?;

        // Memory access control
        let madctl = SetAddressMode::from(options);
        di.write_command(madctl)?;

        // Set inversion mode
        di.write_command(dcs::SetInvertMode::new(options.invert_colors))?;

        // Normal display mode
        di.write_command(dcs::EnterNormalMode)?;

        // Display on
        di.write_command(dcs::SetDisplayOn)?;

        Ok(madctl)
    }
}

// ============================================================================
// GC9107
// ============================================================================

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
        // Software reset
        Self::software_reset(di, delay)?;

        // Exit sleep mode
        di.write_command(dcs::ExitSleepMode)?;
        delay.delay_us(120_000);

        // Inter register enable 1
        di.send_command(0xFE, &[])?;
        // Inter register enable 2
        di.send_command(0xEF, &[])?;

        // Set pixel format to 16-bit
        di.write_command(dcs::SetPixelFormat::new(dcs::PixelFormat::Bpp16))?;

        // Memory access control
        let madctl = SetAddressMode::from(options);
        di.write_command(madctl)?;

        // Set inversion (GC9107 typically needs inversion ON)
        di.write_command(dcs::SetInvertMode::new(options.invert_colors))?;

        // Display on
        di.write_command(dcs::SetDisplayOn)?;

        Ok(madctl)
    }
}

// ============================================================================
// ILI9341
// ============================================================================

/// ILI9341 display driver.
///
/// Common display sizes:
/// - 240x320
#[derive(Debug, Clone, Copy, Default)]
pub struct ILI9341;

impl Model for ILI9341 {
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
        // Software reset
        Self::software_reset(di, delay)?;

        // Exit sleep mode
        di.write_command(dcs::ExitSleepMode)?;
        delay.delay_us(120_000);

        // Power control A
        di.send_command(0xCB, &[0x39, 0x2C, 0x00, 0x34, 0x02])?;

        // Power control B
        di.send_command(0xCF, &[0x00, 0xC1, 0x30])?;

        // Driver timing control A
        di.send_command(0xE8, &[0x85, 0x00, 0x78])?;

        // Driver timing control B
        di.send_command(0xEA, &[0x00, 0x00])?;

        // Power on sequence control
        di.send_command(0xED, &[0x64, 0x03, 0x12, 0x81])?;

        // Pump ratio control
        di.send_command(0xF7, &[0x20])?;

        // Power control 1
        di.send_command(0xC0, &[0x23])?;

        // Power control 2
        di.send_command(0xC1, &[0x10])?;

        // VCOM control 1
        di.send_command(0xC5, &[0x3E, 0x28])?;

        // VCOM control 2
        di.send_command(0xC7, &[0x86])?;

        // Memory access control
        let madctl = SetAddressMode::from(options);
        di.write_command(madctl)?;

        // Pixel format
        di.write_command(dcs::SetPixelFormat::new(dcs::PixelFormat::Bpp16))?;

        // Frame rate control
        di.send_command(0xB1, &[0x00, 0x18])?;

        // Display function control
        di.send_command(0xB6, &[0x08, 0x82, 0x27])?;

        // Enable 3G (gamma)
        di.send_command(0xF2, &[0x00])?;

        // Gamma set
        di.send_command(0x26, &[0x01])?;

        // Positive gamma correction
        di.send_command(
            0xE0,
            &[
                0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1,
                0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
            ],
        )?;

        // Negative gamma correction
        di.send_command(
            0xE1,
            &[
                0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1,
                0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
            ],
        )?;

        // Set inversion mode
        di.write_command(dcs::SetInvertMode::new(options.invert_colors))?;

        // Normal display mode on
        di.write_command(dcs::EnterNormalMode)?;

        // Display on
        di.write_command(dcs::SetDisplayOn)?;

        Ok(madctl)
    }
}