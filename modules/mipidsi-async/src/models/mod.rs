//! Display model implementations.
//!
//! Each model defines the initialization sequence and framebuffer size
//! for a specific display controller.
//!
//! # Supported Models
//!
//! - [`ST7789`] - Common on many boards (240x320)
//! - [`ST7796`] - Large displays (320x480)
//! - [`ST7735s`] - Small TFT displays (132x162)
//! - [`GC9107`] - Used in some small displays (128x160)
//! - [`GC9A01`] - Round displays (240x240)
//! - [`ILI9341`] - Very common TFT controller (240x320)
//! - [`ILI9342C`] - M5Stack displays (320x240)

mod gc9107;
mod gc9a01;
mod ili934x;
mod ili9341;
mod ili9342c;
mod st7735s;
mod st7789;
mod st7796;

pub use gc9107::GC9107;
pub use gc9a01::GC9A01;
pub use ili9341::ILI9341;
pub use ili9342c::ILI9342C;
pub use st7735s::ST7735s;
pub use st7789::ST7789;
pub use st7796::ST7796;

use embedded_graphics_core::pixelcolor::{PixelColor, Rgb565};
use embedded_hal::delay::DelayNs;

use crate::{
    dcs::{self, InterfaceExt, SetAddressMode},
    interface::Interface,
    options::{ModelOptions, Rotation},
};

/// Error during display initialization.
#[derive(Debug)]
pub enum InitError<E> {
    /// Interface error
    Interface(E),
}

impl<E> From<E> for InitError<E> {
    fn from(e: E) -> Self {
        InitError::Interface(e)
    }
}

/// Display model trait.
///
/// This trait defines the interface for display controller models.
/// Each model implements the initialization sequence specific to its controller.
pub trait Model: Sized {
    /// Color format supported by the model.
    type ColorFormat: PixelColor;

    /// Framebuffer size (width, height) of the controller's internal memory.
    ///
    /// This may be larger than the actual display size.
    const FRAMEBUFFER_SIZE: (u16, u16);

    /// Hardware reset duration in microseconds.
    const RESET_DURATION_US: u32 = 10_000;

    /// Delay after hardware reset in microseconds.
    const RESET_DELAY_US: u32 = 120_000;

    /// Initialize the display.
    ///
    /// This method sends the initialization commands to the display controller.
    fn init<DI, DELAY>(
        &mut self,
        di: &mut DI,
        delay: &mut DELAY,
        options: &ModelOptions,
    ) -> Result<SetAddressMode, InitError<DI::Error>>
    where
        DI: Interface,
        DELAY: DelayNs;

    /// Perform software reset.
    fn software_reset<DI, DELAY>(di: &mut DI, delay: &mut DELAY) -> Result<(), DI::Error>
    where
        DI: Interface,
        DELAY: DelayNs,
    {
        di.write_command(dcs::SoftReset)?;
        delay.delay_us(150_000);
        Ok(())
    }

    /// Update the address window.
    ///
    /// This is called before each framebuffer transfer to set the target area.
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

    /// Wake from sleep mode.
    fn wake<DI, DELAY>(di: &mut DI, delay: &mut DELAY) -> Result<(), DI::Error>
    where
        DI: Interface,
        DELAY: DelayNs,
    {
        di.write_command(dcs::ExitSleepMode)?;
        delay.delay_us(120_000);
        Ok(())
    }

    /// Write memory start command.
    ///
    /// This command prepares the display to receive pixel data.
    fn write_memory_start<DI>(di: &mut DI) -> Result<(), DI::Error>
    where
        DI: Interface,
    {
        di.write_command(dcs::WriteMemoryStart)
    }
}