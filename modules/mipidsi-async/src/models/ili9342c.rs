//! ILI9342C display driver.

use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_hal::delay::DelayNs;

use crate::{
    dcs::{self, SetAddressMode},
    interface::Interface,
    options::ModelOptions,
};

use super::{ili934x, InitError, Model};

/// ILI9342C display driver (RGB565).
///
/// Common on M5Stack devices.
///
/// Display size: 320x240
#[derive(Debug, Clone, Copy, Default)]
pub struct ILI9342C;

impl Model for ILI9342C {
    type ColorFormat = Rgb565;

    const FRAMEBUFFER_SIZE: (u16, u16) = (320, 240);

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
        let pf = dcs::PixelFormat::Bpp16;
        ili934x::init_common(di, delay, options, pf).map_err(Into::into)
    }
}