//! ILI9341 display driver.

use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_hal::delay::DelayNs;

use crate::{
    dcs::{self, SetAddressMode},
    interface::Interface,
    options::ModelOptions,
};

use super::{ili934x, InitError, Model};

/// ILI9341 display driver in RGB565 color mode.
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
        let pf = dcs::PixelFormat::Bpp16;
        ili934x::init_common(di, delay, options, pf).map_err(Into::into)
    }
}