//! ST7796 display driver.

use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_hal::delay::DelayNs;

use crate::{
    dcs::SetAddressMode,
    interface::Interface,
    options::ModelOptions,
};

use super::{InitError, Model, ST7789};

/// ST7796 display driver.
///
/// Display size: 320x480
#[derive(Debug, Clone, Copy, Default)]
pub struct ST7796;

impl Model for ST7796 {
    type ColorFormat = Rgb565;

    const FRAMEBUFFER_SIZE: (u16, u16) = (320, 480);

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
        // ST7796 uses the same init sequence as ST7789
        ST7789.init(di, delay, options)
    }
}