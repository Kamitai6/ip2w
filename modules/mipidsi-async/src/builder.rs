//! Display builder for configuration and initialization.

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

use crate::{
    interface::Interface,
    models::{InitError, Model},
    options::{ColorInversion, ColorOrder, ModelOptions, Orientation, RefreshOrder},
    Display,
};

/// Display builder.
///
/// Use this to configure and initialize a display.
///
/// # Example
///
/// ```ignore
/// use mipidsi_async::{Builder, models::ST7789};
///
/// let display = unsafe {
///     Builder::new(ST7789, interface)
///         .reset_pin(&mut rst)
///         .display_size(128, 128)
///         .display_offset(0, 0)
///         .orientation(Orientation::default())
///         .color_order(ColorOrder::Rgb)
///         .invert_colors(ColorInversion::Normal)
///         .init(&mut delay, &raw mut FB_A, &raw mut FB_B)
///         .unwrap()
/// };
/// ```
pub struct Builder<DI, MODEL, RST>
where
    DI: Interface,
    MODEL: Model,
{
    di: DI,
    model: MODEL,
    options: ModelOptions,
    reset_pin: Option<RST>,
}

/// Type alias for Builder without reset pin
pub type BuilderNoRst<DI, MODEL> = Builder<DI, MODEL, NoPin>;

/// Placeholder type for no reset pin
pub struct NoPin;

impl<DI, MODEL> Builder<DI, MODEL, NoPin>
where
    DI: Interface,
    MODEL: Model,
{
    /// Create a new builder with the given model and interface.
    pub fn new(model: MODEL, di: DI) -> Self {
        Self {
            di,
            model,
            options: ModelOptions::with_framebuffer_size(MODEL::FRAMEBUFFER_SIZE),
            reset_pin: None,
        }
    }

    /// Set the reset pin.
    ///
    /// If set, the display will be hardware reset during initialization.
    #[must_use]
    pub fn reset_pin<RST: OutputPin>(self, rst: RST) -> Builder<DI, MODEL, RST> {
        Builder {
            di: self.di,
            model: self.model,
            options: self.options,
            reset_pin: Some(rst),
        }
    }
}

impl<DI, MODEL, RST> Builder<DI, MODEL, RST>
where
    DI: Interface,
    MODEL: Model,
{
    /// Set the display size in pixels.
    ///
    /// This is the visible area of the display, which may be smaller than
    /// the framebuffer size.
    #[must_use]
    pub fn display_size(mut self, width: u16, height: u16) -> Self {
        self.options.display_size = (width, height);
        self
    }

    /// Set the display offset.
    ///
    /// This is useful when the visible area doesn't start at (0, 0) in the
    /// framebuffer.
    #[must_use]
    pub fn display_offset(mut self, x: u16, y: u16) -> Self {
        self.options.display_offset = (x, y);
        self
    }

    /// Set the initial orientation.
    #[must_use]
    pub fn orientation(mut self, orientation: Orientation) -> Self {
        self.options.orientation = orientation;
        self
    }

    /// Set the color order (RGB or BGR).
    #[must_use]
    pub fn color_order(mut self, color_order: ColorOrder) -> Self {
        self.options.color_order = color_order;
        self
    }

    /// Set the color inversion mode.
    #[must_use]
    pub fn invert_colors(mut self, invert_colors: ColorInversion) -> Self {
        self.options.invert_colors = invert_colors;
        self
    }

    /// Set the refresh order.
    #[must_use]
    pub fn refresh_order(mut self, refresh_order: RefreshOrder) -> Self {
        self.options.refresh_order = refresh_order;
        self
    }
}

impl<DI, MODEL> Builder<DI, MODEL, NoPin>
where
    DI: Interface,
    MODEL: Model,
{
    /// Initialize the display without a reset pin.
    ///
    /// # Arguments
    ///
    /// * `delay` - Delay provider
    /// * `framebuffer_a` - Raw pointer to first framebuffer
    /// * `framebuffer_b` - Raw pointer to second framebuffer
    ///
    /// # Safety
    ///
    /// The caller must ensure that the framebuffer pointers remain valid for the
    /// lifetime `'buf` and that no other references to the buffers exist while
    /// the display is in use.
    pub unsafe fn init<'buf, DELAY>(
        mut self,
        delay: &mut DELAY,
        framebuffer_a: *mut [u8],
        framebuffer_b: *mut [u8],
    ) -> Result<Display<'buf, DI, MODEL>, InitError<DI::Error>>
    where
        DELAY: DelayNs,
    {
        let fb_a = &mut *framebuffer_a;
        let fb_b = &mut *framebuffer_b;

        // Verify buffer sizes
        let (width, height) = self.options.display_size;
        let required_size = width as usize * height as usize * 2;
        
        assert!(
            fb_a.len() >= required_size,
            "framebuffer_a too small: need {} bytes, got {}",
            required_size,
            fb_a.len()
        );
        assert!(
            fb_b.len() >= required_size,
            "framebuffer_b too small: need {} bytes, got {}",
            required_size,
            fb_b.len()
        );

        // Initialize the model
        let _madctl = self.model.init(&mut self.di, delay, &self.options)?;

        Ok(Display::new(
            self.di,
            fb_a,
            fb_b,
            self.options,
            width,
            height,
        ))
    }
}

impl<DI, MODEL, RST> Builder<DI, MODEL, RST>
where
    DI: Interface,
    MODEL: Model,
    RST: OutputPin,
{
    /// Initialize the display with a reset pin.
    ///
    /// # Arguments
    ///
    /// * `delay` - Delay provider
    /// * `framebuffer_a` - Raw pointer to first framebuffer
    /// * `framebuffer_b` - Raw pointer to second framebuffer
    ///
    /// # Safety
    ///
    /// The caller must ensure that the framebuffer pointers remain valid for the
    /// lifetime `'buf` and that no other references to the buffers exist while
    /// the display is in use.
    pub unsafe fn init<'buf, DELAY>(
        mut self,
        delay: &mut DELAY,
        framebuffer_a: *mut [u8],
        framebuffer_b: *mut [u8],
    ) -> Result<Display<'buf, DI, MODEL>, InitError<DI::Error>>
    where
        DELAY: DelayNs,
    {
        let fb_a = &mut *framebuffer_a;
        let fb_b = &mut *framebuffer_b;

        // Verify buffer sizes
        let (width, height) = self.options.display_size;
        let required_size = width as usize * height as usize * 2;
        
        assert!(
            fb_a.len() >= required_size,
            "framebuffer_a too small: need {} bytes, got {}",
            required_size,
            fb_a.len()
        );
        assert!(
            fb_b.len() >= required_size,
            "framebuffer_b too small: need {} bytes, got {}",
            required_size,
            fb_b.len()
        );

        // Hardware reset
        if let Some(ref mut rst) = self.reset_pin {
            let _ = rst.set_high();
            delay.delay_us(10);
            let _ = rst.set_low();
            delay.delay_us(MODEL::RESET_DURATION_US);
            let _ = rst.set_high();
            delay.delay_us(MODEL::RESET_DELAY_US);
        }

        // Initialize the model
        let _madctl = self.model.init(&mut self.di, delay, &self.options)?;

        Ok(Display::new(
            self.di,
            fb_a,
            fb_b,
            self.options,
            width,
            height,
        ))
    }
}