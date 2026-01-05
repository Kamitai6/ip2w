//! # mipidsi-async
//!
//! Async framebuffer display driver for MIPI DCS displays.
//!
//! This crate provides a display driver that uses double-buffered framebuffers
//! with async DMA transfers, allowing the CPU to continue processing while
//! display data is being transferred.
//!
//! ## Features
//!
//! - **Double buffering**: Draw to one buffer while the other is being transferred
//! - **Non-blocking transfers**: CPU is free during DMA transfers
//! - **embedded-graphics support**: Implements `DrawTarget` trait
//! - **Multiple display models**: ST7789, GC9107, ILI9341
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────┐
//! │  Application                                            │
//! │    draw(&mut display)   → writes to framebuffer        │
//! │    display.flush_async() → starts DMA transfer         │
//! │    display.poll()       → checks transfer status       │
//! └─────────────────────────────────────────────────────────┘
//!                           │
//!                           ▼
//! ┌─────────────────────────────────────────────────────────┐
//! │  Display<I, M>                                         │
//! │    framebuffer_a: [u8]  ◄── draw while B transfers    │
//! │    framebuffer_b: [u8]  ◄── transfer while A draws    │
//! │    state: Idle | Busy                                  │
//! └─────────────────────────────────────────────────────────┘
//!                           │
//!                           ▼
//! ┌─────────────────────────────────────────────────────────┐
//! │  Interface (user-provided, e.g. esp-hal SPI+DMA)       │
//! │    send_command()    → blocking, short                 │
//! │    start_transfer()  → non-blocking DMA start          │
//! │    is_transfer_done() → polling                        │
//! │    finish_transfer() → cleanup                         │
//! └─────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Example
//!
//! ```ignore
//! use mipidsi_async::{Builder, models::ST7789};
//! use embedded_graphics::prelude::*;
//! use embedded_graphics::pixelcolor::Rgb565;
//!
//! // Create framebuffers (static for no_std)
//! static mut FB_A: [u8; 128 * 128 * 2] = [0; 128 * 128 * 2];
//! static mut FB_B: [u8; 128 * 128 * 2] = [0; 128 * 128 * 2];
//!
//! // Initialize display
//! let mut display = Builder::new(ST7789, interface)
//!     .display_size(128, 128)
//!     .init(&mut delay, unsafe { &mut FB_A }, unsafe { &mut FB_B })
//!     .unwrap();
//!
//! loop {
//!     display.poll();  // Check DMA completion (non-blocking)
//!
//!     // Handle control loop (runs every iteration)
//!     do_control_stuff();
//!
//!     // Update display when idle
//!     if display.is_idle() {
//!         display.clear(Rgb565::BLACK);
//!         // Draw with embedded-graphics
//!         Circle::new(Point::new(50, 50), 20)
//!             .into_styled(PrimitiveStyle::with_fill(Rgb565::RED))
//!             .draw(&mut display)
//!             .unwrap();
//!         display.flush_async().unwrap();
//!     }
//! }
//! ```

#![no_std]
#![warn(missing_docs)]

pub mod dcs;
pub mod interface;
pub mod models;
pub mod options;

mod builder;

pub use builder::{Builder, NoPin};
pub use interface::Interface;
pub use models::{InitError, Model};

use embedded_graphics_core::{
    draw_target::DrawTarget,
    geometry::{OriginDimensions, Size},
    pixelcolor::Rgb565,
    primitives::Rectangle,
    Pixel,
};

use models::Model as ModelTrait;
use options::{MemoryMapping, ModelOptions};

/// Transfer state of the display.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransferState {
    /// No transfer in progress, ready for drawing and flush.
    Idle,
    /// DMA transfer in progress.
    Busy,
}

/// Async framebuffer display driver.
///
/// This driver maintains two framebuffers for double-buffering:
/// - While one buffer is being transferred via DMA, the CPU can draw to the other
/// - `flush_async()` swaps buffers and starts transfer
/// - `poll()` checks for transfer completion
///
/// # Type Parameters
///
/// * `'buf` - Lifetime of the framebuffers
/// * `I` - Interface implementation (e.g., SPI + DMA)
/// * `M` - Display model (e.g., ST7789, GC9107)
pub struct Display<'buf, I, M>
where
    I: Interface,
    M: ModelTrait,
{
    interface: I,

    /// Framebuffer A
    framebuffer_a: &'buf mut [u8],
    /// Framebuffer B
    framebuffer_b: &'buf mut [u8],

    /// Which buffer is currently being drawn to (0 = A, 1 = B)
    draw_buffer: u8,

    /// Current transfer state
    state: TransferState,

    /// Display options
    options: ModelOptions,

    /// Display width in pixels
    width: u16,
    /// Display height in pixels
    height: u16,

    _model: core::marker::PhantomData<M>,
}

impl<'buf, I, M> Display<'buf, I, M>
where
    I: Interface,
    M: ModelTrait,
{
    /// Create a new display (called by Builder).
    pub(crate) fn new(
        interface: I,
        framebuffer_a: &'buf mut [u8],
        framebuffer_b: &'buf mut [u8],
        options: ModelOptions,
        width: u16,
        height: u16,
    ) -> Self {
        Self {
            interface,
            framebuffer_a,
            framebuffer_b,
            draw_buffer: 0,
            state: TransferState::Idle,
            options,
            width,
            height,
            _model: core::marker::PhantomData,
        }
    }

    /// Poll for DMA transfer completion (non-blocking).
    ///
    /// Call this regularly in your main loop. When a transfer completes,
    /// the display transitions from `Busy` to `Idle` state.
    ///
    /// This method handles chunked transfers automatically - for large
    /// framebuffers, it will advance to the next chunk when the current
    /// chunk completes.
    ///
    /// # Example
    ///
    /// ```ignore
    /// loop {
    ///     display.poll();  // Always call first
    ///
    ///     // Your control loop here (runs regardless of display state)
    ///     do_control();
    ///
    ///     // Draw only when idle
    ///     if display.is_idle() {
    ///         draw_stuff(&mut display);
    ///         display.flush_async();
    ///     }
    /// }
    /// ```
    pub fn poll(&mut self) {
        if self.state == TransferState::Busy {
            // poll_transfer handles chunked transfers and returns true when fully done
            if self.interface.poll_transfer() {
                self.state = TransferState::Idle;
            }
        }
    }

    /// Check if the display is idle (ready for flush).
    #[inline]
    pub fn is_idle(&self) -> bool {
        self.state == TransferState::Idle
    }

    /// Check if a transfer is in progress.
    #[inline]
    pub fn is_busy(&self) -> bool {
        self.state == TransferState::Busy
    }

    /// Get current transfer state.
    #[inline]
    pub fn state(&self) -> TransferState {
        self.state
    }

    /// Get mutable reference to the current draw buffer.
    fn current_buffer_mut(&mut self) -> &mut [u8] {
        if self.draw_buffer == 0 {
            self.framebuffer_a
        } else {
            self.framebuffer_b
        }
    }

    /// Clear the current draw buffer with a color.
    pub fn clear_buffer(&mut self, color: Rgb565) {
        let bytes = embedded_graphics_core::pixelcolor::raw::ToBytes::to_be_bytes(color);
        let buffer = self.current_buffer_mut();

        for chunk in buffer.chunks_exact_mut(2) {
            chunk[0] = bytes[0];
            chunk[1] = bytes[1];
        }
    }

    /// Set a pixel in the current draw buffer.
    #[inline]
    pub fn set_pixel(&mut self, x: u16, y: u16, color: Rgb565) {
        if x < self.width && y < self.height {
            let idx = (y as usize * self.width as usize + x as usize) * 2;
            let bytes = embedded_graphics_core::pixelcolor::raw::ToBytes::to_be_bytes(color);
            let buffer = self.current_buffer_mut();
            buffer[idx] = bytes[0];
            buffer[idx + 1] = bytes[1];
        }
    }

    /// Start async transfer of current draw buffer.
    ///
    /// Returns `Ok(true)` if transfer was started, `Ok(false)` if display is busy.
    /// After calling this, the draw buffer is swapped so you can continue
    /// drawing to the other buffer.
    ///
    /// # Example
    ///
    /// ```ignore
    /// if display.is_idle() {
    ///     display.clear(Rgb565::BLACK);
    ///     // Draw something
    ///     display.flush_async()?;
    /// }
    /// ```
    pub fn flush_async(&mut self) -> Result<bool, I::Error> {
        if self.state != TransferState::Idle {
            return Ok(false);
        }

        // Set address window (blocking, but very short)
        self.set_address_window(0, 0, self.width - 1, self.height - 1)?;

        // Send RAMWR command
        M::write_memory_start(&mut self.interface)?;

        // Calculate buffer size for the actual transfer
        let buffer_len = self.width as usize * self.height as usize * 2;
        
        // Remember which buffer we're transferring before swap
        let transfer_buffer = self.draw_buffer;
        
        // Swap to other buffer for next frame (do this before start_transfer to avoid borrow issues)
        self.draw_buffer = (self.draw_buffer + 1) % 2;

        // Get mutable reference to the buffer we've been drawing to and start transfer
        let buffer = if transfer_buffer == 0 {
            &mut self.framebuffer_a[..buffer_len]
        } else {
            &mut self.framebuffer_b[..buffer_len]
        };

        // Start DMA transfer
        self.interface.start_transfer(buffer)?;
        self.state = TransferState::Busy;

        Ok(true)
    }

    /// Blocking flush - waits for transfer to complete.
    ///
    /// This is useful during initialization or when you need to ensure
    /// the display is updated before continuing.
    pub fn flush(&mut self) -> Result<(), I::Error> {
        self.flush_async()?;
        while self.state == TransferState::Busy {
            self.poll();
        }
        Ok(())
    }

    /// Set the address window for the next transfer.
    fn set_address_window(&mut self, sx: u16, sy: u16, ex: u16, ey: u16) -> Result<(), I::Error> {
        // Calculate offset based on orientation
        let mut offset = self.options.display_offset;
        let mapping = MemoryMapping::from(self.options.orientation);
        
        if mapping.reverse_columns {
            offset.0 = M::FRAMEBUFFER_SIZE.0 - (self.options.display_size.0 + offset.0);
        }
        if mapping.reverse_rows {
            offset.1 = M::FRAMEBUFFER_SIZE.1 - (self.options.display_size.1 + offset.1);
        }
        if mapping.swap_rows_and_columns {
            offset = (offset.1, offset.0);
        }

        let (sx, sy, ex, ey) = (
            sx + offset.0,
            sy + offset.1,
            ex + offset.0,
            ey + offset.1,
        );

        M::update_address_window(
            &mut self.interface,
            self.options.orientation.rotation,
            sx,
            sy,
            ex,
            ey,
        )
    }

    /// Get display width in pixels.
    #[inline]
    pub fn width(&self) -> u16 {
        self.width
    }

    /// Get display height in pixels.
    #[inline]
    pub fn height(&self) -> u16 {
        self.height
    }

    /// Release the interface and buffers.
    ///
    /// This consumes the display and returns its components.
    /// Make sure no transfer is in progress before calling this.
    pub fn release(self) -> (I, &'buf mut [u8], &'buf mut [u8]) {
        (self.interface, self.framebuffer_a, self.framebuffer_b)
    }

    /// Get a reference to the interface.
    pub fn interface(&self) -> &I {
        &self.interface
    }

    /// Get a mutable reference to the interface.
    ///
    /// # Safety
    ///
    /// Be careful not to interfere with ongoing transfers.
    pub fn interface_mut(&mut self) -> &mut I {
        &mut self.interface
    }
}

// ============================================================================
// embedded-graphics support
// ============================================================================

impl<'buf, I, M> DrawTarget for Display<'buf, I, M>
where
    I: Interface,
    M: ModelTrait,
{
    type Color = Rgb565;
    type Error = I::Error;

    fn draw_iter<IT>(&mut self, pixels: IT) -> Result<(), Self::Error>
    where
        IT: IntoIterator<Item = Pixel<Self::Color>>,
    {
        let width = self.width;
        let height = self.height;
        let buffer = self.current_buffer_mut();

        for Pixel(coord, color) in pixels {
            if coord.x >= 0 && coord.y >= 0 {
                let x = coord.x as u16;
                let y = coord.y as u16;
                if x < width && y < height {
                    let idx = (y as usize * width as usize + x as usize) * 2;
                    let bytes =
                        embedded_graphics_core::pixelcolor::raw::ToBytes::to_be_bytes(color);
                    buffer[idx] = bytes[0];
                    buffer[idx + 1] = bytes[1];
                }
            }
        }
        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        let width = self.width as i32;
        let height = self.height as i32;
        let stride = self.width as usize;
        let buffer = self.current_buffer_mut();
        let bytes = embedded_graphics_core::pixelcolor::raw::ToBytes::to_be_bytes(color);

        let x_start = area.top_left.x.max(0) as usize;
        let y_start = area.top_left.y.max(0) as usize;
        let x_end = (area.top_left.x + area.size.width as i32).min(width) as usize;
        let y_end = (area.top_left.y + area.size.height as i32).min(height) as usize;

        for y in y_start..y_end {
            let row_start = (y * stride + x_start) * 2;
            let row_end = (y * stride + x_end) * 2;
            for chunk in buffer[row_start..row_end].chunks_exact_mut(2) {
                chunk[0] = bytes[0];
                chunk[1] = bytes[1];
            }
        }

        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        self.clear_buffer(color);
        Ok(())
    }
}

impl<'buf, I, M> OriginDimensions for Display<'buf, I, M>
where
    I: Interface,
    M: ModelTrait,
{
    fn size(&self) -> Size {
        Size::new(self.width as u32, self.height as u32)
    }
}