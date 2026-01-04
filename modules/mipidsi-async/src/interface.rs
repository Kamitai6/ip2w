//! Display interface traits for async framebuffer transfer.
//!
//! This module defines the [`Interface`] trait that must be implemented
//! for each hardware interface (SPI with DMA, parallel, etc.).

/// Async display interface trait.
///
/// This trait defines the interface for communicating with display controllers
/// using framebuffer-based DMA transfers.
///
/// # Implementation Notes
///
/// Implementors should provide non-blocking transfer operations:
///
/// - `start_transfer` initiates a DMA transfer and returns immediately
/// - `is_transfer_done` polls for completion without blocking
/// - `finish_transfer` cleans up after transfer completes
///
/// The `send_command` method may block briefly as commands are typically
/// very short (a few bytes).
///
/// # Example Implementation
///
/// ```ignore
/// impl Interface for MySpiDma {
///     type Error = MyError;
///
///     fn send_command(&mut self, command: u8, args: &[u8]) -> Result<(), Self::Error> {
///         // Set DC pin low for command
///         self.dc.set_low();
///         self.spi.blocking_write(&[command])?;
///         
///         // Set DC pin high for data
///         if !args.is_empty() {
///             self.dc.set_high();
///             self.spi.blocking_write(args)?;
///         }
///         Ok(())
///     }
///
///     fn start_transfer(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
///         self.dc.set_high();
///         self.dma.start_write(buffer)?;
///         Ok(())
///     }
///
///     fn is_transfer_done(&self) -> bool {
///         self.dma.is_done()
///     }
///
///     fn finish_transfer(&mut self) {
///         self.dma.wait_and_reclaim();
///     }
/// }
/// ```
pub trait Interface {
    /// Error type for interface operations.
    type Error: core::fmt::Debug;

    /// Send a command with optional parameters.
    ///
    /// This method may block briefly. Commands are typically very short
    /// (1 byte command + a few bytes of parameters).
    ///
    /// # Arguments
    ///
    /// * `command` - The command byte
    /// * `args` - Command parameters (may be empty)
    fn send_command(&mut self, command: u8, args: &[u8]) -> Result<(), Self::Error>;

    /// Start an async transfer of framebuffer data.
    ///
    /// This method should return immediately after initiating the DMA transfer.
    /// The buffer must remain valid and unchanged until `finish_transfer` is called.
    ///
    /// # Arguments
    ///
    /// * `buffer` - The framebuffer data to transfer (mutable for DMA compatibility)
    fn start_transfer(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error>;

    /// Check if the current transfer is complete.
    ///
    /// This method must not block. Returns `true` if no transfer is in progress
    /// or if the current transfer has completed.
    fn is_transfer_done(&self) -> bool;

    /// Finish the transfer and reclaim resources.
    ///
    /// This should only be called after `is_transfer_done` returns `true`.
    /// After calling this method, a new transfer can be started.
    fn finish_transfer(&mut self);
}

/// Calculate the required buffer size for a display.
///
/// # Arguments
///
/// * `width` - Display width in pixels
/// * `height` - Display height in pixels
/// * `bytes_per_pixel` - Bytes per pixel (2 for RGB565, 3 for RGB666)
///
/// # Example
///
/// ```
/// use mipidsi_async::interface::buffer_size;
///
/// // 128x128 RGB565 display
/// let size = buffer_size(128, 128, 2);
/// assert_eq!(size, 32768);
/// ```
pub const fn buffer_size(width: usize, height: usize, bytes_per_pixel: usize) -> usize {
    width * height * bytes_per_pixel
}

/// Calculate buffer size for RGB565 format.
pub const fn buffer_size_rgb565(width: usize, height: usize) -> usize {
    buffer_size(width, height, 2)
}