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
/// - `poll_transfer` advances chunked transfers (default impl provided)
///
/// The `send_command` method may block briefly as commands are typically
/// very short (a few bytes).
///
/// # Chunked Transfers
///
/// For large framebuffers that need to be split into multiple DMA transfers,
/// override `poll_transfer` to handle chunk advancement. The default
/// implementation assumes single-shot transfers.
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
    /// The buffer must remain valid and unchanged until the transfer is complete.
    ///
    /// For large buffers, implementations may split the transfer into chunks.
    ///
    /// # Arguments
    ///
    /// * `buffer` - The framebuffer data to transfer (mutable for DMA compatibility)
    fn start_transfer(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error>;

    /// Check if the current transfer (or chunk) is complete.
    ///
    /// This method must not block. Returns `true` if no transfer is in progress
    /// or if the current transfer/chunk has completed and needs attention.
    fn is_transfer_done(&self) -> bool;

    /// Finish the current transfer/chunk and reclaim resources.
    ///
    /// This should only be called after `is_transfer_done` returns `true`.
    /// For chunked transfers, this may start the next chunk.
    fn finish_transfer(&mut self);

    /// Check if the interface is completely idle (all transfers done).
    ///
    /// For single-shot transfers, this is the same as `is_transfer_done`.
    /// For chunked transfers, this returns `true` only when all chunks are complete.
    ///
    /// Default implementation returns `is_transfer_done()`.
    fn is_idle(&self) -> bool {
        self.is_transfer_done()
    }

    /// Poll and advance the transfer.
    ///
    /// This method should be called periodically during a transfer.
    /// It checks if the current transfer/chunk is done and advances to the next
    /// chunk if needed.
    ///
    /// Returns `true` if the entire transfer is complete (interface is idle).
    ///
    /// Default implementation handles single-shot transfers.
    /// Override for chunked transfer support.
    fn poll_transfer(&mut self) -> bool {
        if self.is_transfer_done() {
            self.finish_transfer();
            self.is_idle()
        } else {
            false
        }
    }
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