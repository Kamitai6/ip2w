//! ESP32 SPI DMA interface for mipidsi-async
//!
//! This crate provides an implementation of the `mipidsi_async::Interface` trait
//! using ESP32's SPI peripheral with DMA support.
//!
//! Large framebuffers (>32KB) are automatically split into chunks.
//!
//! ## Example
//!
//! ```ignore
//! use esp_display_interface::{DmaSpiInterface, dma_resources};
//! use esp_hal::{
//!     gpio::Output,
//!     spi::master::{Config, Spi},
//! };
//!
//! // Create SPI with DMA
//! let spi = Spi::new(peripherals.SPI2, Config::default())
//!     .with_sck(sclk)
//!     .with_mosi(mosi)
//!     .with_dma(dma_channel);
//!
//! // Create DC pin
//! let dc = Output::new(dc_pin, Level::Low);
//!
//! // Create DMA resources for 128x128 RGB565 display
//! dma_resources!(DISPLAY, 128, 128);
//!
//! // Create interface
//! let interface = unsafe {
//!     DmaSpiInterface::new(
//!         spi, 
//!         dc, 
//!         &raw mut DISPLAY_DESC,
//!         &raw mut DISPLAY_CMD_BUF,
//!     )
//! };
//! ```

#![no_std]

use esp_hal::{
    dma::{DmaDescriptor, DmaTxBuf},
    gpio::Output,
    spi::master::{SpiDma, SpiDmaTransfer},
    Blocking,
};

use mipidsi_async::Interface;

/// Re-export for macro use (hidden from docs)
#[doc(hidden)]
pub use paste::paste;

/// Maximum chunk size for a single DMA transfer.
/// ESP32 DMA: 4092 bytes per descriptor × 8 descriptors = 32736 bytes max.
pub const MAX_CHUNK_SIZE: usize = 4092 * 8;

/// Number of DMA descriptors needed for one chunk
pub const DESCRIPTORS_PER_CHUNK: usize = 8;

/// Error type for DMA SPI interface
#[derive(Debug, Clone, Copy)]
pub enum DmaSpiError {
    /// SPI error during transfer
    Spi,
    /// DMA buffer creation failed
    DmaBuffer,
    /// Interface is in invalid state
    InvalidState,
    /// Command too long for buffer
    CommandTooLong,
}

/// Transfer state
enum TransferState<'d> {
    /// Idle, SPI is available
    Idle(SpiDma<'d, Blocking>),
    /// Transfer in progress (possibly multi-chunk)
    Transferring {
        transfer: SpiDmaTransfer<'d, Blocking, DmaTxBuf>,
        /// Pointer to remaining data (after current chunk)
        remaining_ptr: *mut u8,
        /// Bytes remaining after current chunk
        remaining_len: usize,
    },
    /// Temporary invalid state during transitions
    Invalid,
}

/// SPI DMA interface for displays
///
/// This interface uses ESP32's SPI peripheral with DMA for efficient
/// framebuffer transfers. The DMA transfer is non-blocking, allowing
/// the CPU to perform other tasks while the transfer is in progress.
///
/// Large buffers (>[`MAX_CHUNK_SIZE`]) are automatically split into chunks.
pub struct DmaSpiInterface<'d> {
    /// SPI transfer state (Idle or Transferring)
    state: TransferState<'d>,
    /// Data/Command pin
    dc: Output<'d>,
    /// DMA descriptors (8 descriptors for chunk transfers)
    descriptors: &'d mut [DmaDescriptor],
    /// Buffer for command data
    cmd_buffer: &'d mut [u8],
}

impl<'d> DmaSpiInterface<'d> {
    /// Create a new DMA SPI interface
    ///
    /// # Arguments
    ///
    /// * `spi` - SPI peripheral with DMA configured
    /// * `dc` - Data/Command pin
    /// * `descriptors` - Raw pointer to DMA descriptors (8 descriptors recommended)
    /// * `cmd_buffer` - Raw pointer to command buffer
    ///
    /// # Safety
    ///
    /// The caller must ensure that the pointers remain valid for the lifetime `'d`
    /// and that no other references to the data exist while this interface is in use.
    pub unsafe fn new(
        spi: SpiDma<'d, Blocking>,
        dc: Output<'d>,
        descriptors: *mut [DmaDescriptor],
        cmd_buffer: *mut [u8],
    ) -> Self {
        Self {
            state: TransferState::Idle(spi),
            dc,
            descriptors: &mut *descriptors,
            cmd_buffer: &mut *cmd_buffer,
        }
    }

    /// Get access to the DC pin
    pub fn dc(&mut self) -> &mut Output<'d> {
        &mut self.dc
    }

    /// Check if the interface is idle (not transferring)
    pub fn is_idle(&self) -> bool {
        matches!(self.state, TransferState::Idle(_))
    }

    /// Release the interface, returning the SPI peripheral and DC pin
    ///
    /// # Panics
    ///
    /// Panics if a transfer is in progress.
    pub fn release(self) -> (SpiDma<'d, Blocking>, Output<'d>) {
        match self.state {
            TransferState::Idle(spi) => (spi, self.dc),
            TransferState::Transferring { .. } => {
                panic!("Cannot release interface while transfer is in progress")
            }
            TransferState::Invalid => panic!("Interface in invalid state"),
        }
    }

    /// Reset all DMA descriptors
    fn reset_descriptors(&mut self) {
        for desc in self.descriptors.iter_mut() {
            *desc = DmaDescriptor::EMPTY;
        }
    }

    /// Start DMA transfer for a single chunk
    fn start_chunk(
        &mut self,
        spi: SpiDma<'d, Blocking>,
        chunk_ptr: *mut u8,
        chunk_len: usize,
        remaining_ptr: *mut u8,
        remaining_len: usize,
    ) -> Result<(), DmaSpiError> {
        self.reset_descriptors();

        // SAFETY: Caller guarantees buffer validity
        let buffer_static: &'static mut [u8] = unsafe {
            core::slice::from_raw_parts_mut(chunk_ptr, chunk_len)
        };
        let descriptors_static: &'static mut [DmaDescriptor] = unsafe {
            core::slice::from_raw_parts_mut(
                self.descriptors.as_mut_ptr(),
                self.descriptors.len(),
            )
        };

        let dma_buf = DmaTxBuf::new(descriptors_static, buffer_static)
            .map_err(|_| DmaSpiError::DmaBuffer)?;

        let transfer = spi
            .write(chunk_len, dma_buf)
            .map_err(|_| DmaSpiError::Spi)?;

        self.state = TransferState::Transferring {
            transfer,
            remaining_ptr,
            remaining_len,
        };

        Ok(())
    }

    /// Blocking write for command data
    fn blocking_write(&mut self, data: &[u8]) -> Result<(), DmaSpiError> {
        if data.len() > self.cmd_buffer.len() {
            return Err(DmaSpiError::CommandTooLong);
        }

        self.cmd_buffer[..data.len()].copy_from_slice(data);

        let spi = match core::mem::replace(&mut self.state, TransferState::Invalid) {
            TransferState::Idle(spi) => spi,
            TransferState::Transferring { transfer, remaining_ptr, remaining_len } => {
                self.state = TransferState::Transferring { transfer, remaining_ptr, remaining_len };
                return Err(DmaSpiError::InvalidState);
            }
            TransferState::Invalid => return Err(DmaSpiError::InvalidState),
        };

        self.reset_descriptors();

        let desc_static: &'static mut [DmaDescriptor] = unsafe {
            core::slice::from_raw_parts_mut(
                self.descriptors.as_mut_ptr(),
                self.descriptors.len(),
            )
        };
        let buf_static: &'static mut [u8] = unsafe {
            core::slice::from_raw_parts_mut(
                self.cmd_buffer.as_mut_ptr(),
                data.len(),
            )
        };

        let dma_buf = DmaTxBuf::new(desc_static, buf_static)
            .map_err(|_| DmaSpiError::DmaBuffer)?;

        let transfer = spi
            .write(data.len(), dma_buf)
            .map_err(|_| DmaSpiError::Spi)?;

        let (spi, _) = transfer.wait();
        self.state = TransferState::Idle(spi);

        Ok(())
    }
}

impl<'d> Interface for DmaSpiInterface<'d> {
    type Error = DmaSpiError;

    fn send_command(&mut self, command: u8, args: &[u8]) -> Result<(), Self::Error> {
        self.dc.set_low();
        self.blocking_write(&[command])?;

        if !args.is_empty() {
            self.dc.set_high();
            self.blocking_write(args)?;
        }

        Ok(())
    }

    fn start_transfer(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.dc.set_high();

        let spi = match core::mem::replace(&mut self.state, TransferState::Invalid) {
            TransferState::Idle(spi) => spi,
            TransferState::Transferring { transfer, remaining_ptr, remaining_len } => {
                self.state = TransferState::Transferring { transfer, remaining_ptr, remaining_len };
                return Err(DmaSpiError::InvalidState);
            }
            TransferState::Invalid => return Err(DmaSpiError::InvalidState),
        };

        let total_len = buffer.len();
        let chunk_len = total_len.min(MAX_CHUNK_SIZE);
        let remaining_len = total_len - chunk_len;

        let buffer_ptr = buffer.as_mut_ptr();
        let remaining_ptr = if remaining_len > 0 {
            unsafe { buffer_ptr.add(chunk_len) }
        } else {
            core::ptr::null_mut()
        };

        self.start_chunk(spi, buffer_ptr, chunk_len, remaining_ptr, remaining_len)
    }

    fn is_transfer_done(&self) -> bool {
        match &self.state {
            TransferState::Idle(_) => true,
            TransferState::Transferring { transfer, .. } => {
                // Current chunk is done (may have more chunks pending)
                transfer.is_done()
            }
            TransferState::Invalid => true,
        }
    }

    fn finish_transfer(&mut self) {
        let (transfer, remaining_ptr, remaining_len) = 
            match core::mem::replace(&mut self.state, TransferState::Invalid) {
                TransferState::Transferring { transfer, remaining_ptr, remaining_len } => {
                    (transfer, remaining_ptr, remaining_len)
                }
                TransferState::Idle(spi) => {
                    self.state = TransferState::Idle(spi);
                    return;
                }
                TransferState::Invalid => return,
            };

        // Wait for current chunk
        let (spi, _) = transfer.wait();

        // Start next chunk if there's more data
        if remaining_len > 0 && !remaining_ptr.is_null() {
            let chunk_len = remaining_len.min(MAX_CHUNK_SIZE);
            let new_remaining_len = remaining_len - chunk_len;
            let new_remaining_ptr = if new_remaining_len > 0 {
                unsafe { remaining_ptr.add(chunk_len) }
            } else {
                core::ptr::null_mut()
            };

            // Start next chunk (ignore error, can't propagate from finish_transfer)
            let _ = self.start_chunk(spi, remaining_ptr, chunk_len, new_remaining_ptr, new_remaining_len);
        } else {
            self.state = TransferState::Idle(spi);
        }
    }

    fn is_idle(&self) -> bool {
        matches!(self.state, TransferState::Idle(_))
    }

    fn poll_transfer(&mut self) -> bool {
        match &self.state {
            TransferState::Idle(_) => return true,
            TransferState::Transferring { transfer, remaining_len, .. } => {
                if !transfer.is_done() {
                    // Current chunk still in progress
                    return false;
                }
                // Current chunk done - check if this was the last one
                if *remaining_len == 0 {
                    // Last chunk done, finish and return true
                    self.finish_transfer();
                    return true;
                }
                // More chunks to go, finish current and start next
                self.finish_transfer();
                false
            }
            TransferState::Invalid => true,
        }
    }
}

/// Calculate DMA descriptors needed for buffer size (legacy, for reference)
pub const fn descriptors_needed(buffer_size: usize) -> usize {
    const DMA_CHUNK_SIZE: usize = 4092;
    (buffer_size + DMA_CHUNK_SIZE - 1) / DMA_CHUNK_SIZE
}

/// Macro to create all static DMA resources for a display
///
/// Creates DMA descriptors (8 for chunked transfer), command buffer, and framebuffers.
///
/// # Generated Items
///
/// - `{PREFIX}_DESC`: DMA descriptors (8 descriptors)
/// - `{PREFIX}_CMD_BUF`: Command buffer (64 bytes)
/// - `{PREFIX}_FB_A`: Framebuffer A
/// - `{PREFIX}_FB_B`: Framebuffer B
///
/// # Example
///
/// ```ignore
/// dma_resources!(DISPLAY, 128, 128);
///
/// let interface = unsafe {
///     DmaSpiInterface::new(
///         spi, dc,
///         &raw mut DISPLAY_DESC,
///         &raw mut DISPLAY_CMD_BUF,
///     )
/// };
/// ```
#[macro_export]
macro_rules! dma_resources {
    ($prefix:ident, $width:expr, $height:expr) => {
        $crate::paste! {
            /// DMA descriptors (8 for chunked transfer)
            // #[repr(C, align(4))]
            static mut [<$prefix _DESC>]: [esp_hal::dma::DmaDescriptor; 8] =
                [esp_hal::dma::DmaDescriptor::EMPTY; 8];
            
            /// Command buffer
            static mut [<$prefix _CMD_BUF>]: [u8; 64] = [0u8; 64];
            
            /// Framebuffer A
            static mut [<$prefix _FB_A>]: [u8; $width * $height * 2] = [0u8; $width * $height * 2];
            
            /// Framebuffer B
            static mut [<$prefix _FB_B>]: [u8; $width * $height * 2] = [0u8; $width * $height * 2];
        }
    };
}