//! ESP32 SPI DMA interface for mipidsi-async
//!
//! This crate provides an implementation of the `mipidsi_async::Interface` trait
//! using ESP32's SPI peripheral with DMA support.
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
//! let interface = DmaSpiInterface::new(
//!     spi, 
//!     dc, 
//!     unsafe { &mut DISPLAY_DESC },
//!     unsafe { &mut DISPLAY_CMD_BUF },
//! );
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
    /// Transfer in progress
    Transferring(SpiDmaTransfer<'d, Blocking, DmaTxBuf>),
    /// Temporary invalid state during transitions
    Invalid,
}

/// SPI DMA interface for displays
///
/// This interface uses ESP32's SPI peripheral with DMA for efficient
/// framebuffer transfers. The DMA transfer is non-blocking, allowing
/// the CPU to perform other tasks while the transfer is in progress.
pub struct DmaSpiInterface<'d> {
    /// SPI transfer state (Idle or Transferring)
    state: TransferState<'d>,
    /// Data/Command pin
    dc: Output<'d>,
    /// DMA descriptors (shared between commands and framebuffer)
    descriptors: &'d mut [DmaDescriptor],
    /// Buffer for command data (commands are copied here before DMA)
    cmd_buffer: &'d mut [u8],
}

impl<'d> DmaSpiInterface<'d> {
    /// Create a new DMA SPI interface
    ///
    /// # Arguments
    ///
    /// * `spi` - SPI peripheral with DMA configured
    /// * `dc` - Data/Command pin
    /// * `descriptors` - Raw pointer to DMA descriptors
    /// * `cmd_buffer` - Raw pointer to command buffer
    ///
    /// # Safety
    ///
    /// The caller must ensure that the pointers remain valid for the lifetime `'d`
    /// and that no other references to the data exist while this interface is in use.
    ///
    /// # Example
    ///
    /// ```ignore
    /// // For 128x128 RGB565 display
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
    /// Panics if a transfer is in progress. Call `finish_transfer()` first.
    pub fn release(self) -> (SpiDma<'d, Blocking>, Output<'d>) {
        match self.state {
            TransferState::Idle(spi) => (spi, self.dc),
            TransferState::Transferring(_) => {
                panic!("Cannot release interface while transfer is in progress")
            }
            TransferState::Invalid => panic!("Interface in invalid state"),
        }
    }

    /// Blocking write using DMA (for short command data)
    fn blocking_write(&mut self, data: &[u8]) -> Result<(), DmaSpiError> {
        if data.len() > self.cmd_buffer.len() {
            return Err(DmaSpiError::CommandTooLong);
        }

        // Copy data to command buffer
        self.cmd_buffer[..data.len()].copy_from_slice(data);

        // Take SPI from state
        let spi = match core::mem::replace(&mut self.state, TransferState::Invalid) {
            TransferState::Idle(spi) => spi,
            TransferState::Transferring(t) => {
                self.state = TransferState::Transferring(t);
                return Err(DmaSpiError::InvalidState);
            }
            TransferState::Invalid => return Err(DmaSpiError::InvalidState),
        };

        // Reset descriptors
        for desc in self.descriptors.iter_mut() {
            *desc = DmaDescriptor::EMPTY;
        }

        // Create static references (safe because we own them for 'd)
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

        // Create DMA buffer
        let dma_buf = DmaTxBuf::new(desc_static, buf_static)
            .map_err(|_| DmaSpiError::DmaBuffer)?;

        // Start transfer and wait (blocking)
        let transfer = spi
            .write(data.len(), dma_buf)
            .map_err(|_| DmaSpiError::Spi)?;

        // Wait for completion
        let (spi, _buf) = transfer.wait();

        // Restore state
        self.state = TransferState::Idle(spi);

        Ok(())
    }
}

impl<'d> Interface for DmaSpiInterface<'d> {
    type Error = DmaSpiError;

    fn send_command(&mut self, command: u8, args: &[u8]) -> Result<(), Self::Error> {
        // DC low for command
        self.dc.set_low();

        // Send command byte
        self.blocking_write(&[command])?;

        // DC high for data
        if !args.is_empty() {
            self.dc.set_high();
            self.blocking_write(args)?;
        }

        Ok(())
    }

    fn start_transfer(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error> {
        // DC high for pixel data
        self.dc.set_high();

        // Take ownership of SPI from state
        let spi = match core::mem::replace(&mut self.state, TransferState::Invalid) {
            TransferState::Idle(spi) => spi,
            TransferState::Transferring(transfer) => {
                self.state = TransferState::Transferring(transfer);
                return Err(DmaSpiError::InvalidState);
            }
            TransferState::Invalid => return Err(DmaSpiError::InvalidState),
        };

        // Reset descriptors for reuse
        for desc in self.descriptors.iter_mut() {
            *desc = DmaDescriptor::EMPTY;
        }

        // Create static references
        // SAFETY: The buffer is owned by Display and remains valid until finish_transfer()
        // The descriptors are borrowed for 'd lifetime
        let buffer_static: &'static mut [u8] = unsafe {
            core::slice::from_raw_parts_mut(buffer.as_mut_ptr(), buffer.len())
        };
        let descriptors_static: &'static mut [DmaDescriptor] = unsafe {
            core::slice::from_raw_parts_mut(
                self.descriptors.as_mut_ptr(),
                self.descriptors.len(),
            )
        };

        let dma_buf = DmaTxBuf::new(descriptors_static, buffer_static)
            .map_err(|_| DmaSpiError::DmaBuffer)?;

        // Start DMA transfer
        let transfer = spi
            .write(buffer.len(), dma_buf)
            .map_err(|_| DmaSpiError::Spi)?;

        self.state = TransferState::Transferring(transfer);

        Ok(())
    }

    fn is_transfer_done(&self) -> bool {
        match &self.state {
            TransferState::Idle(_) => true,
            TransferState::Transferring(transfer) => transfer.is_done(),
            TransferState::Invalid => true,
        }
    }

    fn finish_transfer(&mut self) {
        let transfer = match core::mem::replace(&mut self.state, TransferState::Invalid) {
            TransferState::Transferring(transfer) => transfer,
            TransferState::Idle(spi) => {
                self.state = TransferState::Idle(spi);
                return;
            }
            TransferState::Invalid => return,
        };

        let (spi, _dma_buf) = transfer.wait();
        self.state = TransferState::Idle(spi);
    }
}

/// Calculate the number of DMA descriptors needed for a given buffer size
///
/// ESP32 DMA descriptors can handle up to 4092 bytes each.
///
/// # Example
///
/// ```
/// use esp_display_interface::descriptors_needed;
///
/// // 128x128 RGB565
/// assert_eq!(descriptors_needed(128 * 128 * 2), 8);
///
/// // 240x240 RGB565
/// assert_eq!(descriptors_needed(240 * 240 * 2), 29);
/// ```
pub const fn descriptors_needed(buffer_size: usize) -> usize {
    const DMA_CHUNK_SIZE: usize = 4092;
    (buffer_size + DMA_CHUNK_SIZE - 1) / DMA_CHUNK_SIZE
}

/// Macro to create all static DMA resources for a display
///
/// Creates DMA descriptors, command buffer, and double framebuffers.
///
/// # Generated Items
///
/// - `{PREFIX}_DESC`: DMA descriptors array
/// - `{PREFIX}_CMD_BUF`: Command buffer (64 bytes)
/// - `{PREFIX}_FB_A`: Framebuffer A
/// - `{PREFIX}_FB_B`: Framebuffer B
///
/// # Example
///
/// ```ignore
/// use esp_display_interface::dma_resources;
/// use mipidsi_async::{Builder, models::ST7789};
///
/// // Create all resources for 128x128 RGB565 display
/// dma_resources!(DISPLAY, 128, 128);
///
/// let interface = DmaSpiInterface::new(
///     spi, dc,
///     unsafe { &mut DISPLAY_DESC },
///     unsafe { &mut DISPLAY_CMD_BUF },
/// );
///
/// let display = Builder::new(ST7789, interface)
///     .display_size(128, 128)
///     .init(&mut delay, unsafe { &mut DISPLAY_FB_A }, unsafe { &mut DISPLAY_FB_B })
///     .unwrap();
/// ```
#[macro_export]
macro_rules! dma_resources {
    ($prefix:ident, $width:expr, $height:expr) => {
        $crate::paste! {
            const FB_SIZE: usize = $width * $height * 2;
            /// DMA descriptors
            static mut [<$prefix _DESC>]: [esp_hal::dma::DmaDescriptor;
                $crate::descriptors_needed(FB_SIZE)] =
                [esp_hal::dma::DmaDescriptor::EMPTY;
                    $crate::descriptors_needed(FB_SIZE)];
            
            /// Command buffer
            static mut [<$prefix _CMD_BUF>]: [u8; 64] = [0u8; 64];
            
            /// Framebuffer A
            static mut [<$prefix _FB_A>]: [u8; FB_SIZE] = [0u8; FB_SIZE];
            
            /// Framebuffer B
            static mut [<$prefix _FB_B>]: [u8; FB_SIZE] = [0u8; FB_SIZE];
        }
    };
}