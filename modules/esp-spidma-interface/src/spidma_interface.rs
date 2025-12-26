//! DMA SPI interface for display drivers

use core::{
    cell::RefCell, 
    ptr::addr_of_mut, 
    slice::from_raw_parts_mut
};

use esp_hal::{
    dma::{DmaDescriptor, DmaTxBuf},
    gpio::Output,
    spi::master::SpiDmaTransfer,
};
use mipidsi::interface::Interface;

const DMA_BUFFER_SIZE: usize = 4096;
const DMA_CHUNK_SIZE: usize = 4092;

static mut BUFFER1: [u32; DMA_BUFFER_SIZE / 4] = [0u32; DMA_BUFFER_SIZE / 4];
static mut BUFFER2: [u32; DMA_BUFFER_SIZE / 4] = [0u32; DMA_BUFFER_SIZE / 4];

const DESCRIPTOR_COUNT: usize = (DMA_BUFFER_SIZE + DMA_CHUNK_SIZE - 1) / DMA_CHUNK_SIZE;

static mut DESCRIPTORS1: [DmaDescriptor; DESCRIPTOR_COUNT] = [DmaDescriptor::EMPTY; DESCRIPTOR_COUNT];
static mut DESCRIPTORS2: [DmaDescriptor; DESCRIPTOR_COUNT] = [DmaDescriptor::EMPTY; DESCRIPTOR_COUNT];

type SpiDma<'d> = esp_hal::spi::master::SpiDma<'d, esp_hal::Blocking>;

/// DMA SPI interface error
#[derive(Debug, Clone, Copy)]
pub struct DmaError;

/// SPI display interface with DMA support
pub struct SPIInterface<'d> {
    spi: RefCell<Option<SpiDma<'d>>>,
    transfer: RefCell<Option<SpiDmaTransfer<'d, esp_hal::Blocking, DmaTxBuf>>>,
    dc: Output<'d>,
    cs: Option<Output<'d>>,
}

impl<'d> SPIInterface<'d> {
    pub fn new(spi: SpiDma<'d>, dc: Output<'d>, cs: Output<'d>) -> Self {
        Self {
            spi: RefCell::new(Some(spi)),
            transfer: RefCell::new(None),
            dc,
            cs: Some(cs),
        }
    }

    pub fn new_no_cs(spi: SpiDma<'d>, dc: Output<'d>) -> Self {
        Self {
            spi: RefCell::new(Some(spi)),
            transfer: RefCell::new(None),
            dc,
            cs: None,
        }
    }

    fn wait_for_transfer(&self) {
        if let Some(transfer) = self.transfer.borrow_mut().take() {
            let (reclaimed_spi, _) = transfer.wait();
            self.spi.replace(Some(reclaimed_spi));
        }
    }

    fn cs_low(&mut self) {
        if let Some(cs) = self.cs.as_mut() {
            cs.set_low();
        }
    }

    fn cs_high(&mut self) {
        if let Some(cs) = self.cs.as_mut() {
            cs.set_high();
        }
    }

    fn single_transfer(&self, send_buffer: &'static mut [u8], len: usize) {
        let buffer = DmaTxBuf::new(descriptors1(), &mut send_buffer[..len]).unwrap();
        let transfer = self.spi.borrow_mut().take().unwrap().write(len, buffer).unwrap();
        let (reclaimed_spi, _) = transfer.wait();
        self.spi.replace(Some(reclaimed_spi));
    }

    // ダブルバッファリングを使った転送（元の iter_transfer を基に）
    fn iter_transfer<const N: usize>(
        &self,
        iter: &mut impl Iterator<Item = [u8; N]>,
    ) {
        let mut spi = Some(self.spi.borrow_mut().take().unwrap());
        let mut current_buffer = 0;
        let mut transfer: Option<SpiDmaTransfer<'d, esp_hal::Blocking, DmaTxBuf>> = None;

        loop {
            let (buffer, descs) = if current_buffer == 0 {
                (dma_buffer1(), descriptors1())
            } else {
                (dma_buffer2(), descriptors2())
            };

            // バッファにピクセルを詰める
            let mut idx = 0;
            loop {
                match iter.next() {
                    Some(pixel) => {
                        buffer[idx..idx + N].copy_from_slice(&pixel);
                        idx += N;
                    }
                    None => break,
                }

                if idx + N > DMA_BUFFER_SIZE {
                    break;
                }
            }

            // 前の転送を待つ（常に待つ）
            if let Some(t) = transfer.take() {
                let (reclaimed_spi, _) = t.wait();
                spi = Some(reclaimed_spi);
            }

            if idx > 0 {
                let mut dma_buffer = DmaTxBuf::new(descs, &mut buffer[..idx]).unwrap();
                dma_buffer.set_length(idx);
                transfer = Some(spi.take().unwrap().write(idx, dma_buffer).unwrap());
                current_buffer = (current_buffer + 1) % 2;
            } else {
                break;
            }
        }

        // 最後の転送を待つ
        if let Some(t) = transfer.take() {
            let (reclaimed_spi, _) = t.wait();
            spi = Some(reclaimed_spi);
        }
        self.spi.replace(spi);
    }
}

impl Interface for SPIInterface<'_> {
    type Word = u8;
    type Error = DmaError;

    fn send_command(&mut self, command: u8, args: &[u8]) -> Result<(), Self::Error> {
        self.wait_for_transfer();
        self.cs_low();

        self.dc.set_low();
        let buffer = dma_buffer1();
        buffer[0] = command;
        self.single_transfer(buffer, 1);

        if !args.is_empty() {
            self.dc.set_high();
            let buffer = dma_buffer1();
            buffer[..args.len()].copy_from_slice(args);
            self.single_transfer(buffer, args.len());
        }

        self.cs_high();
        Ok(())
    }

    fn send_pixels<const N: usize>(
        &mut self,
        pixels: impl IntoIterator<Item = [Self::Word; N]>,
    ) -> Result<(), Self::Error> {
        self.wait_for_transfer();
        self.cs_low();
        self.dc.set_high();

        let mut iter = pixels.into_iter();
        self.iter_transfer::<N>(&mut iter);

        self.cs_high();
        Ok(())
    }

    fn send_repeated_pixel<const N: usize>(
        &mut self,
        pixel: [Self::Word; N],
        count: u32,
    ) -> Result<(), Self::Error> {
        self.wait_for_transfer();
        self.cs_low();
        self.dc.set_high();

        // repeat イテレータを使って iter_transfer を再利用
        let mut iter = core::iter::repeat(pixel).take(count as usize);
        self.iter_transfer::<N>(&mut iter);

        self.cs_high();
        Ok(())
    }
}

fn descriptors1() -> &'static mut [DmaDescriptor] {
    unsafe { &mut *addr_of_mut!(DESCRIPTORS1) }
}

fn descriptors2() -> &'static mut [DmaDescriptor] {
    unsafe { &mut *addr_of_mut!(DESCRIPTORS2) }
}

fn dma_buffer1() -> &'static mut [u8] {
    unsafe {
        from_raw_parts_mut(addr_of_mut!(BUFFER1) as *mut u8, DMA_BUFFER_SIZE)
    }
}

fn dma_buffer2() -> &'static mut [u8] {
    unsafe {
        from_raw_parts_mut(addr_of_mut!(BUFFER2) as *mut u8, DMA_BUFFER_SIZE)
    }
}
