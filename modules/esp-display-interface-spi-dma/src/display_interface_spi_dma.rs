//! DMA SPI interface for display drivers

use core::cell::RefCell;
use core::ptr::addr_of_mut;
use defmt::info;

// use byte_slice_cast::AsByteSlice;
// use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};
use esp_hal::{
    dma::{DmaDescriptor, DmaTxBuf},
    gpio::Output,
    spi::master::SpiDmaTransfer,
};
use mipidsi::interface::Interface;

const DMA_BUFFER_SIZE: usize = 4096;
const DMA_CHUNK_SIZE: usize = 4092;

#[link_section = ".dram1.bss"]
static mut BUFFER1: [u32; DMA_BUFFER_SIZE / 4] = [0u32; DMA_BUFFER_SIZE / 4];
#[link_section = ".dram1.bss"]
static mut BUFFER2: [u32; DMA_BUFFER_SIZE / 4] = [0u32; DMA_BUFFER_SIZE / 4];

const DESCRIPTOR_COUNT: usize = (DMA_BUFFER_SIZE + DMA_CHUNK_SIZE - 1) / DMA_CHUNK_SIZE;

#[link_section = ".dram1.bss"]
static mut DESCRIPTORS: [DmaDescriptor; DESCRIPTOR_COUNT] = 
    [DmaDescriptor::EMPTY; DESCRIPTOR_COUNT];

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
        let buffer = DmaTxBuf::new(descriptors(), &mut send_buffer[..len]).unwrap();
        let transfer = self.spi.borrow_mut().take().unwrap().write(len, buffer).unwrap();
        let (reclaimed_spi, _) = transfer.wait();
        self.spi.replace(Some(reclaimed_spi));
    }

    /// ダブルバッファリングを使った転送（元の iter_transfer を基に）
    fn iter_transfer<const N: usize>(
        &self,
        iter: &mut impl Iterator<Item = [u8; N]>,
    ) {
        info!("iter_transfer start");
        let mut spi = Some(self.spi.borrow_mut().take().unwrap());
        let mut current_buffer = 0;
        let mut transfer: Option<SpiDmaTransfer<'d, esp_hal::Blocking, DmaTxBuf>> = None;
        let mut chunk_count = 0;

        loop {
            let buffer = if current_buffer == 0 {
                dma_buffer1()
            } else {
                dma_buffer2()
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

            // 前の転送を待つ（ダブルバッファリング）
            if let Some(t) = transfer.take() {
                if idx > 0 {
                    let (reclaimed_spi, _) = t.wait();
                    spi = Some(reclaimed_spi);
                } else {
                    // 最後の転送が実行中、後で回収するため保存
                    self.transfer.replace(Some(t));
                }
            }

            if idx > 0 {
                chunk_count += 1;
                info!("Chunk {}: {} bytes, data: {:02X} {:02X} {:02X} {:02X}", 
                        chunk_count, idx, buffer[0], buffer[1], buffer[2], buffer[3]);
                let mut dma_buffer = DmaTxBuf::new(descriptors(), &mut buffer[..idx]).unwrap();
                dma_buffer.set_length(idx);
                transfer = Some(spi.take().unwrap().write(idx, dma_buffer).unwrap());
                current_buffer = (current_buffer + 1) % 2;
            } else {
                break;
            }
        }

        // SPI を戻す（転送中でなければ）
        info!("iter_transfer done, {} chunks", chunk_count);
        if let Some(s) = spi {
            self.spi.replace(Some(s));
        }
    }
}

impl Interface for SPIInterface<'_> {
    type Word = u8;
    type Error = DmaError;

    fn send_command(&mut self, command: u8, args: &[u8]) -> Result<(), Self::Error> {
        info!("send_command: 0x{:02X}, args len: {}", command, args.len());

        self.wait_for_transfer();
        self.cs_low();

        // DC = Low でコマンド送信
        self.dc.set_low();
        let buffer = dma_buffer1();
        buffer[0] = command;
        self.single_transfer(buffer, 1);

        info!("command sent");

        // DC = High で引数送信
        if !args.is_empty() {
            self.dc.set_high();
            let buffer = dma_buffer1();
            buffer[..args.len()].copy_from_slice(args);
            self.single_transfer(buffer, args.len());
            info!("args sent");
        }

        self.cs_high();
        Ok(())
    }

    fn send_pixels<const N: usize>(
        &mut self,
        pixels: impl IntoIterator<Item = [Self::Word; N]>,
    ) -> Result<(), Self::Error> {
        info!("send_pixels N={}", N);

        self.wait_for_transfer();
        self.cs_low();
        self.dc.set_high();

        let mut iter = pixels.into_iter();
        self.iter_transfer::<N>(&mut iter);

        self.cs_high();
        info!("send_pixels done");
        Ok(())
    }

    fn send_repeated_pixel<const N: usize>(
        &mut self,
        pixel: [Self::Word; N],
        count: u32,
    ) -> Result<(), Self::Error> {
        info!("send_repeated_pixel N={}, count={}", N, count);

        self.wait_for_transfer();
        self.cs_low();
        self.dc.set_high();

        // repeat イテレータを使って iter_transfer を再利用
        let mut iter = core::iter::repeat(pixel).take(count as usize);
        self.iter_transfer::<N>(&mut iter);

        self.cs_high();
        info!("send_repeated_pixel done");
        Ok(())
    }
}

fn descriptors() -> &'static mut [DmaDescriptor] {
    unsafe { &mut *addr_of_mut!(DESCRIPTORS) }
}

fn dma_buffer1() -> &'static mut [u8] {
    unsafe {
        core::slice::from_raw_parts_mut(
            addr_of_mut!(BUFFER1) as *mut u8,
            DMA_BUFFER_SIZE,
        )
    }
}

fn dma_buffer2() -> &'static mut [u8] {
    unsafe {
        core::slice::from_raw_parts_mut(
            addr_of_mut!(BUFFER2) as *mut u8,
            DMA_BUFFER_SIZE,
        )
    }
}
