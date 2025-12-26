#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::cell::RefCell;
use alloc::borrow::ToOwned;
use critical_section::Mutex;
use defmt::info;
use {esp_backtrace as _, esp_println as _};

use esp_hal::{
    clock::CpuClock,
    dma::{DmaPriority, DmaRxBuf, DmaTxBuf, ExternalBurstConfig, DmaChannel, RegisterAccess},
    peripherals::Peripherals,
    gpio::{Event, Input, InputConfig, Io, Level, Output, OutputConfig, Pull},
    spi::{
        master::{Config as SpiCfg, Spi},  
        Mode,
    },
    delay::Delay,
    time::{Duration, Instant, Rate},
    timer::timg::TimerGroup,
    i2c::master::{I2c, Config as I2cCfg},
    main,
    handler,
    ram,
};
use esp_display_interface_spi_dma::display_interface_spi_dma;
use esp_bsp::{lcd_spi, lcd_backlight_init, lcd_display_interface, lcd_display, i2c_init, BoardType, DisplayConfig};
use embedded_graphics::{
    prelude::{IntoStorage, RgbColor, Point, DrawTarget},
    pixelcolor::Rgb565,
    mono_font::{
        ascii::FONT_10X20,
        MonoTextStyleBuilder,
    },
    text::{Alignment, Text},
    Drawable,
};
use embedded_hal::i2c::{I2c as eh_I2c};
use embedded_hal_bus::{
    i2c::{RefCellDevice as I2cRefCellDevice},
    spi::ExclusiveDevice,
};

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

static BUTTON: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));
pub struct Sensor<I2C> {
    i2c: I2C,
    addr: u8,
}
impl<I2C: eh_I2c> Sensor<I2C> {
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self { i2c, addr }
    }
    pub fn write(&mut self, data: &[u8]) -> Result<(), I2C::Error> {
        self.i2c.write(self.addr, data)?;
        Ok(())
    }
    pub fn read_reg(&mut self, reg: u8) -> Result<u8, I2C::Error> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(self.addr, &[reg], &mut buf)?;
        Ok(buf[0])
    }
}
type Lp5562Sensor<I2C> = Sensor<I2C>;

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[main]
fn main() -> ! {
    // generator version: 1.1.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    // esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    let mut delay = Delay::new();
    let timg0 = TimerGroup::new(peripherals.TIMG0);

    // let mut io = Io::new(peripherals.IO_MUX);
    // io.set_interrupt_handler(handler);
    // let mut led = Output::new(peripherals.GPIO2, Level::Low, OutputConfig::default());
    // let config = InputConfig::default().with_pull(Pull::Up);
    // let mut button = Input::new(peripherals.GPIO41, config);

    //IR G47

    let i2c = i2c_init!(peripherals);
    let i2c_ref_cell = RefCell::new(i2c);

    // fn scan_i2c<I2C: eh_I2c>(i2c: &mut I2C) {
    //     info!("Scanning I2C bus...");
    //     for addr in 0x08..0x78 {
    //         // 空のwriteでACKを確認
    //         if i2c.write(addr, &[]).is_ok() {
    //             info!("Found device at 0x{:02X}", addr);
    //         }
    //     }
    //     info!("Scan complete");
    // }
    // let mut i2c_scan = I2cRefCellDevice::new(&i2c_ref_cell);
    // scan_i2c(&mut i2c_scan);

    // LP5562 
    let mut lp5562 = Lp5562Sensor::new(
        I2cRefCellDevice::new(&i2c_ref_cell),
        0x30,
    );
    let _ = lp5562.write(&[0x0D, 0xFF]);
    info!("Reset OK");
    let _ = lp5562.write(&[0x0F, 0xFF]); // White
    info!("Current OK");
    let _ = lp5562.write(&[0x0E, 0xFF]); // White
    info!("PWM OK");
    let _ = lp5562.write(&[0x00, 0x40]);
    info!("Chip EN");
    delay.delay_millis(500);
    let _ = lp5562.write(&[0x08, 0x01]);
    info!("Config OK");
    let _ = lp5562.write(&[0x70, 0x00]);
    info!("LED_MAP OK");

    // info!("Reset: 0x{:02X}", lp5562.read_reg(0x0D).unwrap_or(0));
    // info!("Current: 0x{:02X}", lp5562.read_reg(0x0F).unwrap_or(0));
    // info!("PWM: 0x{:02X}", lp5562.read_reg(0x0E).unwrap_or(0));
    // info!("Chip: 0x{:02X}", lp5562.read_reg(0x00).unwrap_or(0));
    // info!("Config: 0x{:02X}", lp5562.read_reg(0x08).unwrap_or(0));
    // info!("LED_MAP: 0x{:02X}", lp5562.read_reg(0x70).unwrap_or(0));

    // Initialize IMU
    // let mut imu = Mpu6886::new(i2c_bus.acquire_i2c());
    // imu.init(&mut delay::FreeRtos).unwrap();

    // // Initialize Atom Motion motor driver
    // let mut motion = atom_motion::AtomMotion::new(i2c_bus.acquire_i2c());

    // let imu_transform: Matrix3<f32> = Matrix3::from_diagonal(&Vector3::new(-1.0, 1.0, -1.0));


    // esp32では、RX/TX別々にDMAを割り当てるのではなく、それぞれ設定できるだけで同じチャンネルとして扱うらしい
    // let dma_channel = peripherals.DMA_CH0;
    // let (rx, tx) = dma_channel.split();
    // rx.set_priority(DmaPriority::Priority0);
    // tx.set_priority(DmaPriority::Priority0);

    // let lcd_spi = lcd_spi!(peripherals);
    // let di = lcd_display_interface!(peripherals, lcd_spi);
    // let mut display = lcd_display!(peripherals, di, &mut delay).unwrap();

    // DMAなしのSPI
    let spi = Spi::new(
        peripherals.SPI2,
        SpiCfg::default()
            .with_frequency(Rate::from_mhz(40))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(peripherals.GPIO15)
    .with_mosi(peripherals.GPIO21);

    let cs = Output::new(peripherals.GPIO14, Level::High, OutputConfig::default());
    let dc = Output::new(peripherals.GPIO42, Level::Low, OutputConfig::default());
    let mut rst = Output::new(peripherals.GPIO48, Level::High, OutputConfig::default());

    // SpiBusをSpiDeviceに変換
    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

    const DMA_BUFFER_SIZE: usize = 4096;
    static mut BUFFER1: [u32; DMA_BUFFER_SIZE / 4] = [0u32; DMA_BUFFER_SIZE / 4];
    let buffer = unsafe {
        core::slice::from_raw_parts_mut(
            core::ptr::addr_of_mut!(BUFFER1) as *mut u8,
            DMA_BUFFER_SIZE,
        )
    };

    let di = mipidsi::interface::SpiInterface::new(spi_device, dc, buffer);

    let mut display = mipidsi::Builder::new(mipidsi::models::GC9107, di)
    .reset_pin(rst)
    .display_size(128, 128) //GC9107 default (128, 160)
    .display_offset(0, 32) // 160 - 128 = 32
    .color_order(mipidsi::options::ColorOrder::Bgr)
    // .invert_colors(mipidsi::options::ColorInversion::Inverted)
    .init(&mut delay)
    .unwrap();

    display.clear(Rgb565::RED).unwrap();
    delay.delay_millis(1000);
    display.clear(Rgb565::GREEN).unwrap();
    delay.delay_millis(1000);
    display.clear(Rgb565::BLUE).unwrap();
    delay.delay_millis(1000);
    display.clear(Rgb565::WHITE).unwrap();

    // let _ = Text::with_alignment("HELLO WORLD!", Point::new(160, 120), MonoTextStyleBuilder::new().font(&FONT_10X20).text_color(RgbColor::BLACK).build(),  Alignment::Center)
    //     .draw(&mut display);

    
    // esp_rtos::start(timg0.timer0);
    // let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    // let (mut _wifi_controller, _interfaces) =
    //     esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
    //         .expect("Failed to initialize Wi-Fi controller");

    // critical_section::with(|cs| {
    //     button.listen(Event::FallingEdge);
    //     BUTTON.borrow_ref_mut(cs).replace(button)
    // });

    loop {
        info!("Hello world!");

        // motion.set_motor(atom_motion::MotorChannel::M1, -pid.control).unwrap();
        // motion.set_motor(atom_motion::MotorChannel::M2, pid.control).unwrap();
        
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(500) {}
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v~1.0/examples
}

// #[handler]
// #[ram]
// fn handler() {
//     esp_println::println!(
//         "GPIO Interrupt with priority {}",
//         esp_hal::xtensa_lx::interrupt::get_level()
//     );

//     if critical_section::with(|cs| {
//         BUTTON
//             .borrow_ref_mut(cs)
//             .as_mut()
//             .unwrap()
//             .is_interrupt_set()
//     }) {
//         esp_println::println!("Button was the source of the interrupt");
//     } else {
//         esp_println::println!("Button was not the source of the interrupt");
//     }

//     critical_section::with(|cs| {
//         BUTTON
//             .borrow_ref_mut(cs)
//             .as_mut()
//             .unwrap()
//             .clear_interrupt()
//     });
// }