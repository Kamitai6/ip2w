#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::cell::RefCell;
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
use esp_bsp::{lcd_gpios, BoardType, DisplayConfig, define_display_type};
use embedded_graphics::{
    prelude::{RgbColor, Point, DrawTarget},
    pixelcolor::Rgb565,
    mono_font::{
        ascii::FONT_10X20,
        MonoTextStyleBuilder,
    },
    text::{Alignment, Text},
    Drawable,
};

extern crate alloc;

type BoardDisplay = define_display_type!(BoardType::ESP32S3Box);

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

static BUTTON: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));

macro_rules! dma_alloc_buffer {
    ($size:expr, $align:expr) => {{
        let layout = core::alloc::Layout::from_size_align($size, $align).unwrap();
        unsafe {
            let ptr = alloc::alloc::alloc(layout);
            if ptr.is_null() {
                error!("dma_alloc_buffer: alloc failed");
                alloc::alloc::handle_alloc_error(layout);
            }
            core::slice::from_raw_parts_mut(ptr, $size)
        }
    }};
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[main]
fn main() -> ! {
    // generator version: 1.1.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    let delay = Delay::new();

    let descriptors = make_static!([0u32; 8 * 3]);
    let rx_descriptors = make_static!([0u32; 8 * 3]);
    let (_, tx_descriptors) =
        esp_hal::dma_descriptors_chunk_size!(0, DMA_BUFFER_SIZE, DMA_CHUNK_SIZE);
    let tx_buffer = dma_alloc_buffer!(DMA_BUFFER_SIZE, DMA_ALIGNMENT as usize);
    let mut dma_tx_buf =
        DmaTxBuf::new_with_config(tx_descriptors, tx_buffer, DMA_ALIGNMENT).unwrap();
    let (rx_buffer, rx_descriptors, _, _) = esp_hal::dma_buffers!(DMA_BUFFER_SIZE, 0);
    let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

    let (lcd_sclk, lcd_mosi, lcd_cs, lcd_miso, lcd_dc, mut lcd_backlight, lcd_reset) = lcd_gpios!(BoardType::ESP32S3Box, io);

    let mut io = Io::new(peripherals.IO_MUX);
    io.set_interrupt_handler(handler);
    let mut led = Output::new(peripherals.GPIO2, Level::Low, OutputConfig::default());
    let config = InputConfig::default().with_pull(Pull::Up);
    let mut button = Input::new(peripherals.GPIO0, config);

    let sclk = peripherals.GPIO42;
    let mosi = peripherals.GPIO48;
    let miso = unsafe { mosi.clone_unchecked() };
    let cs = peripherals.GPIO38;

    // esp32では、RX/TX別々にDMAを割り当てるのではなく、それぞれ設定できるだけで同じチャンネルとして扱うらしい
    let dma_channel = peripherals.DMA_CH0;
    let (rx, tx) = dma_channel.split();
    rx.set_priority(DmaPriority::Priority0);
    tx.set_priority(DmaPriority::Priority0);

    // Need to set miso first so that mosi can overwrite the
    // output connection (because we are using the same pin to loop back)
    let mut spi = Spi::new(
        peripherals.SPI2,
        SpiCfg::default()
            .with_frequency(Rate::from_khz(100))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(sclk)
    .with_miso(miso)
    .with_mosi(mosi)
    .with_cs(cs)
    .with_dma(peripherals.DMA_CH0);
    // .with_buffers(dma_rx_buf, dma_tx_buf);

    let di = display_interface_spi_dma::new_no_cs(2 * 256 * 192, spi, lcd_dc);
    let display_config = DisplayConfig::for_board(BoardType::ESP32S3Box);
    let mut display: BoardDisplay = match mipidsi::Builder::ili9342c_rgb565(di)
        .with_display_size(display_config.h_res, display_config.v_res)
        .with_orientation(mipidsi::Orientation::PortraitInverted(false))
        .with_color_order(mipidsi::ColorOrder::Bgr)
        .init(&mut delay, Some(lcd_reset)) {
        Ok(display) => display,
        Err(_) => {
            panic!("Display initialization failed");
        }
    };
    lcd_backlight.set_high().unwrap();
    display.clear(Rgb565::WHITE).unwrap();

    Text::with_alignment("HELLO WORLD!", Point::new(160, 120), MonoTextStyleBuilder::new().font(&FONT_10X20).text_color(RgbColor::BLACK).build(),  Alignment::Center)
        .draw(&mut display)
        .unwrap();

    // Initialize I2C
    let i2c_config = I2cCfg::default()
        .with_frequency(Rate::from_khz(400));
    let i2c = I2c::new(peripherals.I2C0, i2c_config).unwrap()
        .with_sda(sda) //need pull up
        .with_scl(scl);

    // Used to share the I2C bus with multiple drivers (IMU and motor)
    let i2c_bus = shared_bus::BusManagerSimple::new(i2c);

    // Initialize IMU
    let mut imu = Mpu6886::new(i2c_bus.acquire_i2c());
    imu.init(&mut delay::FreeRtos).unwrap();

    // Initialize Atom Motion motor driver
    let mut motion = atom_motion::AtomMotion::new(i2c_bus.acquire_i2c());

    let imu_transform: Matrix3<f32> = Matrix3::from_diagonal(&Vector3::new(-1.0, 1.0, -1.0));

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);
    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    critical_section::with(|cs| {
        button.listen(Event::FallingEdge);
        BUTTON.borrow_ref_mut(cs).replace(button)
    });

    loop {
        info!("Hello world!");

        motion.set_motor(atom_motion::MotorChannel::M1, -pid.control).unwrap();
        motion.set_motor(atom_motion::MotorChannel::M2, pid.control).unwrap();
        
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(500) {}
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v~1.0/examples
}

#[handler]
#[ram]
fn handler() {
    esp_println::println!(
        "GPIO Interrupt with priority {}",
        esp_hal::xtensa_lx::interrupt::get_level()
    );

    if critical_section::with(|cs| {
        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .is_interrupt_set()
    }) {
        esp_println::println!("Button was the source of the interrupt");
    } else {
        esp_println::println!("Button was not the source of the interrupt");
    }

    critical_section::with(|cs| {
        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}