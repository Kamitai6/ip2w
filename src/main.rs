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
use esp_bsp::{lcd_spi, lcd_backlight_init, lcd_display_interface, lcd_display, i2c_init, BoardType, DisplayConfig};
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

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

static BUTTON: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));

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

    // let mut io = Io::new(peripherals.IO_MUX);
    // io.set_interrupt_handler(handler);
    // let mut led = Output::new(peripherals.GPIO2, Level::Low, OutputConfig::default());
    // let config = InputConfig::default().with_pull(Pull::Up);
    // let mut button = Input::new(peripherals.GPIO0, config);

    // esp32では、RX/TX別々にDMAを割り当てるのではなく、それぞれ設定できるだけで同じチャンネルとして扱うらしい
    let dma_channel = peripherals.DMA_CH0;
    let (rx, tx) = dma_channel.split();
    rx.set_priority(DmaPriority::Priority0);
    tx.set_priority(DmaPriority::Priority0);

    let mut lcd_spi = lcd_spi!(peripherals);
    let mut lcd_backlight = lcd_backlight_init!(peripherals).unwrap();
    let di = lcd_display_interface!(peripherals, lcd_spi);
    let mut display = lcd_display!(peripherals, di);
    lcd_backlight.set_high();
    display.clear(Rgb565::WHITE).unwrap();

    Text::with_alignment("HELLO WORLD!", Point::new(160, 120), MonoTextStyleBuilder::new().font(&FONT_10X20).text_color(RgbColor::BLACK).build(),  Alignment::Center)
        .draw(&mut display)
        .unwrap();

    let i2c = i2c_init!(peripherals);

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

    // critical_section::with(|cs| {
    //     button.listen(Event::FallingEdge);
    //     BUTTON.borrow_ref_mut(cs).replace(button)
    // });

    loop {
        info!("Hello world!");

        motion.set_motor(atom_motion::MotorChannel::M1, -pid.control).unwrap();
        motion.set_motor(atom_motion::MotorChannel::M2, pid.control).unwrap();
        
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