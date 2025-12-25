# ESP-BSP-RS

Rust Bare Metal Board Support Packages (BSP) for ESP32-based boards with focus on Embassy Async.

## List of Supported Boards

### Actively Supported Boards

- [ESP32-C3-DevKit-RUST](https://github.com/esp-rs/esp-rust-board)
- [ESP32-C3-LcdKit](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c3/esp32-c3-lcdkit/user_guide.html)
- [ESP32-C6-DevKit-C1](https://docs.espressif.com/projects/espressif-esp-dev-kits/en/latest/esp32c6/esp32-c6-devkitc-1/index.html)
- [ESP32-S3-BOX-3](https://github.com/espressif/esp-box)
- [ESP32-S3-USB-OTG](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-usb-otg/user_guide.html)
- [M5Stack-Core2](https://shop.m5stack.com/products/m5stack-core2-esp32-iot-development-kit)
- [M5Stack-CoreS3](https://shop.m5stack.com/products/m5stack-cores3-esp32s3-lotdevelopment-kit)
- [M5Stack-Fire](https://docs.m5stack.com/en/core/fire)
- [WaveShare ESP32 C6 LCD 1.47](https://www.waveshare.com/esp32-c6-lcd-1.47.htm)

### Older boards

These boards are available in BSP for backward compatibility, but not recommended for new projects:

- [ESP-Wrover-Kit](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp-wrover-kit/index.html) - HW discontinued - replaced by ESP32-S3-BOX-3
- [ESP32-S2-Kaluga](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s2/esp32-s2-kaluga-1/index.html)
- [ESP32-S3-BOX](https://github.com/espressif/esp-box) - HW discontinued - replaced by ESP32-S3-BOX-3

## Usage

## Adding the BSP to Your Project

To add the ESP-BSP crate to your project:

```
cargo add esp-bsp
```

### Board-Specific Features

Ensure the correct feature flag is enabled in your Cargo.toml:

```toml
[features]
esp-bsp = { version = "0.4.0", features = ["esp32-s3-box-3"] }
```

### Board Initialization

Use the prelude for a streamlined initialization process.

```rust
use esp_bsp::prelude::*;

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    let mut delay = Delay::new();

    // Initialize I2C for peripherals like accelerometers
    let i2c = i2c_init!(peripherals);

    // Initialize SPI with DMA for LCD display
    let spi = lcd_dma_spi!(peripherals);

    // Create the display interface
    let di = lcd_display_interface!(peripherals, spi);

    // Initialize the display
    let mut display = lcd_display!(peripherals, di)
        .init(&mut delay)
        .unwrap();

    // Turn on the backlight
    lcd_backlight_init!(peripherals);

    // Your application code here
    println!("Display initialized!");
    loop {}
}
```

### Simplified Display Initialization

With `esp_bsp::prelude::*`, the macros ensure correct initialization per board based on the enabled feature.

## Examples

- [ESP32 Conway's Game of Life](https://github.com/georgik/esp32-conways-game-of-life-rs)
- [ESP32 Spooky Maze](https://github.com/georgik/esp32-spooky-maze-game) - Rust Bare Metal

## Changelog

### 0.4.1

- LCD_H_RES, LCD_V_RES, LCD_BYTES_PER_PIXEL, LCD_MEMORY_SIZE are exported via prelude

### 0.4.0

- Renamed BSPs
- Added BSPs for common boards for ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C6

### 0.3.0

- Unified BSP initialization using shared macros.
- Introduced prelude for simplified imports and initialization.

### 0.2.0

- renamed 
