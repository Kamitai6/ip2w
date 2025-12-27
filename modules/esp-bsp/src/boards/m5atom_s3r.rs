pub const LCD_SIZE_W: usize = 128;
pub const LCD_SIZE_H: usize = 128;
pub const LCD_OFFSET_W: usize = 0;
pub const LCD_OFFSET_H: usize = 32;
pub const LCD_BYTES_PER_PIXEL: usize = 2;
pub const LCD_MEMORY_SIZE: usize = LCD_SIZE_W * LCD_SIZE_H * LCD_BYTES_PER_PIXEL;

#[macro_export]
macro_rules! lcd_spi {
    ($peripherals:ident) => {
        ::esp_bsp::shared_lcd_spi!(
            $peripherals,
            $peripherals.DMA_CH0,
            $peripherals.GPIO15, // SCK
            $peripherals.GPIO21 // MOSI
        )
    };
}

#[macro_export]
macro_rules! lcd_reset_pin {
    ($peripherals:ident) => {
        ::esp_hal::gpio::Output::new(
            $peripherals.GPIO48, 
            ::esp_hal::gpio::Level::High, 
            ::esp_hal::gpio::OutputConfig::default())
    };
}

#[macro_export]
macro_rules! lcd_backlight_init {
    ($peripherals:ident) => {{
    }};
}

#[macro_export]
macro_rules! lcd_display_interface {
    ($peripherals:ident, $spi:expr) => {
        ::esp_bsp::shared_lcd_display_interface!($peripherals, $spi, $peripherals.GPIO42, $peripherals.GPIO14) //DC, CS
    };
}

#[macro_export]
macro_rules! lcd_display {
    ($peripherals:ident, $di:expr, $delay:expr) => {
        ::esp_bsp::shared_lcd_display!(
            $di,
            ::mipidsi::models::GC9107,
            ::esp_bsp::lcd_reset_pin!($peripherals),
            $crate::LCD_SIZE_W as u16,
            $crate::LCD_SIZE_H as u16,
            $crate::LCD_OFFSET_W as u16,
            $crate::LCD_OFFSET_H as u16,
            ::mipidsi::options::Orientation::new().rotate(::mipidsi::options::Rotation::Deg180),
            ::mipidsi::options::ColorOrder::Bgr,
            ::mipidsi::options::ColorInversion::Normal
        )
        .init($delay)
    };
}

#[macro_export]
macro_rules! i2c_init {
    ($peripherals:ident) => {{
        ::esp_hal::i2c::master::I2c::new(
            $peripherals.I2C0, 
            ::esp_hal::i2c::master::Config::default()
                .with_frequency(::esp_hal::time::Rate::from_khz(100))
            ).unwrap()
                .with_sda($peripherals.GPIO45)
                .with_scl($peripherals.GPIO0)
    }};
}

pub use {
    i2c_init, lcd_backlight_init, lcd_display, lcd_display_interface,
    lcd_spi,
};
