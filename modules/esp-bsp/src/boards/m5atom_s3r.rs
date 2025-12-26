pub const LCD_H_RES: usize = 128;
pub const LCD_V_RES: usize = 128;
pub const LCD_BYTES_PER_PIXEL: usize = 2;
pub const LCD_MEMORY_SIZE: usize = LCD_H_RES * LCD_V_RES * LCD_BYTES_PER_PIXEL;

#[macro_export]
macro_rules! lcd_spi {
    ($peripherals:ident) => {
        ::esp_bsp::shared_lcd_spi!(
            $peripherals,
            $peripherals.DMA_CH0,
            $peripherals.GPIO15, // SCK
            $peripherals.GPIO21, // MOSI
            $peripherals.GPIO14   // CS
        )
    };
}

#[macro_export]
macro_rules! lcd_reset_pin {
    ($peripherals:ident) => {
        ::esp_hal::gpio::Output::new($peripherals.GPIO48, Level::High, OutputConfig::default())
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
        ::esp_bsp::shared_lcd_display_interface!($peripherals, $spi, $peripherals.GPIO42) //DC
    };
}

#[macro_export]
macro_rules! lcd_display {
    ($peripherals:ident, $di:expr, $delay:expr) => {
        ::esp_bsp::shared_lcd_display!(
            $di,
            mipidsi::models::GC9107,
            $crate::LCD_H_RES as u16,
            $crate::LCD_V_RES as u16,
            mipidsi::options::Orientation::new(),
            mipidsi::options::ColorOrder::Bgr,
            ::esp_bsp::lcd_reset_pin!($peripherals)
        )
        .invert_colors(mipidsi::options::ColorInversion::Inverted)
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
