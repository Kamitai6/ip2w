pub const LCD_H_RES: usize = 320;
pub const LCD_V_RES: usize = 240;
pub const LCD_BYTES_PER_PIXEL: usize = 2;
pub const LCD_MEMORY_SIZE: usize = LCD_H_RES * LCD_V_RES * LCD_BYTES_PER_PIXEL;

#[macro_export]
macro_rules! lcd_spi {
    ($peripherals:ident) => {
        shared_lcd_spi!(
            $peripherals,
            Dma::new($peripherals.DMA).channel0,
            $peripherals.GPIO36, // SCK
            $peripherals.GPIO37, // MOSI
            $peripherals.GPIO3   // CS
        )
    };
}

#[macro_export]
macro_rules! lcd_display_interface {
    ($peripherals:ident, $spi:expr) => {
        shared_lcd_display_interface!($peripherals, crate::LCD_MEMORY_SIZE, $spi, $peripherals.GPIO35)
    };
}

#[macro_export]
macro_rules! lcd_reset_pin {
    ($peripherals:ident) => {
        Output::new($peripherals.GPIO15, Level::Low)
    };
}

#[macro_export]
macro_rules! lcd_backlight_init {
    ($peripherals:ident) => {{

    }};
}

#[macro_export]
macro_rules! i2c_init {
    ($peripherals:ident) => {{
        I2c::new($peripherals.I2C0, esp_hal::i2c::master::Config::default())
            .with_sda($peripherals.GPIO12)
            .with_scl($peripherals.GPIO11)
    }};
}

#[macro_export]
macro_rules! lcd_display {
    ($peripherals:ident, $di:expr) => {
        shared_lcd_display!(
            $di,
            mipidsi::models::ILI9342CRgb565,
            crate::LCD_H_RES as u16,
            crate::LCD_V_RES as u16,
            mipidsi::options::Orientation::new(),
            mipidsi::options::ColorOrder::Bgr,
            lcd_reset_pin!($peripherals)
        )
        .invert_colors(mipidsi::options::ColorInversion::Inverted)
    };
}

pub use {
    i2c_init, lcd_backlight_init, lcd_display, lcd_display_interface, lcd_reset_pin,
    lcd_spi,
};
