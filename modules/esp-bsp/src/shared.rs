#[macro_export]
macro_rules! shared_lcd_spi {
    ($peripherals:ident, $dma_channel:expr, $sck:expr, $mosi:expr, $cs:expr) => {
        ::esp_hal::spi::master::Spi::new(
            $peripherals.SPI2,
            ::esp_hal::spi::master::Config::default()
                .with_frequency(::esp_hal::time::Rate::from_mhz(40))
                .with_mode(::esp_hal::spi::Mode::_0)
        )
        .unwrap()
        .with_sck($sck)
        .with_mosi($mosi)
        .with_cs($cs)
        .with_dma($dma_channel)
    };
}

#[macro_export]
macro_rules! shared_lcd_display_interface {
    ($peripherals:ident, $memory_size:expr, $spi:expr, $dc_pin:expr) => {{
        let lcd_dc = ::esp_hal::gpio::Output::new($dc_pin, Level::Low, ::esp_hal::gpio::OutputConfig::default());
        display_interface_spi_dma::new_no_cs($memory_size, $spi, lcd_dc)
    }};
}

#[macro_export]
macro_rules! shared_lcd_display {
    ($di:expr, $display_model:expr, $width:expr, $height:expr, $orientation:expr, $color_order:expr, $reset_pin:expr) => {{
        ::mipidsi::Builder::new($display_model, $di)
            .display_size($width, $height)
            .orientation($orientation)
            .color_order($color_order)
            .reset_pin($reset_pin)
    }};
}

pub use {
    shared_lcd_spi, shared_lcd_display_interface, shared_lcd_display
};
