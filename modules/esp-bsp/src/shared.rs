#[macro_export]
macro_rules! shared_lcd_spi {
    ($peripherals:ident, $dma_channel:expr, $sck:expr, $mosi:expr) => {
        ::esp_hal::spi::master::Spi::new(
            $peripherals.SPI2,
            ::esp_hal::spi::master::Config::default()
                .with_frequency(::esp_hal::time::Rate::from_mhz(40))
                .with_mode(::esp_hal::spi::Mode::_0)
        )
        .unwrap()
        .with_sck($sck)
        .with_mosi($mosi)
        .with_dma($dma_channel)
    };
}

#[macro_export]
macro_rules! shared_lcd_display_interface {
    ($peripherals:ident, $spi:expr, $dc_pin:expr, $cs_pin:expr) => {{
        let lcd_dc = ::esp_hal::gpio::Output::new($dc_pin, Level::Low, ::esp_hal::gpio::OutputConfig::default());
        let lcd_cs = ::esp_hal::gpio::Output::new($cs_pin, Level::High, ::esp_hal::gpio::OutputConfig::default());
        display_interface_spi_dma::SPIInterface::new($spi, lcd_dc, lcd_cs)
    }};
}

#[macro_export]
macro_rules! shared_lcd_display {
    (
        $di:expr, $display_model:expr, $reset_pin:expr, 
        $size_w:expr, $size_h:expr, 
        $offset_w:expr, $offset_h:expr, 
        $orientation:expr, $color_order:expr, $invert_colors:expr
    ) => {{
        ::mipidsi::Builder::new($display_model, $di)
            .reset_pin($reset_pin)
            .display_size($size_w, $size_h)
            .display_offset($offset_w, $offset_h)
            .orientation($orientation)
            .color_order($color_order)
            .invert_colors($invert_colors)
    }};
}

pub use {
    shared_lcd_spi, shared_lcd_display_interface, shared_lcd_display
};
