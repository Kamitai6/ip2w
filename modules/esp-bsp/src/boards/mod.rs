#[cfg(feature = "esp32-c3-devkit-rust")]
pub mod esp32_c3_devkit_rust;
#[cfg(feature = "esp32-c3-devkit-rust")]
pub use esp32_c3_devkit_rust::{LCD_MEMORY_SIZE, LCD_H_RES, LCD_V_RES, LCD_BYTES_PER_PIXEL};

#[cfg(feature = "esp32-c3-lcdkit")]
pub mod esp32_c3_lcdkit;
#[cfg(feature = "esp32-c3-lcdkit")]
pub use esp32_c3_lcdkit::{LCD_MEMORY_SIZE, LCD_H_RES, LCD_V_RES, LCD_BYTES_PER_PIXEL};

#[cfg(feature = "esp32-c6-devkitc-1")]
pub mod esp32_c6_devkitc_1;
#[cfg(feature = "esp32-c6-devkitc-1")]
pub use esp32_c6_devkitc_1::{LCD_MEMORY_SIZE, LCD_H_RES, LCD_V_RES, LCD_BYTES_PER_PIXEL};

#[cfg(feature = "esp32-s2-kaluga")]
#[macro_use]
pub mod esp32_s2_kaluga;
#[cfg(feature = "esp32-s2-kaluga")]
pub use esp32_s2_kaluga::{LCD_MEMORY_SIZE, LCD_H_RES, LCD_V_RES, LCD_BYTES_PER_PIXEL};

#[cfg(feature = "esp32-s3-box")]
#[macro_use]
pub mod esp32_s3_box;
#[cfg(feature = "esp32-s3-box")]
pub use esp32_s3_box::{LCD_MEMORY_SIZE, LCD_H_RES, LCD_V_RES, LCD_BYTES_PER_PIXEL};

#[cfg(feature = "esp32-s3-usb-otg")]
#[macro_use]
pub mod esp32_s3_usb_otg;
#[cfg(feature = "esp32-s3-usb-otg")]
pub use esp32_s3_usb_otg::{LCD_MEMORY_SIZE, LCD_H_RES, LCD_V_RES, LCD_BYTES_PER_PIXEL};

#[cfg(feature = "esp32-wrover-kit")]
#[macro_use]
pub mod esp32_wrover_kit;
#[cfg(feature = "esp32-wrover-kit")]
pub use esp32_wrover_kit::{LCD_MEMORY_SIZE, LCD_H_RES, LCD_V_RES, LCD_BYTES_PER_PIXEL};

#[cfg(feature = "esp32-s3-box-3")]
#[macro_use]
pub mod esp32_s3_box_3;
#[cfg(feature = "esp32-s3-box-3")]
pub use esp32_s3_box_3::{LCD_MEMORY_SIZE, LCD_H_RES, LCD_V_RES, LCD_BYTES_PER_PIXEL};

#[cfg(feature = "m5stack-core2")]
#[macro_use]
pub mod m5stack_core2;
#[cfg(feature = "m5stack-core2")]
pub use m5stack_core2::{LCD_MEMORY_SIZE, LCD_H_RES, LCD_V_RES, LCD_BYTES_PER_PIXEL};

#[cfg(feature = "m5stack-cores3")]
#[macro_use]
pub mod m5stack_cores3;
#[cfg(feature = "m5stack-cores3")]
pub use m5stack_cores3::{LCD_MEMORY_SIZE, LCD_H_RES, LCD_V_RES, LCD_BYTES_PER_PIXEL};

#[cfg(feature = "m5stack-fire")]
#[macro_use]
pub mod m5stack_fire;
#[cfg(feature = "m5stack-fire")]
pub use m5stack_fire::{LCD_MEMORY_SIZE, LCD_H_RES, LCD_V_RES, LCD_BYTES_PER_PIXEL};

#[cfg(feature = "m5atom-s3r")]
#[macro_use]
pub mod m5atom_s3r;
#[cfg(feature = "m5atom-s3r")]
pub use m5atom_s3r::{LCD_MEMORY_SIZE, LCD_SIZE_W, LCD_SIZE_H, LCD_OFFSET_W, LCD_OFFSET_H, LCD_BYTES_PER_PIXEL};

#[cfg(feature = "waveshare-esp32-c6-lcd-1-47")]
#[macro_use]
pub mod waveshare_esp32_c6_lcd_1_47;
#[cfg(feature = "waveshare-esp32-c6-lcd-1-47")]
pub use waveshare_esp32_c6_lcd_1_47::{LCD_MEMORY_SIZE, LCD_H_RES, LCD_V_RES, LCD_BYTES_PER_PIXEL};

#[cfg(feature = "custom-board")]
#[macro_use]
pub mod custom_board;