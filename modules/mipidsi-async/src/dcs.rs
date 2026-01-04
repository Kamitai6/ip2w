//! MIPI DCS (Display Command Set) commands.

use crate::interface::Interface;
use crate::options::{
    ColorInversion, ColorOrder, HorizontalRefreshOrder, MemoryMapping, ModelOptions,
    Orientation, TearingEffect, VerticalRefreshOrder,
};

/// Common trait for DCS commands
pub trait DcsCommand {
    /// Returns the instruction code
    fn instruction(&self) -> u8;
    
    /// Fills the given buffer with command parameters, returns number of bytes written
    fn fill_params_buf(&self, buffer: &mut [u8]) -> usize;
}

/// Extension trait for Interface with DCS command support
pub trait InterfaceExt: Interface {
    /// Send a DCS command
    fn write_command(&mut self, command: impl DcsCommand) -> Result<(), Self::Error> {
        let mut param_bytes: [u8; 16] = [0; 16];
        let n = command.fill_params_buf(&mut param_bytes);
        self.write_raw(command.instruction(), &param_bytes[..n])
    }

    /// Send a raw command with parameters
    ///
    /// This is intended for commands not in the MIPI DCS command set.
    fn write_raw(&mut self, instruction: u8, param_bytes: &[u8]) -> Result<(), Self::Error> {
        self.send_command(instruction, param_bytes)
    }
}

impl<T: Interface> InterfaceExt for T {}

// ============================================================================
// Basic commands (no parameters)
// ============================================================================

macro_rules! dcs_basic_command {
    ($(#[$doc:meta])* $name:ident, $instruction:expr) => {
        $(#[$doc])*
        pub struct $name;

        impl DcsCommand for $name {
            fn instruction(&self) -> u8 {
                $instruction
            }

            fn fill_params_buf(&self, _buffer: &mut [u8]) -> usize {
                0
            }
        }
    };
}

dcs_basic_command!(
    /// Software Reset (0x01)
    SoftReset,
    0x01
);

dcs_basic_command!(
    /// Enter Sleep Mode (0x10)
    EnterSleepMode,
    0x10
);

dcs_basic_command!(
    /// Exit Sleep Mode (0x11)
    ExitSleepMode,
    0x11
);

dcs_basic_command!(
    /// Enter Partial Mode (0x12)
    EnterPartialMode,
    0x12
);

dcs_basic_command!(
    /// Enter Normal Mode (0x13)
    EnterNormalMode,
    0x13
);

dcs_basic_command!(
    /// Display Inversion Off (0x20)
    ExitInvertMode,
    0x20
);

dcs_basic_command!(
    /// Display Inversion On (0x21)
    EnterInvertMode,
    0x21
);

dcs_basic_command!(
    /// Display Off (0x28)
    SetDisplayOff,
    0x28
);

dcs_basic_command!(
    /// Display On (0x29)
    SetDisplayOn,
    0x29
);

dcs_basic_command!(
    /// Exit Idle Mode (0x38)
    ExitIdleMode,
    0x38
);

dcs_basic_command!(
    /// Enter Idle Mode (0x39)
    EnterIdleMode,
    0x39
);

dcs_basic_command!(
    /// Write Memory Start (0x2C)
    WriteMemoryStart,
    0x2C
);

// ============================================================================
// Set Column Address (0x2A)
// ============================================================================

/// Set Column Address command (0x2A)
pub struct SetColumnAddress {
    start: u16,
    end: u16,
}

impl SetColumnAddress {
    /// Create a new SetColumnAddress command
    pub const fn new(start: u16, end: u16) -> Self {
        Self { start, end }
    }
}

impl DcsCommand for SetColumnAddress {
    fn instruction(&self) -> u8 {
        0x2A
    }

    fn fill_params_buf(&self, buffer: &mut [u8]) -> usize {
        buffer[0] = (self.start >> 8) as u8;
        buffer[1] = self.start as u8;
        buffer[2] = (self.end >> 8) as u8;
        buffer[3] = self.end as u8;
        4
    }
}

// ============================================================================
// Set Page Address (0x2B)
// ============================================================================

/// Set Page Address command (0x2B)
pub struct SetPageAddress {
    start: u16,
    end: u16,
}

impl SetPageAddress {
    /// Create a new SetPageAddress command
    pub const fn new(start: u16, end: u16) -> Self {
        Self { start, end }
    }
}

impl DcsCommand for SetPageAddress {
    fn instruction(&self) -> u8 {
        0x2B
    }

    fn fill_params_buf(&self, buffer: &mut [u8]) -> usize {
        buffer[0] = (self.start >> 8) as u8;
        buffer[1] = self.start as u8;
        buffer[2] = (self.end >> 8) as u8;
        buffer[3] = self.end as u8;
        4
    }
}

// ============================================================================
// Set Address Mode (MADCTL) (0x36)
// ============================================================================

/// Memory Data Access Control (MADCTL) command (0x36)
#[derive(Debug, Clone, Copy, Default)]
pub struct SetAddressMode {
    /// Row address order (MY)
    pub row_address_order: bool,
    /// Column address order (MX)
    pub column_address_order: bool,
    /// Row/Column exchange (MV)
    pub row_column_exchange: bool,
    /// Vertical refresh order (ML)
    pub vertical_refresh_order: VerticalRefreshOrder,
    /// Color order (RGB/BGR)
    pub color_order: ColorOrder,
    /// Horizontal refresh order (MH)
    pub horizontal_refresh_order: HorizontalRefreshOrder,
}

impl SetAddressMode {
    /// Create from orientation
    pub fn from_orientation(orientation: Orientation) -> Self {
        let mapping = MemoryMapping::from(orientation);
        Self {
            row_address_order: mapping.reverse_rows,
            column_address_order: mapping.reverse_columns,
            row_column_exchange: mapping.swap_rows_and_columns,
            ..Default::default()
        }
    }
}

impl From<&ModelOptions> for SetAddressMode {
    fn from(options: &ModelOptions) -> Self {
        let mapping = MemoryMapping::from(options.orientation);
        Self {
            row_address_order: mapping.reverse_rows,
            column_address_order: mapping.reverse_columns,
            row_column_exchange: mapping.swap_rows_and_columns,
            vertical_refresh_order: options.refresh_order.vertical,
            color_order: options.color_order,
            horizontal_refresh_order: options.refresh_order.horizontal,
        }
    }
}

impl DcsCommand for SetAddressMode {
    fn instruction(&self) -> u8 {
        0x36
    }

    fn fill_params_buf(&self, buffer: &mut [u8]) -> usize {
        let mut value = 0u8;
        
        if self.row_address_order {
            value |= 0b1000_0000; // MY
        }
        if self.column_address_order {
            value |= 0b0100_0000; // MX
        }
        if self.row_column_exchange {
            value |= 0b0010_0000; // MV
        }
        if matches!(self.vertical_refresh_order, VerticalRefreshOrder::BottomToTop) {
            value |= 0b0001_0000; // ML
        }
        if matches!(self.color_order, ColorOrder::Bgr) {
            value |= 0b0000_1000; // RGB/BGR
        }
        if matches!(self.horizontal_refresh_order, HorizontalRefreshOrder::RightToLeft) {
            value |= 0b0000_0100; // MH
        }
        
        buffer[0] = value;
        1
    }
}

// ============================================================================
// Set Pixel Format (0x3A)
// ============================================================================

/// Pixel format for RGB interface
#[derive(Debug, Clone, Copy, Default)]
pub enum PixelFormat {
    /// 12 bits per pixel
    Bpp12,
    /// 16 bits per pixel (RGB565)
    #[default]
    Bpp16,
    /// 18 bits per pixel (RGB666)
    Bpp18,
    /// 24 bits per pixel (RGB888)
    Bpp24,
}

impl PixelFormat {
    fn as_bits(self) -> u8 {
        match self {
            Self::Bpp12 => 0b011,
            Self::Bpp16 => 0b101,
            Self::Bpp18 => 0b110,
            Self::Bpp24 => 0b111,
        }
    }
}

/// Set Pixel Format command (0x3A)
pub struct SetPixelFormat {
    /// RGB interface format
    pub rgb: PixelFormat,
    /// Control interface format
    pub control: PixelFormat,
}

impl SetPixelFormat {
    /// Create a new SetPixelFormat with both interfaces set to the same format
    pub const fn new(format: PixelFormat) -> Self {
        Self {
            rgb: format,
            control: format,
        }
    }
}

impl DcsCommand for SetPixelFormat {
    fn instruction(&self) -> u8 {
        0x3A
    }

    fn fill_params_buf(&self, buffer: &mut [u8]) -> usize {
        buffer[0] = (self.rgb.as_bits() << 4) | self.control.as_bits();
        1
    }
}

// ============================================================================
// Set Scroll Area (0x33)
// ============================================================================

/// Set Scroll Area command (0x33)
pub struct SetScrollArea {
    top_fixed: u16,
    scroll_area: u16,
    bottom_fixed: u16,
}

impl SetScrollArea {
    /// Create a new SetScrollArea command
    pub const fn new(top_fixed: u16, scroll_area: u16, bottom_fixed: u16) -> Self {
        Self {
            top_fixed,
            scroll_area,
            bottom_fixed,
        }
    }
}

impl DcsCommand for SetScrollArea {
    fn instruction(&self) -> u8 {
        0x33
    }

    fn fill_params_buf(&self, buffer: &mut [u8]) -> usize {
        buffer[0] = (self.top_fixed >> 8) as u8;
        buffer[1] = self.top_fixed as u8;
        buffer[2] = (self.scroll_area >> 8) as u8;
        buffer[3] = self.scroll_area as u8;
        buffer[4] = (self.bottom_fixed >> 8) as u8;
        buffer[5] = self.bottom_fixed as u8;
        6
    }
}

// ============================================================================
// Set Scroll Start (0x37)
// ============================================================================

/// Set Scroll Start command (0x37)
pub struct SetScrollStart {
    offset: u16,
}

impl SetScrollStart {
    /// Create a new SetScrollStart command
    pub const fn new(offset: u16) -> Self {
        Self { offset }
    }
}

impl DcsCommand for SetScrollStart {
    fn instruction(&self) -> u8 {
        0x37
    }

    fn fill_params_buf(&self, buffer: &mut [u8]) -> usize {
        buffer[0] = (self.offset >> 8) as u8;
        buffer[1] = self.offset as u8;
        2
    }
}

// ============================================================================
// Set Tearing Effect (0x35)
// ============================================================================

/// Set Tearing Effect Line On command (0x35)
pub struct SetTearingEffect {
    mode: TearingEffect,
}

impl SetTearingEffect {
    /// Create a new SetTearingEffect command
    pub const fn new(mode: TearingEffect) -> Self {
        Self { mode }
    }
}

impl DcsCommand for SetTearingEffect {
    fn instruction(&self) -> u8 {
        match self.mode {
            TearingEffect::Off => 0x34, // Tearing Effect Line Off
            _ => 0x35,                   // Tearing Effect Line On
        }
    }

    fn fill_params_buf(&self, buffer: &mut [u8]) -> usize {
        match self.mode {
            TearingEffect::Off => 0,
            TearingEffect::Vertical => {
                buffer[0] = 0x00;
                1
            }
            TearingEffect::HorizontalAndVertical => {
                buffer[0] = 0x01;
                1
            }
        }
    }
}

// ============================================================================
// Set Invert Mode
// ============================================================================

/// Set display inversion mode
pub struct SetInvertMode {
    inverted: bool,
}

impl SetInvertMode {
    /// Create a new SetInvertMode command
    pub const fn new(inversion: ColorInversion) -> Self {
        Self {
            inverted: matches!(inversion, ColorInversion::Inverted),
        }
    }
}

impl DcsCommand for SetInvertMode {
    fn instruction(&self) -> u8 {
        if self.inverted {
            0x21 // INVON
        } else {
            0x20 // INVOFF
        }
    }

    fn fill_params_buf(&self, _buffer: &mut [u8]) -> usize {
        0
    }
}