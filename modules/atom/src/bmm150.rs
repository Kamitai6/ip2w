//! BMM150 3-Axis Magnetometer Driver
//!
//! Standalone BMM150 definitions and compensation algorithms.
//! Can be used with BMI270's AUX interface or directly via I2C.

/// BMM150 I2C addresses
pub const ADDR_CSB_LOW: u8 = 0x10;
pub const ADDR_CSB_HIGH: u8 = 0x13;
pub const DEFAULT_ADDR: u8 = ADDR_CSB_LOW;

/// BMM150 Chip ID
pub const CHIP_ID: u8 = 0x32;

/// Register addresses
#[allow(dead_code)]
pub mod reg {
    pub const CHIP_ID: u8 = 0x40;
    pub const DATA_X_LSB: u8 = 0x42;
    pub const DATA_X_MSB: u8 = 0x43;
    pub const DATA_Y_LSB: u8 = 0x44;
    pub const DATA_Y_MSB: u8 = 0x45;
    pub const DATA_Z_LSB: u8 = 0x46;
    pub const DATA_Z_MSB: u8 = 0x47;
    pub const RHALL_LSB: u8 = 0x48;
    pub const RHALL_MSB: u8 = 0x49;
    pub const INT_STATUS: u8 = 0x4A;
    pub const POWER_CTRL: u8 = 0x4B;
    pub const OP_MODE: u8 = 0x4C;
    pub const INT_CTRL: u8 = 0x4D;
    pub const SENS_CTRL: u8 = 0x4E;
    pub const LOW_THRESHOLD: u8 = 0x4F;
    pub const HIGH_THRESHOLD: u8 = 0x50;
    pub const REP_XY: u8 = 0x51;
    pub const REP_Z: u8 = 0x52;

    // Trim registers
    pub const DIG_X1: u8 = 0x5D;
    pub const DIG_Y1: u8 = 0x5E;
    pub const DIG_Z4_LSB: u8 = 0x62;
    pub const DIG_Z4_MSB: u8 = 0x63;
    pub const DIG_X2: u8 = 0x64;
    pub const DIG_Y2: u8 = 0x65;
    pub const DIG_Z2_LSB: u8 = 0x68;
    pub const DIG_Z2_MSB: u8 = 0x69;
    pub const DIG_Z1_LSB: u8 = 0x6A;
    pub const DIG_Z1_MSB: u8 = 0x6B;
    pub const DIG_XYZ1_LSB: u8 = 0x6C;
    pub const DIG_XYZ1_MSB: u8 = 0x6D;
    pub const DIG_Z3_LSB: u8 = 0x6E;
    pub const DIG_Z3_MSB: u8 = 0x6F;
    pub const DIG_XY2: u8 = 0x70;
    pub const DIG_XY1: u8 = 0x71;
}

/// Power control values
pub mod power {
    pub const SOFT_RESET: u8 = 0x82;
    pub const POWER_ON: u8 = 0x01;
    pub const POWER_OFF: u8 = 0x00;
}

/// Operation mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum OpMode {
    #[default]
    Normal = 0x00,
    Forced = 0x02,
    Sleep = 0x06,
}

/// Data rate (ODR)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum DataRate {
    Hz2 = 0x01,
    Hz6 = 0x02,
    #[default]
    Hz8 = 0x03,
    Hz10 = 0x00,
    Hz15 = 0x04,
    Hz20 = 0x05,
    Hz25 = 0x06,
    Hz30 = 0x07,
}

/// Preset configurations
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Preset {
    /// Low power: XY=3, Z=3 repetitions
    LowPower,
    /// Regular: XY=9, Z=15 repetitions
    #[default]
    Regular,
    /// Enhanced: XY=15, Z=27 repetitions
    Enhanced,
    /// High accuracy: XY=47, Z=83 repetitions
    HighAccuracy,
}

impl Preset {
    /// Returns (rep_xy_reg, rep_z_reg) register values
    /// Actual repetitions = reg_value * 2 + 1
    pub fn repetitions(self) -> (u8, u8) {
        match self {
            Preset::LowPower => (1, 2),
            Preset::Regular => (4, 14),
            Preset::Enhanced => (7, 26),
            Preset::HighAccuracy => (23, 82),
        }
    }
}

/// Trim data for compensation
#[derive(Debug, Clone, Copy, Default)]
pub struct TrimData {
    pub dig_x1: i8,
    pub dig_y1: i8,
    pub dig_x2: i8,
    pub dig_y2: i8,
    pub dig_z1: u16,
    pub dig_z2: i16,
    pub dig_z3: i16,
    pub dig_z4: i16,
    pub dig_xy1: u8,
    pub dig_xy2: i8,
    pub dig_xyz1: u16,
}

/// Raw magnetometer data [LSB]
#[derive(Debug, Clone, Copy, Default)]
pub struct RawData {
    /// X-axis [13-bit signed]
    pub x: i16,
    /// Y-axis [13-bit signed]
    pub y: i16,
    /// Z-axis [15-bit signed]
    pub z: i16,
    /// Hall resistance [14-bit unsigned]
    pub rhall: u16,
}

/// Compensated magnetometer data [µT]
#[derive(Debug, Clone, Copy, Default)]
pub struct Mag {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl RawData {
    pub fn from_bytes(buf: &[u8; 8]) -> Self {
        // X: 13-bit signed (LSB[7:3] + MSB[7:0])
        let x13 = (((buf[1] as u16) << 5) | ((buf[0] as u16) >> 3)) as i16;
        let x = (x13 << 3) >> 3;  // 13-bit sign extend

        // Y: 13-bit signed
        let y13 = (((buf[3] as u16) << 5) | ((buf[2] as u16) >> 3)) as i16;
        let y = (y13 << 3) >> 3;  // 13-bit sign extend

        // Z: 15-bit signed (LSB[7:1] + MSB[7:0])
        let z15 = (((buf[5] as u16) << 7) | ((buf[4] as u16) >> 1)) as i16;
        let z = (z15 << 1) >> 1;  // 15-bit sign extend

        // RHALL: 14-bit unsigned (LSB[7:2] + MSB[7:0])
        let rhall = ((buf[7] as u16) << 6) | ((buf[6] as u16) >> 2);

        Self { x, y, z, rhall }
    }
}

impl TrimData {
    /// Build TrimData from individual register reads
    pub fn new(
        dig_x1: u8,
        dig_y1: u8,
        dig_x2: u8,
        dig_y2: u8,
        dig_z1_lsb: u8,
        dig_z1_msb: u8,
        dig_z2_lsb: u8,
        dig_z2_msb: u8,
        dig_z3_lsb: u8,
        dig_z3_msb: u8,
        dig_z4_lsb: u8,
        dig_z4_msb: u8,
        dig_xy1: u8,
        dig_xy2: u8,
        dig_xyz1_lsb: u8,
        dig_xyz1_msb: u8,
    ) -> Self {
        Self {
            dig_x1: dig_x1 as i8,
            dig_y1: dig_y1 as i8,
            dig_x2: dig_x2 as i8,
            dig_y2: dig_y2 as i8,
            dig_z1: u16::from_le_bytes([dig_z1_lsb, dig_z1_msb]),
            dig_z2: i16::from_le_bytes([dig_z2_lsb, dig_z2_msb]),
            dig_z3: i16::from_le_bytes([dig_z3_lsb, dig_z3_msb]),
            dig_z4: i16::from_le_bytes([dig_z4_lsb, dig_z4_msb]),
            dig_xy1,
            dig_xy2: dig_xy2 as i8,
            dig_xyz1: u16::from_le_bytes([dig_xyz1_lsb, dig_xyz1_msb]),
        }
    }

    /// Apply compensation to raw data, returns [µT]
    pub fn compensate(&self, raw: &RawData) -> Mag {
        if raw.rhall == 0 {
            return Mag::default();
        }

        let rhall = raw.rhall as f32;

        Mag {
            x: self.compensate_x(raw.x, rhall),
            y: self.compensate_y(raw.y, rhall),
            z: self.compensate_z(raw.z, rhall),
        }
    }

    fn compensate_x(&self, raw: i16, rhall: f32) -> f32 {
        if raw == -4096 {
            return 0.0;
        }

        let process_comp_x0 = self.dig_xyz1 as f32 * 16384.0 / rhall;
        let process_comp_x1 = process_comp_x0 - 16384.0;
        let process_comp_x2 =
            self.dig_xy2 as f32 * (process_comp_x1 * process_comp_x1 / 268435456.0);
        let process_comp_x3 = process_comp_x2 + process_comp_x1 * self.dig_xy1 as f32 / 16384.0;
        let process_comp_x4 = self.dig_x2 as f32 + 160.0;
        let process_comp_x5 = raw as f32 * ((process_comp_x3 + 256.0) * process_comp_x4);

        (process_comp_x5 / 8192.0 + self.dig_x1 as f32 * 8.0) / 16.0
    }

    fn compensate_y(&self, raw: i16, rhall: f32) -> f32 {
        if raw == -4096 {
            return 0.0;
        }

        let process_comp_y0 = self.dig_xyz1 as f32 * 16384.0 / rhall;
        let process_comp_y1 = process_comp_y0 - 16384.0;
        let process_comp_y2 =
            self.dig_xy2 as f32 * (process_comp_y1 * process_comp_y1 / 268435456.0);
        let process_comp_y3 = process_comp_y2 + process_comp_y1 * self.dig_xy1 as f32 / 16384.0;
        let process_comp_y4 = self.dig_y2 as f32 + 160.0;
        let process_comp_y5 = raw as f32 * ((process_comp_y3 + 256.0) * process_comp_y4);

        (process_comp_y5 / 8192.0 + self.dig_y1 as f32 * 8.0) / 16.0
    }

    fn compensate_z(&self, raw: i16, rhall: f32) -> f32 {
        if raw == -16384 {
            return 0.0;
        }

        let process_comp_z0 = raw as f32 - self.dig_z4 as f32;
        let process_comp_z1 = rhall - self.dig_xyz1 as f32;
        let process_comp_z2 = self.dig_z3 as f32 * process_comp_z1;
        let process_comp_z3 = self.dig_z1 as f32 * rhall / 32768.0;
        let process_comp_z4 = self.dig_z2 as f32 + process_comp_z3;
        let process_comp_z5 = process_comp_z0 * 131072.0 - process_comp_z2;

        process_comp_z5 / (process_comp_z4 * 4.0) / 16.0
    }
}

/// BMM150 state for use with BMI270 AUX interface
#[derive(Debug, Clone, Default)]
pub struct Bmm150Aux {
    trim: TrimData,
    preset: Preset,
    data_rate: DataRate,
    op_mode: OpMode,
}

impl Bmm150Aux {
    /// Create new BMM150 AUX state
    pub fn new() -> Self {
        Self::default()
    }

    /// Create with specific preset
    pub fn with_preset(mut self, preset: Preset) -> Self {
        self.preset = preset;
        self
    }

    /// Create with specific preset
    pub fn with_data_rate(mut self, data_rate: DataRate) -> Self {
        self.data_rate = data_rate;
        self
    }

    /// Create with specific preset
    pub fn with_op_mode(mut self, op_mode: OpMode) -> Self {
        self.op_mode = op_mode;
        self
    }

    /// Get preset
    pub fn preset(&self) -> Preset {
        self.preset
    }

    /// Get trim data reference
    pub fn trim(&self) -> &TrimData {
        &self.trim
    }

    /// Set trim data (call after reading from device)
    pub fn set_trim(&mut self, trim: TrimData) {
        self.trim = trim;
    }

    /// Parse and compensate raw bytes to Mag [µT]
    pub fn parse_data(&self, buf: &[u8; 8]) -> Mag {
        let raw = RawData::from_bytes(buf);
        self.trim.compensate(&raw)
    }

    /// Compensate raw data to Mag [µT]
    pub fn compensate(&self, raw: &RawData) -> Mag {
        self.trim.compensate(raw)
    }
}
