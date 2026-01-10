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

/// Compensated magnetometer data [16LSB/µT]
#[derive(Debug, Clone, Copy, Default)]
pub struct MagData {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// Compensated magnetometer data [µT]
#[derive(Debug, Clone, Copy, Default)]
pub struct Mag {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Overflow ADC value for X/Y axes
const OVERFLOW_ADCVAL_XYAXES_FLIP: i16 = -4096;
/// Overflow ADC value for Z axis (hall overflow)
const OVERFLOW_ADCVAL_ZAXIS_HALL: i16 = -16384;
/// Overflow output value
const OVERFLOW_OUTPUT: i16 = -32768;
const OVERFLOW_OUTPUT_FLOAT: f32 = 0.0;

const NEGATIVE_SATURATION_Z: i16 = -32767;
const POSITIVE_SATURATION_Z: i16 = 32767;

impl RawData {
    pub fn from_bytes(buf: &[u8; 8]) -> Self {
        // X: 13-bit signed (LSB[7:3] + MSB[7:0])
        let x_lsb = ((buf[0] & 0xF8) >> 3) as u16; // bit[7:3]
        let x_msb = buf[1] as u16;
        let x_raw = (x_msb << 5) | x_lsb;
        let x = if (x_raw & 0x1000) != 0 {
            (x_raw | 0xE000) as i16 // sign-extend 13->16
        } else {
            x_raw as i16
        };

        // Y: 13-bit signed
        let y_lsb = ((buf[2] & 0xF8) >> 3) as u16;
        let y_msb = buf[3] as u16;
        let y_raw = (y_msb << 5) | y_lsb;
        let y = if (y_raw & 0x1000) != 0 {
            (y_raw | 0xE000) as i16
        } else {
            y_raw as i16
        };

        // Z: 15-bit signed (LSB[7:1] + MSB[7:0])
        let z_lsb = ((buf[4] & 0xFE) >> 1) as u16; // bit[7:1]
        let z_msb = buf[5] as u16;
        let z_raw = (z_msb << 7) | z_lsb;
        let z = if (z_raw & 0x4000) != 0 {
            (z_raw | 0x8000) as i16 // sign-extend 15->16
        } else {
            z_raw as i16
        };

        // RHALL: 14-bit unsigned (LSB[7:2] + MSB[7:0])
        let rhall_lsb = ((buf[6] & 0xFC) >> 2) as u16; // bit[7:2]
        let rhall_msb = buf[7] as u16;
        let rhall = (rhall_msb << 6) | rhall_lsb;

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
            dig_xyz1: u16::from_le_bytes([dig_xyz1_lsb, dig_xyz1_msb & 0x7F]),
        }
    }

    /// Apply compensation to raw data, returns [µT]
    /// Based on official Bosch BMM150_USE_FLOATING_POINT implementation
    pub fn compensate(&self, raw: &RawData) -> Mag {
        Mag {
            x: self.compensate_x(raw.x, raw.rhall) as f32,
            y: self.compensate_y(raw.y, raw.rhall) as f32,
            z: self.compensate_z(raw.z, raw.rhall) as f32,
        }
    }

    /// Compensate X-axis data
    fn compensate_x(&self, mag_data_x: i16, data_rhall: u16) -> i16 {
        // Overflow condition check
        if mag_data_x == OVERFLOW_ADCVAL_XYAXES_FLIP {
            return OVERFLOW_OUTPUT;
        }

        let process_comp_x0 = if data_rhall != 0 {
            data_rhall
        }
        else if self.dig_xyz1 != 0 {
            self.dig_xyz1
        }
        else {
            0
        };

        if process_comp_x0 == 0 {
            return OVERFLOW_OUTPUT;
        }

        // Processing compensation equations
        let process_comp_x1 = self.dig_xyz1 as i32 * 16384 / process_comp_x0 as i32;
        let process_comp_x2 = process_comp_x1 as u16 - 0x4000_u16;
        let retval = process_comp_x2 as i16;
        let process_comp_x3 = retval as i32 * retval as i32;
        let process_comp_x4 = self.dig_xy2 as i32 * (process_comp_x3 / 128);
        let process_comp_x5 = (self.dig_xy1 as i16 * 128) as i32;
        let process_comp_x6 = retval as i32 * process_comp_x5;
        let process_comp_x7 = (process_comp_x4 + process_comp_x6) / 512 + 0x100000_i32;
        let process_comp_x8 = (self.dig_x2 as i16 + 0xA0_i16) as i32;
        let process_comp_x9 = (process_comp_x7 * process_comp_x8) / 4096;
        let process_comp_x10 = mag_data_x as i32 * process_comp_x9;
        let retval = (process_comp_x10 / 8192) as i16;

        (retval + (self.dig_x1 as i16 * 8)) / 16
    }

    /// Compensate Y-axis data
    fn compensate_y(&self, mag_data_y: i16, data_rhall: u16) -> i16 {
        // Overflow condition check
        if mag_data_y == OVERFLOW_ADCVAL_XYAXES_FLIP {
            return OVERFLOW_OUTPUT;
        }

        let process_comp_y0 = if data_rhall != 0 {
            data_rhall
        }
        else if self.dig_xyz1 != 0 {
            self.dig_xyz1
        }
        else {
            0
        };

        if process_comp_y0 == 0 {
            return OVERFLOW_OUTPUT;
        }

        // Processing compensation equations
        let process_comp_y1 = self.dig_xyz1 as i32 * 16384 / process_comp_y0 as i32;
        let process_comp_y2 = process_comp_y1 as u16 - 0x4000_u16;
        let retval = process_comp_y2 as i16;
        let process_comp_y3 = retval as i32 * retval as i32;
        let process_comp_y4 = self.dig_xy2 as i32 * (process_comp_y3 / 128);
        let process_comp_y5 = (self.dig_xy1 as i16 * 128) as i32;
        let process_comp_y6 = retval as i32 * process_comp_y5;
        let process_comp_y7 = (process_comp_y4 + process_comp_y6) / 512 + 0x100000_i32;
        let process_comp_y8 = (self.dig_y2 as i16 + 0xA0_i16) as i32;
        let process_comp_y9 = (process_comp_y7 * process_comp_y8) / 4096;
        let process_comp_y10 = mag_data_y as i32 * process_comp_y9;
        let retval = (process_comp_y10 / 8192) as i16;

        (retval + (self.dig_y1 as i16 * 8)) / 16
    }

    /// Compensate Z-axis data
    fn compensate_z(&self, mag_data_z: i16, data_rhall: u16) -> i16 {
        // Overflow condition check
        if mag_data_z == OVERFLOW_ADCVAL_ZAXIS_HALL {
            return OVERFLOW_OUTPUT;
        }

        if self.dig_z2 == 0 || self.dig_z1 == 0 || self.dig_xyz1 == 0 || data_rhall == 0 {
            return OVERFLOW_OUTPUT;
        }

        // Processing compensation equations
        let process_comp_z0 = data_rhall as i16 - self.dig_xyz1 as i16;
        let process_comp_z1 = self.dig_z3 as i32 * process_comp_z0 as i32 / 4;
        let process_comp_z2 = (mag_data_z - self.dig_z4) as i32 * 32768;
        let process_comp_z3 = self.dig_z1 as i32 * data_rhall as i32 * 2;
        let process_comp_z4 = ((process_comp_z3 + 32768) / 65536) as i16;
        let retval = (process_comp_z2 - process_comp_z1) / (self.dig_z2 + process_comp_z4) as i32;

        ((retval / 16) as i16).clamp(NEGATIVE_SATURATION_Z, POSITIVE_SATURATION_Z)
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
        defmt::info!(
            "buf: {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}",
            buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]
        );
        let raw = RawData::from_bytes(buf);
        defmt::info!("Raw: x={}, y={}, z={}, rhall={}", raw.x, raw.y, raw.z, raw.rhall);
        defmt::info!("Trim: x1:{}, y1:{}, x2:{}, y2:{}, z1:{}, z2:{}, z3:{}, z4:{}, xy1:{}, xy2:{}, xyz1:{}", 
            self.trim.dig_x1, self.trim.dig_y1, self.trim.dig_x2, self.trim.dig_y2, self.trim.dig_z1, self.trim.dig_z2, 
            self.trim.dig_z3, self.trim.dig_z4, self.trim.dig_xy1, self.trim.dig_xy2, self.trim.dig_xyz1);
        self.trim.compensate(&raw)
    }

    /// Compensate raw data to Mag [µT]
    pub fn compensate(&self, raw: &RawData) -> Mag {
        self.trim.compensate(raw)
    }
}
