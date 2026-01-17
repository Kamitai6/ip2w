//! BMM150 3-Axis Magnetometer Driver
//!
//! Standalone BMM150 definitions and compensation algorithms.
//! Can be used with BMI270's AUX interface or directly via I2C.

use heapless::Vec;

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

/// Overflow ADC value for X/Y axes
const OVERFLOW_ADCVAL_XYAXES_FLIP: i16 = -4096;
/// Overflow ADC value for Z axis (hall overflow)
const OVERFLOW_ADCVAL_ZAXIS_HALL: i16 = -16384;
/// Overflow output value
const OVERFLOW_OUTPUT_FLOAT: f32 = 0.0;

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

impl RawData {
    pub fn from_bytes(buf: &[u8; 8]) -> Self {
        // X: 13-bit signed (LSB[7:3] + MSB[7:0])
        let x13 = (((buf[1] as u16) << 5) | ((buf[0] as u16) >> 3)) as i16;
        let x = (x13 << 3) >> 3; // 13-bit sign extend

        // Y: 13-bit signed
        let y13 = (((buf[3] as u16) << 5) | ((buf[2] as u16) >> 3)) as i16;
        let y = (y13 << 3) >> 3; // 13-bit sign extend

        // Z: 15-bit signed (LSB[7:1] + MSB[7:0])
        let z15 = (((buf[5] as u16) << 7) | ((buf[4] as u16) >> 1)) as i16;
        let z = (z15 << 1) >> 1; // 15-bit sign extend

        // RHALL: 14-bit unsigned (LSB[7:2] + MSB[7:0])
        let rhall = ((buf[7] as u16) << 6) | ((buf[6] as u16) >> 2);

        Self { x, y, z, rhall }
    }
}

/// Compensated magnetometer data [µT]
#[derive(Debug, Clone, Copy, Default)]
pub struct Mag {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Mag {
    /// Create new Mag from components
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Convert to array
    pub fn to_array(&self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }

    /// Create from array
    pub fn from_array(arr: [f32; 3]) -> Self {
        Self {
            x: arr[0],
            y: arr[1],
            z: arr[2],
        }
    }

    /// Apply hard-iron offset correction (deprecated, use MagCalibration)
    pub fn apply_offset(self, offset: [f32; 3]) -> Self {
        Self {
            x: self.x - offset[0],
            y: self.y - offset[1],
            z: self.z - offset[2],
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
    pub fn compensate(&self, raw: &RawData) -> Mag {
        Mag {
            x: self.compensate_x(raw.x, raw.rhall),
            y: self.compensate_y(raw.y, raw.rhall),
            z: self.compensate_z(raw.z, raw.rhall),
        }
    }

    /// Compensate X-axis data
    fn compensate_x(&self, mag_data_x: i16, data_rhall: u16) -> f32 {
        if mag_data_x == OVERFLOW_ADCVAL_XYAXES_FLIP || data_rhall == 0 || self.dig_xyz1 == 0 {
            return OVERFLOW_OUTPUT_FLOAT;
        }

        let rhall = data_rhall as f32;
        let process_comp_x0 = self.dig_xyz1 as f32 * 16384.0 / rhall;
        let retval = process_comp_x0 - 16384.0;
        let process_comp_x1 = self.dig_xy2 as f32 * retval * retval / 268435456.0;
        let process_comp_x2 = process_comp_x1 + retval * self.dig_xy1 as f32 / 16384.0;
        let process_comp_x3 = self.dig_x2 as f32 + 160.0;
        let process_comp_x4 = mag_data_x as f32 * (process_comp_x2 + 256.0) * process_comp_x3;

        ((process_comp_x4 / 8192.0) + self.dig_x1 as f32 * 8.0) / 16.0
    }

    /// Compensate Y-axis data
    fn compensate_y(&self, mag_data_y: i16, data_rhall: u16) -> f32 {
        if mag_data_y == OVERFLOW_ADCVAL_XYAXES_FLIP || data_rhall == 0 || self.dig_xyz1 == 0 {
            return OVERFLOW_OUTPUT_FLOAT;
        }

        let rhall = data_rhall as f32;
        let process_comp_y0 = self.dig_xyz1 as f32 * 16384.0 / rhall;
        let retval = process_comp_y0 - 16384.0;
        let process_comp_y1 = self.dig_xy2 as f32 * retval * retval / 268435456.0;
        let process_comp_y2 = process_comp_y1 + retval * self.dig_xy1 as f32 / 16384.0;
        let process_comp_y3 = self.dig_y2 as f32 + 160.0;
        let process_comp_y4 = mag_data_y as f32 * (process_comp_y2 + 256.0) * process_comp_y3;

        ((process_comp_y4 / 8192.0) + self.dig_y1 as f32 * 8.0) / 16.0
    }

    /// Compensate Z-axis data
    fn compensate_z(&self, mag_data_z: i16, data_rhall: u16) -> f32 {
        if mag_data_z == OVERFLOW_ADCVAL_ZAXIS_HALL
            || self.dig_z2 == 0
            || self.dig_z1 == 0
            || self.dig_xyz1 == 0
            || data_rhall == 0
        {
            return OVERFLOW_OUTPUT_FLOAT;
        }

        let rhall = data_rhall as f32;
        let process_comp_z0 = mag_data_z as f32 - self.dig_z4 as f32;
        let process_comp_z1 = rhall - self.dig_xyz1 as f32;
        let process_comp_z2 = self.dig_z3 as f32 * process_comp_z1;
        let process_comp_z3 = self.dig_z1 as f32 * rhall / 32768.0;
        let process_comp_z4 = self.dig_z2 as f32 + process_comp_z3;
        let process_comp_z5 = process_comp_z0 * 131072.0 - process_comp_z2;

        (process_comp_z5 / (process_comp_z4 * 4.0)) / 16.0
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
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_preset(mut self, preset: Preset) -> Self {
        self.preset = preset;
        self
    }

    pub fn with_data_rate(mut self, data_rate: DataRate) -> Self {
        self.data_rate = data_rate;
        self
    }

    pub fn with_op_mode(mut self, op_mode: OpMode) -> Self {
        self.op_mode = op_mode;
        self
    }

    pub fn preset(&self) -> Preset {
        self.preset
    }

    pub fn trim(&self) -> &TrimData {
        &self.trim
    }

    pub fn set_trim(&mut self, trim: TrimData) {
        self.trim = trim;
    }

    pub fn parse_data(&self, buf: &[u8; 8]) -> Mag {
        let raw = RawData::from_bytes(buf);
        self.trim.compensate(&raw)
    }

    pub fn compensate(&self, raw: &RawData) -> Mag {
        self.trim.compensate(raw)
    }
}

// ============================================================================
// Calibration
// ============================================================================

/// Maximum number of calibration samples
pub const MAX_CALIBRATION_SAMPLES: usize = 1000;

/// Default minimum distance squared for sample deduplication [µT²]
/// 2µT distance → 4µT²
pub const DEFAULT_MIN_DISTANCE_SQ: f32 = 4.0;

/// Calibration sample update result
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UpdateResult {
    /// Sample was added to buffer
    Added,
    /// Sample was skipped (too close to existing sample)
    Skipped,
    /// Buffer is full, no more samples can be added
    Full,
}

/// Magnetometer calibration parameters (hard-iron + soft-iron)
///
/// Apply to raw magnetometer readings:
/// ```ignore
/// let calib = MagCalibration::new(
///     [3.5, 11.5, 6.5],   // offset from calibrator.hard_iron_offset()
///     [1.0, 0.95, 1.05],  // scale from calibrator.soft_iron_scale()
/// );
/// let mag_corrected = calib.apply(&mag_raw);
/// ```
#[derive(Debug, Clone, Copy)]
pub struct MagCalibration {
    /// Hard-iron offset [µT]
    pub offset: [f32; 3],
    /// Soft-iron scale factors (normalized to average range)
    pub scale: [f32; 3],
}

impl Default for MagCalibration {
    fn default() -> Self {
        Self {
            offset: [0.0, 0.0, 0.0],
            scale: [1.0, 1.0, 1.0],
        }
    }
}

impl MagCalibration {
    /// Create new calibration with offset and scale
    pub fn new(offset: [f32; 3], scale: [f32; 3]) -> Self {
        Self { offset, scale }
    }

    /// Create calibration with offset only (no soft-iron correction)
    pub fn with_offset(offset: [f32; 3]) -> Self {
        Self {
            offset,
            scale: [1.0, 1.0, 1.0],
        }
    }

    /// Apply calibration to magnetometer reading
    ///
    /// Order: subtract offset first, then apply scale
    /// corrected = (raw - offset) * scale
    pub fn apply(&self, mag: &Mag) -> Mag {
        Mag {
            x: (mag.x - self.offset[0]) * self.scale[0],
            y: (mag.y - self.offset[1]) * self.scale[1],
            z: (mag.z - self.offset[2]) * self.scale[2],
        }
    }

    /// Apply calibration to array
    pub fn apply_array(&self, mag: [f32; 3]) -> [f32; 3] {
        [
            (mag[0] - self.offset[0]) * self.scale[0],
            (mag[1] - self.offset[1]) * self.scale[1],
            (mag[2] - self.offset[2]) * self.scale[2],
        ]
    }
}

/// Magnetometer calibrator with sample buffering
///
/// Collects samples for calibration with distance-based deduplication.
/// Supports both hard-iron and soft-iron (axis scaling) calibration.
///
/// # Example
///
/// ```ignore
/// let mut calibrator = MagCalibrator::new();
///
/// // Calibration loop: rotate device in all directions
/// loop {
///     let mag = bmm150.parse_data(&buf);
///     match calibrator.update(&mag) {
///         UpdateResult::Added => { /* show progress */ }
///         UpdateResult::Skipped => { /* too close to existing */ }
///         UpdateResult::Full => break,
///     }
/// }
///
/// // Get calibration parameters
/// let offset = calibrator.hard_iron_offset();
/// let scale = calibrator.soft_iron_scale();
///
/// // Display for user to copy:
/// // offset: [3.5, 11.5, 6.5]
/// // scale: [1.0, 0.95, 1.05]
///
/// // In production code, use hard-coded values:
/// let calib = MagCalibration::new(
///     [3.5, 11.5, 6.5],
///     [1.0, 0.95, 1.05],
/// );
/// let mag_corrected = calib.apply(&mag_raw);
/// ```
pub struct MagCalibrator {
    samples: Vec<[f32; 3], MAX_CALIBRATION_SAMPLES>,
    min: [f32; 3],
    max: [f32; 3],
    min_distance_sq: f32,
}

impl Default for MagCalibrator {
    fn default() -> Self {
        Self::new()
    }
}

impl MagCalibrator {
    /// Create new calibrator with default settings
    pub fn new() -> Self {
        Self {
            samples: Vec::new(),
            min: [f32::MAX, f32::MAX, f32::MAX],
            max: [f32::MIN, f32::MIN, f32::MIN],
            min_distance_sq: DEFAULT_MIN_DISTANCE_SQ,
        }
    }

    /// Create calibrator with custom minimum distance threshold
    ///
    /// # Arguments
    /// * `min_distance` - Minimum distance in µT between samples
    pub fn with_min_distance(mut self, min_distance: f32) -> Self {
        self.min_distance_sq = min_distance * min_distance;
        self
    }

    /// Reset calibrator, clearing all samples
    pub fn reset(&mut self) {
        self.samples.clear();
        self.min = [f32::MAX, f32::MAX, f32::MAX];
        self.max = [f32::MIN, f32::MIN, f32::MIN];
    }

    /// Update calibrator with new magnetometer sample
    ///
    /// The sample is only added if it's far enough from all existing samples
    /// (based on minimum distance threshold).
    pub fn update(&mut self, mag: &Mag) -> UpdateResult {
        if self.samples.is_full() {
            return UpdateResult::Full;
        }

        let point = [mag.x, mag.y, mag.z];

        // Check distance to all existing samples
        if self.is_too_close(&point) {
            return UpdateResult::Skipped;
        }

        // Update min/max
        for i in 0..3 {
            self.min[i] = self.min[i].min(point[i]);
            self.max[i] = self.max[i].max(point[i]);
        }

        // Add sample (unwrap is safe, we checked is_full above)
        let _ = self.samples.push(point);

        UpdateResult::Added
    }

    /// Check if point is too close to any existing sample
    fn is_too_close(&self, point: &[f32; 3]) -> bool {
        for sample in self.samples.iter() {
            let dx = point[0] - sample[0];
            let dy = point[1] - sample[1];
            let dz = point[2] - sample[2];
            let dist_sq = dx * dx + dy * dy + dz * dz;

            if dist_sq < self.min_distance_sq {
                return true;
            }
        }
        false
    }

    /// Get current sample count
    pub fn sample_count(&self) -> usize {
        self.samples.len()
    }

    /// Check if buffer is full
    pub fn is_full(&self) -> bool {
        self.samples.is_full()
    }

    /// Get minimum values for each axis [µT]
    pub fn min(&self) -> [f32; 3] {
        self.min
    }

    /// Get maximum values for each axis [µT]
    pub fn max(&self) -> [f32; 3] {
        self.max
    }

    /// Get range (max - min) for each axis [µT]
    pub fn range(&self) -> [f32; 3] {
        [
            self.max[0] - self.min[0],
            self.max[1] - self.min[1],
            self.max[2] - self.min[2],
        ]
    }

    /// Get current offset estimate (center of min/max bounds)
    pub fn current_offset(&self) -> [f32; 3] {
        [
            (self.min[0] + self.max[0]) / 2.0,
            (self.min[1] + self.max[1]) / 2.0,
            (self.min[2] + self.max[2]) / 2.0,
        ]
    }

    /// Get reference to collected samples (for advanced calibration)
    pub fn samples(&self) -> &[[f32; 3]] {
        &self.samples
    }

    /// Get hard-iron offset from collected samples
    ///
    /// Uses min/max method to find sphere center offset.
    pub fn hard_iron_offset(&self) -> [f32; 3] {
        self.current_offset()
    }

    /// Get soft-iron scale factors from collected samples
    ///
    /// Calculates scale factors to normalize each axis range to the average range.
    /// This corrects for ellipsoid distortion along sensor axes.
    ///
    /// Returns [1.0, 1.0, 1.0] if insufficient data or invalid ranges.
    pub fn soft_iron_scale(&self) -> [f32; 3] {
        let range = self.range();

        // Need minimum range to calculate meaningful scale
        const MIN_VALID_RANGE: f32 = 5.0; // µT

        // Check all axes have sufficient range
        if range[0] < MIN_VALID_RANGE || range[1] < MIN_VALID_RANGE || range[2] < MIN_VALID_RANGE {
            return [1.0, 1.0, 1.0];
        }

        // Calculate average range (expected sphere diameter)
        let avg_range = (range[0] + range[1] + range[2]) / 3.0;

        // Scale each axis to match average range
        // scale[i] = avg_range / range[i]
        [
            avg_range / range[0],
            avg_range / range[1],
            avg_range / range[2],
        ]
    }

    /// Get complete calibration (hard-iron + soft-iron)
    pub fn calibration(&self) -> MagCalibration {
        MagCalibration {
            offset: self.hard_iron_offset(),
            scale: self.soft_iron_scale(),
        }
    }

    /// Quality indicator: estimated sphere coverage
    ///
    /// Returns a value from 0.0 to 1.0 indicating how well the samples
    /// cover the expected sphere. Based on range consistency across axes.
    ///
    /// - 1.0: All axes have similar range (good sphere coverage)
    /// - 0.0: One or more axes have very small range (poor coverage)
    pub fn quality(&self) -> f32 {
        if self.samples.len() < 10 {
            return 0.0;
        }

        let range = self.range();
        let max_range = range[0].max(range[1]).max(range[2]);

        if max_range < 1.0 {
            return 0.0;
        }

        // Calculate ratio of min range to max range
        let min_range = range[0].min(range[1]).min(range[2]);
        let ratio = min_range / max_range;

        // Scale by sample count (more samples = higher confidence)
        let sample_factor = (self.samples.len() as f32 / 100.0).min(1.0);

        ratio * sample_factor
    }
}