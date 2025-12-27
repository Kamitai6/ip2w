//! BMI270 6-Axis IMU Driver
//!
//! A `no_std` compatible driver for the Bosch BMI270 Inertial Measurement Unit.
//! Supports accelerometer and gyroscope with configurable ranges and ODR.
//!
//! # Basic Example
//!
//! ```no_run
//! use bmi270::{Bmi270, Config, AccRange, GyrRange};
//!
//! // Create driver with I2C bus
//! let mut imu = Bmi270::new(i2c);
//!
//! // Initialize with default config
//! imu.init(|us| delay.delay_us(us)).unwrap();
//!
//! // Or with custom config
//! let config = Config {
//!     acc_range: AccRange::G8,
//!     gyr_range: GyrRange::Dps1000,
//!     ..Default::default()
//! };
//! imu.init_with_config(config, &mut |us| delay.delay_us(us)).unwrap();
//!
//! // Read data
//! let (ax, ay, az) = imu.read_accel().unwrap();  // in g
//! let (gx, gy, gz) = imu.read_gyro().unwrap();   // in °/s
//! let temp = imu.read_temperature().unwrap();    // in °C
//!
//! // Or read raw values
//! let raw = imu.read_imu_raw().unwrap();
//! println!("Accel: x={}, y={}, z={}", raw.accel.x, raw.accel.y, raw.accel.z);
//! ```
//!
//! # FOC (Fast Offset Compensation) Example
//!
//! ```no_run
//! use bmi270::{Bmi270, FocAccConfig};
//!
//! let mut imu = Bmi270::new(i2c);
//! imu.init(|us| delay.delay_us(us)).unwrap();
//!
//! // Place sensor flat with Z-axis pointing up, keep it completely still
//! // Then perform calibration:
//! imu.perform_foc(FocAccConfig::z_up(), |us| delay.delay_us(us)).unwrap();
//!
//! // Offsets are now applied automatically
//! let (ax, ay, az) = imu.read_accel().unwrap();
//! // Should read approximately (0, 0, 1) g
//!
//! // You can save offsets for next boot:
//! let acc_offset = imu.read_acc_offset();  // (i32, i32, i32)
//! let gyr_offset = imu.read_gyr_offset();
//! // Save to flash...
//!
//! // And restore later:
//! imu.write_acc_offset(acc_offset);
//! imu.write_gyr_offset(gyr_offset);
//! ```

use embedded_hal::i2c::I2c;

/// BMI270 I2C address when SDO is connected to GND
pub const ADDR_SDO_LOW: u8 = 0x68;
/// BMI270 I2C address when SDO is connected to VDD
pub const ADDR_SDO_HIGH: u8 = 0x69;
/// Default address (SDO low)
pub const DEFAULT_ADDR: u8 = ADDR_SDO_LOW;

/// BMI270 Chip ID
pub const CHIP_ID: u8 = 0x24;

/// BMI270 Register addresses
#[allow(dead_code)]
mod reg {
    pub const CHIP_ID: u8 = 0x00;
    pub const ERR_REG: u8 = 0x02;
    pub const STATUS: u8 = 0x03;
    pub const AUX_X_LSB: u8 = 0x04;
    pub const ACC_X_LSB: u8 = 0x0C;
    pub const ACC_X_MSB: u8 = 0x0D;
    pub const ACC_Y_LSB: u8 = 0x0E;
    pub const ACC_Y_MSB: u8 = 0x0F;
    pub const ACC_Z_LSB: u8 = 0x10;
    pub const ACC_Z_MSB: u8 = 0x11;
    pub const GYR_X_LSB: u8 = 0x12;
    pub const GYR_X_MSB: u8 = 0x13;
    pub const GYR_Y_LSB: u8 = 0x14;
    pub const GYR_Y_MSB: u8 = 0x15;
    pub const GYR_Z_LSB: u8 = 0x16;
    pub const GYR_Z_MSB: u8 = 0x17;
    pub const SENSORTIME_0: u8 = 0x18;
    pub const SENSORTIME_1: u8 = 0x19;
    pub const SENSORTIME_2: u8 = 0x1A;
    pub const EVENT: u8 = 0x1B;
    pub const INT_STATUS_0: u8 = 0x1C;
    pub const INT_STATUS_1: u8 = 0x1D;
    pub const INTERNAL_STATUS: u8 = 0x21;
    pub const TEMPERATURE_LSB: u8 = 0x22;
    pub const TEMPERATURE_MSB: u8 = 0x23;
    pub const FIFO_LENGTH_0: u8 = 0x24;
    pub const FIFO_LENGTH_1: u8 = 0x25;
    pub const FIFO_DATA: u8 = 0x26;
    pub const ACC_CONF: u8 = 0x40;
    pub const ACC_RANGE: u8 = 0x41;
    pub const GYR_CONF: u8 = 0x42;
    pub const GYR_RANGE: u8 = 0x43;
    pub const AUX_CONF: u8 = 0x44;
    pub const FIFO_DOWNS: u8 = 0x45;
    pub const FIFO_WTM_0: u8 = 0x46;
    pub const FIFO_WTM_1: u8 = 0x47;
    pub const FIFO_CONFIG_0: u8 = 0x48;
    pub const FIFO_CONFIG_1: u8 = 0x49;
    pub const INT1_IO_CTRL: u8 = 0x53;
    pub const INT2_IO_CTRL: u8 = 0x54;
    pub const INT_LATCH: u8 = 0x55;
    pub const INT1_MAP_FEAT: u8 = 0x56;
    pub const INT2_MAP_FEAT: u8 = 0x57;
    pub const INT_MAP_DATA: u8 = 0x58;
    pub const INIT_CTRL: u8 = 0x59;
    pub const INIT_ADDR_0: u8 = 0x5B;
    pub const INIT_ADDR_1: u8 = 0x5C;
    pub const INIT_DATA: u8 = 0x5E;
    pub const INTERNAL_ERROR: u8 = 0x5F;
    pub const IF_CONF: u8 = 0x6B;
    pub const ACC_SELF_TEST: u8 = 0x6D;
    pub const GYR_SELF_TEST: u8 = 0x6E;
    pub const NV_CONF: u8 = 0x70;
    pub const OFFSET_0: u8 = 0x71;
    pub const OFFSET_1: u8 = 0x72;
    pub const OFFSET_2: u8 = 0x73;
    pub const OFFSET_3: u8 = 0x74;
    pub const OFFSET_4: u8 = 0x75;
    pub const OFFSET_5: u8 = 0x76;
    pub const OFFSET_6: u8 = 0x77;
    pub const PWR_CONF: u8 = 0x7C;
    pub const PWR_CTRL: u8 = 0x7D;
    pub const CMD: u8 = 0x7E;
}

/// Power control register bits
#[allow(dead_code)]
mod pwr_ctrl {
    pub const AUX_EN: u8 = 0x01;
    pub const GYR_EN: u8 = 0x02;
    pub const ACC_EN: u8 = 0x04;
    pub const TEMP_EN: u8 = 0x08;
}

/// Command register values
#[allow(dead_code)]
mod cmd {
    pub const SOFT_RESET: u8 = 0xB6;
    pub const FIFO_FLUSH: u8 = 0xB0;
}

/// Accelerometer output data rate
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum AccOdr {
    /// 0.78 Hz
    Hz0_78 = 0x01,
    /// 1.56 Hz
    Hz1_56 = 0x02,
    /// 3.12 Hz
    Hz3_12 = 0x03,
    /// 6.25 Hz
    Hz6_25 = 0x04,
    /// 12.5 Hz
    Hz12_5 = 0x05,
    /// 25 Hz
    Hz25 = 0x06,
    /// 50 Hz
    Hz50 = 0x07,
    /// 100 Hz (default)
    #[default]
    Hz100 = 0x08,
    /// 200 Hz
    Hz200 = 0x09,
    /// 400 Hz
    Hz400 = 0x0A,
    /// 800 Hz
    Hz800 = 0x0B,
    /// 1600 Hz
    Hz1600 = 0x0C,
}

/// Accelerometer range
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum AccRange {
    /// ±2g
    G2 = 0x00,
    /// ±4g (default)
    #[default]
    G4 = 0x01,
    /// ±8g
    G8 = 0x02,
    /// ±16g
    G16 = 0x03,
}

impl AccRange {
    /// Get the sensitivity in LSB/g
    pub fn sensitivity(self) -> f32 {
        match self {
            AccRange::G2 => 16384.0,
            AccRange::G4 => 8192.0,
            AccRange::G8 => 4096.0,
            AccRange::G16 => 2048.0,
        }
    }
}

/// Gyroscope output data rate
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum GyrOdr {
    /// 25 Hz
    Hz25 = 0x06,
    /// 50 Hz
    Hz50 = 0x07,
    /// 100 Hz
    Hz100 = 0x08,
    /// 200 Hz (default)
    #[default]
    Hz200 = 0x09,
    /// 400 Hz
    Hz400 = 0x0A,
    /// 800 Hz
    Hz800 = 0x0B,
    /// 1600 Hz
    Hz1600 = 0x0C,
    /// 3200 Hz
    Hz3200 = 0x0D,
}

/// Gyroscope range
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum GyrRange {
    /// ±2000 °/s (default)
    #[default]
    Dps2000 = 0x00,
    /// ±1000 °/s
    Dps1000 = 0x01,
    /// ±500 °/s
    Dps500 = 0x02,
    /// ±250 °/s
    Dps250 = 0x03,
    /// ±125 °/s
    Dps125 = 0x04,
}

impl GyrRange {
    /// Get the sensitivity in LSB/(°/s)
    pub fn sensitivity(self) -> f32 {
        match self {
            GyrRange::Dps2000 => 16.4,
            GyrRange::Dps1000 => 32.8,
            GyrRange::Dps500 => 65.6,
            GyrRange::Dps250 => 131.2,
            GyrRange::Dps125 => 262.4,
        }
    }
}

/// Accelerometer bandwidth parameter
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum AccBwp {
    /// OSR4 averaging
    Osr4 = 0x00,
    /// OSR2 averaging
    Osr2 = 0x01,
    /// Normal mode (default)
    #[default]
    Normal = 0x02,
    /// CIC averaging
    Cic = 0x03,
}

/// Gyroscope bandwidth parameter
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum GyrBwp {
    /// OSR4 mode
    Osr4 = 0x00,
    /// OSR2 mode
    Osr2 = 0x01,
    /// Normal mode (default)
    #[default]
    Normal = 0x02,
}

/// Performance mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum PerfMode {
    /// CIC averaging (power optimized)
    PowerOpt = 0x00,
    /// Continuous filter (performance optimized, default)
    #[default]
    PerfOpt = 0x01,
}

// ============================================================================
// FOC Types
// ============================================================================

/// Accelerometer FOC axis configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FocAccAxis {
    /// Disable FOC for this axis
    Disable,
    /// Axis pointing to +1g (e.g., Z-up)
    Plus1g,
    /// Axis pointing to -1g (e.g., Z-down)
    Minus1g,
    /// Axis pointing to 0g (horizontal)
    Zero,
}

/// FOC configuration for accelerometer
///
/// Specify the expected orientation during calibration.
/// For example, if the sensor is flat with Z pointing up:
/// - x_axis: Zero (horizontal)
/// - y_axis: Zero (horizontal)  
/// - z_axis: Plus1g (pointing up against gravity)
#[derive(Debug, Clone, Copy)]
pub struct FocAccConfig {
    pub x_axis: FocAccAxis,
    pub y_axis: FocAccAxis,
    pub z_axis: FocAccAxis,
}

impl FocAccConfig {
    /// Configuration for sensor flat, Z-axis up
    pub fn z_up() -> Self {
        Self {
            x_axis: FocAccAxis::Zero,
            y_axis: FocAccAxis::Zero,
            z_axis: FocAccAxis::Plus1g,
        }
    }

    /// Configuration for sensor flat, Z-axis down
    pub fn z_down() -> Self {
        Self {
            x_axis: FocAccAxis::Zero,
            y_axis: FocAccAxis::Zero,
            z_axis: FocAccAxis::Minus1g,
        }
    }
}

// ============================================================================
// Data Types
// ============================================================================

/// Raw accelerometer data
#[derive(Debug, Clone, Copy, Default)]
pub struct AccelData {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// Raw gyroscope data
#[derive(Debug, Clone, Copy, Default)]
pub struct GyroData {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// Combined IMU data
#[derive(Debug, Clone, Copy, Default)]
pub struct ImuData {
    pub accel: AccelData,
    pub gyro: GyroData,
}

/// Driver errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    /// I2C communication error
    I2c(E),
    /// Invalid chip ID
    InvalidChipId(u8),
    /// Initialization failed
    InitFailed,
    /// Configuration error
    ConfigError,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::I2c(e)
    }
}

/// BMI270 driver configuration
#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub acc_odr: AccOdr,
    pub acc_range: AccRange,
    pub acc_bwp: AccBwp,
    pub gyr_odr: GyrOdr,
    pub gyr_range: GyrRange,
    pub gyr_bwp: GyrBwp,
    pub perf_mode: PerfMode,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            acc_odr: AccOdr::default(),
            acc_range: AccRange::default(),
            acc_bwp: AccBwp::default(),
            gyr_odr: GyrOdr::default(),
            gyr_range: GyrRange::default(),
            gyr_bwp: GyrBwp::default(),
            perf_mode: PerfMode::default(),
        }
    }
}

/// BMI270 driver
pub struct Bmi270<I2C> {
    i2c: I2C,
    addr: u8,
    acc_range: AccRange,
    gyr_range: GyrRange,
    // Software offset compensation
    acc_offset: (i32, i32, i32),
    gyr_offset: (i32, i32, i32),
}

impl<I2C: I2c> Bmi270<I2C> {
    /// Create a new BMI270 driver with default address (0x68)
    pub fn new(i2c: I2C) -> Self {
        Self::new_with_addr(i2c, DEFAULT_ADDR)
    }

    /// Create a new BMI270 driver with custom address
    pub fn new_with_addr(i2c: I2C, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            acc_range: AccRange::default(),
            gyr_range: GyrRange::default(),
            acc_offset: (0, 0, 0),
            gyr_offset: (0, 0, 0),
        }
    }

    /// Initialize the BMI270 with default configuration
    ///
    /// This performs a soft reset, verifies chip ID, loads the config file,
    /// and configures accelerometer and gyroscope with default settings.
    ///
    /// # Arguments
    /// * `delay` - Delay provider for timing requirements
    pub fn init<D: FnMut(u32)>(&mut self, mut delay: D) -> Result<(), Error<I2C::Error>> {
        self.init_with_config(Config::default(), &mut delay)
    }

    /// Initialize the BMI270 with custom configuration
    pub fn init_with_config<D: FnMut(u32)>(
        &mut self,
        config: Config,
        delay: &mut D,
    ) -> Result<(), Error<I2C::Error>> {
        // Soft reset
        self.soft_reset()?;
        delay(2000); // Wait 2ms after reset

        // Verify chip ID
        let chip_id = self.read_chip_id()?;
        if chip_id != CHIP_ID {
            return Err(Error::InvalidChipId(chip_id));
        }

        // Disable advanced power save mode for config loading
        self.write_reg(reg::PWR_CONF, 0x00)?;
        delay(500);

        // Load configuration file
        self.load_config_file(delay)?;

        // Verify initialization status
        delay(20000); // Wait for initialization
        let status = self.read_reg(reg::INTERNAL_STATUS)?;
        if (status & 0x0F) != 0x01 {
            return Err(Error::InitFailed);
        }

        // Apply configuration
        self.configure(config)?;

        // Enable accelerometer and gyroscope
        self.write_reg(reg::PWR_CTRL, pwr_ctrl::ACC_EN | pwr_ctrl::GYR_EN)?;
        delay(500);

        // Set normal power mode
        self.write_reg(reg::PWR_CONF, 0x00)?;

        self.acc_range = config.acc_range;
        self.gyr_range = config.gyr_range;

        Ok(())
    }

    /// Load BMI270 configuration file
    fn load_config_file<D: FnMut(u32)>(&mut self, delay: &mut D) -> Result<(), Error<I2C::Error>> {
        // Prepare for config load
        self.write_reg(reg::INIT_CTRL, 0x00)?;

        // Write config file in bursts
        let config = &BMI270_CONFIG_FILE;
        let mut index: u16 = 0;

        while (index as usize) < config.len() {
            // Set init address (in words, divide by 2)
            let addr_word = index / 2;
            self.write_reg(reg::INIT_ADDR_0, (addr_word & 0x0F) as u8)?;
            self.write_reg(reg::INIT_ADDR_1, ((addr_word >> 4) & 0xFF) as u8)?;

            // Write burst of data (max 32 bytes at a time for I2C)
            let burst_len = core::cmp::min(32, config.len() - index as usize);
            let mut burst = [0u8; 33]; // reg addr + data
            burst[0] = reg::INIT_DATA;
            burst[1..=burst_len].copy_from_slice(&config[index as usize..index as usize + burst_len]);
            self.i2c.write(self.addr, &burst[..=burst_len])?;

            index += burst_len as u16;
        }

        // Complete config load
        self.write_reg(reg::INIT_CTRL, 0x01)?;
        delay(200000); // Wait 200ms for initialization

        Ok(())
    }

    /// Apply sensor configuration
    fn configure(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
        // Configure accelerometer
        let acc_conf = (config.perf_mode as u8) << 7
            | (config.acc_bwp as u8) << 4
            | (config.acc_odr as u8);
        self.write_reg(reg::ACC_CONF, acc_conf)?;
        self.write_reg(reg::ACC_RANGE, config.acc_range as u8)?;

        // Configure gyroscope
        let gyr_conf = (config.perf_mode as u8) << 7
            | (config.gyr_bwp as u8) << 4
            | (config.gyr_odr as u8);
        self.write_reg(reg::GYR_CONF, gyr_conf)?;
        self.write_reg(reg::GYR_RANGE, config.gyr_range as u8)?;

        Ok(())
    }

    /// Perform software reset
    pub fn soft_reset(&mut self) -> Result<(), I2C::Error> {
        self.write_reg(reg::CMD, cmd::SOFT_RESET)
    }

    /// Read chip ID (should return 0x24)
    pub fn read_chip_id(&mut self) -> Result<u8, I2C::Error> {
        self.read_reg(reg::CHIP_ID)
    }

    /// Read raw accelerometer data (with offset compensation applied)
    pub fn read_accel_raw(&mut self) -> Result<AccelData, I2C::Error> {
        let raw = self.read_accel_raw_internal()?;
        Ok(AccelData {
            x: (raw.x as i32 - self.acc_offset.0) as i16,
            y: (raw.y as i32 - self.acc_offset.1) as i16,
            z: (raw.z as i32 - self.acc_offset.2) as i16,
        })
    }

    /// Read raw accelerometer data (without offset compensation)
    fn read_accel_raw_internal(&mut self) -> Result<AccelData, I2C::Error> {
        let mut buf = [0u8; 6];
        self.read_regs(reg::ACC_X_LSB, &mut buf)?;

        Ok(AccelData {
            x: i16::from_le_bytes([buf[0], buf[1]]),
            y: i16::from_le_bytes([buf[2], buf[3]]),
            z: i16::from_le_bytes([buf[4], buf[5]]),
        })
    }

    /// Read raw gyroscope data (with offset compensation applied)
    pub fn read_gyro_raw(&mut self) -> Result<GyroData, I2C::Error> {
        let raw = self.read_gyro_raw_internal()?;
        Ok(GyroData {
            x: (raw.x as i32 - self.gyr_offset.0) as i16,
            y: (raw.y as i32 - self.gyr_offset.1) as i16,
            z: (raw.z as i32 - self.gyr_offset.2) as i16,
        })
    }

    /// Read raw gyroscope data (without offset compensation)
    fn read_gyro_raw_internal(&mut self) -> Result<GyroData, I2C::Error> {
        let mut buf = [0u8; 6];
        self.read_regs(reg::GYR_X_LSB, &mut buf)?;

        Ok(GyroData {
            x: i16::from_le_bytes([buf[0], buf[1]]),
            y: i16::from_le_bytes([buf[2], buf[3]]),
            z: i16::from_le_bytes([buf[4], buf[5]]),
        })
    }

    /// Read both accelerometer and gyroscope data (with offset compensation)
    pub fn read_imu_raw(&mut self) -> Result<ImuData, I2C::Error> {
        let mut buf = [0u8; 12];
        self.read_regs(reg::ACC_X_LSB, &mut buf)?;

        Ok(ImuData {
            accel: AccelData {
                x: (i16::from_le_bytes([buf[0], buf[1]]) as i32 - self.acc_offset.0) as i16,
                y: (i16::from_le_bytes([buf[2], buf[3]]) as i32 - self.acc_offset.1) as i16,
                z: (i16::from_le_bytes([buf[4], buf[5]]) as i32 - self.acc_offset.2) as i16,
            },
            gyro: GyroData {
                x: (i16::from_le_bytes([buf[6], buf[7]]) as i32 - self.gyr_offset.0) as i16,
                y: (i16::from_le_bytes([buf[8], buf[9]]) as i32 - self.gyr_offset.1) as i16,
                z: (i16::from_le_bytes([buf[10], buf[11]]) as i32 - self.gyr_offset.2) as i16,
            },
        })
    }

    /// Read accelerometer data in g
    pub fn read_accel(&mut self) -> Result<(f32, f32, f32), I2C::Error> {
        let raw = self.read_accel_raw()?;
        let sens = self.acc_range.sensitivity();
        Ok((
            raw.x as f32 / sens,
            raw.y as f32 / sens,
            raw.z as f32 / sens,
        ))
    }

    /// Read gyroscope data in degrees per second
    pub fn read_gyro(&mut self) -> Result<(f32, f32, f32), I2C::Error> {
        let raw = self.read_gyro_raw()?;
        let sens = self.gyr_range.sensitivity();
        Ok((
            raw.x as f32 / sens,
            raw.y as f32 / sens,
            raw.z as f32 / sens,
        ))
    }

    /// Read temperature in Celsius
    pub fn read_temperature(&mut self) -> Result<f32, I2C::Error> {
        let mut buf = [0u8; 2];
        self.read_regs(reg::TEMPERATURE_LSB, &mut buf)?;
        let raw = i16::from_le_bytes([buf[0], buf[1]]);
        // Temperature formula from datasheet
        Ok(raw as f32 / 512.0 + 23.0)
    }

    /// Read sensor time (24-bit counter at 39.0625 µs resolution)
    pub fn read_sensor_time(&mut self) -> Result<u32, I2C::Error> {
        let mut buf = [0u8; 3];
        self.read_regs(reg::SENSORTIME_0, &mut buf)?;
        Ok(u32::from_le_bytes([buf[0], buf[1], buf[2], 0]))
    }

    /// Check if data is ready
    pub fn data_ready(&mut self) -> Result<bool, I2C::Error> {
        let status = self.read_reg(reg::STATUS)?;
        Ok((status & 0xC0) == 0xC0) // ACC and GYR data ready
    }

    /// Enable accelerometer
    pub fn enable_accel(&mut self) -> Result<(), I2C::Error> {
        let pwr = self.read_reg(reg::PWR_CTRL)?;
        self.write_reg(reg::PWR_CTRL, pwr | pwr_ctrl::ACC_EN)
    }

    /// Disable accelerometer
    pub fn disable_accel(&mut self) -> Result<(), I2C::Error> {
        let pwr = self.read_reg(reg::PWR_CTRL)?;
        self.write_reg(reg::PWR_CTRL, pwr & !pwr_ctrl::ACC_EN)
    }

    /// Enable gyroscope
    pub fn enable_gyro(&mut self) -> Result<(), I2C::Error> {
        let pwr = self.read_reg(reg::PWR_CTRL)?;
        self.write_reg(reg::PWR_CTRL, pwr | pwr_ctrl::GYR_EN)
    }

    /// Disable gyroscope
    pub fn disable_gyro(&mut self) -> Result<(), I2C::Error> {
        let pwr = self.read_reg(reg::PWR_CTRL)?;
        self.write_reg(reg::PWR_CTRL, pwr & !pwr_ctrl::GYR_EN)
    }

    /// Set accelerometer range
    pub fn set_acc_range(&mut self, range: AccRange) -> Result<(), I2C::Error> {
        self.write_reg(reg::ACC_RANGE, range as u8)?;
        self.acc_range = range;
        Ok(())
    }

    /// Set gyroscope range
    pub fn set_gyr_range(&mut self, range: GyrRange) -> Result<(), I2C::Error> {
        self.write_reg(reg::GYR_RANGE, range as u8)?;
        self.gyr_range = range;
        Ok(())
    }

    /// Get current accelerometer range
    pub fn acc_range(&self) -> AccRange {
        self.acc_range
    }

    /// Get current gyroscope range
    pub fn gyr_range(&self) -> GyrRange {
        self.gyr_range
    }

    // ========================================================================
    // FOC (Fast Offset Compensation) Methods
    // ========================================================================

    /// Perform accelerometer offset calibration (software-based)
    ///
    /// This performs a software-based calibration by averaging multiple samples
    /// and storing offset values internally. Offsets are applied automatically
    /// when reading accelerometer data.
    ///
    /// The sensor must be stationary in a known orientation during calibration.
    ///
    /// # Example
    /// ```no_run
    /// // Place sensor flat with Z-axis pointing up, then:
    /// imu.perform_acc_foc(FocAccConfig::z_up(), |us| delay.delay_us(us))?;
    /// ```
    ///
    /// # Requirements
    /// - Sensor must be stationary
    /// - Accelerometer must be enabled
    /// - Takes ~500ms to complete (50 samples at 100Hz)
    pub fn perform_acc_foc<D: FnMut(u32)>(
        &mut self,
        config: FocAccConfig,
        mut delay: D,
    ) -> Result<(), Error<I2C::Error>> {
        const NUM_SAMPLES: i32 = 50;
        
        // Ensure accelerometer is enabled
        let pwr = self.read_reg(reg::PWR_CTRL)?;
        if (pwr & pwr_ctrl::ACC_EN) == 0 {
            self.write_reg(reg::PWR_CTRL, pwr | pwr_ctrl::ACC_EN)?;
            delay(50000); // Wait for accel to start
        }

        // Disable advanced power save
        self.write_reg(reg::PWR_CONF, 0x00)?;
        delay(1000);

        // Expected values based on orientation (in LSB for current range)
        let sensitivity = self.acc_range.sensitivity();
        let expected_x: i32 = match config.x_axis {
            FocAccAxis::Plus1g => sensitivity as i32,
            FocAccAxis::Minus1g => -(sensitivity as i32),
            _ => 0,
        };
        let expected_y: i32 = match config.y_axis {
            FocAccAxis::Plus1g => sensitivity as i32,
            FocAccAxis::Minus1g => -(sensitivity as i32),
            _ => 0,
        };
        let expected_z: i32 = match config.z_axis {
            FocAccAxis::Plus1g => sensitivity as i32,
            FocAccAxis::Minus1g => -(sensitivity as i32),
            _ => 0,
        };

        // Collect samples and compute average
        let mut sum_x: i32 = 0;
        let mut sum_y: i32 = 0;
        let mut sum_z: i32 = 0;

        for _ in 0..NUM_SAMPLES {
            delay(10000); // 10ms between samples
            let raw = self.read_accel_raw_internal()?;
            sum_x += raw.x as i32;
            sum_y += raw.y as i32;
            sum_z += raw.z as i32;
        }

        let avg_x = sum_x / NUM_SAMPLES;
        let avg_y = sum_y / NUM_SAMPLES;
        let avg_z = sum_z / NUM_SAMPLES;

        // Store offset (what to subtract to get expected value)
        self.acc_offset = (avg_x - expected_x, avg_y - expected_y, avg_z - expected_z);

        Ok(())
    }

    /// Perform gyroscope offset calibration (software-based)
    ///
    /// This performs a software-based calibration by averaging multiple samples
    /// and storing offset values internally. Offsets are applied automatically
    /// when reading gyroscope data.
    ///
    /// The sensor must be completely stationary during calibration.
    ///
    /// # Example
    /// ```no_run
    /// // Keep sensor completely still, then:
    /// imu.perform_gyr_foc(|us| delay.delay_us(us))?;
    /// ```
    ///
    /// # Requirements
    /// - Sensor must be completely stationary (no vibration)
    /// - Gyroscope must be enabled
    /// - Takes ~500ms to complete
    pub fn perform_gyr_foc<D: FnMut(u32)>(
        &mut self,
        mut delay: D,
    ) -> Result<(), Error<I2C::Error>> {
        const NUM_SAMPLES: i32 = 50;

        // Ensure gyroscope is enabled
        let pwr = self.read_reg(reg::PWR_CTRL)?;
        if (pwr & pwr_ctrl::GYR_EN) == 0 {
            self.write_reg(reg::PWR_CTRL, pwr | pwr_ctrl::GYR_EN)?;
            delay(100000); // Wait for gyro to start (longer than accel)
        }

        // Disable advanced power save
        self.write_reg(reg::PWR_CONF, 0x00)?;
        delay(1000);

        // Collect samples and compute average
        let mut sum_x: i32 = 0;
        let mut sum_y: i32 = 0;
        let mut sum_z: i32 = 0;

        for _ in 0..NUM_SAMPLES {
            delay(10000); // 10ms between samples
            let raw = self.read_gyro_raw_internal()?;
            sum_x += raw.x as i32;
            sum_y += raw.y as i32;
            sum_z += raw.z as i32;
        }

        // Store offset (gyro should read 0 when stationary)
        self.gyr_offset = (sum_x / NUM_SAMPLES, sum_y / NUM_SAMPLES, sum_z / NUM_SAMPLES);

        Ok(())
    }

    /// Perform both accelerometer and gyroscope offset calibration
    ///
    /// Convenience method to calibrate both sensors at once.
    /// Sensor must be stationary in the specified orientation.
    pub fn perform_foc<D: FnMut(u32)>(
        &mut self,
        acc_config: FocAccConfig,
        mut delay: D,
    ) -> Result<(), Error<I2C::Error>> {
        self.perform_acc_foc(acc_config, &mut delay)?;
        self.perform_gyr_foc(&mut delay)?;
        Ok(())
    }

    /// Read current accelerometer offset values (software offset)
    pub fn read_acc_offset(&self) -> (i32, i32, i32) {
        self.acc_offset
    }

    /// Write accelerometer offset values (software offset)
    ///
    /// Use this to restore previously calibrated offsets without running FOC again.
    pub fn write_acc_offset(&mut self, offset: (i32, i32, i32)) {
        self.acc_offset = offset;
    }

    /// Read current gyroscope offset values (software offset)
    pub fn read_gyr_offset(&self) -> (i32, i32, i32) {
        self.gyr_offset
    }

    /// Write gyroscope offset values (software offset)
    ///
    /// Use this to restore previously calibrated offsets without running FOC again.
    pub fn write_gyr_offset(&mut self, offset: (i32, i32, i32)) {
        self.gyr_offset = offset;
    }

    // ========================================================================
    // Low-level register access
    // ========================================================================

    /// Read a single register
    pub fn read_reg(&mut self, reg: u8) -> Result<u8, I2C::Error> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(self.addr, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    /// Read multiple registers
    pub fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), I2C::Error> {
        self.i2c.write_read(self.addr, &[reg], buf)
    }

    /// Write a single register
    pub fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.addr, &[reg, value])
    }

    /// Release the I2C bus
    pub fn release(self) -> I2C {
        self.i2c
    }
}

/// BMI270 configuration file (from Bosch official library)
/// This is the "maximum FIFO" variant configuration
static BMI270_CONFIG_FILE: [u8; 328] = [
    0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x1a, 0x00, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e,
    0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e,
    0x90, 0x32, 0x21, 0x2e, 0x59, 0xf5, 0x10, 0x30, 0x21, 0x2e, 0x6a, 0xf5, 0x1a, 0x24, 0x22, 0x00,
    0x80, 0x2e, 0x3b, 0x00, 0xc8, 0x2e, 0x44, 0x47, 0x22, 0x00, 0x37, 0x00, 0xa4, 0x00, 0xff, 0x0f,
    0xd1, 0x00, 0x07, 0xad, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
    0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
    0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x24, 0xfc, 0xf5, 0x80, 0x30, 0x40, 0x42, 0x50, 0x50,
    0x00, 0x30, 0x12, 0x24, 0xeb, 0x00, 0x03, 0x30, 0x00, 0x2e, 0xc1, 0x86, 0x5a, 0x0e, 0xfb, 0x2f,
    0x21, 0x2e, 0xfc, 0xf5, 0x13, 0x24, 0x63, 0xf5, 0xe0, 0x3c, 0x48, 0x00, 0x22, 0x30, 0xf7, 0x80,
    0xc2, 0x42, 0xe1, 0x7f, 0x3a, 0x25, 0xfc, 0x86, 0xf0, 0x7f, 0x41, 0x33, 0x98, 0x2e, 0xc2, 0xc4,
    0xd6, 0x6f, 0xf1, 0x30, 0xf1, 0x08, 0xc4, 0x6f, 0x11, 0x24, 0xff, 0x03, 0x12, 0x24, 0x00, 0xfc,
    0x61, 0x09, 0xa2, 0x08, 0x36, 0xbe, 0x2a, 0xb9, 0x13, 0x24, 0x38, 0x00, 0x64, 0xbb, 0xd1, 0xbe,
    0x94, 0x0a, 0x71, 0x08, 0xd5, 0x42, 0x21, 0xbd, 0x91, 0xbc, 0xd2, 0x42, 0xc1, 0x42, 0x00, 0xb2,
    0xfe, 0x82, 0x05, 0x2f, 0x50, 0x30, 0x21, 0x2e, 0x21, 0xf2, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e,
    0xf0, 0x6f, 0x02, 0x30, 0x02, 0x42, 0x20, 0x26, 0xe0, 0x6f, 0x02, 0x31, 0x03, 0x40, 0x9a, 0x0a,
    0x02, 0x42, 0xf0, 0x37, 0x05, 0x2e, 0x5e, 0xf7, 0x10, 0x08, 0x12, 0x24, 0x1e, 0xf2, 0x80, 0x42,
    0x83, 0x84, 0xf1, 0x7f, 0x0a, 0x25, 0x13, 0x30, 0x83, 0x42, 0x3b, 0x82, 0xf0, 0x6f, 0x00, 0x2e,
    0x00, 0x2e, 0xd0, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x00, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x3e, 0x84,
    0x00, 0x40, 0x40, 0x42, 0x7e, 0x82, 0xe1, 0x7f, 0xf2, 0x7f, 0x98, 0x2e, 0x6a, 0xd6, 0x21, 0x30,
    0x23, 0x2e, 0x61, 0xf5, 0xeb, 0x2c, 0xe1, 0x6f,
];