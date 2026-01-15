//! INA226 Power Monitor Driver
//!
//! A `no_std` compatible driver for the Texas Instruments INA226
//! current/voltage/power monitor IC via I2C.
//!
//! # Example
//! ```ignore
//! let mut ina = Ina226::new(i2c);
//! ina.configure(
//!     Averages::Avg16,
//!     ConversionTime::Time1100us,
//!     ConversionTime::Time1100us,
//!     Mode::ShuntBusContinuous,
//! )?;
//! ina.calibrate(0.02, 8.192)?;  // 20mΩ shunt, 8.192A max
//!
//! let voltage = ina.bus_voltage()?;
//! let current = ina.current()?;
//! let power = ina.power()?;
//! ```

use embedded_hal::i2c::I2c;

/// Default I2C address for INA226
pub const DEFAULT_ADDR: u8 = 0x40;

/// Bus voltage LSB: 1.25 mV/bit
const BUS_VOLTAGE_LSB: f32 = 0.00125;

/// Shunt voltage LSB: 2.5 µV/bit
const SHUNT_VOLTAGE_LSB: f32 = 0.0000025;

/// Register addresses
mod reg {
    pub const CONFIG: u8 = 0x00;
    pub const SHUNT_VOLTAGE: u8 = 0x01;
    pub const BUS_VOLTAGE: u8 = 0x02;
    pub const POWER: u8 = 0x03;
    pub const CURRENT: u8 = 0x04;
    pub const CALIBRATION: u8 = 0x05;
    pub const MANUFACTURER_ID: u8 = 0xFE;
    pub const DIE_ID: u8 = 0xFF;
}

/// Configuration register bits
mod config {
    pub const RESET_BIT: u16 = 1 << 15;
}

/// Expected manufacturer ID (0x5449 = "TI")
pub const MANUFACTURER_ID_TI: u16 = 0x5449;

/// Expected die ID for INA226
pub const DIE_ID_INA226: u16 = 0x2260;

/// Averaging mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum Averages {
    /// 1 sample (no averaging)
    #[default]
    Avg1 = 0b000,
    /// 4 samples
    Avg4 = 0b001,
    /// 16 samples
    Avg16 = 0b010,
    /// 64 samples
    Avg64 = 0b011,
    /// 128 samples
    Avg128 = 0b100,
    /// 256 samples
    Avg256 = 0b101,
    /// 512 samples
    Avg512 = 0b110,
    /// 1024 samples
    Avg1024 = 0b111,
}

/// ADC conversion time (for both bus and shunt voltage)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum ConversionTime {
    /// 140 µs
    Time140us = 0b000,
    /// 204 µs
    Time204us = 0b001,
    /// 332 µs
    Time332us = 0b010,
    /// 588 µs
    Time588us = 0b011,
    /// 1100 µs (default)
    #[default]
    Time1100us = 0b100,
    /// 2116 µs
    Time2116us = 0b101,
    /// 4156 µs
    Time4156us = 0b110,
    /// 8244 µs
    Time8244us = 0b111,
}

/// Operating mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum Mode {
    /// Power-down
    PowerDown = 0b000,
    /// Shunt voltage, triggered
    ShuntTriggered = 0b001,
    /// Bus voltage, triggered
    BusTriggered = 0b010,
    /// Shunt and bus voltage, triggered
    ShuntBusTriggered = 0b011,
    /// ADC off (disabled)
    AdcOff = 0b100,
    /// Shunt voltage, continuous
    ShuntContinuous = 0b101,
    /// Bus voltage, continuous
    BusContinuous = 0b110,
    /// Shunt and bus voltage, continuous (default)
    #[default]
    ShuntBusContinuous = 0b111,
}

/// INA226 driver
pub struct Ina226<I2C> {
    i2c: I2C,
    addr: u8,
    current_lsb: f32,
    power_lsb: f32,
}

impl<I2C: I2c> Ina226<I2C> {
    /// Create a new INA226 driver with default address (0x40)
    pub fn new(i2c: I2C) -> Self {
        Self::new_with_addr(i2c, DEFAULT_ADDR)
    }

    /// Create a new INA226 driver with custom address
    pub fn new_with_addr(i2c: I2C, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            current_lsb: 0.0,
            power_lsb: 0.0,
        }
    }

    /// Perform a soft reset
    pub fn reset(&mut self) -> Result<(), I2C::Error> {
        self.write_reg(reg::CONFIG, config::RESET_BIT)
    }

    /// Configure the INA226
    ///
    /// # Arguments
    /// * `averages` - Number of samples to average
    /// * `bus_conv_time` - Bus voltage conversion time
    /// * `shunt_conv_time` - Shunt voltage conversion time
    /// * `mode` - Operating mode
    pub fn configure(
        &mut self,
        averages: Averages,
        bus_conv_time: ConversionTime,
        shunt_conv_time: ConversionTime,
        mode: Mode,
    ) -> Result<(), I2C::Error> {
        let config: u16 = ((averages as u16) << 9)
            | ((bus_conv_time as u16) << 6)
            | ((shunt_conv_time as u16) << 3)
            | (mode as u16);

        self.write_reg(reg::CONFIG, config)
    }

    /// Calibrate the INA226 for current and power measurements
    ///
    /// # Arguments
    /// * `r_shunt` - Shunt resistor value in ohms (e.g., 0.02 for 20mΩ)
    /// * `max_current` - Maximum expected current in amps (e.g., 8.192)
    ///
    /// # Note
    /// This must be called before reading current or power values.
    pub fn calibrate(&mut self, r_shunt: f32, max_current: f32) -> Result<(), I2C::Error> {
        // Calculate minimum LSB
        let min_lsb = max_current / 32767.0;

        // Round up to nearest 0.1mA (0.0001A) for cleaner values
        self.current_lsb = libm::ceilf(min_lsb / 0.0001) * 0.0001;

        // Power LSB is fixed at 25 * current LSB
        self.power_lsb = self.current_lsb * 25.0;

        // Calculate calibration value
        // CAL = 0.00512 / (current_lsb * r_shunt)
        let cal_value = (0.00512 / (self.current_lsb * r_shunt)) as u16;

        self.write_reg(reg::CALIBRATION, cal_value)
    }

    /// Read bus voltage in volts
    pub fn bus_voltage(&mut self) -> Result<f32, I2C::Error> {
        let raw = self.raw_bus_voltage()?;
        Ok(raw as f32 * BUS_VOLTAGE_LSB)
    }

    /// Read raw bus voltage register value
    pub fn raw_bus_voltage(&mut self) -> Result<i16, I2C::Error> {
        self.read_reg_signed(reg::BUS_VOLTAGE)
    }

    /// Read shunt voltage in volts
    pub fn shunt_voltage(&mut self) -> Result<f32, I2C::Error> {
        let raw = self.raw_shunt_voltage()?;
        Ok(raw as f32 * SHUNT_VOLTAGE_LSB)
    }

    /// Read raw shunt voltage register value
    pub fn raw_shunt_voltage(&mut self) -> Result<i16, I2C::Error> {
        self.read_reg_signed(reg::SHUNT_VOLTAGE)
    }

    /// Read current in amps
    ///
    /// # Note
    /// `calibrate()` must be called before this function returns valid values.
    pub fn current(&mut self) -> Result<f32, I2C::Error> {
        let raw = self.raw_current()?;
        Ok(raw as f32 * self.current_lsb)
    }

    /// Read raw current register value
    pub fn raw_current(&mut self) -> Result<i16, I2C::Error> {
        self.read_reg_signed(reg::CURRENT)
    }

    /// Read power in watts
    ///
    /// # Note
    /// `calibrate()` must be called before this function returns valid values.
    pub fn power(&mut self) -> Result<f32, I2C::Error> {
        let raw = self.raw_power()?;
        Ok(raw as f32 * self.power_lsb)
    }

    /// Read raw power register value
    pub fn raw_power(&mut self) -> Result<u16, I2C::Error> {
        self.read_reg(reg::POWER)
    }

    /// Read manufacturer ID (should be 0x5449 = "TI")
    pub fn manufacturer_id(&mut self) -> Result<u16, I2C::Error> {
        self.read_reg(reg::MANUFACTURER_ID)
    }

    /// Read die ID (should be 0x2260 for INA226)
    pub fn die_id(&mut self) -> Result<u16, I2C::Error> {
        self.read_reg(reg::DIE_ID)
    }

    /// Check if the device is present and responding
    pub fn is_connected(&mut self) -> Result<bool, I2C::Error> {
        let id = self.manufacturer_id()?;
        Ok(id == MANUFACTURER_ID_TI)
    }

    /// Get the current LSB value (set by calibrate)
    pub fn current_lsb(&self) -> f32 {
        self.current_lsb
    }

    /// Get the power LSB value (set by calibrate)
    pub fn power_lsb(&self) -> f32 {
        self.power_lsb
    }

    /// Read a 16-bit register (unsigned)
    fn read_reg(&mut self, reg: u8) -> Result<u16, I2C::Error> {
        let mut buf = [0u8; 2];
        self.i2c.write_read(self.addr, &[reg], &mut buf)?;
        // INA226 uses big-endian (MSB first)
        Ok(((buf[0] as u16) << 8) | (buf[1] as u16))
    }

    /// Read a 16-bit register (signed)
    fn read_reg_signed(&mut self, reg: u8) -> Result<i16, I2C::Error> {
        let raw = self.read_reg(reg)?;
        Ok(raw as i16)
    }

    /// Write a 16-bit register
    fn write_reg(&mut self, reg: u8, value: u16) -> Result<(), I2C::Error> {
        // INA226 uses big-endian (MSB first)
        let buf = [reg, (value >> 8) as u8, (value & 0xFF) as u8];
        self.i2c.write(self.addr, &buf)
    }

    /// Release the I2C bus
    pub fn release(self) -> I2C {
        self.i2c
    }
}