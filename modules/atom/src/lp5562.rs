//! LP5562 RGB LED Driver
//!
//! A `no_std` compatible driver for the Texas Instruments LP5562 LED driver.

use embedded_hal::i2c::I2c;

/// LP5562 I2C address (when ADDR pin is low)
pub const DEFAULT_ADDR: u8 = 0x30;

/// LP5562 Register addresses
#[allow(dead_code)]
mod reg {
    pub const ENABLE: u8 = 0x00;
    pub const OP_MODE: u8 = 0x01;
    pub const B_PWM: u8 = 0x02;
    pub const G_PWM: u8 = 0x03;
    pub const R_PWM: u8 = 0x04;
    pub const B_CURRENT: u8 = 0x05;
    pub const G_CURRENT: u8 = 0x06;
    pub const R_CURRENT: u8 = 0x07;
    pub const CONFIG: u8 = 0x08;
    pub const ENG1_PC: u8 = 0x09;
    pub const ENG2_PC: u8 = 0x0A;
    pub const ENG3_PC: u8 = 0x0B;
    pub const STATUS: u8 = 0x0C;
    pub const RESET: u8 = 0x0D;
    pub const W_PWM: u8 = 0x0E;
    pub const W_CURRENT: u8 = 0x0F;
    pub const LED_MAP: u8 = 0x70;
}

/// Enable register bits
#[allow(dead_code)]
mod enable {
    pub const CHIP_EN: u8 = 0x40;
    pub const LOG_EN: u8 = 0x80;
}

/// LED channel selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Channel {
    Red,
    Green,
    Blue,
    White,
}

/// LP5562 driver
pub struct Lp5562<I2C> {
    i2c: I2C,
    addr: u8,
}

impl<I2C: I2c> Lp5562<I2C> {
    /// Create a new LP5562 driver with default address (0x30)
    pub fn new(i2c: I2C) -> Self {
        Self::new_with_addr(i2c, DEFAULT_ADDR)
    }

    /// Create a new LP5562 driver with custom address
    pub fn new_with_addr(i2c: I2C, addr: u8) -> Self {
        Self { i2c, addr }
    }

    /// Initialize the LP5562 with default settings
    pub fn init(&mut self) -> Result<(), I2C::Error> {
        // Reset the device
        self.reset()?;
        // Enable chip
        self.enable()?;
        // Use internal clock, set PWM frequency to 558Hz
        self.write_reg(reg::CONFIG, 0x01)?;
        // Map all LEDs to direct PWM control
        self.write_reg(reg::LED_MAP, 0x00)?;
        Ok(())
    }

    /// Software reset
    pub fn reset(&mut self) -> Result<(), I2C::Error> {
        self.write_reg(reg::RESET, 0xFF)
    }

    /// Enable the chip
    pub fn enable(&mut self) -> Result<(), I2C::Error> {
        self.write_reg(reg::ENABLE, enable::CHIP_EN)
    }

    /// Disable the chip
    pub fn disable(&mut self) -> Result<(), I2C::Error> {
        self.write_reg(reg::ENABLE, 0x00)
    }

    /// Set PWM value for a specific channel (0-255)
    pub fn set_pwm(&mut self, channel: Channel, value: u8) -> Result<(), I2C::Error> {
        let reg = match channel {
            Channel::Red => reg::R_PWM,
            Channel::Green => reg::G_PWM,
            Channel::Blue => reg::B_PWM,
            Channel::White => reg::W_PWM,
        };
        self.write_reg(reg, value)
    }

    /// Set current for a specific channel (0-255, actual current = value * 0.1mA)
    pub fn set_current(&mut self, channel: Channel, value: u8) -> Result<(), I2C::Error> {
        let reg = match channel {
            Channel::Red => reg::R_CURRENT,
            Channel::Green => reg::G_CURRENT,
            Channel::Blue => reg::B_CURRENT,
            Channel::White => reg::W_CURRENT,
        };
        self.write_reg(reg, value)
    }

    /// Read a register
    pub fn read_reg(&mut self, reg: u8) -> Result<u8, I2C::Error> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(self.addr, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    /// Write a register
    pub fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.addr, &[reg, value])
    }

    /// Release the I2C bus
    pub fn release(self) -> I2C {
        self.i2c
    }
}