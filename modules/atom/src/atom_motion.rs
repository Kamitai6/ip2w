//! Atom Motion Motor Driver
//!
//! A `no_std` compatible driver for the M5Stack Atom Motion motor driver.
//! Supports 2 DC motors and 4 servo channels via I2C.

use embedded_hal::i2c::I2c;

/// Atom Motion I2C address
pub const DEFAULT_ADDR: u8 = 0x38;

/// Register addresses
mod reg {
    pub const MOTOR1: u8 = 0x20;
    pub const MOTOR2: u8 = 0x21;
    pub const SERVO1: u8 = 0x10;
    pub const SERVO2: u8 = 0x11;
    pub const SERVO3: u8 = 0x12;
    pub const SERVO4: u8 = 0x13;
}

/// DC Motor channel
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorChannel {
    M1,
    M2,
}

/// Servo channel
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ServoChannel {
    S1,
    S2,
    S3,
    S4,
}

/// Atom Motion driver
pub struct AtomMotion<I2C> {
    i2c: I2C,
    addr: u8,
}

impl<I2C: I2c> AtomMotion<I2C> {
    /// Create a new Atom Motion driver with default address (0x38)
    pub fn new(i2c: I2C) -> Self {
        Self::new_with_addr(i2c, DEFAULT_ADDR)
    }

    /// Create a new Atom Motion driver with custom address
    pub fn new_with_addr(i2c: I2C, addr: u8) -> Self {
        Self { i2c, addr }
    }

    /// Set DC motor speed
    /// 
    /// # Arguments
    /// * `channel` - Motor channel (M1 or M2)
    /// * `speed` - Speed value (-127 to 127, 0 = stop)
    pub fn set_motor(&mut self, channel: MotorChannel, speed: i8) -> Result<(), I2C::Error> {
        let reg = match channel {
            MotorChannel::M1 => reg::MOTOR1,
            MotorChannel::M2 => reg::MOTOR2,
        };
        self.write_reg(reg, speed as u8)
    }

    /// Stop a DC motor
    pub fn stop_motor(&mut self, channel: MotorChannel) -> Result<(), I2C::Error> {
        self.set_motor(channel, 0)
    }

    /// Set servo angle
    /// 
    /// # Arguments
    /// * `channel` - Servo channel (S1-S4)
    /// * `angle` - Angle in degrees (0-180)
    pub fn set_servo(&mut self, channel: ServoChannel, angle: u8) -> Result<(), I2C::Error> {
        let reg = match channel {
            ServoChannel::S1 => reg::SERVO1,
            ServoChannel::S2 => reg::SERVO2,
            ServoChannel::S3 => reg::SERVO3,
            ServoChannel::S4 => reg::SERVO4,
        };
        let angle = angle.min(180);
        self.write_reg(reg, angle)
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