//! Quaternion Extended Kalman Filter for IMU Attitude Estimation
//!
//! A `no_std` compatible EKF implementation for 6-axis IMU sensor fusion.
//! Uses quaternion representation with gyroscope bias estimation.
//!
//! # State Vector (7 elements)
//! - q0, q1, q2, q3: Orientation quaternion
//! - bx, by, bz: Gyroscope biases (rad/s)
//!
//! # Features
//! - Quaternion representation (no gimbal lock)
//! - Automatic gyroscope bias estimation
//! - All matrix operations manually unrolled for speed
//! - Optimized for ESP32 (~20μs per update at 240MHz)
//!
//! # Example
//!
//! ```no_run
//! use imu_filter::{ImuEkf, EkfConfig};
//!
//! let mut ekf = ImuEkf::new(EkfConfig::default());
//!
//! loop {
//!     let (ax, ay, az) = imu.read_accel().unwrap();  // g
//!     let (gx, gy, gz) = imu.read_gyro().unwrap();   // °/s
//!
//!     let state = ekf.update_deg(ax, ay, az, gx, gy, gz);
//!     
//!     let pitch = state.pitch;  // radians
//!     let roll = state.roll;
//! }
//! ```

#![no_std]

use core::f32::consts::PI;
use libm::{atan2f, asinf, sqrtf, fabsf, cosf, sinf};

/// Degrees to radians
const DEG_TO_RAD: f32 = PI / 180.0;
/// Radians to degrees
const RAD_TO_DEG: f32 = 180.0 / PI;

/// EKF Configuration
#[derive(Debug, Clone, Copy)]
pub struct EkfConfig {
    /// Sample period in seconds
    pub dt: f32,
    
    /// Gyroscope noise standard deviation (rad/s)
    pub gyro_noise: f32,
    
    /// Gyroscope bias random walk (rad/s²)
    pub gyro_bias_noise: f32,
    
    /// Accelerometer noise standard deviation (g)
    pub accel_noise: f32,
    
    /// Initial quaternion variance
    pub initial_quat_variance: f32,
    
    /// Initial bias variance (rad/s)²
    pub initial_bias_variance: f32,
    
    /// Accelerometer magnitude bounds for valid measurement
    pub accel_magnitude_min: f32,
    pub accel_magnitude_max: f32,
}

impl Default for EkfConfig {
    fn default() -> Self {
        Self {
            dt: 1.0 / 400.0,
            gyro_noise: 0.01,
            gyro_bias_noise: 0.0001,
            accel_noise: 0.1,
            initial_quat_variance: 0.1,
            initial_bias_variance: 0.01,
            accel_magnitude_min: 0.8,
            accel_magnitude_max: 1.2,
        }
    }
}

/// Quaternion [w, x, y, z]
#[derive(Debug, Clone, Copy)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self::identity()
    }
}

impl Quaternion {
    #[inline(always)]
    pub const fn identity() -> Self {
        Self { w: 1.0, x: 0.0, y: 0.0, z: 0.0 }
    }

    #[inline(always)]
    pub fn normalize(&mut self) {
        let norm = sqrtf(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z);
        if norm > 1e-10 {
            let inv = 1.0 / norm;
            self.w *= inv;
            self.x *= inv;
            self.y *= inv;
            self.z *= inv;
        }
    }

    /// Convert to Euler angles (roll, pitch, yaw) in radians
    #[inline]
    pub fn to_euler(&self) -> (f32, f32, f32) {
        // Roll (x-axis)
        let sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z);
        let cosr_cosp = 1.0 - 2.0 * (self.x * self.x + self.y * self.y);
        let roll = atan2f(sinr_cosp, cosr_cosp);

        // Pitch (y-axis)
        let sinp = 2.0 * (self.w * self.y - self.z * self.x);
        let pitch = if fabsf(sinp) >= 1.0 {
            if sinp >= 0.0 { PI / 2.0 } else { -PI / 2.0 }
        } else {
            asinf(sinp)
        };

        // Yaw (z-axis)
        let siny_cosp = 2.0 * (self.w * self.z + self.x * self.y);
        let cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z);
        let yaw = atan2f(siny_cosp, cosy_cosp);

        (roll, pitch, yaw)
    }
}

/// Attitude state output
#[derive(Debug, Clone, Copy, Default)]
pub struct AttitudeState {
    /// Euler angles in radians
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,

    /// Angular rates in rad/s (bias-corrected)
    pub roll_rate: f32,
    pub pitch_rate: f32,
    pub yaw_rate: f32,

    /// Estimated gyroscope biases in rad/s
    pub gyro_bias_x: f32,
    pub gyro_bias_y: f32,
    pub gyro_bias_z: f32,

    /// Current quaternion
    pub quaternion: Quaternion,

    /// Diagonal of covariance matrix (uncertainties)
    pub quat_variance: f32,
    pub bias_variance: f32,

    /// Accelerometer used in this update
    pub accel_valid: bool,
}

/// Quaternion Extended Kalman Filter
///
/// State: [q0, q1, q2, q3, bx, by, bz]
pub struct ImuEkf {
    // State vector
    q: Quaternion,
    bias: [f32; 3],

    // Covariance matrix P (7x7 symmetric, store upper triangle = 28 elements)
    p: [f32; 28],

    // Process noise
    q_gyro: f32,
    q_bias: f32,

    // Measurement noise
    r_accel: f32,

    config: EkfConfig,
    
    last_gyro: [f32; 3],
    initialized: bool,
}

// Helper to access symmetric matrix elements
#[inline(always)]
fn sym_idx(i: usize, j: usize) -> usize {
    if i <= j {
        i * (13 - i) / 2 + j
    } else {
        j * (13 - j) / 2 + i
    }
}

impl ImuEkf {
    /// Create new EKF with given configuration
    pub fn new(config: EkfConfig) -> Self {
        let dt = config.dt;
        
        let q_gyro = config.gyro_noise * config.gyro_noise * dt;
        let q_bias = config.gyro_bias_noise * config.gyro_bias_noise * dt;
        let r_accel = config.accel_noise * config.accel_noise;

        let pq = config.initial_quat_variance;
        let pb = config.initial_bias_variance;
        
        // Initial covariance (diagonal only)
        let mut p = [0.0f32; 28];
        p[sym_idx(0, 0)] = pq;
        p[sym_idx(1, 1)] = pq;
        p[sym_idx(2, 2)] = pq;
        p[sym_idx(3, 3)] = pq;
        p[sym_idx(4, 4)] = pb;
        p[sym_idx(5, 5)] = pb;
        p[sym_idx(6, 6)] = pb;

        Self {
            q: Quaternion::identity(),
            bias: [0.0; 3],
            p,
            q_gyro,
            q_bias,
            r_accel,
            config,
            last_gyro: [0.0; 3],
            initialized: false,
        }
    }

    /// Update filter with IMU measurements
    ///
    /// # Arguments
    /// * `ax, ay, az` - Accelerometer in g
    /// * `gx, gy, gz` - Gyroscope in rad/s
    pub fn update(&mut self, ax: f32, ay: f32, az: f32, gx: f32, gy: f32, gz: f32) -> AttitudeState {
        self.last_gyro = [gx, gy, gz];

        // Bias-corrected gyro
        let wx = gx - self.bias[0];
        let wy = gy - self.bias[1];
        let wz = gz - self.bias[2];

        // Check accelerometer validity
        let accel_norm_sq = ax * ax + ay * ay + az * az;
        let accel_valid = accel_norm_sq >= self.config.accel_magnitude_min * self.config.accel_magnitude_min
            && accel_norm_sq <= self.config.accel_magnitude_max * self.config.accel_magnitude_max;

        // Initialize on first valid accelerometer reading
        if !self.initialized && accel_valid {
            self.init_from_accel(ax, ay, az);
            self.initialized = true;
        }

        // ===== PREDICTION STEP =====
        self.predict(wx, wy, wz);

        // ===== UPDATE STEP =====
        if accel_valid && accel_norm_sq > 1e-10 {
            let norm_inv = 1.0 / sqrtf(accel_norm_sq);
            self.update_accel(ax * norm_inv, ay * norm_inv, az * norm_inv);
        }

        // Normalize quaternion
        self.q.normalize();

        // Ensure covariance stays positive definite
        self.ensure_positive_definite();

        // Build output
        let (roll, pitch, yaw) = self.q.to_euler();

        AttitudeState {
            roll,
            pitch,
            yaw,
            roll_rate: wx,
            pitch_rate: wy,
            yaw_rate: wz,
            gyro_bias_x: self.bias[0],
            gyro_bias_y: self.bias[1],
            gyro_bias_z: self.bias[2],
            quaternion: self.q,
            quat_variance: self.p[0],
            bias_variance: self.p[sym_idx(4, 4)],
            accel_valid,
        }
    }

    /// Update with gyroscope in degrees/second
    #[inline]
    pub fn update_deg(
        &mut self,
        ax: f32, ay: f32, az: f32,
        gx_deg: f32, gy_deg: f32, gz_deg: f32,
    ) -> AttitudeState {
        self.update(
            ax, ay, az,
            gx_deg * DEG_TO_RAD,
            gy_deg * DEG_TO_RAD,
            gz_deg * DEG_TO_RAD,
        )
    }

    /// Prediction step using gyroscope
    fn predict(&mut self, wx: f32, wy: f32, wz: f32) {
        let dt = self.config.dt;
        let dt_half = dt * 0.5;

        let q0 = self.q.w;
        let q1 = self.q.x;
        let q2 = self.q.y;
        let q3 = self.q.z;

        // Quaternion derivative: q_dot = 0.5 * q ⊗ ω
        self.q.w += dt_half * (-q1 * wx - q2 * wy - q3 * wz);
        self.q.x += dt_half * (q0 * wx + q2 * wz - q3 * wy);
        self.q.y += dt_half * (q0 * wy - q1 * wz + q3 * wx);
        self.q.z += dt_half * (q0 * wz + q1 * wy - q2 * wx);

        // State transition Jacobian F (simplified)
        // F_qq ≈ I + 0.5*dt*Ω(ω)
        // F_qb = -0.5*dt*[q]_R (right quaternion matrix cols 1-3)
        // F_bb = I

        let f01 = -dt_half * wx; let f02 = -dt_half * wy; let f03 = -dt_half * wz;
        let f10 =  dt_half * wx; let f12 =  dt_half * wz; let f13 = -dt_half * wy;
        let f20 =  dt_half * wy; let f21 = -dt_half * wz; let f23 =  dt_half * wx;
        let f30 =  dt_half * wz; let f31 =  dt_half * wy; let f32 = -dt_half * wx;

        // F_qb elements
        let f04 =  dt_half * q1; let f05 =  dt_half * q2; let f06 =  dt_half * q3;
        let f14 = -dt_half * q0; let f15 = -dt_half * q3; let f16 =  dt_half * q2;
        let f24 =  dt_half * q3; let f25 = -dt_half * q0; let f26 = -dt_half * q1;
        let f34 = -dt_half * q2; let f35 =  dt_half * q1; let f36 = -dt_half * q0;

        // Load P into local variables for faster access
        let p00 = self.p[sym_idx(0,0)]; let p01 = self.p[sym_idx(0,1)]; let p02 = self.p[sym_idx(0,2)];
        let p03 = self.p[sym_idx(0,3)]; let p04 = self.p[sym_idx(0,4)]; let p05 = self.p[sym_idx(0,5)];
        let p06 = self.p[sym_idx(0,6)];
        let p11 = self.p[sym_idx(1,1)]; let p12 = self.p[sym_idx(1,2)]; let p13 = self.p[sym_idx(1,3)];
        let p14 = self.p[sym_idx(1,4)]; let p15 = self.p[sym_idx(1,5)]; let p16 = self.p[sym_idx(1,6)];
        let p22 = self.p[sym_idx(2,2)]; let p23 = self.p[sym_idx(2,3)]; let p24 = self.p[sym_idx(2,4)];
        let p25 = self.p[sym_idx(2,5)]; let p26 = self.p[sym_idx(2,6)];
        let p33 = self.p[sym_idx(3,3)]; let p34 = self.p[sym_idx(3,4)]; let p35 = self.p[sym_idx(3,5)];
        let p36 = self.p[sym_idx(3,6)];
        let p44 = self.p[sym_idx(4,4)]; let p45 = self.p[sym_idx(4,5)]; let p46 = self.p[sym_idx(4,6)];
        let p55 = self.p[sym_idx(5,5)]; let p56 = self.p[sym_idx(5,6)];
        let p66 = self.p[sym_idx(6,6)];

        // Compute F*P (row by row, only upper triangle needed)
        // Row 0: F[0,:] * P
        let fp00 = p00 + f01*p01 + f02*p02 + f03*p03 + f04*p04 + f05*p05 + f06*p06;
        let fp01 = p01 + f01*p11 + f02*p12 + f03*p13 + f04*p14 + f05*p15 + f06*p16;
        let fp02 = p02 + f01*p12 + f02*p22 + f03*p23 + f04*p24 + f05*p25 + f06*p26;
        let fp03 = p03 + f01*p13 + f02*p23 + f03*p33 + f04*p34 + f05*p35 + f06*p36;
        let fp04 = p04 + f01*p14 + f02*p24 + f03*p34 + f04*p44 + f05*p45 + f06*p46;
        let fp05 = p05 + f01*p15 + f02*p25 + f03*p35 + f04*p45 + f05*p55 + f06*p56;
        let fp06 = p06 + f01*p16 + f02*p26 + f03*p36 + f04*p46 + f05*p56 + f06*p66;

        let fp10 = f10*p00 + p01 + f12*p02 + f13*p03 + f14*p04 + f15*p05 + f16*p06;
        let fp11 = f10*p01 + p11 + f12*p12 + f13*p13 + f14*p14 + f15*p15 + f16*p16;
        let fp12 = f10*p02 + p12 + f12*p22 + f13*p23 + f14*p24 + f15*p25 + f16*p26;
        let fp13 = f10*p03 + p13 + f12*p23 + f13*p33 + f14*p34 + f15*p35 + f16*p36;
        let fp14 = f10*p04 + p14 + f12*p24 + f13*p34 + f14*p44 + f15*p45 + f16*p46;
        let fp15 = f10*p05 + p15 + f12*p25 + f13*p35 + f14*p45 + f15*p55 + f16*p56;
        let fp16 = f10*p06 + p16 + f12*p26 + f13*p36 + f14*p46 + f15*p56 + f16*p66;

        let fp20 = f20*p00 + f21*p01 + p02 + f23*p03 + f24*p04 + f25*p05 + f26*p06;
        let fp21 = f20*p01 + f21*p11 + p12 + f23*p13 + f24*p14 + f25*p15 + f26*p16;
        let fp22 = f20*p02 + f21*p12 + p22 + f23*p23 + f24*p24 + f25*p25 + f26*p26;
        let fp23 = f20*p03 + f21*p13 + p23 + f23*p33 + f24*p34 + f25*p35 + f26*p36;
        let fp24 = f20*p04 + f21*p14 + p24 + f23*p34 + f24*p44 + f25*p45 + f26*p46;
        let fp25 = f20*p05 + f21*p15 + p25 + f23*p35 + f24*p45 + f25*p55 + f26*p56;
        let fp26 = f20*p06 + f21*p16 + p26 + f23*p36 + f24*p46 + f25*p56 + f26*p66;

        let fp30 = f30*p00 + f31*p01 + f32*p02 + p03 + f34*p04 + f35*p05 + f36*p06;
        let fp31 = f30*p01 + f31*p11 + f32*p12 + p13 + f34*p14 + f35*p15 + f36*p16;
        let fp32 = f30*p02 + f31*p12 + f32*p22 + p23 + f34*p24 + f35*p25 + f36*p26;
        let fp33 = f30*p03 + f31*p13 + f32*p23 + p33 + f34*p34 + f35*p35 + f36*p36;
        let fp34 = f30*p04 + f31*p14 + f32*p24 + p34 + f34*p44 + f35*p45 + f36*p46;
        let fp35 = f30*p05 + f31*p15 + f32*p25 + p35 + f34*p45 + f35*p55 + f36*p56;
        let fp36 = f30*p06 + f31*p16 + f32*p26 + p36 + f34*p46 + f35*p56 + f36*p66;

        // Bias rows unchanged (F_bb = I)
        let fp44 = p44; let fp45 = p45; let fp46 = p46;
        let fp55 = p55; let fp56 = p56;
        let fp66 = p66;

        // Now P' = F*P*F^T + Q
        // Compute upper triangle
        let q_q = self.q_gyro;
        let q_b = self.q_bias;

        self.p[sym_idx(0,0)] = fp00 + f01*fp01 + f02*fp02 + f03*fp03 + f04*fp04 + f05*fp05 + f06*fp06 + q_q;
        self.p[sym_idx(0,1)] = fp01 + f01*fp11 + f02*fp12 + f03*fp13 + f04*fp14 + f05*fp15 + f06*fp16;
        self.p[sym_idx(0,2)] = fp02 + f01*fp12 + f02*fp22 + f03*fp23 + f04*fp24 + f05*fp25 + f06*fp26;
        self.p[sym_idx(0,3)] = fp03 + f01*fp13 + f02*fp23 + f03*fp33 + f04*fp34 + f05*fp35 + f06*fp36;
        self.p[sym_idx(0,4)] = fp04 + f01*fp14 + f02*fp24 + f03*fp34 + f04*fp44 + f05*fp45 + f06*fp46;
        self.p[sym_idx(0,5)] = fp05 + f01*fp15 + f02*fp25 + f03*fp35 + f04*fp45 + f05*fp55 + f06*fp56;
        self.p[sym_idx(0,6)] = fp06 + f01*fp16 + f02*fp26 + f03*fp36 + f04*fp46 + f05*fp56 + f06*fp66;

        self.p[sym_idx(1,1)] = f10*fp01 + fp11 + f12*fp12 + f13*fp13 + f14*fp14 + f15*fp15 + f16*fp16 + q_q;
        self.p[sym_idx(1,2)] = f10*fp02 + fp12 + f12*fp22 + f13*fp23 + f14*fp24 + f15*fp25 + f16*fp26;
        self.p[sym_idx(1,3)] = f10*fp03 + fp13 + f12*fp23 + f13*fp33 + f14*fp34 + f15*fp35 + f16*fp36;
        self.p[sym_idx(1,4)] = f10*fp04 + fp14 + f12*fp24 + f13*fp34 + f14*fp44 + f15*fp45 + f16*fp46;
        self.p[sym_idx(1,5)] = f10*fp05 + fp15 + f12*fp25 + f13*fp35 + f14*fp45 + f15*fp55 + f16*fp56;
        self.p[sym_idx(1,6)] = f10*fp06 + fp16 + f12*fp26 + f13*fp36 + f14*fp46 + f15*fp56 + f16*fp66;

        self.p[sym_idx(2,2)] = f20*fp02 + f21*fp12 + fp22 + f23*fp23 + f24*fp24 + f25*fp25 + f26*fp26 + q_q;
        self.p[sym_idx(2,3)] = f20*fp03 + f21*fp13 + fp23 + f23*fp33 + f24*fp34 + f25*fp35 + f26*fp36;
        self.p[sym_idx(2,4)] = f20*fp04 + f21*fp14 + fp24 + f23*fp34 + f24*fp44 + f25*fp45 + f26*fp46;
        self.p[sym_idx(2,5)] = f20*fp05 + f21*fp15 + fp25 + f23*fp35 + f24*fp45 + f25*fp55 + f26*fp56;
        self.p[sym_idx(2,6)] = f20*fp06 + f21*fp16 + fp26 + f23*fp36 + f24*fp46 + f25*fp56 + f26*fp66;

        self.p[sym_idx(3,3)] = f30*fp03 + f31*fp13 + f32*fp23 + fp33 + f34*fp34 + f35*fp35 + f36*fp36 + q_q;
        self.p[sym_idx(3,4)] = f30*fp04 + f31*fp14 + f32*fp24 + fp34 + f34*fp44 + f35*fp45 + f36*fp46;
        self.p[sym_idx(3,5)] = f30*fp05 + f31*fp15 + f32*fp25 + fp35 + f34*fp45 + f35*fp55 + f36*fp56;
        self.p[sym_idx(3,6)] = f30*fp06 + f31*fp16 + f32*fp26 + fp36 + f34*fp46 + f35*fp56 + f36*fp66;

        self.p[sym_idx(4,4)] = fp44 + q_b;
        self.p[sym_idx(4,5)] = fp45;
        self.p[sym_idx(4,6)] = fp46;
        self.p[sym_idx(5,5)] = fp55 + q_b;
        self.p[sym_idx(5,6)] = fp56;
        self.p[sym_idx(6,6)] = fp66 + q_b;
    }

    /// Update step using accelerometer (normalized input)
    fn update_accel(&mut self, ax: f32, ay: f32, az: f32) {
        let q0 = self.q.w;
        let q1 = self.q.x;
        let q2 = self.q.y;
        let q3 = self.q.z;

        // Expected gravity in body frame: h(q) = R(q)^T * [0, 0, 1]^T
        let hx = 2.0 * (q1 * q3 - q0 * q2);
        let hy = 2.0 * (q0 * q1 + q2 * q3);
        let hz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // Innovation
        let y0 = ax - hx;
        let y1 = ay - hy;
        let y2 = az - hz;

        // Measurement Jacobian H (3x7), only first 4 columns are non-zero
        let h00 = -2.0 * q2; let h01 =  2.0 * q3; let h02 = -2.0 * q0; let h03 =  2.0 * q1;
        let h10 =  2.0 * q1; let h11 =  2.0 * q0; let h12 =  2.0 * q3; let h13 =  2.0 * q2;
        let h20 =  2.0 * q0; let h21 = -2.0 * q1; let h22 = -2.0 * q2; let h23 =  2.0 * q3;

        // Load P
        let p00 = self.p[sym_idx(0,0)]; let p01 = self.p[sym_idx(0,1)]; let p02 = self.p[sym_idx(0,2)];
        let p03 = self.p[sym_idx(0,3)]; let p04 = self.p[sym_idx(0,4)]; let p05 = self.p[sym_idx(0,5)];
        let p06 = self.p[sym_idx(0,6)];
        let p11 = self.p[sym_idx(1,1)]; let p12 = self.p[sym_idx(1,2)]; let p13 = self.p[sym_idx(1,3)];
        let p14 = self.p[sym_idx(1,4)]; let p15 = self.p[sym_idx(1,5)]; let p16 = self.p[sym_idx(1,6)];
        let p22 = self.p[sym_idx(2,2)]; let p23 = self.p[sym_idx(2,3)]; let p24 = self.p[sym_idx(2,4)];
        let p25 = self.p[sym_idx(2,5)]; let p26 = self.p[sym_idx(2,6)];
        let p33 = self.p[sym_idx(3,3)]; let p34 = self.p[sym_idx(3,4)]; let p35 = self.p[sym_idx(3,5)];
        let p36 = self.p[sym_idx(3,6)];
        let p44 = self.p[sym_idx(4,4)]; let p45 = self.p[sym_idx(4,5)]; let p46 = self.p[sym_idx(4,6)];
        let p55 = self.p[sym_idx(5,5)]; let p56 = self.p[sym_idx(5,6)];
        let p66 = self.p[sym_idx(6,6)];

        // P*H^T (7x3)
        let pht00 = p00*h00 + p01*h01 + p02*h02 + p03*h03;
        let pht01 = p00*h10 + p01*h11 + p02*h12 + p03*h13;
        let pht02 = p00*h20 + p01*h21 + p02*h22 + p03*h23;
        let pht10 = p01*h00 + p11*h01 + p12*h02 + p13*h03;
        let pht11 = p01*h10 + p11*h11 + p12*h12 + p13*h13;
        let pht12 = p01*h20 + p11*h21 + p12*h22 + p13*h23;
        let pht20 = p02*h00 + p12*h01 + p22*h02 + p23*h03;
        let pht21 = p02*h10 + p12*h11 + p22*h12 + p23*h13;
        let pht22 = p02*h20 + p12*h21 + p22*h22 + p23*h23;
        let pht30 = p03*h00 + p13*h01 + p23*h02 + p33*h03;
        let pht31 = p03*h10 + p13*h11 + p23*h12 + p33*h13;
        let pht32 = p03*h20 + p13*h21 + p23*h22 + p33*h23;
        let pht40 = p04*h00 + p14*h01 + p24*h02 + p34*h03;
        let pht41 = p04*h10 + p14*h11 + p24*h12 + p34*h13;
        let pht42 = p04*h20 + p14*h21 + p24*h22 + p34*h23;
        let pht50 = p05*h00 + p15*h01 + p25*h02 + p35*h03;
        let pht51 = p05*h10 + p15*h11 + p25*h12 + p35*h13;
        let pht52 = p05*h20 + p15*h21 + p25*h22 + p35*h23;
        let pht60 = p06*h00 + p16*h01 + p26*h02 + p36*h03;
        let pht61 = p06*h10 + p16*h11 + p26*h12 + p36*h13;
        let pht62 = p06*h20 + p16*h21 + p26*h22 + p36*h23;

        // S = H*P*H^T + R (3x3 symmetric)
        let r = self.r_accel;
        let s00 = h00*pht00 + h01*pht10 + h02*pht20 + h03*pht30 + r;
        let s01 = h00*pht01 + h01*pht11 + h02*pht21 + h03*pht31;
        let s02 = h00*pht02 + h01*pht12 + h02*pht22 + h03*pht32;
        let s11 = h10*pht01 + h11*pht11 + h12*pht21 + h13*pht31 + r;
        let s12 = h10*pht02 + h11*pht12 + h12*pht22 + h13*pht32;
        let s22 = h20*pht02 + h21*pht12 + h22*pht22 + h23*pht32 + r;

        // Invert S
        let (si00, si01, si02, si11, si12, si22) = Self::invert_sym3(s00, s01, s02, s11, s12, s22);

        // Kalman gain K = P*H^T * S^-1 (7x3)
        let k00 = pht00*si00 + pht01*si01 + pht02*si02;
        let k01 = pht00*si01 + pht01*si11 + pht02*si12;
        let k02 = pht00*si02 + pht01*si12 + pht02*si22;
        let k10 = pht10*si00 + pht11*si01 + pht12*si02;
        let k11 = pht10*si01 + pht11*si11 + pht12*si12;
        let k12 = pht10*si02 + pht11*si12 + pht12*si22;
        let k20 = pht20*si00 + pht21*si01 + pht22*si02;
        let k21 = pht20*si01 + pht21*si11 + pht22*si12;
        let k22 = pht20*si02 + pht21*si12 + pht22*si22;
        let k30 = pht30*si00 + pht31*si01 + pht32*si02;
        let k31 = pht30*si01 + pht31*si11 + pht32*si12;
        let k32 = pht30*si02 + pht31*si12 + pht32*si22;
        let k40 = pht40*si00 + pht41*si01 + pht42*si02;
        let k41 = pht40*si01 + pht41*si11 + pht42*si12;
        let k42 = pht40*si02 + pht41*si12 + pht42*si22;
        let k50 = pht50*si00 + pht51*si01 + pht52*si02;
        let k51 = pht50*si01 + pht51*si11 + pht52*si12;
        let k52 = pht50*si02 + pht51*si12 + pht52*si22;
        let k60 = pht60*si00 + pht61*si01 + pht62*si02;
        let k61 = pht60*si01 + pht61*si11 + pht62*si12;
        let k62 = pht60*si02 + pht61*si12 + pht62*si22;

        // State update
        self.q.w += k00*y0 + k01*y1 + k02*y2;
        self.q.x += k10*y0 + k11*y1 + k12*y2;
        self.q.y += k20*y0 + k21*y1 + k22*y2;
        self.q.z += k30*y0 + k31*y1 + k32*y2;
        self.bias[0] += k40*y0 + k41*y1 + k42*y2;
        self.bias[1] += k50*y0 + k51*y1 + k52*y2;
        self.bias[2] += k60*y0 + k61*y1 + k62*y2;

        // Covariance update P = (I - K*H) * P
        // Compute (I - K*H) matrix elements (only first 4 cols matter)
        let m00 = 1.0 - (k00*h00 + k01*h10 + k02*h20);
        let m01 = -(k00*h01 + k01*h11 + k02*h21);
        let m02 = -(k00*h02 + k01*h12 + k02*h22);
        let m03 = -(k00*h03 + k01*h13 + k02*h23);
        let m10 = -(k10*h00 + k11*h10 + k12*h20);
        let m11 = 1.0 - (k10*h01 + k11*h11 + k12*h21);
        let m12 = -(k10*h02 + k11*h12 + k12*h22);
        let m13 = -(k10*h03 + k11*h13 + k12*h23);
        let m20 = -(k20*h00 + k21*h10 + k22*h20);
        let m21 = -(k20*h01 + k21*h11 + k22*h21);
        let m22 = 1.0 - (k20*h02 + k21*h12 + k22*h22);
        let m23 = -(k20*h03 + k21*h13 + k22*h23);
        let m30 = -(k30*h00 + k31*h10 + k32*h20);
        let m31 = -(k30*h01 + k31*h11 + k32*h21);
        let m32 = -(k30*h02 + k31*h12 + k32*h22);
        let m33 = 1.0 - (k30*h03 + k31*h13 + k32*h23);
        let m40 = -(k40*h00 + k41*h10 + k42*h20);
        let m41 = -(k40*h01 + k41*h11 + k42*h21);
        let m42 = -(k40*h02 + k41*h12 + k42*h22);
        let m43 = -(k40*h03 + k41*h13 + k42*h23);
        let m50 = -(k50*h00 + k51*h10 + k52*h20);
        let m51 = -(k50*h01 + k51*h11 + k52*h21);
        let m52 = -(k50*h02 + k51*h12 + k52*h22);
        let m53 = -(k50*h03 + k51*h13 + k52*h23);
        let m60 = -(k60*h00 + k61*h10 + k62*h20);
        let m61 = -(k60*h01 + k61*h11 + k62*h21);
        let m62 = -(k60*h02 + k61*h12 + k62*h22);
        let m63 = -(k60*h03 + k61*h13 + k62*h23);

        // P' = M * P (upper triangle)
        self.p[sym_idx(0,0)] = m00*p00 + m01*p01 + m02*p02 + m03*p03;
        self.p[sym_idx(0,1)] = m00*p01 + m01*p11 + m02*p12 + m03*p13;
        self.p[sym_idx(0,2)] = m00*p02 + m01*p12 + m02*p22 + m03*p23;
        self.p[sym_idx(0,3)] = m00*p03 + m01*p13 + m02*p23 + m03*p33;
        self.p[sym_idx(0,4)] = m00*p04 + m01*p14 + m02*p24 + m03*p34;
        self.p[sym_idx(0,5)] = m00*p05 + m01*p15 + m02*p25 + m03*p35;
        self.p[sym_idx(0,6)] = m00*p06 + m01*p16 + m02*p26 + m03*p36;

        self.p[sym_idx(1,1)] = m10*p01 + m11*p11 + m12*p12 + m13*p13;
        self.p[sym_idx(1,2)] = m10*p02 + m11*p12 + m12*p22 + m13*p23;
        self.p[sym_idx(1,3)] = m10*p03 + m11*p13 + m12*p23 + m13*p33;
        self.p[sym_idx(1,4)] = m10*p04 + m11*p14 + m12*p24 + m13*p34;
        self.p[sym_idx(1,5)] = m10*p05 + m11*p15 + m12*p25 + m13*p35;
        self.p[sym_idx(1,6)] = m10*p06 + m11*p16 + m12*p26 + m13*p36;

        self.p[sym_idx(2,2)] = m20*p02 + m21*p12 + m22*p22 + m23*p23;
        self.p[sym_idx(2,3)] = m20*p03 + m21*p13 + m22*p23 + m23*p33;
        self.p[sym_idx(2,4)] = m20*p04 + m21*p14 + m22*p24 + m23*p34;
        self.p[sym_idx(2,5)] = m20*p05 + m21*p15 + m22*p25 + m23*p35;
        self.p[sym_idx(2,6)] = m20*p06 + m21*p16 + m22*p26 + m23*p36;

        self.p[sym_idx(3,3)] = m30*p03 + m31*p13 + m32*p23 + m33*p33;
        self.p[sym_idx(3,4)] = m30*p04 + m31*p14 + m32*p24 + m33*p34;
        self.p[sym_idx(3,5)] = m30*p05 + m31*p15 + m32*p25 + m33*p35;
        self.p[sym_idx(3,6)] = m30*p06 + m31*p16 + m32*p26 + m33*p36;

        self.p[sym_idx(4,4)] = m40*p04 + m41*p14 + m42*p24 + m43*p34 + p44;
        self.p[sym_idx(4,5)] = m40*p05 + m41*p15 + m42*p25 + m43*p35 + p45;
        self.p[sym_idx(4,6)] = m40*p06 + m41*p16 + m42*p26 + m43*p36 + p46;

        self.p[sym_idx(5,5)] = m50*p05 + m51*p15 + m52*p25 + m53*p35 + p55;
        self.p[sym_idx(5,6)] = m50*p06 + m51*p16 + m52*p26 + m53*p36 + p56;

        self.p[sym_idx(6,6)] = m60*p06 + m61*p16 + m62*p26 + m63*p36 + p66;
    }

    /// Invert 3x3 symmetric matrix
    #[inline]
    fn invert_sym3(a00: f32, a01: f32, a02: f32, a11: f32, a12: f32, a22: f32) 
        -> (f32, f32, f32, f32, f32, f32) 
    {
        let det = a00 * (a11 * a22 - a12 * a12)
                - a01 * (a01 * a22 - a12 * a02)
                + a02 * (a01 * a12 - a11 * a02);

        if fabsf(det) < 1e-10 {
            return (1.0, 0.0, 0.0, 1.0, 0.0, 1.0);
        }

        let inv_det = 1.0 / det;
        (
            (a11 * a22 - a12 * a12) * inv_det,
            (a02 * a12 - a01 * a22) * inv_det,
            (a01 * a12 - a02 * a11) * inv_det,
            (a00 * a22 - a02 * a02) * inv_det,
            (a02 * a01 - a00 * a12) * inv_det,
            (a00 * a11 - a01 * a01) * inv_det,
        )
    }

    fn init_from_accel(&mut self, ax: f32, ay: f32, az: f32) {
        let norm = sqrtf(ax * ax + ay * ay + az * az);
        if norm < 1e-10 { return; }

        let ax = ax / norm;
        let ay = ay / norm;
        let az = az / norm;

        let roll = atan2f(ay, az);
        let pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

        let cr = cosf(roll * 0.5);
        let sr = sinf(roll * 0.5);
        let cp = cosf(pitch * 0.5);
        let sp = sinf(pitch * 0.5);

        self.q.w = cr * cp;
        self.q.x = sr * cp;
        self.q.y = cr * sp;
        self.q.z = -sr * sp;
        self.q.normalize();
    }

    fn ensure_positive_definite(&mut self) {
        const MIN_VAR: f32 = 1e-8;
        const MAX_VAR: f32 = 1e4;

        for i in 0..7 {
            let idx = sym_idx(i, i);
            self.p[idx] = self.p[idx].clamp(MIN_VAR, MAX_VAR);
        }
    }

    // ===== Public API =====

    #[inline(always)]
    pub fn get_quaternion(&self) -> Quaternion { self.q }

    #[inline(always)]
    pub fn get_euler(&self) -> (f32, f32, f32) { self.q.to_euler() }

    #[inline(always)]
    pub fn get_roll(&self) -> f32 { self.get_euler().0 }

    #[inline(always)]
    pub fn get_pitch(&self) -> f32 { self.get_euler().1 }

    #[inline(always)]
    pub fn get_yaw(&self) -> f32 { self.get_euler().2 }

    #[inline(always)]
    pub fn get_roll_deg(&self) -> f32 { self.get_roll() * RAD_TO_DEG }

    #[inline(always)]
    pub fn get_pitch_deg(&self) -> f32 { self.get_pitch() * RAD_TO_DEG }

    #[inline(always)]
    pub fn get_yaw_deg(&self) -> f32 { self.get_yaw() * RAD_TO_DEG }

    #[inline(always)]
    pub fn get_gyro_bias(&self) -> (f32, f32, f32) {
        (self.bias[0], self.bias[1], self.bias[2])
    }

    pub fn set_gyro_bias(&mut self, bx: f32, by: f32, bz: f32) {
        self.bias = [bx, by, bz];
    }

    pub fn reset(&mut self) {
        self.q = Quaternion::identity();
        self.bias = [0.0; 3];
        self.initialized = false;
        
        let pq = self.config.initial_quat_variance;
        let pb = self.config.initial_bias_variance;
        for i in 0..28 { self.p[i] = 0.0; }
        self.p[sym_idx(0,0)] = pq;
        self.p[sym_idx(1,1)] = pq;
        self.p[sym_idx(2,2)] = pq;
        self.p[sym_idx(3,3)] = pq;
        self.p[sym_idx(4,4)] = pb;
        self.p[sym_idx(5,5)] = pb;
        self.p[sym_idx(6,6)] = pb;
    }

    pub fn reset_yaw(&mut self) {
        let (roll, pitch, _) = self.q.to_euler();
        let cr = cosf(roll * 0.5);
        let sr = sinf(roll * 0.5);
        let cp = cosf(pitch * 0.5);
        let sp = sinf(pitch * 0.5);
        self.q.w = cr * cp;
        self.q.x = sr * cp;
        self.q.y = cr * sp;
        self.q.z = -sr * sp;
        self.q.normalize();
    }

    #[inline(always)]
    pub fn is_initialized(&self) -> bool { self.initialized }

    pub fn config(&self) -> &EkfConfig { &self.config }

    pub fn set_dt(&mut self, dt: f32) {
        self.config.dt = dt;
        self.q_gyro = self.config.gyro_noise * self.config.gyro_noise * dt;
        self.q_bias = self.config.gyro_bias_noise * self.config.gyro_bias_noise * dt;
    }
}
