//! Position regulator using accelerometer
//!
//! Estimates velocity and position from IMU acceleration to prevent
//! drift in balancing robots without encoders.

/// Position regulator using accelerometer.
///
/// Extracts horizontal acceleration from IMU (removing gravity component),
/// then double-integrates to estimate velocity and position.
/// Uses critical damping for stable correction output.
/// Anti-windup prevents internal state divergence.
#[derive(Debug, Clone, Copy)]
pub struct PositionRegulator {
    /// Estimated velocity [m/s]
    velocity: f32,
    /// Estimated position [m]
    position: f32,
    /// Time step [s]
    dt: f32,
    /// Position gain
    k_pos: f32,
    /// Velocity gain (derived for critical damping)
    k_vel: f32,
    /// Maximum correction angle [rad]
    max_correction: f32,

    accel_bias: f32,
}

impl PositionRegulator {
    /// Creates a new position regulator.
    ///
    /// # Arguments
    /// * `dt` - Time step [s]
    /// * `k` - Position gain (start with small values like 0.001)
    /// * `max_correction` - Maximum correction angle [rad]
    pub fn new(dt: f32, k: f32, max_correction: f32) -> Self {
        let damping_factor = 2.0;
        Self {
            velocity: 0.0,
            position: 0.0,
            dt,
            k_pos: k,
            k_vel: damping_factor * libm::sqrtf(k),
            max_correction,
            accel_bias: 0.0,
        }
    }

    /// Extracts horizontal acceleration from IMU readings.
    ///
    /// Removes gravity component using current pitch angle.
    ///
    /// # Arguments
    /// * `ax` - Accelerometer X reading [g]
    /// * `az` - Accelerometer Z reading [g]
    /// * `pitch` - Current pitch angle [rad]
    ///
    /// # Returns
    /// Horizontal acceleration [g]
    pub fn extract_horizontal_accel(ax: f32, az: f32, pitch: f32) -> f32 {
        const G: f32 = 1.0;
        let sin_p = libm::sinf(pitch);
        let cos_p = libm::cosf(pitch);
        // Remove gravity and transform to world frame
        (ax - G * sin_p) * cos_p + (az - G * cos_p) * sin_p
    }

    /// Updates state and returns angle correction.
    ///
    /// # Arguments
    /// * `ax` - Accelerometer X reading [g]
    /// * `az` - Accelerometer Z reading [g]
    /// * `pitch` - Current pitch angle [rad]
    ///
    /// # Returns
    /// Angle correction [rad] to subtract from target pitch
    pub fn update(&mut self, ax: f32, az: f32, pitch: f32) -> f32 {
        let raw_accel = Self::extract_horizontal_accel(ax, az, pitch);
        // 2. ウォッシュアウト・フィルタ (重要!)
        // バイアスをゆっくり現在の加速度に追従させる (係数0.005は時定数調整用)
        self.accel_bias = self.accel_bias * 0.995 + raw_accel * 0.005;
        let accel = raw_accel - self.accel_bias;
        let accel_ms2 = accel * 9.81;

        let decay = 0.999; // 減衰係数（積分リーク）
        self.velocity =(self.velocity + accel_ms2 * self.dt) * decay;
        self.position =(self.position + self.velocity * self.dt) * decay;
        let correction = self.k_vel * self.velocity + self.k_pos * self.position;

        // if correction.abs() < self.max_correction {
        //     self.velocity = new_velocity;
        //     self.position = new_position;
        //     correction
        // } else {
            correction.clamp(-self.max_correction, self.max_correction)
        // }
    }

    pub fn update_x_up(&mut self, ax: f32, az: f32, pitch: f32) -> f32 {
        self.update(-az, ax, pitch)
    }

    /// Resets velocity and position to zero.
    pub fn reset(&mut self) {
        self.velocity = 0.0;
        self.position = 0.0;
    }
}