//! Position drift feedforward controller
//!
//! Estimates position by double-integrating motor commands and generates
//! angle correction to prevent drift in balancing robots without encoders.

/// Position drift feedforward controller.
///
/// Estimates pseudo-velocity and position by integrating motor commands,
/// then outputs angle correction to keep the robot in place.
///
/// Uses critical damping condition (k_vel = 2√k_pos) so only one gain
/// parameter needs tuning. Anti-windup prevents internal state divergence.
#[derive(Debug, Clone, Copy)]
pub struct PositionRegulator {
    /// Estimated velocity [arbitrary units]
    velocity: f32,
    /// Estimated position [arbitrary units]
    position: f32,
    /// Time step [s]
    dt: f32,
    /// Position gain (spring)
    k_pos: f32,
    /// Velocity gain (damper), derived from k_pos for critical damping
    k_vel: f32,
    /// Maximum correction angle [rad]
    max_correction: f32,
}

impl PositionRegulator {
    /// Creates a new position drift corrector.
    ///
    /// Uses critical damping: k_vel = 2 * √k_pos
    ///
    /// # Arguments
    /// * `dt` - Time step [s]
    /// * `k` - Position gain (start with small values like 0.0001)
    /// * `max_correction` - Maximum correction angle [rad]
    pub fn new(dt: f32, k: f32, max_correction: f32) -> Self {
        Self {
            velocity: 0.0,
            position: 0.0,
            dt,
            k_pos: k,
            k_vel: 3.0 * libm::sqrtf(k),
            max_correction,
        }
    }

    /// Updates state and returns angle correction.
    ///
    /// Uses anti-windup: stops integrating when output saturates.
    ///
    /// # Arguments
    /// * `command` - Motor command value (e.g., PWM)
    ///
    /// # Returns
    /// Angle correction [rad] to subtract from target pitch
    pub fn update(&mut self, command: f32) -> f32 {
        let new_velocity = self.velocity + command * self.dt;
        let new_position = self.position + new_velocity * self.dt;
        let correction = self.k_vel * new_velocity + self.k_pos * new_position;

        if correction.abs() < self.max_correction {
            self.velocity = new_velocity;
            self.position = new_position;
            correction
        } else {
            correction.clamp(-self.max_correction, self.max_correction)
        }
    }

    /// Resets velocity and position to zero.
    pub fn reset(&mut self) {
        self.velocity = 0.0;
        self.position = 0.0;
    }
}