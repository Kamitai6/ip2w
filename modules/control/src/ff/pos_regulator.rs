/// Leaky Position Drift Controller
///
/// Output integration with decay factor.
/// The decay prevents "integral windup" which causes the "laggy/floaty" feeling.
/// It acts like a virtual spring that slowly relaxes to the center.
#[derive(Debug, Clone, Copy)]
pub struct PositionRegulator {
    velocity: f32,
    position: f32,
    dt: f32,
    k_pos: f32,
    k_vel: f32,
    max_correction: f32,
    decay: f32,
}

impl PositionRegulator {
    /// Creates a new leaky position drift controller.
    pub fn new(dt: f32, k: f32, decay: f32, max_correction: f32) -> Self {
        Self {
            velocity: 0.0,
            position: 0.0,
            dt,
            k_pos: k,
            k_vel: 3.0 * libm::sqrtf(k), // Critical damping
            max_correction,
            decay,
        }
    }

    /// Updates state based on motor command.
    ///
    /// # Arguments
    /// * `command` - Motor command (PWM value)
    pub fn update(&mut self, command: f32) -> f32 {
        self.velocity += command * self.dt;
        self.velocity *= self.decay;

        self.position += self.velocity * self.dt;
        self.position *= self.decay;

        let correction = self.k_vel * self.velocity + self.k_pos * self.position;
        correction.clamp(-self.max_correction, self.max_correction)
    }

    pub fn reset(&mut self) {
        self.velocity = 0.0;
        self.position = 0.0;
    }
}