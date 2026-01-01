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
    /// Decay factor (0.95 ~ 0.99) to prevent windup
    decay: f32,
}

impl PositionRegulator {
    /// Creates a new leaky position drift controller.
    ///
    /// # Arguments
    /// * `decay` - "Forgetting" factor. 
    ///   - 1.0 = Perfect integration (floaty, drift-prone).
    ///   - 0.99 = Standard (stable).
    ///   - 0.95 = Tighter feel, returns to center faster but resists less.
    pub fn new(dt: f32, k: f32, decay: f32, max_correction: f32) -> Self {
        Self {
            velocity: 0.0,
            position: 0.0,
            dt,
            k_pos: k,
            k_vel: 2.0 * libm::sqrtf(k), // Critical damping
            max_correction,
            decay, // <--- Key magic number
        }
    }

    /// Updates state based on motor command.
    ///
    /// # Arguments
    /// * `command` - Motor command (-1.0 to 1.0 or PWM value)
    pub fn update(&mut self, command: f32) -> f32 {
        // 1. Integrate command to estimate velocity
        // Note: Real motors have lag. We assume instant response here,
        // but the 'decay' helps mask the discrepancy.
        self.velocity += command * self.dt;
        
        // 2. Apply Decay (The Fix for Phase Lag)
        // Instead of remembering velocity forever, we slowly reduce it.
        // This stops the robot from "thinking it's moving fast" when it has actually stopped.
        self.velocity *= self.decay;

        // 3. Integrate velocity to estimate position
        self.position += self.velocity * self.dt;
        
        // 4. Apply Decay to Position too
        // This prevents the "I must return to the start point 10 meters away" behavior.
        self.position *= self.decay;

        // 5. Calculate correction
        let correction = self.k_vel * self.velocity + self.k_pos * self.position;

        if correction.abs() < self.max_correction {
            correction
        } else {
            // Anti-windup: if saturated, don't update state or clamp it?
            // Simple clamping is safer.
            correction.clamp(-self.max_correction, self.max_correction)
        }
    }

    pub fn reset(&mut self) {
        self.velocity = 0.0;
        self.position = 0.0;
    }
}