//! Gravity compensation feedforward controller
//!
//! Computes feedforward torque to counteract gravitational effects on pendulum-like systems.

/// Gravity compensation feedforward controller for pendulum systems.
///
/// Computes the feedforward term to compensate for gravitational torque:
/// τ = m * g * L * sin(θ)
///
/// Since most motor drivers use PWM (voltage) control rather than current control,
/// a conversion factor `k_pwm` is needed to map torque to PWM values.
#[derive(Debug, Clone, Copy)]
pub struct GravityCompensator {
    /// Precomputed: mass * g * length
    tau_max: f32,
}

impl GravityCompensator {
    /// Gravitational acceleration [m/s²]
    const G: f32 = 9.81;

    /// Creates a new gravity compensator.
    ///
    /// # Arguments
    /// * `mass` - Mass of the pendulum [kg]
    /// * `length` - Distance from pivot to center of mass [m]
    ///
    /// # Example
    /// ```
    /// let compensator = GravityCompensator::new(0.3, 0.05);
    /// ```
    pub fn new(mass: f32, length: f32) -> Self {
        let tau_max = mass * Self::G * length;
        Self {
            tau_max,
        }
    }

    /// Computes the feedforward PWM value for gravity compensation.
    ///
    /// # Arguments
    /// * `theta` - Current angle from vertical [rad], positive = forward lean
    ///
    /// # Returns
    /// gravity torque
    pub fn update(&self, theta: f32) -> f32 {
        self.tau_max * libm::sinf(theta)
    }
}
