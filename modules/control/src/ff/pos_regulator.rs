pub struct PositionRegulator {
    velocity: f32,
    position: f32,
    integral: f32,
    output: f32,
    dt: f32,
    k_pos: f32,
    k_vel: f32,
    k_acc: f32,
    k_int: f32,
    max_correction: f32,
    decay: f32,
    alpha: f32,
}

impl PositionRegulator {
    /// Creates a new position regulator.
    /// 
    /// # Arguments
    /// * `dt` - Time step [s]
    /// * `tau` - Response time constant [s]
    /// * `tau_decay` - Decay time constant [s] (how long velocity/position persist)
    /// * `max_correction` - Maximum output [rad]
    pub fn new(dt: f32, tau: f32, tau_decay: f32, max_correction: f32) -> Self {
        let k_pos = 1.0 / (tau * tau);
        let k_vel = 2.0 * libm::sqrtf(k_pos);
        // 時定数から1ステップあたりのdecayを計算
        let decay = libm::expf(-dt / tau_decay);
        Self {
            velocity: 0.0,
            position: 0.0,
            integral: 0.0,
            output: 0.0,
            dt,
            k_pos,
            k_vel,
            k_acc: 0.0,
            k_int: 0.0,
            max_correction,
            decay,
            alpha: 1.0,
        }
    }

    pub fn with_lead(mut self, factor: f32) -> Self {
        self.k_acc = factor * libm::sqrtf(self.k_vel);
        self
    }

    pub fn with_integral(mut self, factor: f32) -> Self {
        self.k_int = self.k_pos * factor;
        self
    }

    /// Add low-pass filter to output
    /// 
    /// # Arguments
    /// * `cutoff_hz` - Cutoff frequency [Hz]
    pub fn with_lowpass(mut self, cutoff_hz: f32) -> Self {
        // alpha = dt / (RC + dt), RC = 1/(2*pi*fc)
        let rc = 1.0 / (2.0 * core::f32::consts::PI * cutoff_hz);
        self.alpha = self.dt / (rc + self.dt);
        self
    }

    pub fn update(&mut self, command: f32) -> f32 {
        self.velocity += command * self.dt;
        self.velocity *= self.decay;
        self.position += self.velocity * self.dt;
        self.position *= self.decay;
        self.integral += self.position * self.dt;
        self.integral *= self.decay;
        
        let output = self.k_pos * self.position
                         + self.k_vel * self.velocity
                         + self.k_acc * command
                         + self.k_int * self.integral;
        
        self.output = self.alpha * output + (1.0 - self.alpha) * self.output;
        self.output.clamp(-self.max_correction, self.max_correction)
    }

    pub fn reset(&mut self) {
        self.velocity = 0.0;
        self.position = 0.0;
    }
}