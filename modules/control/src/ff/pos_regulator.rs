pub struct PositionRegulator {
    velocity: f32,
    position: f32,
    output: f32,  // フィルタ後の出力
    dt: f32,
    k_pos: f32,
    k_vel: f32,
    max_correction: f32,
    decay: f32,
    alpha: f32,  // LPFの係数 (0~1, 小さいほど滑らか)
}

impl PositionRegulator {
    pub fn new(dt: f32, tau: f32, decay: f32, max_correction: f32) -> Self {
        let k_pos = 1.0 / (tau * tau);
        let k_vel = 3.0 * libm::sqrtf(k_pos);
        Self {
            velocity: 0.0,
            position: 0.0,
            output: 0.0,
            dt,
            k_pos,
            k_vel,
            max_correction,
            decay,
            alpha: 1.0,  // デフォルトはフィルタなし
        }
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
        
        let raw = self.k_pos * self.position + self.k_vel * self.velocity;
        
        // EMAフィルタ → クランプの順
        self.output = self.alpha * raw + (1.0 - self.alpha) * self.output;
        self.output.clamp(-self.max_correction, self.max_correction)
    }

    pub fn reset(&mut self) {
        self.velocity = 0.0;
        self.position = 0.0;
        self.output = 0.0;
    }
}