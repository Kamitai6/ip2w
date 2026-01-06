use libm::tanhf;

//実質高級PID
pub struct PositionRegulator {
    velocity: f32,
    position: f32,
    output: f32,
    dt: f32,
    k_pos: f32,
    k_vel: f32,
    k_acc: f32,
    max_output: f32,
    max_acc: f32,
    max_vel: f32,
    alpha: f32,
}
impl PositionRegulator {
    pub fn new(dt: f32, tau: f32, max_output: f32) -> Self {
        let k_pos = 1.0 / (tau * tau);
        let k_vel = 2.0 * libm::sqrtf(k_pos);
        Self {
            velocity: 0.0,
            position: 0.0,
            output: 0.0,
            dt,
            k_pos,
            k_vel,
            k_acc: 0.0,
            max_output,
            max_acc: f32::INFINITY,
            max_vel: f32::INFINITY,
            alpha: 1.0,
        }
    }

    pub fn with_lead(mut self, factor: f32) -> Self {
        self.k_acc = factor * libm::sqrtf(self.k_vel);
        self
    }

    pub fn with_lowpass(mut self, cutoff_hz: f32) -> Self {
        let rc = 1.0 / (2.0 * core::f32::consts::PI * cutoff_hz);
        self.alpha = self.dt / (rc + self.dt);
        self
    }

    pub fn with_max(mut self, max_acc: f32, max_vel: f32) -> Self {
        self.max_acc = max_acc;
        self.max_vel = max_vel;
        self
    }

    // commandはPWM
    pub fn update(&mut self, command: f32) -> f32 {
        let limited_acc = command.clamp(-self.max_acc, self.max_acc);

        self.velocity += limited_acc * self.dt;
        self.velocity = self.velocity.clamp(-self.max_vel, self.max_vel);
        
        self.position += self.velocity * self.dt;
        
        let output = self.k_pos * self.position
                         + self.k_vel * self.velocity
                         + self.k_acc * limited_acc;
        self.output = self.alpha * output + (1.0 - self.alpha) * self.output;
        self.max_output * tanhf(self.output / self.max_output)
    }

    pub fn reset(&mut self) {
        self.velocity = 0.0;
        self.position = 0.0;
        self.output = 0.0;
    }
}