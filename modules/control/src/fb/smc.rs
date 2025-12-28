pub struct SimpleSMC {
    lambda: f32,
    c: f32,
    epsilon: f32,
    ki: f32,
    dt: f32,
    integral: f32,
}

impl SimpleSMC {
    pub fn new(dt: f32, lambda: f32, c: f32, epsilon: f32, ki: f32) -> Self {
        Self { lambda, c, epsilon, ki, dt, integral: 0.0 }
    }
    
    pub fn update(&mut self, err: f32, err_dot: f32) -> f32 {
        let s = err_dot + self.c * err;
        
        self.integral += err * self.dt;
        self.integral = self.integral.clamp(-0.1, 0.1);
        
        let sat_s = if s.abs() < self.epsilon {
            s / self.epsilon
        } else {
            s.signum()
        };
        
        // 符号を修正: - → +
        let u_raw = self.lambda * sat_s + self.ki * self.integral;
        let u = u_raw.clamp(-127.0, 127.0);
        
        defmt::info!("e={} s={} u={}", err, s, u);
        
        u
    }
}