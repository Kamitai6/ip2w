/// Super Twisting Sliding Mode Controller for second-order systems
use libm::{sqrtf, expf};
pub struct SuperTwistingSMC {
    pub dt: f32,        // [s] time step
    pub lambda: f32,    // gain for sqrt term
    pub alpha: f32,     // gain for integral term
    pub c: f32,         // [1/s] sliding surface parameter
    pub epsilon: f32,   // [1/s] boundary layer width (same unit as s)
    pub delta: f32,     // [1/s^2] regularization for sqrt (same unit as |s|)
    pub v_limit: f32,   // integral term limit
    pub v_leak: f32,    // [1/s] decay rate (time constant = 1/v_leak)
    v: f32,
    prev_err: f32,
}
impl SuperTwistingSMC {
    pub fn new(dt: f32, lambda: f32, alpha: f32, c: f32) -> Self {
        Self {
            dt,
            lambda,
            alpha,
            c,
            epsilon: 0.1,
            delta: 0.01,
            v_limit: f32::INFINITY,
            v_leak: 0.0,
            v: 0.0,
            prev_err: 0.0,
        }
    }
    pub fn with_smoothing(mut self, epsilon: f32, delta: f32) -> Self {
        self.epsilon = epsilon;
        self.delta = delta;
        self
    }
    pub fn with_v_regulation(mut self, limit: f32, leak: f32) -> Self {
        self.v_limit = limit;
        self.v_leak = leak;
        self
    }
    pub fn update(&mut self, value: f32, target: f32) -> f32 {
        let err = target - value;
        let err_dot = (err - self.prev_err) / self.dt;
        self.prev_err = err;
        let s = err_dot + self.c * err;
        let sat_s = (s / self.epsilon).clamp(-1.0, 1.0);
        self.v = (expf(-self.v_leak * self.dt) * self.v - self.alpha * sat_s * self.dt)
            .clamp(-self.v_limit, self.v_limit);
        let output = -self.lambda * sqrtf(s.abs() + self.delta) * sat_s + self.v;
        defmt::info!("e={} s={} v={} u={}", err, s, self.v, output);
        output
    }
    pub fn reset(&mut self) {
        self.v = 0.0;
        self.prev_err = 0.0;
    }
}