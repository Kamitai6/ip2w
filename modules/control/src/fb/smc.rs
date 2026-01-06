/// Super Twisting Sliding Mode Controller for second-order systems
use libm::{sqrtf, tanhf};

const DELTA: f32 = 0.00001;
const V_LEAK: f32 = 0.00001;

pub struct SuperTwistingSMC {
    pub dt: f32,        // [s] time step
    pub lambda: f32,    // [出力/√(rad/s)] gain for sqrt term (algebraic)
    pub alpha: f32,     // [出力/s] integral rate (multiplied by dt internally)
    pub c: f32,         // [1/s] sliding surface parameter (algebraic)
    pub epsilon: f32,   // [rad/s] boundary layer width
    pub v_limit: f32,   // [出力] integral term limit
    v: f32,             // [出力] integral state
    prev_err: f32,      // [rad] previous error
}
impl SuperTwistingSMC {
    pub fn new(dt: f32, lambda: f32, alpha: f32, c: f32) -> Self {
        Self {
            dt,
            lambda,
            alpha,
            c,
            epsilon: 0.1,
            v_limit: f32::INFINITY,
            v: 0.0,
            prev_err: 0.0,
        }
    }
    pub fn with_smoothing(mut self, epsilon: f32) -> Self {
        self.epsilon = epsilon;
        self
    }
    pub fn with_v_regulation(mut self, limit: f32) -> Self {
        self.v_limit = limit;
        self
    }
    /// let err = target - value;
    /// let err_dot = (err - self.prev_err) / self.dt;
    pub fn update(&mut self, err: f32, err_dot: f32) -> f32 {
        self.prev_err = err;

        let s = err_dot + self.c * err;
        // let sat_s = (s / self.epsilon).clamp(-1.0, 1.0);
        let sat_s = tanhf(s / self.epsilon);
        
        self.v = ((1.0 - V_LEAK * self.dt) * self.v - self.alpha * sat_s * self.dt)
            .clamp(-self.v_limit, self.v_limit);
        
        let output = -self.lambda * sqrtf(s.abs() + DELTA) * sat_s + self.v;
        // defmt::info!("e={} s={} v={} u={}", err, s, self.v, output);
        
        output
    }
    pub fn reset(&mut self) {
        self.v = 0.0;
        self.prev_err = 0.0;
    }
}