/// Position-type PID controller with anti-windup
pub struct PID {
    pub dt: f32,
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub integral_limit: f32,
    integral: f32,
    prev_err: f32,
}

impl PID {
    pub fn new(dt: f32, kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            dt,
            kp, ki, kd,
            integral_limit: f32::INFINITY,
            integral: 0.0,
            prev_err: 0.0
        }
    }

    pub fn with_integral_limits(mut self, limit: f32) -> Self {
        self.integral_limit = limit;
        self
    }

    pub fn update(&mut self, value: f32, target: f32) -> f32 {
        let err = target - value;
        self.integral = (self.integral + err * self.dt).clamp(-self.integral_limit, self.integral_limit);
        let derivative = (err - self.prev_err) / self.dt;
        self.prev_err = err;

        self.kp * err + self.ki * self.integral + self.kd * derivative
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_err = 0.0;
    }
}