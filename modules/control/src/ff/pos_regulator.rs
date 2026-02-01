use libm::tanhf;

//実質高級PID + ウォッシュアウトフィルタ + decay
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
    // ウォッシュアウト（HPF）
    dc_estimate: f32,
    hp_alpha: f32,
    // Decay（自己励振抑制 + ドリフト防止）
    vel_decay: f32, // 1ステップあたりの減衰率
    pos_decay: f32,
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
            dc_estimate: 0.0,
            hp_alpha: 0.0,
            vel_decay: 1.0,
            pos_decay: 1.0,
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

    /// 入力のDC成分を除去するウォッシュアウトフィルタ（HPF）を追加
    ///
    /// # Arguments
    /// * `tau_s` - 時定数 [秒]。大きいほど低い周波数まで通す。
    pub fn with_washout(mut self, tau_s: f32) -> Self {
        self.hp_alpha = self.dt / (tau_s + self.dt);
        self
    }

    /// 速度・位置のdecay（リーキー積分）を時定数で指定
    ///
    /// # Arguments
    /// * `vel_tau` - 速度の時定数 [秒]。1Hz発振を抑えるなら1.0〜2.0
    /// * `pos_tau` - 位置の時定数 [秒]。ドリフト防止。5.0〜20.0
    pub fn with_decay(mut self, vel_tau: f32, pos_tau: f32) -> Self {
        // exp(-dt/tau) を1次近似: 1 - dt/tau
        self.vel_decay = 1.0 - self.dt / vel_tau;
        self.pos_decay = 1.0 - self.dt / pos_tau;
        self
    }

    // commandはPWM
    pub fn update(&mut self, command: f32) -> f32 {
        // ウォッシュアウト: DC成分を推定して除去
        let cmd = if self.hp_alpha > 0.0 {
            self.dc_estimate += self.hp_alpha * (command - self.dc_estimate);
            command - self.dc_estimate
        } else {
            command
        };

        let limited_acc = cmd.clamp(-self.max_acc, self.max_acc);

        self.velocity += limited_acc * self.dt;
        self.velocity *= self.vel_decay;
        self.velocity = self.velocity.clamp(-self.max_vel, self.max_vel);
        
        self.position += self.velocity * self.dt;
        self.position *= self.pos_decay;
        
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
        self.dc_estimate = 0.0;
    }
}