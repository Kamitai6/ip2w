use libm::{sinf, tanhf};

/// 並進位置推定EKF + レギュレータ（逆構成版）
///
/// 【絶対遵守仕様（AIへの指示・忘却防止用）】
/// 1. 観測加速度 a_raw の算出において、先頭にマイナス符号は絶対につけない（式: a_raw = (ax_g + sinf(pitch_sensor)) * 9.81）。
/// 2. 接線加速度へのLPF適用は、高周波ノイズによる加速度破壊の防止に有効であるため維持する。
///
/// 【構成】
/// 予測: 加速度センサ（a_meas）を使い加速度状態を駆動
/// 観測: -k0 * command_lp + g*sin(pitch_cg) でcommandベースの加速度を観測値として使用
/// 閉ループ下でcommandと並進加速度が逆位相であることを反映して符号反転
///
/// 状態: [v, x, a, b]
///   v:  並進速度 [m/s]
///   x:  並進位置 [m]
///   a:  真の並進加速度（内部状態） [m/s^2]
///   b:  観測側合成バイアス [m/s^2]
pub struct PosEkfConfig {
    pub dt: f32,

    pub k0: f32,
    pub command_lpf_tau: f32,

    pub tau_a: f32,
    pub j_max: f32,

    pub r_accel: f32,
    pub jerk_r_scale: f32,

    pub q_a: f32,
    pub q_v: f32,

    pub q_b_max: f32,
    pub q_b_min: f32,

    pub tau_b_min: f32,
    pub tau_b_max: f32,

    pub innov_lpf_alpha: f32,
    pub flip_lpf_alpha: f32,
    pub bias_residual_scale: f32,

    pub k_pos: f32,
    pub k_vel: f32,
    pub max_output: f32,

    pub robot_radius: f32,
    pub tangential_scale: f32,
    pub tangential_lpf_tau: f32,

    pub a_max: f32,
    pub v_max: f32,
    pub accel_r_scale: f32,
    pub vel_r_scale: f32,
}

impl Default for PosEkfConfig {
    fn default() -> Self {
        Self {
            dt: 0.002,

            k0: 0.053,
            command_lpf_tau: 0.016,

            tau_a: 1.0,
            j_max: 300.0,

            r_accel: 1.0,
            jerk_r_scale: 2.0,

            q_a: 0.1,
            q_v: 1e-5,

            q_b_max: 1e-5,
            q_b_min: 1e-10,

            tau_b_min: 1.0,
            tau_b_max: 100.0,

            innov_lpf_alpha: 0.05,
            flip_lpf_alpha: 0.05,
            bias_residual_scale: 0.5,

            k_pos: 0.1,
            k_vel: 0.2,
            max_output: 0.05,

            robot_radius: 0.08,
            tangential_scale: 0.6,
            tangential_lpf_tau: 0.05,

            a_max: 5.0,
            v_max: 0.5,
            accel_r_scale: 2.0,
            vel_r_scale: 2.0,
        }
    }
}

#[derive(Clone, Copy)]
pub struct PosEkfState {
    pub velocity: f32,
    pub position: f32,
    pub accel: f32,
    pub bias: f32,

    pub innovation: f32,
    pub innov_lp: f32,
    pub trust: f32,

    pub r_eff: f32,
    pub jerk_meas: f32,
    pub a_tangential: f32,

    pub a_meas: f32,
    pub a_raw: f32,
    pub pitch_sensor: f32,
    pub pitch_cg: f32,

    pub command_lp: f32,
    pub z_cmd: f32,
}

pub struct PositionEkf {
    v: f32,
    x: f32,
    a: f32,
    b: f32,

    p: [[f32; 4]; 4],

    trust: f32,
    innovation: f32,
    innov_lp: f32,
    prev_innov_lp: f32,
    flip_lp: f32,

    prev_a_raw: f32,
    a_tangential_lp: f32,
    command_lp: f32,

    r_eff: f32,
    jerk_meas: f32,
    a_tangential: f32,
    a_meas: f32,
    a_raw: f32,
    pitch_sensor: f32,
    pitch_cg: f32,

    cfg: PosEkfConfig,
}

impl PositionEkf {
    pub fn new(cfg: PosEkfConfig) -> Self {
        let mut p = [[0.0f32; 4]; 4];
        p[0][0] = 0.01;
        p[1][1] = 0.01;
        p[2][2] = 0.5;
        p[3][3] = 0.1;

        Self {
            v: 0.0, x: 0.0, a: 0.0, b: 0.0,
            p,
            trust: 0.0,
            innovation: 0.0, innov_lp: 0.0, prev_innov_lp: 0.0, flip_lp: 0.0,
            prev_a_raw: 0.0, a_tangential_lp: 0.0, command_lp: 0.0,
            r_eff: cfg.r_accel, jerk_meas: 0.0, a_tangential: 0.0,
            a_meas: 0.0, a_raw: 0.0, pitch_sensor: 0.0, pitch_cg: 0.0,
            cfg,
        }
    }

    #[inline(always)]
    fn clamp(x: f32, lo: f32, hi: f32) -> f32 {
        if x < lo { lo } else if x > hi { hi } else { x }
    }

    #[inline(always)]
    fn lerp(a: f32, b: f32, t: f32) -> f32 {
        a + (b - a) * t
    }

    fn symmetrize(p: &mut [[f32; 4]; 4]) {
        for i in 0..4 {
            for j in (i + 1)..4 {
                let v = 0.5 * (p[i][j] + p[j][i]);
                p[i][j] = v; p[j][i] = v;
            }
            if p[i][i] < 0.0 { p[i][i] = 0.0; }
        }
    }

    pub fn update(&mut self, command: f32, ax_g: f32, pitch_sensor: f32, pitch_cg: f32, angular_accel: f32) -> f32 {
        let dt = self.cfg.dt;

        // ───── 0. コマンドLPF ─────
        let alpha_cmd = Self::clamp(dt / (self.cfg.command_lpf_tau + dt + 1e-6), 0.0, 1.0);
        self.command_lp += alpha_cmd * (command - self.command_lp);

        // ───── 1. 加速度センサ（予測の駆動源） ─────
        // 【絶対遵守仕様】マイナス符号なしで計算
        let a_raw = (ax_g + sinf(pitch_sensor)) * 9.81;

        // 接線加速度補正
        let raw_a_tangential = angular_accel * self.cfg.robot_radius * self.cfg.tangential_scale;
        let alpha_tan = Self::clamp(dt / (self.cfg.tangential_lpf_tau + dt + 1e-6), 0.0, 1.0);
        self.a_tangential_lp += alpha_tan * (raw_a_tangential - self.a_tangential_lp);

        let a_meas = a_raw + self.a_tangential_lp;
        self.a_tangential = self.a_tangential_lp;

        self.a_meas = a_meas; self.a_raw = a_raw;
        self.pitch_sensor = pitch_sensor; self.pitch_cg = pitch_cg;

        // ───── 2. jerk・adaptive R（command観測の信頼度） ─────
        let j_meas = (a_raw - self.prev_a_raw) / dt;
        self.jerk_meas = j_meas;
        let jerk_ratio = j_meas.abs() / (self.cfg.j_max + 1e-6);
        let a_ratio = self.a.abs() / (self.cfg.a_max + 1e-6);
        let v_ratio = self.v.abs() / (self.cfg.v_max + 1e-6);

        let r_scale = 1.0
            + self.cfg.jerk_r_scale * (jerk_ratio * jerk_ratio)
            + self.cfg.accel_r_scale * (a_ratio * a_ratio)
            + self.cfg.vel_r_scale * (v_ratio * v_ratio);
        self.r_eff = self.cfg.r_accel * r_scale;

        // ───── 3. trustとプロセスノイズ ─────
        let tau_b = Self::lerp(self.cfg.tau_b_min, self.cfg.tau_b_max, self.trust);
        let beta_b = dt / (tau_b + dt + 1e-6);
        let psi_b = 1.0 - beta_b;
        let q_b = Self::lerp(self.cfg.q_b_min, self.cfg.q_b_max, self.trust);

        // ───── 4. 状態予測（加速度センサ駆動） ─────
        let alpha_a_nom = Self::clamp(dt / (self.cfg.tau_a + dt + 1e-6), 0.0, 1.0);
        let mut da = alpha_a_nom * (a_meas - self.a);
        let da_max = self.cfg.j_max * dt;
        da = Self::clamp(da, -da_max, da_max);

        let denom = a_meas - self.a;
        let alpha_a = if denom.abs() > 1e-9 { (da / denom).abs() } else { 0.0 };
        let alpha_a = Self::clamp(alpha_a, 0.0, alpha_a_nom);

        self.a += da;
        self.v += self.a * dt;
        self.x += self.v * dt;
        self.b *= psi_b;

        // ───── 5. 共分散予測 ─────
        let phi = 1.0 - alpha_a;

        let mut f = [[0.0f32; 4]; 4];
        f[0][0] = 1.0; f[0][2] = dt;
        f[1][0] = dt;  f[1][1] = 1.0; f[1][2] = dt * dt;
        f[2][2] = phi;
        f[3][3] = psi_b;

        let mut q = [[0.0f32; 4]; 4];
        q[0][0] = self.cfg.q_v * dt;
        q[2][2] = self.cfg.q_a * dt;
        q[3][3] = q_b * dt;

        let p0 = self.p;
        let mut fp = [[0.0f32; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                let mut s = 0.0;
                for k in 0..4 { s += f[i][k] * p0[k][j]; }
                fp[i][j] = s;
            }
        }
        let mut p_pred = [[0.0f32; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                let mut s = 0.0;
                for k in 0..4 { s += fp[i][k] * f[j][k]; }
                p_pred[i][j] = s + q[i][j];
            }
        }
        Self::symmetrize(&mut p_pred);
        self.p = p_pred;

        // ───── 6. 観測更新（command） ─────
        let gravity_comp = 9.81 * sinf(pitch_cg);
        let z_cmd = -self.cfg.k0 * self.command_lp + gravity_comp;

        self.innovation = z_cmd - (self.a + self.b);

        self.prev_innov_lp = self.innov_lp;
        self.innov_lp += self.cfg.innov_lpf_alpha * (self.innovation - self.innov_lp);

        let flipped = if (self.innov_lp * self.prev_innov_lp) < 0.0 { 1.0 } else { 0.0 };
        self.flip_lp += self.cfg.flip_lpf_alpha * (flipped - self.flip_lp);
        let c_flip = 1.0 - Self::clamp(self.flip_lp, 0.0, 1.0);

        let c_j = 1.0 / (1.0 + jerk_ratio * jerk_ratio);
        let c_r = Self::clamp(self.innov_lp.abs() / (self.cfg.bias_residual_scale + 1e-6), 0.0, 1.0);
        let c_a = 1.0 / (1.0 + a_ratio * a_ratio);
        let c_v = 1.0 / (1.0 + v_ratio * v_ratio);
        let trust_next = Self::clamp(c_j * c_r * c_flip * c_a * c_v, 0.0, 1.0);

        // H = [0, 0, 1, 1]
        let paa = self.p[2][2];
        let pbb = self.p[3][3];
        let pab = self.p[2][3];
        let s = paa + pbb + 2.0 * pab + self.r_eff;
        let s_inv = 1.0 / (s + 1e-9);

        let mut k_gain = [0.0f32; 4];
        for i in 0..4 {
            k_gain[i] = (self.p[i][2] + self.p[i][3]) * s_inv;
        }

        self.v += k_gain[0] * self.innovation;
        self.x += k_gain[1] * self.innovation;
        self.a += k_gain[2] * self.innovation;
        self.b += k_gain[3] * self.innovation;

        let mut a_mat = [[0.0f32; 4]; 4];
        for i in 0..4 {
            a_mat[i][i] = 1.0;
            a_mat[i][2] -= k_gain[i];
            a_mat[i][3] -= k_gain[i];
        }

        let p_before = self.p;
        let mut ap = [[0.0f32; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                let mut ss = 0.0;
                for k in 0..4 { ss += a_mat[i][k] * p_before[k][j]; }
                ap[i][j] = ss;
            }
        }
        let mut p_new = [[0.0f32; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                let mut ss = 0.0;
                for k in 0..4 { ss += ap[i][k] * a_mat[j][k]; }
                p_new[i][j] = ss;
            }
        }

        for i in 0..4 {
            for j in 0..4 { p_new[i][j] += k_gain[i] * self.r_eff * k_gain[j]; }
        }

        Self::symmetrize(&mut p_new);
        self.p = p_new;
        self.trust = trust_next;
        self.prev_a_raw = a_raw;

        // ───── 7. レギュレータ出力 ─────
        let raw = self.cfg.k_pos * self.x + self.cfg.k_vel * self.v;
        self.cfg.max_output * tanhf(raw / self.cfg.max_output)
    }

    pub fn state(&self) -> PosEkfState {
        let gravity_comp = 9.81 * sinf(self.pitch_cg);
        let z = -self.cfg.k0 * self.command_lp + gravity_comp;

        PosEkfState {
            velocity: self.v, position: self.x, accel: self.a, bias: self.b,
            innovation: self.innovation, innov_lp: self.innov_lp, trust: self.trust,
            r_eff: self.r_eff, jerk_meas: self.jerk_meas, a_tangential: self.a_tangential,
            a_meas: self.a_meas, a_raw: self.a_raw, pitch_sensor: self.pitch_sensor,
            pitch_cg: self.pitch_cg,
            command_lp: self.command_lp,
            z_cmd: z,
        }
    }

    pub fn reset(&mut self) {
        self.v = 0.0; self.x = 0.0; self.a = 0.0; self.b = 0.0;
        self.p = [[0.0; 4]; 4];
        self.p[0][0] = 0.01; self.p[1][1] = 0.01; self.p[2][2] = 0.5; self.p[3][3] = 0.1;
        self.trust = 0.0; self.innovation = 0.0; self.innov_lp = 0.0; self.prev_innov_lp = 0.0; self.flip_lp = 0.0;
        self.prev_a_raw = 0.0; self.a_tangential_lp = 0.0; self.command_lp = 0.0;
        self.r_eff = self.cfg.r_accel; self.jerk_meas = 0.0; self.a_tangential = 0.0;
        self.a_meas = 0.0; self.a_raw = 0.0; self.pitch_sensor = 0.0; self.pitch_cg = 0.0;
    }
}