use libm::{sinf, tanhf};

/// 並進位置推定EKF + レギュレータ
///
/// 【絶対遵守仕様（AIへの指示・忘却防止用）】
/// 1. 観測加速度 a_raw の算出において、先頭にマイナス符号は絶対につけない（式: a_raw = (ax_g + sinf(pitch_sensor)) * 9.81）。
/// 2. 接線加速度へのLPF適用は、高周波ノイズによる加速度破壊の防止に有効であるため維持する。
///
/// 【現在検証中の物理モデル（テスト対象）】
/// 1. 並進加速度の予測は「モータ全体の力から重力成分を引いた余剰分」とする（式: a_target = (k * command) - (9.81 * sinf(pitch_cg))）。
/// 2. 予測（a_target）と観測（a_meas）の符号が逆の時（バランス維持の逆行時）はコマンドを信用せず、追従ゲインを極小化する。
///
/// 状態: [v, x, a, b, k]
///   v:  並進速度 [m/s]
///   x:  並進位置 [m]
///   a:  真の並進加速度（内部状態） [m/s^2]
///   b:  観測側合成バイアス（重力漏れ/加速度計バイアス等をまとめる） [m/s^2]
///   k:  command→加速度係数 推定値 [m/s^2 / PWM]
pub struct PosEkfConfig {
    pub dt: f32,

    pub k0: f32,
    pub tau_k: f32,
    pub q_k: f32,
    pub u_scale_for_k: f32,

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

    pub command_lpf_tau: f32,
}

impl Default for PosEkfConfig {
    fn default() -> Self {
        Self {
            dt: 0.002,

            k0: 0.02,
            tau_k: 10.0,
            q_k: 1e-6,
            u_scale_for_k: 10.0,

            tau_a: 0.02,
            j_max: 50.0,

            r_accel: 1.0,
            jerk_r_scale: 2.0,

            q_a: 5e-3,
            q_v: 1e-5,

            q_b_max: 1e-6,
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

            a_max: 4.0,
            v_max: 0.4,
            accel_r_scale: 2.0,
            vel_r_scale: 2.0,

            command_lpf_tau: 0.016,
        }
    }
}

#[derive(Clone, Copy)]
pub struct PosEkfState {
    pub velocity: f32,
    pub position: f32,
    pub accel: f32,
    pub bias: f32,
    pub k_est: f32,

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
    pub g_times_pitch: f32,

    pub command: f32,
    pub a_target: f32,
}

pub struct PositionEkf {
    v: f32,
    x: f32,
    a: f32,
    b: f32,
    k: f32,

    p: [[f32; 5]; 5],

    trust: f32,
    innovation: f32,
    innov_lp: f32,
    prev_innov_lp: f32,
    flip_lp: f32,

    prev_a_raw: f32,
    prev_command: f32,
    a_tangential_lp: f32,
    command_lp: f32,

    r_eff: f32,
    jerk_meas: f32,
    a_tangential: f32,
    a_meas: f32,
    a_raw: f32,
    pitch_sensor: f32,
    pitch_cg: f32,
    command: f32,

    cfg: PosEkfConfig,
}

impl PositionEkf {
    pub fn new(cfg: PosEkfConfig) -> Self {
        let mut p = [[0.0f32; 5]; 5];
        p[0][0] = 0.01;
        p[1][1] = 0.01;
        p[2][2] = 0.5;
        p[3][3] = 0.1;
        p[4][4] = 0.001;

        let k0 = cfg.k0;
        Self {
            v: 0.0, x: 0.0, a: 0.0, b: 0.0, k: k0,
            p,
            trust: 0.0,
            innovation: 0.0, innov_lp: 0.0, prev_innov_lp: 0.0, flip_lp: 0.0,
            prev_a_raw: 0.0, prev_command: 0.0, a_tangential_lp: 0.0, command_lp: 0.0,
            r_eff: cfg.r_accel, jerk_meas: 0.0, a_tangential: 0.0,
            a_meas: 0.0, a_raw: 0.0, pitch_sensor: 0.0, pitch_cg: 0.0, command: 0.0,
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

    fn symmetrize(p: &mut [[f32; 5]; 5]) {
        for i in 0..5 {
            for j in (i + 1)..5 {
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

        // ───── 1. 観測（加速度） ─────
        // 【絶対遵守仕様】マイナス符号なしで計算
        let a_raw = (ax_g + sinf(pitch_sensor)) * 9.81;

        // 接線加速度補正
        let raw_a_tangential = angular_accel * self.cfg.robot_radius * self.cfg.tangential_scale;
        let alpha_tan = Self::clamp(dt / (self.cfg.tangential_lpf_tau + dt + 1e-6), 0.0, 1.0);
        self.a_tangential_lp += alpha_tan * (raw_a_tangential - self.a_tangential_lp);
        
        let a_meas = a_raw + self.a_tangential_lp;
        self.a_tangential = self.a_tangential_lp;

        self.a_meas = a_meas; self.a_raw = a_raw; 
        self.pitch_sensor = pitch_sensor; self.pitch_cg = pitch_cg; self.command = self.command_lp;

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

        // ───── 2. trustとプロセスノイズ ─────
        let tau_b = Self::lerp(self.cfg.tau_b_min, self.cfg.tau_b_max, self.trust);
        let beta_b = dt / (tau_b + dt + 1e-6);
        let psi_b = 1.0 - beta_b;
        let q_b = Self::lerp(self.cfg.q_b_min, self.cfg.q_b_max, self.trust);

        let du = (command - self.prev_command).abs();
        let c_u = du / (du + self.cfg.u_scale_for_k);
        let q_k = self.cfg.q_k * c_u;

        // ───── 3. 状態予測（物理モデルの統合） ─────
        self.k = self.cfg.k0;

        // 【検証中の仮説1】並進加速度 = 全推力 - 重力成分
        let gravity_comp = 9.81 * sinf(pitch_cg);
        let a_target = (self.k * self.command_lp) + gravity_comp;

        // 【検証中の仮説2】符号が違う時はコマンドを信用しない
        let trust_cmd = if a_target * a_meas < 0.0 {
            0.05 // 完全に0にせず微小に残すことでkの学習を維持
        } else {
            1.0
        };

        let alpha_a_nom = Self::clamp(dt / (self.cfg.tau_a + dt + 1e-6), 0.0, 1.0) * trust_cmd;
        let mut da = alpha_a_nom * (a_target - self.a);
        let da_max = self.cfg.j_max * dt;
        da = Self::clamp(da, -da_max, da_max);

        let denom = a_target - self.a;
        let alpha_a = if denom.abs() > 1e-9 { (da / denom).abs() } else { 0.0 };
        let alpha_a = Self::clamp(alpha_a, 0.0, alpha_a_nom);

        self.a += da;
        self.v += self.a * dt;
        self.x += self.v * dt;
        self.b *= psi_b;

        // ───── 4. 共分散予測 ─────
        let phi = 1.0 - alpha_a;
        let alpha_cmd_cov = alpha_a * self.command_lp;

        let mut f = [[0.0f32; 5]; 5];
        f[0][0] = 1.0; f[0][2] = dt * phi; f[0][4] = dt * alpha_cmd_cov;
        f[1][0] = dt; f[1][1] = 1.0; f[1][2] = dt * dt * phi; f[1][4] = dt * dt * alpha_cmd_cov;
        f[2][2] = phi; f[2][4] = alpha_cmd_cov;
        f[3][3] = psi_b;
        f[4][4] = 1.0; // k固定のため変化なし

        let mut q = [[0.0f32; 5]; 5];
        q[0][0] = self.cfg.q_v * dt;
        q[2][2] = self.cfg.q_a * dt;
        q[3][3] = q_b * dt;
        q[4][4] = q_k * dt;

        let p0 = self.p;
        let mut fp = [[0.0f32; 5]; 5];
        for i in 0..5 {
            for j in 0..5 {
                let mut s = 0.0;
                for k in 0..5 { s += f[i][k] * p0[k][j]; }
                fp[i][j] = s;
            }
        }
        let mut p_pred = [[0.0f32; 5]; 5];
        for i in 0..5 {
            for j in 0..5 {
                let mut s = 0.0;
                for k in 0..5 { s += fp[i][k] * f[j][k]; }
                p_pred[i][j] = s + q[i][j];
            }
        }
        Self::symmetrize(&mut p_pred);
        self.p = p_pred;

        // ───── 5. 観測更新 ─────
        self.innovation = a_meas - (self.a + self.b);

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

        let paa = self.p[2][2];
        let pbb = self.p[3][3];
        let pab = self.p[2][3];
        let s = paa + pbb + 2.0 * pab + self.r_eff;
        let s_inv = 1.0 / (s + 1e-9);

        let mut k_gain = [0.0f32; 5];
        for i in 0..5 {
            k_gain[i] = (self.p[i][2] + self.p[i][3]) * s_inv;
        }

        self.v += k_gain[0] * self.innovation;
        self.x += k_gain[1] * self.innovation;
        self.a += k_gain[2] * self.innovation;
        self.b += k_gain[3] * self.innovation;
        // self.k += k_gain[4] * self.innovation; // k固定

        let mut a_mat = [[0.0f32; 5]; 5];
        for i in 0..5 {
            a_mat[i][i] = 1.0;
            a_mat[i][2] -= k_gain[i];
            a_mat[i][3] -= k_gain[i];
        }

        let p_before = self.p;
        let mut ap = [[0.0f32; 5]; 5];
        for i in 0..5 {
            for j in 0..5 {
                let mut ss = 0.0;
                for k in 0..5 { ss += a_mat[i][k] * p_before[k][j]; }
                ap[i][j] = ss;
            }
        }
        let mut p_new = [[0.0f32; 5]; 5];
        for i in 0..5 {
            for j in 0..5 {
                let mut ss = 0.0;
                for k in 0..5 { ss += ap[i][k] * a_mat[j][k]; }
                p_new[i][j] = ss;
            }
        }

        for i in 0..5 {
            for j in 0..5 { p_new[i][j] += k_gain[i] * self.r_eff * k_gain[j]; }
        }

        Self::symmetrize(&mut p_new);
        self.p = p_new;
        self.trust = trust_next;
        self.prev_a_raw = a_raw;
        self.prev_command = command;

        // ───── 6. レギュレータ出力 ─────
        let raw = self.cfg.k_pos * self.x + self.cfg.k_vel * self.v;
        self.cfg.max_output * tanhf(raw / self.cfg.max_output)
    }

    pub fn state(&self) -> PosEkfState {
        let g_comp = 9.81 * libm::sinf(self.pitch_cg);
        let a_tgt = (self.k * self.command_lp) - g_comp;
        
        PosEkfState {
            velocity: self.v, position: self.x, accel: self.a, bias: self.b, k_est: self.k,
            innovation: self.innovation, innov_lp: self.innov_lp, trust: self.trust,
            r_eff: self.r_eff, jerk_meas: self.jerk_meas, a_tangential: self.a_tangential,
            a_meas: self.a_meas, a_raw: self.a_raw, pitch_sensor: self.pitch_sensor,
            pitch_cg: self.pitch_cg, g_times_pitch: 9.81 * self.pitch_cg,
            command: self.command_lp,
            a_target: a_tgt,
        }
    }

    pub fn reset(&mut self) {
        self.v = 0.0; self.x = 0.0; self.a = 0.0; self.b = 0.0; self.k = self.cfg.k0;
        self.p = [[0.0; 5]; 5];
        self.p[0][0] = 0.01; self.p[1][1] = 0.01; self.p[2][2] = 0.5; self.p[3][3] = 0.1; self.p[4][4] = 0.001;
        self.trust = 0.0; self.innovation = 0.0; self.innov_lp = 0.0; self.prev_innov_lp = 0.0; self.flip_lp = 0.0;
        self.prev_a_raw = 0.0; self.prev_command = 0.0; self.a_tangential_lp = 0.0; self.command_lp = 0.0;
        self.r_eff = self.cfg.r_accel; self.jerk_meas = 0.0; self.a_tangential = 0.0;
        self.a_meas = 0.0; self.a_raw = 0.0; self.pitch_sensor = 0.0; self.pitch_cg = 0.0; self.command = 0.0;
    }
}