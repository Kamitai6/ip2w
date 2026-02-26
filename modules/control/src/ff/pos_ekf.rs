use libm::{sinf, cosf, tanhf};

/// 並進位置推定EKF + レギュレータ（v7: ワールド座標変換修正）
///
/// 【v6e→v7 変更】
/// - a_rawの計算にaz_gとcos(pitch)を追加し、ボディ→ワールド座標変換を正しく実施
/// - 旧: a_raw = (ax_g + sinf(pitch_sensor)) * 9.81
/// - 新: a_raw = (cosf(pitch_sensor) * ax_g + sinf(pitch_sensor) * az_g) * 9.81
/// - 重力成分は代数的に消える（separate除去不要）
///
/// 【絶対遵守仕様（AIへの指示・忘却防止用）】
/// 1. a_rawの式は上記の通り。符号変更禁止。
/// 2. a_measの符号反転は物理モデルに基づく座標変換であり、上記仕様とは独立。
///
/// 状態: [v, x, a]（3状態）
///   v: 並進速度 [m/s]
///   x: 並進位置 [m]
///   a: 並進加速度 [m/s²]
///
/// 観測:
///   1. z=a_meas, H=[0,0,1], R=r_eff（加速度センサ、短期補正）
///   2. z=0,      H=[0,1,0], R=r_pos（位置参照、長期ドリフト抑制）
pub struct PosEkfConfig {
    // ── 物理 ──
    pub dt: f32,
    pub torque_to_pwm: f32,
    pub inertia: f32,
    pub wheel_radius: f32,
    pub mass: f32,
    pub robot_radius: f32,
    pub motor_efficiency: f32,

    // ── 入力フィルタ ──
    pub command_lpf_tau: f32,
    pub sign_lpf_tau: f32,

    // ── ハードウェア制約 ──
    pub j_max: f32,
    pub a_max: f32,
    pub v_max: f32,

    // ── 観測ノイズ ──
    pub r_accel: f32,
    pub constraint_r_scale: f32,
    pub r_pos: f32,

    // ── プロセスノイズ ──
    pub q_a_min: f32,
    pub q_a_max: f32,
    pub q_v: f32,
    pub q_x: f32,

    // ── レギュレータ ──
    pub k_pos: f32,
    pub k_vel: f32,
    pub max_output: f32,
}

impl Default for PosEkfConfig {
    fn default() -> Self {
        Self {
            dt: 0.002,

            torque_to_pwm: 10000.0,
            inertia: 0.000363,
            wheel_radius: 0.03,
            mass: 0.1,
            robot_radius: 0.08,
            motor_efficiency: 0.5,

            command_lpf_tau: 0.016,
            sign_lpf_tau: 0.4,

            j_max: 400.0,
            a_max: 50.0,
            v_max: 5.0,

            r_accel: 0.1,
            constraint_r_scale: 2.0,
            r_pos: 1.0,

            q_a_min: 1.0,
            q_a_max: 10.0,
            q_v: 1e-5,
            q_x: 1e-6,

            k_pos: 0.1,
            k_vel: 0.1,
            max_output: 0.35,
        }
    }
}

#[derive(Clone, Copy)]
pub struct PosEkfState {
    pub velocity: f32,
    pub position: f32,
    pub accel: f32,

    pub innovation: f32,
    pub innovation_pos: f32,
    pub sign_agree_lp: f32,
    pub r_eff: f32,

    pub a_meas: f32,
    pub a_raw: f32,
    pub a_target: f32,

    pub command_lp: f32,
    pub tangential_cmd: f32,
    pub tangential_sensor: f32,

    pub q_a: f32,
    pub k_gain_a: f32,
    pub k_gain_pos: f32,
}

pub struct PositionEkf {
    v: f32,
    x: f32,
    a: f32,

    p: [[f32; 3]; 3],

    sign_agree_lp: f32,
    innovation: f32,
    innovation_pos: f32,

    prev_a_meas: f32,
    prev_a_target: f32,
    command_lp: f32,

    r_eff: f32,
    a_meas: f32,
    a_raw: f32,
    a_target: f32,
    tangential_cmd: f32,
    tangential_sensor: f32,
    q_a_actual: f32,
    k_gain_a: f32,
    k_gain_pos: f32,

    cfg: PosEkfConfig,
}

impl PositionEkf {
    pub fn new(cfg: PosEkfConfig) -> Self {
        let mut p = [[0.0f32; 3]; 3];
        p[0][0] = 0.01;
        p[1][1] = 0.01;
        p[2][2] = 0.5;

        Self {
            v: 0.0, x: 0.0, a: 0.0,
            p,
            sign_agree_lp: 0.5,
            innovation: 0.0, innovation_pos: 0.0,
            prev_a_meas: 0.0, prev_a_target: 0.0, command_lp: 0.0,
            r_eff: cfg.r_accel, a_meas: 0.0, a_raw: 0.0, a_target: 0.0,
            tangential_cmd: 0.0, tangential_sensor: 0.0,
            q_a_actual: cfg.q_a_max,
            k_gain_a: 0.0, k_gain_pos: 0.0,
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

    fn symmetrize(p: &mut [[f32; 3]; 3]) {
        for i in 0..3 {
            for j in (i + 1)..3 {
                let v = 0.5 * (p[i][j] + p[j][i]);
                p[i][j] = v; p[j][i] = v;
            }
            if p[i][i] < 0.0 { p[i][i] = 0.0; }
        }
    }

    /// スカラー観測のJoseph形式更新
    /// h_idx: H行列で1になる列インデックス
    fn scalar_observation_update(&mut self, h_idx: usize, innovation: f32, r: f32) -> [f32; 3] {
        let s = self.p[h_idx][h_idx] + r;
        let s_inv = 1.0 / (s + 1e-9);

        let mut k = [0.0f32; 3];
        for i in 0..3 {
            k[i] = self.p[i][h_idx] * s_inv;
        }

        self.v += k[0] * innovation;
        self.x += k[1] * innovation;
        self.a += k[2] * innovation;

        // Joseph形式: P = (I - K*H) * P * (I - K*H)^T + K*R*K^T
        let mut a_mat = [[0.0f32; 3]; 3];
        for i in 0..3 {
            a_mat[i][i] = 1.0;
            a_mat[i][h_idx] -= k[i];
        }

        let p_before = self.p;
        let mut ap = [[0.0f32; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                let mut ss = 0.0;
                for kk in 0..3 { ss += a_mat[i][kk] * p_before[kk][j]; }
                ap[i][j] = ss;
            }
        }
        let mut p_new = [[0.0f32; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                let mut ss = 0.0;
                for kk in 0..3 { ss += ap[i][kk] * a_mat[j][kk]; }
                p_new[i][j] = ss;
            }
        }
        for i in 0..3 {
            for j in 0..3 { p_new[i][j] += k[i] * r * k[j]; }
        }

        Self::symmetrize(&mut p_new);
        self.p = p_new;

        k
    }

    /// # Arguments
    /// * `command` - 制御出力（total_output）
    /// * `ax_g` - 加速度センサX軸 [G]
    /// * `az_g` - 加速度センサZ軸 [G]
    /// * `pitch_sensor` - ピッチ角（センサ位置基準）[rad]
    /// * `pitch_cg` - ピッチ角（重心基準）[rad]（未使用）
    /// * `gyro_angular_accel` - ジャイロLPF微分による角加速度 [rad/s²]
    pub fn update(&mut self, command: f32, ax_g: f32, az_g: f32, pitch_sensor: f32, _pitch_cg: f32, gyro_angular_accel: f32) -> f32 {
        let dt = self.cfg.dt;

        // ───── 0. コマンドLPF ─────
        let alpha_cmd = Self::clamp(dt / (self.cfg.command_lpf_tau + dt + 1e-6), 0.0, 1.0);
        self.command_lp += alpha_cmd * (command - self.command_lp);

        // ───── 1. 接線加速度（ジャイロ由来） ─────
        let tangential_sensor = gyro_angular_accel * self.cfg.robot_radius * 0.8;
        let tangential_cmd = self.cfg.inertia * gyro_angular_accel * 0.8;
        self.tangential_sensor = tangential_sensor;
        self.tangential_cmd = tangential_cmd / (self.cfg.wheel_radius * self.cfg.mass);

        // ───── 2. 観測（センサ側の並進加速度） ─────
        // v7: ボディ→ワールド座標変換（重力は代数的に消える）
        let a_raw = (cosf(pitch_sensor) * ax_g + sinf(pitch_sensor) * az_g) * 9.81;
        let a_meas = -(a_raw + tangential_sensor);
        self.a_raw = a_raw;
        self.a_meas = a_meas;

        // ───── 3. 予測の目標値（command側の並進加速度） ─────
        let tau_total = self.command_lp / self.cfg.torque_to_pwm;
        let a_target = (tau_total - tangential_cmd) * self.cfg.motor_efficiency / (self.cfg.wheel_radius * self.cfg.mass);
        self.a_target = a_target;

        // ───── 4. sign_agree（jerkベース） ─────
        let da_target = a_target - self.prev_a_target;
        let da_meas = a_meas - self.prev_a_meas;
        let sign_agree = if (da_target * da_meas) > 0.0 { 1.0 } else { 0.0 };
        let alpha_sign = Self::clamp(dt / (self.cfg.sign_lpf_tau + dt + 1e-6), 0.0, 1.0);
        self.sign_agree_lp += alpha_sign * (sign_agree - self.sign_agree_lp);

        // ───── 5. adaptive R ─────
        let j_meas = (a_meas - self.prev_a_meas) / dt;
        let jerk_ratio = j_meas.abs() / (self.cfg.j_max + 1e-6);
        let a_ratio = self.a.abs() / (self.cfg.a_max + 1e-6);
        let v_ratio = self.v.abs() / (self.cfg.v_max + 1e-6);

        let r_scale = 1.0
            + self.cfg.constraint_r_scale * (jerk_ratio * jerk_ratio + a_ratio * a_ratio + v_ratio * v_ratio);
        self.r_eff = self.cfg.r_accel * r_scale;

        // ───── 6. sign_agree連動プロセスノイズ ─────
        // sign低 → q_a大 → command予測を信頼しない → 観測優先
        let q_a = Self::lerp(self.cfg.q_a_min, self.cfg.q_a_max, 1.0 - self.sign_agree_lp);
        self.q_a_actual = q_a;

        // ───── 7. 状態予測（a = a_target直接、jerk/accel/vel制限あり） ─────
        let mut da = a_target - self.a;
        let da_max = self.cfg.j_max * dt;
        let jerk_limited = da.abs() > da_max;
        da = Self::clamp(da, -da_max, da_max);

        self.a += da;
        self.a = Self::clamp(self.a, -self.cfg.a_max, self.cfg.a_max);
        self.v += self.a * dt;
        self.v = Self::clamp(self.v, -self.cfg.v_max, self.cfg.v_max);
        self.x += self.v * dt;

        // ───── 8. 共分散予測 ─────
        // F = [1,  0,  dt]    (jerk制限時: ∂v/∂a = dt)
        //     [dt, 1,  dt²]   (jerk制限時: ∂x/∂a = dt²)
        //     [0,  0,  phi]   (phi=1:制限時, phi=0:非制限時)
        let phi = if jerk_limited { 1.0 } else { 0.0 };

        let mut f = [[0.0f32; 3]; 3];
        f[0][0] = 1.0;
        f[0][2] = phi * dt;
        f[1][0] = dt;
        f[1][1] = 1.0;
        f[1][2] = phi * dt * dt;
        f[2][2] = phi;

        let mut q = [[0.0f32; 3]; 3];
        q[0][0] = self.cfg.q_v * dt;
        q[1][1] = self.cfg.q_x * dt;
        q[2][2] = q_a * dt;

        let p0 = self.p;
        let mut fp = [[0.0f32; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                let mut s = 0.0;
                for k in 0..3 { s += f[i][k] * p0[k][j]; }
                fp[i][j] = s;
            }
        }
        let mut p_pred = [[0.0f32; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                let mut s = 0.0;
                for k in 0..3 { s += fp[i][k] * f[j][k]; }
                p_pred[i][j] = s + q[i][j];
            }
        }
        Self::symmetrize(&mut p_pred);
        self.p = p_pred;

        // ───── 9a. 観測更新1: 加速度（短期補正） ─────
        // H1 = [0, 0, 1], R1 = r_eff
        self.innovation = a_meas - self.a;
        let k1 = self.scalar_observation_update(2, self.innovation, self.r_eff);
        self.k_gain_a = k1[2];

        // ───── 9b. 観測更新2: 位置参照（長期ドリフト抑制） ─────
        // H2 = [0, 1, 0], R2 = r_pos
        // z = 0（目標位置）
        self.innovation_pos = 0.0 - self.x;
        let k2 = self.scalar_observation_update(1, self.innovation_pos, self.cfg.r_pos);
        self.k_gain_pos = k2[1];

        self.prev_a_meas = a_meas;
        self.prev_a_target = a_target;

        // ───── 10. レギュレータ出力 ─────
        let raw = self.cfg.k_pos * self.x + self.cfg.k_vel * self.v;
        self.cfg.max_output * tanhf(raw / self.cfg.max_output)
    }

    pub fn state(&self) -> PosEkfState {
        PosEkfState {
            velocity: self.v, position: self.x, accel: self.a,
            innovation: self.innovation, innovation_pos: self.innovation_pos,
            sign_agree_lp: self.sign_agree_lp, r_eff: self.r_eff,
            a_meas: self.a_meas, a_raw: self.a_raw, a_target: self.a_target,
            command_lp: self.command_lp,
            tangential_cmd: self.tangential_cmd, tangential_sensor: self.tangential_sensor,
            q_a: self.q_a_actual,
            k_gain_a: self.k_gain_a, k_gain_pos: self.k_gain_pos,
        }
    }

    pub fn reset(&mut self) {
        self.v = 0.0; self.x = 0.0; self.a = 0.0;
        self.p = [[0.0; 3]; 3];
        self.p[0][0] = 0.01; self.p[1][1] = 0.01; self.p[2][2] = 0.5;
        self.sign_agree_lp = 0.5; self.innovation = 0.0; self.innovation_pos = 0.0;
        self.prev_a_meas = 0.0; self.prev_a_target = 0.0; self.command_lp = 0.0;
        self.r_eff = self.cfg.r_accel; self.a_meas = 0.0; self.a_raw = 0.0; self.a_target = 0.0;
        self.tangential_cmd = 0.0; self.tangential_sensor = 0.0;
        self.q_a_actual = self.cfg.q_a_max;
        self.k_gain_a = 0.0; self.k_gain_pos = 0.0;
    }
}