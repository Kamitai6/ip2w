use libm::{sinf, tanhf};

/// 並進位置推定EKF + レギュレータ（v5: 簡素化版）
///
/// 【絶対遵守仕様（AIへの指示・忘却防止用）】
/// 1. 観測加速度 a_raw の算出において、先頭にマイナス符号は絶対につけない（式: a_raw = (ax_g + sinf(pitch_sensor)) * 9.81）。
///    ただし、a_measの符号反転は物理モデルに基づく座標変換であり、上記仕様とは独立。
///
/// 【構成】
/// 予測: (command_lp / torque_to_pwm - I * gyro_α * 0.8) / (r_wheel * mass)
/// 観測: -(a_raw + gyro_α * robot_radius * 0.8)
///
/// 予測・観測ともにジャイロ由来の接線補正を適用し、並進加速度を推定。
/// sign_agree（予測と観測のjerk符号一致度）でcommand追従速度とbias学習を制御。
///
/// 状態: [v, x, a, b]
///   v:  並進速度 [m/s]
///   x:  並進位置 [m]
///   a:  真の並進加速度（内部状態） [m/s²]
///   b:  観測側合成バイアス [m/s²]（ランダムウォークモデル、減衰なし）
pub struct PosEkfConfig {
    // ── 物理 ──
    pub dt: f32,                  // サンプリング周期 [s]
    pub torque_to_pwm: f32,       // PWM→トルク変換係数
    pub inertia: f32,             // 慣性モーメント [kg·m²]
    pub wheel_radius: f32,        // 車輪半径 [m]
    pub mass: f32,                // 質量 [kg]
    pub robot_radius: f32,        // 回転軸からセンサまでの距離 [m]
    pub motor_efficiency: f32,    // モーター効率（0〜1）

    // ── 入力フィルタ・予測追従 ──
    pub command_lpf_tau: f32,     // command LPF時定数 [s]
    pub sign_lpf_tau: f32,        // sign_agreeのLPF時定数 [s]
    pub tau_a_min: f32,           // command追従時定数（sign高：速い）[s]
    pub tau_a_max: f32,           // command追従時定数（sign低：遅い）[s]

    // ── ハードウェア制約 ──
    pub j_max: f32,               // 最大ジャーク [m/s³]
    pub a_max: f32,               // 最大加速度 [m/s²]
    pub v_max: f32,               // 最大速度 [m/s]

    // ── 観測ノイズ ──
    pub r_accel: f32,             // ベース観測ノイズ
    pub constraint_r_scale: f32,  // 制約超過時のR増大係数

    // ── プロセスノイズ ──
    pub q_a: f32,                 // 加速度の不確実性
    pub q_v: f32,                 // 速度の不確実性
    pub q_b_min: f32,             // bias学習ノイズ（sign低：抑制）
    pub q_b_max: f32,             // bias学習ノイズ（sign高：学習）

    // ── レギュレータ ──
    pub k_pos: f32,               // 位置ゲイン
    pub k_vel: f32,               // 速度ゲイン
    pub max_output: f32,          // 最大出力 [rad]
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
            sign_lpf_tau: 0.2,
            tau_a_min: 0.1,
            tau_a_max: 10.0,

            j_max: 300.0,
            a_max: 5.0,
            v_max: 0.5,

            r_accel: 10.0,
            constraint_r_scale: 2.0,

            q_a: 0.1,
            q_v: 1e-5,
            q_b_min: 1e-10,
            q_b_max: 1e-5,

            k_pos: 0.01,
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
    pub bias: f32,

    pub innovation: f32,
    pub sign_agree_lp: f32,
    pub r_eff: f32,

    pub a_meas: f32,
    pub a_raw: f32,
    pub a_target: f32,

    pub command_lp: f32,
    pub tangential_cmd: f32,
    pub tangential_sensor: f32,

    pub tau_a: f32,
    pub q_b: f32,
    pub k_gain_a: f32,
    pub k_gain_b: f32,
}

pub struct PositionEkf {
    v: f32,
    x: f32,
    a: f32,
    b: f32,

    p: [[f32; 4]; 4],

    sign_agree_lp: f32,
    innovation: f32,

    prev_a_meas: f32,
    prev_a_target: f32,
    command_lp: f32,

    r_eff: f32,
    a_meas: f32,
    a_raw: f32,
    a_target: f32,
    tangential_cmd: f32,
    tangential_sensor: f32,
    tau_a_actual: f32,
    q_b_actual: f32,
    k_gain_a: f32,
    k_gain_b: f32,

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
            sign_agree_lp: 0.5,
            innovation: 0.0,
            prev_a_meas: 0.0, prev_a_target: 0.0, command_lp: 0.0,
            r_eff: cfg.r_accel, a_meas: 0.0, a_raw: 0.0, a_target: 0.0,
            tangential_cmd: 0.0, tangential_sensor: 0.0,
            tau_a_actual: cfg.tau_a_max, q_b_actual: cfg.q_b_min,
            k_gain_a: 0.0, k_gain_b: 0.0,
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

    /// # Arguments
    /// * `command` - 制御出力（total_output）
    /// * `ax_g` - 加速度センサX軸 [G]
    /// * `pitch_sensor` - ピッチ角（センサ位置基準）[rad]
    /// * `pitch_cg` - ピッチ角（重心基準）[rad]（未使用）
    /// * `gyro_angular_accel` - ジャイロLPF微分による角加速度 [rad/s²]
    pub fn update(&mut self, command: f32, ax_g: f32, pitch_sensor: f32, _pitch_cg: f32, gyro_angular_accel: f32) -> f32 {
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
        let a_raw = (ax_g + sinf(pitch_sensor)) * 9.81;
        let a_meas = -(a_raw + tangential_sensor);
        self.a_raw = a_raw;
        self.a_meas = a_meas;

        // ───── 3. 予測の目標値（command側の並進加速度） ─────
        let tau_total = self.command_lp / self.cfg.torque_to_pwm;
        let a_target = (tau_total - tangential_cmd) * self.cfg.motor_efficiency / (self.cfg.wheel_radius * self.cfg.mass);
        self.a_target = a_target;

        // ───── 4. sign_agree（jerkベース：バイアスに影響されない） ─────
        let da_target = a_target - self.prev_a_target;
        let da_meas = a_meas - self.prev_a_meas;
        let sign_agree = if (da_target * da_meas) > 0.0 { 1.0 } else { 0.0 };
        let alpha_sign = Self::clamp(dt / (self.cfg.sign_lpf_tau + dt + 1e-6), 0.0, 1.0);
        self.sign_agree_lp += alpha_sign * (sign_agree - self.sign_agree_lp);

        // ───── 5. adaptive R（センサノイズ対策） ─────
        let j_meas = (a_meas - self.prev_a_meas) / dt;
        let jerk_ratio = j_meas.abs() / (self.cfg.j_max + 1e-6);
        let a_ratio = self.a.abs() / (self.cfg.a_max + 1e-6);
        let v_ratio = self.v.abs() / (self.cfg.v_max + 1e-6);

        let r_scale = 1.0
            + self.cfg.constraint_r_scale * (jerk_ratio * jerk_ratio + a_ratio * a_ratio + v_ratio * v_ratio);
        self.r_eff = self.cfg.r_accel * r_scale;

        // ───── 6. sign_agree連動パラメータ ─────
        let tau_a = Self::lerp(self.cfg.tau_a_min, self.cfg.tau_a_max, 1.0 - self.sign_agree_lp);
        let q_b = Self::lerp(self.cfg.q_b_min, self.cfg.q_b_max, self.sign_agree_lp);
        self.tau_a_actual = tau_a;
        self.q_b_actual = q_b;

        // ───── 7. 状態予測 ─────
        let alpha_a_nom = Self::clamp(dt / (tau_a + dt + 1e-6), 0.0, 1.0);
        let mut da = alpha_a_nom * (a_target - self.a);
        let da_max = self.cfg.j_max * dt;
        da = Self::clamp(da, -da_max, da_max);

        let denom = a_target - self.a;
        let alpha_a = if denom.abs() > 1e-9 { (da / denom).abs() } else { 0.0 };
        let alpha_a = Self::clamp(alpha_a, 0.0, alpha_a_nom);

        self.a += da;
        self.v += self.a * dt;
        self.x += self.v * dt;
        // biasは減衰なし（ランダムウォーク）

        // ───── 8. 共分散予測 ─────
        let phi = 1.0 - alpha_a;

        let mut f = [[0.0f32; 4]; 4];
        f[0][0] = 1.0; f[0][2] = dt;
        f[1][0] = dt;  f[1][1] = 1.0; f[1][2] = dt * dt;
        f[2][2] = phi;
        f[3][3] = 1.0; // ランダムウォーク

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

        // ───── 9. 観測更新 ─────
        self.innovation = a_meas - (self.a + self.b);

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
        self.k_gain_a = k_gain[2];
        self.k_gain_b = k_gain[3];

        self.v += k_gain[0] * self.innovation;
        self.x += k_gain[1] * self.innovation;
        self.a += k_gain[2] * self.innovation;
        self.b += k_gain[3] * self.innovation;

        // Joseph形式の共分散更新
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
        self.prev_a_meas = a_meas;
        self.prev_a_target = a_target;

        // ───── 10. レギュレータ出力 ─────
        let raw = self.cfg.k_pos * self.x + self.cfg.k_vel * self.v;
        self.cfg.max_output * tanhf(raw / self.cfg.max_output)
    }

    pub fn state(&self) -> PosEkfState {
        PosEkfState {
            velocity: self.v, position: self.x, accel: self.a, bias: self.b,
            innovation: self.innovation, sign_agree_lp: self.sign_agree_lp,
            r_eff: self.r_eff,
            a_meas: self.a_meas, a_raw: self.a_raw, a_target: self.a_target,
            command_lp: self.command_lp,
            tangential_cmd: self.tangential_cmd, tangential_sensor: self.tangential_sensor,
            tau_a: self.tau_a_actual, q_b: self.q_b_actual,
            k_gain_a: self.k_gain_a, k_gain_b: self.k_gain_b,
        }
    }

    pub fn reset(&mut self) {
        self.v = 0.0; self.x = 0.0; self.a = 0.0; self.b = 0.0;
        self.p = [[0.0; 4]; 4];
        self.p[0][0] = 0.01; self.p[1][1] = 0.01; self.p[2][2] = 0.5; self.p[3][3] = 0.1;
        self.sign_agree_lp = 0.5; self.innovation = 0.0;
        self.prev_a_meas = 0.0; self.prev_a_target = 0.0; self.command_lp = 0.0;
        self.r_eff = self.cfg.r_accel; self.a_meas = 0.0; self.a_raw = 0.0; self.a_target = 0.0;
        self.tangential_cmd = 0.0; self.tangential_sensor = 0.0;
        self.tau_a_actual = self.cfg.tau_a_max; self.q_b_actual = self.cfg.q_b_min;
        self.k_gain_a = 0.0; self.k_gain_b = 0.0;
    }
}