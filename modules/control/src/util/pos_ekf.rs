use libm::{sinf, cosf};

/// 並進位置推定EKF（v9e: 推定専用、レギュレータ分離）
///
/// 【v9d→v9e 変更】
/// - レギュレータを外部PIDに分離。update()はPosEkfStateを返すのみ。
/// - state()メソッドを廃止、update()が直接PosEkfStateを返す。
/// - tanhf依存を削除。
/// - 予測ステップの積分に一次ホールド（FOH）の厳密解を適用
///
/// 【v9d からの継続仕様】
/// - 状態 [v, x, a, b]（4状態）、bはa_measセンサバイアス
/// - 観測: innovation = a_meas - (a + b), H = [0, 0, 1, 1]
/// - jerk/accel/velベースのadaptive Rは維持
///
/// 【絶対遵守仕様（AIへの指示・忘却防止用）】
/// 1. a_measの全体符号反転（a_meas = -v̇）は既存規約。変更禁止。
/// 2. v̇の式は実験で検証済み（+版が静止時0になることを確認）。
///
/// 状態: [v, x, a, b]（4状態）
///   v: 並進速度 [m/s]
///   x: 並進位置 [m]
///   a: 並進加速度 [m/s²]
///   b: a_measセンサバイアス [m/s²]
///
/// 観測:
///   z=a_meas, H=[0,0,1,1], R=r_eff（加速度センサ、バイアス補正付き）
pub struct PosEkfConfig {
    // ── 物理（連立方程式パラメータ） ──
    pub dt: f32,
    pub k_tau: f32,            // 出力軸トルク定数 [Nm/V]（datasheet: τ_stall / V）
    pub k_b: f32,              // 逆起電力トルク係数 [Nm·s/rad]（datasheet: τ_stall / ω_noload）
    pub v_batt: f32,           // バッテリ電圧 [V]（初期化時に実測値で上書き）
    pub pwm_max: f32,          // PWM最大値
    pub motor_efficiency: f32, // 駆動トルクに掛ける効率係数（<1で推定トルク減少）
    pub wheel_radius: f32,     // r: ホイール半径 [m]
    pub m_p: f32,              // 胴体（振り子）の質量 [kg]
    pub m_w: f32,              // ホイールの質量 [kg]（両輪合計）
    pub i_p: f32,              // 胴体の慣性モーメント [kg·m²]
    pub i_w: f32,              // ホイールの慣性モーメント [kg·m²]（両輪合計）
    pub l: f32,                // 回転軸から胴体重心までの距離 [m]
    pub imu_rx: f32,           // ホイール回転軸→IMU 前方オフセット [m]
    pub imu_rz: f32,           // ホイール回転軸→IMU 上方オフセット [m]

    // ── 入力フィルタ ──
    pub command_lpf_tau: f32,

    // ── ハードウェア制約 ──
    pub j_max: f32,
    pub a_max: f32,
    pub v_max: f32,

    // ── 観測ノイズ ──
    pub r_accel: f32,
    pub constraint_r_scale: f32,

    // ── プロセスノイズ ──
    pub q_a: f32,
    pub q_v: f32,
    pub q_x: f32,
    pub q_bias: f32,
}

impl Default for PosEkfConfig {
    fn default() -> Self {
        Self {
            dt: 0.002,

            k_tau: 0.002452,
            k_b: 0.001081,
            v_batt: 3.7,
            pwm_max: 300.0,
            motor_efficiency: 1.0,
            wheel_radius: 0.03,
            m_p: 0.1,
            m_w: 0.01,
            i_p: 0.000363,
            i_w: 0.000005,
            l: 0.035,
            imu_rx: 0.01,
            imu_rz: 0.08,

            command_lpf_tau: 0.003183,

            j_max: 400.0,
            a_max: 50.0,
            v_max: 5.0,

            r_accel: 0.01,
            constraint_r_scale: 2.0,

            q_a: 10.0,
            q_v: 1e-3,
            q_x: 1e-4,
            q_bias: 1e-4,
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
    pub r_eff: f32,

    pub a_meas: f32,
    pub a_raw: f32,
    pub a_target: f32,

    pub command_lp: f32,
    pub tangential_sensor: f32,
    pub a_centripetal: f32,
    pub tau_eff: f32,

    pub k_gain_a: f32,
    pub k_gain_bias: f32,
}

pub struct PositionEkf {
    v: f32,
    x: f32,
    a: f32,
    b: f32,

    p: [[f32; 4]; 4],

    // プリコンピュート定数
    m_total: f32,  // M = I_w/r² + m_w + m_p

    innovation: f32,

    prev_a_meas: f32,
    prev_a_target: f32,
    command_lp: f32,

    r_eff: f32,
    a_meas: f32,
    a_raw: f32,
    a_target: f32,
    tangential_sensor: f32,
    a_centripetal: f32,
    tau_eff: f32,
    k_gain_a: f32,
    k_gain_bias: f32,

    cfg: PosEkfConfig,
}

impl PositionEkf {
    const G: f32 = 9.81;

    pub fn new(cfg: PosEkfConfig) -> Self {
        let mut p = [[0.0f32; 4]; 4];
        p[0][0] = 0.01;
        p[1][1] = 0.01;
        p[2][2] = 0.5;
        p[3][3] = 0.001;

        let r = cfg.wheel_radius;
        let m_total = cfg.i_w / (r * r) + cfg.m_w + cfg.m_p;

        Self {
            v: 0.0, x: 0.0, a: 0.0, b: 0.0,
            p,
            m_total,
            innovation: 0.0,
            prev_a_meas: 0.0, prev_a_target: 0.0, command_lp: 0.0,
            r_eff: cfg.r_accel, a_meas: 0.0, a_raw: 0.0, a_target: 0.0,
            tangential_sensor: 0.0,
            a_centripetal: 0.0,
            tau_eff: 0.0,
            k_gain_a: 0.0, k_gain_bias: 0.0,
            cfg,
        }
    }

    #[inline(always)]
    fn clamp(x: f32, lo: f32, hi: f32) -> f32 {
        if x < lo { lo } else if x > hi { hi } else { x }
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

    /// 連立方程式から並進加速度を計算
    fn compute_translational_accel(&self, tau_eff: f32, theta: f32, omega: f32) -> f32 {
        let r = self.cfg.wheel_radius;
        let m_p = self.cfg.m_p;
        let i_p = self.cfg.i_p;
        let l = self.cfg.l;

        let cos_t = cosf(theta);
        let sin_t = sinf(theta);

        let num = tau_eff * (i_p / r + m_p * l * cos_t)
                - m_p * m_p * Self::G * l * l * sin_t * cos_t
                + i_p * m_p * l * omega * omega * sin_t;

        let den = self.m_total * i_p - m_p * m_p * l * l * cos_t * cos_t;

        if den.abs() < 1e-9 {
            return 0.0;
        }

        num / den
    }

    /// # Arguments
    /// * `command` - 制御出力（total_output）[PWM]
    /// * `ax_g` - 加速度センサX軸 [G]
    /// * `az_g` - 加速度センサZ軸 [G]
    /// * `pitch_sensor` - ピッチ角（センサ位置基準）[rad]
    /// * `pitch_cg` - ピッチ角（重心基準）[rad]
    /// * `pitch_rate` - MEKFバイアス除去済みピッチ角速度 [rad/s]
    /// * `angular_accel` - ジャイロLPF微分による角加速度 [rad/s²]
    pub fn update(&mut self, command: f32, ax_g: f32, az_g: f32, pitch_sensor: f32, pitch_cg: f32, pitch_rate: f32, angular_accel: f32) -> PosEkfState {
        let dt = self.cfg.dt;

        // ───── 0. コマンドLPF ─────
        let alpha_cmd = Self::clamp(dt / (self.cfg.command_lpf_tau + dt + 1e-6), 0.0, 1.0);
        self.command_lp += alpha_cmd * (command - self.command_lp);

        // ───── 1. 観測（完全なIMU物理モデル） ─────
        let cos_s = cosf(pitch_sensor);
        let sin_s = sinf(pitch_sensor);

        let a_sensor = (ax_g * cos_s + az_g * sin_s) * 9.81;
        let a_tangential = angular_accel
            * (self.cfg.imu_rx * sin_s - self.cfg.imu_rz * cos_s);
        let a_centripetal = pitch_rate * pitch_rate
            * (self.cfg.imu_rx * cos_s + self.cfg.imu_rz * sin_s);

        let v_dot = a_sensor + a_tangential + a_centripetal;
        let a_meas = -v_dot;  // 既存符号規約維持

        self.a_raw = a_sensor;
        self.tangential_sensor = a_tangential;
        self.a_centripetal = a_centripetal;
        self.a_meas = a_meas;

        // ───── 2. 予測の目標値（DCモーターモデル + 連立方程式） ─────
        let v_applied = self.cfg.v_batt * (self.command_lp / self.cfg.pwm_max);
        let omega_wheel = self.v / self.cfg.wheel_radius;
        let tau_eff = self.cfg.motor_efficiency * self.cfg.k_tau * v_applied
                    - self.cfg.k_b * omega_wheel;
        self.tau_eff = tau_eff;
        let a_target = self.compute_translational_accel(tau_eff, pitch_cg, pitch_rate);
        self.a_target = a_target;

        // ───── 3. adaptive R（jerk/accel/velベース） ─────
        let j_meas = (a_meas - self.prev_a_meas) / dt;
        let jerk_ratio = j_meas.abs() / (self.cfg.j_max + 1e-6);
        let a_ratio = self.a.abs() / (self.cfg.a_max + 1e-6);
        let v_ratio = self.v.abs() / (self.cfg.v_max + 1e-6);

        let r_scale = 1.0
            + self.cfg.constraint_r_scale * (jerk_ratio * jerk_ratio + a_ratio * a_ratio + v_ratio * v_ratio);
        self.r_eff = self.cfg.r_accel * r_scale;

        // ───── 4. 状態予測（一次ホールド積分、jerk/accel/vel制限あり） ─────
        let mut da = a_target - self.a;
        let da_max = self.cfg.j_max * dt;
        let jerk_limited = da.abs() > da_max;
        da = Self::clamp(da, -da_max, da_max);

        // a_k (前回の加速度) と a_k+1 (今回の加速度) を保持して積分に使う
        let a_prev = self.a;
        
        self.a += da;
        self.a = Self::clamp(self.a, -self.cfg.a_max, self.cfg.a_max);
        
        let a_next = self.a;

        // 位置の更新 (一次ホールド: a_prev と a_next の両方を考慮)
        self.x += self.v * dt + (a_prev / 3.0 + a_next / 6.0) * dt * dt;

        // 速度の更新 (一次ホールド: 台形積分)
        self.v += (a_prev + a_next) / 2.0 * dt;
        self.v = Self::clamp(self.v, -self.cfg.v_max, self.cfg.v_max);
        
        // b: ランダムウォーク、予測では変化しない

        // ───── 5. 共分散予測（一次ホールドのヤコビアン行列） ─────
        // 一次ホールドの数式に合わせた状態遷移の偏微分 (∂f / ∂x)
        // phi は ∂a_{k+1} / ∂a_k (制限にかかっていれば1、到達していれば0)
        let phi = if jerk_limited { 1.0 } else { 0.0 };

        let mut f = [[0.0f32; 4]; 4];
        f[0][0] = 1.0;
        // ∂v_{k+1} / ∂a_k = 0.5 * (1 + ∂a_{k+1}/∂a_k) * dt
        f[0][2] = 0.5 * (1.0 + phi) * dt;
        
        // ∂x_{k+1} / ∂v_k = dt
        f[1][0] = dt;
        f[1][1] = 1.0;
        // ∂x_{k+1} / ∂a_k = (1/3 + 1/6 * ∂a_{k+1}/∂a_k) * dt^2
        f[1][2] = (1.0 / 3.0 + (1.0 / 6.0) * phi) * dt * dt;
        
        f[2][2] = phi;
        f[3][3] = 1.0;

        let mut q = [[0.0f32; 4]; 4];
        q[0][0] = self.cfg.q_v * dt;
        q[1][1] = self.cfg.q_x * dt;
        q[2][2] = self.cfg.q_a * dt;
        q[3][3] = self.cfg.q_bias * dt;

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

        // ───── 6. 観測更新: H = [0, 0, 1, 1] ─────
        // innovation = a_meas - (a + b)
        self.innovation = a_meas - (self.a + self.b);

        // S = H·P·H^T + R = P[2][2] + P[2][3] + P[3][2] + P[3][3] + R
        let s = self.p[2][2] + self.p[2][3] + self.p[3][2] + self.p[3][3] + self.r_eff;
        let s_inv = 1.0 / (s + 1e-9);

        // K = P·H^T / S → K[i] = (P[i][2] + P[i][3]) / S
        let mut k = [0.0f32; 4];
        for i in 0..4 {
            k[i] = (self.p[i][2] + self.p[i][3]) * s_inv;
        }

        // 状態更新
        self.v += k[0] * self.innovation;
        self.x += k[1] * self.innovation;
        self.a += k[2] * self.innovation;
        self.b += k[3] * self.innovation;

        self.k_gain_a = k[2];
        self.k_gain_bias = k[3];

        // Joseph形式: P = (I - K·H)·P·(I - K·H)^T + K·R·K^T
        // A = I - K·H, where H = [0, 0, 1, 1]
        // A[i][j] = δ_ij - K[i]·H[j]
        //   → A[i][2] -= K[i], A[i][3] -= K[i], 他はδ_ij
        let mut a_mat = [[0.0f32; 4]; 4];
        for i in 0..4 {
            a_mat[i][i] = 1.0;
            a_mat[i][2] -= k[i];
            a_mat[i][3] -= k[i];
        }

        let p_before = self.p;
        let mut ap = [[0.0f32; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                let mut ss = 0.0;
                for kk in 0..4 { ss += a_mat[i][kk] * p_before[kk][j]; }
                ap[i][j] = ss;
            }
        }
        let mut p_new = [[0.0f32; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                let mut ss = 0.0;
                for kk in 0..4 { ss += ap[i][kk] * a_mat[j][kk]; }
                p_new[i][j] = ss;
            }
        }
        for i in 0..4 {
            for j in 0..4 { p_new[i][j] += k[i] * self.r_eff * k[j]; }
        }

        Self::symmetrize(&mut p_new);
        self.p = p_new;

        self.prev_a_meas = a_meas;
        self.prev_a_target = a_target;

        // ───── 7. 状態を返す ─────
        PosEkfState {
            velocity: self.v, position: self.x, accel: self.a,
            bias: self.b,
            innovation: self.innovation,
            r_eff: self.r_eff,
            a_meas: self.a_meas, a_raw: self.a_raw, a_target: self.a_target,
            command_lp: self.command_lp,
            tangential_sensor: self.tangential_sensor,
            a_centripetal: self.a_centripetal,
            tau_eff: self.tau_eff,
            k_gain_a: self.k_gain_a, k_gain_bias: self.k_gain_bias,
        }
    }

    pub fn reset(&mut self) {
        self.v = 0.0; self.x = 0.0; self.a = 0.0; self.b = 0.0;
        self.p = [[0.0; 4]; 4];
        self.p[0][0] = 0.01; self.p[1][1] = 0.01; self.p[2][2] = 0.5; self.p[3][3] = 0.1;
        self.innovation = 0.0;
        self.prev_a_meas = 0.0; self.prev_a_target = 0.0; self.command_lp = 0.0;
        self.r_eff = self.cfg.r_accel; self.a_meas = 0.0; self.a_raw = 0.0; self.a_target = 0.0;
        self.tangential_sensor = 0.0;
        self.a_centripetal = 0.0;
        self.tau_eff = 0.0;
        self.k_gain_a = 0.0; self.k_gain_bias = 0.0;
    }
}