use libm::{sinf, tanhf};

/// 並進位置推定EKF + レギュレータ（v1: 観測側バイアス）
///
/// コマンド積分（高帯域）と加速度計観測（低帯域）を融合して
/// 並進速度・位置・重力バイアスを推定する。
///
/// 【v1での変更】
/// バイアスを「観測側の重力リーク」として推定し、
/// 予測側（コマンド積分）には影響させない。
///
/// 状態: [v, x, g_bias]
///   v:      並進速度 [m/s]
///   x:      並進位置 [m]
///   g_bias: 観測側重力バイアス [m/s²]
///           （姿勢推定誤差・加速度計バイアスによる重力リーク）
///
/// プロセスモデル:
///   v += k_cmd · command · dt     ← バイアスは含まない
///   x += v · dt
///   g_bias ~ random walk
///
/// 観測モデル (H = [0, 0, +1]):
///   z = -(ax_g + sin(pitch)) · 9.81  [m/s²]（加速度計観測）
///   h = k_cmd · command + g_bias      [m/s²]（予測 + バイアス）
///   innovation = z - h = z - k_cmd · command - g_bias

pub struct PosEkfConfig {
    pub dt: f32,

    /// コマンド → 並進加速度の変換係数 [m/s² / PWM]
    pub k_cmd: f32,

    /// 速度プロセスノイズ [m²/s³]
    pub q_v: f32,

    /// バイアスランダムウォーク [m²/s⁵]
    pub q_bias: f32,

    /// 加速度計観測ノイズ [m²/s⁴]
    pub r_accel: f32,

    /// 位置→角度オフセットゲイン [rad/m]
    pub k_pos: f32,

    /// 速度→角度オフセットゲイン [rad/(m/s)]
    pub k_vel: f32,

    /// 最大角度オフセット [rad]
    pub max_output: f32,
}

impl Default for PosEkfConfig {
    fn default() -> Self {
        Self {
            dt: 0.002,
            k_cmd: 0.003,
            q_v: 1e-3,
            q_bias: 1e-6,
            r_accel: 1.0,
            k_pos: 0.1,
            k_vel: 0.2,
            max_output: 0.05,
        }
    }
}

/// デバッグ用
#[derive(Clone, Copy)]
pub struct PosEkfState {
    pub velocity: f32,
    pub position: f32,
    pub g_bias: f32,
    pub innovation: f32,
}

pub struct PositionEkf {
    v: f32,
    x: f32,
    g_bias: f32,
    /// 共分散（3×3対称行列、上三角6要素）
    /// [P_vv, P_vx, P_vb, P_xx, P_xb, P_bb]
    p: [f32; 6],
    innovation: f32,
    cfg: PosEkfConfig,
}

impl PositionEkf {
    pub fn new(cfg: PosEkfConfig) -> Self {
        Self {
            v: 0.0,
            x: 0.0,
            g_bias: 0.0,
            p: [
                0.01, // P_vv
                0.0,  // P_vx
                0.0,  // P_vb
                0.01, // P_xx
                0.0,  // P_xb
                0.1,  // P_bb（重力リークは小さいので初期不確かさも小さめ）
            ],
            innovation: 0.0,
            cfg,
        }
    }

    /// 予測 + 観測更新 + レギュレータ出力
    ///
    /// # Arguments
    /// * `command`  - total_output [PWM]（デッドゾーン前）
    /// * `ax_g`     - LPF後の加速度計X [g]（Z-up計算座標系）
    /// * `pitch`    - MEKFピッチ角 [rad]
    ///
    /// # Returns
    /// 角度オフセット [rad]
    pub fn update(&mut self, command: f32, ax_g: f32, pitch: f32) -> f32 {
        let dt = self.cfg.dt;

        // ══════ 予測 ══════
        // 【変更】バイアスは予測に含めない
        let a_cmd = self.cfg.k_cmd * command;
        self.v += a_cmd * dt;
        self.x += self.v * dt;

        // 共分散予測: P = F·P·Fᵀ + Q
        //
        // 【変更】F行列が変わった
        // F = [[1,  0,  0],    ← バイアスは速度に影響しない
        //      [dt, 1,  0],
        //      [0,  0,  1]]
        let [pv, pvx, pvb, px, pxb, pb] = self.p;
        self.p = [
            /* P_vv */ pv + self.cfg.q_v,
            /* P_vx */ dt * pv + pvx,
            /* P_vb */ pvb,  // 【変更】バイアスと速度の相関は伝播しない
            /* P_xx */ dt * dt * pv + 2.0 * dt * pvx + px,
            /* P_xb */ dt * pvb + pxb,
            /* P_bb */ pb + self.cfg.q_bias,
        ];

        // ══════ 観測更新 ══════
        // 加速度計から水平並進加速度を取り出す
        let a_meas = -(ax_g + sinf(pitch)) * 9.81;

        // 【変更】観測モデル: h = a_cmd + g_bias
        // innovation = z - h = a_meas - a_cmd - g_bias
        let a_pred = a_cmd + self.g_bias;
        self.innovation = a_meas - a_pred;

        // 【変更】H = [0, 0, +1]（符号が変わった）
        // S = H·P·Hᵀ + R = P_bb + R
        let s = self.p[5] + self.cfg.r_accel;
        let s_inv = 1.0 / s;

        // 【変更】カルマンゲイン K = P·Hᵀ / S = [P_vb, P_xb, P_bb]ᵀ / S
        // H = [0, 0, +1] なので符号が反転
        let kv = self.p[2] * s_inv;
        let kx = self.p[4] * s_inv;
        let kb = self.p[5] * s_inv;

        // 状態補正
        self.v += kv * self.innovation;
        self.x += kx * self.innovation;
        self.g_bias += kb * self.innovation;

        // 共分散更新: P = (I − K·H) · P
        // H = [0, 0, +1] なので:
        // P_vv  -= K_v · P_vb = P_vb² / S
        // P_vx  -= K_v · P_xb = P_vb · P_xb / S
        // P_xx  -= K_x · P_xb = P_xb² / S
        // P_vb  -= K_v · P_bb → P_vb · (1 - P_bb/S) = P_vb · R/S
        // P_xb  -= K_x · P_bb → P_xb · R/S
        // P_bb  -= K_b · P_bb → P_bb · R/S
        let rs = self.cfg.r_accel * s_inv;
        self.p[0] -= self.p[2] * self.p[2] * s_inv;
        self.p[1] -= self.p[2] * self.p[4] * s_inv;
        self.p[3] -= self.p[4] * self.p[4] * s_inv;
        self.p[2] *= rs;
        self.p[4] *= rs;
        self.p[5] *= rs;

        // ══════ レギュレータ出力 ══════
        let raw = self.cfg.k_pos * self.x + self.cfg.k_vel * self.v;
        self.cfg.max_output * tanhf(raw / self.cfg.max_output)
    }

    pub fn state(&self) -> PosEkfState {
        PosEkfState {
            velocity: self.v,
            position: self.x,
            g_bias: self.g_bias,
            innovation: self.innovation,
        }
    }

    pub fn reset(&mut self) {
        self.v = 0.0;
        self.x = 0.0;
        self.g_bias = 0.0;
        self.p = [0.01, 0.0, 0.0, 0.01, 0.0, 0.1];
        self.innovation = 0.0;
    }
}