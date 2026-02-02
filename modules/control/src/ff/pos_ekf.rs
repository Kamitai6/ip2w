use libm::{sinf, tanhf};

/// 並進位置推定EKF + レギュレータ
///
/// コマンド積分（高帯域）と加速度計観測（低帯域）を融合して
/// 並進速度・位置・加速度バイアスを推定する。
///
/// バイアス状態が、モーター非線形・重力補償のDC成分・姿勢推定誤差
/// による重力リークをまとめて推定・除去する。
/// （MEKFにおけるgyro bias推定と同じ原理）
///
/// 状態: [v, x, a_bias]
///   v:      並進速度 [m/s]
///   x:      並進位置 [m]
///   a_bias: 加速度バイアス [m/s²]
///
/// プロセスモデル:
///   v += (k_cmd · command − a_bias) · dt
///   x += v · dt
///   a_bias ~ random walk
///
/// 観測 (H = [0, 0, −1]):
///   z = (ax_g + sin(pitch)) · 9.81  [m/s²]
///   h = k_cmd · command − a_bias
///
/// 位置は直接観測できないが、バイアス補正 → 速度補正 → 位置補正
/// と共分散を通じて間接的に修正される。

pub struct PosEkfConfig {
    pub dt: f32,

    /// コマンド → 並進加速度の変換係数 [m/s² / PWM]
    ///
    /// 理論値: 2 / (TORQUE_TO_PWM × wheel_radius × mass)
    /// 実効値はこれより大幅に小さい（SMCが大半のトルクを姿勢維持に消費）
    /// 0.001〜0.01 程度から試す。バイアスが定常誤差を吸収する。
    pub k_cmd: f32,

    /// 速度プロセスノイズ [m²/s³]
    /// コマンドモデルの不確実性。大きい → 加速度計を信頼
    pub q_v: f32,

    /// バイアスランダムウォーク [m²/s⁵]
    /// 大きい → バイアスが速く追従（ノイジーになる）
    /// 小さい → バイアスがゆっくり変化（ドリフト追従が遅い）
    pub q_bias: f32,

    /// 加速度計観測ノイズ [m²/s⁴]
    /// 大きい → 加速度計を信頼しない（コマンドベース寄り）
    /// 小さい → 加速度計を信頼（バイアス収束が速い）
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
    pub a_bias: f32,
    pub innovation: f32,
}

pub struct PositionEkf {
    v: f32,
    x: f32,
    a_bias: f32,
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
            a_bias: 0.0,
            p: [
                0.01, // P_vv
                0.0,  // P_vx
                0.0,  // P_vb
                0.01, // P_xx
                0.0,  // P_xb
                1.0,  // P_bb（バイアス不明なので大きく）
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
    /// main側で `target_angle = 0.0 - pos_ekf.update(...)` として使う
    ///
    /// # Note
    /// 加速度計の前後方向の符号が逆の場合、innovationの符号が
    /// 定常的に反転する。その場合は ax_g の符号を反転して渡す。
    pub fn update(&mut self, command: f32, ax_g: f32, pitch: f32) -> f32 {
        let dt = self.cfg.dt;

        // ══════ 予測 ══════
        let a_cmd = self.cfg.k_cmd * command;
        self.v += (a_cmd - self.a_bias) * dt;
        self.x += self.v * dt;

        // 共分散予測: P = F·P·Fᵀ + Q
        //
        // F = [[1,  0, -dt],
        //      [dt, 1,   0],
        //      [0,  0,   1]]
        let [pv, pvx, pvb, px, pxb, pb] = self.p;
        self.p = [
            /* P_vv */ pv - 2.0 * dt * pvb + dt * dt * pb + self.cfg.q_v,
            /* P_vx */ dt * pv - dt * dt * pvb + pvx - dt * pxb,
            /* P_vb */ pvb - dt * pb,
            /* P_xx */ dt * dt * pv + 2.0 * dt * pvx + px,
            /* P_xb */ dt * pvb + pxb,
            /* P_bb */ pb + self.cfg.q_bias,
        ];

        // ══════ 観測更新 ══════
        // 重力除去して水平並進加速度を取り出す
        // a_meas = (ax + sin(pitch)) · 9.81
        //   ax ≈ −sin(pitch) + a_forward/g のため、重力成分が相殺される
        let a_meas = (ax_g + sinf(pitch)) * 9.81;
        let a_pred = a_cmd - self.a_bias;
        self.innovation = a_meas - a_pred;

        // H = [0, 0, −1]
        // S = H·P·Hᵀ + R = P_bb + R
        let s = self.p[5] + self.cfg.r_accel;
        let s_inv = 1.0 / s;

        // カルマンゲイン K = P·Hᵀ / S = [−P_vb, −P_xb, −P_bb]ᵀ / S
        let kv = -self.p[2] * s_inv;
        let kx = -self.p[4] * s_inv;
        let kb = -self.p[5] * s_inv;

        // 状態補正
        self.v += kv * self.innovation;
        self.x += kx * self.innovation;
        self.a_bias += kb * self.innovation;

        // 共分散更新: P = (I − K·H) · P
        //
        // P_vv  -= P_vb² / S
        // P_vx  -= P_vb · P_xb / S
        // P_xx  -= P_xb² / S
        // P_vb  *= R / S
        // P_xb  *= R / S
        // P_bb  *= R / S
        //
        // ※ 減算を先に実行（predicted P_vb, P_xb を使う）
        //   乗算は後（値を上書きしてよい）
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
            a_bias: self.a_bias,
            innovation: self.innovation,
        }
    }

    pub fn reset(&mut self) {
        self.v = 0.0;
        self.x = 0.0;
        self.a_bias = 0.0;
        self.p = [0.01, 0.0, 0.0, 0.01, 0.0, 1.0];
        self.innovation = 0.0;
    }
}