//! DKF: Dynamics-based Kalman Filter for Disturbance Estimation
//!
//! 2次元カルマンフィルタによる外乱トルク推定
//! 状態: [ω_pitch, d_pitch]
//!
//! 動力学モデル:
//!   I·ω̇ = τ_motor + mgl·sin(θ) + d
//!
//! 既存のMEKF（姿勢推定）と分離して使用する設計

#![no_std]

use libm::sinf;

/// DKFの設定
#[derive(Debug, Clone, Copy)]
pub struct DkfConfig {
    /// サンプリング周期 [s]
    pub dt: f32,
    /// 慣性モーメント [kg·m²]
    pub inertia: f32,
    /// m*g*l [Nm]
    pub mgl: f32,
    /// 角速度プロセスノイズ [rad²/s²]
    pub q_omega: f32,
    /// 外乱プロセスノイズ [Nm²]（小さめ＝低周波追従）
    pub q_disturbance: f32,
    /// ジャイロ観測ノイズ [rad²/s²]
    pub r_gyro: f32,
    /// 初期角速度分散
    pub initial_omega_variance: f32,
    /// 初期外乱分散
    pub initial_disturbance_variance: f32,
    /// 外乱の上限 [Nm]（発散防止）
    pub disturbance_limit: f32,
}

impl Default for DkfConfig {
    fn default() -> Self {
        Self {
            dt: 1.0 / 500.0,
            inertia: 0.002,
            mgl: 0.1 * 9.81 * 0.035, // ≈ 0.034 Nm
            q_omega: 0.01,
            q_disturbance: 0.0001, // 低周波追従
            r_gyro: 0.001,
            initial_omega_variance: 0.1,
            initial_disturbance_variance: 0.01,
            disturbance_limit: 0.5, // ±0.5 Nm
        }
    }
}

/// DKFの出力
#[derive(Debug, Clone, Copy, Default)]
pub struct DkfState {
    /// 推定角速度 [rad/s]
    pub omega: f32,
    /// 推定外乱トルク [Nm]
    pub disturbance: f32,
    /// 角速度の分散
    pub omega_variance: f32,
    /// 外乱の分散
    pub disturbance_variance: f32,
    /// イノベーション（デバッグ用）
    pub innovation: f32,
}

/// 2次元カルマンフィルタによる外乱オブザーバ
///
/// # 使用例
/// ```ignore
/// let mut dkf = Dkf::new(DkfConfig::default());
///
/// // 制御ループ内
/// let state = dkf.update(pitch, pitch_rate);
/// let disturbance = state.disturbance;
///
/// // 制御計算後
/// dkf.set_control_torque(tau);
/// ```
pub struct Dkf {
    // 状態
    omega: f32, // 推定角速度 [rad/s]
    d: f32,     // 推定外乱トルク [Nm]

    // 共分散行列 P (2x2, 対称なので3要素)
    p00: f32, // P[0,0]: ω分散
    p01: f32, // P[0,1] = P[1,0]: 共分散
    p11: f32, // P[1,1]: d分散

    // 前回の制御トルク [Nm]
    tau_prev: f32,

    // 設定
    config: DkfConfig,

    // 事前計算値
    dt_over_i: f32, // dt / I
}

impl Dkf {
    /// 新しいDKFを作成
    pub fn new(config: DkfConfig) -> Self {
        let dt_over_i = config.dt / config.inertia;

        Self {
            omega: 0.0,
            d: 0.0,
            p00: config.initial_omega_variance,
            p01: 0.0,
            p11: config.initial_disturbance_variance,
            tau_prev: 0.0,
            config,
            dt_over_i,
        }
    }

    /// 予測・更新ステップ
    ///
    /// # Arguments
    /// * `pitch` - 現在のピッチ角 [rad]（真の鉛直からの角度、オフセット込み）
    /// * `gyro_pitch` - バイアス補正済みピッチ角速度 [rad/s]（MEKFから）
    ///
    /// # Returns
    /// 外乱推定状態
    pub fn update(&mut self, pitch: f32, gyro_pitch: f32) -> DkfState {
        // ========== PREDICT ==========

        // 重力トルク（倒立状態では不安定化方向）
        let tau_gravity = self.config.mgl * sinf(pitch);

        // 状態予測
        // ω(k|k-1) = ω(k-1) + (dt/I) * [τ_prev + τ_gravity + d]
        let omega_pred =
            self.omega + self.dt_over_i * (-self.tau_prev + tau_gravity + self.d);
        let d_pred = self.d; // 外乱は定数モデル（ランダムウォーク）

        // 共分散予測
        // F = [1, dt/I]
        //     [0,   1 ]
        // P_pred = F * P * F' + Q
        let f01 = self.dt_over_i;

        let p00_pred =
            self.p00 + 2.0 * f01 * self.p01 + f01 * f01 * self.p11 + self.config.q_omega;
        let p01_pred = self.p01 + f01 * self.p11;
        let p11_pred = self.p11 + self.config.q_disturbance;

        // ========== UPDATE ==========

        // 観測: z = gyro_pitch
        // H = [1, 0]
        // イノベーション（観測残差）
        let y = gyro_pitch - omega_pred;

        // イノベーション共分散 S = H * P * H' + R = P[0,0] + R
        let s = p00_pred + self.config.r_gyro;

        if s.abs() < 1e-10 {
            // 数値安定性のため、更新をスキップ
            self.omega = omega_pred;
            self.d = d_pred;
            self.p00 = p00_pred;
            self.p01 = p01_pred;
            self.p11 = p11_pred;

            return DkfState {
                omega: self.omega,
                disturbance: self.d,
                omega_variance: self.p00,
                disturbance_variance: self.p11,
                innovation: y,
            };
        }

        // カルマンゲイン K = P * H' / S
        // H' = [1, 0]' なので K = [P[0,0]/S, P[0,1]/S]'
        let k0 = p00_pred / s;
        let k1 = p01_pred / s;

        // 状態更新
        self.omega = omega_pred + k0 * y;
        self.d = d_pred + k1 * y;

        // 共分散更新 (Joseph形式の簡略版)
        // P = (I - K*H) * P_pred
        // I - K*H = [1-k0, 0]
        //           [-k1,  1]
        self.p00 = (1.0 - k0) * p00_pred;
        self.p01 = (1.0 - k0) * p01_pred;
        self.p11 = p11_pred - k1 * p01_pred;

        // 共分散の正定値性を保証
        self.p00 = self.p00.max(1e-10);
        self.p11 = self.p11.max(1e-10);

        // 外乱の上下限（発散防止）
        self.d = self.d.clamp(-self.config.disturbance_limit, self.config.disturbance_limit);

        DkfState {
            omega: self.omega,
            disturbance: self.d,
            omega_variance: self.p00,
            disturbance_variance: self.p11,
            innovation: y,
        }
    }

    /// 制御トルクを設定（次回予測で使用）
    ///
    /// # Arguments
    /// * `tau` - 制御トルク [Nm]
    #[inline]
    pub fn set_control_torque(&mut self, tau: f32) {
        self.tau_prev = tau;
    }

    /// 推定外乱を取得 [Nm]
    #[inline]
    pub fn get_disturbance(&self) -> f32 {
        self.d
    }

    /// 推定角速度を取得 [rad/s]
    #[inline]
    pub fn get_omega(&self) -> f32 {
        self.omega
    }

    /// リセット
    pub fn reset(&mut self) {
        self.omega = 0.0;
        self.d = 0.0;
        self.p00 = self.config.initial_omega_variance;
        self.p01 = 0.0;
        self.p11 = self.config.initial_disturbance_variance;
        self.tau_prev = 0.0;
    }

    /// 設定を取得
    #[inline]
    pub fn config(&self) -> &DkfConfig {
        &self.config
    }

    /// 共分散行列の要素を取得（デバッグ用）
    pub fn get_covariance(&self) -> (f32, f32, f32) {
        (self.p00, self.p01, self.p11)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dkf_basic() {
        let mut dkf = Dkf::new(DkfConfig::default());

        // 初期状態
        assert_eq!(dkf.get_disturbance(), 0.0);
        assert_eq!(dkf.get_omega(), 0.0);

        // 更新
        let state = dkf.update(0.0, 0.0);
        assert!(state.disturbance.abs() < 0.1);
    }

    #[test]
    fn test_dkf_disturbance_estimation() {
        let config = DkfConfig {
            dt: 0.002,
            inertia: 0.002,
            mgl: 0.034,
            q_omega: 0.01,
            q_disturbance: 0.001, // 少し大きめで速く収束
            r_gyro: 0.001,
            ..Default::default()
        };
        let mut dkf = Dkf::new(config);

        // 一定の外乱がある状況をシミュレート
        // 真の外乱 = 0.01 Nm
        let true_disturbance = 0.01;
        let mut true_omega = 0.0;

        for _ in 0..1000 {
            // 真の動力学（外乱あり）
            let tau = 0.0; // 制御入力なし
            let tau_gravity = 0.034 * sinf(0.0); // pitch = 0
            true_omega += (config.dt / config.inertia)
                * (tau + tau_gravity + true_disturbance);

            // DKF更新
            dkf.set_control_torque(tau);
            let state = dkf.update(0.0, true_omega);

            // 収束確認（最後の方）
            if state.disturbance_variance < 0.001 {
                assert!((state.disturbance - true_disturbance).abs() < 0.005);
            }
        }
    }
}