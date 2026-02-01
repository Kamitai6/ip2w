//! オンライン地磁気バイアス追跡（RLS）
//!
//! オフラインキャリブレーション（MagOfflineEts）で得たパラメータを初期値として、
//! 温度ドリフト等によるハードアイアンオフセットの変動をオンラインで追跡する。
//!
//! ソフトアイアン行列（W）は固定し、オフセット（b）のみを更新する。

use nalgebra::{Matrix3, Vector3};
use libm::sqrtf;
use crate::util::mag_calibration::MagCalibration;

/// オンラインRLSの設定
#[derive(Debug, Clone, Copy)]
pub struct RlsParams {
    /// 忘却係数 λ (0 < λ ≤ 1)
    /// 1に近いほど過去のデータを重視（安定、追従遅い）
    /// 小さいほど最近のデータを重視（追従速い、ノイズに敏感）
    pub lambda: f32,

    /// 初期共分散のスケール
    /// P₀ = p_init * I
    pub p_init: f32,

    /// 更新をスキップするノルム誤差の閾値
    /// |補正後ノルム - 1| > threshold なら外乱と判断
    pub norm_threshold: f32,

    /// 最小更新間隔（サンプル数）
    /// 高レートセンサの場合、毎サンプル更新は不要
    pub update_interval: u32,
}

impl Default for RlsParams {
    fn default() -> Self {
        Self {
            lambda: 0.998,
            p_init: 1.0,
            norm_threshold: 0.3,
            update_interval: 1,
        }
    }
}

/// オンライン地磁気バイアス追跡
///
/// # Usage
///
/// ```ignore
/// // オフラインキャリブレーションの結果
/// let offline_calib = offline_ets.calibrate()?;
///
/// // オンラインRLSを初期化
/// let mut online_rls = MagOnlineRls::new(&offline_calib);
///
/// // メインループ
/// loop {
///     let mag_raw = sensor.read();
///     
///     // RLS更新（必要に応じて）
///     online_rls.update(mag_raw);
///     
///     // 補正済み磁場を取得
///     let mag_cal = online_rls.apply(mag_raw);
///     
///     // EKFに渡す
///     ekf.update_mag(mag_cal);
/// }
/// ```
pub struct MagOnlineRls {
    /// 現在のオフセット推定値 b
    offset: Vector3<f32>,

    /// ソフトアイアン変換行列 W（固定）
    transform: Matrix3<f32>,

    /// Q = WᵀW（計算用、固定）
    q_matrix: Matrix3<f32>,

    /// RLS共分散行列 P
    p_matrix: Matrix3<f32>,

    /// パラメータ
    params: RlsParams,

    /// 更新カウンタ
    sample_count: u32,
}

impl MagOnlineRls {
    /// MagCalibration から初期化
    pub fn new(calib: &MagCalibration) -> Self {
        Self::new_with_params(calib, RlsParams::default())
    }

    /// パラメータを指定して初期化
    pub fn new_with_params(calib: &MagCalibration, params: RlsParams) -> Self {
        let transform = *calib.transform_matrix();
        let q_matrix = calib.q_matrix();

        Self {
            offset: *calib.offset_vector(),
            transform,
            q_matrix,
            p_matrix: Matrix3::identity() * params.p_init,
            params,
            sample_count: 0,
        }
    }

    /// 忘却係数を変更
    pub fn set_lambda(&mut self, lambda: f32) {
        self.params.lambda = lambda.clamp(0.9, 1.0);
    }

    /// 現在のオフセット推定値を取得
    pub fn offset(&self) -> [f32; 3] {
        [self.offset.x, self.offset.y, self.offset.z]
    }

    /// 現在の推定で補正を適用
    /// m_cal = W · (m_raw - b)
    pub fn apply(&self, mag_raw: [f32; 3]) -> [f32; 3] {
        let m = Vector3::new(mag_raw[0], mag_raw[1], mag_raw[2]);
        let m_cal = self.transform * (m - self.offset);
        [m_cal.x, m_cal.y, m_cal.z]
    }

    /// RLS更新
    ///
    /// # Returns
    /// - `true`: 更新が実行された
    /// - `false`: スキップされた（外乱検知または間隔未達）
    pub fn update(&mut self, mag_raw: [f32; 3]) -> bool {
        self.sample_count += 1;

        // 更新間隔チェック
        if self.sample_count % self.params.update_interval != 0 {
            return false;
        }

        let m = Vector3::new(mag_raw[0], mag_raw[1], mag_raw[2]);

        // 現在の推定で補正
        let diff = m - self.offset;
        let m_cal = self.transform * diff;
        let norm_sq = m_cal.norm_squared();
        let norm = sqrtf(norm_sq);

        // 外乱チェック: 補正後ノルムが1から大きく外れていたらスキップ
        if (norm - 1.0).abs() > self.params.norm_threshold {
            return false;
        }

        // RLS更新
        // 誤差: e = 1 - |W(m - b)|²
        // 勾配: φ = ∂|W(m-b)|²/∂b = -2 Q (m - b)
        //       ただし Q = WᵀW
        //
        // ここでは |W(m-b)|² = 1 の制約を使う
        // e = 1 - norm_sq
        let error = 1.0 - norm_sq;

        // φ = 2 Q (m - b)
        let phi = -2.0 * self.q_matrix * diff;

        // RLS ゲイン: K = P φ / (λ + φᵀ P φ)
        let p_phi = self.p_matrix * phi;
        let denom = self.params.lambda + phi.dot(&p_phi);

        if denom.abs() < 1e-10 {
            return false; // 数値的に不安定
        }

        let k = p_phi / denom;

        // オフセット更新: b = b + K e
        self.offset += k * error;

        // 共分散更新: P = (P - K φᵀ P) / λ
        let k_phi_t = k * phi.transpose();
        self.p_matrix = (self.p_matrix - k_phi_t * self.p_matrix) / self.params.lambda;

        // 共分散行列の対称性を維持
        self.p_matrix = (self.p_matrix + self.p_matrix.transpose()) * 0.5;

        true
    }

    /// 共分散行列をリセット
    pub fn reset_covariance(&mut self) {
        self.p_matrix = Matrix3::identity() * self.params.p_init;
    }

    /// 現在のMagCalibrationを取得（更新されたオフセット付き）
    pub fn calibration(&self) -> MagCalibration {
        MagCalibration::new(
            [self.offset.x, self.offset.y, self.offset.z],
            [
                [self.transform[(0, 0)], self.transform[(0, 1)], self.transform[(0, 2)]],
                [self.transform[(1, 0)], self.transform[(1, 1)], self.transform[(1, 2)]],
                [self.transform[(2, 0)], self.transform[(2, 1)], self.transform[(2, 2)]],
            ],
        )
    }

    /// 共分散のトレース（収束の指標）
    pub fn covariance_trace(&self) -> f32 {
        self.p_matrix.trace()
    }

    /// デバッグ用：共分散行列を取得
    pub fn p_matrix(&self) -> [[f32; 3]; 3] {
        [
            [self.p_matrix[(0, 0)], self.p_matrix[(0, 1)], self.p_matrix[(0, 2)]],
            [self.p_matrix[(1, 0)], self.p_matrix[(1, 1)], self.p_matrix[(1, 2)]],
            [self.p_matrix[(2, 0)], self.p_matrix[(2, 1)], self.p_matrix[(2, 2)]],
        ]
    }
}
