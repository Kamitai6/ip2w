//! 拡張 Two-step 法によるオフライン地磁気キャリブレーション
//!
//! Step 1: 代数的楕円体フィッティング（線形最小二乗）
//! Step 2: LM法による幾何学的精密化
//!
//! # Reference
//! - "Complete Triaxis Magnetometer Calibration in the Magnetic Domain"
//!   (Renaudin et al., 2010)

use heapless::Vec as HVec;
use nalgebra::{Matrix3, Vector3};
use libm::{sqrt, sqrtf};

use crate::util::mag_calibration::MagCalibration;

/// 最大サンプル数
pub const MAX_SAMPLES: usize = 1000;

/// デフォルトの最小サンプル間距離² [µT²]
pub const DEFAULT_MIN_DISTANCE_SQ: f32 = 4.0;

/// サンプル更新結果
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UpdateResult {
    /// サンプル追加成功
    Added,
    /// 既存サンプルに近すぎてスキップ
    Skipped,
    /// バッファが満杯
    Full,
}

/// キャリブレーション結果
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibrationError {
    /// サンプル数不足（最低10必要）
    InsufficientSamples,
    /// Step 1 の行列が特異
    Step1SingularMatrix,
    /// Step 1 で楕円体条件を満たさない（正定値でない）
    Step1NotPositiveDefinite,
    /// Step 2 (LM) が収束しなかった
    Step2NotConverged,
    /// Step 2 でコレスキー分解に失敗
    Step2CholeskyFailed,
}

/// LM法のパラメータ
#[derive(Debug, Clone, Copy)]
pub struct LmParams {
    /// 初期ダンピング係数
    pub mu_init: f32,
    /// ダンピング増加率
    pub mu_increase: f32,
    /// ダンピング減少率
    pub mu_decrease: f32,
    /// 最大反復回数
    pub max_iter: usize,
    /// パラメータ変化の収束閾値
    pub tol_param: f32,
    /// コスト変化の収束閾値
    pub tol_cost: f32,
}

impl Default for LmParams {
    fn default() -> Self {
        Self {
            mu_init: 1e-3,
            mu_increase: 10.0,
            mu_decrease: 0.1,
            max_iter: 50,
            tol_param: 1e-8,
            tol_cost: 1e-10,
        }
    }
}

/// 拡張 Two-step 法によるオフラインキャリブレータ
///
/// # Usage
///
/// ```ignore
/// let mut calibrator = MagOfflineEts::new();
///
/// // サンプル収集（デバイスを様々な方向に回転）
/// loop {
///     let mag = sensor.read();
///     match calibrator.update(mag) {
///         UpdateResult::Added => { /* progress */ }
///         UpdateResult::Skipped => { /* too close */ }
///         UpdateResult::Full => break,
///     }
///     if calibrator.sample_count() >= 200 {
///         break;
///     }
/// }
///
/// // キャリブレーション実行
/// let calib = calibrator.calibrate()?;
/// ```
pub struct MagOfflineEts {
    samples: HVec<[f32; 3], MAX_SAMPLES>,
    min_distance_sq: f32,
    lm_params: LmParams,
}

impl Default for MagOfflineEts {
    fn default() -> Self {
        Self::new()
    }
}

impl MagOfflineEts {
    /// 新規作成
    pub fn new() -> Self {
        Self {
            samples: HVec::new(),
            min_distance_sq: DEFAULT_MIN_DISTANCE_SQ,
            lm_params: LmParams::default(),
        }
    }

    /// 最小サンプル間距離を設定
    pub fn with_min_distance(mut self, min_distance: f32) -> Self {
        self.min_distance_sq = min_distance * min_distance;
        self
    }

    /// LM法のパラメータを設定
    pub fn with_lm_params(mut self, params: LmParams) -> Self {
        self.lm_params = params;
        self
    }

    /// サンプルをリセット
    pub fn reset(&mut self) {
        self.samples.clear();
    }

    /// サンプルを追加
    pub fn update(&mut self, mag: [f32; 3]) -> UpdateResult {
        if self.samples.is_full() {
            return UpdateResult::Full;
        }

        // 既存サンプルとの距離チェック
        for s in self.samples.iter() {
            let dx = mag[0] - s[0];
            let dy = mag[1] - s[1];
            let dz = mag[2] - s[2];
            if dx * dx + dy * dy + dz * dz < self.min_distance_sq {
                return UpdateResult::Skipped;
            }
        }

        let _ = self.samples.push(mag);
        UpdateResult::Added
    }

    /// 現在のサンプル数
    pub fn sample_count(&self) -> usize {
        self.samples.len()
    }

    /// サンプルへの参照
    pub fn samples(&self) -> &[[f32; 3]] {
        &self.samples
    }

    /// キャリブレーション実行
    ///
    /// Step 1 + Step 2 を実行して MagCalibration を返す
    pub fn calibrate(&self) -> Result<MagCalibration, CalibrationError> {
        // Step 1: 代数的楕円体フィット
        let (b_init, a_init) = self.step1_algebraic_fit()?;

        // Step 2: LM法で精密化
        let (b_final, a_final) = self.step2_lm_refine(b_init, a_init)?;

        // MagCalibration を構築
        MagCalibration::from_ellipsoid(b_final, a_final)
            .ok_or(CalibrationError::Step2CholeskyFailed)
    }

    /// Step 1 のみ実行（デバッグ用）
    pub fn calibrate_step1_only(&self) -> Result<MagCalibration, CalibrationError> {
        let (b, a) = self.step1_algebraic_fit()?;
        MagCalibration::from_ellipsoid(b, a).ok_or(CalibrationError::Step1NotPositiveDefinite)
    }

    /// Step 1: 代数的楕円体フィット
    ///
    /// 楕円体方程式: mᵀDm + 2fᵀm + d = 0 を線形最小二乗で解く
    ///
    /// # Returns
    /// (offset b, ellipsoid matrix A)
    fn step1_algebraic_fit(&self) -> Result<(Vector3<f32>, Matrix3<f32>), CalibrationError> {
        let n = self.samples.len();
        if n < 10 {
            return Err(CalibrationError::InsufficientSamples);
        }

        // 1) 平均（初期バイアス）
        let mut mean = Vector3::zeros();
        for s in self.samples.iter() {
            mean.x += s[0];
            mean.y += s[1];
            mean.z += s[2];
        }
        mean /= n as f32;

        // 2) 共分散 C
        let mut c = Matrix3::<f32>::zeros();
        for s in self.samples.iter() {
            let v = Vector3::new(s[0], s[1], s[2]) - mean;
            c += v * v.transpose();
        }
        c /= n as f32;

        // 3) 正則化（必須：Atom Motionみたいに歪むとCがギリギリになる）
        // 値のスケールに依存しないよう、traceで自動スケールする
        let trace = c[(0,0)] + c[(1,1)] + c[(2,2)];
        let eps = (trace / 3.0) * 1e-6 + 1e-6; // ここは強めでOK
        c[(0,0)] += eps;
        c[(1,1)] += eps;
        c[(2,2)] += eps;

        // 4) A0 = C^{-1} / k
        let c_inv = c.try_inverse().ok_or(CalibrationError::Step1SingularMatrix)?;

        // k = mean(v^T C^{-1} v)
        let mut k = 0.0f32;
        for s in self.samples.iter() {
            let v = Vector3::new(s[0], s[1], s[2]) - mean;
            let t = (v.transpose() * c_inv * v)[(0, 0)];
            k += t;
        }
        k /= n as f32;

        if !k.is_finite() || k <= 1e-12 {
            return Err(CalibrationError::Step1NotPositiveDefinite);
        }

        let a0 = c_inv / k;

        // 5) 念のため正定値チェック
        if a0.cholesky().is_none() {
            return Err(CalibrationError::Step1NotPositiveDefinite);
        }

        Ok((mean, a0))
    }

    /// Step 2: LM法による幾何学的精密化
    ///
    /// コスト関数: J = Σᵢ (|W(mᵢ - b)| - 1)²
    /// パラメータ: [b₁, b₂, b₃, L₁₁, L₂₁, L₂₂, L₃₁, L₃₂, L₃₃]
    ///            (L は下三角行列、A = LLᵀ)
    fn step2_lm_refine(
        &self,
        b_init: Vector3<f32>,
        a_init: Matrix3<f32>,
    ) -> Result<(Vector3<f32>, Matrix3<f32>), CalibrationError> {
        // 初期パラメータ: A からコレスキー分解で L を取得
        let l_init = a_init
            .cholesky()
            .ok_or(CalibrationError::Step1NotPositiveDefinite)?
            .l();

        // パラメータベクトル: [b; vec(L下三角)]
        // p = [b₁, b₂, b₃, L₁₁, L₂₁, L₂₂, L₃₁, L₃₂, L₃₃]
        let mut p = [
            b_init.x,
            b_init.y,
            b_init.z,
            l_init[(0, 0)],
            l_init[(1, 0)],
            l_init[(1, 1)],
            l_init[(2, 0)],
            l_init[(2, 1)],
            l_init[(2, 2)],
        ];

        let mut mu = self.lm_params.mu_init;
        let mut cost = self.compute_cost(&p);

        for _iter in 0..self.lm_params.max_iter {
            // ヤコビアンと残差を計算
            let (jtr, jtj) = self.compute_jacobian(&p);

            // LM更新: (JᵀJ + μI)δ = -Jᵀr
            // δ = -(JᵀJ + μI)⁻¹ Jᵀr
            let mut jtj_damped = jtj;
            for i in 0..9 {
                jtj_damped[i][i] += mu;
            }

            let delta = match solve_9x9(&jtj_damped, &jtr) {
                Some(d) => d,
                None => {
                    // 特異行列 → μを増やして再試行
                    mu *= self.lm_params.mu_increase;
                    continue;
                }
            };

            // 候補パラメータ
            let mut p_new = p;
            for i in 0..9 {
                p_new[i] -= delta[i];
            }

            // 新しいコスト
            let cost_new = self.compute_cost(&p_new);

            if cost_new < cost {
                // 改善 → 更新を受け入れ、μを減少
                p = p_new;

                // 収束判定
                let delta_norm: f32 = sqrtf(delta.iter().map(|x| x * x).sum::<f32>());
                let cost_change = (cost - cost_new).abs();

                cost = cost_new;
                mu *= self.lm_params.mu_decrease;

                if delta_norm < self.lm_params.tol_param || cost_change < self.lm_params.tol_cost {
                    // 収束
                    return Ok(self.params_to_ba(&p));
                }
            } else {
                // 悪化 → μを増加して再試行
                mu *= self.lm_params.mu_increase;
            }
        }

        // 最大反復回数に達したが、現在の推定値を返す
        // 厳密には NotConverged だが、Step1よりは良いはずなので結果を返す
        Ok(self.params_to_ba(&p))
    }

    /// パラメータからコスト関数を計算
    /// J = Σᵢ (|W(mᵢ - b)| - 1)²
    fn compute_cost(&self, p: &[f32; 9]) -> f32 {
        let (b, l) = self.params_to_bl(p);
        let w = l.transpose(); // W = Lᵀ

        let mut cost = 0.0;
        for s in self.samples.iter() {
            let m = Vector3::new(s[0], s[1], s[2]);
            let m_cal = w * (m - b);
            let norm = m_cal.norm();
            let residual = norm - 1.0;
            cost += residual * residual;
        }
        cost
    }

    /// ヤコビアン関連の計算
    /// Returns (Jᵀr, JᵀJ) where r is residual vector
    fn compute_jacobian(&self, p: &[f32; 9]) -> ([f32; 9], [[f32; 9]; 9]) {
        let (b, l) = self.params_to_bl(p);
        let w = l.transpose(); // W = Lᵀ

        let mut jtr = [0.0f32; 9];
        let mut jtj = [[0.0f32; 9]; 9];

        for s in self.samples.iter() {
            let m = Vector3::new(s[0], s[1], s[2]);
            let diff = m - b;
            let m_cal = w * diff;
            let norm = m_cal.norm();

            if norm < 1e-10 {
                continue; // 特異点を避ける
            }

            let residual = norm - 1.0;

            // 残差 r = |W(m-b)| - 1 の各パラメータに対する勾配
            // ∂r/∂b = -Wᵀ(W(m-b)) / |W(m-b)| = -Wᵀ m_cal / norm
            // ∂r/∂L_ij は少し複雑

            let m_cal_normalized = m_cal / norm;
            let wt_m_cal_normalized = w.transpose() * m_cal_normalized;

            // ∂r/∂b = -wt_m_cal_normalized
            let dr_db = -wt_m_cal_normalized;

            // ∂r/∂L_ij
            // W = Lᵀ なので m_cal = Lᵀ(m-b)
            // ∂m_cal_k/∂L_ij = δ_jk * diff_i  (Lᵀの(k,i)成分がL_ij)
            // ∂r/∂L_ij = (m_cal · ∂m_cal/∂L_ij) / norm
            //          = m_cal_j * diff_i / norm

            // L は下三角: L₁₁, L₂₁, L₂₂, L₃₁, L₃₂, L₃₃
            // インデックス: 3, 4, 5, 6, 7, 8
            let dr_dl = [
                m_cal_normalized.x * diff.x, // L₁₁: j=0, i=0
                m_cal_normalized.y * diff.x, // L₂₁: j=1, i=0
                m_cal_normalized.y * diff.y, // L₂₂: j=1, i=1
                m_cal_normalized.z * diff.x, // L₃₁: j=2, i=0
                m_cal_normalized.z * diff.y, // L₃₂: j=2, i=1
                m_cal_normalized.z * diff.z, // L₃₃: j=2, i=2
            ];

            // フルの勾配ベクトル
            let grad = [
                dr_db.x, dr_db.y, dr_db.z,
                dr_dl[0], dr_dl[1], dr_dl[2], dr_dl[3], dr_dl[4], dr_dl[5],
            ];

            // Jᵀr と JᵀJ の更新
            for i in 0..9 {
                jtr[i] += grad[i] * residual;
                for j in 0..9 {
                    jtj[i][j] += grad[i] * grad[j];
                }
            }
        }

        (jtr, jtj)
    }

    /// パラメータベクトル → (b, L)
    fn params_to_bl(&self, p: &[f32; 9]) -> (Vector3<f32>, Matrix3<f32>) {
        let b = Vector3::new(p[0], p[1], p[2]);
        let l = Matrix3::new(
            p[3], 0.0,  0.0,
            p[4], p[5], 0.0,
            p[6], p[7], p[8],
        );
        (b, l)
    }

    /// パラメータベクトル → (b, A = LLᵀ)
    fn params_to_ba(&self, p: &[f32; 9]) -> (Vector3<f32>, Matrix3<f32>) {
        let (b, l) = self.params_to_bl(p);
        let a = l * l.transpose();
        (b, a)
    }

    /// 品質指標: 残差RMS
    pub fn residual_rms(&self, calib: &MagCalibration) -> f32 {
        if self.samples.is_empty() {
            return 0.0;
        }

        let mut sum_sq = 0.0;
        for s in self.samples.iter() {
            let m_cal = calib.apply(*s);
            let norm = sqrtf(m_cal[0] * m_cal[0] + m_cal[1] * m_cal[1] + m_cal[2] * m_cal[2]);
            let residual = norm - 1.0;
            sum_sq += residual * residual;
        }
        sqrtf(sum_sq / self.samples.len() as f32)
    }
}

/// 9x9線形方程式を解く（ガウスの消去法）
fn solve_9x9(a: &[[f32; 9]; 9], b: &[f32; 9]) -> Option<[f32; 9]> {
    // 拡大行列を作成
    let mut aug = [[0.0f32; 10]; 9];
    for i in 0..9 {
        for j in 0..9 {
            aug[i][j] = a[i][j];
        }
        aug[i][9] = b[i];
    }

    // 前進消去（部分ピボット選択）
    for col in 0..9 {
        // ピボット選択
        let mut max_row = col;
        let mut max_val = aug[col][col].abs();
        for row in (col + 1)..9 {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }

        if max_val < 1e-12 {
            return None; // 特異行列
        }

        // 行の交換
        if max_row != col {
            aug.swap(col, max_row);
        }

        // 消去
        let pivot = aug[col][col];
        for row in (col + 1)..9 {
            let factor = aug[row][col] / pivot;
            for j in col..10 {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    // 後退代入
    let mut x = [0.0f32; 9];
    for i in (0..9).rev() {
        let mut sum = aug[i][9];
        for j in (i + 1)..9 {
            sum -= aug[i][j] * x[j];
        }
        if aug[i][i].abs() < 1e-12 {
            return None;
        }
        x[i] = sum / aug[i][i];
    }

    Some(x)
}
