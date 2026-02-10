use nalgebra::{Matrix3, Vector3};

/// 地磁気キャリブレーションパラメータ
///
/// 補正モデル: m_cal = W · (m_raw - b)
/// 補正後のノルムは1に正規化される（楕円体が単位球に変換される）
///
/// # Example
///
/// ```ignore
/// // オフラインキャリブレーションで得たパラメータ
/// let calib = MagCalibration::new(
///     [10.5, -5.2, 3.1],           // offset
///     [[1.0, 0.0, 0.0],            // transform
///      [0.01, 0.98, 0.0],
///      [0.02, -0.01, 1.02]],
/// );
///
/// // 補正適用
/// let mag_cal = calib.apply(mag_raw);
/// ```
#[derive(Debug, Clone)]
pub struct MagCalibration {
    /// ハードアイアンオフセット b [µT]
    offset: Vector3<f32>,
    /// ソフトアイアン補正行列 W
    /// W = Lᵀ where A = LLᵀ (コレスキー分解)
    transform: Matrix3<f32>,
}

impl MagCalibration {
    /// 単位行列（補正なし）で初期化
    pub fn identity() -> Self {
        Self {
            offset: Vector3::zeros(),
            transform: Matrix3::identity(),
        }
    }

    /// オフセットと変換行列から生成
    ///
    /// # Arguments
    /// * `offset` - ハードアイアンオフセット [µT]
    /// * `transform` - ソフトアイアン変換行列 W（行優先）
    pub fn new(offset: [f32; 3], transform: [[f32; 3]; 3]) -> Self {
        Self {
            offset: Vector3::new(offset[0], offset[1], offset[2]),
            transform: Matrix3::new(
                transform[0][0], transform[0][1], transform[0][2],
                transform[1][0], transform[1][1], transform[1][2],
                transform[2][0], transform[2][1], transform[2][2],
            ),
        }
    }

    /// 楕円体パラメータ (b, A) から生成
    ///
    /// 楕円体制約: (m - b)ᵀ A (m - b) = 1
    /// コレスキー分解 A = LLᵀ を行い、W = Lᵀ を変換行列とする
    ///
    /// # Returns
    /// `None` if A is not positive definite
    pub fn from_ellipsoid(
        offset: Vector3<f32>,
        ellipsoid_matrix: Matrix3<f32>,
    ) -> Option<Self> {
        // コレスキー分解: A = LLᵀ
        let chol = ellipsoid_matrix.cholesky()?;
        let l = chol.l();

        Some(Self {
            offset,
            transform: l.transpose(), // W = Lᵀ
        })
    }

    /// オフセットのみ設定（ソフトアイアン補正なし）
    pub fn with_offset(offset: [f32; 3]) -> Self {
        Self {
            offset: Vector3::new(offset[0], offset[1], offset[2]),
            transform: Matrix3::identity(),
        }
    }

    /// キャリブレーションを適用
    ///
    /// m_cal = W · (m_raw - b)
    pub fn apply(&self, mag: [f32; 3]) -> [f32; 3] {
        let m_raw = Vector3::new(mag[0], mag[1], mag[2]);
        let m_cal = self.transform * (m_raw - self.offset);
        [m_cal.x, m_cal.y, m_cal.z]
    }

    /// オフセットを取得
    pub fn offset(&self) -> [f32; 3] {
        [self.offset.x, self.offset.y, self.offset.z]
    }

    /// オフセットを更新（オンラインRLS用）
    pub fn set_offset(&mut self, offset: [f32; 3]) {
        self.offset = Vector3::new(offset[0], offset[1], offset[2]);
    }

    /// 変換行列を取得（行優先）
    pub fn transform(&self) -> [[f32; 3]; 3] {
        [
            [self.transform[(0, 0)], self.transform[(0, 1)], self.transform[(0, 2)]],
            [self.transform[(1, 0)], self.transform[(1, 1)], self.transform[(1, 2)]],
            [self.transform[(2, 0)], self.transform[(2, 1)], self.transform[(2, 2)]],
        ]
    }

    /// Q = WᵀW を計算（オンラインRLSで使用）
    ///
    /// 補正後ノルム: |W(m-b)|² = (m-b)ᵀ WᵀW (m-b) = (m-b)ᵀ Q (m-b)
    pub fn q_matrix(&self) -> Matrix3<f32> {
        self.transform.transpose() * self.transform
    }

    /// 内部のnalgebra Vector3 を取得
    pub fn offset_vector(&self) -> &Vector3<f32> {
        &self.offset
    }

    /// 内部のnalgebra Matrix3 を取得
    pub fn transform_matrix(&self) -> &Matrix3<f32> {
        &self.transform
    }

    /// offset の可変参照を取得（オンラインRLS用）
    pub fn offset_vector_mut(&mut self) -> &mut Vector3<f32> {
        &mut self.offset
    }
}

impl Default for MagCalibration {
    fn default() -> Self {
        Self::identity()
    }
}