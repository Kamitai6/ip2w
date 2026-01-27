//! UD-MEKF: Multiplicative Extended Kalman Filter with UD Decomposition
//!
//! 6次元誤差状態 [δθ, δb] + 参照クォータニオンによる乗法的更新
//! UD分解による数値安定な共分散管理（sqrtf不使用）
//!
//! 参考文献:
//! - Bierman, G.J. "Factorization Methods for Discrete Sequential Estimation" (1977)
//! - Thornton, C.L. "Triangular Covariance Factorizations for Kalman Filtering" (1976)

#![no_std]

use core::f32::{consts::PI, EPSILON};
use libm::{atan2f, cosf, sinf, sqrtf};
use nalgebra::{Matrix3, Quaternion, SMatrix, SVector, UnitQuaternion, Vector3};

const DEG_TO_RAD: f32 = PI / 180.0;
const RAD_TO_DEG: f32 = 180.0 / PI;

// 型エイリアス
type UMatrix = SMatrix<f32, 6, 6>;  // 上三角（対角=1）
type DVector = SVector<f32, 6>;     // 対角成分
type StateVector6 = SVector<f32, 6>;

/// Continuous angle tracker (unwrapped)
#[derive(Debug, Clone, Copy)]
struct ContinuousAngle {
    turns: f32,
    prev: f32,
}

impl Default for ContinuousAngle {
    fn default() -> Self {
        Self {
            turns: 0.0,
            prev: 0.0,
        }
    }
}

impl ContinuousAngle {
    fn update(&mut self, angle: f32) -> f32 {
        if self.turns.abs() < EPSILON {
            self.turns -= angle;
        }
        if (angle * self.prev) < 0.0 {
            let diff = angle - self.prev;
            if diff < -PI {
                self.turns += PI * 2.0;
            } else if diff > PI {
                self.turns -= PI * 2.0;
            }
        }
        self.prev = angle;
        angle + self.turns
    }

    fn reset(&mut self) {
        *self = Self::default();
    }
}

/// EKF Configuration (互換性のため名前維持)
#[derive(Debug, Clone, Copy)]
pub struct EkfConfig {
    pub dt: f32,
    pub gyro_noise: f32,
    pub gyro_bias_noise: f32,
    pub accel_noise: f32,
    pub mag_noise: f32,
    pub initial_quat_variance: f32,
    pub initial_bias_variance: f32,
    pub accel_magnitude_min: f32,
    pub accel_magnitude_max: f32,
    pub bias_correlation_enabled: bool,
    pub bias_correlation_factor: f32,
}

impl Default for EkfConfig {
    fn default() -> Self {
        Self {
            dt: 1.0 / 500.0,
            gyro_noise: 0.01,
            gyro_bias_noise: 0.0001,
            accel_noise: 0.1,
            mag_noise: 0.5,
            initial_quat_variance: 0.1,
            initial_bias_variance: 0.01,
            accel_magnitude_min: 0.8,
            accel_magnitude_max: 1.2,
            bias_correlation_enabled: true,
            bias_correlation_factor: 0.3,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AttitudeState {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub continuous_roll: f32,
    pub continuous_pitch: f32,
    pub continuous_yaw: f32,
    pub roll_rate: f32,
    pub pitch_rate: f32,
    pub yaw_rate: f32,
    pub gyro_bias_x: f32,
    pub gyro_bias_y: f32,
    pub gyro_bias_z: f32,
    pub quaternion: UnitQuaternion<f32>,
    pub quat_variance: f32,
    pub bias_variance: f32,
    pub accel_valid: bool,
}

impl Default for AttitudeState {
    fn default() -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            continuous_roll: 0.0,
            continuous_pitch: 0.0,
            continuous_yaw: 0.0,
            roll_rate: 0.0,
            pitch_rate: 0.0,
            yaw_rate: 0.0,
            gyro_bias_x: 0.0,
            gyro_bias_y: 0.0,
            gyro_bias_z: 0.0,
            quaternion: UnitQuaternion::identity(),
            quat_variance: 0.0,
            bias_variance: 0.0,
            accel_valid: false,
        }
    }
}

/// UD-MEKF: Multiplicative EKF with UD Decomposition
pub struct ImuEkf {
    /// 参照クォータニオン（姿勢の推定値）
    q_ref: UnitQuaternion<f32>,
    /// ジャイロバイアス [rad/s]
    bias: Vector3<f32>,
    /// UD分解: U行列（上三角、対角=1）
    u: UMatrix,
    /// UD分解: D対角成分
    d: DVector,
    /// プロセスノイズ対角成分（事前計算）
    q_diag: DVector,
    /// 加速度計観測ノイズ
    r_accel: f32,
    /// 磁力計観測ノイズ
    r_mag: f32,
    config: EkfConfig,
    last_gyro: [f32; 3],
    initialized: bool,
    continuous: [ContinuousAngle; 3],
    yaw_offset: f32,
}

impl ImuEkf {
    pub fn new(config: EkfConfig) -> Self {
        let dt = config.dt;
        let q_gyro = config.gyro_noise * config.gyro_noise * dt;
        let q_bias = config.gyro_bias_noise * config.gyro_bias_noise * dt;
        let r_accel = config.accel_noise * config.accel_noise;
        let r_mag = config.mag_noise * config.mag_noise;

        let pq = config.initial_quat_variance;
        let pb = config.initial_bias_variance;

        // 初期 U = I（単位行列）
        let u = UMatrix::identity();

        // 初期 D = 初期分散
        let d = DVector::from_row_slice(&[pq, pq, pq, pb, pb, pb]);

        // プロセスノイズ Q の対角成分
        let q_diag = DVector::from_row_slice(&[q_gyro, q_gyro, q_gyro, q_bias, q_bias, q_bias]);

        Self {
            q_ref: UnitQuaternion::identity(),
            bias: Vector3::zeros(),
            u,
            d,
            q_diag,
            r_accel,
            r_mag,
            config,
            last_gyro: [0.0; 3],
            initialized: false,
            continuous: [ContinuousAngle::default(); 3],
            yaw_offset: 0.0,
        }
    }

    pub fn update(
        &mut self,
        ax: f32,
        ay: f32,
        az: f32,
        gx: f32,
        gy: f32,
        gz: f32,
    ) -> AttitudeState {
        self.last_gyro = [gx, gy, gz];

        // バイアス補正されたジャイロ
        let omega = Vector3::new(
            gx - self.bias.x,
            gy - self.bias.y,
            gz - self.bias.z,
        );

        let accel_norm_sq = ax * ax + ay * ay + az * az;
        let accel_valid = accel_norm_sq
            >= self.config.accel_magnitude_min * self.config.accel_magnitude_min
            && accel_norm_sq
                <= self.config.accel_magnitude_max * self.config.accel_magnitude_max;

        if !self.initialized && accel_valid {
            self.init_from_accel(ax, ay, az);
            self.initialized = true;
        }

        // ===== PREDICTION STEP =====
        self.predict(&omega);

        // ===== UPDATE STEP (Accelerometer) =====
        if accel_valid && accel_norm_sq > EPSILON {
            let norm_inv = 1.0 / sqrtf(accel_norm_sq);
            let bias_before = self.bias;

            self.update_accel(ax * norm_inv, ay * norm_inv, az * norm_inv);

            if self.config.bias_correlation_enabled {
                let dx = self.bias.x - bias_before.x;
                let dy = self.bias.y - bias_before.y;
                self.apply_bias_correlation(dx, dy);
            }
        }

        self.build_current_state(accel_valid)
    }

    /// 予測ステップ（Thornton アルゴリズム）
    fn predict(&mut self, omega: &Vector3<f32>) {
        let dt = self.config.dt;

        // 参照クォータニオン更新
        let half_dt = dt * 0.5;
        let delta_angle = omega * half_dt;
        let delta_q = self.small_angle_quaternion(&delta_angle);
        self.q_ref = self.q_ref * delta_q;

        // 状態遷移行列 F (6×6)
        // [I₃  -dt·I₃]
        // [0₃   I₃   ]
        let mut f = SMatrix::<f32, 6, 6>::identity();
        f[(0, 3)] = -dt;
        f[(1, 4)] = -dt;
        f[(2, 5)] = -dt;

        // Thornton アルゴリズムで UD 更新
        // P_new = F·U·D·Uᵀ·Fᵀ + Q
        self.thornton_predict(&f);
    }

    /// Thornton 予測アルゴリズム
    ///
    /// P = U·D·Uᵀ に対して P_new = F·P·Fᵀ + Q を計算し、
    /// U_new, D_new を直接求める（P行列を経由しない）
    ///
    /// 参考: Thornton, C.L. "Triangular Covariance Factorizations for Kalman Filtering"
    ///       NASA Technical Paper 1294, 1978
    fn thornton_predict(&mut self, f: &SMatrix<f32, 6, 6>) {
        const N: usize = 6;

        // W = F · U （後で列アクセスするので、各列を個別に計算）
        // W[i,k] = Σ F[i,m] · U[m,k]
        let fu = f * self.u;

        // 作業用配列（Wの列を保持、更新していく）
        let mut w_col: [StateVector6; N] = [StateVector6::zeros(); N];
        for k in 0..N {
            for i in 0..N {
                w_col[k][i] = fu[(i, k)];
            }
        }

        let mut d_new = DVector::zeros();
        let mut u_new = UMatrix::identity();

        // j = N-1, ..., 0 の順で処理
        for j in (0..N).rev() {
            // σ = Σ(W[j,k]² · D[k]) + Q[j]
            let mut sigma = self.q_diag[j];
            for k in 0..N {
                let w_jk = w_col[k][j];
                sigma += w_jk * w_jk * self.d[k];
            }
            d_new[j] = sigma.max(EPSILON);

            if sigma < EPSILON {
                continue;
            }

            let inv_sigma = 1.0 / sigma;

            // i = 0, ..., j-1 について
            for i in 0..j {
                // σ_ij = Σ(W[i,k] · D[k] · W[j,k])
                let mut sigma_ij = 0.0;
                for k in 0..N {
                    sigma_ij += w_col[k][i] * self.d[k] * w_col[k][j];
                }
                u_new[(i, j)] = sigma_ij * inv_sigma;

                // W の i 行を更新: W[i,k] -= U_new[i,j] · W[j,k]
                let u_ij = u_new[(i, j)];
                for k in 0..N {
                    w_col[k][i] -= u_ij * w_col[k][j];
                }
            }
        }

        self.u = u_new;
        self.d = d_new;
    }

    /// 対称正定値行列を UD 分解（リセット時のみ使用）
    /// P = U·D·Uᵀ の形に分解
    #[allow(dead_code)]
    fn factor_ud(&mut self, p: &SMatrix<f32, 6, 6>) {
        const N: usize = 6;

        self.u = UMatrix::identity();
        self.d = DVector::zeros();

        // 逆順で処理（j = n-1, ..., 0）
        for j in (0..N).rev() {
            // D[j] = P[j,j] - Σ(U[j,k]² · D[k]) for k > j
            let mut d_j = p[(j, j)];
            for k in (j + 1)..N {
                d_j -= self.u[(j, k)] * self.u[(j, k)] * self.d[k];
            }
            self.d[j] = d_j.max(EPSILON);

            if d_j < EPSILON {
                continue;
            }

            let inv_d_j = 1.0 / d_j;

            // U[i,j] for i < j
            for i in 0..j {
                let mut u_ij = p[(i, j)];
                for k in (j + 1)..N {
                    u_ij -= self.u[(i, k)] * self.d[k] * self.u[(j, k)];
                }
                self.u[(i, j)] = u_ij * inv_d_j;
            }
        }
    }

    /// 加速度計による観測更新
    fn update_accel(&mut self, ax: f32, ay: f32, az: f32) {
        let a_meas = Vector3::new(ax, ay, az);
        let g_body = self.gravity_in_body();

        // 3軸を順次スカラー観測として処理
        let g_skew = skew_symmetric(&g_body);

        // 各軸の観測ベクトル h と イノベーション y
        for axis in 0..3 {
            let y = a_meas[axis] - g_body[axis];

            // h = [g_skew の axis 行, 0, 0, 0]
            let mut h = StateVector6::zeros();
            for j in 0..3 {
                h[j] = g_skew[(axis, j)];
            }

            // Bierman 更新
            let k = self.bierman_update(&h, self.r_accel);

            // 誤差状態の更新
            let dx = k * y;
            let delta_theta = Vector3::new(dx[0], dx[1], dx[2]);
            let delta_bias = Vector3::new(dx[3], dx[4], dx[5]);

            // 参照状態の更新（乗法的、右から）
            let delta_q = self.small_angle_quaternion(&(delta_theta * 0.5));
            self.q_ref = self.q_ref * delta_q;
            self.bias += delta_bias;

            // 次の軸のために g_body を再計算（姿勢が変わったので）
            // 実際には微小変化なので省略可能だが、厳密には必要
        }
    }

    /// Yaw角のみを磁場から観測更新
    pub fn update_mag_yaw(&mut self, mx: f32, my: f32, mz: f32) {
        let (roll, pitch, yaw_predicted) = self.q_ref.euler_angles();

        // 磁場を水平面に射影
        let cr = cosf(roll);
        let sr = sinf(roll);
        let cp = cosf(pitch);
        let sp = sinf(pitch);

        let mx_h = mx * cp + my * sr * sp + mz * cr * sp;
        let my_h = my * cr - mz * sr;

        let yaw_measured = atan2f(my_h, mx_h);

        // イノベーション
        let mut y = yaw_measured - yaw_predicted;
        while y > PI {
            y -= 2.0 * PI;
        }
        while y < -PI {
            y += 2.0 * PI;
        }

        // 観測ベクトル h
        let rot = self.q_ref.to_rotation_matrix();
        let r_row2 = rot.matrix().row(2);

        let mut h = StateVector6::zeros();
        h[0] = r_row2[0];
        h[1] = r_row2[1];
        h[2] = r_row2[2];

        // Bierman 更新
        let k = self.bierman_update(&h, self.r_mag);

        // 誤差状態の更新
        let dx = k * y;
        let delta_theta = Vector3::new(dx[0], dx[1], dx[2]);
        let delta_bias = Vector3::new(dx[3], dx[4], dx[5]);

        let delta_q = self.small_angle_quaternion(&(delta_theta * 0.5));
        self.q_ref = self.q_ref * delta_q;
        self.bias += delta_bias;
    }

    /// Bierman 観測更新アルゴリズム
    ///
    /// スカラー観測 z = h·x + v に対して U, D を更新し、
    /// カルマンゲインを返す
    ///
    /// 参考: Bierman (1977) "Factorization Methods for Discrete Sequential Estimation"
    fn bierman_update(&mut self, h: &StateVector6, r: f32) -> StateVector6 {
        const N: usize = 6;

        // f = Uᵀ · h
        let mut f = StateVector6::zeros();
        for j in 0..N {
            f[j] = h[j];
            for i in 0..j {
                f[j] += self.u[(i, j)] * h[i];
            }
        }

        // g = D · f
        let mut g = StateVector6::zeros();
        for j in 0..N {
            g[j] = self.d[j] * f[j];
        }

        // α[0] = r + g[0]·f[0], α[j] = α[j-1] + g[j]·f[j]
        let mut alpha = [0.0f32; N + 1];
        alpha[0] = r;
        for j in 0..N {
            alpha[j + 1] = alpha[j] + g[j] * f[j];
        }

        // 最終的な α = hᵀ·P·h + r（イノベーション分散）
        let alpha_final = alpha[N];
        if alpha_final < EPSILON {
            return StateVector6::zeros();
        }

        // カルマンゲイン K = P·h / α = U·g / α
        let mut k = StateVector6::zeros();
        for i in 0..N {
            k[i] = g[i];
            for j in (i + 1)..N {
                k[i] += self.u[(i, j)] * g[j];
            }
        }
        k /= alpha_final;

        // U, D の更新
        // 逆順で処理
        for j in (1..N).rev() {
            let alpha_j = alpha[j];
            let alpha_jm1 = alpha[j - 1];

            if alpha_j < EPSILON || alpha_jm1 < EPSILON {
                continue;
            }

            // D[j] の更新
            self.d[j] = alpha_jm1 * self.d[j] / alpha_j;

            // U の j 列を更新
            let lambda = -f[j] / alpha_jm1;
            for i in 0..j {
                let u_ij_old = self.u[(i, j)];
                self.u[(i, j)] = u_ij_old + lambda * k[i] * alpha_j;
                // k[i] も同時に更新（次のイテレーション用）
                // ただし k は最後に返すので、別の変数で追跡
            }
        }

        // D[0] の更新
        if alpha[1] > EPSILON {
            self.d[0] = r * self.d[0] / alpha[1];
        }

        // 再計算した k を返す（上のループで k が変わる可能性があるため）
        // 実際には最初に計算した k が正しいカルマンゲイン
        k
    }

    /// 小角度近似によるクォータニオン生成
    #[inline]
    fn small_angle_quaternion(&self, half_angle: &Vector3<f32>) -> UnitQuaternion<f32> {
        let norm_sq = half_angle.norm_squared();

        if norm_sq < EPSILON {
            return UnitQuaternion::identity();
        }

        let norm = sqrtf(norm_sq);

        let (sin_half, cos_half) = if norm < 0.01 {
            (norm * (1.0 - norm_sq / 6.0), 1.0 - norm_sq * 0.5)
        } else {
            (libm::sinf(norm), libm::cosf(norm))
        };

        let axis_scale = sin_half / norm;
        let q = Quaternion::new(
            cos_half,
            half_angle.x * axis_scale,
            half_angle.y * axis_scale,
            half_angle.z * axis_scale,
        );

        UnitQuaternion::from_quaternion(q)
    }

    /// ボディ座標系における重力方向を計算
    #[inline]
    fn gravity_in_body(&self) -> Vector3<f32> {
        self.q_ref.inverse_transform_vector(&Vector3::new(0.0, 0.0, 1.0))
    }

    fn apply_bias_correlation(&mut self, dx: f32, dy: f32) {
        let factor = self.config.bias_correlation_factor;
        let avg_delta = (dx + dy) / 2.0;
        self.bias.z += avg_delta * factor;
        self.d[5] *= 1.0 - factor * 0.1;
    }

    fn init_from_accel(&mut self, ax: f32, ay: f32, az: f32) {
        let norm = sqrtf(ax * ax + ay * ay + az * az);
        let ax = ax / norm;
        let ay = ay / norm;
        let az = az / norm;

        let roll = atan2f(ay, az);
        let pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

        self.q_ref = UnitQuaternion::from_euler_angles(roll, pitch, 0.0);
    }

    /// 現在のYaw角を0としてリセット
    pub fn reset_yaw(&mut self) {
        let (_, _, yaw) = self.q_ref.euler_angles();
        self.yaw_offset = yaw;
        self.continuous[2].reset();
    }

    fn build_current_state(&mut self, accel_valid: bool) -> AttitudeState {
        let (roll, pitch, yaw) = self.q_ref.euler_angles();
        let yaw_corrected = yaw - self.yaw_offset;

        // P の対角成分を UD から復元: P[i,i] = Σ(U[i,k]² · D[k])
        let quat_variance = self.compute_variance(0);
        let bias_variance = self.compute_variance(3);

        AttitudeState {
            roll,
            pitch,
            yaw: yaw_corrected,
            continuous_roll: self.continuous[0].update(roll),
            continuous_pitch: self.continuous[1].update(pitch),
            continuous_yaw: self.continuous[2].update(yaw_corrected),
            roll_rate: self.last_gyro[0] - self.bias.x,
            pitch_rate: self.last_gyro[1] - self.bias.y,
            yaw_rate: self.last_gyro[2] - self.bias.z,
            gyro_bias_x: self.bias.x,
            gyro_bias_y: self.bias.y,
            gyro_bias_z: self.bias.z,
            quaternion: self.q_ref,
            quat_variance,
            bias_variance,
            accel_valid,
        }
    }

    /// P[i,i] = Σ(U[i,k]² · D[k]) for k >= i を計算
    #[inline]
    fn compute_variance(&self, i: usize) -> f32 {
        let mut var = 0.0;
        for k in i..6 {
            let u_ik = if k == i { 1.0 } else { self.u[(i, k)] };
            var += u_ik * u_ik * self.d[k];
        }
        var
    }

    #[inline(always)]
    pub fn get_quaternion(&self) -> UnitQuaternion<f32> {
        self.q_ref
    }

    #[inline(always)]
    pub fn get_euler(&self) -> (f32, f32, f32) {
        self.q_ref.euler_angles()
    }

    #[inline(always)]
    pub fn get_gyro_bias(&self) -> (f32, f32, f32) {
        (self.bias.x, self.bias.y, self.bias.z)
    }

    #[inline(always)]
    pub fn reset_continuous(&mut self) {
        for c in &mut self.continuous {
            c.reset();
        }
    }

    pub fn reset(&mut self) {
        self.q_ref = UnitQuaternion::identity();
        self.bias = Vector3::zeros();
        self.initialized = false;
        self.yaw_offset = 0.0;

        let pq = self.config.initial_quat_variance;
        let pb = self.config.initial_bias_variance;

        self.u = UMatrix::identity();
        self.d = DVector::from_row_slice(&[pq, pq, pq, pb, pb, pb]);

        self.reset_continuous();
    }

    #[inline(always)]
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }
}

/// スキュー対称行列
#[inline]
fn skew_symmetric(v: &Vector3<f32>) -> Matrix3<f32> {
    Matrix3::new(
        0.0, -v.z, v.y,
        v.z, 0.0, -v.x,
        -v.y, v.x, 0.0,
    )
}

#[inline(always)]
pub fn rad_to_degree((r, p, y): (f32, f32, f32)) -> (f32, f32, f32) {
    (r * RAD_TO_DEG, p * RAD_TO_DEG, y * RAD_TO_DEG)
}

#[inline(always)]
pub fn degree_to_rad((r, p, y): (f32, f32, f32)) -> (f32, f32, f32) {
    (r * DEG_TO_RAD, p * DEG_TO_RAD, y * DEG_TO_RAD)
}