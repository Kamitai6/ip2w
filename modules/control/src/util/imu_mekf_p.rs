//! Multiplicative Extended Kalman Filter (MEKF) for IMU Attitude Estimation
//!
//! 6次元誤差状態 [δθ, δb] + 参照クォータニオンによる乗法的更新
//! 加速度計 + 磁力計による9軸センサーフュージョン

#![no_std]

use core::f32::{consts::PI, EPSILON};
use libm::{atan2f, cosf, sinf, sqrtf};
use nalgebra::{Matrix3, Quaternion, SMatrix, SVector, UnitQuaternion, Vector3};

const DEG_TO_RAD: f32 = PI / 180.0;
const RAD_TO_DEG: f32 = 180.0 / PI;

// 型エイリアス
type CovMatrix = SMatrix<f32, 6, 6>;
type StateVector6 = SVector<f32, 6>;
type ObsMatrix3x6 = SMatrix<f32, 3, 6>;
type KalmanGain3 = SMatrix<f32, 6, 3>;

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

/// Multiplicative Extended Kalman Filter for IMU
pub struct ImuEkf {
    /// 参照クォータニオン（姿勢の推定値）
    q_ref: UnitQuaternion<f32>,
    /// ジャイロバイアス [rad/s]
    bias: Vector3<f32>,
    /// 誤差状態の共分散行列 (6×6)
    /// 誤差状態: [δθx, δθy, δθz, δbx, δby, δbz]
    p: CovMatrix,
    /// プロセスノイズ行列（事前計算）
    q_mat: CovMatrix,
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

        // 初期共分散 (6×6)
        // [δθ variance (3), δb variance (3)]
        let p = CovMatrix::from_diagonal(&StateVector6::from_row_slice(&[
            pq, pq, pq, pb, pb, pb,
        ]));

        // プロセスノイズ行列（事前計算）
        let q_mat = CovMatrix::from_diagonal(&StateVector6::from_row_slice(&[
            q_gyro, q_gyro, q_gyro, q_bias, q_bias, q_bias,
        ]));

        Self {
            q_ref: UnitQuaternion::identity(),
            bias: Vector3::zeros(),
            p,
            q_mat,
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

        self.ensure_positive_definite();

        self.build_current_state(accel_valid)
    }

    /// 予測ステップ
    fn predict(&mut self, omega: &Vector3<f32>) {
        let dt = self.config.dt;

        // 参照クォータニオン更新
        // ボディ座標系での角速度なので、右から掛ける: q_new = q_old ⊗ δq
        let half_dt = dt * 0.5;
        let delta_angle = omega * half_dt;
        let delta_q = self.small_angle_quaternion(&delta_angle);
        self.q_ref = self.q_ref * delta_q;

        // 状態遷移行列 F (6×6)
        // δx_new = F * δx
        // [δθ_new]   [I₃  -dt*I₃] [δθ]
        // [δb_new] = [0₃   I₃   ] [δb]
        let mut f = CovMatrix::identity();
        f[(0, 3)] = -dt;
        f[(1, 4)] = -dt;
        f[(2, 5)] = -dt;

        // 共分散更新: P = F * P * Fᵀ + Q
        self.p = f * self.p * f.transpose() + self.q_mat;
    }

    /// 加速度計による観測更新
    fn update_accel(&mut self, ax: f32, ay: f32, az: f32) {
        // 測定された加速度（正規化済み）
        let a_meas = Vector3::new(ax, ay, az);

        // 予測される重力方向（ボディ座標系）
        // g_body = R(q_ref)ᵀ * [0, 0, 1]ᵀ
        let g_body = self.gravity_in_body();

        // イノベーション
        let y = a_meas - g_body;

        // 観測ヤコビアン H (3×6)
        // h(δθ) = R(δq)ᵀ * g_body ≈ g_body + [g_body]ₓ * δθ
        // ∂h/∂δθ = [g_body]ₓ
        // ∂h/∂δb = 0
        let g_skew = skew_symmetric(&g_body);
        let mut h = ObsMatrix3x6::zeros();
        // 左上3×3に [g_body]ₓ
        for i in 0..3 {
            for j in 0..3 {
                h[(i, j)] = g_skew[(i, j)];
            }
        }
        // 右3×3は0（バイアスに対する偏微分なし）

        // 観測ノイズ R
        let r = Matrix3::from_diagonal(&Vector3::new(self.r_accel, self.r_accel, self.r_accel));

        // イノベーション共分散 S = H * P * Hᵀ + R
        let s = h * self.p * h.transpose() + r;

        // カルマンゲイン K = P * Hᵀ * S⁻¹
        let s_inv = s
            .try_inverse()
            .unwrap_or_else(|| Matrix3::identity() * (1.0 / EPSILON));
        let k: KalmanGain3 = self.p * h.transpose() * s_inv;

        // 誤差状態の推定
        let dx = k * y;
        let delta_theta = Vector3::new(dx[0], dx[1], dx[2]);
        let delta_bias = Vector3::new(dx[3], dx[4], dx[5]);

        // 参照状態の更新（乗法的、右から）
        let delta_q = self.small_angle_quaternion(&(delta_theta * 0.5));
        self.q_ref = self.q_ref * delta_q;
        self.bias += delta_bias;

        // 共分散更新（Joseph形式）
        let i_kh = CovMatrix::identity() - k * h;
        self.p = i_kh * self.p * i_kh.transpose() + k * r * k.transpose();
    }

    /// Yaw角のみを磁場から観測更新
    pub fn update_mag_yaw(&mut self, mx: f32, my: f32, mz: f32) {
        // 現在の姿勢からオイラー角を取得
        let (roll, pitch, yaw_predicted) = self.q_ref.euler_angles();

        // 磁場を水平面に射影（Z-up座標系）
        let cr = cosf(roll);
        let sr = sinf(roll);
        let cp = cosf(pitch);
        let sp = sinf(pitch);

        let mx_h = mx * cp + my * sr * sp + mz * cr * sp;
        let my_h = my * cr - mz * sr;

        // 測定Yaw
        let yaw_measured = atan2f(my_h, mx_h);

        // イノベーション（角度正規化）
        let mut y = yaw_measured - yaw_predicted;
        while y > PI {
            y -= 2.0 * PI;
        }
        while y < -PI {
            y += 2.0 * PI;
        }

        // Yawに対する観測ヤコビアン H (1×6)
        // δθはボディ座標系、Yawはワールド座標系のZ軸回転
        // δyaw = (R(q_ref)の3行目) · δθ_body
        let rot = self.q_ref.to_rotation_matrix();
        let r_row2 = rot.matrix().row(2); // 3行目（0-indexed で 2）
        
        let mut h = SMatrix::<f32, 1, 6>::zeros();
        h[(0, 0)] = r_row2[0];
        h[(0, 1)] = r_row2[1];
        h[(0, 2)] = r_row2[2];
        // バイアス部分は0

        // S = H * P * Hᵀ + R (スカラー)
        let hp = h * self.p;
        let s = (hp * h.transpose())[(0, 0)] + self.r_mag;

        if s.abs() < EPSILON {
            return;
        }

        // K = P * Hᵀ / S (6×1)
        let k = self.p * h.transpose() / s;

        // 誤差状態の推定
        let dx = k * y;
        let delta_theta = Vector3::new(dx[(0, 0)], dx[(1, 0)], dx[(2, 0)]);
        let delta_bias = Vector3::new(dx[(3, 0)], dx[(4, 0)], dx[(5, 0)]);

        // 参照状態の更新（乗法的、右から）
        let delta_q = self.small_angle_quaternion(&(delta_theta * 0.5));
        self.q_ref = self.q_ref * delta_q;
        self.bias += delta_bias;

        // 共分散更新（Joseph形式）
        let i_kh = CovMatrix::identity() - k * h;
        self.p = i_kh * self.p * i_kh.transpose() + k * self.r_mag * k.transpose();
    }

    /// 小角度近似によるクォータニオン生成
    /// delta_angle = ω * dt / 2 のような小さな回転ベクトル
    #[inline]
    fn small_angle_quaternion(&self, half_angle: &Vector3<f32>) -> UnitQuaternion<f32> {
        let norm_sq = half_angle.norm_squared();
        
        if norm_sq < EPSILON {
            return UnitQuaternion::identity();
        }

        // exp(θ/2) = [cos(|θ/2|), sin(|θ/2|) * θ/|θ|]
        // 小角度近似: cos(x) ≈ 1 - x²/2, sin(x) ≈ x
        let norm = sqrtf(norm_sq);
        let half_norm = norm;
        
        let (sin_half, cos_half) = if half_norm < 0.01 {
            // Taylor展開（より精度の高い近似）
            (half_norm * (1.0 - norm_sq / 6.0), 1.0 - norm_sq * 0.5)
        } else {
            (libm::sinf(half_norm), libm::cosf(half_norm))
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
    /// g_body = R(q_ref)ᵀ * [0, 0, 1]ᵀ
    #[inline]
    fn gravity_in_body(&self) -> Vector3<f32> {
        // inverse_transform_vector は R^T * v を計算
        self.q_ref.inverse_transform_vector(&Vector3::new(0.0, 0.0, 1.0))
    }

    fn apply_bias_correlation(&mut self, dx: f32, dy: f32) {
        let factor = self.config.bias_correlation_factor;
        let avg_delta = (dx + dy) / 2.0;
        self.bias.z += avg_delta * factor;
        self.p[(5, 5)] *= 1.0 - factor * 0.1;
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

    fn ensure_positive_definite(&mut self) {
        const MIN_VAR: f32 = EPSILON;
        const MAX_VAR_THETA: f32 = 1.0;
        const MAX_VAR_BIAS: f32 = 0.1;

        // 対角成分のクランプ
        for i in 0..3 {
            self.p[(i, i)] = self.p[(i, i)].clamp(MIN_VAR, MAX_VAR_THETA);
        }
        for i in 3..6 {
            self.p[(i, i)] = self.p[(i, i)].clamp(MIN_VAR, MAX_VAR_BIAS);
        }

        // 対称性の強制と非対角成分のクランプ
        for i in 0..6 {
            for j in (i + 1)..6 {
                let max_val = sqrtf(self.p[(i, i)] * self.p[(j, j)]);
                let avg = (self.p[(i, j)] + self.p[(j, i)]) * 0.5;
                let clamped = avg.clamp(-max_val, max_val);
                self.p[(i, j)] = clamped;
                self.p[(j, i)] = clamped;
            }
        }
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
            quat_variance: self.p[(0, 0)],  // δθx variance
            bias_variance: self.p[(3, 3)],  // δbx variance
            accel_valid,
        }
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
        self.p = CovMatrix::from_diagonal(&StateVector6::from_row_slice(&[
            pq, pq, pq, pb, pb, pb,
        ]));

        self.reset_continuous();
    }

    #[inline(always)]
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    // ===== 検証・デバッグ用メソッド =====

    /// P 行列の対角成分を取得（分散）
    pub fn get_variances(&self) -> [f32; 6] {
        [
            self.p[(0, 0)],
            self.p[(1, 1)],
            self.p[(2, 2)],
            self.p[(3, 3)],
            self.p[(4, 4)],
            self.p[(5, 5)],
        ]
    }

    /// P 行列全体を取得（行優先）
    pub fn get_p(&self) -> [[f32; 6]; 6] {
        let mut result = [[0.0f32; 6]; 6];
        for i in 0..6 {
            for j in 0..6 {
                result[i][j] = self.p[(i, j)];
            }
        }
        result
    }

    /// P 行列の正定値性をチェック
    pub fn check_positive_definite(&self) -> bool {
        // 全ての対角成分が正
        for i in 0..6 {
            if self.p[(i, i)] <= 0.0 {
                return false;
            }
        }
        // 対称性チェック
        for i in 0..6 {
            for j in (i + 1)..6 {
                if (self.p[(i, j)] - self.p[(j, i)]).abs() > 1e-6 {
                    return false;
                }
            }
        }
        true
    }
}

/// スキュー対称行列（外積の行列表現）
/// [v]ₓ = [[0, -vz, vy], [vz, 0, -vx], [-vy, vx, 0]]
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