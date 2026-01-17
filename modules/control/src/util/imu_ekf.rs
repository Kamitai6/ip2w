//! Quaternion Extended Kalman Filter for IMU Attitude Estimation
//!
//! nalgebra版 - 加速度計 + 磁力計による9軸センサーフュージョン

#![no_std]

use core::f32::{consts::PI, EPSILON};
use libm::{atan2f, sqrtf};
use nalgebra::{Matrix3, Quaternion, SMatrix, SVector, UnitQuaternion, Vector3};

const DEG_TO_RAD: f32 = PI / 180.0;
const RAD_TO_DEG: f32 = 180.0 / PI;

// 状態ベクトル: [q0, q1, q2, q3, bias_x, bias_y, bias_z]
type StateVector = SVector<f32, 7>;
type CovMatrix = SMatrix<f32, 7, 7>;
type ObsMatrix3x7 = SMatrix<f32, 3, 7>;
type KalmanGain3 = SMatrix<f32, 7, 3>;

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
            self.prev = angle;
        }
        if (angle * self.prev) < 0.0 {
            let diff = angle - self.prev;
            if diff < -PI {
                self.turns += PI;
            } else if diff > PI {
                self.turns -= PI;
            }
        }
        self.prev = angle;
        angle + self.turns
    }

    fn reset(&mut self) {
        *self = Self::default();
    }
}

/// EKF Configuration
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
            dt: 1.0 / 400.0,
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

pub struct ImuEkf {
    /// 状態ベクトル [q0, q1, q2, q3, bias_x, bias_y, bias_z]
    x: StateVector,
    /// 共分散行列 7x7
    p: CovMatrix,
    /// プロセスノイズ行列（事前計算）
    q_mat: CovMatrix,
    /// 加速度計観測ノイズ
    r_accel: f32,
    /// 磁力計観測ノイズ
    r_mag: f32,
    /// 基準磁場ベクトル（NED座標系）
    mag_ref: Option<Vector3<f32>>,
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

        // 初期状態: 単位クォータニオン + ゼロバイアス
        let x = StateVector::from_row_slice(&[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);

        // 初期共分散
        let p = CovMatrix::from_diagonal(&SVector::<f32, 7>::from_row_slice(&[
            pq, pq, pq, pq, pb, pb, pb,
        ]));

        // プロセスノイズ行列（事前計算）
        let q_mat = CovMatrix::from_diagonal(&SVector::<f32, 7>::from_row_slice(&[
            q_gyro, q_gyro, q_gyro, q_gyro, q_bias, q_bias, q_bias,
        ]));

        Self {
            x,
            p,
            q_mat,
            r_accel,
            r_mag,
            mag_ref: None,
            config,
            last_gyro: [0.0; 3],
            initialized: false,
            continuous: [ContinuousAngle::default(); 3],
            yaw_offset: 0.0,
        }
    }

    /// 基準磁場ベクトルを設定（ハードアイアン補正済みの値）
    ///
    /// デバイスが水平の状態で、補正済み磁力計データを渡す。
    /// 内部でノルム正規化される。
    pub fn set_mag_reference(&mut self, mx: f32, my: f32, mz: f32) {
        let norm = sqrtf(mx * mx + my * my + mz * mz);
        if norm > EPSILON {
            self.mag_ref = Some(Vector3::new(mx / norm, my / norm, mz / norm));
        }
    }

    /// X軸が上向きの座標系で基準磁場を設定
    /// 
    /// X_up状態で、キャリブレーション済み磁力計データを渡す
    pub fn set_mag_reference_x_up(&mut self, mx: f32, my: f32, mz: f32) {
        // update_mag_x_up と同じ変換を適用
        self.set_mag_reference(-mz, my, mx)
    }

    /// 基準磁場ベクトルをクリア
    pub fn clear_mag_reference(&mut self) {
        self.mag_ref = None;
    }

    /// 基準磁場が設定されているか
    pub fn has_mag_reference(&self) -> bool {
        self.mag_ref.is_some()
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

        let wx = gx - self.x[4];
        let wy = gy - self.x[5];
        let wz = gz - self.x[6];

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
        self.predict(wx, wy, wz);

        // ===== UPDATE STEP (Accelerometer) =====
        if accel_valid && accel_norm_sq > EPSILON {
            let norm_inv = 1.0 / sqrtf(accel_norm_sq);
            let bias_before = [self.x[4], self.x[5], self.x[6]];

            self.update_accel(ax * norm_inv, ay * norm_inv, az * norm_inv);

            if self.config.bias_correlation_enabled {
                let dx = self.x[4] - bias_before[0];
                let dy = self.x[5] - bias_before[1];
                self.apply_bias_correlation(dx, dy);
            }
        }

        self.normalize_quaternion();
        self.ensure_positive_definite();

        self.build_current_state(accel_valid)
    }

    /// 磁力計による更新（2段階目）
    ///
    /// ハードアイアン補正済みの磁力計データを渡す。
    /// set_mag_reference()で基準磁場を設定していない場合は何もしない。
    pub fn update_mag(&mut self, mx: f32, my: f32, mz: f32) {
        let mag_ref = match self.mag_ref {
            Some(ref m) => *m,
            None => return,
        };

        // 観測値を正規化
        let mag_norm = sqrtf(mx * mx + my * my + mz * mz);
        if mag_norm < EPSILON {
            return;
        }
        let mx = mx / mag_norm;
        let my = my / mag_norm;
        let mz = mz / mag_norm;

        let q0 = self.x[0];
        let q1 = self.x[1];
        let q2 = self.x[2];
        let q3 = self.x[3];

        // 回転行列 R(q) のボディからワールドへの変換
        // 予測観測 h = R(q)^T * m_ref （ワールドからボディへ）
        let q = UnitQuaternion::from_quaternion(Quaternion::new(q0, q1, q2, q3));
        let h_vec = q.inverse_transform_vector(&mag_ref);

        // イノベーション
        let y = Vector3::new(mx - h_vec.x, my - h_vec.y, mz - h_vec.z);

        // 観測ヤコビアン H (3x7)
        // h = R(q)^T * m_ref の q に対する偏微分
        let h = self.compute_mag_jacobian(&mag_ref);

        // 観測ノイズ R
        let r = Matrix3::from_diagonal(&Vector3::new(self.r_mag, self.r_mag, self.r_mag));

        // イノベーション共分散 S = H * P * H^T + R
        let s = h * self.p * h.transpose() + r;

        // カルマンゲイン K = P * H^T * S^-1
        let s_inv = s
            .try_inverse()
            .unwrap_or_else(|| Matrix3::identity() * EPSILON);
        let k: KalmanGain3 = self.p * h.transpose() * s_inv;

        // 状態更新
        let dx = k * y;
        self.x += dx;

        // Joseph形式の共分散更新
        let i_kh = CovMatrix::identity() - k * h;
        self.p = i_kh * self.p * i_kh.transpose() + k * r * k.transpose();

        self.normalize_quaternion();
        self.ensure_positive_definite();
    }

    /// 磁力計観測のヤコビアン計算
    ///
    /// h = R(q)^T * m_ref の q に対する偏微分
    fn compute_mag_jacobian(&self, m_ref: &Vector3<f32>) -> ObsMatrix3x7 {
        let q0 = self.x[0];
        let q1 = self.x[1];
        let q2 = self.x[2];
        let q3 = self.x[3];

        let mx = m_ref.x;
        let my = m_ref.y;
        let mz = m_ref.z;

        // R(q)^T の各成分:
        // R^T[0,0] = 1 - 2(q2² + q3²)
        // R^T[0,1] = 2(q1q2 - q0q3)
        // R^T[0,2] = 2(q1q3 + q0q2)
        // R^T[1,0] = 2(q1q2 + q0q3)
        // R^T[1,1] = 1 - 2(q1² + q3²)
        // R^T[1,2] = 2(q2q3 - q0q1)
        // R^T[2,0] = 2(q1q3 - q0q2)
        // R^T[2,1] = 2(q2q3 + q0q1)
        // R^T[2,2] = 1 - 2(q1² + q2²)

        // h = R^T * m なので:
        // hx = (1-2q2²-2q3²)mx + 2(q1q2-q0q3)my + 2(q1q3+q0q2)mz
        // hy = 2(q1q2+q0q3)mx + (1-2q1²-2q3²)my + 2(q2q3-q0q1)mz
        // hz = 2(q1q3-q0q2)mx + 2(q2q3+q0q1)my + (1-2q1²-2q2²)mz

        // ∂hx/∂q0 = -2q3*my + 2q2*mz
        // ∂hx/∂q1 = 2q2*my + 2q3*mz
        // ∂hx/∂q2 = -4q2*mx + 2q1*my + 2q0*mz
        // ∂hx/∂q3 = -4q3*mx - 2q0*my + 2q1*mz

        // ∂hy/∂q0 = 2q3*mx - 2q1*mz
        // ∂hy/∂q1 = 2q2*mx - 4q1*my - 2q0*mz
        // ∂hy/∂q2 = 2q1*mx + 2q3*mz
        // ∂hy/∂q3 = 2q0*mx - 4q3*my + 2q2*mz

        // ∂hz/∂q0 = -2q2*mx + 2q1*my
        // ∂hz/∂q1 = 2q3*mx + 2q0*my - 4q1*mz
        // ∂hz/∂q2 = -2q0*mx + 2q3*my - 4q2*mz
        // ∂hz/∂q3 = 2q1*mx + 2q2*my

        let mut h = ObsMatrix3x7::zeros();

        // ∂hx/∂q
        h[(0, 0)] = -2.0 * q3 * my + 2.0 * q2 * mz;
        h[(0, 1)] = 2.0 * q2 * my + 2.0 * q3 * mz;
        h[(0, 2)] = -4.0 * q2 * mx + 2.0 * q1 * my + 2.0 * q0 * mz;
        h[(0, 3)] = -4.0 * q3 * mx - 2.0 * q0 * my + 2.0 * q1 * mz;

        // ∂hy/∂q
        h[(1, 0)] = 2.0 * q3 * mx - 2.0 * q1 * mz;
        h[(1, 1)] = 2.0 * q2 * mx - 4.0 * q1 * my - 2.0 * q0 * mz;
        h[(1, 2)] = 2.0 * q1 * mx + 2.0 * q3 * mz;
        h[(1, 3)] = 2.0 * q0 * mx - 4.0 * q3 * my + 2.0 * q2 * mz;

        // ∂hz/∂q
        h[(2, 0)] = -2.0 * q2 * mx + 2.0 * q1 * my;
        h[(2, 1)] = 2.0 * q3 * mx + 2.0 * q0 * my - 4.0 * q1 * mz;
        h[(2, 2)] = -2.0 * q0 * mx + 2.0 * q3 * my - 4.0 * q2 * mz;
        h[(2, 3)] = 2.0 * q1 * mx + 2.0 * q2 * my;

        // バイアスに対する偏微分は0（列4,5,6は0のまま）

        h
    }

    fn predict(&mut self, wx: f32, wy: f32, wz: f32) {
        let dt = self.config.dt;
        let dt_half = dt * 0.5;

        let q0 = self.x[0];
        let q1 = self.x[1];
        let q2 = self.x[2];
        let q3 = self.x[3];

        // クォータニオン微分
        let dq0 = dt_half * (-q1 * wx - q2 * wy - q3 * wz);
        let dq1 = dt_half * (q0 * wx + q2 * wz - q3 * wy);
        let dq2 = dt_half * (q0 * wy - q1 * wz + q3 * wx);
        let dq3 = dt_half * (q0 * wz + q1 * wy - q2 * wx);

        self.x[0] += dq0;
        self.x[1] += dq1;
        self.x[2] += dq2;
        self.x[3] += dq3;
        // バイアスは変化なし

        // ヤコビアン F の構築
        let mut f = SMatrix::<f32, 7, 7>::identity();

        // クォータニオン部分の微分
        f[(0, 1)] = -dt_half * wx;
        f[(0, 2)] = -dt_half * wy;
        f[(0, 3)] = -dt_half * wz;
        f[(1, 0)] = dt_half * wx;
        f[(1, 2)] = dt_half * wz;
        f[(1, 3)] = -dt_half * wy;
        f[(2, 0)] = dt_half * wy;
        f[(2, 1)] = -dt_half * wz;
        f[(2, 3)] = dt_half * wx;
        f[(3, 0)] = dt_half * wz;
        f[(3, 1)] = dt_half * wy;
        f[(3, 2)] = -dt_half * wx;

        // バイアスに対するクォータニオンの偏微分
        f[(0, 4)] = dt_half * q1;
        f[(0, 5)] = dt_half * q2;
        f[(0, 6)] = dt_half * q3;
        f[(1, 4)] = -dt_half * q0;
        f[(1, 5)] = -dt_half * q3;
        f[(1, 6)] = dt_half * q2;
        f[(2, 4)] = dt_half * q3;
        f[(2, 5)] = -dt_half * q0;
        f[(2, 6)] = -dt_half * q1;
        f[(3, 4)] = -dt_half * q2;
        f[(3, 5)] = dt_half * q1;
        f[(3, 6)] = -dt_half * q0;

        // 共分散更新: P = F * P * F^T + Q
        self.p = f * self.p * f.transpose() + self.q_mat;

        self.ensure_positive_definite();
    }

    fn update_accel(&mut self, ax: f32, ay: f32, az: f32) {
        let q0 = self.x[0];
        let q1 = self.x[1];
        let q2 = self.x[2];
        let q3 = self.x[3];

        // 予測加速度（重力方向）
        let hx = 2.0 * (q1 * q3 - q0 * q2);
        let hy = 2.0 * (q0 * q1 + q2 * q3);
        let hz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // イノベーション
        let y = Vector3::new(ax - hx, ay - hy, az - hz);

        // 観測ヤコビアン H (3x7)
        let mut h = ObsMatrix3x7::zeros();
        h[(0, 0)] = -2.0 * q2;
        h[(0, 1)] = 2.0 * q3;
        h[(0, 2)] = -2.0 * q0;
        h[(0, 3)] = 2.0 * q1;
        h[(1, 0)] = 2.0 * q1;
        h[(1, 1)] = 2.0 * q0;
        h[(1, 2)] = 2.0 * q3;
        h[(1, 3)] = 2.0 * q2;
        h[(2, 0)] = 2.0 * q0;
        h[(2, 1)] = -2.0 * q1;
        h[(2, 2)] = -2.0 * q2;
        h[(2, 3)] = 2.0 * q3;

        // 観測ノイズ R
        let r = Matrix3::from_diagonal(&Vector3::new(self.r_accel, self.r_accel, self.r_accel));

        // イノベーション共分散 S = H * P * H^T + R
        let s = h * self.p * h.transpose() + r;

        // カルマンゲイン K = P * H^T * S^-1
        let s_inv = s
            .try_inverse()
            .unwrap_or_else(|| Matrix3::identity() * EPSILON);
        let k: KalmanGain3 = self.p * h.transpose() * s_inv;

        // 状態更新
        let dx = k * y;
        self.x += dx;

        // Joseph形式の共分散更新: P = (I - K*H) * P * (I - K*H)^T + K * R * K^T
        let i_kh = CovMatrix::identity() - k * h;
        self.p = i_kh * self.p * i_kh.transpose() + k * r * k.transpose();
    }

    fn apply_bias_correlation(&mut self, dx: f32, dy: f32) {
        let factor = self.config.bias_correlation_factor;
        let avg_delta = (dx + dy) / 2.0;
        self.x[6] += avg_delta * factor;
        self.p[(6, 6)] *= 1.0 - factor * 0.1;
    }

    fn init_from_accel(&mut self, ax: f32, ay: f32, az: f32) {
        let norm = sqrtf(ax * ax + ay * ay + az * az);
        let ax = ax / norm;
        let ay = ay / norm;
        let az = az / norm;

        let roll = atan2f(ay, az);
        let pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

        // nalgebraのUnitQuaternionを使用してroll/pitchから生成
        let q = UnitQuaternion::from_euler_angles(roll, pitch, 0.0);
        let quat = q.quaternion();

        self.x[0] = quat.w;
        self.x[1] = quat.i;
        self.x[2] = quat.j;
        self.x[3] = quat.k;
    }

    fn normalize_quaternion(&mut self) {
        let q = Quaternion::new(self.x[0], self.x[1], self.x[2], self.x[3]);
        if q.norm() > EPSILON {
            let q_unit = UnitQuaternion::from_quaternion(q);
            let quat = q_unit.quaternion();
            self.x[0] = quat.w;
            self.x[1] = quat.i;
            self.x[2] = quat.j;
            self.x[3] = quat.k;
        }
    }

    fn ensure_positive_definite(&mut self) {
        const MIN_VAR: f32 = EPSILON;
        const MAX_VAR: f32 = 1.0;
        const MAX_BIAS_VAR: f32 = 0.1;

        // 対角成分のクランプ
        for i in 0..4 {
            self.p[(i, i)] = self.p[(i, i)].clamp(MIN_VAR, MAX_VAR);
        }
        for i in 4..7 {
            self.p[(i, i)] = self.p[(i, i)].clamp(MIN_VAR, MAX_BIAS_VAR);
        }

        // 非対角成分のクランプ（相関係数の制限）
        for i in 0..7 {
            for j in (i + 1)..7 {
                let max_val = libm::sqrtf(self.p[(i, i)] * self.p[(j, j)]);
                let clamped = self.p[(i, j)].clamp(-max_val, max_val);
                self.p[(i, j)] = clamped;
                self.p[(j, i)] = clamped;
            }
        }
    }

    /// 現在のYaw角を0としてリセット
    pub fn reset_yaw(&mut self) {
        let (_, _, yaw) = self.get_euler();
        self.yaw_offset = yaw;
        self.continuous[2].reset();
    }

    fn build_current_state(&mut self, accel_valid: bool) -> AttitudeState {
        let q = Quaternion::new(self.x[0], self.x[1], self.x[2], self.x[3]);
        let q_unit = UnitQuaternion::from_quaternion(q);
        let (roll, pitch, yaw) = q_unit.euler_angles();
        let yaw_corrected = yaw - self.yaw_offset;  // オフセット適用

        AttitudeState {
            roll,
            pitch,
            yaw: yaw_corrected,
            continuous_roll: self.continuous[0].update(roll),
            continuous_pitch: self.continuous[1].update(pitch),
            continuous_yaw: self.continuous[2].update(yaw_corrected),
            roll_rate: self.last_gyro[0] - self.x[4],
            pitch_rate: self.last_gyro[1] - self.x[5],
            yaw_rate: self.last_gyro[2] - self.x[6],
            gyro_bias_x: self.x[4],
            gyro_bias_y: self.x[5],
            gyro_bias_z: self.x[6],
            quaternion: q_unit,
            quat_variance: self.p[(0, 0)],
            bias_variance: self.p[(4, 4)],
            accel_valid,
        }
    }

    pub fn update_x_up(
        &mut self,
        ax: f32,
        ay: f32,
        az: f32,
        gx: f32,
        gy: f32,
        gz: f32,
    ) -> AttitudeState {
        self.update(-az, ay, ax, -gz, gy, gx)
    }

    /// X軸が上向きの座標系で磁力計更新
    pub fn update_mag_x_up(&mut self, mx: f32, my: f32, mz: f32) {
        self.update_mag(-mz, my, mx)
    }

    #[inline(always)]
    pub fn get_quaternion(&self) -> UnitQuaternion<f32> {
        let q = Quaternion::new(self.x[0], self.x[1], self.x[2], self.x[3]);
        UnitQuaternion::from_quaternion(q)
    }

    #[inline(always)]
    pub fn get_euler(&self) -> (f32, f32, f32) {
        self.get_quaternion().euler_angles()
    }

    #[inline(always)]
    pub fn get_gyro_bias(&self) -> (f32, f32, f32) {
        (self.x[4], self.x[5], self.x[6])
    }

    #[inline(always)]
    pub fn reset_continuous(&mut self) {
        for c in &mut self.continuous {
            c.reset();
        }
    }

    pub fn reset(&mut self) {
        self.x = StateVector::from_row_slice(&[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        self.initialized = false;
        self.mag_ref = None;
        self.yaw_offset = 0.0;

        let pq = self.config.initial_quat_variance;
        let pb = self.config.initial_bias_variance;
        self.p = CovMatrix::from_diagonal(&SVector::<f32, 7>::from_row_slice(&[
            pq, pq, pq, pq, pb, pb, pb,
        ]));

        self.reset_continuous();
    }

    #[inline(always)]
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }
}

#[inline(always)]
pub fn rad_to_degree((r, p, y): (f32, f32, f32)) -> (f32, f32, f32) {
    (r * RAD_TO_DEG, p * RAD_TO_DEG, y * RAD_TO_DEG)
}

#[inline(always)]
pub fn degree_to_rad((r, p, y): (f32, f32, f32)) -> (f32, f32, f32) {
    (r * DEG_TO_RAD, p * DEG_TO_RAD, y * DEG_TO_RAD)
}