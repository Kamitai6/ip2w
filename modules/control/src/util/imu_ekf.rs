//! Quaternion Extended Kalman Filter for IMU Attitude Estimation
//!
//! バイアス相関によるyawドリフト軽減版

#![no_std]

use core::f32::consts::PI;
use libm::{atan2f, asinf, sqrtf, cosf, sinf};

const DEG_TO_RAD: f32 = PI / 180.0;
const RAD_TO_DEG: f32 = 180.0 / PI;

/// EKF Configuration
#[derive(Debug, Clone, Copy)]
pub struct EkfConfig {
    pub dt: f32,
    pub gyro_noise: f32,
    pub gyro_bias_noise: f32,
    pub accel_noise: f32,
    pub initial_quat_variance: f32,
    pub initial_bias_variance: f32,
    pub accel_magnitude_min: f32,
    pub accel_magnitude_max: f32,
    
    // ===== バイアス相関パラメータ =====
    /// バイアス相関を有効にするか
    pub bias_correlation_enabled: bool,
    
    /// X/Y軸バイアス変化からZ軸への伝搬係数 (0.0-1.0)
    pub bias_correlation_factor: f32,
}

impl Default for EkfConfig {
    fn default() -> Self {
        Self {
            dt: 1.0 / 400.0,
            gyro_noise: 0.01,
            gyro_bias_noise: 0.0001,
            accel_noise: 0.1,
            initial_quat_variance: 0.1,
            initial_bias_variance: 0.01,
            accel_magnitude_min: 0.8,
            accel_magnitude_max: 1.2,
            
            // バイアス相関のデフォルト値
            bias_correlation_enabled: true,
            bias_correlation_factor: 0.3,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Default for Quaternion {
    fn default() -> Self { Self::identity() }
}

impl Quaternion {
    #[inline(always)]
    pub const fn identity() -> Self {
        Self { w: 1.0, x: 0.0, y: 0.0, z: 0.0 }
    }

    #[inline(always)]
    pub fn normalize(&mut self) {
        let norm = sqrtf(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z);
        if norm > 1e-10 {
            let inv = 1.0 / norm;
            self.w *= inv;
            self.x *= inv;
            self.y *= inv;
            self.z *= inv;
        }
    }

    #[inline]
    pub fn to_euler(&self) -> (f32, f32, f32) {
        let sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z);
        let cosr_cosp = 1.0 - 2.0 * (self.x * self.x + self.y * self.y);
        let roll = atan2f(sinr_cosp, cosr_cosp);

        let sinp = (2.0 * (self.w * self.y - self.z * self.x)).clamp(-1.0, 1.0);
        let pitch = asinf(sinp);

        let siny_cosp = 2.0 * (self.w * self.z + self.x * self.y);
        let cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z);
        let yaw = atan2f(siny_cosp, cosy_cosp);

        (roll, pitch, yaw)
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct AttitudeState {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub roll_rate: f32,
    pub pitch_rate: f32,
    pub yaw_rate: f32,
    pub gyro_bias_x: f32,
    pub gyro_bias_y: f32,
    pub gyro_bias_z: f32,
    pub quaternion: Quaternion,
    pub quat_variance: f32,
    pub bias_variance: f32,
    pub accel_valid: bool,
}

#[inline(always)]
fn sym_idx(i: usize, j: usize) -> usize {
    if i <= j {
        i * (13 - i) / 2 + j
    } else {
        j * (13 - j) / 2 + i
    }
}

pub struct ImuEkf {
    q: Quaternion,
    bias: [f32; 3],
    p: [f32; 28],
    q_gyro: f32,
    q_bias: f32,
    r_accel: f32,
    config: EkfConfig,
    last_gyro: [f32; 3],
    initialized: bool,
}

impl ImuEkf {
    pub fn new(config: EkfConfig) -> Self {
        let dt = config.dt;
        let q_gyro = config.gyro_noise * config.gyro_noise * dt;
        let q_bias = config.gyro_bias_noise * config.gyro_bias_noise * dt;
        let r_accel = config.accel_noise * config.accel_noise;

        let pq = config.initial_quat_variance;
        let pb = config.initial_bias_variance;
        
        let mut p = [0.0f32; 28];
        p[sym_idx(0, 0)] = pq;
        p[sym_idx(1, 1)] = pq;
        p[sym_idx(2, 2)] = pq;
        p[sym_idx(3, 3)] = pq;
        p[sym_idx(4, 4)] = pb;
        p[sym_idx(5, 5)] = pb;
        p[sym_idx(6, 6)] = pb;

        Self {
            q: Quaternion::identity(),
            bias: [0.0; 3],
            p,
            q_gyro,
            q_bias,
            r_accel,
            config,
            last_gyro: [0.0; 3],
            initialized: false,
        }
    }

    pub fn update(&mut self, ax: f32, ay: f32, az: f32, gx: f32, gy: f32, gz: f32) -> AttitudeState {
        self.last_gyro = [gx, gy, gz];

        let wx = gx - self.bias[0];
        let wy = gy - self.bias[1];
        let wz = gz - self.bias[2];

        let accel_norm_sq = ax * ax + ay * ay + az * az;
        let accel_valid = accel_norm_sq >= self.config.accel_magnitude_min * self.config.accel_magnitude_min
            && accel_norm_sq <= self.config.accel_magnitude_max * self.config.accel_magnitude_max;

        if !self.initialized && accel_valid {
            self.init_from_accel(ax, ay, az);
            self.initialized = true;
        }

        // ===== PREDICTION STEP =====
        self.predict(wx, wy, wz);

        // ===== UPDATE STEP =====
        if accel_valid && accel_norm_sq > 1e-10 {
            let norm_inv = 1.0 / sqrtf(accel_norm_sq);
            
            // バイアス変化量を追跡
            let bias_before = self.bias;
            
            self.update_accel(ax * norm_inv, ay * norm_inv, az * norm_inv);
            
            // ===== バイアス相関による Z軸バイアス補正 =====
            if self.config.bias_correlation_enabled {
                let dx = self.bias[0] - bias_before[0];
                let dy = self.bias[1] - bias_before[1];
                self.apply_bias_correlation(dx, dy);
            }
        }

        self.q.normalize();
        self.ensure_positive_definite();

        self.build_current_state(accel_valid)
    }

    /// X/Y軸のバイアス変化をZ軸にも伝搬
    /// X/Y軸のバイアス変化をZ軸にも伝搬
    fn apply_bias_correlation(&mut self, dx: f32, dy: f32) {
        let factor = self.config.bias_correlation_factor;
        let avg_delta = (dx + dy) / 2.0;
        self.bias[2] += avg_delta * factor;
        
        // Z軸バイアスの不確実性を下げる
        // X/Y軸から間接的に観測されたと見なす
        self.p[sym_idx(6, 6)] *= 1.0 - factor * 0.1;
    }

    fn build_current_state(&self, accel_valid: bool) -> AttitudeState {
        let (roll, pitch, yaw) = self.q.to_euler();
        AttitudeState {
            roll,
            pitch,
            yaw,
            roll_rate: self.last_gyro[0] - self.bias[0],
            pitch_rate: self.last_gyro[1] - self.bias[1],
            yaw_rate: self.last_gyro[2] - self.bias[2],
            gyro_bias_x: self.bias[0],
            gyro_bias_y: self.bias[1],
            gyro_bias_z: self.bias[2],
            quaternion: self.q,
            quat_variance: self.p[0],
            bias_variance: self.p[sym_idx(4, 4)],
            accel_valid,
        }
    }

    #[inline]
    pub fn update_deg(
        &mut self,
        ax: f32, ay: f32, az: f32,
        gx_deg: f32, gy_deg: f32, gz_deg: f32,
    ) -> AttitudeState {
        self.update(
            ax, ay, az,
            gx_deg * DEG_TO_RAD,
            gy_deg * DEG_TO_RAD,
            gz_deg * DEG_TO_RAD,
        )
    }

    fn predict(&mut self, wx: f32, wy: f32, wz: f32) {
        let dt = self.config.dt;
        let dt_half = dt * 0.5;

        let q0 = self.q.w;
        let q1 = self.q.x;
        let q2 = self.q.y;
        let q3 = self.q.z;

        self.q.w += dt_half * (-q1 * wx - q2 * wy - q3 * wz);
        self.q.x += dt_half * (q0 * wx + q2 * wz - q3 * wy);
        self.q.y += dt_half * (q0 * wy - q1 * wz + q3 * wx);
        self.q.z += dt_half * (q0 * wz + q1 * wy - q2 * wx);

        let f01 = -dt_half * wx; let f02 = -dt_half * wy; let f03 = -dt_half * wz;
        let f10 =  dt_half * wx; let f12 =  dt_half * wz; let f13 = -dt_half * wy;
        let f20 =  dt_half * wy; let f21 = -dt_half * wz; let f23 =  dt_half * wx;
        let f30 =  dt_half * wz; let f31 =  dt_half * wy; let f32 = -dt_half * wx;

        let f04 =  dt_half * q1; let f05 =  dt_half * q2; let f06 =  dt_half * q3;
        let f14 = -dt_half * q0; let f15 = -dt_half * q3; let f16 =  dt_half * q2;
        let f24 =  dt_half * q3; let f25 = -dt_half * q0; let f26 = -dt_half * q1;
        let f34 = -dt_half * q2; let f35 =  dt_half * q1; let f36 = -dt_half * q0;

        let p00 = self.p[sym_idx(0,0)]; let p01 = self.p[sym_idx(0,1)]; let p02 = self.p[sym_idx(0,2)];
        let p03 = self.p[sym_idx(0,3)]; let p04 = self.p[sym_idx(0,4)]; let p05 = self.p[sym_idx(0,5)];
        let p06 = self.p[sym_idx(0,6)];
        let p11 = self.p[sym_idx(1,1)]; let p12 = self.p[sym_idx(1,2)]; let p13 = self.p[sym_idx(1,3)];
        let p14 = self.p[sym_idx(1,4)]; let p15 = self.p[sym_idx(1,5)]; let p16 = self.p[sym_idx(1,6)];
        let p22 = self.p[sym_idx(2,2)]; let p23 = self.p[sym_idx(2,3)]; let p24 = self.p[sym_idx(2,4)];
        let p25 = self.p[sym_idx(2,5)]; let p26 = self.p[sym_idx(2,6)];
        let p33 = self.p[sym_idx(3,3)]; let p34 = self.p[sym_idx(3,4)]; let p35 = self.p[sym_idx(3,5)];
        let p36 = self.p[sym_idx(3,6)];
        let p44 = self.p[sym_idx(4,4)]; let p45 = self.p[sym_idx(4,5)]; let p46 = self.p[sym_idx(4,6)];
        let p55 = self.p[sym_idx(5,5)]; let p56 = self.p[sym_idx(5,6)];
        let p66 = self.p[sym_idx(6,6)];

        let fp00 = p00 + f01*p01 + f02*p02 + f03*p03 + f04*p04 + f05*p05 + f06*p06;
        let fp01 = p01 + f01*p11 + f02*p12 + f03*p13 + f04*p14 + f05*p15 + f06*p16;
        let fp02 = p02 + f01*p12 + f02*p22 + f03*p23 + f04*p24 + f05*p25 + f06*p26;
        let fp03 = p03 + f01*p13 + f02*p23 + f03*p33 + f04*p34 + f05*p35 + f06*p36;
        let fp04 = p04 + f01*p14 + f02*p24 + f03*p34 + f04*p44 + f05*p45 + f06*p46;
        let fp05 = p05 + f01*p15 + f02*p25 + f03*p35 + f04*p45 + f05*p55 + f06*p56;
        let fp06 = p06 + f01*p16 + f02*p26 + f03*p36 + f04*p46 + f05*p56 + f06*p66;

        let fp10 = f10*p00 + p01 + f12*p02 + f13*p03 + f14*p04 + f15*p05 + f16*p06;
        let fp11 = f10*p01 + p11 + f12*p12 + f13*p13 + f14*p14 + f15*p15 + f16*p16;
        let fp12 = f10*p02 + p12 + f12*p22 + f13*p23 + f14*p24 + f15*p25 + f16*p26;
        let fp13 = f10*p03 + p13 + f12*p23 + f13*p33 + f14*p34 + f15*p35 + f16*p36;
        let fp14 = f10*p04 + p14 + f12*p24 + f13*p34 + f14*p44 + f15*p45 + f16*p46;
        let fp15 = f10*p05 + p15 + f12*p25 + f13*p35 + f14*p45 + f15*p55 + f16*p56;
        let fp16 = f10*p06 + p16 + f12*p26 + f13*p36 + f14*p46 + f15*p56 + f16*p66;

        let fp20 = f20*p00 + f21*p01 + p02 + f23*p03 + f24*p04 + f25*p05 + f26*p06;
        let fp21 = f20*p01 + f21*p11 + p12 + f23*p13 + f24*p14 + f25*p15 + f26*p16;
        let fp22 = f20*p02 + f21*p12 + p22 + f23*p23 + f24*p24 + f25*p25 + f26*p26;
        let fp23 = f20*p03 + f21*p13 + p23 + f23*p33 + f24*p34 + f25*p35 + f26*p36;
        let fp24 = f20*p04 + f21*p14 + p24 + f23*p34 + f24*p44 + f25*p45 + f26*p46;
        let fp25 = f20*p05 + f21*p15 + p25 + f23*p35 + f24*p45 + f25*p55 + f26*p56;
        let fp26 = f20*p06 + f21*p16 + p26 + f23*p36 + f24*p46 + f25*p56 + f26*p66;

        let fp30 = f30*p00 + f31*p01 + f32*p02 + p03 + f34*p04 + f35*p05 + f36*p06;
        let fp31 = f30*p01 + f31*p11 + f32*p12 + p13 + f34*p14 + f35*p15 + f36*p16;
        let fp32 = f30*p02 + f31*p12 + f32*p22 + p23 + f34*p24 + f35*p25 + f36*p26;
        let fp33 = f30*p03 + f31*p13 + f32*p23 + p33 + f34*p34 + f35*p35 + f36*p36;
        let fp34 = f30*p04 + f31*p14 + f32*p24 + p34 + f34*p44 + f35*p45 + f36*p46;
        let fp35 = f30*p05 + f31*p15 + f32*p25 + p35 + f34*p45 + f35*p55 + f36*p56;
        let fp36 = f30*p06 + f31*p16 + f32*p26 + p36 + f34*p46 + f35*p56 + f36*p66;

        let fp44 = p44; let fp45 = p45; let fp46 = p46;
        let fp55 = p55; let fp56 = p56;
        let fp66 = p66;

        let q_q = self.q_gyro;
        let q_b = self.q_bias;

        self.p[sym_idx(0,0)] = fp00 + f01*fp01 + f02*fp02 + f03*fp03 + f04*fp04 + f05*fp05 + f06*fp06 + q_q;
        self.p[sym_idx(0,1)] = fp01 + f01*fp11 + f02*fp12 + f03*fp13 + f04*fp14 + f05*fp15 + f06*fp16;
        self.p[sym_idx(0,2)] = fp02 + f01*fp12 + f02*fp22 + f03*fp23 + f04*fp24 + f05*fp25 + f06*fp26;
        self.p[sym_idx(0,3)] = fp03 + f01*fp13 + f02*fp23 + f03*fp33 + f04*fp34 + f05*fp35 + f06*fp36;
        self.p[sym_idx(0,4)] = fp04 + f01*fp14 + f02*fp24 + f03*fp34 + f04*fp44 + f05*fp45 + f06*fp46;
        self.p[sym_idx(0,5)] = fp05 + f01*fp15 + f02*fp25 + f03*fp35 + f04*fp45 + f05*fp55 + f06*fp56;
        self.p[sym_idx(0,6)] = fp06 + f01*fp16 + f02*fp26 + f03*fp36 + f04*fp46 + f05*fp56 + f06*fp66;

        self.p[sym_idx(1,1)] = f10*fp01 + fp11 + f12*fp12 + f13*fp13 + f14*fp14 + f15*fp15 + f16*fp16 + q_q;
        self.p[sym_idx(1,2)] = f10*fp02 + fp12 + f12*fp22 + f13*fp23 + f14*fp24 + f15*fp25 + f16*fp26;
        self.p[sym_idx(1,3)] = f10*fp03 + fp13 + f12*fp23 + f13*fp33 + f14*fp34 + f15*fp35 + f16*fp36;
        self.p[sym_idx(1,4)] = f10*fp04 + fp14 + f12*fp24 + f13*fp34 + f14*fp44 + f15*fp45 + f16*fp46;
        self.p[sym_idx(1,5)] = f10*fp05 + fp15 + f12*fp25 + f13*fp35 + f14*fp45 + f15*fp55 + f16*fp56;
        self.p[sym_idx(1,6)] = f10*fp06 + fp16 + f12*fp26 + f13*fp36 + f14*fp46 + f15*fp56 + f16*fp66;

        self.p[sym_idx(2,2)] = f20*fp02 + f21*fp12 + fp22 + f23*fp23 + f24*fp24 + f25*fp25 + f26*fp26 + q_q;
        self.p[sym_idx(2,3)] = f20*fp03 + f21*fp13 + fp23 + f23*fp33 + f24*fp34 + f25*fp35 + f26*fp36;
        self.p[sym_idx(2,4)] = f20*fp04 + f21*fp14 + fp24 + f23*fp34 + f24*fp44 + f25*fp45 + f26*fp46;
        self.p[sym_idx(2,5)] = f20*fp05 + f21*fp15 + fp25 + f23*fp35 + f24*fp45 + f25*fp55 + f26*fp56;
        self.p[sym_idx(2,6)] = f20*fp06 + f21*fp16 + fp26 + f23*fp36 + f24*fp46 + f25*fp56 + f26*fp66;

        self.p[sym_idx(3,3)] = f30*fp03 + f31*fp13 + f32*fp23 + fp33 + f34*fp34 + f35*fp35 + f36*fp36 + q_q;
        self.p[sym_idx(3,4)] = f30*fp04 + f31*fp14 + f32*fp24 + fp34 + f34*fp44 + f35*fp45 + f36*fp46;
        self.p[sym_idx(3,5)] = f30*fp05 + f31*fp15 + f32*fp25 + fp35 + f34*fp45 + f35*fp55 + f36*fp56;
        self.p[sym_idx(3,6)] = f30*fp06 + f31*fp16 + f32*fp26 + fp36 + f34*fp46 + f35*fp56 + f36*fp66;

        self.p[sym_idx(4,4)] = fp44 + q_b;
        self.p[sym_idx(4,5)] = fp45;
        self.p[sym_idx(4,6)] = fp46;
        self.p[sym_idx(5,5)] = fp55 + q_b;
        self.p[sym_idx(5,6)] = fp56;
        self.p[sym_idx(6,6)] = fp66 + q_b;

        self.ensure_positive_definite();
    }

    fn update_accel(&mut self, ax: f32, ay: f32, az: f32) {
        let q0 = self.q.w;
        let q1 = self.q.x;
        let q2 = self.q.y;
        let q3 = self.q.z;

        let hx = 2.0 * (q1 * q3 - q0 * q2);
        let hy = 2.0 * (q0 * q1 + q2 * q3);
        let hz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        let y0 = ax - hx;
        let y1 = ay - hy;
        let y2 = az - hz;

        let h00 = -2.0 * q2; let h01 =  2.0 * q3; let h02 = -2.0 * q0; let h03 =  2.0 * q1;
        let h10 =  2.0 * q1; let h11 =  2.0 * q0; let h12 =  2.0 * q3; let h13 =  2.0 * q2;
        let h20 =  2.0 * q0; let h21 = -2.0 * q1; let h22 = -2.0 * q2; let h23 =  2.0 * q3;

        let p00 = self.p[sym_idx(0,0)]; let p01 = self.p[sym_idx(0,1)]; let p02 = self.p[sym_idx(0,2)];
        let p03 = self.p[sym_idx(0,3)]; let p04 = self.p[sym_idx(0,4)]; let p05 = self.p[sym_idx(0,5)];
        let p06 = self.p[sym_idx(0,6)];
        let p11 = self.p[sym_idx(1,1)]; let p12 = self.p[sym_idx(1,2)]; let p13 = self.p[sym_idx(1,3)];
        let p14 = self.p[sym_idx(1,4)]; let p15 = self.p[sym_idx(1,5)]; let p16 = self.p[sym_idx(1,6)];
        let p22 = self.p[sym_idx(2,2)]; let p23 = self.p[sym_idx(2,3)]; let p24 = self.p[sym_idx(2,4)];
        let p25 = self.p[sym_idx(2,5)]; let p26 = self.p[sym_idx(2,6)];
        let p33 = self.p[sym_idx(3,3)]; let p34 = self.p[sym_idx(3,4)]; let p35 = self.p[sym_idx(3,5)];
        let p36 = self.p[sym_idx(3,6)];
        let p44 = self.p[sym_idx(4,4)]; let p45 = self.p[sym_idx(4,5)]; let p46 = self.p[sym_idx(4,6)];
        let p55 = self.p[sym_idx(5,5)]; let p56 = self.p[sym_idx(5,6)];
        let p66 = self.p[sym_idx(6,6)];

        let pht00 = p00*h00 + p01*h01 + p02*h02 + p03*h03;
        let pht01 = p00*h10 + p01*h11 + p02*h12 + p03*h13;
        let pht02 = p00*h20 + p01*h21 + p02*h22 + p03*h23;
        let pht10 = p01*h00 + p11*h01 + p12*h02 + p13*h03;
        let pht11 = p01*h10 + p11*h11 + p12*h12 + p13*h13;
        let pht12 = p01*h20 + p11*h21 + p12*h22 + p13*h23;
        let pht20 = p02*h00 + p12*h01 + p22*h02 + p23*h03;
        let pht21 = p02*h10 + p12*h11 + p22*h12 + p23*h13;
        let pht22 = p02*h20 + p12*h21 + p22*h22 + p23*h23;
        let pht30 = p03*h00 + p13*h01 + p23*h02 + p33*h03;
        let pht31 = p03*h10 + p13*h11 + p23*h12 + p33*h13;
        let pht32 = p03*h20 + p13*h21 + p23*h22 + p33*h23;
        let pht40 = p04*h00 + p14*h01 + p24*h02 + p34*h03;
        let pht41 = p04*h10 + p14*h11 + p24*h12 + p34*h13;
        let pht42 = p04*h20 + p14*h21 + p24*h22 + p34*h23;
        let pht50 = p05*h00 + p15*h01 + p25*h02 + p35*h03;
        let pht51 = p05*h10 + p15*h11 + p25*h12 + p35*h13;
        let pht52 = p05*h20 + p15*h21 + p25*h22 + p35*h23;
        let pht60 = p06*h00 + p16*h01 + p26*h02 + p36*h03;
        let pht61 = p06*h10 + p16*h11 + p26*h12 + p36*h13;
        let pht62 = p06*h20 + p16*h21 + p26*h22 + p36*h23;

        let r = self.r_accel;
        let s00 = h00*pht00 + h01*pht10 + h02*pht20 + h03*pht30 + r;
        let s01 = h00*pht01 + h01*pht11 + h02*pht21 + h03*pht31;
        let s02 = h00*pht02 + h01*pht12 + h02*pht22 + h03*pht32;
        let s11 = h10*pht01 + h11*pht11 + h12*pht21 + h13*pht31 + r;
        let s12 = h10*pht02 + h11*pht12 + h12*pht22 + h13*pht32;
        let s22 = h20*pht02 + h21*pht12 + h22*pht22 + h23*pht32 + r;

        let (si00, si01, si02, si11, si12, si22) = Self::invert_sym3(s00, s01, s02, s11, s12, s22);

        let k00 = pht00*si00 + pht01*si01 + pht02*si02;
        let k01 = pht00*si01 + pht01*si11 + pht02*si12;
        let k02 = pht00*si02 + pht01*si12 + pht02*si22;
        let k10 = pht10*si00 + pht11*si01 + pht12*si02;
        let k11 = pht10*si01 + pht11*si11 + pht12*si12;
        let k12 = pht10*si02 + pht11*si12 + pht12*si22;
        let k20 = pht20*si00 + pht21*si01 + pht22*si02;
        let k21 = pht20*si01 + pht21*si11 + pht22*si12;
        let k22 = pht20*si02 + pht21*si12 + pht22*si22;
        let k30 = pht30*si00 + pht31*si01 + pht32*si02;
        let k31 = pht30*si01 + pht31*si11 + pht32*si12;
        let k32 = pht30*si02 + pht31*si12 + pht32*si22;
        let k40 = pht40*si00 + pht41*si01 + pht42*si02;
        let k41 = pht40*si01 + pht41*si11 + pht42*si12;
        let k42 = pht40*si02 + pht41*si12 + pht42*si22;
        let k50 = pht50*si00 + pht51*si01 + pht52*si02;
        let k51 = pht50*si01 + pht51*si11 + pht52*si12;
        let k52 = pht50*si02 + pht51*si12 + pht52*si22;
        let k60 = pht60*si00 + pht61*si01 + pht62*si02;
        let k61 = pht60*si01 + pht61*si11 + pht62*si12;
        let k62 = pht60*si02 + pht61*si12 + pht62*si22;

        let dq0 = k00*y0 + k01*y1 + k02*y2;
        let dq1 = k10*y0 + k11*y1 + k12*y2;
        let dq2 = k20*y0 + k21*y1 + k22*y2;
        let dq3 = k30*y0 + k31*y1 + k32*y2;
        let db0 = k40*y0 + k41*y1 + k42*y2;
        let db1 = k50*y0 + k51*y1 + k52*y2;
        let db2 = k60*y0 + k61*y1 + k62*y2;

        self.q.w += dq0;
        self.q.x += dq1;
        self.q.y += dq2;
        self.q.z += dq3;
        self.bias[0] += db0;
        self.bias[1] += db1;
        self.bias[2] += db2;

        // Joseph形式の共分散更新
        let m00 = 1.0 - (k00*h00 + k01*h10 + k02*h20);
        let m01 = -(k00*h01 + k01*h11 + k02*h21);
        let m02 = -(k00*h02 + k01*h12 + k02*h22);
        let m03 = -(k00*h03 + k01*h13 + k02*h23);
        let m10 = -(k10*h00 + k11*h10 + k12*h20);
        let m11 = 1.0 - (k10*h01 + k11*h11 + k12*h21);
        let m12 = -(k10*h02 + k11*h12 + k12*h22);
        let m13 = -(k10*h03 + k11*h13 + k12*h23);
        let m20 = -(k20*h00 + k21*h10 + k22*h20);
        let m21 = -(k20*h01 + k21*h11 + k22*h21);
        let m22 = 1.0 - (k20*h02 + k21*h12 + k22*h22);
        let m23 = -(k20*h03 + k21*h13 + k22*h23);
        let m30 = -(k30*h00 + k31*h10 + k32*h20);
        let m31 = -(k30*h01 + k31*h11 + k32*h21);
        let m32 = -(k30*h02 + k31*h12 + k32*h22);
        let m33 = 1.0 - (k30*h03 + k31*h13 + k32*h23);
        let m40 = -(k40*h00 + k41*h10 + k42*h20);
        let m41 = -(k40*h01 + k41*h11 + k42*h21);
        let m42 = -(k40*h02 + k41*h12 + k42*h22);
        let m43 = -(k40*h03 + k41*h13 + k42*h23);
        let m50 = -(k50*h00 + k51*h10 + k52*h20);
        let m51 = -(k50*h01 + k51*h11 + k52*h21);
        let m52 = -(k50*h02 + k51*h12 + k52*h22);
        let m53 = -(k50*h03 + k51*h13 + k52*h23);
        let m60 = -(k60*h00 + k61*h10 + k62*h20);
        let m61 = -(k60*h01 + k61*h11 + k62*h21);
        let m62 = -(k60*h02 + k61*h12 + k62*h22);
        let m63 = -(k60*h03 + k61*h13 + k62*h23);

        let mp00 = m00*p00 + m01*p01 + m02*p02 + m03*p03;
        let mp01 = m00*p01 + m01*p11 + m02*p12 + m03*p13;
        let mp02 = m00*p02 + m01*p12 + m02*p22 + m03*p23;
        let mp03 = m00*p03 + m01*p13 + m02*p23 + m03*p33;
        let mp04 = m00*p04 + m01*p14 + m02*p24 + m03*p34;
        let mp05 = m00*p05 + m01*p15 + m02*p25 + m03*p35;
        let mp06 = m00*p06 + m01*p16 + m02*p26 + m03*p36;

        let mp10 = m10*p00 + m11*p01 + m12*p02 + m13*p03;
        let mp11 = m10*p01 + m11*p11 + m12*p12 + m13*p13;
        let mp12 = m10*p02 + m11*p12 + m12*p22 + m13*p23;
        let mp13 = m10*p03 + m11*p13 + m12*p23 + m13*p33;
        let mp14 = m10*p04 + m11*p14 + m12*p24 + m13*p34;
        let mp15 = m10*p05 + m11*p15 + m12*p25 + m13*p35;
        let mp16 = m10*p06 + m11*p16 + m12*p26 + m13*p36;

        let mp20 = m20*p00 + m21*p01 + m22*p02 + m23*p03;
        let mp21 = m20*p01 + m21*p11 + m22*p12 + m23*p13;
        let mp22 = m20*p02 + m21*p12 + m22*p22 + m23*p23;
        let mp23 = m20*p03 + m21*p13 + m22*p23 + m23*p33;
        let mp24 = m20*p04 + m21*p14 + m22*p24 + m23*p34;
        let mp25 = m20*p05 + m21*p15 + m22*p25 + m23*p35;
        let mp26 = m20*p06 + m21*p16 + m22*p26 + m23*p36;

        let mp30 = m30*p00 + m31*p01 + m32*p02 + m33*p03;
        let mp31 = m30*p01 + m31*p11 + m32*p12 + m33*p13;
        let mp32 = m30*p02 + m31*p12 + m32*p22 + m33*p23;
        let mp33 = m30*p03 + m31*p13 + m32*p23 + m33*p33;
        let mp34 = m30*p04 + m31*p14 + m32*p24 + m33*p34;
        let mp35 = m30*p05 + m31*p15 + m32*p25 + m33*p35;
        let mp36 = m30*p06 + m31*p16 + m32*p26 + m33*p36;

        let mp40 = m40*p00 + m41*p01 + m42*p02 + m43*p03 + p04;
        let mp41 = m40*p01 + m41*p11 + m42*p12 + m43*p13 + p14;
        let mp42 = m40*p02 + m41*p12 + m42*p22 + m43*p23 + p24;
        let mp43 = m40*p03 + m41*p13 + m42*p23 + m43*p33 + p34;
        let mp44 = m40*p04 + m41*p14 + m42*p24 + m43*p34 + p44;
        let mp45 = m40*p05 + m41*p15 + m42*p25 + m43*p35 + p45;
        let mp46 = m40*p06 + m41*p16 + m42*p26 + m43*p36 + p46;

        let mp50 = m50*p00 + m51*p01 + m52*p02 + m53*p03 + p05;
        let mp51 = m50*p01 + m51*p11 + m52*p12 + m53*p13 + p15;
        let mp52 = m50*p02 + m51*p12 + m52*p22 + m53*p23 + p25;
        let mp53 = m50*p03 + m51*p13 + m52*p23 + m53*p33 + p35;
        let mp54 = m50*p04 + m51*p14 + m52*p24 + m53*p34 + p45;
        let mp55 = m50*p05 + m51*p15 + m52*p25 + m53*p35 + p55;
        let mp56 = m50*p06 + m51*p16 + m52*p26 + m53*p36 + p56;

        let mp60 = m60*p00 + m61*p01 + m62*p02 + m63*p03 + p06;
        let mp61 = m60*p01 + m61*p11 + m62*p12 + m63*p13 + p16;
        let mp62 = m60*p02 + m61*p12 + m62*p22 + m63*p23 + p26;
        let mp63 = m60*p03 + m61*p13 + m62*p23 + m63*p33 + p36;
        let mp64 = m60*p04 + m61*p14 + m62*p24 + m63*p34 + p46;
        let mp65 = m60*p05 + m61*p15 + m62*p25 + m63*p35 + p56;
        let mp66 = m60*p06 + m61*p16 + m62*p26 + m63*p36 + p66;

        let krk0 = r * (k00*k00 + k01*k01 + k02*k02);
        let krk1 = r * (k10*k10 + k11*k11 + k12*k12);
        let krk2 = r * (k20*k20 + k21*k21 + k22*k22);
        let krk3 = r * (k30*k30 + k31*k31 + k32*k32);
        let krk4 = r * (k40*k40 + k41*k41 + k42*k42);
        let krk5 = r * (k50*k50 + k51*k51 + k52*k52);
        let krk6 = r * (k60*k60 + k61*k61 + k62*k62);

        self.p[sym_idx(0,0)] = mp00*m00 + mp01*m01 + mp02*m02 + mp03*m03 + krk0;
        self.p[sym_idx(0,1)] = mp00*m10 + mp01*m11 + mp02*m12 + mp03*m13;
        self.p[sym_idx(0,2)] = mp00*m20 + mp01*m21 + mp02*m22 + mp03*m23;
        self.p[sym_idx(0,3)] = mp00*m30 + mp01*m31 + mp02*m32 + mp03*m33;
        self.p[sym_idx(0,4)] = mp00*m40 + mp01*m41 + mp02*m42 + mp03*m43 + mp04;
        self.p[sym_idx(0,5)] = mp00*m50 + mp01*m51 + mp02*m52 + mp03*m53 + mp05;
        self.p[sym_idx(0,6)] = mp00*m60 + mp01*m61 + mp02*m62 + mp03*m63 + mp06;
        self.p[sym_idx(1,1)] = mp10*m10 + mp11*m11 + mp12*m12 + mp13*m13 + krk1;
        self.p[sym_idx(1,2)] = mp10*m20 + mp11*m21 + mp12*m22 + mp13*m23;
        self.p[sym_idx(1,3)] = mp10*m30 + mp11*m31 + mp12*m32 + mp13*m33;
        self.p[sym_idx(1,4)] = mp10*m40 + mp11*m41 + mp12*m42 + mp13*m43 + mp14;
        self.p[sym_idx(1,5)] = mp10*m50 + mp11*m51 + mp12*m52 + mp13*m53 + mp15;
        self.p[sym_idx(1,6)] = mp10*m60 + mp11*m61 + mp12*m62 + mp13*m63 + mp16;
        self.p[sym_idx(2,2)] = mp20*m20 + mp21*m21 + mp22*m22 + mp23*m23 + krk2;
        self.p[sym_idx(2,3)] = mp20*m30 + mp21*m31 + mp22*m32 + mp23*m33;
        self.p[sym_idx(2,4)] = mp20*m40 + mp21*m41 + mp22*m42 + mp23*m43 + mp24;
        self.p[sym_idx(2,5)] = mp20*m50 + mp21*m51 + mp22*m52 + mp23*m53 + mp25;
        self.p[sym_idx(2,6)] = mp20*m60 + mp21*m61 + mp22*m62 + mp23*m63 + mp26;
        self.p[sym_idx(3,3)] = mp30*m30 + mp31*m31 + mp32*m32 + mp33*m33 + krk3;
        self.p[sym_idx(3,4)] = mp30*m40 + mp31*m41 + mp32*m42 + mp33*m43 + mp34;
        self.p[sym_idx(3,5)] = mp30*m50 + mp31*m51 + mp32*m52 + mp33*m53 + mp35;
        self.p[sym_idx(3,6)] = mp30*m60 + mp31*m61 + mp32*m62 + mp33*m63 + mp36;
        self.p[sym_idx(4,4)] = mp40*m40 + mp41*m41 + mp42*m42 + mp43*m43 + mp44 + krk4;
        self.p[sym_idx(4,5)] = mp40*m50 + mp41*m51 + mp42*m52 + mp43*m53 + mp45;
        self.p[sym_idx(4,6)] = mp40*m60 + mp41*m61 + mp42*m62 + mp43*m63 + mp46;
        self.p[sym_idx(5,5)] = mp50*m50 + mp51*m51 + mp52*m52 + mp53*m53 + mp55 + krk5;
        self.p[sym_idx(5,6)] = mp50*m60 + mp51*m61 + mp52*m62 + mp53*m63 + mp56;
        self.p[sym_idx(6,6)] = mp60*m60 + mp61*m61 + mp62*m62 + mp63*m63 + mp66 + krk6;
    }

    #[inline]
    fn invert_sym3(a00: f32, a01: f32, a02: f32, a11: f32, a12: f32, a22: f32) 
        -> (f32, f32, f32, f32, f32, f32) 
    {
        let det = a00 * (a11 * a22 - a12 * a12)
                - a01 * (a01 * a22 - a12 * a02)
                + a02 * (a01 * a12 - a11 * a02);

        let inv_det = 1.0 / det;
        (
            (a11 * a22 - a12 * a12) * inv_det,
            (a02 * a12 - a01 * a22) * inv_det,
            (a01 * a12 - a02 * a11) * inv_det,
            (a00 * a22 - a02 * a02) * inv_det,
            (a02 * a01 - a00 * a12) * inv_det,
            (a00 * a11 - a01 * a01) * inv_det,
        )
    }
    
    fn init_from_accel(&mut self, ax: f32, ay: f32, az: f32) {
        let norm = sqrtf(ax * ax + ay * ay + az * az);
        let ax = ax / norm;
        let ay = ay / norm;
        let az = az / norm;

        let roll = atan2f(ay, az);
        let pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

        let cr = cosf(roll * 0.5);
        let sr = sinf(roll * 0.5);
        let cp = cosf(pitch * 0.5);
        let sp = sinf(pitch * 0.5);

        self.q.w = cr * cp;
        self.q.x = sr * cp;
        self.q.y = cr * sp;
        self.q.z = -sr * sp;
        self.q.normalize();
    }

    fn ensure_positive_definite(&mut self) {
        const MIN_VAR: f32 = 1e-8;
        const MAX_VAR: f32 = 1.0;
        const MAX_BIAS_VAR: f32 = 0.1;

        for i in 0..4 {
            let idx = sym_idx(i, i);
            self.p[idx] = self.p[idx].clamp(MIN_VAR, MAX_VAR);
        }
        for i in 4..7 {
            let idx = sym_idx(i, i);
            self.p[idx] = self.p[idx].clamp(MIN_VAR, MAX_BIAS_VAR);
        }

        for i in 0..7 {
            for j in (i+1)..7 {
                let idx = sym_idx(i, j);
                let max_val = libm::sqrtf(self.p[sym_idx(i,i)] * self.p[sym_idx(j,j)]);
                self.p[idx] = self.p[idx].clamp(-max_val, max_val);
            }
        }
    }

    // ===== Public API =====
    #[inline(always)]
    pub fn get_quaternion(&self) -> Quaternion { self.q }

    #[inline(always)]
    pub fn get_euler(&self) -> (f32, f32, f32) { self.q.to_euler() }

    #[inline(always)]
    pub fn get_roll(&self) -> f32 { self.get_euler().0 }

    #[inline(always)]
    pub fn get_pitch(&self) -> f32 { self.get_euler().1 }

    #[inline(always)]
    pub fn get_yaw(&self) -> f32 { self.get_euler().2 }

    #[inline(always)]
    pub fn get_roll_deg(&self) -> f32 { self.get_roll() * RAD_TO_DEG }

    #[inline(always)]
    pub fn get_pitch_deg(&self) -> f32 { self.get_pitch() * RAD_TO_DEG }

    #[inline(always)]
    pub fn get_yaw_deg(&self) -> f32 { self.get_yaw() * RAD_TO_DEG }

    #[inline(always)]
    pub fn get_gyro_bias(&self) -> (f32, f32, f32) {
        (self.bias[0], self.bias[1], self.bias[2])
    }

    pub fn reset(&mut self) {
        self.q = Quaternion::identity();
        self.bias = [0.0; 3];
        self.initialized = false;
        
        let pq = self.config.initial_quat_variance;
        let pb = self.config.initial_bias_variance;
        for i in 0..28 { self.p[i] = 0.0; }
        self.p[sym_idx(0,0)] = pq;
        self.p[sym_idx(1,1)] = pq;
        self.p[sym_idx(2,2)] = pq;
        self.p[sym_idx(3,3)] = pq;
        self.p[sym_idx(4,4)] = pb;
        self.p[sym_idx(5,5)] = pb;
        self.p[sym_idx(6,6)] = pb;
    }

    pub fn reset_yaw(&mut self) {
        let (roll, pitch, _) = self.q.to_euler();
        let cr = cosf(roll * 0.5);
        let sr = sinf(roll * 0.5);
        let cp = cosf(pitch * 0.5);
        let sp = sinf(pitch * 0.5);
        self.q.w = cr * cp;
        self.q.x = sr * cp;
        self.q.y = cr * sp;
        self.q.z = -sr * sp;
        self.q.normalize();
    }
}