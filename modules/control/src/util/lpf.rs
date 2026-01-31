//! 1次IIRローパスフィルタ（Exponential Moving Average）
//!
//! y[n] = y[n-1] + α·(x[n] - y[n-1])
//! α = dt / (τ + dt),  τ = 1 / (2π·fc)
//!
//! 使用例:
//!   let mut lpf = Lpf::new(40.0, 1.0/500.0);  // fc=40Hz, 500Hzサンプリング
//!   let filtered = lpf.update(raw_value);

#![no_std]

use core::f32::consts::PI;

/// 1軸ローパスフィルタ
#[derive(Debug, Clone, Copy)]
pub struct Lpf {
    state: f32,
    alpha: f32,
    initialized: bool,
}

impl Lpf {
    /// カットオフ周波数 `fc` [Hz], サンプリング周期 `dt` [s] から生成
    ///
    /// fc=40Hz, dt=1/500 → α≈0.33
    /// fc=80Hz, dt=1/500 → α≈0.50
    /// fc=2Hz,  dt=1/500 → α≈0.025
    pub fn new(fc: f32, dt: f32) -> Self {
        let tau = 1.0 / (2.0 * PI * fc);
        let alpha = dt / (tau + dt);
        Self {
            state: 0.0,
            alpha,
            initialized: false,
        }
    }

    /// α を直接指定して生成
    pub fn from_alpha(alpha: f32) -> Self {
        Self {
            state: 0.0,
            alpha: alpha.clamp(0.0, 1.0),
            initialized: false,
        }
    }

    /// フィルタ更新。初回は入力値でそのまま初期化。
    #[inline]
    pub fn update(&mut self, x: f32) -> f32 {
        if !self.initialized {
            self.state = x;
            self.initialized = true;
        } else {
            self.state += self.alpha * (x - self.state);
        }
        self.state
    }

    #[inline(always)]
    pub fn get(&self) -> f32 {
        self.state
    }

    #[inline(always)]
    pub fn alpha(&self) -> f32 {
        self.alpha
    }

    pub fn reset(&mut self) {
        self.state = 0.0;
        self.initialized = false;
    }
}

/// 3軸ローパスフィルタ
#[derive(Debug, Clone, Copy)]
pub struct Lpf3 {
    axes: [Lpf; 3],
}

impl Lpf3 {
    /// 3軸同一カットオフ
    pub fn new(fc: f32, dt: f32) -> Self {
        Self {
            axes: [Lpf::new(fc, dt); 3],
        }
    }

    /// 軸ごとに異なるカットオフ（必要になったら使う）
    pub fn new_per_axis(fc_x: f32, fc_y: f32, fc_z: f32, dt: f32) -> Self {
        Self {
            axes: [
                Lpf::new(fc_x, dt),
                Lpf::new(fc_y, dt),
                Lpf::new(fc_z, dt),
            ],
        }
    }

    #[inline]
    pub fn update(&mut self, x: f32, y: f32, z: f32) -> (f32, f32, f32) {
        (
            self.axes[0].update(x),
            self.axes[1].update(y),
            self.axes[2].update(z),
        )
    }

    /// 配列版（磁力計のように [f32; 3] で扱う場合）
    #[inline]
    pub fn update_array(&mut self, v: [f32; 3]) -> [f32; 3] {
        [
            self.axes[0].update(v[0]),
            self.axes[1].update(v[1]),
            self.axes[2].update(v[2]),
        ]
    }

    #[inline]
    pub fn get(&self) -> (f32, f32, f32) {
        (self.axes[0].get(), self.axes[1].get(), self.axes[2].get())
    }

    pub fn reset(&mut self) {
        for ax in &mut self.axes {
            ax.reset();
        }
    }
}