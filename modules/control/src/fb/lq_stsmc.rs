//! LQ-STSMC: LQ最適スライディング面 + Super Twisting SMC (MIMO 2ch)
//!
//! 方法2 (スライディング面のLQ最適設計) + 方法B (デカップリング)
//!
//! 初期化時にCARE (連続代数リカッチ方程式) を解いて S, SA, D を計算する。
//! Pythonでの事前計算は不要。
//!
//! 状態ベクトル:
//!   x_aug = [x_pos, v_pos, θ_pitch, ω_pitch, ψ_yaw, ω_yaw, ∫e_pos, ∫e_yaw]
//!
//! 出力:
//!   u = [τ_L, τ_R]  (Nm)

use libm::{sqrtf, fabsf, tanhf};

const N_AUG: usize = 8;
const N1: usize = 6; // z₁ 次元 (非駆動)
const N2: usize = 2; // z₂ 次元 (駆動) = 入力数
const H_DIM: usize = 12; // ハミルトン行列サイズ = 2 * N1
const DELTA: f32 = 0.00001;
const V_LEAK: f32 = 0.00001;

// ============================================================
// 行列演算ヘルパー
// ============================================================

/// 行列乗算 (const generics)
fn mat_mul<const R: usize, const C: usize, const K: usize>(
    a: &[[f32; K]; R],
    b: &[[f32; C]; K],
) -> [[f32; C]; R] {
    let mut c = [[0.0f32; C]; R];
    for i in 0..R {
        for j in 0..C {
            let mut sum = 0.0f32;
            for k in 0..K {
                sum += a[i][k] * b[k][j];
            }
            c[i][j] = sum;
        }
    }
    c
}

/// 行列転置 (const generics)
fn mat_transpose<const R: usize, const C: usize>(
    a: &[[f32; C]; R],
) -> [[f32; R]; C] {
    let mut t = [[0.0f32; R]; C];
    for i in 0..R {
        for j in 0..C {
            t[j][i] = a[i][j];
        }
    }
    t
}

/// 2×2 行列逆行列
fn inv_2x2(a: &[[f32; 2]; 2]) -> Option<[[f32; 2]; 2]> {
    let det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
    if fabsf(det) < 1e-10 {
        return None;
    }
    let inv_det = 1.0 / det;
    Some([
        [ a[1][1] * inv_det, -a[0][1] * inv_det],
        [-a[1][0] * inv_det,  a[0][0] * inv_det],
    ])
}

/// Gauss-Jordan 法による行列逆行列 (マクロでサイズ特殊化)
macro_rules! impl_mat_inv {
    ($fn_name:ident, $n:expr) => {
        fn $fn_name(a: &[[f32; $n]; $n]) -> Option<[[f32; $n]; $n]> {
            let mut aug = [[0.0f32; $n * 2]; $n];
            for i in 0..$n {
                for j in 0..$n {
                    aug[i][j] = a[i][j];
                }
                aug[i][$n + i] = 1.0;
            }
            for col in 0..$n {
                // 部分ピボット選択
                let mut max_val = fabsf(aug[col][col]);
                let mut max_row = col;
                for row in (col + 1)..$n {
                    let v = fabsf(aug[row][col]);
                    if v > max_val {
                        max_val = v;
                        max_row = row;
                    }
                }
                if max_val < 1e-10 {
                    return None;
                }
                aug.swap(col, max_row);

                let pivot = aug[col][col];
                for j in 0..($n * 2) {
                    aug[col][j] /= pivot;
                }
                for row in 0..$n {
                    if row == col {
                        continue;
                    }
                    let factor = aug[row][col];
                    for j in 0..($n * 2) {
                        aug[row][j] -= factor * aug[col][j];
                    }
                }
            }
            let mut result = [[0.0f32; $n]; $n];
            for i in 0..$n {
                for j in 0..$n {
                    result[i][j] = aug[i][$n + j];
                }
            }
            Some(result)
        }
    };
}

impl_mat_inv!(inv_6x6, 6);
impl_mat_inv!(inv_12x12, 12);

// ============================================================
// CARE ソルバー (行列符号関数法)
// ============================================================

/// 連続代数リカッチ方程式 (CARE) を解く
///
/// A₁₁ᵀP + PA₁₁ - PA₁₂ R₁⁻¹ A₁₂ᵀP + Q₁ = 0
///
/// 行列符号関数法: H_{k+1} = (H_k + H_k⁻¹) / 2
fn solve_care(
    a: &[[f32; N1]; N1],   // A₁₁ (6×6)
    b: &[[f32; N2]; N1],   // A₁₂ (6×2)
    q_diag: &[f32; N1],    // Q₁ 対角成分
    r_diag: &[f32; N2],    // R₁ 対角成分
) -> Option<[[f32; N1]; N1]> {
    // 1. S_br = A₁₂ · R₁⁻¹ · A₁₂ᵀ (6×6)
    let mut s_br = [[0.0f32; N1]; N1];
    for i in 0..N1 {
        for j in 0..N1 {
            let mut sum = 0.0f32;
            for k in 0..N2 {
                sum += b[i][k] * b[j][k] / r_diag[k];
            }
            s_br[i][j] = sum;
        }
    }

    // 2. ハミルトン行列 H (12×12)
    //    H = [[ A₁₁,    -S_br ],
    //         [-Q₁,     -A₁₁ᵀ ]]
    let mut h = [[0.0f32; H_DIM]; H_DIM];
    for i in 0..N1 {
        for j in 0..N1 {
            h[i][j] = a[i][j];              // 左上: A₁₁
            h[i][N1 + j] = -s_br[i][j];     // 右上: -S_br
            h[N1 + i][N1 + j] = -a[j][i];   // 右下: -A₁₁ᵀ
        }
        h[N1 + i][i] = -q_diag[i];          // 左下: -Q₁ (対角)
    }

    // 3. 行列符号関数反復
    const MAX_ITER: usize = 50;
    const TOL: f32 = 1e-5;

    for _iter in 0..MAX_ITER {
        let h_inv = inv_12x12(&h)?;

        let mut diff_sq = 0.0f32;
        let mut h_new = [[0.0f32; H_DIM]; H_DIM];
        for i in 0..H_DIM {
            for j in 0..H_DIM {
                h_new[i][j] = (h[i][j] + h_inv[i][j]) * 0.5;
                let d = h_new[i][j] - h[i][j];
                diff_sq += d * d;
            }
        }
        h = h_new;

        if diff_sq < TOL * TOL {
            break;
        }
    }

    // 4. W = (I - H_∞) / 2
    //    W₁₁ = 左上 6×6, W₂₁ = 左下 6×6
    let mut w11 = [[0.0f32; N1]; N1];
    let mut w21 = [[0.0f32; N1]; N1];
    for i in 0..N1 {
        for j in 0..N1 {
            let delta = if i == j { 1.0 } else { 0.0 };
            w11[i][j] = (delta - h[i][j]) * 0.5;
            w21[i][j] = -h[N1 + i][j] * 0.5; // I₂₁ = 0
        }
    }

    // 5. P = W₂₁ · W₁₁⁻¹
    let w11_inv = inv_6x6(&w11)?;
    Some(mat_mul::<N1, N1, N1>(&w21, &w11_inv))
}

// ============================================================
// LQ-STSMC 制御器
// ============================================================

/// STSMC チャネルパラメータ
#[derive(Clone)]
pub struct StsmcChannelConfig {
    pub lambda: f32,
    pub alpha: f32,
    pub epsilon: f32,
    pub v_limit: f32,
}

/// LQ-STSMC 設定
pub struct LqStsmcConfig {
    pub dt: f32,

    // 物理モデル定数 (Python transform_v1 の出力、固定)
    pub a11: [[f32; N1]; N1],
    pub a12: [[f32; N2]; N1],
    pub t_mat: [[f32; N_AUG]; N_AUG],
    pub a_aug: [[f32; N_AUG]; N_AUG],
    pub b_aug: [[f32; N2]; N_AUG],

    // チューニングパラメータ (ここだけ変える)
    pub q_diag: [f32; N1],
    pub r_diag: [f32; N2],

    pub ch: [StsmcChannelConfig; N2],
    pub tau_limit: f32,
}

pub struct LqStsmc {
    dt: f32,
    s: [[f32; N_AUG]; N2],
    sa: [[f32; N_AUG]; N2],
    d: [[f32; N2]; N2],
    ch: [StsmcChannelConfig; N2],
    tau_limit: f32,

    w: [f32; N2],
    integral_pos: f32,
    integral_yaw: f32,
}

#[derive(Clone, defmt::Format)]
pub struct LqStsmcOutput {
    pub tau_l: f32,
    pub tau_r: f32,
    pub sigma: [f32; N2],
}

impl LqStsmc {
    /// 初期化: CARE を解いて S, SA, D を計算
    pub fn new(cfg: LqStsmcConfig) -> Self {
        // 1. CARE → P → K
        let p = solve_care(&cfg.a11, &cfg.a12, &cfg.q_diag, &cfg.r_diag)
            .expect("CARE solver failed");

        // K = R₁⁻¹ · A₁₂ᵀ · P  (2×6)
        let a12_t = mat_transpose::<N1, N2>(&cfg.a12); // 2×6
        let a12t_p = mat_mul::<N2, N1, N1>(&a12_t, &p); // 2×6
        let mut k = [[0.0f32; N1]; N2];
        for i in 0..N2 {
            for j in 0..N1 {
                k[i][j] = a12t_p[i][j] / cfg.r_diag[i];
            }
        }

        // 2. S = [K | I₂] · T  (2×8)
        //    [K | I₂] は 2×8 行列: 左6列が K, 右2列が I₂
        let mut k_i2 = [[0.0f32; N_AUG]; N2];
        for i in 0..N2 {
            for j in 0..N1 {
                k_i2[i][j] = k[i][j];
            }
            k_i2[i][N1 + i] = 1.0;
        }
        let s = mat_mul::<N2, N_AUG, N_AUG>(&k_i2, &cfg.t_mat);

        // 3. SA = S · A_aug  (2×8)
        let sa = mat_mul::<N2, N_AUG, N_AUG>(&s, &cfg.a_aug);

        // 4. D = (S · B_aug)⁻¹  (2×2)
        let sb = mat_mul::<N2, N2, N_AUG>(&s, &cfg.b_aug);
        let d = inv_2x2(&sb).expect("SB singular");

        Self {
            dt: cfg.dt,
            s,
            sa,
            d,
            ch: cfg.ch,
            tau_limit: cfg.tau_limit,
            w: [0.0; N2],
            integral_pos: 0.0,
            integral_yaw: 0.0,
        }
    }

    /// メイン更新関数
    ///
    /// x6: [x_pos, v_pos, θ_pitch, ω_pitch, ψ_yaw, ω_yaw]
    /// pos_ref: 位置目標 (通常 0)
    /// yaw_ref: ヨー目標 (通常 0)
    pub fn update(&mut self, x6: &[f32; 6], pos_ref: f32, yaw_ref: f32) -> LqStsmcOutput {
        let dt = self.dt;

        let x_pos_err = x6[0] - pos_ref;
        let yaw_err = x6[4] - yaw_ref;

        self.integral_pos += -x_pos_err * dt;
        self.integral_yaw += -yaw_err * dt;

        let x_aug: [f32; N_AUG] = [
            x_pos_err,
            x6[1],
            x6[2],
            x6[3],
            yaw_err,
            x6[5],
            self.integral_pos,
            self.integral_yaw,
        ];

        // σ = S · x_aug
        let sigma = mat_vec_2x8(&self.s, &x_aug);

        // -SA · x_aug
        let sa_x = mat_vec_2x8(&self.sa, &x_aug);
        let neg_sa_x = [-sa_x[0], -sa_x[1]];

        // Super Twisting per channel
        let mut v = [0.0f32; N2];
        for i in 0..N2 {
            let sat = tanhf(sigma[i] / self.ch[i].epsilon);

            self.w[i] = ((1.0 - V_LEAK * dt) * self.w[i]
                - self.ch[i].alpha * sat * dt)
                .clamp(-self.ch[i].v_limit, self.ch[i].v_limit);

            v[i] = -self.ch[i].lambda * sqrtf(fabsf(sigma[i]) + DELTA) * sat
                + self.w[i];
        }

        // u = D · (-SA·x + v)
        let rhs = [neg_sa_x[0] + v[0], neg_sa_x[1] + v[1]];
        let u = mat_vec_2x2(&self.d, &rhs);

        LqStsmcOutput {
            tau_l: u[0].clamp(-self.tau_limit, self.tau_limit),
            tau_r: u[1].clamp(-self.tau_limit, self.tau_limit),
            sigma,
        }
    }

    pub fn reset(&mut self) {
        self.w = [0.0; N2];
        self.integral_pos = 0.0;
        self.integral_yaw = 0.0;
    }

    /// デバッグ用: 計算された S 行列を返す
    pub fn s_matrix(&self) -> &[[f32; N_AUG]; N2] {
        &self.s
    }

    /// デバッグ用: 計算された D 行列を返す
    pub fn d_matrix(&self) -> &[[f32; N2]; N2] {
        &self.d
    }
}

#[inline]
fn mat_vec_2x8(m: &[[f32; N_AUG]; N2], v: &[f32; N_AUG]) -> [f32; N2] {
    let mut out = [0.0f32; N2];
    for i in 0..N2 {
        let mut sum = 0.0f32;
        for j in 0..N_AUG {
            sum += m[i][j] * v[j];
        }
        out[i] = sum;
    }
    out
}

#[inline]
fn mat_vec_2x2(m: &[[f32; N2]; N2], v: &[f32; N2]) -> [f32; N2] {
    [
        m[0][0] * v[0] + m[0][1] * v[1],
        m[1][0] * v[0] + m[1][1] * v[1],
    ]
}