"""
LQ-STSMC: S, SA, D 行列の計算 + Rust定数出力

前段の transform_v1 と sim_v1 の結果を統合して、
ファームウェアに埋め込む定数を生成する。
"""

import numpy as np
from scipy.linalg import solve_continuous_are

np.set_printoptions(precision=8, suppress=True, linewidth=120)

# ============================================================
# 1. モデル定数 (transform_v1 から)
# ============================================================
A = np.array([
    [0, 1,     0, 0, 0, 0],
    [0, 0, -1.72, 0, 0, 0],
    [0, 0,     0, 1, 0, 0],
    [0, 0, 83.13, 0, 0, 0],
    [0, 0,     0, 0, 0, 1],
    [0, 0,     0, 0, 0, 0],
])

B = np.array([
    [       0,         0],
    [  282.00,    282.00],
    [       0,         0],
    [-4092.67,  -4092.67],
    [       0,         0],
    [13717.42, -13717.42],
])

# 拡大系
n, m = 6, 2
n_aug = n + 2

C_e = np.array([
    [-1, 0, 0, 0,  0, 0],
    [ 0, 0, 0, 0, -1, 0],
])

A_aug = np.zeros((n_aug, n_aug))
A_aug[:n, :n] = A
A_aug[n:, :n] = C_e

B_aug = np.zeros((n_aug, m))
B_aug[:n, :] = B

# 正則変換 T
alpha = -B_aug[3, 0] / B_aug[1, 0]  # = 14.513014...

T = np.zeros((n_aug, n_aug))
T[0, 0] = 1
T[1, 2] = 1
T[2, 4] = 1
T[3, 1] = alpha; T[3, 3] = 1
T[4, 6] = 1
T[5, 7] = 1
T[6, 1] = 1
T[7, 5] = 1

T_inv = np.linalg.inv(T)

# 部分行列
A_z = T @ A_aug @ T_inv
n1, n2 = n_aug - m, m

A11 = A_z[:n1, :n1]
A12 = A_z[:n1, n1:]
A21 = A_z[n1:, :n1]
A22 = A_z[n1:, n1:]
B2  = (T @ B_aug)[n1:, :]

# ============================================================
# 2. Q, R → LQR → K
# ============================================================
# z₁ = [x_pos, θ_pitch, ψ_yaw, w, ∫e_pos, ∫e_yaw]

Q1 = np.diag([
    1.0 / 0.3**2,    # x_pos
    1.0 / 0.05**2,   # θ_pitch
    1.0 / 0.2**2,    # ψ_yaw
    1.0 / 5.0**2,    # w
    1.0 / 1.0**2,    # ∫e_pos
    1.0 / 1.0**2,    # ∫e_yaw
])

R1 = np.diag([
    1.0 / 0.5**2,    # v_pos
    1.0 / 3.0**2,    # ω_yaw
])

P = solve_continuous_are(A11, A12, Q1, R1)
K = np.linalg.solve(R1, A12.T @ P)

print("K (2x6):")
print(K)

# ============================================================
# 3. S, SA, D の計算
# ============================================================
# σ = z₂ + K·z₁ = [K | I₂]·z = [K | I₂]·T·x_aug
# → S = [K | I₂] · T

K_I2 = np.hstack([K, np.eye(n2)])  # 2x8
S = K_I2 @ T                        # 2x8

SA = S @ A_aug                       # 2x8
SB = S @ B_aug                       # 2x2
D = np.linalg.inv(SB)                # 2x2 = (SB)⁻¹

print("\nS (2x8):")
print(S)
print("\nSA (2x8) = S·A_aug:")
print(SA)
print("\nSB (2x2) = S·B_aug:")
print(SB)
print(f"\ndet(SB) = {np.linalg.det(SB):.4f}")
print("\nD (2x2) = (SB)⁻¹:")
print(D)

# ============================================================
# 4. 検証
# ============================================================
# 閉ループ固有値
A_cl = A11 - A12 @ K
eigs = np.linalg.eigvals(A_cl)
print("\n閉ループ固有値:")
for i, e in enumerate(eigs):
    print(f"  λ{i} = {e:.6f}")

# 等価制御の一致検証
# u_eq = -D · SA · x  should equal  -B₂⁻¹ (A₂₁ - A₂₂K + K·A_cl) z₁ (via T)
# Spot check with random x
np.random.seed(42)
x_test = np.random.randn(n_aug)
u_via_DSA = -D @ SA @ x_test
z_test = T @ x_test
z1_test = z_test[:n1]
K_eq = np.linalg.inv(B2) @ (A21 - A22 @ K + K @ A_cl)
u_via_Keq = -K_eq @ z1_test
print(f"\n等価制御の一致検証:")
print(f"  u via D·SA: {u_via_DSA}")
print(f"  u via K_eq: {u_via_Keq}")
print(f"  差のノルム: {np.linalg.norm(u_via_DSA - u_via_Keq):.2e}")

# ============================================================
# 5. Rust定数出力
# ============================================================
def to_rust_2d(name, mat):
    rows = []
    for row in mat:
        elems = ", ".join(f"{v:.8}" for v in row)
        rows.append(f"    [{elems}]")
    inner = ",\n".join(rows)
    return f"const {name}: [[f32; {mat.shape[1]}]; {mat.shape[0]}] = [\n{inner},\n];"

print("\n" + "=" * 60)
print("Rust 定数")
print("=" * 60)
print()
print(to_rust_2d("S_MAT", S))
print()
print(to_rust_2d("SA_MAT", SA))
print()
print(to_rust_2d("D_MAT", D))
print()
