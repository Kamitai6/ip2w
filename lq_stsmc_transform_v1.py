"""
LQ-STSMC 設計: 正則変換と部分行列の計算

状態ベクトル (6状態):
  x = [x_pos, v_pos, θ_pitch, ω_pitch, ψ_yaw, ω_yaw]

拡大系 (8状態):
  x_aug = [x_pos, v_pos, θ_pitch, ω_pitch, ψ_yaw, ω_yaw, ∫e_pos, ∫e_yaw]

入力:
  u = [τ_L, τ_R]

正則変換後:
  z₁ = [x_pos, θ_pitch, ψ_yaw, w, ∫e_pos, ∫e_yaw]  ∈ R⁶ (非駆動)
  z₂ = [v_pos, ω_yaw]                                 ∈ R² (駆動)

  w = ω_pitch + α·v_pos  (入力が消去される線形結合)
"""

import numpy as np

np.set_printoptions(precision=6, suppress=True, linewidth=120)

# ============================================================
# 1. 元の線形化モデル
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

n = 6  # 元の状態数
m = 2  # 入力数

# ============================================================
# 2. 拡大系 (積分器追加)
# ============================================================
# d(∫e_pos)/dt = -x_pos
# d(∫e_yaw)/dt = -ψ_yaw

C_e = np.array([
    [-1, 0, 0, 0,  0, 0],  # -x_pos
    [ 0, 0, 0, 0, -1, 0],  # -ψ_yaw
])

n_aug = n + 2  # = 8

A_aug = np.zeros((n_aug, n_aug))
A_aug[:n, :n] = A
A_aug[n:, :n] = C_e

B_aug = np.zeros((n_aug, m))
B_aug[:n, :] = B

print("=" * 60)
print("拡大系")
print("=" * 60)
print(f"\nA_aug ({n_aug}x{n_aug}):")
print(A_aug)
print(f"\nB_aug ({n_aug}x{m}):")
print(B_aug)

# ============================================================
# 3. 正則変換 T の構成
# ============================================================
# B_aug の非ゼロ行: row 1 (v_pos), row 3 (ω_pitch), row 5 (ω_yaw)
# v_pos と ω_pitch は同じ方向 (τ_sum) → 線形結合で消去
#
# w = ω_pitch + α·v_pos where α = -B_aug[3,0] / B_aug[1,0]
# → B contribution: B_aug[3,:] + α·B_aug[1,:] = 0

alpha = -B_aug[3, 0] / B_aug[1, 0]
print(f"\nα (入力消去係数) = {alpha:.6f}")

# 検証: B_aug[3,:] + α * B_aug[1,:] = 0 ?
cancel_check = B_aug[3, :] + alpha * B_aug[1, :]
print(f"消去検証 (should be ~0): {cancel_check}")

# 変換行列 T: z = T · x_aug
#
# x_aug indices: 0:x_pos, 1:v_pos, 2:θ_pitch, 3:ω_pitch, 4:ψ_yaw, 5:ω_yaw, 6:∫e_pos, 7:∫e_yaw
#
# z₁[0] = x_pos
# z₁[1] = θ_pitch
# z₁[2] = ψ_yaw
# z₁[3] = ω_pitch + α·v_pos
# z₁[4] = ∫e_pos
# z₁[5] = ∫e_yaw
# z₂[0] = v_pos
# z₂[1] = ω_yaw

T = np.zeros((n_aug, n_aug))
# z₁
T[0, 0] = 1                    # x_pos
T[1, 2] = 1                    # θ_pitch
T[2, 4] = 1                    # ψ_yaw
T[3, 1] = alpha; T[3, 3] = 1  # ω_pitch + α·v_pos
T[4, 6] = 1                    # ∫e_pos
T[5, 7] = 1                    # ∫e_yaw
# z₂
T[6, 1] = 1                    # v_pos
T[7, 5] = 1                    # ω_yaw

print(f"\n変換行列 T ({n_aug}x{n_aug}):")
print(T)

T_inv = np.linalg.inv(T)
print(f"\nT⁻¹ ({n_aug}x{n_aug}):")
print(T_inv)

# 検証: T @ T_inv = I
identity_check = np.max(np.abs(T @ T_inv - np.eye(n_aug)))
print(f"\nT·T⁻¹ = I 検証 (max error): {identity_check:.2e}")

# ============================================================
# 4. 変換後の系: ż = T·A_aug·T⁻¹·z + T·B_aug·u
# ============================================================
A_z = T @ A_aug @ T_inv
B_z = T @ B_aug

print("\n" + "=" * 60)
print("変換後の系")
print("=" * 60)
print(f"\nA_z = T·A_aug·T⁻¹ ({n_aug}x{n_aug}):")
print(A_z)
print(f"\nB_z = T·B_aug ({n_aug}x{m}):")
print(B_z)

# ============================================================
# 5. 部分行列の抽出
# ============================================================
n1 = n_aug - m  # = 6
n2 = m          # = 2

A11 = A_z[:n1, :n1]
A12 = A_z[:n1, n1:]
A21 = A_z[n1:, :n1]
A22 = A_z[n1:, n1:]
B2  = B_z[n1:, :]

print("\n" + "=" * 60)
print("部分行列")
print("=" * 60)
print(f"\nA₁₁ ({n1}x{n1}) [非駆動 → 非駆動]:")
print(A11)
print(f"\nA₁₂ ({n1}x{n2}) [駆動 → 非駆動]:")
print(A12)
print(f"\nA₂₁ ({n2}x{n1}) [非駆動 → 駆動]:")
print(A21)
print(f"\nA₂₂ ({n2}x{n2}) [駆動 → 駆動]:")
print(A22)
print(f"\nB₂ ({n2}x{m}) [入力 → 駆動]:")
print(B2)

# B_z の上半分が 0 であることの検証 (z₁ に入力が入らない)
B1_check = B_z[:n1, :]
print(f"\nB₁ (should be ~0) ({n1}x{m}):")
print(B1_check)
print(f"B₁ max abs: {np.max(np.abs(B1_check)):.2e}")

# B₂ の正則性確認
det_B2 = np.linalg.det(B2)
print(f"\ndet(B₂) = {det_B2:.2f}")
print(f"B₂ は{'正則' if abs(det_B2) > 1e-6 else '特異 (問題あり!)'}です")

# ============================================================
# 6. (A₁₁, A₁₂) の可制御性確認
# ============================================================
from numpy.linalg import matrix_rank

ctrb_cols = [A12]
for i in range(1, n1):
    ctrb_cols.append(A11 @ ctrb_cols[-1])
Ctrb = np.hstack(ctrb_cols)
rank_ctrb = matrix_rank(Ctrb)
print(f"\n可制御性行列のランク: {rank_ctrb} / {n1}")
print(f"(A₁₁, A₁₂) は{'可制御' if rank_ctrb == n1 else '不可制御 (問題あり!)'}です")

# ============================================================
# 7. サマリー (ファームウェア埋め込み用定数)
# ============================================================
print("\n" + "=" * 60)
print("ファームウェア埋め込み用定数")
print("=" * 60)
print(f"\nalpha = {alpha}")
print(f"\nA₁₁ = {A11.tolist()}")
print(f"\nA₁₂ = {A12.tolist()}")
print(f"\nB₂ = {B2.tolist()}")
print(f"\nB_aug = {B_aug.tolist()}")
print(f"\nT = {T.tolist()}")
print(f"\nT_inv = {T_inv.tolist()}")
