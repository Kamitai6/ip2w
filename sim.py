"""
LQ-STSMC シミュレーター v2
==========================
lq_stsmc_transform_v2.py の修正済みモデルを使用。

【v1からの変更点】
1. M_w, J_w を両輪合計→1輪あたりに修正
2. 入力順序を明示: u = [τ_R, τ_L]^T
3. 物理パラメータから A, B を直接導出（ハードコードなし）
"""

import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# ============================================================
# 1. 物理パラメータ → A, B
# ============================================================
M_p = 0.1
l   = 0.035
J_p = 0.000363
J_psi = 0.0000729

M_w_total = 0.023
J_w_total = 0.00001035
r = 0.03
W = 0.06
g = 9.81

M_w = M_w_total / 2
J_w = J_w_total / 2

D = (M_p + 2*M_w + 2*J_w/r**2) * (J_p + M_p*l**2) - (M_p*l)**2

a23 = -(M_p*l)**2 * g / D
a43 = (M_p + 2*M_w + 2*J_w/r**2) * M_p*g*l / D
b21 = (J_p + M_p*l**2 + M_p*l*r) / (r * D)
b41 = -(M_p*l + (M_p + 2*M_w + 2*J_w/r**2)*r) / (r * D)
b61 = W / (2*r*J_psi)

print("=" * 60)
print("物理パラメータから導出した係数")
print("=" * 60)
print(f"D    = {D:.8f}")
print(f"a23  = {a23:.6f}")
print(f"a43  = {a43:.6f}")
print(f"b21  = {b21:.6f}")
print(f"b41  = {b41:.6f}")
print(f"b61  = {b61:.6f}")

A = np.array([
    [0, 1,   0, 0, 0, 0],
    [0, 0, a23, 0, 0, 0],
    [0, 0,   0, 1, 0, 0],
    [0, 0, a43, 0, 0, 0],
    [0, 0,   0, 0, 0, 1],
    [0, 0,   0, 0, 0, 0],
])

# u = [τ_R, τ_L]^T
B = np.array([
    [   0,     0],
    [ b21,   b21],
    [   0,     0],
    [ b41,   b41],
    [   0,     0],
    [+b61,  -b61],
])

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

# ============================================================
# 2. 正則変換
# ============================================================
alpha = -B_aug[3, 0] / B_aug[1, 0]

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

A_z = T @ A_aug @ T_inv
B_z = T @ B_aug

n1, n2 = n_aug - m, m

A11 = A_z[:n1, :n1]
A12 = A_z[:n1, n1:]
A21 = A_z[n1:, :n1]
A22 = A_z[n1:, n1:]
B2  = B_z[n1:, :]

# ============================================================
# 3. Q, R の設定
# ============================================================
# z₁ = [x, θ, ψ, w, ∫e_x, ∫e_ψ]
# 仮想入力 z₂ = [ẋ, ψ̇]

Q1 = np.diag([
    1.0 / 0.5**2,    # x
    1.0 / 0.03**2,   # θ
    1.0 / 0.2**2,    # ψ
    1.0 / 5.0**2,    # w
    1.0 / 5.0**2,    # ∫e_x
    1.0 / 1.0**2,    # ∫e_ψ
])

R1 = np.diag([
    1.0 / 0.4**2,    # ẋ  (0.3→0.4: τmax≈0.12Nm=81%利用、飽和しない上限)
    1.0 / 1.0**2,    # ψ̇
])

# ============================================================
# 4. LQR → K → 閉ループ
# ============================================================
P = solve_continuous_are(A11, A12, Q1, R1)
K = np.linalg.solve(R1, A12.T @ P)

A_cl = A11 - A12 @ K

print("\n" + "=" * 60)
print("LQR 結果")
print("=" * 60)
print(f"\nK (2x6):\n{np.array2string(K, precision=4, suppress_small=True)}")

eigs = np.linalg.eigvals(A_cl)
print(f"\n閉ループ固有値:")
for i, e in enumerate(eigs):
    print(f"  λ{i} = {e:.4f}  (|Re| = {abs(e.real):.4f})")

# 等価制御ゲイン
B2_inv = np.linalg.inv(B2)
K_eq = B2_inv @ (A21 - A22 @ K + K @ A_cl)

# ============================================================
# 5. シミュレーション
# ============================================================
T_SIM = 5.0
DT    = 0.002
TAU_MAX = 0.15  # [Nm] モーター最大トルク

def simulate(z1_0, label):
    def dynamics(t, z1):
        return A_cl @ z1

    sol = solve_ivp(dynamics, (0, T_SIM), z1_0,
                    t_eval=np.arange(0, T_SIM, DT),
                    method='RK45', rtol=1e-9, atol=1e-12)

    t   = sol.t
    z1  = sol.y  # (6, N)
    z2  = -K @ z1  # 面上: z₂ = -K·z₁

    # 物理状態に復元
    z_full = np.vstack([z1, z2])
    x_aug  = T_inv @ z_full

    # 等価制御トルク: u = [τ_R, τ_L]
    u_eq = -K_eq @ z1

    return t, x_aug, u_eq, label


# ── シナリオ ──
# z₁ = [x, θ, ψ, w, ∫e_x, ∫e_ψ]
scenarios = [
    (np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0]), "Pos step (x=0.5m)"),
    (np.array([0.0, 0.0, 0.5, 0.0, 0.0, 0.0]), "Yaw step (ψ=0.5rad)"),
    (np.array([0.5, 0.0, 0.3, 0.0, 0.0, 0.0]), "Pos+Yaw combined"),
]

results = [simulate(z0, lbl) for z0, lbl in scenarios]

# ============================================================
# 6. プロット
# ============================================================
fig, axes = plt.subplots(4, len(results),
                         figsize=(6 * len(results), 14), sharex='col')
fig.suptitle("LQ-STSMC v2: Sliding Surface Dynamics (σ=0)\n"
             "u=[τ_R, τ_L], M_w/J_w per-wheel corrected",
             fontsize=13, y=0.98)

for col, (t, x_aug, u_eq, label) in enumerate(results):
    # Row 0: 位置・ヨー
    ax = axes[0, col]
    ax.plot(t, x_aug[0], label="x_pos", lw=1.5)
    ax.plot(t, x_aug[4], label="ψ_yaw", lw=1.5)
    ax.axhline(0, color='gray', ls='--', lw=0.5)
    ax.set_ylabel("Position / Yaw")
    ax.set_title(label, fontsize=11)
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # Row 1: ピッチ
    ax = axes[1, col]
    ax.plot(t, np.degrees(x_aug[2]), label="θ_pitch [deg]", color='red', lw=1.5)
    ax.axhline(0, color='gray', ls='--', lw=0.5)
    ax.set_ylabel("Pitch [deg]")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # Row 2: トルク (u[0]=τ_R, u[1]=τ_L)
    ax = axes[2, col]
    ax.plot(t, u_eq[0], label="τ_R", lw=1.5)
    ax.plot(t, u_eq[1], label="τ_L", lw=1.5)
    ax.axhline( TAU_MAX, color='red', ls='--', lw=0.8, label=f"±{TAU_MAX} Nm")
    ax.axhline(-TAU_MAX, color='red', ls='--', lw=0.8)
    ax.set_ylabel("Torque [Nm]")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # Row 3: 積分状態
    ax = axes[3, col]
    ax.plot(t, x_aug[6], label="∫e_x", lw=1.5)
    ax.plot(t, x_aug[7], label="∫e_ψ", lw=1.5)
    ax.axhline(0, color='gray', ls='--', lw=0.5)
    ax.set_ylabel("Integral states")
    ax.set_xlabel("Time [s]")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("lq_stsmc_sim_v2.png", dpi=150, bbox_inches='tight')
plt.close()
print("\n画像保存: lq_stsmc_sim_v2.png")

# ============================================================
# 7. 数値サマリー
# ============================================================
print("\n" + "=" * 60)
print("数値サマリー")
print("=" * 60)

for t, x_aug, u_eq, label in results:
    print(f"\n--- {label} ---")

    tau_R_max = np.max(np.abs(u_eq[0]))
    tau_L_max = np.max(np.abs(u_eq[1]))
    print(f"  τ_R max: {tau_R_max:.4f} Nm  {'⚠ 飽和!' if tau_R_max > TAU_MAX else '✓ OK'}")
    print(f"  τ_L max: {tau_L_max:.4f} Nm  {'⚠ 飽和!' if tau_L_max > TAU_MAX else '✓ OK'}")

    pitch_max_deg = np.max(np.abs(np.degrees(x_aug[2])))
    print(f"  θ_pitch max: {pitch_max_deg:.2f} deg")

    for idx, name in [(0, "x_pos"), (4, "ψ_yaw")]:
        x0_val = x_aug[idx, 0]
        if abs(x0_val) < 1e-6:
            print(f"  {name} 整定時間: N/A (初期値≈0)")
            continue
        threshold = abs(x0_val) * 0.05
        settled = np.where(np.abs(x_aug[idx]) < threshold)[0]
        if len(settled) > 0:
            for j in range(len(settled)):
                if np.all(np.abs(x_aug[idx, settled[j]:]) < threshold):
                    print(f"  {name} 整定時間 (5%): {t[settled[j]]:.3f} s")
                    break
            else:
                print(f"  {name} 整定時間: > {T_SIM} s (未整定)")
        else:
            print(f"  {name} 整定時間: > {T_SIM} s (未整定)")