"""
LQ-STSMC シミュレーター
======================
方法2 (スライディング面のLQ最適設計) の面上閉ループ系を評価する。

面上 (σ=0) では:
  ż₁ = (A₁₁ - A₁₂·K) · z₁

z₁ = [x_pos, θ_pitch, ψ_yaw, w, ∫e_pos, ∫e_yaw]
z₂ = [v_pos, ω_yaw]  (= -K·z₁ on sliding surface)

評価項目:
  1. トルク飽和チェック (u_eq の最大値)
  2. 干渉の可視化 (位置ステップ→ピッチ/ヨー応答)
  3. 整定時間の確認
"""

import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# ============================================================
# 1. 前段で計算済みの定数 (lq_stsmc_transform_v1.py の出力)
# ============================================================
A11 = np.array([
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 58.167616, 0.0, 0.0, 0.0, 0.0],
    [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, -1.0, 0.0, 0.0, 0.0],
])

A12 = np.array([
    [1.0, 0.0],
    [-14.513014, 0.0],
    [0.0, 1.0],
    [0.0, 0.0],
    [0.0, 0.0],
    [0.0, 0.0],
])

A21 = np.array([
    [0.0, -1.72, 0.0, 0.0, 0.0, 0.0],
    [0.0,  0.0,  0.0, 0.0, 0.0, 0.0],
])

A22 = np.array([
    [0.0, 0.0],
    [0.0, 0.0],
])

B2 = np.array([
    [282.0, 282.0],
    [13717.42, -13717.42],
])

# 元の拡大系 (トルク計算の検算用)
B_aug = np.array([
    [0.0, 0.0],
    [282.0, 282.0],
    [0.0, 0.0],
    [-4092.67, -4092.67],
    [0.0, 0.0],
    [13717.42, -13717.42],
    [0.0, 0.0],
    [0.0, 0.0],
])

T_inv = np.array([
    [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -14.513014, 0.0],
    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
])

n1 = 6  # z₁ の次元
n2 = 2  # z₂ の次元

# ============================================================
# 2. Q, R の設定 (ブライソンの法則ベース)
# ============================================================
# z₁ = [x_pos, θ_pitch, ψ_yaw, w, ∫e_pos, ∫e_yaw]
#
# 方法2では「入力」は z₂ = [v_pos, ω_yaw] (仮想入力)

# --- ここを調整 ---
Q1 = np.diag([
    1.0 / 0.5**2,    # x_pos:    0.5mに緩め、急な位置補正によるトルク飽和を防ぐ
    1.0 / 0.03**2,   # θ_pitch:  0.03rad(約1.7度)に設定し、剛性を上げる
    1.0 / 0.2**2,    # ψ_yaw:    (変更なし)
    1.0 / 5.0**2,    # w:        (変更なし)
    1.0 / 5.0**2,    # ∫e_pos:   (変更なし)
    1.0 / 1.0**2,    # ∫e_yaw:   (変更なし)
])

R1 = np.diag([
    1.0 / 0.3**2,    # v_pos:    速度要求も少し控えめの0.3m/sとする
    1.0 / 1.0**2,    # ω_yaw:    (変更なし)
])
# --- ここまで ---

# ============================================================
# 3. リカッチ方程式 → K → 閉ループ系
# ============================================================
P = solve_continuous_are(A11, A12, Q1, R1)
K = np.linalg.solve(R1, A12.T @ P)  # K = R⁻¹ A₁₂ᵀ P

A_cl = A11 - A12 @ K

print("=" * 60)
print("LQR 結果")
print("=" * 60)
print(f"\nK (2x6):\n{np.array2string(K, precision=4, suppress_small=True)}")

eigs = np.linalg.eigvals(A_cl)
print(f"\n閉ループ固有値:")
for i, e in enumerate(eigs):
    print(f"  λ{i} = {e:.4f}  (|Re| = {abs(e.real):.4f})")

# 等価制御のゲイン:
# σ̇ = ż₂ + K·ż₁ = (A₂₁ - A₂₂K + K(A₁₁ - A₁₂K))z₁ + B₂u = 0
# → u_eq = -B₂⁻¹ (A₂₁ - A₂₂K + K·A_cl) z₁
B2_inv = np.linalg.inv(B2)
K_eq = B2_inv @ (A21 - A22 @ K + K @ A_cl)

# ============================================================
# 4. シミュレーション
# ============================================================
T_SIM = 5.0   # シミュレーション時間 [s]
DT = 0.001    # 出力の時間刻み

# モータの最大トルク [Nm] (飽和チェック用)
TAU_MAX = 0.15

def simulate(z1_0, label):
    """面上の閉ループ系をシミュレーション"""
    def dynamics(t, z1):
        return A_cl @ z1

    t_span = (0, T_SIM)
    t_eval = np.arange(0, T_SIM, DT)

    sol = solve_ivp(dynamics, t_span, z1_0, t_eval=t_eval, method='RK45',
                    rtol=1e-9, atol=1e-12)

    t = sol.t
    z1 = sol.y  # (6, N)

    # z₂ = -K·z₁ (面上)
    z2 = -K @ z1  # (2, N)

    # 元の物理状態に復元: x_aug = T⁻¹ · [z₁; z₂]
    z_full = np.vstack([z1, z2])  # (8, N)
    x_aug = T_inv @ z_full  # (8, N)
    # x_aug: [x_pos, v_pos, θ_pitch, ω_pitch, ψ_yaw, ω_yaw, ∫e_pos, ∫e_yaw]

    # 等価制御トルク: u_eq = -B₂⁻¹ (A₂₁ - A₂₂K + K·A_cl) z₁
    u_eq = -K_eq @ z1  # (2, N)
    # u_eq = [τ_L, τ_R]

    return t, x_aug, u_eq, label


# --- シナリオ定義 ---
# z₁ = [x_pos, θ_pitch, ψ_yaw, w, ∫e_pos, ∫e_yaw]

scenarios = [
    (np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0]), "Pos step (x=0.5m)"),
    (np.array([0.0, 0.0, 0.5, 0.0, 0.0, 0.0]), "Yaw step (psi=0.5rad)"),
    (np.array([0.5, 0.0, 0.3, 0.0, 0.0, 0.0]), "Pos+Yaw combined"),
]

results = [simulate(z0, lbl) for z0, lbl in scenarios]

# ============================================================
# 5. プロット
# ============================================================
fig, axes = plt.subplots(4, len(results), figsize=(6 * len(results), 14),
                         sharex='col')
fig.suptitle("LQ-STSMC: Sliding Surface Dynamics (σ=0)", fontsize=14, y=0.98)

state_labels = ["x_pos [m]", "v_pos [m/s]", "θ_pitch [rad]", "ω_pitch [rad/s]",
                "ψ_yaw [rad]", "ω_yaw [rad/s]", "∫e_pos [m·s]", "∫e_yaw [rad·s]"]

for col, (t, x_aug, u_eq, label) in enumerate(results):
    # Row 0: 位置・ヨー (制御目標)
    ax = axes[0, col]
    ax.plot(t, x_aug[0], label="x_pos", linewidth=1.5)
    ax.plot(t, x_aug[4], label="ψ_yaw", linewidth=1.5)
    ax.axhline(0, color='gray', linestyle='--', linewidth=0.5)
    ax.set_ylabel("Position / Yaw")
    ax.set_title(label, fontsize=11)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Row 1: ピッチ (干渉チェック)
    ax = axes[1, col]
    ax.plot(t, np.degrees(x_aug[2]), label="θ_pitch [deg]", color='red', linewidth=1.5)
    ax.axhline(0, color='gray', linestyle='--', linewidth=0.5)
    ax.set_ylabel("Pitch [deg]")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Row 2: トルク (飽和チェック)
    ax = axes[2, col]
    ax.plot(t, u_eq[0], label="τ_L", linewidth=1.5)
    ax.plot(t, u_eq[1], label="τ_R", linewidth=1.5)
    ax.axhline(TAU_MAX, color='red', linestyle='--', linewidth=0.8, label=f"±{TAU_MAX} Nm")
    ax.axhline(-TAU_MAX, color='red', linestyle='--', linewidth=0.8)
    ax.set_ylabel("Torque [Nm]")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Row 3: 積分状態
    ax = axes[3, col]
    ax.plot(t, x_aug[6], label="∫e_pos", linewidth=1.5)
    ax.plot(t, x_aug[7], label="∫e_yaw", linewidth=1.5)
    ax.axhline(0, color='gray', linestyle='--', linewidth=0.5)
    ax.set_ylabel("Integral states")
    ax.set_xlabel("Time [s]")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("lq_stsmc_sim.png", dpi=150, bbox_inches='tight')
plt.close()

# ============================================================
# 6. 数値サマリー
# ============================================================
print("\n" + "=" * 60)
print("数値サマリー")
print("=" * 60)

for t, x_aug, u_eq, label in results:
    print(f"\n--- {label} ---")

    # トルク最大値
    tau_L_max = np.max(np.abs(u_eq[0]))
    tau_R_max = np.max(np.abs(u_eq[1]))
    print(f"  τ_L max: {tau_L_max:.4f} Nm  {'⚠ 飽和!' if tau_L_max > TAU_MAX else '✓ OK'}")
    print(f"  τ_R max: {tau_R_max:.4f} Nm  {'⚠ 飽和!' if tau_R_max > TAU_MAX else '✓ OK'}")

    # ピッチ最大偏角
    pitch_max_deg = np.max(np.abs(np.degrees(x_aug[2])))
    print(f"  θ_pitch max: {pitch_max_deg:.2f} deg")

    # 整定時間 (x_pos, ψ_yaw が初期値の5%以内に入った時刻)
    for idx, name, init_idx in [(0, "x_pos", 0), (4, "ψ_yaw", 2)]:
        x0_val = x_aug[idx, 0]
        if abs(x0_val) < 1e-6:
            print(f"  {name} 整定時間: N/A (初期値≈0)")
            continue
        threshold = abs(x0_val) * 0.05
        settled = np.where(np.abs(x_aug[idx]) < threshold)[0]
        if len(settled) > 0:
            # 最初に閾値以下になり、以降ずっと閾値以下の時刻を探す
            for j in range(len(settled)):
                if np.all(np.abs(x_aug[idx, settled[j]:]) < threshold):
                    print(f"  {name} 整定時間 (5%): {t[settled[j]]:.3f} s")
                    break
            else:
                print(f"  {name} 整定時間: > {T_SIM} s (未整定)")
        else:
            print(f"  {name} 整定時間: > {T_SIM} s (未整定)")

print(f"\n画像保存: lq_stsmc_sim.png")