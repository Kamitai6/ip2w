import re
import matplotlib.pyplot as plt

# ==========================================
# 1. ここに Rust の udp_println! コードをコピペ
# ==========================================
rust_code = """
udp_println!(
    "{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},",
    p_state.position, p_state.velocity,
    p_state.a_target, p_state.a_meas,
    pure_total_pitch_torque, smd_angular_accel,
    now_angle, state.pitch_rate,
    m1_out, m2_out, 
    m1_pwm, m2_pwm,
);
"""

# ==========================================
# 2. ここに ログデータ をコピペ（またはファイルから読み込み）
# ==========================================
log_data = """
[192.168.4.1]: 0.001,0.043,3.663,-0.819,0.001,-10.011,0.002,-0.269,40.914,-35.326,-40,35,
[192.168.4.1]: 0.009,0.102,-0.273,0.420,0.008,9.789,0.010,0.100,49.579,-44.077,-49,44,
[192.168.4.1]: 0.029,0.242,2.316,0.363,0.004,0.989,-0.015,0.007,44.473,-38.750,-44,38,
[192.168.4.1]: 0.069,0.426,1.530,0.092,0.004,1.580,-0.012,0.026,45.369,-39.535,-45,39,
[192.168.4.1]: 0.126,0.612,1.698,-0.005,0.005,0.513,-0.012,0.048,46.511,-40.705,-46,40,
[192.168.4.1]: 0.203,0.809,2.156,-0.077,0.005,-0.433,-0.011,0.047,46.690,-40.914,-46,40,
[192.168.4.1]: 0.309,1.064,2.564,0.168,0.007,1.285,-0.009,0.083,48.978,-43.166,-48,43,
[192.168.4.1]: 0.439,1.325,3.445,-0.150,0.010,-1.076,-0.007,0.138,52.280,-46.456,-52,46,
[192.168.4.1]: 0.583,1.555,4.739,0.402,0.013,0.798,-0.005,0.195,55.980,-50.161,-55,50,
[192.168.4.1]: 0.750,1.790,6.492,-0.124,0.017,-0.432,0.003,0.263,60.915,-55.203,-60,55,
[192.168.4.1]: 0.941,2.030,9.742,0.382,0.028,5.176,0.039,0.487,75.639,-70.180,-75,70,
[192.168.4.1]: 1.156,2.253,14.949,-0.745,0.042,2.209,0.090,0.748,94.294,-88.440,-94,88,
[192.168.4.1]: 1.391,2.460,21.975,-1.817,0.058,0.009,0.146,1.018,114.517,-108.837,-114,108,
[192.168.4.1]: 1.648,2.666,35.492,0.074,0.105,17.756,0.245,2.043,175.246,-169.709,-127,127,
"""

def parse_rust_vars(code_str):
    """Rustのコード片から変数名のリストを抽出する"""
    # udp_println!( ... ) の中身を抽出
    match = re.search(r'udp_println!\s*\((.*?)\);', code_str, re.DOTALL)
    if not match:
        print("Rustコードのパースに失敗しました。フォーマットを確認してください。")
        return []
    
    args_str = match.group(1)
    
    # 最初の引数であるフォーマット文字列（ "{...}" ）を削除
    args_str = re.sub(r'".*?"\s*,', '', args_str, count=1, flags=re.DOTALL)
    
    # 残りの文字列をカンマで分割し、空白と改行を取り除く
    var_names = [v.strip() for v in args_str.split(',') if v.strip()]
    return var_names

def plot_dynamic_log(rust_code, log_data):
    # 1. 変数名の抽出
    var_names = parse_rust_vars(rust_code)
    num_vars = len(var_names)
    
    if num_vars == 0:
        return
    
    print(f"抽出された変数 ({num_vars}個):", var_names)

    # 2. データの格納用辞書を準備
    data_dict = {name: [] for name in var_names}

    # 3. ログのパース
    for line in log_data.strip().split('\n'):
        if "]:" in line:
            # IPアドレス等のヘッダを分離し、カンマで分割
            parts = line.split("]:")[1].split(',')
            parts = [p.strip() for p in parts if p.strip()]
            
            # 変数の数だけデータが揃っている行のみ処理
            if len(parts) >= num_vars:
                try:
                    vals = [float(p) for p in parts[:num_vars]]
                    for i, name in enumerate(var_names):
                        data_dict[name].append(vals[i])
                except ValueError:
                    continue # 数値変換エラーはスキップ

    time_steps = range(len(data_dict[var_names[0]]))
    if len(time_steps) == 0:
        print("有効なデータ行が見つかりませんでした。")
        return

    # 4. グラフの動的描画（変数の数だけ縦に並べる）
    fig, axs = plt.subplots(num_vars, 1, figsize=(10, 2 * num_vars), sharex=True)
    if num_vars == 1:
        axs = [axs] # 1要素でもリスト化して扱いを統一
        
    for i, name in enumerate(var_names):
        axs[i].plot(time_steps, data_dict[name], marker='.', label=name)
        axs[i].set_ylabel(name, rotation=0, labelpad=40, ha='right') # Y軸ラベルを見やすく
        axs[i].legend(loc='upper right')
        axs[i].grid(True, alpha=0.3)

    axs[-1].set_xlabel('Sample Step')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_dynamic_log(rust_code, log_data)