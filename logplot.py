import re
import matplotlib.pyplot as plt

# ==========================================
# 1. ここに Rust の udp_println! コードをコピペ
# ==========================================
rust_code = """
udp_println!(
    "{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},", 
    m1_out, m2_out,
    p_state.position, p_state.velocity,
    now_angle, state.pitch_rate,
);
"""

# ==========================================
# 2. ここに ログデータ をコピペ（またはファイルから読み込み）
# ==========================================
log_data = """
[192.168.4.1]: -53.346,48.322,-0.000,-0.038,-0.050,-0.062,
[192.168.4.1]: -60.259,55.120,-0.003,-0.052,-0.043,-0.056,
[192.168.4.1]: -48.216,43.281,-0.007,-0.025,-0.018,0.234,
[192.168.4.1]: -42.630,37.655,-0.009,-0.014,-0.002,0.292,
[192.168.4.1]: 50.288,-45.329,-0.005,0.034,0.025,0.142,
[192.168.4.1]: -35.396,30.367,-0.003,-0.008,0.023,0.651,
[192.168.4.1]: 55.320,-50.427,0.001,0.033,0.086,0.440,
[192.168.4.1]: 79.287,-74.455,0.006,0.067,0.147,1.197,
[192.168.4.1]: 106.778,-101.916,0.014,0.098,0.266,2.411,
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