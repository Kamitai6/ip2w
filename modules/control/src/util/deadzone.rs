use core::f32::EPSILON;

pub fn apply_deadzone(input_value: f32, input_limit: f32, output_min: f32, output_max: f32) -> f32 {
    // 1. 入力を制限（安全装置）
    let input = input_value.clamp(-input_limit, input_limit);

    // 2. ほぼ0なら、計算誤差が出ないように完全に0にする
    if input.abs() < EPSILON {
        return 0.0;
    }

    // 3. 比率を計算 (0.0 〜 1.0 の範囲になるので、絶対に桁あふれしない)
    let ratio = input.abs() / input_limit;

    // 4. マッピング (線形補間)
    let output_f32 = output_min + (ratio * (output_max - output_min));

    // 5. 符号を復元
    if input > 0.0 {
        output_f32
    } else {
        -output_f32
    }
}