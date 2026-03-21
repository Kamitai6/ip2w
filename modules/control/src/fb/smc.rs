/// Super Twisting Sliding Mode Controller (PID-like Architecture)
/// 
/// このコントローラは「誤差フィードバック」に特化しています。
/// 物理的な重力補償や目標加速度への先行入力(FF)は、外部で計算して足し合わせる運用を推奨します。
use libm::{sqrtf, tanhf};

const DELTA: f32 = 0.00001; // sqrtのゼロ除算防止
const V_LEAK: f32 = 0.00001; // 積分項のドリフト防止（リーキー積分）

pub struct SuperTwistingSMC {
    pub dt: f32,         // [s] 制御周期
    pub lambda: f32,     // [出力/√(rad/s)] STA比例ゲイン（P項に近い役割）
    pub alpha: f32,      // [出力/s] STA積分ゲイン（I項に近い役割）
    pub c: f32,          // [1/s] スライディング面の傾き (s = edot + c*e)
    pub b_inv: f32,      // [出力/(rad/s^2)] 入力ゲインの逆数 (1/b) ★重要
    pub epsilon: f32,    // [rad/s] 境界層幅 (tanhによる平滑化)
    pub v_limit: f32,    // [出力] 積分項(v)のリミット
    v: f32,              // [出力] 積分状態
    prev_err: f32,       // [rad] 前回の誤差（内部状態保持用）
}

impl SuperTwistingSMC {
    /// 新しいインスタンスを作成します。
    /// b_inv は「操作量を1入れた時に、加速度がどれくらい出るか」の逆数です。
    pub fn new(dt: f32, lambda: f32, alpha: f32, c: f32, b_inv: f32) -> Self {
        Self {
            dt,
            lambda,
            alpha,
            c,
            b_inv,
            epsilon: 0.1,
            v_limit: f32::INFINITY,
            v: 0.0,
            prev_err: 0.0,
        }
    }

    /// スムージング用の境界幅を設定（ビルダーパターン）
    pub fn with_smoothing(mut self, epsilon: f32) -> Self {
        self.epsilon = epsilon;
        self
    }

    /// 積分項のリミットを設定（ビルダーパターン）
    pub fn with_v_regulation(mut self, limit: f32) -> Self {
        self.v_limit = limit;
        self
    }

    /// 制御入力を計算します。
    /// err: target - current (位置偏差)
    /// err_dot: target_dot - current_dot (速度偏差)
    pub fn update(&mut self, err: f32, err_dot: f32) -> f32 {
        self.prev_err = err;

        // 1. スライディング面の定義: s = ė + c*e
        let s = err_dot + self.c * err;

        // 2. 切替関数の平滑化 (チャタリング防止)
        let sat_s = tanhf(s / self.epsilon);

        // 3. 等価制御入力(Ueq)のフィードバック成分 ★ここが追加ポイント
        // ė = -ce を維持するために必要な「モデルベースのFB項」
        // 理論式: u_eq_fb = (1/b) * (-c * ė)
        let u_eq_fb = self.b_inv * (-self.c * err_dot);

        // 4. STA非線形項の積分更新 (Uswの積分部)
        // 外乱やモデリング誤差をこの項が吸収する (PIDのI項に近い)
        self.v = ((1.0 - V_LEAK * self.dt) * self.v - self.alpha * sat_s * self.dt)
            .clamp(-self.v_limit, self.v_limit);

        // 5. STA非線形項の比例部 (Uswの比例部)
        // 偏差がスライディング面から離れるほど強く引き戻す
        let u_sw_p = -self.lambda * sqrtf(s.abs() + DELTA) * sat_s;

        // 全フィードバック操作量の合計
        let output = u_eq_fb + u_sw_p + self.v;

        output
    }

    /// 内部状態をリセットします。
    pub fn reset(&mut self) {
        self.v = 0.0;
        self.prev_err = 0.0;
    }
}