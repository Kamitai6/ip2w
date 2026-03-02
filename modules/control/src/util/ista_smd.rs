use libm::sqrtf;

/// 陰的離散化によるスライディングモード微分器 (ISTA-STSMD)
pub struct ImplicitSmd {
    z0: f32,       // 信号の推定値 (角速度)
    z1: f32,       // 微分信号の推定値 (角加速度)
    lambda1: f32,  // ゲイン1 (誤差の平方根に対するゲイン)
    lambda2: f32,  // ゲイン2 (誤差の符号に対するゲイン)
    h: f32,        // サンプリング周期 [s]
}

impl ImplicitSmd {
    /// 微分器の初期化
    pub fn new(lambda1: f32, lambda2: f32, h: f32) -> Self {
        Self {
            z0: 0.0,
            z1: 0.0,
            lambda1,
            lambda2,
            h,
        }
    }

    /// 起動直後のスパイクを防ぐため、初回の観測値で内部状態を初期化する
    pub fn init_state(&mut self, initial_f: f32) {
        self.z0 = initial_f;
        self.z1 = 0.0;
    }

    /// ジャイロの新しい観測値を入力し、角加速度の推定値を返す
    pub fn update(&mut self, f_next: f32) -> f32 {
        // Step 1: 仮想誤差の計算
        let e_tilde = self.z0 - f_next + self.h * self.z1;

        // 判定用の閾値
        let h2_l2 = self.h * self.h * self.lambda2;

        let e_next: f32;
        let xi: f32;

        // Step 2: スライディング面への到達判定と射影
        if e_tilde.abs() <= h2_l2 {
            // Case A: 突き抜ける場合（スライディング面に完全に固定する）
            e_next = 0.0;
            xi = e_tilde / h2_l2;
        } else {
            // Case B: まだ届かない場合
            // 数学的な sgn(e) の挙動を定義 (Rustの f32::signum は 0.0 のとき 1.0 になるため)
            let sign_e = if e_tilde > 0.0 {
                1.0
            } else if e_tilde < 0.0 {
                -1.0
            } else {
                0.0
            };

            // 解析解の平方根の中身
            let inside_sqrt = self.h * self.h * self.lambda1 * self.lambda1 + 4.0 * (e_tilde.abs() - h2_l2);

            let sqrt_val = sqrtf(inside_sqrt);

            let v = (-self.h * self.lambda1 + sqrt_val) / 2.0;

            e_next = sign_e * v * v;
            xi = sign_e;
        }

        // Step 3: 状態の更新
        self.z1 = self.z1 - self.h * self.lambda2 * xi;
        self.z0 = f_next + e_next;

        // 求まった角加速度を返す
        self.z1
    }

    pub fn get_z0(&self) -> f32 {
        self.z0
    }
}