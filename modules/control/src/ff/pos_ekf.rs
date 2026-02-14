use libm::{sinf, tanhf};

/// 並進位置推定EKF + レギュレータ（v3: trust連続制御 + k0回帰 + joseph更新）
///
/// 状態: [v, x, a, b, k]
///   v:  並進速度 [m/s]
///   x:  並進位置 [m]
///   a:  真の並進加速度（内部状態） [m/s^2]
///   b:  観測側合成バイアス（重力漏れ/加速度計バイアス等をまとめる） [m/s^2]
///   k:  command→加速度係数 推定値 [m/s^2 / PWM]
///
/// 観測:
///   z = a_meas = -(ax_g + sin(pitch)) * 9.81
///   h = a + b
///   H = [0, 0, 1, 1, 0]
///
/// 重要:
///   - b は固定強リークしない。信頼度(trust)で「リーク量」と「追従性(Qb)」を連続制御。
///   - k はユーザ指定の k0(=cfg.k0) へ戻るリーク（平均回帰）を入れる。
///   - あり得ない jerk（観測の形）→ Rを増やしてIMU観測を弱める（連続的）。
///   - 残差の“持続”と“符号反転の多さ”で b の信頼度を作る（イベントではない）。
pub struct PosEkfConfig {
    pub dt: f32,

    /// k の公称値（ユーザが決める） [m/s^2 / PWM]
    pub k0: f32,

    /// a が k*command に追従する時定数 [s]
    pub tau_a: f32,

    /// a の jerk 上限（状態更新のクリップ） [m/s^3]
    pub j_max: f32,

    /// k を k0 に戻す時定数 [s]
    pub tau_k: f32,

    /// 観測ノイズ（ベース） [m^2/s^4]
    pub r_accel: f32,

    /// jerkでRを膨らませる強さ（0以上）
    pub jerk_r_scale: f32,

    /// a のプロセスノイズ（モデル誤差吸収） [m^2/s^5] 相当
    pub q_a: f32,

    /// v のプロセスノイズ（保険） [m^2/s^3]
    pub q_v: f32,

    /// k のプロセスノイズ（励起があるときだけ効かせる想定） [(m/s^2/PWM)^2 / s]
    pub q_k: f32,

    /// k の励起判定スケール（du/(du+u_scale)）の u_scale [PWM]
    pub u_scale_for_k: f32,

    /// b のプロセスノイズ上限（trust=1での強さ） [m^2/s^5]
    pub q_b_max: f32,

    /// b のプロセスノイズ下限（trust=0での下限） [m^2/s^5]
    pub q_b_min: f32,

    /// b のリーク時定数（trust=0→min, trust=1→max） [s]
    pub tau_b_min: f32,
    pub tau_b_max: f32,

    /// 残差LPF（持続判定用） 0..1（大きいほど速い）
    pub innov_lpf_alpha: f32,

    /// 符号反転LPF（反転が多いほどtrustを下げる） 0..1
    pub flip_lpf_alpha: f32,

    /// |LPF残差| を 0..1 に正規化するスケール [m/s^2]
    pub bias_residual_scale: f32,

    /// 位置→角度オフセットゲイン [rad/m]
    pub k_pos: f32,

    /// 速度→角度オフセットゲイン [rad/(m/s)]
    pub k_vel: f32,

    /// 最大角度オフセット [rad]
    pub max_output: f32,

    /// 回転軸からセンサーまでの距離 [m]
    pub robot_radius: f32,

    /// 想定最大並進加速度（ソフトリミット閾値） [m/s²]
    pub a_max: f32,

    /// 想定最大並進速度（ソフトリミット閾値） [m/s]
    pub v_max: f32,

    /// 加速度超過時のR膨張強度
    pub accel_r_scale: f32,

    /// 速度超過時のR膨張強度
    pub vel_r_scale: f32,
}

impl Default for PosEkfConfig {
    fn default() -> Self {
        Self {
            dt: 0.002,

            k0: 0.003,

            tau_a: 0.02,
            j_max: 50.0,

            tau_k: 10.0,

            r_accel: 1.0,
            jerk_r_scale: 2.0,

            q_a: 5e-3,
            q_v: 1e-5,
            q_k: 1e-8,
            u_scale_for_k: 50.0,

            q_b_max: 1e-6,
            q_b_min: 1e-10,

            tau_b_min: 1.0,
            tau_b_max: 100.0,

            innov_lpf_alpha: 0.05,
            flip_lpf_alpha: 0.05,
            bias_residual_scale: 0.5,

            k_pos: 0.1,
            k_vel: 0.2,
            max_output: 0.05,

            robot_radius: 0.08,
            a_max: 4.0,
            v_max: 0.4,
            accel_r_scale: 2.0,
            vel_r_scale: 2.0,
        }
    }
}

/// デバッグ用
#[derive(Clone, Copy)]
pub struct PosEkfState {
    pub velocity: f32,
    pub position: f32,
    pub accel: f32,
    pub bias: f32,
    pub k_est: f32,

    pub innovation: f32,
    pub innov_lp: f32,
    pub trust: f32,

    pub r_eff: f32,
    pub jerk_meas: f32,

    pub a_tangential: f32,

    // 追加デバッグ情報
    pub a_meas: f32,         // 観測加速度（接線補正後）
    pub a_raw: f32,          // 観測加速度（接線補正前）
    pub pitch: f32,          // 入力ピッチ角
    pub command: f32,        // 入力コマンド
    pub g_times_pitch: f32,  // g × pitch（物理的期待値の目安）
}

pub struct PositionEkf {
    // 状態
    v: f32,
    x: f32,
    a: f32,
    b: f32,
    k: f32,

    // 共分散（フル 5x5）
    p: [[f32; 5]; 5],

    // 連続信頼度
    trust: f32,

    // 残差履歴
    innovation: f32,
    innov_lp: f32,
    prev_innov_lp: f32,
    flip_lp: f32,

    // 観測履歴
    prev_a_raw: f32,
    prev_command: f32,

    // デバッグ
    r_eff: f32,
    jerk_meas: f32,
    a_tangential: f32,

    // 追加デバッグ情報
    a_meas: f32,
    a_raw: f32,
    pitch: f32,
    command: f32,

    cfg: PosEkfConfig,
}

impl PositionEkf {
    pub fn new(cfg: PosEkfConfig) -> Self {
        let mut p = [[0.0f32; 5]; 5];
        // 初期共分散（雑に安全側）
        p[0][0] = 0.01;  // v
        p[1][1] = 0.01;  // x
        p[2][2] = 0.5;   // a
        p[3][3] = 0.1;   // b
        p[4][4] = 1e-6;  // k

        Self {
            v: 0.0,
            x: 0.0,
            a: 0.0,
            b: 0.0,
            k: cfg.k0,

            p,

            trust: 0.0,

            innovation: 0.0,
            innov_lp: 0.0,
            prev_innov_lp: 0.0,
            flip_lp: 0.0,

            prev_a_raw: 0.0,
            prev_command: 0.0,

            r_eff: cfg.r_accel,
            jerk_meas: 0.0,
            a_tangential: 0.0,

            // 追加デバッグ情報
            a_meas: 0.0,
            a_raw: 0.0,
            pitch: 0.0,
            command: 0.0,

            cfg,
        }
    }

    #[inline(always)]
    fn clamp(x: f32, lo: f32, hi: f32) -> f32 {
        if x < lo { lo } else if x > hi { hi } else { x }
    }

    #[inline(always)]
    fn lerp(a: f32, b: f32, t: f32) -> f32 {
        a + (b - a) * t
    }

    fn symmetrize(p: &mut [[f32; 5]; 5]) {
        for i in 0..5 {
            for j in (i + 1)..5 {
                let v = 0.5 * (p[i][j] + p[j][i]);
                p[i][j] = v;
                p[j][i] = v;
            }
            if p[i][i] < 0.0 {
                p[i][i] = 0.0;
            }
        }
    }

    /// 更新
    ///
    /// * command       : total_output [PWM]（デッドゾーン前）
    /// * ax_g          : LPF後の加速度計X [g]（Z-up計算座標系）
    /// * pitch         : MEKFピッチ角 [rad]
    /// * pitch_rate    : MEKFピッチ角速度 [rad/s]
    /// * angular_accel : DKF推定角加速度 [rad/s²]
    ///
    /// 戻り値: 角度オフセット [rad]
    pub fn update(&mut self, command: f32, ax_g: f32, pitch: f32,
                  _pitch_rate: f32, angular_accel: f32) -> f32 {
        let dt = self.cfg.dt;

        // ───── 観測（加速度） ─────
        // 重力除去
        let a_raw = (ax_g + sinf(pitch)) * 9.81;

        // 接線加速度補正: IMUオフセット位置での回転起因成分を除去
        // ボディax = ẍ_trans·cos(θ) + α·L  (遠心力項はボディフレームでキャンセル)
        // |θ|が小さい倒立振子では cos(θ)≈1 なので ẍ_trans ≈ a_raw + α·L
        // (符号は座標系依存: 実機検証済み)
        let a_tangential = angular_accel * self.cfg.robot_radius;
        let a_meas = a_raw + a_tangential;
        self.a_tangential = a_tangential;

        // デバッグ情報保存
        self.a_meas = a_meas;
        self.a_raw = a_raw;
        self.pitch = pitch;
        self.command = command;

        // Jerk（加加速度）チェック - a_rawベースで計算
        // 接線加速度補正由来の急変はノイズではないため、補正前のa_rawで判定
        let j_meas = (a_raw - self.prev_a_raw) / dt;
        self.jerk_meas = j_meas;
        let jerk_ratio = j_meas.abs() / (self.cfg.j_max + 1e-6);

        // 加速度・速度のソフトリミット比（推定値ベース）
        let a_ratio = self.a.abs() / (self.cfg.a_max + 1e-6);
        let v_ratio = self.v.abs() / (self.cfg.v_max + 1e-6);

        let r_scale = 1.0
            + self.cfg.jerk_r_scale * (jerk_ratio * jerk_ratio)
            + self.cfg.accel_r_scale * (a_ratio * a_ratio)
            + self.cfg.vel_r_scale * (v_ratio * v_ratio);
        self.r_eff = self.cfg.r_accel * r_scale;

        // ───── trust（前回）から b のリーク・Q を作る ─────
        // trust=0 → 強リーク + 小Q（暴走防止）
        // trust=1 → 弱リーク + 大Q（追従）
        let tau_b = Self::lerp(self.cfg.tau_b_min, self.cfg.tau_b_max, self.trust);
        let beta_b = dt / (tau_b + dt + 1e-6); // 離散一次遅れの係数（0..1に収まる形）
        let psi_b = 1.0 - beta_b;
        let q_b = Self::lerp(self.cfg.q_b_min, self.cfg.q_b_max, self.trust);

        // k の励起（duが大きいほどkを動かしやすくする）
        let du = (command - self.prev_command).abs();
        let c_u = du / (du + self.cfg.u_scale_for_k);
        let q_k = self.cfg.q_k * c_u;

        // ───── 状態予測 ─────
        // k: k0 への平均回帰
        let gamma_k = Self::clamp(dt / (self.cfg.tau_k + dt + 1e-6), 0.0, 1.0);
        let eta_k = 1.0 - gamma_k;
        self.k = eta_k * self.k + gamma_k * self.cfg.k0;

        // a: 1次遅れで a_target=k*command へ追従（jerkでクリップ）
        let a_target = self.k * command;
        let alpha_a_nom = Self::clamp(dt / (self.cfg.tau_a + dt + 1e-6), 0.0, 1.0);
        let mut da = alpha_a_nom * (a_target - self.a);
        let da_max = self.cfg.j_max * dt;
        da = Self::clamp(da, -da_max, da_max);

        // クリップ後の“実効alpha”を使って線形化を合わせる（重要）
        let denom = (a_target - self.a);
        let alpha_a = if denom.abs() > 1e-9 { (da / denom).abs() } else { 0.0 };
        let alpha_a = Self::clamp(alpha_a, 0.0, alpha_a_nom);

        self.a += da;

        // v, x: aを積分
        self.v += self.a * dt;
        self.x += self.v * dt;

        // b: trustに応じたリーク（平均回帰先は0。必要なら基準値へ戻す形に変更可）
        self.b *= psi_b;

        // ───── 共分散予測 ─────
        // 状態 s=[v,x,a,b,k]
        // 更新:
        //   a' = phi*a + alpha_cmd*k      (phi=1-alpha_a, alpha_cmd=alpha_a*command)
        //   v' = v + dt*a'
        //   x' = x + dt*v' = x + dt*v + dt^2*a'
        //   b' = psi_b*b
        //   k' = eta_k*k + const
        let phi = 1.0 - alpha_a;
        let alpha_cmd = alpha_a * command;

        let mut f = [[0.0f32; 5]; 5];
        // v'
        f[0][0] = 1.0;
        f[0][2] = dt * phi;
        f[0][4] = dt * alpha_cmd;
        // x'
        f[1][0] = dt;
        f[1][1] = 1.0;
        f[1][2] = dt * dt * phi;
        f[1][4] = dt * dt * alpha_cmd;
        // a'
        f[2][2] = phi;
        f[2][4] = alpha_cmd;
        // b'
        f[3][3] = psi_b;
        // k'
        f[4][4] = eta_k;

        // Q（対角のみ。単位は厳密でなくても「保険として正しい方向」に効く形）
        let mut q = [[0.0f32; 5]; 5];
        q[0][0] = self.cfg.q_v * dt;
        q[2][2] = self.cfg.q_a * dt;
        q[3][3] = q_b * dt;
        q[4][4] = q_k * dt;

        // P = F P F^T + Q
        let p0 = self.p;
        let mut fp = [[0.0f32; 5]; 5];
        for i in 0..5 {
            for j in 0..5 {
                let mut s = 0.0;
                for k in 0..5 {
                    s += f[i][k] * p0[k][j];
                }
                fp[i][j] = s;
            }
        }
        let mut p_pred = [[0.0f32; 5]; 5];
        for i in 0..5 {
            for j in 0..5 {
                let mut s = 0.0;
                for k in 0..5 {
                    s += fp[i][k] * f[j][k]; // *F^T
                }
                p_pred[i][j] = s + q[i][j];
            }
        }
        Self::symmetrize(&mut p_pred);
        self.p = p_pred;

        // ───── 観測更新 ─────
        // z = a_meas
        // h = a + b
        self.innovation = a_meas - (self.a + self.b);

        // 残差LPF（持続性）
        self.prev_innov_lp = self.innov_lp;
        self.innov_lp += self.cfg.innov_lpf_alpha * (self.innovation - self.innov_lp);

        // 符号反転度（多いほど「ノイズっぽい」）
        let flipped = if (self.innov_lp * self.prev_innov_lp) < 0.0 { 1.0 } else { 0.0 };
        self.flip_lp += self.cfg.flip_lpf_alpha * (flipped - self.flip_lp);
        let c_flip = 1.0 - Self::clamp(self.flip_lp, 0.0, 1.0);

        // trust更新（次ステップに効かせる）
        // - jerkが大きいほどtrust↓（観測の形が怪しい）
        // - 残差LPFが大きいほどtrust↑（持続誤差=バイアスっぽい）
        // - 符号反転が多いほどtrust↓
        // - 加速度/速度が物理上限に近いほどtrust↓（推定値が怪しい）
        let c_j = 1.0 / (1.0 + jerk_ratio * jerk_ratio);
        let c_r = Self::clamp(self.innov_lp.abs() / (self.cfg.bias_residual_scale + 1e-6), 0.0, 1.0);
        let c_a = 1.0 / (1.0 + a_ratio * a_ratio);
        let c_v = 1.0 / (1.0 + v_ratio * v_ratio);
        let trust_next = Self::clamp(c_j * c_r * c_flip * c_a * c_v, 0.0, 1.0);

        // H=[0,0,1,1,0]
        // S = Paa + Pbb + 2*Pab + R
        let paa = self.p[2][2];
        let pbb = self.p[3][3];
        let pab = self.p[2][3];
        let s = paa + pbb + 2.0 * pab + self.r_eff;
        let s_inv = 1.0 / (s + 1e-9);

        // K = P H^T / S = (P[:,a] + P[:,b]) / S
        let mut k_gain = [0.0f32; 5];
        for i in 0..5 {
            k_gain[i] = (self.p[i][2] + self.p[i][3]) * s_inv;
        }

        // 状態更新: x += K * innovation
        self.v += k_gain[0] * self.innovation;
        self.x += k_gain[1] * self.innovation;
        self.a += k_gain[2] * self.innovation;
        self.b += k_gain[3] * self.innovation;
        self.k += k_gain[4] * self.innovation;

        // Joseph形式: P = (I-KH) P (I-KH)^T + K R K^T
        // H は a,b のみに 1。よって KH の列2,3が K、他0。
        let mut a_mat = [[0.0f32; 5]; 5];
        for i in 0..5 {
            a_mat[i][i] = 1.0;
            a_mat[i][2] -= k_gain[i];
            a_mat[i][3] -= k_gain[i];
        }

        let p_before = self.p;

        let mut ap = [[0.0f32; 5]; 5];
        for i in 0..5 {
            for j in 0..5 {
                let mut ss = 0.0;
                for k in 0..5 {
                    ss += a_mat[i][k] * p_before[k][j];
                }
                ap[i][j] = ss;
            }
        }

        let mut p_new = [[0.0f32; 5]; 5];
        for i in 0..5 {
            for j in 0..5 {
                let mut ss = 0.0;
                for k in 0..5 {
                    ss += ap[i][k] * a_mat[j][k];
                }
                p_new[i][j] = ss;
            }
        }

        // + K R K^T
        for i in 0..5 {
            for j in 0..5 {
                p_new[i][j] += k_gain[i] * self.r_eff * k_gain[j];
            }
        }

        Self::symmetrize(&mut p_new);
        self.p = p_new;

        // trustを更新（連続制御）
        self.trust = trust_next;

        // 履歴更新
        self.prev_a_raw = a_raw;
        self.prev_command = command;

        // ───── レギュレータ出力 ─────
        let raw = self.cfg.k_pos * self.x + self.cfg.k_vel * self.v;
        self.cfg.max_output * tanhf(raw / self.cfg.max_output)
    }

    pub fn state(&self) -> PosEkfState {
        PosEkfState {
            velocity: self.v,
            position: self.x,
            accel: self.a,
            bias: self.b,
            k_est: self.k,

            innovation: self.innovation,
            innov_lp: self.innov_lp,
            trust: self.trust,

            r_eff: self.r_eff,
            jerk_meas: self.jerk_meas,

            a_tangential: self.a_tangential,

            // 追加デバッグ情報
            a_meas: self.a_meas,
            a_raw: self.a_raw,
            pitch: self.pitch,
            command: self.command,
            g_times_pitch: 9.81 * self.pitch,
        }
    }

    pub fn reset(&mut self) {
        self.v = 0.0;
        self.x = 0.0;
        self.a = 0.0;
        self.b = 0.0;
        self.k = self.cfg.k0;

        self.p = [[0.0; 5]; 5];
        self.p[0][0] = 0.01;
        self.p[1][1] = 0.01;
        self.p[2][2] = 0.5;
        self.p[3][3] = 0.1;
        self.p[4][4] = 1e-6;

        self.trust = 0.0;

        self.innovation = 0.0;
        self.innov_lp = 0.0;
        self.prev_innov_lp = 0.0;
        self.flip_lp = 0.0;

        self.prev_a_raw = 0.0;
        self.prev_command = 0.0;

        self.r_eff = self.cfg.r_accel;
        self.jerk_meas = 0.0;
        self.a_tangential = 0.0;

        // 追加デバッグ情報
        self.a_meas = 0.0;
        self.a_raw = 0.0;
        self.pitch = 0.0;
        self.command = 0.0;
    }
}