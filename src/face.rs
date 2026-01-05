use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, Ellipse, PrimitiveStyle},
};

/// 感情ステート
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum Emotion {
    #[default]
    Neutral,  // 普通
    Excited,  // 元気・ワクワク
    Relaxed,  // 安心・眠い
    Fear,     // 恐怖
    Impact,   // 衝撃・驚き
    Confused, // 困惑
}

/// 目の形状パラメータ（感情で変化する部分）
#[derive(Clone, Copy)]
struct EyeShape {
    openness: f32,
    pupil_scale: f32,
    motion_speed: f32,
    motion_amplitude: f32,
    show_highlight: bool,
}

impl EyeShape {
    fn from_emotion(emotion: Emotion) -> Self {
        match emotion {
            Emotion::Neutral => Self {
                openness: 1.0,
                pupil_scale: 1.0,
                motion_speed: 1.0,
                motion_amplitude: 1.0,
                show_highlight: true,
            },
            Emotion::Excited => Self {
                openness: 1.2,
                pupil_scale: 0.9,
                motion_speed: 2.5,
                motion_amplitude: 1.3,
                show_highlight: true,
            },
            Emotion::Relaxed => Self {
                openness: 0.5,
                pupil_scale: 1.0,
                motion_speed: 0.3,
                motion_amplitude: 0.5,
                show_highlight: true,
            },
            Emotion::Fear => Self {
                openness: 1.3,
                pupil_scale: 0.5,
                motion_speed: 3.0,
                motion_amplitude: 0.3,
                show_highlight: false,
            },
            Emotion::Impact => Self {
                openness: 1.4,
                pupil_scale: 0.4,
                motion_speed: 0.0,
                motion_amplitude: 0.0,
                show_highlight: true,
            },
            Emotion::Confused => Self {
                openness: 0.9,
                pupil_scale: 1.1,
                motion_speed: 0.8,
                motion_amplitude: 1.2,
                show_highlight: true,
            },
        }
    }
}

// ============================================================
// 使用例
// ============================================================
// ```rust
// let mut face = Face::new(Point::new(120, 120), 50, 20);
//
// loop {
//     let dt = 0.002; // 500Hz = 2ms
//
//     face.set_emotion(Emotion::Excited);
//
//     display.clear(Rgb565::BLACK).unwrap();
//     face.update(&mut display, dt).unwrap(); // これだけでOK
// }
// ```
/// 顔（両目）を管理する構造体
pub struct Face {
    center: Point,
    eye_spacing: i32,
    eye_radius: u32,

    emotion: Emotion,
    shape: EyeShape,

    phase: f32,
    blink_timer: f32,
    is_blinking: bool,
    blink_progress: f32,
    confused_offset: f32,
}

impl Face {
    pub fn new(center: Point, eye_spacing: i32, eye_radius: u32) -> Self {
        Self {
            center,
            eye_spacing,
            eye_radius,
            emotion: Emotion::Neutral,
            shape: EyeShape::from_emotion(Emotion::Neutral),
            phase: 0.0,
            blink_timer: 0.0,
            is_blinking: false,
            blink_progress: 0.0,
            confused_offset: 0.0,
        }
    }

    pub fn set_emotion(&mut self, emotion: Emotion) {
        if self.emotion != emotion {
            self.emotion = emotion;
            self.shape = EyeShape::from_emotion(emotion);
            self.phase = 0.0;
        }
    }

    pub fn emotion(&self) -> Emotion {
        self.emotion
    }

    /// 更新＆描画（毎フレーム呼ぶ）
    pub fn update<D>(&mut self, target: &mut D, dt: f32) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        // === 状態更新 ===
        const TWO_PI: f32 = core::f32::consts::PI * 2.0;
        const BASE_SPEED: f32 = 2.0;

        self.phase += dt * BASE_SPEED * self.shape.motion_speed;
        if self.phase > TWO_PI {
            self.phase -= TWO_PI;
        }

        if self.emotion == Emotion::Confused {
            self.confused_offset += dt * 1.5;
            if self.confused_offset > TWO_PI {
                self.confused_offset -= TWO_PI;
            }
        }

        self.update_blink(dt);

        // === 描画 ===
        let left_center = Point::new(
            self.center.x - self.eye_spacing / 2,
            self.center.y,
        );
        let right_center = Point::new(
            self.center.x + self.eye_spacing / 2,
            self.center.y,
        );

        let (left_offset, right_offset) = self.calculate_pupil_offsets();

        self.draw_eye(target, left_center, left_offset)?;
        self.draw_eye(target, right_center, right_offset)?;

        Ok(())
    }

    fn update_blink(&mut self, dt: f32) {
        const BLINK_INTERVAL: f32 = 3.5;
        const BLINK_DURATION: f32 = 0.15;

        if self.is_blinking {
            self.blink_progress += dt / BLINK_DURATION;
            if self.blink_progress >= 1.0 {
                self.is_blinking = false;
                self.blink_progress = 0.0;
            }
        } else {
            self.blink_timer += dt;
            let trigger = BLINK_INTERVAL + libm::sinf(self.phase * 7.0) * 1.0;
            if self.blink_timer > trigger {
                self.is_blinking = true;
                self.blink_timer = 0.0;
            }
        }
    }

    fn calculate_pupil_offsets(&self) -> (Point, Point) {
        let base_amplitude = (self.eye_radius as f32 * 0.4) * self.shape.motion_amplitude;
        let sin_phase = libm::sinf(self.phase);

        let mut left_dx = (sin_phase * base_amplitude) as i32;
        let mut left_dy = 0;
        let mut right_dx = left_dx;
        let mut right_dy = 0;

        if self.emotion == Emotion::Confused {
            let sin_confused = libm::sinf(self.phase + self.confused_offset);
            let cos_confused = libm::cosf(self.phase * 0.7);
            right_dx = (sin_confused * base_amplitude) as i32;
            right_dy = (cos_confused * base_amplitude * 0.5) as i32;
            left_dy = (libm::sinf(self.phase * 1.3) * base_amplitude * 0.3) as i32;
        }

        (Point::new(left_dx, left_dy), Point::new(right_dx, right_dy))
    }

    fn draw_eye<D>(
        &self,
        target: &mut D,
        center: Point,
        pupil_offset: Point,
    ) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let radius = self.eye_radius;

        let mut openness = self.shape.openness;
        if self.is_blinking {
            let blink_curve = libm::sinf(self.blink_progress * core::f32::consts::PI);
            openness *= 1.0 - blink_curve * 0.9;
        }
        openness = openness.clamp(0.1, 1.5);

        let height = ((radius * 2) as f32 * openness) as u32;
        let width = radius * 2;

        // 白目
        let eye_top_left = Point::new(
            center.x - radius as i32,
            center.y - (height / 2) as i32,
        );
        Ellipse::new(eye_top_left, Size::new(width, height))
            .into_styled(PrimitiveStyle::with_fill(Rgb565::WHITE))
            .draw(target)?;

        if openness < 0.3 {
            return Ok(());
        }

        // 黒目
        let pupil_radius = ((radius / 3) as f32 * self.shape.pupil_scale) as u32;
        let pupil_radius = pupil_radius.max(2);

        let max_offset = (radius - pupil_radius - 2) as i32;
        let clamped_offset = Point::new(
            pupil_offset.x.clamp(-max_offset, max_offset),
            pupil_offset.y.clamp(-max_offset, max_offset),
        );

        Circle::with_center(center + clamped_offset, pupil_radius * 2)
            .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
            .draw(target)?;

        // ハイライト
        if self.shape.show_highlight && pupil_radius > 3 {
            let highlight_offset = Point::new(
                clamped_offset.x - (pupil_radius as i32 / 3),
                clamped_offset.y - (pupil_radius as i32 / 3),
            );
            let highlight_radius = (pupil_radius / 3).max(2);
            Circle::with_center(center + highlight_offset, highlight_radius * 2)
                .into_styled(PrimitiveStyle::with_fill(Rgb565::WHITE))
                .draw(target)?;
        }

        Ok(())
    }
}
