use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Arc, Circle, Line, PrimitiveStyle, Triangle},
};

/// 感情ステート
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum Emotion {
    #[default]
    Neutral,  // 普通（まばたき、瞳孔あり）
    Excited,  // 元気 ＞＜（上下バウンス + ほっぺピンク）
    Relaxed,  // 安心 － －（ゆっくり上下揺れ + zzz）
    Fear,     // 恐怖（目+瞳孔+涙青、震え、まばたき）
    Impact,   // 衝撃 × ×（点滅 + 星）
    Confused, // 困惑（縦長渦巻き回転）
}
impl Emotion {
    pub const UPPER: [Emotion; 3] = [
        Emotion::Neutral,
        Emotion::Excited,
        Emotion::Relaxed,
    ];

    pub const LOWER: [Emotion; 3] = [
        Emotion::Fear,
        Emotion::Impact,
        Emotion::Confused,
    ];
}

/// 顔（両目）を管理する構造体
pub struct Face {
    center: Point,
    eye_spacing: i32,
    eye_width: u32,
    eye_height: u32,
    stroke_width: u32,

    emotion: Emotion,

    phase: f32,
    blink_timer: f32,
    is_blinking: bool,
    blink_progress: f32,

    // 星の表示用
    star_timer: f32,
    current_star: usize,
}

impl Face {
    pub fn new(center: Point, eye_spacing: i32, eye_width: u32) -> Self {
        Self {
            center,
            eye_spacing,
            eye_width,
            eye_height: eye_width * 3 / 2,
            stroke_width: 3,
            emotion: Emotion::Neutral,
            phase: 0.0,
            blink_timer: 0.0,
            is_blinking: false,
            blink_progress: 0.0,
            star_timer: 0.0,
            current_star: 0,
        }
    }

    pub fn set_emotion(&mut self, emotion: Emotion) {
        if self.emotion != emotion {
            self.emotion = emotion;
            self.phase = 0.0;
            self.blink_timer = 0.0;
            self.is_blinking = false;
            self.blink_progress = 0.0;
        }
    }

    pub fn emotion(&self) -> Emotion {
        self.emotion
    }

    // 彩度を抑えた淡い色
    fn color_soft_pink() -> Rgb565 {
        Rgb565::new(25, 35, 22)
    }

    fn color_soft_blue() -> Rgb565 {
        Rgb565::new(18, 40, 28)
    }

    fn color_soft_yellow() -> Rgb565 {
        Rgb565::new(28, 55, 18)
    }

    fn color_soft_gray() -> Rgb565 {
        Rgb565::new(18, 36, 18)
    }

    pub fn update<D>(&mut self, target: &mut D, dt: f32) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        const TWO_PI: f32 = core::f32::consts::PI * 2.0;

        self.phase += dt * 2.0;
        if self.phase > TWO_PI {
            self.phase -= TWO_PI;
        }

        // 星タイマー更新（1つずつ順番に光る）
        self.star_timer += dt;
        if self.star_timer > 0.5 {
            self.star_timer = 0.0;
            self.current_star = (self.current_star + 1) % 4;
        }

        if matches!(self.emotion, Emotion::Neutral | Emotion::Fear) {
            self.update_blink(dt);
        }

        let left_center = Point::new(
            self.center.x - self.eye_spacing / 2,
            self.center.y,
        );
        let right_center = Point::new(
            self.center.x + self.eye_spacing / 2,
            self.center.y,
        );

        match self.emotion {
            Emotion::Neutral => {
                self.draw_neutral_eye(target, left_center)?;
                self.draw_neutral_eye(target, right_center)?;
            }
            Emotion::Excited => {
                self.draw_excited_eye(target, left_center, true)?;
                self.draw_excited_eye(target, right_center, false)?;
            }
            Emotion::Relaxed => {
                self.draw_relaxed_eye(target, left_center)?;
                self.draw_relaxed_eye(target, right_center)?;
                self.draw_zzz(target, right_center)?;
            }
            Emotion::Fear => {
                self.draw_fear_eye(target, left_center, true)?;
                self.draw_fear_eye(target, right_center, false)?;
            }
            Emotion::Impact => {
                self.draw_impact_eye(target, left_center)?;
                self.draw_impact_eye(target, right_center)?;
                self.draw_stars(target)?;
            }
            Emotion::Confused => {
                self.draw_confused_eye(target, left_center)?;
                self.draw_confused_eye(target, right_center)?;
            }
        }

        Ok(())
    }

    fn update_blink(&mut self, dt: f32) {
        const BLINK_DURATION: f32 = 0.15;

        let blink_interval = match self.emotion {
            Emotion::Fear => 1.5,
            _ => 3.5,
        };

        if self.is_blinking {
            self.blink_progress += dt / BLINK_DURATION;
            if self.blink_progress >= 1.0 {
                self.is_blinking = false;
                self.blink_progress = 0.0;
            }
        } else {
            self.blink_timer += dt;
            let trigger = blink_interval + libm::sinf(self.phase * 7.0) * 0.5;
            if self.blink_timer > trigger {
                self.is_blinking = true;
                self.blink_timer = 0.0;
            }
        }
    }

    fn stroke(&self) -> PrimitiveStyle<Rgb565> {
        PrimitiveStyle::with_stroke(Rgb565::WHITE, self.stroke_width)
    }

    fn thin_stroke(&self) -> PrimitiveStyle<Rgb565> {
        PrimitiveStyle::with_stroke(Rgb565::WHITE, (self.stroke_width - 1).max(1))
    }

    fn fill(&self) -> PrimitiveStyle<Rgb565> {
        PrimitiveStyle::with_fill(Rgb565::WHITE)
    }

    /// カプセル型を描画
    fn draw_capsule<D>(
        &self,
        target: &mut D,
        center: Point,
        width: u32,
        height: u32,
        style: PrimitiveStyle<Rgb565>,
    ) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let half_w = (width / 2) as i32;
        let straight_len = height.saturating_sub(width) as i32;
        let half_straight = straight_len / 2;

        let top_arc_y = center.y - half_straight;
        let bottom_arc_y = center.y + half_straight;

        let top_arc = Arc::new(
            Point::new(center.x - half_w, top_arc_y - half_w),
            width,
            Angle::from_degrees(180.0),
            Angle::from_degrees(180.0),
        );
        top_arc.into_styled(style).draw(target)?;

        let bottom_arc = Arc::new(
            Point::new(center.x - half_w, bottom_arc_y - half_w),
            width,
            Angle::from_degrees(0.0),
            Angle::from_degrees(180.0),
        );
        bottom_arc.into_styled(style).draw(target)?;

        if straight_len > 0 {
            Line::new(
                Point::new(center.x - half_w, top_arc_y),
                Point::new(center.x - half_w, bottom_arc_y),
            )
            .into_styled(style)
            .draw(target)?;

            Line::new(
                Point::new(center.x + half_w, top_arc_y),
                Point::new(center.x + half_w, bottom_arc_y),
            )
            .into_styled(style)
            .draw(target)?;
        }

        Ok(())
    }

    /// 涙しずく（色付き塗りつぶし）を描画
    fn draw_teardrop_filled<D>(
        &self,
        target: &mut D,
        top: Point,
        width: u32,
        height: u32,
        color: Rgb565,
    ) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let half_w = (width / 2) as i32;
        let circle_radius = width / 2;
        
        let circle_y = top.y + height as i32 - circle_radius as i32;
        let fill_style = PrimitiveStyle::with_fill(color);

        Triangle::new(
            top,
            Point::new(top.x - half_w, circle_y),
            Point::new(top.x + half_w, circle_y),
        )
        .into_styled(fill_style)
        .draw(target)?;

        Circle::with_center(Point::new(top.x, circle_y), circle_radius * 2)
            .into_styled(fill_style)
            .draw(target)?;

        Ok(())
    }

    /// 縦長の渦巻きを描画
    fn draw_oval_spiral<D>(
        &self,
        target: &mut D,
        center: Point,
        width: u32,
        height: u32,
        rotation: f32,
        style: PrimitiveStyle<Rgb565>,
    ) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let half_w = width as f32 / 2.0;
        let half_h = height as f32 / 2.0;

        let num_points = 40;
        let turns = 2.5;

        let mut prev_point: Option<Point> = None;

        for i in 0..num_points {
            let t = i as f32 / num_points as f32;
            let angle = t * turns * core::f32::consts::PI * 2.0 + rotation;
            let scale = 1.0 - t * 0.85;

            let x = center.x + (libm::cosf(angle) * half_w * scale) as i32;
            let y = center.y + (libm::sinf(angle) * half_h * scale) as i32;
            let current = Point::new(x, y);

            if let Some(prev) = prev_point {
                Line::new(prev, current).into_styled(style).draw(target)?;
            }

            prev_point = Some(current);
        }

        Ok(())
    }

    /// 小さい星を描画
    fn draw_star<D>(
        &self,
        target: &mut D,
        center: Point,
        size: i32,
        style: PrimitiveStyle<Rgb565>,
    ) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        // 縦線
        Line::new(
            Point::new(center.x, center.y - size),
            Point::new(center.x, center.y + size),
        )
        .into_styled(style)
        .draw(target)?;

        // 横線
        Line::new(
            Point::new(center.x - size, center.y),
            Point::new(center.x + size, center.y),
        )
        .into_styled(style)
        .draw(target)?;

        Ok(())
    }

    /// zzz を斜め右上に描画
    fn draw_zzz<D>(&self, target: &mut D, right_eye: Point) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let style = PrimitiveStyle::with_stroke(Self::color_soft_gray(), 2);

        let base_x = right_eye.x + (self.eye_width as i32 / 2) + 3;
        let base_y = right_eye.y - (self.eye_height as i32 / 2) - 3;

        let float_offset = (libm::sinf(self.phase * 0.8) * 2.0) as i32;

        let sizes = [3i32, 4, 5];

        for (i, &size) in sizes.iter().enumerate() {
            let offset = i as i32 * 7;
            let x = base_x + offset;
            let y = base_y - offset + float_offset;

            let half = size / 2;
            Line::new(
                Point::new(x - half, y - half),
                Point::new(x + half, y - half),
            )
            .into_styled(style)
            .draw(target)?;
            Line::new(
                Point::new(x + half, y - half),
                Point::new(x - half, y + half),
            )
            .into_styled(style)
            .draw(target)?;
            Line::new(
                Point::new(x - half, y + half),
                Point::new(x + half, y + half),
            )
            .into_styled(style)
            .draw(target)?;
        }

        Ok(())
    }

    /// 1つだけ星を描画（4か所を順番に）
    fn draw_stars<D>(&self, target: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let style = PrimitiveStyle::with_stroke(Self::color_soft_yellow(), 1);

        // 4か所の固定位置
        let star_positions: [(i32, i32); 4] = [
            (15, 20),
            (112, 18),
            (12, 108),
            (115, 110),
        ];

        // 現在の星だけ描画
        let (x, y) = star_positions[self.current_star];
        self.draw_star(target, Point::new(x, y), 3, style)?;

        Ok(())
    }

    /// Neutral: カプセル型 + 瞳孔（まばたき）
    fn draw_neutral_eye<D>(&self, target: &mut D, center: Point) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let mut openness = 1.0f32;
        if self.is_blinking {
            let blink_curve = libm::sinf(self.blink_progress * core::f32::consts::PI);
            openness = 1.0 - blink_curve * 0.95;
        }

        let actual_height = ((self.eye_height as f32) * openness) as u32;

        if actual_height < self.eye_width / 2 {
            let half_w = (self.eye_width / 2) as i32;
            Line::new(
                Point::new(center.x - half_w, center.y),
                Point::new(center.x + half_w, center.y),
            )
            .into_styled(self.stroke())
            .draw(target)?;
            return Ok(());
        }

        // 白目（カプセル型）
        self.draw_capsule(target, center, self.eye_width, actual_height, self.stroke())?;

        // 瞳孔（大きめ、下にくっつく）
        if openness > 0.4 {
            let pupil_radius = self.eye_width * 2 / 5; // 大きめ
            let half_h = (actual_height / 2) as i32;
            // 瞳孔の下端が目の下端に揃うように
            let pupil_y = center.y + half_h - pupil_radius as i32;

            Circle::with_center(Point::new(center.x, pupil_y), pupil_radius * 2)
                .into_styled(self.fill())
                .draw(target)?;
        }

        Ok(())
    }

    /// Excited: ＞＜（上下バウンス + ほっぺピンク）
    fn draw_excited_eye<D>(&self, target: &mut D, center: Point, is_left: bool) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let bounce = (libm::sinf(self.phase * 5.0) * 4.0) as i32;
        let center = center + Point::new(0, bounce);

        let half_h = (self.eye_height / 2) as i32;
        let half_w = (self.eye_width / 3) as i32;

        // ほっぺ（淡いピンク）
        let cheek_radius = (self.eye_width / 4).max(3);
        let cheek_offset_x = if is_left {
            -(self.eye_width as i32 / 2) - (cheek_radius as i32) - 2
        } else {
            (self.eye_width as i32 / 2) + (cheek_radius as i32) + 2
        };
        let cheek_offset_y = half_h / 2;
        
        Circle::with_center(
            Point::new(center.x + cheek_offset_x, center.y + cheek_offset_y),
            cheek_radius * 2,
        )
        .into_styled(PrimitiveStyle::with_fill(Self::color_soft_pink()))
        .draw(target)?;

        // 目 ＞＜
        if is_left {
            Line::new(
                Point::new(center.x - half_w, center.y - half_h),
                Point::new(center.x + half_w, center.y),
            )
            .into_styled(self.stroke())
            .draw(target)?;
            Line::new(
                Point::new(center.x + half_w, center.y),
                Point::new(center.x - half_w, center.y + half_h),
            )
            .into_styled(self.stroke())
            .draw(target)?;
        } else {
            Line::new(
                Point::new(center.x + half_w, center.y - half_h),
                Point::new(center.x - half_w, center.y),
            )
            .into_styled(self.stroke())
            .draw(target)?;
            Line::new(
                Point::new(center.x - half_w, center.y),
                Point::new(center.x + half_w, center.y + half_h),
            )
            .into_styled(self.stroke())
            .draw(target)?;
        }

        Ok(())
    }

    /// Relaxed: － －（ゆっくり上下揺れ）
    fn draw_relaxed_eye<D>(&self, target: &mut D, center: Point) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let sway = (libm::sinf(self.phase * 0.8) * 3.0) as i32;
        let swayed_center = center + Point::new(0, sway);

        let half_w = (self.eye_width / 2) as i32;

        Line::new(
            Point::new(swayed_center.x - half_w, swayed_center.y),
            Point::new(swayed_center.x + half_w, swayed_center.y),
        )
        .into_styled(self.stroke())
        .draw(target)?;

        Ok(())
    }

    /// Fear: カプセル目 + 涙マーク青（片目の下のみ、震え、まばたき）
    fn draw_fear_eye<D>(&self, target: &mut D, center: Point, is_left: bool) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let shake_x = (libm::sinf(self.phase * 30.0) * 3.0) as i32;
        let shake_y = (libm::cosf(self.phase * 35.0) * 2.0) as i32;
        let shaken_center = center + Point::new(shake_x, shake_y);

        let mut openness = 1.0f32;
        if self.is_blinking {
            let blink_curve = libm::sinf(self.blink_progress * core::f32::consts::PI);
            openness = 1.0 - blink_curve * 0.95;
        }

        let actual_height = ((self.eye_height as f32) * openness) as u32;

        if actual_height < self.eye_width / 2 {
            let half_w = (self.eye_width / 2) as i32;
            Line::new(
                Point::new(shaken_center.x - half_w, shaken_center.y),
                Point::new(shaken_center.x + half_w, shaken_center.y),
            )
            .into_styled(self.stroke())
            .draw(target)?;
        } else {
            self.draw_capsule(target, shaken_center, self.eye_width, actual_height, self.stroke())?;
        }

        // 瞳孔（大きめ、下にくっつく）
        if openness > 0.4 {
            let pupil_radius = self.eye_width * 2 / 5; // 大きめ
            let half_h = (actual_height / 2) as i32;
            // 瞳孔の下端が目の下端に揃うように
            let pupil_y = center.y + half_h - pupil_radius as i32;

            Circle::with_center(Point::new(center.x, pupil_y), pupil_radius * 2)
                .into_styled(self.fill())
                .draw(target)?;
        }

        // 涙マーク（淡い青、左目の下のみ）
        if is_left {
            let tear_width = self.eye_width / 3;
            let tear_height = self.eye_height / 2;
            let tear_top = Point::new(
                shaken_center.x,
                shaken_center.y + (self.eye_height / 2) as i32 + 6,
            );

            self.draw_teardrop_filled(target, tear_top, tear_width, tear_height, Self::color_soft_blue())?;
        }

        Ok(())
    }

    /// Impact: × ×（点滅）
    fn draw_impact_eye<D>(&self, target: &mut D, center: Point) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let blink_cycle = (self.phase * 4.0) % core::f32::consts::PI;
        if blink_cycle > core::f32::consts::PI * 0.7 {
            return Ok(());
        }

        let half_w = (self.eye_width / 3) as i32;
        let half_h = (self.eye_height / 3) as i32;

        Line::new(
            Point::new(center.x - half_w, center.y - half_h),
            Point::new(center.x + half_w, center.y + half_h),
        )
        .into_styled(self.stroke())
        .draw(target)?;

        Line::new(
            Point::new(center.x + half_w, center.y - half_h),
            Point::new(center.x - half_w, center.y + half_h),
        )
        .into_styled(self.stroke())
        .draw(target)?;

        Ok(())
    }

    /// Confused: 縦長渦巻き（回転）
    fn draw_confused_eye<D>(&self, target: &mut D, center: Point) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let rotation = self.phase * 2.0;
        let wobble_x = (libm::sinf(self.phase * 3.0) * 2.0) as i32;
        let wobble_y = (libm::cosf(self.phase * 2.5) * 2.0) as i32;
        let wobbled_center = center + Point::new(wobble_x, wobble_y);

        self.draw_oval_spiral(
            target,
            wobbled_center,
            self.eye_width,
            self.eye_height,
            rotation,
            self.thin_stroke(),
        )?;

        Ok(())
    }
}