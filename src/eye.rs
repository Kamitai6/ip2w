use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, PrimitiveStyle, PrimitiveStyleBuilder},
};

// 目の構造体
pub struct Eye {
    center: Point,
    radius: u32,
    pupil_radius: u32,
    pupil_offset: Point, // 白目の中心からのズレ
}

impl Eye {
    pub fn new(x: i32, y: i32, radius: u32) -> Self {
        Self {
            center: Point::new(x, y),
            radius,
            pupil_radius: radius / 3,
            pupil_offset: Point::zero(),
        }
    }

    // 視線を更新（簡易的に制限）
    pub fn look_at(&mut self, dx: i32, dy: i32) {
        let max_offset = (self.radius - self.pupil_radius - 2) as i32;
        self.pupil_offset = Point::new(
            dx.clamp(-max_offset, max_offset),
            dy.clamp(-max_offset, max_offset),
        );
    }

    // 描画処理
    pub fn draw<D>(&self, target: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        // 1. 白目（白で塗りつぶし）
        Circle::with_center(self.center, self.radius * 2)
            .into_styled(PrimitiveStyle::with_fill(Rgb565::WHITE))
            .draw(target)?;

        // 2. 黒目（黒で塗りつぶし、位置はオフセットを加算）
        Circle::with_center(self.center + self.pupil_offset, self.pupil_radius * 2)
            .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
            .draw(target)?;

        Ok(())
    }
}