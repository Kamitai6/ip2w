#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::{cell::{RefCell, Cell}, f32::consts::PI};
use alloc::borrow::ToOwned;
use critical_section::{Mutex, with};
use defmt::info;
use {esp_backtrace as _, esp_println as _};

use libm::{atan2f};
use esp_hal::{
    clock::CpuClock,
    dma::{DmaPriority, DmaRxBuf, DmaTxBuf, ExternalBurstConfig, DmaChannel, RegisterAccess},
    peripherals::Peripherals,
    gpio::{Event, Input, InputConfig, Io, Level, Output, OutputConfig, Pull},
    spi::{
        master::{Config as SpiCfg, Spi},  
        Mode,
    },
    delay::Delay,
    time::{Duration, Instant, Rate},
    timer::{Timer, timg::{Timer as Timg, TimerGroup}},
    i2c::master::{I2c, Config as I2cCfg},
    main,
    handler,
};
use esp_bsp::{lcd_spi, lcd_backlight_init, lcd_display_interface, lcd_display, i2c0_init, i2c1_init, BoardType, DisplayConfig};
use embedded_graphics::{
    prelude::{IntoStorage, RgbColor, Point, DrawTarget},
    pixelcolor::Rgb565,
    mono_font::{
        ascii::FONT_10X20,
        MonoTextStyleBuilder,
    },
    text::{Alignment, Text},
    Drawable,
};
use embedded_hal::i2c::{I2c as eh_I2c};
use embedded_hal_bus::{
    i2c::{RefCellDevice as I2cRefCellDevice},
    spi::ExclusiveDevice,
};
use atom::{atom_motion, bmi270, lp5562};
use control::{fb, util};

mod events;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

const FREQUENCY: u32 = 500;
const PERIOD_US: u64 = 1_000_000 / FREQUENCY as u64;
const DT: f32 = 1.0 / FREQUENCY as f32;
static TIMER0: Mutex<RefCell<Option<Timg>>> = Mutex::new(RefCell::new(None));
pub static TIMER_COUNTER: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));


pub struct ComplementaryFilter {
    angle: f32,
    alpha: f32,
    initialized: bool,
}

impl ComplementaryFilter {
    pub fn new(tau: f32, dt: f32) -> Self {
        Self {
            angle: 0.0,
            alpha: tau / (tau + dt),
            initialized: false,
        }
    }

    pub fn update(&mut self, accel_angle: f32, gyro_rate: f32, dt: f32) -> f32 {
        if !self.initialized {
            self.angle = accel_angle;
            self.initialized = true;
        } else {
            self.angle = self.alpha * (self.angle + gyro_rate * dt) 
                       + (1.0 - self.alpha) * accel_angle;
        }
        self.angle
    }

    pub fn reset(&mut self) {
        self.initialized = false;
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    let mut delay = Delay::new();
    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let mut button = Input::new(peripherals.GPIO41, InputConfig::default().with_pull(Pull::Up));

    //IR GPIO47

    let i2c0 = i2c0_init!(peripherals);
    let i2c0_ref_cell = RefCell::new(i2c0);

    // LP5562
    let mut lp5562 = lp5562::Lp5562::new(I2cRefCellDevice::new(&i2c0_ref_cell));
    lp5562.init().unwrap();
    delay.delay_millis(10);
    lp5562.set_current(lp5562::Channel::White, 255).unwrap();
    lp5562.set_pwm(lp5562::Channel::White, 255).unwrap(); // 白色点灯

    // Initialize IMU
    let mut imu = bmi270::Bmi270::new(I2cRefCellDevice::new(&i2c0_ref_cell));
    imu.init_with_config(
        bmi270::Config {
            acc_odr: bmi270::AccOdr::Hz400,
            gyr_odr: bmi270::GyrOdr::Hz400,
            acc_range: bmi270::AccRange::G4,
            gyr_range: bmi270::GyrRange::Dps250,
            acc_bwp: bmi270::AccBwp::Normal,
            gyr_bwp: bmi270::GyrBwp::Normal,
            perf_mode: bmi270::PerfMode::PerfOpt,
        }, 
        &mut |us| delay.delay_micros(us)
    ).unwrap();

    /* calibration */
    // imu.perform_acc_foc(bmi270::FocAccConfig::z_up(), |us| delay.delay_micros(us)).unwrap();
    // let (ax, ay, az) = imu.read_acc_offset();
    // info!("AccelOffset: x={}, y={}, z={}", ax, ay, az);
    imu.write_acc_offset((33, -123, -144));
    /* always calibration */
    imu.perform_gyr_foc(|us| delay.delay_micros(us)).unwrap();

    // kalman
    let mut ekf = util::imu_ekf::ImuEkf::new(
        util::imu_ekf::EkfConfig {
            dt: DT,
            gyro_noise: 0.05,
            gyro_bias_noise: 0.0005,
            accel_noise: 0.15,
            accel_magnitude_min: 0.5,
            accel_magnitude_max: 1.5,
            ..Default::default()
    });

    let i2c1 = i2c1_init!(peripherals);
    let i2c1_ref_cell = RefCell::new(i2c1);

    // Atom Motion motor driver
    let mut motion = atom_motion::AtomMotion::new(I2cRefCellDevice::new(&i2c1_ref_cell));

    // let mut smc = fb::smc::SuperTwistingSMC::new(DT, 30.0, 1500.0, 1.0)
    //     .with_smoothing(0.2, 0.001)
    //     .with_v_regulation(127.0, 3.0);
    use fb::smc::SimpleSMC;
    let mut smc = SimpleSMC::new(DT, 127.0, 3.0, 0.05, 600.0);
    // let mut pid = fb::pid::PID::new(DT, 10.0, 130.0, 0.3);

    let lcd_spi = lcd_spi!(peripherals);
    let di = lcd_display_interface!(peripherals, lcd_spi);
    let mut display = lcd_display!(peripherals, di, &mut delay).unwrap();

    display.clear(Rgb565::RED).unwrap();
    delay.delay_millis(10);
    display.clear(Rgb565::GREEN).unwrap();
    delay.delay_millis(10);
    display.clear(Rgb565::BLUE).unwrap();
    delay.delay_millis(10);
    display.clear(Rgb565::WHITE).unwrap();

    let _ = Text::with_alignment("HELLO WORLD!", Point::new(64, 64), MonoTextStyleBuilder::new().font(&FONT_10X20).text_color(RgbColor::BLACK).build(),  Alignment::Center)
        .draw(&mut display);
    
    // esp_rtos::start(timg0.timer0);
    // let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    // let (mut _wifi_controller, _interfaces) =
    //     esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
    //         .expect("Failed to initialize Wi-Fi controller");

    let timer0 = timg0.timer0;
    timer0.set_interrupt_handler(tg0_t0_handler);
    timer0.load_value(Duration::from_micros(PERIOD_US)).unwrap();
    timer0.enable_auto_reload(true);
    timer0.enable_interrupt(true);
    timer0.start();

    with(|cs| {
        TIMER0.borrow_ref_mut(cs).replace(timer0);
    });

    let mut drive = false;
    let mut button_state = false;

    let mut cf = ComplementaryFilter::new(0.1, DT);

    loop {
        // イベントがあるかチェック
        if events::has_pending_events() {
            // 全てのイベントを処理
            while let Some(event) = events::get_event() {
                match event {
                    events::Event::MotionUpdate => {
                        let (ax, ay, az) = imu.read_accel().unwrap(); // g単位
                        let (gx, gy, gz) = util::imu_ekf::degree_to_rad(imu.read_gyro().unwrap()); // rad/s単位
                        let state = ekf.update_x_up(ax, ay, az, gx, gy, gz);
                        // info!("pitch: {}", state.pitch);

                        let accel_angle = atan2f(-az, ax);  // 参考コードと同じ
                        let pitch = cf.update(accel_angle, gy, DT);
                        defmt::info!("accel_angle={} gy={} pitch={}", accel_angle, gy, pitch);

                        if button.is_low() && !button_state  {
                            drive = !drive;
                        }
                        button_state = button.is_low();

                        if drive {
                            let target = -0.1;
                            let value = pitch;
                            let e_dot = gy;
                            let m_sign = -1;

                            let fb = smc.update(target - value, e_dot);
                            // let fb = pid.update(value, target);
                            let ff = 0.0;
                            let output = ((fb + ff) as i8).clamp(-127, 127);

                            if let Err(e) = motion.set_motor(atom_motion::MotorChannel::M1, -output * m_sign) {
                                info!("Motor M1 error: {:?}", e);
                            }
                            if let Err(e) = motion.set_motor(atom_motion::MotorChannel::M2, output * m_sign) {
                                info!("Motor M2 error: {:?}", e);
                            }
                        } else {
                            if let Err(e) = motion.stop_motor(atom_motion::MotorChannel::M1) {
                                info!("Motor M1 error: {:?}", e);
                            }
                            if let Err(e) = motion.stop_motor(atom_motion::MotorChannel::M2) {
                                info!("Motor M2 error: {:?}", e);
                            }
                        }
                    }
                    events::Event::DisplayUpdate => {

                    }
                }
            }
        }
    }
}

// タイマー割り込みハンドラ
#[handler]
fn tg0_t0_handler() {
    with(|cs| {
        let counter = TIMER_COUNTER.borrow(cs).get();
        if let Some(timer) = TIMER0.borrow_ref_mut(cs).as_mut() {
            timer.clear_interrupt();

            let div = 1;
            if counter % div == 0 {
                events::post_event(events::Event::MotionUpdate);
            }
            let div = div * 10;
            if counter % div == 0 {
                events::post_event(events::Event::DisplayUpdate);
            }
            TIMER_COUNTER.borrow(cs).set(counter + 1);
        }
    });
}
