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

fn apply_deadzone(input_value: f32, input_limit: f32, output_min: f32, output_max: f32) -> f32 {
    // 1. 入力を制限（安全装置）
    let input = input_value.clamp(-input_limit, input_limit);

    // 2. ほぼ0なら、計算誤差が出ないように完全に0にする
    if input.abs() < 0.001 {
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

    let button = Input::new(peripherals.GPIO41, InputConfig::default().with_pull(Pull::Up));

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
            gyr_range: bmi270::GyrRange::Dps1000,
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

    // ラムダ(P)は上げ過ぎると発散するから、するところまで上げて、しないギリギリまで下げる
    // アルファ(I)は外乱が大きいほど高くしないといけないから、小さい値からはじめて、ギリギリ外乱に耐えられるまで上げる
    // C(D)も上げ過ぎると発振するから、震えるまで上げて、しないギリギリまで下げる
    // 多分、ラムダとCを適当な値にして、ラムダかCをいい感じに上げながら最適化できそうなほうから合わせて、
    // 片方には強い感じまでやったら、アルファを上げてオフセットなどのモデル誤差を含めた外乱をすべて除けるようにしたら完成
    let mut smc = fb::smc::SuperTwistingSMC::new(DT, 200.0, 1000.0, 25.0)
        .with_smoothing(0.01, 0.00001)
        .with_v_regulation(240.0, 0.0);

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
    let mut m1_pwm = 0;
    let mut m2_pwm = 0;

    info!("Start!");
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

                        if button.is_low() && !button_state  {
                            drive = !drive;
                        }
                        button_state = button.is_low();

                        if state.roll.abs() > 1.0 || state.pitch.abs() > 1.0 {
                            drive = false;
                        }

                        if drive {
                            let e = 0.16775 - state.pitch;
                            let e_dot = 0.0 - gy;
                            let fb = smc.update(e, e_dot);
                            let ff = 0.0;
                            let output = apply_deadzone(fb + ff, 300.0, 30.0, 127.0) as i8; //(限界-5)程度にするのが最適っぽいな
                            m1_pwm = output;
                            m2_pwm = -output;
                        } else {
                            m1_pwm = 0;
                            m2_pwm = 0;
                            smc.reset();
                        }
                        let _ = motion.set_motor(atom_motion::MotorChannel::M1, m1_pwm);
                        let _ = motion.set_motor(atom_motion::MotorChannel::M2, m2_pwm);
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
