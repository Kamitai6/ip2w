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

use esp_hal::{
    clock::CpuClock, 
    delay::Delay, 
    gpio::{Event, Input, InputConfig, Io, Level, Output, OutputConfig, Pull}, 
    handler, 
    main, 
    time::{Duration, Instant, Rate}, 
    timer::{Timer, timg::{Timer as Timg, TimerGroup}},
    i2c::master::{I2c, Config as I2cConfig},
    spi::{master::{Spi, Config as SpiConfig}, Mode as SpiMode},
};
use embedded_graphics::{
    prelude::{IntoStorage, RgbColor, Point, Size, DrawTarget, Primitive},
    pixelcolor::Rgb565,
    mono_font::{
        ascii::FONT_10X20,
        MonoTextStyleBuilder,
    },
    text::{Alignment, Text},
    Drawable,
    primitives::{Circle, PrimitiveStyle},
};
use embedded_hal_bus::i2c::{RefCellDevice as I2cRefCellDevice};
use atom::{atom_motion, bmi270, bmm150, lp5562};
use control::{fb::{pid, smc}, ff::{gravity, pos_regulator}, util::{imu_ekf, deadzone}};
use mipidsi_async::{
    Builder, 
    models::{GC9107, ST7789}, 
    options::{ColorInversion, ColorOrder, Orientation, RefreshOrder, Rotation, VerticalRefreshOrder, HorizontalRefreshOrder}
};
use esp_display_interface::{dma_resources, DmaSpiInterface};

mod events;
mod face;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

const FREQUENCY: u32 = 500;
const PERIOD_US: u64 = 1_000_000 / FREQUENCY as u64;
const DT: f32 = 1.0 / FREQUENCY as f32;
const U_MAX: f32 = 300.0;

static TIMER0: Mutex<RefCell<Option<Timg>>> = Mutex::new(RefCell::new(None));
pub static TIMER_COUNTER: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));


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
    let timg1 = TimerGroup::new(peripherals.TIMG1);
    // let mut wdt0 = timg0.wdt;
    // wdt0.disable();
    // let mut wdt1 = timg1.wdt;
    // wdt1.disable();

    let button = Input::new(peripherals.GPIO41, InputConfig::default().with_pull(Pull::Up));

    //IR GPIO47

    let i2c0 = I2c::new(peripherals.I2C0, 
        I2cConfig::default()
            .with_frequency(Rate::from_khz(800))
        ).unwrap()
            .with_sda(peripherals.GPIO45)
            .with_scl(peripherals.GPIO0);
    let i2c0_ref_cell = RefCell::new(i2c0);

    // LP5562
    let mut lp5562 = lp5562::Lp5562::new(I2cRefCellDevice::new(&i2c0_ref_cell));
    // なぜかよくパニックするので、あとで守勢しなければ
    lp5562.init(&mut |us| delay.delay_micros(us)).unwrap();
    lp5562.set_current(lp5562::Channel::White, 255).unwrap();
    lp5562.set_pwm(lp5562::Channel::White, 255).unwrap(); // 白色点灯

    // Initialize IMU
    let mut imu = bmi270::Bmi270::new(I2cRefCellDevice::new(&i2c0_ref_cell))
        .with_mag(
            bmm150::OpMode::Normal,
            bmm150::DataRate::Hz30,
            bmm150::Preset::Regular,
        );
    imu.init_with_config(
        bmi270::Config {
            acc_odr: bmi270::AccOdr::Hz800,
            gyr_odr: bmi270::GyrOdr::Hz800,
            acc_range: bmi270::AccRange::G4,
            gyr_range: bmi270::GyrRange::Dps500,
            acc_bwp: bmi270::AccBwp::Normal,
            gyr_bwp: bmi270::GyrBwp::Normal,
            perf_mode: bmi270::PerfMode::PerfOpt,
            aux_odr: bmi270::AuxOdr::Hz50, //30Hzなので、これ以上上げると死ぬ
        }, 
        &mut |us| delay.delay_micros(us)
    ).unwrap();

    /* calibration */
    // imu.calibrate_acc(bmi270::FocAccConfig::z_up(), |us| delay.delay_micros(us)).unwrap();
    // let (ax, ay, az) = imu.read_acc_offset();
    // info!("AccelOffset: x={}, y={}, z={}", ax, ay, az);
    imu.write_acc_offset((33, -123, -144));
    /* always calibration */
    imu.calibrate_gyro(|us| delay.delay_micros(us)).unwrap();

    let mut ekf = imu_ekf::ImuEkf::new(
        imu_ekf::EkfConfig {
            dt: DT,
            gyro_noise: 0.05,
            gyro_bias_noise: 0.0005,
            accel_noise: 0.15,
            accel_magnitude_min: 0.5,
            accel_magnitude_max: 1.5,
            ..Default::default()
    });

    let i2c1 = I2c::new(peripherals.I2C1, 
        I2cConfig::default()
            .with_frequency(Rate::from_khz(800))
        ).unwrap()
            .with_sda(peripherals.GPIO38)
            .with_scl(peripherals.GPIO39);
    let i2c1_ref_cell = RefCell::new(i2c1);

    let mut motion = atom_motion::AtomMotion::new(I2cRefCellDevice::new(&i2c1_ref_cell));

    let lcd_spi = Spi::new(
            peripherals.SPI2,
            SpiConfig::default()
                .with_frequency(Rate::from_mhz(80))
                .with_mode(SpiMode::_0)
        ).unwrap()
            .with_sck(peripherals.GPIO15)
            .with_mosi(peripherals.GPIO21)
            .with_cs(peripherals.GPIO14)
            .with_dma(peripherals.DMA_CH0);

    let lcd_dc = esp_hal::gpio::Output::new(
        peripherals.GPIO42, 
        esp_hal::gpio::Level::Low, 
        esp_hal::gpio::OutputConfig::default()
    );
    let lcd_rst = esp_hal::gpio::Output::new(
        peripherals.GPIO48, 
        esp_hal::gpio::Level::High, 
        esp_hal::gpio::OutputConfig::default()
    );

    dma_resources!(DISPLAY, 128, 128);

    let mut display = unsafe {
        let interface = DmaSpiInterface::new(
            lcd_spi, lcd_dc,
            &raw mut DISPLAY_DESC,
            &raw mut DISPLAY_CMD_BUF,
        );

        Builder::new(GC9107, interface)
            .reset_pin(lcd_rst)
            .display_size(128, 128)
            .display_offset(0, 32)
            .orientation(Orientation::new()
                .rotate(Rotation::Deg270))
            .refresh_order(RefreshOrder::new(VerticalRefreshOrder::BottomToTop, HorizontalRefreshOrder::LeftToRight))
            .color_order(ColorOrder::Bgr)
            .invert_colors(ColorInversion::Normal)
            .init(&mut delay, &raw mut DISPLAY_FB_A, &raw mut DISPLAY_FB_B)
            .unwrap()
    };

    let mut face = face::Face::new(Point::new(64, 64), 50, 30);

    // ラムダ(P)上げ過ぎると発散する
    // アルファ(I)外乱が大きいほど高くしないといけない
    // C(D)上げ過ぎると発振する
    let mut smc = smc::SuperTwistingSMC::new(DT, 200.0, 1000.0, 25.0)
        .with_smoothing(0.01)
        .with_v_regulation(U_MAX * 0.8);

    let mut gravity = gravity::GravityCompensator::new(0.1, 0.035, 5000.0);
    let mut pos_regulator = pos_regulator::PositionRegulator::new(DT, 100.0, 0.1)
        .with_lead(0.001)
        .with_max(45.0, 5.0)
        .with_lowpass(5.0);

    let mut yaw_pid = pid::PID::new(DT, 1.0, 0.0, 20.0);
    
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
    let mut target_angle = 0.0;

    info!("Start!");
    loop {
        display.poll();
        if events::has_pending_events() {
            while let Some(event) = events::get_event() {
                match event {
                    events::Event::MotionUpdate => {
                        let d = imu.read_immu().unwrap();
                        let (ax, ay, az) = (d.accel.x, d.accel.y, d.accel.z);
                        let (gx, gy, gz) = (d.gyro.x, d.gyro.y, d.gyro.z);
                        let mag = d.mag.unwrap();
                        let (mx, my, mz) = (mag.x, mag.y, mag.z);
                        // info!("{}, {}, {}", ax, ay, az);
                        info!("{}, {}, {}", mx, my, mz);
                        let state = ekf.update_x_up(ax, ay, az, gx, gy, gz);

                        if button.is_low() && !button_state  {
                            drive = !drive;
                        }
                        button_state = button.is_low();

                        if state.roll.abs() > 1.0 || state.pitch.abs() > 1.0 {
                            drive = false;
                        }

                        if drive {
                            let now_angle = state.pitch - 0.125;
                            let e = target_angle - now_angle;
                            let e_dot = 0.0 - gy;
                            let fb = smc.update(e, e_dot);
                            let ff = -gravity.update(now_angle);
                            let atom_max = atom_motion::MOTOR_SPEED_MAX as f32;
                            let base_out = deadzone::apply_deadzone(fb + ff, U_MAX, 30.0, atom_max); //(限界-5)程度にするのが最適っぽいな

                            let yaw_e = 0.0 - state.yaw;
                            let yaw_e_dot = 0.0 - gz;
                            let yaw_result = yaw_pid.update_with_d(yaw_e, yaw_e_dot);
                            let yaw_out_max = atom_motion::MOTOR_SPEED_MAX as f32 / 3.0;
                            let yaw_out = yaw_result.clamp(-yaw_out_max, yaw_out_max);
                            
                            m1_pwm = (base_out + yaw_out).clamp(-atom_max, atom_max) as i8;
                            m2_pwm = (-base_out + yaw_out).clamp(-atom_max, atom_max) as i8;

                            target_angle = 0.0 - pos_regulator.update(base_out);
                        } else {
                            m1_pwm = 0;
                            m2_pwm = 0;
                            smc.reset();
                            yaw_pid.reset();
                            pos_regulator.reset();
                        }
                        let _ = motion.set_motor(atom_motion::MotorChannel::M1, m1_pwm);
                        let _ = motion.set_motor(atom_motion::MotorChannel::M2, m2_pwm);

                        let counter = critical_section::with(|cs| {
                            TIMER_COUNTER.borrow(cs).get()
                        });
                        if counter % (FREQUENCY / 2) == 0 { //0.5s
                            let emotion = if drive {
                                face::Emotion::UPPER[pseudo_rand(face::Emotion::UPPER.len())]
                            } else {
                                face::Emotion::LOWER[pseudo_rand(face::Emotion::LOWER.len())]
                            };
                            face.set_emotion(emotion);
                        }
                    }
                    events::Event::DisplayUpdate => {
                        // if display.is_idle() {
                        //     display.clear(Rgb565::BLACK).unwrap();

                        //     face.update(&mut display, DT * 10.0).unwrap();

                        //     display.flush_async().unwrap();
                        // }
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

fn pseudo_rand(idx_max: usize) -> usize {
    with(|cs| {
        let c = TIMER_COUNTER.borrow(cs).get();
        (c as usize) % idx_max
    })
}