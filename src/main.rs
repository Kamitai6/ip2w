#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::cell::RefCell;
use alloc::borrow::ToOwned;
use critical_section::Mutex;
use defmt::info;
use {esp_backtrace as _, esp_println as _};

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
    timer::timg::{Timer, TimerGroup},
    i2c::master::{I2c, Config as I2cCfg},
    main,
    handler,
};
use esp_bsp::{lcd_spi, lcd_backlight_init, lcd_display_interface, lcd_display, i2c_init, BoardType, DisplayConfig};
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
use control::util;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

static TIMER0: Mutex<RefCell<Option<Timer>>> = Mutex::new(RefCell::new(None));

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

    // let mut io = Io::new(peripherals.IO_MUX);
    // io.set_interrupt_handler(handler);
    // let mut led = Output::new(peripherals.GPIO2, Level::Low, OutputConfig::default());
    // let config = InputConfig::default().with_pull(Pull::Up);
    // let mut button = Input::new(peripherals.GPIO41, config);

    //IR G47

    let i2c = i2c_init!(peripherals);
    let i2c_ref_cell = RefCell::new(i2c);

    // LP5562
    let mut lp5562 = lp5562::Lp5562::new(I2cRefCellDevice::new(&i2c_ref_cell));
    lp5562.init().unwrap();
    delay.delay_millis(10);
    lp5562.set_current(lp5562::Channel::White, 255).unwrap();
    lp5562.set_pwm(lp5562::Channel::White, 255).unwrap(); // 白色点灯

    // Initialize IMU
    let mut imu = bmi270::Bmi270::new(I2cRefCellDevice::new(&i2c_ref_cell));
    imu.init_with_config(
        bmi270::Config {
            acc_odr: bmi270::AccOdr::Hz400,
            gyr_odr: bmi270::GyrOdr::Hz400,
            acc_range: bmi270::AccRange::G4,
            gyr_range: bmi270::GyrRange::Dps500,
            acc_bwp: bmi270::AccBwp::Normal,
            gyr_bwp: bmi270::GyrBwp::Normal,
            perf_mode: bmi270::PerfMode::PerfOpt,
        }, 
        &mut |us| delay.delay_micros(us)
    ).unwrap();

    // calibration
    // imu.perform_foc(bmi270::FocAccConfig::z_up(), |us| delay.delay_micros(us)).unwrap();
    // let (ax, ay, az) = imu.read_acc_offset(); // (71, -126, -77)
    // let (gx, gy, gz) = imu.read_gyr_offset(); // (-36, 2, -22)
    // info!("AccelOffset: x={}, y={}, z={}", ax, ay, az);
    // info!("GyroOffset: x={}, y={}, z={}", gx, gy, gz);
    imu.write_acc_offset((71, -126, -77)); 
    imu.write_gyr_offset((-36, 2, -22));

    let mut ekf = util::imu_ekf::ImuEkf::new(
        util::imu_ekf::EkfConfig {
            dt: 1.0 / 400.0,
            gyro_noise: 0.01,
            gyro_bias_noise: 0.0001,
            accel_noise: 0.15,
            accel_magnitude_min: 0.5,
            accel_magnitude_max: 1.5,
            ..Default::default()
    });

    // Atom Motion motor driver
    // let mut motion = atom_motion::AtomMotion::new(I2cRefCellDevice::new(&i2c_ref_cell));
    // motion.set_motor(atom_motion::MotorChannel::M1, 100).unwrap();   // 正転
    // motion.set_motor(atom_motion::MotorChannel::M2, -100).unwrap();  // 逆転

    let lcd_spi = lcd_spi!(peripherals);
    let di = lcd_display_interface!(peripherals, lcd_spi);
    let mut display = lcd_display!(peripherals, di, &mut delay).unwrap();

    display.clear(Rgb565::RED).unwrap();
    delay.delay_millis(1000);
    display.clear(Rgb565::GREEN).unwrap();
    delay.delay_millis(1000);
    display.clear(Rgb565::BLUE).unwrap();
    delay.delay_millis(1000);
    display.clear(Rgb565::WHITE).unwrap();

    let _ = Text::with_alignment("HELLO WORLD!", Point::new(64, 64), MonoTextStyleBuilder::new().font(&FONT_10X20).text_color(RgbColor::BLACK).build(),  Alignment::Center)
        .draw(&mut display);
    
    // esp_rtos::start(timg0.timer0);
    // let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    // let (mut _wifi_controller, _interfaces) =
    //     esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
    //         .expect("Failed to initialize Wi-Fi controller");

    let mut timer0 = timg0.timer0;
    timer0.set_interrupt_handler(tg0_t0_handler);
    timer0.enable_interrupt(true);
    timer0.load_alarm_value(1_000_000u64); // マイクロ秒単位
    timer0.set_alarm_active(true);
    timer0.start();

    critical_section::with(|cs| {
        TIMER0.borrow_ref_mut(cs).replace(timer0);
    });

    loop {
        info!("Hello world!");

        // motion.set_motor(atom_motion::MotorChannel::M1, -pid.control).unwrap();
        // motion.set_motor(atom_motion::MotorChannel::M2, pid.control).unwrap();
        
        let (ax, ay, az) = imu.read_accel().unwrap();  // g単位
        let (gx, gy, gz) = imu.read_gyro().unwrap();   // °/s単位
        let state = ekf.update_deg(ax, ay, az, gx, gy, gz);
        
        delay.delay_millis(1000);
    }
}

// タイマー割り込みハンドラ
#[handler]
fn tg0_t0_handler() {
    critical_section::with(|cs| {
        // LED トグル
        // if let Some(led) = LED.borrow_ref_mut(cs).as_mut() {
        //     led.toggle();
        // }

        // 割り込みクリア & 次のアラーム設定
        if let Some(timer) = TIMER0.borrow_ref_mut(cs).as_mut() {
            timer.clear_interrupt();
            timer.load_alarm_value(1_000_000u64); // 1秒後
            timer.set_alarm_active(true);
        }
    });
}
