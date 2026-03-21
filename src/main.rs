#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::{cell::{RefCell, Cell}, f32::consts::PI, fmt::Write};
use alloc::{borrow::ToOwned, string::ToString};
use critical_section::{Mutex, with};
use defmt::info;
use libm::{atan2f, sqrtf, cosf, sinf, fabsf, copysignf};
use heapless::String;
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
use esp_radio::wifi::{AccessPointConfig, ModeConfig};
use blocking_network_stack::Stack as NetworkStack;
use smoltcp::iface::{SocketSet, SocketStorage};
use smoltcp::wire::{IpAddress, Ipv4Address};
use embedded_graphics::{
    prelude::{IntoStorage, RgbColor, Point, Size, DrawTarget, Primitive},
    pixelcolor::Rgb565,
    mono_font::{
        ascii::{FONT_10X20, FONT_6X10},
        MonoTextStyleBuilder,
    },
    text::{Alignment, Text},
    Drawable,
    primitives::{Circle, PrimitiveStyle},
};
use embedded_hal_bus::i2c::{RefCellDevice as I2cRefCellDevice};
use atom::{atom_motion, ina226, bmi270, bmm150, lp5562};
// [CHANGED] pid, smc → lq_stsmc
use control::{fb::lq_stsmc, ff::{gravity, dob_kf}, util::{mekf, mag_calib, mag_ets, mag_rls, lpf, pos_ekf, ista_smd}};
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

// 1回のログの最大文字数（allocを使わずヒープレスで管理）
const UDP_LOG_BUF_SIZE: usize = 256;

struct UdpLoggerState {
    target: Option<(IpAddress, u16)>,
    buffer: String<UDP_LOG_BUF_SIZE>,
}

// どこからでもアクセスできるグローバルな状態
static UDP_LOGGER_STATE: Mutex<RefCell<UdpLoggerState>> = Mutex::new(RefCell::new(UdpLoggerState {
    target: None,
    buffer: String::new(),
}));

/// 実行中の任意のタイミングで宛先を設定・変更できる関数
pub fn udp_logger_init(addr: IpAddress, port: u16) {
    with(|cs| {
        let mut state = UDP_LOGGER_STATE.borrow_ref_mut(cs);
        state.target = Some((addr, port));
    });
}

/// メインループ内で毎回呼び出す関数
pub fn udp_logger_poll<D: smoltcp::phy::Device>(socket: &mut blocking_network_stack::UdpSocket<'_, '_, D>) {
    // UDPの受信処理やスタックの維持のため、送信の有無に関わらず回す
    socket.poll();

    // グローバルバッファからデータを取り出す
    let (data_to_send, target) = with(|cs| {
        let mut state = UDP_LOGGER_STATE.borrow_ref_mut(cs);
        if state.buffer.is_empty() {
            return (None, None);
        }

        let mut temp_buf = [0u8; UDP_LOG_BUF_SIZE];
        let len = state.buffer.len();
        temp_buf[..len].copy_from_slice(state.buffer.as_bytes());
        state.buffer.clear();

        (Some((temp_buf, len)), state.target)
    });

    // データがあれば、UdpSocketの送信キュー(send_request)に突っ込む
    if let (Some((buf, len)), Some((addr, port))) = (data_to_send, target) {
        socket.send_request(addr, port, &buf[..len]);
    }
}

#[macro_export]
macro_rules! udp_println {
    ($($arg:tt)*) => {{
        critical_section::with(|cs| {
            let mut state = UDP_LOGGER_STATE.borrow_ref_mut(cs);
            // ターゲットが設定されている場合のみ処理
            if state.target.is_some() {
                use core::fmt::Write;
                
                // バッファに書き込み（フォーマットエラーや容量オーバー時は一度クリアして最新を優先）
                if write!(&mut state.buffer, $($arg)*).is_err() {
                    state.buffer.clear();
                    let _ = write!(&mut state.buffer, $($arg)*); 
                }
                let _ = write!(&mut state.buffer, "\n"); // 改行を追加
            }
        });
    }};
}

const FREQUENCY: u32 = 500;
const PERIOD_US: u64 = 1_000_000 / FREQUENCY as u64;
const DT: f32 = 1.0 / FREQUENCY as f32;
/// ピッチオフセット [rad]（重心バランス点）
const PITCH_OFFSET: f32 = -0.125;

// ── モーター物理定数 ──
/// トルク定数 [Nm/V] (FM90 datasheet: 0.01471Nm / 6V)
const K_TAU: f32 = 0.002452;
/// 逆起電力定数 [Nm·s/rad] (FM90 datasheet: 0.01471Nm / 13.61rad/s)
const K_B: f32 = 0.001081;
/// ホイール半径 [m]
const WHEEL_RADIUS: f32 = 0.03;
/// モーター効率 [-]
const MOTOR_EFF: f32 = 1.0;
/// クーロン摩擦トルク [Nm]
/// 旧deadzone閾値30PWMから逆算: 30/300 × η·k_τ·V_batt ≈ 0.00123 Nm (V_batt≈5V)
const TAU_COULOMB: f32 = 0.00123;

const MOTION_DIV: u32 = 1;
const DISPLAY_DIV: u32 = 10;
const TEMP_DIV: u32 = 500;
const MAG_DIV: u32 = 10;
const PRINT_DIV: u32 = 50;
const RLS_DIV: u32 = 500;

static TIMER0: Mutex<RefCell<Option<Timg>>> = Mutex::new(RefCell::new(None));
pub static TIMER_COUNTER: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

/// 指令トルク [Nm] → PWM値 (back-EMF逆算)
///
/// モーターモデル: τ = η·k_τ·(V_batt·PWM/PWM_max) - k_b·ω_wheel
/// 逆算: PWM = (τ_desired + k_b·ω_wheel) · PWM_max / (η·k_τ·V_batt)
fn torque_to_pwm(tau_desired: f32, omega_wheel: f32, v_batt: f32, pwm_max: f32) -> f32 {
    let denom = MOTOR_EFF * K_TAU * v_batt;
    if denom.abs() < 1e-9 {
        return 0.0;
    }
    (tau_desired + K_B * omega_wheel) * pwm_max / denom
}

/// クーロン摩擦補償トルク [Nm]
///
/// 動いている場合: τ_coulomb · sign(ω_wheel)
/// 停止時: min(|τ_applied|, τ_static) · sign(τ_applied)
fn coulomb_friction(omega_wheel: f32, tau_applied: f32) -> f32 {
    const OMEGA_THRESHOLD: f32 = 0.1; // 停止判定閾値 [rad/s]

    if fabsf(omega_wheel) > OMEGA_THRESHOLD {
        copysignf(TAU_COULOMB, omega_wheel)
    } else {
        if fabsf(tau_applied) > 1e-4 {
            copysignf(TAU_COULOMB, tau_applied)
        } else {
            0.0
        }
    }
}

// ============================================================
// [CHANGED] 物理モデル定数 (Python transform_v1 の出力、固定)
// これらは物理パラメータのみに依存し、Q/Rには無関係
// ============================================================

/// A₁₁ (6×6): 正則変換後の非駆動部サブシステム
/// z₁ = [x_pos, θ_pitch, ψ_yaw, w, ∫e_pos, ∫e_yaw]
const MODEL_A11: [[f32; 6]; 6] = [
    [0.0, 0.0,       0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0,       0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0,       0.0, 0.0, 0.0, 0.0],
    [0.0, 58.167616, 0.0, 0.0, 0.0, 0.0],
    [-1.0, 0.0,      0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0,      -1.0, 0.0, 0.0, 0.0],
];

/// A₁₂ (6×2): 駆動部 → 非駆動部の結合
const MODEL_A12: [[f32; 2]; 6] = [
    [1.0,       0.0],
    [-14.513014, 0.0],
    [0.0,       1.0],
    [0.0,       0.0],
    [0.0,       0.0],
    [0.0,       0.0],
];

/// T (8×8): 正則変換行列  z = T · x_aug
const MODEL_T: [[f32; 8]; 8] = [
    [1.0, 0.0,       0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0,       1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0,       0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    [0.0, 14.513014, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0,       0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0,       0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    [0.0, 1.0,       0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0,       0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
];

/// A_aug (8×8): 拡大系のシステム行列
const MODEL_A_AUG: [[f32; 8]; 8] = [
    [ 0.0,  1.0,    0.0, 0.0,  0.0, 0.0, 0.0, 0.0],
    [ 0.0,  0.0,  -1.72, 0.0,  0.0, 0.0, 0.0, 0.0],
    [ 0.0,  0.0,    0.0, 1.0,  0.0, 0.0, 0.0, 0.0],
    [ 0.0,  0.0,  83.13, 0.0,  0.0, 0.0, 0.0, 0.0],
    [ 0.0,  0.0,    0.0, 0.0,  0.0, 1.0, 0.0, 0.0],
    [ 0.0,  0.0,    0.0, 0.0,  0.0, 0.0, 0.0, 0.0],
    [-1.0,  0.0,    0.0, 0.0,  0.0, 0.0, 0.0, 0.0],
    [ 0.0,  0.0,    0.0, 0.0, -1.0, 0.0, 0.0, 0.0],
];

/// B_aug (8×2): 拡大系の入力行列
const MODEL_B_AUG: [[f32; 2]; 8] = [
    [       0.0,         0.0],
    [     282.0,       282.0],
    [       0.0,         0.0],
    [   -4092.67,    -4092.67],
    [       0.0,         0.0],
    [   13717.42,   -13717.42],
    [       0.0,         0.0],
    [       0.0,         0.0],
];

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

    info!("Network initialize: 192.168.4.1");

    esp_rtos::start(timg1.timer0);
    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    let (mut wifi_controller, wifi_interfaces) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");
    let ap_config = ModeConfig::AccessPoint(
        AccessPointConfig::default()
            .with_ssid("pendulum".to_string())
    );
    wifi_controller.set_config(&ap_config).unwrap();
    wifi_controller.start().unwrap();
    let mut ap_device = wifi_interfaces.ap;

    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let socket_set = SocketSet::new(&mut socket_set_entries[..]);
    let now = || Instant::now().duration_since_epoch().as_millis();
    let rng_for_net = esp_hal::rng::Rng::new();

    fn create_interface(device: &mut esp_radio::wifi::WifiDevice) -> smoltcp::iface::Interface {
        smoltcp::iface::Interface::new(
            smoltcp::iface::Config::new(smoltcp::wire::HardwareAddress::Ethernet(
                smoltcp::wire::EthernetAddress::from_bytes(&device.mac_address()),
            )),
            device,
            smoltcp::time::Instant::from_micros(
                esp_hal::time::Instant::now().duration_since_epoch().as_micros() as i64,
            ),
        )
    }
    let mut iface = create_interface(&mut ap_device);
    let mut net_stack = NetworkStack::new(
        iface,
        ap_device,
        socket_set,
        now,
        rng_for_net.random(),
    );
    // 固定IP設定（これがないとwork()でIPが設定されない）
    use blocking_network_stack::ipv4::{Configuration, ClientConfiguration, ClientSettings, Subnet, Mask};
    net_stack.update_iface_configuration(
        &Configuration::Client(ClientConfiguration::Fixed(ClientSettings {
            ip: core::net::Ipv4Addr::new(192, 168, 4, 1),
            subnet: Subnet {
                gateway: core::net::Ipv4Addr::new(192, 168, 4, 1),
                mask: Mask(24),
            },
            dns: None,
            secondary_dns: None,
        }))
    ).unwrap();

    let mut udp_rx_meta = [smoltcp::socket::udp::PacketMetadata::EMPTY; 1];
    let mut udp_tx_meta = [smoltcp::socket::udp::PacketMetadata::EMPTY; 1];
    let mut udp_rx_buffer = [0u8; 1024];
    let mut udp_tx_buffer = [0u8; 1024];

    let mut udp_socket = net_stack.get_udp_socket(
        &mut udp_rx_meta,
        &mut udp_rx_buffer,
        &mut udp_tx_meta,
        &mut udp_tx_buffer,
    );
    udp_socket.bind(45678).unwrap();

    let target_ip = smoltcp::wire::IpAddress::Ipv4(smoltcp::wire::Ipv4Address::new(192, 168, 4, 2));
    udp_logger_init(target_ip, 5000);

    info!("periph init start");

    let button = Input::new(peripherals.GPIO41, InputConfig::default().with_pull(Pull::Up));

    //IR GPIO47

    let i2c0 = I2c::new(peripherals.I2C0, 
        I2cConfig::default()
            .with_frequency(Rate::from_khz(400))
        ).unwrap()
            .with_sda(peripherals.GPIO45)
            .with_scl(peripherals.GPIO0);
    let i2c0_ref_cell = RefCell::new(i2c0);

    // LP5562
    let mut lp5562 = lp5562::Lp5562::new(I2cRefCellDevice::new(&i2c0_ref_cell));
    lp5562.init(&mut |us| delay.delay_micros(us)).unwrap();
    lp5562.set_current(lp5562::Channel::White, 255).unwrap();
    lp5562.set_pwm(lp5562::Channel::White, 255).unwrap();
    info!("lp5562 ok");

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

    info!("display ok");

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
            aux_odr: bmi270::AuxOdr::Hz50,
        }, 
        &mut |us| delay.delay_micros(us)
    ).unwrap();

    /* calibration */
    imu.calibrate_acc(bmi270::CalibAccConfig::z_up(), |us| delay.delay_micros(us)).unwrap();
    imu.write_acc_offset((33, -123, -144));

    let calib = bmi270::GyrTempCalibration::from_two_points(
        29.734032, (-3, 2, -5), 
        47.609695, (-5, -13, -13));
    imu.set_gyr_temp_calibration(calib, 1.0);

    // 空読み（必須 & 重要）
    for _ in 0..3 {
        let _ = imu.read_mag().unwrap().unwrap();
        delay.delay_millis(50);
    }

    use alloc::format;

    let mag_calib = mag_calib::MagCalibration::new(
        [80.2899, -1077.7451, -767.7703],
        [[0.025567, 0.000170, -0.001187],
        [0.0000, 0.014415, 0.001860],
        [0.0000, 0.0000, 0.023254]],
    );

    let mut rls = mag_rls::MagOnlineRls::new(&mag_calib);

    let mut lpf_accel = lpf::Lpf3::new(50.0, DT);
    let mut lpf_gyro  = lpf::Lpf3::new(100.0, DT);
    let mut lpf_mag   = lpf::Lpf3::new(5.0,  DT);

    let mut ekf = mekf::ImuEkf::new(
        mekf::EkfConfig {
            dt: DT,
            gyro_noise: 0.05,
            gyro_bias_noise: 0.001,
            accel_noise: 0.15,
            mag_noise: 0.3,
            ..Default::default()
    });

    let mut dkf = dob_kf::DobKf::new(dob_kf::DobKfConfig {
        dt: DT,
        inertia: 0.000363,
        mgl: 0.1 * 9.81 * 0.035,
        q_omega: 0.1,
        q_disturbance: 1e-8,
        r_gyro: 0.001,
        disturbance_limit: 0.05,
        ..Default::default()
    });
    info!("imu ok");

    let i2c1 = I2c::new(peripherals.I2C1, 
        I2cConfig::default()
            .with_frequency(Rate::from_khz(400))
        ).unwrap()
            .with_sda(peripherals.GPIO38)
            .with_scl(peripherals.GPIO39);
    let i2c1_ref_cell = RefCell::new(i2c1);

    let mut motion = atom_motion::AtomMotion::new(I2cRefCellDevice::new(&i2c1_ref_cell));
    let mut ina226 = ina226::Ina226::new(I2cRefCellDevice::new(&i2c1_ref_cell));
    ina226.configure(
        ina226::Averages::Avg16,
        ina226::ConversionTime::Time1100us,
        ina226::ConversionTime::Time1100us,
        ina226::Mode::ShuntBusContinuous,
    ).unwrap();
    ina226.calibrate(0.02, 8.192).unwrap();
    let v = ina226.bus_voltage().unwrap();
    let i = ina226.current().unwrap();
    info!("v:{}, i:{}", v, i);

    let mut v_batt = v;

    let mut face = face::Face::new(Point::new(64, 64), 50, 30);

    // [CHANGED] LQ-STSMC: Q, R をここで調整するだけ
    // z₁ = [x_pos, θ_pitch, ψ_yaw, w, ∫e_pos, ∫e_yaw]
    info!("Solving CARE...");
    let mut lq_ctrl = lq_stsmc::LqStsmc::new(lq_stsmc::LqStsmcConfig {
        dt: DT,
        // 物理モデル定数
        a11: MODEL_A11,
        a12: MODEL_A12,
        t_mat: MODEL_T,
        a_aug: MODEL_A_AUG,
        b_aug: MODEL_B_AUG,
        // --- ここを調整 ---
        q_diag: [
            1.0 / (0.3 * 0.3),    // x_pos:    許容 0.3 m
            1.0 / (0.05 * 0.05),   // θ_pitch:  許容 0.05 rad (~3°)
            1.0 / (0.2 * 0.2),    // ψ_yaw:    許容 0.2 rad (~11°)
            1.0 / (5.0 * 5.0),    // w:        許容 5.0
            1.0 / (1.0 * 1.0),    // ∫e_pos:   許容 1.0 m·s
            1.0 / (1.0 * 1.0),    // ∫e_yaw:   許容 1.0 rad·s
        ],
        r_diag: [
            1.0 / (0.5 * 0.5),    // v_pos:    許容 0.5 m/s
            1.0 / (3.0 * 3.0),    // ω_yaw:    許容 3.0 rad/s
        ],
        // --- ここまで ---
        ch: [
            // ch[0]: pitch/position channel
            lq_stsmc::StsmcChannelConfig {
                lambda: 0.1,
                alpha: 0.01,
                epsilon: 0.05,
                v_leak: 0.01,
                v_limit: 50.0,
            },
            // ch[1]: yaw channel
            lq_stsmc::StsmcChannelConfig {
                lambda: 0.5,
                alpha: 0.05,
                epsilon: 0.05,
                v_leak: 0.01,
                v_limit: 50.0,
            },
        ],
        tau_limit: 0.5,
    });
    info!("CARE solved, LQ-STSMC ready");

    let mut gravity = gravity::GravityCompensator::new(0.1, 0.035);
    let mut pos_ekf = pos_ekf::PositionEkf::new(pos_ekf::PosEkfConfig {
        dt: DT,

        wheel_radius: WHEEL_RADIUS,
        m_p: 0.1,
        m_w: 0.023,
        i_p: 0.000363,
        i_w: 0.00001035,
        l: 0.035,
        imu_rx: 0.01,
        imu_rz: 0.08,

        j_max: 400.0,
        a_max: 50.0,
        v_max: 5.0,

        command_lpf_tau: 0.003183,

        r_accel: 0.01,
        constraint_r_scale: 2.0,

        q_a: 10.0,
        q_v: 1e-3,
        q_x: 1e-4,
        q_bias: 1e-4,

        ..Default::default()
    });
    let mut smd = ista_smd::ImplicitSmd::new(33.54, 550.0, DT);
    smd.init_state(0.0);

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
    let mut counter = 0;

    let mut gyro_lpf = 0.0;
    let mut smd_lpf = 0.0;
    let mut prev_gyro_lpf = 0.0;
    let mut prev_velocity = 0.0f32;
    let mut prev_total_pitch_torque = 0.0f32;

    info!("Start!");
    loop {
        display.poll();
        udp_logger_poll(&mut udp_socket);
        if events::has_pending_events() {
            while let Some(event) = events::get_event() {
                match event {
                    events::Event::MotionUpdate => {
                        if counter % TEMP_DIV == 0 {
                            imu.update_gyr_temp_compensation(DT * TEMP_DIV as f32).unwrap();
                        }
                        let d = imu.read_immu().unwrap();
                        let (ax, ay, az) = (d.accel.z, -d.accel.y, d.accel.x);
                        let (gx, gy, gz) = mekf::degree_to_rad((d.gyro.z, -d.gyro.y, d.gyro.x));
                        let (ax, ay, az) = lpf_accel.update(ax, ay, az);
                        let (gx, gy, gz) = lpf_gyro.update(gx, gy, gz);
                        let state = ekf.update(ax, ay, az, gx, gy, gz);
                        
                        let mag_raw = d.mag.unwrap();
                        if counter % RLS_DIV == 0 {
                            rls.update([mag_raw.x, mag_raw.y, mag_raw.z]);
                        }
                        let mag = rls.apply([mag_raw.x, mag_raw.y, mag_raw.z]);
                        let mag_bmi_axis = [mag[1], mag[0], -mag[2]];
                        let (mx, my, mz) = (mag_bmi_axis[2], -mag_bmi_axis[1], mag_bmi_axis[0]);
                        let (mx, my, mz) = lpf_mag.update(mx, my, mz);
                        if counter % MAG_DIV == 0 {
                            // ekf.update_mag_yaw(mx, my, mz);
                        }
                        let now_angle = state.pitch - PITCH_OFFSET;

                        if counter % PRINT_DIV == 0 {
                            udp_println!(
                                "{:.3},{:.3}", v, i,
                            );
                        }
                        counter += 1;

                        if button.is_low() && !button_state  {
                            drive = !drive;
                        }
                        button_state = button.is_low();

                        if state.roll.abs() > 0.5 || now_angle.abs() > 0.5 {
                            drive = false;
                        }

                        if drive {
                            let dkf_state = dkf.update(now_angle, state.pitch_rate);

                            let alpha_smd = DT / (0.00318 + DT);
                            smd_lpf += alpha_smd * (state.pitch_rate - smd_lpf);
                            let smd_angular_accel = smd.update(smd_lpf);
                            let p_state = pos_ekf.update(
                                prev_total_pitch_torque,
                                ax, az, state.pitch, now_angle,
                                state.pitch_rate, smd_angular_accel,
                            );

                            // LQ-STSMC: 6状態 → [τ_L, τ_R]
                            let x6 = [
                                p_state.position,
                                p_state.velocity,
                                now_angle,
                                state.pitch_rate,
                                state.continuous_yaw,
                                state.yaw_rate,
                            ];
                            let lq_out = lq_ctrl.update(&x6, 0.0, 0.0);

                            // フィードフォワード（ピッチ方向、両輪均等）
                            let tau_gravity = gravity.update(now_angle);
                            let tau_disturbance = dkf_state.disturbance;
                            let tau_ff = tau_gravity + tau_disturbance;

                            let tau_l = lq_out.tau_l + tau_ff;
                            let tau_r = lq_out.tau_r + tau_ff;

                            // クーロン摩擦補償
                            let omega_wheel = prev_velocity / WHEEL_RADIUS;
                            let tau_l_final = tau_l + coulomb_friction(omega_wheel, tau_l);
                            let tau_r_final = tau_r + coulomb_friction(omega_wheel, tau_r);

                            let total_pitch_torque = tau_l_final + tau_r_final;
                            prev_total_pitch_torque = total_pitch_torque;
                            prev_velocity = p_state.velocity;

                            dkf.set_control_torque(total_pitch_torque);

                            // PWM変換
                            let atom_max = atom_motion::MOTOR_SPEED_MAX as f32;
                            let m2_out = torque_to_pwm(tau_l_final, omega_wheel, v_batt, atom_max);
                            let m1_out = -torque_to_pwm(tau_r_final, omega_wheel, v_batt, atom_max);
                            
                            m1_pwm = m1_out.clamp(-atom_max, atom_max) as i8;
                            m2_pwm = m2_out.clamp(-atom_max, atom_max) as i8;

                            if counter % PRINT_DIV == 0 {
                                let ps = p_state.clone();
                                // udp_println!(
                                //     "{:.3}", v
                                // );
                            }
                        } else {
                            m1_pwm = 0;
                            m2_pwm = 0;
                            lq_ctrl.reset();
                            pos_ekf.reset();
                            ekf.reset_yaw();
                            dkf.reset();
                            prev_velocity = 0.0;
                            prev_total_pitch_torque = 0.0;
                        }
                        motion.set_motor(atom_motion::MotorChannel::M1, m1_pwm).unwrap();
                        motion.set_motor(atom_motion::MotorChannel::M2, m2_pwm).unwrap();

                        let counter = critical_section::with(|cs| {
                            TIMER_COUNTER.borrow(cs).get()
                        });
                        if counter % (FREQUENCY / 2) == 0 {
                            let emotion = if drive {
                                face::Emotion::UPPER[pseudo_rand(face::Emotion::UPPER.len())]
                            } else {
                                face::Emotion::LOWER[pseudo_rand(face::Emotion::LOWER.len())]
                            };
                            face.set_emotion(emotion);
                        }
                    }
                    events::Event::DisplayUpdate => {
                        if display.is_idle() {
                            display.clear(Rgb565::BLACK).unwrap();

                            face.update(&mut display, DT * DISPLAY_DIV as f32).unwrap();

                            display.flush_async().unwrap();
                        }
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

            if counter % MOTION_DIV == 0 {
                events::post_event(events::Event::MotionUpdate);
            }
            if counter % DISPLAY_DIV == 0 {
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