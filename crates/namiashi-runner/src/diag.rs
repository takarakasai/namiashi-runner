//! 実機を**動かさずに**確かめるコマンド群。
//!
//! 立ち上げの順番として、まずここが全部通ることを確認してから `run` へ行く。
//! 脚の指令は一切出さず、状態読み出しと受信だけを行う。

use std::time::{Duration, Instant};

use namiashi_hal::ch348;
use namiashi_hal::config::HardwareConfig;
use namiashi_hal::imu::ImuReader;
use namiashi_hal::legs::LegArray;
use namiashi_hal::sbus::SbusReceiver;

use crate::config::AppConfig;
use crate::teleop::{Teleop, TeleopConfig};

/// `ports` — CH348 のポートを物理 UART 番号つきで並べる。何も開かない。
pub fn ports() -> Result<(), String> {
    let ports = ch348::list_ports().map_err(|e| e.to_string())?;
    if ports.is_empty() {
        println!("CH348 のポートが見つかりません（基板の USB は挿さっていますか）");
        return Ok(());
    }
    println!("UART  デバイス                役割");
    for p in &ports {
        println!(
            "{:>4}  {:<24}  {}",
            p.uart_index,
            p.path.display(),
            role_of(p.uart_index)
        );
    }
    Ok(())
}

fn role_of(uart: u16) -> &'static str {
    match uart {
        0 => "LEG1 (FL) RS485",
        1 => "LEG2 (FR) RS485",
        2 => "LEG3 (RL) RS485",
        3 => "LEG4 (RR) RS485",
        4 => "ARMA (RS485/TTL)",
        5 => "IMU (TTL)",
        6 => "S.BUS (受信専用)",
        7 => "ARMB (RS485/TTL)",
        _ => "-",
    }
}

/// リンク状態と、落ちている場合はその理由。
///
/// `link=false` だけだと「送信機が OFF」「電波が弱い」「受信が途切れた」の
/// どれなのか分からず、立ち上げで切り分けられない。受信機はフェイルセーフ中でも
/// フレームを送り続ける（`sbus/doc/spec.md` §6.2: 送信機 OFF でも 66.5 fps 継続）
/// ので、**fps が出ていることはリンクが生きている証拠にならない。**
fn link_status(state: &namiashi_hal::sbus::SbusState, ok: bool) -> String {
    if ok {
        return "link=OK".to_string();
    }
    let mut why: Vec<&str> = Vec::new();
    if state.failsafe {
        why.push("FAILSAFE");
    }
    if state.frame_lost {
        why.push("FRAME_LOST");
    }
    if why.is_empty() {
        // フラグは立っていないのに使えない = 最後のフレームから
        // control.teleop_timeout_ms 以上経っている。
        why.push("TIMEOUT");
    }
    format!("link=NG({})", why.join("+"))
}

/// S.BUS2 テレメトリの表示。
///
/// **送信機（プロポ本体）のバッテリ電圧は取れない。** テレメトリは
/// 受信機 → 送信機の向きに流れるもので、送信機自身の電池電圧は送信機の画面が
/// 出しているだけで S.BUS 線には乗らない。ここで読めるのは受信機側の 2 つ:
///
/// - `Rx-Batt` — 受信機の電源電圧（スロット 0 marker `0xC0`）
/// - `Ext-Volt` — 受信機の外部電圧入力（同 `0xC4`）。主電源を分圧して入れておけば
///   走行用バッテリの電圧がここに出る
///
/// どちらも **S.BUS2 でないと来ない**（S.BUS1 にはテレメトリスロットが無い）。
/// 未受信は `---` で、0 V と紛れないようにする。
fn telemetry(state: &namiashi_hal::sbus::SbusState) -> String {
    fn volts(value: Option<f32>) -> String {
        match value {
            Some(v) => format!("{v:.1}V"),
            None => "---".to_string(),
        }
    }
    if !state.sbus2 {
        return "S.BUS1(テレメトリ無)".to_string();
    }
    // 未知 marker は捨てられて external_v が**古い値のまま残る**。黙って
    // 古い電圧を信じないよう、増えていることを常に見せる。
    let unknown = if state.counters.unknown_slots > 0 {
        format!(" unknown={}", state.counters.unknown_slots)
    } else {
        String::new()
    };
    format!(
        "S.BUS2 Rx-Batt={} Ext-Volt={}{}",
        volts(state.rx_battery_v),
        volts(state.external_v),
        unknown
    )
}

/// `sbus` — プロポの入力を表示し続ける。モータには触れない。
pub fn sbus(cfg: &AppConfig, seconds: f64) -> Result<(), String> {
    let rx = SbusReceiver::connect(&cfg.hardware.sbus).map_err(|e| e.to_string())?;
    println!("{} で受信中（{seconds:.0} 秒）", rx.port());
    rx.wait_ready(Duration::from_secs(3))
        .map_err(|e| format!("{e}。送信機の電源を確認してください"))?;

    let mut teleop = Teleop::new(cfg.teleop.clone(), &cfg.gait, &cfg.hardware.arm);
    let timeout = Duration::from_millis(cfg.control.teleop_timeout_ms);
    let start = Instant::now();
    while start.elapsed().as_secs_f64() < seconds {
        let state = rx.state();
        let cmd = teleop.update(&state, state.is_usable(timeout));
        let raw: Vec<String> = state.channels[..8]
            .iter()
            .enumerate()
            .map(|(i, v)| format!("{}:{v:>4}", i + 1))
            .collect();
        println!(
            "{}  |  v=({:+.3},{:+.3},{:+.3}) h={:+.3} mode={:?} gait={} pose={} chicken={} \
             arm={} {} {} {:.0}fps frames={} desync={}",
            raw.join(" "),
            cmd.vx_m_s,
            cmd.vy_m_s,
            cmd.wz_rad_s,
            cmd.height_offset_m,
            cmd.mode,
            cmd.gait.label(),
            cmd.play_pose,
            cmd.chicken_head,
            match cmd.arm_rad {
                Some(q) => format!("{q:+.3}rad"),
                None => "-".to_string(),
            },
            link_status(&state, cmd.link_ok),
            telemetry(&state),
            state.fps,
            state.counters.frames,
            state.counters.desync_bytes,
        );
        std::thread::sleep(Duration::from_millis(200));
    }
    let state = rx.state();
    if !state.sbus2 {
        println!(
            "\n注意: S.BUS1 で受信しています（テレメトリスロットが来ないので\n\
             Rx-Batt / Ext-Volt は取れません）。受信機の S.BUS2 ポートに繋いでください。"
        );
    } else if state.rx_battery_v.is_none() && state.external_v.is_none() {
        println!(
            "\n注意: S.BUS2 ですがスロット 0 が一度も来ていません（slots={} unknown={}）。",
            state.counters.slots, state.counters.unknown_slots
        );
    }
    Ok(())
}

/// `imu` — IMU の値を表示し続ける。
pub fn imu(cfg: &AppConfig, seconds: f64) -> Result<(), String> {
    let reader = ImuReader::connect(&cfg.hardware.imu).map_err(|e| e.to_string())?;
    println!("{} で受信中（{seconds:.0} 秒）", reader.port());
    reader
        .wait_ready(Duration::from_secs(3))
        .map_err(|e| e.to_string())?;
    let start = Instant::now();
    while start.elapsed().as_secs_f64() < seconds {
        let s = reader.sample_or_level();
        let st = reader.stats();
        println!(
            "rpy=({:+7.2},{:+7.2},{:+7.2})°  gyro=({:+7.2},{:+7.2},{:+7.2})°/s  \
             |a|={:.3} m/s²  {:.0}Hz resync={} err={}",
            s.rpy_rad[0].to_degrees(),
            s.rpy_rad[1].to_degrees(),
            s.rpy_rad[2].to_degrees(),
            s.gyro_rad_s[0].to_degrees(),
            s.gyro_rad_s[1].to_degrees(),
            s.gyro_rad_s[2].to_degrees(),
            (s.accel_m_s2.iter().map(|v| v * v).sum::<f64>()).sqrt(),
            st.rate_hz,
            st.resync_bytes,
            st.errors,
        );
        std::thread::sleep(Duration::from_millis(200));
    }
    Ok(())
}

/// `legs` — 脚バスを開いて状態だけ読む。**指令は一切送らない。**
///
/// ここで各バスの実効周期が出るので、`control.rate_hz` をいくつにできるかが
/// 実測で決まる。
pub fn legs(cfg: &AppConfig, seconds: f64) -> Result<(), String> {
    let array = LegArray::connect(&cfg.hardware).map_err(|e| e.to_string())?;
    println!("脚バスを開きました（指令は送りません。{seconds:.0} 秒観測）");
    for bus in array.buses() {
        println!("  {} → {}", bus.leg().prefix(), bus.port());
    }
    let start = Instant::now();
    while start.elapsed().as_secs_f64() < seconds {
        std::thread::sleep(Duration::from_millis(500));
        for bus in array.buses() {
            let st = bus.stats();
            let s = bus.state();
            println!(
                "{} {:>6.1}Hz 最悪{:>5.2}ms err={:<4} q=[{:+.3} {:+.3} {:+.3}] \
                 T=[{:.0} {:.0} {:.0}]°C ok={}{}",
                bus.leg().prefix(),
                st.rate_hz,
                st.worst_cycle_s * 1e3,
                st.errors,
                s[0].position_rad,
                s[1].position_rad,
                s[2].position_rad,
                s[0].temperature_c,
                s[1].temperature_c,
                s[2].temperature_c,
                s.iter().all(|j| j.ok),
                if bus.last_error().is_empty() {
                    String::new()
                } else {
                    format!(" 直近エラー: {}", bus.last_error())
                }
            );
        }
        println!("--");
    }
    Ok(())
}

/// `check` — 設定とモデルだけを検証する。ハードウェアに触れない。
pub fn check(cfg: &AppConfig) -> Result<(), String> {
    cfg.validate()?;
    println!("設定: OK");

    let robot = crate::robot::load_from_config(cfg)?;
    println!(
        "モデル: {} （関節 {} / nq={}）",
        robot.model.name,
        robot.model.num_joints(),
        robot.model.nq
    );
    println!("ポーズ: {:?}", robot.poses.pose_names().collect::<Vec<_>>());
    println!(
        "シーケンス: {:?}",
        robot.poses.sequence_names().collect::<Vec<_>>()
    );
    if robot.poses.pose(&cfg.control.start_pose).is_none() {
        println!(
            "警告: 初期姿勢 {:?} がモデルにありません",
            cfg.control.start_pose
        );
    }
    if robot.poses.pose(&cfg.poses.greeting).is_none()
        && robot.poses.sequence(&cfg.poses.greeting).is_none()
    {
        println!(
            "警告: ポーズ再生の対象 {:?} がモデルにありません",
            cfg.poses.greeting
        );
    }
    println!("IK→モデル符号表: {:?}", robot.signs);
    println!(
        "運動学の基準姿勢 {:?}: q = {:?}",
        cfg.control.kinematics_pose,
        robot
            .home_q
            .iter()
            .map(|v| (v * 1e3).round() / 1e3)
            .collect::<Vec<_>>()
    );
    print_teleop(&cfg.teleop);
    print_wiring(&cfg.hardware);
    Ok(())
}

fn print_teleop(t: &TeleopConfig) {
    println!("プロポ割り当て:");
    println!(
        "  CH{} 前後  CH{} 左右  CH{} 旋回  CH{} 高さ",
        t.vx.channel, t.vy.channel, t.wz.channel, t.height.channel
    );
    println!(
        "  CH{} モード(脱力/起立/歩行)  CH{} 歩容(Crawl/Walk/Trot)  \
         CH{} ポーズ再生  CH{} チキンヘッド",
        t.mode.channel, t.gait.channel, t.pose.channel, t.chicken_head.channel
    );
    match &t.arm {
        Some(arm) => println!("  CH{} 腕（観測のみ。アプリからは駆動しない）", arm.channel),
        None => println!("  腕の観測チャンネルなし（腕の角度は不明のまま）"),
    }
}

fn print_wiring(hw: &HardwareConfig) {
    println!("配線:");
    for bus in &hw.legs.bus {
        let ids: Vec<String> = bus
            .motors
            .iter()
            .map(|m| format!("{}={}", m.kind, m.id))
            .collect();
        println!("  {} {} [{}]", bus.leg, bus.port.label(), ids.join(" "));
    }
    println!("  IMU {}", hw.imu.port.label());
    println!("  S.BUS {}", hw.sbus.port.label());
    let arm = namiashi_hal::arm::connect(&hw.arm).ok();
    let driven = arm.as_ref().map(|a| a.is_app_driven()).unwrap_or(false);
    println!(
        "  腕 {} protocol={:?} アプリから駆動={}",
        hw.arm.port.label(),
        hw.arm.protocol,
        if driven { "する" } else { "しない" }
    );
    if !driven {
        println!("    → チキンヘッドとポーズ再生の腕動作は無効です");
    }
}
