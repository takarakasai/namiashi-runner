//! `dump` — 実機なしで歩容を再生し、関節角が可動域に収まるか確かめる。
//!
//! モータを繋ぐ前にここで可動域を超える指令が出ていないか見ておくのが、
//! いちばん安い安全確認になる。状態機械そのもの（脱力 → 初期姿勢 → 立ち姿勢
//! → 歩容）を通すので、遷移の途中で行き過ぎる場合もここに出る。

use std::time::{Duration, Instant};

use namiashi_hal::imu::ImuSample;
use namiashi_hal::joint::JOINT_NAMES;

use crate::config::AppConfig;
use crate::controller::{Controller, State};
use crate::jointvec::JointVec;
use crate::teleop::{GaitSelect, ModeRequest, OperatorCommand};
use crate::viz::{self, VizConfig};
use crate::Cli;

pub fn run(cfg: &AppConfig, cli: &Cli) -> Result<(), String> {
    let gait = match cli.str("gait").unwrap_or("crawl") {
        "crawl" => GaitSelect::Crawl,
        "walk" => GaitSelect::Walk,
        "trot" => GaitSelect::Trot,
        other => return Err(format!("未知の歩容 {other:?}（crawl|walk|trot）")),
    };
    let vx = cli.f64("vx").unwrap_or(0.05);
    let vy = cli.f64("vy").unwrap_or(0.0);
    let wz = cli.f64("wz").unwrap_or(0.0);
    let seconds = cli.f64("secs").unwrap_or(4.0);
    let every = cli.usize("every").unwrap_or(20).max(1);
    let viz_cfg = crate::viz_config(cli);
    // 可視化するときは実時間で流さないと早送りになる。--realtime は
    // --viz の有無に関わらず指定できる（表示を目で追いたいときにも使う）。
    let realtime = cli.flag("realtime") || viz_cfg.enabled;

    let robot = crate::robot::load_from_config(cfg)?;
    let mut controller = Controller::new(robot, cfg.clone());
    let dt = 1.0 / cfg.control.rate_hz;
    let imu = level_imu();

    let mut cmd = OperatorCommand {
        vx_m_s: 0.0,
        vy_m_s: 0.0,
        wz_rad_s: 0.0,
        height_offset_m: 0.0,
        arm_rad: None,
        mode: ModeRequest::Stand,
        gait,
        play_pose: false,
        chicken_head: false,
        link_ok: true,
    };

    println!(
        "歩容 {} / v=({vx:+.3}, {vy:+.3}, {wz:+.3}) / {:.0} Hz / {seconds:.1} s",
        gait.label(),
        cfg.control.rate_hz
    );
    println!("t[s]   状態         {}", header());

    let mut publisher = open_viz(&viz_cfg)?;
    let mut violations: Vec<String> = Vec::new();
    let steps = (seconds / dt).ceil() as usize;
    let period = Duration::from_secs_f64(dt);
    let mut next = Instant::now();
    for i in 0..steps {
        let t = i as f64 * dt;
        // 立ち上がってから歩き出す。遷移が終わるまで速度は入れない。
        if controller.state() == State::Active {
            cmd.mode = ModeRequest::Walk;
            cmd.vx_m_s = vx;
            cmd.vy_m_s = vy;
            cmd.wz_rad_s = wz;
        }
        let out = controller.tick(&cmd, &JointVec::zeros(), &imu, dt);
        check_limits(cfg, &out.targets, t, &mut violations);
        if i % every == 0 {
            println!("{t:5.2}  {:<12} {}", out.state.label(), row(&out.targets));
        }
        if let Some(p) = publisher.as_mut() {
            // 実機を持たない机上再生なので planned だけ。受け側はゴーストを
            // 描かず、この 1 本でモデルを駆動する。
            let body = controller.body_view();
            p.maybe_publish(|seq| viz::Frames::planned(viz::frame(seq, t, &out.targets, &body)));
        }
        if realtime {
            next += period;
            let now = Instant::now();
            if next > now {
                std::thread::sleep(next - now);
            } else {
                next = now;
            }
        }
    }

    if violations.is_empty() {
        println!("\n可動域: すべて範囲内");
        Ok(())
    } else {
        println!("\n可動域を超えた指令が {} 件あります:", violations.len());
        // 全部出すと埋もれるので先頭だけ。件数は上に出してある。
        for v in violations.iter().take(20) {
            println!("  {v}");
        }
        Err("可動域を超える指令が出ています。gait の stance_height_m / \
             swing_height_m と config の可動域を見直してください"
            .into())
    }
}

/// [`crate::runner::open_viz`] と同じ。無効なら `None`。
fn open_viz(cfg: &VizConfig) -> Result<Option<viz::Publisher>, String> {
    if !cfg.enabled {
        return Ok(None);
    }
    viz::Publisher::new(cfg).map(Some)
}

fn header() -> String {
    let mut s = String::new();
    for names in JOINT_NAMES.iter() {
        s.push_str(&format!("{:<21}", &names[0][..2]));
    }
    s
}

fn row(q: &JointVec) -> String {
    let mut s = String::new();
    for leg in q.legs.iter() {
        s.push_str(&format!("{:+.3} {:+.3} {:+.3}  ", leg[0], leg[1], leg[2]));
    }
    s
}

/// 可動域は実機設定 (`hardware.legs.bus[].motors[]`) が持っているものを使う。
/// モデルの `<limit>` ではなく実機の設定を見るのは、実際にクランプするのが
/// そちらだから。
fn check_limits(cfg: &AppConfig, q: &JointVec, t: f64, out: &mut Vec<String>) {
    for bus in &cfg.hardware.legs.bus {
        let Ok(slot) = bus.leg_slot() else { continue };
        for (k, motor) in bus.motors.iter().enumerate() {
            let value = q.legs[slot.index()][k];
            if value < motor.min_rad || value > motor.max_rad {
                out.push(format!(
                    "t={t:5.2}s {}_{}_joint = {value:+.4} rad（範囲 {:+.3}..{:+.3}）",
                    bus.leg, motor.kind, motor.min_rad, motor.max_rad
                ));
            }
        }
    }
}

fn level_imu() -> ImuSample {
    ImuSample {
        rpy_rad: [0.0; 3],
        gyro_rad_s: [0.0; 3],
        accel_m_s2: [0.0, 0.0, 9.80665],
        temperature_c: 25.0,
        stamp: std::time::Instant::now(),
    }
}
