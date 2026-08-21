//! 実機の制御ループ。
//!
//! 周期の考え方は [`namiashi_hal::legs`] のとおり: 脚バス 4 本はそれぞれ自由
//! 走行していて、このループは共有スロットに目標を書き最新値を読むだけ。
//! したがってここで守るべきは「一定周期で回ること」だけで、バスの応答を
//! 待つ必要はない。

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use namiashi_hal::arm::ArmServo;
use namiashi_hal::ch348::PortMap;
use namiashi_hal::imu::ImuReader;
use namiashi_hal::joint::{JointCommand, JointMode};
use namiashi_hal::legs::{BusRequest, LegArray};
use namiashi_hal::sbus::SbusReceiver;

use crate::config::AppConfig;
use crate::controller::{Controller, State};
use crate::jointvec::JointVec;
use crate::robot::Robot;
use crate::teleop::Teleop;
use crate::viz::{self, VizConfig};

/// 実機に繋いだ一式。
pub struct Hardware {
    pub legs: LegArray,
    pub imu: ImuReader,
    pub sbus: SbusReceiver,
    pub arm: Box<dyn ArmServo>,
}

impl Hardware {
    /// 全ポートを開く。失敗したらそこまでに開いたものは drop で閉じる。
    ///
    /// **UART 番号の探索は最初に 1 回だけ。** 探索はデバイスを `open` する
    /// ので、1 本開くたびに調べ直すと 2 本目以降が自分自身の `EBUSY` で
    /// 失敗する（実機で踏んだ）。
    pub fn connect(cfg: &AppConfig) -> Result<Self, String> {
        let map = PortMap::discover().map_err(|e| e.to_string())?;
        let legs = LegArray::connect_with(&cfg.hardware, &map).map_err(|e| e.to_string())?;
        for bus in legs.buses() {
            log::info!("脚 {} → {}", bus.leg().prefix(), bus.port());
        }
        let imu = ImuReader::connect_with(&cfg.hardware.imu, &map).map_err(|e| e.to_string())?;
        log::info!("IMU → {}", imu.port());
        let sbus =
            SbusReceiver::connect_with(&cfg.hardware.sbus, &map).map_err(|e| e.to_string())?;
        log::info!("S.BUS → {}", sbus.port());
        let arm = namiashi_hal::arm::connect(&cfg.hardware.arm).map_err(|e| e.to_string())?;
        Ok(Self {
            legs,
            imu,
            sbus,
            arm,
        })
    }

    /// 12 軸の実測角を関節ベクトルにまとめる。
    pub fn measured(&self, arm: f64) -> JointVec {
        let states = self.legs.states();
        let mut q = JointVec::zeros();
        for (leg, leg_states) in states.iter().enumerate() {
            for (k, s) in leg_states.iter().enumerate() {
                q.legs[leg][k] = s.position_rad;
            }
        }
        q.arm = arm;
        q
    }
}

/// `run` サブコマンドの起動オプション。
pub struct RunOptions {
    /// S.BUS を待たずに起動する（受信機なしのベンチ確認用）。
    ///
    /// 受信が無い間は操縦指令がフェイルセーフ（速度 0 / 起立要求）になるので、
    /// **これを付けると electrically 立ち上がってしまう**。ベンチで脚を浮かせて
    /// いるときにだけ使うこと。
    pub allow_no_sbus: bool,
    /// 起動時にゼロ出しを行わない（すでに出してある場合）。
    pub skip_zero: bool,
    /// 状態表示の間隔 (s)。0 で表示しない。
    pub status_interval_s: f64,
    /// ライブ可視化（articara へ Zenoh 配信）。
    pub viz: VizConfig,
}

impl Default for RunOptions {
    fn default() -> Self {
        Self {
            allow_no_sbus: false,
            skip_zero: false,
            status_interval_s: 1.0,
            viz: VizConfig::default(),
        }
    }
}

/// 制御ループ本体。Ctrl-C か致命的エラーで戻る。
pub fn run(cfg: AppConfig, robot: Robot, opts: RunOptions) -> Result<(), String> {
    let mut hw = Hardware::connect(&cfg)?;

    // 受信機を待つ。プロポが無い状態で起立させないための入口チェック。
    match hw.sbus.wait_ready(Duration::from_secs(3)) {
        Ok(_) => log::info!("S.BUS 受信を確認しました"),
        Err(e) if opts.allow_no_sbus => {
            log::warn!("S.BUS が来ていません ({e})。--allow-no-sbus 指定のため続行します")
        }
        Err(e) => {
            return Err(format!(
                "{e}。送信機の電源とポートを確認してください（ベンチで脚を浮かせて \
                 いるなら --allow-no-sbus）"
            ))
        }
    }
    match hw.imu.wait_ready(Duration::from_secs(2)) {
        Ok(_) => log::info!("IMU を確認しました"),
        // IMU はチキンヘッドと姿勢フィードバックにしか使っていないので、
        // 無くても歩容そのものは回る。止めずに警告に留める。
        Err(e) => log::warn!("IMU が来ていません ({e})。水平・静止として扱います"),
    }

    if !opts.skip_zero {
        log::warn!(
            "脚のゼロ出しを行います。**校正姿勢**（config の zero_pose_rad が指す姿勢）で \
             保持されていることを確認してください"
        );
        hw.legs
            .request_all(BusRequest::Zero)
            .map_err(|e| e.to_string())?;
        hw.legs
            .wait_anchored(Duration::from_secs(3))
            .map_err(|e| format!("{e}（モータの電源とボーレートを確認してください）"))?;
        log::info!("ゼロ出し完了");
    }

    let stop = install_signal_handler();
    let mut teleop = Teleop::new(cfg.teleop.clone(), &cfg.gait, &cfg.hardware.arm);
    let teleop_timeout = Duration::from_millis(cfg.control.teleop_timeout_ms);
    let period = Duration::from_secs_f64(1.0 / cfg.control.rate_hz);
    let arm_app_driven = hw.arm.is_app_driven();
    if !arm_app_driven {
        log::info!(
            "腕はアプリから駆動しません（{}）。目標角にはプロポからの観測値を置きます",
            if hw.arm.is_connected() {
                "受信機直結"
            } else {
                "未配線"
            }
        );
    }
    let mut controller = Controller::with_arm(robot, cfg.clone(), arm_app_driven);

    let mut publisher = open_viz(&opts.viz)?;
    let started = Instant::now();
    let mut motors_enabled = false;
    let mut next = Instant::now();
    let mut last_status = Instant::now();
    let mut worst_overrun = Duration::ZERO;
    let mut ticks: u64 = 0;

    log::info!(
        "制御ループ開始: {:.0} Hz（脚バス {:.0} Hz）。Ctrl-C で脱力して終了します",
        cfg.control.rate_hz,
        cfg.hardware.legs.bus_rate_hz
    );

    while !stop.load(Ordering::Relaxed) {
        let sbus = hw.sbus.state();
        let cmd = teleop.update(&sbus, sbus.is_usable(teleop_timeout));
        let imu = hw.imu.sample_or_level();
        // 受信機直結の腕は、プロポのチャンネルから読んだ角度が唯一の手がかり。
        if let Some(observed) = cmd.arm_rad {
            hw.arm.observe(observed);
        }
        let measured = hw.measured(hw.arm.position());

        let out = controller.tick(&cmd, &measured, &imu, period.as_secs_f64());

        // モータの投入・切断は状態が変わった瞬間だけ。毎周期投げると
        // バスの帯域を食うし、`motor_run` の連打はモータ側にも優しくない。
        let want_enabled = out.leg_mode != JointMode::Idle;
        if want_enabled != motors_enabled {
            let req = if want_enabled {
                BusRequest::Enable
            } else {
                BusRequest::Disable
            };
            if let Err(e) = hw.legs.request_all(req) {
                log::error!("{req:?} を送れません: {e}");
                break;
            }
            motors_enabled = want_enabled;
        }

        write_targets(&hw, &out.targets, out.leg_mode, &cfg);
        if arm_app_driven {
            if let Err(e) = hw.arm.set_position(out.targets.arm) {
                log::warn!("腕サーボへの指令に失敗: {e}");
            }
        }

        if let Some(p) = publisher.as_mut() {
            let body = controller.body_view();
            let t = started.elapsed().as_secs_f64();
            p.maybe_publish(|seq| viz::frame(seq, t, &out.targets, &body));
        }

        if controller.state_changed() {
            log::info!("状態: {}", out.state.label());
        }

        ticks += 1;
        if opts.status_interval_s > 0.0
            && last_status.elapsed().as_secs_f64() >= opts.status_interval_s
        {
            log_status(&hw, &controller, &cmd, ticks, worst_overrun);
            last_status = Instant::now();
            worst_overrun = Duration::ZERO;
        }

        next += period;
        let now = Instant::now();
        if next > now {
            std::thread::sleep(next - now);
        } else {
            worst_overrun = worst_overrun.max(now - next);
            next = now;
        }
    }

    log::info!("停止要求を受けました。脱力します");
    let idle = [JointCommand::default(); 3];
    for bus in hw.legs.buses() {
        bus.set_commands(idle);
    }
    let _ = hw.legs.request_all(BusRequest::Disable);
    let _ = hw.arm.relax();
    // バススレッドが Disable を実際に送るまで待ってから drop する。
    std::thread::sleep(Duration::from_millis(100));
    Ok(())
}

/// ライブ可視化の配信器を開く。無効なら `None`。
///
/// `viz` フィーチャを外したビルドで `--viz` を渡されると
/// `Publisher::new` がエラーを返す（黙って無視しない）。
fn open_viz(cfg: &VizConfig) -> Result<Option<viz::Publisher>, String> {
    if !cfg.enabled {
        return Ok(None);
    }
    viz::Publisher::new(cfg).map(Some)
}

/// 目標角を 4 本のバスへ配る。
fn write_targets(hw: &Hardware, targets: &JointVec, mode: JointMode, cfg: &AppConfig) {
    let speed = cfg.hardware.legs.default_max_speed_rad_s;
    let mut cmds = [[JointCommand::default(); 3]; 4];
    for (bus, leg) in cmds.iter_mut().zip(targets.legs.iter()) {
        for (cmd, &q) in bus.iter_mut().zip(leg.iter()) {
            *cmd = JointCommand {
                mode,
                position_rad: q,
                max_speed_rad_s: speed,
                torque_nm: 0.0,
            };
        }
    }
    hw.legs.set_all(&cmds);
}

fn log_status(
    hw: &Hardware,
    controller: &Controller,
    cmd: &crate::teleop::OperatorCommand,
    ticks: u64,
    worst_overrun: Duration,
) {
    let rates: Vec<String> = hw
        .legs
        .buses()
        .iter()
        .map(|b| format!("{}:{:.0}Hz", b.leg().prefix(), b.stats().rate_hz))
        .collect();
    let errors: u64 = hw.legs.buses().iter().map(|b| b.stats().errors).sum();
    // 温度は「いちばん熱い軸」だけ出す。12 軸ぜんぶ並べても読まれない。
    let hottest = hw
        .legs
        .buses()
        .iter()
        .flat_map(|b| b.status())
        .filter(|s| s.valid)
        .map(|s| s.temperature_c)
        .fold(f64::NEG_INFINITY, f64::max);
    let faults = hw.legs.faults();
    let imu = hw.imu.stats();
    let sbus = hw.sbus.state();
    log::info!(
        "[{}] {} v=({:+.3},{:+.3},{:+.3}) 脚[{}] err={} 最高温{} IMU {:.0}Hz \
         S.BUS {}f/{}desync tick={} 遅延最大={:.1}ms",
        controller.state().label(),
        controller.gait_select().label(),
        cmd.vx_m_s,
        cmd.vy_m_s,
        cmd.wz_rad_s,
        rates.join(" "),
        errors,
        if hottest.is_finite() {
            format!("{hottest:.0}°C")
        } else {
            "-".to_string()
        },
        imu.rate_hz,
        sbus.counters.frames,
        sbus.counters.desync_bytes,
        ticks,
        worst_overrun.as_secs_f64() * 1e3,
    );
    // 異常ビットは埋もれさせない。自動で脱力はしない（立っている四足を
    // 脱力させると倒れる）ので、operator がモードスイッチで判断できるよう
    // 毎回はっきり出す。
    for (leg, k, st) in &faults {
        log::error!(
            "  異常: {} 軸{k} エラービット 0x{:02X}（{:.1} V / {:.0} °C）",
            leg.prefix(),
            st.error_raw,
            st.voltage_v,
            st.temperature_c
        );
    }
    if controller.state() == State::PlayingPose {
        if let Some(name) = controller.playing() {
            log::info!("  再生中: {name}");
        }
    }
}

/// Ctrl-C (SIGINT) / SIGTERM で立つフラグ。
static STOP_FLAG: AtomicBool = AtomicBool::new(false);

/// シグナルハンドラを仕掛ける。
///
/// ハンドラの中では `AtomicBool` を立てるだけにして、脱力処理はメインスレッド
/// で行う。ハンドラからシリアル I/O をするのは非同期シグナル安全でないし、
/// 途中で握っているロックがあれば自己デッドロックになる。
///
/// **仕掛けたら、そのコマンドのループは必ず戻り値のフラグを見ること。**
/// SIGINT の既定動作（プロセス終了）を奪うので、見ないループで呼ぶと
/// Ctrl-C がまったく効かなくなる。`diag` の `--forever` もこれを使う。
pub(crate) fn install_signal_handler() -> &'static AtomicBool {
    unsafe {
        let handler = handle_signal as extern "C" fn(libc::c_int) as libc::sighandler_t;
        libc::signal(libc::SIGINT, handler);
        libc::signal(libc::SIGTERM, handler);
    }
    &STOP_FLAG
}

extern "C" fn handle_signal(_sig: libc::c_int) {
    STOP_FLAG.store(true, Ordering::Relaxed);
}
