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
use namiashi_hal::joint::{JointCommand, JointMode, LegSlot};
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

/// 「まだ起動条件が整っていない」失敗に付ける前置き。
///
/// systemd に**再試行してよい失敗**を伝えるためのもの。`main` がこれを見て
/// 終了コード 75 を返し、ユニット側は 75 のときだけ再起動する。
///
/// # なぜ区別するのか
///
/// 起動条件（S.BUS の受信、CH5 が脱力位置）が整わないのは**正常な待ち**で、
/// プロポの電源を入れれば解消する。一方、制御ループ中のクラッシュを自動で
/// 再起動すると**脚が再び動き出す**。同じ異常終了でも、片方は再試行が正しく、
/// もう片方は人が見に行くべき。
///
/// 一律 `Restart=on-failure` にすると後者まで拾い、一律 `Restart=no` にすると
/// 本番で「プロポを後から入れたのに立ち上がらない」になる。
pub const RETRYABLE: &str = "[retryable] ";


/// 追従誤差と可動域逸脱の監視。**検出して言うだけで、何もしない。**
///
/// # 何のために要るのか
///
/// 2026-08-22 の過負荷では、脱調したモータが大電流を引いて電源が電流制限に
/// 落ち、12 軸中 9 軸が低電圧保護に入った。**低電圧保護は時間で自然に解除
/// されるが、待って直るのはフラグだけで、ロボットは既に倒れている。**
/// 落とさないことが目的で、その前兆が**指令と実測が開いていくこと**。
///
/// # なぜ「言うだけ」なのか
///
/// **指令を実測から離れすぎないよう頭打ちにするのが本命の対策**だが、
/// 歩行中の正常な追従誤差を知らないまま閾値を決めると普通の動作まで
/// クランプされる。まずここで実測する。
///
/// 逸脱側も同じで、逸脱したまま脱力させるのは危ない（hip はメカ端まで
/// 余裕がある一方でケーブルが先に限界を迎える。脱力すると外力に任せる
/// ことになり自力で戻れない）。かといって自動で戻す動作を挟むと暴走時に
/// 危険が増す。**決めきれていないので、まず測る。**
struct Watch {
    /// この集計区間での最悪の追従誤差 (rad) と、その軸。
    worst: f64,
    worst_at: Option<(LegSlot, usize)>,
    /// 逸脱している軸を一度だけ言うためのフラグ。毎秒 12 行は読まれない。
    warned_excursion: bool,
    /// 12 軸の可動域。毎周期 config を引き直さないため。
    limits: [[(f64, f64); 3]; 4],
}

impl Watch {
    fn new(cfg: &AppConfig) -> Self {
        let mut limits = [[(f64::NEG_INFINITY, f64::INFINITY); 3]; 4];
        for (slot, dst) in LegSlot::ALL.iter().zip(limits.iter_mut()) {
            let Some(bus) = cfg.hardware.bus_for(*slot) else {
                continue;
            };
            for (m, d) in bus.motors.iter().zip(dst.iter_mut()) {
                *d = (m.min_rad, m.max_rad);
            }
        }
        Self {
            worst: 0.0,
            worst_at: None,
            warned_excursion: false,
            limits,
        }
    }

    /// 1 周期ぶん。**目標を送っている間だけ意味がある**ので脱力中は呼ばない。
    fn tick(&mut self, targets: &JointVec, measured: &JointVec) {
        let mut out = Vec::new();
        for (leg_i, slot) in LegSlot::ALL.iter().enumerate() {
            for k in 0..3 {
                let m = measured.legs[leg_i][k];
                let e = (targets.legs[leg_i][k] - m).abs();
                if e > self.worst {
                    self.worst = e;
                    self.worst_at = Some((*slot, k));
                }
                let (lo, hi) = self.limits[leg_i][k];
                if m < lo || m > hi {
                    out.push(format!(
                        "{} {} {:+.1}°（範囲 [{:+.0}, {:+.0}]）",
                        slot.prefix(),
                        namiashi_hal::joint::LEG_JOINT_KINDS[k],
                        m.to_degrees(),
                        lo.to_degrees(),
                        hi.to_degrees()
                    ));
                }
            }
        }
        if out.is_empty() {
            self.warned_excursion = false;
        } else if !self.warned_excursion {
            self.warned_excursion = true;
            // **脱力させない。** 逸脱したまま力を抜くと外力に任せることに
            // なり、自力で戻れない。operator が判断できるよう言うだけ。
            log::error!(
                "  可動域を出ています（指令は出し続けます）: {}",
                out.join(" / ")
            );
        }
    }

    /// 集計区間の表示用文字列。読んだら最悪値は消す。
    fn take(&mut self) -> String {
        let out = match self.worst_at {
            Some((leg, k)) => format!(
                "{:.3}rad({} {})",
                self.worst,
                leg.prefix(),
                namiashi_hal::joint::LEG_JOINT_KINDS[k]
            ),
            None => "-".to_string(),
        };
        self.worst = 0.0;
        self.worst_at = None;
        out
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
                "{RETRYABLE}{e}。送信機の電源とポートを確認してください（ベンチで脚を浮かせて \
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

    // 位置の基準は**モータの電源 ON マルチターンフレーム**で、バススレッドが
    // 起動時に自動で確立する。ここで姿勢を作る必要はない。
    //
    // かつては起動のたびに rezero していたため、**そのときの姿勢が原点**に
    // なっていた。異常終了から再起動すると崩れた姿勢が原点になる、という
    // 危うさもあった。いまは電源を入れ直さない限り原点は動かない。
    if !opts.skip_zero {
        hw.legs
            .wait_anchored(Duration::from_secs(3))
            .map_err(|e| format!("{e}（モータの電源とボーレートを確認してください）"))?;
        log::info!("マルチターンフレームを確立しました");
    }

    // **12 軸が一度でも読めるまで制御ループに入らない。**
    //
    // `wait_anchored` はフレーム確立で返るが、その時点では共有状態の
    // `JointState` がまだ既定値（`position_rad = 0.0`, `ok = false`）のことが
    // ある。書かれるのは次のトランザクション周回。
    //
    // 脱力からの遷移は**実測値を始点**に張る（`Controller::tick_relaxed`）。
    // 0 を掴むと、実際には −2.7 rad にある calf の目標がいきなり 0 になり、
    // **最初の起立で暴れる**。`--skip-zero` でも省略しない — 読めない 12 軸を
    // 相手に制御ループを回すこと自体が危ない。
    hw.legs
        .wait_first_read(Duration::from_secs(2))
        .map_err(|e| format!("{e}（12 軸すべてが応答している必要があります）"))?;
    log::info!("12 軸の初回読み出しを確認しました");

    // **CH5 が「脱力」でなければ起動しない。**
    //
    // モードスイッチは毎周期そのまま指令になるので、起立や歩行の位置で
    // 起動すると**何の操作もなしにその場で立ち上がる**。立ち上げ手順は
    // 「CH5 が脱力位置で起動する」ことを最初の確認項目にしているが、
    // 人手のチェックリストだけに任せる話ではない。
    //
    // `--allow-no-sbus` のときは見ない。受信が無い＝フェイルセーフ＝起立が
    // その指定の意味そのもので、そこで止めても意味がない。
    if !opts.allow_no_sbus {
        let sbus = hw.sbus.state();
        if cfg.teleop.mode.position(&sbus) != 0 {
            return Err(format!(
                "{RETRYABLE}CH5（モード）が脱力位置にありません（いま {} 段目 / raw {}）。                 **脱力に戻してから起動してください。** このまま起動すると                 操作なしで立ち上がります",
                cfg.teleop.mode.position(&sbus),
                cfg.teleop
                    .mode
                    .channel
                    .checked_sub(1)
                    .and_then(|i| sbus.channels.get(i))
                    .copied()
                    .unwrap_or(0),
            ));
        }
        log::info!("CH5 は脱力位置です");
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
    let mut measured_seen = false;
    let mut fault_hint_shown = false;
    let mut watch = Watch::new(&cfg);
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
        let usable = sbus.is_usable(teleop_timeout);
        // 受信が無いときの扱いは 2 通りあり、混ぜてはいけない。
        //
        // - **フェイルセーフ**（受信していたのに切れた）… 活動度を上げない。
        //   脱力中に切れたら脱力のまま。`Teleop::update` が面倒を見る
        // - **ベンチ**（`--allow-no-sbus`、受信機がそもそも無い）… 起立させたい
        //
        // かつては前者が一律 `Stand` を返しており、後者はそれに乗っかって
        // いた。結果として**脱力中に受信が切れると立ち上がっていた**。
        let cmd = if !usable && opts.allow_no_sbus {
            teleop.bench_stand()
        } else {
            teleop.update(&sbus, usable)
        };
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

        // 脱力中は目標を送っていないので、追従誤差を見ても意味がない。
        if out.leg_mode != JointMode::Idle {
            watch.tick(&out.targets, &measured);
        }
        write_targets(&hw, &out.targets, out.leg_mode, &cfg);
        if arm_app_driven {
            if let Err(e) = hw.arm.set_position(out.targets.arm) {
                log::warn!("腕サーボへの指令に失敗: {e}");
            }
        }

        if let Some(p) = publisher.as_mut() {
            // 最初の読み戻しが済むまで measured を送らない。ゼロ姿勢のフレームは
            // 受け側で「崩れ落ちたロボット」として描かれる。
            // 一度立ったら見に行かない（`all_ok` は 12 軸ぶんロックを取る）。
            if !measured_seen {
                measured_seen = hw.legs.all_ok();
            }
            let body = controller.body_view();
            let t = started.elapsed().as_secs_f64();
            let att = imu.rpy_rad;
            p.maybe_publish(|seq| {
                let planned = viz::frame(seq, t, &out.targets, &body);
                if !measured_seen {
                    return viz::Frames::planned(planned);
                }
                // 胴体の姿勢 3 軸は IMU の実測。位置 x, y と高さはオドメトリが
                // 無いので歩容の値のまま。**実測なのは 12 関節と姿勢だけ**で、
                // 位置を入れたらここを差し替える。
                let measured_body = viz::BodyView {
                    rp: [att[0], att[1]],
                    yaw: att[2],
                    ..body
                };
                viz::Frames::both(planned, viz::frame(seq, t, &measured, &measured_body))
            });
        }

        if controller.state_changed() {
            log::info!("状態: {}", out.state.label());
        }

        ticks += 1;
        if opts.status_interval_s > 0.0
            && last_status.elapsed().as_secs_f64() >= opts.status_interval_s
        {
            log_status(
                &hw,
                &controller,
                &cmd,
                ticks,
                worst_overrun,
                &mut fault_hint_shown,
                &mut watch,
            );
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
pub(crate) fn open_viz(cfg: &VizConfig) -> Result<Option<viz::Publisher>, String> {
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
    fault_hint_shown: &mut bool,
    watch: &mut Watch,
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
         S.BUS {}f/{}desync tick={} 遅延最大={:.1}ms 追従最大={}",
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
        watch.take(),
    );
    // 異常ビットは埋もれさせない。自動で脱力はしない（立っている四足を
    // 脱力させると倒れる）ので、operator がモードスイッチで判断できるよう
    // 毎回はっきり出す。
    for (leg, k, st) in &faults {
        log::error!(
            "  異常: {} {} **{}**（0x{:02X}）{:.1} V / {:.0} °C",
            leg.prefix(),
            namiashi_hal::joint::LEG_JOINT_KINDS[*k],
            st.describe(),
            st.error_raw,
            st.voltage_v,
            st.temperature_c
        );
    }
    // 消し方は毎回書かない。**同じ異常が続いている間は 1 度だけ**。
    if !faults.is_empty() && !*fault_hint_shown {
        *fault_hint_shown = true;
        log::error!(
            "  原因を取り除いてから `namiashi calib clear-error` で消せます\
             （原因が残っている間は消えません — マニュアル §2）"
        );
    }
    if faults.is_empty() {
        *fault_hint_shown = false;
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
