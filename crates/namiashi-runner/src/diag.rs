//! 実機を**動かさずに**確かめるコマンド群。
//!
//! 立ち上げの順番として、まずここが全部通ることを確認してから `run` へ行く。
//! 脚の指令は一切出さず、状態読み出しと受信だけを行う。

use std::io::Write;
use std::time::{Duration, Instant};

use namiashi_hal::ch348;
use namiashi_hal::config::HardwareConfig;
use namiashi_hal::imu::ImuReader;
use namiashi_hal::legs::LegArray;
use namiashi_hal::sbus::{SbusReceiver, SbusState, CHANNELS};

use crate::config::AppConfig;
use crate::jointvec::JointVec;
use crate::teleop::{OperatorCommand, Teleop, TeleopConfig};
use crate::viz::{self, VizConfig};

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
        1 => "LEG2 (RL) RS485",
        2 => "LEG3 (FR) RS485",
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

/// ANSI のコントロールシーケンス導入部。
const CSI: &str = "\x1b[";

fn flag(value: bool) -> &'static str {
    if value {
        "●"
    } else {
        "○"
    }
}

fn warn(value: bool) -> &'static str {
    if value {
        "⚠YES"
    } else {
        "no"
    }
}

/// 端末の表示幅。かな・漢字は 2 桁として数える。
///
/// `format!("{:8}")` は**文字数**で数えるので、役割名（かな・漢字）を混ぜると
/// 桁がずれて表がガタつく。ここは自前に数えるしかない。
fn disp_width(s: &str) -> usize {
    s.chars().map(char_cols).sum()
}

fn char_cols(c: char) -> usize {
    let u = c as u32;
    // かな・漢字・全角記号だけを 2 桁にする。棒グラフの █ · や ● ○ ⚠ は
    // 端末側で 1 桁に描かれるので巻き込まないこと。
    let wide = (0x1100..=0x115F).contains(&u)
        || (0x2E80..=0x303E).contains(&u)
        || (0x3041..=0x33FF).contains(&u)
        || (0x3400..=0x9FFF).contains(&u)
        || (0xF900..=0xFAFF).contains(&u)
        || (0xFF00..=0xFF60).contains(&u)
        || (0xFFE0..=0xFFE6).contains(&u);
    if wide {
        2
    } else {
        1
    }
}

/// 表示幅が `cols` になるまで右に空白を足す。
fn pad(s: &str, cols: usize) -> String {
    let mut out = s.to_string();
    for _ in disp_width(s)..cols {
        out.push(' ');
    }
    out
}

/// チャンネル値の棒グラフ。生値は 0..=2047。
fn bar(raw: u16, cols: usize) -> String {
    let filled = (raw as usize * cols) / 2047;
    (0..cols)
        .map(|i| if i < filled { '█' } else { '·' })
        .collect()
}

/// CH 番号（1 始まり）→ 役割名。
///
/// 「各スティック / スイッチが期待どおりのチャンネルに出る」を確かめるのが
/// このコマンドの主目的（`doc/bringup_checklist.md` §3-2）なので、番号だけ
/// でなく**設定から引いた役割**を並べる。設定を変えれば表示も追従する。
fn channel_roles(t: &TeleopConfig) -> [&'static str; CHANNELS] {
    let mut roles = [""; CHANNELS];
    let mut assign = vec![
        (t.vx.channel, "前後"),
        (t.vy.channel, "左右"),
        (t.wz.channel, "旋回"),
        (t.height.channel, "高さ"),
        (t.mode.channel, "モード"),
        (t.gait.channel, "歩容"),
        (t.pose.channel, "ポーズ"),
        (t.chicken_head.channel, "チキン"),
    ];
    if let Some(arm) = &t.arm {
        assign.push((arm.channel, "腕"));
    }
    for (ch, name) in assign {
        if (1..=CHANNELS).contains(&ch) {
            roles[ch - 1] = name;
        }
    }
    roles
}

/// 再描画表示の全行。
///
/// レイアウトは `board/nm_board/ch348/test/sbus_monitor.py` を踏襲した
/// （ヘッダ + 2 列 8 行のチャンネル表）。そこに namiashi 側の**解釈結果**を
/// 足してある。生値だけ見ても「その値でロボットが何をするつもりか」は
/// 分からず、立ち上げで確かめたいのは後者だから。
fn monitor_lines(
    port: &str,
    state: &SbusState,
    cmd: &OperatorCommand,
    roles: &[&'static str; CHANNELS],
) -> Vec<String> {
    let c = &state.counters;
    let rule = "-".repeat(76);
    let mut out = vec![
        format!(
            "namiashi sbus  {port}  100000 8E2   {:5.1} fps  frames={} slots={} desync={}",
            state.fps, c.frames, c.slots, c.desync_bytes
        ),
        // link= は敢えて固定幅にしない。埋めると無駄な空白が空くうえ、
        // 行がずれるのは「リンク状態が変わったとき」だけ = 気付いてほしい瞬間。
        format!(
            "{}   CH17:{}  CH18:{}   FRAME_LOST:{}   FAILSAFE:{}",
            link_status(state, cmd.link_ok),
            flag(state.ch17),
            flag(state.ch18),
            warn(state.frame_lost),
            warn(state.failsafe),
        ),
        telemetry(state),
        rule.clone(),
    ];

    if c.frames == 0 {
        out.push("  (S.BUS フレーム待ち... 送信機と受信機の電源を確認してください)".to_string());
    } else {
        for row in 0..CHANNELS / 2 {
            let cells: Vec<String> = [row * 2, row * 2 + 1]
                .iter()
                .map(|&k| {
                    let v = state.channels[k];
                    format!("CH{:>2} {} {v:>4} {}", k + 1, pad(roles[k], 8), bar(v, 12))
                })
                .collect();
            out.push(format!("  {}", cells.join("   ")));
        }
    }

    out.push(rule);
    out.push(format!(
        "  vx={:+.3} m/s   vy={:+.3} m/s   wz={:+.3} rad/s   高さ={:+.3} m",
        cmd.vx_m_s, cmd.vy_m_s, cmd.wz_rad_s, cmd.height_offset_m
    ));
    out.push(format!(
        "  モード={:?}   歩容={}   ポーズ={}   チキンヘッド={}   腕={}",
        cmd.mode,
        cmd.gait.label(),
        if cmd.play_pose { "再生" } else { "-" },
        if cmd.chicken_head { "on" } else { "off" },
        match cmd.arm_rad {
            Some(q) => format!("{q:+.3}rad"),
            None => "-".to_string(),
        }
    ));
    out
}

/// `--plain` の 1 行出力。grep やログに落とすとき用。
fn plain_line(state: &SbusState, cmd: &OperatorCommand) -> String {
    let raw: Vec<String> = state.channels[..8]
        .iter()
        .enumerate()
        .map(|(i, v)| format!("{}:{v:>4}", i + 1))
        .collect();
    format!(
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
        link_status(state, cmd.link_ok),
        telemetry(state),
        state.fps,
        state.counters.frames,
        state.counters.desync_bytes,
    )
}

/// 観測ループの終了条件。
///
/// `--forever` のときは経過時間を見ず、SIGINT / SIGTERM だけで抜ける。
/// 立ち上げ中は「手で動かしながら眺める」ので、秒数を決め打ちできない。
struct Deadline {
    start: Instant,
    seconds: Option<f64>,
    stop: &'static std::sync::atomic::AtomicBool,
}

impl Deadline {
    /// `seconds` が `None` なら Ctrl-C まで回る。
    fn new(seconds: Option<f64>) -> Self {
        Self {
            start: Instant::now(),
            seconds,
            // ハンドラを仕掛ける以上、下の `running()` を必ず見ること
            // （SIGINT の既定動作を奪うため）。
            stop: crate::runner::install_signal_handler(),
        }
    }

    fn running(&self) -> bool {
        if self.stop.load(std::sync::atomic::Ordering::Relaxed) {
            return false;
        }
        match self.seconds {
            Some(s) => self.start.elapsed().as_secs_f64() < s,
            None => true,
        }
    }

    /// 画面に出す「いつまで回るか」の説明。
    fn label(&self) -> String {
        match self.seconds {
            Some(s) => format!("{s:.0} 秒"),
            None => "Ctrl-C まで".to_string(),
        }
    }
}

/// `sbus` — プロポの入力を表示し続ける。モータには触れない。
///
/// 既定は再描画表示。`plain` で 1 行 / 更新の逐次出力に切り替わる
/// （リダイレクトやログ採取のとき、ANSI が混ざると読めないため）。
pub fn sbus(cfg: &AppConfig, seconds: Option<f64>, plain: bool) -> Result<(), String> {
    let rx = SbusReceiver::connect(&cfg.hardware.sbus).map_err(|e| e.to_string())?;
    let deadline = Deadline::new(seconds);
    println!("{} で受信中（{}）", rx.port(), deadline.label());
    rx.wait_ready(Duration::from_secs(3))
        .map_err(|e| format!("{e}。送信機の電源を確認してください"))?;

    let mut teleop = Teleop::new(cfg.teleop.clone(), &cfg.gait, &cfg.hardware.arm);
    let timeout = Duration::from_millis(cfg.control.teleop_timeout_ms);
    let roles = channel_roles(&cfg.teleop);
    let port = format!("{}", rx.port());

    let mut stdout = std::io::stdout();
    if !plain {
        // 再描画中にカーソルが踊るのを止める。最後に必ず戻す。
        // Ctrl-C も Deadline 経由でループを抜けるので、復帰処理は飛ばされない。
        let _ = write!(stdout, "{CSI}?25l");
    }
    let mut previous = 0usize;

    while deadline.running() {
        let state = rx.state();
        let cmd = teleop.update(&state, state.is_usable(timeout));
        if plain {
            println!("{}", plain_line(&state, &cmd));
        } else {
            let lines = monitor_lines(&port, &state, &cmd, &roles);
            if previous > 0 {
                let _ = write!(stdout, "{CSI}{previous}A");
            }
            for line in &lines {
                // 2K で行を消してから書く。前の行が長かったときの残骸を防ぐ。
                let _ = writeln!(stdout, "{CSI}2K{line}");
            }
            previous = lines.len();
            let _ = stdout.flush();
        }
        std::thread::sleep(Duration::from_millis(if plain { 200 } else { 50 }));
    }
    if !plain {
        let _ = write!(stdout, "{CSI}?25h");
        let _ = stdout.flush();
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
pub fn imu(cfg: &AppConfig, seconds: Option<f64>) -> Result<(), String> {
    let reader = ImuReader::connect(&cfg.hardware.imu).map_err(|e| e.to_string())?;
    let deadline = Deadline::new(seconds);
    println!("{} で受信中（{}）", reader.port(), deadline.label());
    reader
        .wait_ready(Duration::from_secs(3))
        .map_err(|e| e.to_string())?;
    while deadline.running() {
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
///
/// # `--viz` で出すのは measured ストリームだけ
///
/// 指令を一切出さないので、流せるのは**エンコーダから読んだ実測角**しかない。
/// `viz::VIZ_KEY_MEASURED` にだけ put する（planned は空のまま）。
///
/// - `run --viz` … 指令と実測を対で流す。ゴースト重畳になる
/// - `legs --viz` … 実測のみ。ゴーストは出ない。**指令は一切出さない**
///
/// 立ち上げでやりたいのは後者。脚を手で動かして、画面のモデルが同じように
/// 動くかを見れば、`(バス, id)` → 関節の対応と符号を目で確認できる。
pub fn legs(cfg: &AppConfig, seconds: Option<f64>, viz_cfg: &VizConfig) -> Result<(), String> {
    let array = LegArray::connect(&cfg.hardware).map_err(|e| e.to_string())?;
    let deadline = Deadline::new(seconds);
    println!(
        "脚バスを開きました（指令は送りません。観測: {}）",
        deadline.label()
    );
    for bus in array.buses() {
        println!("  {} → {}", bus.leg().prefix(), bus.port());
    }

    let mut publisher = crate::runner::open_viz(viz_cfg)?;
    if publisher.is_some() {
        println!("  ライブ可視化: **実測角のみ**を配信します（指令は出しません）");
    }
    // IMU は measured の `pose_rp`（胴体の roll/pitch）を埋めるために開く。
    // ここでも**指令は出さない**ので、読むだけで機体は動かない。
    // 開けなくても関節角の確認は続けられるので、失敗は警告に留める。
    let imu = match ImuReader::connect(&cfg.hardware.imu) {
        Ok(r) => {
            println!(
                "  IMU → {}（胴体の roll/pitch を measured に載せます）",
                r.port()
            );
            if let Err(e) = r.wait_ready(Duration::from_secs(3)) {
                println!("  ⚠ IMU がまだ値を返しません: {e}");
            }
            Some(r)
        }
        Err(e) => {
            println!("  ⚠ IMU を開けません: {e}");
            println!("    関節角の確認は続けられますが、pose_rp は [0,0] のままです");
            None
        }
    };
    let mut measured_seen = false;
    let started = Instant::now();
    // 表示は 500 ms ごとで十分だが、可視化はもっと細かく出したい。
    // ループは可視化のレートで回し、テキストは間引く。
    let text_period = Duration::from_millis(500);
    let mut next_text = Instant::now();

    while deadline.running() {
        std::thread::sleep(Duration::from_millis(20));

        if let Some(p) = publisher.as_mut() {
            // 実測角をそのままモデル座標系の JointVec に詰める。腕は
            // GaitVizFrame が運ばないので 0 のまま（articara 側で動かない）。
            let states = array.states();
            let mut q = JointVec::zeros();
            for (leg, leg_states) in states.iter().enumerate() {
                for (k, s) in leg_states.iter().enumerate() {
                    q.legs[leg][k] = s.position_rad;
                }
            }
            // 最初の読み戻しが済むまで送らない（ゼロ姿勢は「崩れ落ちた」絵になる）。
            if !measured_seen {
                measured_seen = states.iter().flatten().all(|s| s.ok);
            }
            if measured_seen {
                // 胴体の姿勢 3 軸は IMU の実測。位置 x, y は測る術が無いので 0、
                // 高さも設定の起立高さを入れるだけで、モデルが床にめり込まない
                // ようにするためのもの（**測定値ではない**）。
                let att = imu
                    .as_ref()
                    .map_or([0.0; 3], |r| r.sample_or_level().rpy_rad);
                let body = viz::BodyView {
                    z: cfg.gait.stance_height_m,
                    rp: [att[0], att[1]],
                    yaw: att[2],
                    ..Default::default()
                };
                let t = started.elapsed().as_secs_f64();
                p.maybe_publish(|seq| viz::Frames::measured(viz::frame(seq, t, &q, &body)));
            }
        }

        if Instant::now() < next_text {
            continue;
        }
        next_text += text_period;
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
        if let Some(r) = imu.as_ref() {
            let s = r.sample_or_level();
            let st = r.stats();
            println!(
                "IMU rpy=({:+7.2},{:+7.2},{:+7.2})°  gyro=({:+7.2},{:+7.2},{:+7.2})°/s  \
                 {:.0}Hz err={}",
                s.rpy_rad[0].to_degrees(),
                s.rpy_rad[1].to_degrees(),
                s.rpy_rad[2].to_degrees(),
                s.gyro_rad_s[0].to_degrees(),
                s.gyro_rad_s[1].to_degrees(),
                s.gyro_rad_s[2].to_degrees(),
                st.rate_hz,
                st.errors,
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
        "  CH{} モード(脱力/初期姿勢/歩行)  CH{} 歩容(Crawl/Walk/Trot)  \
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
