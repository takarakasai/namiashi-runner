//! `calib` — 実機に合わせて設定を確定させる。
//!
//! 起動直後の設定は `sign = +1`、`zero_pose_rad = 0`、可動域は URDF 値、という
//! **推測**でしかない。1 軸でも符号が逆なら起立の瞬間に自壊するし、ゼロ点が
//! 合っていなければモデル角と実機がまるで対応しない。ここはその 3 つを、
//! 実機を 1 軸ずつ小さく動かして確定させ、設定ファイルへ書き戻す道具。
//!
//! ```text
//! calib scan   [--leg FL] [--max-id 32]   応答するモータ id を数える（指令なし）
//! calib move   --leg FL --joint thigh     1 軸だけ小さく動かして符号を決める
//! calib range  --leg FL --joint thigh     脱力させ、手で動かして可動域を測る
//! calib zero   [--pose constrain]         全軸ゼロ出し + zero_pose_rad を記録
//! ```
//!
//! # 安全のための約束
//!
//! - **1 度に 1 軸しか投入しない。** `move` は対象軸だけ `EnableJoint` し、
//!   終わったら必ず `DisableJoint` で戻す。残り 2 軸は最後まで脱力のまま。
//! - **既定の振り幅は 5°、速度は 0.3 rad/s。** 取り違えていても壊れない大きさ。
//! - **書き戻しは `--write` を明示したときだけ。** 測るだけなら実機の設定は
//!   変わらない。

use std::io::{BufRead, Write};
use std::time::{Duration, Instant};

use namiashi_hal::config::HardwareConfig;
use namiashi_hal::joint::{JointCommand, JointMode, LegSlot, LEG_JOINT_KINDS};
use namiashi_hal::legs::{BusRequest, LegArray, LegBus};

use crate::config::AppConfig;
use crate::Cli;

/// 軸を止めてから状態が落ち着くまでの待ち。
const SETTLE: Duration = Duration::from_millis(300);

pub fn run(cfg: &AppConfig, cli: &Cli) -> Result<(), String> {
    match cli.positionals.get(1).map(|s| s.as_str()) {
        Some("scan") => scan(cfg, cli),
        Some("move") => jog(cfg, cli),
        Some("range") => range(cfg, cli),
        Some("zero") => zero(cfg, cli),
        Some("clear-multiturn") => clear_multiturn(cfg, cli),
        Some("single-turn") => single_turn(cfg, cli),
        Some("clear-error") => clear_error(cfg, cli),
        Some("restart") => restart(cfg, cli),
        Some(other) => Err(format!(
            "未知の calib サブコマンド {other:?}\
             （scan|move|range|zero|clear-multiturn|single-turn|clear-error|restart）"
        )),
        None => Err(
            "calib のサブコマンドを指定してください（scan|move|range|zero|clear-multiturn）".into(),
        ),
    }
}

// ── scan ────────────────────────────────────────────────────────────────

/// `calib scan` — 各脚バスで id を舐めて、応答するものを並べる。
///
/// **指令は出さない**（State2 の読み出しだけ）。設定に書いた id が本当に
/// その脚に居るのか、id がぶつかっていないかを、脚を動かさずに確かめる。
fn scan(cfg: &AppConfig, cli: &Cli) -> Result<(), String> {
    let max_id = cli.usize("max-id").unwrap_or(8).clamp(1, 32) as u8;
    let only = leg_filter(cli)?;

    // スキャンは設定の id 表とは無関係に舐めたいので、バスごとに専用の
    // 設定（id 1..max_id）を組んで開く。
    for leg in LegSlot::ALL {
        if only.is_some_and(|l| l != leg) {
            continue;
        }
        let bus_cfg = cfg
            .hardware
            .bus_for(leg)
            .ok_or_else(|| format!("脚 {} の設定がありません", leg.prefix()))?;
        print!("{} {} : ", leg.prefix(), bus_cfg.port.label());
        let _ = std::io::stdout().flush();

        let found = scan_bus(cfg, leg, max_id)?;
        if found.is_empty() {
            println!(
                "応答なし（モータ電源とボーレート {} を確認）",
                cfg.hardware.legs.baud
            );
        } else {
            let expected: Vec<u8> = bus_cfg.motors.iter().map(|m| m.id).collect();
            let ids: Vec<String> = found.iter().map(|id| id.to_string()).collect();
            print!("id {} が応答", ids.join(", "));
            if found != expected {
                print!("（設定は {expected:?}。食い違っています）");
            }
            println!();
        }
    }
    Ok(())
}

/// 1 本のバスで id 1..=max_id を舐める。
fn scan_bus(cfg: &AppConfig, leg: LegSlot, max_id: u8) -> Result<Vec<u8>, String> {
    // 設定の 3 軸を id 1..3 とは限らない値へ差し替えながら開き直すのは重い。
    // ここでは 3 軸ぶんずつ束ねて、必要な回数だけ開く。
    let mut found = Vec::new();
    let mut id = 1u8;
    while id <= max_id {
        let ids: Vec<u8> = (id..=max_id.min(id + 2)).collect();
        let probe = probe_config(cfg, leg, &ids)?;
        let bus = LegBus::open_alone(&probe, leg).map_err(|e| e.to_string())?;
        // 状態読み（`JointMode::Idle`）だけで数周回させ、応答したものを拾う。
        std::thread::sleep(SETTLE);
        for (k, probed) in ids.iter().enumerate() {
            if bus.state()[k].ok {
                found.push(*probed);
            }
        }
        id += 3;
    }
    Ok(found)
}

/// スキャン用に、対象脚の 3 軸 id だけ差し替えた設定を作る。
///
/// 可動域は触らない（指令を出さないので使われない）。3 軸に満たないときは
/// 最後の id を繰り返すのではなく**存在しない id で埋める**: 重複 id は
/// `validate` が弾くし、応答を取り違える元でもある。
fn probe_config(cfg: &AppConfig, leg: LegSlot, ids: &[u8]) -> Result<HardwareConfig, String> {
    let mut hw = cfg.hardware.clone();
    let bus = hw
        .legs
        .bus
        .iter_mut()
        .find(|b| b.leg_slot().ok() == Some(leg))
        .ok_or_else(|| format!("脚 {} の設定がありません", leg.prefix()))?;
    // 使われていない高い id で埋める（1..=32 の範囲内、かつ ids と重複しない）。
    let mut filler = 32u8;
    for (k, motor) in bus.motors.iter_mut().enumerate() {
        motor.id = match ids.get(k) {
            Some(id) => *id,
            None => {
                while ids.contains(&filler) {
                    filler -= 1;
                }
                let f = filler;
                filler -= 1;
                f
            }
        };
    }
    hw.validate().map_err(|e| e.to_string())?;
    Ok(hw)
}

// ── move（符号の確定） ───────────────────────────────────────────────────

/// `calib move` — 1 軸だけを小さく動かし、モデルの + 方向かを人に確認する。
///
/// モータ座標で +δ 動かすので、**設定の `sign` には依存しない**。「モデルの
/// + 方向へ動いたか」への答えがそのまま `sign` になる。
fn jog(cfg: &AppConfig, cli: &Cli) -> Result<(), String> {
    let (leg, k) = target_joint(cli)?;
    let deg = cli.f64("deg").unwrap_or(5.0);
    let speed = cli.f64("speed").unwrap_or(0.3);
    if !(0.5..=30.0).contains(&deg.abs()) {
        return Err("--deg は 0.5..30 の範囲で指定してください（校正は小さく動かす）".into());
    }

    let name = joint_label(cfg, leg, k);
    println!(
        "{name} を モータ座標で {deg:+.1}° 動かします（速度 {speed} rad/s）。\
         他の 2 軸は脱力のままです"
    );
    println!("脚が自由に動ける状態か確認してください。続けるなら Enter、やめるなら Ctrl-C");
    let _ = read_line();

    let bus = LegBus::open_alone(&cfg.hardware, leg).map_err(|e| e.to_string())?;
    let before = measure_one(&bus, k)?;

    // マルチターンフレームは起動時に自動で確立されるので、ここで置き直す
    // 必要は無い。念のため確立を待つだけにする。
    let deadline = Instant::now() + Duration::from_secs(3);
    while !bus.is_anchored() {
        if Instant::now() >= deadline {
            return Err("マルチターンフレームを確立できません\
                        （モータ電源とボーレートを確認してください）"
                .into());
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    std::thread::sleep(SETTLE);
    bus.request(BusRequest::EnableJoint(k))
        .map_err(|e| e.to_string())?;

    // ここから先は必ず戻す。`?` で抜けても止まるよう、結果を受けてから返す。
    let result = jog_once(cfg, &bus, k, before, deg.to_radians(), speed);
    bus.set_commands([JointCommand::default(); 3]);
    let _ = bus.request(BusRequest::DisableJoint(k));
    std::thread::sleep(SETTLE);
    let moved = result?;

    println!(
        "実測: {before:+.4} → {:+.4} rad（Δ {:+.4}）",
        moved.1, moved.0
    );
    // 指令と実測が食い違ったら符号判定に使わない。クランプは上で弾いているが、
    // 機械的な干渉や脱調でもここに落ちる。
    let expected = deg.to_radians().abs();
    if (moved.0.abs() - expected).abs() > expected * 0.2 && moved.0.abs() >= expected * 0.2 {
        println!(
            "⚠ 指令 {:+.2}° に対し実測 {:+.2}°。**符号の判定には使えません。**\n\
             　可動域の端・機械的な干渉・脱調を確認してください",
            deg,
            moved.0.to_degrees()
        );
        return Ok(());
    }
    if moved.0.abs() < deg.to_radians() * 0.2 {
        println!(
            "⚠ ほとんど動いていません。モータ電源・可動域の端・機械的な干渉を確認してください"
        );
        return Ok(());
    }

    let positive = match cli.str("assume") {
        Some(a) => a.starts_with('y'),
        None => {
            println!(
                "この軸はモデルの **+ 方向**（URDF の関節軸まわり右ねじ）へ動きましたか？ [y/n]"
            );
            read_line().trim().starts_with('y')
        }
    };
    let sign = if positive { 1.0 } else { -1.0 };
    println!("{name}: sign = {sign:+.0}");

    if let Some(path) = cli.str("write") {
        let mut cfg = cfg.clone();
        let bi = bus_index(&cfg.hardware, leg)?;
        cfg.hardware.legs.bus[bi].motors[k].sign = sign;
        write_config(&cfg, path)?;
        println!("{path} に書き戻しました");
    } else {
        println!("（--write PATH を付けると設定に書き戻します）");
    }
    Ok(())
}

/// 現在位置から `delta_rad`（モータ座標）動かし、`(Δ, 到達値)` をモデル座標で返す。
fn jog_once(
    cfg: &AppConfig,
    bus: &LegBus,
    k: usize,
    before: f64,
    delta_rad: f64,
    speed: f64,
) -> Result<(f64, f64), String> {
    // **現在位置からの相対移動**として目標を組む。
    //
    // モータ座標で +delta 動かしたいので、モデル座標では sign を掛けて足す。
    // sign を決めるのが目的なので設定値をそのまま使う（そのつもりで動かし、
    // 実際にどちらへ動いたかを人が見る）。
    //
    // **かつては `sign * delta + zero_pose_rad` と書いていた。** `rezero` で
    // 「今いるところ」がモータ座標の 0 になる前提だったが、位置の基準を
    // モータの電源 ON マルチターンフレームへ移した時点でその前提は消えた。
    // 絶対座標の一点を指すので、原点姿勢から離れているほど大きく動く
    // （実測で 73° 動く条件があった）。可動域クランプは ±145° なので止まらない。
    let map = &cfg.hardware.legs.bus[bus_index(&cfg.hardware, bus.leg())?].motors[k];
    let target = before + map.sign * delta_rad;

    // **クランプに当たる状態では測らない。**
    //
    // 目標は可動域へクランプされるので、端の外や端の近くにいると「5° 動かす」
    // つもりが端まで一気に動く。実測 (2026-08-22): RL hip が可動域の外
    // (-75.9°, min は -60°) にいたため、5° の指令が **15.9° の移動**になった。
    // しかも「+ 方向へ動いたか」への答えはクランプの結果であって符号の証拠に
    // ならないのに、そのまま sign として採用されてしまった。
    let clamped = target.clamp(map.min_rad, map.max_rad);
    if (clamped - target).abs() > 1e-9 {
        return Err(format!(
            "現在位置 {:+.4} rad ({:+.1}°) から {:+.1}° 動かすと可動域              [{:+.1}, {:+.1}]° を出るため、指令が端へクランプされます。
             このまま実行すると **{:+.1}° 動いて**しまい、符号の判定にも使えません。
             可動域の内側へ手で戻してからやり直してください。",
            before,
            before.to_degrees(),
            (map.sign * delta_rad).to_degrees(),
            map.min_rad.to_degrees(),
            map.max_rad.to_degrees(),
            (clamped - before).to_degrees(),
        ));
    }

    let mut cmds = [JointCommand::default(); 3];
    cmds[k] = JointCommand {
        mode: JointMode::Position,
        position_rad: target,
        max_speed_rad_s: speed,
        torque_nm: 0.0,
    };
    bus.set_commands(cmds);

    // スルーレート制限があるので、到達には目標差 / 制限レート ぶんかかる。
    let travel_s = delta_rad.abs() / cfg.hardware.legs.max_target_rate_rad_s.max(0.1);
    std::thread::sleep(Duration::from_secs_f64(travel_s + 1.0));

    let after = measure_one(bus, k)?;
    Ok((after - before, after))
}

// ── range（可動域の実測） ───────────────────────────────────────────────

/// `calib range` — 脱力させ、手で端まで動かしてもらって可動域を記録する。
///
/// 自動で端を探しに行かないのは、探る側が壊す側になるから。人が手で押した
/// 範囲を記録するほうが安全で、しかも「実際に組んだ機体で動く範囲」という
/// 正しい答えが出る。
fn range(cfg: &AppConfig, cli: &Cli) -> Result<(), String> {
    let (leg, k) = target_joint(cli)?;
    let secs = cli.f64("secs").unwrap_or(20.0);
    let margin = cli.f64("margin").unwrap_or(0.05);
    let name = joint_label(cfg, leg, k);

    let array = LegArray::connect(&cfg.hardware).map_err(|e| e.to_string())?;
    let bus = array.bus(leg);
    bus.request(BusRequest::Disable)
        .map_err(|e| e.to_string())?;
    std::thread::sleep(SETTLE);

    // 秒数を決め打ちせず Ctrl-C で締める指定。手で端まで動かす作業は時間が
    // 読めないので、20 秒に追われるより「納得したら止める」方が合う。
    // 打ち切っても下の集計と --write はそのまま走る。
    // `--secs 0`（以下）と `--forever` のどちらでも同じ（`main::secs_or_forever`）。
    let forever = crate::secs_or_forever(cli, secs).is_none();
    let stop = crate::runner::install_signal_handler();
    if forever {
        println!("{name} を手でゆっくり端から端まで動かしてください（Ctrl-C で確定）");
    } else {
        println!("{name} を手でゆっくり端から端まで動かしてください（{secs:.0} 秒間 記録します）");
    }
    let start = Instant::now();
    let mut min = f64::INFINITY;
    let mut max = f64::NEG_INFINITY;
    while !stop.load(std::sync::atomic::Ordering::Relaxed)
        && (forever || start.elapsed().as_secs_f64() < secs)
    {
        std::thread::sleep(Duration::from_millis(50));
        let s = bus.state()[k];
        if !s.ok {
            continue;
        }
        min = min.min(s.position_rad);
        max = max.max(s.position_rad);
        if forever {
            print!(
                "\r  min {min:+.4}  max {max:+.4}  （幅 {:+.4} rad）",
                max - min
            );
        } else {
            print!(
                "\r  min {min:+.4}  max {max:+.4}  （残り {:>4.1} s）",
                secs - start.elapsed().as_secs_f64()
            );
        }
        let _ = std::io::stdout().flush();
    }
    println!();

    if !min.is_finite() || !max.is_finite() || (max - min) < 0.05 {
        return Err("動きが記録できませんでした（モータ電源とバスを確認してください）".into());
    }
    // 実測の端そのままだと、指令が機械端に当たる。内側へ `margin` 入れる。
    let (lo, hi) = (min + margin, max - margin);
    println!("{name}: 実測 {min:+.4}..{max:+.4} → 余裕 {margin} を引いて {lo:+.4}..{hi:+.4} rad");

    if let Some(path) = cli.str("write") {
        let mut cfg = cfg.clone();
        let bi = bus_index(&cfg.hardware, leg)?;
        cfg.hardware.legs.bus[bi].motors[k].min_rad = lo;
        cfg.hardware.legs.bus[bi].motors[k].max_rad = hi;
        write_config(&cfg, path)?;
        println!("{path} に書き戻しました");
    } else {
        println!("（--write PATH を付けると設定に書き戻します）");
    }
    Ok(())
}

// ── zero（ゼロ点の確定） ────────────────────────────────────────────────

/// `calib zero` — 全 12 軸をゼロ出しし、その姿勢のモデル関節角を記録する。
///
/// LKMTech V3 の位置制御は `rezero` で置いたソフトゼロからの相対量なので、
/// 「**どの姿勢でゼロ出ししたか**」が分からないとモデル角と対応が付かない。
/// `--pose <名前>` でその姿勢を `.misa` のポーズ名として指定する。
fn zero(cfg: &AppConfig, cli: &Cli) -> Result<(), String> {
    let pose_name = cli
        .str("pose")
        .unwrap_or(&cfg.control.start_pose)
        .to_string();
    let robot = crate::robot::load_from_config(cfg)?;
    let pose = robot.poses.pose(&pose_name).ok_or_else(|| {
        format!(
            "姿勢 {pose_name:?} がモデルにありません（{:?}）",
            robot.poses.pose_names().collect::<Vec<_>>()
        )
    })?;
    let angles = robot
        .poses
        .resolve(&pose.angles, crate::jointvec::JointVec::zeros());

    println!("ロボットを姿勢 {pose_name:?} に保持してください:");
    for (name, q) in angles.iter_named() {
        println!("  {name:<18} {q:+.4} rad ({:+.1}°)", q.to_degrees());
    }
    println!("保持できたら Enter（Ctrl-C で中止）");
    let _ = read_line();

    let array = LegArray::connect(&cfg.hardware).map_err(|e| e.to_string())?;
    array
        .wait_anchored(Duration::from_secs(3))
        .map_err(|e| format!("{e}（モータ電源とボーレートを確認してください）"))?;
    // フレーム確立直後の 1 回目は turn 追従が始まったばかりなので、
    // 数周ぶん回してから読む。
    std::thread::sleep(Duration::from_millis(300));

    // **モータには何も書かない。** いまの絶対角と、保持している姿勢の
    // モデル角との差を求めるだけ。
    //
    //   q_model = sign * q_abs + zero_pose_rad
    //   → zero_pose_rad = q_model_保持姿勢 - sign * q_abs
    //
    // 現在の状態は古い zero_pose_rad で変換済みなので、そこから絶対角を
    // 逆算する（sign * q_abs = q_model_現在 - zero_pose_rad_旧）。
    let mut out = cfg.clone();
    for leg in LegSlot::ALL {
        let bi = bus_index(&out.hardware, leg)?;
        let state = array.bus(leg).state();
        for k in 0..3 {
            if !state[k].ok {
                return Err(format!(
                    "{} 軸{k} の状態が読めません（モータ電源とバスを確認してください）",
                    leg.prefix()
                ));
            }
            let old = cfg.hardware.legs.bus[bi].motors[k].zero_pose_rad;
            let sign_q_abs = state[k].position_rad - old;
            let held = angles.legs[leg.index()][k];
            out.hardware.legs.bus[bi].motors[k].zero_pose_rad = held - sign_q_abs;
        }
    }
    println!("オフセットを求めました（モータには何も書いていません）:");
    for leg in LegSlot::ALL {
        let bi = bus_index(&out.hardware, leg)?;
        let vals: Vec<String> = (0..3)
            .map(|k| {
                let v = out.hardware.legs.bus[bi].motors[k].zero_pose_rad;
                format!("{v:+.4}")
            })
            .collect();
        println!(
            "  {} zero_pose_rad = [{}] rad",
            leg.prefix(),
            vals.join(" ")
        );
    }
    match cli.str("write") {
        Some(path) => {
            write_config(&out, path)?;
            println!("{path} に zero_pose_rad を書き戻しました");
        }
        None => println!("（--write PATH を付けると zero_pose_rad を書き戻します）"),
    }
    Ok(())
}

// ── clear-multiturn ─────────────────────────────────────────────────────

/// `calib clear-multiturn` — マルチターンカウンタを 0 に戻す（`0x95`）。
///
/// **モータ電源の OFF/ON と同じ効果**をマルチターンフレームにだけ与える。
/// A7Z とモータ電源が同一系統で切り分けられない開発環境で、電源を落とさずに
/// 「電源を入れ直した状態」を作るためのもの。
///
/// ROM には書かない。`0x19`（`WriteCurrentPosAsZero`）とは**別物**で、
/// あちらはフラッシュに書くので書き込み回数の上限がある。
fn clear_multiturn(cfg: &AppConfig, cli: &Cli) -> Result<(), String> {
    let only = leg_filter(cli)?;
    // `--joint` があれば 1 軸だけ。検証はまずこれで行うこと。
    let only_joint = match cli.str("joint") {
        None => None,
        Some(name) => Some(
            LEG_JOINT_KINDS
                .iter()
                .position(|k| *k == name)
                .ok_or_else(|| format!("--joint {name:?} が不正です（hip/thigh/calf）"))?,
        ),
    };
    if only_joint.is_some() && only.is_none() {
        return Err("--joint を使うときは --leg も指定してください".into());
    }
    match (only, only_joint) {
        (Some(l), Some(k)) => println!(
            "**{} の {} だけ**マルチターンカウンタを 0 に戻します（モータ電源 OFF/ON 相当）。",
            l.prefix(),
            LEG_JOINT_KINDS[k]
        ),
        (Some(l), None) => println!(
            "**{} の 3 軸**のマルチターンカウンタを 0 に戻します（モータ電源 OFF/ON 相当）。",
            l.prefix()
        ),
        _ => println!(
            "**12 軸すべて**のマルチターンカウンタを 0 に戻します（モータ電源 OFF/ON 相当）。"
        ),
    }
    println!("**ROM には書きません。** 次に本当に電源を切れば元どおりです。");
    println!();
    println!("実行後、この姿勢が新しい原点になります。**zero_pose_rad は");
    println!("この姿勢を基準に測り直してください**（calib zero）。");
    println!("続けるなら Enter、やめるなら Ctrl-C");
    let _ = read_line();

    let array = LegArray::connect(&cfg.hardware).map_err(|e| e.to_string())?;
    array
        .wait_anchored(Duration::from_secs(3))
        .map_err(|e| format!("{e}（モータ電源とボーレートを確認してください）"))?;

    for leg in LegSlot::ALL {
        if only.is_some_and(|l| l != leg) {
            continue;
        }
        let req = match only_joint {
            Some(k) => BusRequest::ClearMultiTurnJoint(k),
            None => BusRequest::ClearMultiTurn,
        };
        array.bus(leg).request(req).map_err(|e| e.to_string())?;
    }
    // 各バスが 0x95 を送ってフレームを張り直すまで待つ。
    std::thread::sleep(SETTLE);
    array
        .wait_anchored(Duration::from_secs(3))
        .map_err(|e| format!("{e}（フレームの張り直しに失敗しました）"))?;
    std::thread::sleep(SETTLE);

    println!();
    println!("完了。いまの絶対角:");
    for leg in LegSlot::ALL {
        if only.is_some_and(|l| l != leg) {
            continue;
        }
        let st = array.bus(leg).state();
        let q: Vec<String> = st
            .iter()
            .map(|s| format!("{:+.4}", s.position_rad))
            .collect();
        println!("  {} q(model) = [{}] rad", leg.prefix(), q.join(" "));
    }
    Ok(())
}

// ── 共通 ────────────────────────────────────────────────────────────────

fn leg_filter(cli: &Cli) -> Result<Option<LegSlot>, String> {
    match cli.str("leg") {
        None => Ok(None),
        Some(s) => LegSlot::from_prefix(&s.to_ascii_uppercase())
            .map(Some)
            .ok_or_else(|| format!("--leg {s:?} が不正です（FL/FR/RL/RR）")),
    }
}

/// `--leg` と `--joint` から対象を決める。どちらも必須。
fn target_joint(cli: &Cli) -> Result<(LegSlot, usize), String> {
    let leg = leg_filter(cli)?.ok_or("--leg FL|FR|RL|RR が必要です")?;
    let joint = cli
        .str("joint")
        .ok_or("--joint hip|thigh|calf が必要です")?;
    let k = LEG_JOINT_KINDS
        .iter()
        .position(|kind| *kind == joint)
        .ok_or_else(|| format!("--joint {joint:?} が不正です（hip|thigh|calf）"))?;
    Ok((leg, k))
}

fn joint_label(cfg: &AppConfig, leg: LegSlot, k: usize) -> String {
    let id = cfg
        .hardware
        .bus_for(leg)
        .and_then(|b| b.motors.get(k))
        .map(|m| m.id)
        .unwrap_or(0);
    format!("{}_{}_joint (id {id})", leg.prefix(), LEG_JOINT_KINDS[k])
}

fn bus_index(hw: &HardwareConfig, leg: LegSlot) -> Result<usize, String> {
    hw.legs
        .bus
        .iter()
        .position(|b| b.leg_slot().ok() == Some(leg))
        .ok_or_else(|| format!("脚 {} の設定がありません", leg.prefix()))
}

/// 1 軸の現在角（モデル座標系）。バススレッドが 1 周するのを待ってから読む。
fn measure_one(bus: &LegBus, k: usize) -> Result<f64, String> {
    let deadline = Instant::now() + Duration::from_secs(2);
    while Instant::now() < deadline {
        let s = bus.state()[k];
        if s.ok {
            return Ok(s.position_rad);
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    Err(format!(
        "{} 軸{k} が応答しません（{}）",
        bus.leg().prefix(),
        bus.last_error()
    ))
}

/// 設定を書き出す。
///
/// **コメントは保たれない**（`AppConfig` から作り直すため）。`config` サブ
/// コマンドが生成したファイルを校正で上書きしていく運用を前提にしている。
fn write_config(cfg: &AppConfig, path: &str) -> Result<(), String> {
    cfg.validate()?;
    let text = cfg.to_toml()?;
    std::fs::write(path, text).map_err(|e| format!("{path} に書けません: {e}"))
}

fn read_line() -> String {
    let mut line = String::new();
    let _ = std::io::stdin().lock().read_line(&mut line);
    line
}

/// `calib single-turn` — 単回転絶対角（`0x94`）を 12 軸ぶん読む。**読むだけ。**
///
/// # 何の役に立つのか
///
/// いま位置の基準にしている `0x92`（マルチターン）は、**電源投入時の姿勢が
/// 0** になる。つまりモータ電源を切ると 12 軸の `zero_pose_rad` が全部無効に
/// なり、メカ作業のたびに測り直しになる。
///
/// `0x94` の基準はドライバの ROM に入ったエンコーダゼロなので、**電源 OFF/ON
/// をまたいで同じ値になる**。代わりに 1 モータ回転で一周する:
///
/// | 軸 | 減速比 | 一周 = 関節 |
/// |---|---|---|
/// | hip / thigh | 10.0 | 36.0° |
/// | calf | 15.556 | 23.1° |
///
/// 何回転目かは別の手段（既知の姿勢で電源を入れる、メカ端に当てる、など）で
/// 決める。**その精度は「一周の半分」でよい**ので、いまの「毎回まったく同じ
/// 伏せ姿勢」より要求がずっと緩い。
///
/// # 使い方
///
/// 再現性の確認:
/// 1. これを実行して `0x94 raw` を控える
/// 2. **関節を動かさずに**モータ電源を OFF → ON
/// 3. もう一度実行して raw が一致するか見る
fn single_turn(cfg: &AppConfig, cli: &Cli) -> Result<(), String> {
    let only = leg_filter(cli)?;
    let array = LegArray::connect(&cfg.hardware).map_err(|e| e.to_string())?;
    array
        .wait_anchored(Duration::from_secs(3))
        .map_err(|e| format!("{e}（モータ電源とボーレートを確認してください）"))?;

    for leg in LegSlot::ALL {
        if only.is_some_and(|l| l != leg) {
            continue;
        }
        array
            .bus(leg)
            .request(BusRequest::ReadSingleTurn)
            .map_err(|e| e.to_string())?;
    }
    std::thread::sleep(SETTLE);

    println!("単回転絶対角 0x94（**読むだけ。何も書きません**）");
    println!();
    println!(
        "{:<4} {:<6} {:>10} {:>10} {:>12} {:>12}",
        "脚", "軸", "0x94 raw", "モータ°", "関節内°", "一周=関節°"
    );
    let mut missing = Vec::new();
    for leg in LegSlot::ALL {
        if only.is_some_and(|l| l != leg) {
            continue;
        }
        let bus = array.bus(leg);
        let raw = bus.single_turn();
        for (k, v) in raw.iter().enumerate() {
            let gear = cfg
                .hardware
                .bus_for(leg)
                .and_then(|b| b.motors.get(k))
                .map_or(cfg.hardware.legs.gear_ratio, |m| {
                    m.gear_ratio_or(cfg.hardware.legs.gear_ratio)
                });
            let wrap_deg = 360.0 / gear;
            match v {
                Some(c) => {
                    let motor_deg = *c as f64 / 100.0;
                    println!(
                        "{:<4} {:<6} {:>10} {:>10.2} {:>12.3} {:>12.2}",
                        leg.prefix(),
                        LEG_JOINT_KINDS[k],
                        c,
                        motor_deg,
                        motor_deg / gear,
                        wrap_deg
                    );
                }
                None => {
                    println!(
                        "{:<4} {:<6} {:>10} {:>10} {:>12} {:>12.2}",
                        leg.prefix(),
                        LEG_JOINT_KINDS[k],
                        "**読めず**",
                        "-",
                        "-",
                        wrap_deg
                    );
                    missing.push(format!("{} {}", leg.prefix(), LEG_JOINT_KINDS[k]));
                }
            }
        }
    }
    println!();
    if missing.is_empty() {
        println!("**raw を控えておくこと。** 電源 OFF/ON をまたいで一致すれば、");
        println!("ここを基準にした電源非依存のゼロ点が作れる。");
    } else {
        println!("**読めなかった軸がある: {}**", missing.join(", "));
        println!("直近エラー:");
        for leg in LegSlot::ALL {
            let e = array.bus(leg).last_error();
            if !e.is_empty() {
                println!("  {} {}", leg.prefix(), e);
            }
        }
    }
    Ok(())
}

/// `calib clear-error` — ドライバの異常フラグを消す（`0x9B`）。
///
/// # 原因が残っていると消えない
///
/// マニュアル §2 に「the error flags cannot be cleared while the motor state
/// has not yet returned to normal」とある。**低電圧保護ならバス電圧を戻して
/// から実行すること。** 電圧が下がったままいくら投げても消えない。
///
/// だから**起動時の自動クリアはしない**。効かないうえ、効いてしまう場合は
/// 「生きている異常を握り潰して動き出す」ことになる。消すのは人が原因を
/// 潰したと判断したときだけ。
fn clear_error(cfg: &AppConfig, cli: &Cli) -> Result<(), String> {
    let only = leg_filter(cli)?;
    let array = LegArray::connect(&cfg.hardware).map_err(|e| e.to_string())?;
    array
        .wait_anchored(Duration::from_secs(3))
        .map_err(|e| format!("{e}（モータ電源とボーレートを確認してください）"))?;
    // 現状を掴むために 1 巡ぶん待つ（status は軸ごとに順番に読まれる）。
    std::thread::sleep(Duration::from_millis(
        cfg.hardware.legs.status_interval_ms * 4,
    ));

    println!("クリア前:");
    let before = report_faults(&array, only);
    if before == 0 {
        println!("  異常なし。何もしません");
        println!();
        println!("**このコマンドは異常が無ければ何も書きません。**");
        println!("電圧つきの 12 軸の状態を見るだけの用途に使えます。");
        return Ok(());
    }

    for leg in LegSlot::ALL {
        if only.is_some_and(|l| l != leg) {
            continue;
        }
        array
            .bus(leg)
            .request(BusRequest::ClearError)
            .map_err(|e| e.to_string())?;
    }
    std::thread::sleep(SETTLE);

    println!();
    println!("クリア後:");
    let after = report_faults(&array, only);
    println!();
    if after == 0 {
        println!("**消えました。**");
    } else {
        println!("**{after} 軸で消えませんでした。原因がまだ残っています。**");
        println!("マニュアル §2: 状態が正常に戻るまでフラグは消せません。");
        println!("  低電圧保護 … 電源電圧を確認（電流制限に当たっていませんか）");
        println!("  過熱       … 冷えるまで待つ");
    }
    Ok(())
}

/// 12 軸（または指定脚）の異常を並べる。戻り値は異常が立っている軸数。
fn report_faults(array: &LegArray, only: Option<LegSlot>) -> usize {
    let mut n = 0;
    for leg in LegSlot::ALL {
        if only.is_some_and(|l| l != leg) {
            continue;
        }
        for (k, st) in array.bus(leg).status().iter().enumerate() {
            if !st.valid {
                println!("  {} {} … 未読", leg.prefix(), LEG_JOINT_KINDS[k]);
                continue;
            }
            if st.faulted() {
                n += 1;
                println!(
                    "  {} {:<5} **{}**（0x{:02X}）  {:.1} V / {:.0} °C",
                    leg.prefix(),
                    LEG_JOINT_KINDS[k],
                    st.describe(),
                    st.error_raw,
                    st.voltage_v,
                    st.temperature_c
                );
            } else {
                println!(
                    "  {} {:<5} 正常  {:.1} V / {:.0} °C",
                    leg.prefix(),
                    LEG_JOINT_KINDS[k],
                    st.voltage_v,
                    st.temperature_c
                );
            }
        }
    }
    n
}

/// `calib restart` — ドライバを再起動する（`0x07`）。**電源再投入と等価。**
///
/// # 何のために要るのか
///
/// 低電圧保護（`0x01`）はヒステリシスを持ち、**トリップ電圧より高い電圧まで
/// 戻さないと解除されない**（実測: トリップは 19.8 V 未満、リセットは
/// 19.9〜20.9 V の間）。安定化電源なら上げて戻せるが、**バッテリの電圧は
/// 上がらない**。試合中にトリップすると交換か充電まで復帰できない。
///
/// 再起動が保護を解除できるなら、そこが**バッテリ運用での唯一の復帰経路**に
/// なる。この副コマンドはそれを確かめるためのもの。
///
/// # 2 つの未知
///
/// - **RS485 マニュアルに `0x07` は無い**（記載は CAN §29 のみ）。RS485 の
///   ファームが受け付けるかは未検証。応答も返らないので、成否は状態を
///   読み直すしかない
/// - **再起動しても電圧が低いままなら、起動時の判定で再びトリップするかも
///   しれない。** そうならこの経路は使えない
///
/// # 代償
///
/// **マルチターン原点がリセットされる。** 実行後の原点は「そのときの姿勢」に
/// なるので、`zero_pose_rad` は伏せ姿勢で実行したときだけ従来どおり有効。
/// **伏せ姿勢以外で実行したら 12 軸のゼロ点は測り直し。**
fn restart(cfg: &AppConfig, cli: &Cli) -> Result<(), String> {
    let only = leg_filter(cli)?;
    let only_joint = match cli.str("joint") {
        None => None,
        Some(name) => Some(
            LEG_JOINT_KINDS
                .iter()
                .position(|k| *k == name)
                .ok_or_else(|| format!("--joint {name:?} が不正です（hip/thigh/calf）"))?,
        ),
    };
    if only_joint.is_some() && only.is_none() {
        return Err("--joint を使うときは --leg も指定してください".into());
    }

    match (only, only_joint) {
        (Some(l), Some(k)) => println!(
            "**{} の {} だけ**ドライバを再起動します（電源再投入と等価）。",
            l.prefix(),
            LEG_JOINT_KINDS[k]
        ),
        (Some(l), None) => println!(
            "**{} の 3 軸**のドライバを再起動します（電源再投入と等価）。",
            l.prefix()
        ),
        _ => println!("**12 軸すべて**のドライバを再起動します（電源再投入と等価）。"),
    }
    println!();
    println!("**マルチターン原点がリセットされます。**");
    println!("実行後は「いまの姿勢」が新しい原点になるので、");
    println!("**伏せ姿勢で実行しない限り zero_pose_rad は測り直しです。**");
    println!();
    println!("`0x07` は RS485 マニュアルに記載がありません（CAN §29 のみ）。");
    println!("応答も返らないので、効いたかどうかは状態を読み直して確かめます。");
    println!();
    println!("続けるなら Enter、やめるなら Ctrl-C");
    let _ = read_line();

    let array = LegArray::connect(&cfg.hardware).map_err(|e| e.to_string())?;
    array
        .wait_anchored(Duration::from_secs(3))
        .map_err(|e| format!("{e}（モータ電源とボーレートを確認してください）"))?;
    std::thread::sleep(Duration::from_millis(
        cfg.hardware.legs.status_interval_ms * 4,
    ));

    println!();
    println!("再起動前:");
    let before = report_faults(&array, only);
    let q_before = array.states();

    for leg in LegSlot::ALL {
        if only.is_some_and(|l| l != leg) {
            continue;
        }
        let req = match only_joint {
            Some(k) => BusRequest::RestartJoint(k),
            None => BusRequest::Restart,
        };
        array.bus(leg).request(req).map_err(|e| e.to_string())?;
    }
    // ドライバの再起動 + フレーム張り直し + status の 1 巡ぶん。
    std::thread::sleep(Duration::from_secs(2));
    array
        .wait_anchored(Duration::from_secs(5))
        .map_err(|e| format!("{e}（再起動後にモータが応答していません）"))?;
    std::thread::sleep(Duration::from_millis(
        cfg.hardware.legs.status_interval_ms * 4,
    ));

    println!();
    println!("再起動後:");
    let after = report_faults(&array, only);

    println!();
    println!("マルチターン角の変化（原点がリセットされたかの確認）:");
    let q_after = array.states();
    for (leg, (b, a)) in LegSlot::ALL.iter().zip(q_before.iter().zip(q_after.iter())) {
        if only.is_some_and(|l| l != *leg) {
            continue;
        }
        let d: Vec<String> = b
            .iter()
            .zip(a.iter())
            .map(|(x, y)| format!("{:+.3}", y.position_rad - x.position_rad))
            .collect();
        println!("  {} Δq(model) = [{}] rad", leg.prefix(), d.join(" "));
    }

    println!();
    if before > 0 && after == 0 {
        println!("**異常が消えました。再起動は保護の解除に使えます。**");
        println!("バッテリ運用での復帰経路になり得ます（0x94 への移行と組み合わせ）。");
    } else if before > 0 {
        println!("**{after} 軸で異常が残りました。再起動では解除できません。**");
        println!("起動時の判定で再びトリップしている可能性が高いです。");
    } else {
        println!("元から異常はありませんでした（再起動が効いたかは別途確認）。");
    }
    println!();
    println!("**Δq が 0 でないなら原点が動いています。zero_pose_rad は測り直しです。**");
    Ok(())
}
