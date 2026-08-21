//! 四脚ロボット namiashi の実機制御アプリ。
//!
//! ```text
//! namiashi ports                    CH348 のポートを UART 番号つきで一覧
//! namiashi config [--out PATH]      既定設定を TOML で書き出す
//! namiashi check                    設定とモデルを検証（実機に触れない）
//! namiashi dump [--gait ..] [--vx]  歩容を実機なしで再生し関節角を検証
//! namiashi calib <sub>              符号・ゼロ点・可動域を実機で確定する
//! namiashi imu | sbus | legs        実機の受信 / 状態だけを観測（動かさない）
//! namiashi run                      制御ループ（プロポ操縦）
//! ```
//!
//! 立ち上げの順番は上から下。`check` → `ports` → `imu` / `sbus` / `legs` が
//! 通ってから `run` に行くと、どこで詰まったのかが常に 1 段で分かる。

mod calib;
mod chicken;
mod config;
mod controller;
mod diag;
mod dump;
mod jointvec;
mod pose;
mod robot;
mod runner;
mod teleop;
mod viz;

use config::AppConfig;

fn main() {
    // 既定は info。うるさければ `RUST_LOG=warn`、追い込むときは `debug`。
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .format_timestamp_millis()
        .init();

    let cli = Cli::parse(std::env::args().skip(1));
    if cli.wants_help() {
        print_help();
        return;
    }

    if let Err(e) = dispatch(&cli) {
        eprintln!("エラー: {e}");
        std::process::exit(1);
    }
}

fn dispatch(cli: &Cli) -> Result<(), String> {
    let command = cli.command();
    match command {
        // 設定を読まずに済むものを先に。
        "ports" => return diag::ports(),
        "config" => return write_config(cli),
        _ => {}
    }

    let cfg = load_config(cli)?;
    match command {
        "check" => diag::check(&cfg),
        "dump" => dump::run(&cfg, cli),
        "calib" => calib::run(&cfg, cli),
        "imu" => diag::imu(&cfg, secs_or_forever(cli, 10.0)),
        "sbus" => diag::sbus(&cfg, secs_or_forever(cli, 10.0), cli.flag("plain")),
        "legs" => diag::legs(&cfg, secs_or_forever(cli, 10.0)),
        "run" => {
            let robot = robot::load_from_config(&cfg)?;
            let opts = runner::RunOptions {
                allow_no_sbus: cli.flag("allow-no-sbus"),
                skip_zero: cli.flag("skip-zero"),
                status_interval_s: cli.f64("status").unwrap_or(1.0),
                viz: viz_config(cli),
            };
            runner::run(cfg, robot, opts)
        }
        other => Err(format!(
            "未知のコマンド {other:?}。`namiashi --help` を見てください"
        )),
    }
}

/// 観測コマンドの継続時間。`--forever` なら `None`（Ctrl-C まで）。
///
/// 立ち上げ中は「手で動かしながら眺める」ので秒数を決め打ちできない。
/// `--secs 0` を無限の意味にするのは避けた。U_BOOT_TIMEOUT=0 が「即起動」
/// ではなく「無限に待つ」で紛らわしいのと同じ罠になるため
/// （`doc/boot_config.md` の U-Boot の節）。
fn secs_or_forever(cli: &Cli, default: f64) -> Option<f64> {
    if cli.flag("forever") {
        None
    } else {
        Some(cli.f64("secs").unwrap_or(default))
    }
}

/// `--viz` 系のオプションを読む。
pub fn viz_config(cli: &Cli) -> viz::VizConfig {
    let d = viz::VizConfig::default();
    viz::VizConfig {
        enabled: cli.flag("viz"),
        key: cli.str("viz-key").unwrap_or(&d.key).to_string(),
        rate_hz: cli.f64("viz-rate").unwrap_or(d.rate_hz),
        endpoint: cli.str("viz-endpoint").map(|s| s.to_string()),
    }
}

/// `--config` があればそれを読み、無ければ既定を使う。
fn load_config(cli: &Cli) -> Result<AppConfig, String> {
    match cli.str("config") {
        Some(path) => {
            let cfg = AppConfig::load(path)?;
            log::info!("設定 {path} を読みました");
            Ok(cfg)
        }
        None => {
            log::info!("設定ファイルの指定がないので既定値を使います（--config で指定）");
            let cfg = AppConfig::default();
            cfg.validate()?;
            Ok(cfg)
        }
    }
}

fn write_config(cli: &Cli) -> Result<(), String> {
    let text = AppConfig::default().to_toml()?;
    match cli.str("out") {
        Some(path) => {
            std::fs::write(path, &text).map_err(|e| format!("{path} に書けません: {e}"))?;
            println!("{path} に既定設定を書き出しました");
        }
        None => print!("{text}"),
    }
    Ok(())
}

fn print_help() {
    println!(
        r#"namiashi — 四脚ロボット namiashi の実機制御アプリ

使い方:
  namiashi <コマンド> [オプション]

コマンド:
  ports                     CH348 のポートを物理 UART 番号つきで一覧（何も開かない）
  config [--out PATH]       既定設定を TOML で出力
  check                     設定とモデルを検証（実機に触れない）
  dump                      歩容を実機なしで再生し、関節角と可動域を検証
  imu    [--secs S]         IMU の値を表示（モータには触れない）
  sbus   [--secs S] [--plain]
                            プロポ入力と解釈結果を表示（同上）
                            既定は再描画表示。--plain で 1 行 / 更新の逐次出力
  legs   [--secs S]         脚バスの状態と実効周期を表示（**指令は送らない**）

  imu / sbus / legs は --forever で Ctrl-C まで回り続ける（--secs より優先）。
  calib  <sub>              符号・ゼロ点・可動域を実機で確定して設定に書き戻す
  run                       制御ループ（プロポ操縦）

共通オプション:
  --config PATH             設定 TOML（省略時は組み込みの既定値）

dump のオプション:
  --gait crawl|walk|trot    歩容（既定 crawl）
  --vx V --vy V --wz V      速度指令（既定 vx=0.05）
  --secs S                  再生時間（既定 4）
  --every N                 N 周期ごとに 1 行出す（既定 20）
  --realtime                実時間で流す（--viz で articara に見せるとき用）

calib のサブコマンド:
  scan  [--leg FL] [--max-id N]         応答するモータ id を数える（指令なし）
  move  --leg FL --joint thigh          1 軸だけ小さく動かして sign を決める
        [--deg D] [--speed R] [--assume y|n] [--write PATH]
  range --leg FL --joint thigh          脱力させ、手で動かして可動域を測る
        [--secs S | --forever] [--margin RAD] [--write PATH]
        --forever なら Ctrl-C で確定（打ち切っても集計と --write は走る）
  zero  [--pose NAME] [--write PATH]    全軸ゼロ出し + zero_pose_rad を記録

  1 度に 1 軸しか投入せず、既定の振り幅は 5°・速度 0.3 rad/s。
  --write を付けたときだけ設定ファイルへ書き戻す。

ライブ可視化のオプション（dump / run 共通）:
  --viz                     各周期の姿勢を Zenoh へ配信し articara に描かせる
  --viz-key KEY             Zenoh キー（既定 go2/gait/planned）
  --viz-rate HZ             配信レート（既定 50）
  --viz-endpoint EP         例 tcp/127.0.0.1:7447（マルチキャスト不可の環境）

run のオプション:
  --allow-no-sbus           受信機なしでも起動する（ベンチで脚を浮かせた時のみ）
  --skip-zero               起動時のゼロ出しを省略する
  --status S                状態表示の間隔 [s]（0 で表示しない、既定 1）

立ち上げの順番:
  namiashi check  →  ports  →  imu / sbus / legs
    →  calib scan  →  calib range/move（12 軸ぶん）  →  calib zero  →  run

articara で見る:
  1) namiashi dump --gait trot --vx 0.1 --secs 60 --realtime --viz \
       --viz-endpoint tcp/127.0.0.1:7447
  2) 別端末で articara を起動しモデル models/namiashi.misa を開く
     （cd ../articara && cargo run --release --features viz）
  3) Live gait feed パネルで同じキー / エンドポイントを入れて Start
"#
    );
}

/// 素朴なコマンドライン解析。`--key value` と `--key=value`、
/// [`BOOL_FLAGS`] に載っているものは値を取らない存在フラグ。
pub struct Cli {
    pub positionals: Vec<String>,
    flags: std::collections::HashMap<String, String>,
}

/// 値を取らないフラグ。ここに無いものは次のトークンを値として食う。
const BOOL_FLAGS: &[&str] =
    &["help", "allow-no-sbus", "skip-zero", "viz", "realtime", "plain", "forever"];

impl Cli {
    pub fn parse(args: impl Iterator<Item = String>) -> Self {
        let mut positionals = Vec::new();
        let mut flags = std::collections::HashMap::new();
        let mut it = args.peekable();
        while let Some(arg) = it.next() {
            let Some(name) = arg.strip_prefix("--") else {
                if arg == "-h" {
                    flags.insert("help".into(), "true".into());
                } else {
                    positionals.push(arg);
                }
                continue;
            };
            if let Some((k, v)) = name.split_once('=') {
                flags.insert(k.to_string(), v.to_string());
            } else if BOOL_FLAGS.contains(&name) {
                flags.insert(name.to_string(), "true".into());
            } else {
                flags.insert(name.to_string(), it.next().unwrap_or_default());
            }
        }
        Self { flags, positionals }
    }

    pub fn command(&self) -> &str {
        self.positionals
            .first()
            .map(|s| s.as_str())
            .unwrap_or("check")
    }

    pub fn wants_help(&self) -> bool {
        self.flags.contains_key("help") || self.positionals.iter().any(|p| p == "help")
    }

    pub fn str(&self, key: &str) -> Option<&str> {
        self.flags
            .get(key)
            .map(|s| s.as_str())
            .filter(|s| !s.is_empty())
    }

    pub fn f64(&self, key: &str) -> Option<f64> {
        self.str(key).and_then(|s| s.parse().ok())
    }

    pub fn usize(&self, key: &str) -> Option<usize> {
        self.str(key).and_then(|s| s.parse().ok())
    }

    pub fn flag(&self, key: &str) -> bool {
        self.flags.get(key).map(|v| v != "false").unwrap_or(false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cli(args: &[&str]) -> Cli {
        Cli::parse(args.iter().map(|s| s.to_string()))
    }

    #[test]
    fn the_default_command_is_the_harmless_one() {
        // 引数なしで実機が動き出さないこと。
        assert_eq!(cli(&[]).command(), "check");
    }

    #[test]
    fn both_flag_spellings_parse() {
        let c = cli(&["run", "--status", "2.5", "--config=/tmp/a.toml"]);
        assert_eq!(c.command(), "run");
        assert_eq!(c.f64("status"), Some(2.5));
        assert_eq!(c.str("config"), Some("/tmp/a.toml"));
    }

    #[test]
    fn a_bool_flag_does_not_eat_the_next_token() {
        let c = cli(&["run", "--allow-no-sbus", "--secs", "3"]);
        assert!(c.flag("allow-no-sbus"));
        assert_eq!(c.f64("secs"), Some(3.0));
    }

    #[test]
    fn help_is_recognised_in_every_spelling() {
        assert!(cli(&["--help"]).wants_help());
        assert!(cli(&["-h"]).wants_help());
        assert!(cli(&["help"]).wants_help());
        assert!(!cli(&["run"]).wants_help());
    }
}
