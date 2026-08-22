//! アプリ設定（TOML）。実機設定 (`namiashi_hal::config`) と同じファイルに同居する。
//!
//! ファイル 1 枚に `[hardware]` と `[control] [gait] [teleop] [poses]` を並べる
//! 形にしてある。配線とチューニングを別ファイルに分けると、現場で片方だけ
//! 持ち出して食い違う。

use namiashi_hal::config::HardwareConfig;
use serde::{Deserialize, Serialize};

use crate::teleop::TeleopConfig;

/// 設定全体。
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct AppConfig {
    #[serde(default)]
    pub control: ControlConfig,
    #[serde(default)]
    pub gait: GaitTuning,
    #[serde(default)]
    pub teleop: TeleopConfig,
    #[serde(default)]
    pub poses: PoseConfig,
    #[serde(default)]
    pub hardware: HardwareConfig,
}

impl AppConfig {
    pub fn from_toml(text: &str) -> Result<Self, String> {
        // **`[hardware]` が無いファイルは設定として受け取らない。**
        //
        // 全フィールドに serde の既定値が入っているので、**TOML なら何でも
        // 「設定」として通ってしまう**。実際、モデルの `.misa` を
        // `--config` に渡すと「設定: OK」と出て、ゼロ点 0 / 符号 +1 の
        // **未校正の既定値**で実機を動かせてしまった。
        //
        // 既定値で走らせたい正規の入口は `--config` を**付けない**こと。
        // ファイルを指定した以上、その中身が設定であることを確かめる。
        let raw: toml::Value =
            toml::from_str(text).map_err(|e| format!("TOML の解析に失敗: {e}"))?;
        if raw.get("hardware").is_none() {
            let looks_like_model = raw.get("pose").is_some() || raw.get("joint").is_some();
            return Err(format!(
                "設定ファイルに [hardware] がありません{}。\
                 既定値で走らせたいなら --config を付けないでください",
                if looks_like_model {
                    "（`[[pose]]` があります — **モデルファイルを渡していませんか**）"
                } else {
                    ""
                }
            ));
        }
        let cfg: AppConfig = toml::from_str(text).map_err(|e| format!("TOML の解析に失敗: {e}"))?;
        cfg.validate()?;
        Ok(cfg)
    }

    pub fn load(path: impl AsRef<std::path::Path>) -> Result<Self, String> {
        let path = path.as_ref();
        let text = std::fs::read_to_string(path)
            .map_err(|e| format!("{} を読めません: {e}", path.display()))?;
        Self::from_toml(&text)
    }

    pub fn to_toml(&self) -> Result<String, String> {
        toml::to_string_pretty(self).map_err(|e| format!("TOML の生成に失敗: {e}"))
    }

    pub fn validate(&self) -> Result<(), String> {
        self.hardware.validate().map_err(|e| e.to_string())?;
        if self.control.rate_hz <= 0.0 {
            return Err("control.rate_hz は正の値が必要です".into());
        }
        // 制御周期がバス周期より速いと、同じ指令を 2 回送るだけで意味がない。
        if self.control.rate_hz > self.hardware.legs.bus_rate_hz {
            return Err(format!(
                "control.rate_hz ({}) が legs.bus_rate_hz ({}) を超えています。\
                 バスが追いつかないので制御周期を落とすか、バス周期を上げてください",
                self.control.rate_hz, self.hardware.legs.bus_rate_hz
            ));
        }
        self.teleop.validate()?;
        if self.gait.max_vx_m_s <= 0.0
            || self.gait.max_vy_m_s <= 0.0
            || self.gait.max_wz_rad_s <= 0.0
        {
            return Err("gait の速度上限は正の値が必要です".into());
        }
        Ok(())
    }
}

/// 制御ループ全体の設定。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControlConfig {
    /// ロボットモデル (`.misa`)。ポーズ・シーケンスもここから読む。
    #[serde(default = "default_model_path")]
    pub model: String,
    /// 制御ループの周期 (Hz)。バス周期以下であること。
    #[serde(default = "default_rate_hz")]
    pub rate_hz: f64,
    /// 姿勢を切り替えるときの既定の遷移時間 (s)。
    #[serde(default = "default_transition_s")]
    pub transition_s: f64,
    /// 起動直後に取る姿勢の名前（`.misa` の `[[pose]]`）。
    ///
    /// **250×350×700 mm の直方体に収める初期姿勢はここで指す。** 実際にどの
    /// 姿勢にするかは別途詰めるので、既定はモデルに入っている畳んだ姿勢
    /// `constrain` にしてある（脚を伸ばしたまま起動するより安全側）。
    #[serde(default = "default_start_pose")]
    pub start_pose: String,
    /// 電源投入後の初回起動で、**いまの姿勢をマルチターン原点に張り直す**か。
    ///
    /// # 何のためにあるのか
    ///
    /// 角度規約は `q_model = sign * q_motor + zero_pose_rad` で、**モータ角 0 が
    /// 伏せ姿勢**と決めてある。モータのマルチターンカウンタは電源投入で 0 に
    /// なるので、本来は「伏せ姿勢でモータ電源を入れる」だけで原点が揃う。
    ///
    /// ところが SBC はモータと同じ電源から作っているので、電源投入の瞬間は
    /// SBC のブート中で、操縦者がまだロボットを置いている最中かもしれない。
    /// 原点が決まる瞬間を人が狙えない。これを有効にすると、原点が決まる瞬間が
    /// **制御ループ開始の直前・脱力が確認できている時点**に移る。
    ///
    /// # 危険と、その封じ方
    ///
    /// このコマンドは**そのときの姿勢を無条件に原点にする**。立脚中に実行すれば
    /// 立脚姿勢が伏せ扱いになり、12 軸すべての `zero_pose_rad` が無効になる。
    ///
    /// したがって**電源投入後の初回起動でしか実行しない**。目印を
    /// `/dev/shm`（tmpfs。再起動で必ず消える）に置いて判定する。試合中に
    /// クラッシュしてサービスが再起動しても、そこでは張り直さない。
    /// SBC とモータが同じ電源なので「再起動で消える」＝「モータ電源が
    /// 入り直した」と一致する。**電源系統を分けたらこの前提は崩れる。**
    #[serde(default)]
    pub zero_multiturn_on_boot: bool,
    /// 脚の運動学を自動検出するときに使う「立った姿勢」の名前。
    ///
    /// この姿勢での順運動学から公称の脚の高さが決まるので、脚を伸ばし切った
    /// 姿勢を指すと歩容の立ち位置が高くなりすぎる。
    #[serde(default = "default_kinematics_pose")]
    pub kinematics_pose: String,
    /// S.BUS が途絶えたとみなすまでの時間 (ms)。
    #[serde(default = "default_teleop_timeout_ms")]
    pub teleop_timeout_ms: u64,
}

fn default_model_path() -> String {
    "models/namiashi.misa".into()
}
fn default_rate_hz() -> f64 {
    200.0
}
fn default_transition_s() -> f64 {
    1.5
}
fn default_start_pose() -> String {
    // namiashi.misa に入っている畳んだ姿勢（thigh 1.0 / calf -2.0）。
    "constrain".into()
}
fn default_kinematics_pose() -> String {
    // namiashi.misa の軽く膝を曲げた姿勢（thigh 0.3 / calf -0.6）。
    "extend".into()
}
fn default_teleop_timeout_ms() -> u64 {
    100
}

impl Default for ControlConfig {
    fn default() -> Self {
        Self {
            model: default_model_path(),
            rate_hz: default_rate_hz(),
            transition_s: default_transition_s(),
            start_pose: default_start_pose(),
            // 既定は false。**姿勢を無条件に原点にする**副作用があるので、
            // 明示的に書いた設定でだけ有効になるようにしてある。
            zero_multiturn_on_boot: false,
            kinematics_pose: default_kinematics_pose(),
            teleop_timeout_ms: default_teleop_timeout_ms(),
        }
    }
}

/// 歩容のチューニング。プロポで選ぶ 3 種（Crawl / Walk / Trot）の共通部分と、
/// 種別ごとの上書きを分けてある。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GaitTuning {
    /// 立ち姿勢での胴体高さ (m)。
    #[serde(default = "default_stance_height")]
    pub stance_height_m: f64,
    /// 遊脚の持ち上げ高さ (m)。
    #[serde(default = "default_swing_height")]
    pub swing_height_m: f64,
    /// 前後速度の上限 (m/s)。スティック全開でこの値。
    #[serde(default = "default_max_vx")]
    pub max_vx_m_s: f64,
    /// 左右（真横）速度の上限 (m/s)。
    #[serde(default = "default_max_vy")]
    pub max_vy_m_s: f64,
    /// 旋回速度の上限 (rad/s)。
    #[serde(default = "default_max_wz")]
    pub max_wz_rad_s: f64,
    /// プロポで胴体高さを動かせる幅 (m)。`stance_height_m ± この値`。
    #[serde(default = "default_height_range")]
    pub height_range_m: f64,
    /// Crawl を `LinearCrawl`（胴体を +X 直線に載せる専用プランナ）で走らせる。
    ///
    /// **これを true にすると横移動と旋回の指令が効かなくなる**（LinearCrawl は
    /// 前進しか扱わない）。直進の安定性を追い込むときだけ使う。
    #[serde(default)]
    pub crawl_use_linear: bool,
    /// 速度指令を 0 から最大まで振り切るのにかける時間 [s]。0 でランプ無し。
    ///
    /// **歩容はスティックが動いた瞬間に出力を階段状に飛ばす。** 実測で
    /// 制御 1 周期あたり Crawl 31.5 / Walk 23.0 / Trot 9.9 rad/s
    /// （Crawl は 5 ms で 9.0°）。2 tick 目以降は滑らかなので跳ぶのは
    /// 切り替わりの 1 点だけで、スティック側を鈍らせれば消える
    /// （0.5 s のランプで Crawl 31.5 → 3.34 rad/s）。
    #[serde(default = "default_velocity_ramp_s")]
    pub velocity_ramp_s: f64,
    /// 速度指令を**落とす**ときのランプ時間 [s]。0 でランプ無し。
    ///
    /// **上げる側 (`velocity_ramp_s`) と別にしてある。止まるのは速い方が
    /// よい。** スティックを中立に戻してから実際に止まるまでが長いと、
    /// リング外へ出る。上げる側は滑らかさのために鈍らせてよいが、
    /// 下げる側を同じ時間にする理由は無い。
    #[serde(default = "default_velocity_ramp_stop_s")]
    pub velocity_ramp_stop_s: f64,
    /// 速度をちょうど 0 にする前に、全脚接地を待つ上限 [s]。0 で待たない。
    ///
    /// 歩容は `v = 0` で静止姿勢へ分岐し、遊脚を一気に接地させる
    /// （実測 35.4 rad/s = 制御 1 周期で 10.2°）。全脚が接地した瞬間に
    /// 0 へ落とせば跳ばない。
    ///
    /// **ただし trot では 4 脚が同時に接地しないことがあり、その場合は
    /// 毎回この時間だけ待ち切る。** 長くすると「スティックを戻しても
    /// 止まらない」になるので、**遊脚 1 回ぶんで足りる長さにする**。
    /// かつて 2.0 s 固定だったため、停止に最大 2.5 s かかっていた。
    #[serde(default = "default_stop_settle_s")]
    pub stop_settle_s: f64,
    /// 胴体姿勢をプロポで傾けられる上限 (rad)。**0 で機能ごと無効**。
    ///
    /// CH8 が ON のとき、CH1 がロール、CH3 がピッチになる（OFF のときは
    /// CH1 = 横移動、CH3 = 高さのまま）。歩容が出した足先位置を回してから
    /// IK を解き直すので、**足は接地したまま胴体だけ傾く**。
    ///
    /// **傾けると脚の可動域を食う。** `start` ポーズの calf は可動域まで
    /// 10°、`constrain_2` は 5.55° しかない。大きくすると IK が届かない脚が
    /// 出る（クランプされて姿勢が崩れる）。`dump` で範囲内に収まる角度を
    /// 確かめてから上げること。
    #[serde(default)]
    pub body_attitude_max_rad: f64,
    /// 胴体姿勢の追従時定数 (s)。CH8 を切り替えた瞬間に胴体が跳ねないよう
    /// 一次遅れを入れる。0 で素通し。
    #[serde(default = "default_body_attitude_tau_s")]
    pub body_attitude_tau_s: f64,
    /// 歩容種別ごとの周期 (s)。指定が無ければ `quadruped-gait` のプリセット値。
    #[serde(default)]
    pub crawl_cycle_s: Option<f64>,
    #[serde(default)]
    pub walk_cycle_s: Option<f64>,
    #[serde(default)]
    pub trot_cycle_s: Option<f64>,
}

fn default_stance_height() -> f64 {
    // 脚は thigh 0.1528 + calf 0.1528 = 0.306 m。膝を曲げた常用姿勢としての初期値。
    0.20
}
fn default_swing_height() -> f64 {
    0.03
}
fn default_max_vx() -> f64 {
    0.15
}
fn default_max_vy() -> f64 {
    0.08
}
fn default_max_wz() -> f64 {
    0.6
}
fn default_velocity_ramp_s() -> f64 {
    0.5
}

fn default_velocity_ramp_stop_s() -> f64 {
    0.15
}

fn default_stop_settle_s() -> f64 {
    0.25
}

fn default_body_attitude_tau_s() -> f64 {
    0.15
}

fn default_height_range() -> f64 {
    0.04
}

impl Default for GaitTuning {
    fn default() -> Self {
        Self {
            stance_height_m: default_stance_height(),
            swing_height_m: default_swing_height(),
            max_vx_m_s: default_max_vx(),
            max_vy_m_s: default_max_vy(),
            max_wz_rad_s: default_max_wz(),
            height_range_m: default_height_range(),
            crawl_use_linear: false,
            velocity_ramp_s: default_velocity_ramp_s(),
            velocity_ramp_stop_s: default_velocity_ramp_stop_s(),
            stop_settle_s: default_stop_settle_s(),
            // **既定は無効。** 制御ループの出力に手が入る機能なので、
            // 設定で明示的に上げるまで従来と 1 ビットも変わらない出力を出す。
            body_attitude_max_rad: 0.0,
            body_attitude_tau_s: default_body_attitude_tau_s(),
            crawl_cycle_s: None,
            walk_cycle_s: None,
            trot_cycle_s: None,
        }
    }
}

/// ポーズ再生の設定。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PoseConfig {
    /// プロポのポーズ再生スイッチで走らせるもの。`.misa` の
    /// `[[sequence]]` 名、または `[[pose]]` 名。
    #[serde(default = "default_greeting")]
    pub greeting: String,
    /// **CH8 を押しながら CH7** で再生するもの。空なら [`Self::greeting`]。
    ///
    /// 振る足を現場で選べるようにするためのもの。空きチャンネルが無いので、
    /// 既にある CH8（姿勢モード）を修飾キーとして使う。
    #[serde(default = "default_greeting_alt")]
    pub greeting_alt: String,
    /// チキンヘッドの基準角 (rad)。胴体ピッチ 0 のときの腕角。
    #[serde(default)]
    pub chicken_head_base_rad: f64,
    /// チキンヘッドの補償ゲイン。1.0 で胴体ピッチを完全に打ち消す。
    #[serde(default = "default_chicken_gain")]
    pub chicken_head_gain: f64,
    /// チキンヘッドの一次遅れ時定数 (s)。IMU のノイズをサーボへ通さないため。
    #[serde(default = "default_chicken_tau")]
    pub chicken_head_tau_s: f64,
}

fn default_greeting() -> String {
    "greeting".into()
}

fn default_greeting_alt() -> String {
    String::new()
}
fn default_chicken_gain() -> f64 {
    1.0
}
fn default_chicken_tau() -> f64 {
    0.05
}

impl Default for PoseConfig {
    fn default() -> Self {
        Self {
            greeting: default_greeting(),
            greeting_alt: default_greeting_alt(),
            chicken_head_base_rad: 0.0,
            chicken_head_gain: default_chicken_gain(),
            chicken_head_tau_s: default_chicken_tau(),
        }
    }
}

#[cfg(test)]
mod tests {

    /// **モデルファイルを `--config` に渡せてしまってはいけない。**
    ///
    /// 全フィールドに serde の既定値があるので、放っておくと TOML なら
    /// 何でも通る。実際 `.misa` が「設定: OK」と出て、ゼロ点 0 / 符号 +1 の
    /// **未校正の既定値**で実機を動かせる状態だった (2026-08-22)。
    #[test]
    fn a_model_file_is_not_accepted_as_a_config() {
        let misa = r#"
[[pose]]
name = "start"

[pose.angles]
FL_hip_joint = 0.0
"#;
        let e = AppConfig::from_toml(misa).unwrap_err();
        assert!(e.contains("[hardware]"), "{e}");
        assert!(e.contains("モデルファイル"), "{e}");
    }

    #[test]
    fn an_empty_toml_is_not_accepted_as_a_config() {
        // ファイルを指定した以上、中身が設定であることを求める。
        // 既定値で走らせたいなら --config を付けない。
        assert!(AppConfig::from_toml("").is_err());
    }

    #[test]
    fn the_shipped_config_still_loads() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../config/namiashi.toml");
        let text = std::fs::read_to_string(path).expect("config/namiashi.toml が読めません");
        AppConfig::from_toml(&text).expect("同梱の設定が読めなくなっている");
    }
    use super::*;

    #[test]
    fn default_config_is_valid() {
        AppConfig::default().validate().unwrap();
    }

    #[test]
    fn default_config_round_trips_through_toml() {
        let cfg = AppConfig::default();
        let back = AppConfig::from_toml(&cfg.to_toml().unwrap()).unwrap();
        assert_eq!(cfg, back);
    }

    /// 設定ファイルを書かずに起動できること。
    ///
    /// **`from_toml("")` ではない。** `--config` を付けない経路は
    /// `AppConfig::default()` を直接使う（`main::load_config`）ので、
    /// 空 TOML が通るかどうかとは無関係。かつてこのテストは
    /// `from_toml("")` を見ており、**「TOML なら何でも設定として通る」
    /// という穴の方を守っていた**。
    #[test]
    fn the_defaults_are_usable_without_a_config_file() {
        AppConfig::default().validate().unwrap();
    }

    #[test]
    fn control_rate_above_the_bus_rate_is_rejected() {
        let mut cfg = AppConfig::default();
        cfg.control.rate_hz = cfg.hardware.legs.bus_rate_hz * 2.0;
        assert!(cfg.validate().is_err());
    }
}
