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
            chicken_head_base_rad: 0.0,
            chicken_head_gain: default_chicken_gain(),
            chicken_head_tau_s: default_chicken_tau(),
        }
    }
}

#[cfg(test)]
mod tests {
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

    #[test]
    fn an_empty_file_yields_the_defaults() {
        // 設定ファイルを書かずに起動できること。
        assert_eq!(AppConfig::from_toml("").unwrap(), AppConfig::default());
    }

    #[test]
    fn control_rate_above_the_bus_rate_is_rejected() {
        let mut cfg = AppConfig::default();
        cfg.control.rate_hz = cfg.hardware.legs.bus_rate_hz * 2.0;
        assert!(cfg.validate().is_err());
    }
}
