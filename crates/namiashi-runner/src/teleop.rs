//! プロポ（S.BUS）入力 → 操縦指令。
//!
//! チャンネル割り当てとエンドポイントは送信機側の設定で変わるので、すべて
//! 設定ファイルに出してある。既定は Futaba の一般的な並び
//! （1=エルロン, 2=エレベータ, 3=スロットル, 4=ラダー, 5〜8=スイッチ）。
//!
//! 値は**生のチャンネル値 (172..1811) で扱う**。µs 換算は送信機・受信機の
//! エンドポイント設定に依存する近似で、表示用と割り切るのが `sbus-protocol`
//! の方針（`raw_to_us` のドキュメント）だから。

use namiashi_hal::sbus::{SbusState, CHANNELS};
use serde::{Deserialize, Serialize};

/// S.BUS の生値の下限・中央・上限（`sbus_protocol::{RAW_MIN, RAW_MAX}` 準拠）。
pub const RAW_MIN: u16 = 172;
pub const RAW_CENTER: u16 = 992;
pub const RAW_MAX: u16 = 1811;

/// 1 本のスティック軸の割り当て。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AxisMap {
    /// プロポの CH 番号（**1 始まり**。送信機の表示と合わせる）。
    pub channel: usize,
    #[serde(default = "default_raw_min")]
    pub raw_min: u16,
    #[serde(default = "default_raw_center")]
    pub raw_center: u16,
    #[serde(default = "default_raw_max")]
    pub raw_max: u16,
    /// 出力の符号を反転する。
    #[serde(default)]
    pub reverse: bool,
    /// 中央付近の不感帯（正規化値、0..1）。スティックのガタで機体が
    /// じりじり動くのを止める。
    #[serde(default = "default_deadband")]
    pub deadband: f64,
    /// エクスポネンシャル（0..1）。中央付近を鈍くする。
    #[serde(default)]
    pub expo: f64,
}

fn default_raw_min() -> u16 {
    RAW_MIN
}
fn default_raw_center() -> u16 {
    RAW_CENTER
}
fn default_raw_max() -> u16 {
    RAW_MAX
}
fn default_deadband() -> f64 {
    0.06
}

impl AxisMap {
    fn new(channel: usize, reverse: bool) -> Self {
        Self {
            channel,
            raw_min: RAW_MIN,
            raw_center: RAW_CENTER,
            raw_max: RAW_MAX,
            reverse,
            deadband: default_deadband(),
            expo: 0.0,
        }
    }

    /// 正規化した軸値 (-1..=1)。チャンネル番号が範囲外なら 0。
    pub fn value(&self, state: &SbusState) -> f64 {
        let Some(raw) = self.raw(state) else {
            return 0.0;
        };
        let raw = raw as f64;
        let center = self.raw_center as f64;
        // 中央から上下で別々に正規化する。送信機のエンドポイントは
        // 上下非対称に設定できるため。
        let span = if raw >= center {
            (self.raw_max as f64 - center).max(1.0)
        } else {
            (center - self.raw_min as f64).max(1.0)
        };
        let mut v = ((raw - center) / span).clamp(-1.0, 1.0);
        v = apply_deadband(v, self.deadband);
        v = apply_expo(v, self.expo);
        if self.reverse {
            -v
        } else {
            v
        }
    }

    fn raw(&self, state: &SbusState) -> Option<u16> {
        // 設定は 1 始まり、配列は 0 始まり。
        let index = self.channel.checked_sub(1)?;
        state.channels.get(index).copied()
    }
}

fn apply_deadband(v: f64, deadband: f64) -> f64 {
    let db = deadband.clamp(0.0, 0.9);
    if v.abs() <= db {
        return 0.0;
    }
    // 不感帯の外側を -1..1 に張り直す。段差なく立ち上がる。
    v.signum() * ((v.abs() - db) / (1.0 - db))
}

fn apply_expo(v: f64, expo: f64) -> f64 {
    let e = expo.clamp(0.0, 1.0);
    (1.0 - e) * v + e * v * v * v
}

/// スイッチの割り当て。段数は `thresholds` の数 + 1。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SwitchMap {
    /// プロポの CH 番号（1 始まり）。
    pub channel: usize,
    /// 段の境界となる生値。昇順。
    pub thresholds: Vec<u16>,
}

impl SwitchMap {
    fn three_position(channel: usize) -> Self {
        Self {
            channel,
            // 172 / 992 / 1811 の 3 段を、その中間で切る。
            thresholds: vec![582, 1401],
        }
    }

    fn two_position(channel: usize) -> Self {
        Self {
            channel,
            thresholds: vec![RAW_CENTER],
        }
    }

    /// 現在の段 (0 始まり)。チャンネルが取れなければ 0。
    pub fn position(&self, state: &SbusState) -> usize {
        let Some(index) = self.channel.checked_sub(1) else {
            return 0;
        };
        let Some(&raw) = state.channels.get(index) else {
            return 0;
        };
        self.thresholds.iter().filter(|&&t| raw >= t).count()
    }

    /// 段数。
    pub fn positions(&self) -> usize {
        self.thresholds.len() + 1
    }
}

/// プロポ割り当てまとめ。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TeleopConfig {
    /// 前後（エレベータ）。
    pub vx: AxisMap,
    /// 左右真横（エルロン）。
    pub vy: AxisMap,
    /// 旋回（ラダー）。
    pub wz: AxisMap,
    /// 胴体高さ（スロットル）。
    pub height: AxisMap,
    /// 動作モード: 0=脱力, 1=起立, 2=歩行。
    pub mode: SwitchMap,
    /// 歩容: 0=Crawl, 1=Walk, 2=Trot。
    pub gait: SwitchMap,
    /// ポーズ再生トリガ（押している間 ON。立ち上がりで 1 回再生）。
    pub pose: SwitchMap,
    /// チキンヘッド ON/OFF。
    pub chicken_head: SwitchMap,
    /// **どちらの前足で手を振るか。** 下段で `poses.greeting`、
    /// 上段で `poses.greeting_alt`。
    ///
    /// ポーズ再生 (CH7 相当) は「押した瞬間」の 1 発なので、選択は別の
    /// スイッチで持つ必要がある。
    #[serde(default = "default_pose_select")]
    pub pose_select: SwitchMap,
    /// 腕サーボが繋がっているチャンネル（**受信機直結のときの観測用**）。
    ///
    /// アプリは腕を駆動しないが、同じチャンネルを読めば実機の腕がどこにいるか
    /// は分かる。ログ・可視化・モデル状態にはそれが要る。`None` で観測しない。
    #[serde(default = "default_arm_axis")]
    pub arm: Option<AxisMap>,
}

fn default_arm_axis() -> Option<AxisMap> {
    Some(AxisMap {
        // 観測用なので不感帯は入れない。実機の角度をそのまま知りたい。
        deadband: 0.0,
        ..AxisMap::new(9, false)
    })
}

impl Default for TeleopConfig {
    fn default() -> Self {
        Self {
            // エレベータは上で前進になるよう素直に、エルロンは
            // 「右に倒すと右へ」= vy 負なので反転。
            vx: AxisMap::new(2, false),
            vy: AxisMap::new(1, true),
            wz: AxisMap::new(4, true),
            height: AxisMap::new(3, false),
            mode: SwitchMap::three_position(5),
            gait: SwitchMap::three_position(6),
            pose: SwitchMap::two_position(7),
            chicken_head: SwitchMap::two_position(8),
            pose_select: default_pose_select(),
            arm: default_arm_axis(),
        }
    }
}

impl TeleopConfig {
    pub fn validate(&self) -> Result<(), String> {
        let mut axes = vec![
            ("vx", &self.vx),
            ("vy", &self.vy),
            ("wz", &self.wz),
            ("height", &self.height),
        ];
        if let Some(arm) = &self.arm {
            axes.push(("arm", arm));
        }
        for (name, axis) in axes {
            if axis.channel == 0 || axis.channel > CHANNELS {
                return Err(format!(
                    "teleop.{name}.channel は 1..={CHANNELS} です ({} が指定されました)",
                    axis.channel
                ));
            }
            if !(axis.raw_min < axis.raw_center && axis.raw_center < axis.raw_max) {
                return Err(format!(
                    "teleop.{name} のエンドポイントが不正です (min < center < max であること)"
                ));
            }
        }
        let switches = [
            ("mode", &self.mode, 3),
            ("gait", &self.gait, 3),
            ("pose", &self.pose, 2),
            ("chicken_head", &self.chicken_head, 2),
        ];
        for (name, sw, want) in switches {
            if sw.channel == 0 || sw.channel > CHANNELS {
                return Err(format!(
                    "teleop.{name}.channel は 1..={CHANNELS} です ({} が指定されました)",
                    sw.channel
                ));
            }
            if sw.positions() < want {
                return Err(format!(
                    "teleop.{name} は {want} 段必要ですが {} 段です",
                    sw.positions()
                ));
            }
            if sw.thresholds.windows(2).any(|w| w[0] >= w[1]) {
                return Err(format!(
                    "teleop.{name}.thresholds は昇順である必要があります"
                ));
            }
        }
        Ok(())
    }
}

/// 動作モードの要求。
///
/// 並び順が**活動度の低い順**になっていることに意味がある。受信が切れた
/// ときのフェイルセーフは、この順序で**直前より上へ行かない**ことを保証する
/// （[`Self::capped_for_failsafe`]）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ModeRequest {
    /// 脱力。
    Relax,
    /// 初期姿勢で保持。
    Stand,
    /// 歩行。
    Walk,
}

impl ModeRequest {
    /// 受信が切れたときに落とし込む先。**活動度を上げない。**
    ///
    /// | 直前 | 受信断後 | 理由 |
    /// |---|---|---|
    /// | `Relax` | `Relax` | **脱力中に受信が切れて立ち上がるのは危ない** |
    /// | `Stand` | `Stand` | 初期姿勢のまま保持 |
    /// | `Walk` | `Walk` | **速度だけ 0 にして、その場で立ったまま**保持 |
    ///
    /// つまり**モードは変えない**。速度をゼロにするのは
    /// [`OperatorCommand::failsafe`] の側。
    ///
    /// `Walk` を `Stand` へ丸めてはいけない。CH5 中段は「初期姿勢で保持」
    /// なので、丸めると**歩行中に受信が切れた瞬間に初期姿勢へしゃがみ込む**。
    /// 求めているのは「速度 0・その場起立」。丸めていた時期があるが、
    /// それは `Stand` が歩容の立ち姿勢を意味していた頃の名残。
    ///
    /// **脱力へ落とすのも禁止。** 荷重がかかった四足を脱力させると崩れる。
    pub fn capped_for_failsafe(self) -> Self {
        self
    }
}

/// 歩容の選択。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GaitSelect {
    Crawl,
    Walk,
    Trot,
}

impl GaitSelect {
    pub fn label(self) -> &'static str {
        match self {
            GaitSelect::Crawl => "Crawl",
            GaitSelect::Walk => "Walk",
            GaitSelect::Trot => "Trot",
        }
    }
}

/// 1 周期ぶんの操縦指令。速度はすでに実単位へスケール済み。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OperatorCommand {
    pub vx_m_s: f64,
    pub vy_m_s: f64,
    pub wz_rad_s: f64,
    /// 立ち姿勢高さのオフセット (m)。
    pub height_offset_m: f64,
    /// プロポから読んだ腕の角度 (rad, モデル座標系)。受信機直結の腕を
    /// 観測しているときだけ `Some`。**指令ではなく観測値。**
    pub arm_rad: Option<f64>,
    pub mode: ModeRequest,
    pub gait: GaitSelect,
    /// ポーズ再生スイッチの**立ち上がり**。押し続けても 1 回しか立たない。
    pub play_pose: bool,
    pub chicken_head: bool,
    /// ポーズ再生で `greeting_alt` を選ぶか（CH10）。
    pub play_alt: bool,
    /// 胴体姿勢の指令 `[roll, pitch]` (rad)。`chicken_head` が false なら `[0, 0]`。
    ///
    /// CH8 が ON のとき、CH1 をロール、CH3 をピッチに読み替える。
    /// **同時に横移動と高さは 0 になる** — 同じスティックを 2 つの意味で
    /// 使えないため。
    pub body_attitude_rad: [f64; 3],
    /// 受信が生きているか。false のときの速度は必ず 0 になっている。
    pub link_ok: bool,
}

impl OperatorCommand {
    /// 受信断・フェイルセーフのときの指令。
    ///
    /// **速度は 0、モードは Stand。** 脱力にしないのは、立っている四足を
    /// 脱力させると倒れるから。受信が戻るまでその場で立ち続けるのが、
    /// 電波が切れたときにいちばん壊れない。
    /// 受信が切れたときの指令。速度はゼロ、モードは `mode`。
    ///
    /// **`mode` は呼び出し側が [`ModeRequest::capped_for_failsafe`] で
    /// 丸めた値を渡すこと。** ここで一律 `Stand` を返していた時期があり、
    /// 脱力中に受信が切れると立ち上がっていた。
    pub fn failsafe(gait: GaitSelect, mode: ModeRequest) -> Self {
        Self {
            vx_m_s: 0.0,
            vy_m_s: 0.0,
            wz_rad_s: 0.0,
            height_offset_m: 0.0,
            // 受信が切れている間の腕の角度は分からない。直近値を握り続けると
            // 「今そこにある」と読めてしまうので、素直に不明にする。
            arm_rad: None,
            mode,
            gait,
            play_pose: false,
            play_alt: false,
            chicken_head: false,
            body_attitude_rad: [0.0; 3],
            link_ok: false,
        }
    }
}

fn default_pose_select() -> SwitchMap {
    SwitchMap::two_position(10)
}

/// プロポ入力の解釈器。スイッチの立ち上がり検出のため状態を持つ。
pub struct Teleop {
    cfg: TeleopConfig,
    max_vx: f64,
    max_vy: f64,
    max_wz: f64,
    height_range: f64,
    /// 胴体姿勢の上限 (rad)。**0 なら CH8 を入れても傾かない**（機能無効）。
    attitude_max: f64,
    /// 腕の可動域。観測した軸値 (-1..1) をこの範囲へ写す。
    arm_range_rad: (f64, f64),
    prev_pose_on: bool,
    last_gait: GaitSelect,
    /// 直近に**受信できていたとき**のモード要求。
    /// 受信断のフェイルセーフをここから頭打ちにする。初期値は最も安全な
    /// `Relax`（一度も受信できていないなら立ち上がる理由がない）。
    last_mode: ModeRequest,
}

impl Teleop {
    pub fn new(
        cfg: TeleopConfig,
        gait: &crate::config::GaitTuning,
        arm: &namiashi_hal::config::ArmConfig,
    ) -> Self {
        Self {
            cfg,
            max_vx: gait.max_vx_m_s,
            max_vy: gait.max_vy_m_s,
            max_wz: gait.max_wz_rad_s,
            height_range: gait.height_range_m,
            attitude_max: gait.body_attitude_max_rad,
            arm_range_rad: (arm.min_rad, arm.max_rad),
            prev_pose_on: false,
            last_gait: GaitSelect::Crawl,
            last_mode: ModeRequest::Relax,
        }
    }

    /// 受信が無い状態で使う指令（`--allow-no-sbus`）。
    ///
    /// **フェイルセーフとは別物。** フェイルセーフは「直前より上へ行かない」
    /// が原則なので、受信が一度も来ていなければ `Relax` のままになる。
    /// ベンチで受信機なしに起立させたいという要求はそれとは別で、
    /// **明示的に起立を合成する**。
    pub fn bench_stand(&self) -> OperatorCommand {
        OperatorCommand::failsafe(self.last_gait, ModeRequest::Stand)
    }

    /// 観測した腕チャンネル (-1..1) を可動域へ写す。
    fn arm_angle(&self, state: &SbusState) -> Option<f64> {
        let axis = self.cfg.arm.as_ref()?;
        let v = axis.value(state);
        let (min, max) = self.arm_range_rad;
        Some(min + (v + 1.0) * 0.5 * (max - min))
    }

    /// 1 周期ぶんの解釈。`usable` が false なら [`OperatorCommand::failsafe`]。
    pub fn update(&mut self, state: &SbusState, usable: bool) -> OperatorCommand {
        if !usable {
            // 受信が戻ったときにスイッチが押しっぱなしでも暴発しないよう、
            // 立ち上がり検出の履歴は「押されている」側に倒しておく。
            self.prev_pose_on = true;
            return OperatorCommand::failsafe(self.last_gait, self.last_mode.capped_for_failsafe());
        }

        let gait = match self.cfg.gait.position(state) {
            0 => GaitSelect::Crawl,
            1 => GaitSelect::Walk,
            _ => GaitSelect::Trot,
        };
        self.last_gait = gait;

        let mode = match self.cfg.mode.position(state) {
            0 => ModeRequest::Relax,
            1 => ModeRequest::Stand,
            _ => ModeRequest::Walk,
        };
        self.last_mode = mode;

        let pose_on = self.cfg.pose.position(state) > 0;
        let play_pose = pose_on && !self.prev_pose_on;
        self.prev_pose_on = pose_on;

        // **CH8 が ON の間、CH1 と CH3 は胴体姿勢に化ける。**
        // 同じスティックを 2 つの意味で使うので、横移動と高さは 0 にする
        // （両方効かせると「傾けながら横に流れる」になって操縦できない）。
        let chicken_head = self.cfg.chicken_head.position(state) > 0;
        let (vy, height, wz, attitude) = if chicken_head {
            // **合成量で頭打ちにする（円形の制限）。**
            //
            // 軸ごとに上限を掛けると、斜めに振り切ったとき √2 倍まで傾く。
            // 実測では roll 単独 0.65 rad まで可動域内なのに、roll と pitch を
            // 同時に 0.50 入れると 917 件逸脱した。スティックは円形に動くので、
            // 制限も円形にするのが素直。
            // roll / pitch は合成量で頭打ち（スティックは円形に動く）。
            // **yaw は別のスティックなので独立に頭打ちにする。**
            let r = self.cfg.vy.value(state) * self.attitude_max;
            let p = self.cfg.height.value(state) * self.attitude_max;
            let n = (r * r + p * p).sqrt();
            let k = if n > self.attitude_max && n > 0.0 {
                self.attitude_max / n
            } else {
                1.0
            };
            let y = self.cfg.wz.value(state) * self.attitude_max;
            (0.0, 0.0, 0.0, [r * k, p * k, y])
        } else {
            (
                self.cfg.vy.value(state) * self.max_vy,
                self.cfg.height.value(state) * self.height_range,
                self.cfg.wz.value(state) * self.max_wz,
                [0.0; 3],
            )
        };

        OperatorCommand {
            vx_m_s: self.cfg.vx.value(state) * self.max_vx,
            vy_m_s: vy,
            wz_rad_s: wz,
            height_offset_m: height,
            arm_rad: self.arm_angle(state),
            mode,
            gait,
            play_pose,
            play_alt: self.cfg.pose_select.position(state) > 0,
            chicken_head,
            body_attitude_rad: attitude,
            link_ok: true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::GaitTuning;

    fn arm_cfg() -> namiashi_hal::config::ArmConfig {
        namiashi_hal::config::HardwareConfig::default().arm
    }

    fn state_with(channels: &[(usize, u16)]) -> SbusState {
        // 「フレームを 1 枚受けて、全チャンネル中央」を起点に書き換える。
        let mut s = SbusState {
            channels: [RAW_CENTER; CHANNELS],
            counters: sbus_counters(1),
            ..SbusState::default()
        };
        for &(ch, raw) in channels {
            s.channels[ch - 1] = raw;
        }
        s
    }

    fn sbus_counters(frames: u64) -> namiashi_hal::sbus::Counters {
        namiashi_hal::sbus::Counters {
            frames,
            ..Default::default()
        }
    }

    #[test]
    fn a_centred_stick_is_exactly_zero() {
        let axis = AxisMap::new(1, false);
        assert_eq!(axis.value(&state_with(&[])), 0.0);
    }

    #[test]
    fn endpoints_map_to_plus_minus_one() {
        let axis = AxisMap {
            deadband: 0.0,
            ..AxisMap::new(1, false)
        };
        assert!((axis.value(&state_with(&[(1, RAW_MAX)])) - 1.0).abs() < 1e-9);
        assert!((axis.value(&state_with(&[(1, RAW_MIN)])) + 1.0).abs() < 1e-9);
    }

    #[test]
    fn reverse_flips_the_sign() {
        let fwd = AxisMap {
            deadband: 0.0,
            ..AxisMap::new(1, false)
        };
        let rev = AxisMap {
            deadband: 0.0,
            ..AxisMap::new(1, true)
        };
        let s = state_with(&[(1, 1500)]);
        assert!((fwd.value(&s) + rev.value(&s)).abs() < 1e-12);
    }

    #[test]
    fn the_deadband_swallows_small_offsets_but_not_the_endpoint() {
        let axis = AxisMap {
            deadband: 0.2,
            ..AxisMap::new(1, false)
        };
        // 中央 +5% は不感帯の中。
        let small = RAW_CENTER + ((RAW_MAX - RAW_CENTER) as f64 * 0.05) as u16;
        assert_eq!(axis.value(&state_with(&[(1, small)])), 0.0);
        // 端は 1.0 のまま（不感帯で頭打ちになってはいけない）。
        assert!((axis.value(&state_with(&[(1, RAW_MAX)])) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn an_out_of_range_channel_reads_as_zero_not_as_a_panic() {
        let axis = AxisMap::new(99, false);
        assert_eq!(axis.value(&state_with(&[])), 0.0);
        let axis0 = AxisMap::new(0, false);
        assert_eq!(axis0.value(&state_with(&[])), 0.0);
    }

    #[test]
    fn a_three_position_switch_reports_zero_one_two() {
        let sw = SwitchMap::three_position(5);
        assert_eq!(sw.position(&state_with(&[(5, RAW_MIN)])), 0);
        assert_eq!(sw.position(&state_with(&[(5, RAW_CENTER)])), 1);
        assert_eq!(sw.position(&state_with(&[(5, RAW_MAX)])), 2);
        assert_eq!(sw.positions(), 3);
    }

    #[test]
    fn the_pose_trigger_fires_once_per_press() {
        let mut t = Teleop::new(TeleopConfig::default(), &GaitTuning::default(), &arm_cfg());
        let off = state_with(&[(7, RAW_MIN)]);
        let on = state_with(&[(7, RAW_MAX)]);
        assert!(!t.update(&off, true).play_pose);
        assert!(t.update(&on, true).play_pose);
        // 押しっぱなしでは 2 回目は立たない。
        assert!(!t.update(&on, true).play_pose);
        assert!(!t.update(&off, true).play_pose);
        assert!(t.update(&on, true).play_pose);
    }

    /// 受信断では速度を必ずゼロにする。**スティックの生値を残さない。**
    ///
    /// モードは直前の要求から頭打ちにするので、ここでは見ない
    /// （[`a_lost_link_never_escalates_the_mode`] 他が受け持つ）。
    /// **このテストはかつて `mode == Stand` を固定していた** — つまり
    /// 「脱力中に受信が切れたら立ち上がる」というバグの方を守っていた。
    #[test]
    fn a_lost_link_zeroes_the_velocity() {
        let mut t = Teleop::new(TeleopConfig::default(), &GaitTuning::default(), &arm_cfg());
        let cmd = t.update(&state_with(&[(2, RAW_MAX), (5, RAW_MAX)]), false);
        assert_eq!(cmd.vx_m_s, 0.0);
        assert_eq!(cmd.vy_m_s, 0.0);
        assert_eq!(cmd.wz_rad_s, 0.0);
        assert!(!cmd.link_ok);
    }

    #[test]
    fn a_press_held_across_a_link_loss_does_not_fire_on_recovery() {
        let mut t = Teleop::new(TeleopConfig::default(), &GaitTuning::default(), &arm_cfg());
        let on = state_with(&[(7, RAW_MAX)]);
        // 受信断のあいだにスイッチが入ったまま復帰しても暴発しない。
        t.update(&on, false);
        assert!(!t.update(&on, true).play_pose);
    }

    /// 受信断のフェイルセーフは**活動度を上げない**。
    ///
    /// かつて一律 `Stand` を返していたため、**脱力中に受信が切れると
    /// 立ち上がっていた**（実機で確認、2026-08-22）。
    /// **斜めに振り切っても合成量が上限を超えない。**
    ///
    /// 軸ごとに掛けると √2 倍になり、実測でも roll+pitch 同時 0.50 で
    /// 可動域を 917 件逸脱した（roll 単独なら 0.65 まで入る）。
    /// **振る足の選択は専用スイッチ (CH10)。** 姿勢モード (CH8) とは独立。
    ///
    /// 当初は空きチャンネルが無いと思って CH8 を修飾キーにしていたが、
    /// 実際は CH7 が腕で CH9/CH10 が空いていた。設定の
    /// `[teleop.arm] channel` が 9 のままで実配線と食い違っていた。
    #[test]
    fn the_wave_leg_is_selected_by_its_own_switch() {
        let mut t = Teleop::new(TeleopConfig::default(), &GaitTuning::default(), &arm_cfg());
        // CH10 下段 → 既定のポーズ。
        let cmd = t.update(&state_with(&[(10, RAW_MIN)]), true);
        assert!(!cmd.play_alt);
        // CH10 上段 → もう一方。
        let cmd = t.update(&state_with(&[(10, RAW_MAX)]), true);
        assert!(cmd.play_alt);
        // **姿勢モード (CH8) とは独立。** CH8 を入れても選択は変わらない。
        let cmd = t.update(&state_with(&[(10, RAW_MIN), (8, RAW_MAX)]), true);
        assert!(!cmd.play_alt, "CH8 が振る足の選択に影響している");
    }

    #[test]
    fn the_body_attitude_is_clamped_by_its_magnitude() {
        let mut gait = GaitTuning::default();
        gait.body_attitude_max_rad = 0.6;
        let mut t = Teleop::new(TeleopConfig::default(), &gait, &arm_cfg());
        // CH8 ON、CH1 と CH3 を両方振り切る。
        let cmd = t.update(
            &state_with(&[(8, RAW_MAX), (1, RAW_MAX), (3, RAW_MAX)]),
            true,
        );
        let [r, p, _y] = cmd.body_attitude_rad;
        let n = (r * r + p * p).sqrt();
        assert!(n <= 0.6 + 1e-9, "合成量 {n:.4} が上限 0.6 を超えた");
        assert!(
            n > 0.5,
            "合成量 {n:.4} が小さすぎる（斜めで効かなくなっている）"
        );
        // 単軸なら上限いっぱいまで入る。
        let cmd = t.update(&state_with(&[(8, RAW_MAX), (1, RAW_MAX)]), true);
        assert!(
            cmd.body_attitude_rad[0].abs() > 0.55,
            "単軸で上限まで入らない: {:?}",
            cmd.body_attitude_rad
        );
        // CH8 が OFF なら傾かず、横移動が戻る。
        // **CH8 を明示的に下げる。** 2 段スイッチの閾値は中央なので、
        // `state_with` の既定（全チャンネル中央）では ON と読まれる。
        let cmd = t.update(&state_with(&[(8, RAW_MIN), (1, RAW_MAX)]), true);
        assert_eq!(cmd.body_attitude_rad, [0.0; 3]);
        assert!(cmd.vy_m_s.abs() > 0.0);
    }

    #[test]
    fn a_lost_link_never_escalates_the_mode() {
        assert_eq!(ModeRequest::Relax.capped_for_failsafe(), ModeRequest::Relax);
        assert_eq!(ModeRequest::Stand.capped_for_failsafe(), ModeRequest::Stand);
        // 歩行中に切れたら**その場で立ったまま**保持。速度だけ 0 になる。
        // `Stand` へ丸めると初期姿勢へしゃがみ込んでしまう。
        assert_eq!(ModeRequest::Walk.capped_for_failsafe(), ModeRequest::Walk);
    }

    #[test]
    fn a_lost_link_holds_relax_when_the_operator_had_asked_for_relax() {
        let mut t = Teleop::new(TeleopConfig::default(), &GaitTuning::default(), &arm_cfg());
        // 脱力を指示してから受信断。
        let cmd = t.update(&state_with(&[(5, RAW_MIN)]), true);
        assert_eq!(cmd.mode, ModeRequest::Relax);
        let lost = t.update(&state_with(&[(5, RAW_MIN)]), false);
        assert_eq!(
            lost.mode,
            ModeRequest::Relax,
            "脱力中の受信断で立ち上がった"
        );
        assert_eq!(lost.vx_m_s, 0.0);
    }

    #[test]
    fn a_lost_link_stops_a_walk_without_relaxing() {
        let mut t = Teleop::new(TeleopConfig::default(), &GaitTuning::default(), &arm_cfg());
        let cmd = t.update(&state_with(&[(5, RAW_MAX), (2, RAW_MAX)]), true);
        assert_eq!(cmd.mode, ModeRequest::Walk);
        assert!(cmd.vx_m_s > 0.0);
        let lost = t.update(&state_with(&[(5, RAW_MAX), (2, RAW_MAX)]), false);
        // モードは歩行のまま = 立ち姿勢を保つ。速度だけ 0。
        assert_eq!(lost.mode, ModeRequest::Walk);
        assert_eq!(lost.vx_m_s, 0.0);
    }

    /// 一度も受信できていなければ `Relax` のまま。**起立させる理由がない。**
    #[test]
    fn a_link_that_never_worked_stays_relaxed() {
        let mut t = Teleop::new(TeleopConfig::default(), &GaitTuning::default(), &arm_cfg());
        let lost = t.update(&state_with(&[(5, RAW_MAX)]), false);
        assert_eq!(lost.mode, ModeRequest::Relax);
        // ベンチ用の合成指令だけが明示的に起立する。
        assert_eq!(t.bench_stand().mode, ModeRequest::Stand);
    }

    #[test]
    fn mode_and_gait_switches_decode_to_the_documented_order() {
        let mut t = Teleop::new(TeleopConfig::default(), &GaitTuning::default(), &arm_cfg());
        let cmd = t.update(&state_with(&[(5, RAW_MIN), (6, RAW_MIN)]), true);
        assert_eq!(cmd.mode, ModeRequest::Relax);
        assert_eq!(cmd.gait, GaitSelect::Crawl);
        let cmd = t.update(&state_with(&[(5, RAW_CENTER), (6, RAW_CENTER)]), true);
        assert_eq!(cmd.mode, ModeRequest::Stand);
        assert_eq!(cmd.gait, GaitSelect::Walk);
        let cmd = t.update(&state_with(&[(5, RAW_MAX), (6, RAW_MAX)]), true);
        assert_eq!(cmd.mode, ModeRequest::Walk);
        assert_eq!(cmd.gait, GaitSelect::Trot);
    }

    #[test]
    fn velocities_are_scaled_to_the_configured_limits() {
        let tuning = GaitTuning::default();
        let mut t = Teleop::new(TeleopConfig::default(), &tuning, &arm_cfg());
        let cmd = t.update(&state_with(&[(2, RAW_MAX)]), true);
        assert!((cmd.vx_m_s - tuning.max_vx_m_s).abs() < 1e-9);
    }

    #[test]
    fn default_teleop_config_is_valid() {
        TeleopConfig::default().validate().unwrap();
    }

    #[test]
    fn an_out_of_range_channel_is_rejected_by_validate() {
        let mut cfg = TeleopConfig::default();
        cfg.vx.channel = 0;
        assert!(cfg.validate().is_err());
        let mut cfg = TeleopConfig::default();
        cfg.mode.channel = 99;
        assert!(cfg.validate().is_err());
    }
}
