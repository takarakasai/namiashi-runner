//! ハードウェア設定（TOML）。
//!
//! 実機の配線・モータ id・符号・可動域はコードではなくここに置く。ゼロ点や
//! 符号は組み立て直すたびに変わるもので、変わるたびに再ビルドが必要な形に
//! しておくと、現場で必ず「とりあえずコードを直す」が起きるため。
//!
//! 既定値は `config/namiashi.toml` に書き出したものと同じで、設定ファイルを
//! 与えなくても [`HardwareConfig::default`] だけで起動できる。

use serde::{Deserialize, Serialize};

use crate::ch348::{self, PortMap};
use crate::error::{Error, Result};
use crate::joint::{LegSlot, LEG_JOINT_KINDS};

/// シリアルポートの指定方法。
///
/// `uart` は CH348 の**物理** UART 番号（基板のコネクタ番号）で、`/dev` 名の
/// 採番順には依存しない。`path` は探索を飛ばして直接開く逃げ道。
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PortSpec {
    Uart(u16),
    Path(String),
}

impl PortSpec {
    /// 実際に開くデバイスパスへ解決する（必要ならその場で 1 回探索する）。
    ///
    /// 複数ポートを続けて開くときは [`Self::resolve_with`] を使うこと。
    /// 探索はデバイスを `open` するので、自分が開いた直後のポートは
    /// `EBUSY` で調べられなくなる。
    pub fn resolve(&self) -> Result<String> {
        match self {
            PortSpec::Uart(index) => Ok(ch348::find_by_uart_index(*index)?
                .to_string_lossy()
                .into_owned()),
            PortSpec::Path(p) => Ok(p.clone()),
        }
    }

    /// 事前に取った探索結果を使って解決する。
    pub fn resolve_with(&self, map: &PortMap) -> Result<String> {
        match self {
            PortSpec::Uart(index) => Ok(map.path_for(*index)?.to_string_lossy().into_owned()),
            PortSpec::Path(p) => Ok(p.clone()),
        }
    }

    /// ログ・エラー表示用のラベル（解決前でも名乗れるもの）。
    pub fn label(&self) -> String {
        match self {
            PortSpec::Uart(i) => format!("UART{i}"),
            PortSpec::Path(p) => p.clone(),
        }
    }
}

/// 1 個のモータの実機パラメータ。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MotorConfig {
    /// 脚内の種別（`hip` / `thigh` / `calf`）。並びの取り違え検出にだけ使う。
    pub kind: String,
    /// RS485 のモータ id。
    pub id: u8,
    /// モデル座標系 → モータ出力軸の符号。`-1.0` で反転。
    #[serde(default = "one")]
    pub sign: f64,
    /// **ゼロ出しを行った姿勢**の、モデル座標系での関節角 (rad)。
    ///
    /// LKMTech V3 の位置制御は `rezero` で置いたソフトゼロからの相対量なので、
    /// 「どの姿勢でゼロ出ししたか」を書いておかないとモデル角と対応が付かない。
    ///
    /// ```text
    /// q_motor = sign * (q_model - zero_pose_rad)
    /// q_model = sign *  q_motor + zero_pose_rad        (sign = ±1)
    /// ```
    ///
    /// 治具姿勢でゼロ出しするなら、その姿勢の関節角をここに書く。
    #[serde(default)]
    pub zero_pose_rad: f64,
    /// モデル座標系での可動域 (rad)。指令はここへクランプされる。
    pub min_rad: f64,
    pub max_rad: f64,
    /// この軸だけの減速比。`None` なら [`LegsConfig::gear_ratio`] を使う。
    ///
    /// **calf だけベルト駆動で、プーリの歯数比 28:18 が MG4005 内蔵の 10:1 に
    /// 上乗せされる**（総減速比 15.5556）。脚のなかで一番負荷がかかる軸なので、
    /// 設計上そうなっている。ここを `LegsConfig::gear_ratio` の 10.0 のまま
    /// にすると calf の角度が 55.6% ずれる。
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gear_ratio: Option<f64>,
}

impl MotorConfig {
    /// この軸の減速比。軸個別の指定が無ければバス共通の値。
    pub fn gear_ratio_or(&self, bus_default: f64) -> f64 {
        self.gear_ratio.unwrap_or(bus_default)
    }
}

fn one() -> f64 {
    1.0
}

/// 1 本の RS485 脚バス。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LegBusConfig {
    /// この基板ポートに繋がっている脚（`FL` / `FR` / `RL` / `RR`）。
    pub leg: String,
    pub port: PortSpec,
    /// hip, thigh, calf の順に 3 個。
    pub motors: Vec<MotorConfig>,
}

impl LegBusConfig {
    pub fn leg_slot(&self) -> Result<LegSlot> {
        LegSlot::from_prefix(&self.leg).ok_or_else(|| {
            Error::Config(format!("脚の名前 {:?} が不正です (FL/FR/RL/RR)", self.leg))
        })
    }
}

/// 脚 12 軸まとめての設定。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LegsConfig {
    /// RS485 ボーレート。モータ側の設定と一致していること。
    #[serde(default = "default_leg_baud")]
    pub baud: u32,
    /// 1 トランザクションの応答待ち上限 (ms)。
    ///
    /// 短すぎると取りこぼし、長すぎると 1 台の無応答がバス全体の周期を
    /// 引きずる。3 台直列なので、周期目標の 1/3 より小さく取る。
    ///
    /// **実効的な下限は約 20 ms。** `lkmotor_driver::Rs485Driver` はシリアルの
    /// read タイムアウトを固定 20 ms (`READ_POLL_TIMEOUT`) で開き、締切の
    /// 判定は read から戻ったあとに行う。したがってここに 5 ms と書いても、
    /// 無応答のモータ 1 台につき約 20 ms 待つ（実測: 3 台無応答で 1 周
    /// 61 ms → 16 Hz）。生きているモータしかいなければ効かない話だが、
    /// 「1 台落ちたときにバスが何 Hz まで落ちるか」はこれで決まる。
    #[serde(default = "default_response_timeout_ms")]
    pub response_timeout_ms: u64,
    /// バススレッドの目標周期 (Hz)。実際に出た周期は `BusStats` で読める。
    #[serde(default = "default_bus_rate_hz")]
    pub bus_rate_hz: f64,
    /// 減速比（モータ軸回転数 / 出力軸回転数）。
    #[serde(default = "default_gear_ratio")]
    pub gear_ratio: f64,
    /// トルク定数 (N·m/A、モータ軸)。`None` なら電流 (A) をそのまま
    /// トルク API に流す `MotorConfig::current_units` 相当になる。
    #[serde(default)]
    pub torque_constant_nm_per_a: Option<f64>,
    /// 位置指令の既定速度上限 (rad/s、出力軸)。モータ側が守る「軸の速さ」。
    #[serde(default = "default_max_speed")]
    pub default_max_speed_rad_s: f64,
    /// **目標角そのもの**の変化率の上限 (rad/s)。0 で無制限。
    ///
    /// `default_max_speed_rad_s` とは別物。あちらは「軸が何 rad/s で回るか」で、
    /// こちらは「目標が何 rad/s で動くか」。歩容の切り替えや IK のクランプで
    /// 目標が跳んだとき、モータ側の速度上限だけだと上限速度で追いに行って
    /// しまう。目標側を鈍らせておけば、跳びがそのまま脚の飛び出しにならない。
    #[serde(default = "default_max_target_rate")]
    pub max_target_rate_rad_s: f64,
    /// State1（電圧・温度・異常ビット）を読む間隔 (ms)。0 で読まない。
    ///
    /// 1 回につき 1 軸だけ読むので、全 3 軸が更新されるのはこの 3 倍の周期。
    #[serde(default = "default_status_interval_ms")]
    pub status_interval_ms: u64,
    pub bus: Vec<LegBusConfig>,
}

fn default_leg_baud() -> u32 {
    1_000_000
}
fn default_response_timeout_ms() -> u64 {
    5
}
fn default_bus_rate_hz() -> f64 {
    500.0
}
fn default_gear_ratio() -> f64 {
    10.0
}

/// calf のベルト駆動プーリの歯数 `(従動, 駆動)`。
///
/// 総減速比は [`default_gear_ratio`] にこの比を掛けたもの = 15.5556。
/// **歯数のまま持つ。** 1.55 と丸めると 0.36% ずれ、calf の端で約 0.5° になる。
pub const CALF_PULLEY_TEETH: (f64, f64) = (28.0, 18.0);

/// hip の可動域の広いほう (rad) = 60°。左足は −側、右足は + 側。
///
/// **メカ端ではなく CAD から決めた運用上の制限。** hip のメカ端は非常に広く、
/// そこまで振るとケーブルが破断しうる。機械が止めてくれない以上、ここで
/// 止めるしかない。
pub const HIP_WIDE_RAD: f64 = 1.0471975511965976;

/// hip の可動域の狭いほう (rad) = 45°。
pub const HIP_NARROW_RAD: f64 = std::f64::consts::FRAC_PI_4;

/// thigh の可動域 (rad)。±145°、4 軸とも共通。実機で確認済み。
pub const THIGH_LIMIT_RAD: f64 = 2.5307274153917776;

/// calf の可動域 (rad)。±154.52338°、2026-08-21 の設計変更後の値。
///
/// thigh の ±2.62 rad (±150.11°) とは違うので、まとめて扱わないこと。
pub const CALF_LIMIT_RAD: f64 = 2.6969417523103556;

fn default_max_speed() -> f64 {
    8.0
}
fn default_max_target_rate() -> f64 {
    // 立ち姿勢の膝は 1.7 rad ほど。3 rad/s なら端から端まで 0.6 s 程度で、
    // 歩容の遊脚（1 周期 0.5 s 級）には十分速く、跳びは吸収できる。
    3.0
}
fn default_status_interval_ms() -> u64 {
    1000
}

/// IMU（CH348 UART5、WitMotion）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ImuConfig {
    pub port: PortSpec,
    #[serde(default = "default_imu_baud")]
    pub baud: u32,
    #[serde(default = "default_imu_timeout_ms")]
    pub response_timeout_ms: u64,
    /// センサ取付姿勢の補正 (rad)。`[roll, pitch, yaw]` をこの順に引く。
    #[serde(default)]
    pub mount_offset_rad: [f64; 3],
}

fn default_imu_baud() -> u32 {
    // IWT603 実測値（wit-imu/doc/communication_spec.md §8）。
    921_600
}
fn default_imu_timeout_ms() -> u64 {
    50
}

/// S.BUS 受信（CH348 UART6、受信専用）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SbusConfig {
    pub port: PortSpec,
    /// フレームが途絶えたとみなすまでの時間 (ms)。S.BUS は 14 ms 周期なので
    /// 数フレーム分。ここを過ぎたらフェイルセーフ扱いにする。
    #[serde(default = "default_sbus_stale_ms")]
    pub stale_after_ms: u64,
}

fn default_sbus_stale_ms() -> u64 {
    100
}

/// 腕（RC サーボ 1 個）。CH348 の ARMA/ARMB を TTL に切り替えて使う。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ArmConfig {
    /// 使うサーボのプロトコル。未配線なら `none`。
    #[serde(default)]
    pub protocol: ArmProtocol,
    pub port: PortSpec,
    #[serde(default = "default_arm_baud")]
    pub baud: u32,
    #[serde(default)]
    pub id: u8,
    #[serde(default = "one")]
    pub sign: f64,
    /// ゼロ出し姿勢のモデル関節角 (rad)。[`MotorConfig::zero_pose_rad`] と同じ意味。
    #[serde(default)]
    pub zero_pose_rad: f64,
    /// モデル座標系での可動域 (rad)。namiashi.urdf の arm_pitch_joint 準拠。
    #[serde(default = "default_arm_min")]
    pub min_rad: f64,
    #[serde(default = "default_arm_max")]
    pub max_rad: f64,
}

fn default_arm_baud() -> u32 {
    115_200
}
fn default_arm_min() -> f64 {
    -2.3
}
fn default_arm_max() -> f64 {
    0.85
}

/// 腕サーボのプロトコル。
///
/// 最終的には「ARMA/ARMB の TTL UART 経由のシリアルサーボ」だが品種が未定で、
/// **初期検討では受信機に直結**する。品種が決まったらここに variant を足して
/// [`crate::arm::ArmServo`] の実装を差し込む。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ArmProtocol {
    /// **受信機直結。** サーボはプロポのチャンネルで直接動き、アプリは駆動
    /// しない。`teleop.arm` を設定すれば同じチャンネルから角度を観測できる。
    #[default]
    ReceiverDirect,
    /// 未配線 / 未実装。指令は捨てられ、状態は「最後に指令した値」を返す。
    None,
}

/// 実機まとめ。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HardwareConfig {
    pub legs: LegsConfig,
    pub imu: ImuConfig,
    pub sbus: SbusConfig,
    pub arm: ArmConfig,
}

impl HardwareConfig {
    /// TOML 文字列から読む。
    pub fn from_toml(text: &str) -> Result<Self> {
        let cfg: HardwareConfig =
            toml::from_str(text).map_err(|e| Error::Config(format!("TOML の解析に失敗: {e}")))?;
        cfg.validate()?;
        Ok(cfg)
    }

    pub fn load(path: impl AsRef<std::path::Path>) -> Result<Self> {
        let path = path.as_ref();
        let text = std::fs::read_to_string(path)
            .map_err(|e| Error::Config(format!("{} を読めません: {e}", path.display())))?;
        Self::from_toml(&text)
    }

    pub fn to_toml(&self) -> Result<String> {
        toml::to_string_pretty(self).map_err(|e| Error::Config(format!("TOML の生成に失敗: {e}")))
    }

    /// 実機を触る前に潰せる誤りを潰す。
    ///
    /// 順番の取り違え（hip/thigh/calf の入れ替え、脚の重複）と可動域の逆転は、
    /// 実機で気づくと壊れる種類の誤りなので、必ずここで落とす。
    pub fn validate(&self) -> Result<()> {
        if self.legs.bus.len() != 4 {
            return Err(Error::Config(format!(
                "脚バスは 4 本必要ですが {} 本しかありません",
                self.legs.bus.len()
            )));
        }
        let mut seen = [false; 4];
        for bus in &self.legs.bus {
            let slot = bus.leg_slot()?;
            if std::mem::replace(&mut seen[slot.index()], true) {
                return Err(Error::Config(format!(
                    "脚 {} が 2 回定義されています",
                    bus.leg
                )));
            }
            if bus.motors.len() != 3 {
                return Err(Error::Config(format!(
                    "脚 {} のモータは 3 個必要ですが {} 個です",
                    bus.leg,
                    bus.motors.len()
                )));
            }
            for (k, motor) in bus.motors.iter().enumerate() {
                if motor.kind != LEG_JOINT_KINDS[k] {
                    return Err(Error::Config(format!(
                        "脚 {} の {} 番目は {:?} であるべきですが {:?} です（並び順は hip, thigh, calf 固定）",
                        bus.leg, k, LEG_JOINT_KINDS[k], motor.kind
                    )));
                }
                if motor.id == 0 {
                    return Err(Error::Config(format!(
                        "脚 {} の {} の id が 0 です（LKMTech V3 の id は 1 始まり）",
                        bus.leg, motor.kind
                    )));
                }
                if motor.min_rad >= motor.max_rad
                    || !motor.min_rad.is_finite()
                    || !motor.max_rad.is_finite()
                {
                    return Err(Error::Config(format!(
                        "脚 {} の {} の可動域が不正です (min {} >= max {})",
                        bus.leg, motor.kind, motor.min_rad, motor.max_rad
                    )));
                }
                if motor.sign != 1.0 && motor.sign != -1.0 {
                    return Err(Error::Config(format!(
                        "脚 {} の {} の sign は ±1.0 のみです ({} が指定されました)",
                        bus.leg, motor.kind, motor.sign
                    )));
                }
            }
            // 同一バス上の id 重複は応答を取り違える。
            let ids: Vec<u8> = bus.motors.iter().map(|m| m.id).collect();
            for (i, id) in ids.iter().enumerate() {
                if ids[..i].contains(id) {
                    return Err(Error::Config(format!(
                        "脚 {} でモータ id {id} が重複しています（同一バス上では区別できません）",
                        bus.leg
                    )));
                }
            }
        }
        if self.legs.bus_rate_hz <= 0.0 {
            return Err(Error::Config("legs.bus_rate_hz は正の値が必要です".into()));
        }
        if self.arm.min_rad >= self.arm.max_rad
            || !self.arm.min_rad.is_finite()
            || !self.arm.max_rad.is_finite()
        {
            return Err(Error::Config("arm の可動域が不正です".into()));
        }
        Ok(())
    }

    /// 脚スロットに対応するバス設定。
    pub fn bus_for(&self, leg: LegSlot) -> Option<&LegBusConfig> {
        self.legs
            .bus
            .iter()
            .find(|b| b.leg_slot().ok() == Some(leg))
    }
}

impl Default for HardwareConfig {
    fn default() -> Self {
        // 既定の配線: 基板の LEG1..4 (UART0..3) = **FL, RL, FR, RR**、
        // バス内は id 1, 2, 3 = hip, thigh, calf。
        // 可動域は namiashi.urdf の <limit> をそのまま写している。
        //
        // **`LegSlot::ALL`（FL, FR, RL, RR）とは順序が違う。** J14..J17 が
        // 左列 → 右列に並ぶ基板なので、左 2 本を先に配線するのが自然な取り回し
        // になる。`ALL` の側は `quadruped_gait::LegId::ALL` と一致させる必要が
        // あり `JointVec` の添字にもなっているので触れない。
        // したがって UART 割り当ては**位置ではなく明示表**で持つ。
        let uart_for = |leg: LegSlot| -> u16 {
            match leg {
                LegSlot::Fl => ch348::uart::LEGS[0],
                LegSlot::Rl => ch348::uart::LEGS[1],
                LegSlot::Fr => ch348::uart::LEGS[2],
                LegSlot::Rr => ch348::uart::LEGS[3],
            }
        };
        let leg_limits = |leg: LegSlot| -> [(f64, f64); 3] {
            // hip は **CAD から読んだ値**で、URDF の <limit> ではない。
            //
            // メカ端は非常に広く、そこまで振るとケーブルが破断しうる。機械が
            // 止めてくれないので、ソフト側で先に止める必要がある。
            //
            // **URDF は左右が逆だった**（大きさ 45/60 は合っていたが割り当てが
            // 入れ替わっていた）。左足が -60..+45、右足が -45..+60。
            let hip = match leg {
                LegSlot::Fl | LegSlot::Rl => (-HIP_WIDE_RAD, HIP_NARROW_RAD),
                LegSlot::Fr | LegSlot::Rr => (-HIP_NARROW_RAD, HIP_WIDE_RAD),
            };
            // calf だけ可動域が違う。2026-08-21 の設計変更で ±154.52338°。
            [
                hip,
                (-THIGH_LIMIT_RAD, THIGH_LIMIT_RAD),
                (-CALF_LIMIT_RAD, CALF_LIMIT_RAD),
            ]
        };
        // 符号は設計データ由来で、実機で確認済み（2026-08-21）。
        //
        //   Roll  (hip)          → **前後**で反転。後ろ 2 本（RL/RR）が -1
        //   Pitch (thigh/calf)   → **左右**で反転。右 2 本（FR/RR）が -1
        //
        // 軸によって反転の軸が違うので、左右対称だろうと決めてかかると外す。
        let leg_signs = |leg: LegSlot| -> [f64; 3] {
            let hip = match leg {
                LegSlot::Fl | LegSlot::Fr => 1.0,
                LegSlot::Rl | LegSlot::Rr => -1.0,
            };
            let pitch = match leg {
                LegSlot::Fl | LegSlot::Rl => 1.0,
                LegSlot::Fr | LegSlot::Rr => -1.0,
            };
            [hip, pitch, pitch]
        };
        // calf だけベルト駆動。プーリの歯数比が内蔵の 10:1 に上乗せされる。
        //
        // **歯数から直接計算する。** 28/18 = 1.5555... で、1.55 と丸めると
        // 0.36% ずれる（calf の端で約 0.5°）。歯数は整数で誤差が無いので、
        // 丸めた小数ではなく比のまま持つ。
        let leg_gear = |k: usize| -> Option<f64> {
            (LEG_JOINT_KINDS[k] == "calf")
                .then_some(default_gear_ratio() * CALF_PULLEY_TEETH.0 / CALF_PULLEY_TEETH.1)
        };
        let bus: Vec<LegBusConfig> = LegSlot::ALL
            .iter()
            .map(|&leg| {
                let limits = leg_limits(leg);
                let signs = leg_signs(leg);
                LegBusConfig {
                    leg: leg.prefix().to_string(),
                    port: PortSpec::Uart(uart_for(leg)),
                    motors: (0..3)
                        .map(|k| MotorConfig {
                            kind: LEG_JOINT_KINDS[k].to_string(),
                            id: (k + 1) as u8,
                            sign: signs[k],
                            zero_pose_rad: 0.0,
                            min_rad: limits[k].0,
                            max_rad: limits[k].1,
                            gear_ratio: leg_gear(k),
                        })
                        .collect(),
                }
            })
            .collect();

        Self {
            legs: LegsConfig {
                baud: default_leg_baud(),
                response_timeout_ms: default_response_timeout_ms(),
                bus_rate_hz: default_bus_rate_hz(),
                gear_ratio: default_gear_ratio(),
                torque_constant_nm_per_a: None,
                default_max_speed_rad_s: default_max_speed(),
                max_target_rate_rad_s: default_max_target_rate(),
                status_interval_ms: default_status_interval_ms(),
                bus,
            },
            imu: ImuConfig {
                port: PortSpec::Uart(ch348::uart::IMU),
                baud: default_imu_baud(),
                response_timeout_ms: default_imu_timeout_ms(),
                mount_offset_rad: [0.0; 3],
            },
            sbus: SbusConfig {
                port: PortSpec::Uart(ch348::uart::SBUS),
                stale_after_ms: default_sbus_stale_ms(),
            },
            arm: ArmConfig {
                protocol: ArmProtocol::default(),
                port: PortSpec::Uart(ch348::uart::ARM_A),
                baud: default_arm_baud(),
                id: 1,
                sign: 1.0,
                zero_pose_rad: 0.0,
                min_rad: default_arm_min(),
                max_rad: default_arm_max(),
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config_is_valid() {
        HardwareConfig::default().validate().unwrap();
    }

    #[test]
    fn default_config_round_trips_through_toml() {
        let cfg = HardwareConfig::default();
        let text = cfg.to_toml().unwrap();
        let back = HardwareConfig::from_toml(&text).unwrap();
        assert_eq!(cfg, back);
    }

    /// 既定の配線は UART0..3 = FL, RL, FR, RR。
    ///
    /// **`LegSlot::ALL`（FL, FR, RL, RR）の順序とは違う。** 位置で振ると
    /// FR と RL が入れ替わるので、ここで実機の配線を固定しておく
    /// （`doc/motor_map.md` の as-built 表と一致すること）。
    /// 符号は設計データ由来で実機確認済み（2026-08-21）。
    ///
    /// **Roll(hip) は前後で、Pitch(thigh/calf) は左右で反転する。** 反転の軸が
    /// 違うので「左右対称だろう」と決めてかかると外す。ここで固定しておく。
    #[test]
    fn default_signs_flip_roll_front_rear_and_pitch_left_right() {
        let cfg = HardwareConfig::default();
        let expected = [
            (LegSlot::Fl, [1.0, 1.0, 1.0]),
            (LegSlot::Rl, [-1.0, 1.0, 1.0]),
            (LegSlot::Fr, [1.0, -1.0, -1.0]),
            (LegSlot::Rr, [-1.0, -1.0, -1.0]),
        ];
        for (leg, signs) in expected {
            let bus = cfg.bus_for(leg).unwrap();
            let got: Vec<f64> = bus.motors.iter().map(|m| m.sign).collect();
            assert_eq!(got, signs.to_vec(), "{} の符号", leg.prefix());
        }
    }

    /// calf だけベルト駆動でプーリの歯数比 28:18 が内蔵の 10:1 に上乗せされる。
    ///
    /// バス共通の `gear_ratio` のままだと calf が 47% ずれるので、
    /// 軸個別に持てていることと値の両方を固定する。
    #[test]
    fn only_calf_overrides_the_gear_ratio() {
        let cfg = HardwareConfig::default();
        let bus_default = cfg.legs.gear_ratio;
        for leg in LegSlot::ALL {
            let bus = cfg.bus_for(leg).unwrap();
            for m in &bus.motors {
                let expected = if m.kind == "calf" {
                    bus_default * CALF_PULLEY_TEETH.0 / CALF_PULLEY_TEETH.1
                } else {
                    bus_default
                };
                assert!(
                    (m.gear_ratio_or(bus_default) - expected).abs() < 1e-9,
                    "{} の {} は減速比 {expected}",
                    leg.prefix(),
                    m.kind
                );
            }
            // calf 以外は個別指定を持たない（共通値に従う）。
            for m in bus.motors.iter().filter(|m| m.kind != "calf") {
                assert_eq!(m.gear_ratio, None, "{} の {}", leg.prefix(), m.kind);
            }
        }
    }

    #[test]
    fn default_wiring_is_uart0_to_3_equals_fl_rl_fr_rr() {
        let cfg = HardwareConfig::default();
        let expected = [
            (LegSlot::Fl, 0u16),
            (LegSlot::Rl, 1),
            (LegSlot::Fr, 2),
            (LegSlot::Rr, 3),
        ];
        for (leg, uart) in expected {
            let bus = cfg.bus_for(leg).unwrap();
            assert_eq!(
                bus.port,
                PortSpec::Uart(uart),
                "{} は UART{uart} のはず",
                leg.prefix()
            );
            assert_eq!(
                bus.motors.iter().map(|m| m.id).collect::<Vec<_>>(),
                vec![1, 2, 3]
            );
        }
    }

    #[test]
    fn swapped_joint_order_is_rejected() {
        let mut cfg = HardwareConfig::default();
        cfg.legs.bus[0].motors.swap(0, 1);
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn duplicate_motor_id_on_one_bus_is_rejected() {
        let mut cfg = HardwareConfig::default();
        cfg.legs.bus[0].motors[1].id = 1;
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn duplicate_leg_is_rejected() {
        let mut cfg = HardwareConfig::default();
        cfg.legs.bus[1].leg = "FL".into();
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn inverted_joint_limits_are_rejected() {
        let mut cfg = HardwareConfig::default();
        cfg.legs.bus[0].motors[0].min_rad = 1.0;
        cfg.legs.bus[0].motors[0].max_rad = -1.0;
        assert!(cfg.validate().is_err());
    }
}
