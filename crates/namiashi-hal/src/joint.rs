//! 関節の並び順とコマンド／状態の値型。
//!
//! 並び順は **`quadruped_gait::LegId::ALL` と同じ FL, FR, RL, RR**、脚内は
//! hip, thigh, calf。基板の LEG1..4 コネクタもこの順に割り当てる（`config` で
//! 変更可）。ここを 1 か所に固定しておかないと、モデル・ゲイト・実機配線の
//! 3 つの並びが独立にずれて、症状が「片脚だけ挙動がおかしい」になる。

/// 脚のスロット番号。`quadruped_gait::LegId` と同じ並び。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(usize)]
pub enum LegSlot {
    Fl = 0,
    Fr = 1,
    Rl = 2,
    Rr = 3,
}

impl LegSlot {
    pub const ALL: [LegSlot; 4] = [LegSlot::Fl, LegSlot::Fr, LegSlot::Rl, LegSlot::Rr];

    pub const fn index(self) -> usize {
        self as usize
    }

    /// モデル／URDF の接頭辞（`FL_hip_joint` の `FL`）。
    pub const fn prefix(self) -> &'static str {
        match self {
            LegSlot::Fl => "FL",
            LegSlot::Fr => "FR",
            LegSlot::Rl => "RL",
            LegSlot::Rr => "RR",
        }
    }

    pub fn from_prefix(s: &str) -> Option<LegSlot> {
        match s {
            "FL" => Some(LegSlot::Fl),
            "FR" => Some(LegSlot::Fr),
            "RL" => Some(LegSlot::Rl),
            "RR" => Some(LegSlot::Rr),
            _ => None,
        }
    }
}

/// 脚内の関節種別。RS485 のモータ id 1,2,3 に対応する既定の並び。
pub const LEG_JOINT_KINDS: [&str; 3] = ["hip", "thigh", "calf"];

/// 12 個の脚関節名を `[leg][kind]` で引ける表。
pub const JOINT_NAMES: [[&str; 3]; 4] = [
    ["FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"],
    ["FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"],
    ["RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"],
    ["RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"],
];

/// 腕（RC サーボ）の関節名。
pub const ARM_JOINT_NAME: &str = "arm_pitch_joint";

/// 関節名 → (脚スロット, 脚内 index)。腕や未知の名前には `None`。
pub fn lookup(joint_name: &str) -> Option<(LegSlot, usize)> {
    for leg in LegSlot::ALL {
        for (k, name) in JOINT_NAMES[leg.index()].iter().enumerate() {
            if *name == joint_name {
                return Some((leg, k));
            }
        }
    }
    None
}

/// 1 関節に対する指令。
///
/// 位置と（将来の）トルクを両方持つのは、モードを跨いだ切り替えで前回値が
/// 消えないようにするため。実際にどちらが使われるかは [`JointMode`] が決める。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointCommand {
    pub mode: JointMode,
    /// モデル座標系の関節角 (rad)。符号・オフセットの実機変換は HAL 側で行う。
    pub position_rad: f64,
    /// 位置制御時の速度上限 (rad/s、出力軸)。
    pub max_speed_rad_s: f64,
    /// トルク指令 (N·m、出力軸)。`JointMode::Torque` のときだけ使う。
    pub torque_nm: f64,
}

impl Default for JointCommand {
    fn default() -> Self {
        Self {
            mode: JointMode::Idle,
            position_rad: 0.0,
            max_speed_rad_s: 0.0,
            torque_nm: 0.0,
        }
    }
}

/// 1 関節の制御モード。
///
/// `Idle` はモータへ何も送らず状態だけ読む。初期化直後と非常停止後の既定値で、
/// 「まだ誰も指令を書いていないスロット」がいきなり 0 rad へ動くことがないよう
/// にするための状態でもある。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointMode {
    Idle,
    Position,
    Torque,
}

/// 1 関節の実測値。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointState {
    /// モデル座標系の関節角 (rad)。
    pub position_rad: f64,
    pub velocity_rad_s: f64,
    pub torque_nm: f64,
    pub temperature_c: f64,
    /// 直近のトランザクションが成功したか。false の値は古い可能性がある。
    pub ok: bool,
}

impl Default for JointState {
    fn default() -> Self {
        Self {
            position_rad: 0.0,
            velocity_rad_s: 0.0,
            torque_nm: 0.0,
            temperature_c: 0.0,
            ok: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn joint_names_follow_the_legid_order() {
        // quadruped_gait::LegId::ALL と同じ並びであることを名前で固定する。
        assert_eq!(JOINT_NAMES[LegSlot::Fl.index()][0], "FL_hip_joint");
        assert_eq!(JOINT_NAMES[LegSlot::Rr.index()][2], "RR_calf_joint");
    }

    #[test]
    fn lookup_round_trips_every_leg_joint() {
        for leg in LegSlot::ALL {
            for (k, name) in JOINT_NAMES[leg.index()].iter().enumerate() {
                assert_eq!(lookup(name), Some((leg, k)), "{name}");
            }
        }
        assert_eq!(lookup(ARM_JOINT_NAME), None);
    }

    #[test]
    fn prefix_round_trips() {
        for leg in LegSlot::ALL {
            assert_eq!(LegSlot::from_prefix(leg.prefix()), Some(leg));
        }
    }
}
