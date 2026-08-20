//! 13 軸ぶんの関節ベクトル（脚 12 + 腕 1）。
//!
//! ゲイト・ポーズ再生・チキンヘッドはどれも「関節角の集合」を作って渡すだけ
//! なので、その入れ物を 1 個に決めてしまう。並びは
//! [`namiashi_hal::joint::JOINT_NAMES`]（FL, FR, RL, RR × hip, thigh, calf）
//! の後ろに腕を足したもの。

use namiashi_hal::joint::{LegSlot, ARM_JOINT_NAME, JOINT_NAMES};

/// 脚 12 軸 + 腕 1 軸。
pub const DOF: usize = 13;

/// 関節角ベクトル (rad, モデル座標系)。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointVec {
    /// `[leg][hip, thigh, calf]`。
    pub legs: [[f64; 3]; 4],
    pub arm: f64,
}

impl Default for JointVec {
    fn default() -> Self {
        Self::zeros()
    }
}

impl JointVec {
    pub const fn zeros() -> Self {
        Self {
            legs: [[0.0; 3]; 4],
            arm: 0.0,
        }
    }

    /// 関節名で引く。未知の名前には `None`。
    // 実機ループでは使わないが、名前引きは [`Self::set`] と対で意味を持つ
    // （試験と将来のログ出力が使う）。
    #[allow(dead_code)]
    pub fn get(&self, joint_name: &str) -> Option<f64> {
        if joint_name == ARM_JOINT_NAME {
            return Some(self.arm);
        }
        namiashi_hal::joint::lookup(joint_name).map(|(leg, k)| self.legs[leg.index()][k])
    }

    /// 関節名で書く。未知の名前なら `false` を返して何もしない。
    pub fn set(&mut self, joint_name: &str, value: f64) -> bool {
        if joint_name == ARM_JOINT_NAME {
            self.arm = value;
            return true;
        }
        match namiashi_hal::joint::lookup(joint_name) {
            Some((leg, k)) => {
                self.legs[leg.index()][k] = value;
                true
            }
            None => false,
        }
    }

    /// 補間に渡すための平坦なベクトル。
    pub fn to_vec(self) -> Vec<f64> {
        let mut v = Vec::with_capacity(DOF);
        for leg in LegSlot::ALL {
            v.extend_from_slice(&self.legs[leg.index()]);
        }
        v.push(self.arm);
        v
    }

    /// [`Self::to_vec`] の逆。長さが足りなければ残りは 0 のまま。
    pub fn from_slice(v: &[f64]) -> Self {
        let mut out = Self::zeros();
        for (i, value) in v.iter().take(DOF).enumerate() {
            if i == DOF - 1 {
                out.arm = *value;
            } else {
                out.legs[i / 3][i % 3] = *value;
            }
        }
        out
    }

    /// `(関節名, 角度)` の列。ログや `.misa` への書き戻しに使う。
    pub fn iter_named(&self) -> impl Iterator<Item = (&'static str, f64)> + '_ {
        LegSlot::ALL
            .into_iter()
            .flat_map(move |leg| {
                (0..3).map(move |k| (JOINT_NAMES[leg.index()][k], self.legs[leg.index()][k]))
            })
            .chain(std::iter::once((ARM_JOINT_NAME, self.arm)))
    }

    /// 2 つの姿勢の最大関節角差 (rad)。「もう着いたか」の判定に使う。
    #[allow(dead_code)]
    pub fn max_abs_diff(&self, other: &JointVec) -> f64 {
        self.to_vec()
            .iter()
            .zip(other.to_vec().iter())
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> JointVec {
        let mut v = JointVec::zeros();
        for (i, (name, _)) in v.iter_named().collect::<Vec<_>>().into_iter().enumerate() {
            let _ = name;
            let _ = i;
        }
        for leg in LegSlot::ALL {
            for k in 0..3 {
                v.legs[leg.index()][k] = (leg.index() * 3 + k) as f64 * 0.1;
            }
        }
        v.arm = -0.5;
        v
    }

    #[test]
    fn flat_vector_round_trips() {
        let v = sample();
        assert_eq!(JointVec::from_slice(&v.to_vec()), v);
    }

    #[test]
    fn flat_vector_has_one_slot_per_dof() {
        assert_eq!(sample().to_vec().len(), DOF);
        assert_eq!(sample().iter_named().count(), DOF);
    }

    #[test]
    fn named_access_matches_the_array_layout() {
        let v = sample();
        assert_eq!(v.get("FL_hip_joint"), Some(v.legs[0][0]));
        assert_eq!(v.get("RR_calf_joint"), Some(v.legs[3][2]));
        assert_eq!(v.get(ARM_JOINT_NAME), Some(v.arm));
        assert_eq!(v.get("no_such_joint"), None);
    }

    #[test]
    fn setting_an_unknown_joint_reports_failure() {
        let mut v = JointVec::zeros();
        assert!(v.set("FL_thigh_joint", 1.0));
        assert!(!v.set("wheel_joint", 1.0));
        assert_eq!(v.legs[0][1], 1.0);
    }

    #[test]
    fn max_abs_diff_finds_the_worst_joint() {
        let a = JointVec::zeros();
        let mut b = JointVec::zeros();
        b.legs[2][1] = -0.75;
        b.arm = 0.25;
        assert!((a.max_abs_diff(&b) - 0.75).abs() < 1e-12);
    }
}
