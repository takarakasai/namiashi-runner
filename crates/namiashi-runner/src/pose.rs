//! `.misa` のポーズ / シーケンス再生。
//!
//! `.misa` は `[[pose]]`（名前つき関節角）と `[[sequence]]`（ポーズを繋いだ
//! 手順）を最初から持っている（`misarta::config::{PoseConfig, SequenceConfig}`）。
//! 挨拶の動作はここに置けば、articara の GUI で作って保存したものをそのまま
//! 実機で再生できる — アプリ側にポーズを書き込む必要はない。
//!
//! ポーズは**部分指定**である点に注意。`.misa` の仕様どおり、書かれていない
//! 関節は「再生開始時の値を保つ」。腕だけ動かすポーズが脚を勝手に 0 へ
//! 引っぱらないのはこのため。

use std::collections::BTreeMap;

use misarta::config::{PoseConfig, SequenceConfig};
use misarta::trajectory::{InterpolationKind, PoseTransition};

use crate::jointvec::JointVec;

/// `.misa` から読んだポーズとシーケンス。
#[derive(Debug, Clone, Default)]
pub struct PoseLibrary {
    poses: Vec<PoseConfig>,
    sequences: Vec<SequenceConfig>,
    home: JointVec,
}

impl PoseLibrary {
    pub fn new(poses: Vec<PoseConfig>, sequences: Vec<SequenceConfig>, home: JointVec) -> Self {
        Self {
            poses,
            sequences,
            home,
        }
    }

    /// `.misa` のパース結果から作る。
    pub fn from_misa(file: &misarta::native::MisaFile) -> Self {
        let mut home = JointVec::zeros();
        for (name, value) in &file.home.joint_positions {
            home.set(name, *value);
        }
        Self::new(file.pose.clone(), file.sequence.clone(), home)
    }

    pub fn pose_names(&self) -> impl Iterator<Item = &str> {
        self.poses.iter().map(|p| p.name.as_str())
    }

    pub fn sequence_names(&self) -> impl Iterator<Item = &str> {
        self.sequences.iter().map(|s| s.name.as_str())
    }

    pub fn pose(&self, name: &str) -> Option<&PoseConfig> {
        self.poses.iter().find(|p| p.name == name)
    }

    pub fn sequence(&self, name: &str) -> Option<&SequenceConfig> {
        self.sequences.iter().find(|s| s.name == name)
    }

    /// `[home]` の関節角。
    pub fn home(&self) -> JointVec {
        self.home
    }

    /// ポーズを現在姿勢に重ねて完全な関節ベクトルにする。
    pub fn resolve(&self, angles: &BTreeMap<String, f64>, current: JointVec) -> JointVec {
        let mut out = current;
        for (name, value) in angles {
            // モデルにあってこの機体に無い関節（未実装の軸など）は黙って無視。
            out.set(name, *value);
        }
        out
    }

    /// 名前を 1 つ受けて再生手順に落とす。ポーズ名でもシーケンス名でもよい。
    pub fn plan(&self, name: &str, from: JointVec) -> Result<Vec<PoseStep>, String> {
        if let Some(seq) = self.sequence(name) {
            let mut steps = Vec::with_capacity(seq.steps.len());
            let mut cursor = from;
            for step in &seq.steps {
                let pose = self.pose(&step.pose_name).ok_or_else(|| {
                    format!(
                        "シーケンス {:?} が参照するポーズ {:?} がモデルにありません",
                        name, step.pose_name
                    )
                })?;
                let target = self.resolve(&pose.angles, cursor);
                steps.push(PoseStep {
                    name: step.pose_name.clone(),
                    target,
                    duration_s: step.duration.max(0.0),
                    kind: step.kind,
                });
                cursor = target;
            }
            if steps.is_empty() {
                return Err(format!("シーケンス {name:?} に手順がありません"));
            }
            return Ok(steps);
        }
        if let Some(pose) = self.pose(name) {
            return Ok(vec![PoseStep {
                name: pose.name.clone(),
                target: self.resolve(&pose.angles, from),
                duration_s: pose.duration.max(0.0),
                kind: pose.kind,
            }]);
        }
        Err(format!(
            "ポーズ / シーケンス {name:?} がモデルにありません（pose: {:?}, sequence: {:?}）",
            self.pose_names().collect::<Vec<_>>(),
            self.sequence_names().collect::<Vec<_>>()
        ))
    }
}

/// 再生手順の 1 段。
#[derive(Debug, Clone, PartialEq)]
pub struct PoseStep {
    pub name: String,
    pub target: JointVec,
    pub duration_s: f64,
    pub kind: InterpolationKind,
}

/// ポーズ / シーケンスの再生器。制御周期ごとに [`Self::tick`] を呼ぶ。
#[derive(Debug, Clone)]
pub struct PosePlayer {
    steps: Vec<PoseStep>,
    index: usize,
    t: f64,
    transition: PoseTransition<f64>,
    current: JointVec,
}

impl PosePlayer {
    /// 現在姿勢 `from` から名前つきの動作を始める。
    pub fn start(lib: &PoseLibrary, name: &str, from: JointVec) -> Result<Self, String> {
        let steps = lib.plan(name, from)?;
        let transition = transition_for(from, &steps[0]);
        // `evaluate(0)` は始点そのものだが、長さ 0 の段では終点になる。
        // 生成直後に 1 度評価しておくと、`tick` を呼ぶ前に `current` を
        // 読んだ呼び出し側にも正しい値が見える。
        let current = JointVec::from_slice(&transition.evaluate(0.0));
        Ok(Self {
            steps,
            index: 0,
            t: 0.0,
            transition,
            current,
        })
    }

    /// 単発の目標姿勢へ遷移するだけの再生器（起立・初期姿勢など）。
    pub fn to_pose(from: JointVec, to: JointVec, duration_s: f64, kind: InterpolationKind) -> Self {
        let step = PoseStep {
            name: "<transition>".into(),
            target: to,
            duration_s: duration_s.max(0.0),
            kind,
        };
        let transition = transition_for(from, &step);
        let current = JointVec::from_slice(&transition.evaluate(0.0));
        Self {
            steps: vec![step],
            index: 0,
            t: 0.0,
            transition,
            current,
        }
    }

    /// `dt` 進めて、その時点の目標関節角を返す。
    /// 終了後に呼び続けても安全（`evaluate` が終点でクランプする）。
    pub fn tick(&mut self, dt: f64) -> JointVec {
        self.t += dt;
        self.current = JointVec::from_slice(&self.transition.evaluate(self.t));

        // 段が終わったら次へ。dt が段の長さより長い場合に 1 周期で
        // 複数段を飛ばせるようループにしてある。
        while self.transition.is_done(self.t) && self.index + 1 < self.steps.len() {
            self.t -= self.transition.duration;
            self.index += 1;
            self.transition = transition_for(self.current, &self.steps[self.index]);
            self.current = JointVec::from_slice(&self.transition.evaluate(self.t));
        }
        self.current
    }

    pub fn is_done(&self) -> bool {
        self.index + 1 >= self.steps.len() && self.transition.is_done(self.t)
    }

    /// 現在の目標姿勢。
    #[allow(dead_code)]
    pub fn current(&self) -> JointVec {
        self.current
    }

    /// 再生中の段の名前（表示用）。
    pub fn step_name(&self) -> &str {
        &self.steps[self.index].name
    }
}

fn transition_for(from: JointVec, step: &PoseStep) -> PoseTransition<f64> {
    PoseTransition::new(
        from.to_vec(),
        step.target.to_vec(),
        step.duration_s,
        step.kind,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use misarta::config::SequenceStepConfig;

    fn pose(name: &str, angles: &[(&str, f64)], duration: f64) -> PoseConfig {
        PoseConfig {
            name: name.into(),
            angles: angles.iter().map(|(k, v)| (k.to_string(), *v)).collect(),
            duration,
            kind: InterpolationKind::QuinticSmooth,
        }
    }

    fn library() -> PoseLibrary {
        let poses = vec![
            pose(
                "crouch",
                &[("FL_thigh_joint", 1.3), ("FL_calf_joint", -2.6)],
                1.0,
            ),
            pose("wave", &[("arm_pitch_joint", -1.0)], 0.5),
        ];
        let seq = SequenceConfig {
            name: "greeting".into(),
            steps: vec![
                SequenceStepConfig {
                    pose_name: "wave".into(),
                    duration: 0.5,
                    kind: InterpolationKind::Linear,
                },
                SequenceStepConfig {
                    pose_name: "crouch".into(),
                    duration: 0.5,
                    kind: InterpolationKind::Linear,
                },
            ],
        };
        PoseLibrary::new(poses, vec![seq], JointVec::zeros())
    }

    #[test]
    fn an_unlisted_joint_keeps_its_current_value() {
        let lib = library();
        let mut current = JointVec::zeros();
        current.legs[1][0] = 0.42; // FR_hip — "crouch" は触れていない
        let steps = lib.plan("crouch", current).unwrap();
        assert_eq!(steps[0].target.legs[1][0], 0.42);
        assert_eq!(steps[0].target.legs[0][1], 1.3);
    }

    #[test]
    fn a_pose_reaches_its_target_exactly() {
        let lib = library();
        let mut player = PosePlayer::start(&lib, "crouch", JointVec::zeros()).unwrap();
        for _ in 0..300 {
            player.tick(0.005);
        }
        assert!(player.is_done());
        assert!((player.current().legs[0][1] - 1.3).abs() < 1e-9);
        assert!((player.current().legs[0][2] + 2.6).abs() < 1e-9);
    }

    #[test]
    fn a_sequence_walks_every_step_in_order() {
        let lib = library();
        let mut player = PosePlayer::start(&lib, "greeting", JointVec::zeros()).unwrap();
        assert_eq!(player.step_name(), "wave");
        // 1 段目 (0.5 s) の途中では腕だけが動いている。
        for _ in 0..50 {
            player.tick(0.005);
        }
        assert!(player.current().arm < 0.0);
        assert_eq!(player.current().legs[0][1], 0.0);
        // 最後まで回すと 2 段目の脚の目標に到達する。
        for _ in 0..300 {
            player.tick(0.005);
        }
        assert!(player.is_done());
        assert!((player.current().legs[0][1] - 1.3).abs() < 1e-9);
        // 1 段目で動かした腕は 2 段目でも保持される（部分指定の合成）。
        assert!((player.current().arm + 1.0).abs() < 1e-9);
    }

    #[test]
    fn a_long_dt_does_not_skip_a_step_silently() {
        let lib = library();
        let mut player = PosePlayer::start(&lib, "greeting", JointVec::zeros()).unwrap();
        // 1 周期で全段（合計 1.0 s）を跨いでも最終姿勢に着く。
        player.tick(2.0);
        assert!(player.is_done());
        assert!((player.current().legs[0][1] - 1.3).abs() < 1e-9);
        assert!((player.current().arm + 1.0).abs() < 1e-9);
    }

    #[test]
    fn an_unknown_name_is_an_error_listing_what_exists() {
        let lib = library();
        let err = PosePlayer::start(&lib, "nope", JointVec::zeros()).unwrap_err();
        assert!(err.contains("crouch"), "{err}");
        assert!(err.contains("greeting"), "{err}");
    }

    #[test]
    fn a_sequence_referencing_a_missing_pose_fails_before_moving() {
        let lib = PoseLibrary::new(
            vec![],
            vec![SequenceConfig {
                name: "broken".into(),
                steps: vec![SequenceStepConfig {
                    pose_name: "ghost".into(),
                    duration: 1.0,
                    kind: InterpolationKind::Linear,
                }],
            }],
            JointVec::zeros(),
        );
        assert!(PosePlayer::start(&lib, "broken", JointVec::zeros()).is_err());
    }

    #[test]
    fn a_plain_transition_interpolates_between_two_vectors() {
        let mut from = JointVec::zeros();
        from.legs[0][0] = -1.0;
        let mut to = JointVec::zeros();
        to.legs[0][0] = 1.0;
        let mut player = PosePlayer::to_pose(from, to, 1.0, InterpolationKind::Linear);
        player.tick(0.5);
        assert!(player.current().legs[0][0].abs() < 1e-9);
        player.tick(0.5);
        assert!(player.is_done());
        assert!((player.current().legs[0][0] - 1.0).abs() < 1e-9);
    }

    #[test]
    fn a_zero_duration_step_completes_immediately() {
        let mut to = JointVec::zeros();
        to.arm = 0.5;
        let mut player = PosePlayer::to_pose(JointVec::zeros(), to, 0.0, InterpolationKind::Linear);
        player.tick(0.005);
        assert!(player.is_done());
        assert!((player.current().arm - 0.5).abs() < 1e-9);
    }
}
