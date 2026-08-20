//! ライブ可視化: 各周期の姿勢を Zenoh で流し、articara の GUI に描かせる。
//!
//! 受け側は articara の **Live gait feed**（`articara --features viz` の
//! `viz_feed::VizFeedState`）で、`quadruped_gait::viz::GaitVizFrame` を JSON で
//! 待っている。既定のキーは [`quadruped_gait::viz::VIZ_KEY_PLANNED`]。
//! articara 側で namiashi のモデルを開いておけば、フレームの 12 関節が
//! 名前（`FL_hip_joint` …）で該当関節に入る。
//!
//! # 送るのはモデル座標系の角度
//!
//! `GaitVizFrame::from_output` は**歩容 / IK の符号**のまま詰めるので、
//! そのまま送ると膝が反転して描かれる（向こうの doc コメントが警告している）。
//! ここは実機へ送るのと同じモデル座標系の [`JointVec`] からフレームを組む。
//! つまり**画面に出る姿勢は、モータへ行く指令そのもの**であって、歩容の
//! 生出力ではない。遷移中もポーズ再生中も描けるのはこのため。
//!
//! # 腕は映らない
//!
//! `GaitVizFrame` は脚 12 関節ぶんしか運ばない（Go2 向けの器なので）。
//! `arm_pitch_joint` は articara 側で動かないままになる。

use quadruped_gait::viz::{GaitVizFrame, VIZ_FORMAT_VERSION};

use crate::jointvec::JointVec;

/// `--viz` 系オプション。
#[derive(Debug, Clone)]
pub struct VizConfig {
    pub enabled: bool,
    /// Zenoh のキー。articara 側の入力欄と一致させる。
    pub key: String,
    /// 送信レート (Hz)。制御周期より低くてよい（描画は 60 fps も要らない）。
    pub rate_hz: f64,
    /// Zenoh の接続先エンドポイント（`tcp/127.0.0.1:7447` など）。
    /// マルチキャスト探索が使えないホスト（同一ホスト / WSL2）で指定する。
    pub endpoint: Option<String>,
}

impl Default for VizConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            key: quadruped_gait::viz::VIZ_KEY_PLANNED.to_string(),
            rate_hz: 50.0,
            endpoint: None,
        }
    }
}

/// 制御ループから可視化フレームを組み立てるのに要る、歩容側の情報。
#[derive(Debug, Clone, Copy, Default)]
pub struct BodyView {
    /// 胴体の世界位置 `[x, y]` (m)。
    pub xy: [f64; 2],
    /// 胴体のヨー角 (rad)。
    pub yaw: f64,
    /// 胴体高さ (m)。
    pub z: f64,
    /// 接地フラグ（FL, FR, RL, RR）。
    pub stance: [bool; 4],
}

/// 目標角と胴体姿勢から可視化フレームを作る。
pub fn frame(seq: u64, t_s: f64, targets: &JointVec, body: &BodyView) -> GaitVizFrame {
    let mut joints = [0.0f64; 12];
    for (slot, leg) in targets.legs.iter().enumerate() {
        joints[3 * slot..3 * slot + 3].copy_from_slice(leg);
    }
    GaitVizFrame {
        version: VIZ_FORMAT_VERSION,
        seq,
        t_s,
        pose: [body.xy[0], body.xy[1], body.z, body.yaw],
        joints,
        stance: body.stance,
    }
}

pub use publisher::Publisher;

/// `viz` フィーチャ無しビルドの空実装。
///
/// 呼び出し側を `#[cfg]` だらけにしないために、型と署名だけ同じものを置く。
/// `new` は必ずエラーを返すので、**配信しているつもりで何も出ていない**という
/// 状態にはならない。
#[cfg(not(feature = "viz"))]
mod publisher {
    use quadruped_gait::viz::GaitVizFrame;

    use super::VizConfig;

    pub struct Publisher(());

    impl Publisher {
        pub fn new(_cfg: &VizConfig) -> Result<Self, String> {
            Err("--viz は `viz` フィーチャ付きでビルドしたときだけ使えます".into())
        }

        pub fn maybe_publish(&mut self, _build: impl FnOnce(u64) -> GaitVizFrame) {}
    }
}

#[cfg(feature = "viz")]
mod publisher {
    use std::time::{Duration, Instant};

    use quadruped_gait::viz::GaitVizFrame;
    use zenoh::Wait;

    use super::VizConfig;

    /// Zenoh へフレームを流す。
    ///
    /// **`Session` を持ち続けること。** `declare_publisher` の戻り値だけを
    /// 保持して Session を落とすとセッションごと閉じ、`put` は静かに失敗して
    /// 「配信しているつもりで何も出ていない」になる（実際に踏んだ）。
    /// `go2-gait-runner` と同じく Session を抱えて `session.put` を使う。
    ///
    /// **送信の失敗は握り潰す。** 可視化が制御ループを乱すことは絶対にあっては
    /// ならないので、購読者がいない・切れたといった事情でループが止まったり
    /// 遅れたりしないようにする。
    pub struct Publisher {
        session: zenoh::Session,
        key: String,
        period: Duration,
        next: Instant,
        seq: u64,
    }

    impl Publisher {
        pub fn new(cfg: &VizConfig) -> Result<Self, String> {
            let mut config = zenoh::Config::default();
            if let Some(ep) = &cfg.endpoint {
                config
                    .insert_json5("listen/endpoints", &format!("[\"{ep}\"]"))
                    .map_err(|e| format!("zenoh listen endpoint '{ep}': {e}"))?;
                let _ = config.insert_json5("scouting/multicast/enabled", "false");
            }
            let session = zenoh::open(config)
                .wait()
                .map_err(|e| format!("zenoh のセッションを開けません: {e}"))?;
            log::info!(
                "ライブ可視化: '{}' へ {:.0} Hz で配信します（articara の Live gait feed で購読）",
                cfg.key,
                cfg.rate_hz
            );
            Ok(Self {
                session,
                key: cfg.key.clone(),
                period: Duration::from_secs_f64(1.0 / cfg.rate_hz.max(1.0)),
                next: Instant::now(),
                seq: 0,
            })
        }

        /// 送信レートに達していれば送る。達していなければ何もしない。
        pub fn maybe_publish(&mut self, build: impl FnOnce(u64) -> GaitVizFrame) {
            let now = Instant::now();
            if now < self.next {
                return;
            }
            self.next = now + self.period;
            let frame = build(self.seq);
            self.seq += 1;
            match serde_json::to_vec(&frame) {
                Ok(bytes) => {
                    let _ = self
                        .session
                        .put(&self.key, bytes)
                        .encoding(zenoh::bytes::Encoding::APPLICATION_JSON)
                        .wait();
                }
                Err(e) => log::debug!("viz フレームを直列化できません: {e}"),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn joints_are_laid_out_slot_major_in_the_fl_fr_rl_rr_order() {
        let mut q = JointVec::zeros();
        for leg in 0..4 {
            for k in 0..3 {
                q.legs[leg][k] = (leg * 3 + k) as f64;
            }
        }
        let f = frame(7, 1.5, &q, &BodyView::default());
        assert_eq!(f.joints, [0., 1., 2., 3., 4., 5., 6., 7., 8., 9., 10., 11.]);
        assert_eq!(f.seq, 7);
        assert_eq!(f.t_s, 1.5);
        assert_eq!(f.version, VIZ_FORMAT_VERSION);
    }

    #[test]
    fn the_body_pose_is_x_y_z_yaw() {
        let body = BodyView {
            xy: [0.3, -0.1],
            yaw: 0.5,
            z: 0.20,
            stance: [true, false, true, false],
        };
        let f = frame(0, 0.0, &JointVec::zeros(), &body);
        assert_eq!(f.pose, [0.3, -0.1, 0.20, 0.5]);
        assert_eq!(f.stance, [true, false, true, false]);
    }
}
