//! ライブ可視化: 各周期の姿勢を Zenoh で流し、articara の GUI に描かせる。
//!
//! 受け側は articara の **Live gait feed**（`articara --features viz` の
//! `viz_feed::VizFeedState`）で、`quadruped_gait::viz::GaitVizFrame` を JSON で
//! 待っている。実装契約は quadruped-gait の `doc/viz_publisher.md` と
//! `quadruped-gait/src/viz.rs` のモジュールドキュメントが正典。
//! articara 側で namiashi のモデルを開いておけば、フレームの 12 関節が
//! 名前（`FL_hip_joint` …）で該当関節に入る。
//!
//! # 2 ストリーム
//!
//! キーを分けて **指令**と**実測**を別々に流す:
//!
//! - [`VIZ_KEY_PLANNED`](quadruped_gait::viz::VIZ_KEY_PLANNED) … コントローラが出した目標
//! - [`VIZ_KEY_MEASURED`](quadruped_gait::viz::VIZ_KEY_MEASURED) … 実機から読み戻した状態
//!
//! 受け側は measured が来ればそれでモデルを駆動し、planned を半透明ゴーストで
//! 重ねる。**1 本のキーに両方流してはいけない**（チャネルが latest-wins なので
//! 上書き合戦になり、指令と実測の間でガタつく）。片方だけでも成立するので、
//! 実機を持たない `dump` は planned のみ、指令を出さない `legs` は measured のみ。
//!
//! 対で送るときは**同一 tick・同一 `seq`**。受け側は 2 ストリームを独立に
//! サンプリングするので、これが両者のズレを 1 配信周期に抑える唯一の保証。
//!
//! # 送るのはモデル座標系の角度
//!
//! `GaitVizFrame::from_output` は**歩容 / IK の符号**のまま詰めるので、
//! そのまま送ると膝が反転して描かれる（向こうの doc コメントが警告している）。
//! ここは実機へ送るのと同じモデル座標系の [`JointVec`] からフレームを組む。
//! つまり**画面に出る姿勢は、モータへ行く指令そのもの**であって、歩容の
//! 生出力ではない。遷移中もポーズ再生中も描けるのはこのため。
//! 実測側も、エンコーダから読んだ値は既にモデル座標系なので符号補正は要らない。
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
    /// 指令ストリームの Zenoh キー。articara 側の入力欄と一致させる。
    pub key_planned: String,
    /// 実測ストリームの Zenoh キー。**planned と同じにしてはいけない。**
    pub key_measured: String,
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
            key_planned: quadruped_gait::viz::VIZ_KEY_PLANNED.to_string(),
            key_measured: quadruped_gait::viz::VIZ_KEY_MEASURED.to_string(),
            rate_hz: 100.0,
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
    /// 胴体高さ (m)。接地面から胴体まで。
    pub z: f64,
    /// 胴体の `[roll, pitch]` (rad)。planned は水平計画なので `[0, 0]`、
    /// measured は IMU の実測値をそのまま入れる。
    pub rp: [f64; 2],
    /// 接地フラグ（FL, FR, RL, RR）。
    pub stance: [bool; 4],
}

/// 関節角と胴体姿勢から可視化フレームを作る。
pub fn frame(seq: u64, t_s: f64, joints_rad: &JointVec, body: &BodyView) -> GaitVizFrame {
    let mut joints = [0.0f64; 12];
    for (slot, leg) in joints_rad.legs.iter().enumerate() {
        joints[3 * slot..3 * slot + 3].copy_from_slice(leg);
    }
    GaitVizFrame {
        version: VIZ_FORMAT_VERSION,
        seq,
        t_s,
        pose: [body.xy[0], body.xy[1], body.z, body.yaw],
        pose_rp: body.rp,
        joints,
        stance: body.stance,
    }
}

/// 1 tick ぶんの送信内容。planned と measured は**同じ `seq`** を持つ。
// `viz` 無しビルドでは Publisher が空実装なので中身が読まれない。
#[cfg_attr(not(feature = "viz"), allow(dead_code))]
#[derive(Debug, Clone, Default)]
pub struct Frames {
    pub planned: Option<GaitVizFrame>,
    pub measured: Option<GaitVizFrame>,
}

impl Frames {
    /// 指令のみ（実機を持たない `dump` など）。
    pub fn planned(f: GaitVizFrame) -> Self {
        Self {
            planned: Some(f),
            measured: None,
        }
    }

    /// 実測のみ（指令を出さない `legs` など）。
    pub fn measured(f: GaitVizFrame) -> Self {
        Self {
            planned: None,
            measured: Some(f),
        }
    }

    /// 指令と実測の対。
    pub fn both(planned: GaitVizFrame, measured: GaitVizFrame) -> Self {
        Self {
            planned: Some(planned),
            measured: Some(measured),
        }
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
    use super::{Frames, VizConfig};

    pub struct Publisher(());

    impl Publisher {
        pub fn new(_cfg: &VizConfig) -> Result<Self, String> {
            Err("--viz は `viz` フィーチャ付きでビルドしたときだけ使えます".into())
        }

        pub fn maybe_publish(&mut self, _build: impl FnOnce(u64) -> Frames) {}
    }
}

#[cfg(feature = "viz")]
mod publisher {
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::mpsc::{sync_channel, SyncSender, TrySendError};
    use std::sync::Arc;
    use std::thread::JoinHandle;
    use std::time::{Duration, Instant};

    use zenoh::Wait;

    use super::{Frames, VizConfig};

    /// 送信スレッドへ渡すキューの深さ。
    ///
    /// 詰まったときに**捨てる**ための緩衝であって、貯めるためではない。
    /// 深くすると詰まりが遅延として現れ、画面が過去を再生し始める。
    const QUEUE_DEPTH: usize = 8;

    /// Zenoh へフレームを流す。
    ///
    /// # 制御ループを止めない
    ///
    /// `session.put(..).wait()` は**ブロッキングのネットワーク呼び出し**で、
    /// JSON 直列化も安くない。どちらも制御ループでやってはいけないので、
    /// 有界チャネルで送信スレッドへ渡し、**満杯なら捨てる**（可視化は lossy で
    /// よい）。捨てた数を数えて終了時に出すので、詰まりが「健全な配信」に
    /// 化けることはない。
    ///
    /// # Session を持ち続けること
    ///
    /// `declare_publisher` の戻り値だけを保持して Session を落とすとセッション
    /// ごと閉じ、`put` は静かに失敗して「配信しているつもりで何も出ていない」に
    /// なる（実際に踏んだ）。Session は送信スレッドが抱えたままにする。
    pub struct Publisher {
        /// `Drop` で先に落として送信スレッドを終わらせるので `Option`。
        tx: Option<SyncSender<Frames>>,
        thread: Option<JoinHandle<()>>,
        period: Duration,
        next: Instant,
        seq: u64,
        queued: u64,
        dropped: Arc<AtomicU64>,
        put: Arc<AtomicU64>,
    }

    impl Publisher {
        pub fn new(cfg: &VizConfig) -> Result<Self, String> {
            if cfg.key_planned == cfg.key_measured {
                return Err(format!(
                    "--viz-key と --viz-key-measured が同じです ('{}')。\
                     チャネルは latest-wins なので 1 本に両方流すと上書きし合います",
                    cfg.key_planned
                ));
            }
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
                "ライブ可視化: 指令 '{}' / 実測 '{}' へ {:.0} Hz で配信します\
                 （articara の Live gait feed で購読）",
                cfg.key_planned,
                cfg.key_measured,
                cfg.rate_hz
            );

            let (tx, rx) = sync_channel::<Frames>(QUEUE_DEPTH);
            let key_planned = cfg.key_planned.clone();
            let key_measured = cfg.key_measured.clone();
            let put = Arc::new(AtomicU64::new(0));
            let put_thread = Arc::clone(&put);
            let thread = std::thread::Builder::new()
                .name("viz-pub".into())
                .spawn(move || {
                    // Session はこのスレッドが抱える。rx が切れたら畳む。
                    for frames in rx {
                        for (key, frame) in [
                            (&key_planned, frames.planned),
                            (&key_measured, frames.measured),
                        ] {
                            let Some(frame) = frame else { continue };
                            match serde_json::to_vec(&frame) {
                                Ok(bytes) => {
                                    // 購読者がいない・切れたといった事情で
                                    // 騒がない。可視化は落ちてよい。
                                    let _ = session
                                        .put(key, bytes)
                                        .encoding(zenoh::bytes::Encoding::APPLICATION_JSON)
                                        .wait();
                                    put_thread.fetch_add(1, Ordering::Relaxed);
                                }
                                Err(e) => {
                                    log::debug!("viz フレームを直列化できません: {e}")
                                }
                            }
                        }
                    }
                    // zenoh の session close は、`--viz-endpoint` で待ち受けて
                    // いると **7 秒ほど固まってからタイムアウトする**（相手が
                    // 繋いでいなくても）。終了をそれだけ待たせる価値は無いので
                    // 別スレッドへ逃がし、join しない。マルチキャスト構成なら
                    // 数 ms で閉じるので、そちらは普通に完了する。
                    std::thread::spawn(move || {
                        let _ = session.close().wait();
                    });
                })
                .map_err(|e| format!("viz 送信スレッドを起動できません: {e}"))?;

            Ok(Self {
                tx: Some(tx),
                thread: Some(thread),
                period: Duration::from_secs_f64(1.0 / cfg.rate_hz.max(1.0)),
                next: Instant::now(),
                seq: 0,
                queued: 0,
                dropped: Arc::new(AtomicU64::new(0)),
                put,
            })
        }

        /// 送信レートに達していれば送信スレッドへ渡す。達していなければ何もしない。
        ///
        /// `build` はフレームの組み立てだけ。直列化も `put` も向こう側でやる。
        pub fn maybe_publish(&mut self, build: impl FnOnce(u64) -> Frames) {
            let now = Instant::now();
            if now < self.next {
                return;
            }
            self.next = now + self.period;
            let frames = build(self.seq);
            self.seq += 1;
            let Some(tx) = self.tx.as_ref() else { return };
            match tx.try_send(frames) {
                Ok(()) => self.queued += 1,
                // 満杯なら捨てる。**ここでブロックしたら制御ループが崩れる。**
                Err(TrySendError::Full(_)) => {
                    self.dropped.fetch_add(1, Ordering::Relaxed);
                }
                Err(TrySendError::Disconnected(_)) => {
                    self.dropped.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
    }

    impl Drop for Publisher {
        fn drop(&mut self) {
            // tx を落とすと rx の for が抜ける。join してから数を出す。
            self.tx.take();
            if let Some(t) = self.thread.take() {
                let _ = t.join();
            }
            let dropped = self.dropped.load(Ordering::Relaxed);
            let put = self.put.load(Ordering::Relaxed);
            if dropped > 0 {
                log::warn!(
                    "ライブ可視化: {} tick を配信、{} tick を取りこぼしました\
                     （送信が制御周期に追いつかず捨てた分。put {} 件）",
                    self.queued,
                    dropped,
                    put
                );
            } else {
                log::info!(
                    "ライブ可視化: {} tick を配信しました（取りこぼし無し、put {} 件）",
                    self.queued,
                    put
                );
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
    fn the_body_pose_is_x_y_z_yaw_and_roll_pitch_rides_alongside() {
        let body = BodyView {
            xy: [0.3, -0.1],
            yaw: 0.5,
            z: 0.20,
            rp: [0.02, -0.05],
            stance: [true, false, true, false],
        };
        let f = frame(0, 0.0, &JointVec::zeros(), &body);
        assert_eq!(f.pose, [0.3, -0.1, 0.20, 0.5]);
        assert_eq!(f.pose_rp, [0.02, -0.05]);
        assert_eq!(f.stance, [true, false, true, false]);
    }

    #[test]
    fn a_planned_and_measured_pair_carries_one_seq() {
        let p = frame(9, 1.0, &JointVec::zeros(), &BodyView::default());
        let m = frame(9, 1.0, &JointVec::zeros(), &BodyView::default());
        let f = Frames::both(p, m);
        assert_eq!(f.planned.unwrap().seq, f.measured.unwrap().seq);
    }

    /// planned と measured が**別キーへ、同じ `seq` で**出ることをループバックで
    /// 確かめる。`--viz` で一番壊れやすいのがここ（1 本に両方流すと受け側が
    /// 上書きし合ってガタつく）。
    ///
    /// zenoh のピア探索を使うので**ネットワーク依存**。既定では走らせない:
    ///
    /// ```sh
    /// cargo test --features viz -- --ignored --nocapture
    /// ```
    #[cfg(feature = "viz")]
    #[test]
    #[ignore = "zenoh のピア探索が要る（ローカルで手動実行する）"]
    fn planned_and_measured_land_on_separate_keys_with_one_seq() {
        use std::time::Duration;
        use zenoh::Wait;

        let cfg = VizConfig {
            enabled: true,
            key_planned: "namiashi/viztest/planned".into(),
            key_measured: "namiashi/viztest/measured".into(),
            rate_hz: 1000.0,
            endpoint: None,
        };

        let sub_session = zenoh::open(zenoh::Config::default()).wait().unwrap();
        let sub_p = sub_session
            .declare_subscriber(&cfg.key_planned)
            .wait()
            .unwrap();
        let sub_m = sub_session
            .declare_subscriber(&cfg.key_measured)
            .wait()
            .unwrap();

        let mut pubr = Publisher::new(&cfg).unwrap();
        // ピア同士が見つかるまでの間に送った分は誰にも届かないので、
        // 見つかるだけの猶予を置いてから送る。
        std::thread::sleep(Duration::from_millis(1500));

        let mut planned = JointVec::zeros();
        planned.legs[0][0] = 0.5;
        let mut measured = JointVec::zeros();
        measured.legs[0][0] = 0.4;
        let body = BodyView {
            rp: [0.01, -0.02],
            ..Default::default()
        };
        for _ in 0..20 {
            pubr.maybe_publish(|seq| {
                Frames::both(
                    frame(seq, 1.0, &planned, &BodyView::default()),
                    frame(seq, 1.0, &measured, &body),
                )
            });
            std::thread::sleep(Duration::from_millis(5));
        }
        drop(pubr);

        fn take(
            sub: &zenoh::pubsub::Subscriber<
                zenoh::handlers::FifoChannelHandler<zenoh::sample::Sample>,
            >,
        ) -> GaitVizFrame {
            let s = sub
                .recv_timeout(Duration::from_secs(3))
                .expect("フレームが届きません（タイムアウト）")
                .expect("購読が閉じています");
            serde_json::from_slice(&s.payload().to_bytes()).unwrap()
        }
        let p = take(&sub_p);
        let m = take(&sub_m);

        // 別キーに、それぞれの中身が出ていること。
        assert_eq!(p.joints[0], 0.5, "planned キーに指令が出ていない");
        assert_eq!(m.joints[0], 0.4, "measured キーに実測が出ていない");
        // measured だけが実測の roll/pitch を運ぶ。
        assert_eq!(p.pose_rp, [0.0, 0.0]);
        assert_eq!(m.pose_rp, [0.01, -0.02]);
        // 同じ tick の対は同じ seq。以降のフレームも 1 対 1 で並ぶ。
        assert_eq!(p.seq, m.seq, "planned と measured の seq がずれている");
    }

    #[test]
    fn the_two_keys_differ_by_default() {
        let d = VizConfig::default();
        assert_ne!(d.key_planned, d.key_measured);
    }
}
