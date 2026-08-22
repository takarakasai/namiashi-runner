//! 動作モードの状態機械。**ハードウェアに一切触れない**ので、実機なしで
//! 遷移そのものを試験できる。
//!
//! ```text
//!   Relaxed ──(スイッチ: 起立/歩行)──▶ GoingToStart ──▶ GoingToStance ──▶ Active
//!      ▲                                                                    │
//!      └────────────────(スイッチ: 脱力)────────────────────────────────────┘
//!                                                    Active ──(ポーズ)──▶ PlayingPose
//!                                                       ▲                   │
//!                                                       └───────────────────┘
//! ```
//!
//! `Active` が起立と歩行の両方を兼ねているのは、`quadruped-gait` が
//! 速度指令ゼロを「脚を接地したまま止まる」として扱うため。状態を分けると、
//! 同じことを 2 か所で書くことになる。

use misarta::trajectory::InterpolationKind;
use namiashi_hal::imu::ImuSample;
use namiashi_hal::joint::JointMode;
use quadruped_gait::{AnyGaitController, GaitGenerator};

use crate::chicken::ChickenHead;
use crate::config::AppConfig;
use crate::jointvec::JointVec;
use crate::pose::PosePlayer;
use crate::robot::{velocity_cmd, Robot};
use crate::teleop::{GaitSelect, ModeRequest, OperatorCommand};
use crate::viz::BodyView;

/// 状態機械の状態。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// 脱力。モータへは指令を送らず状態だけ読む。
    Relaxed,
    /// 初期姿勢（`control.start_pose`）へ遷移中。
    GoingToStart,
    /// 初期姿勢で保持。**通電したまま止まっている。**
    ///
    /// 試合はこの姿勢でスタートボックスに置いて合図を待つ。CH5 の中段が
    /// ここで、上段（歩行）へ倒すと立ち姿勢を経て歩容に入る。
    HoldingStart,
    /// 歩容の立ち姿勢へ遷移中。
    GoingToStance,
    /// 歩容が動いている（速度ゼロなら立ったまま）。
    Active,
    /// ポーズ / シーケンスを再生中。
    PlayingPose,
}

impl State {
    pub fn label(self) -> &'static str {
        match self {
            State::Relaxed => "脱力",
            State::GoingToStart => "初期姿勢へ",
            State::HoldingStart => "初期姿勢",
            State::GoingToStance => "立ち姿勢へ",
            State::Active => "歩容",
            State::PlayingPose => "ポーズ再生",
        }
    }
}

/// 1 周期ぶんの出力。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlOutput {
    /// 12 脚関節 + 腕の目標角 (rad, モデル座標系)。
    pub targets: JointVec,
    /// 脚関節へ与える制御モード。`Relaxed` の間は `Idle`。
    pub leg_mode: JointMode,
    pub state: State,
}

/// 状態機械 + 歩容 + ポーズ再生。
pub struct Controller {
    robot: Robot,
    /// 腕がこちらの指令で動くか。false（受信機直結・未配線）のときは
    /// チキンヘッドもポーズの腕動作も成立しないので、腕の目標には
    /// **観測値をそのまま置く**。指令値を置くと、実機と食い違った角度で
    /// ログと可視化が埋まる。
    arm_app_driven: bool,
    gait: AnyGaitController,
    gait_select: GaitSelect,
    state: State,
    player: Option<PosePlayer>,
    /// 直近に出した目標。遷移の始点であり、`Relaxed` からの復帰点でもある。
    targets: JointVec,
    chicken: ChickenHead,
    cfg: AppConfig,
    /// 状態が変わった直後だけ true。ログ用。
    just_changed: bool,
    /// チキンヘッドが効かないことを 1 度だけ警告するためのフラグ。
    /// 毎周期出すとログが埋まる。
    warned_chicken_head: bool,
    /// 胴体高さ変更が効かないことを 1 度だけ警告するためのフラグ。
    /// 歩容を切り替えたら出し直す。
    warned_body_height: bool,
    /// ランプ後の速度指令 `[vx, vy, wz]`。歩容へ渡すのはこちら。
    ///
    /// **スティックの値を直接渡すと歩容の出力が階段状に飛ぶ**（実測で
    /// 制御 1 周期あたり Crawl 31.5 rad/s = 5 ms で 9.0°）。歩容自体は
    /// 2 tick 目以降は滑らかなので、跳ぶのは切り替わりの 1 点だけ。
    /// ここで鈍らせれば消える。
    ramped_v: [f64; 3],
    /// 停止指令を受けてから全脚接地を待っている時間 [s]。
    /// 待ちが終わらないまま歩き続けないための保険。
    settling_s: f64,
    /// 直近の歩容出力から取った胴体姿勢と接地。可視化にだけ使う。
    ///
    /// 歩容を回していない状態（遷移中・ポーズ再生中）でも姿勢を描きたいので、
    /// 最後に分かった値を保持する。歩容が止まっている間は胴体も動かない
    /// ので、これは嘘ではない。
    body_view: BodyView,
}

impl Controller {
    /// 腕を駆動しない構成（受信機直結・未配線）向け。
    pub fn new(robot: Robot, cfg: AppConfig) -> Self {
        Self::with_arm(robot, cfg, false)
    }

    /// `arm_app_driven` はアプリが腕サーボを駆動できるか
    /// （`namiashi_hal::arm::ArmServo::is_app_driven`）。
    pub fn with_arm(robot: Robot, cfg: AppConfig, arm_app_driven: bool) -> Self {
        let gait_select = GaitSelect::Crawl;
        let gait = robot.build_gait(&cfg.gait, gait_select);
        let chicken = ChickenHead::new(&cfg.poses);
        Self {
            robot,
            arm_app_driven,
            gait,
            gait_select,
            state: State::Relaxed,
            player: None,
            targets: JointVec::zeros(),
            chicken,
            cfg,
            just_changed: false,
            warned_chicken_head: false,
            warned_body_height: false,
            ramped_v: [0.0; 3],
            settling_s: 0.0,
            body_view: BodyView::default(),
        }
    }

    pub fn state(&self) -> State {
        self.state
    }

    pub fn gait_select(&self) -> GaitSelect {
        self.gait_select
    }

    #[allow(dead_code)]
    pub fn robot(&self) -> &Robot {
        &self.robot
    }

    /// 直前の [`Self::tick`] で状態が変わったか。
    pub fn state_changed(&self) -> bool {
        self.just_changed
    }

    /// 可視化用の胴体姿勢と接地フラグ。
    pub fn body_view(&self) -> BodyView {
        self.body_view
    }

    /// 再生中のポーズ名（表示用）。
    pub fn playing(&self) -> Option<&str> {
        self.player.as_ref().map(|p| p.step_name())
    }

    /// 1 周期進める。
    ///
    /// `measured` は実機の現在角。`Relaxed` の間はこれを目標として持ち回るので、
    /// 起立に移った瞬間に「今いる位置」から遷移が始まる（0 rad へ飛ばない）。
    pub fn tick(
        &mut self,
        cmd: &OperatorCommand,
        measured: &JointVec,
        imu: &ImuSample,
        dt: f64,
    ) -> ControlOutput {
        let before = self.state;

        // 歩容の切り替えは、脚が地面にあるあいだにやると踏み替えが飛ぶ。
        // 脱力中と遷移中だけ差し替える。
        if cmd.gait != self.gait_select && !matches!(self.state, State::Active | State::PlayingPose)
        {
            self.set_gait(cmd.gait);
        }

        match self.state {
            State::Relaxed => self.tick_relaxed(cmd, measured),
            State::GoingToStart => self.tick_going_to_start(cmd, dt),
            State::HoldingStart => self.tick_holding_start(cmd),
            State::GoingToStance => self.tick_transition(dt, State::Active),
            State::Active => self.tick_active(cmd, imu, dt),
            State::PlayingPose => self.tick_pose(cmd, dt),
        }

        // 脱力要求はどの状態からでも即座に効く。
        if cmd.mode == ModeRequest::Relax && self.state != State::Relaxed {
            self.enter_relaxed();
        }

        self.just_changed = before != self.state;
        ControlOutput {
            targets: self.targets,
            leg_mode: if self.state == State::Relaxed {
                JointMode::Idle
            } else {
                JointMode::Position
            },
            state: self.state,
        }
    }

    fn tick_relaxed(&mut self, cmd: &OperatorCommand, measured: &JointVec) {
        // 脱力中の「目標」は実測値。次に起立するときの始点になる。
        self.targets = *measured;
        if cmd.mode != ModeRequest::Relax {
            let start = match self.robot.poses.pose(&self.cfg.control.start_pose) {
                Some(p) => self.robot.poses.resolve(&p.angles, *measured),
                None => {
                    log::warn!(
                        "初期姿勢 {:?} がモデルにありません。立ち姿勢へ直接向かいます",
                        self.cfg.control.start_pose
                    );
                    self.stance_targets()
                }
            };
            self.player = Some(PosePlayer::to_pose(
                *measured,
                start,
                self.cfg.control.transition_s,
                InterpolationKind::QuinticSmooth,
            ));
            self.state = State::GoingToStart;
        }
    }

    /// 初期姿勢へ向かう遷移。着いたら**保持**する（歩行要求があれば通す）。
    fn tick_going_to_start(&mut self, cmd: &OperatorCommand, dt: f64) {
        let done = match self.player.as_mut() {
            Some(player) => {
                self.targets = player.tick(dt);
                player.is_done()
            }
            None => true,
        };
        if !done {
            return;
        }
        self.player = None;
        if cmd.mode == ModeRequest::Walk {
            self.begin_stance_transition();
        } else {
            self.state = State::HoldingStart;
        }
    }

    /// 初期姿勢で保持。歩行要求で立ち姿勢へ。
    ///
    /// **目標は動かさない。** 脱力要求は [`Self::tick`] の共通処理が拾う。
    fn tick_holding_start(&mut self, cmd: &OperatorCommand) {
        // 腕を駆動できない構成では、目標にも観測値を置いて食い違わせない。
        if !self.arm_app_driven {
            if let Some(observed) = cmd.arm_rad {
                self.targets.arm = observed;
            }
        }
        if cmd.mode == ModeRequest::Walk {
            self.begin_stance_transition();
        }
    }

    /// いまの目標から歩容の立ち姿勢へ向かう遷移を張る。
    fn begin_stance_transition(&mut self) {
        let stance = self.stance_targets();
        self.player = Some(PosePlayer::to_pose(
            self.targets,
            stance,
            self.cfg.control.transition_s,
            InterpolationKind::QuinticSmooth,
        ));
        self.state = State::GoingToStance;
    }

    /// いまの目標から初期姿勢へ戻る遷移を張る。歩行 → 中段で使う。
    fn begin_start_pose_transition(&mut self) {
        let start = match self.robot.poses.pose(&self.cfg.control.start_pose) {
            Some(p) => self.robot.poses.resolve(&p.angles, self.targets),
            None => {
                log::warn!(
                    "初期姿勢 {:?} がモデルにありません。立ち姿勢のまま保持します",
                    self.cfg.control.start_pose
                );
                return;
            }
        };
        self.player = Some(PosePlayer::to_pose(
            self.targets,
            start,
            self.cfg.control.transition_s,
            InterpolationKind::QuinticSmooth,
        ));
        self.state = State::GoingToStart;
    }

    /// 遷移中。終わったら `next` へ。
    fn tick_transition(&mut self, dt: f64, next: State) {
        let done = match self.player.as_mut() {
            Some(player) => {
                self.targets = player.tick(dt);
                player.is_done()
            }
            None => true,
        };
        if !done {
            return;
        }
        self.player = None;
        match next {
            State::GoingToStance => {
                // 歩容の立ち姿勢を求めてから、そこへ向かう遷移を張る。
                let stance = self.stance_targets();
                self.player = Some(PosePlayer::to_pose(
                    self.targets,
                    stance,
                    self.cfg.control.transition_s,
                    InterpolationKind::QuinticSmooth,
                ));
                self.state = State::GoingToStance;
            }
            _ => {
                // 歩容へ引き渡す。位相は最初から。
                self.gait.reset();
                self.state = State::Active;
            }
        }
    }

    fn tick_active(&mut self, cmd: &OperatorCommand, imu: &ImuSample, dt: f64) {
        if cmd.play_pose {
            self.start_pose_playback();
            return;
        }
        // CH5 を中段へ戻したら初期姿勢へ帰る。**歩容を抜ける唯一の道**
        // （脱力は共通処理が拾う）。速度ゼロで立ち続けたいなら上段のまま
        // スティックを中立にすればよい。
        if cmd.mode == ModeRequest::Stand {
            self.begin_start_pose_transition();
            return;
        }
        // 胴体高さはスティックで上下できる。歩容の立ち位置そのものを動かす。
        // **ただし受け付けるのは LinearCrawl だけ**（`gait_supports_body_height`）。
        // 他の歩容では `set_body_height_m` が黙って捨てるので、動かしても
        // 何も起きない。黙って無視すると「配線かプロポの故障」に見えるので、
        // 実際にスティックが動いたときに 1 度だけ言う。
        self.gait
            .set_body_height_m(self.cfg.gait.stance_height_m + cmd.height_offset_m);
        if cmd.height_offset_m != 0.0
            && !self.warned_body_height
            && !crate::robot::gait_supports_body_height(crate::robot::gait_mode_of(
                self.gait_select,
                self.cfg.gait.crawl_use_linear,
            ))
        {
            log::warn!(
                "歩容 {} は実行中の胴体高さ変更を受け付けません（CH3 は効きません）。\
                 受け付けるのは LinearCrawl のみで、そちらは横移動と旋回を受け付けません。\
                 高さを変えるなら設定の gait.stance_height_m を変えて起動し直してください",
                self.gait_select.label()
            );
            self.warned_body_height = true;
        }
        let want = match cmd.mode {
            ModeRequest::Walk => [cmd.vx_m_s, cmd.vy_m_s, cmd.wz_rad_s],
            // 起立中は歩容を止める（速度ゼロ = 接地したまま）。
            _ => [0.0; 3],
        };
        let v = self.ramp_velocity(want, dt);
        self.gait.set_velocity_cmd(velocity_cmd(v[0], v[1], v[2]));
        self.gait
            .set_body_attitude_observed(imu.rpy_rad[0], imu.rpy_rad[1]);
        let out = self.gait.tick(dt);
        if !out.all_reachable() {
            log::warn!("IK が届かない脚があります（姿勢がクランプされました）");
        }
        let arm = self.arm_target(cmd, imu.rpy_rad[1], dt);
        self.body_view = BodyView {
            xy: [
                out.body_state.world_position.x,
                out.body_state.world_position.y,
            ],
            yaw: out.body_state.world_yaw,
            z: self.cfg.gait.stance_height_m + cmd.height_offset_m,
            // 歩容は水平計画なので planned の roll/pitch は常に 0。
            // 姿勢を計画する制御（MPC 等）を入れたらここに載せる。
            rp: [0.0, 0.0],
            stance: [
                out.legs[0].phase.is_stance,
                out.legs[1].phase.is_stance,
                out.legs[2].phase.is_stance,
                out.legs[3].phase.is_stance,
            ],
        };
        self.targets = self.robot.output_to_joints(&out, arm);
    }

    fn tick_pose(&mut self, cmd: &OperatorCommand, dt: f64) {
        let done = match self.player.as_mut() {
            Some(player) => {
                self.targets = player.tick(dt);
                player.is_done()
            }
            None => true,
        };
        // 腕を駆動できない構成では、ポーズが腕を動かすつもりでも実機は
        // 受信機に従う。目標にも観測値を置いて食い違わせない。
        if !self.arm_app_driven {
            if let Some(observed) = cmd.arm_rad {
                self.targets.arm = observed;
            }
        }
        // 再生中に歩行へ切り替えたら、その時点で立ち姿勢へ戻す。
        if done || cmd.mode == ModeRequest::Walk {
            self.player = Some(PosePlayer::to_pose(
                self.targets,
                self.stance_targets(),
                self.cfg.control.transition_s,
                InterpolationKind::QuinticSmooth,
            ));
            self.state = State::GoingToStance;
        }
    }

    fn start_pose_playback(&mut self) {
        let name = self.cfg.poses.greeting.clone();
        match PosePlayer::start(&self.robot.poses, &name, self.targets) {
            Ok(player) => {
                log::info!("ポーズ {name:?} を再生します");
                self.player = Some(player);
                self.state = State::PlayingPose;
            }
            // 再生できないなら歩容のまま。動作を止めないほうが安全。
            Err(e) => log::warn!("ポーズを再生できません: {e}"),
        }
    }

    /// この周期の腕の目標角。
    ///
    /// 駆動できるならチキンヘッドの出力、できないなら観測値
    /// （観測もできなければ直前値を保つ）。
    fn arm_target(&mut self, cmd: &OperatorCommand, body_pitch_rad: f64, dt: f64) -> f64 {
        if self.arm_app_driven {
            return self.chicken.update(cmd.chicken_head, body_pitch_rad, dt);
        }
        if cmd.chicken_head && !self.warned_chicken_head {
            log::warn!(
                "チキンヘッドが ON ですが、腕はアプリから駆動できない構成です\
                 （受信機直結 / 未配線）。指令は出しません"
            );
            self.warned_chicken_head = true;
        }
        cmd.arm_rad.unwrap_or(self.targets.arm)
    }

    fn enter_relaxed(&mut self) {
        self.player = None;
        self.state = State::Relaxed;
        self.chicken.reset();
        // 次に立ち上がったとき、前回の速度から歩き出さないように。
        self.ramped_v = [0.0; 3];
    }

    /// 歩容が「今この設定で立つ」姿勢。時間を進めずに取り出す。
    fn stance_targets(&mut self) -> JointVec {
        self.gait.set_body_height_m(self.cfg.gait.stance_height_m);
        self.gait.set_velocity_cmd(velocity_cmd(0.0, 0.0, 0.0));
        let out = self.gait.tick(0.0);
        self.robot.output_to_joints(&out, self.targets.arm)
    }

    /// 速度指令を `gait.velocity_ramp_s` で鈍らせる。
    ///
    /// 各軸の上限レートは「その軸の最大値 ÷ ランプ時間」なので、**どの軸も
    /// 全開から中立まで同じ時間**で戻る。歩き出す側だけでなく**止まる側にも
    /// 同じレートがかかる**（実測では止まるときも 27.9 rad/s 跳んでいた）。
    fn ramp_velocity(&mut self, want: [f64; 3], dt: f64) -> [f64; 3] {
        let ramp_s = self.cfg.gait.velocity_ramp_s;
        if ramp_s <= 0.0 || dt <= 0.0 {
            self.ramped_v = want;
            return want;
        }
        let full = [
            self.cfg.gait.max_vx_m_s,
            self.cfg.gait.max_vy_m_s,
            self.cfg.gait.max_wz_rad_s,
        ];
        for k in 0..3 {
            let step = (full[k].abs() / ramp_s) * dt;
            let d = (want[k] - self.ramped_v[k]).clamp(-step, step);
            self.ramped_v[k] += d;
        }

        // **歩容は速度ちょうど 0 で静止姿勢へ分岐する。** 遊脚がある瞬間に
        // 0 を渡すと、その脚を一気に接地位置へ引き戻して目標が跳ぶ
        // （実測 35.4 rad/s = 制御 1 周期で 10.2°）。ランプだけでは消えず、
        // 跳ぶ時刻がランプ終了へ移るだけだった。0.001 m/s を渡し続けた
        // 場合は 3.34 rad/s で収まる ＝ **跳びの原因は 0 への分岐そのもの**。
        //
        // なので全脚が接地している瞬間まで微速で歩かせ、そこで 0 に落とす。
        const CREEP_M_S: f64 = 1e-3;
        const SETTLE_TIMEOUT_S: f64 = 2.0;
        let stopping = want.iter().all(|v| *v == 0.0);
        let nearly_stopped = self.ramped_v.iter().all(|v| v.abs() <= CREEP_M_S);
        if stopping && nearly_stopped {
            // 接地フラグは前周期の歩容出力（`body_view`）から取る。
            let all_stance = self.body_view.stance.iter().all(|&s| s);
            if all_stance || self.settling_s >= SETTLE_TIMEOUT_S {
                self.settling_s = 0.0;
                self.ramped_v = [0.0; 3];
                return self.ramped_v;
            }
            // 待っている間だけ微速。**保持しない**ので、接地した次の周期で 0 になる。
            self.settling_s += dt;
            return [CREEP_M_S, 0.0, 0.0];
        }
        self.settling_s = 0.0;
        self.ramped_v
    }

    fn set_gait(&mut self, select: GaitSelect) {
        log::info!("歩容を {} に切り替えます", select.label());
        self.gait = self.robot.build_gait(&self.cfg.gait, select);
        self.gait_select = select;
        // 歩容ごとに可否が違うので、切り替えたら言い直す。
        self.warned_body_height = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::teleop::{GaitSelect, ModeRequest};

    fn imu() -> ImuSample {
        ImuSample {
            rpy_rad: [0.0; 3],
            gyro_rad_s: [0.0; 3],
            accel_m_s2: [0.0, 0.0, 9.80665],
            temperature_c: 25.0,
            stamp: std::time::Instant::now(),
        }
    }

    fn cmd(mode: ModeRequest) -> OperatorCommand {
        OperatorCommand {
            vx_m_s: 0.0,
            vy_m_s: 0.0,
            wz_rad_s: 0.0,
            height_offset_m: 0.0,
            arm_rad: None,
            mode,
            gait: GaitSelect::Crawl,
            play_pose: false,
            chicken_head: false,
            link_ok: true,
        }
    }

    fn controller() -> Controller {
        let cfg = AppConfig::default();
        let robot = Robot::load(&test_model_path(), &cfg.control.kinematics_pose)
            .expect("テスト用モデルを読めません");
        Controller::new(robot, cfg)
    }

    fn test_model_path() -> String {
        // crates/namiashi-runner から見たリポジトリルート。
        format!("{}/../../models/namiashi.misa", env!("CARGO_MANIFEST_DIR"))
    }

    /// 状態が `want` になるまで回す。回りすぎたら失敗。
    /// 停止 ↔ 歩行の切り替わりで目標角が跳ばないこと。
    ///
    /// **歩容は速度指令が変わった瞬間に出力を階段状に飛ばす。** ランプ無しの
    /// 実測は制御 1 周期（5 ms）あたり Crawl 31.5 / Walk 23.0 / Trot 9.9 rad/s
    /// で、Crawl は 1 周期で 9.0° 飛んでいた。2 tick 目以降は滑らかなので、
    /// 跳ぶのは切り替わりの 1 点だけ。
    #[test]
    fn starting_and_stopping_a_walk_does_not_step_the_targets() {
        let dt = 0.005;
        for select in [GaitSelect::Crawl, GaitSelect::Walk, GaitSelect::Trot] {
            let mut c = controller();
            let mut stand = cmd(ModeRequest::Walk);
            stand.gait = select;
            run_until(&mut c, &stand, State::Active, 20.0);
            let mut prev = c.tick(&stand, &JointVec::zeros(), &imu(), dt).targets;

            let mut go = stand;
            go.vx_m_s = 0.10;
            let mut worst: f64 = 0.0;
            // 歩き出して、また止まる。**両側に跳びがある。**
            for phase in [go, stand] {
                for _ in 0..400 {
                    let q = c.tick(&phase, &JointVec::zeros(), &imu(), dt).targets;
                    for l in 0..4 {
                        for k in 0..3 {
                            worst = worst.max((q.legs[l][k] - prev.legs[l][k]).abs() / dt);
                        }
                    }
                    prev = q;
                }
            }
            // ランプと全脚接地待ちを入れて 3.3〜5.4 rad/s。無しでは 9.9〜31.5。
            assert!(
                worst < 6.0,
                "{} で目標が {:.2} rad/s 跳んだ（{:.1}°/周期）",
                select.label(),
                worst,
                (worst * dt).to_degrees()
            );
        }
    }

    /// **スティックが中立を一瞬通っても歩容が止まらない。**
    ///
    /// 歩容の `PhaseGenerator::advance` は `vel.is_zero()`（**厳密な等値
    /// 比較**）で即座に `holding` に入り、全脚を接地・`cycle_position = 0`
    /// として静止姿勢を出す。方向転換でスティックが中立を通過すると
    /// 3 軸そろって 0.0 になり、**その瞬間だけ立脚静止に落ちる**。
    /// 位相は凍結されるだけで戻らないので、復帰すると前の動きの続きに
    /// 見える（実機で「前の指令が残っているような動き」として観測）。
    ///
    /// ランプが時間的なヒステリシスになって、これを防ぐ。
    #[test]
    fn a_momentary_stick_centre_does_not_stop_the_gait() {
        let dt = 0.005;
        for select in [GaitSelect::Crawl, GaitSelect::Walk, GaitSelect::Trot] {
            let mut c = controller();
            let mut go = cmd(ModeRequest::Walk);
            go.gait = select;
            go.vx_m_s = 0.10;
            run_until(&mut c, &go, State::Active, 20.0);
            for _ in 0..400 {
                c.tick(&go, &JointVec::zeros(), &imu(), dt);
            }
            // 中立を 25 ms 通過して反対側へ。
            let mut centre = go;
            centre.vx_m_s = 0.0;
            let mut back = go;
            back.vx_m_s = -0.10;
            for _ in 0..5 {
                c.tick(&centre, &JointVec::zeros(), &imu(), dt);
                assert!(
                    c.ramped_v.iter().any(|v| *v != 0.0),
                    "{} で中立通過の瞬間に速度がちょうど 0 になった                     （歩容が立脚静止へ落ちる）",
                    select.label()
                );
            }
            for _ in 0..5 {
                c.tick(&back, &JointVec::zeros(), &imu(), dt);
            }
        }
    }

    /// ランプは止まる側にもかかる。片側だけでは足りない。
    #[test]
    fn the_velocity_ramp_applies_in_both_directions() {
        let mut cfg = AppConfig::default();
        cfg.gait.velocity_ramp_s = 0.5;
        let robot = Robot::load(&test_model_path(), &cfg.control.kinematics_pose).unwrap();
        let mut c = Controller::new(robot, cfg);
        let dt = 0.01;
        let mut go = cmd(ModeRequest::Walk);
        go.vx_m_s = 0.15;
        run_until(&mut c, &go, State::Active, 20.0);
        // 全開まで 0.5 s かかる → 0.1 s 時点では半分にも達していない。
        let mut c2 = controller();
        run_until(&mut c2, &cmd(ModeRequest::Walk), State::Active, 20.0);
        for _ in 0..10 {
            c2.tick(&go, &JointVec::zeros(), &imu(), dt);
        }
        assert!(
            c2.ramped_v[0] < 0.15 * 0.5,
            "0.1 s で {:.3} m/s まで来ている（ランプが効いていない）",
            c2.ramped_v[0]
        );
        // 中立へ戻すときも同じレート。
        let before = c2.ramped_v[0];
        c2.tick(&cmd(ModeRequest::Walk), &JointVec::zeros(), &imu(), dt);
        assert!(c2.ramped_v[0] < before);
        assert!(c2.ramped_v[0] > 0.0, "1 周期で 0 まで落ちている");
    }

    fn run_until(c: &mut Controller, command: &OperatorCommand, want: State, max_s: f64) {
        let dt = 0.005;
        let mut t = 0.0;
        while t < max_s {
            let out = c.tick(command, &JointVec::zeros(), &imu(), dt);
            if out.state == want {
                return;
            }
            t += dt;
        }
        panic!("{:?} に到達しませんでした（今は {:?}）", want, c.state());
    }

    #[test]
    fn it_starts_relaxed_and_sends_no_position_command() {
        let mut c = controller();
        let out = c.tick(&cmd(ModeRequest::Relax), &JointVec::zeros(), &imu(), 0.005);
        assert_eq!(out.state, State::Relaxed);
        assert_eq!(out.leg_mode, JointMode::Idle);
    }

    #[test]
    fn relaxed_targets_follow_the_measured_angles() {
        // 起立に移った瞬間に 0 rad へ飛ばないための性質。
        let mut c = controller();
        let mut measured = JointVec::zeros();
        measured.legs[2][1] = 0.61;
        let out = c.tick(&cmd(ModeRequest::Relax), &measured, &imu(), 0.005);
        assert_eq!(out.targets, measured);
    }

    #[test]
    fn standing_goes_through_the_start_pose_then_the_stance() {
        let mut c = controller();
        let stand = cmd(ModeRequest::Walk);
        c.tick(&stand, &JointVec::zeros(), &imu(), 0.005);
        assert_eq!(c.state(), State::GoingToStart);
        run_until(&mut c, &stand, State::GoingToStance, 10.0);
        run_until(&mut c, &stand, State::Active, 10.0);
        assert_eq!(
            c.tick(&stand, &JointVec::zeros(), &imu(), 0.005).leg_mode,
            JointMode::Position
        );
    }

    #[test]
    fn the_start_pose_is_actually_reached_before_the_stance_transition() {
        let cfg = AppConfig::default();
        let mut c = controller();
        let stand = cmd(ModeRequest::Walk);
        run_until(&mut c, &stand, State::GoingToStance, 10.0);
        // GoingToStance に入った時点の目標は start_pose と一致しているはず。
        let start = c
            .robot()
            .poses
            .pose(&cfg.control.start_pose)
            .map(|p| c.robot().poses.resolve(&p.angles, JointVec::zeros()))
            .expect("start_pose がモデルにありません");
        let out = c.tick(&stand, &JointVec::zeros(), &imu(), 0.0);
        assert!(
            out.targets.max_abs_diff(&start) < 1e-6,
            "start_pose に着く前に次の遷移へ進んでいます"
        );
    }

    /// CH5 中段は**初期姿勢で止まる**。立ち姿勢まで行かない。
    ///
    /// 試合はこの姿勢でスタートボックスに置いて合図を待つ。
    #[test]
    fn the_middle_switch_position_holds_the_start_pose() {
        let mut c = controller();
        run_until(&mut c, &cmd(ModeRequest::Stand), State::HoldingStart, 20.0);
        let held = c
            .tick(&cmd(ModeRequest::Stand), &JointVec::zeros(), &imu(), 0.005)
            .targets;
        // 通電したまま保持する。**脱力しない。**
        let out = c.tick(&cmd(ModeRequest::Stand), &JointVec::zeros(), &imu(), 0.005);
        assert_eq!(out.state, State::HoldingStart);
        assert_eq!(out.leg_mode, JointMode::Position);
        // 何周期回しても動かない。
        for _ in 0..200 {
            c.tick(&cmd(ModeRequest::Stand), &JointVec::zeros(), &imu(), 0.005);
        }
        let still = c
            .tick(&cmd(ModeRequest::Stand), &JointVec::zeros(), &imu(), 0.005)
            .targets;
        assert!(
            still.max_abs_diff(&held) < 1e-9,
            "初期姿勢で保持できていない"
        );
        // 保持しているのは設定の初期姿勢そのもの。
        let want = {
            let p = c.robot.poses.pose(&c.cfg.control.start_pose).unwrap();
            c.robot.poses.resolve(&p.angles, JointVec::zeros())
        };
        assert!(
            held.max_abs_diff(&want) < 1e-6,
            "初期姿勢と違う姿勢で止まっている"
        );
    }

    /// `--allow-no-sbus`（受信機なしのベンチ）は**初期姿勢で止まる**。
    ///
    /// CH5 中段の意味を変えた副作用。以前は立ち姿勢を経て歩容まで行って
    /// いた。**受信機が無いまま歩容へ入る道を残す理由がない**ので、
    /// この方が安全側。段階 7-2 の手順もこれに合わせてある。
    #[test]
    fn the_bench_command_stops_at_the_start_pose() {
        use crate::teleop::{Teleop, TeleopConfig};
        let t = Teleop::new(
            TeleopConfig::default(),
            &crate::config::GaitTuning::default(),
            &namiashi_hal::config::HardwareConfig::default().arm,
        );
        let bench = t.bench_stand();
        assert_eq!(bench.mode, ModeRequest::Stand);
        let mut c = controller();
        run_until(&mut c, &bench, State::HoldingStart, 20.0);
        // 放っておいても歩容へは進まない。
        for _ in 0..2000 {
            assert_eq!(
                c.tick(&bench, &JointVec::zeros(), &imu(), 0.005).state,
                State::HoldingStart
            );
        }
    }

    /// 中段 → 上段で歩容へ、上段 → 中段で初期姿勢へ戻る。
    #[test]
    fn the_switch_walks_from_the_start_pose_and_returns_to_it() {
        let mut c = controller();
        run_until(&mut c, &cmd(ModeRequest::Stand), State::HoldingStart, 20.0);
        run_until(&mut c, &cmd(ModeRequest::Walk), State::Active, 20.0);
        // 上段のままスティック中立なら、立ち姿勢で止まっていられる。
        for _ in 0..100 {
            assert_eq!(
                c.tick(&cmd(ModeRequest::Walk), &JointVec::zeros(), &imu(), 0.005)
                    .state,
                State::Active
            );
        }
        // 中段へ戻すと初期姿勢へ帰る。
        run_until(&mut c, &cmd(ModeRequest::Stand), State::HoldingStart, 20.0);
    }

    #[test]
    fn relax_takes_effect_from_any_state() {
        let mut c = controller();
        run_until(&mut c, &cmd(ModeRequest::Walk), State::Active, 20.0);
        let out = c.tick(&cmd(ModeRequest::Relax), &JointVec::zeros(), &imu(), 0.005);
        assert_eq!(out.state, State::Relaxed);
        assert_eq!(out.leg_mode, JointMode::Idle);
        // 初期姿勢の保持中からも同じ。
        let mut c = controller();
        run_until(&mut c, &cmd(ModeRequest::Stand), State::HoldingStart, 20.0);
        let out = c.tick(&cmd(ModeRequest::Relax), &JointVec::zeros(), &imu(), 0.005);
        assert_eq!(out.state, State::Relaxed);
        assert_eq!(out.leg_mode, JointMode::Idle);
    }

    #[test]
    fn walking_forward_moves_the_legs() {
        let mut c = controller();
        run_until(&mut c, &cmd(ModeRequest::Walk), State::Active, 20.0);
        let stance = c
            .tick(&cmd(ModeRequest::Walk), &JointVec::zeros(), &imu(), 0.0)
            .targets;
        let mut walk = cmd(ModeRequest::Walk);
        walk.vx_m_s = 0.1;
        let mut moved = false;
        for _ in 0..400 {
            let out = c.tick(&walk, &JointVec::zeros(), &imu(), 0.005);
            if out.targets.max_abs_diff(&stance) > 1e-3 {
                moved = true;
            }
        }
        assert!(moved, "歩行指令を出しても関節が動いていません");
    }

    #[test]
    fn the_gait_can_be_switched_while_relaxed_but_not_while_active() {
        let mut c = controller();
        let mut relax_trot = cmd(ModeRequest::Relax);
        relax_trot.gait = GaitSelect::Trot;
        c.tick(&relax_trot, &JointVec::zeros(), &imu(), 0.005);
        assert_eq!(c.gait_select(), GaitSelect::Trot);

        // 起立の途中も Trot のまま要求し続ける（遷移中の切り替えは許される）。
        let mut stand_trot = cmd(ModeRequest::Walk);
        stand_trot.gait = GaitSelect::Trot;
        run_until(&mut c, &stand_trot, State::Active, 20.0);
        assert_eq!(c.gait_select(), GaitSelect::Trot);

        // Active 中の切り替え要求は無視される（踏み替えの途中で歩容が飛ばない）。
        let mut walk_crawl = cmd(ModeRequest::Walk);
        walk_crawl.gait = GaitSelect::Crawl;
        c.tick(&walk_crawl, &JointVec::zeros(), &imu(), 0.005);
        assert_eq!(c.gait_select(), GaitSelect::Trot);
    }

    #[test]
    fn an_unknown_pose_name_does_not_break_the_gait() {
        let mut cfg = AppConfig::default();
        cfg.poses.greeting = "no_such_pose".into();
        let robot = Robot::load(&test_model_path(), &cfg.control.kinematics_pose).unwrap();
        let mut c = Controller::new(robot, cfg);
        run_until(&mut c, &cmd(ModeRequest::Walk), State::Active, 20.0);
        let mut play = cmd(ModeRequest::Walk);
        play.play_pose = true;
        let out = c.tick(&play, &JointVec::zeros(), &imu(), 0.005);
        assert_eq!(out.state, State::Active);
    }

    #[test]
    fn a_non_driven_arm_follows_the_observed_angle_not_the_chicken_head() {
        // 受信機直結（既定）。チキンヘッドを ON にしても腕の目標は観測値のまま。
        let mut c = controller();
        run_until(&mut c, &cmd(ModeRequest::Walk), State::Active, 20.0);
        let mut with_arm = cmd(ModeRequest::Walk);
        with_arm.chicken_head = true;
        with_arm.arm_rad = Some(-0.7);
        let pitched = ImuSample {
            rpy_rad: [0.0, 0.4, 0.0],
            ..imu()
        };
        for _ in 0..200 {
            c.tick(&with_arm, &JointVec::zeros(), &pitched, 0.005);
        }
        let out = c.tick(&with_arm, &JointVec::zeros(), &pitched, 0.005);
        assert!(
            (out.targets.arm + 0.7).abs() < 1e-9,
            "腕の目標が観測値ではなくチキンヘッドの出力になっています: {}",
            out.targets.arm
        );
    }

    #[test]
    fn a_driven_arm_does_run_the_chicken_head() {
        let cfg = AppConfig::default();
        let robot = Robot::load(&test_model_path(), &cfg.control.kinematics_pose).unwrap();
        let mut c = Controller::with_arm(robot, cfg, true);
        run_until(&mut c, &cmd(ModeRequest::Walk), State::Active, 20.0);
        let mut on = cmd(ModeRequest::Walk);
        on.chicken_head = true;
        let pitched = ImuSample {
            rpy_rad: [0.0, 0.4, 0.0],
            ..imu()
        };
        for _ in 0..2000 {
            c.tick(&on, &JointVec::zeros(), &pitched, 0.005);
        }
        let out = c.tick(&on, &JointVec::zeros(), &pitched, 0.005);
        // 胴体ピッチ +0.4 を打ち消すので腕は −0.4 付近。
        assert!(
            (out.targets.arm + 0.4).abs() < 1e-3,
            "チキンヘッドが効いていません: {}",
            out.targets.arm
        );
    }

    #[test]
    fn a_non_driven_arm_holds_its_last_value_when_the_link_is_lost() {
        let mut c = controller();
        run_until(&mut c, &cmd(ModeRequest::Walk), State::Active, 20.0);
        let mut seen = cmd(ModeRequest::Walk);
        seen.arm_rad = Some(0.3);
        c.tick(&seen, &JointVec::zeros(), &imu(), 0.005);
        // 受信断で観測値が無くなっても 0 へ飛ばない。
        let lost = OperatorCommand::failsafe(GaitSelect::Crawl, ModeRequest::Stand);
        let out = c.tick(&lost, &JointVec::zeros(), &imu(), 0.005);
        assert!((out.targets.arm - 0.3).abs() < 1e-9, "{}", out.targets.arm);
    }

    #[test]
    fn playing_a_pose_returns_to_the_stance() {
        let mut cfg = AppConfig::default();
        // モデルに入っているシーケンス名を使う。
        cfg.poses.greeting = "jump".into();
        let robot = Robot::load(&test_model_path(), &cfg.control.kinematics_pose).unwrap();
        let mut c = Controller::new(robot, cfg);
        run_until(&mut c, &cmd(ModeRequest::Walk), State::Active, 20.0);
        let mut play = cmd(ModeRequest::Walk);
        play.play_pose = true;
        assert_eq!(
            c.tick(&play, &JointVec::zeros(), &imu(), 0.005).state,
            State::PlayingPose
        );
        run_until(&mut c, &cmd(ModeRequest::Walk), State::Active, 20.0);
    }
}
