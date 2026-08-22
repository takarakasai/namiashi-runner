//! ロボットモデルの読み込みと歩容コントローラの組み立て。
//!
//! `.misa` から `misarta::model::Model` を作り、`quadruped-gait` の自動検出で
//! 脚の運動学（リンク長・関節符号）をモデルから直接取る。歩容側の設定を
//! 手書きの数値表にしないことで、モデルを直したのにコード側が古いまま、
//! という食い違いが起きないようにする（`go2-gait-runner` と同じ方針）。

use misarta::model::Model;
use quadruped_gait::{
    auto_detect_kinematics_config, joint_signs, AnyGaitController, ControllerOutput, GaitConfig,
    GaitGenerator, GaitMode, GaitType, KinematicsConfig, KneePattern, VelocityCmd,
    DEFAULT_FOOT_LINKS,
};

use crate::config::{AppConfig, GaitTuning};
use crate::jointvec::JointVec;
use crate::pose::PoseLibrary;
use crate::teleop::GaitSelect;

/// 読み込み済みのロボット。
pub struct Robot {
    pub model: Model<f64>,
    pub kin: KinematicsConfig,
    /// IK 出力 → モデル（URDF）符号の変換表。`q_model = q_ik * sign`。
    pub signs: [[f64; 3]; 4],
    pub poses: PoseLibrary,
    /// 運動学の自動検出に使った姿勢（`nq` 長）。
    pub home_q: Vec<f64>,
}

impl Robot {
    /// `.misa` を読んで運動学を自動検出する。
    ///
    /// `kinematics_pose` は「立った姿勢」の名前。自動検出はこの姿勢での FK を
    /// 使って公称の脚の高さを決めるので、脚を伸ばし切った姿勢を渡すと歩容の
    /// 立ち位置が高くなりすぎる。
    pub fn load(misa_path: &str, kinematics_pose: &str) -> Result<Self, String> {
        let parsed = misarta::native::load(misa_path)
            .map_err(|e| format!("{misa_path} の読み込みに失敗: {e:?}"))?;
        if !parsed.report.is_empty() {
            log::warn!(
                "{misa_path} の読み込みで警告があります: {:?}",
                parsed.report
            );
        }
        let (model, _visual, _collision) = misarta::native::build_model(&parsed.file)
            .map_err(|e| format!("モデルの構築に失敗: {e:?}"))?;

        let poses = PoseLibrary::from_misa(&parsed.file);
        let posture = resolve_kinematics_posture(&poses, kinematics_pose);
        let home_q = build_q(&model, &posture);

        let kin = auto_detect_kinematics_config(&model, &DEFAULT_FOOT_LINKS, &home_q)
            .map_err(|errs| format!("脚の運動学を自動検出できません: {errs:?}"))?;
        let signs = joint_signs(&model, &kin)?;

        Ok(Self {
            model,
            kin,
            signs,
            poses,
            home_q,
        })
    }

    /// 歩容コントローラを組み立てる。
    pub fn build_gait(&self, tuning: &GaitTuning, select: GaitSelect) -> AnyGaitController {
        let mut cfg =
            GaitConfig::for_type(gait_type_of(select)).with_swing_height(tuning.swing_height_m);
        if let Some(period) = cycle_period_of(tuning, select) {
            cfg = cfg.with_cycle_period(period);
        }
        let mode = gait_mode_of(select, tuning.crawl_use_linear);
        let mut ctrl =
            AnyGaitController::new(mode, cfg, self.kin_at_height(tuning.stance_height_m));
        // 膝はすべて後ろ向き（namiashi は thigh + / calf − で畳む）。
        ctrl.set_knee_pattern(KneePattern::BothBack);
        // LinearCrawl はこちらで胴体高さを持つ。CHAMP 系は
        // `nominal_foot_body` を見るので上の `kin_at_height` が効く。
        ctrl.set_body_height_m(tuning.stance_height_m);
        ctrl
    }

    /// 立ち高さを `stance_height_m` にした運動学設定。
    ///
    /// `LegKinematics::nominal_foot_body` は「立ったときに足がいる位置」で、
    /// 自動検出は `kinematics_pose` での順運動学からこれを決める。つまり
    /// 立ち高さは基準姿勢に引きずられる。CHAMP 系のコントローラは
    /// `set_body_height_m` を見ない（あれは LinearCrawl 専用）ので、
    /// 設定した高さをどの歩容でも効かせるにはここを書き換えるしかない。
    fn kin_at_height(&self, stance_height_m: f64) -> KinematicsConfig {
        let mut kin = self.kin.clone();
        for leg in [&mut kin.fl, &mut kin.fr, &mut kin.rl, &mut kin.rr] {
            leg.nominal_foot_body.z = -stance_height_m;
        }
        kin
    }

    /// 歩容の出力（IK 座標系）をモデル座標系の関節ベクトルへ直す。
    ///
    /// 腕は歩容の管轄外なので `arm` はそのまま持ち越す。
    /// 胴体を `[roll, pitch]` (rad) 傾けた姿勢の関節角。
    ///
    /// **足先は世界座標で動かさない。** 歩容が出した足先位置（胴体座標系）を
    /// 逆向きに回してから IK を解き直すので、接地したまま胴体だけが傾く。
    ///
    /// 歩容側に胴体姿勢の制御は無い（`set_body_attitude_observed` は
    /// FullCentroidal 専用で、既定の Champ では no-op）。ここで足すしかない。
    ///
    /// `[0, 0]` のときは [`Self::output_to_joints`] にそのまま委ねる。
    /// **無効時に 1 ビットも変わらないことを保証するため**、丸め誤差の入る
    /// 経路を通さない。
    pub fn output_to_joints_tilted(
        &self,
        out: &ControllerOutput,
        arm: f64,
        attitude_rad: [f64; 3],
    ) -> (JointVec, bool) {
        let [roll, pitch, yaw] = attitude_rad;
        if roll == 0.0 && pitch == 0.0 && yaw == 0.0 {
            return (self.output_to_joints(out, arm), out.all_reachable());
        }
        let mut q = JointVec::zeros();
        q.arm = arm;
        let mut reachable = true;
        // 胴体を +roll/+pitch/+yaw 傾ける = 胴体座標系で見た足先を逆向きに
        // 回す。順序は Rz(−yaw) → Rx(−roll) → Ry(−pitch)。
        //
        // **yaw は「足を接地したまま胴体をひねる」** 動作になる。足先は
        // 胴体中心まわりに接線方向へ動くので、hip の可動域を食う。
        let (sy_, cy) = (-yaw).sin_cos();
        let (sr, cr) = (-roll).sin_cos();
        let (sp, cp) = (-pitch).sin_cos();
        for (slot, leg_out) in out.legs.iter().enumerate() {
            let f = leg_out.foot_body;
            // Rz(−yaw)
            let (x0, y0) = (f.x * cy - f.y * sy_, f.x * sy_ + f.y * cy);
            // Rx(−roll)
            let (y1, z1) = (y0 * cr - f.z * sr, y0 * sr + f.z * cr);
            // Ry(−pitch)
            let (x2, z2) = (x0 * cp + z1 * sp, -x0 * sp + z1 * cp);
            let target = nalgebra::Vector3::new(x2, y1, z2);
            // 膝はすべて後ろ向き（`build_gait` の `KneePattern::BothBack`）。
            // ここが食い違うと逆向きに曲がった解が返る。
            let sol = quadruped_gait::solve_leg_ik(self.kin.leg(leg_out.leg), target, false);
            reachable &= sol.is_reachable();
            let (hip, thigh, calf) = sol.angles();
            let s = self.signs[slot];
            q.legs[slot] = [hip * s[0], thigh * s[1], calf * s[2]];
        }
        (q, reachable)
    }

    pub fn output_to_joints(&self, out: &ControllerOutput, arm: f64) -> JointVec {
        let mut q = JointVec::zeros();
        q.arm = arm;
        for (name, q_ik) in out.iter_joint_targets() {
            let Some((leg, k)) = namiashi_hal::joint::lookup(name) else {
                log::warn!("歩容が知らない関節 {name} を出力しました");
                continue;
            };
            q.legs[leg.index()][k] = q_ik * self.signs[leg.index()][k];
        }
        q
    }
}

/// 選択された歩容に対応する `quadruped-gait` の歩容種別。
pub fn gait_type_of(select: GaitSelect) -> GaitType {
    match select {
        GaitSelect::Crawl => GaitType::Crawl,
        GaitSelect::Walk => GaitType::Walk,
        GaitSelect::Trot => GaitType::Trot,
    }
}

/// 選択された歩容に対応するコントローラ。
///
/// 既定はすべて CHAMP 系。**`LinearCrawl` は胴体を +X 直線に載せる専用の
/// プランナで、横移動 (vy) と旋回 (wz) の指令を受け付けない**ので、
/// 「前後・左右・旋回をプロポで操る」という要件には合わない。直進の
/// 安定性を追い込みたいときだけ `gait.crawl_use_linear = true` で選ぶ。
pub fn gait_mode_of(select: GaitSelect, crawl_use_linear: bool) -> GaitMode {
    match select {
        GaitSelect::Crawl if crawl_use_linear => GaitMode::LinearCrawl,
        _ => GaitMode::Champ,
    }
}

/// 手振りポーズは**実際の運動学で計算して `.misa` に書いてある**。
///
/// # 3 本足で立つので重心移動が要る
///
/// 立ち姿勢の足先は胴体座標系で `(±0.147, ±0.109, −0.200)`。前脚を 1 本
/// 上げると支持三角形の対角線がちょうど胴体中心を通るので、**何もしないと
/// 倒れる**（余裕を計算すると 0.00001 m）。上げる前に胴体を後ろ＋対角側へ
/// 0.055 m 逃がしてある（内向き法線 `(∓0.596, ±0.803)` 方向）。
///
/// # 数値の出どころ
///
/// `solve_leg_ik` に足先位置を渡して解き、`signs` を掛けてモデル角にした
/// ものをそのまま `.misa` に書いた。**手で書いた数字ではない。** 生成に
/// 使った計算は `git log` を辿れば残っている（一度きりの作業なので
/// テストとしては残していない）。
///
/// 全 8 ポーズの可動域の余裕は 12° 以上（一番きついのは `wave_fr_a` の
/// FR hip で +12.61°）。
///
/// その歩容が実行中の胴体高さ変更（プロポ CH3）を受け付けるか。
///
/// **`LinearCrawl` だけが受け付ける。** `AnyGaitController::set_body_height_m`
/// は他のモードでは `_ => {}` で**黙って捨てる**ので、上位からは成功と
/// 区別がつかない。既定は全歩容 `Champ` なので、既定設定では CH3 は
/// どこにも効かない。
///
/// 設定の `gait.stance_height_m` は別経路（構築時の `GaitConfig`）なので
/// こちらは全歩容で効く。**「設定で高さを変えたら姿勢が変わった」ことを
/// 「実行中の高さ変更が効く」根拠にしてはいけない**（実際に取り違えた）。
pub fn gait_supports_body_height(mode: GaitMode) -> bool {
    matches!(mode, GaitMode::LinearCrawl)
}

fn cycle_period_of(tuning: &GaitTuning, select: GaitSelect) -> Option<f64> {
    match select {
        GaitSelect::Crawl => tuning.crawl_cycle_s,
        GaitSelect::Walk => tuning.walk_cycle_s,
        GaitSelect::Trot => tuning.trot_cycle_s,
    }
}

/// 操縦指令 → 歩容への速度指令。
pub fn velocity_cmd(vx: f64, vy: f64, wz: f64) -> VelocityCmd {
    VelocityCmd { vx, vy, wz }
}

/// 運動学の自動検出に使う姿勢を決める。
///
/// 指定された名前が無ければ `[home]`、それも空なら全ゼロ。落とさないのは、
/// モデルを差し替えたときにポーズ名が揃っていなくても起動できるほうが
/// 現場では役に立つため。ただし黙って変えるのは危ないので警告は出す。
fn resolve_kinematics_posture(poses: &PoseLibrary, name: &str) -> JointVec {
    if let Some(pose) = poses.pose(name) {
        return poses.resolve(&pose.angles, JointVec::zeros());
    }
    log::warn!(
        "姿勢 {name:?} がモデルにありません。[home] を使います（あるポーズ: {:?}）",
        poses.pose_names().collect::<Vec<_>>()
    );
    poses.home()
}

/// 関節ベクトルを misarta の `q`（長さ `model.nq`）へ展開する。
fn build_q(model: &Model<f64>, posture: &JointVec) -> Vec<f64> {
    let mut q = model.neutral_q();
    for (name, value) in posture.iter_named() {
        set_joint(model, &mut q, name, value);
    }
    q
}

/// 名前で 1 関節ぶんの `q` を書く。モデルに無ければ何もしない。
fn set_joint(model: &Model<f64>, q: &mut [f64], name: &str, value: f64) {
    if let Some(i) = model.joints.iter().position(|j| j.name == name) {
        q[model.q_idx[i]] = value;
    }
}

/// アプリ設定からロボットを読む。
pub fn load_from_config(cfg: &AppConfig) -> Result<Robot, String> {
    Robot::load(&cfg.control.model, &cfg.control.kinematics_pose)
}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn gait_selection_maps_to_the_documented_controllers() {
        assert_eq!(gait_type_of(GaitSelect::Crawl), GaitType::Crawl);
        assert_eq!(gait_type_of(GaitSelect::Walk), GaitType::Walk);
        assert_eq!(gait_type_of(GaitSelect::Trot), GaitType::Trot);
        // 既定では 3 種とも CHAMP 系。横移動と旋回を受けるのはこちらだけ。
        for select in [GaitSelect::Crawl, GaitSelect::Walk, GaitSelect::Trot] {
            assert_eq!(gait_mode_of(select, false), GaitMode::Champ);
        }
        assert_eq!(gait_mode_of(GaitSelect::Crawl, true), GaitMode::LinearCrawl);
        assert_eq!(gait_mode_of(GaitSelect::Walk, true), GaitMode::Champ);
    }

    /// 既定設定では CH3（胴体高さ）はどの歩容でも効かない。
    ///
    /// `AnyGaitController::set_body_height_m` が `LinearCrawl` 以外を
    /// `_ => {}` で捨てるため。**黙って捨てられるので、警告を出す側の
    /// 判定がここと食い違うと誰も気づけない。**
    #[test]
    fn only_linear_crawl_takes_a_body_height_change() {
        assert!(gait_supports_body_height(GaitMode::LinearCrawl));
        assert!(!gait_supports_body_height(GaitMode::Champ));
        // 既定 (crawl_use_linear = false) では 3 歩容とも Champ なので、
        // CH3 はどこにも効かない。
        for select in [GaitSelect::Crawl, GaitSelect::Walk, GaitSelect::Trot] {
            assert!(
                !gait_supports_body_height(gait_mode_of(select, false)),
                "{select:?} で高さ変更が効くことになっている"
            );
        }
        // crawl_use_linear = true なら Crawl だけ効く。
        assert!(gait_supports_body_height(gait_mode_of(
            GaitSelect::Crawl,
            true
        )));
        assert!(!gait_supports_body_height(gait_mode_of(
            GaitSelect::Trot,
            true
        )));
    }

    /// 同梱モデルが既定設定の指す名前を全部持っていること。モデルを
    /// 差し替えたときにここが落ちれば、実機で「ポーズが無い」と気づく前に
    /// 分かる。
    #[test]
    fn the_shipped_model_has_every_pose_the_default_config_names() {
        let mut cfg = AppConfig::default();
        // 既定のパスはリポジトリルート相対。テストの作業ディレクトリは
        // crate ディレクトリなので、ここだけ絶対パスにする。
        cfg.control.model = shipped_model_path();
        let robot = load_from_config(&cfg).expect("同梱モデルを読めません");
        assert!(
            robot.poses.pose(&cfg.control.start_pose).is_some(),
            "初期姿勢 {:?} がモデルにありません",
            cfg.control.start_pose
        );
        assert!(
            robot.poses.pose(&cfg.control.kinematics_pose).is_some(),
            "運動学の基準姿勢 {:?} がモデルにありません",
            cfg.control.kinematics_pose
        );
        assert!(
            robot.poses.sequence(&cfg.poses.greeting).is_some()
                || robot.poses.pose(&cfg.poses.greeting).is_some(),
            "挨拶動作 {:?} がモデルにありません",
            cfg.poses.greeting
        );
    }

    /// 同梱モデルの絶対パス（`crates/namiashi-runner` から見たリポジトリルート）。
    fn shipped_model_path() -> String {
        format!("{}/../../models/namiashi.misa", env!("CARGO_MANIFEST_DIR"))
    }

    #[test]
    fn the_stance_height_is_written_into_the_nominal_foot_position() {
        let robot = Robot::load(&shipped_model_path(), "extend").unwrap();
        let kin = robot.kin_at_height(0.21);
        for leg in [&kin.fl, &kin.fr, &kin.rl, &kin.rr] {
            assert!((leg.nominal_foot_body.z + 0.21).abs() < 1e-12);
        }
    }

    #[test]
    fn a_missing_kinematics_pose_falls_back_to_home() {
        let mut home = JointVec::zeros();
        home.legs[0][1] = 0.5;
        let lib = PoseLibrary::new(vec![], vec![], home);
        assert_eq!(resolve_kinematics_posture(&lib, "nope"), home);
    }
}
