//! 腕（`arm_pitch_joint`）の RC サーボ。
//!
//! 配線は基板の ARMA / ARMB を TTL に切り替えて使うシリアルサーボ
//! （`nm_board/ch348/spec_rev2_0_0_asbuilt.md` §4、CN3 / CN4 の GH3:
//! `GND / ARMx_TX / ARMx_TTL_RX`）。ただし**初期検討では受信機に直結**し、
//! アプリは腕を駆動しない（[`ArmProtocol::ReceiverDirect`]）。品種が決まったら
//! [`ArmServo`] の実装を足して `protocol` を切り替える。
//!
//! # 「駆動する」と「繋がっている」を分けてある
//!
//! 受信機直結の腕は、**動いてはいるがアプリの指令では動かない**。ここを 1 つの
//! bool にまとめると「サーボがある = 指令が効く」と読めてしまい、チキンヘッドや
//! ポーズ再生の腕動作が黙って無効化されていることに気づけない。だから
//! [`ArmServo::is_connected`]（実物があるか）と
//! [`ArmServo::is_app_driven`]（こちらの指令で動くか）を別にしている。

use crate::config::{ArmConfig, ArmProtocol};
use crate::error::Result;

/// 腕サーボ 1 軸。
///
/// 単位はモデル座標系の rad で、実機との符号・ゼロ点・可動域の差は実装側が
/// [`ArmConfig`] を見て吸収する（脚と同じ約束）。
pub trait ArmServo: Send {
    /// 目標角 (rad, モデル座標系) を送る。可動域は実装がクランプする。
    ///
    /// [`Self::is_app_driven`] が false の実装では**何もしない**。
    fn set_position(&mut self, q_model_rad: f64) -> Result<()>;

    /// 外から観測した現在角 (rad, モデル座標系) を教える。
    ///
    /// 受信機直結のときに、プロポのチャンネルから割り出した角度を入れる。
    /// アプリは駆動しないが、ログ・可視化・モデル状態には実際の角度が要る。
    fn observe(&mut self, _q_model_rad: f64) {}

    /// 現在角 (rad, モデル座標系)。駆動しているなら直近の指令値、
    /// 観測しているなら直近の観測値。
    fn position(&self) -> f64;

    /// 脱力させる。対応していなければ何もしない。
    fn relax(&mut self) -> Result<()> {
        Ok(())
    }

    /// 実物が繋がっているか。
    fn is_connected(&self) -> bool;

    /// **こちらの指令で動くか。** false なら [`Self::set_position`] は無視され、
    /// チキンヘッドやポーズ再生の腕動作は成立しない。
    fn is_app_driven(&self) -> bool;
}

/// 設定に従って腕サーボを開く。
///
/// 現状どちらの `protocol` もポートを開かない（受信機直結・未配線のどちらでも
/// アプリは腕バスを触らない）。品種が決まったら、ここで開く実装を足す。
pub fn connect(cfg: &ArmConfig) -> Result<Box<dyn ArmServo>> {
    match cfg.protocol {
        ArmProtocol::ReceiverDirect => {
            log::info!(
                "腕サーボ: 受信機直結。アプリからは駆動しません\
                 （チキンヘッドとポーズ再生の腕動作は無効です）"
            );
            Ok(Box::new(ReceiverDirectArm::new(cfg)))
        }
        ArmProtocol::None => {
            log::info!("腕サーボ: protocol = none（未配線。指令は破棄されます）");
            Ok(Box::new(NullArm::new(cfg)))
        }
    }
}

/// 受信機直結の腕。**指令は捨て、観測値だけを持つ。**
///
/// `set_position` が指令を覚えないのは意図的。覚えてしまうと
/// [`ArmServo::position`] が「アプリがそう命じた角度」を返し、実機の腕が
/// どこにあるかとは無関係な値でログと可視化が埋まる。
pub struct ReceiverDirectArm {
    min_rad: f64,
    max_rad: f64,
    observed: f64,
}

impl ReceiverDirectArm {
    pub fn new(cfg: &ArmConfig) -> Self {
        Self {
            min_rad: cfg.min_rad,
            max_rad: cfg.max_rad,
            observed: 0.0f64.clamp(cfg.min_rad, cfg.max_rad),
        }
    }
}

impl ArmServo for ReceiverDirectArm {
    fn set_position(&mut self, _q_model_rad: f64) -> Result<()> {
        Ok(())
    }

    fn observe(&mut self, q_model_rad: f64) {
        self.observed = q_model_rad.clamp(self.min_rad, self.max_rad);
    }

    fn position(&self) -> f64 {
        self.observed
    }

    fn is_connected(&self) -> bool {
        true
    }

    fn is_app_driven(&self) -> bool {
        false
    }
}

/// 未配線のときの受け皿。
///
/// 「繋がっていない」ことを黙って成功にしないよう、[`ArmServo::is_connected`]
/// は常に `false` を返す。指令は覚える（実機なしで上位ロジックを試すため）。
pub struct NullArm {
    min_rad: f64,
    max_rad: f64,
    q: f64,
}

impl NullArm {
    pub fn new(cfg: &ArmConfig) -> Self {
        Self {
            min_rad: cfg.min_rad,
            max_rad: cfg.max_rad,
            q: 0.0f64.clamp(cfg.min_rad, cfg.max_rad),
        }
    }
}

impl ArmServo for NullArm {
    fn set_position(&mut self, q_model_rad: f64) -> Result<()> {
        self.q = q_model_rad.clamp(self.min_rad, self.max_rad);
        Ok(())
    }

    fn position(&self) -> f64 {
        self.q
    }

    fn is_connected(&self) -> bool {
        false
    }

    fn is_app_driven(&self) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::HardwareConfig;

    fn cfg() -> ArmConfig {
        HardwareConfig::default().arm
    }

    #[test]
    fn the_default_wiring_is_receiver_direct_and_not_app_driven() {
        let arm = connect(&cfg()).unwrap();
        assert!(arm.is_connected(), "受信機直結の腕は実在する");
        assert!(!arm.is_app_driven(), "アプリからは駆動しない");
    }

    #[test]
    fn a_receiver_direct_arm_ignores_commands_and_reports_what_it_observed() {
        let mut arm = ReceiverDirectArm::new(&cfg());
        arm.observe(-0.5);
        arm.set_position(0.8).unwrap();
        // 指令ではなく観測値が見えること。ここが指令値だとログが実機と食い違う。
        assert_eq!(arm.position(), -0.5);
    }

    #[test]
    fn an_observed_angle_is_clamped_to_the_joint_limits() {
        let c = cfg();
        let mut arm = ReceiverDirectArm::new(&c);
        arm.observe(10.0);
        assert_eq!(arm.position(), c.max_rad);
        arm.observe(-10.0);
        assert_eq!(arm.position(), c.min_rad);
    }

    #[test]
    fn null_arm_clamps_commands_and_never_claims_to_be_connected() {
        let c = cfg();
        let mut arm = NullArm::new(&c);
        arm.set_position(10.0).unwrap();
        assert_eq!(arm.position(), c.max_rad);
        assert!(!arm.is_connected());
        assert!(!arm.is_app_driven());
    }

    #[test]
    fn connecting_does_not_open_a_port_for_either_protocol() {
        for protocol in [ArmProtocol::ReceiverDirect, ArmProtocol::None] {
            let mut c = cfg();
            c.protocol = protocol;
            // 探索も open もせずに成功する（腕が未配線の基板でも起動できる）。
            assert!(connect(&c).is_ok(), "{protocol:?}");
        }
    }
}
