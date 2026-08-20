//! namiashi 実機のハードウェア抽象層。
//!
//! 実機の I/F は `nm_board/ch348` rev2 基板（CH348L, USB-C → 8ch UART）で確定
//! している。本 crate はその 8 本の UART をそれぞれの役割に束ねる層で、上位
//! （`namiashi-runner`）からはモデルの関節名／関節角だけが見えるようにする。
//!
//! ```text
//!  UART0..3  RS485  LEG1..4  ─ LKMTech V3 ×3 (hip/thigh/calf)   → legs::LegArray
//!  UART4     RS485/TTL ARMA  ─ シリアルサーボ (arm_pitch_joint)  → arm::ArmServo
//!  UART5     TTL    IMU      ─ WitMotion                         → imu::ImuReader
//!  UART6     反転TTL S.BUS   ─ 受信専用（プロポ）                 → sbus::SbusReceiver
//!  UART7     RS485/TTL ARMB  ─ 予備
//! ```
//!
//! # スレッドモデル
//!
//! RS485 は半二重の要求応答で、1 トランザクションが USB の往復レイテンシに
//! 律速される。したがって**バス 1 本につきスレッド 1 本**（`misa-actuator` の
//! `doc/handover.md` が言う「1 バス 1 制御ループ」）とし、各スレッドは自分の
//! ペースで自由走行する。制御ループはバスの完了を待たず、共有スロットへ目標を
//! 書き、最新のフィードバックを読むだけ。こうしておくと制御周期がバスの
//! ジッタから切り離され、実際に何 Hz 出ているかは [`legs::BusStats`] で
//! 観測できる（実測してから制御周期を決めるための土台）。
//!
//! IMU と S.BUS も同様に受信スレッドを持ち、最新値を共有スロットに置く。

pub mod arm;
pub mod ch348;
pub mod config;
pub mod error;
pub mod imu;
pub mod joint;
pub mod legs;
pub mod sbus;

pub use error::{Error, Result};
pub use joint::{JointCommand, JointMode, JointState, LegSlot, JOINT_NAMES, LEG_JOINT_KINDS};
