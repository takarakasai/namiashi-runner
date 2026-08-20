//! HAL 共通のエラー型。
//!
//! 下位 crate のエラーはすべてここへ畳む。呼び出し側が「どのポートで」
//! 失敗したかを常に言えるよう、ポート名を持つ variant を用意している
//! （`/dev/ttyCH9344USB*` が 8 本あるので、名前のないエラーは切り分けの
//! 役に立たない）。

use std::time::Duration;

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("{port} を開けませんでした: {source}")]
    OpenPort {
        port: String,
        #[source]
        source: serialport::Error,
    },

    #[error("CH348 のポート探索に失敗しました: {0}")]
    Discovery(String),

    #[error("{port} (LKMTech id={motor_id}): {source}")]
    Motor {
        port: String,
        motor_id: u8,
        #[source]
        source: lkmotor_driver::Error,
    },

    #[error("IMU ({port}): {source}")]
    Imu {
        port: String,
        #[source]
        source: wit_imu::Error,
    },

    #[error("S.BUS ({port}): {source}")]
    SbusDriver {
        port: String,
        #[source]
        source: sbus::Error,
    },

    #[error("設定が不正です: {0}")]
    Config(String),

    #[error("{what} が {} ms 以内に応答しませんでした", .timeout.as_millis())]
    Timeout { what: String, timeout: Duration },

    #[error(transparent)]
    Io(#[from] std::io::Error),
}
