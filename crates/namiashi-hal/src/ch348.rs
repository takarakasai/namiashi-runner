//! CH348L の物理 UART 番号でポートを引く。
//!
//! `/dev/ttyCH9344USB0..7` の番号は列挙順で決まるので、基板の UART 番号
//! （= どのコネクタか）とは原理的に無関係。`nm_board/ch348/spec_rev2_0_0_asbuilt.md`
//! §10.7 が固定デバイス名に依存しないことを要件にしているのはこのためで、
//! ch9344 ドライバの `GETUARTINDEX` ioctl で物理 UART 番号を取得して対応付ける。
//!
//! **探索そのものは [`sbus::discover`] のものを使う。** 同じ ioctl と sysfs
//! 探索をここにも書くと、片方だけ直したときに黙ってずれる。S.BUS 以外
//! （脚の RS485、IMU）も同じ基板の別 UART なので、`discover::list_ch348_ports`
//! をそのまま使えばよい。このモジュールが足しているのは
//! **「開く前に 1 回だけ調べる」という運用の型**（[`PortMap`]）と、
//! 基板の UART 番号 → 役割の対応表だけ。

use std::path::PathBuf;

use sbus::discover;

use crate::error::{Error, Result};

/// CH348 のポート（`sbus::discover` の型をそのまま使う）。
pub use sbus::discover::Ch348Port;

/// CH348 の USB ベンダ ID（沁恒 / WCH）。
pub use sbus::discover::CH348_VID;

/// 基板の役割ごとの UART 番号（as-built §4）。
pub mod uart {
    /// LEG1..4（RS485）。
    pub const LEGS: [u16; 4] = [0, 1, 2, 3];
    /// ARMA（RS485 / TTL 切替）。
    pub const ARM_A: u16 = 4;
    /// IMU（TTL 直結）。
    pub const IMU: u16 = 5;
    /// S.BUS（反転 TTL、受信専用）。`sbus::SBUS_UART_INDEX` と同じ値。
    pub const SBUS: u16 = sbus::SBUS_UART_INDEX;
    /// ARMB（RS485 / TTL 切替）。
    pub const ARM_B: u16 = 7;
}

/// 1 度の探索結果。
///
/// **探索は開く前にまとめて 1 回だけ行うこと。** UART 番号の問い合わせは
/// デバイスを `open` する必要があり、すでに自分が開いているポートは
/// `EBUSY` で開けない（`sbus::discover` はそれを警告つきで読み飛ばす）。
/// ポートを 1 本開くたびに探索し直す作りにすると、2 本目以降が自分自身の
/// せいで「見つからない」になる — 実機で踏んだ。
#[derive(Debug, Clone, Default)]
pub struct PortMap {
    ports: Vec<Ch348Port>,
}

impl PortMap {
    /// いま挿さっている CH348 のポートを一度に調べる。
    pub fn discover() -> Result<Self> {
        let ports = discover::list_ch348_ports()
            .map_err(|e| Error::Discovery(format!("CH348 のポート探索に失敗しました: {e}")))?;
        Ok(Self { ports })
    }

    /// 調べ済みの一覧から作る（試験・再利用用）。
    pub fn from_ports(ports: Vec<Ch348Port>) -> Self {
        let mut ports = ports;
        ports.sort_by_key(|p| p.uart_index);
        Self { ports }
    }

    pub fn ports(&self) -> &[Ch348Port] {
        &self.ports
    }

    /// 物理 UART 番号 `index` のポートパス。
    pub fn path_for(&self, index: u16) -> Result<PathBuf> {
        if self.ports.is_empty() {
            return Err(Error::Discovery(
                "CH348 のポートが 1 本も見つかりません（基板の USB は挿さっていますか）".into(),
            ));
        }
        self.ports
            .iter()
            .find(|p| p.uart_index == index)
            .map(|p| p.path.clone())
            .ok_or_else(|| {
                let seen: Vec<String> = self
                    .ports
                    .iter()
                    .map(|p| format!("{}={}", p.uart_index, p.path.display()))
                    .collect();
                Error::Discovery(format!(
                    "UART{index} が見つかりません（見つかったのは {}）",
                    seen.join(", ")
                ))
            })
    }
}

/// CH348 のポートを物理 UART 番号つきで列挙する。UART 番号昇順。
pub fn list_ports() -> Result<Vec<Ch348Port>> {
    Ok(PortMap::discover()?.ports)
}

/// 物理 UART 番号 `index` のポートパスを返す（その場で 1 回探索する）。
///
/// 複数のポートを開くときは [`PortMap::discover`] を 1 回だけ呼ぶこと。
pub fn find_by_uart_index(index: u16) -> Result<PathBuf> {
    PortMap::discover()?.path_for(index)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uart_map_matches_the_as_built_spec() {
        // spec_rev2_0_0_asbuilt.md §4 の表をコードで固定する。
        assert_eq!(uart::LEGS, [0, 1, 2, 3]);
        assert_eq!(uart::ARM_A, 4);
        assert_eq!(uart::IMU, 5);
        assert_eq!(uart::SBUS, 6);
        assert_eq!(uart::ARM_B, 7);
    }

    #[test]
    fn listing_ports_never_panics_without_hardware() {
        // 実機が無い CI でも探索そのものは動く（結果は空か Discovery エラー）。
        let _ = list_ports();
        let _ = PortMap::discover();
    }

    #[test]
    fn an_empty_map_says_no_board_rather_than_no_such_uart() {
        let err = PortMap::default().path_for(0).unwrap_err().to_string();
        assert!(err.contains("1 本も見つかりません"), "{err}");
    }

    #[test]
    fn a_map_without_the_wanted_uart_lists_what_it_did_find() {
        let map = PortMap::from_ports(vec![Ch348Port {
            path: PathBuf::from("/dev/ttyCH9344USB0"),
            uart_index: 0,
        }]);
        let err = map.path_for(uart::SBUS).unwrap_err().to_string();
        assert!(err.contains("UART6"), "{err}");
        assert!(err.contains("0=/dev/ttyCH9344USB0"), "{err}");
    }
}
