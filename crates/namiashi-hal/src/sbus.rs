//! S.BUS 受信（CH348 UART6、反転はハードウェア U15 が行うので受信専用）。
//!
//! ワイヤの復号もポート探索も [`sbus`] クレートが持っている。ここが足すのは
//! **受信スレッドと、制御ループから見た「今この瞬間の値」**だけ:
//! `sbus::Sbus::poll` はブロックする I/O なので制御周期から直接は呼べず、
//! 専用スレッドで回して最新の [`SbusState`] を共有スロットに置く。
//!
//! [`SbusState`] は `sbus::State` に「最後にフレームを受けた時刻」を足した
//! もの。`sbus::State` は時刻を持たない純関数的な集計（フィクスチャ再生でも
//! 同じ結果になるように、というのが向こうの設計）なので、「何 ms 前の値か」
//! はこちら側で持つ必要がある。操縦入力を信用してよいかの判定はそれで決まる。

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use sbus::{Sbus, State};

use crate::ch348::PortMap;
use crate::config::SbusConfig;
use crate::error::{Error, Result};

/// S.BUS のボーレート。
pub use sbus::BAUD;

/// 累積カウンタ。
pub use sbus::Counters;

/// S.BUS の生値の下限・中央・上限（`sbus_protocol::{RAW_MIN, RAW_MAX}` 準拠）。
pub const RAW_MIN: u16 = 172;
pub const RAW_CENTER: u16 = 992;
pub const RAW_MAX: u16 = 1811;

/// チャンネル数（アナログ 16ch）。
pub const CHANNELS: usize = 16;

/// 受信状態のスナップショット。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SbusState {
    /// 生のチャンネル値 (0..=2047)。フレーム未受信なら全 0。
    pub channels: [u16; CHANNELS],
    pub ch17: bool,
    pub ch18: bool,
    /// 受信機が RF フレームを落としている。
    pub frame_lost: bool,
    /// 受信機がフェイルセーフに入っている。
    pub failsafe: bool,
    /// S.BUS2（テレメトリスロットあり）で受信しているか。
    pub sbus2: bool,
    /// 受信機電源電圧 (V)。S.BUS2 のスロット 0 が来ていれば。
    pub rx_battery_v: Option<f32>,
    /// 外部電圧入力 (V)。同上。
    pub external_v: Option<f32>,
    /// 受信レート (frames/s)。`sbus::State::fps`。
    pub fps: f32,
    /// 最後に制御フレームを受けた時刻。
    pub stamp: Instant,
    pub counters: Counters,
}

impl Default for SbusState {
    /// フレーム未受信の状態。`is_usable` は false を返す。
    fn default() -> Self {
        Self::initial()
    }
}

impl SbusState {
    fn initial() -> Self {
        Self {
            channels: [0; CHANNELS],
            ch17: false,
            ch18: false,
            frame_lost: false,
            failsafe: false,
            sbus2: false,
            rx_battery_v: None,
            external_v: None,
            fps: 0.0,
            stamp: Instant::now(),
            counters: Counters::default(),
        }
    }

    /// `sbus::State` からスナップショットを起こす。`stamp` は呼び出し側が
    /// 「新しいフレームが来たとき」だけ更新する。
    fn from_driver(state: &State, stamp: Instant) -> Self {
        let frame = state.frame;
        Self {
            channels: frame.map(|f| f.channels).unwrap_or([0; CHANNELS]),
            ch17: frame.map(|f| f.ch17).unwrap_or(false),
            ch18: frame.map(|f| f.ch18).unwrap_or(false),
            frame_lost: frame.map(|f| f.frame_lost).unwrap_or(false),
            failsafe: frame.map(|f| f.failsafe).unwrap_or(false),
            sbus2: state.sbus2,
            rx_battery_v: state.rx_battery_v,
            external_v: state.external_v,
            fps: state.fps,
            stamp,
            counters: state.counters,
        }
    }

    /// チャンネル値をパルス幅 (µs) で。表示用（`sbus_protocol::raw_to_us` は
    /// 送信機のエンドポイント設定に依存する近似なので、制御には生値を使う）。
    pub fn channel_us(&self, index: usize) -> Option<u16> {
        self.channels
            .get(index)
            .map(|&raw| sbus::protocol::raw_to_us(raw))
    }

    /// フレームが `max_age` 以内に来ているか。
    pub fn is_fresh(&self, max_age: Duration) -> bool {
        self.counters.frames > 0 && self.stamp.elapsed() <= max_age
    }

    /// 操縦入力として信用してよいか。
    ///
    /// フェイルセーフ中・フレーム途絶中は false。ここが false のときに
    /// 速度指令を 0 にするのは上位（`namiashi-runner` の teleop）の仕事。
    pub fn is_usable(&self, max_age: Duration) -> bool {
        self.is_fresh(max_age) && !self.failsafe
    }
}

#[derive(Debug)]
struct SbusSlot {
    state: Mutex<SbusState>,
    last_error: Mutex<String>,
}

/// S.BUS 受信スレッドのハンドル。
pub struct SbusReceiver {
    port: String,
    stale_after: Duration,
    slot: Arc<SbusSlot>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl SbusReceiver {
    pub fn connect(cfg: &SbusConfig) -> Result<Self> {
        Self::connect_with(cfg, &PortMap::discover()?)
    }

    /// 事前に取った探索結果を使って開く。
    pub fn connect_with(cfg: &SbusConfig, map: &PortMap) -> Result<Self> {
        let port = cfg.port.resolve_with(map)?;
        let driver = Sbus::open(&port).map_err(|e| Error::SbusDriver {
            port: port.clone(),
            source: e,
        })?;

        let slot = Arc::new(SbusSlot {
            state: Mutex::new(SbusState::initial()),
            last_error: Mutex::new(String::new()),
        });
        let stop = Arc::new(AtomicBool::new(false));
        let worker = SbusWorker {
            driver,
            slot: Arc::clone(&slot),
            stop: Arc::clone(&stop),
        };
        let thread = std::thread::Builder::new()
            .name("sbus".into())
            .spawn(move || worker.run())?;

        Ok(Self {
            port,
            stale_after: Duration::from_millis(cfg.stale_after_ms),
            slot,
            stop,
            thread: Some(thread),
        })
    }

    pub fn port(&self) -> &str {
        &self.port
    }

    pub fn state(&self) -> SbusState {
        *lock(&self.slot.state)
    }

    /// 操縦入力として信用してよいか（設定の `stale_after_ms` で判定）。
    pub fn is_usable(&self) -> bool {
        self.state().is_usable(self.stale_after)
    }

    pub fn last_error(&self) -> String {
        lock(&self.slot.last_error).clone()
    }

    /// 最初の制御フレームが来るまで待つ。
    pub fn wait_ready(&self, timeout: Duration) -> Result<SbusState> {
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            let s = self.state();
            if s.counters.frames > 0 {
                return Ok(s);
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        Err(Error::Timeout {
            what: format!("S.BUS ({})", self.port),
            timeout,
        })
    }
}

impl Drop for SbusReceiver {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

struct SbusWorker {
    driver: Sbus,
    slot: Arc<SbusSlot>,
    stop: Arc<AtomicBool>,
}

impl SbusWorker {
    fn run(mut self) {
        let mut frames_seen = 0u64;
        let mut stamp = Instant::now();
        while !self.stop.load(Ordering::Relaxed) {
            // `poll` は 1 回の read ぶんを流し込んで戻る（read タイムアウトは
            // ドライバ側が短く持っている）。空読みは 0 を返すだけで異常ではない。
            match self.driver.poll(|_| {}) {
                Ok(_) => {}
                Err(e) => {
                    *lock(&self.slot.last_error) = e.to_string();
                    // 抜線などで即座に失敗し続けるとホットループになるので、
                    // 少し待ってから再試行する。
                    std::thread::sleep(Duration::from_millis(20));
                    continue;
                }
            }
            let state = self.driver.state();
            // 新しい制御フレームが来たときだけ時刻を進める。テレメトリ
            // スロットだけで `stamp` を更新すると、送信機が切れていても
            // 「新鮮」に見えてしまう。
            if state.counters.frames != frames_seen {
                frames_seen = state.counters.frames;
                stamp = Instant::now();
            }
            *lock(&self.slot.state) = SbusState::from_driver(state, stamp);
        }
        log::info!("S.BUS の受信スレッドを停止しました");
    }
}

fn lock<T>(m: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|p| p.into_inner())
}

#[cfg(test)]
mod tests {
    use super::*;
    use sbus::{Event, Parser};

    /// `sbus-protocol` の doctest と同じ、実機由来の 1 フレーム
    /// （footer 0x04 = S.BUS2 group 0）+ Ext-Volt スロット。
    const STREAM: [u8; 28] = [
        0x0F, 0xF9, 0x5B, 0xDF, 0x02, 0xED, 0x07, 0x04, 0x20, 0x00, 0x1F, 0xF8, 0x40, 0x00, 0x3E,
        0x00, 0x01, 0x08, 0x40, 0x00, 0x02, 0x10, 0x80, 0x00, 0x04, 0x03, 0xC4, 0xF1,
    ];

    /// バイト列を `sbus::State` に流してからスナップショットを取る。
    /// 受信スレッドがやっていることと同じ経路。
    fn snapshot(bytes: &[u8]) -> SbusState {
        let mut state = State::default();
        let mut parser = Parser::new();
        parser.push_slice(bytes, |e: Event| state.apply(&e));
        SbusState::from_driver(&state, Instant::now())
    }

    #[test]
    fn a_capture_updates_channels_and_telemetry() {
        let s = snapshot(&STREAM);
        assert_eq!(s.counters.frames, 1);
        assert_eq!(s.counters.desync_bytes, 0);
        assert!(s.sbus2);
        assert_eq!(s.external_v, Some(24.1));
        assert!(s.channel_us(0).is_some());
    }

    #[test]
    fn desync_bytes_are_counted_not_hidden() {
        let mut noisy = vec![0xAA, 0xBB];
        noisy.extend_from_slice(&STREAM);
        let s = snapshot(&noisy);
        assert_eq!(s.counters.frames, 1);
        assert_eq!(s.counters.desync_bytes, 2);
    }

    #[test]
    fn a_state_with_no_frames_is_never_usable() {
        let s = SbusState::initial();
        assert!(!s.is_fresh(Duration::from_secs(10)));
        assert!(!s.is_usable(Duration::from_secs(10)));
    }

    #[test]
    fn failsafe_is_not_usable_even_when_fresh() {
        let mut s = snapshot(&STREAM);
        s.failsafe = true;
        assert!(s.is_fresh(Duration::from_secs(10)));
        assert!(!s.is_usable(Duration::from_secs(10)));
    }

    #[test]
    fn a_stale_stamp_is_not_usable_even_without_failsafe() {
        let mut s = snapshot(&STREAM);
        s.stamp = Instant::now() - Duration::from_millis(500);
        assert!(!s.failsafe);
        assert!(!s.is_usable(Duration::from_millis(100)));
    }
}
