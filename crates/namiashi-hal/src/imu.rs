//! IMU（CH348 UART5、WitMotion IWT603）の受信スレッド。
//!
//! `wit-imu` の `read_sample` は加速度・角速度・姿勢角が揃うまでブロックする
//! ので、制御ループから直接呼ぶわけにはいかない。専用スレッドで回して最新値を
//! 共有スロットに置き、制御ループはそれを読むだけにする。
//!
//! 単位は SI に直して渡す（rad, rad/s, m/s²）。IWT603 は 921600 bps で
//! 各パケット 200 Hz（`wit-imu/doc/communication_spec.md` §8）なので、制御周期
//! より十分速い。

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use wit_imu::WitImu;

use crate::ch348::PortMap;
use crate::config::ImuConfig;
use crate::error::{Error, Result};

/// 標準重力 (m/s²)。加速度を g から SI へ直すのに使う。
const G: f64 = 9.80665;

/// IMU の最新値。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    /// 姿勢角 (rad)。`[roll, pitch, yaw]`、取付補正済み。
    pub rpy_rad: [f64; 3],
    /// 角速度 (rad/s)、センサ座標系。
    pub gyro_rad_s: [f64; 3],
    /// 加速度 (m/s²)、重力込み、センサ座標系。
    pub accel_m_s2: [f64; 3],
    pub temperature_c: f64,
    /// このサンプルを受け取った時刻。古さの判定に使う。
    pub stamp: Instant,
}

impl ImuSample {
    fn placeholder() -> Self {
        Self {
            rpy_rad: [0.0; 3],
            gyro_rad_s: [0.0; 3],
            accel_m_s2: [0.0, 0.0, G],
            temperature_c: 0.0,
            stamp: Instant::now(),
        }
    }

    /// `max_age` より古ければ「来ていない」とみなす。
    pub fn is_fresh(&self, max_age: Duration) -> bool {
        self.stamp.elapsed() <= max_age
    }
}

/// IMU 受信の稼働統計。
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ImuStats {
    pub samples: u64,
    pub errors: u64,
    pub rate_hz: f64,
    /// プロトコルの再同期で捨てたバイト数。0 近傍でないなら配線かボーレートを疑う。
    pub resync_bytes: u64,
}

#[derive(Debug)]
struct ImuSlot {
    sample: Mutex<Option<ImuSample>>,
    stats: Mutex<ImuStats>,
    last_error: Mutex<String>,
}

/// IMU 受信スレッドのハンドル。
pub struct ImuReader {
    port: String,
    slot: Arc<ImuSlot>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl ImuReader {
    /// ポートを開いて受信スレッドを起こす。
    pub fn connect(cfg: &ImuConfig) -> Result<Self> {
        Self::connect_with(cfg, &PortMap::discover()?)
    }

    /// 事前に取った探索結果を使って開く。
    pub fn connect_with(cfg: &ImuConfig, map: &PortMap) -> Result<Self> {
        let port = cfg.port.resolve_with(map)?;
        let timeout = Duration::from_millis(cfg.response_timeout_ms);
        let imu = WitImu::open(&port, cfg.baud, timeout).map_err(|e| Error::Imu {
            port: port.clone(),
            source: e,
        })?;

        let slot = Arc::new(ImuSlot {
            sample: Mutex::new(None),
            stats: Mutex::new(ImuStats::default()),
            last_error: Mutex::new(String::new()),
        });
        let stop = Arc::new(AtomicBool::new(false));
        let worker = ImuWorker {
            imu,
            mount_offset_rad: cfg.mount_offset_rad,
            slot: Arc::clone(&slot),
            stop: Arc::clone(&stop),
        };
        let thread = std::thread::Builder::new()
            .name("imu".into())
            .spawn(move || worker.run())?;

        Ok(Self {
            port,
            slot,
            stop,
            thread: Some(thread),
        })
    }

    pub fn port(&self) -> &str {
        &self.port
    }

    /// 最新サンプル。1 個も受け取っていなければ `None`。
    pub fn sample(&self) -> Option<ImuSample> {
        *lock(&self.slot.sample)
    }

    /// 最新サンプル、まだ何も来ていなければ「水平・静止」を返す。
    ///
    /// 姿勢推定を持たない制御（位置制御のゲイト）を IMU 無しでも走らせる
    /// ための逃げ道。使う側は [`Self::sample`] で有無を判定できる。
    pub fn sample_or_level(&self) -> ImuSample {
        self.sample().unwrap_or_else(ImuSample::placeholder)
    }

    pub fn stats(&self) -> ImuStats {
        *lock(&self.slot.stats)
    }

    pub fn last_error(&self) -> String {
        lock(&self.slot.last_error).clone()
    }

    /// 最初のサンプルが来るまで待つ。
    pub fn wait_ready(&self, timeout: Duration) -> Result<ImuSample> {
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            if let Some(s) = self.sample() {
                return Ok(s);
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        Err(Error::Timeout {
            what: format!("IMU ({})", self.port),
            timeout,
        })
    }
}

impl Drop for ImuReader {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

struct ImuWorker {
    imu: WitImu,
    mount_offset_rad: [f64; 3],
    slot: Arc<ImuSlot>,
    stop: Arc<AtomicBool>,
}

impl ImuWorker {
    fn run(mut self) {
        let mut window_start = Instant::now();
        let mut window_samples = 0u64;

        while !self.stop.load(Ordering::Relaxed) {
            match self.imu.read_sample() {
                Ok(s) => {
                    let sample = convert(&s, self.mount_offset_rad);
                    *lock(&self.slot.sample) = Some(sample);
                    window_samples += 1;
                    let mut st = lock(&self.slot.stats);
                    st.samples += 1;
                    st.resync_bytes = self.imu.resync_bytes();
                    let elapsed = window_start.elapsed().as_secs_f64();
                    if elapsed >= 1.0 {
                        st.rate_hz = window_samples as f64 / elapsed;
                        window_samples = 0;
                        window_start = Instant::now();
                    }
                }
                Err(e) => {
                    // タイムアウトは「まだ揃っていない」であって異常とは
                    // 限らないが、区別せず数える。連続して増えるなら配線。
                    lock(&self.slot.stats).errors += 1;
                    *lock(&self.slot.last_error) = e.to_string();
                }
            }
        }
        log::info!("IMU の受信スレッドを停止しました");
    }
}

/// WitMotion のサンプルを SI + 取付補正済みへ直す。
fn convert(s: &wit_imu::Sample, mount_offset_rad: [f64; 3]) -> ImuSample {
    let to_rad = std::f64::consts::PI / 180.0;
    let mut rpy = [0.0f64; 3];
    for i in 0..3 {
        rpy[i] = s.angle_deg[i] as f64 * to_rad - mount_offset_rad[i];
    }
    ImuSample {
        rpy_rad: rpy,
        gyro_rad_s: [
            s.gyro_dps[0] as f64 * to_rad,
            s.gyro_dps[1] as f64 * to_rad,
            s.gyro_dps[2] as f64 * to_rad,
        ],
        accel_m_s2: [
            s.accel_g[0] as f64 * G,
            s.accel_g[1] as f64 * G,
            s.accel_g[2] as f64 * G,
        ],
        temperature_c: s.temperature_c as f64,
        stamp: Instant::now(),
    }
}

fn lock<T>(m: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|p| p.into_inner())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn raw(angle_deg: [f32; 3], gyro_dps: [f32; 3], accel_g: [f32; 3]) -> wit_imu::Sample {
        wit_imu::Sample {
            accel_g,
            gyro_dps,
            angle_deg,
            temperature_c: 25.0,
            magnetic_raw: None,
        }
    }

    #[test]
    fn units_are_converted_to_si() {
        let s = convert(
            &raw([90.0, 0.0, 0.0], [180.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
            [0.0; 3],
        );
        assert!((s.rpy_rad[0] - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
        assert!((s.gyro_rad_s[0] - std::f64::consts::PI).abs() < 1e-9);
        assert!((s.accel_m_s2[2] - G).abs() < 1e-9);
    }

    #[test]
    fn mount_offset_is_subtracted() {
        let s = convert(&raw([10.0, 0.0, 0.0], [0.0; 3], [0.0; 3]), [0.1, 0.0, 0.0]);
        assert!((s.rpy_rad[0] - (10.0f64.to_radians() - 0.1)).abs() < 1e-9);
    }

    #[test]
    fn placeholder_reports_gravity_and_level_attitude() {
        // IMU 未接続時の代替値が「傾いている」と読めてはならない。
        let p = ImuSample::placeholder();
        assert_eq!(p.rpy_rad, [0.0; 3]);
        assert!((p.accel_m_s2[2] - G).abs() < 1e-12);
    }
}
