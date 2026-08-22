//! 脚 12 軸 = RS485 バス 4 本 × LKMTech V3 モータ 3 個。
//!
//! # なぜバスごとにスレッドなのか
//!
//! RS485 は半二重の要求応答で、1 トランザクションは「送って、相手の返事を
//! 待つ」。待ち時間は USB の往復レイテンシに律速され、ワイヤ上のビット時間
//! （1 Mbps で 13 バイトなら 130 µs）より桁で大きい。したがってバスを跨いだ
//! 並列化だけが効き、同一バス内は直列にしかならない。`misa-actuator` の
//! `doc/handover.md` が「1 バス 1 制御ループ」を推す理由でもある。
//!
//! # なぜ自由走行なのか
//!
//! 制御ループがバスの完了を待つ形にすると、制御周期の上限が**最も遅い
//! バス**で決まってしまい、しかもそれが何 Hz なのかは実機を繋ぐまで
//! 分からない。ここでは各バスを自由走行させ、制御ループは共有スロットへ
//! 目標を書き最新値を読むだけにしてある。実際に出ている周期は
//! [`BusStats`] で観測できるので、「まず測ってから制御周期を決める」が
//! できる。
//!
//! # 座標変換
//!
//! 上位はモデル（URDF / `.misa`）の関節角だけを扱う。実機との差は
//! [`crate::config::MotorConfig`] の `sign` と `zero_pose_rad` に閉じている。
//!
//! ```text
//! q_motor = sign * (q_model - zero_pose_rad)
//! q_model = sign *  q_motor + zero_pose_rad          (sign = ±1)
//! ```

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender, TryRecvError};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use lkmotor_driver::{Motor, MotorConfig as LkMotorConfig, MotorId, Rs485Driver};

use crate::ch348::PortMap;
use crate::config::{HardwareConfig, LegBusConfig, LegsConfig, MotorConfig};
use crate::error::{Error, Result};
use crate::joint::{JointCommand, JointMode, JointState, LegSlot};

/// 1 本のバスの稼働統計。制御周期を決めるための一次データ。
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct BusStats {
    /// 完了した周回数。
    pub ticks: u64,
    /// トランザクション失敗の累計（3 軸合計）。
    pub errors: u64,
    /// 直近 1 秒の実効周期 (Hz)。
    pub rate_hz: f64,
    /// 直近の周回に要した時間 (s)。3 軸分のトランザクション時間。
    pub last_cycle_s: f64,
    /// 起動以降の最悪周回時間 (s)。
    pub worst_cycle_s: f64,
}

/// 1 軸のゆっくり変わる状態（State1）。
///
/// 電圧・温度・エラービットは State2（周期で読んでいるほう）には入っておらず、
/// 別トランザクションが要る。制御周期で毎回読むとバス帯域の 1/2 を食うので、
/// `status_interval_ms` ごとに 1 軸ずつ回して読む。
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct JointStatus {
    pub voltage_v: f64,
    pub temperature_c: f64,
    /// State1 のエラービット生値。0 なら異常なし。
    pub error_raw: u8,
    /// 一度でも読めたか。false の値は意味を持たない。
    pub valid: bool,
}

impl JointStatus {
    /// 何らかの異常ビットが立っているか。
    pub fn faulted(&self) -> bool {
        self.valid && self.error_raw != 0
    }

    /// 立っている異常ビットを日本語で並べる。異常なしなら空。
    ///
    /// **生値の `0x01` だけ出しても現場では何も分からない。** マニュアル
    /// §1 の `errorState` は 1 ビットずつ意味が違い、対処もまるで別物
    /// （低電圧は電源、過熱は冷却待ち）。
    pub fn describe(&self) -> String {
        if !self.faulted() {
            return String::new();
        }
        const BITS: [(u8, &str); 4] = [
            (0x01, "低電圧保護"),
            (0x02, "高電圧保護"),
            (0x04, "ドライバ過熱"),
            (0x08, "モータ過熱"),
        ];
        let mut out: Vec<&str> = BITS
            .iter()
            .filter(|(bit, _)| self.error_raw & bit != 0)
            .map(|(_, name)| *name)
            .collect();
        if self.error_raw & !0x0F != 0 {
            out.push("未定義ビット");
        }
        out.join(" + ")
    }
}

/// ドライバの PID ゲイン（`0x30`）。各ループの Kp・Ki。
///
/// **`Kd` は無い。** この基板が応答する旧インタフェースは Kp/Ki だけを
/// 返す（documented な `0xC0` は Kd も持つが、この基板は応答しない）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct JointPids {
    pub position_kp: u8,
    pub position_ki: u8,
    pub speed_kp: u8,
    pub speed_ki: u8,
    pub current_kp: u8,
    pub current_ki: u8,
}

/// バススレッドへの制御要求。周期指令とは別経路にしてある。
///
/// 位置指令は「最新の 1 個だけが意味を持つ」ので共有スロットで上書きするが、
/// enable / ゼロ出しは 1 回ずつ確実に届く必要があるのでキューにする。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BusRequest {
    /// クローズドループ有効化（`0x88`）。3 軸まとめて。
    Enable,
    /// 停止（`0x81`）。モータは脱力する。3 軸まとめて。
    Disable,
    /// モータの電源 ON マルチターンフレームとの差を読み直す。
    ///
    /// **かつては `rezero`（現在位置をソフトゼロにする）だった。** いまは
    /// モータ自身の絶対角を基準にしているので、原点を置き直すことはない。
    /// 起動時は自動で確立されるため、通常は送る必要がない。
    Zero,
    /// 1 軸だけ投入する。校正で 1 軸ずつ動かすため。
    EnableJoint(usize),
    /// 1 軸だけ停止する。
    DisableJoint(usize),
    /// モータのマルチターンカウンタを 0 に戻す（`0x95`）。
    ///
    /// **モータ電源の OFF/ON と同じ効果**をマルチターンフレームにだけ与える。
    /// A7Z とモータ電源が同一系統で切り分けられない開発環境で、電源を落とさずに
    /// 「電源を入れ直した状態」を作るためのもの。
    ///
    /// ROM には何も書かない（`0x19` とは別物）。実行後はフレームを張り直す。
    ClearMultiTurn,
    /// 1 軸だけマルチターンカウンタを 0 に戻す。検証用。
    ClearMultiTurnJoint(usize),
    /// ドライバを再起動する（`0x07`）。**電源再投入と等価。**
    ///
    /// **マルチターン原点がリセットされる。** 実行後の原点は「そのときの
    /// 姿勢」になるので、`zero_pose_rad` はその姿勢を基準に測り直しになる
    /// （伏せ姿勢で実行すれば従来どおりの約束で維持できる）。
    ///
    /// **RS485 マニュアルに載っていないコマンド**（記載は CAN §29 のみ）。
    /// 応答も返らないので、成否は状態を読み直して確かめる。
    Restart,
    /// 1 軸だけ再起動する。検証用。
    RestartJoint(usize),
    /// ドライバの異常フラグを消す（`0x9B`）。3 軸まとめて。
    ///
    /// **原因が残っている間は消えない。** マニュアル §2 が
    /// 「the error flags cannot be cleared while the motor state has not yet
    /// returned to normal」と明記している。低電圧保護ならバス電圧を戻して
    /// から投げること。結果は [`LegBus::status`] を読み直して確かめる。
    ClearError,
    /// ドライバの PID ゲインを 3 軸ぶん読む（`0x30`）。**読むだけ。**
    ///
    /// 位置ループ / 速度ループ / 電流ループの Kp・Ki（各 u8）。
    /// コンプライアンスを効かせたいとき、まず現状を知るために使う。
    ///
    /// **`0x30` は RS485 マニュアルに記載が無い。** documented な `0xC0` に
    /// この基板が応答しない一方、`0x30` には応答する実績がある
    /// （`motor_map.md`）。書き込み側は未実装。
    ///
    /// 結果は [`LegBus::pids`] で取る。
    ReadPid,
    /// 単回転絶対角（`0x94`）を 3 軸ぶん読む。**読むだけ。**
    ///
    /// `0x92`（マルチターン）が電源投入時の姿勢を 0 とするのに対し、こちらは
    /// ドライバの ROM に入ったエンコーダゼロが基準なので、**電源 OFF/ON を
    /// またいで同じ値になる**。代わりに 1 モータ回転で一周するので、
    /// 何回転目かは別の手段（既知の電源投入姿勢など）で決める必要がある。
    ///
    /// 結果は [`LegBus::single_turn`] で取る。
    ReadSingleTurn,
}

#[derive(Debug, Default)]
struct BusSlot {
    cmd: Mutex<[JointCommand; 3]>,
    state: Mutex<[JointState; 3]>,
    stats: Mutex<BusStats>,
    status: Mutex<[JointStatus; 3]>,
    /// 直近のエラー文言（表示用）。エラーがなければ空。
    last_error: Mutex<String>,
    /// ゼロ出し済みか。位置指令はこれが立つまで送らない。
    anchored: AtomicBool,
    /// 直近の [`BusRequest::ReadSingleTurn`] の結果（0.01°/LSB, 0..=35999）。
    /// 読めなかった軸は `None`。
    single_turn: Mutex<[Option<u32>; 3]>,
    /// 直近の [`BusRequest::ReadPid`] の結果。読めなかった軸は `None`。
    pids: Mutex<[Option<JointPids>; 3]>,
}

/// 1 本の脚バスへのハンドル。
pub struct LegBus {
    leg: LegSlot,
    port: String,
    slot: Arc<BusSlot>,
    requests: Sender<BusRequest>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl LegBus {
    pub fn leg(&self) -> LegSlot {
        self.leg
    }

    /// 実際に開いているデバイスパス。
    pub fn port(&self) -> &str {
        &self.port
    }

    /// 3 軸ぶんの指令を差し替える。次の周回から反映される。
    pub fn set_commands(&self, cmds: [JointCommand; 3]) {
        *lock(&self.slot.cmd) = cmds;
    }

    /// 最新のフィードバック。
    pub fn state(&self) -> [JointState; 3] {
        *lock(&self.slot.state)
    }

    /// 直近の [`BusRequest::ReadSingleTurn`] で読んだ単回転絶対角
    /// （0.01°/LSB, 0..=35999）。まだ読んでいない・読めなかった軸は `None`。
    pub fn single_turn(&self) -> [Option<u32>; 3] {
        *lock(&self.slot.single_turn)
    }

    /// 直近の [`BusRequest::ReadPid`] で読んだゲイン。
    pub fn pids(&self) -> [Option<JointPids>; 3] {
        *lock(&self.slot.pids)
    }

    pub fn stats(&self) -> BusStats {
        *lock(&self.slot.stats)
    }

    /// 3 軸の電圧・温度・エラービット。
    pub fn status(&self) -> [JointStatus; 3] {
        *lock(&self.slot.status)
    }

    /// この脚のどれかに異常ビットが立っているか。
    pub fn faulted(&self) -> bool {
        self.status().iter().any(|s| s.faulted())
    }

    pub fn last_error(&self) -> String {
        lock(&self.slot.last_error).clone()
    }

    /// ゼロ出し済みか。false の間、位置指令はバススレッド側で握り潰される。
    pub fn is_anchored(&self) -> bool {
        self.slot.anchored.load(Ordering::Relaxed)
    }

    /// 制御要求を送る。スレッドが死んでいる場合はエラー。
    pub fn request(&self, req: BusRequest) -> Result<()> {
        self.requests
            .send(req)
            .map_err(|_| Error::Config(format!("{} のバススレッドが停止しています", self.port)))
    }
}

impl LegBus {
    /// 停止フラグを立てるだけ。join はしない。
    ///
    /// [`LegArray`] が 4 本まとめて畳むときに、**先に全部へ知らせる**ために
    /// 使う。1 本ずつ「立てて join」を繰り返すと停止が直列化する。
    fn signal_stop(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

impl Drop for LegBus {
    /// スレッドを止めてから戻る。
    ///
    /// 「落ちるアプリがモータを駆動したまま生き残る」ことがないように、
    /// 停止フラグを立てて join する（`misa-actuator-core::Session` と同じ方針）。
    fn drop(&mut self) {
        self.signal_stop();
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

/// 脚 4 本ぶん。
pub struct LegArray {
    buses: [LegBus; 4],
}

impl Drop for LegArray {
    /// **先に 4 本すべてへ停止を知らせてから**、各バスの drop に join させる。
    ///
    /// これが無いと `buses` が順に drop され、1 本ずつ「フラグを立てて join」に
    /// なる。バスが停止に気付くのは 1 周の切れ目なので、モータが無応答で 1 周が
    /// 長いときに**停止時間が 4 倍**になる（実測: 1 周 10 秒 → 終了に 41 秒）。
    fn drop(&mut self) {
        for bus in &self.buses {
            bus.signal_stop();
        }
    }
}

impl LegArray {
    /// 設定に従って 4 本のポートを開き、バススレッドを起こす。
    ///
    /// 途中で失敗した場合、それまでに開いたバスは drop で畳まれる。
    pub fn connect(cfg: &HardwareConfig) -> Result<Self> {
        Self::connect_with(cfg, &PortMap::discover()?)
    }

    /// 事前に取った探索結果を使って開く。
    pub fn connect_with(cfg: &HardwareConfig, map: &PortMap) -> Result<Self> {
        let mut buses: Vec<LegBus> = Vec::with_capacity(4);
        for leg in LegSlot::ALL {
            let bus_cfg = cfg
                .bus_for(leg)
                .ok_or_else(|| Error::Config(format!("脚 {} の設定がありません", leg.prefix())))?;
            buses.push(LegBus::spawn(leg, bus_cfg, &cfg.legs, map)?);
        }
        let buses: [LegBus; 4] = buses
            .try_into()
            .map_err(|_| Error::Config("脚バスの本数が 4 ではありません".into()))?;
        Ok(Self { buses })
    }

    pub fn bus(&self, leg: LegSlot) -> &LegBus {
        &self.buses[leg.index()]
    }

    pub fn buses(&self) -> &[LegBus; 4] {
        &self.buses
    }

    /// 全バスへ同じ制御要求を送る。
    pub fn request_all(&self, req: BusRequest) -> Result<()> {
        for bus in &self.buses {
            bus.request(req)?;
        }
        Ok(())
    }

    /// 12 軸ぶんの指令をまとめて書く。並びは `[leg][hip, thigh, calf]`。
    pub fn set_all(&self, cmds: &[[JointCommand; 3]; 4]) {
        for (bus, c) in self.buses.iter().zip(cmds.iter()) {
            bus.set_commands(*c);
        }
    }

    /// 12 軸ぶんの最新値。
    pub fn states(&self) -> [[JointState; 3]; 4] {
        [
            self.buses[0].state(),
            self.buses[1].state(),
            self.buses[2].state(),
            self.buses[3].state(),
        ]
    }

    /// 全軸が直近のトランザクションに成功しているか。
    pub fn all_ok(&self) -> bool {
        self.states().iter().flatten().all(|s| s.ok)
    }

    /// 異常ビットが立っている軸を `(脚, 軸番号, 状態)` で返す。
    pub fn faults(&self) -> Vec<(LegSlot, usize, JointStatus)> {
        let mut out = Vec::new();
        for bus in &self.buses {
            for (k, st) in bus.status().iter().enumerate() {
                if st.faulted() {
                    out.push((bus.leg(), k, *st));
                }
            }
        }
        out
    }

    /// 全バスがゼロ出し済みか。
    pub fn all_anchored(&self) -> bool {
        self.buses.iter().all(|b| b.is_anchored())
    }

    /// 全バスがゼロ出しを終えるまで待つ。
    pub fn wait_anchored(&self, timeout: Duration) -> Result<()> {
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            if self.all_anchored() {
                return Ok(());
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        Err(Error::Timeout {
            what: "脚のゼロ出し".into(),
            timeout,
        })
    }

    /// 12 軸すべてが**一度でも読めた**状態になるまで待つ。
    ///
    /// # なぜ [`Self::wait_anchored`] では足りないか
    ///
    /// `wait_anchored` は `establish_frame` が通った時点で返るが、そこは
    /// **共有状態の [`JointState`] をまだ書いていない**。書くのは次の
    /// トランザクション周回。したがって `wait_anchored` の直後に
    /// [`Self::states`] を読むと、既定値（`position_rad = 0.0`, `ok = false`）
    /// が返ることがある。
    ///
    /// **脱力からの遷移はその実測値を始点にする。** 0 を掴むと、実際には
    /// −2.7 rad にある関節の目標がいきなり 0 になり、最初の起立で暴れる。
    /// 制御ループへ入る前にこれを待つこと。
    pub fn wait_first_read(&self, timeout: Duration) -> Result<()> {
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            if self.all_ok() {
                return Ok(());
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        Err(Error::Timeout {
            what: "12 軸の初回読み出し".into(),
            timeout,
        })
    }
}

/// モデル ↔ モータの座標変換と可動域を持つ 1 軸ぶんの写像。
#[derive(Debug, Clone, Copy)]
struct JointMap {
    sign: f64,
    zero_pose_rad: f64,
    min_rad: f64,
    max_rad: f64,
}

impl JointMap {
    fn new(m: &MotorConfig) -> Self {
        Self {
            sign: m.sign,
            zero_pose_rad: m.zero_pose_rad,
            min_rad: m.min_rad,
            max_rad: m.max_rad,
        }
    }

    /// モデル角 → モータ出力軸角。可動域でクランプしてから変換する。
    fn to_motor(self, q_model: f64) -> f64 {
        let clamped = q_model.clamp(self.min_rad, self.max_rad);
        self.sign * (clamped - self.zero_pose_rad)
    }

    /// モータ出力軸角 → モデル角。
    fn to_model(self, q_motor: f64) -> f64 {
        self.sign * q_motor + self.zero_pose_rad
    }

    /// 速度・トルクは符号だけ。
    fn rate_to_model(self, v_motor: f64) -> f64 {
        self.sign * v_motor
    }
}

impl LegBus {
    /// **1 本だけ**開く。校正のように 1 脚しか触らない用途向け。
    ///
    /// [`LegArray::connect`] は 4 本まとめて開くので、1 脚を調べるだけでも
    /// 残り 3 本のポートを掴んでしまう。触る範囲は要る分だけにしたい。
    pub fn open(cfg: &HardwareConfig, leg: LegSlot, map: &PortMap) -> Result<Self> {
        let bus_cfg = cfg
            .bus_for(leg)
            .ok_or_else(|| Error::Config(format!("脚 {} の設定がありません", leg.prefix())))?;
        Self::spawn(leg, bus_cfg, &cfg.legs, map)
    }

    /// 探索から自分でやる版。
    pub fn open_alone(cfg: &HardwareConfig, leg: LegSlot) -> Result<Self> {
        Self::open(cfg, leg, &PortMap::discover()?)
    }

    fn spawn(
        leg: LegSlot,
        bus_cfg: &LegBusConfig,
        legs: &LegsConfig,
        map: &PortMap,
    ) -> Result<Self> {
        let port = bus_cfg.port.resolve_with(map)?;
        let timeout = Duration::from_millis(legs.response_timeout_ms);
        let driver = Rs485Driver::open(&port, legs.baud, timeout).map_err(|e| match e {
            lkmotor_driver::Error::SerialPort(source) => Error::OpenPort {
                port: port.clone(),
                source,
            },
            other => Error::Motor {
                port: port.clone(),
                motor_id: 0,
                source: other,
            },
        })?;

        // **減速比は軸ごとに引く。** calf だけベルト駆動で歯数比 28:18 が
        // 内蔵減速機に上乗せされるため、バス共通の値では calf が 47% ずれる。
        let motor_cfg_for = |ratio: f64| match legs.torque_constant_nm_per_a {
            Some(kt) => LkMotorConfig::new(ratio as f32, kt as f32),
            // Kt 未知のうちは電流 (A) をそのままトルク API に通す。
            None => LkMotorConfig::current_units(ratio as f32),
        };

        let mut motors = Vec::with_capacity(3);
        let mut maps = Vec::with_capacity(3);
        for m in &bus_cfg.motors {
            let id = MotorId::new(m.id)
                .ok_or_else(|| Error::Config(format!("モータ id {} は範囲外です", m.id)))?;
            motors.push(Motor::new(
                id,
                motor_cfg_for(m.gear_ratio_or(legs.gear_ratio)),
            ));
            maps.push(JointMap::new(m));
        }
        let motors: [Motor; 3] = motors
            .try_into()
            .map_err(|_| Error::Config("脚あたりのモータは 3 個です".into()))?;
        let maps: [JointMap; 3] = maps
            .try_into()
            .map_err(|_| Error::Config("脚あたりのモータは 3 個です".into()))?;

        let slot = Arc::new(BusSlot::default());
        let stop = Arc::new(AtomicBool::new(false));
        let (tx, rx) = mpsc::channel();

        let worker = BusWorker {
            leg,
            port: port.clone(),
            driver,
            motors,
            maps,
            period: Duration::from_secs_f64(1.0 / legs.bus_rate_hz),
            default_max_speed: legs.default_max_speed_rad_s,
            max_target_rate: legs.max_target_rate_rad_s,
            issued: [None; 3],
            status_interval: Duration::from_millis(legs.status_interval_ms),
            next_status: Instant::now(),
            status_cursor: 0,
            frame_offset: [0.0; 3],
            frame_ready: false,
            slot: Arc::clone(&slot),
            stop: Arc::clone(&stop),
            requests: rx,
        };
        let thread = std::thread::Builder::new()
            .name(format!("leg-{}", leg.prefix()))
            .spawn(move || worker.run())?;

        Ok(LegBus {
            leg,
            port,
            slot,
            requests: tx,
            stop,
            thread: Some(thread),
        })
    }
}

struct BusWorker {
    leg: LegSlot,
    port: String,
    driver: Rs485Driver,
    motors: [Motor; 3],
    maps: [JointMap; 3],
    period: Duration,
    default_max_speed: f64,
    /// 目標角を 1 秒あたり何 rad まで動かしてよいか。0 で無制限。
    max_target_rate: f64,
    /// 実際に送っている目標角（モデル座標系）。スルーレート制限の状態。
    /// `None` は「まだ一度も位置指令を出していない」。
    issued: [Option<f64>; 3],
    /// State1 を読む間隔。0 で読まない。
    status_interval: Duration,
    next_status: Instant,
    /// 次に State1 を読む軸。1 周回で 1 軸だけ読み、帯域を食わないようにする。
    status_cursor: usize,
    /// ホスト追従角 → **モータの電源 ON マルチターンフレーム**への差 (rad)。
    ///
    /// `Motor::measure`（`0x9C`）が返す位置は、ホスト側が `raw_origin` から
    /// 積算した相対値でしかない。一方 `0x92` はモータが電源 ON から数えている
    /// 絶対角で、**アプリを再起動しても変わらない**。起動時に 1 回だけ両方を
    /// 読んでその差を覚えておけば、以降は `0x9C` だけで絶対角が出せる。
    frame_offset: [f64; 3],
    /// [`Self::establish_frame`] が成功したか。
    frame_ready: bool,
    slot: Arc<BusSlot>,
    stop: Arc<AtomicBool>,
    requests: Receiver<BusRequest>,
}

impl BusWorker {
    fn run(mut self) {
        let mut next = Instant::now();
        let mut window_start = Instant::now();
        let mut window_ticks = 0u64;
        let mut last_cycle = Instant::now();

        while !self.stop.load(Ordering::Relaxed) {
            match self.drain_requests() {
                Ok(()) => {}
                Err(Disconnected) => break,
            }

            if !self.frame_ready && !self.establish_frame() {
                // 読めるようになるまで毎周期試す。モータ電源が後から入る
                // 使い方もあるので、ここで諦めない。
                let now = Instant::now();
                next = next.max(now) + self.period;
                std::thread::sleep(self.period);
                continue;
            }

            let cycle_start = Instant::now();
            // スルーレート制限に使う実経過時間。目標周期ではなく実測を使う
            // のは、バスが遅れている間に目標だけ規定どおり進んでしまうと
            // 制限の意味が無くなるため。
            let dt = last_cycle.elapsed().as_secs_f64().min(0.5);
            last_cycle = Instant::now();
            let cmds = *lock(&self.slot.cmd);
            let mut states = [JointState::default(); 3];
            let mut errors = 0u64;

            for k in 0..3 {
                // 軸の切れ目でも停止を見る。無応答のモータが混ざると 1 軸で
                // 秒単位かかることがあり、周期の切れ目まで待つと Ctrl-C が
                // そのぶん遅れる。
                if self.stop.load(Ordering::Relaxed) {
                    break;
                }
                match self.step_joint(k, &cmds[k], dt) {
                    Ok(state) => states[k] = state,
                    Err(e) => {
                        errors += 1;
                        // 直近の値を残したまま ok だけ落とす。位置が 0 に
                        // 化けて上位が「原点にいる」と誤認するほうが危険。
                        states[k] = JointState {
                            ok: false,
                            ..lock(&self.slot.state)[k]
                        };
                        *lock(&self.slot.last_error) = format!("{} 軸{k}: {e}", self.leg.prefix());
                    }
                }
            }
            *lock(&self.slot.state) = states;
            self.poll_status();

            let cycle = cycle_start.elapsed().as_secs_f64();
            window_ticks += 1;
            {
                let mut st = lock(&self.slot.stats);
                st.ticks += 1;
                st.errors += errors;
                st.last_cycle_s = cycle;
                st.worst_cycle_s = st.worst_cycle_s.max(cycle);
                let elapsed = window_start.elapsed().as_secs_f64();
                if elapsed >= 1.0 {
                    st.rate_hz = window_ticks as f64 / elapsed;
                    window_ticks = 0;
                    window_start = Instant::now();
                }
            }

            // 目標周期に満たなければ休む。超過している場合は詰めずに
            // 「今から次の周期」に置き直す（遅れを取り返そうとして
            // バスを叩き続けると、遅れの原因ごと悪化させるだけ）。
            next += self.period;
            let now = Instant::now();
            if next > now {
                std::thread::sleep(next - now);
            } else {
                next = now;
            }
        }

        // 出るときは脱力させる。ただし**応答が取れている軸だけ**。
        //
        // 届かない軸へ投げても意味が無いうえに高くつく。モータ電源が OFF だと
        // CH348 の write が 1 回おきに約 5.2 秒詰まる（実測。pyserial で
        // 13 バイトを連続送信すると 0.3ms → 5203ms → 0.2ms → 5120ms）。
        // 3 軸ぶん投げると停止に 15 秒級かかる。
        //
        // 安全性は損なわない。こちらから届かないモータは、こちらが励磁して
        // いるモータではない（電源が無いか経路が切れている）。届く軸には
        // 従来どおり必ず disable を送る。
        let reachable = *lock(&self.slot.state);
        for (k, motor) in self.motors.iter_mut().enumerate() {
            if reachable[k].ok {
                let _ = motor.disable(&mut self.driver);
            } else {
                log::debug!(
                    "{} 軸{k} は応答が無いので disable を省略しました",
                    self.leg.prefix()
                );
            }
        }
        log::info!(
            "{} ({}) のバススレッドを停止しました",
            self.leg.prefix(),
            self.port
        );
    }

    /// モータの電源 ON マルチターンフレームとの差を求める。
    ///
    /// 軸ごとに `0x92`（絶対角）と `0x9C`（ホスト追従角）を 1 回ずつ読み、
    /// その差を [`Self::frame_offset`] に置く。**起動時に 1 回だけ**で、
    /// 制御周期のトランザクション数は増えない。
    ///
    /// これがあると「アプリ起動のたびに原点が変わる」問題が消える。`rezero`
    /// はホスト側にソフト原点を置くだけなので、プロセスが死ぬと失われ、次の
    /// 起動では**そのときの姿勢**が原点になっていた。モータ自身は電源が入って
    /// いる限りマルチターン角を保持しているので、そちらを基準にすればよい。
    fn establish_frame(&mut self) -> bool {
        let mut offsets = [0.0f64; 3];
        for (k, offset) in offsets.iter_mut().enumerate() {
            let abs = match self.motors[k].read_absolute_angle(&mut self.driver) {
                Ok(v) => v as f64,
                Err(e) => {
                    *lock(&self.slot.last_error) =
                        format!("{} 軸{k}: 絶対角の読み出しに失敗: {e}", self.leg.prefix());
                    return false;
                }
            };
            // `measure` は turn 追従の初期化も兼ねる（`prev_raw` を埋める）。
            let rel = match self.motors[k].measure(&mut self.driver) {
                Ok(fb) => fb.position_rad as f64,
                Err(e) => {
                    *lock(&self.slot.last_error) =
                        format!("{} 軸{k}: 状態の読み出しに失敗: {e}", self.leg.prefix());
                    return false;
                }
            };
            *offset = abs - rel;
        }
        self.frame_offset = offsets;
        self.frame_ready = true;
        self.slot.anchored.store(true, Ordering::Relaxed);
        log::info!(
            "{} のマルチターンフレームを確立しました（オフセット {:+.4} {:+.4} {:+.4} rad）",
            self.leg.prefix(),
            offsets[0],
            offsets[1],
            offsets[2]
        );
        true
    }

    /// 1 軸ぶんのトランザクション。
    fn step_joint(
        &mut self,
        k: usize,
        cmd: &JointCommand,
        dt: f64,
    ) -> lkmotor_driver::Result<JointState> {
        let map = self.maps[k];
        let fb = match cmd.mode {
            // フレーム確立前の位置指令は送らない。まだ絶対角との対応が
            // 付いていないので、送ると見当違いの位置へ動く。状態読みに
            // 落としておけば起動直後もフィードバックは回る。
            JointMode::Position if self.frame_ready => {
                let speed = if cmd.max_speed_rad_s > 0.0 {
                    cmd.max_speed_rad_s
                } else {
                    self.default_max_speed
                };
                let target = self.slew(k, cmd.position_rad, dt);
                // **絶対マルチターン指令（0xA4）**。ソフト原点は使わない。
                self.motors[k].set_position_absolute(
                    &mut self.driver,
                    map.to_motor(target) as f32,
                    speed as f32,
                )?
            }
            JointMode::Torque => {
                // モデル座標のトルクをモータ座標へ。符号だけ。
                self.issued[k] = None;
                self.motors[k].set_torque(&mut self.driver, (map.sign * cmd.torque_nm) as f32)?
            }
            JointMode::Idle | JointMode::Position => {
                // 位置制御をしていない間は履歴を捨てる。次に位置制御へ入る
                // ときは「今いるところ」から出発させたい。
                self.issued[k] = None;
                self.motors[k].measure(&mut self.driver)?
            }
        };
        Ok(JointState {
            // ホスト追従角に起動時の差を足して絶対角に戻す。
            position_rad: map.to_model(fb.position_rad as f64 + self.frame_offset[k]),
            velocity_rad_s: map.rate_to_model(fb.velocity_rad_per_s as f64),
            torque_nm: map.rate_to_model(fb.torque_nm as f64),
            temperature_c: fb.temperature_c as f64,
            ok: true,
        })
    }

    /// 目標角の変化を `max_target_rate` [rad/s] で頭打ちにする。
    ///
    /// モータ側の速度上限（`set_position` の第 2 引数）とは別物。あちらは
    /// 「軸が何 rad/s で回るか」で、こちらは「**目標そのもの**が何 rad/s で
    /// 動くか」。歩容の切り替えや IK のクランプで目標が跳んだとき、あちらは
    /// 上限速度で追いに行ってしまう。ここで目標側を鈍らせておくと、跳びが
    /// そのまま脚の飛び出しにならない。
    ///
    /// 位置制御に入った最初の 1 回は**実測位置から**出発する。前回の目標を
    /// 覚えたままだと、脱力中に手で動かされた分をいきなり戻しに行く。
    fn slew(&mut self, k: usize, want: f64, dt: f64) -> f64 {
        let from = match self.issued[k] {
            Some(prev) => prev,
            None => lock(&self.slot.state)[k].position_rad,
        };
        let target = if self.max_target_rate > 0.0 && dt > 0.0 {
            let step = self.max_target_rate * dt;
            from + (want - from).clamp(-step, step)
        } else {
            want
        };
        self.issued[k] = Some(target);
        target
    }

    /// `status_interval` ごとに 1 軸だけ State1 を読む。
    ///
    /// 3 軸まとめて読むと 1 周回のトランザクションが倍になり、周期が目に見えて
    /// 落ちる。1 軸ずつ回せば 1 周あたりの追加は 1 トランザクションで済み、
    /// 3 × `status_interval` で全軸が更新される。
    fn poll_status(&mut self) {
        if self.status_interval.is_zero() || Instant::now() < self.next_status {
            return;
        }
        self.next_status = Instant::now() + self.status_interval;
        let k = self.status_cursor;
        self.status_cursor = (self.status_cursor + 1) % 3;
        match self.motors[k].read_status(&mut self.driver) {
            Ok(st) => {
                let entry = JointStatus {
                    voltage_v: st.voltage_v as f64,
                    temperature_c: st.temperature_c as f64,
                    error_raw: st.error.raw(),
                    valid: true,
                };
                let previous = lock(&self.slot.status)[k];
                lock(&self.slot.status)[k] = entry;
                // 新しく立った異常だけを言う。毎秒同じ行を出しても読まれない。
                if entry.faulted() && entry.error_raw != previous.error_raw {
                    log::error!(
                        "{} 軸{k} で異常ビット 0x{:02X}（{:.1} V / {:.0} °C）",
                        self.leg.prefix(),
                        entry.error_raw,
                        entry.voltage_v,
                        entry.temperature_c
                    );
                }
            }
            Err(e) => log::debug!("{} 軸{k} の State1 読み出しに失敗: {e}", self.leg.prefix()),
        }
    }

    fn drain_requests(&mut self) -> std::result::Result<(), Disconnected> {
        loop {
            match self.requests.try_recv() {
                Ok(req) => self.handle_request(req),
                Err(TryRecvError::Empty) => return Ok(()),
                Err(TryRecvError::Disconnected) => return Err(Disconnected),
            }
        }
    }

    fn handle_request(&mut self, req: BusRequest) {
        // 軸単位の要求はその軸だけ。校正で 1 軸ずつ動かすときに、
        // 残り 2 軸をクローズドループへ入れてしまわないため。
        let single;
        let joints: &[usize] = match req {
            BusRequest::EnableJoint(k)
            | BusRequest::DisableJoint(k)
            | BusRequest::RestartJoint(k)
            | BusRequest::ClearMultiTurnJoint(k) => {
                if k >= 3 {
                    log::warn!("{} に軸 {k} はありません", self.leg.prefix());
                    return;
                }
                single = [k];
                &single
            }
            _ => &[0, 1, 2],
        };
        for &k in joints {
            let result = match req {
                BusRequest::Enable | BusRequest::EnableJoint(_) => {
                    self.motors[k].enable(&mut self.driver)
                }
                BusRequest::Disable | BusRequest::DisableJoint(_) => {
                    self.issued[k] = None;
                    self.motors[k].disable(&mut self.driver)
                }
                // Zero はフレームの張り直し。1 軸ずつではなく 3 軸まとめて
                // やる必要があるので、ここでは何もせず下で処理する。
                BusRequest::Zero => Ok(()),
                BusRequest::ClearMultiTurn | BusRequest::ClearMultiTurnJoint(_) => {
                    self.motors[k].clear_multi_turn(&mut self.driver)
                }
                BusRequest::Restart | BusRequest::RestartJoint(_) => {
                    self.motors[k].restart(&mut self.driver)
                }
                BusRequest::ClearError => {
                    // **`0x9B` の応答を状態として使ってはいけない。**
                    //
                    // マニュアル §2 は「応答は status1 と同じ」としか言わず、
                    // それが消す前か後かを書いていない。実機で測ると**消す前**
                    // だった: 応答は異常が立ったまま返るのに、直後に読み直すと
                    // 消えている（2026-08-22）。
                    //
                    // これを「クリア失敗」と読んで「時間が経つと自然に消える」
                    // という誤った結論を出した。**送ったら読み直す。**
                    self.motors[k]
                        .clear_error(&mut self.driver)
                        .and_then(|_| self.motors[k].read_status(&mut self.driver))
                        .map(|st| {
                            lock(&self.slot.status)[k] = JointStatus {
                                voltage_v: st.voltage_v as f64,
                                temperature_c: st.temperature_c as f64,
                                error_raw: st.error.raw(),
                                valid: true,
                            };
                        })
                }
                BusRequest::ReadPid => {
                    lock(&self.slot.pids)[k] = None;
                    let id = self.motors[k].id();
                    <Rs485Driver as lkmotor_driver::LkCommands>::read_legacy_pids(
                        &mut self.driver,
                        id,
                    )
                    .map(|p| {
                        lock(&self.slot.pids)[k] = Some(JointPids {
                            position_kp: p.position_kp,
                            position_ki: p.position_ki,
                            speed_kp: p.speed_kp,
                            speed_ki: p.speed_ki,
                            current_kp: p.current_kp,
                            current_ki: p.current_ki,
                        });
                    })
                }
                // 失敗した軸に古い値を残さないよう、読む前に落とす。
                BusRequest::ReadSingleTurn => {
                    lock(&self.slot.single_turn)[k] = None;
                    self.motors[k]
                        .read_single_turn_angle_centideg(&mut self.driver)
                        .map(|v| lock(&self.slot.single_turn)[k] = Some(v))
                }
            };
            if let Err(e) = result {
                *lock(&self.slot.last_error) = format!("{} 軸{k} {req:?}: {e}", self.leg.prefix());
                log::warn!("{} 軸{k} の {req:?} に失敗: {e}", self.leg.prefix());
                if req == BusRequest::Zero {
                    // 3 軸そろって成功していなければアンカー済みにしない。
                    self.slot.anchored.store(false, Ordering::Relaxed);
                    return;
                }
            }
        }
        let cleared = matches!(
            req,
            BusRequest::ClearMultiTurn
                | BusRequest::ClearMultiTurnJoint(_)
                | BusRequest::Restart
                | BusRequest::RestartJoint(_)
        );
        if req == BusRequest::Zero || cleared {
            // Zero は**モータには何も書かない**。マルチターンフレームとの差を
            // 読み直すだけ。電源を入れ直した後など、フレームがずれた
            // 可能性があるときに使う。
            //
            // ClearMultiTurn / Restart はモータ側の原点が動いたので、当然
            // フレームも張り直す必要がある。
            //
            // **Restart は応答が返らない。** 送った直後はドライバが再起動
            // 中で、`establish_frame` の読み出しに答えられない。少し待つ。
            self.frame_ready = false;
            self.slot.anchored.store(false, Ordering::Relaxed);
            if matches!(req, BusRequest::Restart | BusRequest::RestartJoint(_)) {
                std::thread::sleep(Duration::from_millis(500));
            }
            if cleared {
                log::warn!(
                    "{} のマルチターンカウンタを 0 に戻しました。\
                     **zero_pose_rad は今の姿勢を基準に測り直すこと**",
                    self.leg.prefix()
                );
            }
            self.establish_frame();
        }
        // Disable ではアンカーを落とさない。`Motor::set_position` の基準は
        // モータ自身のマルチターン角（`0x92`）に置いた絶対値なので、脱力して
        // 外力で動かされてもゼロ点はずれない。ここで落としてしまうと、
        // 再起立のたびに「今いる姿勢」でゼロを引き直すことになり、
        // 校正姿勢とは無関係な原点が入ってしまう。
    }
}

struct Disconnected;

/// mutex の poison を握り潰す。
///
/// 制御スレッドのどこかが panic したからといって、他のバスへの指令を
/// 止めるほうが安全とは限らない（脚が 1 本止まった四足は倒れる）。
/// `misa_actuator::Shared` と同じ方針。
fn lock<T>(m: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|p| p.into_inner())
}

#[cfg(test)]
mod tests {

    #[test]
    fn error_bits_are_spelled_out_not_left_as_hex() {
        // 生値 0x01 だけ出しても現場では分からない。対処が違う。
        let low = JointStatus {
            error_raw: 0x01,
            valid: true,
            ..Default::default()
        };
        assert_eq!(low.describe(), "低電圧保護");
        let both = JointStatus {
            error_raw: 0x0C,
            valid: true,
            ..Default::default()
        };
        assert_eq!(both.describe(), "ドライバ過熱 + モータ過熱");
        // 未読・正常では何も言わない。
        assert_eq!(JointStatus::default().describe(), "");
        let ok = JointStatus {
            valid: true,
            ..Default::default()
        };
        assert_eq!(ok.describe(), "");
        // 知らないビットも取りこぼさない。
        let odd = JointStatus {
            error_raw: 0x80,
            valid: true,
            ..Default::default()
        };
        assert_eq!(odd.describe(), "未定義ビット");
    }
    use super::*;

    fn map(sign: f64, zero: f64) -> JointMap {
        JointMap {
            sign,
            zero_pose_rad: zero,
            min_rad: -3.0,
            max_rad: 3.0,
        }
    }

    #[test]
    fn model_motor_transform_round_trips() {
        for (sign, zero) in [(1.0, 0.0), (-1.0, 0.0), (1.0, 0.7), (-1.0, -0.4)] {
            let m = map(sign, zero);
            for q in [-1.0, 0.0, 0.25, 1.5] {
                let back = m.to_model(m.to_motor(q));
                assert!((back - q).abs() < 1e-12, "sign={sign} zero={zero} q={q}");
            }
        }
    }

    #[test]
    fn zero_pose_angle_maps_to_motor_zero() {
        // ゼロ出しした姿勢のモデル角は、モータ側では 0 でなければならない。
        let m = map(-1.0, 1.3);
        assert!(m.to_motor(1.3).abs() < 1e-12);
    }

    #[test]
    fn commands_are_clamped_to_the_joint_limits() {
        let m = JointMap {
            sign: 1.0,
            zero_pose_rad: 0.0,
            min_rad: -0.5,
            max_rad: 0.5,
        };
        assert_eq!(m.to_motor(10.0), 0.5);
        assert_eq!(m.to_motor(-10.0), -0.5);
    }

    /// スルーレート制限だけを取り出した参照実装。`BusWorker::slew` と同じ式で、
    /// ワーカー（シリアルポートを持つ）を組まずに性質を試験するためのもの。
    fn slew_step(from: f64, want: f64, rate: f64, dt: f64) -> f64 {
        if rate > 0.0 && dt > 0.0 {
            let step = rate * dt;
            from + (want - from).clamp(-step, step)
        } else {
            want
        }
    }

    #[test]
    fn the_slew_limit_caps_how_far_the_target_moves_per_tick() {
        // 3 rad/s × 2 ms = 0.006 rad。1.5 rad の跳びでもこれしか進まない。
        let out = slew_step(0.0, 1.5, 3.0, 0.002);
        assert!((out - 0.006).abs() < 1e-12, "{out}");
    }

    #[test]
    fn the_slew_limit_is_symmetric_and_converges() {
        let mut q = 0.0;
        for _ in 0..1000 {
            q = slew_step(q, -1.0, 3.0, 0.002);
        }
        assert!((q + 1.0).abs() < 1e-9, "{q}");
    }

    #[test]
    fn a_small_move_passes_through_untouched() {
        // 制限より小さい変化はそのまま通る（余計な遅れを入れない）。
        let out = slew_step(0.0, 0.001, 3.0, 0.002);
        assert!((out - 0.001).abs() < 1e-12, "{out}");
    }

    #[test]
    fn a_zero_rate_disables_the_limit() {
        assert_eq!(slew_step(0.0, 1.5, 0.0, 0.002), 1.5);
    }

    #[test]
    fn a_status_with_no_reading_is_not_a_fault() {
        // 一度も読めていない軸を「異常なし」とも「異常あり」とも言わない。
        let unread = JointStatus::default();
        assert!(!unread.valid);
        assert!(!unread.faulted());
        let ok = JointStatus {
            valid: true,
            error_raw: 0,
            ..Default::default()
        };
        assert!(!ok.faulted());
        let bad = JointStatus {
            valid: true,
            error_raw: 0x08,
            ..Default::default()
        };
        assert!(bad.faulted());
    }

    #[test]
    fn negative_sign_flips_velocity_and_torque() {
        let m = map(-1.0, 0.9);
        assert_eq!(m.rate_to_model(2.0), -2.0);
    }
}
