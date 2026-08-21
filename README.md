# namiashi-runner

四脚ロボット **namiashi**（LKMTech V3 モータ ×12 + 腕の RC サーボ ×1）を
プロポ（Futaba S.BUS）で操縦する実機アプリ。

`go2-gait-runner` が Unitree Go2 に対して果たしている役割の namiashi 版で、
歩容そのもの（`quadruped-gait`）やモデル（`misarta` / `.misa`）には手を入れず、
**実機の配線・座標変換・モード遷移・操縦入力**だけをここに持つ。

## できること

| | |
|---|---|
| 移動 | 前進後退 / 左右旋回 / 左右真横移動（スティック 3 軸） |
| 歩容 | Crawl / Walk / Trot をプロポのスイッチで切替 |
| 姿勢 | 胴体高さをスティックで上下 |
| 演出 | `.misa` のポーズ / シーケンス再生（挨拶）、チキンヘッド※ |
| 可視化 | `--viz` で articara に実時間描画（Zenoh） |
| 安全 | 受信断・フェイルセーフで速度 0・その場起立、Ctrl-C で脱力 |

※ チキンヘッドと挨拶の腕動作は、腕サーボをアプリから駆動できる構成でのみ
有効。現状は受信機直結なので腕は**観測のみ**（[未確定・既知の制限](#未確定既知の制限)）。

**引き継ぎ・設計判断・SBC 移行の手順**は [`doc/handover.md`](doc/handover.md)、
**配線とモータ id の対応表**は [`doc/motor_map.md`](doc/motor_map.md)。

## ハードウェア

`nm_board/ch348` rev2 基板（CH348L, USB-C → 8ch UART）が実機 I/F。
UART の割り当ては `spec_rev2_0_0_asbuilt.md` §4 のとおり:

| UART | 役割 | I/F |
|---|---|---|
| 0–3 | LEG1–4 = FL / FR / RL / RR（各 3 モータ） | RS485 |
| 4 | ARMA（腕サーボ） | RS485 / TTL 切替 |
| 5 | IMU（WitMotion IWT603） | TTL |
| 6 | S.BUS（受信専用） | 反転 TTL |
| 7 | ARMB（予備） | RS485 / TTL 切替 |

`/dev/ttyCH9344USB*` の番号は列挙順で決まるので当てにせず、ch9344 の
`GETUARTINDEX` ioctl で**物理 UART 番号**を引いて対応付ける。

## ビルド

依存はすべて GitHub の**公開**リポジトリへの git 依存なので、新しいマシン
（SBC など）でも兄弟チェックアウトは要らない。SSH 鍵も認証情報も要らない。

```sh
git clone --recurse-submodules https://github.com/takarakasai/namiashi-runner.git
cd namiashi-runner
cargo build --release      # 依存は cargo が GitHub から取ってくる
cargo test
```

**`--recurse-submodules` を忘れないこと。** `models/` は
[`namiashi_description`](https://github.com/takarakasai/namiashi_description) の
submodule で、モデル（`.misa`）と meshes がそこにある。忘れると `models/` が
空のままで `check` が「読み込みに失敗」になる。後から入れるなら:

```sh
git submodule update --init
```

Zenoh（`--viz`）が要らない環境ではこちらのほうが軽い（20 MB / ビルドも速い）:

```sh
cargo build --release --no-default-features
```

### 兄弟クレートも一緒に直したいとき

`misa-actuator` や `sbus` を namiashi-runner と併行して直す場合だけ、
path override を張る:

```sh
./scripts/dev-siblings.sh          # 兄弟を clone / 更新し .cargo/config.toml に [patch] を書く
./scripts/dev-siblings.sh --off    # git 依存へ戻す
```

**これを実行していない間、ローカルの兄弟チェックアウトへの変更はビルドに
反映されない。** cargo は `Cargo.lock` が指す GitHub の revision を見る。
「直したのに変わらない」の原因はたいていこれ。`cargo tree -p namiashi-hal`
で解決先（ローカルパスか git URL か）が確認できる。

`.cargo/config.toml` は追跡していない（人ごと・マシンごとに違うため）。

## 使い方

**立ち上げは上から順に。** 各段が通ってから次へ行くと、詰まった場所が常に
1 段で分かる。

```sh
namiashi check                 # 設定とモデルの検証（実機に触れない）
namiashi ports                 # CH348 のポート一覧（何も開かない）
namiashi dump --gait trot      # 歩容を実機なしで再生し可動域を検証
namiashi imu  --secs 10        # IMU 受信の確認
namiashi sbus --secs 10        # プロポ入力と解釈結果の確認
namiashi legs --secs 10        # 脚バスの状態と実効周期（**指令は送らない**）
namiashi calib scan            # 応答するモータ id を数える（指令は送らない）
namiashi run  --config config/namiashi.toml
```

設定は 1 枚の TOML（`config/namiashi.toml`）。雛形は
`namiashi config --out config/namiashi.toml` で生成できる。

### プロポ割り当て（既定）

| CH | 役割 |
|---|---|
| 1 | 左右真横（エルロン、反転） |
| 2 | 前後（エレベータ） |
| 3 | 胴体高さ（スロットル） |
| 4 | 旋回（ラダー、反転） |
| 5 | モード 3 段: 脱力 / 起立 / 歩行 |
| 6 | 歩容 3 段: Crawl / Walk / Trot |
| 7 | ポーズ再生（立ち上がりで 1 回） |
| 8 | チキンヘッド ON/OFF |
| 9 | 腕サーボ（受信機が直接駆動。アプリは角度を**観測するだけ**） |

チャンネル・エンドポイント・不感帯・エクスポは全部 `[teleop]` で変更できる。
`namiashi sbus` を見ながら合わせるのが早い。

## 校正（実機に通電したら最初にやること）

起動直後の設定は `sign = +1` / `zero_pose_rad = 0` / 可動域は URDF 値、という
**推測**でしかない。1 軸でも符号が逆なら起立の瞬間に自壊する。`calib` は
その 3 つを 1 軸ずつ実機で確定して設定へ書き戻す。

```sh
# 1) 誰が居るか（指令は送らない）
namiashi calib scan --max-id 8

# 2) 可動域を実測（脱力させ、手で端から端まで動かす）
namiashi calib range --leg FL --joint thigh --write config/namiashi.toml

# 3) 符号を確定（1 軸だけ 5° 動かし、モデルの + 方向か答える）
namiashi calib move  --leg FL --joint thigh --write config/namiashi.toml

#    2) と 3) を 12 軸ぶん繰り返す

# 4) ゼロ点（指定した姿勢で保持してからゼロ出し、その姿勢角を記録）
namiashi calib zero --pose constrain --write config/namiashi.toml
```

安全のための約束:

- **1 度に 1 軸しか投入しない。** `move` は対象軸だけ `EnableJoint` し、
  終わったら必ず `DisableJoint` で戻す。残り 2 軸は最後まで脱力のまま
- **開くポートも 1 本だけ。** `--leg FL` なら UART0 しか掴まない
- **既定の振り幅は 5°、速度 0.3 rad/s。** 取り違えていても壊れない大きさ
- **`--write` を明示したときだけ**設定ファイルへ書き戻す

`--write` の書き戻しは `AppConfig` から TOML を作り直すので、**手書きの
コメントは消える**。`namiashi config --out` で生成したファイルを校正で
上書きしていく運用を前提にしている。

## 安全側の作り

| 何 | どこ | 効き方 |
|---|---|---|
| 目標角のスルーレート制限 | `legs.max_target_rate_rad_s`（既定 3 rad/s） | 歩容切替や IK クランプで目標が跳んでも、脚の飛び出しにならない |
| 軸の速度上限 | `legs.default_max_speed_rad_s`（既定 8 rad/s） | モータ側が守る「軸が何 rad/s で回るか」 |
| 可動域クランプ | `motors[].min_rad` / `max_rad` | HAL が指令を必ず内側へ丸める |
| 異常ビット監視 | `legs.status_interval_ms`（既定 1 s） | 過電流・過熱・ストールを検出して ERROR ログ |
| 受信断 | `control.teleop_timeout_ms` | 速度 0・その場起立 |
| Ctrl-C / SIGTERM | — | 全軸を脱力してから終了 |

**スルーレート制限と軸の速度上限は別物。** 前者は「目標が何 rad/s で動くか」、
後者は「軸が何 rad/s で回るか」。目標が 1.5 rad 跳んだとき、後者だけだと
8 rad/s で追いに行ってしまう。

**異常ビットで自動脱力はしない。** 立っている四足を脱力させると倒れるので、
ERROR を毎回はっきり出したうえで、止めるかどうかは operator がモード
スイッチで決める。

## 動作モード

```text
  脱力 ──(CH5: 起立/歩行)──▶ 初期姿勢へ ──▶ 立ち姿勢へ ──▶ 歩容
   ▲                                                        │
   └────────────────(CH5: 脱力)─────────────────────────────┘
                                        歩容 ──(CH7)──▶ ポーズ再生 ──▶ 立ち姿勢へ
```

- **脱力中の目標角は実測角**。起立に移った瞬間に 0 rad へ飛ばない。
- **歩容の切り替えは脱力中と遷移中だけ**受け付ける。接地中に切り替えると
  踏み替えが飛ぶため。
- **受信断・フェイルセーフでは速度 0 のまま起立を続ける**。立っている四足を
  脱力させると倒れるので、電波が切れたときに最も壊れない選択にしてある。

## articara で描画して確かめる

歩容を実機に流す前に、`quadruped-gait` の
[`GaitVizFrame`](https://github.com/takarakasai/quadruped-gait) を Zenoh へ配信し、
articara の **Live gait feed** に描かせて目で確認できる。実機なし（`dump`）でも
実機を動かしながら（`run`）でも同じキーに流れる。

```sh
# 1) 配信側（実機なしで歩容だけ流す。--viz は自動で実時間になる）
namiashi dump --gait trot --vx 0.1 --secs 60 --viz --viz-endpoint tcp/127.0.0.1:7447

# 2) 受信側（別端末）
cd ../articara && cargo run --release --features viz -- \
    --model ../namiashi_description/namiashi.misa
#   → Live gait feed パネルで endpoint に tcp/127.0.0.1:7447 を入れて Start
```

`--viz-endpoint` はマルチキャスト探索が効かないホスト（同一ホスト / WSL2）向け。
効く環境なら両側とも省略してよい。実機を動かしながら見るなら `run` にも
同じ `--viz` 系オプションを付ける。

**送っているのはモデル座標系の角度**、つまりモータへ行く指令そのもの。
`GaitVizFrame::from_output` は歩容 / IK の符号のまま詰めるので、それを
そのまま流すと膝が反転して描かれる（向こうの doc コメントの警告）。
ここでは実機へ送るのと同じ [`JointVec`] からフレームを組んでいるので、
**画面に出た姿勢がそのまま実機の指令**になる。遷移中やポーズ再生中も描ける
のはこのため。ただし `GaitVizFrame` は脚 12 関節しか運ばないので、
`arm_pitch_joint` は articara 側で動かない。

## 設計上の要点

### 脚バスは 1 本 1 スレッドで自由走行

RS485 は半二重の要求応答で、待ち時間は USB の往復レイテンシに律速される
（ワイヤ上のビット時間より桁で大きい）。バスを跨いだ並列化だけが効くので、
バス 1 本にスレッド 1 本を割り当て、制御ループは共有スロットに目標を書いて
最新値を読むだけにしてある。

こうすると制御周期がバスのジッタから切り離され、**実際に何 Hz 出ているかを
`namiashi legs` で測ってから `control.rate_hz` を決められる**。

### 座標変換は HAL に閉じている

上位（歩容・ポーズ・チキンヘッド）はモデル（URDF / `.misa`）の関節角しか
扱わない。実機との差は設定の `sign` と `zero_pose_rad` だけ:

```text
q_motor = sign * (q_model − zero_pose_rad)
q_model = sign *  q_motor + zero_pose_rad
```

`zero_pose_rad` は **ゼロ出しを行った姿勢のモデル関節角**。LKMTech V3 の
位置制御は `rezero` で置いたソフトゼロからの相対量なので、「どの姿勢で
ゼロ出ししたか」を書いておかないとモデル角と対応が付かない。

### 立ち高さは `nominal_foot_body` に書き込む

`quadruped-gait` の `set_body_height_m` は `LinearCrawl` 専用で、CHAMP 系は
`LegKinematics::nominal_foot_body` を見る。`gait.stance_height_m` をどの歩容
でも効かせるため、コントローラを組むたびにこの Z を書き換えている
（`robot::Robot::kin_at_height`）。

### 既定の歩容モードは 3 種とも CHAMP 系

`GaitMode::LinearCrawl` は胴体を +X 直線に載せる専用プランナで、**横移動
(vy) と旋回 (wz) の指令を受け付けない**。「前後・左右・旋回をプロポで操る」
という要件に合わないので既定では使わない。直進の安定性を追い込みたいときだけ
`gait.crawl_use_linear = true` で選ぶ。

## 未確定・既知の制限

- **初期姿勢（250×350×700 mm の直方体に収める姿勢）は未確定。**
  `control.start_pose` が指す `.misa` のポーズ名で決まる。暫定で
  モデルに入っている `constrain`（thigh 1.0 / calf −2.0）を指している。
- **腕は受信機直結で、アプリからは駆動しない**（`[hardware.arm].protocol =
  "receiver_direct"`）。したがって**チキンヘッドと挨拶の腕動作は現状無効**。
  `teleop.arm` のチャンネル（既定 CH9）から角度を**観測**して、ログ・可視化・
  モデル状態には実際の角度を入れている。サーボの品種が決まったら
  `ArmProtocol` に variant を足し、`namiashi_hal::arm::ArmServo` の実装を
  差し込めば `is_app_driven() = true` になり、両方が自動的に有効になる。
- **無応答モータ 1 台あたり約 20 ms 待つ。** `lkmotor_driver::Rs485Driver` が
  シリアルの read タイムアウトを固定 20 ms で開き、締切判定を read の後に
  行うため、`response_timeout_ms` を 5 と書いても効かない。実測で 3 台無応答の
  バスは 16 Hz まで落ちる。生きているモータしかいなければ影響しないが、
  1 台落ちたときの縮退性能はこれで決まる。直すなら misa-actuator 側。
- **制御はまだ位置制御のみ。** `JointMode::Torque` の口は空けてあるが、
  MPC / WBC のトルク制御へ進むのは基盤が動いてから。LKMTech の MIT は
  ホスト側エミュレーション（`measure` + `set_torque` の 2 往復）なので、
  通信レートが半分になる点に注意。

## 構成

```
crates/
├── namiashi-hal/         実機の抽象化
│   ├── ch348.rs          UART 番号 → ポート（探索は sbus::discover に委譲）
│   ├── config.rs         配線・モータ id・符号・可動域（TOML）
│   ├── legs.rs           RS485 脚バス ×4（バス 1 本 1 スレッド）
│   ├── imu.rs            WitMotion 受信スレッド
│   ├── sbus.rs           S.BUS 受信スレッド（sbus クレートの上）
│   ├── arm.rs            ArmServo トレイト + 受信機直結 / 未配線
│   └── joint.rs          関節の並び順と値型
└── namiashi-runner/      アプリ
    ├── config.rs         制御・歩容・操縦・ポーズの設定
    ├── robot.rs          .misa の読み込みと歩容の組み立て
    ├── teleop.rs         S.BUS → 操縦指令
    ├── controller.rs     モード遷移の状態機械（実機非依存）
    ├── pose.rs           .misa のポーズ / シーケンス再生
    ├── chicken.rs        チキンヘッド
    ├── runner.rs         実機の制御ループ
    ├── calib.rs          符号・ゼロ点・可動域の実機校正
    ├── dump.rs           実機なしの歩容検証
    ├── viz.rs            articara へのライブ配信（Zenoh）
    └── diag.rs           実機を動かさない確認コマンド
```
