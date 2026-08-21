# handover — namiashi-runner

2026-08-17〜19 に PC 環境で作った内容の引き継ぎ。**コードが語れないこと**
（なぜそうしたか、何が確かめてあって何が未確認か、次に何をするか）をここに置く。
使い方そのものは [`README.md`](../README.md)、配線表は
[`doc/motor_map.md`](motor_map.md)。

---

## 1. これは何か

四脚ロボット namiashi（LKMTech MG4005 ×12 + 腕の RC サーボ ×1）を Futaba の
プロポ（S.BUS）で操縦する実機アプリ。`go2-gait-runner` が Unitree Go2 に対して
果たしている役割の namiashi 版。

**歩容そのものは書いていない。** 歩容は `quadruped-gait`、モデルと運動学は
`misarta` / `.misa`、モータ通信は `misa-actuator`、S.BUS は `sbus`、IMU は
`wit-imu` が持っている。ここにあるのは**実機の配線・座標変換・モード遷移・
操縦入力**だけ。上流に手を入れずに済ませたのは意図的で、歩容の改良が別リポジトリ
で進んでもこちらは追従するだけで済むようにしてある。

```
crates/
├── namiashi-hal/     実機の抽象化（ポート探索・脚バス・IMU・S.BUS・腕）
└── namiashi-runner/  アプリ（設定・歩容組み立て・操縦・状態機械・制御ループ）
```

---

## 2. 動作を確認した範囲 / していない範囲

**「動く」と書いてあるものは実際に走らせて確かめたもの。** ここを混ぜると
引き継いだ人が踏む。

| 項目 | 状態 | 根拠 |
|---|---|---|
| ビルド・テスト | ✅ | 94 テスト通過、clippy 警告なし、`--no-default-features` も通る |
| CH348 ポート探索 | ✅ 実機 | `namiashi ports` が UART0–7 を正しく役割付け |
| IMU 受信 | ✅ 実機 | IWT603 @921600、重力 9.6 m/s²、resync 0 |
| S.BUS 受信 | ✅ 実機 | 70 fps、desync 0、送信機 OFF で failsafe 判定が正しく効いた |
| 脚バスを開く | ✅ 実機 | 4 ポート同時オープン成功 |
| **モータとの通信** | ❌ **未** | **モータに通電したことが一度もない** |
| 歩容の関節角 | ✅ 机上 | Crawl/Walk/Trot × 前後・横・旋回で可動域内 |
| articara 可視化 | ✅ | 同じ購読コードで受信、Trot の対角接地と旋回を確認 |
| 校正コマンド | ⚠️ 半分 | `scan` は実機で動作（応答なしを正しく報告）。`move`/`range`/`zero` は**モータ未通電のため未検証** |
| 制御ループ (`run`) | ❌ **未** | 一度も実行していない |

**いちばん大きい未知は通信レート。** RS485 の 1 トランザクションは USB の往復
レイテンシに律速され、それが何 µs なのかはモータを繋がないと分からない。
`control.rate_hz` を 200 にできるのか 50 が精一杯なのか、Trot が現実的かは
すべてここで決まる。`namiashi legs` が各バスの実効周期を出すので、**通電したら
最初にこれを測ること**。

---

## 3. 設計上の判断（なぜそうなっているか）

### 脚バスは 1 本 1 スレッドで自由走行

RS485 は半二重の要求応答で、待ち時間は USB の往復に律速される（ワイヤ上の
ビット時間より桁で大きい）。**バスを跨いだ並列化だけが効き、同一バス内は
直列にしかならない。**

制御ループがバスの完了を待つ形にすると、制御周期の上限が最も遅いバスで決まり、
しかもそれが何 Hz かは実機を繋ぐまで分からない。そこで各バスを自由走行させ、
制御ループは共有スロットに目標を書き最新値を読むだけにした。**「まず測ってから
制御周期を決める」ができる**のはこのため。

### 座標変換は HAL に閉じている

上位（歩容・ポーズ・チキンヘッド）はモデルの関節角しか扱わない。実機との差は
設定の 2 つだけ:

```text
q_motor = sign * (q_model − zero_pose_rad)
q_model = sign *  q_motor + zero_pose_rad        (sign = ±1)
```

`zero_pose_rad` は **ゼロ出しを行った姿勢のモデル関節角**。LKMTech V3 の位置
制御は `rezero` で置いたソフトゼロからの相対量なので、「どの姿勢でゼロ出し
したか」が無いとモデル角と対応が付かない。

### `Disable` でアンカーを落とさない

`Motor::set_position` の基準はモータ自身のマルチターン角（`0x92`）に置いた
**絶対値**なので、脱力して外力で動かされてもゼロ点はずれない。ここで落とすと
再起立のたびに「今いる姿勢」でゼロを引き直すことになり、校正姿勢とは無関係な
原点が入る。

### 立ち高さは `nominal_foot_body` に書き込む

`quadruped-gait` の `set_body_height_m` は `LinearCrawl` 専用で、CHAMP 系は
`LegKinematics::nominal_foot_body` を見る。`gait.stance_height_m` をどの歩容でも
効かせるため、コントローラを組むたびに Z を書き換えている
（`robot::Robot::kin_at_height`）。

### 既定の歩容モードは 3 種とも CHAMP 系

`GaitMode::LinearCrawl` は胴体を +X 直線に載せる専用プランナで、**横移動 (vy) と
旋回 (wz) の指令を受け付けない**（机上で hip が 0 のまま動かないことを確認）。
「前後・左右・旋回をプロポで操る」要件に合わないので既定から外した。直進の
安定性を追い込むときだけ `gait.crawl_use_linear = true`。

### 受信断で脱力しない

立っている四足を脱力させると倒れる。受信断・フェイルセーフでは**速度 0 のまま
その場で立ち続ける**のが、電波が切れたときに最も壊れない。異常ビット検出でも
同じ理由で自動脱力せず、ERROR を出して operator の判断に委ねている。

### 腕は「繋がっている」と「駆動できる」を分けてある

受信機直結の腕は**動いてはいるがアプリの指令では動かない**。1 つの bool に
まとめると「サーボがある = 指令が効く」と読めてしまい、チキンヘッドが黙って
無効化されていることに気づけない。`is_connected()` と `is_app_driven()` は別。

---

## 4. 実機で踏んで直した罠

引き継いだ人が同じ穴に落ちないように。

1. **CH348 の UART 探索はデバイスを `open` する。** ポートを 1 本開くたびに
   探索し直すと、2 本目以降が自分自身の `EBUSY` で「見つからない」になる。
   `PortMap::discover()` を**開く前に 1 回だけ**呼ぶ設計にしてある。
2. **zenoh 1.x で `Session` を落とすと `put` が静かに失敗する。**
   `declare_publisher` の戻り値だけ保持していて、配信しているつもりで無音
   だった。`Session` を抱えて `session.put(key, ..)` を使う。
3. **`GaitVizFrame::from_output` は歩容 / IK の符号のまま詰める。** そのまま
   流すと articara で膝が反転して描かれる。実機へ送るのと同じモデル座標系の
   `JointVec` からフレームを組んでいる。
4. **`lkmotor_driver::Rs485Driver` の応答タイムアウトは実質 20 ms が下限。**
   シリアルの read タイムアウトが固定 20 ms (`READ_POLL_TIMEOUT`) で、締切
   判定が read の後にあるため、`response_timeout_ms = 5` と書いても効かない。
   実測で 3 台無応答のバスは 16 Hz まで落ちた。**1 台落ちたときの縮退性能が
   これで決まる。** 直すなら misa-actuator 側。
5. **Xvfb + llvmpipe では articara の GUI が起動しない**（eframe が MSAA 4x の
   glutin config を要求して落ちる）。ヘッドレスで確認できるのは
   `articara --script-headless <rhai> <model>` まで。

---

## 5. SBC への移行

### 5.1 リポジトリはすべて push 済み（2026-08-19 完了）

移行のブロッカーだった 2 件は解消した。

| リポジトリ | remote | 状態 | SBC に要る？ |
|---|---|:--:|:--:|
| namiashi-runner | `github.com/takarakasai/namiashi-runner` | ✅ push 済み | ✅ clone する |
| wit-imu | `github.com/takarakasai/wit-imu` | ✅ push 済み | — cargo が取る |
| misa-actuator | `github.com/takarakasai/misa-actuator` | ✅ 同期済み | — cargo が取る |
| misarta | `github.com/takarakasai/misarta` | ✅ 同期済み | — cargo が取る |
| misa-wbc | `github.com/takarakasai/misa-wbc` | ✅ 同期済み | — cargo が取る |
| quadruped-gait | `github.com/takarakasai/quadruped-gait` | ✅ 同期済み | — cargo が取る |
| sbus | `github.com/takarakasai/sbus` | ✅ 同期済み | — cargo が取る |
| namiashi_description | 〃 | 2 件未取得 | ❌ 不要（モデルは同梱） |
| articara | 〃 | 未コミット多数 | ❌ 不要（可視化は PC 側、§5.6） |

**7 リポジトリとも https で公開**されているので、SBC に SSH 鍵も認証情報も
要らない。依存は git 依存に切り替えたので、**兄弟チェックアウトを並べる必要も
無くなった**。

> `misa-actuator` は remote-tracking ref が古いと「140 件未 push」に見える。
> 判断する前に必ず `git fetch` すること（実際は同期済みだった）。

### 5.2 移行手順

```sh
# SBC 側。これだけ。--recurse-submodules を忘れると models/ が空になる。
git clone --recurse-submodules https://github.com/takarakasai/namiashi-runner.git
cd namiashi-runner
cargo build --release --no-default-features     # viz 不要なら軽いほう

# 実機に触れない順に確認
./target/release/namiashi check
./target/release/namiashi ports        # ch9344 が入っていないとここで落ちる
./target/release/namiashi imu  --secs 10
./target/release/namiashi sbus --secs 10
./target/release/namiashi legs --secs 10        # 指令は送らない
```

`Cargo.lock` を追跡しているので、SBC は PC とまったく同じ revision を引く。

### 5.3 兄弟クレートを併行して直すとき

git 依存に切り替えた副作用として、**ローカルの兄弟チェックアウトへの変更は
既定ではビルドに反映されない**。反映させたいときだけ:

```sh
./scripts/dev-siblings.sh          # .cargo/config.toml に [patch] を書き出す
./scripts/dev-siblings.sh --off    # git 依存へ戻す
```

`cargo tree -p namiashi-hal` で解決先（ローカルパスか git URL か）が見える。
**「直したのに変わらない」の原因はたいていこれ。**

`paths` override ではなく `[patch]` を使っているのは、前者が workspace 継承
された依存（`sbus-protocol = { workspace = true }` など）と噛み合わず
「altered the original list of dependencies」の警告を出すため。

`Cargo.toml` の git URL は **`quadruped-gait` が使っている綴りと一字一句
同じでなければならない**（cargo は git ソースを URL 文字列で識別するので、
`git@` 形式や末尾 `.git` の有無が違うだけで別ソース扱いになり、`misarta` の
型が二重に生えてビルドが通らない）。`Cargo.lock` の `misarta` エントリが
1 個であることで確認できる。

### 5.4 SBC 側の前提

| 項目 | 内容 |
|---|---|
| **ch9344 ドライバ** | CH348 は標準カーネルに入っていない。SBC のカーネル向けに `nm_board/ch348/ch9344ser_linux` をビルド（DKMS 推奨）。**これが無いと `/dev/ttyCH9344USB*` が生えず、UART 番号の ioctl も使えない** |
| **シリアルの権限** | 実行ユーザを `dialout` に入れるか udev ルールを置く。入っていないと全ポートが `Permission denied` |
| **Rust** | edition 2024 を使うので **1.85 以上** |
| **ビルド時間** | PC（32 コア）で release 44 秒 / CPU 時間 10 分。4 コアの SBC なら 10〜20 分を見込む。`clarabel` と `zenoh` が重い（`--no-default-features` なら zenoh が丸ごと消える） |
| **バイナリサイズ** | release 283 MB（`debug = true` のため）。`--no-default-features`（viz 無し）で 62 MB、`strip` すると 20 MB。SBC のストレージが厳しければ strip して配る |

### 5.5 リアルタイム性

制御ループは 200 Hz 目標だが、**優先度制御は入れていない**。SBC でジッタが
問題になるなら起動側で:

```sh
sudo chrt -f 50 ./namiashi run --config config/namiashi.toml
# CPU ガバナも performance に
```

`run` の状態行に「遅延最大」が出るので、まずそれを見てから判断すること。

### 5.6 可視化は SBC + PC で分けられる

`--viz` は Zenoh なので、**SBC で `run`、PC の articara で描画**ができる。
同一 LAN でマルチキャストが通るならオプション不要、通らなければ両側に
`--viz-endpoint tcp/<SBC の IP>:7447` を指定する。SBC にディスプレイは要らない。

---

## 6. 未確定・保留

| 件 | 状態 |
|---|---|
| **初期姿勢（250×350×700 mm）** | **未確定。相談したいと言われている。** `control.start_pose` が指す `.misa` のポーズ名で決まり、暫定で `constrain`（thigh 1.0 / calf −2.0）。articara でポーズを作って `.misa` に保存すれば名前を書くだけで反映される |
| **腕サーボの品種** | 未定（追って連絡）。初期検討は受信機直結。`ArmProtocol` に variant を足して `ArmServo` を実装すれば `is_app_driven() = true` になり、チキンヘッドと挨拶の腕動作が自動で有効になる |
| **`sign` / `zero_pose_rad` / 可動域** | 全 12 軸未校正。`calib` で確定させる |
| **トルク制御** | `JointMode::Torque` の口は空けてあるが未使用。MPC / WBC へ進むのは位置制御で歩いてから。LKMTech の MIT はホスト側エミュレーション（`measure` + `set_torque` の 2 往復）なので通信レートが半分になる |
| **CI** | 兄弟クレートは全部 `.github/workflows/ci.yml` を持っているが、ここはまだ無い。remote はできたので足せる状態 |

---

## 7. 次にやること（推奨順）

1. ~~`wit-imu` の push / `namiashi-runner` の remote~~ — **2026-08-19 完了**
2. **モータに通電して `namiashi legs`** — 各バスの実効周期を測り、
   `control.rate_hz` を決める。ここが全ての前提
3. **`calib` を 12 軸ぶん**（`scan` → `range` → `move` → `zero`）。
   1 軸ずつ、脚を浮かせて
4. **`dump` で再確認** — 校正後の可動域で歩容が範囲内か
5. **脚を浮かせて `run --allow-no-sbus`** → プロポを繋いで `run`
6. **接地して歩行**。まず Crawl、`--viz` で articara に出しながら
