# namiashi 実機立ち上げチェックリスト

対象: 四脚ロボット **namiashi**（LKMTech MG4005 ×12 + 腕 RC サーボ ×1）
制御機: `radxa-cubie-a7z`
関連: [`boot_config.md`](boot_config.md) / [`runtime_tuning.md`](runtime_tuning.md) /
[`viz_live.md`](viz_live.md) /
`handover.md` / `motor_map.md`

作成日: 2026-08-21（実機組み上がり直後）

---

## この文書の使い方

**上から 1 段階ずつ、合格条件を満たしてから次へ進む。** 飛ばさない。

各段階に「危険度」を付けてある。

| 危険度 | 意味 |
|---|---|
| 🟢 | モータは一切動かない（読み出しのみ） |
| 🟡 | 1 軸だけ小さく動く / 脱力させて手で動かす |
| 🔴 | 複数軸が同時に動きうる |

---

## 段階 0: 着手前の準備【🟢】

### 0-1. `response_timeout_ms` を 20 に戻す ★重要★

`misa-actuator` に当てた 20 ms パッチにより、**タイムアウトの実効値が変わっている。**

| | 実効タイムアウト |
|---|---|
| パッチ前 | 20 ms（`response_timeout_ms = 5` は無視されていた） |
| パッチ後（現在） | **5 ms**（設定どおり効く） |

組み上がったばかりの機体でモータの応答が 6〜15 ms かかると、**パッチ前なら通っていたものが
初めてタイムアウトする**。症状は「モータが応答しない」に見え、配線を疑って時間を溶かす。

```toml
# config/namiashi.toml
[hardware.legs]
response_timeout_ms = 20    # 立ち上げ中は 5 → 20
```

**この値ならパッチ有無で挙動が完全に一致する**（旧実装の下限 20 ms = 新実装の設定値 20 ms）。
新ハードウェアと新ソフトウェアを同時にデバッグする状況を避けられる。再ビルド不要。

- [ ] `response_timeout_ms = 20` に変更した

1〜2 ms への切り詰めは、全軸が確実に動いてから 500 Hz 化の作業として別途行う。

### 0-2. 設定を git に退避

`calib --write` は `config/namiashi.toml` を**丸ごと再生成**する（手書きコメントは消える）。

```sh
cd ~/work/namiashi-runner
git checkout -b calib/initial-bringup
git add -A && git commit -m "wip: before calibration"
```

- [ ] 校正用ブランチを切った
- [ ] **軸ごとにコミットする**方針を決めた（途中で戻せるように）

### 0-3. システム側の設定を絞る

| 設定 | 立ち上げ中 | 理由 |
|---|---|---|
| `chrt` / RT 優先度 | **使わない** | 暴走したプロセスを殺しにくくなる |
| ウォッチドッグ | **有効化しない** | ベンチでのハング → 自動リセットは原因究明を妨げる |
| `performance` governor | そのまま | 問題なし |
| `usbcore.autosuspend=-1` | そのまま | CH348 の経路保護 |
| 常駐デーモン | **整理済み** | 2026-08-21 に棚卸し済み（下記） |
| VSCode Remote | **切る** | node + claude で 1.4 GiB / CPU 4〜12% を食う |

- [ ] `chrt` を使わないことを確認
- [ ] ウォッチドッグ未適用のままであることを確認（`systemctl show -p RuntimeWatchdogUSec` が 0）
- [ ] 常駐サービスが 13 個であることを確認（`daemon-baseline.txt` と突き合わせ）
- [ ] 計測を伴う段階では VSCode Remote の接続を切る

常駐デーモンは 2026-08-21 に 17 → 13、タイマー 5 → 2 まで削ってある。
差分が出たら `daemon-baseline.txt` と比較し、詳細は
[`runtime_tuning.md`](runtime_tuning.md)「調査3: 常駐デーモンの棚卸し」を参照。
再適用が要る場合は [`cleanup-daemons.sh`](cleanup-daemons.sh)。

### 0-4. 物理的な安全確保

- [ ] **脚が接地していない**（機体を吊るす / 台に載せて脚を浮かせる）
- [ ] モータ電源を**即座に切れる**手段がある（スイッチ / ブレーカ）
- [ ] 電流制限のある電源を使っている（可能なら）
- [ ] 可動範囲に人・工具・ケーブルが無い
- [ ] `Ctrl-C` で全軸脱力することを認識している（README「安全側の作り」）

---

## 段階 1: 実機に触れない検証【🟢】

### 1-1. 設定とモデル

```sh
./target/release/namiashi check --config config/namiashi.toml
```

**合格条件:**
- [ ] `設定: OK`
- [ ] 関節 18 / nq=13
- [ ] 配線表が `motor_map.md` と一致（FL=UART0 / FR=UART1 / RL=UART2 / RR=UART3 / IMU=UART5 / S.BUS=UART6）
- [ ] `response_timeout_ms` の変更が反映されている

### 1-2. ポートの役割付け

```sh
./target/release/namiashi ports --config config/namiashi.toml
```

**合格条件:**
- [ ] UART 0〜7 が 8 本すべて表示される
- [ ] 役割が `motor_map.md` の as-built 表と一致

**異常時:** `/dev/ttyCH9344USB*` が無ければ ch9344 ドライバか USB 接続。
`dkms status` で `ch9344, 2.3-0450213 ... installed` を確認。

---

## 段階 2: 通信の確認【🟢 モータは動かない】

> `JointMode::Idle` が既定なので、**バスを開いただけでは励磁されない**。
> 指令を書くまでモータは動かない（`joint.rs` で確認済み）。

### 2-1. モータ電源 ON

- [ ] 脚が浮いていることを再確認してから投入

### 2-2. id スキャン（脚ごと）★最初の関門★

```sh
./target/release/namiashi calib scan --leg FL --max-id 8 --config config/namiashi.toml
./target/release/namiashi calib scan --leg FR --max-id 8 --config config/namiashi.toml
./target/release/namiashi calib scan --leg RL --max-id 8 --config config/namiashi.toml
./target/release/namiashi calib scan --leg RR --max-id 8 --config config/namiashi.toml
```

**指令は出さない**（State2 の読み出しのみ）。

**合格条件（脚ごとに）:**
- [ ] **id 1, 2, 3 の 3 つだけ**が応答する
- [ ] 4 脚すべてで同じ結果

**なぜこれが最初の関門か:** 4 バスとも id は `1, 2, 3` の繰り返しで、
**軸の identity は `(バス, id)` の組**でしか決まらない。取り違えると
「片脚だけ挙動がおかしい」という切り分けにくい症状になる（`motor_map.md`）。

**異常時に疑うもの:**

| 症状 | 疑うもの |
|---|---|
| 1 つも応答しない | モータ電源、RS485 の A/B 極性（J14〜J17 の 1=B, 2=A）、ボーレート |
| 一部しか応答しない | 該当モータの配線、終端抵抗、id 設定 |
| 4 個以上応答する | id の重複、別バスとの短絡 |
| **タイムアウトが多発** | 段階 0-1 の `response_timeout_ms = 20` を確認 |

### 2-3. 脚バスの実効周期

```sh
./target/release/namiashi legs --secs 10 --config config/namiashi.toml
```

**指令は送らない。**

**合格条件:**
- [ ] 12 軸すべての状態が読める（`ok` が真）
- [ ] ~~実効周期が `bus_rate_hz = 500` に近い~~ → **約 420 Hz が実力**（下記）
- [ ] 異常ビット（過電流・過熱・ストール）が出ていない

**異常時:** 周期が大きく落ちる場合、応答しない軸がある。段階 2-2 に戻る。

#### 初通電の実測（2026-08-21）

```
FL 415.8Hz 最悪 6.00ms   FR 442.7Hz 最悪 5.59ms
RL 420.7Hz 最悪 5.61ms   RR 417.6Hz 最悪 6.02ms      12 軸とも ok=true / err=0
```

**500 Hz には届かない。** 律速はワイヤ上の時間ではなく **CH348 の USB 往復
レイテンシ**（1 トランザクションあたり約 700 µs）。詳細と帰結は
[`runtime_tuning.md`](runtime_tuning.md)「初通電での実測」。
**約 420 Hz を下回るようなら異常**、という基準で読むこと。

最悪応答 5.59〜6.02 ms は §0-1 が警告していた 6〜15 ms の帯にちょうど入った。
**`response_timeout_ms = 5` のままならここでタイムアウト多発になっていた。**

### 2-4. 関節対応の目視確認【🟢 articara / 指令は送らない】

`(バス, id)` → 関節の対応と符号は、**数字を睨むより画面で見た方が速い**。
`legs --viz` は**エンコーダの実測角**を Zenoh へ流すので、PC の articara に
実機の姿勢がそのまま出る。**指令は一切送らない。**

> `run --viz` が流すのは**目標角**なので、これとは狙いが逆。
> 目標角は「指令どおりの姿勢」しか描かず、実機がその通り動いたかは映らない。
> 取り違えると「画面で合っているから実機も合っている」と誤解する。

```sh
# SBC
./target/release/namiashi legs --secs 0 --viz --config config/namiashi.toml
# PC（モデルは namiashi_description を clone するだけ。scp は要らない）
cd articara && cargo run --release --features viz -- \
    --model ../namiashi_description/namiashi.misa
#   → Live gait feed パネルにキー go2/gait/planned を入れて Start
```

**手順の全体・ネットワークの 3 通り・繋がらないときの切り分けは
[`viz_live.md`](viz_live.md) にまとめてある。**

**この段階で確認できること / できないこと:**

| | |
|---|---|
| ✅ `(バス, id)` → 関節の対応 | FL の thigh を動かして、画面で FL の thigh が動くか |
| ✅ 符号の向き | 曲げた向きと画面の向きが一致するか |
| ❌ **角度の絶対値** | **ゼロ点未校正なのでオフセットしている。段階 6 まで意味を持たない** |

**絶対角が合っていなくても異常ではない。** 現在の表示は「エンコーダの生値 −
未設定のゼロ点」であって、機構的な角度ではない。ここで見るのは**動きの対応**だけ。

- [ ] 4 脚 × 3 軸すべて、手で動かすと画面の対応する関節が動く
- [ ] 動かした向きと画面の向きが一致する（違えば段階 5 の `sign` で直す）
- [ ] **別の脚が動かない**（動いたら `(バス, id)` の取り違え → 段階 2-2 に戻る）

**腕は映らない。** `GaitVizFrame` は脚 12 関節ぶんしか運ばない器（Go2 向け）なので、
`arm_pitch_joint` は articara 側で動かないまま。異常ではない。

---

## 段階 3: センサの確認【🟢 モータは動かない】

### 3-1. IMU

```sh
./target/release/namiashi imu --secs 10 --config config/namiashi.toml
```

- [ ] 値が更新される
- [ ] 機体を静止させたとき値が安定
- [ ] 機体を傾けたとき**傾けた向きに**値が動く（符号の妥当性）

### 3-2. S.BUS（プロポ）

```sh
./target/release/namiashi sbus --secs 10 --config config/namiashi.toml
```

出力例（2026-08-21 実測）:

```
namiashi sbus  /dev/ttyCH9344USB6  100000 8E2    65.7 fps  frames=198 slots=49 desync=0
link=OK   CH17:○  CH18:○   FRAME_LOST:no   FAILSAFE:no
S.BUS2 Rx-Batt=4.9V Ext-Volt=26.0V
----------------------------------------------------------------------------
  CH 1 左右     1017 █████·······   CH 2 前後     1006 █████·······
  CH 3 高さ     1035 ██████······   CH 4 旋回     1014 █████·······
  CH 5 モード     64 ············   CH 6 歩容       64 ············
  CH 7 ポーズ   1984 ███████████·   CH 8 チキン   1984 ███████████·
  CH 9 腕         64 ············   CH10          1984 ███████████·
  CH11          1024 ██████······   CH12          1024 ██████······
  CH13          1024 ██████······   CH14          1024 ██████······
  CH15          1024 ██████······   CH16          1024 ██████······
----------------------------------------------------------------------------
  vx=+0.000 m/s   vy=-0.000 m/s   wz=-0.000 rad/s   高さ=+0.000 m
  モード=Relax   歩容=Crawl   ポーズ=-   チキンヘッド=on   腕=-2.300rad
```

**役割名は設定から引いている**ので、`config/namiashi.toml` の
`[teleop.*] channel` を変えれば表示も追従する。「期待どおりのチャンネルに出るか」は
この欄を見れば分かる。

ログに採る / grep したいときは `--plain` で 1 行 / 更新の逐次出力になる。

**`--secs 0`（以下）または `--forever` で Ctrl-C まで回り続ける**（どちらでも同じ）。
スティックとスイッチを一通り確かめる作業は時間が読めないので、秒数に追われるより
こちらが向く。`imu` / `legs` / `calib range` も同じ。

- [ ] 送信機の電源 ON で値が入る
- [ ] 各スティック / スイッチが期待どおりのチャンネルに出る
  （CH1 左右 / CH2 前後 / CH3 高さ / CH4 旋回 / CH5 モード / CH6 歩容 / CH7 ポーズ / CH8 チキンヘッド / CH9 腕）
- [ ] **CH5 が「脱力」位置**にあることを確認
- [ ] 送信機を切ると `link=NG(FAILSAFE)` になる
- [ ] `S.BUS2` と表示される（`S.BUS1` だと電圧テレメトリが来ない）
- [ ] `Ext-Volt` が走行用バッテリの実電圧と一致する
- [ ] `desync=0` / `unknown=` が出ていない

**`link=NG()` の読み方:**

| 表示 | 意味 |
|---|---|
| `FAILSAFE` | 受信機がフェイルセーフ中。送信機 OFF がこれ |
| `FRAME_LOST` | 受信機が RF フレームを落としている。電波が弱い |
| `TIMEOUT` | フラグは正常なのに最後のフレームから `teleop_timeout_ms` 以上経過 |

**fps が出ていてもリンクが生きている証拠にはならない。** 受信機はフェイルセーフ中も
フレームを送り続ける（送信機 OFF でも 66.5 fps 継続、`sbus/doc/spec.md` §6.2）。

**異常時:** UART6 は**反転 TTL・受信専用**（`motor_map.md`）。信号が来ない場合は
CN2 の結線（1=GND, 2=+5V, 3=RX）と受信機の S.BUS 出力設定。

**`unknown=` が増えていく場合は電圧が測定範囲を超えている可能性がある。**
未知 marker は捨てられるので、`Ext-Volt` は**古い値のまま固まる**（0 V にはならない）。
現在のデコードは 10 ビット = 102.3 V まで（`sbus/doc/spec.md` §5.2）。

---

## 段階 4: 可動域の実測【🟡 脱力させ、手で動かす】

> 現在の `min_rad` / `max_rad` は **URDF の値**であって実機で確かめた値ではない
> （`motor_map.md`）。
>
> **calf の下限 −2.62 rad (−150°) は狭すぎることが判明している。** 設計データの
> 初期姿勢が前脚 calf **−162°**、伏せ姿勢のオフセットが **−160°** で、どちらも
> URDF の範囲から 10〜12° はみ出す。機構は実際にそこまで曲がる。
> **この段階で calf を実測して広げること。** 広げるまで `start` ポーズは
> −150° にクランプされ、スタート台に収まらない。

`calib range` は対象バスを `Disable`（脱力）してから 20 秒間、手で動かした範囲を記録する。
**自動で端を探しに行かない**設計（探る側が壊す側になるため）。

`--secs 0`（以下）または `--forever` なら秒数ではなく **Ctrl-C で確定**する。
打ち切っても集計と `--write` はそのまま走るので、「納得したら止める」使い方ができる。

```sh
# 12 軸ぶん繰り返す。--write を付けたときだけ設定に書き戻る
./target/release/namiashi calib range --leg FL --joint hip   --write config/namiashi.toml --config config/namiashi.toml
./target/release/namiashi calib range --leg FL --joint thigh --write config/namiashi.toml --config config/namiashi.toml
./target/release/namiashi calib range --leg FL --joint calf  --write config/namiashi.toml --config config/namiashi.toml
# FR / RL / RR も同様
```

**各軸の合格条件:**
- [ ] 脱力していて手で自由に動く
- [ ] `min` / `max` が更新されていく（画面に出る）
- [ ] 記録された範囲が機構的に妥当（機械端に当てず、余裕 0.05 rad が引かれる）

**注意:**
- **hip の可動域は左右で反転**（FL/RL は −0.785…+1.05、FR/RR は −1.05…+0.785）
- 「動きが記録できませんでした」→ モータ電源とバスを確認

- [ ] 12 軸すべて完了
- [ ] 軸ごとにコミットした

---

## 段階 5: 符号の確定【🟡 1 軸だけ 5° 動く】

> ~~`sign` は全 12 軸が **+1.0 の推測値**（`motor_map.md`）。~~
>
> **2026-08-21 に設計データから確定し、実機でも確認済み**（`motor_map.md` の表）。
> Roll は前後で、Pitch は左右で反転する。この段階は**確定値の裏取り**として
> 実施する。1 軸ずつ動かして、表と食い違わないことを見る。
> **1 軸でも符号が逆なら起立の瞬間に自壊する**（`calib.rs` の冒頭コメント）。

`calib move` は **対象バス 1 本だけを開き**（`open_alone`）、**対象軸だけ `EnableJoint`**、
終わったら**必ず `DisableJoint`** で戻す。既定の振り幅 5°、速度 0.3 rad/s。

```sh
./target/release/namiashi calib move --leg FL --joint hip --write config/namiashi.toml --config config/namiashi.toml
# 12 軸ぶん繰り返す
```

**各軸の手順:**
1. 「脚が自由に動ける状態か確認してください」で Enter
2. 5° 動く
3. **「モデルの + 方向（URDF の関節軸まわり右ねじ）へ動いたか」に y/n で答える**
4. `sign` が確定して書き戻る

**合格条件:**
- [ ] 動いた軸が**指定した軸である**（別の軸が動いたら配線の取り違え → 段階 2-2 へ戻る）
- [ ] 動き幅が指令とおおむね一致（「⚠ ほとんど動いていません」が出ない）
- [ ] + 方向の判定に確信がある（迷ったら URDF / モデル図で確認してから答える）

**異常時:**

| 症状 | 疑うもの |
|---|---|
| 別の軸が動いた | `(バス, id)` の取り違え。段階 2-2 からやり直す |
| ほとんど動かない | モータ電源、可動域の端、機械的干渉 |
| 動きが大きすぎる | `--deg` の指定ミス（既定 5°、範囲 0.5〜30°） |

- [ ] 12 軸すべて完了
- [ ] 軸ごとにコミットした

---

## 段階 6: ゼロ点の確定【🟡 全軸を脱力させ、手で姿勢を保持】

> ~~`calib move` はバス単位で `BusRequest::Zero` を投げるため、**実行のたびに
> エンコーダのゼロがずれる**。したがって `calib zero` は必ず段階 5 の後に行う。~~
>
> **2026-08-21 に解消。** 位置の基準を**モータの電源 ON マルチターンフレーム**
> （`0x92` / `0xA4`）へ移した。`rezero`（ホスト側のソフト原点）は使わないので、
> **アプリを何度起動しても、`calib` を何度回してもゼロは動かない。**
> 動くのは**モータの電源を入れ直したとき**だけ。
>
> それでも段階 5 → 6 の順は維持する。`sign` が違うと `zero_pose_rad` の
> 符号も狂うため。

```sh
./target/release/namiashi calib zero --pose constrain --write config/namiashi.toml --config config/namiashi.toml
```

**手順:**
1. 画面に各関節の目標角が表示される
2. **ロボットを手でその姿勢に保持する**
3. Enter → いまの絶対角と保持姿勢の差から `zero_pose_rad` を算出

**モータには何も書かない。** 読み出しだけで求まる:

```
q_model = sign * q_abs + zero_pose_rad
  → zero_pose_rad = q_model(保持姿勢) - sign * q_abs
```

**合格条件:**
- [ ] 保持した姿勢が表示された角度と目視で一致している
- [ ] 12 軸ぶんの `zero_pose_rad` が画面に出る
- [ ] `--write` を付けた場合、設定に書き戻された

**電源を入れ直したら測り直すこと。** MG4005 はアブソリュートエンコーダを
持たないので、電源 OFF/ON でマルチターン角が 0 に戻る。`zero_pose_rad` は
「電源 ON した姿勢のモデル角」なので、**毎回同じ姿勢で電源を入れるなら
測り直しは不要**。設計データの伏せ姿勢オフセットはこの前提のもの。

**★ 追加の検証（推奨）★**

段階 4 の可動域は**ゼロ点確定前の座標系**で測っている。ゼロ点を入れ直した後、
値が実機と整合しているか確認する。

```sh
./target/release/namiashi legs --secs 5 --config config/namiashi.toml
```

- [ ] 各軸を手で端まで動かしたときの読み値が、設定の `min_rad`/`max_rad` の内側に収まる
- [ ] ずれていたら段階 4 をやり直す

---

## 段階 7: 脚を浮かせて自律動作【🔴 複数軸が同時に動く】

> **ここから危険度が上がる。** 必ず脚を浮かせた状態で。

### 7-1. まずモデル上で確認（実機に触れない）

```sh
./target/release/namiashi dump --gait crawl --vx 0.05 --secs 10 --config config/namiashi.toml
```

- [ ] 関節角が可動域内に収まる（クランプ警告が出ない）
- [ ] 校正後の設定で破綻しない

### 7-2. 受信機なしで起動

```sh
./target/release/namiashi run --allow-no-sbus --config config/namiashi.toml
```

- [ ] **脚が浮いている**
- [ ] 電源を即座に切れる位置にいる
- [ ] 起動時のゼロ出しが通る
- [ ] 状態表示の「遅延最大」を記録する ← **これが後の性能評価の基準値になる**
- [ ] 異常ビットが出ない
- [ ] `Ctrl-C` で全軸脱力することを確認

**異常時は即座に電源を切る。** 特に「起立の瞬間に自壊」は符号ミスの典型症状。

---

## 段階 8: プロポ操縦【🔴】

```sh
./target/release/namiashi run --config config/namiashi.toml
```

- [ ] **CH5 が「脱力」位置**で起動する
- [ ] 脚を浮かせたまま CH5 を「起立」へ → 姿勢を確認
- [ ] スティック操作が期待どおりの軸に効く
- [ ] 送信機を切る → **速度 0・その場起立**（フェイルセーフ）を確認
- [ ] `Ctrl-C` で脱力

**接地させるのは、以上がすべて確認できてから。**

---

## 立ち上げ完了後にやること

この文書の範囲外だが、完了したら以下へ進む。

| 項目 | 参照 |
|---|---|
| `response_timeout_ms` を 20 → 1〜2 に戻す | [`runtime_tuning.md`](runtime_tuning.md) 「モータバスの 20 ms 問題」 |
| 20 ms パッチの効果測定（無応答モータでの縮退性能） | 同上 |
| `control.rate_hz` を 200 → 500 | 同上 |
| IPA / cpufreq のジッタ判定（`run` の遅延最大で） | 同上「調査1」 |
| RT 優先度の付与（`chrt -f 50` または systemd） | 同上「RT 優先度の付与」 |
| ウォッチドッグ有効化 | [`boot_config.md`](boot_config.md) |
| `namiashi.service` のインストール | [`runtime_tuning.md`](runtime_tuning.md) |

---

## 記録欄

| 段階 | 日付 | 結果 | 備考 |
|---|---|---|---|
| 0 準備 | 2026-08-21 | ✅ | `response_timeout_ms` 5→20、ブランチ `calib/initial-bringup`。脚は台上で宙吊り、安定化電源 5 A 制限 |
| 1 設定・ポート | 2026-08-21 | ✅ | 関節 18 / nq=13、UART 0〜7 が `motor_map.md` と一致 |
| 2 通信 (scan/legs) | 2026-08-21 | ✅ | **`baud` が 1 M ではなく 2 M だった**（下記）。4 脚とも id 1,2,3。12 軸 `ok=true` / `err=0` / 32〜34 °C |
| 3 センサ (imu/sbus) | 2026-08-21 | ✅ | IMU 200 Hz / resync=0、傾け・揺らしに追従。S.BUS `link=OK` 67 fps / desync=0 |
| 4 可動域 ×12 | | | |
| 5 符号 ×12 | | | |
| 6 ゼロ点 | | | |
| 7 浮かせて run | | | 遅延最大: |
| 8 プロポ操縦 | | | |
