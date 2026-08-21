# articara で実機を可視化する

SBC（`radxa-cubie-a7z`）が姿勢を Zenoh へ流し、PC の articara が描く。
**SBC にディスプレイは要らない。**

作成日: 2026-08-21（実機組み上がり直後、段階 2-4 で実際に使った構成）

---

## 何を流すかを間違えないこと

`--viz` は 3 つのコマンドに付くが、**流すものが違う**。

| コマンド | 流すもの | モータ | 用途 |
|---|---|---|---|
| `legs --viz` | **エンコーダの実測角** | 🟢 **触れない** | 実機が今どうなっているかを見る |
| `run --viz` | モータへ行く**目標角** | 🔴 動く | 指令どおりの姿勢を見る |
| `dump --viz` | 歩容の計算結果 | 🟢 実機不要 | 実機なしで歩容を確認 |

**`run --viz` は「指令どおりの姿勢」しか描かない。** 実機がその通り動いたかは
映らないので、**実機の確認には使えない**。取り違えると「画面で合っているから
実機も合っている」と誤解する。

立ち上げで実機を確かめたいなら `legs --viz`。起動時に

```
ライブ可視化: **実測角**を配信します（目標角ではありません）
```

と出るので、そこで確認できる。

---

## 準備

### PC 側

モデルは [`namiashi_description`](https://github.com/takarakasai/namiashi_description)
にある。**SBC から scp する必要は無い**（SBC 側の `models/` も同じものの submodule）。

```sh
git clone https://github.com/takarakasai/namiashi_description.git
```

articara は `viz` フィーチャつきでビルドする。

```sh
cd articara && cargo build --release --features viz
```

### SBC 側

`viz` は既定で有効なので、通常のビルドでよい。
`--no-default-features` でビルドした場合は `--viz` が無言で効かなくなる。

---

## 手順

### 1. SBC で配信を始める

```sh
cd ~/work/namiashi-runner
./target/release/namiashi legs --secs 0 --viz --config config/namiashi.toml
```

`--secs 0`（または `--forever`）で Ctrl-C まで回り続ける。

期待するログ:

```
脚バスを開きました（指令は送りません。観測: Ctrl-C まで）
  FL → /dev/ttyCH9344USB0
  ...
Zenoh can be reached at: tcp/192.168.0.21:33969      ← 到達アドレス
Listening scout messages on 224.0.0.224:7446         ← マルチキャスト探索
ライブ可視化: 'go2/gait/planned' へ 50 Hz で配信します
  ライブ可視化: **実測角**を配信します（目標角ではありません）
```

### 2. PC で articara を起動する

```sh
cd articara
cargo run --release --features viz -- --model ../namiashi_description/namiashi.misa
```

### 3. articara で購読を始める

**Live gait feed** パネルにキーを入れて Start。

| 欄 | 値 |
|---|---|
| キー | `go2/gait/planned`（`--viz-key` で変えていなければ） |
| エンドポイント | マルチキャストが通るなら**空でよい**。通らなければ下記 |

---

## ネットワーク

同一 LAN でマルチキャストが通れば**設定は要らない**。SBC は
`224.0.0.224:7446` で scout し、`tcp/<SBC の IP>:<動的ポート>` で待つ。

### マルチキャストが通っているかを確かめる

**SBC 側だけで分かること**（`legs --viz` を動かした状態で）:

```sh
ss -ulnp | grep 7446          # 224.0.0.224:7446 に bind しているか
ip maddr show wlan0 | grep 01:00:5e:00:00:e0   # group に join しているか
ip link show wlan0            # MULTICAST フラグが立っているか
```

`01:00:5e:00:00:e0` は `224.0.0.224` に対応するリンク層アドレス
（IPv4 マルチキャストの MAC は `01:00:5e` + group の下位 23 ビット）。
これが出ていれば join できている。

**端から端まで通っているか**は [`scout-check.py`](scout-check.py) で見る。
zenoh が使う group を**受動的に**覗くだけなので、`legs --viz` を動かしたまま
実行してよい。

```sh
# SBC と PC の両方で（相手側で articara を起動した状態で）
python3 doc/scout-check.py
```

```
224.0.0.224:7446 を受信中（15 秒）。自分の IP: ['127.0.0.1', '192.168.0.21']

  192.168.0.21    自分          (3 bytes)
  192.168.0.30    **他ホスト**  (52 bytes)     ← これが出れば通っている
```

**自分の IP しか出なければ届いていない。** 相手側で zenoh アプリ（articara）が
動いているかをまず確認し、動いているのに出ないなら経路で落ちている。

> **WiFi 越しは通らないことがよくある。** AP のクライアント分離や IGMP snooping
> で無線クライアント間のマルチキャストが落とされるため。SBC は wlan0 なので、
> PC も無線だと特に踏みやすい。**その場合はマルチキャストを諦めて下記 A に
> 切り替えるのが早い。**

通らない場合は 2 通り。

### A. 固定ポートで待ち受けて、articara から繋ぐ

```sh
# SBC
./target/release/namiashi legs --secs 0 --viz --viz-endpoint tcp/0.0.0.0:7447 \
    --config config/namiashi.toml
# articara のエンドポイント欄: tcp/192.168.0.21:7447
```

> **`--viz-endpoint` は「待ち受け側」の設定である。**
> 内部では `listen/endpoints` を設定し、**同時にマルチキャスト探索を切る**。
> 「PC の articara に繋ぎに行く指定」だと誤解すると、いくら待っても繋がらない。

### B. SSH トンネル

直接繋げない場合（別セグメント、ファイアウォール）。

```sh
# PC 側で
ssh -L 7447:127.0.0.1:7447 takara@192.168.0.21
# SBC 側:      --viz-endpoint tcp/127.0.0.1:7447
# articara 側: tcp/127.0.0.1:7447
```

---

## 確認できること / できないこと

| | |
|---|---|
| ✅ `(バス, id)` → 関節の対応 | FL の thigh を動かして、画面で FL の thigh が動くか |
| ✅ 符号の向き | 曲げた向きと画面の向きが一致するか |
| ❌ **角度の絶対値** | **ゼロ点未校正のうちは合わない**（段階 6 まで） |
| ❌ 腕 | `arm_pitch_joint` は**映らない**（下記） |

**絶対角が合っていなくても異常ではない。** 段階 6 のゼロ出しを通すまで、表示は
「エンコーダの生値 − 未設定のゼロ点」であって機構的な角度ではない。
そこまでの間に見るのは**動きの対応と符号**だけ。

**腕は映らない。** `GaitVizFrame` が脚 12 関節ぶんしか運ばない器（Go2 向け）の
ため。`arm_pitch_joint` は articara 側で動かないままになる。異常ではない。

---

## 繋がらないときの切り分け

**上から順に潰す。** どこで切れているかで原因が変わる。

### 1. SBC 側で publisher が開いているか

```
ライブ可視化: 'go2/gait/planned' へ 50 Hz で配信します
```

出ていなければ `--viz` が効いていない。`--no-default-features` でビルドした
バイナリではないか確認する（その場合 `--viz` は**無言で**何もしない）。

### 2. Zenoh が到達可能アドレスを出しているか

```
Zenoh can be reached at: tcp/192.168.0.21:33969
```

`127.0.0.1` しか出ていなければ、そのアドレスへは PC から繋げない。

### 3. PC から SBC に届くか

```sh
ping 192.168.0.21
nc -vz 192.168.0.21 7447      # 固定ポートにした場合
```

### 4. articara がフレームを受けているか

キーが一致しているか。SBC 側で `--viz-key` を変えたなら articara 側も同じ値に。

### 5. 姿勢が出るが動かない

`legs` のテキスト出力側で `q=[...]` が動いているか確認する。動いていなければ
可視化ではなく**バスの問題**。段階 2-2 / 2-3 に戻る。

---

## 関連

- 立ち上げ手順のなかでの位置づけ: [`bringup_checklist.md`](bringup_checklist.md) §2-4
- SBC + PC で分ける構成の背景: `handover.md` §5.6
- モデルの置き場と submodule: `README.md`
