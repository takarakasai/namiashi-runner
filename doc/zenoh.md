# 依頼: 歩容ライブ可視化フレームの配信実装

このリポジトリのコントローラから、歩容の可視化フレームを Zenoh で配信してほしい。
受信側（articara の Live feed）は実装・検証済みで、下記の契約どおりに送れば
「実測=不透明 / 指令=半透明ゴースト」の重畳表示がそのまま成立する。

## 正典

実装契約はこの2つ。相違があればコード側が正。
- 契約文書: https://github.com/takarakasai/quadruped-gait/blob/main/doc/viz_publisher.md
- 型と規約: https://github.com/takarakasai/quadruped-gait/blob/main/quadruped-gait/src/viz.rs
  （`cargo doc -p quadruped-gait --features viz --open` の `viz` モジュール）

まず上の2つを読むこと。以下は要点の写しで、判断に迷ったら上を見る。

## 依存

```toml
quadruped-gait = { git = "https://github.com/takarakasai/quadruped-gait.git", features = ["viz"] }
zenoh = "1.9"
serde_json = "1"
```

必要 rev は `d2ce2f3` 以降（最低でも `1500acc`。`GaitVizFrame::pose_rp` と
`VIZ_KEY_MEASURED` を含む）。`viz` feature が `GaitVizFrame` に serde derive を付ける。

## 送るもの

`quadruped_gait::viz::GaitVizFrame` を JSON で put する。
Encoding は `zenoh::bytes::Encoding::APPLICATION_JSON`。

2ストリーム、キーは必ず別にする（チャネルが latest-wins なので1本だと上書き合戦になる）:

| 用途 | キー定数 | 既定値 | 中身 |
|---|---|---|---|
| 指令 | `viz::VIZ_KEY_PLANNED` | `go2/gait/planned` | コントローラが出した目標 |
| 実測 | `viz::VIZ_KEY_MEASURED` | `go2/gait/measured` | ロボットから読み戻した実状態 |

実測を持たないなら planned だけでよい（受信側はそれでモデルを駆動し、ゴーストを描かない）。

フレームの中身:

| フィールド | 型 | 内容 |
|---|---|---|
| `version` | u32 | `VIZ_FORMAT_VERSION`。不一致は受信側が捨てる |
| `seq` | u64 | 単調増加。planned と measured の対で同じ値 |
| `t_s` | f64 | 走行開始からの秒 |
| `pose` | [f64;4] | 胴体 world `[x, y, z, yaw]`（m, rad）。z は接地面からの胴体高さ |
| `pose_rp` | [f64;2] | 胴体 `[roll, pitch]`（rad）。水平なら `[0,0]` |
| `joints` | [f64;12] | slot 順 **FL, FR, RL, RR** × (hip, thigh, calf) |
| `stance` | [bool;4] | 同 slot 順の接地フラグ |

## 事故りやすい点（この順に間違えやすい）

1. **関節角の符号規約**。`GaitVizFrame::from_output()` は IK 規約で埋めるので、
   実機に送るのと同じ符号補正をかけてから publish する。忘れると膝が逆に曲がって描画される。
   逆に、ロボットから読み戻した角度は既にモデル規約なので符号補正は不要で、
   **slot 順への並べ替えだけ**でよい。
2. **planned と measured は同一 tick・同一 `seq`**。受信側は2ストリームを独立に
   サンプリングするので、これが両者のズレを1配信周期に抑える唯一の保証になる。
3. **measured の `pose` には実測値を入れる**。指令姿勢を入れると2体が構造的に重なり、
   絵は綺麗になるがロボットが実際どこにいるかが消える。重ねたい受信側は自分でアンカーし直す。
4. **put を制御ループから出す**。`session.put(..).wait()` はブロッキングのネットワーク
   呼び出しで、JSON 直列化も安くない。有界チャネル（深さ8程度）で publisher スレッドへ渡し、
   満杯なら `try_send` の失敗として捨てる。可視化は lossy でよい。捨てた数を数えて
   終了時に出すと、詰まりが「健全な配信」に見えなくなる。
5. **最初の状態読み戻しが済むまで measured を送らない**。ゼロ姿勢のフレームは
   「崩れ落ちたロボット」として描画される。

配信レートは制御周期から間引く（100 Hz 相当が既定。500 Hz 制御なら5 tick に1回）。

## Zenoh 設定

通常は multicast の自動探索でよい。同一ホスト / WSL2 / multicast 不可の環境では、
配信側が `listen/endpoints = ["tcp/0.0.0.0:7447"]` で待ち受け、
`scouting/multicast/enabled = false` にする（受信側が connect する）。
配信元が複数ある場合は別ポートで待ち受けること。

## 参考実装

https://github.com/takarakasai/go2-gait-runner の `src/main.rs` の `mod viz_pub`。
`VizPublisher::new` がセッションを持つスレッドを立て、`publish()` はフレーム構築と
`try_send` だけを行う形になっている。丸ごと真似してよい。

## 完了の確認

実機なしで確認できる:
1. 配信側を `listen/endpoints = tcp/0.0.0.0:7447` 相当で起動
2. 受信側 articara を `cargo run --features viz -- <model>.misa` で起動
3. Live feed (Zenoh) 窓を開き、endpoint に `tcp/127.0.0.1:7447` を入れて Subscribe
4. `● target — frame #N` と `● measured — frame #N` の両方が増えれば経路 OK
5. anchor を `full` にして関節差が、`world` にして胴体差が見えることを確認

## 拡張したくなったら

`GaitVizFrame` に足りないものがあれば、`#[serde(default)]` 付きの追加フィールドなら
`VIZ_FORMAT_VERSION` 据え置きで前方後方互換のまま拡張できる（`pose_rp` がその前例）。
その場合は quadruped-gait 側の変更と push が必要なので、こちらに一声かけてほしい。
