#!/usr/bin/env bash
# SBC（や新しい PC）で namiashi-runner をビルドできる状態にする。
#
# 依存は兄弟ディレクトリへの path 依存なので、**同じ相対配置**で clone する
# 必要がある。この配置を手で作ると必ずどれか 1 つ抜けるので、スクリプトにした。
#
#   <ルート>/
#   ├── namiashi-runner/   ← このリポジトリ
#   ├── misa-actuator/
#   ├── misarta/
#   ├── misa-wbc/
#   ├── quadruped-gait/
#   ├── sbus/
#   └── wit-imu/
#
# 使い方: リポジトリの中から
#   ./scripts/bootstrap.sh          # 兄弟を clone / 更新して cargo build
#   ./scripts/bootstrap.sh --no-build
set -euo pipefail

cd "$(dirname "$0")/.."
ROOT="$(cd .. && pwd)"
echo "兄弟リポジトリの配置先: $ROOT"

# name<TAB>url。misa-actuator だけ https、他は SSH（それぞれの origin に合わせた）。
SIBLINGS=(
  "misa-actuator	https://github.com/takarakasai/misa-actuator.git"
  "misarta	git@github.com:takarakasai/misarta.git"
  "misa-wbc	git@github.com:takarakasai/misa-wbc.git"
  "quadruped-gait	git@github.com:takarakasai/quadruped-gait.git"
  "sbus	git@github.com:takarakasai/sbus.git"
  "wit-imu	git@github.com:takarakasai/wit-imu.git"
)

missing=0
for entry in "${SIBLINGS[@]}"; do
  name="${entry%%	*}"
  url="${entry##*	}"
  dir="$ROOT/$name"
  if [ -d "$dir/.git" ]; then
    echo "  [有] $name — 更新します"
    git -C "$dir" pull --ff-only || echo "     ⚠ fast-forward できませんでした（手で確認を）"
  elif [ -d "$dir" ]; then
    # ディレクトリはあるが git 管理下にない。clone すると必ず失敗するので、
    # 「取得できなかった」ではなく「これは何なのか」を言う。
    echo "  [?] $name — ディレクトリはあるが git リポジトリではありません: $dir"
    echo "     push 済みのものへ差し替えるか、そのまま使うなら手で管理してください"
    missing=$((missing + 1))
  else
    echo "  [無] $name — clone します: $url"
    # misarta は submodule を持つ構成があるので再帰で取る。
    if ! git clone --recurse-submodules "$url" "$dir"; then
      echo "     ❌ clone に失敗しました: $url"
      missing=$((missing + 1))
    fi
  fi
done

if [ "$missing" -ne 0 ]; then
  cat >&2 <<'MSG'

❌ 取得できなかったリポジトリがあります。
   wit-imu が失敗した場合、まだ GitHub に push されていない可能性があります
   （doc/handover.md §5.1 参照）。PC 側から手でコピーするか、先に push してください。
MSG
  exit 1
fi

if [ "${1:-}" = "--no-build" ]; then
  echo "配置完了（--no-build なのでビルドはしません）"
  exit 0
fi

echo
echo "ビルドします（SBC では 10〜20 分かかることがあります）"
# viz（Zenoh）は SBC では要らないことが多い。要るなら --features viz を足す。
cargo build --release --no-default-features
echo
echo "完了: target/release/namiashi"
echo "次は  ./target/release/namiashi check  →  ports  →  imu / sbus / legs"
