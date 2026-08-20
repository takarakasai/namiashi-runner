#!/usr/bin/env bash
# ローカルの兄弟チェックアウトで**併行開発**するための設定を書き出す。
#
# 通常の依存は GitHub の git 依存なので、SBC でも新しい PC でも
# `git clone && cargo build` だけで立ち上がる。**このスクリプトは不要。**
#
# 要るのは「namiashi-runner と一緒に misa-actuator や sbus も直しながら試す」
# ときだけ。`.cargo/config.toml` に `[patch]` を書き出して、ビルドがローカルの
# チェックアウトを見るようにする。
#
# `paths` override ではなく `[patch]` を使うのは、前者が workspace 継承された
# 依存（`sbus-protocol = { workspace = true }` など）とうまく噛み合わず
# 「altered the original list of dependencies」の警告を出すため。
# `[patch]` は go2-gait-runner が使っているのと同じ、素直に効く方法。
#
#   ./scripts/dev-siblings.sh          兄弟を clone / 更新して override を書く
#   ./scripts/dev-siblings.sh --off    override を消して git 依存へ戻す
#
# `.cargo/config.toml` は **追跡していない**（人ごと・マシンごとに違うため）。
set -euo pipefail

cd "$(dirname "$0")/.."
ROOT="$(cd .. && pwd)"
CONFIG=".cargo/config.toml"

if [ "${1:-}" = "--off" ]; then
  rm -f "$CONFIG"
  rmdir .cargo 2>/dev/null || true
  echo "path override を外しました。以降は GitHub の git 依存を使います"
  echo "（Cargo.lock が書き換わっている場合は  git checkout Cargo.lock  で戻す）"
  exit 0
fi

echo "兄弟リポジトリの配置先: $ROOT"

# name<TAB>url<TAB>「パッケージ名=リポジトリ内のパス」を空白区切りで
SIBLINGS=(
  "misa-actuator	https://github.com/takarakasai/misa-actuator.git	misa-actuator=crates/misa-actuator lkmotor-driver=crates/lkmotor-driver"
  "misarta	https://github.com/takarakasai/misarta.git	misarta=."
  "misa-wbc	https://github.com/takarakasai/misa-wbc.git	misa-wbc=."
  "quadruped-gait	https://github.com/takarakasai/quadruped-gait.git	quadruped-gait=quadruped-gait"
  "sbus	https://github.com/takarakasai/sbus.git	sbus=crates/sbus sbus-protocol=crates/sbus-protocol"
  "wit-imu	https://github.com/takarakasai/wit-imu.git	wit-imu=crates/wit-imu"
)

sections=()
for entry in "${SIBLINGS[@]}"; do
  IFS=$'\t' read -r name url subdirs <<<"$entry"
  dir="$ROOT/$name"
  if [ -d "$dir/.git" ]; then
    echo "  [有] $name"
    # 追跡ブランチが未設定のチェックアウトでも動くよう、remote と branch を明示する。
    branch="$(git -C "$dir" rev-parse --abbrev-ref HEAD)"
    git -C "$dir" pull --ff-only origin "$branch" >/dev/null 2>&1 \
      || echo "     ⚠ fast-forward できませんでした（ローカルの変更を確認してください）"
  elif [ -d "$dir" ]; then
    echo "  [?] $name — ディレクトリはあるが git リポジトリではありません: $dir"
  else
    echo "  [無] $name — clone します"
    git clone --recurse-submodules "$url" "$dir"
  fi
  # [patch] は「どの git URL を」「どのパッケージについて」置き換えるかを書く。
  entries=""
  for spec in $subdirs; do
    pkg="${spec%%=*}"
    sub="${spec#*=}"
    path="../$name/$sub"
    entries+="$pkg = { path = \"${path%/.}\" }"$'\n'
  done
  sections+=("[patch.\"$url\"]"$'\n'"$entries")
done

mkdir -p .cargo
{
  echo "# ./scripts/dev-siblings.sh が生成。**コミットしないこと**（.gitignore 済み）。"
  echo "# ローカルの兄弟チェックアウトを見るようにする [patch]。"
  echo "# 戻すには ./scripts/dev-siblings.sh --off"
  echo
  for sec in "${sections[@]}"; do echo "$sec"; done
} > "$CONFIG"

echo
echo "$CONFIG を書きました。以降のビルドはローカルのチェックアウトを見ます"
echo
echo "git 依存へ戻すには: ./scripts/dev-siblings.sh --off"
