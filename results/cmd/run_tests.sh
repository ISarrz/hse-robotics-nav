#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "Usage: $0 <dst_param> <results_subdir>"
  echo "Example: $0 10 10_dst"
  exit 1
fi

DST_PARAM="$1"
RESULTS_SUBDIR="$2"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="$ROOT_DIR/build"

RESULTS_DIR="$ROOT_DIR/results/$RESULTS_SUBDIR"
CHANGES_DIR="$RESULTS_DIR/changes"

mkdir -p "$RESULTS_DIR" "$CHANGES_DIR"

cd "$BUILD_DIR"

./dynamic_environment generate-points 1000 "$DST_PARAM"

cp "$ROOT_DIR/data/points.txt" "$RESULTS_DIR/points.txt"
cp "$ROOT_DIR/data/map.txt" "$RESULTS_DIR/map.txt"
./dynamic_environment test static > "$RESULTS_DIR/static.txt"

for dst in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15; do
  ./dynamic_environment generate 5000 "$dst"
  cp "$ROOT_DIR/data/changes.txt" "$CHANGES_DIR/changes_5000_$dst.txt"
  ./dynamic_environment test dynamic > "$RESULTS_DIR/dynamic_5000_$dst.txt"
done
