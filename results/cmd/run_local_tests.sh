#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="$ROOT_DIR/build"
RESULTS_DIR="$ROOT_DIR/results/local"

mkdir -p "$RESULTS_DIR"
cd "$BUILD_DIR"

echo "Running global static baseline..."
./dynamic_environment test static > "$RESULTS_DIR/global_static.txt"

echo "Running local static tests (MPPI / RMPPI)..."
./dynamic_environment test local-static > "$RESULTS_DIR/local_static.txt"

for dst in 1 5 10; do
  echo "Dynamic tests with objects_per_step=$dst..."
  ./dynamic_environment generate 5000 "$dst"
  ./dynamic_environment test dynamic step  > "$RESULTS_DIR/global_dynamic_step_$dst.txt"
  ./dynamic_environment test local-dynamic > "$RESULTS_DIR/local_dynamic_$dst.txt"
done

echo "Done. Results saved to $RESULTS_DIR/"
