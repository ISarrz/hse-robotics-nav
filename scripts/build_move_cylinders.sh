#!/bin/bash

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
SRC="$SCRIPT_DIR/move_cylinders.cpp"
OUT_DIR="$REPO_DIR/build"
OUT="$OUT_DIR/move_cylinders"

mkdir -p "$OUT_DIR"

source /opt/ros/jazzy/setup.bash

PKG_CFG_PATH="/opt/ros/jazzy/opt/gz_transport_vendor/lib/pkgconfig"
PKG_CFG_PATH="$PKG_CFG_PATH:/opt/ros/jazzy/opt/gz_msgs_vendor/lib/pkgconfig"
PKG_CFG_PATH="$PKG_CFG_PATH:/opt/ros/jazzy/opt/gz_math_vendor/lib/pkgconfig"
PKG_CFG_PATH="$PKG_CFG_PATH:/opt/ros/jazzy/opt/gz_cmake_vendor/lib/pkgconfig"
PKG_CFG_PATH="$PKG_CFG_PATH:/opt/ros/jazzy/opt/gz_utils_vendor/lib/pkgconfig"
export PKG_CONFIG_PATH="$PKG_CFG_PATH${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"

CFLAGS=$(pkg-config --cflags gz-transport13 gz-msgs10)
LIBS=$(pkg-config --libs gz-transport13 gz-msgs10)

echo "Compiling $SRC -> $OUT"
g++ -O2 -std=c++17 "$SRC" $CFLAGS $LIBS -o "$OUT"
echo "Done: $OUT"
