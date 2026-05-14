#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Running all test suites..."
echo "============================"
echo ""

echo "Running 10_dst tests..."
bash "$SCRIPT_DIR/run_tests.sh" 0 0_dst
echo "✓ 10_dst completed"
echo ""

echo "Running 10_dst tests..."
bash "$SCRIPT_DIR/run_tests.sh" 10 10_dst
echo "✓ 10_dst completed"
echo ""

echo "Running 30_dst tests..."
bash "$SCRIPT_DIR/run_tests.sh" 30 30_dst
echo "✓ 30_dst completed"
echo ""

echo "Running 50_dst tests..."
bash "$SCRIPT_DIR/run_tests.sh" 50 50_dst
echo "✓ 50_dst completed"
echo ""

echo "Running 70_dst tests..."
bash "$SCRIPT_DIR/run_tests.sh" 70 70_dst
echo "✓ 70_dst completed"
echo ""

echo "============================"
echo "All test suites completed!"
