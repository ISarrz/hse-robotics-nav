#!/usr/bin/env python3

import os
import sys
from pathlib import Path
from typing import Dict, List, Tuple

def parse_test_file(filepath: str) -> Tuple[List[float], List[float], List[float], List[float]]:
    astar_times = []
    astar_vals = []
    dlite_times = []
    dlite_vals = []

    try:
        with open(filepath, 'r') as f:
            lines = f.readlines()
            for line in lines[3:]:
                line = line.strip()
                if not line:
                    continue
                parts = [p.strip() for p in line.split('|')]
                if len(parts) >= 7:
                    try:
                        astar_times.append(float(parts[2]))
                        astar_vals.append(float(parts[3]))
                        dlite_times.append(float(parts[5]))
                        dlite_vals.append(float(parts[6]))
                    except (ValueError, IndexError):
                        continue
    except FileNotFoundError:
        pass

    return astar_times, astar_vals, dlite_times, dlite_vals

def calculate_average(values: List[float]) -> float:
    if not values:
        return 0.0
    return sum(values) / len(values)

def extract_steps(filepath: str) -> str:
    try:
        with open(filepath, 'r') as f:
            for line in f:
                if "Detected from header" in line and "objects_per_step" in line:
                    parts = line.split("objects_per_step=")
                    if len(parts) > 1:
                        return parts[1].split()[0]
    except FileNotFoundError:
        pass
    return "?"

def format_line(col1: str, col2: str, col3: str, col1_width: int = 15, col2_width: int = 20, col3_width: int = 20) -> str:
    return f"    {col1:<{col1_width}} {col2:<{col2_width}} {col3:<{col3_width}}"

def main():
    script_dir = Path(__file__).parent
    root_dir = script_dir.parent.parent
    results_dir = root_dir / "results"
    output_file = results_dir / "statistics.txt"

    output_lines = []

    output_lines.append("STATISTICS SUMMARY".ljust(70))
    output_lines.append("=" * 70)
    output_lines.append("")

    test_dirs = ["10_dst", "30_dst", "50_dst", "70_dst", "no_dst"]

    for test_name in test_dirs:
        test_dir = results_dir / test_name
        if not test_dir.exists():
            continue

        output_lines.append(f"Test: {test_name}".ljust(70))
        output_lines.append("---".ljust(70))
        output_lines.append("")

        static_file = test_dir / "static.txt"
        if static_file.exists():
            astar_times, astar_lens, dlite_times, dlite_lens = parse_test_file(str(static_file))

            output_lines.append("  Static Test:".ljust(70))

            astar_avg_time = calculate_average(astar_times)
            astar_avg_len = calculate_average(astar_lens)
            dlite_avg_time = calculate_average(dlite_times)
            dlite_avg_len = calculate_average(dlite_lens)

            output_lines.append(format_line("A*", f"time: {astar_avg_time:.3f}ms", f"length: {astar_avg_len:.1f}"))
            output_lines.append(format_line("D*Lite", f"time: {dlite_avg_time:.3f}ms", f"length: {dlite_avg_len:.1f}"))

        for dynamic_file in sorted(test_dir.glob("dynamic_5000_*.txt")):
            steps = extract_steps(str(dynamic_file))
            astar_times, astar_steps, dlite_times, dlite_steps = parse_test_file(str(dynamic_file))

            output_lines.append(f"  Dynamic Test (steps={steps}):".ljust(70))

            astar_avg_time = calculate_average(astar_times)
            astar_avg_steps = calculate_average(astar_steps)
            dlite_avg_time = calculate_average(dlite_times)
            dlite_avg_steps = calculate_average(dlite_steps)

            output_lines.append(format_line("A*", f"time: {astar_avg_time:.3f}ms", f"steps: {astar_avg_steps:.1f}"))
            output_lines.append(format_line("D*Lite", f"time: {dlite_avg_time:.3f}ms", f"steps: {dlite_avg_steps:.1f}"))

        output_lines.append("")

    with open(output_file, 'w') as f:
        f.write('\n'.join(output_lines))

    print(f"Statistics saved to: {output_file}")
    print('\n'.join(output_lines))

if __name__ == "__main__":
    main()
