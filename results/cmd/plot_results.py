#!/usr/bin/env python3

import matplotlib.pyplot as plt
import matplotlib
from pathlib import Path
from typing import List, Tuple
import re

matplotlib.use('Agg')

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

def extract_steps(filepath: str) -> int:
    try:
        with open(filepath, 'r') as f:
            for line in f:
                if "Detected from header" in line and "objects_per_step" in line:
                    parts = line.split("objects_per_step=")
                    if len(parts) > 1:
                        return int(parts[1].split()[0])
    except (FileNotFoundError, ValueError):
        pass
    return 0


def dynamic_files_by_mode(test_dir: Path, mode: str):
    def numeric_key(path: Path) -> int:
        match = re.search(r"_(\d+)\.txt$", path.name)
        return int(match.group(1)) if match else 10**9

    if mode == "step":
        mode_files = sorted(test_dir.glob("dynamic_step_5000_*.txt"), key=numeric_key)
        if mode_files:
            return mode_files
        return sorted(test_dir.glob("dynamic_5000_*.txt"), key=numeric_key)
    if mode == "change":
        return sorted(test_dir.glob("dynamic_change_5000_*.txt"), key=numeric_key)
    return []

def main():
    script_dir = Path(__file__).parent
    root_dir = script_dir.parent.parent
    results_dir = root_dir / "results"

    test_names = ["0_dst", "10_dst", "30_dst", "50_dst", "70_dst"]
    colors = ['gray', 'blue', 'green', 'orange', 'red']

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5))

    astar_all_times = []
    dlite_all_times = []

    for test_name in test_names:
        test_dir = results_dir / test_name
        if not test_dir.exists():
            continue

        for mode in ("step", "change"):
            for dynamic_file in dynamic_files_by_mode(test_dir, mode):
                astar_times, _, dlite_times, _ = parse_test_file(str(dynamic_file))
                if astar_times:
                    astar_all_times.extend(astar_times)
                if dlite_times:
                    dlite_all_times.extend(dlite_times)

    astar_y_max = max(astar_all_times) * 1.05 if astar_all_times else 1.0
    dlite_y_max = max(dlite_all_times) * 1.05 if dlite_all_times else 1.0

    for test_name, color in zip(test_names, colors):
        test_dir = results_dir / test_name
        if not test_dir.exists():
            continue

        for mode, marker, linestyle, mode_label in (
            ("step", "o", "-", "step"),
            ("change", "^", "--", "change"),
        ):
            data_points = []
            for dynamic_file in dynamic_files_by_mode(test_dir, mode):
                steps = extract_steps(str(dynamic_file))
                astar_times, _, _, _ = parse_test_file(str(dynamic_file))

                if astar_times and steps >= 0:
                    data_points.append((steps, calculate_average(astar_times)))

            if data_points:
                data_points.sort(key=lambda x: x[0])
                steps_list = [x[0] for x in data_points]
                times_list = [x[1] for x in data_points]
                ax1.plot(
                    steps_list,
                    times_list,
                    marker=marker,
                    linestyle=linestyle,
                    label=f"{test_name} ({mode_label})",
                    color=color,
                    linewidth=2,
                )

    ax1.set_xlabel('Количество изменяющихся препятствий за ход', fontsize=11)
    ax1.set_ylabel('Среднее время (мс)', fontsize=11)
    ax1.set_title('Динамические тесты A* (step vs change)', fontsize=12, fontweight='bold')
    ax1.set_ylim(0, astar_y_max)
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    for test_name, color in zip(test_names, colors):
        test_dir = results_dir / test_name
        if not test_dir.exists():
            continue

        data_points = []

        for dynamic_file in dynamic_files_by_mode(test_dir, "step"):
            steps = extract_steps(str(dynamic_file))
            _, _, dlite_times, _ = parse_test_file(str(dynamic_file))

            if dlite_times and steps >= 0:
                data_points.append((steps, calculate_average(dlite_times)))

        if data_points:
            data_points.sort(key=lambda x: x[0])
            steps_list = [x[0] for x in data_points]
            times_list = [x[1] for x in data_points]
            ax2.plot(steps_list, times_list, marker='s', label=test_name, color=color, linewidth=2)

    ax2.set_xlabel('Количество изменяющихся препятствий за ход', fontsize=11)
    ax2.set_ylabel('Среднее время (мс)', fontsize=11)
    ax2.set_title('Динамические тесты D* Lite', fontsize=12, fontweight='bold')
    ax2.set_ylim(0, dlite_y_max)
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    output_file = results_dir / "global_dynamic_comparison.png"
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Graph saved to: {output_file}")

    fig2, ax3 = plt.subplots(figsize=(10, 6))

    static_data = {}
    for test_name, color in zip(test_names, colors):
        test_dir = results_dir / test_name
        if not test_dir.exists():
            continue

        static_file = test_dir / "static.txt"
        if static_file.exists():
            astar_times, astar_lens, dlite_times, dlite_lens = parse_test_file(str(static_file))

            static_data[test_name] = {
                'astar_time': calculate_average(astar_times),
                'astar_len': calculate_average(astar_lens),
                'dlite_time': calculate_average(dlite_times),
                'dlite_len': calculate_average(dlite_lens)
            }

    import numpy as np

    if not static_data:
        print("No static data found, skipping static graph")
        print("Graphs generated successfully!")
        return

    x = np.arange(len(static_data))

    astar_times = [static_data[name]['astar_time'] for name in static_data.keys()]
    dlite_times = [static_data[name]['dlite_time'] for name in static_data.keys()]

    ax3.plot(x, astar_times, marker='o', label='A*', color='skyblue', linewidth=2, markersize=8)
    ax3.plot(x, dlite_times, marker='s', label='D*Lite', color='salmon', linewidth=2, markersize=8)

    ax3.set_xlabel('Конфигурация теста', fontsize=11)
    ax3.set_ylabel('Среднее время (мс)', fontsize=11)
    ax3.set_title('Сравнение статических тестов', fontsize=12, fontweight='bold')
    ax3.set_xticks(x)
    ax3.set_xticklabels(static_data.keys())
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    output_file2 = results_dir / "global_static_comparison.png"
    plt.savefig(output_file2, dpi=150, bbox_inches='tight')
    print(f"Graph saved to: {output_file2}")

    print("Graphs generated successfully!")

if __name__ == "__main__":
    main()
