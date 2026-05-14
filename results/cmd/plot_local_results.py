#!/usr/bin/env python3

import re
from pathlib import Path
from typing import Dict, List

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

matplotlib.use("Agg")

COLORS = {
    "MPPI":          "#4878CF",
    "RA-MPPI":       "#D65F5F",
    "MPPI-pred":     "#2E8B57",
    "RA-MPPI-pred":  "#9467BD",
}
LABELS = list(COLORS.keys())
OFFSETS = {
    "MPPI":         1,
    "RA-MPPI":      7,
    "MPPI-pred":    13,
    "RA-MPPI-pred": 19,
}


class PlannerStats:
    def __init__(self):
        self.times: List[float] = []
        self.steps: List[float] = []
        self.tort:  List[float] = []
        self.turn:  List[float] = []
        self.nm:    List[float] = []
        self.found = 0
        self.total = 0

    def success_rate(self):
        return self.found / self.total if self.total > 0 else 0.0

    @staticmethod
    def avg(lst):
        return sum(lst) / len(lst) if lst else 0.0

    @staticmethod
    def std(lst):
        if len(lst) < 2:
            return 0.0
        m = sum(lst) / len(lst)
        return (sum((x - m) ** 2 for x in lst) / len(lst)) ** 0.5


def parse_file(filepath: str) -> Dict[str, PlannerStats]:
    stats = {name: PlannerStats() for name in LABELS}
    try:
        with open(filepath) as f:
            for line in f:
                parts = [p.strip() for p in line.split("|")]
                if len(parts) < 25:
                    continue
                try:
                    float(parts[2])
                except ValueError:
                    continue
                for name, off in OFFSETS.items():
                    s = stats[name]
                    s.total += 1
                    s.times.append(float(parts[off + 1]))
                    if parts[off].strip() == "Path found":
                        s.found += 1
                        s.steps.append(float(parts[off + 2]))
                        s.tort.append(float(parts[off + 3]))
                        s.turn.append(float(parts[off + 4]))
                        s.nm.append(float(parts[off + 5]))
    except FileNotFoundError:
        pass
    return stats


def plot_static(results_dir: Path, output_path: Path) -> None:
    stats = parse_file(str(results_dir / "local_static.txt"))
    if not stats["MPPI"].total:
        print("No local static data found, skipping static plot")
        return

    colors = [COLORS[n] for n in LABELS]
    x = np.arange(len(LABELS))
    w = 0.6

    def vals(fn):
        return [fn(stats[n]) for n in LABELS]

    metrics = [
        ("Доля успешных навигаций (%)",     "%",    vals(lambda s: s.success_rate() * 100),       True),
        ("Среднее число шагов",             "шаги", vals(lambda s: s.avg(s.steps)),               False),
        ("Стабильность (σ шагов)",          "шаги", vals(lambda s: s.std(s.steps)),               False),
        ("Извилистость пути",               "",     vals(lambda s: s.avg(s.tort)),                False),
        ("Среднее время планирования (мс)", "мс",   vals(lambda s: s.avg(s.times)),               False),
        ("Риск (доля шагов у препятствий)", "",     vals(lambda s: s.avg(s.nm)),                  False),
    ]

    fig, axes = plt.subplots(2, 3, figsize=(16, 9))
    fig.suptitle("Статическая среда: сравнение MPPI / RA-MPPI и их prediction-вариантов",
                 fontsize=14, fontweight="bold")

    for ax, (title, ylabel, vals_, pct) in zip(axes.flat, metrics):
        bars = ax.bar(x, vals_, width=w, color=colors)
        ax.set_title(title, fontsize=11)
        if ylabel:
            ax.set_ylabel(ylabel)
        ax.set_xticks(x)
        ax.set_xticklabels(LABELS, rotation=15, ha="right", fontsize=9)
        ax.grid(axis="y", alpha=0.3)
        if pct:
            ax.set_ylim(0, 110)
        for bar, val in zip(bars, vals_):
            fmt = f"{val:.1f}%" if pct else f"{val:.3f}"
            ax.text(bar.get_x() + bar.get_width() / 2,
                    bar.get_height() * 1.02,
                    fmt, ha="center", va="bottom", fontsize=9, fontweight="bold")

    plt.tight_layout()
    plt.savefig(str(output_path), dpi=150, bbox_inches="tight")
    print(f"Local static comparison saved to: {output_path}")
    plt.close()


def _load_dynamic(results_dir: Path) -> Dict[int, Dict[str, PlannerStats]]:
    data = {}
    for lpath in sorted(results_dir.glob("local_dynamic_*.txt")):
        m = re.search(r"_(\d+)\.txt$", lpath.name)
        if not m:
            continue
        n = int(m.group(1))
        s = parse_file(str(lpath))
        if s["MPPI"].total:
            data[n] = s
    return data


def plot_dynamic(results_dir: Path, output_path: Path) -> None:
    data = _load_dynamic(results_dir)
    if not data:
        print("No local dynamic data found, skipping dynamic plot")
        return

    xs = sorted(data.keys())

    def series(name, fn):
        return [fn(data[x][name]) for x in xs]

    panels = [
        ("Доля успешных навигаций (%)",     "%",    lambda s: s.success_rate() * 100),
        ("Среднее число шагов",             "шаги", lambda s: s.avg(s.steps)),
        ("Стабильность (σ шагов)",          "шаги", lambda s: s.std(s.steps)),
        ("Извилистость пути",               "",     lambda s: s.avg(s.tort)),
        ("Среднее время планирования (мс)", "мс",   lambda s: s.avg(s.times)),
        ("Риск (доля шагов у препятствий)", "",     lambda s: s.avg(s.nm)),
    ]

    fig, axes = plt.subplots(2, 3, figsize=(18, 9))
    fig.suptitle("Динамическая среда: MPPI / RA-MPPI и их prediction-варианты",
                 fontsize=14, fontweight="bold")

    xlabel = "Препятствий перемещается за ход"
    markers = {"MPPI": "o", "RA-MPPI": "s", "MPPI-pred": "^", "RA-MPPI-pred": "D"}
    linestyles = {"MPPI": "-", "RA-MPPI": "--", "MPPI-pred": "-.", "RA-MPPI-pred": ":"}

    for ax, (title, ylabel, fn) in zip(axes.flat, panels):
        for name in LABELS:
            ys = series(name, fn)
            ax.plot(xs, ys, marker=markers[name], linestyle=linestyles[name],
                    label=name, color=COLORS[name], linewidth=2, markersize=7)
        ax.set_title(title, fontsize=11)
        ax.set_xlabel(xlabel)
        if ylabel:
            ax.set_ylabel(ylabel)
        ax.legend(fontsize=9)
        ax.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(str(output_path), dpi=150, bbox_inches="tight")
    print(f"Local dynamic comparison saved to: {output_path}")
    plt.close()


def main() -> None:
    script_dir  = Path(__file__).parent
    root_dir    = script_dir.parent.parent
    results_dir = root_dir / "results"
    local_dir   = results_dir / "local"
    if not local_dir.exists():
        print(f"No local results directory found at {local_dir}")
        return
    plot_static(local_dir,  results_dir / "local_static_comparison.png")
    plot_dynamic(local_dir, results_dir / "local_dynamic_comparison.png")
    print("Done.")


if __name__ == "__main__":
    main()
