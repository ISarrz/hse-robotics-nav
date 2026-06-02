#!/usr/bin/env python3
import csv
import sys

PLANNER_NAMES = {
    "dijkstra_mppi":    "dijkstra + MPPI",
    "astar_mppi":       "astar + MPPI",
    "dstar_mppi":       "dstar + MPPI",
    "dijkstra_ramppi":  "dijkstra + RA-MPPI",
    "astar_ramppi":     "astar + RA-MPPI",
    "dstar_ramppi":     "dstar + RA-MPPI",
}
WORLD_NAMES = {
    "static":     r"static",
    "dynamic_1":  r"dynamic\_1",
    "dynamic_5":  r"dynamic\_5",
    "dynamic_10": r"dynamic\_10",
}

def fmt(v, decimals=3):
    if v == "":
        return "---"
    try:
        f = float(v)
        return f"{f:.{decimals}f}".replace(".", "{,}")
    except ValueError:
        return v

csv_path = sys.argv[1] if len(sys.argv) > 1 else "results_summary.csv"
with open(csv_path, newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        planner = PLANNER_NAMES.get(row["planner"], row["planner"])
        world   = WORLD_NAMES.get(row["world"], row["world"])
        sc = row["success_count"]
        fc = row["fail_count"]
        dist = fmt(row.get("avg_dist_m", ""), 3)
        path = fmt(row.get("avg_path_m", ""), 2)
        dur  = fmt(row.get("avg_duration_s", ""), 1)
        print(f"            {planner:<22} & {world:<12} & {sc} & {fc} & {dist} & {path} & {dur} \\\\\\hline")
