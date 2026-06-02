#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

TIMESTAMP=$(date "+%Y%m%d_%H%M%S")
BAG_DIR="${1:-$HOME/bags/full_run_$TIMESTAMP}"
GOAL_X=1.8
GOAL_Y=0.5

PLANNERS=(dijkstra_mppi astar_mppi dstar_mppi dijkstra_ramppi astar_ramppi dstar_ramppi)
WORLDS=(static dynamic_1 dynamic_5 dynamic_10)
N_RUNS=10

mkdir -p "$BAG_DIR"
LOG_DIR="$BAG_DIR/logs"
mkdir -p "$LOG_DIR"
RESULTS_FILE="$BAG_DIR/results_summary.csv"

echo "planner,world,success_count,fail_count,avg_dist_m,avg_path_m,avg_duration_s,avg_near_misses" > "$RESULTS_FILE"
echo "=== Run started at $(date) ===" | tee "$BAG_DIR/run.log"
echo "Bag dir:    $BAG_DIR" | tee -a "$BAG_DIR/run.log"
echo "Goal:       ($GOAL_X, $GOAL_Y)" | tee -a "$BAG_DIR/run.log"
echo "Planners:   ${PLANNERS[*]}" | tee -a "$BAG_DIR/run.log"
echo "Worlds:     ${WORLDS[*]}" | tee -a "$BAG_DIR/run.log"
echo "Runs:       $N_RUNS per scenario" | tee -a "$BAG_DIR/run.log"
echo "" | tee -a "$BAG_DIR/run.log"

source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
export TURTLEBOT3_MODEL=burger

total=$(( ${#PLANNERS[@]} * ${#WORLDS[@]} ))
scenario=0

for WORLD in "${WORLDS[@]}"; do
    for PLANNER in "${PLANNERS[@]}"; do
        scenario=$((scenario + 1))
        TAG="$PLANNER+$WORLD"

        echo "" | tee -a "$BAG_DIR/run.log"
        echo "################################################################" | tee -a "$BAG_DIR/run.log"
        echo "# [$scenario/$total] $TAG  (started $(date '+%H:%M:%S'))" | tee -a "$BAG_DIR/run.log"
        echo "################################################################" | tee -a "$BAG_DIR/run.log"

        success_count=0
        fail_count=0
        sum_dist=0; sum_path=0; sum_dur=0; sum_near=0

        for RUN in $(seq 1 "$N_RUNS"); do
            echo "" | tee -a "$BAG_DIR/run.log"
            echo "### $TAG  run $RUN/$N_RUNS  at $(date '+%H:%M:%S')" | tee -a "$BAG_DIR/run.log"

            pkill -9 -f "gz sim" 2>/dev/null || true
            pkill -9 -f "nav2_bringup" 2>/dev/null || true
            pkill -9 -f "component_container" 2>/dev/null || true
            pkill -9 -f "move_cylinders" 2>/dev/null || true
            pkill -9 -f "parameter_bridge" 2>/dev/null || true
            pkill -9 -f "ros2 bag" 2>/dev/null || true
            sleep 3
            rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true

            TEST_LOG="$LOG_DIR/${PLANNER}_${WORLD}_run${RUN}.log"
            timeout --signal=KILL 360 \
                bash "$SCRIPT_DIR/run_one_test.sh" "$PLANNER" "$BAG_DIR" "$WORLD" \
                > "$TEST_LOG" 2>&1 || \
                echo "    WARNING: run_one_test.sh non-zero exit" | tee -a "$BAG_DIR/run.log"

            BAG_PATH="$BAG_DIR/${PLANNER}_${WORLD}"
            if [[ -d "$BAG_PATH" ]]; then
                OUT=$(python3 "$SCRIPT_DIR/analyze_bag.py" "$BAG_PATH" "$GOAL_X" "$GOAL_Y" 2>&1 || true)
                echo "$OUT" | tee -a "$BAG_DIR/run.log"
                SUC=$(echo "$OUT"    | grep -oP 'Success:\s+\K(YES|NO)' | head -n1)
                DIST=$(echo "$OUT"   | grep -oP 'dist=\K[0-9]+\.[0-9]+'  | head -n1)
                PATH_M=$(echo "$OUT" | grep -oP 'Path length:\s+\K[0-9]+\.[0-9]+' | head -n1)
                DUR_S=$(echo "$OUT"  | grep -oP 'Duration:\s+\K[0-9]+\.[0-9]+'    | head -n1)
                NEAR=$(echo "$OUT"   | grep -oP 'Near-misses:\s+\K[0-9]+'         | head -n1)

                if [[ "$SUC" == "YES" ]]; then
                    success_count=$((success_count + 1))
                    sum_dist=$(awk "BEGIN{printf \"%.3f\", $sum_dist + ${DIST:-0}}")
                    sum_path=$(awk "BEGIN{printf \"%.3f\", $sum_path + ${PATH_M:-0}}")
                    sum_dur=$(awk  "BEGIN{printf \"%.1f\", $sum_dur  + ${DUR_S:-0}}")
                    sum_near=$((sum_near + ${NEAR:-0}))
                    echo "    SUCCESS on run $RUN" | tee -a "$BAG_DIR/run.log"
                    mv "$BAG_PATH" "${BAG_PATH}_run${RUN}_success" 2>/dev/null || true
                else
                    fail_count=$((fail_count + 1))
                    echo "    FAILED on run $RUN (dist=${DIST:-?})" | tee -a "$BAG_DIR/run.log"
                    mv "$BAG_PATH" "${BAG_PATH}_run${RUN}_fail" 2>/dev/null || true
                fi
            else
                fail_count=$((fail_count + 1))
                echo "    NO_BAG on run $RUN" | tee -a "$BAG_DIR/run.log"
            fi
            sleep 3
        done

        if [[ $success_count -gt 0 ]]; then
            avg_dist=$(awk "BEGIN{printf \"%.3f\", $sum_dist / $success_count}")
            avg_path=$(awk "BEGIN{printf \"%.3f\", $sum_path / $success_count}")
            avg_dur=$(awk  "BEGIN{printf \"%.1f\",  $sum_dur  / $success_count}")
            avg_near=$(awk "BEGIN{printf \"%.1f\",  $sum_near / $success_count}")
        else
            avg_dist=""; avg_path=""; avg_dur=""; avg_near=""
        fi

        echo "$PLANNER,$WORLD,$success_count,$fail_count,$avg_dist,$avg_path,$avg_dur,$avg_near" >> "$RESULTS_FILE"
        echo "    SUMMARY: $success_count/$N_RUNS success, $fail_count fail" | tee -a "$BAG_DIR/run.log"

        sleep 3
    done
done

pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "nav2_bringup" 2>/dev/null || true
pkill -9 -f "component_container" 2>/dev/null || true
pkill -9 -f "move_cylinders" 2>/dev/null || true

echo "" | tee -a "$BAG_DIR/run.log"
echo "################################################################" | tee -a "$BAG_DIR/run.log"
echo "# ALL TESTS DONE at $(date)" | tee -a "$BAG_DIR/run.log"
echo "################################################################" | tee -a "$BAG_DIR/run.log"
column -t -s, "$RESULTS_FILE" | tee -a "$BAG_DIR/run.log"
echo "" | tee -a "$BAG_DIR/run.log"
echo "Results CSV: $RESULTS_FILE" | tee -a "$BAG_DIR/run.log"
echo "Log dir:     $LOG_DIR" | tee -a "$BAG_DIR/run.log"
