#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

PLANNER=${1:?Usage: $0 <planner> <world> <bag_dir>}
WORLD=${2:?}
BAG_DIR=${3:?}

PARAMS="$REPO_DIR/nav2_params/my_nav2_params_${PLANNER}.yaml"
BAG_NAME="${PLANNER}_${WORLD}"
TIMEOUT=120

GOAL_X=1.8
GOAL_Y=0.5
GOAL_YAW=0.0

if [[ ! -f "$PARAMS" ]]; then
    echo "ERROR: params file not found: $PARAMS"
    exit 1
fi

mkdir -p "$BAG_DIR"
rm -rf "${BAG_DIR}/${BAG_NAME}"

echo "=== Test: planner=$PLANNER  world=$WORLD ==="
echo "    params: $PARAMS"
echo "    bag:    $BAG_DIR/$BAG_NAME"

source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

ros2 bag record \
    --output "${BAG_DIR}/${BAG_NAME}" \
    /odom \
    /amcl_pose \
    /collision_monitor_state \
    /plan \
    /goal_pose \
    /navigate_to_pose/_action/status \
    &
BAG_PID=$!

sleep 2

echo "Sending goal: ($GOAL_X, $GOAL_Y)"
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map', stamp: {sec: 0, nanosec: 0}},
             pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0},
                    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" &
ACTION_PID=$!

echo "Waiting up to ${TIMEOUT}s for goal..."
DEADLINE=$((SECONDS + TIMEOUT))
while [[ $SECONDS -lt $DEADLINE ]]; do
    if ! kill -0 "$ACTION_PID" 2>/dev/null; then
        echo "Action completed before timeout."
        break
    fi
    sleep 2
done
kill "$ACTION_PID" 2>/dev/null || true
wait "$ACTION_PID" 2>/dev/null || true

kill $BAG_PID 2>/dev/null || true
wait $BAG_PID 2>/dev/null || true

echo "=== Done: $BAG_NAME ==="
