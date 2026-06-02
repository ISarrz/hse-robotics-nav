#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

PLANNER=${1:?Usage: $0 <planner> [bag_dir] [world]}
BAG_DIR="${2:-$HOME/bags}"
WORLD="${3:-static}"
MAP=/opt/ros/jazzy/share/turtlebot3_navigation2/map/map.yaml
PARAMS="$REPO_DIR/nav2_params/my_nav2_params_${PLANNER}.yaml"
START_X=-2.0
START_Y=-0.5
GOAL_X=1.8
GOAL_Y=0.5

case "$WORLD" in
    static)     WORLD_FILE="" ;;
    dynamic_1)  WORLD_FILE="$REPO_DIR/worlds/turtlebot3_dynamic_1.world" ;;
    dynamic_10) WORLD_FILE="$REPO_DIR/worlds/turtlebot3_dynamic_10.world" ;;
    dynamic_5)  WORLD_FILE="$REPO_DIR/worlds/turtlebot3_dynamic_5.world" ;;
    dynamic_15) WORLD_FILE="$REPO_DIR/worlds/turtlebot3_dynamic_15.world" ;;
    *) echo "ERROR: unknown world '$WORLD'"; exit 1 ;;
esac

if [[ ! -f "$PARAMS" ]]; then
    echo "ERROR: params file not found: $PARAMS"
    exit 1
fi

export TURTLEBOT3_MODEL=burger
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

echo "[0/6] Killing stale Nav2/Gazebo processes and cleaning DDS shm..."
pkill -9 -f "nav2_bringup" 2>/dev/null || true
pkill -9 -f "component_container" 2>/dev/null || true
pkill -9 -f "lifecycle_manager" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
sleep 4
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
echo "    DDS shm cleaned: $(ls /dev/shm/ 2>/dev/null | grep -c fastrtps || echo 0) fastrtps files remain"

echo "[1/6] Starting Gazebo (world=$WORLD)..."
if [[ -n "$WORLD_FILE" ]]; then
    ros2 launch "$REPO_DIR/launch/my_robot_launch.py" world:="$WORLD_FILE" &
else
    ros2 launch "$REPO_DIR/launch/my_robot_launch.py" &
fi
GAZEBO_PID=$!
echo "    Gazebo PID=$GAZEBO_PID"
if [[ "$WORLD" == "static" ]]; then
    GAZEBO_WAIT=15
else
    GAZEBO_WAIT=30
fi
echo "    Waiting ${GAZEBO_WAIT}s for Gazebo to load..."
sleep "$GAZEBO_WAIT"

MOVER_PID=""
if [[ "$WORLD" != "static" ]]; then
    MOVER_BIN="$REPO_DIR/build/move_cylinders"
    MOVER_CFG="${WORLD_FILE%.world}.cylinders.json"
    if [[ -x "$MOVER_BIN" && -f "$MOVER_CFG" ]]; then
        echo "[1b/6] Starting cylinder mover ($MOVER_CFG)..."
        "$MOVER_BIN" "$MOVER_CFG" --rate 10 &
        MOVER_PID=$!
        echo "    Mover PID=$MOVER_PID"
    else
        echo "    WARNING: mover binary or config missing — cylinders will not move"
        echo "    bin=$MOVER_BIN cfg=$MOVER_CFG"
    fi
fi

echo "[2/6] Starting Nav2 ($PLANNER)..."
ros2 launch nav2_bringup bringup_launch.py \
    map:="$MAP" \
    params_file:="$PARAMS" \
    use_sim_time:=True &
NAV2_PID=$!
echo "    Nav2 PID=$NAV2_PID"

echo "[3/6] Waiting for /odom, then publishing initial pose..."
ODOM_DEADLINE=$((SECONDS + 60))
while [[ $SECONDS -lt $ODOM_DEADLINE ]]; do
    if timeout 2 ros2 topic echo --once /odom 2>/dev/null | grep -q "header"; then
        echo "    /odom ready (${SECONDS}s). Waiting 1s for TF buffer..."
        sleep 1
        break
    fi
    echo "    Waiting for /odom... (${SECONDS}s)"
    sleep 1
done
INITIAL_POSE="{header: {frame_id: 'map'}, pose: {pose: {position: {x: $START_X, y: $START_Y, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}"
for attempt in 1 2 3 4 5; do
    ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "$INITIAL_POSE"
    echo "    Initial pose sent (attempt $attempt). Checking AMCL..."
    sleep 2
    if timeout 4 ros2 topic echo --once /amcl_pose 2>/dev/null | grep -q "header"; then
        echo "    AMCL localized successfully."
        break
    fi
    [[ $attempt -lt 5 ]] && echo "    AMCL not ready, retrying in 2s..." && sleep 2
done

echo "[4/6] Waiting for /navigate_to_pose action server (up to 90s)..."
DEADLINE=$((SECONDS + 90))
NAV_READY=false
while [[ $SECONDS -lt $DEADLINE ]]; do
    if ros2 action list 2>/dev/null | grep -q "navigate_to_pose"; then
        echo "    /navigate_to_pose is ready! (${SECONDS}s)"
        NAV_READY=true
        break
    fi
    echo "    waiting for navigate_to_pose... (${SECONDS}s)"
    sleep 5
done

if [[ "$NAV_READY" != "true" ]]; then
    echo "    WARNING: /navigate_to_pose not ready — test will likely fail"
fi

echo "[5/6] Running test: $PLANNER $WORLD (hard timeout 130s)..."
timeout --signal=KILL 130 bash "$SCRIPT_DIR/record_test.sh" "$PLANNER" "$WORLD" "$BAG_DIR" || \
    echo "    WARNING: record_test.sh hit hard timeout or failed."

echo "[6/6] Analyzing results..."
python3 "$SCRIPT_DIR/analyze_bag.py" "$BAG_DIR/${PLANNER}_${WORLD}" "$GOAL_X" "$GOAL_Y" || true

echo ""
echo "=== Test complete. Shutting down all ROS programs... ==="
[[ -n "$MOVER_PID" ]] && kill "$MOVER_PID" 2>/dev/null || true
kill "$NAV2_PID" 2>/dev/null || true
kill "$GAZEBO_PID" 2>/dev/null || true
sleep 1
pkill -9 -f "move_cylinders" 2>/dev/null || true
pkill -9 -f "nav2_bringup" 2>/dev/null || true
pkill -9 -f "component_container" 2>/dev/null || true
pkill -9 -f "lifecycle_manager" 2>/dev/null || true
pkill -9 -f "ros2 launch" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "parameter_bridge" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pkill -9 -f "ros2 bag record" 2>/dev/null || true
pkill -9 -f "ros2 action send_goal" 2>/dev/null || true
sleep 2
echo "All ROS programs terminated."
