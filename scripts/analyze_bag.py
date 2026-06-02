#!/usr/bin/env python3
import sys
import math
import os

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


GOAL_TOLERANCE = 0.35


def open_bag(bag_path):
    reader = rosbag2_py.SequentialReader()
    storage_opts = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    conv_opts = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    reader.open(storage_opts, conv_opts)
    return reader


def get_type_map(reader):
    return {meta.name: meta.type for meta in reader.get_all_topics_and_types()}


def analyze(bag_path, goal_x=1.5, goal_y=1.5):
    if not os.path.exists(bag_path):
        print(f"ERROR: bag not found: {bag_path}")
        sys.exit(1)

    reader = open_bag(bag_path)
    type_map = get_type_map(reader)

    amcl_msgs = []
    odom_msgs = []
    near_miss_count = 0

    while reader.has_next():
        topic, data, t_ns = reader.read_next()

        if topic == '/amcl_pose' and topic in type_map:
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            amcl_msgs.append((t_ns, x, y))

        elif topic == '/odom' and topic in type_map:
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            odom_msgs.append((t_ns, x, y))

        elif topic == '/collision_monitor_state' and topic in type_map:
            try:
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                if hasattr(msg, 'polygons_triggered') and msg.polygons_triggered:
                    near_miss_count += 1
            except Exception:
                pass

    poses = amcl_msgs if amcl_msgs else odom_msgs
    frame = "map" if amcl_msgs else "odom"

    if not poses:
        print("ERROR: no pose messages in bag")
        return None

    last_x, last_y = poses[-1][1], poses[-1][2]
    dist_to_goal = math.sqrt((last_x - goal_x)**2 + (last_y - goal_y)**2)
    success = dist_to_goal < GOAL_TOLERANCE

    path_length = 0.0
    src = odom_msgs if odom_msgs else amcl_msgs
    for i in range(1, len(src)):
        dx = src[i][1] - src[i-1][1]
        dy = src[i][2] - src[i-1][2]
        path_length += math.sqrt(dx*dx + dy*dy)

    t_start = poses[0][0]
    t_end = poses[-1][0]
    duration_s = (t_end - t_start) / 1e9

    bag_name = os.path.basename(bag_path)
    print(f"\n{'='*52}")
    print(f"Bag:          {bag_name}")
    print(f"Goal:         ({goal_x}, {goal_y})  [map frame]")
    print(f"Final pose:   ({last_x:.3f}, {last_y:.3f})  [{frame} frame]  n={len(poses)}")
    print(f"{'='*52}")
    print(f"Success:      {'YES ✓' if success else 'NO  ✗'}  (dist={dist_to_goal:.3f} m, tol={GOAL_TOLERANCE} m)")
    print(f"Path length:  {path_length:.3f} m  (from {len(odom_msgs)} odom msgs)")
    print(f"Duration:     {duration_s:.1f} s")
    print(f"Near-misses:  {near_miss_count}")
    print(f"{'='*52}\n")

    result = {
        'bag': bag_name,
        'success': success,
        'dist_to_goal_m': round(dist_to_goal, 3),
        'dist_m': round(dist_to_goal, 3),
        'path_m': round(path_length, 3),
        'path_length_m': round(path_length, 3),
        'duration_s': round(duration_s, 1),
        'near_misses': near_miss_count,
        'n_poses': len(poses),
        'n_odom': len(odom_msgs),
    }

    import json
    json_path = os.path.join(os.path.dirname(bag_path), 'analysis.json')
    with open(json_path, 'w') as f:
        json.dump(result, f)
    print(f"Analysis saved: {json_path}")

    return result


if __name__ == '__main__':
    bag_path = sys.argv[1] if len(sys.argv) > 1 else None
    goal_x = float(sys.argv[2]) if len(sys.argv) > 2 else 1.8
    goal_y = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5

    if not bag_path:
        print("Usage: python3 analyze_bag.py <bag_path> [goal_x goal_y]")
        sys.exit(1)

    analyze(bag_path, goal_x, goal_y)
