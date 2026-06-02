#!/usr/bin/env python3
import json
import math
import os
import random

WORLD_SRC = "/opt/ros/jazzy/share/turtlebot3_gazebo/worlds/turtlebot3_world.world"

R = 0.10
H = 0.4
Z = H / 2

CORRIDOR_COORDS = (-1.6, -0.55, 0.55, 1.6)
LINE_HALF = 1.6


def gen_corridor(seed):
    rng = random.Random(seed)
    axis = rng.choice(('x', 'y'))
    perp = rng.choice(CORRIDOR_COORDS)
    amp = LINE_HALF
    center = 0.0
    period = rng.uniform(45.0, 60.0)
    phase = rng.uniform(0.0, 2 * math.pi)
    return axis, perp, amp, center, period, phase


def cylinder_sdf(i, axis, perp, amp, center, period, phase):
    init = center + amp * math.sin(phase)
    if axis == 'x':
        x0, y0 = init, perp
    else:
        x0, y0 = perp, init
    return f"""
    <model name="cylinder_{i}">
      <static>true</static>
      <pose>{x0:.3f} {y0:.3f} {Z:.3f} 0 0 0</pose>
      <link name="body">
        <kinematic>true</kinematic>
        <gravity>false</gravity>
        <inertial>
          <mass>1000.0</mass>
          <inertia>
            <ixx>10.0</ixx><iyy>10.0</iyy><izz>10.0</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{R}</radius>
              <length>{H}</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{R}</radius>
              <length>{H}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def make_world(n_cylinders, out_path):
    with open(WORLD_SRC) as f:
        content = f.read()

    configs = []
    if n_cylinders >= 1:
        configs.append(('y', -0.55, 0.7, 0.50, 30.0, 0.0))
    for i in range(1, n_cylinders):
        configs.append(gen_corridor(seed=i * 100 + 42))
    cylinders = "\n".join(
        cylinder_sdf(i, *cfg) for i, cfg in enumerate(configs)
    )
    content = content.replace("</world>", cylinders + "\n  </world>")

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        f.write(content)

    config_path = out_path.replace(".world", ".cylinders.json")
    with open(config_path, "w") as f:
        json.dump({
            "z": Z,
            "cylinders": [
                {
                    "name": f"cylinder_{i}",
                    "axis": axis, "perp": perp,
                    "amplitude": amp, "center": center,
                    "period": period, "phase": phase,
                }
                for i, (axis, perp, amp, center, period, phase) in enumerate(configs)
            ],
        }, f, indent=2)

    print(f"Written: {out_path}  ({n_cylinders} cylinders)")
    print(f"         {config_path}")


if __name__ == "__main__":
    repo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    base = os.path.join(repo_dir, "worlds")
    make_world(1,  os.path.join(base, "turtlebot3_dynamic_1.world"))
    make_world(5,  os.path.join(base, "turtlebot3_dynamic_5.world"))
    make_world(10, os.path.join(base, "turtlebot3_dynamic_10.world"))
    print("Done.")
