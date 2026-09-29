#!/usr/bin/env python3
"""
Generate Gazebo Harmonic SDF for a 15m x 15m maze world.
Converts a 10x10 binary matrix into merged SDF collision/visual boxes with
dead-ends, loops, cylinder obstacles, and an ultra-narrow 0.40m gap.
"""

import argparse
import os
from typing import List, Tuple

# 10x10 grid: 1 = wall, 0 = path (SPEC.md Section 3.2)
MAZE_GRID = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 1, 0, 0, 0, 0, 1],
    [1, 0, 1, 0, 1, 0, 1, 1, 0, 1],
    [1, 0, 1, 0, 0, 0, 0, 1, 0, 1],
    [1, 0, 1, 1, 1, 1, 0, 1, 0, 1],
    [1, 0, 0, 0, 0, 1, 0, 0, 0, 1],
    [1, 1, 1, 0, 1, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 0, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
]


def run_length_merge_row(grid: List[List[int]], cell_size: float = 1.5, wall_height: float = 1.2) -> List[Tuple[float, float, float, float, float, float]]:
    """
    Merge adjacent wall cells in each row into single bounding boxes.
    Returns list of (x, y, z, size_x, size_y, size_z).
    """
    boxes = []
    rows = len(grid)
    cols = len(grid[0]) if rows > 0 else 0

    for r in range(rows):
        c = 0
        while c < cols:
            if grid[r][c] == 1:
                start_c = c
                while c < cols and grid[r][c] == 1:
                    c += 1
                length = c - start_c
                center_x = (start_c + (length - 1) / 2.0 + 0.5) * cell_size
                center_y = (r + 0.5) * cell_size
                size_x = length * cell_size
                size_y = cell_size
                boxes.append((center_x, center_y, wall_height / 2.0, size_x, size_y, wall_height))
            else:
                c += 1
    return boxes


def generate_maze_sdf(
    grid: List[List[int]] = MAZE_GRID,
    cell_size: float = 1.5,
    wall_height: float = 1.2,
) -> str:
    """Generate the full valid Gazebo Harmonic SDF string for maze world."""
    wall_boxes = run_length_merge_row(grid, cell_size, wall_height)

    # Ultra-narrow gap (0.40m) placed at corridor cell (col 3, row 6 -> x=5.25, y=9.75)
    # Between cell (3,6) and adjacent wall: constrict gap to exactly 0.40m
    constriction_box = (4.70, 9.75, wall_height / 2.0, 1.10, 0.40, wall_height)

    # 3 Cylinder obstacles in corridors (SPEC.md Section 3.2)
    cylinders = [
        (3.75, 2.25, 0.5, 0.20, 1.0),   # cell (2, 1) corridor
        (9.75, 5.25, 0.5, 0.20, 1.0),   # cell (6, 3) corridor
        (7.50, 11.25, 0.5, 0.20, 1.0),  # cell (5, 7) corridor
    ]

    links_xml = []
    for idx, (x, y, z, sx, sy, sz) in enumerate(wall_boxes):
        links_xml.append(f"""      <collision name="wall_col_{idx}">
        <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
        <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
      <visual name="wall_vis_{idx}">
        <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
        <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        <material>
          <ambient>0.25 0.27 0.30 1</ambient>
          <diffuse>0.40 0.42 0.45 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>""")

    cx, cy, cz, csx, csy, csz = constriction_box
    links_xml.append(f"""      <collision name="narrow_gap_col">
        <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>
        <geometry><box><size>{csx:.3f} {csy:.3f} {csz:.3f}</size></box></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
      <visual name="narrow_gap_vis">
        <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>
        <geometry><box><size>{csx:.3f} {csy:.3f} {csz:.3f}</size></box></geometry>
        <material>
          <ambient>0.45 0.25 0.20 1</ambient>
          <diffuse>0.65 0.35 0.25 1</diffuse>
        </material>
      </visual>""")

    for c_idx, (cx, cy, cz, cr, cl) in enumerate(cylinders):
        links_xml.append(f"""      <collision name="cyl_col_{c_idx}">
        <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>
        <geometry><cylinder><radius>{cr:.3f}</radius><length>{cl:.3f}</length></cylinder></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
      <visual name="cyl_vis_{c_idx}">
        <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>
        <geometry><cylinder><radius>{cr:.3f}</radius><length>{cl:.3f}</length></cylinder></geometry>
        <material>
          <ambient>0.15 0.30 0.45 1</ambient>
          <diffuse>0.25 0.50 0.70 1</diffuse>
        </material>
      </visual>""")

    model_body = "\n".join(links_xml)

    sdf_content = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="maze">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <gravity>0 0 -9.81</gravity>
    <physics name="accurate_omni_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <scene>
      <ambient>0.35 0.38 0.42 1</ambient>
      <background>0.08 0.10 0.14 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>

    <light name="sun" type="directional">
      <pose>7.5 7.5 15 0.35 -0.45 0</pose>
      <direction>-0.3 0.3 -0.9</direction>
      <diffuse>0.9 0.9 0.95 1</diffuse>
      <specular>0.3 0.3 0.35 1</specular>
      <attenuation><range>50</range></attenuation>
      <cast_shadows>true</cast_shadows>
    </light>

    <model name="maze_floor">
      <static>true</static>
      <pose>7.5 7.5 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>18 18</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>18 18</size></plane></geometry>
          <material>
            <ambient>0.10 0.12 0.16 1</ambient>
            <diffuse>0.15 0.18 0.22 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="maze">
      <static>true</static>
      <link name="walls">
{model_body}
      </link>
    </model>
  </world>
</sdf>
"""
    return sdf_content


def main():
    parser = argparse.ArgumentParser(description="Generate 15x15m maze SDF for Gazebo Harmonic")
    parser.add_argument(
        "-o", "--output",
        default=os.path.join(os.path.dirname(os.path.dirname(__file__)), "worlds", "maze.sdf"),
        help="Target SDF file path"
    )
    args = parser.parse_args()

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    content = generate_maze_sdf()
    with open(args.output, "w", encoding="utf-8") as f:
        f.write(content)
    print(f"Generated maze SDF at {args.output}")


if __name__ == "__main__":
    main()
