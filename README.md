<!--
SPDX-License-Identifier: Apache-2.0
SPDX-FileCopyrightText: 2026 pang-yann
-->

# Livox Mid-360(s) plugin for Gazebo Sim

This plugin post-processes Gazebo's `gpu_lidar` point cloud to produce
non-repetitive, Livox Mid-360-like scans. It samples the raw regular
azimuth/elevation grid using a configurable Livox scan pattern and publishes
an ordered point cloud.

The result is an approximation: it cannot recover detail absent from the raw
`gpu_lidar` grid. Use a sufficiently dense grid with matching angular bounds.

## Requirements

- ROS 2 Jazzy
- Gazebo Harmonic
- `ros_gz_sim` and `ros_gz_bridge`
- `colcon`

### MID360 reference specifications

These are the nominal sensor characteristics this package approximates:

| Specification | Typical value |
| --- | --- |
| Horizontal FOV | 360° |
| Vertical FOV | −7° to 52° |
| Range precision (1σ) | ≤2 cm at 10 m; ≤4 cm at 0.2 m |
| Angular precision (1σ) | <0.15° |
| Point rate | 200,000 points/s, first return |
| Frame rate | 10 Hz |

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select livox_mid360s_gz
source install/setup.bash
```

## Demo

```bash
ros2 launch livox_mid360s_gz mid360s_demo.launch.py
```

Inspect the sampled output with:

```bash
ros2 topic echo /livox/mid360/points --once
```

## Implementation

The processing pipeline is:

```text
gpu_lidar -> raw PointCloudPacked -> Livox scan-pattern sampler
          -> Livox-like PointCloudPacked -> optional ROS PointCloud2 bridge
```

The `gpu_lidar` sensor publishes to `<topic>/points`. The post-sampler
subscribes to that topic, selects the nearest raw-grid range for each Livox
direction, and publishes a new point cloud.

### Recommended SDF layout

Place the `gpu_lidar` sensor and the System plugin on the same model. Keep the
raw sensor topic separate from the sampled output topic:

```xml
<sensor name="gpu_lidar" type="gpu_lidar">
  <topic>/livox/mid360/raw</topic>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>1440</samples>
        <min_angle>-3.141592653589793</min_angle>
        <max_angle>3.141592653589793</max_angle>
      </horizontal>
      <vertical>
        <samples>251</samples>
        <min_angle>-0.126012280626355</min_angle>
        <max_angle>0.963770320680978</max_angle>
      </vertical>
    </scan>
    <range>
        <min>0.1</min>
        <max>40.0</max>
        <resolution>0.02</resolution>
    </range>
  </ray>
</sensor>

<plugin filename="liblivox_mid360s_gz_system.so"
        name="livox_mid360s_gz::LivoxMid360sSystem">
  <input_topic>/livox/mid360/raw/points</input_topic>
  <output_topic>/livox/mid360/points</output_topic>
  <scan_rate>10</scan_rate>
  <point_rate>200000</point_rate>
</plugin>
```

The example `1440 × 251` source grid uses the official simulator's vertical
range (−7.22° to 55.22°) and gives approximately 0.25° spacing in both axes.
It renders about 3.6 million grid samples per second at 10 Hz. A denser
`2400 × 400` grid gives approximately 0.15° spacing and more closely matches
the MID360's published angular precision, but renders about 9.6 million
samples per second. Choose the denser grid when edge fidelity is important;
use the smaller grid when faster simulation is preferred.

The plugin does not replace `gpu_lidar` or alter Gazebo's collision queries.
The standard `ros_gz_bridge` can convert the output to ROS `PointCloud2`.

### Configuration

Supported optional plugin elements:

- `scan_pattern`: absolute path, or filename below installed `scan_patterns`
- `horizontal_min`, `horizontal_max`: raw grid bounds in radians
- `vertical_min`, `vertical_max`: raw grid bounds in radians
- `azimuth_offset`: fixed yaw rotation applied to every pattern azimuth;
  default `-pi` (−180°)
- `elevation_offset`: default `0`
- `line_count`: default `4`
- `points_per_frame`: default `point_rate / scan_rate`
- `drop_invalid_points`: default `true`

The horizontal and vertical bounds must match the corresponding `gpu_lidar`
SDF bounds. The output includes Livox-like coordinates, intensity, channel
(`ring`/`line`), timing, and `tag` fields. The input timestamp and frame ID are
preserved.

## Attribution

`scan_patterns/mid360.csv` comes from
[Livox-SDK/livox_laser_simulation](https://github.com/Livox-SDK/livox_laser_simulation)
and is licensed under MIT. The original file is kept unchanged; frame
alignment is applied at runtime by adding `azimuth_offset` to each CSV
azimuth. With the default `-pi`, the pattern is rotated 180° around the
sensor's vertical axis to match the Gazebo frame convention.

Referenced implementations are from
[LCAS/livox_laser_simulation_ros2](https://github.com/LCAS/livox_laser_simulation_ros2/blob/main/LICENSE).
