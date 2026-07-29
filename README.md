<!-- 
SPDX-License-Identifier: Apache-2.0
SPDX-FileCopyrightText: 2026 pang-yann 
-->

# Livox Mid-360 Gazebo Sim plugin

GPU-accelerated Livox Mid-360 approximation for ROS 2 Jazzy and Gazebo
Harmonic.

Gazebo's built-in `gpu_lidar` renders a regular azimuth/elevation grid. This
plugin selects grid returns in Livox scan-pattern order and projects each
selected range along the requested Livox direction. Collision queries remain
GPU accelerated.

Stock `GpuLidarSensor` cannot render arbitrary per-ray directions. Accuracy
therefore depends on the source grid resolution, especially near depth
discontinuities.

## Output

The plugin consumes `gz.msgs.PointCloudPacked` from the built-in
`gpu_lidar` `<topic>/points` endpoint and publishes another
`gz.msgs.PointCloudPacked` containing:

- `x`, `y`, `z`: exact Livox direction with nearest GPU-grid range
- `intensity`
- `ring` and `line`: Livox channel in `[0, 3]`
- `time`: point offset in seconds
- `offset_time`: point offset in nanoseconds
- `tag`

The input simulation timestamp and frame ID are preserved. The standard
`ros_gz_bridge` PointCloud2 conversion retains these fields.

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

ROS output:

```bash
ros2 topic echo /livox/mid360/points --once
```

## Plugin configuration

Attach the System plugin to the model containing the `gpu_lidar`:

```xml
<plugin filename="liblivox_mid360s_gz_system.so"
        name="livox_mid360s_gz::LivoxMid360sSystem">
  <input_topic>/livox/mid360/raw/points</input_topic>
  <output_topic>/livox/mid360/points</output_topic>
  <scan_rate>10</scan_rate>
  <point_rate>200000</point_rate>
</plugin>
```

Supported optional elements:

- `scan_pattern`: absolute path, or filename below installed `scan_patterns`
- `horizontal_min`, `horizontal_max`: source GPU grid bounds in radians
- `vertical_min`, `vertical_max`: source GPU grid bounds in radians
- `azimuth_offset`: pattern-frame correction; default `-pi`
- `elevation_offset`: default `0`
- `line_count`: default `4`
- `points_per_frame`: default `point_rate / scan_rate`
- `drop_invalid_points`: default `true`

Plugin bounds must match the `gpu_lidar` SDF bounds.

## Attribution

`scan_patterns/mid360.csv` comes from
[Livox-SDK/livox_laser_simulation](https://github.com/Livox-SDK/livox_laser_simulation) licensed under MIT. The original file is
kept unchanged; frame correction is applied at runtime with
`azimuth_offset`.

Referenced implementations against [LCAS/livox_laser_simulation_ros2](https://github.com/LCAS/livox_laser_simulation_ros2/blob/main/LICENSE)