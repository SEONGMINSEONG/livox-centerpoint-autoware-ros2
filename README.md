# Livox CenterPoint to Autoware Detection Workspace

This ROS 2 workspace connects a Livox LiDAR point cloud pipeline to Autoware object detection output through a TensorRT CenterPoint detector.

The Livox driver is included in this workspace. The full runtime flow is:

```text
Livox LiDAR
  -> livox_ros_driver2
  -> /livox/lidar (sensor_msgs/PointCloud2)
  -> sensor preprocessing
  -> CenterPoint TensorRT inference
  -> /bbox_array (ws_msgs/BboxArray)
  -> centerpoint_livox_adapter
  -> /perception/object_recognition/detection/objects
     (autoware_auto_perception_msgs/DetectedObjects)
```

RViz visualization is published in parallel:

```text
/bbox_array -> /bbox_markers (visualization_msgs/MarkerArray) -> RViz
```

## Repository Layout

```text
src/
  centerpoint-livox/   CenterPoint TensorRT detector and ROS 2 node
  livox_ros_driver2/   Livox LiDAR driver
  ws_msgs/             Message definitions used by centerpoint-livox
```

The Autoware adapter package is in the companion workspace:

```text
~/workspace/livox_detection_ws/src/centerpoint_livox_adapter
```

## Source and Modification Notice

This workspace is assembled from existing open-source projects and local ROS 2 integration code:

| Path | Source |
| --- | --- |
| `centerpoint-livox/` | Based on `https://github.com/Tream733/centerpoint-livox.git` |
| `livox_ros_driver2/` | From `https://github.com/Livox-SDK/livox_ros_driver2.git` |
| `ws_msgs/` | From `https://github.com/Tream733/ws_msgs.git` |
| `centerpoint_livox_adapter` | Local adapter package in the companion `livox_detection_ws` workspace |

Notes:

- The `centerpoint-livox` GitHub page may show a fork source such as `gokulp01/ros2-ublox-zedf9p`; this workspace uses the CenterPoint-Livox detector code and adds the ROS 2 Autoware integration described here.
- Local changes add Livox-oriented sensor preprocessing, default frame accumulation, rear inference, RViz MarkerArray output, RViz auto-launch, and Autoware adapter launch support.
- This repository is intended to document the modified integration state, not only the upstream detector.

## Sensor Preprocessing

The CenterPoint node applies preprocessing before inference:

- point cloud range filtering: `[0, -44.8, -2, 224, 44.8, 4]`
- optional ground and angle correction
- rear inference by rotating points 180 degrees
- frame accumulation, default `num_accumulated_frames=2`

Frame accumulation has no ego-motion compensation. Use `num_accumulated_frames:=1` when the vehicle is moving.

## Main Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | Input point cloud from Livox driver |
| `/bbox_array` | `ws_msgs/msg/BboxArray` | CenterPoint detection boxes |
| `/bbox_markers` | `visualization_msgs/msg/MarkerArray` | RViz box visualization |
| `/perception/object_recognition/detection/objects` | `autoware_auto_perception_msgs/msg/DetectedObjects` | Autoware detection output, published by adapter |

## Build

```bash
cd ~/workspace/centerpoint-livox_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run CenterPoint Only With RViz

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/centerpoint-livox_ws/install/setup.bash
ros2 launch centerpoint centerpoint_livox.launch.py
```

## Run CenterPoint + Autoware Adapter + RViz

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/centerpoint-livox_ws/install/setup.bash
source ~/workspace/livox_detection_ws/install/setup.bash
ros2 launch centerpoint_livox_adapter centerpoint_livox_with_adapter.launch.py
```

RViz opens automatically and displays:

- `/livox/lidar`
- `/bbox_markers`

Autoware receives detection objects on:

```text
/perception/object_recognition/detection/objects
```

## Useful Launch Arguments

| Argument | Default | Description |
| --- | --- | --- |
| `input_topic` / `pointcloud_topic` | `/livox/lidar` | Input PointCloud2 topic |
| `output_topic` / `bbox_topic` | `/bbox_array` | CenterPoint bbox output |
| `marker_topic` | `/bbox_markers` | RViz MarkerArray output |
| `rviz` | `true` | Start RViz automatically |
| `num_accumulated_frames` | `2` | Number of point cloud frames to accumulate |
| `enable_rear_inference` | `true` | Run rear-side inference by rotating points |
| `offset_ground` | `0.0` | Ground z offset before inference |
| `offset_angle_degrees` | `0.0` | Sensor angle correction before inference |

Disable RViz:

```bash
ros2 launch centerpoint_livox_adapter centerpoint_livox_with_adapter.launch.py rviz:=false
```
