# centerpoint-livox ROS 2 Autoware Integration

This package connects a Livox LiDAR point cloud pipeline to Autoware object detection output.
It runs a Livox-trained CenterPoint model with TensorRT, publishes 3D bounding boxes, and provides RViz visualization through standard ROS 2 markers.

This package is part of a Livox LiDAR to Autoware detection integration and supports the `driver -> sensor preprocessing -> CenterPoint -> adapter -> Autoware detection publish` flow.

The intended flow is:

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

## What Is Included

- `centerpoint`: TensorRT CenterPoint ROS 2 node.
- `livox_ros_driver2`: Livox LiDAR driver. The driver is included in the workspace.
- `ws_msgs`: message package used by `centerpoint-livox`.
- `launch/centerpoint_livox.launch.py`: CenterPoint + RViz launch file.
- `rviz/centerpoint_livox.rviz`: RViz config using only default RViz plugins.

The Autoware adapter is provided from the companion `livox_detection_ws` workspace as `centerpoint_livox_adapter`.

## Source and Modification Notice

This package is based on `https://github.com/Tream733/centerpoint-livox.git`.
The workspace also includes `livox_ros_driver2` from `https://github.com/Livox-SDK/livox_ros_driver2.git` and `ws_msgs` from `https://github.com/Tream733/ws_msgs.git`.
The GitHub page for `centerpoint-livox` may show a fork source such as `gokulp01/ros2-ublox-zedf9p`; this README documents the modified CenterPoint-Livox integration used in this workspace.

Local modifications include:

- ROS 2 launch files for CenterPoint and RViz
- Livox-oriented sensor preprocessing
- default frame accumulation
- rear inference support
- RViz MarkerArray publishing on `/bbox_markers`
- Autoware adapter launch support through the companion `centerpoint_livox_adapter` package

## Sensor Preprocessing

Before inference, the node applies Livox-oriented preprocessing:

- point cloud range filtering: `[0, -44.8, -2, 224, 44.8, 4]`
- optional ground and angle correction
- rear inference by 180 degree point rotation
- frame accumulation, default `num_accumulated_frames=2`

Frame accumulation has no ego-motion compensation. It is useful for stationary sensor tests, but use `num_accumulated_frames:=1` when the vehicle is moving.

## Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | Input point cloud from Livox driver |
| `/bbox_array` | `ws_msgs/msg/BboxArray` | CenterPoint 3D detection boxes |
| `/bbox_markers` | `visualization_msgs/msg/MarkerArray` | RViz boxes |
| `/perception/object_recognition/detection/objects` | `autoware_auto_perception_msgs/msg/DetectedObjects` | Autoware detection output, published by adapter |

## Build

```bash
cd ~/workspace/centerpoint-livox_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run CenterPoint With RViz

This launches CenterPoint and RViz. No extra RViz command is needed.

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/centerpoint-livox_ws/install/setup.bash
ros2 launch centerpoint centerpoint_livox.launch.py
```

## Run CenterPoint + Adapter + RViz

Use this when you want the CenterPoint output to be converted into Autoware detection objects.

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/centerpoint-livox_ws/install/setup.bash
source ~/workspace/livox_detection_ws/install/setup.bash
ros2 launch centerpoint_livox_adapter centerpoint_livox_with_adapter.launch.py
```

## Useful Launch Arguments

| Argument | Default | Description |
| --- | --- | --- |
| `pointcloud_topic` or `input_topic` | `/livox/lidar` | Input PointCloud2 topic |
| `bbox_topic` or `output_topic` | `/bbox_array` | CenterPoint bbox output |
| `marker_topic` | `/bbox_markers` | RViz MarkerArray output |
| `rviz` | `true` | Start RViz automatically |
| `num_accumulated_frames` | `2` | Number of point cloud frames to accumulate |
| `enable_rear_inference` | `true` | Run rear-side inference by rotating points |
| `offset_ground` | `0.0` | Ground z offset before inference |
| `offset_angle_degrees` | `0.0` | Sensor angle correction before inference |

Example without RViz:

```bash
ros2 launch centerpoint_livox_adapter centerpoint_livox_with_adapter.launch.py rviz:=false
```

## RViz

The launch file opens:

```text
rviz/centerpoint_livox.rviz
```

This RViz config shows:

- `/livox/lidar`
- `/bbox_markers`

It does not require Autoware RViz plugins. This avoids crashes caused by old `~/.rviz2/default.rviz` files or missing Autoware RViz plugins.

## References

- [Center-based 3D Object Detection and Tracking](https://arxiv.org/abs/2006.11275)
- [tianweiy/CenterPoint](https://github.com/tianweiy/CenterPoint)
- [Livox-SDK/livox_detection](https://github.com/Livox-SDK/livox_detection)
