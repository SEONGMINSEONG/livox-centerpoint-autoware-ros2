from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    param_path = LaunchConfiguration("param_path")
    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")
    marker_topic = LaunchConfiguration("marker_topic")
    output_frame_id = LaunchConfiguration("output_frame_id")
    publish_markers = LaunchConfiguration("publish_markers")
    marker_lifetime_sec = LaunchConfiguration("marker_lifetime_sec")
    marker_alpha = LaunchConfiguration("marker_alpha")
    offset_ground = LaunchConfiguration("offset_ground")
    offset_angle_degrees = LaunchConfiguration("offset_angle_degrees")
    undo_ground_offset_for_output = LaunchConfiguration("undo_ground_offset_for_output")
    enable_rear_inference = LaunchConfiguration("enable_rear_inference")
    num_accumulated_frames = LaunchConfiguration("num_accumulated_frames")
    debug_every_n_frames = LaunchConfiguration("debug_every_n_frames")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "param_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("centerpoint"), "cfgs", "centerpoint.yaml"]
                ),
            ),
            DeclareLaunchArgument("input_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("output_topic", default_value="bbox_array"),
            DeclareLaunchArgument("marker_topic", default_value="/bbox_markers"),
            DeclareLaunchArgument("output_frame_id", default_value=""),
            DeclareLaunchArgument("publish_markers", default_value="true"),
            DeclareLaunchArgument("marker_lifetime_sec", default_value="0.2"),
            DeclareLaunchArgument("marker_alpha", default_value="0.45"),
            DeclareLaunchArgument("offset_ground", default_value="0.0"),
            DeclareLaunchArgument("offset_angle_degrees", default_value="0.0"),
            DeclareLaunchArgument("undo_ground_offset_for_output", default_value="false"),
            DeclareLaunchArgument("enable_rear_inference", default_value="true"),
            DeclareLaunchArgument("num_accumulated_frames", default_value="2"),
            DeclareLaunchArgument("debug_every_n_frames", default_value="20"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("centerpoint"), "rviz", "centerpoint_livox.rviz"]
                ),
            ),
            Node(
                package="centerpoint",
                executable="centerpoint_node",
                name="centerpoint_livox",
                output="screen",
                parameters=[
                    {
                        "param_path": param_path,
                        "input_topic": input_topic,
                        "output_topic": output_topic,
                        "marker_topic": marker_topic,
                        "output_frame_id": output_frame_id,
                        "publish_markers": ParameterValue(publish_markers, value_type=bool),
                        "marker_lifetime_sec": ParameterValue(marker_lifetime_sec, value_type=float),
                        "marker_alpha": ParameterValue(marker_alpha, value_type=float),
                        "offset_ground": ParameterValue(offset_ground, value_type=float),
                        "offset_angle_degrees": ParameterValue(offset_angle_degrees, value_type=float),
                        "undo_ground_offset_for_output": ParameterValue(
                            undo_ground_offset_for_output, value_type=bool
                        ),
                        "enable_rear_inference": ParameterValue(enable_rear_inference, value_type=bool),
                        "num_accumulated_frames": ParameterValue(num_accumulated_frames, value_type=int),
                        "debug_every_n_frames": ParameterValue(debug_every_n_frames, value_type=int),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(rviz),
            ),
        ]
    )
