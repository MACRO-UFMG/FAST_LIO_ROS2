from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("input_topic", default_value="/camera/camera/imu"),
            DeclareLaunchArgument("output_topic", default_value="/camera/camera/imu_body"),
            DeclareLaunchArgument("output_frame_id", default_value="camera_imu_body_frame"),
            DeclareLaunchArgument("queue_depth", default_value="200"),
            Node(
                package="imu_frame_adapter",
                executable="imu_frame_adapter_node",
                name="imu_frame_adapter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "output_frame_id": LaunchConfiguration("output_frame_id"),
                        "queue_depth": LaunchConfiguration("queue_depth"),
                    }
                ],
            ),
        ]
    )
