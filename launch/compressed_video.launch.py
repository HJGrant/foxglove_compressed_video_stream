from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Path to the other launch file
    oakd_launch = os.path.join(
        get_package_share_directory('depthai_ros_driver'),
        'launch', 'camera.launch.py'
    )

    # Include another launch file
    include_other_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(oakd_launch)
    )

    # Launch a node
    video_stream_node = Node(
        package="foxglove_compressed_video",
        executable="foxglove_compressed_video_stream",
        name="foxglove_compressed_video_stream_node",
        output="screen",
        remappings=[
            ("/image_raw", "/oak/rgb/image_raw"),
            ("/compressed_video", "/oak/compressed_video")
        ]
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge_node'
    )

    return LaunchDescription([
        include_other_launch,
        video_stream_node,
        foxglove_bridge
    ])
