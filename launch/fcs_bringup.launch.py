import os

from launch import LaunchDescription
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    package_dir = get_package_share_directory('ros2_astra_camera')
    parameters_path = os.path.join(
        package_dir, 'launch', 'params', 'astra_pro.yaml')

    start_astra_camera_node_cmd = Node(
        package="ros2_astra_camera",
        executable="astra_camera_node",
        name="astra_camera_node",
        parameters=[parameters_path]
    )

    start_uvc_camera_node_cmd = Node(
        package="ros2_astra_camera",
        executable="uvc_camera_node",
        name="uvc_camera_node",
        output="screen",
        emulate_tty=True,
        parameters=[parameters_path]
    )

    return LaunchDescription([
        start_astra_camera_node_cmd,
        start_uvc_camera_node_cmd
    ])
