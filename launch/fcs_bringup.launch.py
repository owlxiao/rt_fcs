import os

from launch import LaunchDescription
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import GroupAction, Shutdown
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            "is_preview",
            default_value="true",
            description=""
        ),

        DeclareLaunchArgument(
            "num_classes",
            default_value="1",
            description="num classes"
        ),

        DeclareLaunchArgument(
            "engine_file_path",
            # FIXME
            default_value="./install/detector/share/detector/model/model_trt.engine",
            description="engine file path"
        ),

        DeclareLaunchArgument(
            "subscribe_image_topic_name",
            default_value="/camera/color/image_raw",
            description="topic name for source image"
        ),

        DeclareLaunchArgument(
            "publish_objects_topic_name",
            default_value="/detector/objects",
            description="topic name for publishing objects"
        ),

        DeclareLaunchArgument(
            "class_labels_path",
            default_value="./install/detector/share/detector/labels/names.txt",
            description="path to class labels"
        )
    ]

    def get_camera_node(package, plugin):
        package_dir = get_package_share_directory('ros2_astra_camera')
        parameters_path = os.path.join(
            package_dir, 'launch', 'params', 'astra_pro.yaml')

        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            parameters=[parameters_path],

        )

    def get_camera_detector_container(camera_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                camera_node,
                ComposableNode(
                    package='detector',
                    plugin='rt_vision::DetectorNode',
                    name='detector',
                    parameters=[{
                        "is_preview": LaunchConfiguration("is_preview"),
                        "engine_file_path": LaunchConfiguration("engine_file_path"),
                        "num_classes": LaunchConfiguration("num_classes"),
                        "subscribe_image_topic_name": LaunchConfiguration("subscribe_image_topic_name"),
                        "publish_objects_topic_name": LaunchConfiguration("publish_objects_topic_name"),
                        "class_labels_path": LaunchConfiguration("class_labels_path"),
                    }],
                )
            ],
            output='both',
            emulate_tty=True,
            on_exit=Shutdown(),
        )

    astra_camera_node = get_camera_node(
        'ros2_astra_camera', 'ros2_astra_camera::CameraDriver')

    cam_detector = get_camera_detector_container(astra_camera_node)

    return LaunchDescription(
        launch_args + [
            cam_detector
        ])
