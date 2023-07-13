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
            "subscribe_camera_info_topic_name",
            default_value="/camera/color/camera_info",
            description="topic name for camera info"
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
        ),

        DeclareLaunchArgument(
            "conf_bbox_thresh",
            default_value="0.9",
            description="bbox thresh"
        ),

        DeclareLaunchArgument(
            "conf_nms_thresh",
            default_value="0.45",
            description="nms thresh"
        )
    ]

    def get_camera_node(package, plugin, name):
        package_dir = get_package_share_directory('ros2_astra_camera')
        parameters_path = os.path.join(
            package_dir, 'launch', 'params', 'astra_pro.yaml')

        return ComposableNode(
            package=package,
            plugin=plugin,
            name=name,
            parameters=[parameters_path],

        )

    # FIXME: refactor
    def get_fcs_container(camera_nodes):
        return ComposableNodeContainer(
            name='rt_fcs',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=camera_nodes + [
                ComposableNode(
                    package="rt_fcs_tf2",
                    plugin="rt_fcs::FcsTf2Node",
                    name="rt_fcs_tf2",
                ),

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
                        "conf_bbox_thresh": LaunchConfiguration("conf_bbox_thresh"),
                        "conf_nms_thresh": LaunchConfiguration("conf_nms_thresh"),
                        "subscribe_camera_info_topic_name": LaunchConfiguration("subscribe_camera_info_topic_name"),
                    }],
                )
            ],
            output='both',
            emulate_tty=True,
            on_exit=Shutdown(),
        )
    # FIXME
    uvc_camera_node = get_camera_node(
        'ros2_astra_camera', 'ros2_astra_camera::CameraDriver', 'uvc_camera_node')

    astra_camera_node = get_camera_node(
        'ros2_astra_camera', 'ros2_astra_camera::AstraDriver', 'astra_camera_node')

    fcs = get_fcs_container(
        [astra_camera_node, uvc_camera_node])

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='both',
        emulate_tty=True,
        on_exit=Shutdown(),
        # FIXME: use arguments file
        arguments=["serial", "--dev", "/dev/ttyACM0"]
    )

    return LaunchDescription(
        launch_args + [
            micro_ros_agent,
            fcs
        ])
