from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    start_rosbridge_arg = DeclareLaunchArgument(
        'start_rosbridge',
        default_value='true',
        description='If true, launch rosbridge_server (set false when rosbridge is already running)',
    )

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            )
        ),
        launch_arguments={
            'max_queue_size': '1',
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_rosbridge')),
    )

    web_gui_node = Node(
        package='ros_slam_webui',
        executable='web_server',
        name='ros_slam_webui_node',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        start_rosbridge_arg,
        rosbridge_launch,
        web_gui_node,
    ])
