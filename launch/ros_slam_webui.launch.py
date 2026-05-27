from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Rosbridge launch
    # ── QoS / 성능 파라미터 ──────────────────────────────────────────────────
    # max_queue_size: rosbridge 내부 수신 큐 깊이 (기본 100 → 1 로 줄여
    #   RELIABLE 대용량 토픽(PointCloud2)의 backpressure·burst 억제)
    # ────────────────────────────────────────────────────────────────────────
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
        }.items()
    )

    # Web GUI node
    web_gui_node = Node(
        package='ros_slam_webui',
        executable='web_server',
        name='ros_slam_webui_node',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        rosbridge_launch,
        web_gui_node
    ])
