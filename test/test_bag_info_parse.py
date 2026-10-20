"""bag 정보 파싱: 토픽별 Type·메시지 수(topic_details) (Task 3.4 백엔드)."""
from types import SimpleNamespace

import pytest

web_server = pytest.importorskip('ros_slam_webui.web_server')

# 실제 Jazzy `ros2 bag info` 출력
SAMPLE = """
Files:             20230531_bag_0.db3
Bag size:          15.6 GiB
Storage id:        sqlite3
ROS Distro:        jazzy
Duration:          561.473462220s
Start:             May 31 2023 19:25:02.848784971 (1685528702.848784971)
End:               May 31 2023 19:34:24.322247191 (1685529264.322247191)
Messages:          185236
Topic information: Topic: /camera/color/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 16832 | \
Serialization Format: cdr
                   Topic: /camera/color/image | Type: sensor_msgs/msg/Image | Count: 16832 | Serialization Format: cdr
                   Topic: /imu | Type: sensor_msgs/msg/Imu | Count: 140368 | Serialization Format: cdr
                   Topic: /livox/lidar | Type: livox_ros_driver2/msg/CustomMsg | Count: 5592 | Serialization Format: cdr
                   Topic: /pose/position | Type: geometry_msgs/msg/PointStamped | Count: 5612 | Serialization Format: cdr
Service:           0
Service information:
"""

SAMPLE_TOPICS = ['/camera/color/camera_info', '/camera/color/image', '/imu', '/livox/lidar', '/pose/position']


def test_parse_topics_and_counts():
    r = web_server.parse_ros2_bag_info(SAMPLE)
    assert r['topics'] == SAMPLE_TOPICS
    assert r['topic_details'][0] == {
        'name': '/camera/color/camera_info', 'type': 'sensor_msgs/msg/CameraInfo', 'count': 16832}
    assert r['topic_details'][2] == {'name': '/imu', 'type': 'sensor_msgs/msg/Imu', 'count': 140368}
    assert r['topic_details'][3] == {'name': '/livox/lidar', 'type': 'livox_ros_driver2/msg/CustomMsg', 'count': 5592}
    assert abs(r['duration'] - 561.473462220) < 1e-6


def test_missing_fields_are_tolerated():
    r = web_server.parse_ros2_bag_info('Topic: /x | Count: abc')
    assert r['topic_details'] == [{'name': '/x', 'type': '', 'count': None}]


def test_empty_output():
    assert web_server.parse_ros2_bag_info('') == {'topics': [], 'topic_details': [], 'duration': 0.0}


class _Logger:
    def info(self, *_a, **_k):
        pass

    warn = error = info


def test_get_bag_info_ros2_returns_topic_details(monkeypatch):
    node = SimpleNamespace(
        bag_path='/data/20230531_bag', bag_topics=[], bag_duration=0.0,
        get_logger=lambda: _Logger(), _detect_bag_format=lambda _p: 'ros2_db3')
    monkeypatch.setattr(web_server.subprocess, 'run',
                        lambda *_a, **_k: SimpleNamespace(returncode=0, stdout=SAMPLE, stderr=''))
    info = web_server.WebGUINode.get_bag_info(node)
    assert info['topics'] == SAMPLE_TOPICS
    assert info['topic_details'][4] == {
        'name': '/pose/position', 'type': 'geometry_msgs/msg/PointStamped', 'count': 5612}
    assert info['bag_type'] == 'ros2'
    assert info['bag_format'] == 'ros2_db3'
    assert node.bag_topics == SAMPLE_TOPICS
    assert abs(node.bag_duration - 561.473462220) < 1e-6


class _FakeRos1Reader:
    def __init__(self, _path):
        self.topics = {
            '/velodyne_points': SimpleNamespace(msgtype='sensor_msgs/msg/PointCloud2', msgcount=1200),
            '/legacy': SimpleNamespace(msgtype='sensor_msgs/msg/Imu'),
        }
        self.start_time = 0
        self.end_time = 2_000_000_000

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        return False


def test_get_ros1_bag_info_topic_details(monkeypatch):
    rosbag1 = pytest.importorskip('rosbags.rosbag1')
    monkeypatch.setattr(rosbag1, 'Reader', _FakeRos1Reader)
    node = SimpleNamespace(bag_path='/data/x.bag', bag_topics=[], bag_duration=0.0, get_logger=lambda: _Logger())
    info = web_server.WebGUINode._get_ros1_bag_info(node)
    assert info['topic_details'] == [
        {'name': '/velodyne_points', 'type': 'sensor_msgs/msg/PointCloud2', 'count': 1200},
        {'name': '/legacy', 'type': 'sensor_msgs/msg/Imu', 'count': None},
    ]
    assert [t['name'] for t in info['topics']] == ['/velodyne_points', '/legacy']
    assert info['topics'][0]['publishable'] is True
    assert node.bag_topics == ['/velodyne_points', '/legacy']
