#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.serialization import serialize_message
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, Imu, CameraInfo, PointCloud2
from geometry_msgs.msg import PointStamped
from rosgraph_msgs.msg import Clock
from cv_bridge import CvBridge
import cv2
import struct
import glob
import threading
import asyncio
import json
import os
import time
import socketserver
from http.server import HTTPServer, SimpleHTTPRequestHandler


class ThreadedHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
    """HTTP 서버: 요청마다 새 스레드로 처리해 저장 작업 중 폴링 응답 지연 제거"""
    daemon_threads = True
from urllib.parse import parse_qs, urlparse
import subprocess
import signal
import yaml
from pathlib import Path as PathLib
import rosbag2_py

# ── Optional: numpy (PointCloud2 binary 파싱용) ──────────────────────────────
try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False
    print("Warning: numpy not available. PC2 WebSocket server disabled.")

# ── Optional: websockets (PC2 Binary WebSocket 서버용) ───────────────────────
try:
    import websockets
    WEBSOCKETS_AVAILABLE = True
except ImportError:
    WEBSOCKETS_AVAILABLE = False
    print("Warning: websockets not available. PC2 WebSocket server disabled.")

# Try to import ruamel.yaml for better formatting
try:
    from ruamel.yaml import YAML
    RUAMEL_AVAILABLE = True
except ImportError:
    RUAMEL_AVAILABLE = False
    print("Warning: ruamel.yaml not available. Comments and formatting may not be preserved.")

# Try to import Livox custom messages
try:
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
    LIVOX_AVAILABLE = True
except ImportError:
    LIVOX_AVAILABLE = False
    print("Warning: livox_ros_driver2 messages not available. LiDAR publishing will be disabled.")

# Try to import pose_graph_optimization service
try:
    from pose_graph_optimization.srv import SaveMap
    SAVEMAP_AVAILABLE = True
except ImportError:
    SAVEMAP_AVAILABLE = False
    print("Warning: pose_graph_optimization SaveMap service not available. Map saving will be disabled.")

# Global variables for signal handling
_web_server = None
_ros_node = None


class Ros1BagPlayerThread(threading.Thread):
    """ROS1 .bag 파일을 rosbags로 읽어 rclpy Publisher로 실시간 ROS2 publish하는 스레드.

    Attributes:
        bag_path (str): ROS1 .bag 파일 경로
        topics (list[str]): publish할 토픽 이름 목록 (빈 리스트 = 전체)
        playback_rate (float): 재생 속도 배율 (1.0 = 원본 속도)
        ros_node (rclpy.node.Node): publisher를 생성할 ROS2 노드 참조
    """

    def __init__(self, bag_path, topics, playback_rate, ros_node):
        super().__init__(daemon=True)
        self._bag_path = bag_path
        self._topics = set(topics) if topics else None  # None = 전체 토픽
        self._playback_rate = max(playback_rate, 0.01)
        self._ros_node = ros_node

        # 제어 플래그
        self._stop_flag = False
        self._play_event = threading.Event()
        self._play_event.set()  # 기본적으로 재생 상태

        # 상태 추적
        self._status = 'stopped'   # 'playing' | 'paused' | 'stopped'
        self._elapsed_sec = 0.0
        self._total_sec = 0.0
        self._lock = threading.Lock()

        # 동적으로 생성된 ROS2 publisher 캐시 {topic_name: publisher}
        self._publishers = {}

    # ------------------------------------------------------------------
    # 제어 메서드
    # ------------------------------------------------------------------
    def pause(self):
        """재생 일시정지"""
        self._play_event.clear()
        with self._lock:
            self._status = 'paused'

    def resume(self):
        """재생 재개"""
        self._play_event.set()
        with self._lock:
            self._status = 'playing'

    def stop(self):
        """스레드 종료 요청"""
        self._stop_flag = True
        self._play_event.set()  # block 해제 후 종료
        with self._lock:
            self._status = 'stopped'

    def set_rate(self, new_rate: float):
        """재생 중 속도 배율 변경 (즉시 반영).

        Args:
            new_rate (float): 새 속도 배율 (예: 2.0 = 2배속). 0.01 미만은 0.01로 클램프.
        """
        with self._lock:
            self._playback_rate = max(new_rate, 0.01)

    def get_status(self):
        """현재 상태 딕셔너리 반환"""
        with self._lock:
            return {
                'status': self._status,
                'elapsed_sec': self._elapsed_sec,
                'total_sec': self._total_sec,
            }

    # ------------------------------------------------------------------
    # 내부 헬퍼 메서드
    # ------------------------------------------------------------------
    def _resolve_ros2_type(self, ros1_type_str):
        """ROS1 메시지 타입 문자열을 ROS2 Python 클래스로 동적 import.

        rosbags 라이브러리는 ROS1 bag에서도 ROS2 포맷으로 타입을 반환합니다.
        - ROS1 포맷: 'sensor_msgs/Image'       (parts 2개)
        - ROS2 포맷: 'sensor_msgs/msg/Image'   (parts 3개)
        두 포맷을 모두 처리합니다.

        Args:
            ros1_type_str (str): 예) 'sensor_msgs/msg/Image' 또는 'sensor_msgs/Image'

        Returns:
            type | None: 성공 시 메시지 클래스, 실패 시 None
        """
        import importlib
        try:
            parts = ros1_type_str.split('/')
            if len(parts) == 2:
                # ROS1 포맷: 'sensor_msgs/Image'
                pkg, msg_class = parts[0], parts[1]
            elif len(parts) == 3 and parts[1] == 'msg':
                # ROS2 포맷: 'sensor_msgs/msg/Image'
                pkg, msg_class = parts[0], parts[2]
            else:
                return None
            mod = importlib.import_module(f'{pkg}.msg')
            cls = getattr(mod, msg_class, None)
            return cls
        except Exception:
            return None

    def _get_or_create_publisher(self, topic_name, ros1_type_str, msg_cls):
        """토픽별 ROS2 publisher를 캐시해서 반환 (없으면 생성).

        Args:
            topic_name (str): publish할 토픽 이름
            ros1_type_str (str): ROS1 메시지 타입 문자열 (로그용)
            msg_cls (type): ROS2 메시지 클래스

        Returns:
            rclpy Publisher | None
        """
        if topic_name in self._publishers:
            return self._publishers[topic_name]

        try:
            pub = self._ros_node.create_publisher(msg_cls, topic_name, 10)
            self._publishers[topic_name] = pub
            self._ros_node.get_logger().info(
                f'[Ros1BagPlayer] Created publisher: {topic_name} ({ros1_type_str})'
            )
            return pub
        except Exception as e:
            self._ros_node.get_logger().error(
                f'[Ros1BagPlayer] Failed to create publisher for {topic_name}: {e}'
            )
            return None

    def _destroy_publishers(self):
        """생성한 모든 publisher 정리"""
        for topic_name, pub in self._publishers.items():
            try:
                self._ros_node.destroy_publisher(pub)
            except Exception:
                pass
        self._publishers.clear()

    # ------------------------------------------------------------------
    # 메인 실행 루프
    # ------------------------------------------------------------------
    def run(self):
        """rosbags Reader로 순차 읽기 → rclpy publisher로 publish.

        rosbags 최신 API:
          - get_typestore(Stores.ROS1_NOETIC) 로 typestore 생성 (reader.typestore 없음)
          - typestore.deserialize_ros1(rawdata, conn.msgtype) 로 역직렬화
          - 역직렬화 결과는 dataclass 기반 객체 (NamedTuple 아님)
        """
        try:
            from rosbags.rosbag1 import Reader
            from rosbags.typesys import get_typestore, Stores
        except ImportError as e:
            self._ros_node.get_logger().error(
                f'[Ros1BagPlayer] rosbags not available: {e}'
            )
            with self._lock:
                self._status = 'stopped'
            return

        with self._lock:
            self._status = 'playing'

        try:
            # typestore는 Reader 밖에서 한 번만 생성
            typestore = get_typestore(Stores.ROS1_NOETIC)

            with Reader(self._bag_path) as reader:
                # 전체 재생 시간 계산 (nanoseconds → seconds)
                total_ns = reader.end_time - reader.start_time
                with self._lock:
                    self._total_sec = total_ns / 1e9

                # 토픽별 메시지 타입 정보 수집
                # reader.topics: {topic_name: TopicInfo}
                topic_type_map = {}  # topic_name → ros1_type_str
                for topic_name, topic_info in reader.topics.items():
                    if self._topics is not None and topic_name not in self._topics:
                        continue
                    topic_type_map[topic_name] = topic_info.msgtype

                # publisher 사전 생성 (publishable한 토픽만)
                publishable_topics = set()
                for topic_name, ros1_type_str in topic_type_map.items():
                    msg_cls = self._resolve_ros2_type(ros1_type_str)
                    if msg_cls is not None:
                        pub = self._get_or_create_publisher(topic_name, ros1_type_str, msg_cls)
                        if pub is not None:
                            publishable_topics.add(topic_name)
                    else:
                        self._ros_node.get_logger().warn(
                            f'[Ros1BagPlayer] Skipping {topic_name} ({ros1_type_str}): '
                            'ROS2 type not found'
                        )

                if not publishable_topics:
                    self._ros_node.get_logger().warn(
                        '[Ros1BagPlayer] No publishable topics found. Stopping.'
                    )
                    with self._lock:
                        self._status = 'stopped'
                    return

                prev_ros_time = None   # 직전 메시지의 ROS timestamp (nanoseconds)
                start_ns = reader.start_time

                # 메시지 순차 순회
                for conn, timestamp, rawdata in reader.messages():
                    if self._stop_flag:
                        break

                    # 일시정지 대기 (blocking)
                    self._play_event.wait()
                    if self._stop_flag:
                        break

                    topic_name = conn.topic
                    ros1_type_str = conn.msgtype

                    # 선택되지 않은 토픽 스킵
                    if topic_name not in publishable_topics:
                        continue

                    # elapsed 업데이트
                    elapsed_ns = timestamp - start_ns
                    with self._lock:
                        self._elapsed_sec = elapsed_ns / 1e9

                    # 메시지 간 시간차 기반 sleep (속도 제어)
                    if prev_ros_time is not None:
                        dt_ns = timestamp - prev_ros_time
                        if dt_ns > 0:
                            sleep_sec = (dt_ns / 1e9) / self._playback_rate
                            # 최대 2초 sleep 제한 (긴 공백 방지)
                            time.sleep(min(sleep_sec, 2.0))
                    prev_ros_time = timestamp

                    # 역직렬화 + publish
                    try:
                        msg_cls = self._resolve_ros2_type(ros1_type_str)
                        if msg_cls is None:
                            continue

                        # rosbags 최신 API: typestore.deserialize_ros1()
                        ros1_msg = typestore.deserialize_ros1(rawdata, conn.msgtype)
                        # ROS2 메시지로 변환
                        ros2_msg = self._convert_ros1_to_ros2(ros1_msg, msg_cls)
                        if ros2_msg is None:
                            continue

                        pub = self._publishers.get(topic_name)
                        if pub is not None:
                            pub.publish(ros2_msg)

                    except Exception as e:
                        self._ros_node.get_logger().debug(
                            f'[Ros1BagPlayer] Publish error on {topic_name}: {e}'
                        )

        except Exception as e:
            self._ros_node.get_logger().error(
                f'[Ros1BagPlayer] Fatal error during playback: {e}'
            )
            import traceback
            traceback.print_exc()
        finally:
            self._destroy_publishers()
            with self._lock:
                self._status = 'stopped'
            self._ros_node.get_logger().info('[Ros1BagPlayer] Playback finished.')

    @staticmethod
    def _ros2_cls_from_rosbags_name(rosbags_type_name: str):
        """rosbags 타입명에서 ROS2 메시지 클래스를 임포트.

        예: 'sensor_msgs__msg__PointField' → sensor_msgs.msg.PointField
        ROS2에 없는 타입이면 None 반환.
        """
        import importlib
        parts = rosbags_type_name.split('__')
        # rosbags 이름 패턴: pkg__msg__ClassName  (3 parts)
        if len(parts) == 3 and parts[1] == 'msg':
            pkg, _, cls_name = parts
            try:
                mod = importlib.import_module(f'{pkg}.msg')
                return getattr(mod, cls_name, None)
            except Exception:
                pass
        return None

    def _convert_ros1_to_ros2(self, ros1_msg, ros2_cls):
        """rosbags 메시지 객체(dataclass 기반)를 ROS2 Python 메시지로 재귀 변환.

        rosbags 최신 버전에서 역직렬화 결과는 dataclass로,
        NamedTuple의 __struct_fields__ 가 없습니다.
        - 중첩 메시지: dataclasses.is_dataclass() 로 판별
        - 배열: numpy.ndarray → list() 변환
        - ROS1에만 있는 필드(예: header.seq)는 ROS2 메시지에 없으면 자동 스킵
        - 빈 배열([])의 중첩 메시지 배열: rosbags 타입명으로 ROS2 타입 추론
          (PointCloud2.fields 같은 기본 빈 배열 필드에서 SIGABRT 방지)

        Args:
            ros1_msg: rosbags 역직렬화된 메시지 객체 (dataclass)
            ros2_cls (type): 대상 ROS2 메시지 클래스

        Returns:
            ROS2 메시지 인스턴스 | None
        """
        import dataclasses
        import numpy as np

        try:
            ros2_msg = ros2_cls()

            if not dataclasses.is_dataclass(ros1_msg):
                return None

            for field in dataclasses.fields(ros1_msg):
                field_name = field.name
                # __msgtype__ 같은 rosbags 내부 메타 필드 스킵
                if field_name.startswith('__'):
                    continue
                # ROS2 메시지에 해당 필드가 없으면 스킵 (예: header.seq)
                if not hasattr(ros2_msg, field_name):
                    continue

                src_val = getattr(ros1_msg, field_name)
                dst_attr = getattr(ros2_msg, field_name)

                if dataclasses.is_dataclass(src_val):
                    # 단일 중첩 메시지 → 재귀 변환
                    nested_cls = type(dst_attr)
                    converted = self._convert_ros1_to_ros2(src_val, nested_cls)
                    if converted is not None:
                        setattr(ros2_msg, field_name, converted)

                elif isinstance(src_val, np.ndarray):
                    # ── numpy 배열 → ROS2 필드 고속 할당 ──
                    # uint8 배열(예: PointCloud2.data 7.6MB): bytes()가 tolist()보다 16배 빠름
                    #   tolist(): 37ms / bytes(): 2.3ms  (7.6MB 기준 실측)
                    # tolist()로 Python list를 생성하면 메시지당 37ms 추가 지연 →
                    #   10Hz LiDAR 실효 publish 주기가 137ms(7.3Hz)로 떨어지는 원인.
                    try:
                        if src_val.dtype == np.uint8:
                            # uint8[] (PointCloud2.data, Image.data 등) → bytes
                            setattr(ros2_msg, field_name, bytes(src_val))
                        else:
                            # float32[], float64[], int32[] 등 → list (호환성 유지)
                            setattr(ros2_msg, field_name, src_val.tolist())
                    except Exception:
                        try:
                            setattr(ros2_msg, field_name, src_val.tolist())
                        except Exception:
                            try:
                                setattr(ros2_msg, field_name, list(src_val))
                            except Exception:
                                pass

                elif isinstance(src_val, (list, tuple)) and len(src_val) > 0:
                    elem = src_val[0]
                    if dataclasses.is_dataclass(elem):
                        # 중첩 메시지 배열 (예: PointCloud2.fields → [PointField, ...])
                        dst_list = getattr(ros2_msg, field_name)
                        if dst_list:
                            # 배열에 기본 원소가 있으면 타입을 그대로 사용
                            nested_cls = type(dst_list[0])
                        else:
                            # 기본 빈 배열: rosbags 타입명에서 ROS2 타입 추론
                            # type(elem).__name__ 예: 'sensor_msgs__msg__PointField'
                            nested_cls = self._ros2_cls_from_rosbags_name(
                                type(elem).__name__
                            )
                        if nested_cls is not None:
                            converted_list = [
                                self._convert_ros1_to_ros2(m, nested_cls)
                                for m in src_val
                            ]
                            setattr(ros2_msg, field_name,
                                    [m for m in converted_list if m is not None])
                        # nested_cls 추론 실패 → 스킵 (SIGABRT 방지)
                    else:
                        try:
                            setattr(ros2_msg, field_name, type(dst_attr)(src_val))
                        except Exception:
                            try:
                                setattr(ros2_msg, field_name, list(src_val))
                            except Exception:
                                pass

                else:
                    try:
                        setattr(ros2_msg, field_name, src_val)
                    except Exception:
                        pass

            return ros2_msg

        except Exception:
            return None


class Ros1BagRecorderThread(threading.Thread):
    """rclpy subscriber로 ROS2 토픽을 구독하여 rosbags.rosbag1.Writer로
    ROS1 .bag 파일에 직접 기록하는 스레드.

    변환 흐름:
        rclpy subscriber → serialize_message() → CDR bytes
        → typestore.deserialize_cdr(bytes, msgtype)   # rosbags 객체
        → typestore.serialize_ros1(obj, msgtype)       # ROS1 raw bytes
        → rosbag1.Writer.write(connection, timestamp, raw)

    Attributes:
        output_path (str): 출력 ROS1 .bag 파일 경로
        topic_type_map (dict): {'/topic': 'sensor_msgs/msg/PointCloud2'} 형태의 맵
        ros_node (rclpy.node.Node): subscriber를 생성할 ROS2 노드 참조
    """

    def __init__(self, output_path, topic_type_map, ros_node):
        super().__init__(daemon=True)
        self._output_path = output_path
        self._topic_type_map = topic_type_map  # {'/topic': 'sensor_msgs/msg/PointCloud2'}
        self._ros_node = ros_node

        self._stop_flag = False
        self._start_time = None
        self._lock = threading.Lock()
        self._status = 'recording'

    # ------------------------------------------------------------------
    # 제어 메서드
    # ------------------------------------------------------------------
    def stop(self):
        """스레드 종료 요청"""
        self._stop_flag = True
        with self._lock:
            self._status = 'stopped'

    def get_status(self):
        """현재 상태 딕셔너리 반환"""
        with self._lock:
            elapsed = time.time() - self._start_time if self._start_time else 0.0
            return {
                'status': self._status,
                'elapsed_sec': elapsed,
            }

    # ------------------------------------------------------------------
    # 내부 헬퍼 메서드
    # ------------------------------------------------------------------
    @staticmethod
    def _import_ros2_msg_class(ros2_type):
        """ROS2 타입명으로 메시지 클래스를 동적 import.

        예: 'sensor_msgs/msg/PointCloud2' → sensor_msgs.msg.PointCloud2

        Returns:
            type | None: 성공 시 메시지 클래스, 실패 시 None
        """
        import importlib
        try:
            parts = ros2_type.split('/')
            if len(parts) == 3 and parts[1] == 'msg':
                pkg, _, cls_name = parts
                mod = importlib.import_module(f'{pkg}.msg')
                return getattr(mod, cls_name, None)
        except Exception:
            pass
        return None

    # ------------------------------------------------------------------
    # 메인 실행 루프
    # ------------------------------------------------------------------
    def run(self):
        """rclpy subscriber로 CDR bytes 수신 → rosbag1 Writer로 .bag 기록.

        rosbags API 주의사항:
          - writer.add_connection()의 msgtype은 ROS2 포맷('sensor_msgs/msg/PointCloud2')을
            그대로 사용해야 함. ROS1 포맷('sensor_msgs/PointCloud2')은 typestore에 없어 실패.
          - CDR → ROS1 변환은 typestore.cdr_to_ros1(cdr_bytes, typename) 을 사용.

        아키텍처:
          - 녹화 전용 임시 ROS2 노드(recorder_node)를 생성하고,
            SingleThreadedExecutor를 이 스레드 안에서 직접 spin하여 콜백을 처리한다.
          - 메인 스레드의 rclpy.spin()에 의존하지 않아 새 subscription이 누락되지 않는다.
        """
        try:
            from rosbags.rosbag1 import Writer
            from rosbags.typesys import get_typestore, Stores
            from rosbags.convert.converter import migrate_bytes
        except ImportError as e:
            self._ros_node.get_logger().error(f'[Ros1BagRecorder] rosbags not available: {e}')
            with self._lock:
                self._status = 'stopped'
            return

        import rclpy
        from rclpy.node import Node as RclpyNode
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.serialization import serialize_message

        self._start_time = time.time()
        msg_queue = []
        queue_lock = threading.Lock()

        # src: ROS2 typestore — CDR 역직렬화 (ROS2 Header 정의, seq 없음)
        # dst: ROS1 typestore — ROS1 직렬화 + add_connection msgdef 생성 (Header.seq 포함)
        # migrate_bytes()가 두 typestore 간 필드 차이(예: Header.seq)를 자동 처리함
        src_typestore = get_typestore(Stores.ROS2_JAZZY)
        dst_typestore = get_typestore(Stores.ROS1_NOETIC)
        migrate_cache: dict = {}

        # ── 녹화 전용 임시 노드 + 전용 Executor 생성 ──────────────────────────
        # 메인 노드(self._ros_node)의 SingleThreadedExecutor는 새 subscription을
        # 실시간으로 감지하지 못하는 경우가 있어, 독립 노드를 사용한다.
        recorder_node = None
        executor = None
        try:
            # 노드 이름 중복 방지: 짧은 타임스탬프로 유일성 확보
            _node_id = int(time.time() * 1000) % 100000
            recorder_node = RclpyNode(f'ros1_bag_recorder_{_node_id}')
            executor = SingleThreadedExecutor()
            executor.add_node(recorder_node)
        except Exception as e:
            self._ros_node.get_logger().error(
                f'[Ros1BagRecorder] Failed to create recorder node: {e}'
            )
            with self._lock:
                self._status = 'stopped'
            return

        def make_callback(topic_name):
            """토픽별 subscriber callback 생성 (closure로 topic_name 캡처)"""
            def callback(msg):
                if self._stop_flag:
                    return
                ts_ns = int(time.time() * 1e9)
                cdr_bytes = bytes(serialize_message(msg))
                with queue_lock:
                    msg_queue.append((topic_name, ts_ns, cdr_bytes))
            return callback

        written_count = 0
        try:
            with Writer(self._output_path) as writer:
                connections = {}

                for topic_name, ros2_type in self._topic_type_map.items():
                    msg_cls = self._import_ros2_msg_class(ros2_type)

                    if msg_cls is None:
                        self._ros_node.get_logger().warn(
                            f'[Ros1BagRecorder] Cannot import {ros2_type}, skipping {topic_name}'
                        )
                        continue

                    # rosbag1 Writer에 connection 등록
                    # dst_typestore(ROS1_NOETIC)로 msgdef 생성 → ROS1 bag에 올바른 메시지 정의 기록
                    # dst_typestore에 타입이 없는 경우 src_typestore에서 등록 시도
                    if ros2_type not in dst_typestore.fielddefs:
                        try:
                            from rosbags.typesys import get_types_from_msg
                            typs = get_types_from_msg(
                                src_typestore.generate_msgdef(ros2_type, ros_version=1)[0],
                                ros2_type,
                            )
                            typs.pop('std_msgs/msg/Header', None)  # Header는 ROS1 버전 유지
                            dst_typestore.register(typs)
                            self._ros_node.get_logger().info(
                                f'[Ros1BagRecorder] Registered custom type in dst_typestore: {ros2_type}'
                            )
                        except Exception as reg_e:
                            self._ros_node.get_logger().warn(
                                f'[Ros1BagRecorder] Cannot register type {ros2_type} in ROS1 typestore: {reg_e}'
                            )
                            continue
                    try:
                        conn = writer.add_connection(topic_name, ros2_type, typestore=dst_typestore)
                        connections[topic_name] = conn
                        self._ros_node.get_logger().info(
                            f'[Ros1BagRecorder] Connection registered: {topic_name} ({ros2_type})'
                        )
                    except Exception as e:
                        self._ros_node.get_logger().warn(
                            f'[Ros1BagRecorder] Failed to add connection for {topic_name}: {e}'
                        )
                        continue

                    # 녹화 전용 노드에 subscriber 생성 (메인 노드의 spin과 독립)
                    try:
                        recorder_node.create_subscription(
                            msg_cls, topic_name, make_callback(topic_name), 10
                        )
                        self._ros_node.get_logger().info(
                            f'[Ros1BagRecorder] Subscribed: {topic_name}'
                        )
                    except Exception as e:
                        self._ros_node.get_logger().warn(
                            f'[Ros1BagRecorder] Failed to subscribe to {topic_name}: {e}'
                        )

                self._ros_node.get_logger().info(
                    f'[Ros1BagRecorder] Recording started → {self._output_path} '
                    f'({len(connections)} topics)'
                )

                last_log_time = time.time()

                # 메인 루프 — 전용 executor를 여기서 직접 spin하여 콜백 처리 후 bag에 기록
                while not self._stop_flag:
                    # 이 스레드에서 recorder_node의 콜백을 직접 처리
                    executor.spin_once(timeout_sec=0.01)

                    with queue_lock:
                        pending = list(msg_queue)
                        msg_queue.clear()

                    for topic_name, ts_ns, cdr_bytes in pending:
                        conn = connections.get(topic_name)
                        if conn is None:
                            continue
                        try:
                            # ROS2 CDR bytes → ROS1 raw bytes 변환
                            #
                            # migrate_bytes()는 rosbags.convert 공식 변환 경로로:
                            # 1. src_typestore(ROS2_JAZZY)로 CDR 역직렬화 (Header에 seq 없음)
                            # 2. migrate_message()로 필드 매핑 (ROS1 Header의 seq=0 자동 추가 등)
                            # 3. dst_typestore(ROS1_NOETIC)로 ROS1 직렬화
                            raw = bytes(migrate_bytes(
                                src_typestore, dst_typestore,
                                conn.msgtype, conn.msgtype,
                                migrate_cache, cdr_bytes,
                                src_is2=True, dst_is2=False,
                            ))
                            writer.write(conn, ts_ns, raw)
                            written_count += 1
                        except Exception as e:
                            self._ros_node.get_logger().warn(
                                f'[Ros1BagRecorder] Write error on {topic_name} '
                                f'({conn.msgtype}): {e}'
                            )

                    # 5초마다 진행 상황 로그
                    now = time.time()
                    if now - last_log_time >= 5.0:
                        self._ros_node.get_logger().info(
                            f'[Ros1BagRecorder] Written {written_count} messages so far...'
                        )
                        last_log_time = now

        except Exception as e:
            self._ros_node.get_logger().error(f'[Ros1BagRecorder] Fatal error: {e}')
            import traceback
            traceback.print_exc()
        finally:
            # 전용 노드 정리
            if executor is not None and recorder_node is not None:
                try:
                    executor.remove_node(recorder_node)
                    recorder_node.destroy_node()
                except Exception:
                    pass
            with self._lock:
                self._status = 'stopped'
            self._ros_node.get_logger().info(
                f'[Ros1BagRecorder] Recording finished. Total messages written: {written_count}'
            )


class PC2WebSocketServer:
    """Python 백엔드 직접 PointCloud2 → Binary WebSocket 스트리밍 서버.

    rosbridge를 우회하여 PointCloud2를 Python에서 직접 구독한 뒤
    numpy로 XYZ + colorField(intensity/rgb)를 추출해 binary 패킷으로
    브라우저에 전달한다. JSON/base64 오버헤드가 없어 메시지 크기가
    ~10 MB → ~600 KB 수준으로 줄어든다.

    Binary 패킷 포맷 (little-endian):
      [3B]  magic = b'PC2'
      [1B]  version = 1
      [1B]  flags  (bit0=has_intensity, bit1=has_rgb)
      [4B]  uint32  topic_name 길이
      [4B]  uint32  frame_id 길이
      [4B]  uint32  point_count
      [N B] topic_name  (UTF-8)
      [M B] frame_id    (UTF-8)
      [count*12 B] XYZ float32 interleaved  (x0,y0,z0, x1,y1,z1, ...)
      [count*4  B] colorField float32        (intensity 또는 0.0)
      [count*4  B] rgb uint32               (has_rgb 일 때만)

    Ports:
      8081 — WebSocket (ws://host:8081)

    Client → Server 명령 (JSON 문자열):
      { "cmd": "subscribe",   "topic": "/ouster/points" }
      { "cmd": "unsubscribe", "topic": "/ouster/points" }
    """

    MAX_POINTS   = 50_000   # 다운샘플링 상한
    THROTTLE_SEC = 0.05     # 최대 20Hz (50 ms) — binary 전송 ~600KB이므로 충분

    # PointCloud2 field datatype → numpy dtype 매핑
    _DTYPE = {
        1: np.int8,   2: np.uint8,
        3: np.int16,  4: np.uint16,
        5: np.int32,  6: np.uint32,
        7: np.float32, 8: np.float64,
    } if NUMPY_AVAILABLE else {}

    def __init__(self, ros_node, port: int = 8081):
        self._node = ros_node
        self._port = port
        self._loop: asyncio.AbstractEventLoop | None = None
        self._lock = threading.Lock()
        # ── PointCloud2 전용 ───────────────────────────────────────────────────
        # topic_name → set[websocket]
        self._clients: dict = {}
        # topic_name → rclpy Subscription
        self._subs: dict = {}
        # topic_name → 마지막 전송 단조시각 (throttle)
        self._last_sent: dict = {}
        # ── 범용 Plot 토픽 (throttle 없이 원래 주기로 전송) ────────────────────
        # topic_name → { ws: set[field_path, ...] }
        self._plot_clients: dict = {}
        # topic_name → rclpy Subscription
        self._plot_subs: dict = {}
        # ── 전체 연결 클라이언트 (broadcast용) ─────────────────────────────────
        self._all_clients: set = set()

    # ── 공개 API ─────────────────────────────────────────────────────────────

    def start(self):
        """별도 daemon 스레드에서 asyncio WebSocket 서버를 시작한다."""
        if not WEBSOCKETS_AVAILABLE or not NUMPY_AVAILABLE:
            self._node.get_logger().warn(
                '[PC2WS] websockets 또는 numpy 미설치 — PC2 Binary WS 비활성화')
            return
        t = threading.Thread(
            target=self._run_loop, daemon=True, name='pc2-ws-server')
        t.start()

    # ── 내부: asyncio 루프 ────────────────────────────────────────────────────

    def _run_loop(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._serve())

    async def _serve(self):
        try:
            async with websockets.serve(
                    self._handler, '0.0.0.0', self._port,
                    max_size=None,
                    ping_interval=20,
                    ping_timeout=20):
                self._node.get_logger().info(
                    f'[PC2WS] Binary WebSocket server on ws://0.0.0.0:{self._port}')
                await asyncio.Future()   # 종료 없이 영원히 실행
        except Exception as e:
            self._node.get_logger().error(f'[PC2WS] server error: {e}')

    async def _handler(self, websocket):
        """WebSocket 연결 핸들러 — subscribe/unsubscribe/subscribe_plot 명령 수신."""
        my_pc2_topics: set  = set()   # PointCloud2 binary 구독
        my_plot_topics: set = set()   # 범용 plot JSON 구독
        # 전체 클라이언트 집합에 등록 (broadcast용)
        with self._lock:
            self._all_clients.add(websocket)
        try:
            async for raw in websocket:
                try:
                    msg = json.loads(raw)
                except Exception:
                    continue
                cmd   = msg.get('cmd', '')
                topic = msg.get('topic', '').strip()
                if not topic:
                    continue
                if cmd == 'subscribe':
                    self._add_client(topic, websocket)
                    my_pc2_topics.add(topic)
                elif cmd == 'unsubscribe':
                    self._remove_client(topic, websocket)
                    my_pc2_topics.discard(topic)
                elif cmd == 'subscribe_plot':
                    # 범용 토픽 plot 구독 (throttle 없이 원래 주기)
                    # msg_type: 클라이언트가 전달한 토픽 타입 (서버 조회 불필요)
                    fields   = msg.get('fields', [])
                    msg_type = msg.get('msg_type', '').strip()
                    if fields:
                        self._add_plot_client(topic, websocket, fields, msg_type)
                        my_plot_topics.add(topic)
                elif cmd == 'unsubscribe_plot':
                    fields = msg.get('fields', [])
                    self._remove_plot_client(topic, websocket, fields if fields else None)
                    with self._lock:
                        remaining = self._plot_clients.get(topic, {}).get(websocket)
                    if not remaining:
                        my_plot_topics.discard(topic)
        except Exception:
            pass
        finally:
            with self._lock:
                self._all_clients.discard(websocket)
            for t in list(my_pc2_topics):
                self._remove_client(t, websocket)
            for t in list(my_plot_topics):
                self._remove_plot_client(t, websocket, None)

    # ── 클라이언트 / 구독 관리 ────────────────────────────────────────────────

    def _add_client(self, topic: str, ws):
        with self._lock:
            if topic not in self._clients:
                self._clients[topic] = set()
            self._clients[topic].add(ws)
            if topic not in self._subs:
                sub = self._node.create_subscription(
                    PointCloud2, topic,
                    lambda m, t=topic: self._on_pc2(m, t),
                    10)
                self._subs[topic]      = sub
                self._last_sent[topic] = 0.0
                self._node.get_logger().info(f'[PC2WS] subscribed → {topic}')

    def _remove_client(self, topic: str, ws):
        with self._lock:
            s = self._clients.get(topic)
            if not s:
                return
            s.discard(ws)
            if not s:
                sub = self._subs.pop(topic, None)
                if sub:
                    self._node.destroy_subscription(sub)
                self._clients.pop(topic, None)
                self._last_sent.pop(topic, None)
                self._node.get_logger().info(f'[PC2WS] unsubscribed ← {topic}')

    # ── rclpy 콜백 ───────────────────────────────────────────────────────────

    def _on_pc2(self, msg: PointCloud2, topic_name: str):
        """PointCloud2 수신 → throttle → binary + JSON 메타데이터 → asyncio 브로드캐스트.

        전송 패킷 두 종류:
          1) binary bytes   : XYZ + color 데이터 (3D Viewer용)
          2) JSON string    : 헤더 스탬프·포인트 수 등 메타데이터 (Plot 탭용)
             {"type":"pc2meta","topic":"...","stamp_sec":N,"stamp_nanosec":N,
              "frame_id":"...","point_count":N}

        JavaScript 쪽에서 ws.binaryType='arraybuffer' 이므로
        ArrayBuffer → binary 핸들러, string → JSON 핸들러로 자동 분리된다.
        """
        now = time.monotonic()
        with self._lock:
            if now - self._last_sent.get(topic_name, 0.0) < self.THROTTLE_SEC:
                return
            clients = self._clients.get(topic_name, set()).copy()
        if not clients:
            return

        # ── 1) JSON 메타데이터 패킷 (헤더 스탬프 등) ────────────────────────
        stamp = msg.header.stamp
        meta_json = json.dumps({
            'type':          'pc2meta',
            'topic':         topic_name,
            'stamp_sec':     stamp.sec,
            'stamp_nanosec': stamp.nanosec,
            'frame_id':      msg.header.frame_id,
            'point_count':   msg.width * msg.height,
        }, separators=(',', ':'))

        # ── 2) binary 패킷 (XYZ + color) ────────────────────────────────────
        payload = self._build_payload(msg, topic_name)
        if payload is None:
            return

        with self._lock:
            self._last_sent[topic_name] = now

        loop = self._loop
        if loop and loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self._broadcast_both(clients, meta_json, payload), loop)

    async def _broadcast_both(self, clients, meta_json: str, binary_payload: bytes):
        """각 클라이언트에 JSON 메타데이터(text) + binary 데이터 순서로 전송."""
        for ws in list(clients):
            try:
                await ws.send(meta_json)       # text → JSON 파싱 경로
                await ws.send(binary_payload)  # binary → ArrayBuffer 경로
            except Exception:
                pass

    async def _broadcast(self, clients, payload: bytes):
        for ws in list(clients):
            try:
                await ws.send(payload)
            except Exception:
                pass

    async def _broadcast_text(self, clients, data: str):
        """text(JSON) 메시지를 여러 클라이언트에 전송."""
        for ws in list(clients):
            try:
                await ws.send(data)
            except Exception:
                pass

    async def _broadcast_json_all_async(self, data: dict):
        """연결된 모든 클라이언트에 JSON 메시지를 전송한다 (asyncio coroutine)."""
        with self._lock:
            clients = list(self._all_clients)
        payload = json.dumps(data)
        for ws in clients:
            try:
                await ws.send(payload)
            except Exception:
                pass

    def broadcast_json_all(self, data: dict):
        """연결된 모든 WebSocket 클라이언트에 JSON 메시지를 broadcast한다.

        스레드 안전: asyncio 이벤트 루프에 코루틴을 스케줄링하여 전송.
        """
        if self._loop is None or not self._all_clients:
            return
        asyncio.run_coroutine_threadsafe(
            self._broadcast_json_all_async(data), self._loop)

    # ── 범용 토픽 Plot 구독 (throttle 없이 원래 주기) ─────────────────────────

    def _add_plot_client(self, topic: str, ws, fields: list, msg_type: str = ''):
        """일반 토픽의 특정 필드를 실시간 plot하기 위한 클라이언트 등록.

        msg_type: 클라이언트(browser)가 이미 알고 있는 토픽 타입 문자열.
          전달하면 get_topic_names_and_types() 조회 없이 즉시 subscription 생성.
          타이밍 문제(bag 재생 직후 조회 실패)를 방지한다.
        """
        need_sub = False
        with self._lock:
            if topic not in self._plot_clients:
                self._plot_clients[topic] = {}
            if ws not in self._plot_clients[topic]:
                self._plot_clients[topic][ws] = set()
            self._plot_clients[topic][ws].update(fields)
            if topic not in self._plot_subs:
                need_sub = True

        if need_sub:
            self._create_plot_subscription(topic, msg_type)

    def _remove_plot_client(self, topic: str, ws, fields=None):
        """plot 클라이언트 제거. fields=None 이면 해당 ws의 모든 필드 제거."""
        with self._lock:
            client_map = self._plot_clients.get(topic, {})
            if ws not in client_map:
                return
            if fields is None:
                del client_map[ws]
            else:
                client_map[ws].difference_update(fields)
                if not client_map[ws]:
                    del client_map[ws]
            # 해당 topic 구독자가 0이면 subscription 삭제
            if not client_map:
                self._plot_clients.pop(topic, None)
                sub = self._plot_subs.pop(topic, None)
                if sub:
                    try:
                        self._node.destroy_subscription(sub)
                    except Exception:
                        pass
                self._node.get_logger().info(f'[PC2WS/plot] unsubscribed ← {topic}')

    def _create_plot_subscription(self, topic: str, msg_type: str = ''):
        """토픽 타입을 자동 감지하여 rclpy subscription 동적 생성.

        msg_type이 주어지면 ROS2 DDS 조회(get_topic_names_and_types) 없이
        즉시 subscription을 생성한다. bag 재생 직후 등 타이밍 문제를 방지.
        msg_type이 없으면 DDS에서 조회한다 (fallback).
        """
        # ── 1) 클라이언트가 전달한 타입 우선 사용 ─────────────────────────────
        type_str = msg_type.strip() if msg_type else ''

        # ── 2) fallback: DDS 조회 ─────────────────────────────────────────────
        if not type_str:
            try:
                for name, types in self._node.get_topic_names_and_types():
                    if name == topic and types:
                        type_str = types[0]
                        break
            except Exception as e:
                self._node.get_logger().error(f'[PC2WS/plot] 토픽 타입 조회 오류: {e}')

        if not type_str:
            self._node.get_logger().warn(
                f'[PC2WS/plot] 토픽 타입 못 찾음 (msg_type 미제공, DDS 조회 실패): {topic}')
            return

        MsgClass = self._get_msg_class(type_str)
        if MsgClass is None:
            self._node.get_logger().warn(
                f'[PC2WS/plot] 메시지 타입 로드 실패: {type_str}')
            return

        sub = self._node.create_subscription(
            MsgClass,
            topic,
            lambda msg, t=topic: self._on_plot_msg(msg, t),
            10
        )
        with self._lock:
            self._plot_subs[topic] = sub
        self._node.get_logger().info(
            f'[PC2WS/plot] subscribed → {topic} ({type_str})')

    def _on_plot_msg(self, msg, topic_name: str):
        """범용 토픽 메시지 수신 → 요청된 필드 추출 → JSON broadcast.

        throttle 없이 원래 주기 그대로 전송한다.
        헤더가 있으면 header.stamp를 timestamp로 사용하고,
        없으면 현재 단조 시간을 사용한다.
        """
        with self._lock:
            client_map = self._plot_clients.get(topic_name, {})
            if not client_map:
                return
            # 모든 클라이언트의 필드 합집합
            all_fields: set = set()
            for fields in client_map.values():
                all_fields.update(fields)
            clients = set(client_map.keys())

        # 타임스탬프 추출
        stamp_sec, stamp_nanosec = 0, 0
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            stamp_sec     = msg.header.stamp.sec
            stamp_nanosec = msg.header.stamp.nanosec
        else:
            t = time.time()
            stamp_sec     = int(t)
            stamp_nanosec = int((t - stamp_sec) * 1e9)

        # 요청된 필드 값 추출
        values = {}
        for field in all_fields:
            # 특수 계산 필드 처리
            if field == 'point_count':
                # PointCloud2: point_count = width * height
                if hasattr(msg, 'width') and hasattr(msg, 'height'):
                    values[field] = float(msg.width * msg.height)
                continue
            val = self._extract_nested(msg, field)
            if val is not None:
                values[field] = val

        if not values:
            return

        data = json.dumps({
            'type':          'plot_data',
            'topic':         topic_name,
            'stamp_sec':     stamp_sec,
            'stamp_nanosec': stamp_nanosec,
            'values':        values,
        }, separators=(',', ':'))

        loop = self._loop
        if loop and loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self._broadcast_text(clients, data), loop)

    @staticmethod
    def _extract_nested(obj, field_path: str):
        """슬래시 또는 점 표기법으로 중첩 필드 값 추출.

        예) 'linear_acceleration/x'  →  obj.linear_acceleration.x
            'header/stamp/sec'       →  obj.header.stamp.sec
        """
        for part in field_path.replace('.', '/').split('/'):
            if hasattr(obj, part):
                obj = getattr(obj, part)
            else:
                return None
        if isinstance(obj, (int, float, bool)):
            return float(obj)
        if isinstance(obj, str):
            return obj
        return None

    @staticmethod
    def _get_msg_class(type_str: str):
        """'sensor_msgs/msg/Imu'  →  sensor_msgs.msg.Imu 클래스 반환.
           'sensor_msgs/Imu'      →  sensor_msgs.msg.Imu (deprecated 형식 대응)
        """
        import importlib
        parts = type_str.split('/')
        try:
            if len(parts) == 3:                   # package/msg/Class
                module = importlib.import_module(f'{parts[0]}.{parts[1]}')
                return getattr(module, parts[2])
            elif len(parts) == 2:                 # package/Class (구형)
                module = importlib.import_module(f'{parts[0]}.msg')
                return getattr(module, parts[1])
        except Exception:
            return None
        return None

    # ── PointCloud2 → binary 패킷 변환 ───────────────────────────────────────

    def _build_payload(self, msg: PointCloud2, topic_name: str):
        """PointCloud2 메시지를 binary 패킷으로 변환. 실패 시 None 반환."""
        try:
            frame_id  = msg.header.frame_id if msg.header else ''
            field_map = {f.name: f for f in msg.fields}

            if not ('x' in field_map and 'y' in field_map and 'z' in field_map):
                return None

            n_total    = msg.width * msg.height
            point_step = msg.point_step
            if n_total == 0 or point_step == 0:
                return None

            # raw bytes → uint8 numpy array → (N, point_step) 형태
            raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            if raw.size < n_total * point_step:
                n_total = raw.size // point_step
            arr = raw[:n_total * point_step].reshape(n_total, point_step)

            def _extract_f32(field_name):
                f   = field_map[field_name]
                off = f.offset
                dt  = self._DTYPE.get(f.datatype, np.float32)
                bw  = dt().itemsize
                return np.frombuffer(
                    arr[:, off:off + bw].tobytes(), dtype=dt
                ).astype(np.float32)

            x = _extract_f32('x')
            y = _extract_f32('y')
            z = _extract_f32('z')

            # NaN/Inf 필터링
            valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
            x, y, z = x[valid], y[valid], z[valid]

            n = len(x)
            if n == 0:
                return None

            # 다운샘플링 (voxel-free: 균등 step)
            step = max(1, n // self.MAX_POINTS)
            x, y, z = x[::step], y[::step], z[::step]
            n_out = len(x)

            xyz = np.column_stack([x, y, z]).astype(np.float32)

            # intensity 추출
            has_intensity = 'intensity' in field_map
            color_f32 = np.zeros(n_out, dtype=np.float32)
            if has_intensity:
                ci = _extract_f32('intensity')
                color_f32 = ci[valid][::step][:n_out]

            # RGB 추출
            has_rgb = 'rgb' in field_map or 'rgba' in field_map
            rgb_u32 = np.zeros(n_out, dtype=np.uint32)
            if has_rgb:
                rkey = 'rgb' if 'rgb' in field_map else 'rgba'
                f    = field_map[rkey]
                ri   = np.frombuffer(
                    arr[:, f.offset:f.offset + 4].tobytes(), dtype=np.uint32)
                rgb_u32 = ri[valid][::step][:n_out]

            flags  = (0x01 if has_intensity else 0) | (0x02 if has_rgb else 0)
            topic_b = topic_name.encode('utf-8')
            frame_b = frame_id.encode('utf-8')

            header = struct.pack(
                '<3sBBIII',
                b'PC2', 1, flags,
                len(topic_b), len(frame_b), n_out)

            parts = [header, topic_b, frame_b, xyz.tobytes(), color_f32.tobytes()]
            if has_rgb:
                parts.append(rgb_u32.tobytes())
            return b''.join(parts)

        except Exception as e:
            self._node.get_logger().error(f'[PC2WS] _build_payload error: {e}')
            return None


class WebGUINode(Node):
    def __init__(self):
        super().__init__('web_gui_node')

        # SLAM GUI state
        self.slam_map1 = ""
        self.slam_map2 = ""
        self.slam_output = ""
        self.slam_status = "Ready"
        self.slam_process = None

        # Localization state
        self.localization_process = None

        # Bag Player state
        self.bag_path = ""
        self.bag_playing = False
        self.bag_paused = False
        self.bag_process = None
        self.bag_playback_rate = 1.0  # 현재 설정된 재생 속도 배율

        # Bag Recorder state
        self.recorder_bag_name = ""
        self.recorder_recording = False
        self.recorder_process = None
        self.recorder_ros1_thread = None   # Ros1BagRecorderThread 인스턴스
        self.recorder_mode = 'ros2'        # 'ros2' | 'ros1'
        self.bag_topics = []
        self.bag_selected_topics = []
        self.bag_duration = 0.0  # Duration in seconds
        self.bag_current_time = 0.0  # Current playback time in seconds
        self.bag_start_offset = 0.0  # Start offset for playback
        self.bag_start_real_time = 0.0  # Real time when playback started
        self.bag_pause_time = 0.0  # Time when paused

        # File Player state
        self.player_path = ""
        self.player_playing = False
        self.player_paused = False
        self.player_loop = False
        self.player_skip_stop = True
        self.player_auto_start = False
        self.player_speed = 1.0
        self.player_timestamp = 0
        self.player_slider_pos = 0
        self.player_initial_stamp = 0
        self.player_last_stamp = 0
        self.player_data_loaded = False
        self.player_processed_stamp = 0
        self.player_prev_time = 0
        self.save_bag_progress = None   # None: idle, "0%"~"100%": saving in progress
        self.save_bag_saving = False    # True while background save thread is running
        self.save_bag_success = False   # Result of last save operation

        # ROS1 Bag Player state
        self.ros1_player_thread = None
        self.ros1_player_rate = 1.0

        # File Player ROS2 Publishers/Subscribers — lazy initialized on load_player_data()
        # (not created at startup to avoid polluting the topic list before file player is used)
        # ConPR 전용 publishers
        self.pose_pub = None
        self.imu_pub = None
        self.cam_pub = None
        self.cam_info_pub = None
        self.livox_pub = None
        # 공통 (ConPR + KITTI 모두 사용)
        self.clock_pub = None
        self.start_sub = None
        self.stop_sub = None
        # KITTI 전용 publishers
        self.kitti_velo_pub = None
        self.kitti_cam_pub = None
        # 초기화 플래그 (각 데이터셋 전용 publishers 이중 생성 방지)
        self._conpr_pubs_initialized = False
        self._kitti_pubs_initialized = False

        # CV Bridge for image conversion
        self.cv_bridge = CvBridge()

        # SLAM Subscribers — lazy initialized on start_slam_mapping()
        self.slam_complete_sub = None

        # File Player data structures
        self.data_stamp = {}
        self.pose_data = {}
        self.imu_data = {}
        self.livox_file_list = []  # List of LiDAR .bin files
        self.cam_file_list = []    # List of camera image files
        self.livox_cache = {}      # Cache for loaded LiDAR data
        self.cam_cache = {}        # Cache for loaded camera images

        # Playback thread
        self.playback_thread = None
        self.playback_active = False
        self.player_seek_requested = False  # seek 후 worker 인덱스 재설정 신호
        self.player_seek_to_stamp  = 0      # seek 목표 타임스탬프 (HTTP 스레드→worker 전달용)

        # Timer for playback (matching C++ implementation)
        self.create_timer(0.0001, self.timer_callback)  # 100us = 0.0001s

        # Timer for bag playback time tracking
        self.create_timer(0.1, self.bag_timer_callback)  # 100ms = 0.1s

        # Setup reusable environment for subprocess calls
        self._setup_ros_environment()

        # ── PC2 Binary WebSocket 서버 (포트 8081) ─────────────────────────────
        # rosbridge를 우회해 PointCloud2를 Python에서 직접 처리 후 binary 전송
        self.pc2_ws_server = PC2WebSocketServer(self, port=8081)
        self.pc2_ws_server.start()

        # ── KITTI 변환기 상태 ──────────────────────────────────────────────────
        self.kitti_converter_running = False   # 변환 진행 중 여부
        self.kitti_convert_thread = None       # 변환 백그라운드 스레드

        # ── ROS2 bag 모드 플래그 (player_play_toggle → bag_play_toggle 위임) ─
        self.player_is_ros2_bag = False        # True 이면 File Player가 bag 모드

        # ── KITTI direct play 모드 ──────────────────────────────────────────
        self.player_is_kitti = False           # True 이면 KITTI 파일 직접 재생
        self.kitti_drive_path = None           # 로드된 KITTI drive 디렉토리 경로

        self.get_logger().info('Web GUI Node initialized with full ROS2 integration')

    def _setup_ros_environment(self):
        """
        Setup reusable ROS2 environment for subprocess calls.
        This avoids re-sourcing setup.bash on every subprocess call.
        """
        # Get sourced environment once and cache it
        bash_cmd = (
            'source /opt/ros/jazzy/setup.bash && '
            'source /home/kkw/localization_ws/install/setup.bash && '
            'env'
        )
        try:
            result = subprocess.run(
                ['bash', '-c', bash_cmd],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                # Parse environment variables
                self._ros_env = os.environ.copy()
                for line in result.stdout.split('\n'):
                    if '=' in line:
                        key, _, value = line.partition('=')
                        self._ros_env[key] = value
                self.get_logger().info('ROS2 environment cached successfully')
            else:
                # Fallback to current environment
                self._ros_env = os.environ.copy()
                self.get_logger().warn('Failed to source ROS2 environment, using current environment')
        except Exception as e:
            self._ros_env = os.environ.copy()
            self.get_logger().error(f'Error setting up ROS2 environment: {str(e)}')

        # Add DISPLAY and XAUTHORITY for GUI applications (rviz2)
        # Try to get DISPLAY from environment or default to :0
        if 'DISPLAY' in os.environ:
            self._ros_env['DISPLAY'] = os.environ['DISPLAY']
        else:
            # Default to :0 if not set (common for local X server)
            self._ros_env['DISPLAY'] = ':0'
            self.get_logger().info('DISPLAY not set, defaulting to :0')
        
        # Try to get XAUTHORITY from environment or try common locations
        if 'XAUTHORITY' in os.environ:
            self._ros_env['XAUTHORITY'] = os.environ['XAUTHORITY']
            self.get_logger().info(f'Using XAUTHORITY from environment: {os.environ["XAUTHORITY"]}')
        else:
            # Try common XAUTHORITY locations (including Wayland)
            import glob
            xauth_paths = [
                os.path.expanduser('~/.Xauthority'),
                '/run/user/{}/gdm/Xauthority'.format(os.getuid()),
                '/run/user/{}/.mutter-Xwaylandauth.*'.format(os.getuid()),  # Wayland
                '/var/run/gdm/auth-for-{}-*/database'.format(os.getenv('USER', 'root'))
            ]
            xauth_found = False
            for xauth_pattern in xauth_paths:
                # Handle glob patterns
                if '*' in xauth_pattern:
                    matches = glob.glob(xauth_pattern)
                    if matches:
                        xauth_path = matches[0]  # Use first match
                        if os.path.exists(xauth_path):
                            self._ros_env['XAUTHORITY'] = xauth_path
                            self.get_logger().info(f'Found XAUTHORITY at: {xauth_path}')
                            xauth_found = True
                            break
                else:
                    if os.path.exists(xauth_pattern):
                        self._ros_env['XAUTHORITY'] = xauth_pattern
                        self.get_logger().info(f'Found XAUTHORITY at: {xauth_pattern}')
                        xauth_found = True
                        break
            
            if not xauth_found:
                # Try to find any XAUTHORITY file in /run/user/
                user_run_dir = f'/run/user/{os.getuid()}'
                if os.path.exists(user_run_dir):
                    wayland_auth_files = glob.glob(f'{user_run_dir}/.mutter-Xwaylandauth.*')
                    if wayland_auth_files:
                        self._ros_env['XAUTHORITY'] = wayland_auth_files[0]
                        self.get_logger().info(f'Found Wayland XAUTHORITY at: {wayland_auth_files[0]}')
                    else:
                        # Fallback to user's home directory (even if it doesn't exist)
                        self._ros_env['XAUTHORITY'] = os.path.expanduser('~/.Xauthority')
                        self.get_logger().warn('XAUTHORITY not found, using ~/.Xauthority (may not exist)')
                else:
                    self._ros_env['XAUTHORITY'] = os.path.expanduser('~/.Xauthority')
                    self.get_logger().warn('XAUTHORITY not found, using ~/.Xauthority (may not exist)')

    def _init_common_ros_interfaces(self):
        """공통 인터페이스 초기화: /clock publisher + file_player 구독.
        ConPR/KITTI 어느 쪽이든 처음 로드 시 한 번만 호출.
        """
        if self.clock_pub is None:
            self.clock_pub = self.create_publisher(Clock, '/clock', 1)
        if self.start_sub is None:
            self.start_sub = self.create_subscription(
                Bool, '/file_player_start', self.file_player_start_callback, 1)
        if self.stop_sub is None:
            self.stop_sub = self.create_subscription(
                Bool, '/file_player_stop', self.file_player_stop_callback, 1)

    def _init_file_player_ros_interfaces(self):
        """ConPR 전용 publisher 초기화 (lazy).

        ConPR 데이터를 처음 로드할 때만 호출.
        KITTI 데이터를 로드해도 ConPR 토픽은 생성되지 않는다.
        """
        self._init_common_ros_interfaces()
        if self._conpr_pubs_initialized:
            return

        self.pose_pub     = self.create_publisher(PointStamped, '/pose/position', 1000)
        self.imu_pub      = self.create_publisher(Imu, '/imu', 1000)
        self.cam_pub      = self.create_publisher(Image, '/camera/color/image', 1000)
        self.cam_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 1000)

        if LIVOX_AVAILABLE:
            self.livox_pub = self.create_publisher(CustomMsg, '/livox/lidar', 1000)

        self._conpr_pubs_initialized = True
        self.get_logger().info('ConPR File Player publishers initialized')

    def _init_kitti_ros_interfaces(self):
        """KITTI 전용 publisher 초기화 (lazy).

        KITTI 데이터를 처음 로드할 때만 호출.
        ConPR 토픽(/pose, /imu 등)은 생성하지 않는다.
        """
        self._init_common_ros_interfaces()
        if self._kitti_pubs_initialized:
            return

        self.kitti_velo_pub = self.create_publisher(
            PointCloud2, '/kitti/velo/pointcloud', 1000)
        self.kitti_cam_pub  = self.create_publisher(
            Image, '/kitti/camera_color_left/image_raw', 1000)

        self._kitti_pubs_initialized = True
        self.get_logger().info('KITTI File Player publishers initialized')

    def _init_slam_subscriber(self):
        """Lazy initialization of SLAM-related subscribers.

        Called once when SLAM mapping is first started via start_slam_mapping().
        This prevents /lt_mapping_complete from appearing in the topic list at startup.
        """
        if self.slam_complete_sub is not None:
            return  # Already initialized

        self.slam_complete_sub = self.create_subscription(
            Bool, '/lt_mapping_complete', self.slam_complete_callback, 10)
        self.get_logger().info('SLAM subscriber (/lt_mapping_complete) initialized')

    def _read_process_output(self, process, output_lock, output_attr_name, max_lines=10):
        """
        Thread function to read process output and store in terminal output buffer.

        Args:
            process: The subprocess.Popen object to read from
            output_lock: Threading lock for output synchronization
            output_attr_name: Name of the attribute to store output (e.g., 'slam_terminal_output')
            max_lines: Maximum number of lines to keep in buffer (default: 10)
        """
        try:
            for line in iter(process.stdout.readline, ''):
                if line:
                    with output_lock:
                        current_output = getattr(self, output_attr_name)
                        current_output += line
                        # Keep only last max_lines lines
                        lines = current_output.split('\n')
                        if len(lines) > max_lines:
                            # Keep last max_lines lines (including any incomplete line at the end)
                            current_output = '\n'.join(lines[-max_lines:])
                        setattr(self, output_attr_name, current_output)
        except Exception as e:
            self.get_logger().error(f'Error reading process output: {str(e)}')

    def _stop_process(self, process, process_name, output_lock=None, output_attr_name=None):
        """
        Stop a running process gracefully using SIGINT, SIGTERM, and SIGKILL as needed.

        Args:
            process: The subprocess.Popen object to stop
            process_name: Name of the process for logging
            output_lock: Optional threading lock for output synchronization
            output_attr_name: Optional name of output attribute to append termination message

        Returns:
            bool: True if process was stopped, False if no process was running
        """
        try:
            if process and process.poll() is None:
                self.get_logger().info(f'Stopping {process_name} process (PID: {process.pid})...')

                # Get process group ID
                try:
                    pgid = os.getpgid(process.pid)
                    self.get_logger().info(f'Process group ID: {pgid}')

                    # Send SIGINT (Ctrl+C) to the entire process group
                    os.killpg(pgid, signal.SIGINT)
                    self.get_logger().info('Sent SIGINT to process group')

                    # Wait for process to terminate
                    # Increase timeout for GUI applications like rviz2
                    try:
                        process.wait(timeout=8)  # Increased from 5 to 8 seconds
                        self.get_logger().info(f'{process_name} process terminated gracefully')
                    except subprocess.TimeoutExpired:
                        self.get_logger().warn('Process did not terminate with SIGINT, sending SIGTERM')
                        os.killpg(pgid, signal.SIGTERM)
                        try:
                            process.wait(timeout=8)  # Increased from 5 to 8 seconds
                            self.get_logger().info(f'{process_name} process terminated with SIGTERM')
                        except subprocess.TimeoutExpired:
                            self.get_logger().warn('Process did not terminate with SIGTERM, sending SIGKILL')
                            os.killpg(pgid, signal.SIGKILL)
                            process.wait(timeout=3)  # Increased from 2 to 3 seconds
                            self.get_logger().info(f'{process_name} process killed with SIGKILL')

                except ProcessLookupError:
                    self.get_logger().warn('Process already terminated')
                except Exception as e:
                    self.get_logger().error(f'Error during process termination: {str(e)}')
                    # Fallback: try to terminate the process directly
                    process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()

                # Add termination message to terminal output if requested
                if output_lock and output_attr_name:
                    with output_lock:
                        current_output = getattr(self, output_attr_name)
                        current_output += f'\n[{process_name} process stopped by user]\n'
                        setattr(self, output_attr_name, current_output)

                return True
            else:
                self.get_logger().warn(f'No {process_name} process is running')
                return False
        except Exception as e:
            self.get_logger().error(f'Failed to stop {process_name} process: {str(e)}')
            import traceback
            traceback.print_exc()
            return False

    def _kill_processes_by_pattern(self, patterns):
        """
        Kill processes matching the given patterns.

        Args:
            patterns: List of pattern strings to search for in process command lines
        """
        try:
            # Get all processes
            result = subprocess.run(['ps', 'aux'], capture_output=True, text=True)
            lines = result.stdout.split('\n')

            for line in lines:
                # Check if line matches any pattern
                for pattern in patterns:
                    if pattern in line:
                        parts = line.split()
                        if len(parts) > 1:
                            pid = int(parts[1])
                            self.get_logger().info(f'Killing process matching "{pattern}": PID {pid}')
                            try:
                                os.kill(pid, signal.SIGTERM)
                            except ProcessLookupError:
                                pass
                        break  # Move to next line after finding a match

            time.sleep(0.5)
        except Exception as e:
            self.get_logger().error(f'Error killing processes by pattern: {str(e)}')

    # SLAM Functions
    def set_slam_map1(self, path):
        self.slam_map1 = path
        self.slam_status = f"Map 1 loaded - {path}"
        self.get_logger().info(f'Map 1 set to: {path}')

    def set_slam_map2(self, path):
        self.slam_map2 = path
        self.slam_status = f"Map 2 loaded - {path}"
        self.get_logger().info(f'Map 2 set to: {path}')

    def set_slam_output(self, directory_name):
        """Set output directory name (not full path, just directory name)"""
        # Extract just the directory name if a full path is provided
        if '/' in directory_name:
            directory_name = os.path.basename(directory_name.rstrip('/'))

        self.slam_output = directory_name
        self.slam_status = f"Output directory set to - {directory_name}"
        self.get_logger().info(f'Output directory name set to: {directory_name}')

    def start_slam_mapping(self):
        """Start FAST_LIO mapping"""
        self.get_logger().info('=== Starting FAST_LIO SLAM Mapping ===')

        # Ensure SLAM subscriber is ready before launching the process
        self._init_slam_subscriber()

        # Kill any existing SLAM processes first
        self.kill_slam_processes()
        time.sleep(0.5)

        # Launch mapping without capturing terminal output
        try:
            # Create command with environment setup
            bash_cmd = (
                'source /opt/ros/jazzy/setup.bash && '
                'source /home/kkw/localization_ws/install/setup.bash && '
                'ros2 launch fast_lio mapping.launch.py'
            )

            # Run command
            cmd = ['bash', '-c', bash_cmd]

            self.get_logger().info('Starting FAST_LIO mapping (no terminal capture)')

            # Launch process - capture stderr to check for rviz2 errors
            # Log DISPLAY and XAUTHORITY for debugging
            self.get_logger().info(f'DISPLAY: {self._ros_env.get("DISPLAY", "NOT SET")}')
            self.get_logger().info(f'XAUTHORITY: {self._ros_env.get("XAUTHORITY", "NOT SET")}')
            
            # Capture stderr to log rviz2 errors
            stderr_file = open('/tmp/web_gui_slam_stderr.log', 'w')
            self.slam_process = subprocess.Popen(
                cmd,
                env=self._ros_env,
                stdout=subprocess.DEVNULL,
                stderr=stderr_file,
                text=True,
                start_new_session=True
            )

            self.get_logger().info('FAST_LIO mapping started with PID: {}'.format(self.slam_process.pid))
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to start FAST_LIO mapping: {str(e)}')
            import traceback
            traceback.print_exc()
            return False

    def stop_slam_mapping(self):
        """Stop SLAM mapping process (like Ctrl+C)"""
        result = self._stop_process(
            self.slam_process,
            'SLAM'
        )
        if result:
            self.slam_process = None
        return result

    def save_slam_map(self, directory):
        """Save SLAM map to specified directory"""
        try:
            if not SAVEMAP_AVAILABLE:
                self.get_logger().error('SaveMap service not available')
                return False, 'SaveMap service not available'

            self.get_logger().info(f'Requesting to save SLAM map to directory: {directory}')

            # Create service client
            client = self.create_client(SaveMap, 'save_trajectory')

            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('save_trajectory service not available')
                return False, 'save_trajectory service not available. Is pose_graph_optimization node running?'

            # Create request
            request = SaveMap.Request()
            request.directory_name = directory

            # Call service
            future = client.call_async(request)

            # Wait for response (with timeout)
            timeout = 30.0  # 30 seconds
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > timeout:
                    self.get_logger().error('Service call timed out')
                    return False, 'Service call timed out after 30 seconds'
                rclpy.spin_once(self, timeout_sec=0.1)

            # Get response
            response = future.result()

            if response.success:
                self.get_logger().info(f'Map saved successfully: {response.message}')
                return True, response.message
            else:
                self.get_logger().error(f'Map save failed: {response.message}')
                return False, response.message

        except Exception as e:
            self.get_logger().error(f'Failed to save map: {str(e)}')
            import traceback
            traceback.print_exc()
            return False, str(e)

    def start_localization_mapping(self):
        """Start Localization mapping process"""
        if self.localization_process and self.localization_process.poll() is None:
            self.get_logger().warn('Localization mapping is already running')
            return True

        # Kill any existing Localization processes first
        self.kill_localization_processes()
        time.sleep(0.5)

        # Launch localization without capturing terminal output
        try:
            # Create command with environment setup
            bash_cmd = (
                'source /opt/ros/jazzy/setup.bash && '
                'source /home/kkw/localization_ws/install/setup.bash && '
                'ros2 launch fast_lio localization.launch.py'
            )

            # Run command
            cmd = ['bash', '-c', bash_cmd]

            self.get_logger().info('Starting FAST_LIO localization (no terminal capture)')

            # Launch process - capture stderr to check for rviz2 errors
            # Log DISPLAY and XAUTHORITY for debugging
            self.get_logger().info(f'DISPLAY: {self._ros_env.get("DISPLAY", "NOT SET")}')
            self.get_logger().info(f'XAUTHORITY: {self._ros_env.get("XAUTHORITY", "NOT SET")}')
            
            # Capture stderr to log rviz2 errors
            stderr_file = open('/tmp/web_gui_localization_stderr.log', 'w')
            self.localization_process = subprocess.Popen(
                cmd,
                env=self._ros_env,
                stdout=subprocess.DEVNULL,
                stderr=stderr_file,
                text=True,
                start_new_session=True
            )

            self.get_logger().info('FAST_LIO localization started with PID: {}'.format(self.localization_process.pid))
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to start FAST_LIO localization: {str(e)}')
            import traceback
            traceback.print_exc()
            return False

    def stop_localization_mapping(self):
        """Stop Localization mapping process (like Ctrl+C)"""
        result = self._stop_process(
            self.localization_process,
            'Localization'
        )
        if result:
            self.localization_process = None
        return result

    def kill_localization_processes(self):
        """Kill any running Localization processes"""
        self._kill_processes_by_pattern(['localization.launch.py'])

    def run_slam_optimization(self):
        if not self.slam_map1 or not self.slam_map2:
            self.slam_status = "Error: Please load both Map 1 and Map 2"
            return False

        if not self.slam_output:
            self.slam_status = "Error: Please set output directory"
            return False

        self.slam_status = "Running Multi-Session Optimization..."
        self.get_logger().info('=== Starting Multi-Session SLAM Optimization ===')
        self.get_logger().info(f'Map 1: {self.slam_map1}')
        self.get_logger().info(f'Map 2: {self.slam_map2}')
        self.get_logger().info(f'Output: {self.slam_output}')

        # Kill any existing processes first
        self.kill_slam_processes()
        time.sleep(0.5)

        # Update parameters
        self.update_slam_parameters()
        time.sleep(0.1)  # Wait for file write to complete

        # Launch optimization in new terminal
        try:
            # Create command to run in new terminal
            bash_cmd = (
                'source /opt/ros/jazzy/setup.bash && '
                'source /home/kkw/localization_ws/install/setup.bash && '
                'ros2 launch long_term_mapping lt_mapper.launch.py'
            )

            # Open new terminal and run the command
            cmd = [
                'gnome-terminal',
                '--title=Multi-Session SLAM Optimization',
                '--',
                'bash', '-c',
                f'{bash_cmd}; echo ""; echo "Press Enter to close this window..."; read'
            ]

            self.get_logger().info(f'Opening new terminal for optimization')

            # Launch in new terminal - it's independent from web_gui
            subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True  # Create new process group
            )

            self.get_logger().info('Optimization launched in new terminal')
            self.slam_status = "Optimization launched in new terminal"
            return True
        except Exception as e:
            self.slam_status = f"Error: Failed to start optimization - {str(e)}"
            self.get_logger().error(f'Failed to start optimization: {str(e)}')
            import traceback
            traceback.print_exc()
            return False

    def update_slam_parameters(self):
        param_file = "/home/kkw/localization_ws/src/long_term_mapping/config/params.yaml"

        try:
            with open(param_file, 'r') as f:
                config = yaml.safe_load(f)

            # Update parameters
            if '/**' in config and 'ros__parameters' in config['/**']:
                config['/**']['ros__parameters']['directory1'] = self.slam_map1
                config['/**']['ros__parameters']['directory2'] = self.slam_map2
                config['/**']['ros__parameters']['output_directory'] = self.slam_output

            # Save with proper YAML formatting (default_flow_style=False for readability)
            with open(param_file, 'w') as f:
                yaml.dump(config, f, default_flow_style=False, sort_keys=False)

            self.get_logger().info('SLAM parameters updated successfully')
            self.get_logger().info(f'  directory1: {self.slam_map1}')
            self.get_logger().info(f'  directory2: {self.slam_map2}')
            self.get_logger().info(f'  output_directory: {self.slam_output}')
        except Exception as e:
            self.get_logger().error(f'Failed to update SLAM parameters: {str(e)}')

    def slam_complete_callback(self, msg):
        if msg.data:
            self.slam_status = "Optimization complete!"
            self.get_logger().info('Optimization completed successfully')

    def get_slam_state(self):
        # Check if SLAM process is running
        is_running = self.slam_process is not None and self.slam_process.poll() is None
        return {
            'map1': self.slam_map1,
            'map2': self.slam_map2,
            'output': self.slam_output,
            'status': self.slam_status,
            'is_running': is_running
        }
    
    def get_localization_state(self):
        # Check if Localization process is running
        is_running = self.localization_process is not None and self.localization_process.poll() is None
        return {
            'is_running': is_running
        }

    # Bag Recorder Functions
    def set_recorder_bag_name(self, bag_name):
        """Set the bag name for recording"""
        self.recorder_bag_name = bag_name
        self.get_logger().info(f'Recorder bag name set to: {bag_name}')
        return True

    def get_recorder_topics(self):
        """Get list of current ROS2 topics with type information.

        Returns:
            list[dict]: [{'name': '/topic', 'type': 'pkg/msg/Type'}, ...]
        """
        try:
            # ros2 topic list -t: 타입 정보 포함 출력 (예: /topic [sensor_msgs/msg/PointCloud2])
            cmd = ['bash', '-c', 'source /opt/ros/jazzy/setup.bash && ros2 topic list -t']
            result = subprocess.run(
                cmd,
                env=self._ros_env,
                capture_output=True,
                text=True,
                timeout=5
            )

            if result.returncode == 0:
                topics = []
                for line in result.stdout.strip().split('\n'):
                    line = line.strip()
                    if not line:
                        continue
                    # ros2 topic list -t 출력 형식: /topic_name [pkg/msg/Type]
                    if ' [' in line and line.endswith(']'):
                        name, rest = line.split(' [', 1)
                        topic_type = rest[:-1].strip()  # trailing ']' 제거
                        topics.append({'name': name.strip(), 'type': topic_type})
                    else:
                        # 타입 정보 없는 경우 fallback
                        topics.append({'name': line, 'type': ''})
                self.get_logger().info(f'Found {len(topics)} ROS2 topics')
                return topics
            else:
                self.get_logger().error(f'Failed to get ROS2 topics. Return code: {result.returncode}')
                self.get_logger().error(f'stderr: {result.stderr}')
                return []
        except Exception as e:
            self.get_logger().error(f'Error getting topics: {str(e)}')
            return []

    def record_bag(self, topics, save_as_ros1=False):
        """Start or stop bag recording.

        Args:
            topics: 녹화할 토픽 목록. 문자열 리스트 또는 {'name', 'type'} dict 리스트.
            save_as_ros1 (bool): True이면 Ros1BagRecorderThread로 .bag 직접 기록,
                                 False이면 기존 ros2 bag record subprocess 사용.

        Returns:
            bool: 성공 여부
        """
        if self.recorder_recording:
            # Stop recording — ros1 thread 또는 ros2 subprocess 정리
            if self.recorder_ros1_thread:
                self.get_logger().info('Stopping ROS1 bag recording...')
                self.recorder_ros1_thread.stop()
                self.recorder_ros1_thread = None
            elif self.recorder_process:
                self.get_logger().info('Stopping bag recording...')
                self.recorder_process.terminate()
                try:
                    self.recorder_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.recorder_process.kill()
                self.recorder_process = None
            self.recorder_recording = False
            self.recorder_mode = 'ros2'
            return True
        else:
            # Start recording
            if not self.recorder_bag_name:
                self.get_logger().error('Bag name not set')
                return False

            if not topics or len(topics) == 0:
                self.get_logger().error('No topics selected')
                return False

            # topics는 문자열 리스트 또는 {name, type} dict 리스트 모두 지원
            topic_names = []
            topic_type_map = {}
            for t in topics:
                if isinstance(t, dict):
                    name = t.get('name', '')
                    tp = t.get('type', '')
                    if name:
                        topic_names.append(name)
                        if tp:
                            topic_type_map[name] = tp
                elif isinstance(t, str) and t:
                    topic_names.append(t)

            if not topic_names:
                self.get_logger().error('No valid topics selected')
                return False

            if save_as_ros1:
                # ROS1 .bag 직접 녹화 (Ros1BagRecorderThread)
                if not topic_type_map:
                    self.get_logger().error('Topic type information required for ROS1 recording')
                    return False

                output_path = f'/home/kkw/dataset/{self.recorder_bag_name}.bag'
                self.get_logger().info(f'Starting ROS1 bag recording to: {output_path}')
                self.get_logger().info(f'Recording topics: {", ".join(topic_names)}')

                try:
                    self.recorder_ros1_thread = Ros1BagRecorderThread(
                        output_path, topic_type_map, self
                    )
                    self.recorder_ros1_thread.start()
                    self.recorder_recording = True
                    self.recorder_mode = 'ros1'
                    self.get_logger().info('ROS1 bag recording started')
                    return True
                except Exception as e:
                    self.get_logger().error(f'Failed to start ROS1 recording: {str(e)}')
                    return False
            else:
                # 기존 ros2 bag record subprocess
                output_dir = f'/home/kkw/dataset/{self.recorder_bag_name}'
                cmd = [
                    'bash', '-c',
                    f'cd /home/kkw/dataset && '
                    f'source /opt/ros/jazzy/setup.bash && '
                    f'ros2 bag record -o {self.recorder_bag_name} ' + ' '.join(topic_names)
                ]

                self.get_logger().info(f'Starting bag recording in: {output_dir}')
                self.get_logger().info(f'Recording topics: {", ".join(topic_names)}')

                try:
                    self.recorder_process = subprocess.Popen(
                        cmd,
                        env=self._ros_env,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        start_new_session=True
                    )
                    self.recorder_recording = True
                    self.recorder_mode = 'ros2'
                    self.get_logger().info('Bag recording started')
                    return True
                except Exception as e:
                    self.get_logger().error(f'Failed to start recording: {str(e)}')
                    return False

    def get_recorder_state(self):
        """Get current recorder state"""
        return {
            'bag_name': self.recorder_bag_name,
            'recording': self.recorder_recording,
            'mode': self.recorder_mode,  # 'ros2' | 'ros1'
        }

    # File Player Functions

    def _is_kitti_drive_path(self, path: str) -> bool:
        """경로가 KITTI drive 디렉토리인지 확인한다.
        velodyne_points/timestamps.txt 파일 존재 여부로 판별한다.
        """
        if not path or not os.path.isdir(path):
            return False
        ts_file = os.path.join(path, 'velodyne_points', 'timestamps.txt')
        return os.path.isfile(ts_file)

    def _load_kitti_direct(self, path: str) -> bool:
        """KITTI drive 디렉토리를 직접 File Player로 로드한다.

        velodyne_points/timestamps.txt 에서 타임스탬프를 읽어
        기존 data_stamp 구조(timestamp_ns → frame_idx_str)를 구축한다.
        playback_worker 에서 player_is_kitti 플래그를 보고 KITTI 파일을 직접 읽어 publish.
        """
        from ros2_autonav_webui.kitti_converter import KittiConverter

        # 기존 재생 정지
        self.player_playing = False
        self.player_paused  = False
        if self.playback_active:
            self.playback_active = False
            old_thread = self.playback_thread
            self.playback_thread = None
            if old_thread and old_thread.is_alive():
                old_thread.join(timeout=1.0)

        ts_file = os.path.join(path, 'velodyne_points', 'timestamps.txt')
        try:
            conv = KittiConverter()
            timestamps_ns = conv._load_timestamps(ts_file)
        except Exception as e:
            self.get_logger().error(f'Failed to read KITTI timestamps: {e}')
            return False

        if not timestamps_ns:
            self.get_logger().error(f'No valid timestamps in {ts_file}')
            return False

        # data_stamp: {timestamp_ns: frame_index_str}
        self.data_stamp = {}
        for idx, ts_ns in enumerate(timestamps_ns):
            if ts_ns > 0:
                self.data_stamp[ts_ns] = f'{idx:010d}'

        if not self.data_stamp:
            self.get_logger().error('data_stamp is empty after parsing KITTI timestamps')
            return False

        sorted_stamps = sorted(self.data_stamp.keys())
        self.player_initial_stamp   = sorted_stamps[0]
        self.player_last_stamp      = sorted_stamps[-1]
        self.player_timestamp       = self.player_initial_stamp
        self.player_processed_stamp = 0
        self.player_slider_pos      = 0
        self.player_seek_requested  = False
        self.player_seek_to_stamp   = self.player_initial_stamp

        self.player_path        = path
        self.player_is_kitti    = True
        self.player_is_ros2_bag = False
        self.kitti_drive_path   = path

        self.livox_cache = {}
        self.cam_cache   = {}

        # KITTI 전용 publisher만 초기화 (ConPR 토픽 오염 방지)
        self._init_kitti_ros_interfaces()
        self.player_data_loaded = True

        self.get_logger().info(
            f'KITTI drive loaded: {path} '
            f'({len(self.data_stamp)} frames, '
            f'{(sorted_stamps[-1] - sorted_stamps[0]) / 1e9:.1f}s)'
        )
        return True

    def _publish_kitti_frame(self, frame_idx: int, stamp_ns: int):
        """KITTI 프레임(velodyne + camera)을 ROS2 토픽으로 publish한다."""
        from ros2_autonav_webui.kitti_converter import KittiConverter
        from rclpy.time import Time as RclpyTime
        from builtin_interfaces.msg import Time as TimeMsg

        drive_path = self.kitti_drive_path
        if not drive_path:
            return

        stamp_msg = TimeMsg()
        stamp_msg.sec      = int(stamp_ns // 1_000_000_000)
        stamp_msg.nanosec  = int(stamp_ns %  1_000_000_000)

        # ── Velodyne PointCloud2 ─────────────────────────────────────
        bin_path = os.path.join(
            drive_path, 'velodyne_points', 'data', f'{frame_idx:010d}.bin')
        if os.path.isfile(bin_path):
            try:
                conv = KittiConverter()
                pc2_msg = conv._make_pointcloud2_msg(bin_path, stamp_msg)
                if pc2_msg and self.kitti_velo_pub:
                    self.kitti_velo_pub.publish(pc2_msg)
            except Exception as e:
                self.get_logger().warn(f'KITTI velodyne publish failed (frame {frame_idx}): {e}')

        # ── Camera image (image_02 우선, 없으면 image_00) ─────────────
        for cam_dir, encoding in [('image_02', 'bgr8'), ('image_00', 'mono8')]:
            img_path = os.path.join(
                drive_path, cam_dir, 'data', f'{frame_idx:010d}.png')
            if os.path.isfile(img_path):
                try:
                    conv = KittiConverter()
                    img_msg = conv._make_image_msg(img_path, encoding, stamp_msg)
                    if img_msg and self.kitti_cam_pub:
                        self.kitti_cam_pub.publish(img_msg)
                except Exception as e:
                    self.get_logger().warn(f'KITTI image publish failed (frame {frame_idx}): {e}')
                break

    def _is_ros2_bag_path(self, path: str) -> bool:
        """경로가 ROS2 bag (.db3 파일 또는 bag 디렉토리)인지 확인한다."""
        if not path:
            return False
        # .db3 파일 직접 지정
        if path.endswith('.db3') and os.path.exists(path):
            return True
        # 디렉토리인 경우: metadata.yaml 또는 .db3 파일 포함 여부 확인
        if os.path.isdir(path):
            if os.path.exists(os.path.join(path, 'metadata.yaml')):
                return True
            db3_files = glob.glob(os.path.join(path, '*.db3'))
            if db3_files:
                return True
        return False

    def _load_ros2_bag_player(self, path: str) -> bool:
        """ROS2 bag 경로를 기존 bag_play_toggle 인프라로 로드한다.

        .db3 파일이 지정된 경우 부모 디렉토리를 bag_path로 사용한다.

        player_play_toggle()에서 bag_play_toggle()로 위임되도록
        player_path / player_data_loaded / player_is_ros2_bag 도 함께 설정한다.
        """
        # 기존 ConPR playback 스레드 정지
        self.player_playing = False
        self.player_paused = False
        if self.playback_active:
            self.playback_active = False
            old_thread = self.playback_thread
            self.playback_thread = None
            if old_thread and old_thread.is_alive():
                old_thread.join(timeout=1.0)

        # 기존 bag 재생 중이면 중지
        if self.bag_playing:
            if self.bag_process:
                self.bag_process.terminate()
                try:
                    self.bag_process.wait(timeout=5)
                except Exception:
                    self.bag_process.kill()
                self.bag_process = None
            self.bag_playing = False
            self.bag_paused = False

        # .db3 파일인 경우 부모 디렉토리를 bag 경로로 사용
        if path.endswith('.db3'):
            bag_dir = os.path.dirname(path)
        else:
            bag_dir = path

        self.bag_path = bag_dir

        # ── File Player UI 상태 동기화 ─────────────────────────────────────
        # UI가 player_path / player_data_loaded 를 읽으므로 올바른 값으로 갱신
        self.player_path = bag_dir
        self.player_data_loaded = True   # play 버튼 활성화
        self.player_is_ros2_bag = True   # player_play_toggle 에서 분기 용도
        self.player_slider_pos = 0
        self.player_timestamp = 0
        self.livox_cache = {}
        self.cam_cache = {}

        self.get_logger().info(f'Loaded ROS2 bag for player: {bag_dir}')
        return True

    def load_player_data(self, path):
        """Load file player data from the specified path"""
        # KITTI drive 디렉토리인 경우 직접 플레이어로 로드
        if self._is_kitti_drive_path(path):
            self.get_logger().info(f'Detected KITTI drive path: {path}')
            return self._load_kitti_direct(path)

        # ROS2 bag 경로인 경우 기존 bag playback 인프라로 위임
        if self._is_ros2_bag_path(path):
            self.get_logger().info(f'Detected ROS2 bag path: {path}')
            return self._load_ros2_bag_player(path)

        # 기존 재생 스레드를 완전히 정지시킨 후 새 데이터 로드
        # (두 번째 디렉토리 로드 후 재생 안 되는 버그 수정)
        self.player_playing = False
        self.player_paused = False
        self.player_processed_stamp = 0
        self.player_prev_time = 0
        self.player_slider_pos = 0
        self.player_timestamp = 0
        self.player_seek_requested = False
        self.player_seek_to_stamp  = 0
        self.player_is_ros2_bag = False   # ConPR 모드로 복귀
        self.player_is_kitti = False      # KITTI 모드 해제

        if self.playback_active:
            self.playback_active = False
            thread = self.playback_thread
            self.playback_thread = None
            if thread and thread.is_alive():
                thread.join(timeout=1.0)

        # 캐시 초기화 (이전 데이터 완전 제거)
        self.livox_cache = {}
        self.cam_cache = {}

        self.player_path = path
        self.player_data_loaded = False

        try:
            # Check if data_stamp.csv exists
            stamp_file = os.path.join(path, 'data_stamp.csv')
            if not os.path.exists(stamp_file):
                self.get_logger().error(f'data_stamp.csv not found in {path}')
                return False

            # Load data stamps
            self.data_stamp = {}
            with open(stamp_file, 'r') as f:
                for line in f:
                    try:
                        parts = line.strip().split(',')
                        if len(parts) == 2:
                            stamp = int(parts[0])
                            data_name = parts[1]
                            self.data_stamp[stamp] = data_name
                    except ValueError as e:
                        self.get_logger().warn(f'Skipping malformed line in data_stamp.csv: {line.strip()} - {str(e)}')
                        continue

            if not self.data_stamp:
                self.get_logger().error('No valid data found in data_stamp.csv')
                return False

            timestamps = sorted(self.data_stamp.keys())
            self.player_initial_stamp = timestamps[0]
            self.player_last_stamp = timestamps[-1]
            self.player_timestamp = self.player_initial_stamp

            self.get_logger().info(f'Loaded {len(self.data_stamp)} data stamps')

            # Load pose data
            pose_file = os.path.join(path, 'pose.csv')
            if os.path.exists(pose_file):
                self.pose_data = {}
                with open(pose_file, 'r') as f:
                    for line in f:
                        try:
                            parts = line.strip().split(',')
                            if len(parts) == 4:
                                stamp = int(parts[0])
                                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                                self.pose_data[stamp] = (x, y, z)
                        except ValueError as e:
                            self.get_logger().warn(f'Skipping malformed line in pose.csv: {line.strip()} - {str(e)}')
                            continue
                self.get_logger().info(f'Loaded {len(self.pose_data)} pose data points')

            # Load IMU data (stamp, q_x, q_y, q_z, q_w, w_x, w_y, w_z, a_x, a_y, a_z)
            imu_file = os.path.join(path, 'imu.csv')
            if os.path.exists(imu_file):
                self.imu_data = {}
                with open(imu_file, 'r') as f:
                    for line in f:
                        try:
                            parts = line.strip().split(',')
                            if len(parts) >= 11:
                                stamp = int(parts[0])
                                # Store IMU data as tuple (q_x, q_y, q_z, q_w, w_x, w_y, w_z, a_x, a_y, a_z)
                                imu = tuple(float(p) for p in parts[1:11])
                                self.imu_data[stamp] = imu
                        except ValueError as e:
                            self.get_logger().warn(f'Skipping malformed line in imu.csv: {line.strip()} - {str(e)}')
                            continue
                self.get_logger().info(f'Loaded {len(self.imu_data)} IMU data points')

            # Load LiDAR file list
            lidar_dir = os.path.join(path, 'LiDAR')
            if os.path.exists(lidar_dir):
                self.livox_file_list = sorted(glob.glob(os.path.join(lidar_dir, '*.bin')))
                self.get_logger().info(f'Found {len(self.livox_file_list)} LiDAR files')
            else:
                self.livox_file_list = []
                self.get_logger().warn('LiDAR directory not found')

            # Load Camera file list (directory name: 'Camera')
            cam_dir = os.path.join(path, 'Camera')
            if os.path.exists(cam_dir):
                # Support multiple image formats
                patterns = ['*.jpg', '*.png', '*.jpeg', '*.JPG', '*.PNG']
                self.cam_file_list = []
                for pattern in patterns:
                    self.cam_file_list.extend(glob.glob(os.path.join(cam_dir, pattern)))
                self.cam_file_list = sorted(self.cam_file_list)
                self.get_logger().info(f'Found {len(self.cam_file_list)} camera images in Camera/')
            else:
                self.cam_file_list = []
                self.get_logger().warn('Camera directory not found (expected: {}/Camera/)'.format(path))

            self.player_data_loaded = True
            # Lazy-initialize File Player ROS2 publishers/subscribers on first load
            self._init_file_player_ros_interfaces()
            return True

        except Exception as e:
            self.get_logger().error(f'Failed to load player data: {str(e)}')
            import traceback
            traceback.print_exc()
            return False

    # ── KITTI 변환 함수 ────────────────────────────────────────────────────────

    def scan_kitti_directory(self, path: str) -> dict:
        """KITTI 데이터셋 디렉토리를 탐색하여 calib/drive 정보를 반환한다.

        Args:
            path: 사용자가 선택한 날짜 디렉토리 (예: /path/to/2011_09_30)

        Returns:
            {'success': True, 'scan_result': {...}} or {'success': False, 'error': '...'}
        """
        try:
            from ros2_autonav_webui.kitti_converter import KittiConverter
            converter = KittiConverter()
            result = converter.scan_directory(path)
            self.get_logger().info(
                f'KITTI scan complete: date={result["date"]}, '
                f'{len(result["drive_dirs"])} drive(s) found')
            return {'success': True, 'scan_result': result}
        except Exception as e:
            self.get_logger().error(f'KITTI scan failed: {str(e)}')
            import traceback
            traceback.print_exc()
            return {'success': False, 'error': str(e)}

    def start_kitti_conversion(
        self,
        base_dir: str,
        calib_dir: str,
        data_path: str,
        drive_name: str,
    ) -> dict:
        """KITTI 데이터를 ROS2 bag으로 변환하는 백그라운드 스레드를 시작한다.

        변환 진행률은 WebSocket(포트 8081)을 통해 전체 클라이언트에 push된다.

        Returns:
            {'success': True, 'output_bag_path': '...'} or {'success': False, 'error': '...'}
        """
        if self.kitti_converter_running:
            return {'success': False, 'error': 'Conversion already in progress'}

        output_bag_path = os.path.join(base_dir, 'converted', drive_name)

        def _run():
            self.kitti_converter_running = True
            try:
                from ros2_autonav_webui.kitti_converter import KittiConverter
                converter = KittiConverter()

                def _progress_cb(pct: int, msg: str):
                    self.pc2_ws_server.broadcast_json_all({
                        'type': 'kitti_convert_progress',
                        'progress': pct,
                        'message': msg,
                    })

                self.get_logger().info(
                    f'KITTI conversion started: {data_path} → {output_bag_path}')
                converter.convert_to_ros2bag(
                    calib_dir=calib_dir,
                    data_path=data_path,
                    output_bag_path=output_bag_path,
                    progress_cb=_progress_cb,
                )
                self.get_logger().info(f'KITTI conversion complete: {output_bag_path}')
                self.pc2_ws_server.broadcast_json_all({
                    'type': 'kitti_convert_done',
                    'bag_path': output_bag_path,
                })
            except Exception as e:
                self.get_logger().error(f'KITTI conversion failed: {str(e)}')
                import traceback
                traceback.print_exc()
                self.pc2_ws_server.broadcast_json_all({
                    'type': 'kitti_convert_error',
                    'error': str(e),
                })
            finally:
                self.kitti_converter_running = False

        self.kitti_convert_thread = threading.Thread(
            target=_run, daemon=True, name='kitti-convert')
        self.kitti_convert_thread.start()
        return {'success': True, 'message': 'Conversion started', 'output_bag_path': output_bag_path}

    def load_livox_data(self, stamp):
        """Load LiDAR data from .bin file for given timestamp"""
        if not LIVOX_AVAILABLE or not self.livox_pub:
            return None

        # Check cache first
        if stamp in self.livox_cache:
            return self.livox_cache[stamp]

        # Find matching .bin file
        bin_filename = f"{stamp}.bin"
        bin_path = os.path.join(self.player_path, 'LiDAR', bin_filename)

        if not os.path.exists(bin_path):
            return None

        try:
            # Read binary file
            with open(bin_path, 'rb') as f:
                data = f.read()

            # Parse CustomPoint data
            # Each point: x(float32), y(float32), z(float32), reflectivity(uint8), tag(uint8), line(uint8), offset_time(uint32)
            # Total: 4+4+4+1+1+1+4 = 19 bytes per point
            point_size = 19
            num_points = len(data) // point_size

            msg = CustomMsg()
            msg.header.stamp = Time(nanoseconds=stamp).to_msg()
            msg.header.frame_id = 'livox'
            msg.timebase = stamp
            msg.point_num = num_points
            msg.lidar_id = 0
            msg.rsvd = [0, 0, 0]

            # Parse points
            for i in range(num_points):
                offset = i * point_size
                point_data = data[offset:offset + point_size]

                if len(point_data) < point_size:
                    break

                # Unpack: 3 floats (x,y,z), 3 uint8 (reflectivity, tag, line), 1 uint32 (offset_time)
                x, y, z = struct.unpack('fff', point_data[0:12])
                reflectivity, tag, line = struct.unpack('BBB', point_data[12:15])
                offset_time, = struct.unpack('I', point_data[15:19])

                point = CustomPoint()
                point.x = x
                point.y = y
                point.z = z
                point.reflectivity = reflectivity
                point.tag = tag
                point.line = line
                point.offset_time = offset_time

                msg.points.append(point)

            # Cache the message
            self.livox_cache[stamp] = msg
            return msg

        except Exception as e:
            self.get_logger().error(f'Failed to load LiDAR data for stamp {stamp}: {str(e)}')
            return None

    def load_camera_data(self, stamp):
        """Load camera image for given timestamp"""
        # Check cache first
        if stamp in self.cam_cache:
            return self.cam_cache[stamp]

        # Find matching image file in 'Camera' directory
        # Image files might be named as: stamp.jpg or stamp.png
        img_path = None
        for ext in ['.jpg', '.png', '.jpeg', '.JPG', '.PNG']:
            test_path = os.path.join(self.player_path, 'Camera', f'{stamp}{ext}')
            if os.path.exists(test_path):
                img_path = test_path
                break

        if not img_path:
            return None

        try:
            # Read image using OpenCV
            cv_image = cv2.imread(img_path)
            if cv_image is None:
                return None

            # Convert to ROS Image message
            img_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            img_msg.header.stamp = Time(nanoseconds=stamp).to_msg()
            img_msg.header.frame_id = 'camera'

            # Create CameraInfo message (with default values)
            cam_info_msg = CameraInfo()
            cam_info_msg.header.stamp = img_msg.header.stamp
            cam_info_msg.header.frame_id = 'camera'
            cam_info_msg.height = cv_image.shape[0]
            cam_info_msg.width = cv_image.shape[1]

            # Cache the messages
            self.cam_cache[stamp] = (img_msg, cam_info_msg)
            return (img_msg, cam_info_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to load camera data for stamp {stamp}: {str(e)}')
            return None

    def timer_callback(self):
        """Timer callback (100μs 주기).

        - 정지 상태: processed_stamp 를 0 으로 리셋
        - KITTI 재생 중: 100μs 마다 /clock 을 publish 하여 시각화 끊김 최소화
          (KITTI 데이터는 10Hz 이지만 clock 은 더 자주 갱신해야 RViz2 가 부드러움)
        """
        if not self.player_playing:
            self.player_processed_stamp = 0
            return

        # KITTI 재생 중: 매 timer tick 마다 /clock 갱신 (100μs → 10000Hz)
        # player_processed_stamp 는 playback_worker 가 관리하므로 읽기만 한다.
        if getattr(self, 'player_is_kitti', False) and self.clock_pub:
            try:
                clock_ns = self.player_initial_stamp + self.player_processed_stamp
                clock_msg = Clock()
                clock_msg.clock = Time(nanoseconds=clock_ns).to_msg()
                self.clock_pub.publish(clock_msg)
            except Exception:
                pass

    def bag_timer_callback(self):
        """Timer callback to update bag current time during playback"""
        if self.bag_playing and not self.bag_paused:
            current_real_time = time.time()
            elapsed_time = current_real_time - self.bag_start_real_time
            # bag_playback_rate 배속을 반영하여 bag 시간 업데이트
            self.bag_current_time = self.bag_start_offset + elapsed_time * self.bag_playback_rate

            # Stop when reaching end
            if self.bag_current_time >= self.bag_duration:
                self.bag_current_time = self.bag_duration
                # Note: We don't auto-stop here, let ros2 bag play finish naturally

    def player_play_toggle(self):
        """Toggle play/stop"""
        if not self.player_data_loaded:
            self.get_logger().warn('No data loaded. Please load data first.')
            return False

        # ── ROS2 bag 모드: bag_play_toggle()로 위임 ────────────────────────
        if getattr(self, 'player_is_ros2_bag', False):
            self.get_logger().info('ROS2 bag mode: delegating to bag_play_toggle()')
            return self.bag_play_toggle()

        self.player_playing = not self.player_playing
        self.player_paused = False

        if self.player_playing:
            self.get_logger().info('Starting playback...')

            # 이전 스레드가 살아 있으면 완전히 종료 후 새로 시작
            # (디렉토리 재선택 후 재생 안 되는 버그 근본 해결)
            if self.playback_active:
                self.playback_active = False
                old_thread = self.playback_thread
                self.playback_thread = None
                if old_thread and old_thread.is_alive():
                    old_thread.join(timeout=0.5)

            self.player_prev_time = time.time()
            self.playback_active = True
            self.playback_thread = threading.Thread(
                target=self.playback_worker, daemon=True
            )
            self.playback_thread.start()
        else:
            # End 버튼: 처음 위치로 리셋, 스레드도 정지
            self.get_logger().info('Stopping playback - resetting to beginning...')
            if self.playback_active:
                self.playback_active = False
                old_thread = self.playback_thread
                self.playback_thread = None
                if old_thread and old_thread.is_alive():
                    old_thread.join(timeout=0.5)
            self.player_processed_stamp = 0
            self.player_timestamp = self.player_initial_stamp
            self.player_slider_pos = 0
            self.player_paused = False

        return True

    def player_pause_toggle(self):
        """Toggle pause/resume"""
        if self.player_playing:
            self.player_paused = not self.player_paused
            status = "Paused" if self.player_paused else "Resumed"
            self.get_logger().info(f'Playback {status}')
            return True
        return False

    def get_bag_info(self):
        """Get bag file info including topics and duration.

        Branches based on file extension:
        - .bag  → ROS1 bag (parsed via rosbags library)
        - other → ROS2 bag (parsed via ros2 bag info command)
        """
        if not self.bag_path:
            self.get_logger().warn('No bag file loaded.')
            return {'topics': [], 'duration': 0.0, 'bag_type': 'ros2'}

        if self.bag_path.endswith('.bag'):
            return self._get_ros1_bag_info()

        try:
            # Use ros2 bag info to get topic list and duration
            cmd = ['ros2', 'bag', 'info', self.bag_path]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)

            if result.returncode != 0:
                self.get_logger().error(f'Failed to get bag info: {result.stderr}')
                return {'topics': [], 'duration': 0.0, 'bag_type': 'ros2'}

            # Parse output to extract topics and duration
            topics = []
            duration = 0.0
            lines = result.stdout.split('\n')

            self.get_logger().info('Parsing bag info output:')

            for line in lines:
                self.get_logger().info(f'  Line: {line.strip()}')

                # Parse duration (e.g., "Duration: 123.456s")
                if 'Duration' in line and 's' in line:
                    try:
                        # Extract duration value
                        duration_str = line.split(':')[1].strip()
                        # Remove 's' and convert to float
                        duration = float(duration_str.replace('s', '').strip())
                        self.get_logger().info(f'  Found duration: {duration}s')
                    except:
                        pass

                # Parse topics - looking for lines with "Topic: /topic_name | Count: X | Connection: Y"
                if 'Topic:' in line and '|' in line:
                    try:
                        # Extract topic name between "Topic:" and first "|"
                        parts = line.split('|')
                        topic_part = parts[0]
                        topic_name = topic_part.split('Topic:')[1].strip()
                        topics.append(topic_name)
                        self.get_logger().info(f'  Found topic: {topic_name}')
                    except:
                        pass

            self.bag_topics = topics
            self.bag_duration = duration
            self.get_logger().info(f'Bag info: {len(topics)} topics, duration: {duration}s')
            self.get_logger().info(f'Topics: {topics}')

            return {'topics': topics, 'duration': duration, 'bag_type': 'ros2'}

        except subprocess.TimeoutExpired:
            self.get_logger().error('Timeout while getting bag info')
            return {'topics': [], 'duration': 0.0, 'bag_type': 'ros2'}
        except Exception as e:
            self.get_logger().error(f'Failed to get bag info: {str(e)}')
            import traceback
            traceback.print_exc()
            return {'topics': [], 'duration': 0.0, 'bag_type': 'ros2'}

    def _get_ros1_bag_info(self):
        """Get ROS1 .bag file info using the rosbags library.

        각 토픽에 대해 ROS2 Python 패키지로 import 가능 여부를 검사하여
        publishable 필드를 포함한 딕셔너리 목록을 반환합니다.

        Returns:
            dict: {
                'topics': list[dict],  # {name, type, publishable} 형태
                'duration': float,
                'bag_type': 'ros1'
            }
        """
        import importlib

        def _check_publishable(ros1_type_str):
            """importlib으로 ROS2 메시지 클래스 존재 여부 검사.

            rosbags 라이브러리는 ROS1 bag에서도 ROS2 포맷으로 타입을 반환합니다.
            - ROS1 포맷: 'sensor_msgs/Image'       (parts 2개)
            - ROS2 포맷: 'sensor_msgs/msg/Image'   (parts 3개)
            두 포맷을 모두 처리합니다.

            Args:
                ros1_type_str (str): 예) 'sensor_msgs/msg/Image' 또는 'sensor_msgs/Image'

            Returns:
                bool: True if importable and class exists
            """
            try:
                parts = ros1_type_str.split('/')
                if len(parts) == 2:
                    # ROS1 포맷: 'sensor_msgs/Image'
                    pkg, msg_class = parts[0], parts[1]
                elif len(parts) == 3 and parts[1] == 'msg':
                    # ROS2 포맷: 'sensor_msgs/msg/Image'
                    pkg, msg_class = parts[0], parts[2]
                else:
                    return False
                mod = importlib.import_module(f'{pkg}.msg')
                return hasattr(mod, msg_class)
            except Exception:
                return False

        try:
            from rosbags.rosbag1 import Reader
            with Reader(self.bag_path) as reader:
                # {topic_name: TopicInfo}
                raw_topics = reader.topics
                duration = (reader.end_time - reader.start_time) / 1e9

            # publishable 여부 포함 딕셔너리 목록 생성
            topic_dicts = []
            topic_names = []  # 기존 bag_topics 호환용
            for topic_name, topic_info in raw_topics.items():
                ros1_type = topic_info.msgtype
                publishable = _check_publishable(ros1_type)
                topic_dicts.append({
                    'name': topic_name,
                    'type': ros1_type,
                    'publishable': publishable,
                })
                topic_names.append(topic_name)

            # 기존 호환 상태 변수 업데이트 (이름 목록)
            self.bag_topics = topic_names
            self.bag_duration = duration

            publishable_count = sum(1 for t in topic_dicts if t['publishable'])
            self.get_logger().info(
                f'ROS1 bag info: {len(topic_dicts)} topics '
                f'({publishable_count} publishable), duration: {duration:.3f}s'
            )
            for t in topic_dicts:
                flag = '✓' if t['publishable'] else '✗'
                self.get_logger().info(f'  [{flag}] {t["name"]} ({t["type"]})')

            return {'topics': topic_dicts, 'duration': duration, 'bag_type': 'ros1'}

        except ImportError:
            self.get_logger().error(
                'rosbags library not found. Install with: pip install rosbags'
            )
            return {'topics': [], 'duration': 0.0, 'bag_type': 'ros1'}
        except Exception as e:
            self.get_logger().error(f'Failed to read ROS1 bag: {str(e)}')
            import traceback
            traceback.print_exc()
            return {'topics': [], 'duration': 0.0, 'bag_type': 'ros1'}

    # ------------------------------------------------------------------
    # ROS1 Bag Player — 상태 메서드
    # ------------------------------------------------------------------
    def start_ros1_playback(self, bag_path, topics, rate):
        """ROS1 bag 재생 시작.

        기존 스레드가 있으면 중지한 후 새 스레드를 시작합니다.

        Args:
            bag_path (str): ROS1 .bag 파일 경로
            topics (list[str]): publish할 토픽 목록 (빈 리스트 = 전체)
            rate (float): 재생 속도 배율

        Returns:
            bool: True if successfully started
        """
        # 기존 스레드 정리
        self.stop_ros1_playback()

        self.ros1_player_rate = rate
        self.ros1_player_thread = Ros1BagPlayerThread(bag_path, topics, rate, self)
        self.ros1_player_thread.start()
        self.get_logger().info(
            f'[ROS1 Player] Started: {bag_path}, topics={topics or "ALL"}, rate={rate}x'
        )
        return True

    def pause_ros1_playback(self):
        """ROS1 bag 재생 일시정지/재개 토글.

        Returns:
            dict: {'paused': bool}
        """
        thread = self.ros1_player_thread
        if thread is None or not thread.is_alive():
            return {'paused': False}

        status = thread.get_status()
        if status['status'] == 'paused':
            thread.resume()
            self.get_logger().info('[ROS1 Player] Resumed')
            return {'paused': False}
        else:
            thread.pause()
            self.get_logger().info('[ROS1 Player] Paused')
            return {'paused': True}

    def stop_ros1_playback(self):
        """ROS1 bag 재생 중지 및 스레드 join.

        Returns:
            bool: True
        """
        thread = self.ros1_player_thread
        if thread is not None and thread.is_alive():
            thread.stop()
            thread.join(timeout=5.0)
            self.get_logger().info('[ROS1 Player] Stopped')
        self.ros1_player_thread = None
        return True

    def get_ros1_playback_status(self):
        """현재 ROS1 재생 상태 반환.

        Returns:
            dict: {'status': str, 'elapsed_sec': float, 'total_sec': float}
        """
        thread = self.ros1_player_thread
        if thread is None or not thread.is_alive():
            return {'status': 'stopped', 'elapsed_sec': 0.0, 'total_sec': 0.0}
        return thread.get_status()

    def convert_ros1_bag(self):
        """Convert ROS1 .bag file to ROS2 bag format using rosbags-convert.

        Output directory: {bag_filename_without_ext}/ (same parent directory, no _ros2 suffix)

        Returns:
            dict: {'success': bool, 'output_path': str, 'error': str (on failure)}
        """
        if not self.bag_path:
            return {'success': False, 'error': 'No bag file loaded'}

        if not self.bag_path.endswith('.bag'):
            return {'success': False, 'error': 'Not a ROS1 .bag file'}

        try:
            import os
            import shutil
            bag_dir = os.path.dirname(self.bag_path)
            bag_name = os.path.splitext(os.path.basename(self.bag_path))[0]
            output_dir = os.path.join(bag_dir, bag_name)

            # 이미 변환된 디렉토리가 존재하면 삭제 후 재변환
            if os.path.isdir(output_dir):
                self.get_logger().info(f'Removing existing output dir: {output_dir}')
                shutil.rmtree(output_dir)

            self.get_logger().info(
                f'Converting ROS1 bag: {self.bag_path} -> {output_dir}'
            )

            # rosbags-convert 경로 탐색 (pip user install 경로 포함)
            convert_cmd = shutil.which('rosbags-convert') or '/home/kkw/.local/bin/rosbags-convert'
            if not os.path.isfile(convert_cmd):
                self.get_logger().error('rosbags-convert not found. Install with: pip install rosbags')
                return {'success': False, 'error': 'rosbags-convert not found. Run: pip install rosbags'}

            cmd = [convert_cmd, '--src', self.bag_path, '--dst', output_dir]
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=300  # Allow up to 5 minutes for large bags
            )

            if result.returncode != 0:
                error_msg = result.stderr.strip() or result.stdout.strip()
                self.get_logger().error(f'rosbags-convert failed: {error_msg}')
                return {'success': False, 'error': error_msg}

            self.get_logger().info(f'ROS1 bag converted successfully: {output_dir}')
            return {'success': True, 'output_path': output_dir}

        except subprocess.TimeoutExpired:
            self.get_logger().error('Timeout during ROS1 bag conversion')
            return {'success': False, 'error': 'Conversion timed out'}
        except FileNotFoundError:
            self.get_logger().error('rosbags-convert not found. Install with: pip install rosbags')
            return {'success': False, 'error': 'rosbags-convert not found. Run: pip install rosbags'}
        except Exception as e:
            self.get_logger().error(f'Failed to convert ROS1 bag: {str(e)}')
            import traceback
            traceback.print_exc()
            return {'success': False, 'error': str(e)}

    def convert_ros2_to_ros1_bag(self):
        """Convert ROS2 bag directory to ROS1 .bag format using rosbags-convert.

        Output: {bag_dirname}.bag (같은 부모 디렉토리, .bag 확장자)
        예: /path/to/my_bag/ → /path/to/my_bag.bag

        Returns:
            dict: {'success': bool, 'output_path': str, 'error': str (on failure)}
        """
        if not self.bag_path:
            return {'success': False, 'error': 'No bag file loaded'}

        if self.bag_path.endswith('.bag'):
            return {'success': False, 'error': 'Current bag is already a ROS1 .bag file'}

        try:
            import os
            import shutil
            bag_dir = self.bag_path.rstrip('/')
            output_path = bag_dir + '.bag'

            # 이미 변환된 .bag 파일이 존재하면 삭제 후 재변환
            if os.path.isfile(output_path):
                self.get_logger().info(f'Removing existing output file: {output_path}')
                os.remove(output_path)

            self.get_logger().info(
                f'Converting ROS2 bag: {self.bag_path} -> {output_path}'
            )

            # rosbags-convert 경로 탐색 (pip user install 경로 포함)
            convert_cmd = shutil.which('rosbags-convert') or '/home/kkw/.local/bin/rosbags-convert'
            if not os.path.isfile(convert_cmd):
                self.get_logger().error('rosbags-convert not found. Install with: pip install rosbags')
                return {'success': False, 'error': 'rosbags-convert not found. Run: pip install rosbags'}

            cmd = [convert_cmd, '--src', self.bag_path, '--dst', output_path]
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=300  # Allow up to 5 minutes for large bags
            )

            if result.returncode != 0:
                error_msg = result.stderr.strip() or result.stdout.strip()
                self.get_logger().error(f'rosbags-convert failed: {error_msg}')
                return {'success': False, 'error': error_msg}

            self.get_logger().info(f'ROS2 bag converted to ROS1 successfully: {output_path}')
            return {'success': True, 'output_path': output_path}

        except subprocess.TimeoutExpired:
            self.get_logger().error('Timeout during ROS2→ROS1 bag conversion')
            return {'success': False, 'error': 'Conversion timed out'}
        except FileNotFoundError:
            self.get_logger().error('rosbags-convert not found. Install with: pip install rosbags')
            return {'success': False, 'error': 'rosbags-convert not found. Run: pip install rosbags'}
        except Exception as e:
            self.get_logger().error(f'Failed to convert ROS2 bag to ROS1: {str(e)}')
            import traceback
            traceback.print_exc()
            return {'success': False, 'error': str(e)}

    def bag_play_toggle(self, selected_topics=None, start_offset=None, rate=1.0):
        """Toggle bag play/stop with optional topic selection, start offset, and playback rate.

        Args:
            selected_topics (list[str]|None): 재생할 토픽 목록 (None = 전체)
            start_offset (float|None): 시작 오프셋 (초)
            rate (float): 재생 속도 배율 (기본 1.0, 예: 0.5 = 절반 속도)
        """
        if not self.bag_path:
            self.get_logger().warn('No bag file loaded. Please load a bag file first.')
            return False

        if self.bag_playing:
            # Stop playback
            self.get_logger().info('Stopping bag playback...')
            if self.bag_process:
                self.bag_process.terminate()
                try:
                    self.bag_process.wait(timeout=5)
                except:
                    self.bag_process.kill()
                self.bag_process = None
            self.bag_playing = False
            self.bag_paused = False
            self.bag_current_time = 0.0
            self.bag_start_real_time = 0.0
        else:
            # Start playback using ros2 bag play
            self.get_logger().info(f'Starting bag playback: {self.bag_path}')
            try:
                # Build command with topic selection
                cmd = ['ros2', 'bag', 'play', self.bag_path]

                # Add start offset if specified
                if start_offset is not None and start_offset > 0:
                    cmd.extend(['--start-offset', str(start_offset)])
                    self.bag_start_offset = start_offset
                    self.bag_current_time = start_offset
                    self.get_logger().info(f'Starting from offset: {start_offset}s')
                else:
                    self.bag_start_offset = 0.0
                    self.bag_current_time = 0.0

                # Add playback rate (ros2 bag play --rate <rate>)
                rate = max(0.01, float(rate))
                self.bag_playback_rate = rate  # 현재 속도 저장
                if rate != 1.0:
                    cmd.extend(['--rate', str(rate)])
                self.get_logger().info(f'Playback rate: {rate}x')

                # Add topic filter if topics are selected
                if selected_topics and len(selected_topics) > 0:
                    self.bag_selected_topics = selected_topics
                    cmd.append('--topics')
                    cmd.extend(selected_topics)
                    self.get_logger().info(f'Playing selected topics: {selected_topics}')
                else:
                    self.get_logger().info('Playing all topics')

                self.get_logger().info(f'Command: {" ".join(cmd)}')

                # Debug: print environment variables
                self.get_logger().info(f'ROS_DOMAIN_ID: {self._ros_env.get("ROS_DOMAIN_ID", "not set")}')
                self.get_logger().info(f'ROS_DISTRO: {self._ros_env.get("ROS_DISTRO", "not set")}')

                self.bag_process = subprocess.Popen(cmd,
                                                     env=self._ros_env,
                                                     stdout=subprocess.PIPE,
                                                     stderr=subprocess.STDOUT)  # Combine stderr with stdout
                self.bag_playing = True
                self.bag_paused = False
                self.bag_start_real_time = time.time()
                self.get_logger().info('Bag playback started successfully')

                # Start thread to read output
                import threading
                def read_output():
                    for line in iter(self.bag_process.stdout.readline, b''):
                        if line:
                            self.get_logger().info(f'[bag play] {line.decode().strip()}')
                threading.Thread(target=read_output, daemon=True).start()
            except Exception as e:
                self.get_logger().error(f'Failed to start bag playback: {str(e)}')
                return False

        return True

    def bag_pause_toggle(self):
        """Toggle bag playback pause/resume"""
        if not self.bag_playing or not self.bag_process:
            self.get_logger().warn('No bag playback in progress')
            return False

        try:
            if self.bag_paused:
                # Resume playback
                self.get_logger().info('Resuming bag playback...')
                os.kill(self.bag_process.pid, signal.SIGCONT)
                self.bag_paused = False
                # Adjust start time to account for pause duration
                pause_duration = time.time() - self.bag_pause_time
                self.bag_start_real_time += pause_duration
            else:
                # Pause playback
                self.get_logger().info('Pausing bag playback...')
                os.kill(self.bag_process.pid, signal.SIGSTOP)
                self.bag_paused = True
                self.bag_pause_time = time.time()

            return True
        except Exception as e:
            self.get_logger().error(f'Failed to pause/resume bag: {str(e)}')
            return False

    def set_bag_position(self, position_ratio):
        """Set bag playback position (0.0 to 1.0)"""
        if self.bag_duration <= 0:
            return False

        target_time = position_ratio * self.bag_duration

        # If playing, restart from new position
        if self.bag_playing:
            # Stop current playback
            self.bag_play_toggle()
            time.sleep(0.1)
            # Restart from new position (현재 속도 유지)
            self.bag_play_toggle(self.bag_selected_topics, target_time, self.bag_playback_rate)
        else:
            # Just update the position
            self.bag_current_time = target_time
            self.bag_start_offset = target_time

        self.get_logger().info(f'Set bag position to {target_time}s ({position_ratio*100}%)')
        return True

    def set_bag_playback_rate(self, rate: float) -> dict:
        """ROS2 bag 재생 중 속도 변경.

        재생 중이면 /rosbag2_player/set_rate 서비스를 호출해 즉시 반영.
        정지/일시정지 상태면 self.bag_playback_rate만 갱신하여 다음 재생에 적용.

        Args:
            rate (float): 새 속도 배율 (> 0)

        Returns:
            dict: {'success': bool, 'rate': float, 'message': str}
        """
        rate = max(0.01, float(rate))

        if not self.bag_playing or not self.bag_process:
            # 재생 중이 아님 – 다음 Play 시 적용
            self.bag_playback_rate = rate
            self.get_logger().info(f'[bag set_rate] Stored rate={rate}x (not playing)')
            return {'success': True, 'rate': rate, 'message': 'Rate stored for next playback'}

        # 재생 중: /rosbag2_player/set_rate 서비스 호출
        try:
            from rosbag2_interfaces.srv import SetRate
        except ImportError:
            self.get_logger().warn(
                '[bag set_rate] rosbag2_interfaces not available; cannot change rate live'
            )
            return {
                'success': False, 'rate': rate,
                'message': 'rosbag2_interfaces not available'
            }

        client = self.create_client(SetRate, '/rosbag2_player/set_rate')
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('[bag set_rate] /rosbag2_player/set_rate service not available')
            return {
                'success': False, 'rate': rate,
                'message': '/rosbag2_player/set_rate service not available'
            }

        req = SetRate.Request()
        req.rate = rate
        future = client.call_async(req)

        # 동기 대기 (폴링, 최대 2초)
        # rclpy.spin()이 메인 스레드에서 실행 중이므로 HTTP 핸들러 스레드에서는
        # future.done()을 폴링하는 방식으로 완료를 기다린다.
        timeout = 2.0
        start = time.time()
        while not future.done() and time.time() - start < timeout:
            time.sleep(0.01)

        if not future.done():
            self.get_logger().warn(f'[bag set_rate] Service call timed out for rate={rate}')
            return {'success': False, 'rate': rate, 'message': 'Service call timed out'}

        try:
            result = future.result()
            if result is not None and result.success:
                # 속도 변경 성공 시, 타임라인 추적 기준을 현재 시점으로 재설정
                # 이전 속도로 진행된 bag 시간을 새 시작 오프셋으로 저장
                now = time.time()
                if not self.bag_paused:
                    elapsed = now - self.bag_start_real_time
                    self.bag_start_offset = self.bag_start_offset + elapsed * self.bag_playback_rate
                    self.bag_start_real_time = now
                self.bag_playback_rate = rate
                self.get_logger().info(f'[bag set_rate] Rate changed to {rate}x via service')
                return {'success': True, 'rate': rate, 'message': f'Rate set to {rate}x'}
            else:
                self.get_logger().warn(f'[bag set_rate] Service returned failure for rate={rate}')
                return {'success': False, 'rate': rate, 'message': 'Service returned failure'}
        except Exception as e:
            self.get_logger().error(f'[bag set_rate] Service call failed: {e}')
            return {'success': False, 'rate': rate, 'message': str(e)}

    def set_ros1_bag_rate(self, rate: float) -> dict:
        """ROS1 bag 재생 중 속도 변경.

        재생 중이면 Ros1BagPlayerThread.set_rate()를 호출해 즉시 반영.

        Args:
            rate (float): 새 속도 배율 (> 0)

        Returns:
            dict: {'success': bool, 'rate': float, 'message': str}
        """
        rate = max(0.01, float(rate))
        self.ros1_player_rate = rate

        thread = self.ros1_player_thread
        if thread is not None and thread.is_alive():
            thread.set_rate(rate)
            self.get_logger().info(f'[ROS1 set_rate] Rate changed to {rate}x during playback')
            return {'success': True, 'rate': rate, 'message': f'Rate set to {rate}x'}
        else:
            self.get_logger().info(f'[ROS1 set_rate] Stored rate={rate}x (not playing)')
            return {'success': True, 'rate': rate, 'message': 'Rate stored for next playback'}

    def get_bag_state(self):
        """Get current bag player state"""
        return {
            'path': self.bag_path,
            'playing': self.bag_playing,
            'paused': self.bag_paused,
            'topics': self.bag_topics,
            'selected_topics': self.bag_selected_topics,
            'duration': self.bag_duration,
            'current_time': self.bag_current_time
        }

    def playback_worker(self):
        """Worker thread for playing back data (matches C++ DataStampThread)

        개선 사항:
        - timestamps 는 Play 시작 시 한 번 복사 → 새 디렉토리 로드 후 재생 시 갱신됨
        - 인덱스(current_idx) 추적으로 매 루프 O(n) 전체 순회를 O(k) 로 단축
          (k: 이번 루프에서 실제로 발행할 스탬프 수)
        - 배속(player_speed)은 worker 내부 wall-clock으로 직접 계산
          (timer_callback 의존 제거 → KITTI/ConPR 모두 안정적 동작)
        """
        self.get_logger().info('Playback worker started')

        timestamps = []      # Play 시작 시 data_stamp 에서 복사
        current_idx = 0      # 다음 처리할 timestamps 인덱스
        was_playing = False  # 이전 루프의 재생 상태 (재시작 감지용)
        was_paused = False   # pause 상태 추적 (resume 시 기준 시간 재조정)
        _dbg_iter = 0        # 진단용 카운터

        # wall-clock 기준 타이밍 (timer_callback 불필요)
        _wall_start = 0.0    # play/resume 시점 wall time
        _proc_start = 0      # play/resume 시점 player_processed_stamp

        while self.playback_active:
            time.sleep(0.001)  # 1ms sleep

            if not self.player_playing:
                if was_playing:
                    # 방금 정지 → 다음 재생을 위해 인덱스 초기화
                    current_idx = 0
                    timestamps = []
                was_playing = False
                was_paused = False
                time.sleep(0.01)
                continue

            # ── wall-clock 기반 player_processed_stamp 갱신 ──────────────
            now = time.time()
            if self.player_seek_requested:
                # seek 발생 → 일반 wall-clock 갱신을 막아 player_processed_stamp
                # 덮어쓰기 방지.  실제 seek 처리는 아래 elif 블록에서 수행.
                # _proc_start 를 seek 목표로 미리 설정해 두면 혹시 elif 가 같은
                # 이터레이션에서 실행되지 않더라도 다음 이터레이션 정상 진행 가능.
                _wall_start = now
                _proc_start = self.player_seek_to_stamp - self.player_initial_stamp
            elif not self.player_paused:
                if was_paused:
                    # pause 에서 resume 됨 → 기준 시간 재설정 (멈춘 시간 제외)
                    _wall_start = now
                    _proc_start = self.player_processed_stamp
                    was_paused = False
                elif _wall_start > 0:
                    elapsed_ns = int((now - _wall_start) * 1e9 * self.player_speed)
                    self.player_processed_stamp = _proc_start + elapsed_ns
            else:
                if not was_paused:
                    was_paused = True
            # ─────────────────────────────────────────────────────────────

            # Play 시작 시(또는 재시작 시) timestamps 를 현재 data_stamp 로 갱신
            if not was_playing:
                timestamps = sorted(self.data_stamp.keys())
                # current_idx 를 player_timestamp 직후로 이동 (seek 후 재생 대비)
                current_idx = 0
                while current_idx < len(timestamps) and timestamps[current_idx] <= self.player_timestamp:
                    current_idx += 1
                self.player_seek_requested = False
                # wall-clock 기준 초기화
                _wall_start = now
                _proc_start = self.player_processed_stamp
                was_paused = self.player_paused
                self.get_logger().info(
                    f'[KITTI DBG] Playback started: {len(timestamps)} stamps, '
                    f'idx={current_idx}, speed={self.player_speed}, '
                    f'is_kitti={getattr(self, "player_is_kitti", False)}, '
                    f'initial={self.player_initial_stamp}, last={self.player_last_stamp}, '
                    f'player_ts={self.player_timestamp}, proc={self.player_processed_stamp}'
                )
                if len(timestamps) > 1:
                    self.get_logger().info(
                        f'[KITTI DBG] ts[0]={timestamps[0]}, ts[1]={timestamps[1]}, '
                        f'gap={timestamps[1]-timestamps[0]}ns'
                    )
            elif self.player_seek_requested:
                # 재생 중 seek ─────────────────────────────────────────────
                # player_seek_to_stamp 에서 목표 위치를 읽는다.
                # (HTTP 스레드가 player_processed_stamp 를 직접 쓰지 않으므로
                #  여기서 처음이자 유일하게 worker 가 값을 확정한다.)
                seek_stamp = self.player_seek_to_stamp
                self.player_processed_stamp = seek_stamp - self.player_initial_stamp
                self.player_timestamp = seek_stamp
                self.player_seek_requested = False
                current_idx = 0
                while current_idx < len(timestamps) and timestamps[current_idx] <= seek_stamp:
                    current_idx += 1
                _wall_start = now
                _proc_start = self.player_processed_stamp
                self.get_logger().info(
                    f'Seek done: stamp={seek_stamp}, idx={current_idx}'
                )
            was_playing = True
            _dbg_iter += 1

            # 현재 목표 스탬프 계산
            target_stamp = self.player_initial_stamp + self.player_processed_stamp

            # 진단 로그 (100ms마다 한 번)
            if _dbg_iter % 100 == 1:
                next_ts = timestamps[current_idx] if current_idx < len(timestamps) else -1
                self.get_logger().info(
                    f'[KITTI DBG] iter={_dbg_iter} proc={self.player_processed_stamp} '
                    f'target={target_stamp} next_ts={next_ts} '
                    f'diff={next_ts - target_stamp if next_ts > 0 else "N/A"} '
                    f'n_ts={len(timestamps)} idx={current_idx}'
                )

            # 인덱스를 앞으로 전진하면서 target_stamp 이하의 스탬프만 발행 (O(k))
            while current_idx < len(timestamps):
                stamp = timestamps[current_idx]
                if stamp > target_stamp:
                    break
                current_idx += 1

                if stamp <= self.player_timestamp:
                    # 이미 발행한 스탬프 (seek 복귀 시 skip)
                    continue

                data_type = self.data_stamp.get(stamp, "")

                # ── KITTI direct play ──────────────────────────────────────
                if getattr(self, 'player_is_kitti', False):
                    try:
                        frame_idx = int(data_type)
                        self._publish_kitti_frame(frame_idx, stamp)
                        # 클락 메시지 발행
                        if self.clock_pub:
                            clock_msg = Clock()
                            clock_msg.clock = Time(nanoseconds=stamp).to_msg()
                            self.clock_pub.publish(clock_msg)
                    except Exception as e:
                        self.get_logger().warn(f'KITTI frame publish error: {e}')
                    # player_timestamp는 예외 여부와 무관하게 항상 갱신
                    self.player_timestamp = stamp
                    continue  # ConPR 분기 스킵

                if data_type == "pose" and stamp in self.pose_data:
                    x, y, z = self.pose_data[stamp]
                    msg = PointStamped()
                    msg.header.stamp = Time(nanoseconds=stamp).to_msg()
                    msg.header.frame_id = 'imu_link'
                    msg.point.x = x
                    msg.point.y = y
                    msg.point.z = z
                    self.pose_pub.publish(msg)

                elif data_type == "imu" and stamp in self.imu_data:
                    imu_values = self.imu_data[stamp]
                    msg = Imu()
                    msg.header.stamp = Time(nanoseconds=stamp).to_msg()
                    msg.header.frame_id = 'imu_link'
                    # IMU data: q_x, q_y, q_z, q_w, w_x, w_y, w_z, a_x, a_y, a_z
                    msg.orientation.x = imu_values[0]
                    msg.orientation.y = imu_values[1]
                    msg.orientation.z = imu_values[2]
                    msg.orientation.w = imu_values[3]
                    msg.angular_velocity.x = imu_values[4]
                    msg.angular_velocity.y = imu_values[5]
                    msg.angular_velocity.z = imu_values[6]
                    msg.linear_acceleration.x = imu_values[7]
                    msg.linear_acceleration.y = imu_values[8]
                    msg.linear_acceleration.z = imu_values[9]
                    self.imu_pub.publish(msg)

                elif data_type == "livox":
                    livox_msg = self.load_livox_data(stamp)
                    if livox_msg and self.livox_pub:
                        self.livox_pub.publish(livox_msg)
                    else:
                        self.get_logger().warn(
                            f'Failed to load LiDAR data for stamp {stamp}',
                            throttle_duration_sec=5.0
                        )

                elif data_type == "cam":
                    cam_data = self.load_camera_data(stamp)
                    if cam_data:
                        img_msg, cam_info_msg = cam_data
                        self.cam_pub.publish(img_msg)
                        self.cam_info_pub.publish(cam_info_msg)
                    else:
                        self.get_logger().warn(
                            f'Failed to load camera data for stamp {stamp}',
                            throttle_duration_sec=5.0
                        )

                # 클락 메시지 발행
                if self.clock_pub:
                    try:
                        clock_msg = Clock()
                        clock_msg.clock = Time(nanoseconds=stamp).to_msg()
                        self.clock_pub.publish(clock_msg)
                    except Exception as e:
                        self.get_logger().warn(f'Clock publish error: {e}')

                self.player_timestamp = stamp

            # 슬라이더 위치 업데이트
            if self.player_last_stamp > self.player_initial_stamp:
                progress = (target_stamp - self.player_initial_stamp) / \
                           (self.player_last_stamp - self.player_initial_stamp)
                self.player_slider_pos = int(min(progress, 1.0) * 10000)

            # 재생 종료 체크
            if target_stamp >= self.player_last_stamp:
                if self.player_loop:
                    self.get_logger().info('Looping playback...')
                    self.player_processed_stamp = 0
                    self.player_timestamp = self.player_initial_stamp
                    current_idx = 0
                    # wall-clock 기준도 리셋 (리셋 없으면 elapsed_ns 폭주)
                    _wall_start = now
                    _proc_start = 0
                else:
                    self.get_logger().info('Playback finished')
                    self.player_playing = False
                    self.player_processed_stamp = 0
                    current_idx = 0

        self.get_logger().info('Playback worker stopped')

    def reset_player_position(self, position):
        """Reset playback position (0-10000)

        player_processed_stamp / player_timestamp 는 playback_worker 에서만 쓰도록
        race-condition 을 방지한다.  HTTP 핸들러 스레드는 player_seek_to_stamp 와
        player_seek_requested 만 설정하고 나머지는 worker 에 위임한다.
        """
        if not self.player_data_loaded:
            return

        ratio = position / 10000.0
        total_duration = self.player_last_stamp - self.player_initial_stamp
        target_stamp = int(self.player_initial_stamp + int(ratio * total_duration))

        # 슬라이더 위치는 즉시 반영 (시각적 피드백)
        self.player_slider_pos = position
        # seek 목표를 worker 에 전달 — player_processed_stamp 직접 쓰기 ×
        self.player_seek_to_stamp = target_stamp
        self.player_seek_requested = True  # 마지막에 설정 (원자성 보장)

        self.get_logger().info(f'Seek requested: pos={position} → stamp={target_stamp}')

    def save_rosbag(self):
        """Save loaded data to rosbag2 format"""
        if not self.player_data_loaded:
            self.get_logger().error('No data loaded. Please load data first.')
            return False

        try:
            bag_path = os.path.join(self.player_path, "output")
            self.save_bag_progress = "0%"
            self.get_logger().info(f'Starting rosbag conversion to: {bag_path}')

            # Create writer
            writer = rosbag2_py.SequentialWriter()

            storage_options = rosbag2_py.StorageOptions(
                uri=bag_path,
                storage_id='sqlite3'
            )

            converter_options = rosbag2_py.ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )

            writer.open(storage_options, converter_options)

            # Create topics with correct TopicMetadata format (id is required)
            from rosbag2_py import TopicMetadata

            pose_topic = TopicMetadata(
                id=0,
                name='/pose/position',
                type='geometry_msgs/msg/PointStamped',
                serialization_format='cdr'
            )
            writer.create_topic(pose_topic)

            imu_topic = TopicMetadata(
                id=1,
                name='/imu',
                type='sensor_msgs/msg/Imu',
                serialization_format='cdr'
            )
            writer.create_topic(imu_topic)

            # Create LiDAR topic if available
            topic_id = 2
            if LIVOX_AVAILABLE and len(self.livox_file_list) > 0:
                livox_topic = TopicMetadata(
                    id=topic_id,
                    name='/livox/lidar',
                    type='livox_ros_driver2/msg/CustomMsg',
                    serialization_format='cdr'
                )
                writer.create_topic(livox_topic)
                topic_id += 1

            # Create Camera topics if available
            if len(self.cam_file_list) > 0:
                cam_topic = TopicMetadata(
                    id=topic_id,
                    name='/camera/color/image',
                    type='sensor_msgs/msg/Image',
                    serialization_format='cdr'
                )
                writer.create_topic(cam_topic)
                topic_id += 1

                cam_info_topic = TopicMetadata(
                    id=topic_id,
                    name='/camera/color/camera_info',
                    type='sensor_msgs/msg/CameraInfo',
                    serialization_format='cdr'
                )
                writer.create_topic(cam_info_topic)
                topic_id += 1

            # Calculate total items for progress tracking
            livox_stamps = []
            cam_stamps = []
            if LIVOX_AVAILABLE and len(self.livox_file_list) > 0:
                livox_stamps = [stamp for stamp, dtype in self.data_stamp.items() if dtype == "livox"]
            if len(self.cam_file_list) > 0:
                cam_stamps = [stamp for stamp, dtype in self.data_stamp.items() if dtype == "cam"]

            total_items = len(self.pose_data) + len(self.imu_data) + len(livox_stamps) + len(cam_stamps)
            processed_items = 0
            last_pct = -1

            def update_progress():
                """퍼센트가 바뀔 때만 상태 업데이트 + GIL 반납 (최대 100회)"""
                nonlocal processed_items, last_pct
                processed_items += 1
                if total_items > 0:
                    pct = int(processed_items / total_items * 100)
                    if pct != last_pct:
                        self.save_bag_progress = f"{pct}%"
                        last_pct = pct
                        time.sleep(0)  # GIL 반납 → HTTP 스레드가 폴링 요청 처리 가능

            # Write pose data
            self.get_logger().info(f'Writing {len(self.pose_data)} pose messages...')
            for stamp, (x, y, z) in sorted(self.pose_data.items()):
                msg = PointStamped()
                msg.header.stamp = Time(nanoseconds=stamp).to_msg()
                msg.header.frame_id = 'imu_link'
                msg.point.x = x
                msg.point.y = y
                msg.point.z = z

                writer.write(
                    '/pose/position',
                    serialize_message(msg),
                    stamp
                )
                update_progress()

            # Write IMU data
            self.get_logger().info(f'Writing {len(self.imu_data)} IMU messages...')
            for stamp, imu_values in sorted(self.imu_data.items()):
                msg = Imu()
                msg.header.stamp = Time(nanoseconds=stamp).to_msg()
                msg.header.frame_id = 'imu_link'
                msg.orientation.x = imu_values[0]
                msg.orientation.y = imu_values[1]
                msg.orientation.z = imu_values[2]
                msg.orientation.w = imu_values[3]
                msg.angular_velocity.x = imu_values[4]
                msg.angular_velocity.y = imu_values[5]
                msg.angular_velocity.z = imu_values[6]
                msg.linear_acceleration.x = imu_values[7]
                msg.linear_acceleration.y = imu_values[8]
                msg.linear_acceleration.z = imu_values[9]

                writer.write(
                    '/imu',
                    serialize_message(msg),
                    stamp
                )
                update_progress()

            # Write LiDAR data
            if LIVOX_AVAILABLE and len(livox_stamps) > 0:
                self.get_logger().info(f'Writing {len(livox_stamps)} LiDAR messages...')
                for stamp in sorted(livox_stamps):
                    livox_msg = self.load_livox_data(stamp)
                    if livox_msg:
                        writer.write(
                            '/livox/lidar',
                            serialize_message(livox_msg),
                            stamp
                        )
                    update_progress()

            # Write Camera data
            if len(cam_stamps) > 0:
                self.get_logger().info(f'Writing {len(cam_stamps)} camera messages...')
                for stamp in sorted(cam_stamps):
                    cam_data = self.load_camera_data(stamp)
                    if cam_data:
                        img_msg, cam_info_msg = cam_data
                        writer.write(
                            '/camera/color/image',
                            serialize_message(img_msg),
                            stamp
                        )
                        writer.write(
                            '/camera/color/camera_info',
                            serialize_message(cam_info_msg),
                            stamp
                        )
                    update_progress()

            del writer
            self.save_bag_progress = None
            self.get_logger().info('Rosbag conversion complete!')
            return True

        except Exception as e:
            self.save_bag_progress = None
            self.get_logger().error(f'Failed to save rosbag: {str(e)}')
            import traceback
            traceback.print_exc()
            return False

    def start_save_rosbag(self):
        """Start save_rosbag() in a background thread so HTTP server stays responsive."""
        if self.save_bag_saving:
            self.get_logger().warn('Bag save already in progress')
            return False

        def _run():
            self.save_bag_saving = True
            self.save_bag_success = False
            try:
                self.save_bag_success = self.save_rosbag()
            finally:
                self.save_bag_saving = False
                self.save_bag_progress = None

        t = threading.Thread(target=_run, daemon=True)
        t.start()
        return True

    def file_player_start_callback(self, msg):
        if msg.data and not self.player_playing:
            self.player_play_toggle()

    def file_player_stop_callback(self, msg):
        if msg.data and self.player_playing:
            self.player_play_toggle()

    def get_player_state(self):
        return {
            'path': self.player_path,
            'playing': self.player_playing,
            'paused': self.player_paused,
            'loop': self.player_loop,
            'skip_stop': self.player_skip_stop,
            'auto_start': self.player_auto_start,
            'timestamp': self.player_timestamp,
            'slider_pos': self.player_slider_pos,
            'data_loaded': self.player_data_loaded,
            'save_bag_progress': self.save_bag_progress,
            'save_bag_saving': self.save_bag_saving,
            'save_bag_success': self.save_bag_success
        }

    def kill_slam_processes(self):
        """Kill all SLAM-related processes and bag playback"""
        try:
            # Kill bag playback process if running
            if self.bag_process and self.bag_process.poll() is None:
                self.get_logger().info(f'Terminating bag playback process PID: {self.bag_process.pid}')
                self.bag_process.terminate()
                try:
                    self.bag_process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.get_logger().info('Bag process did not terminate, killing...')
                    self.bag_process.kill()
                self.bag_process = None
                self.bag_playing = False

            # First try to terminate the subprocess gracefully
            if self.slam_process and self.slam_process.poll() is None:
                self.get_logger().info(f'Terminating SLAM process PID: {self.slam_process.pid}')
                self.slam_process.terminate()
                try:
                    self.slam_process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.get_logger().info('Process did not terminate, killing...')
                    self.slam_process.kill()
                self.slam_process = None

            # Then kill any remaining related processes
            subprocess.run(['pkill', '-9', '-f', 'LTmapping'], check=False,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(['pkill', '-9', '-f', 'rviz2'], check=False,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(['pkill', '-9', '-f', 'lt_mapper.launch.py'], check=False,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            self.get_logger().info('SLAM processes killed')
            self.slam_status = "Ready"
        except Exception as e:
            self.get_logger().error(f'Error killing processes: {str(e)}')


# File browser functions
def browse_directory(start_path="/home"):
    """Get list of directories and files in the given path"""
    try:
        entries = []
        path = PathLib(start_path)

        # Add parent directory option
        if path.parent != path:
            entries.append({
                'name': '..',
                'path': str(path.parent),
                'is_dir': True,
                'is_file': False
            })

        # List directories and files
        if path.exists() and path.is_dir():
            # First add directories
            for entry in sorted(path.iterdir()):
                if entry.is_dir():
                    entries.append({
                        'name': entry.name,
                        'path': str(entry),
                        'is_dir': True,
                        'is_file': False
                    })

            # Then add files
            for entry in sorted(path.iterdir()):
                if entry.is_file():
                    entries.append({
                        'name': entry.name,
                        'path': str(entry),
                        'is_dir': False,
                        'is_file': True
                    })

        return {
            'success': True,
            'current_path': str(path),
            'entries': entries
        }
    except Exception as e:
        return {
            'success': False,
            'error': str(e),
            'current_path': start_path,
            'entries': []
        }


class WebRequestHandler(SimpleHTTPRequestHandler):
    node = None
    web_dir = None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=WebRequestHandler.web_dir, **kwargs)

    def do_GET(self):
        parsed_path = urlparse(self.path)

        if parsed_path.path == '/api/slam/state':
            self.send_json_response(self.node.get_slam_state())
        elif parsed_path.path == '/api/localization/state':
            self.send_json_response(self.node.get_localization_state())
        elif parsed_path.path == '/api/player/state':
            self.send_json_response(self.node.get_player_state())
        elif parsed_path.path == '/api/bag/state':
            self.send_json_response(self.node.get_bag_state())
        elif parsed_path.path == '/api/bag/get_info':
            info = self.node.get_bag_info()
            self.send_json_response({
                'success': True,
                'topics': info['topics'],
                'duration': info['duration'],
                'bag_type': info.get('bag_type', 'ros2')
            })
        elif parsed_path.path == '/api/bag/ros1_play_status':
            self.send_json_response(self.node.get_ros1_playback_status())
        elif parsed_path.path == '/api/recorder/state':
            self.send_json_response(self.node.get_recorder_state())
        elif parsed_path.path == '/api/recorder/get_topics':
            topics = self.node.get_recorder_topics()
            self.send_json_response({'success': True, 'topics': topics})
        elif parsed_path.path.startswith('/api/browse'):
            # Parse query parameters
            query = parse_qs(parsed_path.query)
            path = query.get('path', ['/home'])[0]
            result = browse_directory(path)
            self.send_json_response(result)
        elif parsed_path.path == '/api/ping':
            # Simple ping endpoint for latency measurement
            self.send_json_response({'success': True, 'timestamp': time.time()})
        elif parsed_path.path == '/api/ros_domain_id':
            # Get ROS DOMAIN ID from environment
            domain_id = os.environ.get('ROS_DOMAIN_ID', '0')
            self.send_json_response({'success': True, 'domain_id': domain_id})
        elif parsed_path.path == '/api/plot/get_topics':
            # Get ROS2 topics for Plot (topic name strings, backward compatibility)
            try:
                topic_infos = self.node.get_recorder_topics()
                # get_recorder_topics() 반환값이 dict 리스트이므로 이름만 추출
                topic_names = [
                    t['name'] if isinstance(t, dict) else t
                    for t in topic_infos
                ]
                self.send_json_response({'success': True, 'topics': topic_names})
            except Exception as e:
                self.send_json_response({'success': False, 'error': str(e)})
        elif parsed_path.path == '/api/viewer/pc2_topics':
            # PC2 전용: 현재 활성화된 PointCloud2 토픽 목록
            try:
                all_topics = self.node.get_recorder_topics()
                pc2_topics = [
                    t['name'] if isinstance(t, dict) else t
                    for t in all_topics
                    if (t.get('type', '') if isinstance(t, dict) else '') in (
                        'sensor_msgs/msg/PointCloud2',
                        'sensor_msgs/PointCloud2',
                    )
                ]
                self.send_json_response({'success': True, 'topics': pc2_topics})
            except Exception as e:
                self.send_json_response({'success': False, 'error': str(e), 'topics': []})
        else:
            # Serve static files
            if parsed_path.path == '/':
                self.path = '/index.html'
            super().do_GET()

    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        data = json.loads(post_data.decode('utf-8'))

        parsed_path = urlparse(self.path)
        response = {'success': False}

        # SLAM API endpoints
        if parsed_path.path == '/api/slam/set_map1':
            self.node.set_slam_map1(data.get('path', ''))
            response = {'success': True, 'status': self.node.slam_status}
        elif parsed_path.path == '/api/slam/set_map2':
            self.node.set_slam_map2(data.get('path', ''))
            response = {'success': True, 'status': self.node.slam_status}
        elif parsed_path.path == '/api/slam/set_output':
            self.node.set_slam_output(data.get('path', ''))
            response = {'success': True, 'status': self.node.slam_status}
        elif parsed_path.path == '/api/slam/optimize':
            success = self.node.run_slam_optimization()
            response = {'success': success, 'status': self.node.slam_status}
        elif parsed_path.path == '/api/slam/start_mapping':
            success = self.node.start_slam_mapping()
            response = {'success': success, 'message': 'SLAM mapping started' if success else 'Failed to start SLAM mapping'}
        elif parsed_path.path == '/api/slam/stop_mapping':
            success = self.node.stop_slam_mapping()
            response = {'success': success, 'message': 'SLAM mapping stopped' if success else 'Failed to stop SLAM mapping'}
        elif parsed_path.path == '/api/slam/save_map':
            directory = data.get('directory', 'map')
            success, message = self.node.save_slam_map(directory)
            response = {'success': success, 'message': message}

        # Localization API endpoints
        elif parsed_path.path == '/api/localization/start_mapping':
            success = self.node.start_localization_mapping()
            response = {'success': success, 'message': 'Localization mapping started' if success else 'Failed to start Localization mapping'}
        elif parsed_path.path == '/api/localization/stop_mapping':
            success = self.node.stop_localization_mapping()
            response = {'success': success, 'message': 'Localization mapping stopped' if success else 'Failed to stop Localization mapping'}
        # Bag Player API endpoints
        elif parsed_path.path == '/api/bag/load':
            path = data.get('path', '')
            self.node.bag_path = path
            # Automatically get bag info when loading
            info = self.node.get_bag_info()
            response = {
                'success': True,
                'message': 'Bag path set',
                'path': path,
                'topics': info['topics'],
                'duration': info['duration'],
                'bag_type': info.get('bag_type', 'ros2')
            }
        elif parsed_path.path == '/api/bag/play':
            selected_topics = data.get('topics', [])
            start_offset = data.get('start_offset', None)
            rate = float(data.get('rate', 1.0))
            success = self.node.bag_play_toggle(selected_topics, start_offset, rate)
            response = {'success': success, 'playing': self.node.bag_playing}
        elif parsed_path.path == '/api/bag/pause':
            success = self.node.bag_pause_toggle()
            response = {'success': success, 'paused': self.node.bag_paused}
        elif parsed_path.path == '/api/bag/set_position':
            position = data.get('position', 0)  # 0-10000
            position_ratio = position / 10000.0
            success = self.node.set_bag_position(position_ratio)
            response = {'success': success}
        elif parsed_path.path == '/api/bag/set_rate':
            rate = float(data.get('rate', 1.0))
            bag_type = data.get('bag_type', 'ros2')  # 'ros1' or 'ros2'
            if bag_type == 'ros1':
                result = self.node.set_ros1_bag_rate(rate)
            else:
                result = self.node.set_bag_playback_rate(rate)
            response = result
        elif parsed_path.path == '/api/bag/convert_ros1':
            result = self.node.convert_ros1_bag()
            response = result
        elif parsed_path.path == '/api/bag/convert_to_ros1':
            result = self.node.convert_ros2_to_ros1_bag()
            response = result

        # ROS1 Bag Player API endpoints
        elif parsed_path.path == '/api/bag/play_ros1':
            bag_path = data.get('bag_path', self.node.bag_path)
            topics = data.get('topics', [])
            playback_rate = float(data.get('playback_rate', 1.0))
            success = self.node.start_ros1_playback(bag_path, topics, playback_rate)
            response = {'success': success, 'message': 'ROS1 playback started'}
        elif parsed_path.path == '/api/bag/pause_ros1':
            result = self.node.pause_ros1_playback()
            response = {'success': True, 'paused': result.get('paused', False)}
        elif parsed_path.path == '/api/bag/stop_ros1':
            success = self.node.stop_ros1_playback()
            response = {'success': success, 'message': 'ROS1 playback stopped'}

        # Bag Recorder API endpoints
        elif parsed_path.path == '/api/recorder/set_bag_name':
            bag_name = data.get('bag_name', '')
            success = self.node.set_recorder_bag_name(bag_name)
            response = {'success': success}
        elif parsed_path.path == '/api/recorder/record':
            topics = data.get('topics', [])
            save_as_ros1 = data.get('save_as_ros1', False)
            success = self.node.record_bag(topics, save_as_ros1=save_as_ros1)
            response = {
                'success': success,
                'recording': self.node.recorder_recording,
                'mode': self.node.recorder_mode,
            }

        # SLAM Config API endpoints
        elif parsed_path.path == '/api/slam/load_config_file':
            config_path = data.get('path', '')
            try:
                with open(config_path, 'r') as f:
                    config_data = yaml.safe_load(f)

                # Extract parameters from ROS2 yaml format
                if '/**' in config_data and 'ros__parameters' in config_data['/**']:
                    params = config_data['/**']['ros__parameters']
                    response = {'success': True, 'config': params}
                else:
                    # If not in ROS2 format, return as is
                    response = {'success': True, 'config': config_data}

                self.node.get_logger().info(f'Loaded config from: {config_path}')
            except Exception as e:
                self.node.get_logger().error(f'Failed to load config: {str(e)}')
                response = {'success': False, 'message': str(e)}

        elif parsed_path.path == '/api/slam/save_config_file':
            config_path = data.get('path', '')
            config_params = data.get('config', {})
            try:
                if RUAMEL_AVAILABLE:
                    # Use ruamel.yaml to preserve comments and formatting
                    from ruamel.yaml.comments import CommentedMap, CommentedSeq
                    from ruamel.yaml.scalarstring import DoubleQuotedScalarString

                    yaml_handler = YAML()
                    yaml_handler.preserve_quotes = True
                    yaml_handler.default_flow_style = False  # Ensure block style
                    yaml_handler.width = 1000
                    yaml_handler.indent(mapping=4, sequence=4, offset=0)

                    # Helper function to convert dict to CommentedMap recursively
                    def convert_to_commented_map(obj, original=None):
                        if isinstance(obj, dict):
                            cm = CommentedMap()
                            for key, value in obj.items():
                                orig_value = original.get(key) if isinstance(original, dict) else None
                                cm[key] = convert_to_commented_map(value, orig_value)
                            return cm
                        elif isinstance(obj, list):
                            # Convert all lists to flow style (single line with brackets)
                            # Preserve float types in list elements
                            converted_list = []
                            for i, item in enumerate(obj):
                                orig_item = original[i] if isinstance(original, list) and i < len(original) else None
                                if isinstance(orig_item, float) and isinstance(item, (int, float)):
                                    converted_list.append(float(item))
                                else:
                                    converted_list.append(convert_to_commented_map(item, orig_item))
                            cs = CommentedSeq(converted_list)
                            cs.fa.set_flow_style()
                            return cs
                        elif isinstance(obj, str):
                            # Wrap strings in double quotes
                            return DoubleQuotedScalarString(obj)
                        elif isinstance(original, float) and isinstance(obj, (int, float)):
                            # Preserve float type
                            return float(obj)
                        else:
                            return obj

                    # Read existing config file
                    with open(config_path, 'r') as f:
                        config_data = yaml_handler.load(f)

                    # Update parameters in ROS2 yaml format
                    if '/**' in config_data and 'ros__parameters' in config_data['/**']:
                        ros_params = config_data['/**']['ros__parameters']

                        # Helper function to preserve numeric types (float vs int)
                        def preserve_numeric_type(old_value, new_value):
                            # If old value was float, keep new value as float even if it's whole number
                            if isinstance(old_value, float) and isinstance(new_value, (int, float)):
                                return float(new_value)
                            # For lists, recursively preserve types
                            elif isinstance(old_value, list) and isinstance(new_value, list):
                                return [preserve_numeric_type(old_value[i] if i < len(old_value) else new_value[i], new_value[i])
                                        for i in range(len(new_value))]
                            return new_value

                        # Update all parameters
                        for key, value in config_params.items():
                            # Get old value to check its type
                            old_value = ros_params.get(key)

                            # Convert nested dictionaries to CommentedMap to preserve block style
                            if isinstance(value, dict):
                                ros_params[key] = convert_to_commented_map(value, old_value)
                            # Preserve numeric types (especially float)
                            elif old_value is not None:
                                ros_params[key] = preserve_numeric_type(old_value, value)
                            else:
                                ros_params[key] = value

                        # Format matrix parameters (9 elements = 3x3 matrix)
                        matrix_params = ['extrinsic_R', 'extrinsic_g2o_R']
                        for param in matrix_params:
                            if param in ros_params and isinstance(ros_params[param], list) and len(ros_params[param]) == 9:
                                # Create flow style list with custom formatting
                                formatted_list = CommentedSeq(ros_params[param])
                                formatted_list.fa.set_flow_style()
                                ros_params[param] = formatted_list

                        # Format vector parameters (3 elements)
                        vector_params = ['extrinsic_T', 'extrinsic_g2o_T']
                        for param in vector_params:
                            if param in ros_params and isinstance(ros_params[param], list):
                                formatted_list = CommentedSeq(ros_params[param])
                                formatted_list.fa.set_flow_style()
                                ros_params[param] = formatted_list
                    else:
                        config_data = convert_to_commented_map(config_params)

                    # Save with ruamel.yaml
                    with open(config_path, 'w') as f:
                        yaml_handler.dump(config_data, f)

                    # Post-process: Fix 3x3 matrix formatting
                    with open(config_path, 'r') as f:
                        content = f.read()

                    # Format 9-element arrays as 3x3 matrices
                    import re

                    # Find extrinsic_R and extrinsic_g2o_R patterns
                    def format_matrix(match):
                        indent = match.group(1)
                        param_name = match.group(2)
                        values = match.group(3)

                        # Parse values
                        nums = [v.strip() for v in values.split(',')]
                        if len(nums) == 9:
                            # Format as 3x3 matrix
                            line1 = f"{indent}{param_name}: [{nums[0]}, {nums[1]}, {nums[2]},"
                            line2 = f"{indent}            {nums[3]}, {nums[4]}, {nums[5]},"
                            line3 = f"{indent}            {nums[6]}, {nums[7]}, {nums[8]}]"
                            return f"{line1}\n{line2}\n{line3}"
                        return match.group(0)

                    # Replace 9-element arrays
                    content = re.sub(
                        r'^(\s*)(extrinsic_R|extrinsic_g2o_R):\s*\[([\d\.,\s\-]+)\]',
                        format_matrix,
                        content,
                        flags=re.MULTILINE
                    )

                    # Write back
                    with open(config_path, 'w') as f:
                        f.write(content)

                else:
                    # Fallback to regular yaml (no comment preservation)
                    with open(config_path, 'r') as f:
                        config_data = yaml.safe_load(f)

                    if '/**' in config_data and 'ros__parameters' in config_data['/**']:
                        config_data['/**']['ros__parameters'] = config_params
                    else:
                        config_data = config_params

                    class IndentDumper(yaml.Dumper):
                        def increase_indent(self, flow=False, indentless=False):
                            return super(IndentDumper, self).increase_indent(flow, False)

                    with open(config_path, 'w') as f:
                        yaml.dump(
                            config_data,
                            f,
                            Dumper=IndentDumper,
                            default_flow_style=False,
                            sort_keys=False,
                            indent=4,
                            width=1000,
                            allow_unicode=True
                        )

                self.node.get_logger().info(f'Saved config to: {config_path}')
                response = {'success': True, 'message': 'Config saved successfully'}
            except Exception as e:
                self.node.get_logger().error(f'Failed to save config: {str(e)}')
                import traceback
                traceback.print_exc()
                response = {'success': False, 'message': str(e)}

        # File Player API endpoints
        elif parsed_path.path == '/api/player/scan_kitti':
            # KITTI 디렉토리 탐색
            # body: { "path": "/path/to/2011_09_30" }
            path = data.get('path', '')
            if not path:
                response = {'success': False, 'error': 'Missing path'}
            elif not os.path.isdir(path):
                response = {'success': False, 'error': f'Directory not found: {path}'}
            else:
                response = self.node.scan_kitti_directory(path)

        elif parsed_path.path == '/api/player/convert_kitti':
            # KITTI → ROS2 bag 변환
            # body: { "base_dir": "...", "calib_dir": "...", "data_path": "...", "drive_name": "..." }
            base_dir   = data.get('base_dir', '')
            calib_dir  = data.get('calib_dir', '')
            data_path  = data.get('data_path', '')
            drive_name = data.get('drive_name', '')
            if not all([base_dir, calib_dir, data_path, drive_name]):
                response = {'success': False, 'error': 'Missing required fields: base_dir, calib_dir, data_path, drive_name'}
            else:
                response = self.node.start_kitti_conversion(base_dir, calib_dir, data_path, drive_name)

        elif parsed_path.path == '/api/player/load_data':
            path = data.get('path', '')
            success = self.node.load_player_data(path)
            response = {'success': success, 'message': 'Data loaded' if success else 'Failed to load data'}
        elif parsed_path.path == '/api/player/play':
            success = self.node.player_play_toggle()
            response = {'success': success, 'playing': self.node.player_playing}
        elif parsed_path.path == '/api/player/pause':
            success = self.node.player_pause_toggle()
            response = {'success': success, 'paused': self.node.player_paused}
        elif parsed_path.path == '/api/player/save_bag':
            started = self.node.start_save_rosbag()
            response = {'success': started, 'message': 'Save started' if started else 'Save already in progress'}
        elif parsed_path.path == '/api/player/set_loop':
            self.node.player_loop = data.get('loop', False)
            response = {'success': True}
        elif parsed_path.path == '/api/player/set_skip_stop':
            self.node.player_skip_stop = data.get('skip_stop', False)
            response = {'success': True}
        elif parsed_path.path == '/api/player/set_auto_start':
            self.node.player_auto_start = data.get('auto_start', False)
            response = {'success': True}
        elif parsed_path.path == '/api/player/set_slider':
            position = data.get('position', 0)
            self.node.reset_player_position(position)
            response = {'success': True}

        self.send_json_response(response)

    def send_json_response(self, data):
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode('utf-8'))

    def log_message(self, format, *args):
        pass


def get_local_ip():
    """Get the local IP address"""
    try:
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "localhost"


def run_web_server(node, port=8080):
    WebRequestHandler.node = node

    # Determine web directory
    web_dir = None
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory('ros2_autonav_webui')
        web_dir = os.path.join(share_dir, 'web')
        node.get_logger().info(f'Using web directory: {web_dir}')
    except Exception as e:
        # Fallback for development
        web_dir = os.path.join(os.path.dirname(__file__), '..', 'web')
        web_dir = os.path.abspath(web_dir)
        node.get_logger().info(f'Using fallback web directory: {web_dir}')

    # Check if web directory exists
    if not os.path.exists(web_dir):
        node.get_logger().error(f'Web directory not found: {web_dir}')
        return

    # Check if index.html exists
    index_path = os.path.join(web_dir, 'index.html')
    if not os.path.exists(index_path):
        node.get_logger().error(f'index.html not found: {index_path}')
        return

    WebRequestHandler.web_dir = web_dir
    global _web_server
    _web_server = ThreadedHTTPServer(('0.0.0.0', port), WebRequestHandler)

    # Get local IP for network access
    local_ip = get_local_ip()

    node.get_logger().info(f'======================================')
    node.get_logger().info(f'Web server started on port {port}')
    node.get_logger().info(f'Local access:   http://localhost:{port}')
    node.get_logger().info(f'Network access: http://{local_ip}:{port}')
    node.get_logger().info(f'======================================')

    try:
        _web_server.serve_forever()
    except Exception as e:
        node.get_logger().error(f'Web server error: {str(e)}')
    finally:
        _web_server.server_close()


def signal_handler(signum, frame):
    """Handle SIGTERM and SIGINT for graceful shutdown"""
    global _web_server, _ros_node
    
    signal_name = signal.Signals(signum).name
    logger_msg = f'Received {signal_name}, shutting down gracefully...'
    if _ros_node:
        _ros_node.get_logger().info(logger_msg)
    else:
        print(logger_msg)
    
    # Shutdown web server
    if _web_server:
        shutdown_msg = 'Shutting down web server...'
        if _ros_node:
            _ros_node.get_logger().info(shutdown_msg)
        else:
            print(shutdown_msg)
        _web_server.shutdown()
    
    # Clean up ROS node
    if _ros_node:
        _ros_node.get_logger().info('Cleaning up processes...')
        _ros_node.kill_slam_processes()
        _ros_node.kill_localization_processes()
        _ros_node.destroy_node()
        rclpy.shutdown()
    
    # Exit
    import sys
    sys.exit(0)

def main(args=None):
    global _ros_node
    
    rclpy.init(args=args)

    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    _ros_node = WebGUINode()

    # Start web server in a separate thread
    web_thread = threading.Thread(target=run_web_server, args=(_ros_node, 8080), daemon=True)
    web_thread.start()

    local_ip = get_local_ip()
    _ros_node.get_logger().info(f'Web GUI is running with full ROS2 integration.')
    _ros_node.get_logger().info(f'Open http://localhost:8080 or http://{local_ip}:8080 in your browser.')

    try:
        rclpy.spin(_ros_node)
    except KeyboardInterrupt:
        _ros_node.get_logger().info('Keyboard interrupt received')
    finally:
        _ros_node.get_logger().info('Cleaning up...')
        _ros_node.kill_slam_processes()
        _ros_node.kill_localization_processes()
        _ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
