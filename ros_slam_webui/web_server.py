#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, Imu, CameraInfo, LaserScan, NavSatFix, PointCloud2, PointField
from geometry_msgs.msg import PointStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
import cv2
import struct
import glob
import queue
import threading
import asyncio
import json
import os
import time
import socketserver
from http.server import HTTPServer, SimpleHTTPRequestHandler

try:
    import psutil as _psutil
except ImportError:
    _psutil = None


class ThreadedHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
    """HTTP 서버: 요청마다 새 스레드로 처리해 저장 작업 중 폴링 응답 지연 제거"""
    daemon_threads = True
    allow_reuse_address = True


# 센서 워커가 한 프레임을 처리한 뒤 고정 sleep으로 CPU를 양보하는 시간(초).
# 비율 기반(elapsed × ratio)은 무거운 작업(카메라×4, Ouster 등 80-120ms)에서
# sleep이 40-60ms까지 늘어나 frame period(100ms)를 초과 → drop-frame 유발.
# 3ms 고정으로: 무거운 작업(80ms + 3ms = 83ms) < 100ms → drop 없음,
# 동시에 HTTP 핸들러가 3ms 창을 통해 GIL·CPU 획득 → 레이턴시 균일.
_POST_PUBLISH_YIELD_S = 0.003


class _SensorPublishWorker:
    """중량 센서(LiDAR/Radar) 파일 I/O + publish 전담 백그라운드 스레드.

    참고: file_player_mulran ROSThread.cpp OusterThread / RadarpolarThread 패턴.

    - 재생 워커(playback_worker)가 파일 읽기에 블록되지 않도록 분리.
    - maxsize=1 의 bounded queue 로 "drop-frame" backpressure 구현.
      이전 프레임 처리가 끝나지 않았으면 새 프레임을 DROP 해 타이밍 누적 방지.
    """

    def __init__(self):
        self._queue: queue.Queue = queue.Queue(maxsize=1)
        self._active = True
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name='sensor-pub-worker')
        self._thread.start()

    # ── public ──────────────────────────────────────────────────────────────

    def push(self, fn, *args) -> bool:
        """비블록 push.  큐가 가득 차면 프레임을 DROP하고 False 반환."""
        try:
            self._queue.put_nowait((fn, args))
            return True
        except queue.Full:
            return False

    def stop(self, timeout: float = 1.0):
        """백그라운드 스레드 종료 (데이터셋 전환 시 호출)."""
        self._active = False
        try:
            self._queue.put_nowait(None)   # sentinel
        except queue.Full:
            pass
        self._thread.join(timeout=timeout)

    def clear(self):
        """큐에 적재된 미처리 항목을 모두 버린다.

        정지/일시정지 직후 워커가 마지막 프레임을 publish해 발생하는
        레이턴시 스파이크를 방지한다. 현재 처리 중인 항목은 중단할 수 없다.
        """
        try:
            while True:
                self._queue.get_nowait()
        except queue.Empty:
            pass

    # ── internal ─────────────────────────────────────────────────────────────

    def _run(self):
        while self._active:
            try:
                item = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if item is None:
                break
            fn, args = item
            try:
                t0 = time.monotonic()
                fn(*args)
                elapsed = time.monotonic() - t0
                # 작업 후 고정 3ms sleep으로 GIL·CPU 양보.
                # 비율 기반(elapsed×0.5)은 카메라×4(~80ms) 처리 시 40ms sleep →
                # 합계 120ms > 10Hz 주기(100ms) → drop-frame 유발.
                # 3ms 고정: 80ms 작업도 83ms < 100ms → drop 없음 + HTTP 핸들러 응답 가능.
                if elapsed > 0.002:  # 2ms 미만 경량 작업은 sleep 불필요
                    time.sleep(_POST_PUBLISH_YIELD_S)
            except Exception:
                pass
from urllib.parse import parse_qs, urlparse
import subprocess
import signal
import math
import yaml
from pathlib import Path as PathLib
rosbag2_py = None
ROSBAG2_AVAILABLE = False

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

# Try to import pose_graph_optimization service (ROS1)
try:
    from fast_lio.srv import SaveMap
    SAVEMAP_AVAILABLE = True
except ImportError:
    try:
        from pose_graph_optimization.srv import SaveMap
        SAVEMAP_AVAILABLE = True
    except ImportError:
        SAVEMAP_AVAILABLE = False
        print("Warning: SaveMap service not available. Map saving will be disabled.")

try:
    from std_srvs.srv import Trigger as RosTrigger
    TRIGGER_AVAILABLE = True
except ImportError:
    TRIGGER_AVAILABLE = False

# Global variables for signal handling
_web_server = None
_ros_node = None
_web_server_thread = None


def _format_port_in_use_error(port, param_name):
    """Build a helpful log message when a TCP port is already bound."""
    msg = f'Port {port} is already in use.'
    try:
        import subprocess
        out = subprocess.check_output(
            ['ss', '-ltnp'], stderr=subprocess.DEVNULL, text=True, timeout=1.0
        )
        for line in out.splitlines():
            if f':{port}' in line:
                msg += f' ({line.strip()})'
                break
    except Exception:
        pass
    msg += (
        f' Stop the previous ros_slam_webui_node'
        f' (e.g. rosnode kill /ros_slam_webui_node)'
        f' or change ~{param_name} in launch.'
    )
    return msg

# File Player PointCloud2 토픽 (create_publisher 이름과 반드시 동일 — API·UI 동기화의 단일 출처)
KITTI_FILE_PLAYER_PC2_TOPIC = '/kitti/velo/pointcloud'
# KITTI 카메라 ID → (이미지 디렉토리명, image 토픽, camera_info 토픽, 인코딩)
_KITTI_CAM_ID_MAP = {
    '00': ('image_00', '/kitti/camera_gray_left/image_raw',   '/kitti/camera_gray_left/camera_info',   'mono8'),
    '01': ('image_01', '/kitti/camera_gray_right/image_raw',  '/kitti/camera_gray_right/camera_info',  'mono8'),
    '02': ('image_02', '/kitti/camera_color_left/image_raw',  '/kitti/camera_color_left/camera_info',  'bgr8'),
    '03': ('image_03', '/kitti/camera_color_right/image_raw', '/kitti/camera_color_right/camera_info', 'bgr8'),
}
KAIST_FILE_PLAYER_PC2_TOPICS = ['/ns2/velodyne_points', '/ns1/velodyne_points']
MULRAN_FILE_PLAYER_PC2_TOPIC = '/os1_points'
# MulRan /clock: ROSThread 기준 10ms 이상 간격 — direct play에서 과도한 publish 방지
_MULRAN_CLOCK_MIN_INTERVAL_NS = 10_000_000


def _patch_rosbag2_tf_static_qos(output_dir: str, logger) -> None:
    """ROS1 환경에서는 ROS2 bag QoS 패치가 불필요하므로 no-op."""
    return


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
        self._loop = False
        self._seek_requested = False
        self._seek_to_sec = 0.0
        self._play_event = threading.Event()
        self._play_event.set()  # 기본적으로 재생 상태

        # 상태 추적
        self._status = 'stopped'   # 'playing' | 'paused' | 'stopped'
        self._elapsed_sec = 0.0
        self._total_sec = 0.0
        self._lock = threading.Lock()
        self._timing_reset_requested = False
        self._seek_pause_restore = False

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
            self._timing_reset_requested = True

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
            self._timing_reset_requested = True

    def set_loop(self, loop: bool):
        """루프 재생 여부 설정.

        Args:
            loop (bool): True이면 재생 완료 후 처음부터 반복.
        """
        self._loop = loop

    def set_seek(self, time_sec: float):
        """재생 위치 이동 (seek). 재생 중/일시정지 중 호출 가능.

        Args:
            time_sec (float): 이동할 시간(초). 0 이상 total_sec 이하.
        """
        with self._lock:
            clamped = max(0.0, float(time_sec))
            if self._total_sec > 0.0:
                clamped = min(clamped, self._total_sec)
            self._seek_to_sec = clamped
            self._seek_requested = True
            self._timing_reset_requested = True
            self._seek_pause_restore = (self._status == 'paused')
        self._play_event.set()  # 일시정지 중이면 block 해제

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

        ROS1 tf 패키지의 tf/tfMessage는 ROS2에 없으므로 tf2_msgs/msg/TFMessage로 매핑.

        Args:
            ros1_type_str (str): 예) 'sensor_msgs/msg/Image' 또는 'sensor_msgs/Image'

        Returns:
            type: 성공 시 메시지 클래스, 실패 시 None
        """
        import importlib
        # ROS1 tf/tfMessage → ROS2 tf2_msgs/msg/TFMessage (tf 패키지는 ROS2에 없음)
        if ros1_type_str in ('tf/tfMessage', 'tf/msg/tfMessage'):
            return TFMessage
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

    def _get_or_create_publisher(self, topic_name, msg_cls):
        """토픽별 ROS1 publisher를 캐시해서 반환 (없으면 생성).

        Args:
            topic_name (str): publish할 토픽 이름
            msg_cls (type): 메시지 클래스

        Returns:
            rospy.Publisher | None
        """
        if topic_name in self._publishers:
            return self._publishers[topic_name]

        try:
            latch = (topic_name == '/tf_static')
            q_size = 1 if topic_name in ('/tf_static',) else 10
            pub = rospy.Publisher(topic_name, msg_cls, queue_size=q_size, latch=latch)
            self._publishers[topic_name] = pub
            rospy.loginfo(f'[Ros1BagPlayer] Created publisher: {topic_name}')
            return pub
        except Exception as e:
            rospy.logerr(
                f'[Ros1BagPlayer] Failed to create publisher for {topic_name}: {e}'
            )
            return None

    def _destroy_publishers(self):
        """생성한 모든 publisher 정리"""
        for topic_name, pub in self._publishers.items():
            try:
                pub.unregister()
            except Exception:
                pass
        self._publishers.clear()

    # ------------------------------------------------------------------
    # 메인 실행 루프
    # ------------------------------------------------------------------
    def run(self):
        """rosbag 모듈로 ROS1 .bag 순차 읽기 → rospy.Publisher로 publish (Phase 3 ROS1)."""
        try:
            import rosbag
        except ImportError as e:
            rospy.logerr(f'[Ros1BagPlayer] rosbag not available: {e}')
            with self._lock:
                self._status = 'stopped'
            return

        with self._lock:
            self._status = 'playing'

        try:
            # 메타 정보는 별도 핸들에서 1회만 읽는다.
            with rosbag.Bag(self._bag_path, 'r') as bag_meta:
                start_time_sec = bag_meta.get_start_time()
                end_time_sec = bag_meta.get_end_time()
                total_sec = end_time_sec - start_time_sec
            with self._lock:
                self._total_sec = total_sec

            PREFETCH_QUEUE_SIZE = 256
            SENTINEL_SEEK = ('__SEEK__', None, None)
            SENTINEL_END = ('__END__', None, None)
            start_offset_sec = 0.0

            while True:
                with self._lock:
                    start_offset_sec = max(0.0, min(start_offset_sec, self._total_sec))
                    self._elapsed_sec = start_offset_sec
                    self._timing_reset_requested = True

                selected = list(self._topics) if self._topics else None

                # seek 지점부터 재시작할 수 있도록 rosbag 시작 시간 지정
                start_time_obj = None
                if start_offset_sec > 0.0:
                    start_time_obj = rospy.Time.from_sec(start_time_sec + start_offset_sec)

                prefetch_queue = queue.Queue(maxsize=PREFETCH_QUEUE_SIZE)
                reader_cancel = threading.Event()

                def _push_sentinel(sentinel):
                    placed = False
                    while not placed:
                        try:
                            prefetch_queue.put(sentinel, timeout=0.1)
                            placed = True
                        except queue.Full:
                            if self._stop_flag or reader_cancel.is_set():
                                return

                with rosbag.Bag(self._bag_path, 'r') as bag_reader:
                    def _reader_task():
                        try:
                            msg_iter = bag_reader.read_messages(topics=selected, start_time=start_time_obj)
                            for topic, msg, t in msg_iter:
                                if self._stop_flag or reader_cancel.is_set():
                                    return
                                if self._seek_requested:
                                    _push_sentinel(SENTINEL_SEEK)
                                    return
                                placed = False
                                while not placed:
                                    try:
                                        prefetch_queue.put((topic, msg, t), timeout=0.1)
                                        placed = True
                                    except queue.Full:
                                        if self._stop_flag or reader_cancel.is_set():
                                            return
                                        if self._seek_requested:
                                            _push_sentinel(SENTINEL_SEEK)
                                            return
                        except Exception as e:
                            rospy.logwarn(f'[Ros1BagPlayer] reader task error: {e}')
                        finally:
                            _push_sentinel(SENTINEL_END)

                    reader_thread = threading.Thread(target=_reader_task, daemon=True)
                    reader_thread.start()

                    wall_anchor = time.monotonic()
                    bag_elapsed_at_anchor = start_offset_sec
                    seek_break = False
                    ended = False

                    while True:
                        if self._stop_flag:
                            break

                        try:
                            item = prefetch_queue.get(timeout=0.1)
                        except queue.Empty:
                            if self._seek_requested:
                                seek_break = True
                                break
                            continue

                        if item == SENTINEL_END:
                            ended = True
                            break

                        if item == SENTINEL_SEEK:
                            seek_break = True
                            break

                        topic, msg, t = item
                        msg_elapsed = t.to_sec() - start_time_sec
                        with self._lock:
                            self._elapsed_sec = msg_elapsed

                        # wall-clock 기반 publish 시점 대기.
                        while True:
                            if self._stop_flag:
                                break

                            if self._seek_requested:
                                seek_break = True
                                break

                            if not self._play_event.is_set():
                                # pause 중에는 elapsed 진행을 정지하고, resume 시 anchor 재설정.
                                while (not self._play_event.is_set()) and (not self._stop_flag):
                                    if self._seek_requested:
                                        seek_break = True
                                        break
                                    time.sleep(0.02)
                                if self._stop_flag or seek_break:
                                    break
                                wall_anchor = time.monotonic()
                                bag_elapsed_at_anchor = msg_elapsed
                                with self._lock:
                                    self._timing_reset_requested = False
                                continue

                            with self._lock:
                                if self._timing_reset_requested:
                                    wall_anchor = time.monotonic()
                                    bag_elapsed_at_anchor = msg_elapsed
                                    self._timing_reset_requested = False
                                rate = self._playback_rate

                            target_wall = wall_anchor + max(
                                0.0, (msg_elapsed - bag_elapsed_at_anchor) / max(rate, 0.01)
                            )
                            now = time.monotonic()
                            wait_sec = target_wall - now
                            if wait_sec <= 0.0:
                                break
                            time.sleep(min(wait_sec, 0.02))

                        if self._stop_flag or seek_break:
                            break

                        # publisher 생성 및 publish
                        pub = self._get_or_create_publisher(topic, type(msg))
                        if pub is not None:
                            try:
                                pub.publish(msg)
                            except Exception as e:
                                rospy.logdebug(
                                    f'[Ros1BagPlayer] Publish error on {topic}: {e}'
                                )

                    reader_cancel.set()
                    reader_thread.join(timeout=5.0)
                    if reader_thread.is_alive():
                        rospy.logwarn(
                            '[Ros1BagPlayer] reader thread did not exit cleanly; '
                            'stopping playback to avoid bag read corruption.'
                        )
                        self._stop_flag = True

                if self._stop_flag:
                    break

                if seek_break:
                    with self._lock:
                        seek_target = max(0.0, min(self._seek_to_sec, self._total_sec))
                        self._elapsed_sec = seek_target
                        self._seek_requested = False
                        self._timing_reset_requested = True
                        restore_pause = self._seek_pause_restore
                        self._seek_pause_restore = False
                    start_offset_sec = seek_target
                    if restore_pause:
                        self._play_event.clear()
                    continue

                if not ended or not self._loop:
                    break

                start_offset_sec = 0.0
                with self._lock:
                    self._elapsed_sec = 0.0
                    self._timing_reset_requested = True
                rospy.loginfo('[Ros1BagPlayer] Looping playback.')

        except Exception as e:
            rospy.logerr(f'[Ros1BagPlayer] Fatal error during playback: {e}')
            import traceback
            traceback.print_exc()
        finally:
            self._destroy_publishers()
            with self._lock:
                self._status = 'stopped'
            rospy.loginfo('[Ros1BagPlayer] Playback finished.')

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

    def _convert_pointcloud2_fast(self, ros1_msg) -> PointCloud2:
        """PointCloud2 전용 고속 변환 (재귀 루프 생략)."""
        try:
            from std_msgs.msg import Header
            msg = PointCloud2()
            h = ros1_msg.header
            s = h.stamp
            msg.header = Header(
                stamp=rospy.Time(getattr(s, 'sec', 0), getattr(s, 'nsec', getattr(s, 'nanosec', 0))),
                frame_id=str(h.frame_id)
            )
            msg.height = int(ros1_msg.height)
            msg.width = int(ros1_msg.width)
            msg.is_dense = bool(ros1_msg.is_dense)
            msg.is_bigendian = bool(ros1_msg.is_bigendian)
            msg.point_step = int(ros1_msg.point_step)
            msg.row_step = int(ros1_msg.row_step)
            if hasattr(ros1_msg.data, 'tobytes'):
                msg.data = ros1_msg.data.tobytes()
            elif isinstance(ros1_msg.data, (bytes, bytearray)):
                msg.data = bytes(ros1_msg.data)
            else:
                msg.data = bytes(ros1_msg.data)
            for f in ros1_msg.fields:
                pf = PointField()
                pf.name = str(f.name)
                pf.offset = int(f.offset)
                pf.datatype = int(f.datatype)
                pf.count = int(f.count)
                msg.fields.append(pf)
            return msg
        except Exception:
            return None

    def _convert_image_fast(self, ros1_msg) -> Image:
        """Image 전용 고속 변환 (재귀 루프 생략)."""
        try:
            from std_msgs.msg import Header
            msg = Image()
            h = ros1_msg.header
            s = h.stamp
            msg.header = Header(
                stamp=rospy.Time(getattr(s, 'sec', 0), getattr(s, 'nsec', getattr(s, 'nanosec', 0))),
                frame_id=str(h.frame_id)
            )
            msg.height = int(ros1_msg.height)
            msg.width = int(ros1_msg.width)
            msg.encoding = str(ros1_msg.encoding)
            msg.is_bigendian = bool(ros1_msg.is_bigendian)
            msg.step = int(ros1_msg.step)
            if hasattr(ros1_msg.data, 'tobytes'):
                msg.data = ros1_msg.data.tobytes()
            elif isinstance(ros1_msg.data, (bytes, bytearray)):
                msg.data = bytes(ros1_msg.data)
            else:
                msg.data = bytes(ros1_msg.data)
            return msg
        except Exception:
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

        # 대용량 메시지 고속 경로 (PointCloud2, Image)
        if ros2_cls is PointCloud2:
            return self._convert_pointcloud2_fast(ros1_msg)
        if ros2_cls is Image:
            return self._convert_image_fast(ros1_msg)

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
            type: 성공 시 메시지 클래스, 실패 시 None
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
        """rospy.Subscriber로 메시지 수신 → rosbag.Bag('w')으로 .bag 기록 (Phase 3 ROS1)."""
        try:
            import rosbag
        except ImportError as e:
            rospy.logerr(f'[Ros1BagRecorder] rosbag not available: {e}')
            with self._lock:
                self._status = 'stopped'
            return

        self._start_time = time.time()
        bag_lock = threading.Lock()
        written_count = 0
        subs = []

        def _resolve_msg_class(ros_type_str):
            """ROS1/ROS2 타입명으로 메시지 클래스 동적 import."""
            import importlib
            parts = ros_type_str.split('/')
            if len(parts) == 2:
                pkg, cls_name = parts[0], parts[1]
            elif len(parts) == 3 and parts[1] == 'msg':
                pkg, cls_name = parts[0], parts[2]
            else:
                return None
            try:
                mod = importlib.import_module(f'{pkg}.msg')
                return getattr(mod, cls_name, None)
            except Exception:
                return None

        try:
            with rosbag.Bag(self._output_path, 'w') as bag:

                def make_callback(topic_name):
                    def callback(msg):
                        if self._stop_flag:
                            return
                        with bag_lock:
                            try:
                                bag.write(topic_name, msg)
                                nonlocal written_count
                                written_count += 1
                            except Exception as e:
                                rospy.logwarn(
                                    f'[Ros1BagRecorder] Write error on {topic_name}: {e}'
                                )
                    return callback

                for topic_name, ros_type in self._topic_type_map.items():
                    msg_cls = _resolve_msg_class(ros_type)
                    if msg_cls is None:
                        rospy.logwarn(
                            f'[Ros1BagRecorder] Cannot import {ros_type}, skipping {topic_name}'
                        )
                        continue
                    try:
                        sub = rospy.Subscriber(
                            topic_name, msg_cls,
                            make_callback(topic_name),
                            queue_size=100
                        )
                        subs.append(sub)
                        rospy.loginfo(f'[Ros1BagRecorder] Subscribed: {topic_name}')
                    except Exception as e:
                        rospy.logwarn(
                            f'[Ros1BagRecorder] Failed to subscribe to {topic_name}: {e}'
                        )

                rospy.loginfo(
                    f'[Ros1BagRecorder] Recording started → {self._output_path} '
                    f'({len(subs)} topics)'
                )

                last_log_time = time.time()
                while not self._stop_flag:
                    time.sleep(0.05)
                    now = time.time()
                    if now - last_log_time >= 5.0:
                        rospy.loginfo(
                            f'[Ros1BagRecorder] Written {written_count} messages so far...'
                        )
                        last_log_time = now

        except Exception as e:
            rospy.logerr(f'[Ros1BagRecorder] Fatal error: {e}')
            import traceback
            traceback.print_exc()
        finally:
            for sub in subs:
                try:
                    sub.unregister()
                except Exception:
                    pass
            with self._lock:
                self._status = 'stopped'
            rospy.loginfo(
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

    MAX_POINTS   = 30_000   # 일반 PC2 다운샘플링 상한
    CLOUD_REGISTERED_MAX_POINTS = 12_000
    THROTTLE_SEC = 0.05     # 최대 20Hz (50 ms) — binary 전송 ~600KB이므로 충분
    # /cloud_registered 전용: 브라우저 map_accumulator가 0.3m 복셀을 하므로
    # 서버는 가벼운 step만 적용 (np.unique 복셀은 CPU를 수백 ms 점유해 analytics max time 악화)
    CLOUD_REGISTERED_THROTTLE_SEC = 0.1  # 10Hz — 누적 맵용이라 20Hz 불필요

    # PointCloud2 field datatype → numpy dtype 매핑
    _DTYPE = {
        1: np.int8,   2: np.uint8,
        3: np.int16,  4: np.uint16,
        5: np.int32,  6: np.uint32,
        7: np.float32, 8: np.float64,
    } if NUMPY_AVAILABLE else {}

    # Image WebSocket 스트리밍 설정
    IMG_THROTTLE_SEC = 0.1     # 10Hz — stereo 10Hz에 맞추고 과부하 방지 (30Hz → 10Hz)
    IMG_JPEG_QUALITY = 75      # JPEG 품질 (80 → 75, 화질 유지하며 전송량 절감)
    IMG_MAX_DIM      = 800     # 최대 단변 길이(픽셀): 초과 시 비율 유지 리사이즈

    # Path(nav_msgs/Path) WebSocket 스트리밍 설정
    #   fast_lio /path 는 매 스캔(~20Hz)마다 "누적된 전체 경로"를 재발행하므로
    #   메시지가 시간에 따라 선형으로 커진다. rospy 가 매 메시지를 통째로
    #   역직렬화하면 GIL 을 오래 점유해 /Odometry·/ouster/points 스핀이 burst 로
    #   지연된다. 아래 설정으로 (1) 수신 rate 제한, (2) 역직렬화 없이 raw 버퍼에서
    #   좌표만 벡터 추출, (3) pose 상한 을 적용해 부하를 상수화한다.
    PATH_THROTTLE_SEC = 0.2    # 5Hz — 경로 시각화에 충분, 역직렬화/전송 부하 1/4 감소
    PATH_MAX_POSES    = 8000   # pose 상한 (초과 시 stride 다운샘플링)
    PATH_BUFF_SIZE    = 8 * 1024 * 1024  # AnyMsg 수신 버퍼(누적 경로 대비 여유)

    def __init__(self, ros_node, port: int = 8881):
        self._node = ros_node
        self._port = port
        self._loop: asyncio.AbstractEventLoop = None
        self._lock = threading.Lock()
        # ── PointCloud2 전용 ───────────────────────────────────────────────────
        # topic_name → set[websocket]
        self._clients: dict = {}
        # topic_name → rclpy Subscription
        self._subs: dict = {}
        # topic_name → 마지막 전송 단조시각 (throttle)
        self._last_sent: dict = {}
        # ── Livox CustomMsg (PC2와 동일한 binary 포맷으로 스트리밍) ─────────────
        self._livox_clients: dict = {}
        self._livox_subs: dict = {}
        self._livox_last_sent: dict = {}
        # ── 범용 Plot 토픽 (throttle 없이 원래 주기로 전송) ────────────────────
        # topic_name → { ws: set[field_path, ...] }
        self._plot_clients: dict = {}
        # topic_name → rclpy Subscription
        self._plot_subs: dict = {}
        # ── 전체 연결 클라이언트 (broadcast용) ─────────────────────────────────
        self._all_clients: set = set()
        # ── Image 전용 (JPEG 바이너리 스트리밍) ────────────────────────────────
        # topic_name → set[websocket]
        self._img_clients: dict = {}
        # topic_name → rclpy Subscription
        self._img_subs: dict = {}
        # topic_name → 마지막 전송 단조시각 (throttle)
        self._img_last_sent: dict = {}
        # CvBridge 인스턴스 (Image → OpenCV 변환)
        self._cv_bridge = CvBridge()
        # ── Backpressure 제어: in-flight 브로드캐스트 플래그 ────────────────────
        # 이전 ws.send()가 완료되기 전에 새 태스크가 asyncio 큐에 쌓이는 것을 방지.
        # 브라우저가 느릴 때 asyncio 태스크 누적 → latency 기하급수적 증가를 막는다.
        self._pc2_sending: dict = {}    # topic_name → bool
        self._img_sending: dict = {}    # topic_name → bool
        self._livox_sending: dict = {}  # topic_name → bool
        # ── Path (nav_msgs/Path) 전용 ─────────────────────────────────────────────
        self._path_clients: dict = {}    # topic → set[websocket]
        self._path_subs: dict = {}       # topic → rclpy Subscription
        self._path_last_sent: dict = {}  # topic → float (단조시각)
        self._path_sending: dict = {}    # topic → bool (backpressure)
        # ── Latched (TRANSIENT_LOCAL) 토픽 전용 ─────────────────────────────────
        self._latched_subs: dict = {}        # topic → rclpy Subscription
        self._latched_clients: dict = {}     # topic → set[websocket]
        self._latched_cache: dict = {}       # topic → bytes  (마지막 binary payload)
        self._latched_meta_cache: dict = {}  # topic → str    (마지막 JSON meta)

    # ── 공개 API ─────────────────────────────────────────────────────────────

    def start(self):
        """별도 daemon 스레드에서 asyncio WebSocket 서버를 시작한다."""
        if not WEBSOCKETS_AVAILABLE or not NUMPY_AVAILABLE:
            rospy.logwarn(
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
                    ping_timeout=20,
                    reuse_address=True):
                rospy.loginfo(
                    f'[PC2WS] Binary WebSocket server on ws://0.0.0.0:{self._port}')
                await asyncio.Future()   # 종료 없이 영원히 실행
        except OSError as e:
            if getattr(e, 'errno', None) == 98:
                rospy.logerr(_format_port_in_use_error(self._port, 'pc2_ws_port'))
            else:
                rospy.logerr(f'[PC2WS] server error: {e}')
        except Exception as e:
            rospy.logerr(f'[PC2WS] server error: {e}')

    async def _handler(self, websocket):
        """WebSocket 연결 핸들러 — subscribe/unsubscribe/subscribe_plot/subscribe_image 명령 수신."""
        my_pc2_topics: set      = set()   # PointCloud2 binary 구독
        my_plot_topics: set     = set()   # 범용 plot JSON 구독
        my_img_topics: set      = set()   # Image JPEG binary 구독
        my_latched_topics: set  = set()   # TRANSIENT_LOCAL PointCloud2 구독
        my_path_topics: set     = set()   # nav_msgs/Path binary 구독
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
                elif cmd == 'subscribe_image':
                    # sensor_msgs/Image → JPEG 바이너리 스트리밍
                    self._add_image_client(topic, websocket)
                    my_img_topics.add(topic)
                elif cmd == 'unsubscribe_image':
                    self._remove_image_client(topic, websocket)
                    my_img_topics.discard(topic)
                elif cmd == 'subscribe_path':
                    self._add_path_client(topic, websocket)
                    my_path_topics.add(topic)
                elif cmd == 'unsubscribe_path':
                    self._remove_path_client(topic, websocket)
                    my_path_topics.discard(topic)
                elif cmd == 'subscribe_latched':
                    # TRANSIENT_LOCAL QoS 구독 (latched 토픽용)
                    self._add_latched_client(topic, websocket)
                    my_latched_topics.add(topic)
                elif cmd == 'unsubscribe_latched':
                    self._remove_latched_client(topic, websocket)
                    my_latched_topics.discard(topic)
        except Exception:
            pass
        finally:
            with self._lock:
                self._all_clients.discard(websocket)
            for t in list(my_pc2_topics):
                self._remove_client(t, websocket)
            for t in list(my_plot_topics):
                self._remove_plot_client(t, websocket, None)
            for t in list(my_img_topics):
                self._remove_image_client(t, websocket)
            for t in list(my_latched_topics):
                self._remove_latched_client(t, websocket)
            for t in list(my_path_topics):
                self._remove_path_client(t, websocket)

    # ── 클라이언트 / 구독 관리 ────────────────────────────────────────────────

    # topic → 타입 문자열 캐시 (DDS 반복 조회 방지)
    _topic_type_cache: dict = {}

    def _get_topic_type(self, topic: str) -> str:
        """토픽 타입 조회 (PointCloud2 또는 CustomMsg 등) — 결과를 캐시에 저장."""
        if topic in self._topic_type_cache:
            return self._topic_type_cache[topic]
        try:
            for name, types in self._node.get_topic_names_and_types():
                if name == topic and types:
                    self._topic_type_cache[topic] = types[0]
                    return types[0]
        except Exception:
            pass
        return None

    def _add_client(self, topic: str, ws):
        # DDS 조회를 lock 외부에서 수행:
        # get_topic_names_and_types()는 DDS 전체 토픽을 열거하므로
        # publisher가 누적될수록 느려짐 — lock 내부 호출 시 _on_pc2/_on_image 콜백이 블로킹됨
        topic_type = self._get_topic_type(topic)
        is_livox = (topic_type == 'livox_ros_driver2/msg/CustomMsg' and LIVOX_AVAILABLE)

        with self._lock:
            if is_livox:
                if topic not in self._livox_clients:
                    self._livox_clients[topic] = set()
                self._livox_clients[topic].add(ws)
                if topic not in self._livox_subs:
                    sub = rospy.Subscriber(
                        topic, CustomMsg,
                        lambda m, t=topic: self._on_livox(m, t),
                        queue_size=10)
                    self._livox_subs[topic] = sub
                    self._livox_last_sent[topic] = 0.0
                    rospy.loginfo(f'[PC2WS] subscribed (Livox) → {topic}')
            else:
                if topic not in self._clients:
                    self._clients[topic] = set()
                self._clients[topic].add(ws)
                if topic not in self._subs:
                    sub = rospy.Subscriber(
                        topic, PointCloud2,
                        lambda m, t=topic: self._on_pc2(m, t),
                        queue_size=1)
                    self._subs[topic] = sub
                    self._last_sent[topic] = 0.0
                    rospy.loginfo(f'[PC2WS] subscribed → {topic}')

    def _remove_client(self, topic: str, ws):
        with self._lock:
            # Livox 클라이언트 확인
            s_livox = self._livox_clients.get(topic)
            if s_livox:
                s_livox.discard(ws)
                if not s_livox:
                    self._livox_clients.pop(topic, None)
                    # ROS2 구독은 유지 — DDS peer discovery를 살려 재연결 시 즉시 데이터 수신
                    rospy.loginfo(f'[PC2WS] all Livox clients gone, sub kept ← {topic}')
                return
            # PointCloud2 클라이언트
            s = self._clients.get(topic)
            if not s:
                return
            s.discard(ws)
            if not s:
                self._clients.pop(topic, None)
                # ROS2 구독은 유지 — DDS peer discovery를 살려 재연결 시 즉시 데이터 수신
                rospy.loginfo(f'[PC2WS] all PC2 clients gone, sub kept ← {topic}')

    def _presubscribe_pc2(self, topic: str):
        """Publisher 생성과 동시에 PointCloud2 ROS2 구독을 미리 생성해 DDS 발견을 워밍업한다.

        브라우저가 subscribe 명령을 보내기 전에 구독을 생성해 두면, 이후 브라우저 구독 시
        이미 DDS peer discovery가 완료되어 있어 첫 프레임 수신 즉시 visualization에 표시된다.
        """
        with self._lock:
            if topic not in self._subs:
                sub = rospy.Subscriber(
                    topic, PointCloud2,
                    lambda m, t=topic: self._on_pc2(m, t),
                    queue_size=10)
                self._subs[topic] = sub
                self._last_sent[topic] = 0.0
                rospy.loginfo(f'[PC2WS] pre-subscribed (warmup) → {topic}')

    # ── TRANSIENT_LOCAL (latched) 토픽 ─────────────────────────────────────────

    def _add_latched_client(self, topic: str, ws):
        """TRANSIENT_LOCAL + RELIABLE QoS로 구독 생성 후 캐시된 마지막 메시지를 즉시 재전송.

        /Laser_map 같은 latched 토픽은 publisher가 한 번 발행 후 업데이트가 드물 수 있다.
        TRANSIENT_LOCAL QoS로 구독해야 늦게 연결한 subscriber도 마지막 메시지를 수신한다.
        """
        cached_bin  = None
        cached_meta = None
        with self._lock:
            if topic not in self._latched_clients:
                self._latched_clients[topic] = set()
            self._latched_clients[topic].add(ws)
            if topic not in self._latched_subs:
                # ROS1: latch 토픽은 일반 Subscriber로 구독 (마지막 메시지 캐시는 별도 관리)
                sub = rospy.Subscriber(
                    topic, PointCloud2,
                    lambda m, t=topic: self._on_pc2_latched(m, t),
                    queue_size=1)
                self._latched_subs[topic] = sub
                rospy.loginfo(
                    f'[PC2WS] subscribed (latched) → {topic}')
            cached_bin  = self._latched_cache.get(topic)
            cached_meta = self._latched_meta_cache.get(topic)

        # 이미 캐시된 메시지가 있으면 이 클라이언트에게 즉시 재전송
        if cached_bin and cached_meta:
            loop = self._loop
            if loop and loop.is_running():
                async def _replay(b=cached_bin, m=cached_meta, w=ws):
                    try:
                        await w.send(m)
                        await w.send(b)
                    except Exception:
                        pass
                asyncio.run_coroutine_threadsafe(_replay(), loop)

    def _remove_latched_client(self, topic: str, ws):
        with self._lock:
            s = self._latched_clients.get(topic)
            if s:
                s.discard(ws)

    def _on_pc2_latched(self, msg: PointCloud2, topic_name: str):
        """TRANSIENT_LOCAL 토픽 콜백 — 페이로드 빌드 후 캐시 저장 + 브로드캐스트."""
        with self._lock:
            clients = self._latched_clients.get(topic_name, set()).copy()
        if not clients:
            return
        stamp = msg.header.stamp
        meta_json = json.dumps({
            'type':          'pc2meta',
            'topic':         topic_name,
            'stamp_sec':     stamp.secs,
            'stamp_nanosec': stamp.nsecs,
            'frame_id':      msg.header.frame_id,
            'point_count':   msg.width * msg.height,
        }, separators=(',', ':'))
        loop = self._loop
        if loop and loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self._build_and_broadcast_latched(msg, topic_name, clients, meta_json), loop)

    async def _build_and_broadcast_latched(
            self, msg: 'PointCloud2', topic_name: str, clients: set, meta_json: str):
        """_build_payload를 executor에서 실행, 결과를 캐시 저장 후 클라이언트에 전송."""
        try:
            loop = asyncio.get_running_loop()
            payload = await loop.run_in_executor(None, self._build_payload, msg, topic_name)
            if not payload:
                return
            # 캐시 갱신 (새 클라이언트 재전송용)
            with self._lock:
                self._latched_cache[topic_name]      = payload
                self._latched_meta_cache[topic_name] = meta_json
            for ws in list(clients):
                try:
                    await ws.send(meta_json)
                    await ws.send(payload)
                except Exception:
                    pass
        except Exception as e:
            rospy.logerr(
                f'[PC2WS] latched broadcast error ({topic_name}): {e}')

    def _presubscribe_image(self, topic: str):
        """Image 토픽을 미리 구독해 DDS 발견을 워밍업한다.

        Stereo 이미지처럼 브라우저가 명시적으로 subscribe하기 전부터 구독을 생성해 두면
        재생 시작 시 즉시 이미지를 수신할 수 있다.
        """
        with self._lock:
            if topic not in self._img_subs:
                sub = rospy.Subscriber(
                    topic, Image,
                    lambda m, t=topic: self._on_image(m, t),
                    queue_size=10)
                self._img_subs[topic] = sub
                self._img_last_sent[topic] = 0.0
                rospy.loginfo(f'[ImgWS] pre-subscribed (warmup) → {topic}')

    # ── Image 클라이언트 / 구독 관리 ─────────────────────────────────────────

    def _add_image_client(self, topic: str, ws):
        """sensor_msgs/Image 토픽을 JPEG 바이너리로 브라우저에 스트리밍하기 위한 클라이언트 등록."""
        with self._lock:
            if topic not in self._img_clients:
                self._img_clients[topic] = set()
            self._img_clients[topic].add(ws)
            if topic not in self._img_subs:
                sub = rospy.Subscriber(
                    topic, Image,
                    lambda m, t=topic: self._on_image(m, t),
                    queue_size=10)
                self._img_subs[topic]      = sub
                self._img_last_sent[topic] = 0.0
                rospy.loginfo(f'[ImgWS] subscribed → {topic}')

    def _remove_image_client(self, topic: str, ws):
        with self._lock:
            s = self._img_clients.get(topic)
            if not s:
                return
            s.discard(ws)
            if not s:
                self._img_clients.pop(topic, None)
                # ROS2 구독은 유지 — DDS peer discovery를 살려 재연결 시 즉시 이미지 수신
                rospy.loginfo(f'[ImgWS] all clients gone, sub kept ← {topic}')

    # ── Path 클라이언트 / 구독 관리 ───────────────────────────────────────────

    def _add_path_client(self, topic: str, ws):
        """nav_msgs/Path 토픽을 바이너리 PTH 패킷으로 스트리밍하기 위한 클라이언트 등록.

        rospy.AnyMsg 로 구독해 콜백에서 "역직렬화 없이" raw 바이트만 받는다.
        누적 경로(수천~수만 pose)를 매 메시지마다 통째로 역직렬화하던 비용을 제거해
        메인 스핀의 GIL 점유(→ /Odometry·points burst)를 근본적으로 줄인다.
        """
        with self._lock:
            if topic not in self._path_clients:
                self._path_clients[topic] = set()
            self._path_clients[topic].add(ws)
            if topic not in self._path_subs:
                sub = rospy.Subscriber(
                    topic, rospy.AnyMsg,
                    lambda m, t=topic: self._on_path(m, t),
                    queue_size=1,
                    buff_size=self.PATH_BUFF_SIZE)
                self._path_subs[topic]      = sub
                self._path_last_sent[topic] = 0.0
                rospy.loginfo(f'[PathWS] subscribed → {topic}')

    def _remove_path_client(self, topic: str, ws):
        with self._lock:
            s = self._path_clients.get(topic)
            if not s:
                return
            s.discard(ws)
            if not s:
                self._path_clients.pop(topic, None)
                rospy.loginfo(f'[PathWS] all clients gone, sub kept ← {topic}')

    # ── rclpy 콜백 (Path) ────────────────────────────────────────────────────

    def _on_path(self, msg, topic_name: str):
        """nav_msgs/Path(AnyMsg) 수신 → throttle → binary PTH 패킷 → asyncio 브로드캐스트.

        핵심: 콜백 자체는 raw 바이트만 참조하고 즉시 리턴한다(역직렬화 X).
        throttle/backpressure 로 대부분의 메시지를 값싸게 버려 rospy 수신 스레드가
        GIL 을 오래 잡지 않게 한다. 실제 파싱/직렬화는 executor 스레드에서 수행.
        """
        now = time.monotonic()
        with self._lock:
            if now - self._path_last_sent.get(topic_name, 0.0) < self.PATH_THROTTLE_SEC:
                return
            if self._path_sending.get(topic_name, False):
                return
            clients = self._path_clients.get(topic_name, set()).copy()
            if not clients:
                return
            self._path_last_sent[topic_name] = now
            self._path_sending[topic_name] = True

        raw = getattr(msg, '_buff', None)
        loop = self._loop
        if raw is not None and loop and loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self._build_and_broadcast_path(raw, topic_name, clients), loop)
        else:
            with self._lock:
                self._path_sending[topic_name] = False

    async def _build_and_broadcast_path(self, raw: bytes, topic_name: str, clients: set):
        """_build_path_payload를 thread pool에서 실행 후 브로드캐스트."""
        try:
            loop = asyncio.get_running_loop()
            payload = await loop.run_in_executor(
                None, self._build_path_payload, raw, topic_name)
            if payload:
                for ws in list(clients):
                    try:
                        await ws.send(payload)
                    except Exception:
                        pass
        except Exception as e:
            rospy.logwarn(
                f'[PathWS] broadcast error ({topic_name}): {e}')
        finally:
            with self._lock:
                self._path_sending[topic_name] = False

    def _build_path_payload(self, raw: bytes, topic_name: str) -> bytes:
        """nav_msgs/Path raw 직렬화 바이트 → PTH binary 패킷 생성.

        전체 메시지를 rospy 객체로 역직렬화하지 않고, 직렬화 레이아웃에서
        pose 위치(XYZ)만 numpy 로 벡터 추출한다(파이썬 pose 루프 제거).
        PATH_MAX_POSES 초과 시 균등 decimation 으로 부하/전송량을 상수화한다.

        Binary 패킷 포맷 (little-endian):
          [3B]  magic = b'PTH'
          [1B]  version = 1
          [4B]  uint32  topic_name 바이트 길이
          [4B]  uint32  frame_id 바이트 길이
          [4B]  uint32  total_pose_count
          [N B] topic_name (UTF-8)
          [M B] frame_id  (UTF-8)
          [count*12 B] XYZ float32 interleaved (x0,y0,z0, x1,y1,z1, ...)
        """
        try:
            frame_b, xyz = self._parse_path_positions(raw)
            if xyz is None or xyz.size == 0:
                return None
            n = xyz.shape[0]
            topic_b = topic_name.encode('utf-8')
            header = struct.pack('<3sBIII',
                                 b'PTH', 1,
                                 len(topic_b), len(frame_b), n)
            return b''.join([header, topic_b, frame_b,
                             np.ascontiguousarray(xyz).tobytes()])
        except Exception as e:
            rospy.logerr(
                f'[PathWS] _build_path_payload error ({topic_name}): {e}')
            return None

    def _parse_path_positions(self, raw: bytes):
        """nav_msgs/Path 직렬화 바이트에서 (frame_id_bytes, xyz_float32) 추출.

        ROS1 직렬화 레이아웃:
          Header header      : uint32 seq, time(2×uint32), string frame_id
          PoseStamped[] poses:
            uint32 N
            (per pose) Header(seq,stamp,frame_id) + Pose(pos 3×f64, quat 4×f64)

        각 PoseStamped 의 frame_id 길이가 동일하면(대다수 SLAM 퍼블리셔) pose 블록
        stride 가 상수이므로 structured dtype 로 위치만 한 번에 벡터 추출한다.
        길이가 제각각이면 안전하게 순차 파싱으로 폴백한다.
        반환 xyz: shape (n, 3) float32.
        """
        mv = memoryview(raw)
        total = len(mv)
        # ── outer Header ─────────────────────────────────────────────────
        off = 4 + 8                                   # seq(4) + stamp(8)
        outer_fl = struct.unpack_from('<I', mv, off)[0]
        off += 4
        frame_b = bytes(mv[off:off + outer_fl])
        off += outer_fl
        # ── poses 배열 길이 ──────────────────────────────────────────────
        n = struct.unpack_from('<I', mv, off)[0]
        off += 4
        if n <= 0:
            return frame_b, None
        poses_start = off
        # 첫 pose 헤더의 frame_id 길이로 stride 추정 (seq4 + stamp8 = 12)
        pose_fl = struct.unpack_from('<I', mv, poses_start + 12)[0]
        pre       = 16 + pose_fl                       # seq4+stamp8+len4+str
        stride    = pre + 56                           # pos(24) + quat(32)

        if stride > 0 and (total - poses_start) == n * stride:
            # 빠른 경로: 상수 stride → structured dtype 로 위치만 추출
            pose_dtype = np.dtype([
                ('pre',  'V%d' % pre),
                ('pos',  '<f8', (3,)),
                ('post', 'V32'),
            ])
            arr = np.frombuffer(raw, dtype=pose_dtype, count=n, offset=poses_start)
            xyz = arr['pos'].astype(np.float32)        # (n, 3)
        else:
            # 폴백: pose 별 frame_id 길이가 달라 stride 가 가변인 경우 순차 파싱
            xyz = np.empty((n, 3), dtype=np.float32)
            p = poses_start
            for i in range(n):
                fl = struct.unpack_from('<I', mv, p + 12)[0]
                base = p + 16 + fl
                xyz[i, 0], xyz[i, 1], xyz[i, 2] = struct.unpack_from('<3d', mv, base)
                p = base + 56

        # ── pose 상한 초과 시 균등 decimation ────────────────────────────
        if xyz.shape[0] > self.PATH_MAX_POSES:
            step = int(math.ceil(xyz.shape[0] / self.PATH_MAX_POSES))
            xyz = xyz[::step]
        return frame_b, xyz

    # ── rclpy 콜백 (Image) ────────────────────────────────────────────────────

    def _on_image(self, msg: Image, topic_name: str):
        """Image 메시지 수신 → 비동기 JPEG 인코딩 → binary 패킷 → asyncio 브로드캐스트.

        ROS2 callback 스레드 블로킹 최소화:
          - throttle 체크 후 즉시 리턴 (인코딩을 thread pool executor로 위임)
          - Bayer 패턴(bayer_*): 디베이어링 없이 mono8 패스스루 (~10x 비용 절감)
          - 대형 이미지: IMG_MAX_DIM 초과 시 비율 유지 리사이즈

        Binary 패킷 포맷 (little-endian):
          [3B]  magic = b'IMG'
          [1B]  version = 1
          [4B]  uint32  topic_name 바이트 길이
          [4B]  uint32  jpeg_data 바이트 길이
          [N B] topic_name (UTF-8)
          [L B] JPEG data
        """
        now = time.monotonic()
        with self._lock:
            if now - self._img_last_sent.get(topic_name, 0.0) < self.IMG_THROTTLE_SEC:
                return
            # 이전 인코딩/브로드캐스트가 완료되지 않았으면 skip (asyncio 큐 누적 방지)
            if self._img_sending.get(topic_name, False):
                return
            clients = self._img_clients.get(topic_name, set()).copy()
            if not clients:
                return
            self._img_last_sent[topic_name] = now
            self._img_sending[topic_name] = True

        # 무거운 인코딩 작업을 thread pool로 위임 → ROS2 callback 스레드 즉시 해방
        loop = self._loop
        if loop and loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self._encode_and_broadcast_image(msg, topic_name, clients), loop)
        else:
            with self._lock:
                self._img_sending[topic_name] = False

    def _encode_image_to_payload(self, msg: Image, topic_name: str) -> bytes:
        """JPEG 인코딩 (thread pool executor에서 실행 — ROS2 callback 스레드 외부).

        최적화:
          - bayer_* encoding: 디베이어링 없이 raw 데이터를 mono8로 직접 사용
          - 대형 이미지: IMG_MAX_DIM 초과 시 비율 유지 축소
        """
        try:
            encoding = msg.encoding.lower()
            if encoding.startswith('bayer_'):
                # Bayer 패턴 → mono8 패스스루: cv2.cvtColor(Bayer→BGR) 비용 제거
                raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                cv_img = raw.reshape(msg.height, msg.width)
            elif encoding in ('rgb8', 'bgr8', 'mono8', 'rgba8', 'bgra8',
                              '8uc1', '8uc3', '8uc4'):
                cv_img = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                try:
                    cv_img = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                except Exception:
                    return None

            # 대형 이미지 리사이즈 (IMG_MAX_DIM 초과 시)
            h, w = cv_img.shape[:2]
            if max(h, w) > self.IMG_MAX_DIM:
                scale = self.IMG_MAX_DIM / max(h, w)
                cv_img = cv2.resize(cv_img,
                                    (int(w * scale), int(h * scale)),
                                    interpolation=cv2.INTER_LINEAR)

            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.IMG_JPEG_QUALITY]
            ret, jpeg_buf = cv2.imencode('.jpg', cv_img, encode_param)
            if not ret:
                return None
            jpeg_bytes = jpeg_buf.tobytes()

            topic_bytes = topic_name.encode('utf-8')
            header = struct.pack('<3sBII',
                                 b'IMG', 1,
                                 len(topic_bytes),
                                 len(jpeg_bytes))
            return header + topic_bytes + jpeg_bytes
        except Exception as e:
            rospy.logwarn(f'[ImgWS] encode error ({topic_name}): {e}')
            return None

    async def _encode_and_broadcast_image(self, msg: Image, topic_name: str, clients: set):
        """인코딩(thread pool) → 브로드캐스트(asyncio) 파이프라인.

        완료 후 반드시 _img_sending 플래그를 해제하여 다음 콜백이 처리될 수 있도록 한다.
        """
        try:
            loop = asyncio.get_running_loop()
            payload = await loop.run_in_executor(
                None,  # 기본 ThreadPoolExecutor
                self._encode_image_to_payload,
                msg,
                topic_name,
            )
            if payload:
                await self._broadcast(clients, payload)
        except Exception as e:
            rospy.logwarn(
                f'[ImgWS] async encode/broadcast error ({topic_name}): {e}')
        finally:
            with self._lock:
                self._img_sending[topic_name] = False

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
        throttle = (self.CLOUD_REGISTERED_THROTTLE_SEC
                     if topic_name == '/cloud_registered' else self.THROTTLE_SEC)
        with self._lock:
            if now - self._last_sent.get(topic_name, 0.0) < throttle:
                return
            # 이전 브로드캐스트가 완료되지 않았으면 skip (asyncio 큐 누적 방지)
            if self._pc2_sending.get(topic_name, False):
                return
            clients = self._clients.get(topic_name, set()).copy()
            if not clients:
                return
            self._last_sent[topic_name] = now
            self._pc2_sending[topic_name] = True

        # ── 1) JSON 메타데이터 패킷 (헤더 스탬프 등) ────────────────────────
        stamp = msg.header.stamp
        meta_json = json.dumps({
            'type':          'pc2meta',
            'topic':         topic_name,
            'stamp_sec':     stamp.secs,
            'stamp_nanosec': stamp.nsecs,
            'frame_id':      msg.header.frame_id,
            'point_count':   msg.width * msg.height,
        }, separators=(',', ':'))

        # ── 2) _build_payload + broadcast를 asyncio coroutine으로 위임 ────────
        # _build_payload(numpy heavy)를 run_in_executor로 실행해
        # rospy 콜백 스레드 블로킹을 제거하고, 완료 후 _pc2_sending 플래그를 해제한다.
        loop = self._loop
        if loop and loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self._build_and_broadcast_pc2(msg, topic_name, clients, meta_json), loop)
        else:
            with self._lock:
                self._pc2_sending[topic_name] = False

    async def _build_and_broadcast_livox(
            self, msg, topic_name: str, clients: set, meta_json: str):
        """_build_livox_payload를 thread pool에서 실행 후 브로드캐스트."""
        try:
            loop = asyncio.get_running_loop()
            payload = await loop.run_in_executor(
                None, self._build_livox_payload, msg, topic_name)
            if payload:
                for ws in list(clients):
                    try:
                        await ws.send(meta_json)
                        await ws.send(payload)
                    except Exception:
                        pass
        except Exception as e:
            rospy.logwarn(
                f'[PC2WS] async build/broadcast Livox error ({topic_name}): {e}')
        finally:
            with self._lock:
                self._livox_sending[topic_name] = False

    async def _build_and_broadcast_pc2(
            self, msg: 'PointCloud2', topic_name: str, clients: set, meta_json: str):
        """_build_payload를 thread pool executor에서 실행 후 브로드캐스트.

        완료 후 반드시 _pc2_sending 플래그를 해제하여 다음 콜백이 처리될 수 있도록 한다.
        """
        try:
            loop = asyncio.get_running_loop()
            payload = await loop.run_in_executor(None, self._build_payload, msg, topic_name)
            if payload:
                for ws in list(clients):
                    try:
                        await ws.send(meta_json)
                        await ws.send(payload)
                    except Exception:
                        pass
        except Exception as e:
            rospy.logwarn(f'[PC2WS] async build/broadcast error ({topic_name}): {e}')
        finally:
            with self._lock:
                self._pc2_sending[topic_name] = False

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
                rospy.loginfo(f'[PC2WS/plot] unsubscribed ← {topic}')

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
                rospy.logerr(f'[PC2WS/plot] 토픽 타입 조회 오류: {e}')

        if not type_str:
            rospy.logwarn(
                f'[PC2WS/plot] 토픽 타입 못 찾음 (msg_type 미제공, DDS 조회 실패): {topic}')
            return

        MsgClass = self._get_msg_class(type_str)
        if MsgClass is None:
            rospy.logwarn(
                f'[PC2WS/plot] 메시지 타입 로드 실패: {type_str}')
            return

        sub = rospy.Subscriber(
            topic,
            MsgClass,
            lambda msg, t=topic: self._on_plot_msg(msg, t),
            queue_size=10
        )
        with self._lock:
            self._plot_subs[topic] = sub
        rospy.loginfo(
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
            stamp_sec     = msg.header.stamp.secs
            stamp_nanosec = msg.header.stamp.nsecs
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
            'header/stamp/secs'      →  obj.header.stamp.secs  (ROS1)
        """
        for part in field_path.replace('.', '/').split('/'):
            # ROS1 rospy.Time 호환: sec→secs, nanosec→nsecs 자동 변환
            if not hasattr(obj, part):
                if part == 'sec' and hasattr(obj, 'secs'):
                    part = 'secs'
                elif part == 'nanosec' and hasattr(obj, 'nsecs'):
                    part = 'nsecs'
                else:
                    return None
            obj = getattr(obj, part)
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

    @staticmethod
    def _voxel_downsample(xyz: np.ndarray, voxel_size: float) -> np.ndarray:
        """numpy 기반 복셀 다운샘플링. 각 복셀에서 첫 번째 점만 유지."""
        if len(xyz) == 0:
            return xyz
        voxel_coords = np.floor(xyz / voxel_size).astype(np.int32)
        _, unique_idx = np.unique(voxel_coords, axis=0, return_index=True)
        return xyz[np.sort(unique_idx)]

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

            # 균등 step 다운샘플링 (모든 PC2 토픽 동일)
            # /cloud_registered의 np.unique 복셀은 제거함:
            #   - 브라우저 map_accumulator_worker가 0.3m 전역 복셀(centroid)을 수행
            #   - 서버 np.unique(axis=0)는 대용량에서 GIL을 수백 ms 점유 → analytics max time 악화
            max_pts = (self.CLOUD_REGISTERED_MAX_POINTS
                       if topic_name == '/cloud_registered' else self.MAX_POINTS)
            step = max(1, n // max_pts)
            x, y, z = x[::step], y[::step], z[::step]
            n_out = len(x)
            subsample = lambda arr_v, _s=step, _n=n_out: arr_v[::_s][:_n]

            xyz = np.column_stack([x, y, z]).astype(np.float32)

            # intensity 추출
            has_intensity = 'intensity' in field_map
            color_f32 = np.zeros(n_out, dtype=np.float32)
            if has_intensity:
                ci = _extract_f32('intensity')
                color_f32 = subsample(ci[valid])

            # RGB 추출
            has_rgb = 'rgb' in field_map or 'rgba' in field_map
            rgb_u32 = np.zeros(n_out, dtype=np.uint32)
            if has_rgb:
                rkey = 'rgb' if 'rgb' in field_map else 'rgba'
                f    = field_map[rkey]
                ri   = np.frombuffer(
                    arr[:, f.offset:f.offset + 4].tobytes(), dtype=np.uint32)
                rgb_u32 = subsample(ri[valid])

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
            rospy.logerr(f'[PC2WS] _build_payload error: {e}')
            return None

    # ── Livox CustomMsg → PC2 호환 binary ──────────────────────────────────────

    def _build_livox_payload(self, msg, topic_name: str):
        """Livox CustomMsg를 PC2와 동일한 binary 포맷으로 변환."""
        if not LIVOX_AVAILABLE:
            return None
        try:
            frame_id = msg.header.frame_id if msg.header else 'livox'
            points = msg.points or []
            n_total = min(len(points), self.MAX_POINTS)
            if n_total == 0:
                return None

            x = np.array([p.x for p in points[:n_total]], dtype=np.float32)
            y = np.array([p.y for p in points[:n_total]], dtype=np.float32)
            z = np.array([p.z for p in points[:n_total]], dtype=np.float32)
            reflectivity = np.array(
                [getattr(p, 'reflectivity', 0.0) for p in points[:n_total]],
                dtype=np.float32)

            valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
            x, y, z = x[valid], y[valid], z[valid]
            reflectivity = reflectivity[valid]
            n_out = len(x)
            if n_out == 0:
                return None

            xyz = np.column_stack([x, y, z]).astype(np.float32)
            flags = 0x01  # has_intensity (reflectivity)
            topic_b = topic_name.encode('utf-8')
            frame_b = frame_id.encode('utf-8')
            header = struct.pack(
                '<3sBBIII',
                b'PC2', 1, flags,
                len(topic_b), len(frame_b), n_out)
            return b''.join([
                header, topic_b, frame_b,
                xyz.tobytes(), reflectivity.astype(np.float32).tobytes()])
        except Exception as e:
            rospy.logerr(f'[PC2WS] _build_livox_payload error: {e}')
            return None

    def _on_livox(self, msg, topic_name: str):
        """Livox CustomMsg 수신 → PC2 호환 binary + JSON 메타데이터 → 브로드캐스트."""
        now = time.monotonic()
        with self._lock:
            if now - self._livox_last_sent.get(topic_name, 0.0) < self.THROTTLE_SEC:
                return
            # 이전 브로드캐스트가 완료되지 않았으면 skip (asyncio 큐 누적 방지)
            if self._livox_sending.get(topic_name, False):
                return
            clients = self._livox_clients.get(topic_name, set()).copy()
            if not clients:
                return
            self._livox_last_sent[topic_name] = now
            self._livox_sending[topic_name] = True

        stamp = msg.header.stamp if msg.header else None
        frame_id = msg.header.frame_id if msg.header else ''
        point_count = len(msg.points) if msg.points else 0

        meta_json = json.dumps({
            'type': 'pc2meta',
            'topic': topic_name,
            'stamp_sec': stamp.secs if stamp else 0,
            'stamp_nanosec': stamp.nsecs if stamp else 0,
            'frame_id': frame_id,
            'point_count': point_count,
        }, separators=(',', ':'))

        loop = self._loop
        if loop and loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self._build_and_broadcast_livox(msg, topic_name, clients, meta_json), loop)
        else:
            with self._lock:
                self._livox_sending[topic_name] = False


class WebGUINode:
    def __init__(self):
        rospy.init_node('ros_slam_webui_node', anonymous=False)

        self.web_port = int(rospy.get_param('~web_port', 8880))
        self.pc2_ws_port = int(rospy.get_param('~pc2_ws_port', 8881))

        # SLAM GUI state
        self.slam_map1 = ""
        self.slam_map2 = ""
        self.slam_output = ""
        self.slam_status = "Ready"
        self.slam_process = None
        self.slam_optimization_process = None

        # Multi-Session Optimization async state
        self.slam_opt_running = False
        self.slam_opt_status = {'running': False, 'done': False, 'success': None, 'message': ''}

        # Save map async state
        self.slam_map_saving = False
        self.slam_map_save_cancelled = False
        self.slam_map_save_status = {'saving': False, 'done': False, 'success': None, 'message': ''}
        # Last requested save directory (used by Save Map Result Viewer / path whitelist)
        self.slam_last_saved_dir = ""

        # Localization state
        self.localization_process = None

        # FAST-LIO config files used by roslaunch (config_file:=...)
        fast_lio_paths = get_fast_lio_config_paths()
        if fast_lio_paths.get('success'):
            self._slam_config_file = fast_lio_paths['mapping_config']
            self._localization_config_file = fast_lio_paths['localization_config']
            rospy.loginfo(f'Default SLAM config: {self._slam_config_file}')
            rospy.loginfo(f'Default localization config: {self._localization_config_file}')
        else:
            self._slam_config_file = None
            self._localization_config_file = None
            rospy.logwarn(
                'FAST-LIO config directory not found: {}'.format(
                    fast_lio_paths.get('message', 'unknown')
                )
            )

        # Bag Player state
        self.bag_path = ""
        self.bag_playing = False
        self.bag_paused = False
        self.bag_process = None
        self.bag_playback_rate = 1.0  # 현재 설정된 재생 속도 배율
        self.bag_player_loop = False  # 루프 재생 여부

        # Bag Recorder state
        self.recorder_bag_name = ""
        self.recorder_recording = False
        self.recorder_process = None
        self.recorder_ros1_thread = None   # Ros1BagRecorderThread 인스턴스
        self.recorder_mode = 'ros2'        # 'ros2' | 'ros1'
        self.bag_topics = []
        self.bag_topic_infos = []  # [{name, type, publishable}, ...] — recorder 토픽 보충용
        self.bag_selected_topics = []
        self.bag_duration = 0.0  # Duration in seconds
        self.bag_current_time = 0.0  # Current playback time in seconds
        self.bag_start_offset = 0.0  # Start offset for playback
        self.bag_start_real_time = 0.0  # Real time when playback started
        self.bag_pause_time = 0.0  # Time when paused
        self._bag_stop_pause_offset = None  # 서비스 실패 시 stop+restart 방식에서 일시정지 위치(초)
        self._bag_play_lock = threading.Lock()  # bag_play_toggle 과 _bag_process_monitor 간 경쟁 방지

        # File Player state
        self.player_path = ""
        self.player_playing = False
        self.player_paused = False
        self.player_loop = False
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
        self.save_bag_message = None    # KITTI처럼 단계별 메시지 (예: "Converting pose messages...")
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
        self.kitti_cam_pub = None          # 하위호환: /kitti/camera_color_left/image_raw
        self.kitti_imu_pub = None          # /kitti/oxts/imu
        self.kitti_gps_fix_pub = None      # /kitti/oxts/gps/fix
        self.kitti_gps_vel_pub = None      # /kitti/oxts/gps/vel
        self.kitti_cam_pubs = {}           # {cam_id: publisher} — 4채널 카메라
        self.kitti_cam_info_pubs = {}      # {cam_id: publisher} — camera_info
        self.kitti_tf_static_pub = None    # /tf_static publisher
        self.kitti_tf_pub = None           # /tf publisher
        # KITTI calib / oxts / TF 관련 상태
        self.kitti_calib_dir = None        # calib 파일 디렉토리 경로
        self.kitti_oxts_files = []         # oxts 파일 경로 목록 (sorted)
        self.kitti_oxts_timestamps = []    # oxts 타임스탬프 (ns) 목록
        self.kitti_origin_oxts = None      # Mercator 원점 OXTS 데이터
        self.kitti_mercator_scale = None   # Mercator 투영 스케일
        # 초기화 플래그 (각 데이터셋 전용 publishers 이중 생성 방지)
        self._conpr_pubs_initialized = False
        self._kitti_pubs_initialized = False
        # KAIST 전용 publishers (lazy init)
        self.kaist_imu_pub = None
        self.kaist_gps_pub = None
        self.kaist_vrs_pub = None
        self.kaist_vlp_left_pub = None
        self.kaist_vlp_right_pub = None
        self.kaist_sick_back_pub = None
        self.kaist_sick_mid_pub = None
        self.kaist_stereo_left_pub = None
        self.kaist_stereo_right_pub = None
        self.kaist_tf_static_pub = None
        self.kaist_tf_pub = None

        # CV Bridge for image conversion
        self.cv_bridge = CvBridge()

        # SLAM Subscribers — lazy initialized on start_slam_mapping()
        self.slam_complete_sub = None

        # KITTI 카메라 방향 감지 캐시 (로드 시 1회만 탐색)
        self.kitti_cam_dir = ''            # 하위호환 (단일 카메라)
        self.kitti_cam_encoding = ''       # 하위호환 (단일 카메라)
        self.kitti_cam_dirs_map = {}       # {cam_id: dir_path} — 존재하는 카메라 dirs
        self.kitti_calib_cam_to_cam = None  # camera_info 생성용 calib 데이터

        # File Player data structures
        self.data_stamp = {}
        self.pose_data = {}
        self.imu_data = {}
        self.livox_file_list = []      # List of LiDAR .bin files
        self.cam_file_list = []        # List of camera image files
        self.livox_stamp_to_path = {}  # stamp → .bin 경로 맵 (O(1) 룩업, os.path.exists 제거)
        self.cam_stamp_to_path = {}    # stamp → 이미지 경로 맵 (O(1) 룩업)
        self.livox_cache = {}          # Cache for loaded LiDAR data
        self.cam_cache = {}            # Cache for loaded camera images

        # Playback thread
        self.playback_thread = None
        self.playback_active = False
        self.player_seek_requested = False  # seek 후 worker 인덱스 재설정 신호
        self.player_seek_to_stamp  = 0      # seek 목표 타임스탬프 (HTTP 스레드→worker 전달용)

        # Timer for playback (matching C++ implementation)
        rospy.Timer(rospy.Duration(0.01), lambda event: self.timer_callback())  # 10ms = 100Hz
        # Timer for bag playback time tracking
        rospy.Timer(rospy.Duration(0.1), lambda event: self.bag_timer_callback())  # 100ms = 0.1s

        # Setup reusable environment for subprocess calls
        self._setup_ros_environment()

        # ros2 topic list -t 결과 캐시 (서브프로세스 비용·메인 스레드 지연 완화)
        self._ros_topics_list_cache = None  # (monotonic_time, list[dict])
        self._ros_topics_list_cache_ttl_sec = 2.0

        # ── PC2 Binary WebSocket 서버 (포트 8081) ─────────────────────────────
        # rosbridge를 우회해 PointCloud2를 Python에서 직접 처리 후 binary 전송
        self.pc2_ws_server = PC2WebSocketServer(self, port=self.pc2_ws_port)
        self.pc2_ws_server.start()

        # ── KITTI 변환기 상태 ──────────────────────────────────────────────────
        self.kitti_converter_running = False   # 변환 진행 중 여부
        self.kitti_convert_thread = None       # 변환 백그라운드 스레드

        # ── ROS2 bag 모드 플래그 (player_play_toggle → bag_play_toggle 위임) ─
        self.player_is_ros2_bag = False        # True 이면 File Player가 ROS2 bag 모드
        self.player_is_ros1_bag = False        # True 이면 File Player가 ROS1 .bag 모드

        # ── KITTI direct play 모드 ──────────────────────────────────────────
        self.player_is_kitti = False           # True 이면 KITTI 파일 직접 재생
        self.kitti_drive_path = None           # 로드된 KITTI drive 디렉토리 경로
        self._kitti_conv = None                # KittiConverter 캐시 (프레임당 인스턴스 생성 방지)

        # ── KAIST direct play 모드 ──────────────────────────────────────────
        self.player_is_kaist = False           # True 이면 KAIST 파일 직접 재생
        self.kaist_dataset_path = None          # 로드된 KAIST 시퀀스 디렉토리 경로
        self.kaist_global_poses = []            # [(stamp_ns, R, T), ...]
        self.kaist_imu_data = ([], [])          # (stamps, rows) for bisect O(log n) lookup
        self.kaist_gps_data = ([], [])
        self.kaist_vrs_data = ([], [])
        self._kaist_pubs_initialized = False    # KAIST publisher 초기화 여부
        self._kaist_conv = None                 # KaistConverter 캐시 (프레임당 인스턴스 생성 방지)
        self.kaist_converter_running = False   # KAIST 변환 진행 중 여부
        self.kaist_convert_thread = None       # KAIST 변환 백그라운드 스레드
        # 성능 최적화: 센서별 stamp 세트 (O(1) 룩업 — os.path.isfile() 대체)
        self.kaist_vlp_left_stamps: set = set()
        self.kaist_vlp_right_stamps: set = set()
        self.kaist_sick_back_stamps: set = set()
        self.kaist_sick_mid_stamps: set = set()
        self.kaist_stereo_stamps: set = set()
        self.kaist_imu_stamps: set = set()
        self.kaist_gps_stamps: set = set()
        self.kaist_vrs_stamps: set = set()
        self.kaist_pose_stamp_set: set = set()    # global_pose 타임스탬프 세트
        self.kaist_pose_stamps_sorted: list = []  # 정렬된 pose 타임스탬프 (bisect용)
        self.kaist_static_tf_msg = None           # 로드 시 생성된 static TF (매 프레임 /tf에 포함용)
        # 성능 최적화: 경로 캐시 (매 프레임 os.path.join 생략)
        self.kaist_vlp_left_dir: str = ''
        self.kaist_vlp_right_dir: str = ''
        self.kaist_sick_back_dir: str = ''
        self.kaist_sick_mid_dir: str = ''
        self.kaist_stereo_left_dir: str = ''
        self.kaist_stereo_right_dir: str = ''

        # ── MulRan direct play 모드 ─────────────────────────────────────────
        self.player_is_mulran = False
        self.mulran_dataset_path = None
        self.mulran_ctx = None                 # MulRanConverter._load_sequence_context 결과
        self.mulran_events_by_stamp = {}       # stamp_ns → [sensor_name, ...] (data_stamp.csv 순서 유지)
        self.mulran_static_tf_msg = None       # 로드 시 생성된 static TF (매 프레임 /tf에 포함용)
        self._mulran_conv = None
        self._mulran_pubs_initialized = False
        self._mulran_last_clock_pub_ns = None
        self.mulran_converter_running = False
        self.mulran_convert_thread = None
        # 레퍼런스 OusterThread/RadarpolarThread 패턴: 센서별 백그라운드 publish 워커
        self._mulran_ouster_worker: '_SensorPublishWorker | None' = None
        self._mulran_radar_worker: '_SensorPublishWorker | None' = None
        # KAIST VLP/SICK/Stereo 백그라운드 publish 워커
        self._kaist_vlp_left_worker: '_SensorPublishWorker | None' = None
        self._kaist_vlp_right_worker: '_SensorPublishWorker | None' = None
        self._kaist_sick_back_worker: '_SensorPublishWorker | None' = None
        self._kaist_sick_mid_worker: '_SensorPublishWorker | None' = None
        self._kaist_stereo_worker: '_SensorPublishWorker | None' = None
        # KITTI Velodyne / Camera 백그라운드 publish 워커
        self._kitti_velo_worker: '_SensorPublishWorker | None' = None
        self._kitti_cam_worker: '_SensorPublishWorker | None' = None
        # ConPR Livox / Camera 백그라운드 publish 워커
        self._conpr_livox_worker: '_SensorPublishWorker | None' = None
        self._conpr_cam_worker: '_SensorPublishWorker | None' = None

        rospy.loginfo('Web GUI Node initialized with full ROS2 integration')

    def _setup_ros_environment(self):
        """
        Setup reusable ROS1 environment for subprocess calls.
        Since this node is launched from a sourced ROS environment,
        os.environ already contains all required ROS variables.
        """
        # The process is already running in a sourced ROS1 environment,
        # so simply copy the current environment variables (no subprocess needed).
        self._ros_env = os.environ.copy()
        rospy.loginfo('ROS1 environment cached from current process environment')

        # Add DISPLAY and XAUTHORITY for GUI applications (rviz2)
        # Try to get DISPLAY from environment or default to :0
        if 'DISPLAY' in os.environ:
            self._ros_env['DISPLAY'] = os.environ['DISPLAY']
        else:
            # Default to :0 if not set (common for local X server)
            self._ros_env['DISPLAY'] = ':0'
            rospy.loginfo('DISPLAY not set, defaulting to :0')
        
        # Try to get XAUTHORITY from environment or try common locations
        if 'XAUTHORITY' in os.environ:
            self._ros_env['XAUTHORITY'] = os.environ['XAUTHORITY']
            rospy.loginfo(f'Using XAUTHORITY from environment: {os.environ["XAUTHORITY"]}')
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
                            rospy.loginfo(f'Found XAUTHORITY at: {xauth_path}')
                            xauth_found = True
                            break
                else:
                    if os.path.exists(xauth_pattern):
                        self._ros_env['XAUTHORITY'] = xauth_pattern
                        rospy.loginfo(f'Found XAUTHORITY at: {xauth_pattern}')
                        xauth_found = True
                        break
            
            if not xauth_found:
                # Try to find any XAUTHORITY file in /run/user/
                user_run_dir = f'/run/user/{os.getuid()}'
                if os.path.exists(user_run_dir):
                    wayland_auth_files = glob.glob(f'{user_run_dir}/.mutter-Xwaylandauth.*')
                    if wayland_auth_files:
                        self._ros_env['XAUTHORITY'] = wayland_auth_files[0]
                        rospy.loginfo(f'Found Wayland XAUTHORITY at: {wayland_auth_files[0]}')
                    else:
                        # Fallback to user's home directory (even if it doesn't exist)
                        self._ros_env['XAUTHORITY'] = os.path.expanduser('~/.Xauthority')
                        rospy.logwarn('XAUTHORITY not found, using ~/.Xauthority (may not exist)')
                else:
                    self._ros_env['XAUTHORITY'] = os.path.expanduser('~/.Xauthority')
                    rospy.logwarn('XAUTHORITY not found, using ~/.Xauthority (may not exist)')

    def _init_common_ros_interfaces(self):
        """공통 인터페이스 초기화: /clock publisher + file_player 구독.
        ConPR/KITTI 어느 쪽이든 처음 로드 시 한 번만 호출.
        """
        if self.clock_pub is None:
            self.clock_pub = rospy.Publisher('/clock', Clock, queue_size=1)
        if self.start_sub is None:
            self.start_sub = rospy.Subscriber(
                '/file_player_start', Bool, self.file_player_start_callback, queue_size=1)
        if self.stop_sub is None:
            self.stop_sub = rospy.Subscriber(
                '/file_player_stop', Bool, self.file_player_stop_callback, queue_size=1)

    def _init_file_player_ros_interfaces(self):
        """ConPR 전용 publisher 초기화 (lazy).

        ConPR 데이터를 처음 로드할 때만 호출.
        KITTI 데이터를 로드해도 ConPR 토픽은 생성되지 않는다.
        """
        self._init_common_ros_interfaces()
        if self._conpr_pubs_initialized:
            return

        self.pose_pub     = rospy.Publisher('/pose/position', PointStamped, queue_size=10)
        self.imu_pub      = rospy.Publisher('/imu', Imu, queue_size=20)
        self.cam_pub      = rospy.Publisher('/camera/color/image', Image, queue_size=5)
        self.cam_info_pub = rospy.Publisher('/camera/color/camera_info', CameraInfo, queue_size=10)

        if LIVOX_AVAILABLE:
            self.livox_pub = rospy.Publisher('/livox/lidar', CustomMsg, queue_size=5)

        self._conpr_pubs_initialized = True
        rospy.loginfo('ConPR File Player publishers initialized')

    def _destroy_conpr_publishers(self):
        """ConPR publishers 정리. ROS1 bag 재생 시 /livox/lidar 등 토픽 충돌 방지.

        변환된 ROS1 bag은 /livox/lidar를 PointCloud2로 저장하므로,
        기존 livox_pub(CustomMsg)가 있으면 create_publisher 충돌 발생.
        """
        if not self._conpr_pubs_initialized:
            return
        for name, pub in [
                ('pose_pub', self.pose_pub),
                ('imu_pub', self.imu_pub),
                ('cam_pub', self.cam_pub),
                ('cam_info_pub', self.cam_info_pub),
                ('livox_pub', self.livox_pub),
        ]:
            if pub is not None:
                try:
                    pub.unregister()
                except Exception as e:
                    rospy.logwarn(f'[ConPR] destroy {name}: {e}')
        self.pose_pub = None
        self.imu_pub = None
        self.cam_pub = None
        self.cam_info_pub = None
        self.livox_pub = None
        self._conpr_pubs_initialized = False
        rospy.loginfo('ConPR publishers destroyed (for bag playback)')

    def _destroy_kaist_publishers(self):
        """KAIST publishers 정리.

        데이터셋 전환 시 DDS 큐를 비워 latency 누적을 방지한다.
        다음 KAIST 로드 시 _init_kaist_ros_interfaces()가 새로 생성한다.
        """
        if not self._kaist_pubs_initialized:
            return
        _pubs = [
            ('kaist_imu_pub', self.kaist_imu_pub),
            ('kaist_gps_pub', self.kaist_gps_pub),
            ('kaist_vrs_pub', self.kaist_vrs_pub),
            ('kaist_vlp_left_pub', self.kaist_vlp_left_pub),
            ('kaist_vlp_right_pub', self.kaist_vlp_right_pub),
            ('kaist_sick_back_pub', self.kaist_sick_back_pub),
            ('kaist_sick_mid_pub', self.kaist_sick_mid_pub),
            ('kaist_stereo_left_pub', self.kaist_stereo_left_pub),
            ('kaist_stereo_right_pub', self.kaist_stereo_right_pub),
            ('kaist_tf_static_pub', self.kaist_tf_static_pub),
            ('kaist_tf_pub', self.kaist_tf_pub),
        ]
        for name, pub in _pubs:
            if pub is not None:
                try:
                    pub.unregister()
                except Exception as e:
                    rospy.logwarn(f'[KAIST] destroy {name}: {e}')
        self.kaist_imu_pub = None
        self.kaist_gps_pub = None
        self.kaist_vrs_pub = None
        self.kaist_vlp_left_pub = None
        self.kaist_vlp_right_pub = None
        self.kaist_sick_back_pub = None
        self.kaist_sick_mid_pub = None
        self.kaist_stereo_left_pub = None
        self.kaist_stereo_right_pub = None
        self.kaist_tf_static_pub = None
        self.kaist_tf_pub = None
        self._kaist_pubs_initialized = False
        rospy.loginfo('KAIST publishers destroyed (DDS queue cleared)')

    def _init_kitti_ros_interfaces(self):
        """KITTI 전용 publisher 초기화 (lazy).

        KITTI 데이터를 처음 로드할 때만 호출.
        ConPR 토픽(/pose, /imu 등)은 생성하지 않는다.
        """
        self._init_common_ros_interfaces()
        if self._kitti_pubs_initialized:
            return

        # queue_size: 대용량 메시지(PC2/Image)는 5, 경량 메시지(IMU/GPS/TF)는 10
        self.kitti_velo_pub = rospy.Publisher(
            KITTI_FILE_PLAYER_PC2_TOPIC, PointCloud2, queue_size=5)

        # 경량 센서 데이터 publishers (IMU, GPS)
        self.kitti_imu_pub     = rospy.Publisher('/kitti/oxts/imu', Imu, queue_size=20)
        self.kitti_gps_fix_pub = rospy.Publisher('/kitti/oxts/gps/fix', NavSatFix, queue_size=10)
        self.kitti_gps_vel_pub = rospy.Publisher('/kitti/oxts/gps/vel', TwistStamped, queue_size=10)

        # 카메라 publishers (4채널 × image + camera_info)
        for cam_id, (_, img_topic, info_topic, _enc) in _KITTI_CAM_ID_MAP.items():
            self.kitti_cam_pubs[cam_id]      = rospy.Publisher(img_topic,  Image,      queue_size=5)
            self.kitti_cam_info_pubs[cam_id] = rospy.Publisher(info_topic, CameraInfo, queue_size=5)
        # 하위호환: kitti_cam_pub → color_left
        self.kitti_cam_pub = self.kitti_cam_pubs.get('02')

        # /tf_static: latch=True → 늦게 subscribe해도 최신 값 수신
        self.kitti_tf_static_pub = rospy.Publisher('/tf_static', TFMessage, queue_size=1, latch=True)
        self.kitti_tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=10)

        self._kitti_pubs_initialized = True
        rospy.loginfo('KITTI File Player publishers initialized')

        # DDS warmup: KITTI velodyne PC2 구독 미리 생성 (첫 프레임 즉시 수신 보장)
        self.pc2_ws_server._presubscribe_pc2(KITTI_FILE_PLAYER_PC2_TOPIC)

    def _init_kaist_ros_interfaces(self):
        """KAIST 전용 publisher 초기화 (lazy).

        KAIST 데이터를 처음 로드할 때만 호출.
        11개 토픽: Imu, NavSatFix×2, PointCloud2×2, LaserScan×2, Image×2, TF×2
        """
        self._init_common_ros_interfaces()
        if self._kaist_pubs_initialized:
            return

        # QoS: 대용량 메시지(PC2/LaserScan/Image)는 depth=5, 경량 메시지(IMU/GPS)는 depth=10~20
        self.kaist_imu_pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=20)
        self.kaist_gps_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)
        self.kaist_vrs_pub = rospy.Publisher('/vrs_gps/fix', NavSatFix, queue_size=10)
        self.kaist_vlp_left_pub = rospy.Publisher(
            KAIST_FILE_PLAYER_PC2_TOPICS[0], PointCloud2, queue_size=5)
        self.kaist_vlp_right_pub = rospy.Publisher(
            KAIST_FILE_PLAYER_PC2_TOPICS[1], PointCloud2, queue_size=5)
        self.kaist_sick_back_pub = rospy.Publisher(
            '/lms511_back/scan', LaserScan, queue_size=5)
        self.kaist_sick_mid_pub = rospy.Publisher(
            '/lms511_middle/scan', LaserScan, queue_size=5)
        self.kaist_stereo_left_pub = rospy.Publisher(
            '/stereo/left/image_raw', Image, queue_size=5)
        self.kaist_stereo_right_pub = rospy.Publisher(
            '/stereo/right/image_raw', Image, queue_size=5)

        self.kaist_tf_static_pub = rospy.Publisher('/tf_static', TFMessage, queue_size=1, latch=True)
        self.kaist_tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=10)

        self._kaist_pubs_initialized = True
        rospy.loginfo('KAIST File Player publishers initialized')

        # DDS warmup: publisher 생성과 동시에 PC2 구독을 미리 생성해 첫 프레임 즉시 수신 보장
        # (브라우저 subscribe 명령이 도달하기 전부터 DDS peer discovery 진행)
        for _topic in KAIST_FILE_PLAYER_PC2_TOPICS:
            self.pc2_ws_server._presubscribe_pc2(_topic)

    # ── 센서 백그라운드 워커 관리 ────────────────────────────────────────────

    def _stop_heavy_sensor_workers(self):
        """모든 중량 센서 publish 워커를 정지한다 (데이터셋 전환 전 호출)."""
        for attr in (
            '_mulran_ouster_worker', '_mulran_radar_worker',
            '_kaist_vlp_left_worker', '_kaist_vlp_right_worker',
            '_kaist_sick_back_worker', '_kaist_sick_mid_worker',
            '_kaist_stereo_worker',
            '_kitti_velo_worker', '_kitti_cam_worker',
            '_conpr_livox_worker', '_conpr_cam_worker',
        ):
            w = getattr(self, attr, None)
            if w is not None:
                w.stop(timeout=0.5)
                setattr(self, attr, None)

    def _start_mulran_workers(self):
        """MulRan 전용 Ouster / Radar 백그라운드 워커를 (재)시작한다."""
        self._mulran_ouster_worker = _SensorPublishWorker()
        self._mulran_ouster_worker._thread.name = 'mulran-ouster'
        self._mulran_radar_worker = _SensorPublishWorker()
        self._mulran_radar_worker._thread.name = 'mulran-radar'

    def _clear_all_sensor_workers(self):
        """모든 센서 워커 큐를 비워 pending 프레임 publish를 취소한다.

        정지/일시정지 직후 백그라운드 워커가 큐에 남은 대용량 프레임을
        publish하면 ROS2/DDS 레이어가 수백 ms 동안 바빠져 HTTP ping 응답이
        지연된다. clear()로 미처리 항목을 버려 이 현상을 방지한다.
        모든 데이터셋(KAIST/KITTI/MulRan/ConPR) 워커를 포함한다.
        """
        for attr in (
            '_kaist_vlp_left_worker', '_kaist_vlp_right_worker',
            '_kaist_sick_back_worker', '_kaist_sick_mid_worker',
            '_kaist_stereo_worker',
            '_kitti_velo_worker', '_kitti_cam_worker',
            '_mulran_ouster_worker', '_mulran_radar_worker',
            '_conpr_livox_worker', '_conpr_cam_worker',
        ):
            worker = getattr(self, attr, None)
            if worker is not None:
                worker.clear()

    def _start_kaist_workers(self):
        """KAIST 전용 VLP / SICK / Stereo 백그라운드 워커를 (재)시작한다."""
        self._kaist_vlp_left_worker = _SensorPublishWorker()
        self._kaist_vlp_left_worker._thread.name = 'kaist-vlp-left'
        self._kaist_vlp_right_worker = _SensorPublishWorker()
        self._kaist_vlp_right_worker._thread.name = 'kaist-vlp-right'
        self._kaist_sick_back_worker = _SensorPublishWorker()
        self._kaist_sick_back_worker._thread.name = 'kaist-sick-back'
        self._kaist_sick_mid_worker = _SensorPublishWorker()
        self._kaist_sick_mid_worker._thread.name = 'kaist-sick-mid'
        self._kaist_stereo_worker = _SensorPublishWorker()
        self._kaist_stereo_worker._thread.name = 'kaist-stereo'

    def _start_kitti_workers(self):
        """KITTI 전용 Velodyne / Camera 백그라운드 워커를 (재)시작한다."""
        self._kitti_velo_worker = _SensorPublishWorker()
        self._kitti_velo_worker._thread.name = 'kitti-velo'
        self._kitti_cam_worker = _SensorPublishWorker()
        self._kitti_cam_worker._thread.name = 'kitti-cam'

    def _start_conpr_workers(self):
        """ConPR 전용 Livox / Camera 백그라운드 워커를 (재)시작한다."""
        self._conpr_livox_worker = _SensorPublishWorker()
        self._conpr_livox_worker._thread.name = 'conpr-livox'
        self._conpr_cam_worker = _SensorPublishWorker()
        self._conpr_cam_worker._thread.name = 'conpr-cam'

    def _init_mulran_ros_interfaces(self):
        """MulRan 전용 publisher 초기화 (lazy)."""
        self._init_common_ros_interfaces()
        if self._mulran_pubs_initialized:
            return

        # QoS: 대용량 메시지(PC2/Image)는 depth=5, 경량 메시지(IMU/GPS)는 depth=10~20
        self.mulran_ouster_pub = rospy.Publisher(
            MULRAN_FILE_PLAYER_PC2_TOPIC, PointCloud2, queue_size=5)
        self.mulran_radar_pub = rospy.Publisher(
            '/radar/polar', Image, queue_size=5)
        self.mulran_imu_pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=20)
        self.mulran_gps_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)
        self.mulran_gt_pub = rospy.Publisher('/gt', Odometry, queue_size=10)
        self.mulran_tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=10)

        self.mulran_tf_static_pub = rospy.Publisher(
            '/tf_static', TFMessage, queue_size=1, latch=True)

        self._mulran_pubs_initialized = True
        rospy.loginfo('MulRan File Player publishers initialized')

        # DDS warmup: MulRan Ouster PC2 구독 미리 생성 (첫 프레임 즉시 수신 보장)
        self.pc2_ws_server._presubscribe_pc2(MULRAN_FILE_PLAYER_PC2_TOPIC)

    def _find_kitti_calib_dir(self, drive_path):
        """드라이브 경로에서 calib 디렉토리를 탐색하여 반환한다.

        KITTI 데이터셋 디렉토리 구조:
          <base>/<date>/<date>_drive_<id>_sync/   ← drive_path
          <base>/<date>/<date>_calib/              ← calib dir (sibling of drive)
          또는
          <base>/<date>_calib/                     ← calib dir (parent 레벨)

        탐색 전략:
          1. drive_path 부모 디렉토리에서 '*_calib' 패턴 항목 탐색 (형제 calib 우선)
          2. drive_path 조부모 디렉토리에서 '*_calib' 패턴 항목 탐색

        Args:
            drive_path (str): KITTI 드라이브 데이터 디렉토리 경로

        Returns:
            str: calib 파일(.txt)이 실제로 존재하는 디렉토리 경로.
                        찾지 못하면 None 반환.
        """
        drive_path = os.path.realpath(drive_path)
        candidates = []

        # 탐색 범위: 부모 → 조부모 → 증조부모 (date 폴더에 calib 형제로 있을 수 있음)
        d = drive_path
        for _ in range(4):
            d = os.path.dirname(d)
            if not d or d == drive_path:
                break
            if os.path.isdir(d):
                try:
                    for entry in sorted(os.listdir(d)):
                        if entry.endswith('_calib') and os.path.isdir(os.path.join(d, entry)):
                            candidates.append(os.path.join(d, entry))
                except OSError:
                    pass

        # 후보 calib 디렉토리에서 실제 calib .txt 파일 유무로 유효성 검사
        for calib_base in candidates:
            # KITTI raw 구조: <date>_calib/<date>/ 하위에 txt가 있을 수 있음
            inner = None
            try:
                for sub in sorted(os.listdir(calib_base)):
                    sub_path = os.path.join(calib_base, sub)
                    if os.path.isdir(sub_path) and glob.glob(os.path.join(sub_path, '*.txt')):
                        inner = sub_path
                        break
            except OSError:
                pass

            # 내부 날짜 서브디렉토리가 있으면 그 쪽을 우선, 없으면 base 자체 검사
            for calib_dir in ([inner, calib_base] if inner else [calib_base]):
                if calib_dir and glob.glob(os.path.join(calib_dir, 'calib_*.txt')):
                    rospy.loginfo(f'KITTI calib dir found: {calib_dir}')
                    return calib_dir

        rospy.logwarn(f'KITTI calib dir not found for drive path: {drive_path}')
        return None

    def _init_slam_subscriber(self):
        """Lazy initialization of SLAM-related subscribers.

        Called once when SLAM mapping is first started via start_slam_mapping().
        This prevents /lt_mapping_complete from appearing in the topic list at startup.
        """
        if self.slam_complete_sub is not None:
            return  # Already initialized

        self.slam_complete_sub = rospy.Subscriber(
            '/lt_mapping_complete', Bool, self.slam_complete_callback, queue_size=10)
        rospy.loginfo('SLAM subscriber (/lt_mapping_complete) initialized')

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
            rospy.logerr(f'Error reading process output: {str(e)}')

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
                rospy.loginfo(f'Stopping {process_name} process (PID: {process.pid})...')

                # Get process group ID
                try:
                    pgid = os.getpgid(process.pid)
                    rospy.loginfo(f'Process group ID: {pgid}')

                    # Send SIGINT (Ctrl+C) to the entire process group
                    os.killpg(pgid, signal.SIGINT)
                    rospy.loginfo('Sent SIGINT to process group')

                    # Wait for process to terminate
                    # Increase timeout for GUI applications like rviz2
                    try:
                        process.wait(timeout=8)  # Increased from 5 to 8 seconds
                        rospy.loginfo(f'{process_name} process terminated gracefully')
                    except subprocess.TimeoutExpired:
                        rospy.logwarn('Process did not terminate with SIGINT, sending SIGTERM')
                        os.killpg(pgid, signal.SIGTERM)
                        try:
                            process.wait(timeout=8)  # Increased from 5 to 8 seconds
                            rospy.loginfo(f'{process_name} process terminated with SIGTERM')
                        except subprocess.TimeoutExpired:
                            rospy.logwarn('Process did not terminate with SIGTERM, sending SIGKILL')
                            os.killpg(pgid, signal.SIGKILL)
                            process.wait(timeout=3)  # Increased from 2 to 3 seconds
                            rospy.loginfo(f'{process_name} process killed with SIGKILL')

                except ProcessLookupError:
                    rospy.logwarn('Process already terminated')
                except Exception as e:
                    rospy.logerr(f'Error during process termination: {str(e)}')
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
                rospy.logwarn(f'No {process_name} process is running')
                return False
        except Exception as e:
            rospy.logerr(f'Failed to stop {process_name} process: {str(e)}')
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
                            rospy.loginfo(f'Killing process matching "{pattern}": PID {pid}')
                            try:
                                os.kill(pid, signal.SIGTERM)
                            except ProcessLookupError:
                                pass
                        break  # Move to next line after finding a match

            time.sleep(0.5)
        except Exception as e:
            rospy.logerr(f'Error killing processes by pattern: {str(e)}')

    # SLAM Functions
    def set_slam_map1(self, path):
        self.slam_map1 = path
        self.slam_status = f"Map 1 loaded - {path}"
        rospy.loginfo(f'Map 1 set to: {path}')

    def set_slam_map2(self, path):
        self.slam_map2 = path
        self.slam_status = f"Map 2 loaded - {path}"
        rospy.loginfo(f'Map 2 set to: {path}')

    def set_slam_output(self, directory_name):
        """Set output directory name (not full path, just directory name)"""
        # Extract just the directory name if a full path is provided
        if '/' in directory_name:
            directory_name = os.path.basename(directory_name.rstrip('/'))

        self.slam_output = directory_name
        self.slam_status = f"Output directory set to - {directory_name}"
        rospy.loginfo(f'Output directory name set to: {directory_name}')

    def start_slam_mapping(self):
        """Start FAST_LIO mapping"""
        rospy.loginfo('=== Starting FAST_LIO SLAM Mapping ===')

        # Ensure SLAM subscriber is ready before launching the process
        self._init_slam_subscriber()

        # Kill any existing SLAM processes first
        self.kill_slam_processes()
        time.sleep(0.5)

        # Launch mapping without capturing terminal output
        try:
            # Phase 4: ROS1 roslaunch 명령으로 변환
            cmd = ['roslaunch', 'fast_lio', 'mapping.launch', 'use_rviz:=false']
            if hasattr(self, '_slam_config_file') and self._slam_config_file:
                cmd.append(f'config_file:={self._slam_config_file}')

            rospy.loginfo('Starting FAST_LIO mapping via roslaunch')
            # stdout/stderr는 UI에 표시하지 않으므로 DEVNULL로 버린다.
            # PIPE로 두면 아무도 drain하지 않아 파이프 버퍼 포화 시 자식
            # 프로세스(fast_lio)의 write()가 주기적으로 블로킹되어 토픽 hz burst 발생.
            self.slam_process = subprocess.Popen(
                cmd,
                env={**os.environ},
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                text=True,
                start_new_session=True
            )

            rospy.loginfo('FAST_LIO mapping started with PID: {}'.format(self.slam_process.pid))
            return True
        except Exception as e:
            rospy.logerr(f'Failed to start FAST_LIO mapping: {str(e)}')
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
        """Start async SLAM map save, return immediately"""
        if self.slam_map_saving:
            return False, 'Map save already in progress'

        # 저장 디렉토리 보관 (결과 뷰어 및 경로 화이트리스트에서 사용)
        self.slam_last_saved_dir = directory

        self.slam_map_saving = True
        self.slam_map_save_cancelled = False
        self.slam_map_save_status = {'saving': True, 'done': False, 'success': None, 'message': 'Initializing...'}

        t = threading.Thread(target=self._save_slam_map_worker, args=(directory,), daemon=True)
        t.start()
        return True, 'Map save started'

    def _save_slam_map_worker(self, directory):
        """Background worker: call save_trajectory service (ROS1) and wait for response"""
        def _finish(success, message):
            self.slam_map_saving = False
            self.slam_map_save_status = {'saving': False, 'done': True, 'success': success, 'message': message}

        try:
            if not SAVEMAP_AVAILABLE:
                _finish(False, 'SaveMap service not available')
                return

            rospy.loginfo(f'Requesting to save SLAM map to directory: {directory}')
            self.slam_map_save_status['message'] = 'Connecting to save_trajectory service...'

            # Phase 4: ROS1 방식 서비스 호출
            rospy.wait_for_service('/save_trajectory', timeout=10.0)
            proxy = rospy.ServiceProxy('/save_trajectory', SaveMap)
            self.slam_map_save_status['message'] = 'Saving map (generating point cloud, removing dynamic objects)...'
            rospy.loginfo('save_trajectory service called. Waiting for response...')

            resp = proxy(directory_name=directory)

            if resp.success:
                rospy.loginfo(f'Map saved successfully: {resp.message}')
                _finish(True, resp.message)
            else:
                rospy.logerr(f'Map save failed: {resp.message}')
                _finish(False, resp.message)

        except rospy.ROSException as e:
            rospy.logerr(f'save_trajectory service not available: {str(e)}')
            _finish(False, f'Service not available: {str(e)}')
        except Exception as e:
            rospy.logerr(f'Failed to save map: {str(e)}')
            import traceback
            traceback.print_exc()
            _finish(False, str(e))

    def cancel_save_slam_map(self):
        """Signal worker to stop"""
        if not self.slam_map_saving:
            return False, 'No map save in progress'

        self.slam_map_save_cancelled = True

        # ROS1: cancel_save_trajectory 서비스 호출 (존재하는 경우)
        try:
            from std_srvs.srv import Trigger
            rospy.wait_for_service('/cancel_save_trajectory', timeout=1.0)
            cancel_proxy = rospy.ServiceProxy('/cancel_save_trajectory', Trigger)
            cancel_proxy()
        except Exception as e:
            rospy.logwarn(f'Failed to call cancel_save_trajectory: {e}')

        return True, 'Cancel signal sent'

    def get_save_map_status(self):
        """Return current save map status"""
        return dict(self.slam_map_save_status)

    def set_slam_config_file(self, config_path):
        """Register the YAML config file path for the next SLAM roslaunch."""
        resolved = resolve_fast_lio_config_path(config_path)
        if not resolved or not os.path.isfile(resolved):
            return False, f'Config file not found: {config_path}'
        self._slam_config_file = resolved
        rospy.loginfo(f'SLAM config file set to: {resolved}')
        return True, resolved

    def set_localization_config_file(self, config_path):
        """Register the YAML config file path for the next localization roslaunch."""
        resolved = resolve_fast_lio_config_path(config_path)
        if not resolved or not os.path.isfile(resolved):
            return False, f'Config file not found: {config_path}'
        self._localization_config_file = resolved
        rospy.loginfo(f'Localization config file set to: {resolved}')
        return True, resolved

    def start_localization_mapping(self):
        """Start Localization mapping process"""
        if self.localization_process and self.localization_process.poll() is None:
            rospy.logwarn('Localization mapping is already running')
            return True

        # Kill any existing Localization processes first
        self.kill_localization_processes()
        time.sleep(0.5)

        # Launch localization without capturing terminal output
        try:
            # Create command with environment setup
            # Phase 4: ROS1 roslaunch 명령으로 변환
            cmd = ['roslaunch', 'fast_lio', 'localization.launch', 'use_rviz:=false']
            if hasattr(self, '_localization_config_file') and self._localization_config_file:
                cmd.append(f'config_file:={self._localization_config_file}')

            rospy.loginfo('Starting FAST_LIO localization via roslaunch')
            # stdout/stderr는 UI에 표시하지 않으므로 DEVNULL로 버린다.
            # PIPE로 두면 아무도 drain하지 않아 파이프 버퍼 포화 시 자식
            # 프로세스(fast_lio)의 write()가 주기적으로 블로킹되어 토픽 hz burst 발생.
            self.localization_process = subprocess.Popen(
                cmd,
                env={**os.environ},
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                text=True,
                start_new_session=True
            )

            rospy.loginfo('FAST_LIO localization started with PID: {}'.format(self.localization_process.pid))
            return True
        except Exception as e:
            rospy.logerr(f'Failed to start FAST_LIO localization: {str(e)}')
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
        self._kill_processes_by_pattern(['localization.launch'])

    def run_slam_optimization(self):
        if not self.slam_map1 or not self.slam_map2:
            self.slam_status = "Error: Please load both Map 1 and Map 2"
            return False, self.slam_status

        if not self.slam_output:
            self.slam_status = "Error: Please set output directory"
            return False, self.slam_status

        if self.slam_opt_running:
            return False, 'Optimization already in progress'

        # 이미 실행 중인 optimization 프로세스가 있으면 종료
        if self.slam_optimization_process and self.slam_optimization_process.poll() is None:
            rospy.loginfo('Stopping existing optimization process before restart')
            self._stop_process(self.slam_optimization_process, 'Long-term Mapping')
            self.slam_optimization_process = None

        self.slam_opt_running = True
        self.slam_opt_status = {'running': True, 'done': False, 'success': None, 'message': 'Initializing...'}
        self.slam_status = "Running Multi-Session Optimization..."
        rospy.loginfo('=== Starting Multi-Session SLAM Optimization ===')
        rospy.loginfo(f'Map 1: {self.slam_map1}')
        rospy.loginfo(f'Map 2: {self.slam_map2}')
        rospy.loginfo(f'Output: {self.slam_output}')

        # 기존 lt_mapper 잔류 프로세스 정리
        self._kill_processes_by_pattern(['lt_mapper.launch.py', 'long_term_mapping'])
        time.sleep(0.5)

        # params.yaml 업데이트
        self.update_slam_parameters()
        time.sleep(0.1)

        try:
            # ROS1: roslaunch 방식
            cmd = ['roslaunch', 'long_term_mapping', 'lt_mapper.launch']

            popen_env = {**os.environ, 'PYTHONUNBUFFERED': '1'}

            self.slam_optimization_process = subprocess.Popen(
                cmd,
                env=popen_env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True
            )

            rospy.loginfo(
                f'Long-term mapping started (PID: {self.slam_optimization_process.pid})')
            self.slam_opt_status['message'] = 'Running optimization...'

            t = threading.Thread(
                target=self._read_optimization_output,
                args=(self.slam_optimization_process,),
                daemon=True
            )
            t.start()
            return True, 'Optimization started'

        except Exception as e:
            self.slam_status = f"Error: Failed to start optimization - {str(e)}"
            self.slam_opt_running = False
            self.slam_opt_status = {'running': False, 'done': True, 'success': False, 'message': str(e)}
            rospy.logerr(f'Failed to start optimization: {str(e)}')
            import traceback
            traceback.print_exc()
            return False, str(e)

    # 메인 터미널 및 UI 상태에 표시할 LTmapping 메시지 화이트리스트
    _OPT_MSG_WHITELIST = (
        'Session Edge Loading Complete.',
        'Place Recognition Complete.',
        'Loop Edge Generation Complete.',
        'Pose Factor loading Complete.',
        'Graph Optimization Complete.',
        'Map Merging Complete.',
        'Map Update Complete.',
        'Long Term SLAM Complete.',
    )

    def _set_opt_success(self, process):
        """LTmapping 노드 완료 확정 (중복 호출 방지)"""
        if self.slam_optimization_process is not process:
            return
        if self.slam_opt_status.get('done'):
            return
        self.slam_status = "Optimization complete!"
        self.slam_opt_status = {'running': False, 'done': True, 'success': True,
                                'message': 'Long Term SLAM Complete.'}
        self.slam_opt_running = False
        rospy.loginfo('=== Long-term mapping completed successfully ===')

    def _read_optimization_output(self, process):
        """long_term_mapping 프로세스 출력을 읽어 화이트리스트 메시지만 get_logger로 포워드.

        완료 감지 우선순위:
        1. ros2 launch가 출력하는 "LTmapping" + "finished cleanly" (노드 정상 종료 이벤트)
        2. "[LTmapping]: Long Term SLAM Complete." 메시지 감지 (메시지 방식 fallback)
        3. process.wait() returncode=0 (전체 launch 프로세스 종료)
        """
        try:
            for line in iter(process.stdout.readline, ''):
                line = line.rstrip('\n')
                if not line:
                    continue

                # ── 우선순위 1: LTmapping 노드 종료 이벤트 감지 ────────────────────────
                # ros2 launch가 출력하는 형태: "[launch]: process[LTmapping-1]: process has finished cleanly"
                if 'LTmapping' in line and 'finished cleanly' in line:
                    rospy.loginfo(f'[launch] LTmapping node finished: {line.strip()}')
                    self._set_opt_success(process)
                    continue

                # ── 우선순위 2: [LTmapping]: 메시지 파싱 ────────────────────────────────
                if '[LTmapping]:' in line:
                    msg_part = line.split('[LTmapping]:')[-1].strip()
                    # 화이트리스트에 있는 메시지만 출력 및 UI 업데이트
                    if any(msg_part.startswith(w) for w in self._OPT_MSG_WHITELIST):
                        rospy.loginfo(f'[LTmapping] {msg_part}')
                        if self.slam_optimization_process is process \
                                and not self.slam_opt_status.get('done'):
                            self.slam_opt_status['message'] = msg_part

                        # "Long Term SLAM Complete." 메시지 fallback 감지
                        if msg_part.startswith('Long Term SLAM Complete.'):
                            self._set_opt_success(process)

            # ── 우선순위 3: 전체 launch 프로세스 종료 시 ────────────────────────────────
            process.wait()
            if self.slam_optimization_process is not process:
                return
            if self.slam_opt_status.get('done'):
                return
            if process.returncode == 0:
                self._set_opt_success(process)
            else:
                msg = f'Process exited with code {process.returncode}'
                self.slam_status = f"Optimization failed (exit: {process.returncode})"
                self.slam_opt_status = {'running': False, 'done': True, 'success': False,
                                        'message': msg}
                rospy.logerr(f'Long-term mapping exited with code: {process.returncode}')
        except Exception as e:
            if self.slam_optimization_process is process:
                rospy.logerr(f'Error reading optimization output: {str(e)}')
                self.slam_opt_status = {'running': False, 'done': True, 'success': False, 'message': str(e)}
        finally:
            self.slam_opt_running = False
            if self.slam_optimization_process is process:
                self.slam_optimization_process = None

    def cancel_optimization(self):
        """Multi-Session Optimization 프로세스 즉시 중단.

        상태를 Cancelled/Exited로 바로 설정하고 HTTP 응답을 즉시 반환한 뒤,
        실제 프로세스 킬은 백그라운드 스레드에서 SIGKILL로 처리한다.

        slam_opt_running이 False여도 (계산 완료 후 RViz가 살아있는 상태)
        slam_optimization_process가 살아있으면 프로세스 그룹을 킬한다.
        """
        proc = self.slam_optimization_process
        if not proc or proc.poll() is not None:
            return False, 'No optimization process running'

        rospy.loginfo('Cancelling optimization by user request')

        # 먼저 참조를 끊어 _read_optimization_output 스레드가 상태를 덮어쓰지 못하게 한다.
        self.slam_optimization_process = None
        self.slam_opt_running = False
        self.slam_opt_status = {'running': False, 'done': True, 'success': False,
                                'message': 'Cancelled by user'}
        self.slam_status = "Optimization cancelled"

        # 프로세스 킬은 백그라운드에서 처리 (HTTP 응답 블로킹 방지)
        def _kill_proc():
            try:
                if proc and proc.poll() is None:
                    pgid = os.getpgid(proc.pid)
                    os.killpg(pgid, signal.SIGKILL)
                    proc.wait(timeout=5)
                    rospy.loginfo('Optimization process killed (SIGKILL)')
            except ProcessLookupError:
                pass
            except Exception as e:
                rospy.logerr(f'Error killing optimization process: {str(e)}')

        threading.Thread(target=_kill_proc, daemon=True).start()
        return True, 'Cancelled'

    def get_optimization_status(self):
        """현재 optimization 상태 반환"""
        return dict(self.slam_opt_status)

    def update_slam_parameters(self):
        lt_dir = _find_sibling_package_dir('long_term_mapping')
        if lt_dir:
            param_file = str(lt_dir / 'config' / 'params.yaml')
        else:
            param_file = "/home/kkw/localization_ws/src/long_term_mapping/config/params.yaml"
            rospy.logwarn('long_term_mapping package not found in workspace; using fallback path')

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

            rospy.loginfo('SLAM parameters updated successfully')
            rospy.loginfo(f'  directory1: {self.slam_map1}')
            rospy.loginfo(f'  directory2: {self.slam_map2}')
            rospy.loginfo(f'  output_directory: {self.slam_output}')
        except Exception as e:
            rospy.logerr(f'Failed to update SLAM parameters: {str(e)}')

    def slam_complete_callback(self, msg):
        if msg.data:
            self.slam_status = "Optimization complete!"
            rospy.loginfo('Optimization completed successfully')

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

    def get_slam_result_paths(self):
        """Return file paths for Multi-Session SLAM result visualization."""
        lt_dir = _find_sibling_package_dir('long_term_mapping')
        output_dir = str(lt_dir / self.slam_output) if (lt_dir and self.slam_output) else ''
        return {
            'success': True,
            'map1_poses': (self.slam_map1 + '/optimized_poses.txt') if self.slam_map1 else '',
            'map2_poses': (self.slam_map2 + '/optimized_poses.txt') if self.slam_map2 else '',
            'output_poses': (output_dir + '/optimized_poses.txt') if output_dir else '',
            'map1_pcd': (output_dir + '/FirstMap.pcd') if output_dir else '',
            'map2_pcd': (output_dir + '/SecondMap.pcd') if output_dir else '',
            'pd_pcd': (output_dir + '/Debug/PD.pcd') if output_dir else '',
            'nd_pcd': (output_dir + '/Debug/ND.pcd') if output_dir else '',
            'first_ue_pcd': (output_dir + '/Debug/FirstUE.pcd') if output_dir else '',
            'second_ue_pcd': (output_dir + '/Debug/SecondUE.pcd') if output_dir else '',
            'output_edges': (output_dir + '/edges.txt') if output_dir else '',
            'output_dir': output_dir,
        }

    def get_save_map_result_paths(self, directory):
        """Return file paths for the LiDAR SLAM Save Map result visualization.

        Args:
            directory (str): The save directory name passed to save_trajectory
                             (pose_graph_optimization/{directory}).

        Returns:
            dict: Existing file paths only (missing files are returned as '').
        """
        pgo_dir = _find_sibling_package_dir('pose_graph_optimization')
        if not pgo_dir or not directory:
            return {'success': False, 'message': 'pose_graph_optimization directory or save directory not found'}

        saved = pgo_dir / directory

        def _path_if_exists(name):
            candidate = saved / name
            return str(candidate) if candidate.is_file() else ''

        return {
            'success': True,
            'optimized_map_pcd': _path_if_exists('OptimizedMap.pcd'),
            'static_map_pcd': _path_if_exists('StaticMap.pcd'),
            'lio_poses': _path_if_exists('odom_poses.txt'),
            'pgo_poses': _path_if_exists('optimized_poses.txt'),
            'edges': _path_if_exists('edges.txt'),
            'output_dir': str(saved),
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
        rospy.loginfo(f'Recorder bag name set to: {bag_name}')
        return True

    def invalidate_ros_topics_list_cache(self):
        """토픽 목록 API 캐시 무효화 (load_data·bag 로드 직후 목록이 바뀔 때)."""
        self._ros_topics_list_cache = None

    def _is_bag_playback_active(self):
        """rosbag play 또는 Ros1BagPlayerThread 재생 중 여부."""
        if self.bag_playing:
            return True
        thread = getattr(self, 'ros1_player_thread', None)
        if thread is None or not thread.is_alive():
            return False
        return thread.get_status().get('status') in ('playing', 'paused')

    def _merge_bag_topic_infos(self, topics, seen):
        """bag 재생 중 master에 아직 없는 토픽을 bag 메타데이터로 보충."""
        if not self._is_bag_playback_active():
            return
        for entry in getattr(self, 'bag_topic_infos', []) or []:
            if not isinstance(entry, dict):
                continue
            name = entry.get('name', '')
            tp = entry.get('type', '')
            key = (name, tp)
            if name and key not in seen:
                seen.add(key)
                topics.append({'name': name, 'type': tp})

    def get_recorder_topics(self):
        """Get list of current ROS1 topics with type information.

        같은 프로세스의 rosgraph Master API로 조회한다 (ROS2 rclpy 그래프와 동일 패턴).
        subprocess rostopic list는 PATH/환경 문제로 빈 목록을 반환할 수 있어 fallback으로만 사용.

        Returns:
            list[dict]: [{'name': '/topic', 'type': 'pkg/Type'}, ...]
        """
        try:
            now = time.monotonic()
            cache = getattr(self, '_ros_topics_list_cache', None)
            ttl = getattr(self, '_ros_topics_list_cache_ttl_sec', 0.75)
            if cache is not None:
                ts, topics = cache
                if (now - ts) < ttl and topics is not None:
                    return topics

            topics = []
            seen = set()

            import rosgraph
            master = rosgraph.Master(rospy.get_name())
            for name, tp in master.getTopicTypes():
                key = (name, tp)
                if key not in seen and name:
                    seen.add(key)
                    topics.append({'name': name, 'type': tp})

            if not topics:
                result = subprocess.run(
                    ['rostopic', 'list', '-v'],
                    capture_output=True, text=True, timeout=3,
                    env={**os.environ},
                )
                for line in result.stdout.split('\n'):
                    line = line.strip()
                    if line.startswith('*'):
                        parts = line.split('[')
                        if len(parts) >= 2:
                            name = parts[0].lstrip('* ').strip()
                            tp = parts[1].split(']')[0].strip()
                            key = (name, tp)
                            if key not in seen and name:
                                seen.add(key)
                                topics.append({'name': name, 'type': tp})

            self._merge_bag_topic_infos(topics, seen)

            self._ros_topics_list_cache = (now, topics)
            rospy.logdebug(f'get_recorder_topics: {len(topics)} (rosgraph master)')
            return topics
        except Exception as e:
            rospy.logerr(f'Error getting topics: {str(e)}')
            return []

    def _player_load_result(
            self, success, message, dataset=None, player_pc2_topics=None):
        """load_data HTTP 응답용.

        player_pc2_topics:
          - list: 이 모드에서 웹 노드가 발행하는 PointCloud2 (UI가 구독 동기화)
          - None: bag 등 자동 동기화 불가 → 클라이언트는 추적 중인 file-player PC2만 해제
        """
        return {
            'success': success,
            'message': message,
            'dataset': dataset,
            'player_pc2_topics': player_pc2_topics,
        }

    def record_bag(self, topics, bag_format='ros2_mcap'):
        """Start or stop bag recording.

        Args:
            topics: 녹화할 토픽 목록. 문자열 리스트 또는 {'name', 'type'} dict 리스트.
            bag_format (str): 'ros2_mcap' | 'ros2_db3' | 'ros1'
                              ros2_mcap - ros2 bag record (mcap, 기본값)
                              ros2_db3  - ros2 bag record -s sqlite3
                              ros1      - Ros1BagRecorderThread (.bag 직접 기록)

        Returns:
            bool: 성공 여부
        """
        if self.recorder_recording:
            # Stop recording — ros1 thread 또는 ros2 subprocess 정리
            if self.recorder_ros1_thread:
                rospy.loginfo('Stopping ROS1 bag recording...')
                self.recorder_ros1_thread.stop()
                self.recorder_ros1_thread = None
            elif self.recorder_process:
                rospy.loginfo('Stopping bag recording...')
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
                rospy.logerr('Bag name not set')
                return False

            if not topics or len(topics) == 0:
                rospy.logerr('No topics selected')
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
                rospy.logerr('No valid topics selected')
                return False

            # ROS1: .bag 파일 직접 녹화
            base_name = self.recorder_bag_name.rstrip('.bag').rstrip('/')
            if PathLib(base_name).is_absolute():
                output_path = f'{base_name}.bag'
                parent_dir = str(PathLib(base_name).parent)
                PathLib(parent_dir).mkdir(parents=True, exist_ok=True)
            else:
                output_path = f'/home/kkw/dataset/{base_name}.bag'

            rospy.loginfo(f'Starting bag recording to: {output_path}')
            rospy.loginfo(f'Recording topics: {", ".join(topic_names)}')

            if topic_type_map:
                # Ros1BagRecorderThread 방식 (타입 정보 있을 때)
                try:
                    self.recorder_ros1_thread = Ros1BagRecorderThread(
                        output_path, topic_type_map, self
                    )
                    self.recorder_ros1_thread.start()
                    self.recorder_recording = True
                    self.recorder_mode = 'ros1'
                    rospy.loginfo('ROS1 bag recording started (thread mode)')
                    return True
                except Exception as e:
                    rospy.logerr(f'Failed to start ROS1 recording: {str(e)}')
                    return False
            else:
                # rosbag record subprocess 방식 (타입 정보 없을 때)
                PathLib(output_path).parent.mkdir(parents=True, exist_ok=True)
                cmd = ['rosbag', 'record', '-O', output_path] + topic_names
                try:
                    self.recorder_process = subprocess.Popen(
                        cmd,
                        env={**os.environ},
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        start_new_session=True
                    )
                    self.recorder_recording = True
                    self.recorder_mode = 'ros1'
                    rospy.loginfo('ROS1 bag recording started (subprocess mode)')
                    return True
                except Exception as e:
                    rospy.logerr(f'Failed to start recording: {str(e)}')
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

    def _is_kaist_dataset_path(self, path: str) -> bool:
        """경로가 KAIST Complex Urban 시퀀스 디렉토리인지 확인한다.

        sensor_data/VLP_left_stamp.csv 또는 sensor_data/data_stamp.csv 존재 여부로 판별.
        """
        if not path or not os.path.isdir(path):
            return False
        sensor_dir = os.path.join(path, 'sensor_data')
        if not os.path.isdir(sensor_dir):
            return False
        vlp_stamp = os.path.join(sensor_dir, 'VLP_left_stamp.csv')
        data_stamp = os.path.join(sensor_dir, 'data_stamp.csv')
        return os.path.isfile(vlp_stamp) or os.path.isfile(data_stamp)

    def _is_mulran_dataset_path(self, path: str) -> bool:
        """MulRan 시퀀스 루트인지 판별한다.

        KAIST도 sensor_data/data_stamp.csv 를 가질 수 있으므로, load_player_data 에서
        KAIST 검사보다 먼저 호출해야 한다. MulRan은 (stamp,sensor) CSV + Ouster/레이더 레이아웃으로 구분한다.
        """
        if not path or not os.path.isdir(path):
            return False
        try:
            from ros_slam_webui.mulran_converter import MulRanConverter
            conv = MulRanConverter()
            sd = conv._find_sensor_data_dir(path)
            if sd is None:
                return False
            stamp_csv = os.path.join(sd, 'data_stamp.csv')
            rows = conv._parse_data_stamp(stamp_csv)
            if not rows:
                return False
            if not (conv._get_ouster_dir(sd, path) or conv._get_radar_polar_dir(sd, path)):
                return False
            return True
        except Exception:
            return False

    def _load_kitti_direct(self, path: str) -> dict:
        """KITTI drive 디렉토리를 직접 File Player로 로드한다.

        velodyne_points/timestamps.txt 에서 타임스탬프를 읽어
        기존 data_stamp 구조(timestamp_ns → frame_idx_str)를 구축한다.
        playback_worker 에서 player_is_kitti 플래그를 보고 KITTI 파일을 직접 읽어 publish.
        """
        from ros_slam_webui.kitti_converter import KittiConverter

        # 기존 재생 정지
        self.player_playing = False
        self.player_paused  = False
        if self.playback_active:
            self.playback_active = False
            old_thread = self.playback_thread
            self.playback_thread = None
            if old_thread and old_thread.is_alive():
                old_thread.join(timeout=1.0)

        # 이전 데이터셋의 중량 센서 워커 정지
        self._stop_heavy_sensor_workers()

        self.player_is_mulran = False
        self.mulran_ctx = None
        self.mulran_events_by_stamp = {}

        ts_file = os.path.join(path, 'velodyne_points', 'timestamps.txt')
        try:
            conv = KittiConverter()
            timestamps_ns = conv._load_timestamps(ts_file)
        except Exception as e:
            rospy.logerr(f'Failed to read KITTI timestamps: {e}')
            return self._player_load_result(
                False, str(e), 'kitti', None)

        if not timestamps_ns:
            rospy.logerr(f'No valid timestamps in {ts_file}')
            return self._player_load_result(
                False, f'No valid timestamps in {ts_file}', 'kitti', None)

        # data_stamp: {timestamp_ns: frame_index_str}
        self.data_stamp = {}
        for idx, ts_ns in enumerate(timestamps_ns):
            if ts_ns > 0:
                self.data_stamp[ts_ns] = f'{idx:010d}'

        if not self.data_stamp:
            rospy.logerr('data_stamp is empty after parsing KITTI timestamps')
            return self._player_load_result(
                False, 'data_stamp empty', 'kitti', None)

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
        self.kitti_static_tf_msg = None  # 재생 중 /tf_static 주기 재발행용

        self.livox_cache = {}
        self.cam_cache   = {}

        # 카메라 방향 로드 시 1회 감지 → 재생 중 os.path.isfile() 제거
        self.kitti_cam_dirs_map = {}
        self.kitti_calib_cam_to_cam = None
        for cam_id, (img_dir_name, _, _, _enc) in _KITTI_CAM_ID_MAP.items():
            _cam_data_dir = os.path.join(path, img_dir_name, 'data')
            if os.path.isdir(_cam_data_dir):
                self.kitti_cam_dirs_map[cam_id] = _cam_data_dir
        # 하위호환 단일 카메라 필드
        if '02' in self.kitti_cam_dirs_map:
            self.kitti_cam_dir = self.kitti_cam_dirs_map['02']
            self.kitti_cam_encoding = 'bgr8'
        elif '00' in self.kitti_cam_dirs_map:
            self.kitti_cam_dir = self.kitti_cam_dirs_map['00']
            self.kitti_cam_encoding = 'mono8'
        else:
            self.kitti_cam_dir = ''
            self.kitti_cam_encoding = ''

        # KITTI 전용 publisher만 초기화 (ConPR 토픽 오염 방지)
        self._init_kitti_ros_interfaces()
        # KITTI Velodyne / Camera 백그라운드 워커 시작
        self._start_kitti_workers()

        # ── calib 디렉토리 탐색 → Static TF 1회 publish ──────────────────────
        # kitti_calib_dir / oxts 상태 초기화 (재로드 대비)
        self.kitti_calib_dir = None
        self.kitti_oxts_files = []
        self.kitti_oxts_timestamps = []
        self.kitti_origin_oxts = None
        self.kitti_mercator_scale = None

        calib_dir = self._find_kitti_calib_dir(path)
        if calib_dir:
            self.kitti_calib_dir = calib_dir
            try:
                calib_imu_to_velo = conv._parse_calib_file(
                    os.path.join(calib_dir, 'calib_imu_to_velo.txt'))
                calib_velo_to_cam = conv._parse_calib_file(
                    os.path.join(calib_dir, 'calib_velo_to_cam.txt'))
                calib_cam_to_cam = conv._parse_calib_file(
                    os.path.join(calib_dir, 'calib_cam_to_cam.txt'))
                self.kitti_calib_cam_to_cam = calib_cam_to_cam  # camera_info 발행용 저장
                # stamp: 현재 시각 사용
                stamp = rospy.Time.now()
                static_tf_msg = conv._build_static_tf(
                    calib_imu_to_velo, calib_velo_to_cam,
                    calib_cam_to_cam=calib_cam_to_cam,
                    stamp=stamp,
                )
                # transient_local QoS 덕분에 늦게 subscribe해도 수신됨
                if self.kitti_tf_static_pub and static_tf_msg:
                    self.kitti_static_tf_msg = static_tf_msg
                    self.kitti_tf_static_pub.publish(static_tf_msg)
                    n_tf = len(static_tf_msg.transforms) if static_tf_msg.transforms else 0
                    dbg = ''
                    if n_tf >= 2:
                        t = static_tf_msg.transforms[1].transform.translation
                        dbg = f' imu→velo=({t.x:.2f},{t.y:.2f},{t.z:.2f})'
                    rospy.loginfo(
                        f'KITTI static TF from {calib_dir}: {n_tf} transforms{dbg}')
            except Exception as e:
                rospy.logwarn(f'KITTI static TF publish failed: {e}')
        else:
            rospy.logwarn(f'KITTI calib directory not found near: {path}')
            # calib 없어도 velo_link 연결을 위해 identity chain 생성
            try:
                static_tf_msg = conv._build_static_tf({}, {}, stamp=None)
                if static_tf_msg and static_tf_msg.transforms:
                    self.kitti_static_tf_msg = static_tf_msg
                    rospy.loginfo('KITTI fallback identity TF chain created')
            except Exception:
                pass

        # ── oxts 파일 목록 + 타임스탬프 파싱 + Mercator 원점 계산 ─────────────
        oxts_dir = os.path.join(path, 'oxts')
        oxts_ts_file = os.path.join(oxts_dir, 'timestamps.txt')
        if os.path.isfile(oxts_ts_file):
            try:
                self.kitti_oxts_timestamps = conv._load_timestamps(oxts_ts_file)
                oxts_data_dir = os.path.join(oxts_dir, 'data')
                if os.path.isdir(oxts_data_dir):
                    self.kitti_oxts_files = sorted(
                        glob.glob(os.path.join(oxts_data_dir, '*.txt')))
                    # Mercator 원점: 첫 번째 OXTS 데이터 기준
                    if self.kitti_oxts_files:
                        first_oxts = conv._load_oxts_file(self.kitti_oxts_files[0])
                        if first_oxts:
                            self.kitti_origin_oxts = first_oxts
                            self.kitti_mercator_scale = math.cos(
                                math.radians(first_oxts[0]))
                            rospy.loginfo(
                                f'KITTI oxts loaded: {len(self.kitti_oxts_files)} files, '
                                f'origin lat={first_oxts[0]:.4f}'
                            )
            except Exception as e:
                rospy.logwarn(f'KITTI oxts parsing failed: {e}')
        else:
            rospy.logwarn(f'KITTI oxts timestamps not found: {oxts_ts_file}')

        self.player_data_loaded = True

        rospy.loginfo(
            f'KITTI drive loaded: {path} '
            f'({len(self.data_stamp)} frames, '
            f'{(sorted_stamps[-1] - sorted_stamps[0]) / 1e9:.1f}s)'
        )
        return self._player_load_result(
            True, 'KITTI loaded', 'kitti', [KITTI_FILE_PLAYER_PC2_TOPIC])

    # ── KITTI 백그라운드 워커 헬퍼 ────────────────────────────────────────────

    def _kitti_do_velo(self, bin_path: str, stamp_msg, pub, conv):
        """백그라운드: KITTI Velodyne 바이너리 파일 읽기 + publish."""
        try:
            pc2_msg = conv._make_pointcloud2_msg(bin_path, stamp_msg)
            if pc2_msg and pub:
                pub.publish(pc2_msg)
        except Exception:
            pass

    def _kitti_do_cam(self, cam_dirs_map: dict, cam_pubs: dict, cam_info_pubs: dict,
                      calib_cam_to_cam, frame_idx: int, stamp_msg, conv):
        """백그라운드: KITTI 카메라 이미지(최대 4채널) 읽기 + publish."""
        for cam_id, cam_dir in cam_dirs_map.items():
            img_pub = cam_pubs.get(cam_id)
            if not img_pub:
                continue
            _, _, _, encoding = _KITTI_CAM_ID_MAP[cam_id]
            img_path = os.path.join(cam_dir, f'{frame_idx:010d}.png')
            try:
                img_msg = conv._make_image_msg(img_path, encoding, stamp_msg)
                if img_msg:
                    img_pub.publish(img_msg)
            except Exception:
                pass
            info_pub = cam_info_pubs.get(cam_id)
            if info_pub and calib_cam_to_cam:
                try:
                    info_msg = conv._make_camera_info_msg(calib_cam_to_cam, cam_id, stamp_msg)
                    if info_msg:
                        info_pub.publish(info_msg)
                except Exception:
                    pass

    def _publish_kitti_frame(self, frame_idx: int, stamp_ns: int):
        """KITTI 프레임(velodyne + IMU/GPS + camera + TF)을 ROS2 토픽으로 publish한다.

        Velodyne PC2 읽기(~4MB)와 Camera PNG 읽기(최대 4채널)는 백그라운드 워커로
        비블로킹 처리 (OusterThread 패턴). OXTS(경량 텍스트)와 TF는 동기 처리.
        """
        drive_path = self.kitti_drive_path
        if not drive_path:
            return

        if self._kitti_conv is None:
            from ros_slam_webui.kitti_converter import KittiConverter
            self._kitti_conv = KittiConverter()
        conv = self._kitti_conv

        stamp_msg = rospy.Time(int(stamp_ns // 1_000_000_000), int(stamp_ns % 1_000_000_000))

        # ── Velodyne PointCloud2 → 백그라운드 워커 ──────────────────────────
        bin_path = os.path.join(
            drive_path, 'velodyne_points', 'data', f'{frame_idx:010d}.bin')
        if self._kitti_velo_worker:
            self._kitti_velo_worker.push(
                self._kitti_do_velo, bin_path, stamp_msg, self.kitti_velo_pub, conv)
        else:
            self._kitti_do_velo(bin_path, stamp_msg, self.kitti_velo_pub, conv)

        # ── OXTS: IMU + GPS fix + GPS vel (경량 텍스트, 동기 처리) ──────────
        # current_oxts는 아래 TF에서도 재사용
        current_oxts = None
        if self.kitti_oxts_files and frame_idx < len(self.kitti_oxts_files):
            try:
                current_oxts = conv._load_oxts_file(self.kitti_oxts_files[frame_idx])
            except Exception:
                pass

        if current_oxts:
            try:
                if self.kitti_imu_pub:
                    self.kitti_imu_pub.publish(conv._make_imu_msg(current_oxts, stamp_msg))
                if self.kitti_gps_fix_pub:
                    self.kitti_gps_fix_pub.publish(
                        conv._make_navsatfix_msg(current_oxts, stamp_msg))
                if self.kitti_gps_vel_pub:
                    self.kitti_gps_vel_pub.publish(
                        conv._make_twist_stamped_msg(current_oxts, stamp_msg))
            except Exception:
                pass

        # ── Camera images + camera_info → 백그라운드 워커 ───────────────────
        if self.kitti_cam_dirs_map:
            if self._kitti_cam_worker:
                self._kitti_cam_worker.push(
                    self._kitti_do_cam,
                    dict(self.kitti_cam_dirs_map),
                    self.kitti_cam_pubs,
                    self.kitti_cam_info_pubs,
                    self.kitti_calib_cam_to_cam,
                    frame_idx,
                    stamp_msg,
                    conv,
                )
            else:
                self._kitti_do_cam(
                    self.kitti_cam_dirs_map, self.kitti_cam_pubs,
                    self.kitti_cam_info_pubs, self.kitti_calib_cam_to_cam,
                    frame_idx, stamp_msg, conv)

        # ── TF: /tf에 dynamic + static 통합 발행 (동기, 경량) ───────────────
        if self.kitti_tf_pub:
            all_transforms = []
            if (current_oxts and self.kitti_origin_oxts and self.kitti_mercator_scale):
                try:
                    dyn_msg = conv._make_dynamic_tf(
                        current_oxts, self.kitti_origin_oxts,
                        self.kitti_mercator_scale, stamp_msg
                    )
                    all_transforms.extend(dyn_msg.transforms)
                except Exception:
                    pass
            if self.kitti_static_tf_msg and self.kitti_static_tf_msg.transforms:
                for t in self.kitti_static_tf_msg.transforms:
                    t_copy = TransformStamped()
                    t_copy.header.stamp = stamp_msg
                    t_copy.header.frame_id = t.header.frame_id
                    t_copy.child_frame_id = t.child_frame_id
                    t_copy.transform = t.transform
                    all_transforms.append(t_copy)
            if all_transforms:
                from tf2_msgs.msg import TFMessage
                tf_msg = TFMessage()
                tf_msg.transforms = all_transforms
                self.kitti_tf_pub.publish(tf_msg)

    def _load_kaist_direct(self, path: str) -> dict:
        """KAIST 시퀀스 디렉토리를 직접 File Player로 로드한다.

        VLP_left_stamp.csv 또는 data_stamp.csv에서 타임스탬프를 읽어
        data_stamp 구조(stamp_ns → frame_idx_str)를 구축한다.
        global_pose.csv, xsens_imu.csv, gps.csv, vrs_gps.csv 사전 로드.
        /tf_static 1회 publish.
        """
        from ros_slam_webui.kaist_converter import KaistConverter

        # 기존 재생 정지
        self.player_playing = False
        self.player_paused = False
        if self.playback_active:
            self.playback_active = False
            old_thread = self.playback_thread
            self.playback_thread = None
            if old_thread and old_thread.is_alive():
                old_thread.join(timeout=1.0)

        # 이전 데이터셋의 중량 센서 워커 정지
        self._stop_heavy_sensor_workers()

        self.player_is_mulran = False
        self.mulran_ctx = None
        self.mulran_events_by_stamp = {}

        sensor_dir = os.path.join(path, 'sensor_data')
        calib_dir = os.path.join(path, 'calibration')
        pose_csv = os.path.join(path, 'global_pose.csv')

        try:
            conv = KaistConverter()
        except Exception as e:
            rospy.logerr(f'Failed to import KaistConverter: {e}')
            return self._player_load_result(
                False, str(e), 'kaist', None)

        # 마스터 타임라인: 모든 센서의 stamp 병합 (VLP Left/Right, IMU, GPS, SICK, global_pose 등)
        all_stamps = set()

        # ── VLP Left ─────────────────────────────────────────────────────────
        vlp_left_stamps = conv._load_stamp_csv(
            os.path.join(sensor_dir, 'VLP_left_stamp.csv'))
        if not vlp_left_stamps:
            vlp_left_stamps = conv._load_stamp_csv(
                os.path.join(sensor_dir, 'data_stamp.csv'))
        _vlp_left_set = set(ts for ts in vlp_left_stamps if ts > 0)
        all_stamps.update(_vlp_left_set)
        self.kaist_vlp_left_stamps = _vlp_left_set
        self.kaist_vlp_left_dir = os.path.join(sensor_dir, 'VLP_left')

        # ── VLP Right ────────────────────────────────────────────────────────
        _vlp_right_found_dir = ''
        for _sub in ('VLP_right', 'vlp_right'):
            _d = os.path.join(sensor_dir, _sub)
            if os.path.isdir(_d):
                _vlp_right_found_dir = _d
                break
        vlp_right_stamps = conv._load_stamp_csv(
            os.path.join(sensor_dir, 'VLP_right_stamp.csv'))
        if not vlp_right_stamps and _vlp_right_found_dir:
            from pathlib import Path
            vlp_right_stamps = [
                int(f.stem) for f in sorted(Path(_vlp_right_found_dir).glob('*.bin'))
                if f.stem.isdigit()
            ]
        _vlp_right_set = set(ts for ts in vlp_right_stamps if ts > 0)
        all_stamps.update(_vlp_right_set)
        self.kaist_vlp_right_stamps = _vlp_right_set
        self.kaist_vlp_right_dir = _vlp_right_found_dir

        # ── SICK Back ────────────────────────────────────────────────────────
        _sick_back_found_dir = ''
        _sick_back_stamps_list: list = []
        for sick_sub in ('SICK_back', 'lms511_back'):
            sick_back_dir = os.path.join(sensor_dir, sick_sub)
            stamp_file = os.path.join(sensor_dir, f'{sick_sub.replace("lms511", "SICK")}_stamp.csv')
            sick_back_stamps = conv._load_stamp_csv(stamp_file)
            if not sick_back_stamps and os.path.isdir(sick_back_dir):
                from pathlib import Path
                sick_back_stamps = [
                    int(f.stem) for f in sorted(Path(sick_back_dir).glob('*.bin'))
                    if f.stem.isdigit()
                ]
                _sick_back_found_dir = sick_back_dir
                _sick_back_stamps_list = sick_back_stamps
                break
            if sick_back_stamps:
                _sick_back_found_dir = sick_back_dir
                _sick_back_stamps_list = sick_back_stamps
                break
        _sick_back_set = set(ts for ts in _sick_back_stamps_list if ts > 0)
        all_stamps.update(_sick_back_set)
        self.kaist_sick_back_stamps = _sick_back_set
        self.kaist_sick_back_dir = _sick_back_found_dir

        # ── SICK Middle ──────────────────────────────────────────────────────
        _sick_mid_found_dir = ''
        _sick_mid_stamps_list: list = []
        for sick_sub in ('SICK_middle', 'lms511_middle'):
            sick_mid_dir = os.path.join(sensor_dir, sick_sub)
            stamp_file = os.path.join(sensor_dir, f'{sick_sub.replace("lms511", "SICK")}_stamp.csv')
            sick_mid_stamps = conv._load_stamp_csv(stamp_file)
            if not sick_mid_stamps and os.path.isdir(sick_mid_dir):
                from pathlib import Path
                sick_mid_stamps = [
                    int(f.stem) for f in sorted(Path(sick_mid_dir).glob('*.bin'))
                    if f.stem.isdigit()
                ]
                _sick_mid_found_dir = sick_mid_dir
                _sick_mid_stamps_list = sick_mid_stamps
                break
            if sick_mid_stamps:
                _sick_mid_found_dir = sick_mid_dir
                _sick_mid_stamps_list = sick_mid_stamps
                break
        _sick_mid_set = set(ts for ts in _sick_mid_stamps_list if ts > 0)
        all_stamps.update(_sick_mid_set)
        self.kaist_sick_mid_stamps = _sick_mid_set
        self.kaist_sick_mid_dir = _sick_mid_found_dir

        # ── Global Pose ──────────────────────────────────────────────────────
        self.kaist_global_poses = conv._parse_global_pose(pose_csv)
        _pose_set = set(p[0] for p in self.kaist_global_poses if p[0] > 0)
        all_stamps.update(_pose_set)
        self.kaist_pose_stamp_set = _pose_set
        self.kaist_pose_stamps_sorted = sorted(_pose_set)

        # ── IMU ──────────────────────────────────────────────────────────────
        imu_file = os.path.join(sensor_dir, 'xsens_imu.csv')
        if not os.path.exists(imu_file):
            imu_file = os.path.join(sensor_dir, 'imu.csv')
        imu_rows = conv._load_kaist_imu_csv(imu_file)
        imu_rows.sort(key=lambda r: r.get('stamp', 0))
        self.kaist_imu_data = ([r['stamp'] for r in imu_rows], imu_rows)
        _imu_set = set(self.kaist_imu_data[0])
        all_stamps.update(_imu_set)
        self.kaist_imu_stamps = _imu_set

        # ── GPS ──────────────────────────────────────────────────────────────
        gps_rows = conv._load_kaist_gps_csv(os.path.join(sensor_dir, 'gps.csv'))
        gps_rows.sort(key=lambda r: r.get('stamp', 0))
        self.kaist_gps_data = ([r['stamp'] for r in gps_rows], gps_rows)
        _gps_set = set(self.kaist_gps_data[0])
        all_stamps.update(_gps_set)
        self.kaist_gps_stamps = _gps_set

        # ── VRS GPS ──────────────────────────────────────────────────────────
        vrs_file = os.path.join(sensor_dir, 'vrs_gps.csv')
        vrs_rows = conv._load_kaist_gps_csv(vrs_file) if os.path.exists(vrs_file) else []
        vrs_rows.sort(key=lambda r: r.get('stamp', 0))
        self.kaist_vrs_data = ([r['stamp'] for r in vrs_rows], vrs_rows)
        _vrs_set = set(self.kaist_vrs_data[0])
        all_stamps.update(_vrs_set)
        self.kaist_vrs_stamps = _vrs_set

        # ── Stereo ───────────────────────────────────────────────────────────
        # image/stereo_left/ 디렉토리가 실제로 존재할 때만 로드 (CSV만 있고 이미지 없는 경우 스킵)
        _stereo_left_dir = os.path.join(sensor_dir, 'image', 'stereo_left')
        _stereo_right_dir = os.path.join(sensor_dir, 'image', 'stereo_right')
        if os.path.isdir(_stereo_left_dir):
            stereo_stamps = conv._load_stamp_csv(os.path.join(sensor_dir, 'stereo_stamp.csv'))
            if not stereo_stamps:
                from pathlib import Path
                stereo_stamps = [
                    int(f.stem) for f in sorted(Path(_stereo_left_dir).glob('*.png'))
                    if f.stem.isdigit()
                ]
        else:
            stereo_stamps = []
            rospy.logdebug(
                f'KAIST stereo: image/stereo_left/ not found in {sensor_dir}, skipping stereo stamps')
        _stereo_set = set(ts for ts in stereo_stamps if ts > 0)
        all_stamps.update(_stereo_set)
        self.kaist_stereo_stamps = _stereo_set
        self.kaist_stereo_left_dir = _stereo_left_dir
        self.kaist_stereo_right_dir = _stereo_right_dir

        if not all_stamps:
            rospy.logerr('KAIST: No valid timestamps from any sensor')
            return self._player_load_result(
                False, 'No valid timestamps', 'kaist', None)

        # data_stamp: {timestamp_ns: str(timestamp_ns)} — KAIST bin 파일명이 stamp.bin
        self.data_stamp = {ts_ns: str(ts_ns) for ts_ns in all_stamps if ts_ns > 0}

        if not self.data_stamp:
            rospy.logerr('data_stamp is empty after parsing KAIST timestamps')
            return self._player_load_result(
                False, 'data_stamp empty', 'kaist', None)

        sorted_stamps = sorted(self.data_stamp.keys())
        self.player_initial_stamp = sorted_stamps[0]
        self.player_last_stamp = sorted_stamps[-1]
        self.player_timestamp = self.player_initial_stamp
        self.player_processed_stamp = 0
        self.player_slider_pos = 0
        self.player_seek_requested = False
        self.player_seek_to_stamp = self.player_initial_stamp

        self.player_path = path
        self.player_is_kitti = False   # KAIST 모드 진입 시 KITTI 해제
        self.player_is_kaist = True
        self.player_is_ros2_bag = False
        self.kaist_dataset_path = path

        self.livox_cache = {}
        self.cam_cache = {}

        self._init_kaist_ros_interfaces()
        # 중량 센서 백그라운드 워커 (재)시작 (VLP/SICK/Stereo 파일 I/O 비동기화)
        self._start_kaist_workers()

        # Stereo image DDS warmup: 브라우저 subscribe 전에 미리 구독 생성
        # (_init_kaist_ros_interfaces 내부의 _kaist_pubs_initialized 체크와 무관하게 매 로드마다 실행)
        if self.kaist_stereo_stamps:
            for _stereo_topic in ('/stereo/left/image_raw', '/stereo/right/image_raw'):
                self.pc2_ws_server._presubscribe_image(_stereo_topic)

        # global_pose, imu, gps, vrs는 위 마스터 타임라인 구축 시 이미 로드됨

        # Static TF 1회 publish
        if os.path.isdir(calib_dir):
            try:
                stamp_time = conv._ns_to_time_msg(sorted_stamps[0])
                static_tf_msg = conv._build_static_tf(calib_dir, stamp_time)
                if static_tf_msg and self.kaist_tf_static_pub:
                    self.kaist_tf_static_pub.publish(static_tf_msg)
                    self.kaist_static_tf_msg = static_tf_msg  # 매 프레임 /tf 재발행용 저장
                    rospy.loginfo(f'KAIST static TF published from: {calib_dir}')
            except Exception as e:
                rospy.logwarn(f'KAIST static TF publish failed: {e}')
        else:
            rospy.logwarn(f'KAIST calibration directory not found: {calib_dir}')

        self.player_data_loaded = True

        rospy.loginfo(
            f'KAIST sequence loaded: {path} '
            f'({len(self.data_stamp)} frames, '
            f'{(sorted_stamps[-1] - sorted_stamps[0]) / 1e9:.1f}s)'
        )
        return self._player_load_result(
            True, 'KAIST loaded', 'kaist', list(KAIST_FILE_PLAYER_PC2_TOPICS))

    def _load_mulran_direct(self, path: str) -> dict:
        """MulRan 시퀀스를 File Player로 직접 로드한다 (data_stamp.csv 타임라인)."""
        from ros_slam_webui.mulran_converter import MulRanConverter

        self.player_playing = False
        self.player_paused = False
        if self.playback_active:
            self.playback_active = False
            old_thread = self.playback_thread
            self.playback_thread = None
            if old_thread and old_thread.is_alive():
                old_thread.join(timeout=1.0)

        # 이전 데이터셋의 중량 센서 워커 정지
        self._stop_heavy_sensor_workers()

        try:
            conv = MulRanConverter()
            ctx = conv._load_sequence_context(path)
        except Exception as e:
            rospy.logerr(f'MulRan load failed: {e}')
            return self._player_load_result(False, str(e), 'mulran', None)

        if not ctx['data_stamps']:
            return self._player_load_result(
                False, 'No data_stamp entries', 'mulran', None)

        events_by_stamp = {}
        for stamp_ns, sensor_name in ctx['data_stamps']:
            events_by_stamp.setdefault(stamp_ns, []).append(sensor_name)

        sorted_stamps = sorted(events_by_stamp.keys())
        self.data_stamp = {s: 'mulran' for s in sorted_stamps}
        self.mulran_events_by_stamp = events_by_stamp
        self.mulran_ctx = ctx
        self.mulran_dataset_path = path

        self.player_initial_stamp = sorted_stamps[0]
        self.player_last_stamp = sorted_stamps[-1]
        self.player_timestamp = self.player_initial_stamp
        self.player_processed_stamp = 0
        self.player_slider_pos = 0
        self.player_seek_requested = False
        self.player_seek_to_stamp = self.player_initial_stamp

        self.player_path = path
        self.player_is_kitti = False
        self.player_is_kaist = False
        self.player_is_mulran = True
        self.player_is_ros2_bag = False
        self.player_is_ros1_bag = False

        self.livox_cache = {}
        self.cam_cache = {}
        self._mulran_last_clock_pub_ns = None

        self._init_mulran_ros_interfaces()
        # 중량 센서 백그라운드 워커 (재)시작 (Ouster/Radar 파일 I/O 비동기화)
        self._start_mulran_workers()

        stamp0 = conv._ns_to_time_msg(sorted_stamps[0])
        tf_static_msg = conv.build_mulran_tf_static_message(
            stamp0,
            ctx.get('calib_ouster_xyz_rpy'),
            ctx.get('calib_radar_xyz_rpy'),
        )
        if tf_static_msg and getattr(self, 'mulran_tf_static_pub', None):
            self.mulran_tf_static_pub.publish(tf_static_msg)
            self.mulran_static_tf_msg = tf_static_msg   # 매 프레임 /tf 재발행용 저장
            rospy.loginfo(
                'MulRan /tf_static published: base_link → ouster, radar_polar (고정 외장 상수)')

        self.player_data_loaded = True
        rospy.loginfo(
            f'MulRan sequence loaded: {path} ({len(self.data_stamp)} timeline stamps)'
        )
        pc2_topics = [MULRAN_FILE_PLAYER_PC2_TOPIC] if ctx.get('ouster_dir') else []
        return self._player_load_result(
            True, 'MulRan loaded', 'mulran', pc2_topics if pc2_topics else None)

    def _mulran_do_ouster(self, bin_path: str, stamp_time, ouster_pub, conv):
        """백그라운드 스레드: Ouster 바이너리 파일 읽기 + publish."""
        msg = conv._make_ouster_pc2(bin_path, stamp_time)
        if msg and ouster_pub:
            ouster_pub.publish(msg)

    def _mulran_do_radar(self, png_path: str, stamp_time, radar_pub, conv):
        """백그라운드 스레드: Radar PNG 파일 읽기 + publish."""
        msg = conv._make_radar_image(png_path, stamp_time)
        if msg and radar_pub:
            radar_pub.publish(msg)

    def _publish_mulran_frame(self, stamp_ns: int):
        """MulRan data_stamp 한 시각의 센서 이벤트를 publish.

        레퍼런스 ROSThread.cpp 패턴:
        - Ouster / Radar: 백그라운드 워커(_SensorPublishWorker)에 위임 (파일 I/O 비블록)
        - IMU / GPS: 비트맵 조회만 하므로 인라인으로 즉시 publish
        - TF: 루프 밖에서 한 번만 publish (이전 코드에서 매 센서 이벤트마다 반복 발행하던 비효율 제거)
        """
        ctx = self.mulran_ctx
        if not ctx:
            return
        if self._mulran_conv is None:
            from ros_slam_webui.mulran_converter import MulRanConverter
            self._mulran_conv = MulRanConverter()
        conv = self._mulran_conv
        stamp_time = conv._ns_to_time_msg(stamp_ns)

        has_gt_event = False  # GT/Ouster 이벤트가 있을 때만 TF publish

        for sensor_name in self.mulran_events_by_stamp.get(stamp_ns, []):
            sn = sensor_name.lower()
            if sn == 'ouster' and ctx['ouster_dir'] and self.mulran_ouster_pub:
                bin_path = os.path.join(ctx['ouster_dir'], f'{stamp_ns}.bin')
                if self._mulran_ouster_worker:
                    self._mulran_ouster_worker.push(
                        self._mulran_do_ouster, bin_path, stamp_time,
                        self.mulran_ouster_pub, conv)
                else:
                    # fallback: 워커 없으면 동기 처리
                    self._mulran_do_ouster(bin_path, stamp_time, self.mulran_ouster_pub, conv)
                has_gt_event = True
            elif sn == 'radar' and ctx['radar_dir'] and self.mulran_radar_pub:
                png_path = os.path.join(ctx['radar_dir'], f'{stamp_ns}.png')
                if self._mulran_radar_worker:
                    self._mulran_radar_worker.push(
                        self._mulran_do_radar, png_path, stamp_time,
                        self.mulran_radar_pub, conv)
                else:
                    self._mulran_do_radar(png_path, stamp_time, self.mulran_radar_pub, conv)
            elif sn == 'imu' and ctx['imu_bisect'][0] and self.mulran_imu_pub:
                row = conv._find_nearest(ctx['imu_bisect'], stamp_ns)
                if row:
                    imu_msg = conv._make_imu_msg(row, stamp_time, ctx['imu_version'])
                    self.mulran_imu_pub.publish(imu_msg)
            elif sn == 'gps' and ctx['gps_bisect'][0] and self.mulran_gps_pub:
                row = conv._find_nearest(ctx['gps_bisect'], stamp_ns)
                if row:
                    gps_msg = conv._make_navsatfix_msg(row, stamp_time)
                    self.mulran_gps_pub.publish(gps_msg)
                has_gt_event = True

        # TF: 루프 밖에서 한 번만 publish (이전에는 매 센서 이벤트마다 반복 → 100Hz TF 발행 낭비)
        # Ouster(10Hz) 또는 GPS(10Hz) 이벤트 시에만 발행해 최대 20Hz로 제한
        if has_gt_event and self.mulran_tf_pub:
            all_tfs = []
            if ctx['global_poses']:
                pose = conv._find_nearest_pose(
                    ctx['pose_stamps'], ctx['global_poses'], stamp_ns)
                if pose:
                    _, R, T = pose
                    if self.mulran_gt_pub:
                        odom_msg = conv._make_gt_odometry(R, T, stamp_time)
                        if odom_msg:
                            self.mulran_gt_pub.publish(odom_msg)
                    dyn_tf = conv._make_dynamic_tf(R, T, stamp_time)
                    if dyn_tf:
                        all_tfs.extend(dyn_tf.transforms)
            # Static TF — 매 프레임 /tf에 포함 (rosbridge TRANSIENT_LOCAL 수신 불안정 대응)
            if self.mulran_static_tf_msg:
                for t in self.mulran_static_tf_msg.transforms:
                    t_copy = TransformStamped()
                    t_copy.header.stamp = stamp_time
                    t_copy.header.frame_id = t.header.frame_id
                    t_copy.child_frame_id = t.child_frame_id
                    t_copy.transform = t.transform
                    all_tfs.append(t_copy)
            if all_tfs:
                combined_tf = TFMessage()
                combined_tf.transforms = all_tfs
                self.mulran_tf_pub.publish(combined_tf)

        if self.clock_pub:
            last = self._mulran_last_clock_pub_ns
            if last is None or (stamp_ns - last) >= _MULRAN_CLOCK_MIN_INTERVAL_NS:
                self._mulran_last_clock_pub_ns = stamp_ns
                clock_msg = Clock()
                clock_msg.clock = rospy.Time(stamp_ns // 10**9, stamp_ns % 10**9)
                self.clock_pub.publish(clock_msg)

    # ── KAIST 백그라운드 워커 함수 ─────────────────────────────────────────────

    def _kaist_do_vlp(self, bin_path: str, frame_id: str, stamp_time, pub, conv):
        """백그라운드: VLP 바이너리 파일 읽기 + publish."""
        msg = conv._make_vlp_msg(bin_path, frame_id, stamp_time)
        if msg and pub:
            pub.publish(msg)

    def _kaist_do_sick(self, bin_path: str, frame_id: str, stamp_time, pub, conv):
        """백그라운드: SICK 바이너리 파일 읽기 + publish."""
        msg = conv._make_laserscan_msg(bin_path, frame_id, stamp_time)
        if msg and pub:
            pub.publish(msg)

    def _kaist_do_stereo(self, left_path: str, right_path: str, stamp_time,
                         left_pub, right_pub, conv):
        """백그라운드: Stereo 양쪽 PNG 읽기 + publish."""
        if left_path and left_pub:
            msg = conv._make_stereo_msg(left_path, stamp_time, 'stereo_left')
            if msg:
                left_pub.publish(msg)
        if right_path and right_pub:
            msg = conv._make_stereo_msg(right_path, stamp_time, 'stereo_right')
            if msg:
                right_pub.publish(msg)

    def _publish_kaist_frame(self, stamp_ns: int):
        """KAIST 프레임(VLP, SICK, Stereo, IMU, GPS, VRS, Dynamic TF)을 ROS2 토픽으로 publish한다.

        최적화:
        - 센서별 stamp 세트 O(1) 룩업으로 os.path.isfile() syscall 제거
        - global_pose 탐색: O(n) 선형 스캔 → bisect O(log n)
        - 각 센서를 해당 센서 stamp에서만 publish (불필요한 nearest-stamp 조회 제거)
        - 경로 캐시: 매 프레임 os.path.join 생략
        - VLP/SICK/Stereo: 백그라운드 워커로 파일 I/O 비블록화 (레퍼런스 패턴)
        """
        import bisect

        if not self.kaist_dataset_path:
            return

        if self._kaist_conv is None:
            from ros_slam_webui.kaist_converter import KaistConverter
            self._kaist_conv = KaistConverter()
        conv = self._kaist_conv
        stamp_time = conv._ns_to_time_msg(stamp_ns)

        # TF — dynamic(pose stamp에서만) + static(매 프레임, rosbridge TRANSIENT_LOCAL 수신 불안정 대응)
        if self.kaist_tf_pub:
            all_tfs = []
            # Dynamic TF (world → base_link) — pose stamp에서만 추가
            if stamp_ns in self.kaist_pose_stamp_set and self.kaist_pose_stamps_sorted:
                idx = bisect.bisect_left(self.kaist_pose_stamps_sorted, stamp_ns)
                if idx < len(self.kaist_global_poses):
                    _, R, T = self.kaist_global_poses[idx]
                    dyn_tf = conv._make_dynamic_tf(R, T, stamp_time)
                    if dyn_tf:
                        all_tfs.extend(dyn_tf.transforms)
            # Static TF — 매 프레임 /tf에 포함
            if self.kaist_static_tf_msg:
                for t in self.kaist_static_tf_msg.transforms:
                    t_copy = TransformStamped()
                    t_copy.header.stamp = stamp_time
                    t_copy.header.frame_id = t.header.frame_id
                    t_copy.child_frame_id = t.child_frame_id
                    t_copy.transform = t.transform
                    all_tfs.append(t_copy)
            if all_tfs:
                combined_tf = TFMessage()
                combined_tf.transforms = all_tfs
                self.kaist_tf_pub.publish(combined_tf)

        # IMU — IMU stamp에서만 publish (경량, 동기 처리)
        if stamp_ns in self.kaist_imu_stamps and self.kaist_imu_data and self.kaist_imu_pub:
            imu_row = conv._find_nearest_by_stamp(self.kaist_imu_data, stamp_ns)
            if imu_row:
                imu_msg = conv._make_imu_msg(imu_row, stamp_time)
                self.kaist_imu_pub.publish(imu_msg)

        # GPS — GPS stamp에서만 publish (경량, 동기 처리)
        if stamp_ns in self.kaist_gps_stamps and self.kaist_gps_data and self.kaist_gps_pub:
            gps_row = conv._find_nearest_by_stamp(self.kaist_gps_data, stamp_ns)
            if gps_row:
                gps_msg = conv._make_navsatfix_msg(gps_row, stamp_time)
                self.kaist_gps_pub.publish(gps_msg)

        # VRS GPS — VRS stamp에서만 publish (경량, 동기 처리)
        if stamp_ns in self.kaist_vrs_stamps and self.kaist_vrs_data and self.kaist_vrs_pub:
            vrs_row = conv._find_nearest_by_stamp(self.kaist_vrs_data, stamp_ns)
            if vrs_row:
                vrs_msg = conv._make_navsatfix_msg(vrs_row, stamp_time)
                self.kaist_vrs_pub.publish(vrs_msg)

        # VLP Left — 백그라운드 워커로 파일 I/O 비블록화
        if stamp_ns in self.kaist_vlp_left_stamps and self.kaist_vlp_left_pub:
            bin_path = f'{self.kaist_vlp_left_dir}/{stamp_ns}.bin'
            if self._kaist_vlp_left_worker:
                self._kaist_vlp_left_worker.push(
                    self._kaist_do_vlp, bin_path, 'left_velodyne',
                    stamp_time, self.kaist_vlp_left_pub, conv)
            else:
                self._kaist_do_vlp(bin_path, 'left_velodyne', stamp_time,
                                   self.kaist_vlp_left_pub, conv)

        # VLP Right — 백그라운드 워커로 파일 I/O 비블록화
        if stamp_ns in self.kaist_vlp_right_stamps and self.kaist_vlp_right_dir and self.kaist_vlp_right_pub:
            bin_path = f'{self.kaist_vlp_right_dir}/{stamp_ns}.bin'
            if self._kaist_vlp_right_worker:
                self._kaist_vlp_right_worker.push(
                    self._kaist_do_vlp, bin_path, 'right_velodyne',
                    stamp_time, self.kaist_vlp_right_pub, conv)
            else:
                self._kaist_do_vlp(bin_path, 'right_velodyne', stamp_time,
                                   self.kaist_vlp_right_pub, conv)

        # SICK Back — 백그라운드 워커로 파일 I/O 비블록화
        if stamp_ns in self.kaist_sick_back_stamps and self.kaist_sick_back_dir and self.kaist_sick_back_pub:
            bin_path = f'{self.kaist_sick_back_dir}/{stamp_ns}.bin'
            if self._kaist_sick_back_worker:
                self._kaist_sick_back_worker.push(
                    self._kaist_do_sick, bin_path, 'back_sick',
                    stamp_time, self.kaist_sick_back_pub, conv)
            else:
                self._kaist_do_sick(bin_path, 'back_sick', stamp_time,
                                    self.kaist_sick_back_pub, conv)

        # SICK Middle — 백그라운드 워커로 파일 I/O 비블록화
        if stamp_ns in self.kaist_sick_mid_stamps and self.kaist_sick_mid_dir and self.kaist_sick_mid_pub:
            bin_path = f'{self.kaist_sick_mid_dir}/{stamp_ns}.bin'
            if self._kaist_sick_mid_worker:
                self._kaist_sick_mid_worker.push(
                    self._kaist_do_sick, bin_path, 'middle_sick',
                    stamp_time, self.kaist_sick_mid_pub, conv)
            else:
                self._kaist_do_sick(bin_path, 'middle_sick', stamp_time,
                                    self.kaist_sick_mid_pub, conv)

        # Stereo Left + Right — 한 워커에서 두 이미지 동시 처리
        if stamp_ns in self.kaist_stereo_stamps:
            left_path = (f'{self.kaist_stereo_left_dir}/{stamp_ns}.png'
                         if self.kaist_stereo_left_dir and self.kaist_stereo_left_pub else '')
            right_path = (f'{self.kaist_stereo_right_dir}/{stamp_ns}.png'
                          if self.kaist_stereo_right_dir and self.kaist_stereo_right_pub else '')
            if left_path or right_path:
                if self._kaist_stereo_worker:
                    self._kaist_stereo_worker.push(
                        self._kaist_do_stereo, left_path, right_path, stamp_time,
                        self.kaist_stereo_left_pub, self.kaist_stereo_right_pub, conv)
                else:
                    self._kaist_do_stereo(left_path, right_path, stamp_time,
                                          self.kaist_stereo_left_pub,
                                          self.kaist_stereo_right_pub, conv)

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

    def _load_ros2_bag_player(self, path: str) -> dict:
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
        self.player_is_kitti = False
        self.player_is_kaist = False
        self.player_is_mulran = False
        self.mulran_ctx = None
        self.mulran_events_by_stamp = {}
        self.player_slider_pos = 0
        self.player_timestamp = 0
        self.livox_cache = {}
        self.cam_cache = {}

        rospy.loginfo(f'Loaded ROS2 bag for player: {bag_dir}')
        return self._player_load_result(
            True, 'ROS2 bag path set', 'ros2_bag', None)

    def _load_ros1_bag_player(self, path: str) -> dict:
        """ROS1 .bag 파일 경로를 File Player 인프라로 로드한다.

        변환 완료 후 _onKittiConvertDone 또는 수동 load 시 호출된다.
        play 버튼이 눌리면 player_play_toggle() → start_ros1_playback()으로 위임.
        """
        # ConPR publishers 정리 (변환된 bag은 /livox/lidar를 PointCloud2로 저장 → 충돌 방지)
        self._destroy_conpr_publishers()

        # 기존 ConPR playback 스레드 정지
        self.player_playing = False
        self.player_paused = False
        if self.playback_active:
            self.playback_active = False
            old_thread = self.playback_thread
            self.playback_thread = None
            if old_thread and old_thread.is_alive():
                old_thread.join(timeout=1.0)

        # 기존 ROS1 재생 중이면 중지
        self.stop_ros1_playback()

        self.bag_path = path
        self.player_path = path
        self.player_data_loaded = True
        self.player_is_ros2_bag = False
        self.player_is_ros1_bag = True
        self.player_is_kitti = False
        self.player_is_kaist = False
        self.player_is_mulran = False
        self.mulran_ctx = None
        self.mulran_events_by_stamp = {}
        self.player_slider_pos = 0
        self.player_timestamp = 0
        self.livox_cache = {}
        self.cam_cache = {}

        rospy.loginfo(f'Loaded ROS1 bag for player: {path}')
        return self._player_load_result(
            True, 'ROS1 bag path set', 'ros1_bag', None)

    def load_player_data(self, path):
        """Load file player data from the specified path"""
        self.invalidate_ros_topics_list_cache()

        # KITTI drive 디렉토리인 경우 직접 플레이어로 로드
        if self._is_kitti_drive_path(path):
            rospy.loginfo(f'Detected KITTI drive path: {path}')
            return self._load_kitti_direct(path)

        # MulRan (KAIST와 data_stamp.csv 경로가 겹칠 수 있어 KAIST보다 먼저 판별)
        if self._is_mulran_dataset_path(path):
            rospy.loginfo(f'Detected MulRan sequence path: {path}')
            return self._load_mulran_direct(path)

        # KAIST 시퀀스 디렉토리인 경우 직접 플레이어로 로드
        if self._is_kaist_dataset_path(path):
            rospy.loginfo(f'Detected KAIST sequence path: {path}')
            return self._load_kaist_direct(path)

        # ROS1 .bag 파일인 경우 ROS1 bag player 인프라로 위임
        if path.endswith('.bag') and os.path.isfile(path):
            rospy.loginfo(f'Detected ROS1 .bag path: {path}')
            return self._load_ros1_bag_player(path)

        # ROS2 bag 경로인 경우 기존 bag playback 인프라로 위임
        if self._is_ros2_bag_path(path):
            rospy.loginfo(f'Detected ROS2 bag path: {path}')
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
        self.player_is_ros1_bag = False   # ROS1 모드 해제
        self.player_is_kitti = False      # KITTI 모드 해제
        self.player_is_kaist = False      # KAIST 모드 해제
        self.player_is_mulran = False
        self.mulran_ctx = None
        self.mulran_events_by_stamp = {}

        # ROS1 bag 재생 중이면 중지 (PointCloud2 publisher 정리 → ConPR CustomMsg 생성 가능)
        self.stop_ros1_playback()

        if self.playback_active:
            self.playback_active = False
            thread = self.playback_thread
            self.playback_thread = None
            if thread and thread.is_alive():
                thread.join(timeout=1.0)

        # 이전 데이터셋의 중량 센서 워커 정지
        self._stop_heavy_sensor_workers()

        # 캐시 초기화 (이전 데이터 완전 제거)
        self.livox_cache = {}
        self.cam_cache = {}

        self.player_path = path
        self.player_data_loaded = False

        try:
            # Check if data_stamp.csv exists
            stamp_file = os.path.join(path, 'data_stamp.csv')
            if not os.path.exists(stamp_file):
                rospy.logerr(f'data_stamp.csv not found in {path}')
                return self._player_load_result(
                    False, f'data_stamp.csv not found in {path}', 'conpr', [])

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
                        rospy.logwarn(f'Skipping malformed line in data_stamp.csv: {line.strip()} - {str(e)}')
                        continue

            if not self.data_stamp:
                rospy.logerr('No valid data found in data_stamp.csv')
                return self._player_load_result(
                    False, 'No valid data in data_stamp.csv', 'conpr', [])

            timestamps = sorted(self.data_stamp.keys())
            self.player_initial_stamp = timestamps[0]
            self.player_last_stamp = timestamps[-1]
            self.player_timestamp = self.player_initial_stamp

            rospy.loginfo(f'Loaded {len(self.data_stamp)} data stamps')

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
                            rospy.logwarn(f'Skipping malformed line in pose.csv: {line.strip()} - {str(e)}')
                            continue
                rospy.loginfo(f'Loaded {len(self.pose_data)} pose data points')

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
                            rospy.logwarn(f'Skipping malformed line in imu.csv: {line.strip()} - {str(e)}')
                            continue
                rospy.loginfo(f'Loaded {len(self.imu_data)} IMU data points')

            # Load LiDAR file list + stamp→path 맵 (O(1) 룩업, os.path.exists 제거)
            lidar_dir = os.path.join(path, 'LiDAR')
            if os.path.exists(lidar_dir):
                self.livox_file_list = sorted(glob.glob(os.path.join(lidar_dir, '*.bin')))
                self.livox_stamp_to_path = {}
                for _fpath in self.livox_file_list:
                    try:
                        self.livox_stamp_to_path[int(os.path.splitext(os.path.basename(_fpath))[0])] = _fpath
                    except ValueError:
                        pass
                rospy.loginfo(f'Found {len(self.livox_file_list)} LiDAR files')
            else:
                self.livox_file_list = []
                self.livox_stamp_to_path = {}
                rospy.logwarn('LiDAR directory not found')

            # Load Camera file list + stamp→path 맵 (O(1) 룩업)
            cam_dir = os.path.join(path, 'Camera')
            if os.path.exists(cam_dir):
                patterns = ['*.jpg', '*.png', '*.jpeg', '*.JPG', '*.PNG']
                self.cam_file_list = []
                for pattern in patterns:
                    self.cam_file_list.extend(glob.glob(os.path.join(cam_dir, pattern)))
                self.cam_file_list = sorted(self.cam_file_list)
                self.cam_stamp_to_path = {}
                for _fpath in self.cam_file_list:
                    try:
                        self.cam_stamp_to_path[int(os.path.splitext(os.path.basename(_fpath))[0])] = _fpath
                    except ValueError:
                        pass
                rospy.loginfo(f'Found {len(self.cam_file_list)} camera images in Camera/')
            else:
                self.cam_file_list = []
                self.cam_stamp_to_path = {}
                rospy.logwarn('Camera directory not found (expected: {}/Camera/)'.format(path))

            self.player_data_loaded = True
            # Lazy-initialize File Player ROS2 publishers/subscribers on first load
            self._init_file_player_ros_interfaces()
            # ConPR Livox / Camera 백그라운드 워커 시작
            self._start_conpr_workers()
            return self._player_load_result(
                True, 'ConPR data loaded', 'conpr', [])

        except Exception as e:
            rospy.logerr(f'Failed to load player data: {str(e)}')
            import traceback
            traceback.print_exc()
            return self._player_load_result(False, str(e), 'conpr', [])

    # ── KITTI 변환 함수 ────────────────────────────────────────────────────────

    def scan_kitti_directory(self, path: str) -> dict:
        """KITTI 데이터셋 디렉토리를 탐색하여 calib/drive 정보를 반환한다.

        Args:
            path: 사용자가 선택한 날짜 디렉토리 (예: /path/to/2011_09_30)

        Returns:
            {'success': True, 'scan_result': {...}} or {'success': False, 'error': '...'}
        """
        try:
            from ros_slam_webui.kitti_converter import KittiConverter
            converter = KittiConverter()
            result = converter.scan_directory(path)
            rospy.loginfo(
                f'KITTI scan complete: date={result["date"]}, '
                f'{len(result["drive_dirs"])} drive(s) found')
            return {'success': True, 'scan_result': result}
        except Exception as e:
            rospy.logerr(f'KITTI scan failed: {str(e)}')
            import traceback
            traceback.print_exc()
            return {'success': False, 'error': str(e)}

    def start_kitti_conversion(
        self,
        base_dir: str,
        calib_dir: str,
        data_path: str,
        drive_name: str,
        bag_format: str = 'ros2',
    ) -> dict:
        """KITTI 데이터를 ROS2 bag 또는 ROS1 .bag으로 변환하는 백그라운드 스레드를 시작한다.

        변환 진행률은 WebSocket(포트 8081)을 통해 전체 클라이언트에 push된다.

        Args:
            bag_format: 출력 bag 형식 - 'ros2' (기본) 또는 'ros1'
                        'ros1'이면 KittiConverter.convert_to_ros1bag()로 직접 변환 (.bag).
                        'ros2'이면 KittiConverter.convert_to_ros2bag()로 변환 (_bag 디렉토리).

        Returns:
            {'success': True, 'output_bag_path': '...'} or {'success': False, 'error': '...'}
        """
        if self.kitti_converter_running:
            return {'success': False, 'error': 'Conversion already in progress'}

        if bag_format == 'ros1':
            final_output_path = os.path.join(base_dir, f"{drive_name}.bag")
        else:
            final_output_path = os.path.join(base_dir, f"{drive_name}_bag")

        def _run():
            self.kitti_converter_running = True
            try:
                from ros_slam_webui.kitti_converter import KittiConverter
                converter = KittiConverter()

                def _progress_cb(pct: int, msg: str):
                    self.pc2_ws_server.broadcast_json_all({
                        'type': 'kitti_convert_progress',
                        'progress': pct,
                        'message': msg,
                    })

                rospy.loginfo(
                    f'KITTI conversion started: {data_path} → {final_output_path} '
                    f'[format={bag_format}]')

                if bag_format == 'ros1':
                    # ROS1: KITTI → ROS1 .bag 직접 변환 (중간 파일 없음)
                    converter.convert_to_ros1bag(
                        calib_dir=calib_dir,
                        data_path=data_path,
                        output_bag_path=final_output_path,
                        progress_cb=_progress_cb,
                    )
                else:
                    # ROS2: KITTI → ROS2 bag 변환
                    converter.convert_to_ros2bag(
                        calib_dir=calib_dir,
                        data_path=data_path,
                        output_bag_path=final_output_path,
                        progress_cb=_progress_cb,
                    )

                rospy.loginfo(f'KITTI conversion complete: {final_output_path}')
                self.pc2_ws_server.broadcast_json_all({
                    'type': 'kitti_convert_done',
                    'bag_path': final_output_path,
                })
            except Exception as e:
                rospy.logerr(f'KITTI conversion failed: {str(e)}')
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
        return {'success': True, 'message': 'Conversion started', 'output_bag_path': final_output_path}

    # ── KAIST 변환 함수 ────────────────────────────────────────────────────────

    def scan_kaist_directory(self, path: str) -> dict:
        """KAIST Complex Urban 데이터셋 디렉토리를 탐색하여 시퀀스 목록을 반환한다.

        Args:
            path: 사용자가 선택한 디렉토리 (예: /path/to/complex_urban)

        Returns:
            {'success': True, 'sequences': [{name, path}, ...]} or {'success': False, 'error': '...'}
        """
        try:
            from ros_slam_webui.kaist_converter import KaistConverter
            converter = KaistConverter()
            result = converter.scan_directory(path)
            rospy.loginfo(
                f'KAIST scan complete: {len(result["sequences"])} sequence(s) found')
            return result
        except Exception as e:
            rospy.logerr(f'KAIST scan failed: {str(e)}')
            import traceback
            traceback.print_exc()
            return {'success': False, 'error': str(e)}

    def start_kaist_conversion(
        self,
        sequence_dir: str,
        output_path: str,
        sensors: list = None,
        bag_format: str = 'ros2',
    ) -> dict:
        """KAIST 시퀀스를 ROS1/ROS2 bag으로 변환하는 백그라운드 스레드를 시작한다.

        변환 진행률은 WebSocket(포트 8081)을 통해 전체 클라이언트에 push된다.

        Args:
            sequence_dir: KAIST 시퀀스 디렉토리 (calibration/, sensor_data/, global_pose.csv 포함)
            output_path: 출력 경로 (ROS2: 디렉토리, ROS1: 무시하고 sequence_name.bag 사용)
            sensors: 포함할 센서 목록 (None이면 전체)
            bag_format: 'ros2' (기본) 또는 'ros1'

        Returns:
            {'success': True, 'output_bag_path': '...'} or {'success': False, 'error': '...'}
        """
        if self.kaist_converter_running:
            return {'success': False, 'error': 'Conversion already in progress'}

        if bag_format == 'ros1':
            seq_name = os.path.basename(sequence_dir.rstrip(os.sep))
            output_bag_path = os.path.join(
                os.path.dirname(sequence_dir), seq_name + '.bag'
            )
        else:
            output_bag_path = output_path

        def _run():
            self.kaist_converter_running = True
            try:
                from ros_slam_webui.kaist_converter import KaistConverter
                converter = KaistConverter()

                def _progress_cb(pct: int, msg: str):
                    self.pc2_ws_server.broadcast_json_all({
                        'type': 'kaist_convert_progress',
                        'progress': pct,
                        'message': msg,
                    })

                rospy.loginfo(
                    f'KAIST conversion started: {sequence_dir} → {output_bag_path} [format={bag_format}]')

                if bag_format == 'ros1':
                    converter.convert_to_ros1bag(
                        sequence_dir=sequence_dir,
                        output_bag_path=output_bag_path,
                        sensors=sensors,
                        progress_cb=_progress_cb,
                    )
                else:
                    converter.convert_to_ros2bag(
                        sequence_dir=sequence_dir,
                        output_path=output_bag_path,
                        sensors=sensors,
                        progress_cb=_progress_cb,
                    )

                rospy.loginfo(f'KAIST conversion complete: {output_bag_path}')
                self.pc2_ws_server.broadcast_json_all({
                    'type': 'kaist_convert_done',
                    'bag_path': output_bag_path,
                })
            except Exception as e:
                rospy.logerr(f'KAIST conversion failed: {str(e)}')
                import traceback
                traceback.print_exc()
                self.pc2_ws_server.broadcast_json_all({
                    'type': 'kaist_convert_error',
                    'error': str(e),
                })
            finally:
                self.kaist_converter_running = False

        self.kaist_convert_thread = threading.Thread(
            target=_run, daemon=True, name='kaist-convert')
        self.kaist_convert_thread.start()
        return {'success': True, 'message': 'Conversion started', 'output_bag_path': output_bag_path}

    def scan_mulran_directory(self, path: str) -> dict:
        """MulRan 데이터셋 베이스 디렉토리를 탐색하여 시퀀스 목록을 반환한다."""
        try:
            from ros_slam_webui.mulran_converter import MulRanConverter
            converter = MulRanConverter()
            result = converter.scan_directory(path)
            rospy.loginfo(
                f'MulRan scan complete: {len(result["sequences"])} sequence(s) found')
            return result
        except Exception as e:
            rospy.logerr(f'MulRan scan failed: {str(e)}')
            import traceback
            traceback.print_exc()
            return {'success': False, 'error': str(e)}

    def start_mulran_conversion(
        self,
        sequence_dir: str,
        output_path: str,
        sensors: list = None,
        bag_format: str = 'ros2',
    ) -> dict:
        """MulRan 시퀀스를 ROS1/ROS2 bag으로 변환하는 백그라운드 스레드를 시작한다."""
        if self.mulran_converter_running:
            return {'success': False, 'error': 'Conversion already in progress'}

        if bag_format == 'ros1':
            seq_name = os.path.basename(sequence_dir.rstrip(os.sep))
            output_bag_path = os.path.join(
                os.path.dirname(sequence_dir), seq_name + '.bag'
            )
        else:
            output_bag_path = output_path

        def _run():
            self.mulran_converter_running = True
            try:
                from ros_slam_webui.mulran_converter import MulRanConverter
                converter = MulRanConverter()

                def _progress_cb(pct: int, msg: str):
                    self.pc2_ws_server.broadcast_json_all({
                        'type': 'mulran_convert_progress',
                        'progress': pct,
                        'message': msg,
                    })

                rospy.loginfo(
                    f'MulRan conversion started: {sequence_dir} → {output_bag_path} [format={bag_format}]')

                if bag_format == 'ros1':
                    converter.convert_to_ros1bag(
                        sequence_dir=sequence_dir,
                        output_bag_path=output_bag_path,
                        sensors=sensors,
                        progress_cb=_progress_cb,
                    )
                else:
                    converter.convert_to_ros2bag(
                        sequence_dir=sequence_dir,
                        output_path=output_bag_path,
                        sensors=sensors,
                        progress_cb=_progress_cb,
                    )

                rospy.loginfo(f'MulRan conversion complete: {output_bag_path}')
                self.pc2_ws_server.broadcast_json_all({
                    'type': 'mulran_convert_done',
                    'bag_path': output_bag_path,
                })
            except Exception as e:
                rospy.logerr(f'MulRan conversion failed: {str(e)}')
                import traceback
                traceback.print_exc()
                self.pc2_ws_server.broadcast_json_all({
                    'type': 'mulran_convert_error',
                    'error': str(e),
                })
            finally:
                self.mulran_converter_running = False

        self.mulran_convert_thread = threading.Thread(
            target=_run, daemon=True, name='mulran-convert')
        self.mulran_convert_thread.start()
        return {'success': True, 'message': 'Conversion started', 'output_bag_path': output_bag_path}

    # ── ConPR 백그라운드 워커 헬퍼 ───────────────────────────────────────────

    def _conpr_do_livox(self, stamp: int):
        """백그라운드: ConPR Livox 바이너리 파일 읽기(또는 캐시) + publish."""
        msg = self.load_livox_data(stamp)
        if msg and self.livox_pub:
            self.livox_pub.publish(msg)

    def _conpr_do_cam(self, stamp: int):
        """백그라운드: ConPR 카메라 이미지 파일 읽기(또는 캐시) + publish."""
        cam_data = self.load_camera_data(stamp)
        if cam_data:
            img_msg, cam_info_msg = cam_data
            if self.cam_pub:
                self.cam_pub.publish(img_msg)
            if self.cam_info_pub:
                self.cam_info_pub.publish(cam_info_msg)

    def load_livox_data(self, stamp):
        """Load LiDAR data from .bin file for given timestamp"""
        if not LIVOX_AVAILABLE or not self.livox_pub:
            return None

        # Check cache first
        if stamp in self.livox_cache:
            return self.livox_cache[stamp]

        # stamp→path 맵에서 O(1) 룩업 (os.path.exists() syscall 제거)
        bin_path = self.livox_stamp_to_path.get(stamp)
        if not bin_path:
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
            msg.header.stamp = rospy.Time(stamp // 10**9, stamp % 10**9)
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
            rospy.logerr(f'Failed to load LiDAR data for stamp {stamp}: {str(e)}')
            return None

    def load_camera_data(self, stamp):
        """Load camera image for given timestamp"""
        # Check cache first
        if stamp in self.cam_cache:
            return self.cam_cache[stamp]

        # stamp→path 맵에서 O(1) 룩업 (확장자별 os.path.exists() 루프 제거)
        img_path = self.cam_stamp_to_path.get(stamp)
        if not img_path:
            return None

        try:
            # Read image using OpenCV
            cv_image = cv2.imread(img_path)
            if cv_image is None:
                return None

            # Convert to ROS Image message
            img_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            img_msg.header.stamp = rospy.Time(stamp // 10**9, stamp % 10**9)
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
            rospy.logerr(f'Failed to load camera data for stamp {stamp}: {str(e)}')
            return None

    def timer_callback(self):
        """Timer callback (10ms 주기 = 100Hz).

        - 정지 상태: processed_stamp 를 0 으로 리셋
        - KITTI 재생 중: 10ms 마다 /clock 을 publish 하여 시각화 업데이트
          (이전 100μs=10,000Hz는 GIL을 초당 200ms+ 점유 → latency 스파이크 원인)
        """
        if not self.player_playing:
            self.player_processed_stamp = 0
            return

        # KITTI 재생 중: 10ms마다 /clock 갱신 (100Hz로도 RViz2 시각화 충분히 부드러움)
        # playback_worker가 VLP 프레임마다(10Hz) 이미 clock 발행하므로 보조 역할
        if getattr(self, 'player_is_kitti', False) and self.clock_pub:
            try:
                clock_ns = self.player_initial_stamp + self.player_processed_stamp
                clock_msg = Clock()
                clock_msg.clock = rospy.Time(clock_ns // 10**9, clock_ns % 10**9)
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

            # 끝 도달 시: loop 모드면 0으로 리셋, 아니면 duration에 고정
            if self.bag_current_time >= self.bag_duration:
                if self.bag_player_loop:
                    self.bag_start_real_time = current_real_time
                    self.bag_start_offset = 0.0
                    self.bag_current_time = 0.0
                else:
                    self.bag_current_time = self.bag_duration

    def player_play_toggle(self):
        """Toggle play/stop"""
        if not self.player_data_loaded:
            rospy.logwarn('No data loaded. Please load data first.')
            return False

        # ── ROS2 bag 모드: bag_play_toggle()로 위임 ────────────────────────
        if getattr(self, 'player_is_ros2_bag', False):
            rospy.loginfo('ROS2 bag mode: delegating to bag_play_toggle()')
            return self.bag_play_toggle()

        # ── ROS1 .bag 모드: start/stop_ros1_playback()으로 위임 ─────────────
        if getattr(self, 'player_is_ros1_bag', False):
            thread = self.ros1_player_thread
            if thread is not None and thread.is_alive():
                rospy.loginfo('ROS1 bag mode: stopping playback')
                self.stop_ros1_playback()
                return True
            else:
                rospy.loginfo(
                    f'ROS1 bag mode: starting playback ({self.bag_path})')
                return self.start_ros1_playback(
                    self.bag_path,
                    topics=None,
                    rate=getattr(self, 'ros1_player_rate', 1.0),
                )

        self.player_playing = not self.player_playing
        self.player_paused = False

        if self.player_playing:
            rospy.loginfo('Starting playback...')

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
            rospy.loginfo('Stopping playback - resetting to beginning...')
            if self.playback_active:
                self.playback_active = False
                old_thread = self.playback_thread
                self.playback_thread = None
                if old_thread and old_thread.is_alive():
                    old_thread.join(timeout=0.5)
            # 워커 큐 클리어: 정지 직후 대용량 publish로 인한 레이턴시 스파이크 방지
            self._clear_all_sensor_workers()
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
            rospy.loginfo(f'Playback {status}')
            if self.player_paused:
                # 일시정지 시 워커 큐 클리어: 잔여 대용량 프레임 publish 방지
                self._clear_all_sensor_workers()
            return True
        return False

    def get_bag_info(self):
        """Get bag file info including topics and duration.

        Branches based on file extension:
        - .bag  → ROS1 bag (parsed via rosbags library)
        - other → ROS2 bag (parsed via ros2 bag info command)
        """
        if not self.bag_path:
            rospy.logwarn('No bag file loaded.')
            return {'topics': [], 'duration': 0.0, 'bag_type': 'ros2'}

        # ROS1: 모든 .bag 파일은 _get_ros1_bag_info()로 처리
        if self.bag_path.endswith('.bag') or os.path.isfile(self.bag_path):
            return self._get_ros1_bag_info()

        try:
            # rosbag info (ROS1)
            cmd = ['rosbag', 'info', '--yaml', self.bag_path]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)

            if result.returncode != 0:
                rospy.logerr(f'Failed to get bag info: {result.stderr}')
                return {'topics': [], 'duration': 0.0, 'bag_type': 'ros1'}

            # Parse YAML output
            topics = []
            duration = 0.0
            try:
                import yaml as _yaml
                data = _yaml.safe_load(result.stdout) or {}
                duration = float(data.get('duration', 0.0))
                for t in (data.get('topics') or []):
                    topics.append(t.get('topic', ''))
            except Exception:
                pass

            self.bag_topics = topics
            self.bag_duration = duration
            rospy.loginfo(f'Bag info: {len(topics)} topics, duration: {duration}s')

            return {'topics': topics, 'duration': duration, 'bag_type': 'ros1'}

        except subprocess.TimeoutExpired:
            rospy.logerr('Timeout while getting bag info')
            return {'topics': [], 'duration': 0.0, 'bag_type': 'ros1'}
        except Exception as e:
            rospy.logerr(f'Failed to get bag info: {str(e)}')
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
            - ROS1 tf/tfMessage → ROS2 tf2_msgs/msg/TFMessage 매핑

            Args:
                ros1_type_str (str): 예) 'sensor_msgs/msg/Image' 또는 'sensor_msgs/Image'

            Returns:
                bool: True if importable and class exists
            """
            if ros1_type_str in ('tf/tfMessage', 'tf/msg/tfMessage'):
                return True
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
            import rosbag as _rosbag
            with _rosbag.Bag(self.bag_path, 'r') as bag:
                raw_topics = bag.get_type_and_topic_info().topics  # {topic: TopicTuple}
                start_t = bag.get_start_time()
                end_t = bag.get_end_time()
                duration = max(0.0, end_t - start_t)

            topic_dicts = []
            topic_names = []
            for topic_name, topic_info in raw_topics.items():
                ros1_type = topic_info.msg_type   # 'sensor_msgs/PointCloud2' 형식
                publishable = _check_publishable(ros1_type)
                topic_dicts.append({
                    'name': topic_name,
                    'type': ros1_type,
                    'publishable': publishable,
                })
                topic_names.append(topic_name)

            self.bag_topics = topic_names
            self.bag_topic_infos = topic_dicts
            self.bag_duration = duration

            publishable_count = sum(1 for t in topic_dicts if t['publishable'])
            rospy.loginfo(
                f'ROS1 bag info: {len(topic_dicts)} topics '
                f'({publishable_count} publishable), duration: {duration:.3f}s'
            )
            for t in topic_dicts:
                flag = 'O' if t['publishable'] else 'X'
                rospy.loginfo(f'  [{flag}] {t["name"]} ({t["type"]})')

            return {'topics': topic_dicts, 'duration': duration, 'bag_type': 'ros1'}

        except Exception as e:
            rospy.logerr(f'Failed to read ROS1 bag: {str(e)}')
            import traceback
            traceback.print_exc()
            return {'topics': [], 'duration': 0.0, 'bag_type': 'ros1'}

    # ------------------------------------------------------------------
    # ROS1 Bag Player — 상태 메서드
    # ------------------------------------------------------------------
    def start_ros1_playback(self, bag_path, topics, rate):
        """ROS1 bag 재생 시작.

        기존 스레드가 있으면 중지한 후 새 스레드를 시작합니다.
        ConPR livox_pub(CustomMsg)가 /livox/lidar에 있으면 PointCloud2 publisher 생성 실패하므로
        재생 직전에 반드시 정리합니다.

        Args:
            bag_path (str): ROS1 .bag 파일 경로
            topics (list[str]): publish할 토픽 목록 (빈 리스트 = 전체)
            rate (float): 재생 속도 배율

        Returns:
            bool: True if successfully started
        """
        # ConPR CustomMsg publisher 정리 (같은 /livox/lidar 토픽 충돌 방지)
        self.player_playing = False
        self.player_paused = False
        if self.playback_active:
            self.playback_active = False
            old_thread = self.playback_thread
            self.playback_thread = None
            if old_thread and old_thread.is_alive():
                old_thread.join(timeout=1.0)
        self._destroy_conpr_publishers()

        # 기존 ROS1 스레드 정리
        self.stop_ros1_playback()

        self.ros1_player_rate = rate
        self.ros1_player_thread = Ros1BagPlayerThread(bag_path, topics, rate, self)
        self.ros1_player_thread.set_loop(self.bag_player_loop)
        self.ros1_player_thread.start()
        self.invalidate_ros_topics_list_cache()
        rospy.loginfo(
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
            rospy.loginfo('[ROS1 Player] Resumed')
            return {'paused': False}
        else:
            thread.pause()
            rospy.loginfo('[ROS1 Player] Paused')
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
            rospy.loginfo('[ROS1 Player] Stopped')
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
        """ROS1 환경에서는 ROS1→ROS2 bag 변환이 불필요하므로 비활성화.

        Returns:
            dict: {'success': False, 'error': str}
        """
        return {'success': False, 'error': 'ROS1 bag conversion not supported in ROS1 mode'}

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
                rospy.loginfo(f'Removing existing output file: {output_path}')
                os.remove(output_path)

            rospy.loginfo(
                f'Converting ROS2 bag: {self.bag_path} -> {output_path}'
            )

            # rosbags-convert 경로 탐색 (pip user install 경로 포함)
            convert_cmd = shutil.which('rosbags-convert') or '/home/kkw/.local/bin/rosbags-convert'
            if not os.path.isfile(convert_cmd):
                rospy.logerr('rosbags-convert not found. Install with: pip install rosbags')
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
                rospy.logerr(f'rosbags-convert failed: {error_msg}')
                return {'success': False, 'error': error_msg}

            rospy.loginfo(f'ROS2 bag converted to ROS1 successfully: {output_path}')
            return {'success': True, 'output_path': output_path}

        except subprocess.TimeoutExpired:
            rospy.logerr('Timeout during ROS2→ROS1 bag conversion')
            return {'success': False, 'error': 'Conversion timed out'}
        except FileNotFoundError:
            rospy.logerr('rosbags-convert not found. Install with: pip install rosbags')
            return {'success': False, 'error': 'rosbags-convert not found. Run: pip install rosbags'}
        except Exception as e:
            rospy.logerr(f'Failed to convert ROS2 bag to ROS1: {str(e)}')
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
            rospy.logwarn('No bag file loaded. Please load a bag file first.')
            return False

        with self._bag_play_lock:
            return self._bag_play_toggle_locked(selected_topics, start_offset, rate)

    def _bag_play_toggle_locked(self, selected_topics=None, start_offset=None, rate=1.0):
        """bag_play_toggle의 실제 구현 (락 획득 후 호출)."""
        if self.bag_playing:
            # Stop playback
            rospy.loginfo('Stopping bag playback...')
            # stop+restart 방식 일시정지 상태: bag_process가 None일 수 있으므로 guard
            if self.bag_process:
                self.bag_process.terminate()
                try:
                    self.bag_process.wait(timeout=5)
                except Exception:
                    self.bag_process.kill()
                self.bag_process = None
            self.bag_playing = False
            self.bag_paused = False
            self.bag_current_time = 0.0
            self.bag_start_real_time = 0.0
            self._bag_stop_pause_offset = None
        else:
            # Start playback using rosbag play (ROS1)
            rospy.loginfo(f'Starting bag playback: {self.bag_path}')
            try:
                # Build command with topic selection (ROS1 rosbag play)
                cmd = ['rosbag', 'play', self.bag_path]

                # Add start offset if specified
                if start_offset is not None and start_offset > 0:
                    cmd.extend(['-s', str(start_offset)])
                    self.bag_start_offset = start_offset
                    self.bag_current_time = start_offset
                    rospy.loginfo(f'Starting from offset: {start_offset}s')
                else:
                    self.bag_start_offset = 0.0
                    self.bag_current_time = 0.0

                # Add playback rate (rosbag play -r <rate>)
                rate = max(0.01, float(rate))
                self.bag_playback_rate = rate
                if rate != 1.0:
                    cmd.extend(['-r', str(rate)])
                rospy.loginfo(f'Playback rate: {rate}x')

                if self.bag_player_loop:
                    rospy.loginfo('Loop playback enabled (managed by monitor thread)')

                # Add topic filter if topics are selected
                if selected_topics and len(selected_topics) > 0:
                    topics_to_play = list(selected_topics)
                    for tf_topic in ('/tf', '/tf_static'):
                        if tf_topic not in topics_to_play:
                            topics_to_play.append(tf_topic)
                            rospy.loginfo(f'[bag play] Including {tf_topic} for 3D Viewer TF')
                    self.bag_selected_topics = topics_to_play
                    cmd.extend(topics_to_play)
                    rospy.loginfo(f'Playing selected topics: {topics_to_play}')
                else:
                    rospy.loginfo('Playing all topics')

                rospy.loginfo(f'Command: {" ".join(cmd)}')

                self.bag_process = subprocess.Popen(cmd,
                                                     env={**os.environ},
                                                     stdout=subprocess.PIPE,
                                                     stderr=subprocess.STDOUT)
                self.bag_playing = True
                self.bag_paused = False
                self.bag_start_real_time = time.time()
                self.invalidate_ros_topics_list_cache()
                rospy.loginfo('Bag playback started successfully')

                # Start thread to read output
                import threading
                def read_output():
                    for line in iter(self.bag_process.stdout.readline, b''):
                        if line:
                            rospy.loginfo(f'[bag play] {line.decode().strip()}')
                threading.Thread(target=read_output, daemon=True).start()

                # Start monitor thread: detect natural process exit → update playing state
                current_process = self.bag_process
                threading.Thread(
                    target=self._bag_process_monitor,
                    args=(current_process,),
                    daemon=True
                ).start()
            except Exception as e:
                rospy.logerr(f'Failed to start bag playback: {str(e)}')
                return False

        return True

    def _bag_process_monitor(self, process_ref):
        """ROS2 bag play 프로세스 자연 종료를 감지하여 playing 상태를 업데이트한다.

        - 루프 OFF: 프로세스 종료 시 bag_playing = False, 시간 0으로 리셋
        - 루프 ON : 락 안에서 직접 새 프로세스 시작 (bag_play_toggle 호출 시 데드락/경쟁 방지)
        """
        try:
            process_ref.wait()
        except Exception:
            return

        with self._bag_play_lock:
            # 수동 정지(bag_process가 교체 또는 None)된 경우 스킵
            if self.bag_process is not process_ref or not self.bag_playing:
                return

            if not self.bag_player_loop:
                # 비루프: 재생 완료 → 상태 리셋
                rospy.loginfo('[bag monitor] Bag ended naturally → resetting state')
                self.bag_playing = False
                self.bag_process = None
                self.bag_current_time = 0.0
                self.bag_start_offset = 0.0
                return

            # 루프: 락 안에서 직접 프로세스 재시작 (bag_play_toggle 재진입 방지)
            rospy.loginfo('[bag monitor] Bag ended → restarting from position 0 (loop)')
            try:
                cmd = ['ros2', 'bag', 'play', self.bag_path]
                rate = self.bag_playback_rate
                if rate != 1.0:
                    cmd.extend(['--rate', str(rate)])
                if self.bag_selected_topics and len(self.bag_selected_topics) > 0:
                    topics_to_play = list(self.bag_selected_topics)
                    for tf_topic in ('/tf', '/tf_static'):
                        if tf_topic not in topics_to_play:
                            topics_to_play.append(tf_topic)
                    cmd.append('--topics')
                    cmd.extend(topics_to_play)

                new_proc = subprocess.Popen(
                    cmd, env=self._ros_env,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT
                )
                self.bag_process = new_proc
                self.bag_playing = True
                self.bag_paused = False
                self.bag_start_real_time = time.time()
                self.bag_current_time = 0.0
                self.bag_start_offset = 0.0

                def _read_loop_output():
                    for line in iter(new_proc.stdout.readline, b''):
                        if line:
                            rospy.loginfo(f'[bag loop] {line.decode().strip()}')
                threading.Thread(target=_read_loop_output, daemon=True).start()
                threading.Thread(
                    target=self._bag_process_monitor,
                    args=(new_proc,),
                    daemon=True
                ).start()

            except Exception as e:
                self.bag_playing = False
                self.bag_process = None
                self.bag_current_time = 0.0
                rospy.logerr(f'[bag monitor] Loop restart failed: {e}')

    def bag_pause_toggle(self):
        """Toggle bag playback pause/resume.

        우선순위:
        1. rosbag2_interfaces 서비스(/rosbag2_player/pause, /rosbag2_player/resume) 사용
        2. 서비스 실패 시 → 안전한 stop+restart 방식 사용

        ▶ SIGSTOP/SIGCONT 방식을 사용하지 않는 이유 ◀
        SIGSTOP으로 프로세스를 동결하면 DDS 하트비트 스레드도 멈춘다.
        SIGCONT 재개 시 DDS 리더(FAST_LIO 등)가 라이터 재연결을 감지하고
        old timestamp 메시지를 재전송 → FAST_LIO imu_buffer clear → 궤적 발산.
        stop+restart 방식은 프로세스를 깨끗이 종료 후 동일 offset에서 재시작하므로
        타임스탬프 연속성이 보장된다.
        """
        # stop+restart 일시정지 상태(bag_process=None, bag_paused=True)도 허용
        if not self.bag_playing:
            rospy.logwarn('No bag playback in progress')
            return False
        if not self.bag_process and not self.bag_paused:
            rospy.logwarn('No bag playback process found')
            return False

        # ROS1: rosbag play에는 Pause/Resume 서비스가 없으므로 stop+restart 방식 사용
        if self.bag_paused:
            # ── Resume ────────────────────────────────────────────────────────
            rospy.loginfo('Resuming bag playback...')
            success = False
            pause_offset = self._bag_stop_pause_offset
            if pause_offset is not None:
                rospy.loginfo(f'Resuming via stop+restart at offset {pause_offset:.2f}s')
                try:
                    new_proc = self._start_bag_process_at_offset(pause_offset)
                    if new_proc:
                        self.bag_process = new_proc
                        self._bag_stop_pause_offset = None
                        success = True
                        rospy.loginfo('Resumed via stop+restart')
                except Exception as e:
                    rospy.logerr(f'stop+restart resume failed: {e}')
            else:
                try:
                    new_proc = self._start_bag_process_at_offset(self.bag_current_time)
                    if new_proc:
                        if self.bag_process:
                            try:
                                self.bag_process.terminate()
                                self.bag_process.wait(timeout=2)
                            except Exception:
                                pass
                        self.bag_process = new_proc
                        success = True
                except Exception as e:
                    rospy.logerr(f'Fallback restart failed: {e}')

            if success:
                self.bag_paused = False
                pause_duration = time.time() - self.bag_pause_time
                self.bag_start_real_time += pause_duration
            return success

        else:
            # ── Pause ─────────────────────────────────────────────────────────
            rospy.loginfo('Pausing bag playback...')
            success = False

            # ROS1: 프로세스 종료 + 위치 기록 방식
            if self.bag_process:
                self._bag_stop_pause_offset = self.bag_current_time
                rospy.loginfo(
                    f'Pausing via process stop (offset={self._bag_stop_pause_offset:.2f}s); '
                    'will restart from this position on resume'
                )
                try:
                    self.bag_process.terminate()
                    self.bag_process.wait(timeout=3)
                except Exception:
                    try:
                        self.bag_process.kill()
                    except Exception:
                        pass
                self.bag_process = None
                success = True

            if success:
                self.bag_paused = True
                self.bag_pause_time = time.time()
            return success

    def _start_bag_process_at_offset(self, start_offset: float):
        """지정 offset(초)에서 rosbag play 서브프로세스를 새로 시작한다 (ROS1).

        stop+restart 방식의 pause/resume fallback에서 사용.
        bag_start_real_time / bag_start_offset 은 호출 측에서 관리한다.
        """
        cmd = ['rosbag', 'play', self.bag_path]
        if start_offset > 0.0:
            cmd.extend(['-s', str(start_offset)])
        rate = getattr(self, 'bag_playback_rate', 1.0)
        if rate != 1.0:
            cmd.extend(['-r', str(rate)])
        topics = getattr(self, 'bag_selected_topics', [])
        if topics:
            cmd.extend(topics)
        rospy.loginfo(f'[restart] {" ".join(cmd)}')
        proc = subprocess.Popen(
            cmd,
            env={**os.environ},
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )

        def _read_output():
            for line in iter(proc.stdout.readline, b''):
                if line:
                    rospy.loginfo(f'[bag play] {line.decode().strip()}')
        threading.Thread(target=_read_output, daemon=True).start()
        return proc

    def set_bag_position(self, position_ratio):
        """Set bag playback position (0.0 to 1.0). ROS1/ROS2 bag 모두 지원."""
        if self.bag_duration <= 0:
            return False

        target_time = position_ratio * self.bag_duration

        # ROS1 bag 재생 중: Ros1BagPlayerThread.set_seek() 호출
        thread = self.ros1_player_thread
        if thread is not None and thread.is_alive():
            thread.set_seek(target_time)
            self.bag_current_time = target_time
            rospy.loginfo(f'[ROS1] Set bag position to {target_time}s ({position_ratio*100}%)')
            return True

        # ROS2 bag: 기존 로직
        if self.bag_playing:
            self.bag_play_toggle()
            time.sleep(0.1)
            self.bag_play_toggle(self.bag_selected_topics, target_time, self.bag_playback_rate)
        else:
            self.bag_current_time = target_time
            self.bag_start_offset = target_time

        rospy.loginfo(f'Set bag position to {target_time}s ({position_ratio*100}%)')
        return True

    def set_bag_playback_rate(self, rate: float) -> dict:
        """ROS1 bag 재생 중 속도 변경.

        ROS1에서는 rosbag play -r 옵션으로만 속도 설정 가능 → stop+restart 방식 사용.

        Args:
            rate (float): 새 속도 배율 (> 0)

        Returns:
            dict: {'success': bool, 'rate': float, 'message': str}
        """
        rate = max(0.01, float(rate))

        if not self.bag_playing or not self.bag_process:
            self.bag_playback_rate = rate
            rospy.loginfo(f'[bag set_rate] Stored rate={rate}x (not playing)')
            return {'success': True, 'rate': rate, 'message': 'Rate stored for next playback'}

        # ROS1: 현재 위치에서 새 속도로 재시작
        try:
            current_offset = self.bag_current_time
            self.bag_playback_rate = rate
            new_proc = self._start_bag_process_at_offset(current_offset)
            if new_proc:
                old_proc = self.bag_process
                self.bag_process = new_proc
                if old_proc:
                    try:
                        old_proc.terminate()
                        old_proc.wait(timeout=2)
                    except Exception:
                        pass
                now = time.time()
                self.bag_start_real_time = now
                rospy.loginfo(f'[ROS1 bag set_rate] Rate changed to {rate}x via restart')
                return {'success': True, 'rate': rate, 'message': f'Rate set to {rate}x'}
            return {'success': False, 'rate': rate, 'message': 'Failed to restart bag process'}
        except Exception as e:
            rospy.logerr(f'[bag set_rate] Service call failed: {e}')
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
            rospy.loginfo(f'[ROS1 set_rate] Rate changed to {rate}x during playback')
            return {'success': True, 'rate': rate, 'message': f'Rate set to {rate}x'}
        else:
            rospy.loginfo(f'[ROS1 set_rate] Stored rate={rate}x (not playing)')
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
            'current_time': self.bag_current_time,
            'loop': self.bag_player_loop,
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
        rospy.loginfo('Playback worker started')

        timestamps = []      # Play 시작 시 data_stamp 에서 복사
        current_idx = 0      # 다음 처리할 timestamps 인덱스
        was_playing = False  # 이전 루프의 재생 상태 (재시작 감지용)
        was_paused = False   # pause 상태 추적 (resume 시 기준 시간 재조정)

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
                time.sleep(0.05)  # 재생 중지 시 CPU 절약
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
                rospy.loginfo(
                    f'Playback started: {len(timestamps)} stamps, speed={self.player_speed}'
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
                rospy.loginfo(
                    f'Seek done: stamp={seek_stamp}, idx={current_idx}'
                )
            was_playing = True

            # 현재 목표 스탬프 계산
            target_stamp = self.player_initial_stamp + self.player_processed_stamp

            # Stop/Pause 시 즉시 중단 (inner loop 내부에서도 체크 — 배치 처리 중 반응)
            # 배치당 최대 프레임 수 제한으로 latency 스파이크 방지
            # 모든 데이터셋 통일 8: 이전에 KAIST만 8 이었으나 KITTI/MulRan도 batch 20은
            # 100ms 이상 main-thread를 점유해 HTTP ping 지연 → 레이턴시 스파이크 유발
            _batch_limit = 8
            _batch_count = 0

            # 인덱스를 앞으로 전진하면서 target_stamp 이하의 스탬프만 발행 (O(k))
            while current_idx < len(timestamps):
                if not self.player_playing or self.player_paused:
                    break
                if _batch_count >= _batch_limit:
                    break
                stamp = timestamps[current_idx]
                if stamp > target_stamp:
                    break
                current_idx += 1
                _batch_count += 1

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
                            clock_msg.clock = rospy.Time(stamp // 10**9, stamp % 10**9)
                            self.clock_pub.publish(clock_msg)
                    except Exception as e:
                        rospy.logwarn(f'KITTI frame publish error: {e}')
                    # player_timestamp는 예외 여부와 무관하게 항상 갱신
                    self.player_timestamp = stamp
                    time.sleep(0)  # GIL 해제 → HTTP 핸들러 스레드에 CPU 양보
                    continue  # ConPR 분기 스킵

                # ── KAIST direct play ──────────────────────────────────────
                if getattr(self, 'player_is_kaist', False):
                    try:
                        self._publish_kaist_frame(stamp)
                        if self.clock_pub:
                            clock_msg = Clock()
                            clock_msg.clock = rospy.Time(stamp // 10**9, stamp % 10**9)
                            self.clock_pub.publish(clock_msg)
                    except Exception as e:
                        rospy.logwarn(f'KAIST frame publish error: {e}')
                    self.player_timestamp = stamp
                    time.sleep(0)  # GIL 해제 → HTTP 핸들러 스레드(ping 응답 등)에 CPU 양보
                    continue  # ConPR 분기 스킵

                if getattr(self, 'player_is_mulran', False):
                    try:
                        # /clock 는 _publish_mulran_frame 내부에서 10ms 간격으로 throttle
                        self._publish_mulran_frame(stamp)
                    except Exception as e:
                        rospy.logwarn(f'MulRan frame publish error: {e}')
                    self.player_timestamp = stamp
                    time.sleep(0)  # GIL 해제 → HTTP 핸들러 스레드에 CPU 양보
                    continue

                if data_type == "pose" and stamp in self.pose_data:
                    x, y, z = self.pose_data[stamp]
                    msg = PointStamped()
                    msg.header.stamp = rospy.Time(stamp // 10**9, stamp % 10**9)
                    msg.header.frame_id = 'imu_link'
                    msg.point.x = x
                    msg.point.y = y
                    msg.point.z = z
                    self.pose_pub.publish(msg)

                elif data_type == "imu" and stamp in self.imu_data:
                    imu_values = self.imu_data[stamp]
                    msg = Imu()
                    msg.header.stamp = rospy.Time(stamp // 10**9, stamp % 10**9)
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
                    # 백그라운드 워커로 비블로킹 처리 (ConPR Livox .bin 파싱이 무거움)
                    if self._conpr_livox_worker:
                        self._conpr_livox_worker.push(self._conpr_do_livox, stamp)
                    else:
                        self._conpr_do_livox(stamp)

                elif data_type == "cam":
                    # 백그라운드 워커로 비블로킹 처리
                    if self._conpr_cam_worker:
                        self._conpr_cam_worker.push(self._conpr_do_cam, stamp)
                    else:
                        self._conpr_do_cam(stamp)

                # 클락 메시지 발행
                if self.clock_pub:
                    try:
                        clock_msg = Clock()
                        clock_msg.clock = rospy.Time(stamp // 10**9, stamp % 10**9)
                        self.clock_pub.publish(clock_msg)
                    except Exception as e:
                        rospy.logwarn(f'Clock publish error: {e}')

                self.player_timestamp = stamp

            # 슬라이더 위치 업데이트
            if self.player_last_stamp > self.player_initial_stamp:
                progress = (target_stamp - self.player_initial_stamp) / \
                           (self.player_last_stamp - self.player_initial_stamp)
                self.player_slider_pos = int(min(progress, 1.0) * 10000)

            # 재생 종료 체크
            if target_stamp >= self.player_last_stamp:
                if self.player_loop:
                    rospy.loginfo('Looping playback...')
                    self.player_processed_stamp = 0
                    self.player_timestamp = self.player_initial_stamp
                    current_idx = 0
                    # wall-clock 기준도 리셋 (리셋 없으면 elapsed_ns 폭주)
                    _wall_start = now
                    _proc_start = 0
                else:
                    rospy.loginfo('Playback finished')
                    self.player_playing = False
                    self.player_processed_stamp = 0
                    self.player_slider_pos = 0
                    current_idx = 0

        rospy.loginfo('Playback worker stopped')

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

        rospy.loginfo(f'Seek requested: pos={position} → stamp={target_stamp}')

    def save_rosbag(self):
        """로드된 데이터를 ROS1 .bag 형식으로 저장한다 (rosbag 네이티브 모듈 사용)."""
        return self.save_rosbag_ros1()

    def save_rosbag_ros1(self):
        """로드된 데이터를 ROS1 .bag 형식으로 저장한다 (rosbag 네이티브 모듈 사용).

        Livox 데이터는 CustomMsg 대신 sensor_msgs/PointCloud2로 변환하여 저장한다.
        출력 경로: {player_path}/{name}.bag
        """
        if not self.player_data_loaded:
            rospy.logerr('No data loaded. Please load data first.')
            return False

        try:
            import rosbag as _rosbag
        except ImportError as e:
            rospy.logerr(f'rosbag 모듈이 필요합니다: {e}')
            return False

        def _ns_to_rospy_time(stamp_ns: int) -> rospy.Time:
            return rospy.Time(stamp_ns // 10 ** 9, stamp_ns % 10 ** 9)

        def _livox_to_pc2(livox_msg, stamp_ns: int) -> PointCloud2:
            """Livox CustomMsg → sensor_msgs/PointCloud2 변환."""
            _fields = [
                PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                PointField(name='tag',       offset=16, datatype=PointField.UINT8,   count=1),
                PointField(name='line',      offset=17, datatype=PointField.UINT8,   count=1),
            ]
            _step = 18
            _n = len(livox_msg.points)
            _buf = bytearray(_n * _step)
            for _i, _pt in enumerate(livox_msg.points):
                _off = _i * _step
                struct.pack_into('ffff', _buf, _off, _pt.x, _pt.y, _pt.z, float(_pt.reflectivity))
                struct.pack_into('BB', _buf, _off + 16, _pt.tag, _pt.line)
            _pc2 = PointCloud2()
            _pc2.header.stamp = _ns_to_rospy_time(stamp_ns)
            _pc2.header.frame_id = getattr(livox_msg.header, 'frame_id', None) or 'livox'
            _pc2.height = 1
            _pc2.width = _n
            _pc2.fields = _fields
            _pc2.is_bigendian = False
            _pc2.point_step = _step
            _pc2.row_step = _step * _n
            _pc2.data = bytes(_buf)
            _pc2.is_dense = True
            return _pc2

        try:
            from pathlib import Path as _Path
            bag_name = os.path.basename(os.path.normpath(self.player_path)) or 'output'
            bag_path = str(_Path(self.player_path) / f'{bag_name}.bag')
            self.save_bag_progress = '0%'
            self.save_bag_message = 'Starting conversion...'
            rospy.loginfo(f'Starting ROS1 bag save to: {bag_path}')

            livox_stamps = []
            if LIVOX_AVAILABLE and len(self.livox_file_list) > 0:
                livox_stamps = [s for s, dtype in self.data_stamp.items() if dtype == 'livox']
                rospy.loginfo(f'Livox → PointCloud2: {len(livox_stamps)} frames')

            cam_stamps = []
            if len(self.cam_file_list) > 0:
                cam_stamps = [s for s, dtype in self.data_stamp.items() if dtype == 'cam']

            total_items = (len(self.pose_data) + len(self.imu_data)
                           + len(cam_stamps) + len(livox_stamps))
            processed_items = 0
            last_pct = -1

            def update_progress():
                nonlocal processed_items, last_pct
                processed_items += 1
                if total_items > 0:
                    pct = int(processed_items / total_items * 100)
                    if pct != last_pct:
                        self.save_bag_progress = f'{pct}%'
                        last_pct = pct
                        time.sleep(0)  # GIL 반납

            if os.path.exists(bag_path):
                os.remove(bag_path)

            with _rosbag.Bag(bag_path, 'w') as bag:
                # Pose 데이터
                self.save_bag_message = 'Converting pose messages...'
                rospy.loginfo(f'Writing {len(self.pose_data)} pose messages...')
                for stamp_ns, (x, y, z) in sorted(self.pose_data.items()):
                    msg = PointStamped()
                    msg.header.stamp = _ns_to_rospy_time(stamp_ns)
                    msg.header.frame_id = 'imu_link'
                    msg.point.x = x
                    msg.point.y = y
                    msg.point.z = z
                    bag.write('/pose/position', msg, msg.header.stamp)
                    update_progress()

                # IMU 데이터
                self.save_bag_message = 'Converting IMU messages...'
                rospy.loginfo(f'Writing {len(self.imu_data)} IMU messages...')
                for stamp_ns, vals in sorted(self.imu_data.items()):
                    msg = Imu()
                    msg.header.stamp = _ns_to_rospy_time(stamp_ns)
                    msg.header.frame_id = 'imu_link'
                    msg.orientation.x = vals[0]
                    msg.orientation.y = vals[1]
                    msg.orientation.z = vals[2]
                    msg.orientation.w = vals[3]
                    msg.angular_velocity.x = vals[4]
                    msg.angular_velocity.y = vals[5]
                    msg.angular_velocity.z = vals[6]
                    msg.linear_acceleration.x = vals[7]
                    msg.linear_acceleration.y = vals[8]
                    msg.linear_acceleration.z = vals[9]
                    bag.write('/imu', msg, msg.header.stamp)
                    update_progress()

                # Livox LiDAR 데이터 (PointCloud2로 변환)
                if livox_stamps:
                    self.save_bag_message = 'Converting LiDAR messages...'
                    rospy.loginfo(f'Writing {len(livox_stamps)} Livox frames as PointCloud2...')
                    for stamp_ns in sorted(livox_stamps):
                        livox_msg = self.load_livox_data(stamp_ns)
                        if livox_msg:
                            pc2_msg = _livox_to_pc2(livox_msg, stamp_ns)
                            bag.write('/livox/lidar', pc2_msg, pc2_msg.header.stamp)
                        update_progress()

                # Camera 데이터
                if cam_stamps:
                    self.save_bag_message = 'Converting camera messages...'
                    rospy.loginfo(f'Writing {len(cam_stamps)} camera frames...')
                    for stamp_ns in sorted(cam_stamps):
                        cam_data = self.load_camera_data(stamp_ns)
                        if cam_data:
                            img_msg, cam_info_msg = cam_data
                            bag.write('/camera/color/image', img_msg, img_msg.header.stamp)
                            bag.write('/camera/color/camera_info',
                                      cam_info_msg, cam_info_msg.header.stamp)
                        update_progress()

            self.save_bag_progress = None
            self.save_bag_message = None
            rospy.loginfo(f'ROS1 bag save complete: {bag_path}')
            return True

        except Exception as e:
            self.save_bag_progress = None
            self.save_bag_message = None
            rospy.logerr(f'Failed to save ROS1 bag: {str(e)}')
            import traceback
            traceback.print_exc()
            return False

    def start_save_rosbag(self, bag_format: str = 'ros2'):
        """save_rosbag() 또는 save_rosbag_ros1()을 백그라운드 스레드에서 실행한다.

        Args:
            bag_format: 'ros2' (기본) — ROS2 bag (output/ 디렉토리)
                        'ros1'        — ROS1 .bag 파일 (output.bag)
        """
        if self.save_bag_saving:
            rospy.logwarn('Bag save already in progress')
            return False

        def _run():
            self.save_bag_saving = True
            self.save_bag_success = False
            try:
                if bag_format == 'ros1':
                    self.save_bag_success = self.save_rosbag_ros1()
                else:
                    self.save_bag_success = self.save_rosbag()
            finally:
                self.save_bag_saving = False
                self.save_bag_progress = None
                self.save_bag_message = None

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
            'auto_start': self.player_auto_start,
            'timestamp': self.player_timestamp,
            'slider_pos': self.player_slider_pos,
            'data_loaded': self.player_data_loaded,
            'save_bag_progress': self.save_bag_progress,
            'save_bag_message': self.save_bag_message,
            'save_bag_saving': self.save_bag_saving,
            'save_bag_success': self.save_bag_success,
            'speed': round(self.player_speed, 2),
        }

    def kill_slam_processes(self):
        """Kill all SLAM-related processes and bag playback"""
        try:
            # Kill bag playback process if running
            if self.bag_process and self.bag_process.poll() is None:
                rospy.loginfo(f'Terminating bag playback process PID: {self.bag_process.pid}')
                self.bag_process.terminate()
                try:
                    self.bag_process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    rospy.loginfo('Bag process did not terminate, killing...')
                    self.bag_process.kill()
                self.bag_process = None
                self.bag_playing = False

            # First try to terminate the subprocess gracefully
            if self.slam_process and self.slam_process.poll() is None:
                rospy.loginfo(f'Terminating SLAM process PID: {self.slam_process.pid}')
                self.slam_process.terminate()
                try:
                    self.slam_process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    rospy.loginfo('Process did not terminate, killing...')
                    self.slam_process.kill()
                self.slam_process = None

            # Then kill any remaining related processes
            subprocess.run(['pkill', '-9', '-f', 'LTmapping'], check=False,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(['pkill', '-9', '-f', 'rviz2'], check=False,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(['pkill', '-9', '-f', 'lt_mapper.launch.py'], check=False,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            rospy.loginfo('SLAM processes killed')
            self.slam_status = "Ready"
        except Exception as e:
            rospy.logerr(f'Error killing processes: {str(e)}')


# File browser functions
FAST_LIO_PACKAGE_CANDIDATES = (
    'FAST_LIO_Localization_and_Mapping',
    'FAST_LIO_ROS2',
    'fast_lio',
)


def _workspace_src_candidates():
    """Return likely ROS workspace src directories for sibling package discovery."""
    candidates = []

    def add_candidate(path, prepend=False):
        path = PathLib(path).expanduser()
        if path.exists() and path.is_dir() and path not in candidates:
            if prepend:
                candidates.insert(0, path)
            else:
                candidates.append(path)

    # Highest priority: src dir that contains the running ros_slam_webui package.
    try:
        import rospkg
        pkg_path = PathLib(rospkg.RosPack().get_path('ros_slam_webui'))
        for parent in (pkg_path, *pkg_path.parents):
            if parent.name == 'src':
                add_candidate(parent, prepend=True)
                break
    except Exception:
        pass

    # Source-tree execution: .../<ws>/src/ros_slam_webui/ros_slam_webui/web_server.py
    for parent in PathLib(__file__).resolve().parents:
        if parent.name == 'src':
            add_candidate(parent, prepend=True)
            break

    # Installed execution: use colcon/ament prefixes to infer workspace src.
    prefix_env = os.environ.get('COLCON_PREFIX_PATH', '') + os.pathsep + os.environ.get('AMENT_PREFIX_PATH', '')
    for prefix in [p for p in prefix_env.split(os.pathsep) if p]:
        prefix_path = PathLib(prefix).expanduser()
        for parent in (prefix_path, *prefix_path.parents):
            if parent.name == 'install':
                add_candidate(parent.parent / 'src')
                break

    # Catkin devel/install: infer workspace src from ROS_PACKAGE_PATH.
    ros_pkg_path = os.environ.get('ROS_PACKAGE_PATH', '')
    for entry in [p for p in ros_pkg_path.split(os.pathsep) if p]:
        entry_path = PathLib(entry).expanduser()
        for parent in (entry_path, *entry_path.parents):
            if parent.name == 'src':
                add_candidate(parent)
                break

    cwd = PathLib.cwd()
    for parent in (cwd, *cwd.parents):
        if (parent / 'src').is_dir():
            add_candidate(parent / 'src')
            break

    add_candidate(PathLib.home() / 'catkin_ws' / 'src')
    add_candidate(PathLib.home() / 'localization_ws' / 'src')
    return candidates


def _find_fast_lio_config_dir():
    """Find FAST-LIO config directory in the same workspace when available."""
    env_dir = os.environ.get('FAST_LIO_CONFIG_DIR')
    if env_dir:
        config_dir = PathLib(env_dir).expanduser()
        if config_dir.is_dir():
            return config_dir

    for src_dir in _workspace_src_candidates():
        for package_name in FAST_LIO_PACKAGE_CANDIDATES:
            config_dir = src_dir / package_name / 'config'
            if (config_dir / 'mapping_config.yaml').is_file() and (config_dir / 'localization_config.yaml').is_file():
                return config_dir
    return None


def _find_sibling_package_dir(package_name):
    """Find any sibling ROS package directory in the same workspace."""
    for src_dir in _workspace_src_candidates():
        pkg_dir = src_dir / package_name
        if pkg_dir.is_dir():
            return pkg_dir
    return None


def get_sibling_package_dirs():
    """Return auto-detected directories for sibling packages used by the UI."""
    result = {'success': True}
    for pkg in ('long_term_mapping', 'pose_graph_optimization', 'FAST_LIO_Localization_and_Mapping', 'FAST_LIO_ROS2'):
        pkg_dir = _find_sibling_package_dir(pkg)
        if pkg_dir:
            result[pkg] = str(pkg_dir)
    return result


def get_fast_lio_config_paths():
    config_dir = _find_fast_lio_config_dir()
    if not config_dir:
        return {
            'success': False,
            'message': 'FAST-LIO config directory not found in the current workspace',
            'searched_src_dirs': [str(p) for p in _workspace_src_candidates()],
        }

    return {
        'success': True,
        'config_dir': str(config_dir),
        'mapping_config': str(config_dir / 'mapping_config.yaml'),
        'localization_config': str(config_dir / 'localization_config.yaml'),
    }


def resolve_fast_lio_config_path(config_path):
    """Resolve legacy/default FAST_LIO_ROS2 paths to the discovered FAST-LIO config directory."""
    if not config_path:
        return config_path

    path = PathLib(config_path).expanduser()
    if path.exists():
        return str(path)

    if path.name not in ('mapping_config.yaml', 'localization_config.yaml'):
        return str(path)

    config_dir = _find_fast_lio_config_dir()
    if not config_dir:
        return str(path)

    resolved = config_dir / path.name
    return str(resolved) if resolved.is_file() else str(path)


def browse_directory(start_path="/home"):
    """Get list of directories and files in the given path"""
    try:
        entries = []
        path = PathLib(start_path).expanduser()

        # Add parent directory option
        if path.parent != path:
            entries.append({
                'name': '..',
                'path': str(path.parent),
                'is_dir': True,
                'is_file': False
            })

        # List directories and files (hide dotfiles / dot-directories)
        if path.exists() and path.is_dir():
            visible = sorted(
                e for e in path.iterdir() if not e.name.startswith('.')
            )
            for entry in visible:
                if entry.is_dir():
                    entries.append({
                        'name': entry.name,
                        'path': str(entry),
                        'is_dir': True,
                        'is_file': False
                    })
            for entry in visible:
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

        if parsed_path.path == '/api/ros_version':
            self.send_json_response({'version': 1})
            return
        elif parsed_path.path == '/api/server_config':
            self.send_json_response({
                'web_port': self.node.web_port,
                'pc2_ws_port': self.node.pc2_ws_port,
                'rosbridge_port': int(rospy.get_param('/rosbridge_websocket/port', 9090)),
            })
            return
        elif parsed_path.path == '/api/system/info':
            total_ram_mb = 0
            cpu_cores = 1
            if _psutil is not None:
                try:
                    total_ram_mb = int(_psutil.virtual_memory().total / (1024 * 1024))
                    cpu_cores = _psutil.cpu_count(logical=True) or 1
                except Exception:
                    pass
            self.send_json_response({'total_ram_mb': total_ram_mb, 'cpu_cores': cpu_cores})
        elif parsed_path.path == '/api/slam/state':
            self.send_json_response(self.node.get_slam_state())
        elif parsed_path.path == '/api/slam/save_map_status':
            self.send_json_response(self.node.get_save_map_status())
        elif parsed_path.path == '/api/slam/optimization_status':
            self.send_json_response(self.node.get_optimization_status())
        elif parsed_path.path == '/api/slam/default_config_paths':
            self.send_json_response(get_fast_lio_config_paths())
        elif parsed_path.path == '/api/slam/sibling_package_dirs':
            self.send_json_response(get_sibling_package_dirs())
        elif parsed_path.path == '/api/slam/result_paths':
            self.send_json_response(self.node.get_slam_result_paths())
        elif parsed_path.path == '/api/slam/save_map_result':
            query = parse_qs(parsed_path.query)
            directory = query.get('directory', [''])[0]
            self.send_json_response(self.node.get_save_map_result_paths(directory))
        elif parsed_path.path == '/api/slam/poses':
            query = parse_qs(parsed_path.query)
            file_path = query.get('path', [''])[0]
            self._serve_slam_poses(file_path)
        elif parsed_path.path == '/api/slam/edges':
            query = parse_qs(parsed_path.query)
            file_path = query.get('path', [''])[0]
            self._serve_slam_edges(file_path)
        elif parsed_path.path == '/api/slam/pcd':
            query = parse_qs(parsed_path.query)
            file_path = query.get('path', [''])[0]
            self._serve_slam_pcd(file_path)
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
            # PC2 전용: 현재 활성화된 PointCloud2 토픽 목록 (rosbridge 불필요)
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
        elif parsed_path.path == '/api/viewer/livox_topics':
            # Livox CustomMsg 토픽 목록 (Python 백엔드, rosbridge 불필요)
            try:
                all_topics = self.node.get_recorder_topics()
                livox_topics = [
                    t['name'] if isinstance(t, dict) else t
                    for t in all_topics
                    if (t.get('type', '') if isinstance(t, dict) else '') == 'livox_ros_driver2/msg/CustomMsg'
                ]
                self.send_json_response({'success': True, 'topics': livox_topics})
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
            success, message = self.node.run_slam_optimization()
            response = {'success': success, 'message': message, 'status': self.node.slam_status}
        elif parsed_path.path == '/api/slam/cancel_optimization':
            success, message = self.node.cancel_optimization()
            response = {'success': success, 'message': message}
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
        elif parsed_path.path == '/api/slam/cancel_save_map':
            success, message = self.node.cancel_save_slam_map()
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
            # ConPR → bag 전환 시 CustomMsg publisher 및 재생 정리 (같은 /livox/lidar 토픽 충돌 방지)
            if path:
                self.node._destroy_conpr_publishers()
                self.node.player_playing = False
                self.node.player_paused = False
                if self.node.playback_active:
                    self.node.playback_active = False
                    old_thread = self.node.playback_thread
                    self.node.playback_thread = None
                    if old_thread and old_thread.is_alive():
                        old_thread.join(timeout=1.0)
                self.node.stop_ros1_playback()
            self.node.invalidate_ros_topics_list_cache()
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
        elif parsed_path.path == '/api/bag/set_loop':
            loop = data.get('loop', False)
            self.node.bag_player_loop = bool(loop)
            if self.node.ros1_player_thread is not None:
                self.node.ros1_player_thread.set_loop(self.node.bag_player_loop)
            response = {'success': True, 'loop': self.node.bag_player_loop}

        # Bag Recorder API endpoints
        elif parsed_path.path == '/api/recorder/set_bag_name':
            bag_name = data.get('bag_name', '')
            success = self.node.set_recorder_bag_name(bag_name)
            response = {'success': success}
        elif parsed_path.path == '/api/recorder/record':
            topics = data.get('topics', [])
            bag_format = data.get('bag_format', 'ros2_mcap')
            success = self.node.record_bag(topics, bag_format=bag_format)
            response = {
                'success': success,
                'recording': self.node.recorder_recording,
                'mode': self.node.recorder_mode,
            }

        # SLAM Config API endpoints
        elif parsed_path.path == '/api/slam/load_config_file':
            config_path = resolve_fast_lio_config_path(data.get('path', ''))
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

                rospy.loginfo(f'Loaded config from: {config_path}')
            except Exception as e:
                rospy.logerr(f'Failed to load config: {str(e)}')
                response = {'success': False, 'message': str(e)}

        elif parsed_path.path == '/api/slam/save_config_file':
            config_path = resolve_fast_lio_config_path(data.get('path', ''))
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
                        if isinstance(config_data, dict):
                            config_data = convert_to_commented_map(config_params, config_data)
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

                rospy.loginfo(f'Saved config to: {config_path}')
                response = {
                    'success': True,
                    'message': 'Config saved successfully',
                    'path': config_path,
                }
            except Exception as e:
                rospy.logerr(f'Failed to save config: {str(e)}')
                import traceback
                traceback.print_exc()
                response = {'success': False, 'message': str(e)}

        elif parsed_path.path == '/api/slam/set_config_file':
            config_path = data.get('path', '')
            success, message = self.node.set_slam_config_file(config_path)
            response = {
                'success': success,
                'message': message,
                'path': message if success else None,
            }

        elif parsed_path.path == '/api/localization/set_config_file':
            config_path = data.get('path', '')
            success, message = self.node.set_localization_config_file(config_path)
            response = {
                'success': success,
                'message': message,
                'path': message if success else None,
            }

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
            # KITTI → ROS2 bag 또는 ROS1 .bag 변환
            # body: { "base_dir": "...", "calib_dir": "...", "data_path": "...",
            #         "drive_name": "...", "bag_format": "ros2"|"ros1" }
            base_dir   = data.get('base_dir', '')
            calib_dir  = data.get('calib_dir', '')
            data_path  = data.get('data_path', '')
            drive_name = data.get('drive_name', '')
            bag_format = data.get('bag_format', 'ros2')
            if not all([base_dir, calib_dir, data_path, drive_name]):
                response = {'success': False, 'error': 'Missing required fields: base_dir, calib_dir, data_path, drive_name'}
            else:
                response = self.node.start_kitti_conversion(
                    base_dir, calib_dir, data_path, drive_name, bag_format
                )

        elif parsed_path.path == '/api/player/scan_kaist':
            # KAIST 디렉토리 탐색
            # body: { "path": "/path/to/complex_urban" }
            path = data.get('path', '')
            if not path:
                response = {'success': False, 'error': 'Missing path'}
            elif not os.path.isdir(path):
                response = {'success': False, 'error': f'Directory not found: {path}'}
            else:
                response = self.node.scan_kaist_directory(path)

        elif parsed_path.path == '/api/player/convert_kaist':
            # KAIST → ROS1/ROS2 bag 변환
            # body: { "sequence_dir": "...", "output_path": "...", "sensors": [...], "bag_format": "ros2"|"ros1" }
            sequence_dir = data.get('sequence_dir', '')
            output_path = data.get('output_path', '')
            sensors = data.get('sensors')
            bag_format = data.get('bag_format', 'ros2')
            if not sequence_dir:
                response = {'success': False, 'error': 'Missing sequence_dir'}
            elif not output_path:
                response = {'success': False, 'error': 'Missing output_path'}
            else:
                response = self.node.start_kaist_conversion(
                    sequence_dir=sequence_dir,
                    output_path=output_path,
                    sensors=sensors,
                    bag_format=bag_format,
                )

        elif parsed_path.path == '/api/player/scan_mulran':
            path = data.get('path', '')
            if not path:
                response = {'success': False, 'error': 'Missing path'}
            elif not os.path.isdir(path):
                response = {'success': False, 'error': f'Directory not found: {path}'}
            else:
                response = self.node.scan_mulran_directory(path)

        elif parsed_path.path == '/api/player/convert_mulran':
            sequence_dir = data.get('sequence_dir', '')
            output_path = data.get('output_path', '')
            sensors = data.get('sensors')
            bag_format = data.get('bag_format', 'ros2')
            if not sequence_dir:
                response = {'success': False, 'error': 'Missing sequence_dir'}
            elif not output_path:
                response = {'success': False, 'error': 'Missing output_path'}
            else:
                response = self.node.start_mulran_conversion(
                    sequence_dir=sequence_dir,
                    output_path=output_path,
                    sensors=sensors,
                    bag_format=bag_format,
                )

        elif parsed_path.path == '/api/player/load_data':
            path = data.get('path', '')
            load_out = self.node.load_player_data(path)
            if isinstance(load_out, dict):
                response = {
                    'success': load_out.get('success', False),
                    'message': load_out.get('message', ''),
                    'dataset': load_out.get('dataset'),
                    'player_pc2_topics': load_out.get('player_pc2_topics'),
                }
            else:
                ok = bool(load_out)
                response = {
                    'success': ok,
                    'message': 'Data loaded' if ok else 'Failed to load data',
                    'dataset': None,
                    'player_pc2_topics': None,
                }
        elif parsed_path.path == '/api/player/play':
            success = self.node.player_play_toggle()
            response = {'success': success, 'playing': self.node.player_playing}
        elif parsed_path.path == '/api/player/pause':
            success = self.node.player_pause_toggle()
            response = {'success': success, 'paused': self.node.player_paused}
        elif parsed_path.path == '/api/player/save_bag':
            # body: { "bag_format": "ros2" | "ros1" }  (기본값 "ros2")
            bag_fmt = data.get('bag_format', 'ros2')
            started = self.node.start_save_rosbag(bag_format=bag_fmt)
            response = {'success': started, 'message': 'Save started' if started else 'Save already in progress'}
        elif parsed_path.path == '/api/player/set_loop':
            self.node.player_loop = data.get('loop', False)
            response = {'success': True}
        elif parsed_path.path == '/api/player/set_auto_start':
            self.node.player_auto_start = data.get('auto_start', False)
            response = {'success': True}
        elif parsed_path.path == '/api/player/set_rate':
            rate = float(data.get('rate', 1.0))
            rate = max(0.1, min(2.0, rate))
            self.node.player_speed = rate
            response = {'success': True, 'rate': rate}
        elif parsed_path.path == '/api/player/set_slider':
            position = data.get('position', 0)
            self.node.reset_player_position(position)
            response = {'success': True}

        self.send_json_response(response)

    def _is_allowed_slam_path(self, file_path):
        """Check if file_path is under one of the permitted SLAM directories."""
        if not file_path:
            return False
        result_paths = self.node.get_slam_result_paths()
        allowed_dirs = [
            self.node.slam_map1,
            self.node.slam_map2,
            result_paths.get('output_dir', ''),
        ]

        # Save Map 결과 저장 디렉토리 (pose_graph_optimization/{directory})도 허용
        saved_dir = getattr(self.node, 'slam_last_saved_dir', '')
        if saved_dir:
            pgo_dir = _find_sibling_package_dir('pose_graph_optimization')
            if pgo_dir:
                allowed_dirs.append(str(pgo_dir / saved_dir))

        return any(allowed and file_path.startswith(allowed) for allowed in allowed_dirs)

    def _serve_slam_poses(self, file_path):
        """Parse optimized_poses.txt and return pose array as JSON."""
        if not self._is_allowed_slam_path(file_path):
            self.send_json_response({'success': False, 'error': 'Invalid or unauthorized path'})
            return
        try:
            poses = []
            with open(file_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    parts = line.split()
                    if len(parts) >= 8:
                        # Format: timestamp x y z qx qy qz qw
                        poses.append({
                            'x': float(parts[1]),
                            'y': float(parts[2]),
                            'z': float(parts[3]),
                            'qx': float(parts[4]),
                            'qy': float(parts[5]),
                            'qz': float(parts[6]),
                            'qw': float(parts[7]),
                        })
            self.send_json_response({'success': True, 'poses': poses})
        except FileNotFoundError:
            self.send_json_response({'success': False, 'error': 'File not found', 'poses': []})
        except Exception as e:
            self.send_json_response({'success': False, 'error': str(e), 'poses': []})

    def _serve_slam_edges(self, file_path):
        """Parse edges.txt and return loop closure edges (|from_idx - to_idx| != 1) as JSON."""
        if not self._is_allowed_slam_path(file_path):
            self.send_json_response({'success': False, 'error': 'Invalid or unauthorized path'})
            return
        try:
            loop_closures = []
            with open(file_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    parts = line.split()
                    if len(parts) < 2:
                        continue
                    from_idx = int(parts[0])
                    to_idx = int(parts[1])
                    if abs(from_idx - to_idx) != 1:
                        loop_closures.append({'from_idx': from_idx, 'to_idx': to_idx})
            self.send_json_response({'success': True, 'loop_closures': loop_closures})
        except FileNotFoundError:
            self.send_json_response({'success': False, 'error': 'File not found', 'loop_closures': []})
        except Exception as e:
            self.send_json_response({'success': False, 'error': str(e), 'loop_closures': []})

    def _serve_slam_pcd(self, file_path):
        """Serve a PCD file as binary octet-stream."""
        if not self._is_allowed_slam_path(file_path):
            self.send_json_response({'success': False, 'error': 'Invalid or unauthorized path'})
            return
        try:
            with open(file_path, 'rb') as f:
                data = f.read()
            self.send_response(200)
            self.send_header('Content-Type', 'application/octet-stream')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(data)
        except FileNotFoundError:
            self.send_json_response({'success': False, 'error': 'File not found'})
        except Exception as e:
            self.send_json_response({'success': False, 'error': str(e)})

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


def run_web_server(node, port=8880):
    global _web_server

    if _web_server is not None:
        rospy.logwarn(f'Web server already running; skipping duplicate start.')
        return

    WebRequestHandler.node = node

    # Determine web directory using rospkg (ROS1 native)
    web_dir = None
    try:
        import rospkg
        rospack = rospkg.RosPack()
        share_dir = rospack.get_path('ros_slam_webui')
        web_dir = os.path.join(share_dir, 'web')
        rospy.loginfo(f'Using web directory: {web_dir}')
    except Exception as e:
        # Fallback for development
        web_dir = os.path.join(os.path.dirname(__file__), '..', 'web')
        web_dir = os.path.abspath(web_dir)
        rospy.loginfo(f'Using fallback web directory: {web_dir}')

    # Check if web directory exists
    if not os.path.exists(web_dir):
        rospy.logerr(f'Web directory not found: {web_dir}')
        return

    # Check if index.html exists
    index_path = os.path.join(web_dir, 'index.html')
    if not os.path.exists(index_path):
        rospy.logerr(f'index.html not found: {index_path}')
        return

    WebRequestHandler.web_dir = web_dir
    try:
        _web_server = ThreadedHTTPServer(('0.0.0.0', port), WebRequestHandler)
    except OSError as e:
        if getattr(e, 'errno', None) == 98:
            rospy.logerr(_format_port_in_use_error(port, 'web_port'))
        else:
            rospy.logerr(f'Failed to start web server on port {port}: {e}')
        _web_server = None
        return

    # Get local IP for network access
    local_ip = get_local_ip()

    rospy.loginfo(f'======================================')
    rospy.loginfo(f'Web server started on port {port}')
    rospy.loginfo(f'Local access:   http://localhost:{port}')
    rospy.loginfo(f'Network access: http://{local_ip}:{port}')
    rospy.loginfo(f'======================================')

    try:
        _web_server.serve_forever()
    except Exception as e:
        rospy.logerr(f'Web server error: {str(e)}')
    finally:
        if _web_server is not None:
            _web_server.server_close()
            _web_server = None


def signal_handler(signum, frame):
    """Handle SIGTERM and SIGINT for graceful shutdown"""
    global _web_server, _ros_node
    
    signal_name = signal.Signals(signum).name
    logger_msg = f'Received {signal_name}, shutting down gracefully...'
    if _ros_node:
        rospy.loginfo(logger_msg)
    else:
        print(logger_msg)
    
    # Shutdown web server
    if _web_server:
        shutdown_msg = 'Shutting down web server...'
        if _ros_node:
            rospy.loginfo(shutdown_msg)
        else:
            print(shutdown_msg)
        _web_server.shutdown()

    # Clean up ROS node
    if _ros_node:
        rospy.loginfo('Cleaning up processes...')
        _ros_node.kill_slam_processes()
        _ros_node.kill_localization_processes()

    # Exit
    import sys
    sys.exit(0)


def main(args=None):
    global _ros_node, _web_server_thread

    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    _ros_node = WebGUINode()
    web_port = _ros_node.web_port

    # Start web server in a separate thread
    if _web_server_thread is None or not _web_server_thread.is_alive():
        _web_server_thread = threading.Thread(
            target=run_web_server, args=(_ros_node, web_port), daemon=True
        )
        _web_server_thread.start()

    local_ip = get_local_ip()
    rospy.loginfo(f'Web GUI is running with full ROS1 integration.')
    rospy.loginfo(
        f'Open http://localhost:{web_port} or http://{local_ip}:{web_port} in your browser.'
    )

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Keyboard interrupt received')
    finally:
        rospy.loginfo('Cleaning up...')
        _ros_node.kill_slam_processes()
        _ros_node.kill_localization_processes()


if __name__ == '__main__':
    main()
