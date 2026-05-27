#!/usr/bin/env python3
"""
mulran_converter.py
MulRan Dataset → ROS2 bag (.db3) / ROS1 bag (.bag) 변환 모듈

참고 구현체: /home/kkw/dataset/Mulran/file_player_mulran-master/src/ROSThread.cpp

출력 토픽 (Direct Play / Bag 변환):
  /os1_points          sensor_msgs/msg/PointCloud2  (Ouster LiDAR, frame_id=ouster)
  /radar/polar         sensor_msgs/msg/Image        (MONO8, frame_id=radar_polar)
  /imu/data_raw        sensor_msgs/msg/Imu          (frame_id=imu)
  /gps/fix             sensor_msgs/msg/NavSatFix     (frame_id=gps)
  /gt                  nav_msgs/msg/Odometry        (GT pose, frame_id=world)
  /tf                  tf2_msgs/msg/TFMessage       (dynamic: world→base_link, global_pose 기반)
  /tf_static           tf2_msgs/msg/TFMessage       (base_link→ouster, base_link→radar_polar, 고정 외장값)
  /clock               rosgraph_msgs/msg/Clock      (10ms 이상 간격 시만 publish)

고정 외장 (MulRan 차량 기준, txt 파일 미사용 — 상수 MULRAN_CALIB_*):
  Ouster: x,y,z (m), r,p,y (deg) / 레이더: 동일

디렉토리 구조 지원:
  [구형] {seq}/sensor_data/data_stamp.csv, Ouster/, radar/polar/, gps.csv, xsens_imu.csv
  [신형] {seq}/data_stamp.csv, Ouster/, radar/polar/, gps.csv, xsens_imu.csv
  global_pose.csv는 항상 {seq}/에 위치
"""

import bisect
import math
import os
import shutil
import struct
from pathlib import Path

import numpy as np
from builtin_interfaces.msg import Time

# 변환(bag 쓰기) 전용 의존성 — 없어도 직접 재생(scan/play)은 정상 동작
try:
    import rosbag2_py
    from rosbag2_py import TopicMetadata
    from rclpy.serialization import serialize_message
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, Imu, NavSatFix, NavSatStatus, PointCloud2, PointField
    from tf2_msgs.msg import TFMessage
    _ROSBAG2_AVAILABLE = True
except ImportError:
    rosbag2_py = None
    TopicMetadata = None
    serialize_message = None
    TransformStamped = None
    Odometry = None
    Image = None
    Imu = None
    NavSatFix = None
    NavSatStatus = None
    PointCloud2 = None
    PointField = None
    TFMessage = None
    _ROSBAG2_AVAILABLE = False

# CV2는 선택적 의존성 (레이더 PNG 로딩)
try:
    import cv2
    _CV2_AVAILABLE = True
except ImportError:
    cv2 = None
    _CV2_AVAILABLE = False

# /clock publish 최소 간격 (참고: ROSThread.cpp의 10ms 정책)
_CLOCK_MIN_INTERVAL_NS = 10_000_000  # 10 ms

# TF 트리 (global_pose의 child_frame_id=base_link 와 정합)
MULRAN_FRAME_BASE_LINK = 'base_link'
MULRAN_FRAME_OUSTER = 'ouster'
MULRAN_FRAME_RADAR_POLAR = 'radar_polar'

# MulRan 고정 외장 (구 calib_base2outer.txt / calib_base2radar.txt 수치를 코드에 반영)
# (x, y, z) m, (roll, pitch, yaw) deg — ZYX Tait–Bryan → quaternion
MULRAN_CALIB_OUTER_XYZ_RPY_M_DEG = (
    1.7042, -0.021, 1.8047, 0.0001, 0.0003, 179.6654)
MULRAN_CALIB_RADAR_XYZ_RPY_M_DEG = (1.5, -0.04, 1.97, 0.0, 0.0, 0.9)


class MulRanConverter:
    """MulRan Dataset을 ROS2/ROS1 bag 파일로 변환하는 클래스.

    참고 코드:
        ROSThread.cpp (file_player_mulran-master) — data_stamp.csv 기반 멀티스레드 publish,
        Ouster 바이너리, 레이더 폴라 이미지, GPS/IMU 로드, SaveRosbag() 로직.

    Usage:
        converter = MulRanConverter()
        result = converter.scan_directory('/path/to/mulran')
        converter.convert_to_ros2bag(
            sequence_dir=result['sequences'][0]['path'],
            output_path='/path/to/output_bag',
            progress_cb=lambda pct, msg: print(f'{pct}% - {msg}')
        )
    """

    # ──────────────────────────────────────────────────────────────
    # Public API
    # ──────────────────────────────────────────────────────────────

    def scan_directory(self, base_dir: str) -> dict:
        """MulRan 데이터셋 디렉토리를 탐색하여 시퀀스 목록을 반환한다.

        사용자가 ``.../Mulran`` 같은 **데이터셋 루트**만 골라도, 그 아래 ``ParkingLot``,
        ``DCC01`` 등 ``data_stamp.csv`` 가 있는 시퀀스를 재귀적으로 모은다.

        구형(sensor_data/data_stamp.csv)과 신형(시퀀스 루트/data_stamp.csv) 모두 지원.

        Args:
            base_dir: 사용자가 선택한 디렉토리 (시퀀스 한 개 또는 상위 루트)

        Returns:
            {'success': True, 'sequences': [{name, path, sensor_data_dir}, ...]}
        """
        base_dir = os.path.abspath(base_dir)
        sequences: list[dict] = []
        seen_paths: set[str] = set()

        skip_names = {
            'file_player_mulran-master',
            '__pycache__',
        }

        def _try_add(path: str) -> bool:
            ap = os.path.abspath(path)
            if ap in seen_paths:
                return False
            bn = os.path.basename(ap)
            if bn == 'sensor_data' or bn.startswith('.'):
                return False
            if bn.endswith('_converted'):
                return False
            sensor_data_dir = self._find_sensor_data_dir(ap)
            if sensor_data_dir is None:
                return False
            seen_paths.add(ap)
            sequences.append({
                'name': bn,
                'path': ap,
                'sensor_data_dir': sensor_data_dir,
            })
            return True

        def _walk(root: str, depth: int, max_depth: int) -> None:
            ap = os.path.abspath(root)
            if depth > max_depth:
                return
            _try_add(ap)
            if not os.path.isdir(ap):
                return
            try:
                subs = sorted(os.listdir(ap))
            except OSError:
                return
            for name in subs:
                if name in skip_names or name.startswith('.'):
                    continue
                full = os.path.join(ap, name)
                if not os.path.isdir(full):
                    continue
                _walk(full, depth + 1, max_depth)

        if os.path.isdir(base_dir):
            _walk(base_dir, 0, 6)
        else:
            _try_add(base_dir)

        sequences.sort(key=lambda s: s['name'].lower())
        return {'success': True, 'sequences': sequences}

    def convert_to_ros2bag(
        self,
        sequence_dir: str,
        output_path: str,
        sensors: list | None = None,
        progress_cb=None,
    ) -> None:
        """MulRan 시퀀스를 ROS2 bag (.db3) 파일로 변환한다.

        Args:
            sequence_dir: 시퀀스 루트 디렉토리 (data_stamp.csv 또는 sensor_data/ 포함)
            output_path: 출력 bag 경로 (확장자 없음)
            sensors: 포함할 센서 목록 (None이면 전체). 예: ['ouster','radar','imu','gps','gt']
            progress_cb: 진행률 콜백 signature: progress_cb(progress: int, message: str)
        """
        if not _ROSBAG2_AVAILABLE:
            raise RuntimeError(
                'rosbag2_py가 필요합니다. ROS2 환경에서 source setup 후 실행하세요.'
            )

        _sent_max = [-1]

        def _progress(pct: int, msg: str):
            if progress_cb and pct > _sent_max[0]:
                _sent_max[0] = pct
                progress_cb(pct, msg)

        _progress(0, 'Loading MulRan data...')
        ctx = self._load_sequence_context(sequence_dir)

        if not ctx['data_stamps']:
            raise RuntimeError(
                'data_stamp.csv가 비어 있거나 파싱할 수 없습니다. 시퀀스 경로를 확인하세요.'
            )

        # ── 출력 경로 준비 ──────────────────────────────────────────
        if os.path.exists(output_path):
            shutil.rmtree(output_path)
        parent_dir = os.path.dirname(output_path)
        if parent_dir:
            os.makedirs(parent_dir, exist_ok=True)

        _progress(2, 'Initializing ROS2 bag writer...')
        writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(uri=output_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        )
        writer.open(storage_options, converter_options)

        # ── 토픽 등록 ──────────────────────────────────────────────
        topics = self._get_topic_list(ctx, sensors)
        for idx, (topic_name, topic_type) in enumerate(topics):
            writer.create_topic(TopicMetadata(
                id=idx,
                name=topic_name,
                type=topic_type,
                serialization_format='cdr',
            ))

        # ── /tf_static 1회 기록 (캘리브가 있을 때) ─────────────────
        first_ns = ctx['data_stamps'][0][0]
        stamp0 = self._ns_to_time_msg(first_ns)
        tf_static_msg = self.build_mulran_tf_static_message(
            stamp0,
            ctx.get('calib_ouster_xyz_rpy'),
            ctx.get('calib_radar_xyz_rpy'),
        )
        topic_names = [t[0] for t in topics]
        if tf_static_msg and '/tf_static' in topic_names and serialize_message:
            writer.write('/tf_static', serialize_message(tf_static_msg), first_ns)

        _progress(3, 'Writing MulRan messages...')
        total = len(ctx['data_stamps'])
        processed = 0
        _last_pct = [0]

        def _tick(msg: str):
            nonlocal processed
            processed += 1
            pct = min(3 + int(processed / total * 95), 98)
            if pct >= _last_pct[0] + 2 or pct >= 98:
                _last_pct[0] = pct
                _progress(pct, msg)

        for ts_ns, sensor_name in ctx['data_stamps']:
            stamp_time = self._ns_to_time_msg(ts_ns)
            self._write_sensor_ros2(
                writer, ctx, sensors, ts_ns, stamp_time, sensor_name
            )
            _tick(f'Converting {sensor_name}...')

        del writer
        _progress(100, 'Conversion complete!')

    def convert_to_ros1bag(
        self,
        sequence_dir: str,
        output_bag_path: str,
        sensors: list | None = None,
        progress_cb=None,
    ) -> None:
        """MulRan 시퀀스를 ROS1 .bag 파일로 변환한다 (rosbags 라이브러리 사용).

        rosbags.rosbag1.Writer + migrate_bytes()로 CDR → ROS1 raw bytes 변환.

        Args:
            sequence_dir: 시퀀스 루트 디렉토리
            output_bag_path: 출력 .bag 파일 경로
            sensors: 포함할 센서 목록 (None이면 전체)
            progress_cb: 진행률 콜백 signature: progress_cb(progress: int, message: str)
        """
        try:
            from rosbags.rosbag1 import Writer as Ros1Writer
            from rosbags.typesys import get_typestore, Stores
            from rosbags.convert.converter import migrate_bytes as _migrate_bytes
        except ImportError as e:
            raise RuntimeError(
                f'rosbags 라이브러리가 필요합니다. 설치: pip install rosbags\n원인: {e}'
            )

        _sent_max = [-1]

        def _progress(pct: int, msg: str):
            if progress_cb and pct > _sent_max[0]:
                _sent_max[0] = pct
                progress_cb(pct, msg)

        _progress(0, 'Loading MulRan data...')
        ctx = self._load_sequence_context(sequence_dir)

        if not ctx['data_stamps']:
            raise RuntimeError(
                'data_stamp.csv가 비어 있거나 파싱할 수 없습니다. 시퀀스 경로를 확인하세요.'
            )

        src_typestore = get_typestore(Stores.ROS2_JAZZY)
        dst_typestore = get_typestore(Stores.ROS1_NOETIC)
        migrate_cache: dict = {}

        # ── 출력 경로 준비 ──────────────────────────────────────────
        if os.path.isfile(output_bag_path):
            os.remove(output_bag_path)
        parent_dir = os.path.dirname(output_bag_path)
        if parent_dir:
            os.makedirs(parent_dir, exist_ok=True)

        def _ensure_type(ros2_type: str) -> bool:
            if ros2_type in dst_typestore.fielddefs:
                return True
            try:
                from rosbags.typesys import get_types_from_msg
                typs = get_types_from_msg(
                    src_typestore.generate_msgdef(ros2_type, ros_version=1)[0],
                    ros2_type,
                )
                typs.pop('std_msgs/msg/Header', None)
                dst_typestore.register(typs)
                return True
            except Exception:
                return False

        def _cdr_to_ros1(conn, cdr_bytes: bytes) -> bytes:
            return bytes(_migrate_bytes(
                src_typestore, dst_typestore,
                conn.msgtype, conn.msgtype,
                migrate_cache, cdr_bytes,
                src_is2=True, dst_is2=False,
            ))

        topics = self._get_topic_list(ctx, sensors)
        total = len(ctx['data_stamps'])
        processed = 0
        _last_pct = [0]

        def _tick(msg: str):
            nonlocal processed
            processed += 1
            pct = min(3 + int(processed / total * 95), 98)
            if pct >= _last_pct[0] + 2 or pct >= 98:
                _last_pct[0] = pct
                _progress(pct, msg)

        _progress(2, 'Initializing ROS1 bag writer...')
        with Ros1Writer(output_bag_path) as writer:
            connections: dict = {}
            for topic_name, ros2_type in topics:
                if not _ensure_type(ros2_type):
                    continue
                try:
                    conn = writer.add_connection(
                        topic_name, ros2_type, typestore=dst_typestore
                    )
                    connections[topic_name] = conn
                except Exception:
                    pass

            def _write(topic_name: str, ros2_msg, ts_ns: int):
                conn = connections.get(topic_name)
                if conn is None:
                    return
                try:
                    cdr = bytes(serialize_message(ros2_msg))
                    raw = _cdr_to_ros1(conn, cdr)
                    writer.write(conn, ts_ns, raw)
                except Exception:
                    pass

            first_ns = ctx['data_stamps'][0][0]
            stamp0 = self._ns_to_time_msg(first_ns)
            tf_static_msg = self.build_mulran_tf_static_message(
                stamp0,
                ctx.get('calib_ouster_xyz_rpy'),
                ctx.get('calib_radar_xyz_rpy'),
            )
            if tf_static_msg and '/tf_static' in connections:
                _write('/tf_static', tf_static_msg, first_ns)

            _progress(3, 'Writing MulRan messages...')
            for ts_ns, sensor_name in ctx['data_stamps']:
                stamp_time = self._ns_to_time_msg(ts_ns)
                self._write_sensor_ros1(
                    _write, ctx, sensors, ts_ns, stamp_time, sensor_name
                )
                _tick(f'Converting {sensor_name}...')

        _progress(100, 'Conversion complete!')

    # ──────────────────────────────────────────────────────────────
    # 디렉토리 감지 헬퍼
    # ──────────────────────────────────────────────────────────────

    def _find_sensor_data_dir(self, sequence_dir: str) -> str | None:
        """sequence_dir에서 sensor_data 루트를 찾아 반환한다.

        구형: {seq}/sensor_data/data_stamp.csv → sensor_data_dir = {seq}/sensor_data
        신형: {seq}/data_stamp.csv             → sensor_data_dir = {seq}

        Returns:
            sensor_data 디렉토리 경로 또는 None (MulRan 아닌 경우)
        """
        old_sensor_dir = os.path.join(sequence_dir, 'sensor_data')
        if os.path.isfile(os.path.join(old_sensor_dir, 'data_stamp.csv')):
            return old_sensor_dir
        if os.path.isfile(os.path.join(sequence_dir, 'data_stamp.csv')):
            return sequence_dir
        return None

    def _rpy_deg_zyx_to_quaternion(
        self, roll_deg: float, pitch_deg: float, yaw_deg: float,
    ) -> tuple[float, float, float, float]:
        """ZYX Tait–Bryan (intrinsic: R = Rz*Ry*Rx) 오일러를 quaternion (x,y,z,w)로 변환."""
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return float(qx), float(qy), float(qz), float(qw)

    def _make_transform_stamped(
        self,
        parent: str,
        child: str,
        xyz: tuple[float, float, float],
        quat_xyzw: tuple[float, float, float, float],
        stamp: Time,
    ):
        """base_link → 센서 정적 변환 (ROS: parent→child)."""
        if TransformStamped is None:
            return None
        qx, qy, qz, qw = quat_xyzw
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(xyz[0])
        t.transform.translation.y = float(xyz[1])
        t.transform.translation.z = float(xyz[2])
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        return t

    def build_mulran_tf_static_message(
        self,
        stamp: Time,
        calib_ouster: tuple[float, float, float, float, float, float] | None,
        calib_radar: tuple[float, float, float, float, float, float] | None,
    ):
        """base_link→ouster / base_link→radar_polar 정적 TF (외장 수치 튜플)."""
        if TFMessage is None:
            return None
        transforms = []
        if calib_ouster:
            x, y, z, rr, pp, yy = calib_ouster
            q = self._rpy_deg_zyx_to_quaternion(rr, pp, yy)
            ts = self._make_transform_stamped(
                MULRAN_FRAME_BASE_LINK, MULRAN_FRAME_OUSTER, (x, y, z), q, stamp)
            if ts:
                transforms.append(ts)
        if calib_radar:
            x, y, z, rr, pp, yy = calib_radar
            q = self._rpy_deg_zyx_to_quaternion(rr, pp, yy)
            ts = self._make_transform_stamped(
                MULRAN_FRAME_BASE_LINK, MULRAN_FRAME_RADAR_POLAR, (x, y, z), q, stamp)
            if ts:
                transforms.append(ts)
        if not transforms:
            return None
        msg = TFMessage()
        msg.transforms = transforms
        return msg

    def _want_tf_static_in_bag(self, _ctx: dict, sensors: list | None) -> bool:
        if sensors is None:
            return True
        if 'tf_static' in sensors:
            return True
        if 'ouster' in sensors or 'radar' in sensors:
            return True
        return False

    def _get_ouster_dir(self, sensor_data_dir: str, sequence_dir: str | None = None) -> str | None:
        """Ouster .bin 파일이 있는 디렉토리를 반환한다."""
        candidates = []
        if sequence_dir:
            candidates.extend([
                os.path.join(sequence_dir, 'sensor_data', 'Ouster'),
                os.path.join(sequence_dir, 'sensor_data', 'ouster'),
            ])
        for name in ('Ouster', 'ouster'):
            candidates.append(os.path.join(sensor_data_dir, name))
        for d in candidates:
            if os.path.isdir(d):
                return d
        return None

    def _get_radar_polar_dir(self, sensor_data_dir: str, sequence_dir: str) -> str | None:
        """radar/polar PNG 파일이 있는 디렉토리를 반환한다."""
        candidates = [
            os.path.join(sensor_data_dir, 'radar', 'polar'),
            os.path.join(sensor_data_dir, 'polar'),
            os.path.join(sequence_dir, 'radar', 'polar'),
            os.path.join(sequence_dir, 'sensor_data', 'radar', 'polar'),
        ]
        for d in candidates:
            if os.path.isdir(d):
                return d
        return None

    # ──────────────────────────────────────────────────────────────
    # 시퀀스 컨텍스트 로딩 (공통)
    # ──────────────────────────────────────────────────────────────

    def _load_sequence_context(self, sequence_dir: str) -> dict:
        """변환에 필요한 모든 데이터를 메모리에 로드하여 컨텍스트 dict로 반환한다.

        Returns:
            {
              'sequence_dir': str,
              'sensor_data_dir': str,
              'data_stamps': list of (stamp_ns: int, sensor_name: str),
              'ouster_dir': str | None,
              'radar_dir': str | None,
              'gps_bisect': (stamps_list, rows_list),   # bisect 검색용
              'imu_bisect': (stamps_list, rows_list),
              'imu_version': int,  # 1 (orientation only) or 2 (full)
              'global_poses': list of (stamp_ns, R, T),  # T는 절대 좌표
              'pose_stamps': list of int,  # bisect용
            }
        """
        sequence_dir = os.path.abspath(sequence_dir)
        sensor_data_dir = self._find_sensor_data_dir(sequence_dir)
        if sensor_data_dir is None:
            raise RuntimeError(
                f'MulRan 시퀀스를 인식할 수 없습니다. data_stamp.csv를 확인하세요: {sequence_dir}'
            )

        stamp_csv = os.path.join(sensor_data_dir, 'data_stamp.csv')
        data_stamps = self._parse_data_stamp(stamp_csv)

        ouster_dir = self._get_ouster_dir(sensor_data_dir, sequence_dir)
        radar_dir = self._get_radar_polar_dir(sensor_data_dir, sequence_dir)

        gps_path = os.path.join(sensor_data_dir, 'gps.csv')
        if not os.path.isfile(gps_path):
            gps_path = os.path.join(sequence_dir, 'sensor_data', 'gps.csv')
        imu_path = os.path.join(sensor_data_dir, 'xsens_imu.csv')
        if not os.path.isfile(imu_path):
            imu_path = os.path.join(sequence_dir, 'sensor_data', 'xsens_imu.csv')

        gps_rows = self._load_gps_csv(gps_path)
        imu_rows, imu_version = self._load_imu_csv(imu_path)

        global_poses = self._parse_global_pose(os.path.join(sequence_dir, 'global_pose.csv'))

        gps_bisect = self._to_bisect(gps_rows)
        imu_bisect = self._to_bisect(imu_rows)

        poses_sorted = sorted(global_poses, key=lambda p: p[0]) if global_poses else []
        pose_stamps = [p[0] for p in poses_sorted]

        return {
            'sequence_dir': sequence_dir,
            'sensor_data_dir': sensor_data_dir,
            'data_stamps': data_stamps,
            'ouster_dir': ouster_dir,
            'radar_dir': radar_dir,
            'gps_bisect': gps_bisect,
            'imu_bisect': imu_bisect,
            'imu_version': imu_version,
            'global_poses': poses_sorted,
            'pose_stamps': pose_stamps,
            'calib_ouster_xyz_rpy': MULRAN_CALIB_OUTER_XYZ_RPY_M_DEG,
            'calib_radar_xyz_rpy': MULRAN_CALIB_RADAR_XYZ_RPY_M_DEG,
        }

    def _get_topic_list(self, ctx: dict, sensors: list | None) -> list:
        """변환할 토픽 목록을 반환한다.

        Returns:
            list of (topic_name: str, ros2_type: str)
        """
        topics = []

        def _want(sensor: str) -> bool:
            return sensors is None or sensor in sensors

        if ctx['ouster_dir'] and _want('ouster'):
            topics.append(('/os1_points', 'sensor_msgs/msg/PointCloud2'))
        if ctx['radar_dir'] and _want('radar'):
            topics.append(('/radar/polar', 'sensor_msgs/msg/Image'))
        if ctx['imu_bisect'][0] and _want('imu'):
            topics.append(('/imu/data_raw', 'sensor_msgs/msg/Imu'))
        if ctx['gps_bisect'][0] and _want('gps'):
            topics.append(('/gps/fix', 'sensor_msgs/msg/NavSatFix'))
        if ctx['global_poses'] and _want('gt'):
            topics.append(('/gt', 'nav_msgs/msg/Odometry'))
            topics.append(('/tf', 'tf2_msgs/msg/TFMessage'))

        if self._want_tf_static_in_bag(ctx, sensors):
            topics.append(('/tf_static', 'tf2_msgs/msg/TFMessage'))

        return topics

    # ──────────────────────────────────────────────────────────────
    # 센서별 쓰기 (ROS2 bag)
    # ──────────────────────────────────────────────────────────────

    def _write_sensor_ros2(
        self,
        writer,
        ctx: dict,
        sensors: list | None,
        ts_ns: int,
        stamp_time: Time,
        sensor_name: str,
    ) -> None:
        """data_stamp의 한 항목에 해당하는 메시지를 ROS2 bag writer에 기록한다."""
        def _want(s: str) -> bool:
            return sensors is None or s in sensors

        sn = sensor_name.lower()

        if sn == 'ouster' and ctx['ouster_dir'] and _want('ouster'):
            bin_path = os.path.join(ctx['ouster_dir'], f'{ts_ns}.bin')
            msg = self._make_ouster_pc2(bin_path, stamp_time)
            if msg:
                writer.write('/os1_points', serialize_message(msg), ts_ns)

        elif sn == 'radar' and ctx['radar_dir'] and _want('radar'):
            png_path = os.path.join(ctx['radar_dir'], f'{ts_ns}.png')
            msg = self._make_radar_image(png_path, stamp_time)
            if msg:
                writer.write('/radar/polar', serialize_message(msg), ts_ns)

        elif sn == 'imu' and ctx['imu_bisect'][0] and _want('imu'):
            row = self._find_nearest(ctx['imu_bisect'], ts_ns)
            if row:
                msg = self._make_imu_msg(row, stamp_time, ctx['imu_version'])
                writer.write('/imu/data_raw', serialize_message(msg), ts_ns)

        elif sn == 'gps' and ctx['gps_bisect'][0] and _want('gps'):
            row = self._find_nearest(ctx['gps_bisect'], ts_ns)
            if row:
                msg = self._make_navsatfix_msg(row, stamp_time)
                writer.write('/gps/fix', serialize_message(msg), ts_ns)

        # GT pose는 모든 센서 이벤트에서 nearest 포즈를 기록
        if ctx['global_poses'] and _want('gt'):
            pose = self._find_nearest_pose(ctx['pose_stamps'], ctx['global_poses'], ts_ns)
            if pose:
                _, R, T = pose
                odom_msg = self._make_gt_odometry(R, T, stamp_time)
                tf_msg = self._make_dynamic_tf(R, T, stamp_time)
                if odom_msg:
                    writer.write('/gt', serialize_message(odom_msg), ts_ns)
                if tf_msg:
                    writer.write('/tf', serialize_message(tf_msg), ts_ns)

    # ──────────────────────────────────────────────────────────────
    # 센서별 쓰기 (ROS1 bag)
    # ──────────────────────────────────────────────────────────────

    def _write_sensor_ros1(
        self,
        _write,
        ctx: dict,
        sensors: list | None,
        ts_ns: int,
        stamp_time: Time,
        sensor_name: str,
    ) -> None:
        """data_stamp의 한 항목에 해당하는 메시지를 ROS1 bag writer에 기록한다."""
        def _want(s: str) -> bool:
            return sensors is None or s in sensors

        sn = sensor_name.lower()

        if sn == 'ouster' and ctx['ouster_dir'] and _want('ouster'):
            bin_path = os.path.join(ctx['ouster_dir'], f'{ts_ns}.bin')
            msg = self._make_ouster_pc2(bin_path, stamp_time)
            if msg:
                _write('/os1_points', msg, ts_ns)

        elif sn == 'radar' and ctx['radar_dir'] and _want('radar'):
            png_path = os.path.join(ctx['radar_dir'], f'{ts_ns}.png')
            msg = self._make_radar_image(png_path, stamp_time)
            if msg:
                _write('/radar/polar', msg, ts_ns)

        elif sn == 'imu' and ctx['imu_bisect'][0] and _want('imu'):
            row = self._find_nearest(ctx['imu_bisect'], ts_ns)
            if row:
                msg = self._make_imu_msg(row, stamp_time, ctx['imu_version'])
                _write('/imu/data_raw', msg, ts_ns)

        elif sn == 'gps' and ctx['gps_bisect'][0] and _want('gps'):
            row = self._find_nearest(ctx['gps_bisect'], ts_ns)
            if row:
                msg = self._make_navsatfix_msg(row, stamp_time)
                _write('/gps/fix', msg, ts_ns)

        if ctx['global_poses'] and _want('gt'):
            pose = self._find_nearest_pose(ctx['pose_stamps'], ctx['global_poses'], ts_ns)
            if pose:
                _, R, T = pose
                odom_msg = self._make_gt_odometry(R, T, stamp_time)
                tf_msg = self._make_dynamic_tf(R, T, stamp_time)
                if odom_msg:
                    _write('/gt', odom_msg, ts_ns)
                if tf_msg:
                    _write('/tf', tf_msg, ts_ns)

    # ──────────────────────────────────────────────────────────────
    # CSV 파싱 헬퍼
    # ──────────────────────────────────────────────────────────────

    def _parse_data_stamp(self, filepath: str) -> list:
        """data_stamp.csv를 파싱하여 [(stamp_ns, sensor_name), ...] 리스트를 반환한다.

        포맷: {nanosec_stamp},{sensor_name}
        참고: ROSThread.cpp의 multimap<int64_t, string> 구조와 동일한 의미.
        동일 stamp에 복수 센서가 허용된다.

        Args:
            filepath: data_stamp.csv 경로

        Returns:
            정렬된 (stamp_ns: int, sensor_name: str) 튜플 리스트
        """
        result = []
        if not os.path.exists(filepath):
            return result

        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = line.split(',', 1)
                if len(parts) != 2:
                    continue
                try:
                    stamp_ns = int(parts[0])
                    sensor_name = parts[1].strip()
                    result.append((stamp_ns, sensor_name))
                except ValueError:
                    continue

        result.sort(key=lambda x: x[0])
        return result

    def _parse_global_pose(self, filepath: str) -> list:
        """global_pose.csv를 파싱하여 [(stamp_ns, R, T), ...] 리스트를 반환한다.

        포맷: stamp, r00,r01,r02,tx, r10,r11,r12,ty, r20,r21,r22,tz
        참고: ROSThread.cpp SaveRosbag() 에서 T[i,j] = row[1 + 4*i + j].

        Args:
            filepath: global_pose.csv 파일 경로

        Returns:
            [(stamp_ns: int, R: np.ndarray 3x3, T: np.ndarray 3), ...]
            T는 절대 좌표 (UTM 등 큰 값일 수 있음)
        """
        result = []
        if not os.path.exists(filepath):
            return result

        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = line.split(',')
                if len(parts) < 13:
                    continue
                try:
                    stamp_ns = int(parts[0])
                    vals = [float(p) for p in parts[1:13]]
                    R = np.array([
                        vals[0:3],   # r00, r01, r02
                        vals[4:7],   # r10, r11, r12
                        vals[8:11],  # r20, r21, r22
                    ], dtype=np.float64)
                    T = np.array([vals[3], vals[7], vals[11]], dtype=np.float64)
                    result.append((stamp_ns, R, T))
                except (ValueError, IndexError):
                    continue

        return result

    def _load_gps_csv(self, filepath: str) -> list:
        """gps.csv를 파싱하여 NavSatFix용 dict 리스트를 반환한다.

        포맷: stamp, lat, lon, alt, cov[0..8] (13열)
        참고: ROSThread.cpp의 fscanf 포맷과 동일.

        Returns:
            [{'stamp': int, 'lat': float, 'lon': float, 'alt': float, 'cov': list[9]}, ...]
        """
        if not os.path.exists(filepath):
            return []

        result = []
        try:
            data = np.loadtxt(filepath, delimiter=',')
            if data.ndim == 1:
                data = data.reshape(1, -1)
            for row in data:
                if len(row) < 13:
                    continue
                result.append({
                    'stamp': int(row[0]),
                    'lat': float(row[1]),
                    'lon': float(row[2]),
                    'alt': float(row[3]),
                    'cov': [float(row[i]) for i in range(4, 13)],
                })
        except Exception:
            pass

        return result

    def _load_imu_csv(self, filepath: str) -> tuple:
        """xsens_imu.csv를 파싱하여 (rows: list, version: int)를 반환한다.

        포맷 v1 (8열):  stamp, qx, qy, qz, qw, euler_x, euler_y, euler_z
        포맷 v2 (17열): stamp, qx, qy, qz, qw, euler_x, euler_y, euler_z,
                        gx, gy, gz, ax, ay, az, mx, my, mz
        참고: ROSThread.cpp의 fscanf 분기 (length == 8 or 17).

        Returns:
            (rows: list of dict, version: int)  version = 1 or 2
        """
        if not os.path.exists(filepath):
            return [], 0

        result = []
        version = 0

        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = line.split(',')
                n = len(parts)
                if n < 8:
                    continue
                try:
                    stamp_ns = int(parts[0])
                    qx, qy, qz, qw = float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])
                    row: dict = {'stamp': stamp_ns, 'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw}

                    if n >= 17:
                        # full: + euler + gyro + accel + mag
                        row['gx'] = float(parts[8])
                        row['gy'] = float(parts[9])
                        row['gz'] = float(parts[10])
                        row['ax'] = float(parts[11])
                        row['ay'] = float(parts[12])
                        row['az'] = float(parts[13])
                        row['mx'] = float(parts[14])
                        row['my'] = float(parts[15])
                        row['mz'] = float(parts[16])
                        version = 2
                    else:
                        version = max(version, 1)

                    result.append(row)
                except (ValueError, IndexError):
                    continue

        return result, version

    # ──────────────────────────────────────────────────────────────
    # 메시지 생성 헬퍼
    # ──────────────────────────────────────────────────────────────

    def _make_ouster_pc2(self, bin_path: str, stamp: Time) -> PointCloud2 | None:
        """Ouster .bin 파일에서 sensor_msgs/PointCloud2 메시지를 생성한다.

        .bin 포맷: float32×4 per point (x, y, z, intensity).
        ring = (k % 64) + 1 (참고: ROSThread.cpp OusterThread, Ouster OS1-64 기준).

        필드 레이아웃 (point_step = 20):
          x         offset= 0, FLOAT32
          y         offset= 4, FLOAT32
          z         offset= 8, FLOAT32
          intensity offset=12, FLOAT32
          ring      offset=16, UINT32
        """
        if PointCloud2 is None or PointField is None:
            return None
        if not os.path.exists(bin_path):
            return None

        try:
            raw = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
        except Exception:
            return None

        n_points = raw.shape[0]
        ring_arr = (np.arange(n_points, dtype=np.uint32) % 64) + 1

        # 각 포인트: x(4) y(4) z(4) intensity(4) ring(4) = 20 bytes
        point_step = 20
        buf = np.empty((n_points, point_step), dtype=np.uint8)
        buf[:, 0:16] = raw.view(np.uint8).reshape(n_points, 16)
        buf[:, 16:20] = ring_arr.view(np.uint8).reshape(n_points, 4)

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = MULRAN_FRAME_OUSTER
        msg.height = 1
        msg.width = n_points
        msg.is_bigendian = False
        msg.is_dense = False
        msg.point_step = point_step
        msg.row_step = point_step * n_points

        fields = []
        _defs = [
            ('x',         0,  PointField.FLOAT32),
            ('y',         4,  PointField.FLOAT32),
            ('z',         8,  PointField.FLOAT32),
            ('intensity', 12, PointField.FLOAT32),
            ('ring',      16, PointField.UINT32),
        ]
        for fname, offset, dtype in _defs:
            f = PointField()
            f.name = fname
            f.offset = offset
            f.datatype = dtype
            f.count = 1
            fields.append(f)
        msg.fields = fields
        msg.data = buf.tobytes()
        return msg

    def _make_radar_image(self, png_path: str, stamp: Time) -> Image | None:
        """radar/polar PNG 파일에서 sensor_msgs/Image (MONO8) 메시지를 생성한다.

        참고: ROSThread.cpp RadarpolarThread — encoding = MONO8, frame_id = radar_polar (/tf_static 과 정합).
        """
        if Image is None:
            return None
        if not os.path.exists(png_path):
            return None

        try:
            if _CV2_AVAILABLE and cv2 is not None:
                img = cv2.imread(png_path, cv2.IMREAD_GRAYSCALE)
                if img is None:
                    return None
            else:
                # fallback: PNG 헤더에서 직접 읽기 (Pillow 없이)
                import zlib
                import struct as _struct
                with open(png_path, 'rb') as fh:
                    data = fh.read()
                if data[:8] != b'\x89PNG\r\n\x1a\n':
                    return None
                pos = 8
                idat_chunks = []
                width = height = 0
                bit_depth = color_type = 0
                while pos < len(data):
                    length = _struct.unpack('>I', data[pos:pos + 4])[0]
                    chunk_type = data[pos + 4:pos + 8]
                    chunk_data = data[pos + 8:pos + 8 + length]
                    if chunk_type == b'IHDR':
                        width, height = _struct.unpack('>II', chunk_data[:8])
                        bit_depth, color_type = chunk_data[8], chunk_data[9]
                    elif chunk_type == b'IDAT':
                        idat_chunks.append(chunk_data)
                    elif chunk_type == b'IEND':
                        break
                    pos += 12 + length

                if not idat_chunks or color_type not in (0, 3):
                    return None

                raw_data = zlib.decompress(b''.join(idat_chunks))
                stride = width + 1  # filter byte per row
                img = np.empty((height, width), dtype=np.uint8)
                for row in range(height):
                    src = raw_data[row * stride + 1:(row + 1) * stride]
                    img[row, :len(src)] = np.frombuffer(src, dtype=np.uint8)
        except Exception:
            return None

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = MULRAN_FRAME_RADAR_POLAR
        msg.height = int(img.shape[0])
        msg.width = int(img.shape[1])
        msg.encoding = 'mono8'
        msg.is_bigendian = False
        msg.step = int(img.shape[1])
        msg.data = img.tobytes()
        return msg

    def _make_imu_msg(self, row: dict, stamp: Time, version: int) -> Imu:
        """xsens_imu.csv 한 행으로 sensor_msgs/Imu 메시지를 생성한다.

        v1(8열): orientation만 포함.
        v2(17열): orientation + angular_velocity + linear_acceleration.
        참고: ROSThread.cpp ImuThread.
        """
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = 'imu'

        msg.orientation.x = float(row.get('qx', 0.0))
        msg.orientation.y = float(row.get('qy', 0.0))
        msg.orientation.z = float(row.get('qz', 0.0))
        msg.orientation.w = float(row.get('qw', 1.0))

        if version >= 2:
            msg.angular_velocity.x = float(row.get('gx', 0.0))
            msg.angular_velocity.y = float(row.get('gy', 0.0))
            msg.angular_velocity.z = float(row.get('gz', 0.0))
            msg.linear_acceleration.x = float(row.get('ax', 0.0))
            msg.linear_acceleration.y = float(row.get('ay', 0.0))
            msg.linear_acceleration.z = float(row.get('az', 0.0))
            # covariance (대각선 = 3.0, 참고 ROSThread.cpp)
            for i in (0, 4, 8):
                msg.orientation_covariance[i] = 3.0
                msg.angular_velocity_covariance[i] = 3.0
                msg.linear_acceleration_covariance[i] = 3.0

        return msg

    def _make_navsatfix_msg(self, row: dict, stamp: Time) -> NavSatFix:
        """gps.csv 한 행으로 sensor_msgs/NavSatFix 메시지를 생성한다."""
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = 'gps'
        msg.latitude = float(row.get('lat', 0.0))
        msg.longitude = float(row.get('lon', 0.0))
        msg.altitude = float(row.get('alt', 0.0))

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        cov = row.get('cov', [0.0] * 9)
        for i in range(min(9, len(cov))):
            msg.position_covariance[i] = float(cov[i])
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        return msg

    def _make_gt_odometry(self, R: np.ndarray, T: np.ndarray, stamp: Time) -> Odometry | None:
        """global_pose R,T로 nav_msgs/Odometry (/gt) 메시지를 생성한다.

        참고: ROSThread.cpp SaveRosbag() — frame_id = world, topic = /gt.
        """
        if Odometry is None:
            return None

        qx, qy, qz, qw = self._rotation_matrix_to_quaternion(R)
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'world'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = float(T[0])
        msg.pose.pose.position.y = float(T[1])
        msg.pose.pose.position.z = float(T[2])
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        return msg

    def _make_dynamic_tf(self, R: np.ndarray, T: np.ndarray, stamp: Time) -> TFMessage | None:
        """global_pose R,T로 world → base_link dynamic TF를 생성한다."""
        if TFMessage is None or TransformStamped is None:
            return None

        qx, qy, qz, qw = self._rotation_matrix_to_quaternion(R)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(T[0])
        t.transform.translation.y = float(T[1])
        t.transform.translation.z = float(T[2])
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        tf_msg = TFMessage()
        tf_msg.transforms = [t]
        return tf_msg

    # ──────────────────────────────────────────────────────────────
    # 수학 / 타임스탬프 헬퍼
    # ──────────────────────────────────────────────────────────────

    def _rotation_matrix_to_quaternion(self, R: np.ndarray) -> tuple:
        """3×3 회전 행렬을 quaternion (x, y, z, w)으로 변환한다 (Shepperd's method)."""
        tr = R[0, 0] + R[1, 1] + R[2, 2]
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2.0
            w = 0.25 * S
            x = (R[2, 1] - R[1, 2]) / S
            y = (R[0, 2] - R[2, 0]) / S
            z = (R[1, 0] - R[0, 1]) / S
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w = (R[2, 1] - R[1, 2]) / S
            x = 0.25 * S
            y = (R[0, 1] + R[1, 0]) / S
            z = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            w = (R[0, 2] - R[2, 0]) / S
            x = (R[0, 1] + R[1, 0]) / S
            y = 0.25 * S
            z = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            w = (R[1, 0] - R[0, 1]) / S
            x = (R[0, 2] + R[2, 0]) / S
            y = (R[1, 2] + R[2, 1]) / S
            z = 0.25 * S
        return float(x), float(y), float(z), float(w)

    def _ns_to_time_msg(self, ns: int) -> Time:
        """나노초 정수를 builtin_interfaces/Time 메시지로 변환한다."""
        msg = Time()
        msg.sec = int(ns // 1_000_000_000)
        msg.nanosec = int(ns % 1_000_000_000)
        return msg

    # ──────────────────────────────────────────────────────────────
    # bisect 헬퍼 (nearest stamp 검색)
    # ──────────────────────────────────────────────────────────────

    def _to_bisect(self, rows: list) -> tuple:
        """rows를 stamp 기준 정렬 후 bisect 검색용 (stamps_list, rows_list) 튜플로 반환한다."""
        if not rows:
            return ([], [])
        s = sorted(rows, key=lambda r: r.get('stamp', 0))
        return ([r['stamp'] for r in s], s)

    def _find_nearest(self, bisect_data: tuple, stamp_ns: int) -> dict | None:
        """bisect_data (stamps_list, rows_list)에서 stamp_ns에 가장 가까운 행을 반환한다."""
        stamps_list, rows_list = bisect_data
        if not stamps_list:
            return None
        idx = bisect.bisect_left(stamps_list, stamp_ns)
        if idx == 0:
            return rows_list[0]
        if idx >= len(rows_list):
            return rows_list[-1]
        if abs(stamps_list[idx] - stamp_ns) < abs(stamps_list[idx - 1] - stamp_ns):
            return rows_list[idx]
        return rows_list[idx - 1]

    def _find_nearest_pose(
        self,
        pose_stamps: list,
        poses_sorted: list,
        stamp_ns: int,
    ) -> tuple | None:
        """pose_stamps에서 stamp_ns에 가장 가까운 (stamp, R, T)를 반환한다."""
        if not poses_sorted:
            return None
        idx = bisect.bisect_left(pose_stamps, stamp_ns)
        if idx == 0:
            return poses_sorted[0]
        if idx >= len(poses_sorted):
            return poses_sorted[-1]
        if abs(pose_stamps[idx] - stamp_ns) < abs(pose_stamps[idx - 1] - stamp_ns):
            return poses_sorted[idx]
        return poses_sorted[idx - 1]
