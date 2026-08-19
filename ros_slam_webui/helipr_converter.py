#!/usr/bin/env python3
"""
helipr_converter.py
HeLiPR Dataset → ROS2 bag (.db3) / ROS1 bag (.bag) 변환 모듈

참고 구현체: /home/kkw/Downloads/HeLiPR-File-Player-master/src/ROSThread.{h,cpp} (ROS1 file_player)

출력 토픽 (Direct Play / Bag 변환):
  /ouster/points       sensor_msgs/msg/PointCloud2       (frame_id=ouster)
  /velodyne/points     sensor_msgs/msg/PointCloud2       (frame_id=velodyne)
  /avia/points         livox_ros_driver2/msg/CustomMsg   (frame_id=livox_avia)
  /aeva/points         sensor_msgs/msg/PointCloud2       (frame_id=aeva)
  /imu/data_raw        sensor_msgs/msg/Imu               (frame_id=imu)
  /imu/mag             sensor_msgs/msg/MagneticField     (frame_id=imu, IMU 포맷 v2 전용)
  /gps/fix             sensor_msgs/msg/NavSatFix         (inspva.csv 기반, frame_id=inspva)
  /gt                  nav_msgs/msg/Odometry             (LiDAR_GT/*.txt 기반, frame_id=world)
  /tf                  tf2_msgs/msg/TFMessage            (dynamic world→base_link, GT 기반)
  /clock               rosgraph_msgs/msg/Clock           (10ms 이상 간격 시만 publish)

디렉토리 구조 (참고: ROSThread.cpp Ready(), HeLiPR 공식 배포 구조):
  {seq}/stamp.csv                    {stamp_ns},{sensor_name}
                                      sensor_name ∈ {inspva, imu, ouster, velodyne, livox_avia, aeva}
  {seq}/Inertial_data/inspva.csv     {stamp_ns},lat,lon,height,vN,vE,vU,roll,pitch,azimuth,"status: N"
  {seq}/Inertial_data/xsens_imu.csv  {stamp_ns},qx,qy,qz,qw,ex,ey,ez,gx,gy,gz,ax,ay,az,mx,my,mz (8열 또는 17열)
  {seq}/LiDAR/Ouster/{stamp_ns}.bin    x,y,z,intensity(f32) t(u32) reflectivity,ring,ambient(u16) = 26B/point
  {seq}/LiDAR/Velodyne/{stamp_ns}.bin  x,y,z,intensity(f32) ring(u16) time(f32) = 22B/point
  {seq}/LiDAR/Avia/{stamp_ns}.bin      x,y,z(f32) reflectivity,tag,line(u8) offset_time(u32) = 19B/point (Livox CustomPoint)
  {seq}/LiDAR/Aeva/{stamp_ns}.bin      x,y,z,reflectivity,velocity(f32) time_offset_ns(i32) line_index(u8)
                                        [+intensity(f32) if stamp_ns > 1691936557946849179] = 25B or 29B/point
  {seq}/LiDAR_GT/{Ouster,Velodyne,Aeva,Avia}_gt.txt   {stamp_ns} x y z qx qy qz qw

GT 우선순위: Ouster > Velodyne > Aeva > Avia (제공 여부에 따라 자동 선택, world→base_link TF/Odometry 생성용).
캘리브레이션(Calibration/*)은 사용하지 않는다 — 레퍼런스 file player도 센서별 프레임을 그대로 publish하며
센서 간 정적 외장을 요구하지 않는다.
"""

import bisect
import glob
import os
import re
import shutil
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
    from sensor_msgs.msg import Imu, MagneticField, NavSatFix, NavSatStatus, PointCloud2, PointField
    from tf2_msgs.msg import TFMessage
    _ROSBAG2_AVAILABLE = True
except ImportError:
    rosbag2_py = None
    TopicMetadata = None
    serialize_message = None
    TransformStamped = None
    Odometry = None
    Imu = None
    MagneticField = None
    NavSatFix = None
    NavSatStatus = None
    PointCloud2 = None
    PointField = None
    TFMessage = None
    _ROSBAG2_AVAILABLE = False

# Livox CustomMsg (Avia LiDAR) — ConPR과 동일 의존성
try:
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
    _LIVOX_AVAILABLE = True
except ImportError:
    CustomMsg = None
    CustomPoint = None
    _LIVOX_AVAILABLE = False

# /clock publish 최소 간격 (참고: 다른 데이터셋과 동일한 10ms 정책)
_CLOCK_MIN_INTERVAL_NS = 10_000_000  # 10 ms

# 프레임 이름 (레퍼런스 ROSThread.cpp의 header.frame_id 그대로 사용)
HELIPR_FRAME_OUSTER = 'ouster'
HELIPR_FRAME_VELODYNE = 'velodyne'
HELIPR_FRAME_AVIA = 'livox_avia'
HELIPR_FRAME_AEVA = 'aeva'
HELIPR_FRAME_INSPVA = 'inspva'
HELIPR_FRAME_IMU = 'imu'

# Aeva intensity 필드는 이 임계 timestamp(ns) 초과 시퀀스에만 존재 (참고: ROSThread.cpp AevaThread)
_AEVA_INTENSITY_THRESHOLD_NS = 1691936557946849179

# ── 바이너리 포인트 레이아웃 (numpy structured dtype, little-endian, padding 없음) ──
_OUSTER_DTYPE = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'),
    ('t', '<u4'), ('reflectivity', '<u2'), ('ring', '<u2'), ('ambient', '<u2'),
])  # 26 bytes/point

_VELODYNE_DTYPE = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'),
    ('ring', '<u2'), ('time', '<f4'),
])  # 22 bytes/point

_AVIA_DTYPE = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
    ('reflectivity', 'u1'), ('tag', 'u1'), ('line', 'u1'), ('offset_time', '<u4'),
])  # 19 bytes/point (Livox CustomPoint)

_AEVA_DTYPE_BASE = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
    ('reflectivity', '<f4'), ('velocity', '<f4'),
    ('time_offset_ns', '<i4'), ('line_index', 'u1'),
])  # 25 bytes/point

_AEVA_DTYPE_FULL = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
    ('reflectivity', '<f4'), ('velocity', '<f4'),
    ('time_offset_ns', '<i4'), ('line_index', 'u1'), ('intensity', '<f4'),
])  # 29 bytes/point

# GT 파일 탐색 우선순위 (가장 흔히 제공되는 순서)
_GT_LIDAR_PRIORITY = ['Ouster', 'Velodyne', 'Aeva', 'Avia']

# stamp.csv 센서 이름 → LiDAR 디렉토리 키 매핑
_LIDAR_SENSOR_NAMES = {'ouster', 'velodyne', 'livox_avia', 'aeva'}


class HeliprConverter:
    """HeLiPR Dataset을 ROS2/ROS1 bag 파일로 변환하는 클래스.

    참고 코드:
        ROSThread.cpp (HeLiPR-File-Player-master) — stamp.csv 기반 멀티스레드 publish,
        Ouster/Velodyne/Avia/Aeva 바이너리 파싱, inspva/xsens_imu 로드.

    Usage:
        converter = HeliprConverter()
        result = converter.scan_directory('/path/to/helipr')
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
        """HeLiPR 데이터셋 디렉토리를 탐색하여 시퀀스 목록을 반환한다.

        사용자가 데이터셋 **루트**만 골라도, 그 아래 ``stamp.csv`` + ``LiDAR/`` 를
        가진 시퀀스를 재귀적으로 모은다.

        Args:
            base_dir: 사용자가 선택한 디렉토리 (시퀀스 한 개 또는 상위 루트)

        Returns:
            {'success': True, 'sequences': [{name, path}, ...]}
        """
        base_dir = os.path.abspath(base_dir)
        sequences: list[dict] = []
        seen_paths: set[str] = set()

        skip_names = {'HeLiPR-File-Player-master', '__pycache__'}

        def _try_add(path: str) -> bool:
            ap = os.path.abspath(path)
            if ap in seen_paths:
                return False
            bn = os.path.basename(ap)
            if bn.startswith('.'):
                return False
            if bn.endswith('_converted'):
                return False
            if not self._is_helipr_sequence(ap):
                return False
            seen_paths.add(ap)
            sequences.append({'name': bn, 'path': ap})
            return True

        def _walk(root: str, depth: int, max_depth: int) -> None:
            ap = os.path.abspath(root)
            if depth > max_depth:
                return
            if _try_add(ap):
                return  # 시퀀스 발견 시 하위는 더 내려가지 않는다 (LiDAR/, LiDAR_GT/ 오탐 방지)
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
        storage_id: str = 'sqlite3',
        progress_cb=None,
    ) -> None:
        """HeLiPR 시퀀스를 ROS2 bag 파일로 변환한다.

        Args:
            sequence_dir: 시퀀스 루트 디렉토리 (stamp.csv 포함)
            output_path: 출력 bag 경로 (확장자 없음)
            sensors: 포함할 센서 목록 (None이면 전체). 예: ['ouster','velodyne','avia','aeva','imu','gps','gt']
            storage_id: rosbag2 storage 플러그인 - 'sqlite3'(기본, db3) 또는 'mcap'
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

        _progress(0, 'Loading HeLiPR data...')
        ctx = self._load_sequence_context(sequence_dir)

        if not ctx['data_stamps']:
            raise RuntimeError(
                'stamp.csv가 비어 있거나 파싱할 수 없습니다. 시퀀스 경로를 확인하세요.'
            )

        if os.path.exists(output_path):
            shutil.rmtree(output_path)
        parent_dir = os.path.dirname(output_path)
        if parent_dir:
            os.makedirs(parent_dir, exist_ok=True)

        _progress(2, 'Initializing ROS2 bag writer...')
        writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(uri=output_path, storage_id=storage_id)
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        )
        writer.open(storage_options, converter_options)

        topics = self._get_topic_list(ctx, sensors)
        for idx, (topic_name, topic_type) in enumerate(topics):
            writer.create_topic(TopicMetadata(
                id=idx,
                name=topic_name,
                type=topic_type,
                serialization_format='cdr',
            ))

        _progress(3, 'Writing HeLiPR messages...')
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
            self._write_sensor_ros2(writer, ctx, sensors, ts_ns, stamp_time, sensor_name)
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
        """HeLiPR 시퀀스를 ROS1 .bag 파일로 변환한다 (rosbags 라이브러리 사용).

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

        _progress(0, 'Loading HeLiPR data...')
        ctx = self._load_sequence_context(sequence_dir)

        if not ctx['data_stamps']:
            raise RuntimeError(
                'stamp.csv가 비어 있거나 파싱할 수 없습니다. 시퀀스 경로를 확인하세요.'
            )

        src_typestore = get_typestore(Stores.ROS2_JAZZY)
        dst_typestore = get_typestore(Stores.ROS1_NOETIC)
        migrate_cache: dict = {}

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

        topics = self._get_topic_list(ctx, sensors, ros1=True)
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

            _progress(3, 'Writing HeLiPR messages...')
            for ts_ns, sensor_name in ctx['data_stamps']:
                stamp_time = self._ns_to_time_msg(ts_ns)
                self._write_sensor_ros1(_write, ctx, sensors, ts_ns, stamp_time, sensor_name)
                _tick(f'Converting {sensor_name}...')

        _progress(100, 'Conversion complete!')

    # ──────────────────────────────────────────────────────────────
    # 디렉토리 감지 헬퍼
    # ──────────────────────────────────────────────────────────────

    def _is_helipr_sequence(self, path: str) -> bool:
        """path가 HeLiPR 시퀀스 루트(stamp.csv + LiDAR 데이터 디렉토리)인지 판별한다.

        참고 코드(ROSThread.cpp)는 ``<seq>/LiDAR/<Type>`` 레이아웃을 가정하지만,
        실제 공개 배포된 HeLiPR 데이터셋은 ``<seq>/<Type>`` 처럼 LiDAR 상위 폴더 없이
        바로 시퀀스 루트에 Ouster/Velodyne/Avia/Aeva 폴더를 두는 경우가 있다.
        두 레이아웃을 모두 지원한다 (_get_lidar_dir 참고).
        """
        if not os.path.isfile(os.path.join(path, 'stamp.csv')):
            return False
        return any(
            self._get_lidar_dir(path, name) is not None
            for name in ('Ouster', 'Velodyne', 'Avia', 'Aeva')
        )

    def _get_lidar_dir(self, sequence_dir: str, name: str) -> str | None:
        """LiDAR 데이터 디렉토리를 대소문자/레이아웃(LiDAR/<name> 또는 <name>) 무관하게 탐색한다."""
        candidates = [
            os.path.join(sequence_dir, 'LiDAR', name),
            os.path.join(sequence_dir, 'LiDAR', name.lower()),
            os.path.join(sequence_dir, 'LiDAR', name.upper()),
            os.path.join(sequence_dir, name),
            os.path.join(sequence_dir, name.lower()),
            os.path.join(sequence_dir, name.upper()),
        ]
        for d in candidates:
            if os.path.isdir(d):
                return d
        return None

    def _find_inertial_file(self, sequence_dir: str, filename: str) -> str | None:
        candidates = [
            os.path.join(sequence_dir, 'Inertial_data', filename),
            os.path.join(sequence_dir, 'inertial_data', filename),
            os.path.join(sequence_dir, filename),
        ]
        for c in candidates:
            if os.path.isfile(c):
                return c
        return None

    def _find_lidar_gt(self, sequence_dir: str) -> tuple:
        """LiDAR_GT/*.txt 를 우선순위(Ouster>Velodyne>Aeva>Avia)로 탐색한다.

        Returns:
            (rows: list of dict, source_name: str | None)
        """
        gt_dir = None
        for cand in (os.path.join(sequence_dir, 'LiDAR_GT'), os.path.join(sequence_dir, 'lidar_gt')):
            if os.path.isdir(cand):
                gt_dir = cand
                break
        if gt_dir is None:
            return [], None

        for name in _GT_LIDAR_PRIORITY:
            for fname in (f'{name}_gt.txt', f'{name.lower()}_gt.txt', f'{name.upper()}_gt.txt'):
                fp = os.path.join(gt_dir, fname)
                if os.path.isfile(fp):
                    rows = self._parse_lidar_gt(fp)
                    if rows:
                        return rows, name

        try:
            candidates = sorted(glob.glob(os.path.join(gt_dir, '*_gt.txt')) +
                                 glob.glob(os.path.join(gt_dir, '*_GT.txt')))
        except Exception:
            candidates = []
        for fp in candidates:
            rows = self._parse_lidar_gt(fp)
            if rows:
                return rows, os.path.splitext(os.path.basename(fp))[0]
        return [], None

    # ──────────────────────────────────────────────────────────────
    # 시퀀스 컨텍스트 로딩 (공통)
    # ──────────────────────────────────────────────────────────────

    def _load_sequence_context(self, sequence_dir: str) -> dict:
        """변환/직접재생에 필요한 모든 데이터를 메모리에 로드하여 컨텍스트 dict로 반환한다.

        Returns:
            {
              'sequence_dir': str,
              'data_stamps': list of (stamp_ns: int, sensor_name: str),
              'ouster_dir', 'velodyne_dir', 'avia_dir', 'aeva_dir': str | None,
              'imu_bisect': (stamps_list, rows_list),
              'imu_version': int,  # 1 (orientation only) or 2 (full incl. mag)
              'inspva_bisect': (stamps_list, rows_list),
              'gt_bisect': (stamps_list, rows_list),
              'gt_source': str | None,  # 어떤 LiDAR 기준 GT를 사용 중인지
            }
        """
        sequence_dir = os.path.abspath(sequence_dir)
        if not self._is_helipr_sequence(sequence_dir):
            raise RuntimeError(
                f'HeLiPR 시퀀스를 인식할 수 없습니다. stamp.csv / LiDAR 디렉토리를 확인하세요: {sequence_dir}'
            )

        stamp_csv = os.path.join(sequence_dir, 'stamp.csv')
        data_stamps = self._parse_data_stamp(stamp_csv)

        ouster_dir = self._get_lidar_dir(sequence_dir, 'Ouster')
        velodyne_dir = self._get_lidar_dir(sequence_dir, 'Velodyne')
        avia_dir = self._get_lidar_dir(sequence_dir, 'Avia')
        aeva_dir = self._get_lidar_dir(sequence_dir, 'Aeva')

        inspva_path = self._find_inertial_file(sequence_dir, 'inspva.csv')
        imu_path = self._find_inertial_file(sequence_dir, 'xsens_imu.csv')

        inspva_rows = self._load_inspva_csv(inspva_path) if inspva_path else []
        imu_rows, imu_version = self._load_imu_csv(imu_path) if imu_path else ([], 0)

        gt_rows, gt_source = self._find_lidar_gt(sequence_dir)

        return {
            'sequence_dir': sequence_dir,
            'data_stamps': data_stamps,
            'ouster_dir': ouster_dir,
            'velodyne_dir': velodyne_dir,
            'avia_dir': avia_dir,
            'aeva_dir': aeva_dir,
            'inspva_bisect': self._to_bisect(inspva_rows),
            'imu_bisect': self._to_bisect(imu_rows),
            'imu_version': imu_version,
            'gt_bisect': self._to_bisect(gt_rows),
            'gt_source': gt_source,
        }

    def _get_topic_list(self, ctx: dict, sensors: list | None, ros1: bool = False) -> list:
        """변환할 토픽 목록을 반환한다. Returns: list of (topic_name, ros2_type).

        ros1=True: avia는 CustomMsg 대신 PointCloud2로 기록하므로(타입 미등록 문제 회피),
        _LIVOX_AVAILABLE 여부와 무관하게 avia_dir만 있으면 토픽을 추가한다.
        """
        topics = []

        def _want(sensor: str) -> bool:
            return sensors is None or sensor in sensors

        if ctx['ouster_dir'] and _want('ouster'):
            topics.append(('/ouster/points', 'sensor_msgs/msg/PointCloud2'))
        if ctx['velodyne_dir'] and _want('velodyne'):
            topics.append(('/velodyne/points', 'sensor_msgs/msg/PointCloud2'))
        if ctx['avia_dir'] and _want('avia'):
            if ros1:
                topics.append(('/avia/points', 'sensor_msgs/msg/PointCloud2'))
            elif _LIVOX_AVAILABLE:
                topics.append(('/avia/points', 'livox_ros_driver2/msg/CustomMsg'))
        if ctx['aeva_dir'] and _want('aeva'):
            topics.append(('/aeva/points', 'sensor_msgs/msg/PointCloud2'))
        if ctx['imu_bisect'][0] and _want('imu'):
            topics.append(('/imu/data_raw', 'sensor_msgs/msg/Imu'))
            if ctx['imu_version'] >= 2:
                topics.append(('/imu/mag', 'sensor_msgs/msg/MagneticField'))
        if ctx['inspva_bisect'][0] and _want('gps'):
            topics.append(('/gps/fix', 'sensor_msgs/msg/NavSatFix'))
        if ctx['gt_bisect'][0] and _want('gt'):
            topics.append(('/gt', 'nav_msgs/msg/Odometry'))
            topics.append(('/tf', 'tf2_msgs/msg/TFMessage'))

        return topics

    # ──────────────────────────────────────────────────────────────
    # 센서별 쓰기 (ROS2 bag)
    # ──────────────────────────────────────────────────────────────

    def _write_sensor_ros2(self, writer, ctx, sensors, ts_ns, stamp_time, sensor_name) -> None:
        def _want(s: str) -> bool:
            return sensors is None or s in sensors

        sn = sensor_name.lower()

        if sn == 'ouster' and ctx['ouster_dir'] and _want('ouster'):
            msg = self._make_ouster_pc2(os.path.join(ctx['ouster_dir'], f'{ts_ns}.bin'), stamp_time)
            if msg:
                writer.write('/ouster/points', serialize_message(msg), ts_ns)

        elif sn == 'velodyne' and ctx['velodyne_dir'] and _want('velodyne'):
            msg = self._make_velodyne_pc2(os.path.join(ctx['velodyne_dir'], f'{ts_ns}.bin'), stamp_time)
            if msg:
                writer.write('/velodyne/points', serialize_message(msg), ts_ns)

        elif sn == 'livox_avia' and ctx['avia_dir'] and _want('avia') and _LIVOX_AVAILABLE:
            msg = self._make_avia_custom_msg(os.path.join(ctx['avia_dir'], f'{ts_ns}.bin'), ts_ns, stamp_time)
            if msg:
                writer.write('/avia/points', serialize_message(msg), ts_ns)

        elif sn == 'aeva' and ctx['aeva_dir'] and _want('aeva'):
            msg = self._make_aeva_pc2(os.path.join(ctx['aeva_dir'], f'{ts_ns}.bin'), ts_ns, stamp_time)
            if msg:
                writer.write('/aeva/points', serialize_message(msg), ts_ns)

        elif sn == 'imu' and ctx['imu_bisect'][0] and _want('imu'):
            row = self._find_nearest(ctx['imu_bisect'], ts_ns)
            if row:
                writer.write('/imu/data_raw', serialize_message(
                    self._make_imu_msg(row, stamp_time, ctx['imu_version'])), ts_ns)
                if ctx['imu_version'] >= 2:
                    mag_msg = self._make_mag_msg(row, stamp_time)
                    if mag_msg:
                        writer.write('/imu/mag', serialize_message(mag_msg), ts_ns)

        elif sn == 'inspva' and ctx['inspva_bisect'][0] and _want('gps'):
            row = self._find_nearest(ctx['inspva_bisect'], ts_ns)
            if row:
                writer.write('/gps/fix', serialize_message(
                    self._make_navsatfix_msg(row, stamp_time)), ts_ns)

        if ctx['gt_bisect'][0] and _want('gt'):
            row = self._find_nearest(ctx['gt_bisect'], ts_ns)
            if row:
                odom_msg = self._make_gt_odometry(row, stamp_time)
                tf_msg = self._make_dynamic_tf(row, stamp_time)
                if odom_msg:
                    writer.write('/gt', serialize_message(odom_msg), ts_ns)
                if tf_msg:
                    writer.write('/tf', serialize_message(tf_msg), ts_ns)

    # ──────────────────────────────────────────────────────────────
    # 센서별 쓰기 (ROS1 bag)
    # ──────────────────────────────────────────────────────────────

    def _write_sensor_ros1(self, _write, ctx, sensors, ts_ns, stamp_time, sensor_name) -> None:
        def _want(s: str) -> bool:
            return sensors is None or s in sensors

        sn = sensor_name.lower()

        if sn == 'ouster' and ctx['ouster_dir'] and _want('ouster'):
            msg = self._make_ouster_pc2(os.path.join(ctx['ouster_dir'], f'{ts_ns}.bin'), stamp_time)
            if msg:
                _write('/ouster/points', msg, ts_ns)

        elif sn == 'velodyne' and ctx['velodyne_dir'] and _want('velodyne'):
            msg = self._make_velodyne_pc2(os.path.join(ctx['velodyne_dir'], f'{ts_ns}.bin'), stamp_time)
            if msg:
                _write('/velodyne/points', msg, ts_ns)

        elif sn == 'livox_avia' and ctx['avia_dir'] and _want('avia'):
            msg = self._make_avia_pc2(os.path.join(ctx['avia_dir'], f'{ts_ns}.bin'), stamp_time)
            if msg:
                _write('/avia/points', msg, ts_ns)

        elif sn == 'aeva' and ctx['aeva_dir'] and _want('aeva'):
            msg = self._make_aeva_pc2(os.path.join(ctx['aeva_dir'], f'{ts_ns}.bin'), ts_ns, stamp_time)
            if msg:
                _write('/aeva/points', msg, ts_ns)

        elif sn == 'imu' and ctx['imu_bisect'][0] and _want('imu'):
            row = self._find_nearest(ctx['imu_bisect'], ts_ns)
            if row:
                _write('/imu/data_raw', self._make_imu_msg(row, stamp_time, ctx['imu_version']), ts_ns)
                if ctx['imu_version'] >= 2:
                    mag_msg = self._make_mag_msg(row, stamp_time)
                    if mag_msg:
                        _write('/imu/mag', mag_msg, ts_ns)

        elif sn == 'inspva' and ctx['inspva_bisect'][0] and _want('gps'):
            row = self._find_nearest(ctx['inspva_bisect'], ts_ns)
            if row:
                _write('/gps/fix', self._make_navsatfix_msg(row, stamp_time), ts_ns)

        if ctx['gt_bisect'][0] and _want('gt'):
            row = self._find_nearest(ctx['gt_bisect'], ts_ns)
            if row:
                odom_msg = self._make_gt_odometry(row, stamp_time)
                tf_msg = self._make_dynamic_tf(row, stamp_time)
                if odom_msg:
                    _write('/gt', odom_msg, ts_ns)
                if tf_msg:
                    _write('/tf', tf_msg, ts_ns)

    # ──────────────────────────────────────────────────────────────
    # CSV 파싱 헬퍼
    # ──────────────────────────────────────────────────────────────

    def _parse_data_stamp(self, filepath: str) -> list:
        """stamp.csv를 파싱하여 [(stamp_ns, sensor_name), ...] 리스트를 반환한다.

        포맷: {nanosec_stamp},{sensor_name}
        sensor_name ∈ {inspva, imu, ouster, velodyne, livox_avia, aeva} (참고: ROSThread.cpp Ready()).
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

    def _load_inspva_csv(self, filepath: str) -> list:
        """inspva.csv를 파싱하여 NavSatFix용 dict 리스트를 반환한다.

        포맷: stamp, lat, lon, height, vN, vE, vU, roll, pitch, azimuth, "status: N" (11열)
        참고: ROSThread.cpp Ready()의 fscanf 포맷.
        """
        if not filepath or not os.path.exists(filepath):
            return []

        result = []
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = line.split(',')
                if len(parts) < 11:
                    continue
                try:
                    stamp_ns = int(parts[0])
                    lat, lon, height = float(parts[1]), float(parts[2]), float(parts[3])
                    vn, ve, vu = float(parts[4]), float(parts[5]), float(parts[6])
                    roll, pitch, azimuth = float(parts[7]), float(parts[8]), float(parts[9])
                    m = re.search(r'-?\d+', parts[10])
                    status = int(m.group()) if m else 0
                    result.append({
                        'stamp': stamp_ns, 'lat': lat, 'lon': lon, 'height': height,
                        'vn': vn, 've': ve, 'vu': vu,
                        'roll': roll, 'pitch': pitch, 'azimuth': azimuth, 'status': status,
                    })
                except (ValueError, IndexError):
                    continue
        return result

    def _load_imu_csv(self, filepath: str) -> tuple:
        """xsens_imu.csv를 파싱하여 (rows: list, version: int)를 반환한다.

        포맷 v1 (8열):  stamp, qx, qy, qz, qw, euler_x, euler_y, euler_z
        포맷 v2 (17열): stamp, qx, qy, qz, qw, euler_x, euler_y, euler_z,
                        gx, gy, gz, ax, ay, az, mx, my, mz
        참고: ROSThread.cpp Ready() fscanf 분기 (length == 8 or 17).
        """
        if not filepath or not os.path.exists(filepath):
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

    def _parse_lidar_gt(self, filepath: str) -> list:
        """LiDAR_GT/*.txt 를 파싱하여 [{'stamp','x','y','z','qx','qy','qz','qw'}, ...] 를 반환한다.

        포맷: stamp x y z qx qy qz qw (공백 또는 콤마 구분, 참고: HeLiPR 공식 배포 문서).
        """
        result = []
        if not os.path.exists(filepath):
            return result

        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = line.replace(',', ' ').split()
                if len(parts) < 8:
                    continue
                try:
                    stamp_raw = parts[0]
                    stamp_ns = int(stamp_raw) if '.' not in stamp_raw else int(round(float(stamp_raw)))
                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                    qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
                    result.append({
                        'stamp': stamp_ns, 'x': x, 'y': y, 'z': z,
                        'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
                    })
                except (ValueError, IndexError):
                    continue

        result.sort(key=lambda r: r['stamp'])
        return result

    # ──────────────────────────────────────────────────────────────
    # 바이너리 포인트클라우드 파싱 헬퍼
    # ──────────────────────────────────────────────────────────────

    def _read_records(self, bin_path: str, dtype: np.dtype):
        """bin_path를 dtype 레코드 배열로 읽는다 (파일 없음/빈 파일 시 None)."""
        if not bin_path or not os.path.exists(bin_path):
            return None
        try:
            with open(bin_path, 'rb') as f:
                raw = f.read()
        except Exception:
            return None
        n = len(raw) // dtype.itemsize
        if n == 0:
            return None
        return np.frombuffer(raw[:n * dtype.itemsize], dtype=dtype)

    def _make_pc2_from_array(self, arr, frame_id: str, stamp: Time, field_types: dict):
        """구조화 numpy 배열을 sensor_msgs/PointCloud2 로 변환한다 (zero-copy data)."""
        if PointCloud2 is None or PointField is None or arr is None:
            return None
        n_points = int(arr.shape[0])
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = n_points
        msg.is_bigendian = False
        msg.is_dense = False
        msg.point_step = int(arr.dtype.itemsize)
        msg.row_step = msg.point_step * n_points

        fields = []
        for name in arr.dtype.names:
            pf_type = field_types.get(name)
            if pf_type is None:
                continue
            f = PointField()
            f.name = name
            f.offset = int(arr.dtype.fields[name][1])
            f.datatype = pf_type
            f.count = 1
            fields.append(f)
        msg.fields = fields
        msg.data = arr.tobytes()
        return msg

    def _make_ouster_pc2(self, bin_path: str, stamp: Time):
        """Ouster .bin → PointCloud2 (x,y,z,intensity,t,reflectivity,ring,ambient)."""
        arr = self._read_records(bin_path, _OUSTER_DTYPE)
        if arr is None or PointField is None:
            return None
        field_types = {
            'x': PointField.FLOAT32, 'y': PointField.FLOAT32, 'z': PointField.FLOAT32,
            'intensity': PointField.FLOAT32, 't': PointField.UINT32,
            'reflectivity': PointField.UINT16, 'ring': PointField.UINT16, 'ambient': PointField.UINT16,
        }
        return self._make_pc2_from_array(arr, HELIPR_FRAME_OUSTER, stamp, field_types)

    def _make_velodyne_pc2(self, bin_path: str, stamp: Time):
        """Velodyne .bin → PointCloud2 (x,y,z,intensity,ring,time)."""
        arr = self._read_records(bin_path, _VELODYNE_DTYPE)
        if arr is None or PointField is None:
            return None
        field_types = {
            'x': PointField.FLOAT32, 'y': PointField.FLOAT32, 'z': PointField.FLOAT32,
            'intensity': PointField.FLOAT32, 'ring': PointField.UINT16, 'time': PointField.FLOAT32,
        }
        return self._make_pc2_from_array(arr, HELIPR_FRAME_VELODYNE, stamp, field_types)

    def _make_aeva_pc2(self, bin_path: str, stamp_ns: int, stamp: Time):
        """Aeva .bin → PointCloud2 (x,y,z,reflectivity,velocity,time_offset_ns,line_index[,intensity])."""
        dtype = _AEVA_DTYPE_FULL if stamp_ns > _AEVA_INTENSITY_THRESHOLD_NS else _AEVA_DTYPE_BASE
        arr = self._read_records(bin_path, dtype)
        if arr is None or PointField is None:
            return None
        field_types = {
            'x': PointField.FLOAT32, 'y': PointField.FLOAT32, 'z': PointField.FLOAT32,
            'reflectivity': PointField.FLOAT32, 'velocity': PointField.FLOAT32,
            'time_offset_ns': PointField.INT32, 'line_index': PointField.UINT8,
            'intensity': PointField.FLOAT32,
        }
        return self._make_pc2_from_array(arr, HELIPR_FRAME_AEVA, stamp, field_types)

    def _make_avia_pc2(self, bin_path: str, stamp: Time):
        """Avia .bin → sensor_msgs/PointCloud2 (x,y,z,intensity=reflectivity,tag,line).

        ROS1 bag 변환용 대체 표현. rosbags 라이브러리의 ROS2_JAZZY 내장 typestore는
        서드파티 livox_ros_driver2/CustomMsg 타입을 모르므로(TypesysError: unknown type),
        CustomMsg를 그대로 ROS1 bag에 기록할 수 없다. 대신 web_server.py의
        save_rosbag_ros1()/_livox_custommsg_to_pointcloud2()와 동일하게 표준 PointCloud2로
        변환하여 저장한다 (rosbridge/3D Viewer 호환성도 보장됨).
        """
        arr = self._read_records(bin_path, _AVIA_DTYPE)
        if arr is None or PointField is None:
            return None
        out = np.zeros(arr.shape[0], dtype=np.dtype([
            ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
            ('intensity', '<f4'), ('tag', 'u1'), ('line', 'u1'),
        ]))
        out['x'] = arr['x']
        out['y'] = arr['y']
        out['z'] = arr['z']
        out['intensity'] = arr['reflectivity'].astype('<f4')
        out['tag'] = arr['tag']
        out['line'] = arr['line']
        field_types = {
            'x': PointField.FLOAT32, 'y': PointField.FLOAT32, 'z': PointField.FLOAT32,
            'intensity': PointField.FLOAT32, 'tag': PointField.UINT8, 'line': PointField.UINT8,
        }
        return self._make_pc2_from_array(out, HELIPR_FRAME_AVIA, stamp, field_types)

    def _make_avia_custom_msg(self, bin_path: str, stamp_ns: int, stamp: Time):
        """Avia .bin → livox_ros_driver2/CustomMsg (참고: ROSThread.cpp AviaThread, ConPR load_livox_data)."""
        if CustomMsg is None or CustomPoint is None:
            return None
        arr = self._read_records(bin_path, _AVIA_DTYPE)
        if arr is None:
            return None

        msg = CustomMsg()
        msg.header.stamp = stamp
        msg.header.frame_id = HELIPR_FRAME_AVIA
        msg.timebase = int(stamp_ns)
        msg.point_num = int(arr.shape[0])
        msg.lidar_id = 0
        msg.rsvd = [0, 0, 0]

        xs = arr['x'].tolist()
        ys = arr['y'].tolist()
        zs = arr['z'].tolist()
        refl = arr['reflectivity'].tolist()
        tags = arr['tag'].tolist()
        lines = arr['line'].tolist()
        offs = arr['offset_time'].tolist()

        points = []
        for i in range(arr.shape[0]):
            p = CustomPoint()
            p.x = xs[i]
            p.y = ys[i]
            p.z = zs[i]
            p.reflectivity = refl[i]
            p.tag = tags[i]
            p.line = lines[i]
            p.offset_time = offs[i]
            points.append(p)
        msg.points = points
        return msg

    # ──────────────────────────────────────────────────────────────
    # 메시지 생성 헬퍼 (IMU / GPS / GT)
    # ──────────────────────────────────────────────────────────────

    def _make_imu_msg(self, row: dict, stamp: Time, version: int):
        """xsens_imu.csv 한 행으로 sensor_msgs/Imu 메시지를 생성한다."""
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = HELIPR_FRAME_IMU

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
            for i in (0, 4, 8):
                msg.orientation_covariance[i] = 3.0
                msg.angular_velocity_covariance[i] = 3.0
                msg.linear_acceleration_covariance[i] = 3.0

        return msg

    def _make_mag_msg(self, row: dict, stamp: Time):
        """xsens_imu.csv 한 행(v2)으로 sensor_msgs/MagneticField 메시지를 생성한다."""
        if MagneticField is None or 'mx' not in row:
            return None
        msg = MagneticField()
        msg.header.stamp = stamp
        msg.header.frame_id = HELIPR_FRAME_IMU
        msg.magnetic_field.x = float(row.get('mx', 0.0))
        msg.magnetic_field.y = float(row.get('my', 0.0))
        msg.magnetic_field.z = float(row.get('mz', 0.0))
        return msg

    def _make_navsatfix_msg(self, row: dict, stamp: Time):
        """inspva.csv 한 행으로 sensor_msgs/NavSatFix 메시지를 생성한다."""
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = HELIPR_FRAME_INSPVA
        msg.latitude = float(row.get('lat', 0.0))
        msg.longitude = float(row.get('lon', 0.0))
        msg.altitude = float(row.get('height', 0.0))
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        return msg

    def _make_gt_odometry(self, row: dict, stamp: Time):
        """LiDAR_GT 한 행(quaternion 직접 제공)으로 nav_msgs/Odometry (/gt) 메시지를 생성한다."""
        if Odometry is None:
            return None
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'world'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = float(row['x'])
        msg.pose.pose.position.y = float(row['y'])
        msg.pose.pose.position.z = float(row['z'])
        msg.pose.pose.orientation.x = float(row['qx'])
        msg.pose.pose.orientation.y = float(row['qy'])
        msg.pose.pose.orientation.z = float(row['qz'])
        msg.pose.pose.orientation.w = float(row['qw'])
        return msg

    def _make_dynamic_tf(self, row: dict, stamp: Time):
        """LiDAR_GT 한 행으로 world → base_link dynamic TF를 생성한다."""
        if TFMessage is None or TransformStamped is None:
            return None
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(row['x'])
        t.transform.translation.y = float(row['y'])
        t.transform.translation.z = float(row['z'])
        t.transform.rotation.x = float(row['qx'])
        t.transform.rotation.y = float(row['qy'])
        t.transform.rotation.z = float(row['qz'])
        t.transform.rotation.w = float(row['qw'])

        tf_msg = TFMessage()
        tf_msg.transforms = [t]
        return tf_msg

    # ──────────────────────────────────────────────────────────────
    # 타임스탬프 / bisect 헬퍼
    # ──────────────────────────────────────────────────────────────

    def _ns_to_time_msg(self, ns: int) -> Time:
        """나노초 정수를 builtin_interfaces/Time 메시지로 변환한다."""
        msg = Time()
        msg.sec = int(ns // 1_000_000_000)
        msg.nanosec = int(ns % 1_000_000_000)
        return msg

    def _to_bisect(self, rows: list) -> tuple:
        """rows를 stamp 기준 정렬 후 bisect 검색용 (stamps_list, rows_list) 튜플로 반환한다."""
        if not rows:
            return ([], [])
        s = sorted(rows, key=lambda r: r.get('stamp', 0))
        return ([r['stamp'] for r in s], s)

    def _find_nearest(self, bisect_data: tuple, stamp_ns: int):
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
