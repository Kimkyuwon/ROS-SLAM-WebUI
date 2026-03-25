#!/usr/bin/env python3
"""
kaist_converter.py
KAIST Complex Urban Dataset → ROS2 bag (.db3) 변환 모듈

기능:
  - KaistConverter.scan_directory():     KAIST 데이터셋 디렉토리 구조 탐색
  - KaistConverter.convert_to_ros2bag(): KAIST 데이터를 ROS2 bag 파일로 변환

출력 토픽 (03.task_plan_rules.mdc 기준):
  /imu/data_raw                    sensor_msgs/msg/Imu
  /gps/fix                         sensor_msgs/msg/NavSatFix
  /vrs_gps/fix                     sensor_msgs/msg/NavSatFix
  /ns2/velodyne_points             sensor_msgs/msg/PointCloud2  (VLP Left)
  /ns1/velodyne_points             sensor_msgs/msg/PointCloud2  (VLP Right)
  /lms511_back/scan                sensor_msgs/msg/LaserScan
  /lms511_middle/scan              sensor_msgs/msg/LaserScan
  /stereo/left/image_raw           sensor_msgs/msg/Image       (bayer_bggr8)
  /stereo/right/image_raw          sensor_msgs/msg/Image       (bayer_bggr8)
  /tf                              tf2_msgs/msg/TFMessage      (dynamic: world→base_link)
  /tf_static                       tf2_msgs/msg/TFMessage      (static: base_link→센서들)
"""

import bisect
import csv
import math
import os
import re
import shutil
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import (
    Image,
    Imu,
    LaserScan,
    NavSatFix,
    NavSatStatus,
    PointCloud2,
    PointField,
)

# 변환(bag 쓰기) 전용 의존성 - 없어도 직접 재생(scan/play)은 정상 동작
try:
    import cv2
    import rosbag2_py
    from rosbag2_py import TopicMetadata
    from rclpy.serialization import serialize_message
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    _ROSBAG2_AVAILABLE = True
except ImportError:
    cv2 = None
    rosbag2_py = None
    TopicMetadata = None
    serialize_message = None
    TransformStamped = None
    TFMessage = None
    _ROSBAG2_AVAILABLE = False

# SICK LMS511 LaserScan 파라미터 (kaist2bag-main/sick_converter.cpp 기준)
SICK_ANGLE_MIN = -1.65806281567
SICK_ANGLE_INCREMENT = 0.0116355288774
SICK_RANGE_MIN = 0.0
SICK_RANGE_MAX = 81.0


class KaistConverter:
    """KAIST Complex Urban Dataset을 ROS2 bag 파일로 변환하는 클래스.

    Usage:
        converter = KaistConverter()
        result = converter.scan_directory('/path/to/complex_urban')
        converter.convert_to_ros2bag(
            sequence_dir=result['sequences'][0]['path'],
            output_path='/path/to/output_bag',
            progress_cb=lambda pct, msg: print(f'{pct}% - {msg}')
        )
    """

    # ──────────────────────────────────────────────────────────────
    # Public API - scan_directory
    # ──────────────────────────────────────────────────────────────

    def scan_directory(self, base_dir: str) -> dict:
        """KAIST 데이터셋 최상위 디렉토리를 탐색하여 시퀀스 목록을 반환한다.

        calibration/ + sensor_data/ 존재 시 시퀀스로 인식.

        Args:
            base_dir: 사용자가 선택한 디렉토리 (예: /path/to/complex_urban)

        Returns:
            dict 형식:
            {
              'success': True,
              'sequences': [
                {'name': 'urban27-dongtan', 'path': '/path/.../urban27-dongtan'}, ...
              ]
            }
        """
        base_dir = os.path.abspath(base_dir)
        sequences = []

        # 1) base_dir 자체가 시퀀스인지 확인
        if self._is_kaist_sequence(base_dir):
            name = os.path.basename(base_dir)
            sequences.append({'name': name, 'path': base_dir})

        # 2) 하위 디렉토리에서 시퀀스 탐색
        if os.path.isdir(base_dir):
            for entry in sorted(os.listdir(base_dir)):
                full_path = os.path.join(base_dir, entry)
                if os.path.isdir(full_path) and self._is_kaist_sequence(full_path):
                    name = entry
                    if not any(s['path'] == full_path for s in sequences):
                        sequences.append({'name': name, 'path': full_path})

        return {
            'success': True,
            'sequences': sequences,
        }

    def convert_to_ros2bag(
        self,
        sequence_dir: str,
        output_path: str,
        sensors: list | None = None,
        progress_cb=None,
    ) -> None:
        """KAIST 시퀀스를 ROS2 bag (.db3) 파일로 변환한다.

        Args:
            sequence_dir: 시퀀스 디렉토리 (calibration/, sensor_data/, global_pose.csv 포함)
            output_path: 출력 bag 경로 (확장자 없음, 예: /path/to/output_bag)
            sensors: 포함할 센서 목록 (None이면 전체). 예: ['vlp_left','vlp_right','imu','gps',...]
            progress_cb: 진행률 콜백 (선택), signature: progress_cb(progress: int, message: str)
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

        sensor_dir = os.path.join(sequence_dir, 'sensor_data')
        calib_dir = os.path.join(sequence_dir, 'calibration')
        pose_csv = os.path.join(sequence_dir, 'global_pose.csv')

        # ── 1. 데이터 로드 ─────────────────────────────────────────
        _progress(0, 'Loading KAIST data...')
        global_poses = self._parse_global_pose(pose_csv)
        imu_file = os.path.join(sensor_dir, 'xsens_imu.csv')
        if not os.path.exists(imu_file):
            imu_file = os.path.join(sensor_dir, 'imu.csv')
        imu_rows = self._load_kaist_imu_csv(imu_file)
        gps_rows = self._load_kaist_gps_csv(os.path.join(sensor_dir, 'gps.csv'))
        vrs_file = os.path.join(sensor_dir, 'vrs_gps.csv')
        vrs_rows = self._load_kaist_gps_csv(vrs_file) if os.path.exists(vrs_file) else []

        # VLP Left (VLP_left_stamp.csv 또는 data_stamp.csv)
        vlp_left_stamps = self._load_stamp_csv(os.path.join(sensor_dir, 'VLP_left_stamp.csv'))
        if not vlp_left_stamps:
            vlp_left_stamps = self._load_stamp_csv(os.path.join(sensor_dir, 'data_stamp.csv'))
        vlp_left_dir = os.path.join(sensor_dir, 'VLP_left')
        has_vlp_left = os.path.isdir(vlp_left_dir) and vlp_left_stamps

        # VLP Right
        vlp_right_stamps = self._load_stamp_csv(os.path.join(sensor_dir, 'VLP_right_stamp.csv'))
        vlp_right_dir = os.path.join(sensor_dir, 'VLP_right')
        has_vlp_right = os.path.isdir(vlp_right_dir) and vlp_right_stamps

        # SICK Back (stamp from file or directory listing)
        sick_back_stamp_file = os.path.join(sensor_dir, 'SICK_back_stamp.csv')
        sick_back_stamps = self._load_stamp_csv(sick_back_stamp_file)
        if not sick_back_stamps:
            sick_back_dir = os.path.join(sensor_dir, 'SICK_back')
            if os.path.isdir(sick_back_dir):
                for f in sorted(Path(sick_back_dir).glob('*.bin')):
                    try:
                        sick_back_stamps.append(int(f.stem))
                    except ValueError:
                        pass
        sick_back_dir = os.path.join(sensor_dir, 'SICK_back')
        has_sick_back = os.path.isdir(sick_back_dir) and sick_back_stamps

        # SICK Middle
        sick_mid_stamp_file = os.path.join(sensor_dir, 'SICK_middle_stamp.csv')
        sick_mid_stamps = self._load_stamp_csv(sick_mid_stamp_file)
        if not sick_mid_stamps:
            sick_mid_dir = os.path.join(sensor_dir, 'SICK_middle')
            if os.path.isdir(sick_mid_dir):
                for f in sorted(Path(sick_mid_dir).glob('*.bin')):
                    try:
                        sick_mid_stamps.append(int(f.stem))
                    except ValueError:
                        pass
        sick_mid_dir = os.path.join(sensor_dir, 'SICK_middle')
        has_sick_mid = os.path.isdir(sick_mid_dir) and sick_mid_stamps

        # Stereo
        stereo_left_dir = os.path.join(sensor_dir, 'image', 'stereo_left')
        stereo_left_files = []
        if os.path.isdir(stereo_left_dir):
            for f in sorted(Path(stereo_left_dir).glob('*.png')):
                try:
                    stereo_left_files.append((int(f.stem), str(f)))
                except ValueError:
                    pass
        stereo_right_dir = os.path.join(sensor_dir, 'image', 'stereo_right')
        stereo_right_files = []
        if os.path.isdir(stereo_right_dir):
            for f in sorted(Path(stereo_right_dir).glob('*.png')):
                try:
                    stereo_right_files.append((int(f.stem), str(f)))
                except ValueError:
                    pass

        # 마스터 타임라인: 모든 stamp 병합 후 정렬
        all_stamps = set()
        if has_vlp_left:
            all_stamps.update(vlp_left_stamps)
        if has_vlp_right:
            all_stamps.update(vlp_right_stamps)
        if has_sick_back:
            all_stamps.update(sick_back_stamps)
        if has_sick_mid:
            all_stamps.update(sick_mid_stamps)
        for stamp, _ in stereo_left_files:
            all_stamps.add(stamp)
        for stamp, _ in stereo_right_files:
            all_stamps.add(stamp)
        if imu_rows:
            all_stamps.update(r['stamp'] for r in imu_rows)
        if gps_rows:
            all_stamps.update(r['stamp'] for r in gps_rows)
        if vrs_rows:
            all_stamps.update(r['stamp'] for r in vrs_rows)
        if global_poses:
            all_stamps.update(p[0] for p in global_poses)

        if not all_stamps:
            raise RuntimeError('변환할 데이터가 없습니다. sensor_data를 확인하세요.')

        master_stamps = sorted(all_stamps)

        # ── 1.5 bisect용 정렬 데이터 (O(log n) 검색) ─────────────────
        def _to_bisect(rows: list, key: str = 'stamp'):
            if not rows:
                return ([], [])
            s = sorted(rows, key=lambda r: r.get(key, 0))
            return ([r[key] for r in s], s)

        imu_bisect = _to_bisect(imu_rows)
        gps_bisect = _to_bisect(gps_rows)
        vrs_bisect = _to_bisect(vrs_rows)
        poses_sorted = sorted(global_poses, key=lambda p: p[0]) if global_poses else []
        pose_stamps = [p[0] for p in poses_sorted] if poses_sorted else []

        # ── 2. 출력 경로 준비 ─────────────────────────────────────
        if os.path.exists(output_path):
            shutil.rmtree(output_path)
        parent_dir = os.path.dirname(output_path)
        if parent_dir:
            os.makedirs(parent_dir, exist_ok=True)

        # ── 3. rosbag2 Writer 초기화 ─────────────────────────────
        _progress(2, 'Initializing bag writer...')
        writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(uri=output_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        )
        writer.open(storage_options, converter_options)

        # ── 4. 토픽 등록 ─────────────────────────────────────────
        topics = [
            ('/imu/data_raw', 'sensor_msgs/msg/Imu'),
            ('/gps/fix', 'sensor_msgs/msg/NavSatFix'),
            ('/vrs_gps/fix', 'sensor_msgs/msg/NavSatFix'),
            ('/ns2/velodyne_points', 'sensor_msgs/msg/PointCloud2'),
            ('/ns1/velodyne_points', 'sensor_msgs/msg/PointCloud2'),
            ('/lms511_back/scan', 'sensor_msgs/msg/LaserScan'),
            ('/lms511_middle/scan', 'sensor_msgs/msg/LaserScan'),
            ('/stereo/left/image_raw', 'sensor_msgs/msg/Image'),
            ('/stereo/right/image_raw', 'sensor_msgs/msg/Image'),
            ('/tf', 'tf2_msgs/msg/TFMessage'),
            ('/tf_static', 'tf2_msgs/msg/TFMessage'),
        ]
        for idx, (topic_name, topic_type) in enumerate(topics):
            writer.create_topic(TopicMetadata(
                id=idx,
                name=topic_name,
                type=topic_type,
                serialization_format='cdr',
            ))

        # ── 5. Static TF (1회) ─────────────────────────────────────
        _progress(3, 'Writing static TF...')
        first_stamp = master_stamps[0]
        stamp_time = self._ns_to_time_msg(first_stamp)
        static_tf_msg = self._build_static_tf(calib_dir, stamp_time)
        if static_tf_msg:
            writer.write('/tf_static', serialize_message(static_tf_msg), first_stamp)

        # ── 6. 진행률 추적 (2% 간격으로 throttling) ─────────────────
        total = len(master_stamps)
        processed = 0
        _last_pct = [0]

        def _tick(msg: str):
            nonlocal processed
            processed += 1
            pct = min(3 + int(processed / total * 95), 98)
            if pct >= _last_pct[0] + 2 or pct >= 98:
                _last_pct[0] = pct
                _progress(pct, msg)

        def _find_pose(ts_ns: int):
            if not poses_sorted:
                return None
            idx = bisect.bisect_left(pose_stamps, ts_ns)
            if idx == 0:
                return poses_sorted[0]
            if idx >= len(poses_sorted):
                return poses_sorted[-1]
            if abs(pose_stamps[idx] - ts_ns) < abs(pose_stamps[idx - 1] - ts_ns):
                return poses_sorted[idx]
            return poses_sorted[idx - 1]

        # ── 7. 메시지 기록 (마스터 타임라인 기준) ─────────────────
        vlp_left_set = set(vlp_left_stamps) if has_vlp_left else set()
        vlp_right_set = set(vlp_right_stamps) if has_vlp_right else set()
        sick_back_set = set(sick_back_stamps) if has_sick_back else set()
        sick_mid_set = set(sick_mid_stamps) if has_sick_mid else set()
        stereo_left_map = {s: p for s, p in stereo_left_files}
        stereo_right_map = {s: p for s, p in stereo_right_files}

        # 이미지 병렬 디코딩 (스테레오 I/O 오버랩)
        _img_executor = ThreadPoolExecutor(max_workers=4) if cv2 else None

        for ts_ns in master_stamps:
            stamp_time = self._ns_to_time_msg(ts_ns)

            # 이미지 디코딩 미리 시작 (다른 작업과 오버랩)
            left_future = None
            right_future = None
            if _img_executor and cv2:
                if ts_ns in stereo_left_map:
                    left_future = _img_executor.submit(
                        cv2.imread, stereo_left_map[ts_ns], cv2.IMREAD_UNCHANGED
                    )
                if ts_ns in stereo_right_map:
                    right_future = _img_executor.submit(
                        cv2.imread, stereo_right_map[ts_ns], cv2.IMREAD_UNCHANGED
                    )

            # Dynamic TF (world → base_link)
            pose = _find_pose(ts_ns)
            if pose and TFMessage:
                _, R, T = pose
                tf_msg = self._make_dynamic_tf(R, T, stamp_time)
                if tf_msg:
                    writer.write('/tf', serialize_message(tf_msg), ts_ns)

            # IMU (bisect O(log n))
            imu_row = self._find_nearest_by_stamp(imu_bisect, ts_ns)
            if imu_row:
                imu_msg = self._make_imu_msg(imu_row, stamp_time)
                writer.write('/imu/data_raw', serialize_message(imu_msg), ts_ns)

            # GPS (bisect)
            gps_row = self._find_nearest_by_stamp(gps_bisect, ts_ns)
            if gps_row:
                gps_msg = self._make_navsatfix_msg(gps_row, stamp_time)
                writer.write('/gps/fix', serialize_message(gps_msg), ts_ns)

            # VRS GPS (bisect)
            vrs_row = self._find_nearest_by_stamp(vrs_bisect, ts_ns)
            if vrs_row:
                vrs_msg = self._make_navsatfix_msg(vrs_row, stamp_time)
                writer.write('/vrs_gps/fix', serialize_message(vrs_msg), ts_ns)

            # VLP Left
            if ts_ns in vlp_left_set:
                bin_path = os.path.join(vlp_left_dir, f'{ts_ns}.bin')
                vlp_msg = self._make_vlp_msg(bin_path, 'left_velodyne', stamp_time)
                if vlp_msg:
                    writer.write('/ns2/velodyne_points', serialize_message(vlp_msg), ts_ns)

            # VLP Right
            if ts_ns in vlp_right_set:
                bin_path = os.path.join(vlp_right_dir, f'{ts_ns}.bin')
                vlp_msg = self._make_vlp_msg(bin_path, 'right_velodyne', stamp_time)
                if vlp_msg:
                    writer.write('/ns1/velodyne_points', serialize_message(vlp_msg), ts_ns)

            # SICK Back
            if ts_ns in sick_back_set:
                bin_path = os.path.join(sick_back_dir, f'{ts_ns}.bin')
                scan_msg = self._make_laserscan_msg(bin_path, 'back_sick', stamp_time)
                if scan_msg:
                    writer.write('/lms511_back/scan', serialize_message(scan_msg), ts_ns)

            # SICK Middle
            if ts_ns in sick_mid_set:
                bin_path = os.path.join(sick_mid_dir, f'{ts_ns}.bin')
                scan_msg = self._make_laserscan_msg(bin_path, 'middle_sick', stamp_time)
                if scan_msg:
                    writer.write('/lms511_middle/scan', serialize_message(scan_msg), ts_ns)

            # Stereo Left/Right (병렬 디코딩으로 I/O 오버랩)
            if ts_ns in stereo_left_map:
                if left_future is not None:
                    img = left_future.result()
                    if img is not None:
                        img_msg = self._make_stereo_msg_from_ndarray(img, stamp_time, 'stereo_left')
                        if img_msg:
                            writer.write('/stereo/left/image_raw', serialize_message(img_msg), ts_ns)
                else:
                    img_msg = self._make_stereo_msg(stereo_left_map[ts_ns], stamp_time, 'stereo_left')
                    if img_msg:
                        writer.write('/stereo/left/image_raw', serialize_message(img_msg), ts_ns)

            if ts_ns in stereo_right_map:
                if right_future is not None:
                    img = right_future.result()
                    if img is not None:
                        img_msg = self._make_stereo_msg_from_ndarray(img, stamp_time, 'stereo_right')
                        if img_msg:
                            writer.write('/stereo/right/image_raw', serialize_message(img_msg), ts_ns)
                else:
                    img_msg = self._make_stereo_msg(stereo_right_map[ts_ns], stamp_time, 'stereo_right')
                    if img_msg:
                        writer.write('/stereo/right/image_raw', serialize_message(img_msg), ts_ns)

            _tick('Converting KAIST data...')

        if _img_executor:
            _img_executor.shutdown(wait=True)

        del writer
        _progress(100, 'Conversion complete!')

    def convert_to_ros1bag(
        self,
        sequence_dir: str,
        output_bag_path: str,
        sensors: list | None = None,
        progress_cb=None,
    ) -> None:
        """KAIST 시퀀스를 ROS1 .bag 파일로 변환한다 (rosbags 라이브러리 사용).

        rosbags.rosbag1.Writer + migrate_bytes()를 통해 ROS2 CDR 직렬화 후
        즉시 ROS1 raw bytes로 변환하여 .bag에 기록한다.

        Args:
            sequence_dir: 시퀀스 디렉토리 (calibration/, sensor_data/, global_pose.csv 포함)
            output_bag_path: 출력 ROS1 .bag 파일 경로 (예: /path/to/urban27-dongtan.bag)
            sensors: 포함할 센서 목록 (None이면 전체)
            progress_cb: 진행률 콜백 (선택), signature: progress_cb(progress: int, message: str)
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

        src_typestore = get_typestore(Stores.ROS2_JAZZY)
        dst_typestore = get_typestore(Stores.ROS1_NOETIC)
        migrate_cache: dict = {}

        sensor_dir = os.path.join(sequence_dir, 'sensor_data')
        calib_dir = os.path.join(sequence_dir, 'calibration')
        pose_csv = os.path.join(sequence_dir, 'global_pose.csv')

        # ── 1. 데이터 로드 (convert_to_ros2bag와 동일) ─────────────────
        _progress(0, 'Loading KAIST data...')
        global_poses = self._parse_global_pose(pose_csv)
        imu_file = os.path.join(sensor_dir, 'xsens_imu.csv')
        if not os.path.exists(imu_file):
            imu_file = os.path.join(sensor_dir, 'imu.csv')
        imu_rows = self._load_kaist_imu_csv(imu_file)
        gps_rows = self._load_kaist_gps_csv(os.path.join(sensor_dir, 'gps.csv'))
        vrs_file = os.path.join(sensor_dir, 'vrs_gps.csv')
        vrs_rows = self._load_kaist_gps_csv(vrs_file) if os.path.exists(vrs_file) else []

        vlp_left_stamps = self._load_stamp_csv(os.path.join(sensor_dir, 'VLP_left_stamp.csv'))
        if not vlp_left_stamps:
            vlp_left_stamps = self._load_stamp_csv(os.path.join(sensor_dir, 'data_stamp.csv'))
        vlp_left_dir = os.path.join(sensor_dir, 'VLP_left')
        has_vlp_left = os.path.isdir(vlp_left_dir) and vlp_left_stamps

        vlp_right_stamps = self._load_stamp_csv(os.path.join(sensor_dir, 'VLP_right_stamp.csv'))
        vlp_right_dir = os.path.join(sensor_dir, 'VLP_right')
        has_vlp_right = os.path.isdir(vlp_right_dir) and vlp_right_stamps

        sick_back_stamp_file = os.path.join(sensor_dir, 'SICK_back_stamp.csv')
        sick_back_stamps = self._load_stamp_csv(sick_back_stamp_file)
        if not sick_back_stamps:
            sick_back_dir = os.path.join(sensor_dir, 'SICK_back')
            if os.path.isdir(sick_back_dir):
                for f in sorted(Path(sick_back_dir).glob('*.bin')):
                    try:
                        sick_back_stamps.append(int(f.stem))
                    except ValueError:
                        pass
        sick_back_dir = os.path.join(sensor_dir, 'SICK_back')
        has_sick_back = os.path.isdir(sick_back_dir) and sick_back_stamps

        sick_mid_stamp_file = os.path.join(sensor_dir, 'SICK_middle_stamp.csv')
        sick_mid_stamps = self._load_stamp_csv(sick_mid_stamp_file)
        if not sick_mid_stamps:
            sick_mid_dir = os.path.join(sensor_dir, 'SICK_middle')
            if os.path.isdir(sick_mid_dir):
                for f in sorted(Path(sick_mid_dir).glob('*.bin')):
                    try:
                        sick_mid_stamps.append(int(f.stem))
                    except ValueError:
                        pass
        sick_mid_dir = os.path.join(sensor_dir, 'SICK_middle')
        has_sick_mid = os.path.isdir(sick_mid_dir) and sick_mid_stamps

        stereo_left_dir = os.path.join(sensor_dir, 'image', 'stereo_left')
        stereo_left_files = []
        if os.path.isdir(stereo_left_dir):
            for f in sorted(Path(stereo_left_dir).glob('*.png')):
                try:
                    stereo_left_files.append((int(f.stem), str(f)))
                except ValueError:
                    pass
        stereo_right_dir = os.path.join(sensor_dir, 'image', 'stereo_right')
        stereo_right_files = []
        if os.path.isdir(stereo_right_dir):
            for f in sorted(Path(stereo_right_dir).glob('*.png')):
                try:
                    stereo_right_files.append((int(f.stem), str(f)))
                except ValueError:
                    pass

        all_stamps = set()
        if has_vlp_left:
            all_stamps.update(vlp_left_stamps)
        if has_vlp_right:
            all_stamps.update(vlp_right_stamps)
        if has_sick_back:
            all_stamps.update(sick_back_stamps)
        if has_sick_mid:
            all_stamps.update(sick_mid_stamps)
        for stamp, _ in stereo_left_files:
            all_stamps.add(stamp)
        for stamp, _ in stereo_right_files:
            all_stamps.add(stamp)
        if imu_rows:
            all_stamps.update(r['stamp'] for r in imu_rows)
        if gps_rows:
            all_stamps.update(r['stamp'] for r in gps_rows)
        if vrs_rows:
            all_stamps.update(r['stamp'] for r in vrs_rows)
        if global_poses:
            all_stamps.update(p[0] for p in global_poses)

        if not all_stamps:
            raise RuntimeError('변환할 데이터가 없습니다. sensor_data를 확인하세요.')

        master_stamps = sorted(all_stamps)

        # ── 1.5 bisect용 정렬 데이터 (ROS2와 동일) ─────────────────
        def _to_bisect(rows: list, key: str = 'stamp'):
            if not rows:
                return ([], [])
            s = sorted(rows, key=lambda r: r.get(key, 0))
            return ([r[key] for r in s], s)

        imu_bisect = _to_bisect(imu_rows)
        gps_bisect = _to_bisect(gps_rows)
        vrs_bisect = _to_bisect(vrs_rows)
        poses_sorted = sorted(global_poses, key=lambda p: p[0]) if global_poses else []
        pose_stamps = [p[0] for p in poses_sorted] if poses_sorted else []

        # ── 2. 출력 경로 준비 ─────────────────────────────────────
        if os.path.isfile(output_bag_path):
            os.remove(output_bag_path)
        parent_dir = os.path.dirname(output_bag_path)
        if parent_dir:
            os.makedirs(parent_dir, exist_ok=True)

        # ── 3. 토픽 목록 ─────────────────────────────────────────
        topics = [
            ('/imu/data_raw', 'sensor_msgs/msg/Imu'),
            ('/gps/fix', 'sensor_msgs/msg/NavSatFix'),
            ('/vrs_gps/fix', 'sensor_msgs/msg/NavSatFix'),
            ('/ns2/velodyne_points', 'sensor_msgs/msg/PointCloud2'),
            ('/ns1/velodyne_points', 'sensor_msgs/msg/PointCloud2'),
            ('/lms511_back/scan', 'sensor_msgs/msg/LaserScan'),
            ('/lms511_middle/scan', 'sensor_msgs/msg/LaserScan'),
            ('/stereo/left/image_raw', 'sensor_msgs/msg/Image'),
            ('/stereo/right/image_raw', 'sensor_msgs/msg/Image'),
            ('/tf', 'tf2_msgs/msg/TFMessage'),
            ('/tf_static', 'tf2_msgs/msg/TFMessage'),
        ]

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

        total = len(master_stamps)
        processed = 0
        _last_pct = [0]

        def _tick(msg: str):
            nonlocal processed
            processed += 1
            pct = min(3 + int(processed / total * 95), 98)
            if pct >= _last_pct[0] + 2 or pct >= 98:
                _last_pct[0] = pct
                _progress(pct, msg)

        def _find_pose(ts_ns: int):
            if not poses_sorted:
                return None
            idx = bisect.bisect_left(pose_stamps, ts_ns)
            if idx == 0:
                return poses_sorted[0]
            if idx >= len(poses_sorted):
                return poses_sorted[-1]
            if abs(pose_stamps[idx] - ts_ns) < abs(pose_stamps[idx - 1] - ts_ns):
                return poses_sorted[idx]
            return poses_sorted[idx - 1]

        first_stamp = master_stamps[0]
        stamp_time = self._ns_to_time_msg(first_stamp)
        static_tf_msg = self._build_static_tf(calib_dir, stamp_time)

        vlp_left_set = set(vlp_left_stamps) if has_vlp_left else set()
        vlp_right_set = set(vlp_right_stamps) if has_vlp_right else set()
        sick_back_set = set(sick_back_stamps) if has_sick_back else set()
        sick_mid_set = set(sick_mid_stamps) if has_sick_mid else set()
        stereo_left_map = {s: p for s, p in stereo_left_files}
        stereo_right_map = {s: p for s, p in stereo_right_files}

        _progress(2, 'Initializing ROS1 bag writer...')
        with Ros1Writer(output_bag_path) as writer:
            connections: dict = {}
            for topic_name, ros2_type in topics:
                if not _ensure_type(ros2_type):
                    continue
                try:
                    conn = writer.add_connection(topic_name, ros2_type, typestore=dst_typestore)
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

            _img_executor = ThreadPoolExecutor(max_workers=4) if cv2 else None

            static_tf_written = False
            for ts_ns in master_stamps:
                stamp_time = self._ns_to_time_msg(ts_ns)

                left_future = None
                right_future = None
                if _img_executor and cv2:
                    if ts_ns in stereo_left_map:
                        left_future = _img_executor.submit(
                            cv2.imread, stereo_left_map[ts_ns], cv2.IMREAD_UNCHANGED
                        )
                    if ts_ns in stereo_right_map:
                        right_future = _img_executor.submit(
                            cv2.imread, stereo_right_map[ts_ns], cv2.IMREAD_UNCHANGED
                        )

                if not static_tf_written and static_tf_msg:
                    _write('/tf_static', static_tf_msg, ts_ns)
                    static_tf_written = True

                pose = _find_pose(ts_ns)
                if pose and TFMessage:
                    _, R, T = pose
                    tf_msg = self._make_dynamic_tf(R, T, stamp_time)
                    if tf_msg:
                        _write('/tf', tf_msg, ts_ns)

                imu_row = self._find_nearest_by_stamp(imu_bisect, ts_ns)
                if imu_row:
                    _write('/imu/data_raw', self._make_imu_msg(imu_row, stamp_time), ts_ns)

                gps_row = self._find_nearest_by_stamp(gps_bisect, ts_ns)
                if gps_row:
                    _write('/gps/fix', self._make_navsatfix_msg(gps_row, stamp_time), ts_ns)

                vrs_row = self._find_nearest_by_stamp(vrs_bisect, ts_ns)
                if vrs_row:
                    _write('/vrs_gps/fix', self._make_navsatfix_msg(vrs_row, stamp_time), ts_ns)

                if ts_ns in vlp_left_set:
                    bin_path = os.path.join(vlp_left_dir, f'{ts_ns}.bin')
                    vlp_msg = self._make_vlp_msg(bin_path, 'left_velodyne', stamp_time)
                    if vlp_msg:
                        _write('/ns2/velodyne_points', vlp_msg, ts_ns)

                if ts_ns in vlp_right_set:
                    bin_path = os.path.join(vlp_right_dir, f'{ts_ns}.bin')
                    vlp_msg = self._make_vlp_msg(bin_path, 'right_velodyne', stamp_time)
                    if vlp_msg:
                        _write('/ns1/velodyne_points', vlp_msg, ts_ns)

                if ts_ns in sick_back_set:
                    bin_path = os.path.join(sick_back_dir, f'{ts_ns}.bin')
                    scan_msg = self._make_laserscan_msg(bin_path, 'back_sick', stamp_time)
                    if scan_msg:
                        _write('/lms511_back/scan', scan_msg, ts_ns)

                if ts_ns in sick_mid_set:
                    bin_path = os.path.join(sick_mid_dir, f'{ts_ns}.bin')
                    scan_msg = self._make_laserscan_msg(bin_path, 'middle_sick', stamp_time)
                    if scan_msg:
                        _write('/lms511_middle/scan', scan_msg, ts_ns)

                if ts_ns in stereo_left_map:
                    if left_future is not None:
                        img = left_future.result()
                        if img is not None:
                            img_msg = self._make_stereo_msg_from_ndarray(img, stamp_time, 'stereo_left')
                            if img_msg:
                                _write('/stereo/left/image_raw', img_msg, ts_ns)
                    else:
                        img_msg = self._make_stereo_msg(stereo_left_map[ts_ns], stamp_time, 'stereo_left')
                        if img_msg:
                            _write('/stereo/left/image_raw', img_msg, ts_ns)

                if ts_ns in stereo_right_map:
                    if right_future is not None:
                        img = right_future.result()
                        if img is not None:
                            img_msg = self._make_stereo_msg_from_ndarray(img, stamp_time, 'stereo_right')
                            if img_msg:
                                _write('/stereo/right/image_raw', img_msg, ts_ns)
                    else:
                        img_msg = self._make_stereo_msg(stereo_right_map[ts_ns], stamp_time, 'stereo_right')
                        if img_msg:
                            _write('/stereo/right/image_raw', img_msg, ts_ns)

                _tick('Converting KAIST data...')

            if _img_executor:
                _img_executor.shutdown(wait=True)

        _progress(100, 'Conversion complete!')

    def _is_kaist_sequence(self, path: str) -> bool:
        """calibration/ + sensor_data/ 존재 시 True."""
        calib_dir = os.path.join(path, 'calibration')
        sensor_dir = os.path.join(path, 'sensor_data')
        if not os.path.isdir(calib_dir) or not os.path.isdir(sensor_dir):
            return False
        # VLP_left_stamp.csv 또는 data_stamp.csv 존재 확인
        vlp_stamp = os.path.join(sensor_dir, 'VLP_left_stamp.csv')
        data_stamp = os.path.join(sensor_dir, 'data_stamp.csv')
        return os.path.isfile(vlp_stamp) or os.path.isfile(data_stamp)

    # ──────────────────────────────────────────────────────────────
    # Calibration 파싱 - _parse_vehicle2sensor_calib
    # ──────────────────────────────────────────────────────────────

    def _parse_vehicle2sensor_calib(self, filepath: str) -> tuple:
        """Vehicle2{Sensor}.txt 파일을 파싱하여 (R, T)를 반환한다.

        포맷:
          RPY: {roll} {pitch} {yaw}  (degree)
          R: {r00} {r01} {r02} {r10} {r11} {r12} {r20} {r21} {r22}
          T: {tx} {ty} {tz}

        Args:
            filepath: calibration 파일 경로

        Returns:
            (R: np.ndarray 3x3, T: np.ndarray 3) 또는 (None, None)
        """
        if not os.path.exists(filepath):
            return None, None

        R = None
        T = None

        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if line.startswith('R:'):
                    vals = line[2:].strip().split()
                    if len(vals) >= 9:
                        R = np.array([float(v) for v in vals[:9]], dtype=np.float64).reshape(3, 3)
                elif line.startswith('T:'):
                    vals = line[2:].strip().split()
                    if len(vals) >= 3:
                        T = np.array([float(v) for v in vals[:3]], dtype=np.float64)

        return R, T

    # ──────────────────────────────────────────────────────────────
    # global_pose 파싱 - _parse_global_pose
    # ──────────────────────────────────────────────────────────────

    def _parse_global_pose(self, pose_csv: str) -> list:
        """global_pose.csv를 파싱하여 [(stamp_ns, R, T), ...] 리스트를 반환한다.

        지원 포맷 (urban27-dongtan 등):
          stamp, r00,r01,r02, tx, r10,r11,r12, ty, r20,r21,r22, tz
          (R과 T가 교차: tx,ty,tz가 UTM 등 큰 값일 수 있음)

        대안 포맷:
          stamp, r00,r01,r02, r10,r11,r12, r20,r21,r22, tx,ty,tz

        Args:
            pose_csv: global_pose.csv 파일 경로

        Returns:
            [(stamp_ns: int, R: np.ndarray 3x3, T: np.ndarray 3), ...]
        """
        result = []
        if not os.path.exists(pose_csv):
            return result

        with open(pose_csv, 'r') as f:
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

                    # R 행렬 유효성 검사 (회전행렬 원소는 [-1,1])
                    # 표준: stamp, r00..r22, tx,ty,tz
                    R_std = np.array(vals[:9], dtype=np.float64).reshape(3, 3)
                    T_std = np.array(vals[9:12], dtype=np.float64)
                    r_flat_std = np.abs(R_std).flatten()

                    if np.all(r_flat_std <= 1.5):
                        result.append((stamp_ns, R_std, T_std))
                    else:
                        # 교차 포맷: r00,r01,r02, tx, r10,r11,r12, ty, r20,r21,r22, tz
                        R_alt = np.array([
                            vals[0:3],   # r00,r01,r02
                            vals[4:7],   # r10,r11,r12
                            vals[8:11],  # r20,r21,r22
                        ], dtype=np.float64)
                        T_alt = np.array([vals[3], vals[7], vals[11]], dtype=np.float64)
                        r_flat_alt = np.abs(R_alt).flatten()
                        if np.all(r_flat_alt <= 1.5):
                            result.append((stamp_ns, R_alt, T_alt))
                except (ValueError, IndexError):
                    continue
        return result

    # ──────────────────────────────────────────────────────────────
    # 수학 헬퍼
    # ──────────────────────────────────────────────────────────────

    def _rotation_matrix_to_quaternion(self, R: np.ndarray) -> tuple:
        """3×3 회전 행렬을 quaternion (x, y, z, w)으로 변환한다.

        Shepperd's method 사용 (수치 안정성 보장).
        KITTI kitti_converter와 동일.
        """
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

    def _rpy_to_rotation_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Roll, Pitch, Yaw (rad)를 3×3 회전 행렬로 변환한다.

        회전 순서 (intrinsic): Rz(yaw) @ Ry(pitch) @ Rx(roll)
        """
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)

        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
        return Rz @ Ry @ Rx

    # ──────────────────────────────────────────────────────────────
    # 타임스탬프 헬퍼
    # ──────────────────────────────────────────────────────────────

    def _ns_to_time_msg(self, ns: int) -> Time:
        """나노초 정수를 builtin_interfaces/Time 메시지로 변환한다."""
        msg = Time()
        msg.sec = int(ns // 1_000_000_000)
        msg.nanosec = int(ns % 1_000_000_000)
        return msg

    def _load_stamp_csv(self, filepath: str) -> list:
        """타임스탬프 CSV 파일을 읽어 나노초 정수 리스트로 반환한다.

        VLP_left_stamp.csv, SICK_back_stamp.csv 등: 1줄 1개 stamp (nanosec)
        """
        stamps = []
        if not os.path.exists(filepath):
            return stamps
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    stamps.append(int(line))
                except ValueError:
                    continue
        return stamps

    def _load_kaist_imu_csv(self, filepath: str) -> list:
        """KAIST xsens_imu.csv 파싱 (헤더 없음). numpy로 벌크 로드 후 변환.

        포맷: stamp, qx, qy, qz, qw, ex, ey, ez, gx, gy, gz, ax, ay, az, mx, my, mz (17열)
        """
        if not os.path.exists(filepath):
            return []
        try:
            data = np.loadtxt(filepath, delimiter=',')
            if data.ndim == 1:
                data = data.reshape(1, -1)
            result = []
            for row in data:
                if len(row) < 17:
                    continue
                stamp_ns = int(row[0])
                if 0 < abs(stamp_ns) < 1e15:
                    stamp_ns = int(stamp_ns * 1e9)
                result.append({
                    'stamp': stamp_ns,
                    'qx': float(row[1]), 'qy': float(row[2]), 'qz': float(row[3]), 'qw': float(row[4]),
                    'ex': float(row[5]), 'ey': float(row[6]), 'ez': float(row[7]),
                    'gx': float(row[8]), 'gy': float(row[9]), 'gz': float(row[10]),
                    'ax': float(row[11]), 'ay': float(row[12]), 'az': float(row[13]),
                    'mx': float(row[14]), 'my': float(row[15]), 'mz': float(row[16]),
                })
            return result
        except Exception:
            return []

    def _load_kaist_gps_csv(self, filepath: str) -> list:
        """KAIST gps.csv / vrs_gps.csv 파싱 (헤더 없음). numpy로 벌크 로드.

        포맷: stamp, lat, lon, alt, cov0..cov8 (13열)
        """
        if not os.path.exists(filepath):
            return []
        try:
            data = np.loadtxt(filepath, delimiter=',')
            if data.ndim == 1:
                data = data.reshape(1, -1)
            result = []
            for row in data:
                if len(row) < 13:
                    continue
                stamp_ns = int(row[0])
                if 0 < abs(stamp_ns) < 1e15:
                    stamp_ns = int(stamp_ns * 1e9)
                cov = [float(row[i]) if i < len(row) else 0.0 for i in range(4, 13)]
                result.append({
                    'stamp': stamp_ns,
                    'lat': float(row[1]), 'lon': float(row[2]), 'alt': float(row[3]),
                    'cov': cov,
                })
            return result
        except Exception:
            return []

    def _load_csv_with_header(self, filepath: str) -> list:
        """헤더가 있는 CSV 파일을 파싱하여 dict 리스트로 반환한다.

        KAIST 포맷: timestamp, latitude, longitude, altitude (gps)
        xsens_imu/imu: timestamp, quaternion x/y/z/w, Gyro x/y/z, Acceleration x/y/z
        """
        result = []
        if not os.path.exists(filepath):
            return result
        try:
            with open(filepath, 'r', newline='', encoding='utf-8') as f:
                reader = csv.DictReader(f, skipinitialspace=True)
                for row in reader:
                    # 키를 소문자로 정규화하여 대소문자 무관 검색 지원
                    d = {}
                    for k, v in row.items():
                        k_orig = k.strip()
                        if not k_orig:
                            continue
                        k_lower = k_orig.lower().replace(' ', '_')
                        try:
                            val = float(v) if v else 0.0
                        except ValueError:
                            val = v.strip() if v else ''
                        d[k_lower] = val
                        d[k_orig] = val  # 원본 키도 유지 (하위 호환)
                    # stamp 컬럼 정규화 (timestamp, nanosec 등 → stamp)
                    stamp_val = None
                    for key in ('stamp', 'timestamp', 'nanosec', 'time'):
                        for dk, dv in list(d.items()):
                            if dk.lower() == key and dv is not None and str(dv).strip():
                                try:
                                    stamp_val = int(float(dv))
                                    break
                                except (ValueError, TypeError):
                                    pass
                        if stamp_val is not None:
                            break
                    if stamp_val is not None:
                        # ROS timestamp가 초 단위(1e9 미만)이면 나노초로 변환
                        if 0 < abs(stamp_val) < 1e15:
                            stamp_val = int(stamp_val * 1e9)
                        d['stamp'] = stamp_val
                    result.append(d)
        except Exception:
            pass
        return result

    def _load_gps_csv(self, filepath: str) -> list:
        """gps.csv / vrs_gps.csv를 파싱하여 NavSatFix용 dict 리스트로 반환한다.

        KAIST: timestamp, latitude, longitude, altitude, 9-tuple covariance
        """
        rows = self._load_csv_with_header(filepath)
        for r in rows:
            cov = []
            for i in range(9):
                val = self._get_field(r, f'cov{i}', f'c{i}', f'cov_{i}')
                cov.append(val if isinstance(val, (int, float)) else 0.0)
            r['cov'] = cov
        return rows

    def _find_nearest_by_stamp(self, items: list, stamp_ns: int, key: str = 'stamp'):
        """stamp_ns에 가장 가까운 항목을 반환한다. items는 stamp 기준 정렬되어 있어야 한다.
        stamps 리스트를 인자로 받으면 bisect O(log n) 검색, 없으면 선형 스캔.
        """
        if not items:
            return None
        import bisect
        # items가 (stamps_list, rows_list) 튜플이면 bisect 사용 (web_server에서 캐시)
        if isinstance(items, tuple) and len(items) == 2:
            stamps_list, rows_list = items
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
        # 기존: dict 리스트 (정렬 가정, 선형 스캔)
        best = items[0]
        best_diff = abs(items[0].get(key, 0) - stamp_ns)
        for it in items:
            diff = abs(it.get(key, 0) - stamp_ns)
            if diff < best_diff:
                best_diff = diff
                best = it
        return best

    def _find_nearest_pose(self, poses: list, stamp_ns: int):
        """global_pose 리스트에서 stamp_ns에 가장 가까운 (stamp_ns, R, T)를 반환한다."""
        if not poses:
            return None
        best = poses[0]
        best_diff = abs(poses[0][0] - stamp_ns)
        for p in poses:
            diff = abs(p[0] - stamp_ns)
            if diff < best_diff:
                best_diff = diff
                best = p
        return best

    # ──────────────────────────────────────────────────────────────
    # 메시지 생성 헬퍼 - _make_vlp_msg, _make_imu_msg, ...
    # ──────────────────────────────────────────────────────────────

    def _make_vlp_msg(self, bin_path: str, frame_id: str, stamp: Time) -> PointCloud2 | None:
        """VLP .bin 파일에서 sensor_msgs/PointCloud2 메시지를 생성한다.

        .bin 형식: float32×4 per point (x, y, z, intensity) — KITTI와 동일
        """
        try:
            points = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
        except Exception:
            return None

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = points.shape[0]
        msg.is_bigendian = False
        msg.is_dense = True

        field_names = ['x', 'y', 'z', 'intensity']
        fields = []
        for idx, fname in enumerate(field_names):
            f = PointField()
            f.name = fname
            f.offset = idx * 4
            f.datatype = PointField.FLOAT32
            f.count = 1
            fields.append(f)
        msg.fields = fields

        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.data = points.tobytes()
        return msg

    def _get_field(self, fields: dict, *keys, default=0.0):
        """여러 가능한 컬럼명 중 첫 번째로 존재하는 값을 반환한다. 대소문자 무관."""
        for k in keys:
            k_lower = k.lower().replace(' ', '_')
            for fk, fv in fields.items():
                if (fk == k or fk == k_lower or
                        fk.lower().replace(' ', '_') == k_lower) and fv is not None and fv != '':
                    try:
                        return float(fv)
                    except (ValueError, TypeError):
                        pass
        return default

    def _make_imu_msg(self, fields: dict, stamp: Time) -> Imu:
        """xsens_imu.csv / imu.csv 한 행으로 sensor_msgs/Imu 메시지를 생성한다.

        KAIST Ver2: timestamp, quaternion x/y/z/w, Euler x/y/z, Gyro x/y/z,
        Acceleration x/y/z, MagnetField x/y/z
        """
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = 'imu'

        msg.orientation.x = self._get_field(
            fields, 'qx', 'quat_x', 'quaternion_x', 'orientation_x')
        msg.orientation.y = self._get_field(
            fields, 'qy', 'quat_y', 'quaternion_y', 'orientation_y')
        msg.orientation.z = self._get_field(
            fields, 'qz', 'quat_z', 'quaternion_z', 'orientation_z')
        msg.orientation.w = self._get_field(
            fields, 'qw', 'quat_w', 'quaternion_w', 'orientation_w', default=1.0)

        msg.angular_velocity.x = self._get_field(
            fields, 'gx', 'gyro_x', 'gyro_x', 'angular_velocity_x')
        msg.angular_velocity.y = self._get_field(
            fields, 'gy', 'gyro_y', 'gyro_y', 'angular_velocity_y')
        msg.angular_velocity.z = self._get_field(
            fields, 'gz', 'gyro_z', 'gyro_z', 'angular_velocity_z')

        msg.linear_acceleration.x = self._get_field(
            fields, 'ax', 'acc_x', 'acceleration_x', 'linear_acceleration_x')
        msg.linear_acceleration.y = self._get_field(
            fields, 'ay', 'acc_y', 'acceleration_y', 'linear_acceleration_y')
        msg.linear_acceleration.z = self._get_field(
            fields, 'az', 'acc_z', 'acceleration_z', 'linear_acceleration_z')

        return msg

    def _make_navsatfix_msg(self, fields: dict, stamp: Time) -> NavSatFix:
        """gps.csv / vrs_gps.csv 한 행으로 sensor_msgs/NavSatFix 메시지를 생성한다.

        KAIST: timestamp, latitude, longitude, altitude, 9-tuple covariance
        """
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = 'gps'
        msg.latitude = self._get_field(fields, 'lat', 'latitude')
        msg.longitude = self._get_field(fields, 'lon', 'longitude')
        msg.altitude = self._get_field(fields, 'alt', 'altitude', 'altitude_m')

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        cov = fields.get('cov', [0] * 9)
        for i in range(min(9, len(cov))):
            msg.position_covariance[i] = float(cov[i])
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        return msg

    def _make_stereo_msg_from_ndarray(self, img: np.ndarray, stamp: Time, frame_id: str = 'stereo_left') -> Image | None:
        """이미 디코딩된 ndarray에서 sensor_msgs/Image 메시지를 생성한다 (병렬 디코딩용)."""
        if img is None or img.size == 0:
            return None
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = 'bayer_bggr8'
        msg.is_bigendian = False
        msg.step = int(img.strides[0])
        msg.data = img.tobytes()
        return msg

    def _make_stereo_msg(self, img_path: str, stamp: Time, frame_id: str = 'stereo_left') -> Image | None:
        """스테레오 .png 파일에서 sensor_msgs/Image 메시지를 생성한다.

        encoding: bayer_bggr8 (원본 Bayer 패턴 유지)
        """
        if cv2 is None or not os.path.exists(img_path):
            return None
        try:
            img = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
            if img is None:
                return None
        except Exception:
            return None

        return self._make_stereo_msg_from_ndarray(img, stamp, frame_id)

    def _make_laserscan_msg(self, bin_path: str, frame_id: str, stamp: Time) -> LaserScan | None:
        """SICK .bin 파일에서 sensor_msgs/LaserScan 메시지를 생성한다.

        .bin 형식: float range, float intensity 반복 (kaist2bag-main/sick_converter.cpp 기준)
        """
        if not os.path.exists(bin_path):
            return None
        try:
            data = np.fromfile(bin_path, dtype=np.float32)
        except Exception:
            return None

        ranges = []
        intensities = []
        i = 0
        while i + 2 <= len(data):
            ranges.append(float(data[i]))
            intensities.append(float(data[i + 1]))
            i += 2

        if not ranges:
            return None

        n = len(ranges)
        angle_max = SICK_ANGLE_MIN + (n - 1) * SICK_ANGLE_INCREMENT

        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.angle_min = SICK_ANGLE_MIN
        msg.angle_max = angle_max
        msg.angle_increment = SICK_ANGLE_INCREMENT
        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = SICK_RANGE_MIN
        msg.range_max = SICK_RANGE_MAX
        msg.ranges = ranges
        msg.intensities = intensities
        return msg

    # ──────────────────────────────────────────────────────────────
    # TF 헬퍼 - _build_static_tf, _make_dynamic_tf
    # ──────────────────────────────────────────────────────────────

    def _build_static_tf(self, calibration_dir: str, stamp: Time) -> TFMessage:
        """calibration/Vehicle2*.txt 파일들로 /tf_static TFMessage를 생성한다.

        base_link → imu_link → left_velodyne / right_velodyne / back_sick / middle_sick / stereo
        """
        if TFMessage is None:
            return None

        transforms = []

        # base_link → imu_link (Vehicle2IMU)
        imu_file = os.path.join(calibration_dir, 'Vehicle2IMU.txt')
        R_imu, T_imu = self._parse_vehicle2sensor_calib(imu_file)
        if R_imu is not None and T_imu is not None:
            qx, qy, qz, qw = self._rotation_matrix_to_quaternion(R_imu)
            t = TransformStamped()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'imu_link'
            t.transform.translation.x = float(T_imu[0])
            t.transform.translation.y = float(T_imu[1])
            t.transform.translation.z = float(T_imu[2])
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            transforms.append(t)
        else:
            # identity
            t = TransformStamped()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'imu_link'
            t.transform.rotation.w = 1.0
            transforms.append(t)

        # base_link → left_velodyne (Vehicle2LeftVLP)
        vlp_left_file = os.path.join(calibration_dir, 'Vehicle2LeftVLP.txt')
        R_left, T_left = self._parse_vehicle2sensor_calib(vlp_left_file)
        if R_left is not None and T_left is not None:
            qx, qy, qz, qw = self._rotation_matrix_to_quaternion(R_left)
            t = TransformStamped()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'left_velodyne'
            t.transform.translation.x = float(T_left[0])
            t.transform.translation.y = float(T_left[1])
            t.transform.translation.z = float(T_left[2])
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            transforms.append(t)

        # base_link → right_velodyne (Vehicle2RightVLP)
        vlp_right_file = os.path.join(calibration_dir, 'Vehicle2RightVLP.txt')
        R_right, T_right = self._parse_vehicle2sensor_calib(vlp_right_file)
        if R_right is not None and T_right is not None:
            qx, qy, qz, qw = self._rotation_matrix_to_quaternion(R_right)
            t = TransformStamped()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'right_velodyne'
            t.transform.translation.x = float(T_right[0])
            t.transform.translation.y = float(T_right[1])
            t.transform.translation.z = float(T_right[2])
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            transforms.append(t)

        # base_link → back_sick (SICK_back)
        sick_back_file = os.path.join(calibration_dir, 'Vehicle2BackSick.txt')
        if not os.path.exists(sick_back_file):
            sick_back_file = os.path.join(calibration_dir, 'Vehicle2SICK_back.txt')
        if not os.path.exists(sick_back_file):
            sick_back_file = os.path.join(calibration_dir, 'Vehicle2SICK.txt')
        R_sick, T_sick = self._parse_vehicle2sensor_calib(sick_back_file)
        if R_sick is not None and T_sick is not None:
            qx, qy, qz, qw = self._rotation_matrix_to_quaternion(R_sick)
            t = TransformStamped()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'back_sick'
            t.transform.translation.x = float(T_sick[0])
            t.transform.translation.y = float(T_sick[1])
            t.transform.translation.z = float(T_sick[2])
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            transforms.append(t)

        # base_link → middle_sick (SICK_middle)
        sick_middle_file = os.path.join(calibration_dir, 'Vehicle2MiddleSick.txt')
        if not os.path.exists(sick_middle_file):
            sick_middle_file = os.path.join(calibration_dir, 'Vehicle2SICK_middle.txt')
        if os.path.exists(sick_middle_file):
            R_mid, T_mid = self._parse_vehicle2sensor_calib(sick_middle_file)
            if R_mid is not None and T_mid is not None:
                qx, qy, qz, qw = self._rotation_matrix_to_quaternion(R_mid)
                t = TransformStamped()
                t.header.frame_id = 'base_link'
                t.child_frame_id = 'middle_sick'
                t.transform.translation.x = float(T_mid[0])
                t.transform.translation.y = float(T_mid[1])
                t.transform.translation.z = float(T_mid[2])
                t.transform.rotation.x = qx
                t.transform.rotation.y = qy
                t.transform.rotation.z = qz
                t.transform.rotation.w = qw
                transforms.append(t)

        tf_msg = TFMessage()
        tf_msg.transforms = transforms
        return tf_msg

    def _make_dynamic_tf(self, R: np.ndarray, T: np.ndarray, stamp: Time) -> TFMessage:
        """global_pose R,T로 world → base_link dynamic TF를 생성한다."""
        if TFMessage is None:
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
