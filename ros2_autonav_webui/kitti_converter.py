#!/usr/bin/env python3
"""
kitti_converter.py
KITTI Raw Dataset → ROS2 bag (.db3) 변환 모듈

기능:
  - KittiConverter.scan_directory():     KITTI 데이터셋 디렉토리 구조 탐색
  - KittiConverter.convert_to_ros2bag(): KITTI 데이터를 ROS2 bag 파일로 변환

출력 토픽:
  /kitti/oxts/imu                          sensor_msgs/msg/Imu
  /kitti/oxts/gps/fix                      sensor_msgs/msg/NavSatFix
  /kitti/oxts/gps/vel                      geometry_msgs/msg/TwistStamped
  /kitti/velo/pointcloud                   sensor_msgs/msg/PointCloud2
  /kitti/camera_gray_left/image_raw        sensor_msgs/msg/Image      (mono8)
  /kitti/camera_gray_left/camera_info      sensor_msgs/msg/CameraInfo
  /kitti/camera_gray_right/image_raw       sensor_msgs/msg/Image      (mono8, 있을 경우)
  /kitti/camera_gray_right/camera_info     sensor_msgs/msg/CameraInfo (있을 경우)
  /kitti/camera_color_left/image_raw       sensor_msgs/msg/Image      (bgr8, 있을 경우)
  /kitti/camera_color_left/camera_info     sensor_msgs/msg/CameraInfo (있을 경우)
  /kitti/camera_color_right/image_raw      sensor_msgs/msg/Image      (bgr8, 있을 경우)
  /kitti/camera_color_right/camera_info    sensor_msgs/msg/CameraInfo (있을 경우)
  /tf                                      tf2_msgs/msg/TFMessage     (dynamic: world→base_link)
  /tf_static                               tf2_msgs/msg/TFMessage     (static: base_link→imu→velo→cam)
"""

import math
import os
import re
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField

# 변환(bag 쓰기) 전용 의존성 - 없어도 직접 재생(scan/play)은 정상 동작
try:
    import rosbag2_py
    from rosbag2_py import TopicMetadata
    from rclpy.serialization import serialize_message
    from geometry_msgs.msg import TransformStamped, TwistStamped
    from sensor_msgs.msg import CameraInfo, NavSatFix, NavSatStatus
    from tf2_msgs.msg import TFMessage
    _ROSBAG2_AVAILABLE = True
except ImportError:
    rosbag2_py = None          # type: ignore
    TopicMetadata = None       # type: ignore
    serialize_message = None   # type: ignore
    TransformStamped = None    # type: ignore
    TwistStamped = None        # type: ignore
    CameraInfo = None          # type: ignore
    NavSatFix = None           # type: ignore
    NavSatStatus = None        # type: ignore
    TFMessage = None           # type: ignore
    _ROSBAG2_AVAILABLE = False

# 지구 반경 (Mercator 투영용)
_EARTH_RADIUS = 6_378_137.0  # meters


class KittiConverter:
    """KITTI Raw Dataset을 ROS2 bag 파일로 변환하는 클래스.

    Usage:
        converter = KittiConverter()
        result = converter.scan_directory('/path/to/2011_09_30')
        converter.convert_to_ros2bag(
            calib_dir=result['calib_dir'],
            data_path=result['drive_dirs'][0]['data_path'],
            output_bag_path='/path/to/2011_09_30/converted/drive_0034_sync',
            progress_cb=lambda pct, msg: print(f'{pct}% - {msg}')
        )
    """

    # ──────────────────────────────────────────────────────────────
    # Public API
    # ──────────────────────────────────────────────────────────────

    def scan_directory(self, base_dir: str) -> dict:
        """KITTI 데이터셋 최상위 디렉토리를 탐색하여 calib/drive 정보를 반환한다.

        Args:
            base_dir: 사용자가 선택한 날짜 디렉토리 (예: /path/to/2011_09_30)

        Returns:
            dict 형식:
            {
              'date': '2011_09_30',
              'calib_dir': '/path/.../2011_09_30',    # calib 파일 실제 위치
              'drive_dirs': [
                {
                  'name': '2011_09_30_drive_0034_sync',
                  'drive_type': 'sync',   # or 'extract'
                  'drive_id': '0034',
                  'data_path': '/path/.../2011_09_30_drive_0034_sync/2011_09_30/2011_09_30_drive_0034_sync'
                }, ...
              ]
            }
        """
        base_dir = os.path.abspath(base_dir)
        date = os.path.basename(base_dir)

        calib_dir = None
        drive_dirs = []

        for entry in sorted(os.listdir(base_dir)):
            full_path = os.path.join(base_dir, entry)
            if not os.path.isdir(full_path):
                continue

            # calib 디렉토리 탐지: 패턴 *_calib
            if re.search(r'_calib$', entry):
                inner = os.path.join(full_path, date)
                calib_dir = inner if os.path.isdir(inner) else full_path
                continue

            # drive 디렉토리 탐지: 패턴 *_drive_\d+_(sync|extract)
            m = re.search(r'_drive_(\d+)_(sync|extract)$', entry)
            if m:
                drive_id = m.group(1)
                drive_type = m.group(2)
                # 실제 센서 데이터 경로: {drive_top}/{date}/{drive_name}
                data_path = os.path.join(full_path, date, entry)
                if not os.path.isdir(data_path):
                    data_path = full_path  # fallback
                drive_dirs.append({
                    'name': entry,
                    'drive_type': drive_type,
                    'drive_id': drive_id,
                    'data_path': data_path,
                    # calib_dir 는 나중에 루프가 끝난 뒤 후처리로 채운다
                    'calib_dir': None,
                })

        # drive entry 에 calib_dir 채우기 (같은 날짜 디렉토리에서 발견된 calib 사용)
        for d in drive_dirs:
            if d.get('calib_dir') is None:
                d['calib_dir'] = calib_dir

        # drive 가 하나도 없으면 날짜 패턴(YYYY_MM_DD) 서브디렉토리를 재귀 탐색.
        # 사용자가 /kitti/ (root) 을 선택했을 때도 /kitti/2011_09_30/ 등을 자동으로 찾는다.
        if not drive_dirs:
            for entry in sorted(os.listdir(base_dir)):
                full_path = os.path.join(base_dir, entry)
                if not os.path.isdir(full_path):
                    continue
                if re.match(r'^\d{4}_\d{2}_\d{2}$', entry):  # 날짜 디렉토리 패턴
                    sub = self.scan_directory(full_path)
                    if sub['drive_dirs']:
                        drive_dirs.extend(sub['drive_dirs'])
                        if calib_dir is None:
                            calib_dir = sub.get('calib_dir')

        return {
            'date': date,
            'calib_dir': calib_dir,
            'drive_dirs': drive_dirs,
        }

    def convert_to_ros2bag(
        self,
        calib_dir: str,
        data_path: str,
        output_bag_path: str,
        progress_cb=None,
    ) -> None:
        """KITTI 데이터를 ROS2 bag (.db3) 파일로 변환한다.

        Args:
            calib_dir:        calib 파일 디렉토리
                              (calib_cam_to_cam.txt, calib_imu_to_velo.txt, calib_velo_to_cam.txt 포함)
            data_path:        drive 데이터 디렉토리
                              (image_00/, velodyne_points/, oxts/ 등 포함)
            output_bag_path:  출력 ROS2 bag 디렉토리 경로 (확장자 없음)
                              예: '/path/to/2011_09_30/converted/drive_0034_sync'
            progress_cb:      진행률 콜백 (선택)
                              signature: progress_cb(progress: int, message: str)
        """
        # 단조증가 보장: 한 번 전송한 퍼센트보다 작은 값은 무시
        _sent_max = [-1]

        def _progress(pct: int, msg: str):
            if progress_cb and pct > _sent_max[0]:
                _sent_max[0] = pct
                progress_cb(pct, msg)

        # ── 1. calib 파일 파싱 ──────────────────────────────────────
        _progress(0, 'Parsing calibration files...')
        calib_imu_to_velo = self._parse_calib_file(
            os.path.join(calib_dir, 'calib_imu_to_velo.txt'))
        calib_velo_to_cam = self._parse_calib_file(
            os.path.join(calib_dir, 'calib_velo_to_cam.txt'))
        calib_cam_to_cam = self._parse_calib_file(
            os.path.join(calib_dir, 'calib_cam_to_cam.txt'))

        # ── 2. OXTS 파일 목록 및 타임스탬프 ─────────────────────────
        oxts_dir = os.path.join(data_path, 'oxts')
        oxts_timestamps = self._load_timestamps(os.path.join(oxts_dir, 'timestamps.txt'))
        oxts_files = sorted(Path(os.path.join(oxts_dir, 'data')).glob('*.txt')) \
            if os.path.isdir(os.path.join(oxts_dir, 'data')) else []

        # ── 3. Velodyne 파일 목록 및 타임스탬프 ──────────────────────
        velo_dir = os.path.join(data_path, 'velodyne_points')
        velo_timestamps = self._load_timestamps(os.path.join(velo_dir, 'timestamps.txt'))
        velo_files = sorted(Path(os.path.join(velo_dir, 'data')).glob('*.bin')) \
            if os.path.isdir(os.path.join(velo_dir, 'data')) else []

        # ── 4. Camera 이미지 파일 목록 ───────────────────────────────
        cam0_dir = os.path.join(data_path, 'image_00', 'data')
        cam1_dir = os.path.join(data_path, 'image_01', 'data')
        cam2_dir = os.path.join(data_path, 'image_02', 'data')
        cam3_dir = os.path.join(data_path, 'image_03', 'data')
        cam0_files = sorted(Path(cam0_dir).glob('*.png')) if os.path.isdir(cam0_dir) else []
        cam1_files = sorted(Path(cam1_dir).glob('*.png')) if os.path.isdir(cam1_dir) else []
        cam2_files = sorted(Path(cam2_dir).glob('*.png')) if os.path.isdir(cam2_dir) else []
        cam3_files = sorted(Path(cam3_dir).glob('*.png')) if os.path.isdir(cam3_dir) else []
        has_cam1 = len(cam1_files) > 0
        has_cam2 = len(cam2_files) > 0
        has_cam3 = len(cam3_files) > 0

        # ── 5. 출력 경로 준비 ────────────────────────────────────────────
        # rosbag2_py.SequentialWriter 가 bag 디렉토리를 스스로 생성하므로
        # 이미 존재하는 경우 먼저 삭제하고, 부모 디렉토리만 미리 만든다.
        import shutil
        if os.path.exists(output_bag_path):
            shutil.rmtree(output_bag_path)
        os.makedirs(os.path.dirname(output_bag_path), exist_ok=True)

        # ── 6. rosbag2 Writer 초기화 ─────────────────────────────────
        _progress(2, 'Initializing bag writer...')
        writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(uri=output_bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        )
        writer.open(storage_options, converter_options)

        # ── 7. 토픽 등록 ─────────────────────────────────────────────
        topics = [
            ('/kitti/oxts/imu',                           'sensor_msgs/msg/Imu'),
            ('/kitti/oxts/gps/fix',                       'sensor_msgs/msg/NavSatFix'),
            ('/kitti/oxts/gps/vel',                       'geometry_msgs/msg/TwistStamped'),
            ('/kitti/velo/pointcloud',                    'sensor_msgs/msg/PointCloud2'),
            ('/kitti/camera_gray_left/image_raw',         'sensor_msgs/msg/Image'),
            ('/kitti/camera_gray_left/camera_info',       'sensor_msgs/msg/CameraInfo'),
            ('/tf',                                       'tf2_msgs/msg/TFMessage'),
            ('/tf_static',                                'tf2_msgs/msg/TFMessage'),
        ]
        if has_cam1:
            topics.append(('/kitti/camera_gray_right/image_raw',   'sensor_msgs/msg/Image'))
            topics.append(('/kitti/camera_gray_right/camera_info', 'sensor_msgs/msg/CameraInfo'))
        if has_cam2:
            topics.append(('/kitti/camera_color_left/image_raw',   'sensor_msgs/msg/Image'))
            topics.append(('/kitti/camera_color_left/camera_info', 'sensor_msgs/msg/CameraInfo'))
        if has_cam3:
            topics.append(('/kitti/camera_color_right/image_raw',   'sensor_msgs/msg/Image'))
            topics.append(('/kitti/camera_color_right/camera_info', 'sensor_msgs/msg/CameraInfo'))

        for idx, (topic_name, topic_type) in enumerate(topics):
            writer.create_topic(TopicMetadata(
                id=idx,
                name=topic_name,
                type=topic_type,
                serialization_format='cdr',
            ))

        # ── 8. Static TF 계산 (calib 행렬에서) ───────────────────────
        _progress(3, 'Computing static TF transforms...')
        first_stamp = self._ns_to_time_msg(oxts_timestamps[0]) if oxts_timestamps else None
        static_tf_msg = self._build_static_tf(
            calib_imu_to_velo, calib_velo_to_cam,
            calib_cam_to_cam=calib_cam_to_cam,
            stamp=first_stamp,
        )

        # ── 9. Mercator 투영 원점 설정 ───────────────────────────────
        origin_oxts = None
        mercator_scale = None
        if oxts_files:
            first_oxts = self._load_oxts_file(str(oxts_files[0]))
            if first_oxts:
                origin_oxts = first_oxts
                mercator_scale = math.cos(math.radians(first_oxts[0]))

        # ── 10. 진행률 추적 ──────────────────────────────────────────
        n_oxts = min(len(oxts_timestamps), len(oxts_files))
        n_velo = min(len(velo_timestamps), len(velo_files))
        n_cam0 = len(cam0_files)
        n_cam1 = len(cam1_files)
        n_cam2 = len(cam2_files)
        n_cam3 = len(cam3_files)
        total = max(n_oxts + n_velo + n_cam0 + n_cam1 + n_cam2 + n_cam3, 1)
        processed = 0

        def _tick(message: str):
            nonlocal processed
            processed += 1
            pct = min(3 + int(processed / total * 95), 98)
            # _progress 내부에서 단조증가 보장 + 1% 단위 throttle
            _progress(pct, message)

        # ── 11. OXTS → IMU + NavSatFix + Dynamic TF ──────────────────
        _progress(3, 'Converting OXTS data...')
        static_tf_written = False
        for i, (ts_ns, oxts_file) in enumerate(zip(oxts_timestamps, oxts_files)):
            oxts = self._load_oxts_file(str(oxts_file))
            if oxts is None:
                _tick('Converting OXTS data...')
                continue

            stamp_msg = self._ns_to_time_msg(ts_ns)

            # static TF는 첫 번째 OXTS 타임스탬프에 한 번만 기록
            if not static_tf_written:
                writer.write('/tf_static', serialize_message(static_tf_msg), ts_ns)
                static_tf_written = True

            # IMU 메시지
            imu_msg = self._make_imu_msg(oxts, stamp_msg)
            writer.write('/kitti/oxts/imu', serialize_message(imu_msg), ts_ns)

            # NavSatFix 메시지
            gps_msg = self._make_navsatfix_msg(oxts, stamp_msg)
            writer.write('/kitti/oxts/gps/fix', serialize_message(gps_msg), ts_ns)

            # TwistStamped 속도 메시지
            vel_msg = self._make_twist_stamped_msg(oxts, stamp_msg)
            writer.write('/kitti/oxts/gps/vel', serialize_message(vel_msg), ts_ns)

            # Dynamic TF: world → base_link
            if origin_oxts is not None and mercator_scale is not None:
                tf_msg = self._make_dynamic_tf(oxts, origin_oxts, mercator_scale, stamp_msg)
                writer.write('/tf', serialize_message(tf_msg), ts_ns)

            _tick('Converting OXTS data...')

        # ── 12. Velodyne PointCloud2 ─────────────────────────────────
        _progress(50, 'Converting velodyne data...')
        for ts_ns, velo_file in zip(velo_timestamps, velo_files):
            stamp_msg = self._ns_to_time_msg(ts_ns)
            pc2_msg = self._make_pointcloud2_msg(str(velo_file), stamp_msg)
            if pc2_msg:
                writer.write('/kitti/velo/pointcloud', serialize_message(pc2_msg), ts_ns)
            _tick('Converting velodyne data...')

        # ── 13. Camera 0 (gray left) ─────────────────────────────────
        if cam0_files:
            _progress(70, 'Converting gray left camera images...')
            # cam0 타임스탬프: OXTS 타임스탬프를 공유 (같은 개수인 경우)
            cam0_ts = (oxts_timestamps[:len(cam0_files)]
                       if len(oxts_timestamps) >= len(cam0_files)
                       else list(range(len(cam0_files))))
            for ts_ns, img_file in zip(cam0_ts, cam0_files):
                stamp_msg = self._ns_to_time_msg(ts_ns)
                img_msg = self._make_image_msg(
                    str(img_file), 'mono8', stamp_msg,
                    frame_id='camera_gray_left_link')
                if img_msg:
                    writer.write(
                        '/kitti/camera_gray_left/image_raw',
                        serialize_message(img_msg),
                        ts_ns,
                    )
                caminfo_msg = self._make_camera_info_msg(calib_cam_to_cam, '00', stamp_msg)
                writer.write(
                    '/kitti/camera_gray_left/camera_info',
                    serialize_message(caminfo_msg),
                    ts_ns,
                )
                _tick('Converting gray left camera images...')

        # ── 13.5. Camera 1 (gray right, 있을 경우) ───────────────────
        if has_cam1:
            _progress(75, 'Converting gray right camera images...')
            cam1_ts = (oxts_timestamps[:len(cam1_files)]
                       if len(oxts_timestamps) >= len(cam1_files)
                       else list(range(len(cam1_files))))
            for ts_ns, img_file in zip(cam1_ts, cam1_files):
                stamp_msg = self._ns_to_time_msg(ts_ns)
                img_msg = self._make_image_msg(
                    str(img_file), 'mono8', stamp_msg,
                    frame_id='camera_gray_right_link')
                if img_msg:
                    writer.write(
                        '/kitti/camera_gray_right/image_raw',
                        serialize_message(img_msg),
                        ts_ns,
                    )
                caminfo_msg = self._make_camera_info_msg(calib_cam_to_cam, '01', stamp_msg)
                writer.write(
                    '/kitti/camera_gray_right/camera_info',
                    serialize_message(caminfo_msg),
                    ts_ns,
                )
                _tick('Converting gray right camera images...')

        # ── 14. Camera 2 (color left, 있을 경우) ─────────────────────
        if has_cam2:
            _progress(85, 'Converting color left camera images...')
            cam2_ts = (oxts_timestamps[:len(cam2_files)]
                       if len(oxts_timestamps) >= len(cam2_files)
                       else list(range(len(cam2_files))))
            for ts_ns, img_file in zip(cam2_ts, cam2_files):
                stamp_msg = self._ns_to_time_msg(ts_ns)
                img_msg = self._make_image_msg(
                    str(img_file), 'bgr8', stamp_msg,
                    frame_id='camera_color_left_link')
                if img_msg:
                    writer.write(
                        '/kitti/camera_color_left/image_raw',
                        serialize_message(img_msg),
                        ts_ns,
                    )
                caminfo_msg = self._make_camera_info_msg(calib_cam_to_cam, '02', stamp_msg)
                writer.write(
                    '/kitti/camera_color_left/camera_info',
                    serialize_message(caminfo_msg),
                    ts_ns,
                )
                _tick('Converting color left camera images...')

        # ── 14.5. Camera 3 (color right, 있을 경우) ──────────────────
        if has_cam3:
            _progress(92, 'Converting color right camera images...')
            cam3_ts = (oxts_timestamps[:len(cam3_files)]
                       if len(oxts_timestamps) >= len(cam3_files)
                       else list(range(len(cam3_files))))
            for ts_ns, img_file in zip(cam3_ts, cam3_files):
                stamp_msg = self._ns_to_time_msg(ts_ns)
                img_msg = self._make_image_msg(
                    str(img_file), 'bgr8', stamp_msg,
                    frame_id='camera_color_right_link')
                if img_msg:
                    writer.write(
                        '/kitti/camera_color_right/image_raw',
                        serialize_message(img_msg),
                        ts_ns,
                    )
                caminfo_msg = self._make_camera_info_msg(calib_cam_to_cam, '03', stamp_msg)
                writer.write(
                    '/kitti/camera_color_right/camera_info',
                    serialize_message(caminfo_msg),
                    ts_ns,
                )
                _tick('Converting color right camera images...')

        del writer
        _progress(100, 'Conversion complete!')

    def convert_to_ros1bag(
        self,
        calib_dir: str,
        data_path: str,
        output_bag_path: str,
        progress_cb=None,
    ) -> None:
        """KITTI 데이터를 ROS1 .bag 파일로 직접 변환한다 (rosbags 라이브러리 사용).

        rosbags.rosbag1.Writer + migrate_bytes()를 통해 ROS2 CDR 직렬화 후
        즉시 ROS1 raw bytes로 변환하여 .bag에 기록한다.
        중간 ROS2 bag 파일을 생성하지 않아 공간/시간 절약.

        Args:
            calib_dir:        calib 파일 디렉토리
            data_path:        drive 데이터 디렉토리
            output_bag_path:  출력 ROS1 .bag 파일 경로 (예: '/path/to/drive_name.bag')
            progress_cb:      진행률 콜백 (선택), signature: (progress: int, message: str)
        """
        try:
            from rosbags.rosbag1 import Writer as Ros1Writer
            from rosbags.typesys import get_typestore, Stores
            from rosbags.convert.converter import migrate_bytes as _migrate_bytes
        except ImportError as e:
            raise RuntimeError(
                f'rosbags 라이브러리가 필요합니다. 설치: pip install rosbags\n원인: {e}'
            )

        # 단조증가 보장
        _sent_max = [-1]

        def _progress(pct: int, msg: str):
            if progress_cb and pct > _sent_max[0]:
                _sent_max[0] = pct
                progress_cb(pct, msg)

        src_typestore = get_typestore(Stores.ROS2_JAZZY)
        dst_typestore = get_typestore(Stores.ROS1_NOETIC)
        migrate_cache: dict = {}

        # ── 1. calib 파일 파싱 ──────────────────────────────────────
        _progress(0, 'Parsing calibration files...')
        calib_imu_to_velo = self._parse_calib_file(
            os.path.join(calib_dir, 'calib_imu_to_velo.txt'))
        calib_velo_to_cam = self._parse_calib_file(
            os.path.join(calib_dir, 'calib_velo_to_cam.txt'))
        calib_cam_to_cam = self._parse_calib_file(
            os.path.join(calib_dir, 'calib_cam_to_cam.txt'))

        # ── 2. OXTS ─────────────────────────────────────────────────
        oxts_dir = os.path.join(data_path, 'oxts')
        oxts_timestamps = self._load_timestamps(os.path.join(oxts_dir, 'timestamps.txt'))
        oxts_files = sorted(Path(os.path.join(oxts_dir, 'data')).glob('*.txt')) \
            if os.path.isdir(os.path.join(oxts_dir, 'data')) else []

        # ── 3. Velodyne ─────────────────────────────────────────────
        velo_dir = os.path.join(data_path, 'velodyne_points')
        velo_timestamps = self._load_timestamps(os.path.join(velo_dir, 'timestamps.txt'))
        velo_files = sorted(Path(os.path.join(velo_dir, 'data')).glob('*.bin')) \
            if os.path.isdir(os.path.join(velo_dir, 'data')) else []

        # ── 4. Camera 파일 목록 ──────────────────────────────────────
        cam0_dir = os.path.join(data_path, 'image_00', 'data')
        cam1_dir = os.path.join(data_path, 'image_01', 'data')
        cam2_dir = os.path.join(data_path, 'image_02', 'data')
        cam3_dir = os.path.join(data_path, 'image_03', 'data')
        cam0_files = sorted(Path(cam0_dir).glob('*.png')) if os.path.isdir(cam0_dir) else []
        cam1_files = sorted(Path(cam1_dir).glob('*.png')) if os.path.isdir(cam1_dir) else []
        cam2_files = sorted(Path(cam2_dir).glob('*.png')) if os.path.isdir(cam2_dir) else []
        cam3_files = sorted(Path(cam3_dir).glob('*.png')) if os.path.isdir(cam3_dir) else []
        has_cam1 = len(cam1_files) > 0
        has_cam2 = len(cam2_files) > 0
        has_cam3 = len(cam3_files) > 0

        # ── 5. 출력 경로 준비 ────────────────────────────────────────
        import shutil as _shutil
        if os.path.isfile(output_bag_path):
            os.remove(output_bag_path)
        parent_dir = os.path.dirname(output_bag_path)
        if parent_dir:
            os.makedirs(parent_dir, exist_ok=True)

        # ── 6. 토픽 목록 ────────────────────────────────────────────
        topics = [
            ('/kitti/oxts/imu',                           'sensor_msgs/msg/Imu'),
            ('/kitti/oxts/gps/fix',                       'sensor_msgs/msg/NavSatFix'),
            ('/kitti/oxts/gps/vel',                       'geometry_msgs/msg/TwistStamped'),
            ('/kitti/velo/pointcloud',                    'sensor_msgs/msg/PointCloud2'),
            ('/kitti/camera_gray_left/image_raw',         'sensor_msgs/msg/Image'),
            ('/kitti/camera_gray_left/camera_info',       'sensor_msgs/msg/CameraInfo'),
            ('/tf',                                       'tf2_msgs/msg/TFMessage'),
            ('/tf_static',                                'tf2_msgs/msg/TFMessage'),
        ]
        if has_cam1:
            topics.append(('/kitti/camera_gray_right/image_raw',   'sensor_msgs/msg/Image'))
            topics.append(('/kitti/camera_gray_right/camera_info', 'sensor_msgs/msg/CameraInfo'))
        if has_cam2:
            topics.append(('/kitti/camera_color_left/image_raw',   'sensor_msgs/msg/Image'))
            topics.append(('/kitti/camera_color_left/camera_info', 'sensor_msgs/msg/CameraInfo'))
        if has_cam3:
            topics.append(('/kitti/camera_color_right/image_raw',   'sensor_msgs/msg/Image'))
            topics.append(('/kitti/camera_color_right/camera_info', 'sensor_msgs/msg/CameraInfo'))

        # ── 7. 진행률 추적 ───────────────────────────────────────────
        n_oxts = min(len(oxts_timestamps), len(oxts_files))
        n_velo = min(len(velo_timestamps), len(velo_files))
        n_cam0 = len(cam0_files)
        n_cam1 = len(cam1_files)
        n_cam2 = len(cam2_files)
        n_cam3 = len(cam3_files)
        total = max(n_oxts + n_velo + n_cam0 + n_cam1 + n_cam2 + n_cam3, 1)
        processed = 0

        def _tick(message: str):
            nonlocal processed
            processed += 1
            pct = min(3 + int(processed / total * 95), 98)
            _progress(pct, message)

        # ── 8. Static TF 계산 ────────────────────────────────────────
        _progress(2, 'Initializing ROS1 bag writer...')
        first_stamp = self._ns_to_time_msg(oxts_timestamps[0]) if oxts_timestamps else None
        static_tf_msg = self._build_static_tf(
            calib_imu_to_velo, calib_velo_to_cam,
            calib_cam_to_cam=calib_cam_to_cam,
            stamp=first_stamp,
        )

        # ── 9. Mercator 투영 원점 ────────────────────────────────────
        origin_oxts = None
        mercator_scale = None
        if oxts_files:
            first_oxts = self._load_oxts_file(str(oxts_files[0]))
            if first_oxts:
                origin_oxts = first_oxts
                mercator_scale = math.cos(math.radians(first_oxts[0]))

        # ── 10. ROS1 bag Writer 열기 및 토픽 등록 ────────────────────
        def _ensure_type(ros2_type: str) -> bool:
            """dst_typestore에 타입이 없으면 src_typestore에서 등록 시도. 성공 시 True."""
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
            """ROS2 CDR bytes → ROS1 raw bytes 변환."""
            return bytes(_migrate_bytes(
                src_typestore, dst_typestore,
                conn.msgtype, conn.msgtype,
                migrate_cache, cdr_bytes,
                src_is2=True, dst_is2=False,
            ))

        _progress(3, 'Converting OXTS data...')
        with Ros1Writer(output_bag_path) as writer:
            # 커넥션 등록 (토픽별 1회)
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

            # ── 11. OXTS → IMU + NavSatFix + TwistStamped + Dynamic TF ──
            static_tf_written = False
            for ts_ns, oxts_file in zip(oxts_timestamps, oxts_files):
                oxts = self._load_oxts_file(str(oxts_file))
                if oxts is None:
                    _tick('Converting OXTS data...')
                    continue
                stamp_msg = self._ns_to_time_msg(ts_ns)

                if not static_tf_written:
                    _write('/tf_static', static_tf_msg, ts_ns)
                    static_tf_written = True

                _write('/kitti/oxts/imu',     self._make_imu_msg(oxts, stamp_msg),           ts_ns)
                _write('/kitti/oxts/gps/fix', self._make_navsatfix_msg(oxts, stamp_msg),      ts_ns)
                _write('/kitti/oxts/gps/vel', self._make_twist_stamped_msg(oxts, stamp_msg),  ts_ns)

                if origin_oxts is not None and mercator_scale is not None:
                    _write('/tf', self._make_dynamic_tf(
                        oxts, origin_oxts, mercator_scale, stamp_msg), ts_ns)

                _tick('Converting OXTS data...')

            # ── 12. Velodyne PointCloud2 ─────────────────────────────
            _progress(50, 'Converting velodyne data...')
            for ts_ns, velo_file in zip(velo_timestamps, velo_files):
                stamp_msg = self._ns_to_time_msg(ts_ns)
                pc2_msg = self._make_pointcloud2_msg(str(velo_file), stamp_msg)
                if pc2_msg:
                    _write('/kitti/velo/pointcloud', pc2_msg, ts_ns)
                _tick('Converting velodyne data...')

            # ── 13. Camera 0 (gray left) ─────────────────────────────
            if cam0_files:
                _progress(70, 'Converting gray left camera images...')
                cam0_ts = (oxts_timestamps[:len(cam0_files)]
                           if len(oxts_timestamps) >= len(cam0_files)
                           else list(range(len(cam0_files))))
                for ts_ns, img_file in zip(cam0_ts, cam0_files):
                    stamp_msg = self._ns_to_time_msg(ts_ns)
                    img_msg = self._make_image_msg(
                        str(img_file), 'mono8', stamp_msg, frame_id='camera_gray_left_link')
                    if img_msg:
                        _write('/kitti/camera_gray_left/image_raw', img_msg, ts_ns)
                    _write('/kitti/camera_gray_left/camera_info',
                           self._make_camera_info_msg(calib_cam_to_cam, '00', stamp_msg), ts_ns)
                    _tick('Converting gray left camera images...')

            # ── 13.5. Camera 1 (gray right) ──────────────────────────
            if has_cam1:
                _progress(75, 'Converting gray right camera images...')
                cam1_ts = (oxts_timestamps[:len(cam1_files)]
                           if len(oxts_timestamps) >= len(cam1_files)
                           else list(range(len(cam1_files))))
                for ts_ns, img_file in zip(cam1_ts, cam1_files):
                    stamp_msg = self._ns_to_time_msg(ts_ns)
                    img_msg = self._make_image_msg(
                        str(img_file), 'mono8', stamp_msg, frame_id='camera_gray_right_link')
                    if img_msg:
                        _write('/kitti/camera_gray_right/image_raw', img_msg, ts_ns)
                    _write('/kitti/camera_gray_right/camera_info',
                           self._make_camera_info_msg(calib_cam_to_cam, '01', stamp_msg), ts_ns)
                    _tick('Converting gray right camera images...')

            # ── 14. Camera 2 (color left) ────────────────────────────
            if has_cam2:
                _progress(85, 'Converting color left camera images...')
                cam2_ts = (oxts_timestamps[:len(cam2_files)]
                           if len(oxts_timestamps) >= len(cam2_files)
                           else list(range(len(cam2_files))))
                for ts_ns, img_file in zip(cam2_ts, cam2_files):
                    stamp_msg = self._ns_to_time_msg(ts_ns)
                    img_msg = self._make_image_msg(
                        str(img_file), 'bgr8', stamp_msg, frame_id='camera_color_left_link')
                    if img_msg:
                        _write('/kitti/camera_color_left/image_raw', img_msg, ts_ns)
                    _write('/kitti/camera_color_left/camera_info',
                           self._make_camera_info_msg(calib_cam_to_cam, '02', stamp_msg), ts_ns)
                    _tick('Converting color left camera images...')

            # ── 14.5. Camera 3 (color right) ─────────────────────────
            if has_cam3:
                _progress(92, 'Converting color right camera images...')
                cam3_ts = (oxts_timestamps[:len(cam3_files)]
                           if len(oxts_timestamps) >= len(cam3_files)
                           else list(range(len(cam3_files))))
                for ts_ns, img_file in zip(cam3_ts, cam3_files):
                    stamp_msg = self._ns_to_time_msg(ts_ns)
                    img_msg = self._make_image_msg(
                        str(img_file), 'bgr8', stamp_msg, frame_id='camera_color_right_link')
                    if img_msg:
                        _write('/kitti/camera_color_right/image_raw', img_msg, ts_ns)
                    _write('/kitti/camera_color_right/camera_info',
                           self._make_camera_info_msg(calib_cam_to_cam, '03', stamp_msg), ts_ns)
                    _tick('Converting color right camera images...')

        _progress(100, 'ROS1 bag conversion complete!')

    # ──────────────────────────────────────────────────────────────
    # Private helpers - calib 파일 파싱
    # ──────────────────────────────────────────────────────────────

    def _parse_calib_file(self, filepath: str) -> dict:
        """KITTI calibration 텍스트 파일을 파싱하여 딕셔너리로 반환한다.

        지원 형식:
          - 단일 라인: 'key: v1 v2 v3 ...'
          - 다중 라인: 'Tr_xxx:' 다음 3줄에 4값씩 (KITTI 표준 3x4 행렬)
        """
        result = {}
        if not os.path.exists(filepath):
            return result
        with open(filepath, 'r') as f:
            lines = [ln.strip() for ln in f if ln.strip()]
        i = 0
        while i < len(lines):
            line = lines[i]
            if ':' not in line:
                i += 1
                continue
            key, val_str = line.split(':', 1)
            key = key.strip()
            vals = val_str.strip().split()
            # 다중 라인: Tr_* 또는 R, T 등 키 다음 줄들이 값 연속인 경우
            if len(vals) < 9 and i + 1 < len(lines):
                next_ln = lines[i + 1]
                if ':' not in next_ln and next_ln:
                    # 다음 줄에 숫자만 있으면 현재 키에 추가
                    while i + 1 < len(lines):
                        next_ln = lines[i + 1]
                        if ':' in next_ln:
                            break
                        extra = next_ln.split()
                        if all(self._is_float(s) for s in extra):
                            vals.extend(extra)
                            i += 1
                        else:
                            break
            if not vals:
                result[key] = val_str.strip() if val_str.strip() else []
            elif len(vals) == 1:
                try:
                    result[key] = float(vals[0])
                except ValueError:
                    result[key] = vals[0]
            else:
                try:
                    result[key] = [float(v) for v in vals]
                except ValueError:
                    result[key] = vals
            i += 1
        return result

    def _is_float(self, s: str) -> bool:
        try:
            float(s)
            return True
        except (ValueError, TypeError):
            return False

    # ──────────────────────────────────────────────────────────────
    # Private helpers - 타임스탬프
    # ──────────────────────────────────────────────────────────────

    def _load_timestamps(self, filepath: str) -> list:
        """KITTI timestamps.txt 파일을 파싱하여 나노초 정수 리스트로 반환한다."""
        timestamps = []
        if not os.path.exists(filepath):
            return timestamps
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                ts_ns = self._parse_kitti_timestamp(line)
                timestamps.append(ts_ns)
        return timestamps

    def _parse_kitti_timestamp(self, ts_str: str) -> int:
        """KITTI 타임스탬프 문자열을 나노초 정수로 변환한다.

        입력 형식: '2011-09-26 13:02:25.5808822600'
        반환값: 나노초 정수 (예: 1317038545580882260)
        """
        ts_str = ts_str.strip()
        try:
            if '.' in ts_str:
                base, frac = ts_str.split('.', 1)
            else:
                base = ts_str
                frac = '0'
            dt = datetime.strptime(base, '%Y-%m-%d %H:%M:%S')
            # 소수 부분을 9자리(나노초)로 맞춤
            frac_ns_str = frac.ljust(9, '0')[:9]
            extra_ns = int(frac_ns_str)
            epoch_s = int(dt.replace(tzinfo=timezone.utc).timestamp())
            return epoch_s * 1_000_000_000 + extra_ns
        except Exception:
            return 0

    def _ns_to_time_msg(self, ns: int) -> Time:
        """나노초 정수를 builtin_interfaces/Time 메시지로 변환한다."""
        msg = Time()
        msg.sec = int(ns // 1_000_000_000)
        msg.nanosec = int(ns % 1_000_000_000)
        return msg

    # ──────────────────────────────────────────────────────────────
    # Private helpers - OXTS 데이터
    # ──────────────────────────────────────────────────────────────

    def _load_oxts_file(self, filepath: str):
        """OXTS .txt 파일을 파싱하여 float 리스트로 반환한다.

        OXTS 형식 (30개 값):
          [0]  lat    위도 (deg)
          [1]  lon    경도 (deg)
          [2]  alt    고도 (m)
          [3]  roll   롤 (rad)
          [4]  pitch  피치 (rad)
          [5]  yaw    요 (rad)
          [6]  vn     북쪽 속도 (m/s)
          [7]  ve     동쪽 속도 (m/s)
          [11] ax     IMU x축 가속도 (m/s^2)
          [12] ay     IMU y축 가속도 (m/s^2)
          [13] az     IMU z축 가속도 (m/s^2)
          [17] wx     IMU x축 각속도 (rad/s)
          [18] wy     IMU y축 각속도 (rad/s)
          [19] wz     IMU z축 각속도 (rad/s)
          [23] pos_accuracy 위치 정확도 (m)
        """
        try:
            with open(filepath, 'r') as f:
                vals = [float(v) for v in f.read().strip().split()]
            return vals if len(vals) >= 6 else None
        except Exception:
            return None

    def _make_imu_msg(self, oxts: list, stamp: Time) -> Imu:
        """OXTS 데이터로 sensor_msgs/Imu 메시지를 생성한다."""
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = 'imu_link'

        # orientation: roll, pitch, yaw → 회전 행렬 → quaternion
        roll, pitch, yaw = oxts[3], oxts[4], oxts[5]
        R = self._rpy_to_rotation_matrix(roll, pitch, yaw)
        qx, qy, qz, qw = self._rotation_matrix_to_quaternion(R)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        # angular_velocity (IMU 프레임): wx, wy, wz
        if len(oxts) > 19:
            msg.angular_velocity.x = oxts[17]
            msg.angular_velocity.y = oxts[18]
            msg.angular_velocity.z = oxts[19]

        # linear_acceleration (IMU 프레임): ax, ay, az
        if len(oxts) > 13:
            msg.linear_acceleration.x = oxts[11]
            msg.linear_acceleration.y = oxts[12]
            msg.linear_acceleration.z = oxts[13]

        return msg

    def _make_navsatfix_msg(self, oxts: list, stamp: Time) -> NavSatFix:
        """OXTS 데이터로 sensor_msgs/NavSatFix 메시지를 생성한다."""
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = 'gps_link'
        msg.latitude = oxts[0]
        msg.longitude = oxts[1]
        msg.altitude = oxts[2]

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        # 위치 공분산 (대각 행렬)
        if len(oxts) > 23:
            pos_acc = oxts[23]
            msg.position_covariance[0] = pos_acc ** 2       # north
            msg.position_covariance[4] = pos_acc ** 2       # east
            msg.position_covariance[8] = (pos_acc * 3.0) ** 2  # down (낮은 정확도)
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        return msg

    # ──────────────────────────────────────────────────────────────
    # Private helpers - calib R/T 추출 (다양한 KITTI 형식 지원)
    # ──────────────────────────────────────────────────────────────

    def _extract_rt_from_calib(self, calib: dict, tr_key: str = None,
                               r00_key: str = None) -> tuple:
        """calib 딕셔너리에서 R(3x3), T(3x1)를 추출한다.

        지원 형식:
          - R: 9 values, T: 3 values (표준 KITTI)
          - R_00: 9 values, T_00: 3 values (calib_velo_to_cam 대체)
          - Tr_*: 12 values (3x4 행렬, row-major [R|T])
        """
        R, T = None, None

        # 형식 1: R, T 별도 키 (R/T 또는 R_00/T_00)
        pairs = [('R', 'T')]
        if r00_key and len(r00_key) >= 4:
            pairs.append((r00_key, f'T_{r00_key[-2:]}'))
        for rk, tk in pairs:
            if rk in calib and tk in calib:
                r_vals = calib[rk]
                t_vals = calib[tk]
                if isinstance(r_vals, (list, tuple)) and len(r_vals) >= 9:
                    R = np.array(r_vals, dtype=np.float64).reshape(3, 3)
                if isinstance(t_vals, (list, tuple)) and len(t_vals) >= 3:
                    T = np.array(t_vals[:3], dtype=np.float64)
                if R is not None and T is not None:
                    break

        # 형식 2: Tr_* 3x4 행렬 (row-major)
        if R is None or T is None:
            keys_to_try = [tr_key] if tr_key else ['Tr_imu_to_velo', 'Tr_velo_to_cam']
            for key in keys_to_try:
                if not key or not key.startswith('Tr_') or key not in calib:
                    continue
                vals = calib[key]
                if isinstance(vals, (list, tuple)) and len(vals) >= 12:
                    arr = np.array(vals[:12], dtype=np.float64).reshape(3, 4)
                    R = arr[:, :3]
                    T = arr[:, 3]
                    break

        return (R, T)

    def _invert_rt(self, R: np.ndarray, T: np.ndarray) -> tuple:
        """KITTI calib (A→B: p_B = R*p_A + T)를 ROS TF (child→parent)용으로 역변환.

        ROS TF: p_parent = R_inv * p_child + t_inv
        반환: (R^T, -R^T @ T)
        """
        R_inv = R.T
        T_inv = -R_inv @ np.array(T).flatten()
        return R_inv, T_inv

    # ──────────────────────────────────────────────────────────────
    # Private helpers - TF 변환
    # ──────────────────────────────────────────────────────────────

    def _build_static_tf(
        self,
        calib_imu_to_velo: dict,
        calib_velo_to_cam: dict,
        calib_cam_to_cam: dict = None,
        stamp: Time = None,
    ) -> TFMessage:
        """calib 행렬에서 Static TF 메시지를 생성한다.

        변환 체인:
          base_link → imu_link      (identity)
          imu_link  → velo_link     (calib_imu_to_velo)
          velo_link → camera_*_link  (calib_velo_to_cam + calib_cam_to_cam)

        Args:
            calib_imu_to_velo: calib_imu_to_velo.txt 파싱 결과
            calib_velo_to_cam: calib_velo_to_cam.txt 파싱 결과
            calib_cam_to_cam: calib_cam_to_cam.txt 파싱 결과 (카메라 1,2,3용, 선택)
            stamp: TF header stamp (None이면 0,0)
        """
        transforms = []
        if stamp is None:
            stamp = Time()
            stamp.sec = 0
            stamp.nanosec = 0

        # base_link → imu_link (identity transform)
        t0 = TransformStamped()
        if stamp:
            t0.header.stamp = stamp
        t0.header.frame_id = 'base_link'
        t0.child_frame_id = 'imu_link'
        t0.transform.rotation.w = 1.0
        transforms.append(t0)

        # imu_link → velo_link (calib_imu_to_velo: imu→velo, ROS TF는 child→parent이므로 역변환)
        R_imu, T_imu = self._extract_rt_from_calib(calib_imu_to_velo, 'Tr_imu_to_velo')
        if R_imu is not None and T_imu is not None:
            R_tf, T_tf = self._invert_rt(R_imu, T_imu)
            qx, qy, qz, qw = self._rotation_matrix_to_quaternion(R_tf)
            t1 = TransformStamped()
            if stamp:
                t1.header.stamp = stamp
            t1.header.frame_id = 'imu_link'
            t1.child_frame_id = 'velo_link'
            t1.transform.translation.x = float(T_tf[0])
            t1.transform.translation.y = float(T_tf[1])
            t1.transform.translation.z = float(T_tf[2])
            t1.transform.rotation.x = qx
            t1.transform.rotation.y = qy
            t1.transform.rotation.z = qz
            t1.transform.rotation.w = qw
            transforms.append(t1)
        else:
            # calib 파싱 실패 시 identity fallback (velo_link 연결 유지)
            t1 = TransformStamped()
            if stamp:
                t1.header.stamp = stamp
            t1.header.frame_id = 'imu_link'
            t1.child_frame_id = 'velo_link'
            t1.transform.rotation.w = 1.0
            transforms.append(t1)

        # velo_link → camera_*_link (calib_velo_to_cam: velo→cam, ROS TF는 child→parent이므로 역변환)
        R_velo, T_velo = self._extract_rt_from_calib(
            calib_velo_to_cam, 'Tr_velo_to_cam', 'R_00')
        if R_velo is not None and T_velo is not None:
            # cam0 (gray left): velo → cam0, 역변환하여 velo_link(parent)→camera(child)
            R_tf, T_tf = self._invert_rt(R_velo, T_velo)
            self._append_velo_to_cam_tf(
                transforms, R_tf, T_tf, 'camera_gray_left_link', stamp)

            # cam1,2,3: velo → cam0 → cam_i (calib_cam_to_cam R_0i, T_0i)
            if calib_cam_to_cam:
                cam_specs = [
                    ('01', 'camera_gray_right_link'),
                    ('02', 'camera_color_left_link'),
                    ('03', 'camera_color_right_link'),
                ]
                for cam_id, frame_id in cam_specs:
                    R_0i = self._get_calib_matrix(calib_cam_to_cam, f'R_{cam_id}', 3, 3)
                    T_0i = self._get_calib_matrix(calib_cam_to_cam, f'T_{cam_id}', 3, 1)
                    if R_0i is not None and T_0i is not None:
                        # KITTI: p_cam0 = R_velo @ p_velo + T_velo,
                        #        p_cami = R_0i @ p_cam0 + T_0i
                        #   =>  p_cami = (R_0i @ R_velo) @ p_velo + (R_0i @ T_velo + T_0i)
                        # (기존 R_velo @ R_0i 는 행렬 순서가 반대라 좌우 배치가 전후로 뒤틀림)
                        t_0i = np.array(T_0i, dtype=np.float64).flatten()
                        R_combined = R_0i @ R_velo
                        T_combined = R_0i @ np.array(T_velo, dtype=np.float64).flatten() + t_0i
                        R_tf_i, T_tf_i = self._invert_rt(R_combined, T_combined)
                        self._append_velo_to_cam_tf(
                            transforms, R_tf_i, T_tf_i, frame_id, stamp)
        else:
            # calib_velo_to_cam 파싱 실패 시 velo→cam0 identity (최소 연결)
            t_cam0 = TransformStamped()
            if stamp:
                t_cam0.header.stamp = stamp
            t_cam0.header.frame_id = 'velo_link'
            t_cam0.child_frame_id = 'camera_gray_left_link'
            t_cam0.transform.rotation.w = 1.0
            transforms.append(t_cam0)

        tf_msg = TFMessage()
        tf_msg.transforms = transforms
        return tf_msg

    def _get_calib_matrix(self, calib: dict, key: str, rows: int, cols: int) -> np.ndarray:
        """calib에서 key에 해당하는 행렬을 추출한다."""
        if key not in calib:
            return None
        vals = calib[key]
        if not isinstance(vals, (list, tuple)) or len(vals) < rows * cols:
            return None
        return np.array(vals[:rows * cols], dtype=np.float64).reshape(rows, cols)

    def _append_velo_to_cam_tf(
        self,
        transforms: list,
        R: np.ndarray,
        T: np.ndarray,
        child_frame_id: str,
        stamp: Time,
    ) -> None:
        """velo_link → child_frame_id 변환을 transforms에 추가한다."""
        qx, qy, qz, qw = self._rotation_matrix_to_quaternion(R)
        t = TransformStamped()
        if stamp:
            t.header.stamp = stamp
        t.header.frame_id = 'velo_link'
        t.child_frame_id = child_frame_id
        t.transform.translation.x = float(T[0])
        t.transform.translation.y = float(T[1])
        t.transform.translation.z = float(T[2])
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        transforms.append(t)

    def _make_dynamic_tf(
        self,
        oxts: list,
        origin_oxts: list,
        scale: float,
        stamp: Time,
    ) -> TFMessage:
        """OXTS GPS/IMU 데이터로 world → base_link dynamic TF를 생성한다.

        Args:
            oxts:       현재 OXTS 데이터
            origin_oxts: 첫 번째 OXTS 데이터 (Mercator 원점)
            scale:       Mercator 투영 스케일 (cos(lat_origin_rad))
            stamp:       타임스탬프
        """
        # Mercator 좌표계에서의 상대 위치
        mx, my = self._lat_lon_to_mercator(oxts[0], oxts[1], scale)
        ox, oy = self._lat_lon_to_mercator(origin_oxts[0], origin_oxts[1], scale)
        tx = mx - ox
        ty = my - oy
        tz = oxts[2] - origin_oxts[2]

        # 방향: roll, pitch, yaw → quaternion
        roll, pitch, yaw = oxts[3], oxts[4], oxts[5]
        R = self._rpy_to_rotation_matrix(roll, pitch, yaw)
        qx, qy, qz, qw = self._rotation_matrix_to_quaternion(R)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = tz
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        tf_msg = TFMessage()
        tf_msg.transforms = [t]
        return tf_msg

    # ──────────────────────────────────────────────────────────────
    # Private helpers - Velodyne PointCloud2
    # ──────────────────────────────────────────────────────────────

    def _make_pointcloud2_msg(self, bin_file: str, stamp: Time):
        """Velodyne .bin 파일에서 sensor_msgs/PointCloud2 메시지를 생성한다.

        .bin 파일 형식: float32 × 4 per point (x, y, z, intensity)
        """
        try:
            points = np.fromfile(bin_file, dtype=np.float32).reshape(-1, 4)
        except Exception:
            return None

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = 'velo_link'
        msg.height = 1
        msg.width = points.shape[0]
        msg.is_bigendian = False
        msg.is_dense = True

        # field 정의: x, y, z, intensity (각 4바이트 float32)
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

        msg.point_step = 16      # 4개 × 4바이트
        msg.row_step = msg.point_step * msg.width
        msg.data = points.tobytes()
        return msg

    # ──────────────────────────────────────────────────────────────
    # Private helpers - 이미지
    # ──────────────────────────────────────────────────────────────

    def _make_image_msg(self, img_file: str, encoding: str, stamp: Time,
                        frame_id: str = None):
        """PNG 파일에서 sensor_msgs/Image 메시지를 생성한다.

        Args:
            img_file: PNG 파일 경로
            encoding: 'mono8' (gray camera) 또는 'bgr8' (color camera)
            stamp:    타임스탬프
            frame_id: 헤더 frame_id (None 이면 encoding 기반 기본값 사용)
        """
        try:
            if encoding == 'mono8':
                img = cv2.imread(img_file, cv2.IMREAD_GRAYSCALE)
            else:
                img = cv2.imread(img_file, cv2.IMREAD_COLOR)
            if img is None:
                return None
        except Exception:
            return None

        if frame_id is None:
            frame_id = 'camera_gray_left_link' if encoding == 'mono8' else 'camera_color_left_link'

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step = int(img.strides[0])
        msg.data = img.tobytes()
        return msg

    def _make_camera_info_msg(self, calib_cam_to_cam: dict, cam_id: str, stamp: Time):
        """calib_cam_to_cam.txt 파싱 결과로 sensor_msgs/CameraInfo 메시지를 생성한다.

        Args:
            calib_cam_to_cam: _parse_calib_file()로 파싱한 calib_cam_to_cam.txt 딕셔너리
            cam_id:           카메라 ID ('00', '01', '02', '03')
            stamp:            타임스탬프

        Returns:
            CameraInfo 메시지 (데이터 부재 시 빈 메시지 반환)
        """
        _frame_id_map = {
            '00': 'camera_gray_left_link',
            '01': 'camera_gray_right_link',
            '02': 'camera_color_left_link',
            '03': 'camera_color_right_link',
        }

        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = _frame_id_map.get(cam_id, f'camera_{cam_id}_link')

        # 이미지 크기: S_{cam_id} = [width, height]
        s_key = f'S_{cam_id}'
        if s_key in calib_cam_to_cam:
            s = calib_cam_to_cam[s_key]
            msg.width = int(s[0])
            msg.height = int(s[1])

        # 내부 파라미터 행렬 K (3×3, 9 values)
        k_key = f'K_{cam_id}'
        if k_key in calib_cam_to_cam:
            msg.k = [float(v) for v in calib_cam_to_cam[k_key]]

        # 왜곡 계수 D
        d_key = f'D_{cam_id}'
        if d_key in calib_cam_to_cam:
            msg.d = [float(v) for v in calib_cam_to_cam[d_key]]
            msg.distortion_model = 'plumb_bob'

        # 정류화 행렬 R_rect (3×3, 9 values)
        r_key = f'R_rect_{cam_id}'
        if r_key in calib_cam_to_cam:
            msg.r = [float(v) for v in calib_cam_to_cam[r_key]]

        # 프로젝션 행렬 P_rect (3×4, 12 values)
        p_key = f'P_rect_{cam_id}'
        if p_key in calib_cam_to_cam:
            msg.p = [float(v) for v in calib_cam_to_cam[p_key]]

        return msg

    def _make_twist_stamped_msg(self, oxts: list, stamp: Time):
        """OXTS 데이터로 geometry_msgs/TwistStamped 메시지를 생성한다.

        OXTS 인덱스:
          [6]  vn  북쪽 속도 (m/s)
          [7]  ve  동쪽 속도 (m/s)
          [8]  vf  전방(차체) 속도 (m/s)

        Args:
            oxts:  _load_oxts_file()로 읽은 OXTS float 리스트
            stamp: 타임스탬프

        Returns:
            TwistStamped 메시지
        """
        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'

        if len(oxts) > 8:
            msg.twist.linear.x = oxts[8]   # vf: 전방 속도 (차체 x축)
            msg.twist.linear.y = oxts[6]   # vn: 북쪽 속도
            msg.twist.linear.z = oxts[7]   # ve: 동쪽 속도

        return msg

    # ──────────────────────────────────────────────────────────────
    # Private helpers - 수학 함수
    # ──────────────────────────────────────────────────────────────

    def _lat_lon_to_mercator(self, lat: float, lon: float, scale: float) -> tuple:
        """WGS84 위도/경도를 Mercator 투영 좌표 (x, y)로 변환한다.

        Args:
            lat:   위도 (deg)
            lon:   경도 (deg)
            scale: cos(lat_origin_rad) - 원점 위도의 코사인

        Returns:
            (x, y) 좌표 (m)
        """
        mx = scale * math.radians(lon) * _EARTH_RADIUS
        my = scale * _EARTH_RADIUS * math.log(math.tan(math.radians(90.0 + lat) / 2.0))
        return mx, my

    def _rpy_to_rotation_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Roll, Pitch, Yaw (rad)를 3×3 회전 행렬로 변환한다.

        회전 순서 (intrinsic): Rz(yaw) @ Ry(pitch) @ Rx(roll)
        """
        cr, sr = math.cos(roll),  math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw),   math.sin(yaw)

        Rx = np.array([[1,  0,   0],
                       [0,  cr, -sr],
                       [0,  sr,  cr]], dtype=np.float64)
        Ry = np.array([[ cp, 0, sp],
                       [  0, 1,  0],
                       [-sp, 0, cp]], dtype=np.float64)
        Rz = np.array([[cy, -sy, 0],
                       [sy,  cy, 0],
                       [ 0,   0, 1]], dtype=np.float64)
        return Rz @ Ry @ Rx

    def _rotation_matrix_to_quaternion(self, R: np.ndarray) -> tuple:
        """3×3 회전 행렬을 quaternion (x, y, z, w)으로 변환한다.

        Shepperd's method 사용 (수치 안정성 보장).
        """
        tr = R[0, 0] + R[1, 1] + R[2, 2]
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2.0      # S = 4w
            w = 0.25 * S
            x = (R[2, 1] - R[1, 2]) / S
            y = (R[0, 2] - R[2, 0]) / S
            z = (R[1, 0] - R[0, 1]) / S
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0  # S = 4x
            w = (R[2, 1] - R[1, 2]) / S
            x = 0.25 * S
            y = (R[0, 1] + R[1, 0]) / S
            z = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0  # S = 4y
            w = (R[0, 2] - R[2, 0]) / S
            x = (R[0, 1] + R[1, 0]) / S
            y = 0.25 * S
            z = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0  # S = 4z
            w = (R[1, 0] - R[0, 1]) / S
            x = (R[0, 2] + R[2, 0]) / S
            y = (R[1, 2] + R[2, 1]) / S
            z = 0.25 * S
        return float(x), float(y), float(z), float(w)
