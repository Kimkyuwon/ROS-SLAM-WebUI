#!/usr/bin/env python3
"""
kitti_converter.py
KITTI Raw Dataset → ROS2 bag (.db3) 변환 모듈

기능:
  - KittiConverter.scan_directory():     KITTI 데이터셋 디렉토리 구조 탐색
  - KittiConverter.convert_to_ros2bag(): KITTI 데이터를 ROS2 bag 파일로 변환

출력 토픽:
  /kitti/oxts/imu                     sensor_msgs/msg/Imu
  /kitti/oxts/gps/fix                 sensor_msgs/msg/NavSatFix
  /kitti/velo/pointcloud              sensor_msgs/msg/PointCloud2
  /kitti/camera_gray_left/image_raw   sensor_msgs/msg/Image  (mono8)
  /kitti/camera_color_left/image_raw  sensor_msgs/msg/Image  (bgr8, 있을 경우)
  /tf                                 tf2_msgs/msg/TFMessage (dynamic: world→base_link)
  /tf_static                          tf2_msgs/msg/TFMessage (static: base_link→imu→velo→cam)
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
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import NavSatFix, NavSatStatus
    from tf2_msgs.msg import TFMessage
    _ROSBAG2_AVAILABLE = True
except ImportError:
    rosbag2_py = None          # type: ignore
    TopicMetadata = None       # type: ignore
    serialize_message = None   # type: ignore
    TransformStamped = None    # type: ignore
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
        cam2_dir = os.path.join(data_path, 'image_02', 'data')
        cam0_files = sorted(Path(cam0_dir).glob('*.png')) if os.path.isdir(cam0_dir) else []
        cam2_files = sorted(Path(cam2_dir).glob('*.png')) if os.path.isdir(cam2_dir) else []
        has_cam2 = len(cam2_files) > 0

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
            ('/kitti/oxts/imu',                      'sensor_msgs/msg/Imu'),
            ('/kitti/oxts/gps/fix',                  'sensor_msgs/msg/NavSatFix'),
            ('/kitti/velo/pointcloud',               'sensor_msgs/msg/PointCloud2'),
            ('/kitti/camera_gray_left/image_raw',    'sensor_msgs/msg/Image'),
            ('/tf',                                  'tf2_msgs/msg/TFMessage'),
            ('/tf_static',                           'tf2_msgs/msg/TFMessage'),
        ]
        if has_cam2:
            topics.append(('/kitti/camera_color_left/image_raw', 'sensor_msgs/msg/Image'))

        for idx, (topic_name, topic_type) in enumerate(topics):
            writer.create_topic(TopicMetadata(
                id=idx,
                name=topic_name,
                type=topic_type,
                serialization_format='cdr',
            ))

        # ── 8. Static TF 계산 (calib 행렬에서) ───────────────────────
        _progress(3, 'Computing static TF transforms...')
        static_tf_msg = self._build_static_tf(calib_imu_to_velo, calib_velo_to_cam)

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
        n_cam2 = len(cam2_files)
        total = max(n_oxts + n_velo + n_cam0 + n_cam2, 1)
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
            _progress(70, 'Converting gray camera images...')
            # cam0 타임스탬프: OXTS 타임스탬프를 공유 (같은 개수인 경우)
            cam0_ts = (oxts_timestamps[:len(cam0_files)]
                       if len(oxts_timestamps) >= len(cam0_files)
                       else list(range(len(cam0_files))))
            for ts_ns, img_file in zip(cam0_ts, cam0_files):
                stamp_msg = self._ns_to_time_msg(ts_ns)
                img_msg = self._make_image_msg(str(img_file), 'mono8', stamp_msg)
                if img_msg:
                    writer.write(
                        '/kitti/camera_gray_left/image_raw',
                        serialize_message(img_msg),
                        ts_ns,
                    )
                _tick('Converting gray camera images...')

        # ── 14. Camera 2 (color left, 있을 경우) ─────────────────────
        if has_cam2:
            _progress(85, 'Converting color camera images...')
            cam2_ts = (oxts_timestamps[:len(cam2_files)]
                       if len(oxts_timestamps) >= len(cam2_files)
                       else list(range(len(cam2_files))))
            for ts_ns, img_file in zip(cam2_ts, cam2_files):
                stamp_msg = self._ns_to_time_msg(ts_ns)
                img_msg = self._make_image_msg(str(img_file), 'bgr8', stamp_msg)
                if img_msg:
                    writer.write(
                        '/kitti/camera_color_left/image_raw',
                        serialize_message(img_msg),
                        ts_ns,
                    )
                _tick('Converting color camera images...')

        del writer
        _progress(100, 'Conversion complete!')

    # ──────────────────────────────────────────────────────────────
    # Private helpers - calib 파일 파싱
    # ──────────────────────────────────────────────────────────────

    def _parse_calib_file(self, filepath: str) -> dict:
        """KITTI calibration 텍스트 파일을 파싱하여 딕셔너리로 반환한다.

        각 행은 'key: value [value ...]' 형식이며,
        숫자 값은 float 또는 float 리스트로 변환된다.
        """
        result = {}
        if not os.path.exists(filepath):
            return result
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if ':' not in line:
                    continue
                key, val_str = line.split(':', 1)
                key = key.strip()
                vals = val_str.strip().split()
                if not vals:
                    result[key] = val_str.strip()
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
        return result

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
    # Private helpers - TF 변환
    # ──────────────────────────────────────────────────────────────

    def _build_static_tf(
        self,
        calib_imu_to_velo: dict,
        calib_velo_to_cam: dict,
    ) -> TFMessage:
        """calib 행렬에서 Static TF 메시지를 생성한다.

        변환 체인:
          base_link → imu_link      (identity)
          imu_link  → velo_link     (calib_imu_to_velo)
          velo_link → camera_gray_left_link (calib_velo_to_cam)
        """
        transforms = []

        # base_link → imu_link (identity transform)
        t0 = TransformStamped()
        t0.header.frame_id = 'base_link'
        t0.child_frame_id = 'imu_link'
        t0.transform.rotation.w = 1.0
        transforms.append(t0)

        # imu_link → velo_link (from calib_imu_to_velo: R(9), T(3))
        if 'R' in calib_imu_to_velo and 'T' in calib_imu_to_velo:
            R = np.array(calib_imu_to_velo['R']).reshape(3, 3)
            T = np.array(calib_imu_to_velo['T'])
            qx, qy, qz, qw = self._rotation_matrix_to_quaternion(R)
            t1 = TransformStamped()
            t1.header.frame_id = 'imu_link'
            t1.child_frame_id = 'velo_link'
            t1.transform.translation.x = float(T[0])
            t1.transform.translation.y = float(T[1])
            t1.transform.translation.z = float(T[2])
            t1.transform.rotation.x = qx
            t1.transform.rotation.y = qy
            t1.transform.rotation.z = qz
            t1.transform.rotation.w = qw
            transforms.append(t1)

        # velo_link → camera_gray_left_link (from calib_velo_to_cam: R(9), T(3))
        if 'R' in calib_velo_to_cam and 'T' in calib_velo_to_cam:
            R = np.array(calib_velo_to_cam['R']).reshape(3, 3)
            T = np.array(calib_velo_to_cam['T'])
            qx, qy, qz, qw = self._rotation_matrix_to_quaternion(R)
            t2 = TransformStamped()
            t2.header.frame_id = 'velo_link'
            t2.child_frame_id = 'camera_gray_left_link'
            t2.transform.translation.x = float(T[0])
            t2.transform.translation.y = float(T[1])
            t2.transform.translation.z = float(T[2])
            t2.transform.rotation.x = qx
            t2.transform.rotation.y = qy
            t2.transform.rotation.z = qz
            t2.transform.rotation.w = qw
            transforms.append(t2)

        tf_msg = TFMessage()
        tf_msg.transforms = transforms
        return tf_msg

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

    def _make_image_msg(self, img_file: str, encoding: str, stamp: Time):
        """PNG 파일에서 sensor_msgs/Image 메시지를 생성한다.

        Args:
            img_file: PNG 파일 경로
            encoding: 'mono8' (gray camera) 또는 'bgr8' (color camera)
            stamp:    타임스탬프
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
