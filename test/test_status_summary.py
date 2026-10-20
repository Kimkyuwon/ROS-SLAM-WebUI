"""get_status_summary: 메모리 속성만 읽어 모듈별 실행 상태를 돌려준다."""
from types import SimpleNamespace

import pytest

web_server = pytest.importorskip('ros_slam_webui.web_server')


class _Proc:
    def __init__(self, alive):
        self._alive = alive

    def poll(self):
        return None if self._alive else 0


class _Ros1Thread:
    def __init__(self, status):
        self._status = status

    def is_alive(self):
        return True

    def get_status(self):
        return {'status': self._status}


def _node(**over):
    base = dict(
        slam_process=None, slam_map_saving=False, localization_process=None, slam_opt_running=False,
        ros1_player_thread=None, bag_path='', bag_playing=False, bag_paused=False,
        recorder_recording=False, recorder_mode='ros2',
        player_data_loaded=False, player_playing=False, player_paused=False,
        player_is_ros2_bag=False, player_is_ros1_bag=False,
        kitti_converter_running=False, kaist_converter_running=False,
        mulran_converter_running=False, helipr_converter_running=False, save_bag_saving=False,
        _is_alive=web_server.WebGUINode._is_alive)
    base.update(over)
    return SimpleNamespace(**base)


def _summary(**over):
    return web_server.WebGUINode.get_status_summary(_node(**over))


def test_idle():
    r = _summary()
    assert r['success'] is True
    assert r['data']['slam'] == {'running': False, 'saving_map': False}
    assert r['data']['bag_player'] == {'loaded': False, 'playing': False, 'paused': False}


def test_slam_running_and_saving():
    r = _summary(slam_process=_Proc(True), slam_map_saving=True)
    assert r['data']['slam'] == {'running': True, 'saving_map': True}


def test_dead_process_is_not_running():
    assert _summary(localization_process=_Proc(False))['data']['localization']['running'] is False


def test_ros1_bag_playing_counts_as_playing():
    r = _summary(bag_path='/x.bag', ros1_player_thread=_Ros1Thread('playing'))
    assert r['data']['bag_player'] == {'loaded': True, 'playing': True, 'paused': False}


def test_file_player_converting():
    assert _summary(mulran_converter_running=True)['data']['file_player']['converting'] is True
