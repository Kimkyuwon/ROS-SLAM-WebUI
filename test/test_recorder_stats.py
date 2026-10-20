"""Recorder 상태의 경과 시간·파일 크기 (Task 3.8 백엔드)."""
import time
from types import SimpleNamespace

import pytest

web_server = pytest.importorskip('ros_slam_webui.web_server')


def test_path_size_file_and_dir(tmp_path):
    f = tmp_path / 'a.bag'
    f.write_bytes(b'x' * 10)
    d = tmp_path / 'b_bag'
    d.mkdir()
    (d / 'b_0.mcap').write_bytes(b'y' * 7)
    (d / 'metadata.yaml').write_bytes(b'z' * 3)
    assert web_server.path_size_bytes(str(f)) == 10
    assert web_server.path_size_bytes(str(d)) == 10
    assert web_server.path_size_bytes(str(tmp_path / 'none')) == 0
    assert web_server.path_size_bytes('') == 0


def _fake_node(**kw):
    state = dict(recorder_bag_name='b', recorder_recording=False, recorder_mode='ros2',
                 recorder_started_at=None, recorder_output_path='')
    state.update(kw)
    return SimpleNamespace(**state)


def test_state_idle_has_zero_stats():
    node = _fake_node()
    st = web_server.WebGUINode.get_recorder_state(node)
    assert st['elapsed_sec'] == 0.0
    assert st['size_bytes'] == 0


def test_state_recording_reports_elapsed_and_size(tmp_path):
    out = tmp_path / 'rec.bag'
    out.write_bytes(b'x' * 42)
    node = _fake_node(recorder_recording=True, recorder_started_at=time.time() - 5.0,
                      recorder_output_path=str(out))
    st = web_server.WebGUINode.get_recorder_state(node)
    assert 4.9 <= st['elapsed_sec'] < 7.0
    assert st['size_bytes'] == 42
    assert st['recording'] is True
