"""Save Map 이름 검증·존재 확인, 노드 stderr 로그 조회 (Task 2.10 백엔드)."""
from types import SimpleNamespace

import pytest

web_server = pytest.importorskip('ros_slam_webui.web_server')


@pytest.mark.parametrize('name', ['parking05', 'b2_run-3', 'A' * 64])
def test_valid_names(name):
    assert web_server.validate_map_dir_name(name) is None


@pytest.mark.parametrize('name', ['', 'Map', '..', 'a/b', 'a b', 'x;rm', 'A' * 65, '../etc', None, 'map\n'])
def test_invalid_names(name):
    assert web_server.validate_map_dir_name(name)


def test_tail_text_file(tmp_path):
    log = tmp_path / 'x.log'
    log.write_text('\n'.join(f'line{i}' for i in range(100)))
    assert web_server.tail_text_file(str(log), 3) == ['line97', 'line98', 'line99']
    assert web_server.tail_text_file(str(tmp_path / 'none.log'), 3) == []


def test_tail_text_file_zero_lines(tmp_path):
    log = tmp_path / 'x.log'
    log.write_text('a\nb\n')
    assert web_server.tail_text_file(str(log), 0) == []


@pytest.mark.parametrize('raw, expected', [
    (None, 50), ('', 50), ('abc', 50), ('50', 50), ('1', 1), ('0', 1), ('-5', 1),
    ('200', 200), ('201', 200), ('99999', 200),
])
def test_clamp_log_lines(raw, expected):
    assert web_server.clamp_log_lines(raw) == expected


def test_save_map_check_valid_and_missing(tmp_path):
    result = web_server.get_save_map_check('parking05', tmp_path)
    assert result == {'success': True, 'data': {'valid': True, 'reason': '', 'exists': False}}


def test_save_map_check_existing_dir(tmp_path):
    (tmp_path / 'parking05').mkdir()
    result = web_server.get_save_map_check('parking05', tmp_path)
    assert result['data'] == {'valid': True, 'reason': '', 'exists': True}


def test_save_map_check_reserved_name(tmp_path):
    (tmp_path / 'Map').mkdir()
    result = web_server.get_save_map_check('Map', tmp_path)
    assert result['success'] is True
    assert result['data']['valid'] is False
    assert result['data']['reason']
    assert result['data']['exists'] is False


def test_save_map_check_traversal_never_touches_fs(tmp_path):
    result = web_server.get_save_map_check('..', tmp_path)
    assert result['data']['valid'] is False
    assert result['data']['exists'] is False


def test_save_map_check_without_pgo_dir():
    result = web_server.get_save_map_check('parking05', None)
    assert result['data'] == {'valid': True, 'reason': '', 'exists': False}


def test_process_log_reads_known_name(tmp_path, monkeypatch):
    log = tmp_path / 'slam.log'
    log.write_text('\n'.join(f'l{i}' for i in range(10)))
    monkeypatch.setitem(web_server.PROCESS_LOG_PATHS, 'slam', str(log))
    result = web_server.get_process_log('slam', '2')
    assert result == {'success': True, 'data': {'lines': ['l8', 'l9']}}


def test_process_log_missing_file(tmp_path, monkeypatch):
    monkeypatch.setitem(web_server.PROCESS_LOG_PATHS, 'localization', str(tmp_path / 'none.log'))
    assert web_server.get_process_log('localization', '50') == {'success': True, 'data': {'lines': []}}


@pytest.mark.parametrize('name', ['', 'bag', '../etc/passwd', '/tmp/web_gui_slam_stderr.log', None])
def test_process_log_rejects_unknown_name(name):
    result = web_server.get_process_log(name, '50')
    assert result['success'] is False
    assert result['error']


def test_process_log_paths_keys():
    assert set(web_server.PROCESS_LOG_PATHS) == {'slam', 'localization'}


def test_save_slam_map_rejects_invalid_name_before_starting():
    node = SimpleNamespace(slam_map_saving=False, slam_last_saved_dir='prev')
    success, message = web_server.WebGUINode.save_slam_map(node, 'Map')
    assert success is False
    assert message
    assert node.slam_map_saving is False
    assert node.slam_last_saved_dir == 'prev'
