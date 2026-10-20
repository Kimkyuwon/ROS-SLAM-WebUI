"""Multi-Session 입력/출력 검증과 LTmapping 실패 감지 (Task 3.11 백엔드)."""
import io
import logging
from pathlib import Path
from types import SimpleNamespace

import pytest

web_server = pytest.importorskip('ros_slam_webui.web_server')


def _make_session(root, with_static=True):
    (root / 'Scans').mkdir(parents=True)
    (root / 'Scans' / '0.pcd').write_bytes(b'x')
    (root / 'optimized_poses.txt').write_text('0 0 0 0 0 0 0 1\n')
    (root / 'edges.txt').write_text('0 1 ' + ' '.join(['0'] * 12) + '\n')
    if with_static:
        (root / 'StaticMap.pcd').write_bytes(b'x')


def test_session_dir_ok(tmp_path):
    _make_session(tmp_path / 's1')
    assert web_server.check_session_dir(str(tmp_path / 's1')) == []


def test_session_dir_missing_static(tmp_path):
    _make_session(tmp_path / 's1', with_static=False)
    assert web_server.check_session_dir(str(tmp_path / 's1')) == ['StaticMap.pcd']


def test_session_dir_empty_scans_and_not_a_dir(tmp_path):
    (tmp_path / 'e' / 'Scans').mkdir(parents=True)
    assert web_server.check_session_dir(str(tmp_path / 'e')) == [
        'Scans', 'optimized_poses.txt', 'edges.txt', 'StaticMap.pcd']
    assert len(web_server.check_session_dir(str(tmp_path / 'nope'))) == 4
    assert len(web_server.check_session_dir('')) == 4


@pytest.mark.parametrize('name', ['', '.', '..', 'a/b', 'Campus1', None])
def test_bad_output_names(name):
    assert web_server.validate_output_name(name, '/x/Campus1', '/x/Campus2')


def test_same_as_map2_rejected_even_with_trailing_slash():
    assert web_server.validate_output_name('Campus2', '/x/Campus1', '/x/Campus2/')


def test_good_output_name():
    assert web_server.validate_output_name('Merged_01', '/x/Campus1', '/x/Campus2') is None


def test_outputs_missing(tmp_path):
    (tmp_path / 'optimized_poses.txt').write_text('')
    assert set(web_server.check_multi_outputs(str(tmp_path))) == {
        'optimized_poses.txt', 'FirstMap.pcd', 'SecondMap.pcd'}


def test_outputs_ok(tmp_path):
    for name in ('optimized_poses.txt', 'FirstMap.pcd', 'SecondMap.pcd'):
        (tmp_path / name).write_bytes(b'x')
    assert web_server.check_multi_outputs(str(tmp_path)) == []


@pytest.mark.parametrize('line,expected', [
    ('[LTmapping-1] Error: Could not open pose file: /x/p.txt', 'Error: Could not open pose file: /x/p.txt'),
    ('Failed to load first map poses!', 'Failed to load first map poses!'),
    ('[LTmapping-1] Failed to load second map edges!', 'Failed to load second map edges!'),
    ('[LTmapping]: Place Recognition Complete.', None),
    ('some Error: in the middle', None),
])
def test_extract_ltmapping_error(line, expected):
    assert web_server.extract_ltmapping_error(line) == expected


def test_extract_loop_count():
    line = '[LTmapping]: Loop Edge Generation Complete. size : 37'
    assert web_server.extract_loop_count(line) == 37
    assert web_server.extract_loop_count('[LTmapping]: Map Merging Complete.') is None


def _fake_node(output='out1', lt_dir=None, monkeypatch=None):
    node = SimpleNamespace(
        slam_output=output, slam_status='', slam_opt_running=True,
        slam_opt_status={'running': True, 'done': False, 'success': None, 'message': 'Running optimization...'},
        slam_optimization_process=None, _OPT_MSG_WHITELIST=web_server.WebGUINode._OPT_MSG_WHITELIST,
        get_logger=lambda: logging.getLogger('test'))
    node._set_opt_success = lambda proc, last_error='': web_server.WebGUINode._set_opt_success(
        node, proc, last_error)
    return node


def _run_reader(node, lines, returncode=0):
    proc = SimpleNamespace(stdout=io.StringIO(''.join(line + '\n' for line in lines)),
                           wait=lambda: None, returncode=returncode)
    node.slam_optimization_process = proc
    web_server.WebGUINode._read_optimization_output(node, proc)
    return proc


@pytest.fixture
def lt_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(web_server, '_find_sibling_package_dir', lambda name: tmp_path)
    return tmp_path


def _write_outputs(lt_dir, name='out1'):
    out = lt_dir / name
    out.mkdir()
    for n in ('optimized_poses.txt', 'FirstMap.pcd', 'SecondMap.pcd'):
        (out / n).write_bytes(b'x')


def test_node_died_marks_failure_with_last_error(lt_dir):
    node = _fake_node()
    _run_reader(node, [
        '[LTmapping-1] Failed to load first map poses!',
        '[ERROR] [launch]: process[LTmapping-1]: process has died [pid 1, exit code 1, cmd ...].',
    ], returncode=1)
    st = node.slam_opt_status
    assert st['done'] and st['success'] is False and st['running'] is False
    assert 'LTmapping 노드가 비정상 종료했습니다' in st['message']
    assert 'Failed to load first map poses!' in st['message']
    assert node.slam_opt_running is False


def test_success_requires_output_files(lt_dir):
    node = _fake_node()
    _run_reader(node, ['[LTmapping]: Long Term SLAM Complete.'])
    st = node.slam_opt_status
    assert st['success'] is False
    assert st['message'].startswith('결과 파일 없음: ')
    assert 'FirstMap.pcd' in st['message']


def test_success_with_output_files_and_loop_count(lt_dir):
    _write_outputs(lt_dir)
    node = _fake_node()
    _run_reader(node, [
        '[LTmapping]: Loop Edge Generation Complete. size : 12',
        '[LTmapping]: Long Term SLAM Complete.',
    ])
    st = node.slam_opt_status
    assert st['success'] is True and st['done'] is True
    assert st['loop_count'] == 12


def test_loop_count_visible_while_running(lt_dir):
    node = _fake_node()
    seen = []
    lines = iter(['[LTmapping]: Loop Edge Generation Complete. size : 0\n'])

    def readline():
        line = next(lines, '')
        if not line:
            seen.append(dict(node.slam_opt_status))
        return line

    proc = SimpleNamespace(stdout=SimpleNamespace(readline=readline), wait=lambda: None, returncode=None)
    node.slam_optimization_process = proc
    web_server.WebGUINode._read_optimization_output(node, proc)
    assert seen[0]['loop_count'] == 0 and seen[0]['running'] is True


def test_stale_process_does_not_touch_status(lt_dir):
    node = _fake_node()
    proc = SimpleNamespace(stdout=io.StringIO('[launch]: process[LTmapping-1]: process has died [pid 1]\n'),
                           wait=lambda: None, returncode=1)
    node.slam_optimization_process = object()
    before = dict(node.slam_opt_status)
    web_server.WebGUINode._read_optimization_output(node, proc)
    assert node.slam_opt_status == before


def test_optimization_status_adds_outputs_ok_and_loop_count(lt_dir):
    node = SimpleNamespace(slam_output='out1', slam_opt_status={
        'running': False, 'done': True, 'success': True, 'message': 'Long Term SLAM Complete.', 'loop_count': 5})
    st = web_server.WebGUINode.get_optimization_status(node)
    assert st['outputs_ok'] is False and st['loop_count'] == 5
    _write_outputs(lt_dir)
    assert web_server.WebGUINode.get_optimization_status(node)['outputs_ok'] is True
    node.slam_opt_status = {'running': True, 'done': False, 'success': None, 'message': ''}
    st = web_server.WebGUINode.get_optimization_status(node)
    assert st['outputs_ok'] is False and st['loop_count'] is None
