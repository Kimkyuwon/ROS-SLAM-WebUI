"""LiDAR SLAM·Localization 헤더 액션·상태 배지·단계 표시줄 (API는 page.route로 흉내, 실제 노드 실행 안 함)."""
import json

from ui_check_common import finish, go_screen, open_page, shot

GO_BG = 'rgb(63, 208, 122)'


def current_step_text(page, stepper_id):
    return page.evaluate(
        f"""() => {{
            const li = document.querySelector('#{stepper_id} [aria-current="step"]');
            return li ? li.lastChild.textContent.trim() : null;
        }}""")


def badge_text(page, badge_id):
    return page.locator(f'#{badge_id} .ui-status-text').inner_text().strip()


def check_slam_ready(page, failures):
    go_screen(page, 'lidar-slam-subtab')
    page.wait_for_timeout(500)
    start = page.locator('#lidar-slam-header-actions #slam-start-button')
    if not start.count():
        failures.append('slam: #slam-start-button not in header actions')
        return
    if not start.is_enabled():
        failures.append('slam: start disabled in Ready')
    for btn_id in ('slam-stop-button', 'slam-save-map-button'):
        btn = page.locator(f'#lidar-slam-header-actions #{btn_id}')
        if not btn.count() or btn.is_enabled():
            failures.append(f'slam: {btn_id} missing or enabled in Ready')
    step = current_step_text(page, 'lidar-slam-stepper')
    if step != 'Config':
        failures.append(f'slam: current step {step!r} != Config')
    bg = start.evaluate('el => getComputedStyle(el).backgroundColor')
    if bg != GO_BG:
        failures.append(f'slam: start background {bg} != {GO_BG}')
    if badge_text(page, 'lidar-slam-header-badge') != 'Ready':
        failures.append('slam: badge not Ready')
    save_title = page.locator('#slam-save-map-button').get_attribute('title') or ''
    if 'SLAM 실행 중에만' not in save_title:
        failures.append(f'slam: save map lock title {save_title!r}')
    for btn_id in ('slam-start-button', 'slam-stop-button', 'slam-save-map-button'):
        if not page.locator(f'#{btn_id} .ui-icon use').count() or not page.locator(f'#{btn_id} .ui-btn-label').count():
            failures.append(f'slam: {btn_id} icon/label missing')
    shot(page, 'p2-slam-ready')


def check_slam_start_fail(page, failures):
    """데이터 없이 Start → Starting… → 실패 응답 → Ready 복귀."""
    held = []
    page.on('dialog', lambda d: d.dismiss())
    page.route('**/api/slam/start_mapping', lambda route: held.append(route))
    page.evaluate('() => { startSlamMapping(); }')
    page.wait_for_timeout(500)
    text = badge_text(page, 'lidar-slam-header-badge')
    if text != 'Starting…':
        failures.append(f'slam: badge after Start {text!r} != Starting…')
    if page.locator('#slam-start-button').is_enabled():
        failures.append('slam: start enabled while Starting')
    empty_title = page.locator('#lidar-slam-viewer-empty .ui-viewer-empty-title').text_content().strip()
    if empty_title != 'rosbridge 연결 대기 중…':
        failures.append(f'slam: empty title while Starting {empty_title!r}')
    shot(page, 'p2-slam-starting')
    if not held:
        failures.append('slam: start_mapping request not intercepted')
        return
    held[0].fulfill(status=200, content_type='application/json',
                    body=json.dumps({'success': False, 'message': 'mock failure'}))
    page.wait_for_timeout(1000)
    text = badge_text(page, 'lidar-slam-header-badge')
    if text != 'Ready':
        failures.append(f'slam: badge after start failure {text!r} != Ready')
    if not page.locator('#slam-start-button').is_enabled():
        failures.append('slam: start not re-enabled after failure')
    page.unroute('**/api/slam/start_mapping')
    shot(page, 'p2-slam-start-failed')


def check_slam_running(page, failures):
    """is_running=true 상태를 흉내 → Running 배지, Stop 활성, KF 0이면 Save Map 잠금."""
    def fulfill_state(route):
        resp = route.fetch()
        data = resp.json()
        data['is_running'] = True
        route.fulfill(response=resp, body=json.dumps(data))

    page.route('**/api/slam/state', fulfill_state)
    page.wait_for_timeout(3000)
    text = badge_text(page, 'lidar-slam-header-badge')
    if not text.startswith('Running'):
        failures.append(f'slam: running badge {text!r}')
    if not page.locator('#slam-stop-button').is_enabled():
        failures.append('slam: stop disabled while Running')
    save = page.locator('#slam-save-map-button')
    if save.is_enabled() or '키프레임' not in (save.get_attribute('title') or ''):
        failures.append('slam: save map not locked with keyframe reason (kf none)')
    if current_step_text(page, 'lidar-slam-stepper') != 'Mapping':
        failures.append('slam: running step != Mapping')
    page.evaluate("document.getElementById('ad-kf_count').textContent = '12'; uiShell.refresh();")
    if not save.is_enabled():
        failures.append('slam: save map not enabled with kf_count=12')
    shot(page, 'p2-slam-running')
    # 진행 중인 fetch 핸들러가 끝날 때까지 기다리고 푼다 (다음 검사가 같은 URL을 다시 가로챔)
    page.unroute_all(behavior='wait')
    page.evaluate("document.getElementById('ad-kf_count').textContent = '—'")


def check_slam_legacy_nav(page, failures):
    """옛 탭 모드(legacyNav=1)에서도 헤더 액션 버튼이 보이고 누를 수 있어야 한다."""
    page.evaluate("localStorage.setItem('webui.legacyNav', '1')")
    try:
        page.reload(wait_until='load')
        page.wait_for_selector('#slam-tab')
        go_screen(page, 'lidar-slam-subtab')
        page.wait_for_timeout(500)
        start = page.locator('#slam-start-button')
        if not start.is_visible():
            failures.append('slam legacy: start button not visible')
        else:
            start.click(trial=True, timeout=3000)
        if page.locator('#lidar-slam-subtab .ui-screen-heading').is_visible():
            failures.append('slam legacy: header heading visible')
        if page.locator('#lidar-slam-stepper').is_visible():
            failures.append('slam legacy: stepper visible')
        if not page.locator('#lidar-slam-subtab > .section > h2').is_visible():
            failures.append('slam legacy: old h2 hidden')
        shot(page, 'p2-slam-legacy-nav')
    except Exception as e:  # trial click 시간 초과 = 클릭 불가
        failures.append(f'slam legacy: {e}')
    finally:
        page.evaluate("localStorage.removeItem('webui.legacyNav')")


def check_slam_layout(page, failures, width=1440):
    """지도 8 / 정보 4 배치, 빈 지도 칸, Config 패널 아래 버튼 (SLAM 미실행)."""
    if not page.locator('.ui-screen-grid .ui-col-main #lidar-slam-viewer-empty').is_visible():
        failures.append('slam layout: empty viewer not visible in main column')
    if not page.locator('.ui-col-side #slam-config-container').count():
        failures.append('slam layout: config container not in side column')
    if not page.locator('#slam-config-container .ui-btn[onclick*="loadSlamConfig"]').count():
        failures.append('slam layout: Config Load .ui-btn not in config container')
    if width == 1440:
        sizes = page.evaluate("""() => {
            const m = document.querySelector('#lidar-slam-subtab .ui-col-main');
            const s = document.querySelector('#lidar-slam-subtab .ui-col-side');
            return m && s ? [m.getBoundingClientRect().width, s.getBoundingClientRect().width] : null;
        }""")
        if not sizes or not sizes[1] or abs(sizes[0] / sizes[1] - 2) > 0.3:
            failures.append(f'slam layout: main/side width {sizes} not ~2:1')
    shot(page, f'p2-slam-layout-{width}')


def mock_slam_running(page):
    def fulfill_state(route):
        resp = route.fetch()
        data = resp.json()
        data['is_running'] = True
        route.fulfill(response=resp, body=json.dumps(data))
    page.route('**/api/slam/state', fulfill_state)


def check_slam_save_overlay(page, failures):
    """save_map 성공 + save_map_status 진행 중 흉내 → 지도 칸 안 오버레이·진행 바·경과 시간."""
    status = {'saving': True, 'done': False, 'success': False, 'message': 'Saving map... (10 s elapsed)'}
    mock_slam_running(page)
    page.route('**/api/slam/save_map', lambda route: route.fulfill(
        status=200, content_type='application/json', body=json.dumps({'success': True, 'message': 'started'})))
    page.route('**/api/slam/save_map_status', lambda route: route.fulfill(
        status=200, content_type='application/json', body=json.dumps(status)))
    page.wait_for_timeout(2500)
    page.evaluate("""() => {
        document.getElementById('save-map-directory').value = 'mock_map';
        confirmSaveMap();
    }""")
    page.wait_for_timeout(500)
    overlay = page.locator('.ui-col-main #slam-save-map-status-area.ui-viewer-overlay')
    if not overlay.is_visible():
        failures.append('slam save: overlay not visible in main column')
    if not page.locator('#slam-save-map-status-area .ui-progress').is_visible():
        failures.append('slam save: progress bar not visible while saving')
    cancel = page.locator('#slam-save-map-cancel-btn')
    if cancel.inner_text().strip() != '기다리기 중단' or cancel.get_attribute('title') != '노드의 저장은 끝까지 진행됩니다':
        failures.append(f'slam save: cancel text/title {cancel.inner_text()!r}')
    first = page.locator('#slam-save-map-elapsed').inner_text().strip()
    page.wait_for_timeout(2200)
    second = page.locator('#slam-save-map-elapsed').inner_text().strip()
    if first == second or not second.startswith('0:0'):
        failures.append(f'slam save: elapsed not ticking {first!r} -> {second!r}')
    if badge_text(page, 'lidar-slam-header-badge') != 'Saving Map…':
        failures.append('slam save: badge not Saving Map…')
    shot(page, 'p2-slam-saving')
    status.update({'saving': False, 'done': True, 'success': True, 'message': 'Map saved successfully to directory: mock_map'})
    page.wait_for_timeout(2500)
    if page.locator('#slam-save-map-status-area .ui-progress').is_visible():
        failures.append('slam save: progress bar visible after success')
    shot(page, 'p2-slam-saved')
    page.unroute_all(behavior='wait')


def check_slam_config_load_failed(page, failures):
    """기본 설정 읽기가 실패해도 Config Load 버튼이 보여야 한다 (버튼이 Config 패널 안에 있으므로)."""
    page.route('**/api/slam/load_config_file', lambda route: route.abort())
    page.reload(wait_until='load')
    page.wait_for_selector('#slam-tab')
    go_screen(page, 'lidar-slam-subtab')
    page.wait_for_timeout(1500)
    if not page.locator('#slam-config-container .ui-btn[onclick*="loadSlamConfig"]').is_visible():
        failures.append('slam config: Config Load not visible after config load failure')
    shot(page, 'p2-slam-config-load-failed')
    page.unroute('**/api/slam/load_config_file')


FAKE_ANALYTICS_JS = """(args) => {
    const [n, dop] = args;
    for (let i = 0; i < n; i++) {
        slamAnalyticsDashboard._onMessage({
            imu_freq: 199 + (i % 3), lid_freq: 10, cpu_usage: 50 + (i % 7), ram_usage: 812,
            traj_dist: 90 + i * 0.2, run_time: 420 + i,
            imu_time: 0.002, state_time: 0.008 + (i % 4) * 0.001, map_time: 0.004, total_time: 0.031,
            scan_dop: dop, matching_dop: dop, kf_count: 12, kf_dist_last: 1.5,
            imu_mean: 0.002, imu_max: 0.004, state_mean: 0.008, state_max: 0.012,
            map_mean: 0.004, map_max: 0.009, total_mean: 0.03, total_max: 0.05,
        });
    }
}"""


def spark_points(page, key):
    return page.evaluate(f"""() => {{
        const line = document.querySelector('#analytics-spark-{key} polyline');
        return line ? (line.getAttribute('points') || '') : '';
    }}""")


def check_slam_analytics(page, failures, width=1440):
    """Analytics를 오른쪽 칸 KPI 2열·스파크라인으로 — 가짜 메시지를 처리 함수에 직접 넣는다 (rosbridge 구독 없음)."""
    page.evaluate('slamAnalyticsDashboard.show()')
    page.wait_for_timeout(500)
    if page.locator('#slam-analytics-dashboard .ui-kpi').count() < 4:
        failures.append(f'slam analytics ({width}): fewer than 4 .ui-kpi')
    if page.locator('#slam-analytics-detail').is_visible():
        failures.append(f'slam analytics ({width}): full statistics not collapsed')
    widths = page.evaluate("""() => {
        const svg = document.querySelector('#analytics-chart-proctime .main-svg');
        const side = document.querySelector('#lidar-slam-subtab .ui-col-side');
        return svg && side ? [svg.getBoundingClientRect().width, side.getBoundingClientRect().width] : null;
    }""")
    if not widths or widths[0] > widths[1]:
        failures.append(f'slam analytics ({width}): chart width {widths} exceeds side column')
    page.evaluate(FAKE_ANALYTICS_JS, [12, 0])
    if not page.locator('#analytics-dop-off').is_visible():
        failures.append(f'slam analytics ({width}): DOP off note hidden with all-zero DOP')
    page.evaluate(FAKE_ANALYTICS_JS, [30, 0.7])
    page.wait_for_timeout(500)
    if page.locator('#analytics-dop-off').is_visible():
        failures.append(f'slam analytics ({width}): DOP off note visible with non-zero DOP')
    for key in ('imu', 'lid', 'cpu'):
        points = spark_points(page, key)
        if len(points.split()) < 30:
            failures.append(f'slam analytics ({width}): sparkline {key} has {len(points.split())} points')
    if page.locator('#analytics-imu-hz').inner_text().strip() not in ('199', '200', '201'):
        failures.append(f'slam analytics ({width}): imu hz value not rendered')
    shot(page, f'p2-slam-analytics-{width}')
    page.evaluate('slamAnalyticsDashboard.unsubscribe(); slamAnalyticsDashboard.hide();')
    if spark_points(page, 'imu'):
        failures.append(f'slam analytics ({width}): sparkline not cleared on reset')


def check_slam_kf_reset(page, failures):
    """Stop→Start 경로(unsubscribe → _resetWidgets)에서 이전 kf_count가 지워져 Save Map이 다시 잠겨야 한다."""
    page.evaluate('slamAnalyticsDashboard.show()')
    page.evaluate(FAKE_ANALYTICS_JS, [3, 0.7])
    if page.locator('#ad-kf_count').inner_text().strip() != '12':
        failures.append('slam kf reset: fake kf_count not rendered')
    page.evaluate('slamAnalyticsDashboard.unsubscribe(); slamAnalyticsDashboard.hide();')
    kf = page.locator('#ad-kf_count').inner_text().strip()
    if kf != '—':
        failures.append(f'slam kf reset: ad-kf_count {kf!r} after reset')
    mock_slam_running(page)
    page.wait_for_timeout(3000)
    save = page.locator('#slam-save-map-button')
    # 앞 저장 검사가 남긴 justSaved로 배지는 "Map Saved · …"일 수 있어 Stop 활성으로 실행 흉내를 확인
    if not page.locator('#slam-stop-button').is_enabled():
        failures.append('slam kf reset: running mock not applied')
    if save.is_enabled() or '키프레임' not in (save.get_attribute('title') or ''):
        failures.append('slam kf reset: save map unlocked after reset (stale kf_count)')
    page.unroute_all(behavior='wait')


def fulfill_ok(route):
    route.fulfill(status=200, content_type='application/json', body=json.dumps({'success': True, 'message': 'mock'}))


def config_lock_state(page):
    return page.evaluate("""() => {
        const c = document.getElementById('slam-config-container');
        const inputs = [...c.querySelectorAll('input, select, textarea')];
        const btns = [...c.querySelectorAll('.ui-panel-footer .ui-btn')];
        const hint = c.querySelector('p.ui-readonly-only');
        return {
            visible: c.offsetParent !== null,
            inputs: inputs.length,
            inputsDisabled: inputs.filter(el => el.disabled).length,
            btnsDisabled: btns.filter(el => el.disabled).length,
            hintVisible: Boolean(hint && hint.offsetParent !== null),
        };
    }""")


def check_slam_config_readonly(page, failures):
    """Start 성공 → Config 패널이 숨지 않고 읽기 전용, Stop 뒤 다시 열림."""
    go_screen(page, 'lidar-slam-subtab')
    page.wait_for_selector('#slam-config-parameters [data-config-key]', state='attached')
    page.on('dialog', lambda d: d.accept())
    page.route('**/api/slam/start_mapping', fulfill_ok)
    mock_slam_running(page)
    page.locator('#slam-start-button').click()
    page.wait_for_timeout(1500)
    st = config_lock_state(page)
    if not st['visible']:
        failures.append('slam config readonly: panel hidden while running')
    if not st['inputs'] or st['inputsDisabled'] != st['inputs']:
        failures.append(f'slam config readonly: inputs not all disabled {st}')
    if st['btnsDisabled'] != 2 or not st['hintVisible']:
        failures.append(f'slam config readonly: footer buttons/hint {st}')
    if st['visible']:
        page.locator('#slam-config-container').scroll_into_view_if_needed()
    shot(page, 'p2-slam-config-readonly')
    page.unroute_all(behavior='wait')
    page.route('**/api/slam/stop_mapping', fulfill_ok)
    page.locator('#slam-stop-button').click()
    page.wait_for_timeout(1500)
    st = config_lock_state(page)
    if st['inputsDisabled'] or st['btnsDisabled'] or st['hintVisible']:
        failures.append(f'slam config readonly: not reopened after stop {st}')
    page.unroute_all(behavior='wait')
    shot(page, 'p2-slam-config-reopened')


def fulfill_json(data):
    return lambda route: route.fulfill(status=200, content_type='application/json', body=json.dumps(data))


def check_slam_result_viewer(page, failures):
    """Save Map 결과 뷰어: 툴바 aria, 범례 칩 aria-pressed, 창 크기 변경 시 캔버스 크기 맞춤 (지도 파일 없이)."""
    go_screen(page, 'lidar-slam-subtab')
    labels = page.evaluate("""() => [...document.querySelectorAll('#savemap-viewer-actions button')]
        .map(b => (b.getAttribute('aria-label') || '').trim())""")
    if not labels or not all(labels):
        failures.append(f'slam result: toolbar aria-label missing {labels}')
    rows = page.evaluate("""() => [...document.querySelectorAll('#savemap-result-viewer .slam-legend-row')]
        .map(r => r.getAttribute('aria-pressed'))""")
    if not rows or None in rows:
        failures.append(f'slam result: legend aria-pressed missing {rows}')
    static_title = page.locator('#savemap-result-viewer .slam-legend-row[data-layer="static"]').get_attribute('title')
    if static_title != '동적 객체·이상치 제거(지면 포함)':
        failures.append(f'slam result: static chip title {static_title!r}')
    page.route('**/api/slam/save_map_result*', fulfill_json({}))
    page.evaluate("saveMapResultViewer.show({ directory: 'mock_map' })")
    page.wait_for_selector('#savemap-result-viewer', state='visible')
    page.wait_for_timeout(1000)
    size_js = """() => {
        const c = document.getElementById('savemap-result-canvas-container');
        const cv = document.getElementById('savemap-result-canvas');
        return [c.clientWidth, cv.clientWidth, cv.clientHeight];
    }"""
    before = page.evaluate(size_js)
    dir_text = page.evaluate("() => { const el = document.getElementById('savemap-result-dir'); return el ? el.textContent.trim() : null; }")
    if dir_text != 'pose_graph_optimization/mock_map/':
        failures.append('slam result: save location caption not filled')
    page.locator('#savemap-fs-topview-btn').click()
    page.wait_for_timeout(300)
    if page.locator('#savemap-fs-topview-btn').get_attribute('aria-pressed') != 'true':
        failures.append('slam result: top view aria-pressed not synced')
    shot(page, 'p2-slam-result-viewer')
    vp = page.viewport_size
    page.set_viewport_size({'width': 1280, 'height': vp['height']})
    page.wait_for_timeout(600)
    after = page.evaluate(size_js)
    page.set_viewport_size(vp)
    if not before[1] or after[0] >= before[0] or abs(after[1] - after[0]) > 2:
        failures.append(f'slam result: canvas did not follow container resize {before} -> {after}')
    page.evaluate('saveMapResultViewer.hideAndReset()')
    page.unroute_all(behavior='wait')


def check_slam_restore_failed(page, failures):
    """새로고침 뒤 복원했는데 파일을 하나도 못 읽으면 빈 지도 칸에 복원 실패 글자."""
    go_screen(page, 'lidar-slam-subtab')
    page.route('**/api/slam/save_map_status', fulfill_json(
        {'saving': False, 'done': True, 'success': True, 'message': 'Map saved successfully to directory: old_map'}))
    page.route('**/api/slam/save_map_result*', fulfill_json({'success': False, 'message': 'not allowed'}))
    page.evaluate('async () => { _saveMapRestoreChecked = false; await _maybeRestoreSaveMapViewer(); }')
    page.wait_for_timeout(500)
    title = page.locator('#lidar-slam-viewer-empty .ui-viewer-empty-title').inner_text().strip()
    if not page.locator('#lidar-slam-viewer-empty').is_visible() or title != '서버가 다시 시작되어 이전 결과를 열 수 없습니다':
        failures.append(f'slam restore failed: empty slot title {title!r}')
    if page.locator('#savemap-result-viewer').is_visible():
        failures.append('slam restore failed: empty result viewer left visible')
    shot(page, 'p2-slam-restore-failed')
    page.unroute_all(behavior='wait')


def check_live_toolbars(page, failures):
    """Live 뷰어(SLAM·Localization) 툴바: aria-label, show() 직후 Follow 켜짐 표시, Reset 뒤 Top View 꺼짐, 풀스크린 아이콘."""
    go_screen(page, 'lidar-slam-subtab')
    labels = page.evaluate("""() => [...document.querySelectorAll('#slam-live-viewer-actions button, #loc-viewer-actions button')]
        .map(b => (b.getAttribute('aria-label') || '').trim())""")
    if len(labels) != 10 or not all(labels):
        failures.append(f'live toolbar: aria-label missing {labels}')
    legend = page.locator('#slam-live-legend').text_content()
    for text in ('LIO (원래 자세)', 'Pose Graph (최적화)', '지도: 최근 50 m'):
        if text not in legend:
            failures.append(f'live legend: {text!r} missing')
    pressed_js = """(id) => { const b = document.getElementById(id);
        return [b.classList.contains('active'), b.getAttribute('aria-pressed')]; }"""
    # 미실행이면 상태 폴링이 Live 뷰어를 숨기므로 실행 중을 흉내 낸다.
    # 구독 연결은 rosbridge 없이 끝나지 않을 수 있어 show()를 기다리지 않는다
    mock_slam_running(page)
    page.evaluate('() => { slamLiveViewer.show(); }')
    page.wait_for_timeout(1000)
    if page.evaluate(pressed_js, 'slam-live-follow-btn') != [True, 'true']:
        failures.append(f"slam live: follow not shown ON after show() {page.evaluate(pressed_js, 'slam-live-follow-btn')}")
    if page.evaluate(pressed_js, 'slam-live-fs-topview-btn') != [True, 'true']:
        failures.append('slam live: top view not pressed after show()')
    page.locator('#slam-live-reset-btn').click()
    if page.evaluate(pressed_js, 'slam-live-fs-topview-btn') != [False, 'false']:
        failures.append('slam live: top view still pressed after Reset')
    shot(page, 'p2-slam-live-toolbar')
    page.locator('#slam-live-fullscreen-btn').click()
    page.wait_for_timeout(500)
    icons = page.evaluate("""() => [!!document.fullscreenElement,
        getComputedStyle(document.getElementById('slam-live-fullscreen-icon-expand')).display,
        getComputedStyle(document.getElementById('slam-live-fullscreen-icon-collapse')).display]""")
    if icons[0]:
        if icons[1] != 'none' or icons[2] == 'none':
            failures.append(f'slam live: fullscreen icon not switched {icons}')
        page.evaluate('document.exitFullscreen()')
        page.wait_for_timeout(300)
    else:
        print('INFO: headless fullscreen unavailable — fullscreen icon not checked')
    page.evaluate('slamLiveViewer.hide()')
    page.unroute_all(behavior='wait')

    def fulfill_loc_running(route):
        resp = route.fetch()
        data = resp.json()
        data['is_running'] = True
        route.fulfill(response=resp, body=json.dumps(data))

    page.route('**/api/localization/state', fulfill_loc_running)
    go_screen(page, 'localization-subtab')
    page.evaluate('() => { locLiveViewer.show(); }')
    page.wait_for_timeout(1000)
    if page.evaluate(pressed_js, 'loc-follow-btn') != [True, 'true']:
        failures.append('loc live: follow not shown ON after show()')
    page.locator('#loc-reset-btn').click()
    if page.evaluate(pressed_js, 'loc-fs-topview-btn') != [False, 'false']:
        failures.append('loc live: top view still pressed after Reset')
    shot(page, 'p2-loc-live-toolbar')
    page.evaluate('locLiveViewer.hide()')
    page.unroute_all(behavior='wait')


if __name__ == '__main__':
    failures = []
    all_errors = []
    with open_page() as (page, errors):
        check_slam_ready(page, failures)
        check_slam_layout(page, failures)
        check_slam_analytics(page, failures)
        if not failures:
            check_slam_start_fail(page, failures)
            check_slam_running(page, failures)
            check_slam_save_overlay(page, failures)
            check_slam_kf_reset(page, failures)
            check_slam_legacy_nav(page, failures)
        all_errors += errors
    with open_page(width=1280) as (page, errors):
        go_screen(page, 'lidar-slam-subtab')
        page.wait_for_timeout(500)
        check_slam_layout(page, failures, width=1280)
        check_slam_analytics(page, failures, width=1280)
        all_errors += errors
    with open_page() as (page, errors):
        check_slam_config_load_failed(page, failures)
        # 일부러 끊은 요청이 남기는 콘솔 에러는 빼고 JS 예외만 본다
        all_errors += [e for e in errors if e.startswith('pageerror')]
    with open_page() as (page, errors):
        check_slam_config_readonly(page, failures)
        all_errors += errors
    with open_page() as (page, errors):
        check_live_toolbars(page, failures)
        all_errors += errors
    with open_page() as (page, errors):
        check_slam_result_viewer(page, failures)
        check_slam_restore_failed(page, failures)
        all_errors += errors
    # 흉내 낸 Start 실패가 남기는 기존 console.error
    finish(failures, [e for e in all_errors if e != 'Failed to start SLAM mapping'])
