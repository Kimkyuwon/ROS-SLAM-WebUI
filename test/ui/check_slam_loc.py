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


def check_loc_ready(page, failures):
    """Localization 진입: 단계 Config, Start 활성, 지도 경로(.pcd), 8/4 배치, Config 버튼이 패널 안."""
    go_screen(page, 'localization-subtab')
    page.wait_for_selector('#localization-config-parameters [data-config-key]', state='attached')
    page.wait_for_timeout(800)
    step = current_step_text(page, 'localization-stepper')
    if step != 'Config':
        failures.append(f'loc: current step {step!r} != Config')
    start = page.locator('#localization-header-actions #localization-start-button')
    if not start.count() or not start.is_enabled():
        failures.append('loc: start button missing in header or disabled')
    map_path = page.evaluate("() => { const el = document.getElementById('localization-map-path'); return el ? el.textContent : ''; }")
    if '.pcd' not in map_path or not map_path.startswith('FAST_LIO/'):
        failures.append(f'loc: map path {map_path!r}')
    if not page.locator('.ui-col-main #localization-viewer-empty').is_visible():
        failures.append('loc: empty viewer not visible')
    if not page.locator('.ui-col-side #localization-config-container .ui-btn[onclick*="loadLocalizationConfig"]').is_visible():
        failures.append('loc: Config Load not visible in config panel')
    shot(page, 'p2-loc-ready')


def check_loc_stop_late_state(page, failures):
    """Stop 전에 보낸 상태 요청이 늦게 is_running=true로 와도 Live 뷰어가 다시 뜨지 않는다."""
    go_screen(page, 'localization-subtab')
    held = []

    def as_running(route):
        resp = route.fetch()
        data = resp.json()
        data['is_running'] = True
        route.fulfill(response=resp, body=json.dumps(data))

    # 첫 요청은 붙잡아 늦게 돌려주고, 뒤 요청도 running으로 답한다 (Stop 직후 DDS 반영 지연 흉내)
    def handle(route):
        if not held:
            held.append(route)
        else:
            as_running(route)

    page.on('dialog', lambda d: d.dismiss())
    page.route('**/api/localization/state', handle)
    page.wait_for_timeout(700)
    page.evaluate('() => { stopLocalizationMapping(); }')
    page.wait_for_timeout(1500)
    if held:
        as_running(held[0])
    else:
        failures.append('loc stop: state request not intercepted')
    # Stop 2초 뒤 (LOC_RESTORE_HOLDOFF_MS 3초 안) 확인
    page.wait_for_timeout(500)
    if page.locator('#localization-live-viewer').is_visible():
        failures.append('loc stop: live viewer reappeared from a late state response')
    page.evaluate('locLiveViewer.hide()')
    page.unroute_all(behavior='wait')


FAKE_LOC_JS = """(args) => {
    const [n, init, ratio, updated] = args;
    for (let i = 0; i < n; i++) {
        locAnalyticsDashboard._onMessage({
            imu_freq: 200, lid_freq: 10, cpu_usage: 41, ram_usage: 900, traj_dist: 12 + i * 0.1, run_time: 30 + i,
            imu_time: 0.002, state_time: 0.008, map_time: 0.004, total_time: 0.02,
            scan_dop: 0.5, matching_dop: 0.4, dop_ratio: 0.7, match_ratio: ratio, linear_velo: 0.84,
            is_initialized: init, map_updated: updated && i % 2 === 0,
            imu_mean: 0.002, imu_max: 0.004, state_mean: 0.008, state_max: 0.012,
            map_mean: 0.004, map_max: 0.009, total_mean: 0.02, total_max: 0.03,
            scan_size: 1234, kf_count: 3,
        });
    }
}"""

LOC_STATE_RUNNING = '**/api/localization/state'


def loc_config_lock_state(page):
    return page.evaluate("""() => {
        const c = document.getElementById('localization-config-container');
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


def check_loc_running(page, failures):
    """실행 중(상태 API를 가짜로): Not Init 배지·읽기 전용 Config·10초 뒤 안내, 메시지 뒤 Initialized · Tracking, KPI·러그·리셋."""
    go_screen(page, 'localization-subtab')
    page.wait_for_selector('#localization-config-parameters [data-config-key]', state='attached')

    def as_running(route):
        resp = route.fetch()
        data = resp.json()
        data['is_running'] = True
        route.fulfill(response=resp, body=json.dumps(data))

    def summary_running(route):
        resp = route.fetch()
        body = resp.json()
        body['data']['localization']['running'] = True
        route.fulfill(response=resp, body=json.dumps(body))

    page.route(LOC_STATE_RUNNING, as_running)
    page.route('**/api/system/status_summary', summary_running)
    page.wait_for_timeout(3000)  # 요약 폴링 2초 주기
    try:
        text = badge_text(page, 'localization-header-badge')
        if text != 'Running · Not Init':
            failures.append(f'loc running: badge {text!r} != Running · Not Init')
        if page.locator('#localization-start-button').is_enabled() or not page.locator('#localization-stop-button').is_enabled():
            failures.append('loc running: start/stop button state')
        st = loc_config_lock_state(page)
        if not st['visible'] or not st['inputs'] or st['inputsDisabled'] != st['inputs']:
            failures.append(f'loc running: config not read-only {st}')
        if st['btnsDisabled'] != 2 or not st['hintVisible']:
            failures.append(f'loc running: footer buttons/hint {st}')
        if page.locator('#localization-viewer-empty').is_visible():
            failures.append('loc running: empty viewer still visible with live viewer')
        if page.locator('#localization-init-hint').is_visible():
            failures.append('loc running: init hint visible before 10s')
        page.wait_for_timeout(10500)
        if not page.locator('#localization-init-hint').is_visible():
            failures.append('loc running: init hint not visible after 10s Not Init')

        page.evaluate("document.querySelector('#localization-config-parameters [data-config-key$=\"odom_mode\"]').value = '0'")
        page.evaluate(FAKE_LOC_JS, [450, True, 0.88, True])
        page.wait_for_timeout(800)
        text = badge_text(page, 'localization-header-badge')
        if text != 'Initialized · Tracking':
            failures.append(f'loc running: badge after init {text!r}')
        step = current_step_text(page, 'localization-stepper')
        if step != 'Tracking':
            failures.append(f'loc running: step {step!r} != Tracking')
        if page.locator('#localization-init-hint').is_visible():
            failures.append('loc running: init hint visible after Initialized')
        if page.locator('#loc-analytics-match-ratio').inner_text().strip() != '0.88':
            failures.append('loc analytics: match ratio not 0.88')
        wheel = page.locator('#loc-analytics-wheel-velo').inner_text().strip()
        if wheel != '휠 오도메트리 꺼짐 (odom_mode 0)':
            failures.append(f'loc analytics: wheel text {wheel!r} with odom_mode 0')
        shot(page, 'p2-loc-running')
        page.evaluate("document.querySelector('#localization-config-parameters [data-config-key$=\"odom_mode\"]').value = '1'")
        page.evaluate(FAKE_LOC_JS, [1, True, 0.3, False])
        wheel = page.locator('#loc-analytics-wheel-velo').inner_text().strip()
        if wheel != '0.84 m/s':
            failures.append(f'loc analytics: wheel text {wheel!r} with odom_mode 1')
        if page.locator('#loc-kpi-match').get_attribute('data-tone') != 'warn' \
                or page.locator('#loc-analytics-match-hint').inner_text().strip() != '낮음':
            failures.append('loc analytics: low match ratio not warn/낮음')
        counts = page.evaluate("""() => ['loc-analytics-chart-proctime', 'loc-analytics-chart-dop']
            .map(id => Math.max(...document.getElementById(id).data.map(t => t.x.length)))""")
        if max(counts) > 400:
            failures.append(f'loc analytics: trace points {counts} exceed 400')
        spark = page.evaluate("""() => {
            const line = document.querySelector('#loc-analytics-spark-match polyline');
            return line ? line.getAttribute('points').split(' ').length : 0;
        }""")
        if spark < 30:
            failures.append(f'loc analytics: match sparkline has {spark} points')
        rug = page.locator('#loc-analytics-dashboard .ui-loc-rug').inner_text()
        if 'Map update' not in rug or '업데이트됨' not in rug or '건너뜀' not in rug:
            failures.append(f'loc analytics: rug title/legend {rug!r}')
        tip = page.locator('.ui-loc-rug-key[data-kind="skipped"]').get_attribute('title') or ''
        if 'dop_ratio < 0.6' not in tip:
            failures.append(f'loc analytics: rug tooltip {tip!r}')
        if '31 항목' not in page.locator('#loc-analytics-detail-toggle').inner_text():
            failures.append('loc analytics: detail toggle lacks 31 항목')
        widths = page.evaluate("""() => {
            const svg = document.querySelector('#loc-analytics-chart-proctime .main-svg');
            const side = document.querySelector('#localization-subtab .ui-col-side');
            return svg && side ? [svg.getBoundingClientRect().width, side.getBoundingClientRect().width] : null;
        }""")
        if not widths or widths[0] > widths[1]:
            failures.append(f'loc analytics: chart width {widths} exceeds side column')
        before = page.evaluate("document.getElementById('loc-rug-canvas').width")
        page.set_viewport_size({'width': 1280, 'height': 900})
        page.wait_for_timeout(600)
        after = page.evaluate("document.getElementById('loc-rug-canvas').width")
        if before == after:
            failures.append(f'loc analytics: rug canvas width unchanged on resize ({before})')
        page.set_viewport_size({'width': 1440, 'height': 900})
    finally:
        page.unroute_all(behavior='wait')
        page.evaluate('locAnalyticsDashboard.unsubscribe(); locAnalyticsDashboard.hide(); locLiveViewer.hide();')
    reset = page.evaluate("""() => ({
        header: document.getElementById('loc-header-init-badge').textContent,
        headerCls: document.getElementById('loc-header-init-badge').className,
        rug: document.getElementById('loc-init-status-dot').textContent,
        rugCls: document.getElementById('loc-init-status-dot').className,
        detail: document.getElementById('lad-scan_size').textContent,
        match: document.getElementById('loc-analytics-match-ratio').textContent,
    })""")
    if reset['header'] != '— Not Init' or 'ok' in reset['headerCls'].split() or 'bad' in reset['headerCls'].split():
        failures.append(f'loc reset: header badge {reset}')
    if reset['rug'] != '— Not Init' or 'ok' in reset['rugCls'].split() or 'bad' in reset['rugCls'].split():
        failures.append(f'loc reset: rug badge {reset}')
    if reset['detail'] != '—' or reset['match'] != '—':
        failures.append(f'loc reset: values not cleared {reset}')


WARN_COLOR = 'rgb(232, 176, 75)'
STOP_COLOR = 'rgb(240, 97, 109)'
LOG_LINES = [
    '[laserMapping]: IMU Initial Done',
    '[laserMapping]: No point, skip this scan!',
    '[laserLocalization]: Map file cannot open: /x.pcd',
    '[laserMapping]: lidar loop back, clear buffer',
]


def check_save_map_name(page, failures):
    """Save Map 이름 검증(300ms 디바운스)·덮어쓰기 확인. save_map_check·save_map은 page.route로 흉내."""
    checks = []
    posts = []

    def on_check(route):
        name = route.request.url.split('name=', 1)[1] if 'name=' in route.request.url else ''
        checks.append(name)
        body = {'valid': True, 'reason': '', 'exists': name == 'ok_name'}
        if name == 'Map':
            body = {'valid': False, 'reason': '"Map"은(는) 예약된 이름입니다', 'exists': False}
        route.fulfill(status=200, content_type='application/json', body=json.dumps({'success': True, 'data': body}))

    def on_save(route):
        posts.append(route.request.post_data)
        route.fulfill(status=200, content_type='application/json',
                      body=json.dumps({'success': False, 'message': 'mock'}))

    page.route('**/api/slam/save_map_check*', on_check)
    page.route('**/api/slam/save_map', on_save)
    page.on('dialog', lambda d: d.dismiss())
    go_screen(page, 'lidar-slam-subtab')
    page.evaluate('() => { saveSlamMap(); }')
    confirm = page.locator('#save-map-confirm-btn')
    error = page.locator('#save-map-name-error')
    if not confirm.is_disabled():
        failures.append('save-map: confirm not locked with empty name')
    if error.is_visible():
        failures.append('save-map: error shown for empty name')
    if checks:
        failures.append(f'save-map: empty name asked the server {checks}')
    hint = page.locator('#save-map-location-hint').inner_text()
    if 'pose_graph_optimization/<이름>/' not in hint:
        failures.append(f'save-map: location hint {hint!r}')
    if '영문·숫자·_·- 만, 1~64자, Map 사용 불가' not in page.locator('#save-map-modal').inner_text():
        failures.append('save-map: rule text missing')

    inp = page.locator('#save-map-directory')
    inp.fill('Map')
    page.wait_for_timeout(100)
    if checks:
        failures.append(f'save-map: check called before debounce {checks}')
    page.wait_for_timeout(500)
    if checks != ['Map']:
        failures.append(f'save-map: debounced check calls {checks}')
    if not confirm.is_disabled():
        failures.append('save-map: confirm not locked for Map')
    if '예약된 이름' not in error.inner_text() or not error.is_visible():
        failures.append(f'save-map: error text {error.inner_text()!r}')
    if error.evaluate('el => getComputedStyle(el).color') != STOP_COLOR:
        failures.append('save-map: error not stop color')
    shot(page, 'p2-save-map-name-error')
    inp.press('Enter')
    page.wait_for_timeout(200)
    if posts:
        failures.append('save-map: Enter on locked confirm sent a save request')

    inp.fill('ok_name')
    page.wait_for_timeout(700)
    if confirm.is_disabled():
        failures.append('save-map: confirm locked for valid name')
    if not page.locator('#save-map-exists-note').is_visible():
        failures.append('save-map: exists note hidden')
    if 'pose_graph_optimization/ok_name/' not in page.locator('#save-map-location-hint').inner_text():
        failures.append('save-map: location hint not live-updated')
    shot(page, 'p2-save-map-name-exists')
    confirm.click()
    page.wait_for_selector('.ui-dialog', timeout=2000)
    dialog_text = page.locator('.ui-dialog').inner_text()
    if '덮어쓰기' not in dialog_text or '같은 이름의 지도가 있습니다' not in dialog_text:
        failures.append(f'save-map: overwrite dialog text {dialog_text!r}')
    if page.locator('.ui-dialog .ui-btn').first.inner_text() != '취소':
        failures.append('save-map: dialog cancel button missing')
    shot(page, 'p2-save-map-overwrite-confirm')
    page.locator('.ui-dialog .ui-btn').first.click()
    page.wait_for_timeout(300)
    if posts:
        failures.append(f'save-map: save_map sent after overwrite cancel {posts}')
    if not page.locator('#save-map-modal').is_visible():
        failures.append('save-map: modal closed after overwrite cancel')

    inp.press('Enter')
    page.wait_for_selector('.ui-dialog', timeout=2000)
    page.locator('.ui-dialog .ui-btn').last.click()
    page.wait_for_timeout(500)
    if len(posts) != 1 or 'ok_name' not in posts[0]:
        failures.append(f'save-map: save_map after overwrite ok {posts}')
    page.unroute_all(behavior='wait')


def check_node_log(page, failures, kind, screen):
    """노드 로그 패널: 열기·줄 색·3초 폴링·닫으면 폴링 중단 (process_log는 page.route로 흉내)."""
    calls = []
    name = 'slam' if kind == 'slam' else 'localization'

    def on_log(route):
        if f'name={name}&' in route.request.url:
            calls.append(route.request.url)
        route.fulfill(status=200, content_type='application/json',
                      body=json.dumps({'success': True, 'data': {'lines': LOG_LINES}}))

    page.route('**/api/system/process_log*', on_log)
    go_screen(page, screen)
    page.wait_for_timeout(300)
    btn = page.locator(f'#{kind}-log-btn')
    panel = page.locator(f'#{kind}-log-panel')
    if not btn.count() or btn.get_attribute('aria-label') != '노드 로그':
        failures.append(f'{kind} log: button missing or aria-label wrong')
        return
    first = page.evaluate(
        f"document.querySelector('#{screen} .ui-screen-actions').firstElementChild.id")
    if first != f'{kind}-log-btn':
        failures.append(f'{kind} log: button not first in header actions ({first})')
    if panel.is_visible() or calls:
        failures.append(f'{kind} log: panel visible or polled while closed')
    side_first = page.evaluate(f"document.querySelector('#{screen} .ui-col-side').firstElementChild.id")
    if side_first != f'{kind}-log-panel':
        failures.append(f'{kind} log: panel not first in side column ({side_first})')
    btn.click()
    page.wait_for_selector(f'#{kind}-log-pre > div', timeout=3000)
    if not panel.is_visible():
        failures.append(f'{kind} log: panel not visible after click')
    if f'name={name}&lines=50' not in calls[0]:
        failures.append(f'{kind} log: request url {calls[0]}')
    rows = page.eval_on_selector_all(
        f'#{kind}-log-pre > div',
        "els => els.map(e => [e.textContent, getComputedStyle(e).color])")
    if [r[0] for r in rows] != LOG_LINES:
        failures.append(f'{kind} log: rows {rows}')
    else:
        colors = [r[1] for r in rows]
        if colors[1] != WARN_COLOR or colors[3] != WARN_COLOR or colors[2] != STOP_COLOR or colors[0] in (
                WARN_COLOR, STOP_COLOR):
            failures.append(f'{kind} log: line colors {colors}')
    if '50줄' not in panel.inner_text() or 'stdout(cout)' not in panel.inner_text():
        failures.append(f'{kind} log: title/hint text missing')
    shot(page, f'p2-{kind}-node-log')
    n = len(calls)
    page.wait_for_timeout(3600)
    if len(calls) <= n:
        failures.append(f'{kind} log: no 3s polling while open ({n} -> {len(calls)})')
    btn.click()
    page.wait_for_timeout(300)
    n = len(calls)
    page.wait_for_timeout(3600)
    if panel.is_visible() or len(calls) != n:
        failures.append(f'{kind} log: still visible/polling after close')
    page.evaluate(f"uiShell.openNodeLog('{kind}')")
    page.wait_for_timeout(300)
    if not panel.is_visible():
        failures.append(f'{kind} log: uiShell.openNodeLog did not open')
    other = 'localization-subtab' if kind == 'slam' else 'lidar-slam-subtab'
    go_screen(page, other)
    page.wait_for_timeout(2500)
    n = len(calls)
    page.wait_for_timeout(3600)
    if len(calls) != n:
        failures.append(f'{kind} log: polling continues while screen hidden')
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
        check_loc_ready(page, failures)
        check_loc_stop_late_state(page, failures)
        all_errors += [e for e in errors if 'Failed to stop Localization' not in e]
    with open_page() as (page, errors):
        check_loc_running(page, failures)
        all_errors += errors
    with open_page() as (page, errors):
        check_live_toolbars(page, failures)
        all_errors += errors
    with open_page() as (page, errors):
        check_slam_result_viewer(page, failures)
        check_slam_restore_failed(page, failures)
        all_errors += errors
    with open_page() as (page, errors):
        check_save_map_name(page, failures)
        all_errors += errors
    with open_page() as (page, errors):
        check_node_log(page, failures, 'slam', 'lidar-slam-subtab')
        check_node_log(page, failures, 'loc', 'localization-subtab')
        all_errors += errors
    # 흉내 낸 Start 실패가 남기는 기존 console.error
    finish(failures, [e for e in all_errors if e != 'Failed to start SLAM mapping'])
