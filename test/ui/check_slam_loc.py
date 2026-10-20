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
    page.unroute('**/api/slam/state')
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
    for url in ('**/api/slam/state', '**/api/slam/save_map', '**/api/slam/save_map_status'):
        page.unroute(url)


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


if __name__ == '__main__':
    failures = []
    all_errors = []
    with open_page() as (page, errors):
        check_slam_ready(page, failures)
        check_slam_layout(page, failures)
        if not failures:
            check_slam_start_fail(page, failures)
            check_slam_running(page, failures)
            check_slam_save_overlay(page, failures)
            check_slam_legacy_nav(page, failures)
        all_errors += errors
    with open_page(width=1280) as (page, errors):
        go_screen(page, 'lidar-slam-subtab')
        page.wait_for_timeout(500)
        check_slam_layout(page, failures, width=1280)
        all_errors += errors
    with open_page() as (page, errors):
        check_slam_config_load_failed(page, failures)
        # 일부러 끊은 요청이 남기는 콘솔 에러는 빼고 JS 예외만 본다
        all_errors += [e for e in errors if e.startswith('pageerror')]
    # 흉내 낸 Start 실패가 남기는 기존 console.error
    finish(failures, [e for e in all_errors if e != 'Failed to start SLAM mapping'])
