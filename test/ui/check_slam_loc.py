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


if __name__ == '__main__':
    failures = []
    with open_page() as (page, errors):
        check_slam_ready(page, failures)
        if not failures:
            check_slam_start_fail(page, failures)
            check_slam_running(page, failures)
            check_slam_legacy_nav(page, failures)
        # 흉내 낸 Start 실패가 남기는 기존 console.error
        finish(failures, [e for e in errors if e != 'Failed to start SLAM mapping'])
