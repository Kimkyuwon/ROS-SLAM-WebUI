"""Multi-Session·Bag·Recorder·File 화면 (API는 page.route, 무거운 노드 실행 안 함)."""
import json

from ui_check_common import finish, go_screen, open_page, shot


def check_multi_form(page, failures):
    go_screen(page, 'multi-session-slam-subtab')
    page.wait_for_timeout(400)
    run = page.locator('#multi-session-header-actions #slam-opt-run-btn')
    if not run.count():
        failures.append('multi: run button not in header')
        return
    if run.is_enabled():
        failures.append('multi: run enabled with empty maps')
    count = page.locator('#multi-session-progress li').count()
    if count != 5:
        failures.append(f'multi: progress items {count} != 5')
    page.locator('#slam-output').fill('..')
    page.wait_for_timeout(200)
    if run.is_enabled():
        failures.append('multi: run enabled for output ".."')
    err = page.locator('#multi-session-output-error')
    if not err.is_visible() or not err.inner_text().strip():
        failures.append('multi: output error not shown')
    page.locator('#slam-output').fill('Result_A')
    page.wait_for_timeout(1200)
    if page.locator('#slam-output').input_value() != 'Result_A':
        failures.append('multi: output overwritten by state poll')
    shot(page, 'p3-multi-form')


def check_multi_restore(page, failures):
    def state_complete(route):
        resp = route.fetch()
        data = resp.json()
        data['status'] = 'Optimization complete!'
        route.fulfill(response=resp, body=json.dumps(data))

    page.route('**/api/slam/state', state_complete)
    go_screen(page, 'multi-session-slam-subtab')
    page.wait_for_timeout(1000)
    text = page.locator('#slam-opt-run-btn').inner_text()
    if 'Exit' not in text:
        failures.append(f'multi restore: button {text!r}')
    shown = page.evaluate(
        "() => document.getElementById('slam-result-viewer').style.display !== 'none'")
    if not shown:
        failures.append('multi restore: result viewer not shown')
    shot(page, 'p3-multi-restore')
    page.unroute_all(behavior='wait')


def check_multi_resume(page, failures):
    seen = []

    def opt_status(route):
        seen.append(route.request.url)
        route.fulfill(
            status=200, content_type='application/json',
            body=json.dumps({
                'running': True, 'done': False, 'success': False,
                'message': 'Session Edge Loading Complete.',
            }))

    def summary_running(route):
        resp = route.fetch()
        body = resp.json()
        body['data']['optimization']['running'] = True
        route.fulfill(response=resp, body=json.dumps(body))

    page.route('**/api/slam/optimization_status', opt_status)
    page.route('**/api/system/status_summary', summary_running)
    go_screen(page, 'multi-session-slam-subtab')
    page.wait_for_timeout(6000)
    if not seen:
        failures.append('multi resume: optimization_status not requested')
    page.unroute_all(behavior='wait')


def check_bag_topics(page, failures):
    go_screen(page, 'bag-player-subtab')
    page.evaluate(
        """() => applyBagPlayerInfo('/x', {
            topics: ['/a', '/b', {name: '/c', type: 'std_msgs/msg/String', publishable: true}],
            topic_details: [
                {name: '/a', type: 'sensor_msgs/msg/Imu', count: 20},
                {name: '/b', type: 'nav_msgs/msg/Odometry', count: 10},
                {name: '/c', type: 'std_msgs/msg/String', count: 5},
            ],
            duration: 10,
            bag_type: 'ros2',
        }, {resetSlider: true})""")
    rows = page.locator('#bag-topic-table tbody tr')
    if rows.count() != 3:
        failures.append(f'bag topics: rows {rows.count()} != 3')
        return
    hz = rows.nth(0).locator('td').nth(3).inner_text().strip()
    if hz != '2.0':
        failures.append(f'bag topics: hz {hz!r} != 2.0')
    before = page.evaluate('() => bagPlayerState.selectedTopics.length')
    rows.nth(0).locator('input').uncheck()
    after = page.evaluate('() => bagPlayerState.selectedTopics.length')
    if after != before - 1:
        failures.append(f'bag topics: selected {before} -> {after}')
    shot(page, 'p3-bag-topics')
    shot(page, 'p3-bag-convert')


def check_bag_playbar(page, failures):
    go_screen(page, 'bag-player-subtab')
    page.wait_for_timeout(300)
    bar = page.locator('#bag-playbar')
    box = bar.bounding_box()
    height = page.viewport_size['height']
    if not box:
        failures.append('playbar: missing')
        return
    if abs((box['y'] + box['height']) - height) > 12:
        failures.append(f'playbar: bottom {box["y"] + box["height"]} != {height}')
    page.mouse.wheel(0, 900)
    page.wait_for_timeout(200)
    box2 = bar.bounding_box()
    if not box2 or abs((box2['y'] + box2['height']) - height) > 12:
        failures.append('playbar: left the viewport bottom after scroll')
    play = page.locator('#bag-play-button')
    if play.is_enabled():
        failures.append('playbar: play enabled with no bag')
    if play.locator('svg').count() < 1 or play.inner_text().strip() != 'Play':
        failures.append(f'playbar: label {play.inner_text()!r}')
    shot(page, 'p3-bag-playbar')


def check_recorder(page, failures):
    seen = []

    def rec_state(route):
        seen.append(route.request.url)
        route.fulfill(
            status=200, content_type='application/json',
            body=json.dumps({
                'recording': True, 'elapsed_sec': 3, 'size_bytes': 67108864,
                'mode': 'ros2_mcap',
            }))

    page.route('**/api/recorder/state', rec_state)
    go_screen(page, 'bag-recorder-subtab')
    page.wait_for_timeout(1200)
    if not seen:
        failures.append('recorder: state not requested')
    if page.locator('#recorder-stat-time').inner_text().strip() != '0:03':
        failures.append(f'recorder time {page.locator("#recorder-stat-time").inner_text()!r}')
    if page.locator('#recorder-stat-size').inner_text().strip() != '64 MB':
        failures.append(f'recorder size {page.locator("#recorder-stat-size").inner_text()!r}')
    badge = page.locator('#bag-recorder-header-badge .ui-status-text').inner_text().strip()
    if badge != 'Recording · 0:03':
        failures.append(f'recorder badge {badge!r}')
    if page.locator('#recorder-format-select').is_enabled():
        failures.append('recorder: format enabled while recording')
    shot(page, 'p3-recorder')
    page.unroute_all(behavior='wait')


def check_file_player(page, failures):
    go_screen(page, 'file-player-subtab')
    page.locator('#file-player-dataset-seg [data-dataset="kitti"]').click()
    page.wait_for_timeout(300)
    value = page.locator('#dataset-format-select').input_value()
    if value != 'kitti':
        failures.append(f'file dataset {value!r}')
    if not page.locator('#kitti-ui').is_visible():
        failures.append('file: kitti ui hidden')
    if page.locator('#conpr-save-row').is_visible():
        failures.append('file: conpr save row still visible')
    checked = page.locator('#file-player-dataset-seg [data-dataset="kitti"]').get_attribute('aria-checked')
    if checked != 'true':
        failures.append(f'file seg aria-checked {checked!r}')
    if not page.locator('#file-playbar').count():
        failures.append('file: playbar missing')
    shot(page, 'p3-file-player')


if __name__ == '__main__':
    failures = []
    all_errors = []
    with open_page() as (page, errors):
        check_multi_form(page, failures)
        all_errors += errors
    with open_page() as (page, errors):
        check_multi_restore(page, failures)
        all_errors += errors
    with open_page() as (page, errors):
        check_multi_resume(page, failures)
        all_errors += errors
    with open_page() as (page, errors):
        check_bag_topics(page, failures)
        all_errors += errors
    with open_page() as (page, errors):
        check_bag_playbar(page, failures)
        all_errors += errors
    with open_page() as (page, errors):
        check_recorder(page, failures)
        all_errors += errors
    with open_page() as (page, errors):
        check_file_player(page, failures)
        all_errors += errors
    finish(failures, all_errors)
