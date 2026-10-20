"""3D Viewer 헤더 Reset/Snapshot·Displays 눈 토글·Fixed Frame 배지·Views 정리 (서버 필요, ROS 노드 실행 안 함)."""
from ui_check_common import finish, go_screen, open_page, shot

VIEWS_HIDDEN_LABELS = ('Distance', 'Yaw', 'Pitch', 'Scale')


def visible_view_labels(page):
    return page.evaluate(
        """() => [...document.querySelectorAll('#views-params-container .views-row')]
            .filter(row => row.offsetParent !== null)
            .map(row => row.textContent.trim())""")


def check_header_actions(page, failures):
    for btn_id, label in (('viewer-reset-btn', 'Reset'), ('viewer-snapshot-btn', 'Snapshot')):
        btn = page.locator(f'#viewer3d-header-actions #{btn_id}')
        if not btn.count():
            failures.append(f'viz: #{btn_id} not inside #viewer3d-header-actions')
            continue
        if 'ui-btn' not in (btn.get_attribute('class') or '').split():
            failures.append(f'viz: #{btn_id} missing ui-btn class')
        if not btn.locator('.ui-icon use').count():
            failures.append(f'viz: #{btn_id} icon missing')
        if btn.locator('.ui-btn-label').inner_text().strip() != label:
            failures.append(f'viz: #{btn_id} label != {label}')
        if not btn.is_visible():
            failures.append(f'viz: #{btn_id} not visible')
    reset_title = page.locator('#viewer-reset-btn').get_attribute('title')
    if reset_title != '씬 초기화 (구독은 유지)':
        failures.append(f'viz: reset title {reset_title!r}')
    for fn in ('resetAll3DViewer', 'take3DSnapshot', 'resetViewCamera'):
        if page.evaluate(f"typeof window.{fn}") != 'function':
            failures.append(f'viz: window.{fn} missing')


def check_views_panel(page, failures):
    page.wait_for_selector('#views-params-container .views-row', state='attached')
    labels = visible_view_labels(page)
    for name in VIEWS_HIDDEN_LABELS:
        if any(text.startswith(name) for text in labels):
            failures.append(f'viz: Views {name} row still visible (orbit)')
    if not any(text.startswith('Invert Z') for text in labels):
        failures.append('viz: Invert Z Axis row should stay visible')
    page.evaluate("() => { switchViewType('topdown'); }")
    page.wait_for_timeout(200)
    if any(text.startswith('Scale') for text in visible_view_labels(page)):
        failures.append('viz: Views Scale row still visible (topdown)')
    page.evaluate("() => { switchViewType('orbit'); }")
    for field in ('Distance', 'Yaw', 'Pitch', 'Scale'):
        shown = page.evaluate(
            """(name) => [...document.querySelectorAll('#views-params-container .views-row')]
                .some(row => row.textContent.trim().startsWith(name) && row.offsetParent !== null)""", field)
        if shown:
            failures.append(f'viz: {field} visible after switching back')


def check_display_toggles(page, failures):
    page.evaluate(
        """() => {
            viewer3DState.selectedPathTopics.push('/check_viz_path');
            renderDisplayPanel();
        }""")
    box = page.locator('#viewer-display-panel-content .display-topic-item-header input[type="checkbox"]')
    if not box.count():
        failures.append('viz: no display checkbox rendered')
    else:
        first = box.first
        if 'ui-eye-toggle' not in (first.get_attribute('class') or '').split():
            failures.append('viz: display checkbox missing ui-eye-toggle')
        if first.get_attribute('aria-label') != '/check_viz_path 표시':
            failures.append(f'viz: aria-label {first.get_attribute("aria-label")!r}')
        size = first.evaluate('el => { const r = el.getBoundingClientRect(); return [r.width, r.height]; }')
        if size != [20, 20]:
            failures.append(f'viz: eye toggle size {size} != [20, 20]')
        mask_on = first.evaluate('el => getComputedStyle(el).webkitMaskImage')
        first.evaluate('el => el.click()')
        mask_off = first.evaluate('el => getComputedStyle(el).webkitMaskImage')
        if mask_on == mask_off:
            failures.append('viz: eye / eye-off mask does not change on toggle')
        first.evaluate('el => el.click()')
    bare = page.evaluate(
        """() => [...document.querySelectorAll('#viewer-display-panel-content .display-topic-item-header '
            + 'input[type="checkbox"]')].filter(c => !c.getAttribute('aria-label')).length""")
    if bare:
        failures.append(f'viz: {bare} display checkbox without aria-label')
    shot(page, 'p4-3d-displays')
    page.evaluate(
        """() => {
            viewer3DState.selectedPathTopics.length = 0;
            renderDisplayPanel();
        }""")


def badge_state(page):
    return page.evaluate(
        """() => {
            const b = document.getElementById('viewer3d-header-badge');
            return [b.dataset.tone, b.querySelector('.ui-status-text').textContent.trim()];
        }""")


def check_fixed_frame_badge(page, failures):
    tone, text = badge_state(page)
    if not text.startswith('Fixed Frame'):
        failures.append(f'viz: badge text {text!r} does not start with Fixed Frame')
    shot(page, 'p4-3d-badge-initial')

    page.evaluate(
        """() => {
            viewer3DState.tfFrameTree.clear();
            _allKnownFrames.clear();
            viewer3DState.topicFrameIds.clear();
            setFixedFrame('map');
            viewer3DState.tfFrameTree.set('base_link', {
                parentId: 'odom', translation: {x: 0, y: 0, z: 0}, quaternion: {x: 0, y: 0, z: 0, w: 1},
                stamp: {sec: 1, nanosec: 0}, isStatic: false});
            updateFixedFrameBadge();
        }""")
    tone, text = badge_state(page)
    if tone != 'warn' or text != 'Fixed Frame · map (TF에 없음 — odom 권장)':
        failures.append(f'viz: warn badge got {tone!r} {text!r}')
    shot(page, 'p4-3d-frame-warning')

    page.locator('#viewer3d-header-badge').click()
    frame = page.evaluate('() => viewer3DState.fixedFrame')
    tone, text = badge_state(page)
    if frame != 'odom' or text != 'Fixed Frame · odom' or tone == 'warn':
        failures.append(f'viz: after click frame={frame!r} badge={tone!r} {text!r}')
    shot(page, 'p4-3d-odom')

    page.evaluate(
        """() => {
            viewer3DState.tfFrameTree.clear();
            _allKnownFrames.clear();
            viewer3DState.topicFrameIds.clear();
            resetFixedFrameAndViewTargetsToDefault();
        }""")


def main():
    failures = []
    with open_page() as (page, errors):
        go_screen(page, '3d-viewer-subtab')
        page.wait_for_timeout(1000)
        check_header_actions(page, failures)
        check_views_panel(page, failures)
        check_display_toggles(page, failures)
        check_fixed_frame_badge(page, failures)
        shot(page, 'p4-3d-viewer')
        finish(failures, errors)


if __name__ == '__main__':
    main()
