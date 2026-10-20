"""사이드바 8항목 클릭 → 해당 서브탭 활성 + aria-current, 옛 탭 숨김."""
from ui_check_common import SCREEN_TABS, finish, open_page, shot

failures = []
with open_page() as (page, errors):
    if page.locator('.ui-sidebar-item').count() != 8:
        failures.append('sidebar item count')
        finish(failures, errors)
    if page.locator('.tab-navigation').is_visible():
        failures.append('old tab nav visible')
    for subtab_id in SCREEN_TABS:
        page.locator(f'.ui-sidebar-item[data-screen="{subtab_id}"]').click()
        page.wait_for_selector(f'[id="{subtab_id}"].active')
        cur = page.locator('.ui-sidebar-item[aria-current="page"]').get_attribute('data-screen')
        if cur != subtab_id:
            failures.append(f'aria-current {cur} != {subtab_id}')
        title = page.evaluate(f"uiShell.SCREENS['{subtab_id}'].title")
        heading = page.locator('.subtab-content.active .ui-screen-header h1')
        if not heading.count() or heading.first.inner_text().strip() != title:
            failures.append(f'{subtab_id} header title != {title}')
        if page.locator('.tab-content h2:visible').count():
            failures.append(f'{subtab_id} old h2 visible')
    if page.locator('#ui-conn-rosbridge[data-tone]').count() != 1:
        failures.append('conn rosbridge badge missing')
    web_port = page.locator('#ui-conn-port-web')
    if not web_port.count() or web_port.inner_text().strip() != '8080':
        failures.append('conn web port != 8080')
    page.wait_for_timeout(4000)
    latency = page.locator('#ui-conn-latency')
    if not latency.count() or not latency.inner_text().strip().endswith('ms'):
        failures.append('conn latency not ms')
    shot(page, 'p1-shell')

    # 맨 아래로 스크롤해도 사이드바는 화면 높이 그대로 왼쪽에 붙어 있어야 한다
    page.locator('.ui-sidebar-item[data-screen="lidar-slam-subtab"]').click()
    page.wait_for_selector('#slam-config-parameters [data-config-key]', state='attached')
    vh = page.viewport_size['height']
    cases = (
        ('analytics', 'slamAnalyticsDashboard.show()'),  # Plotly 내부 div가 칸 밖으로 넘치는 경우
        ('config-open', 'slamAnalyticsDashboard.hide(); toggleSlamConfig()'),  # 본문이 길어진 경우
    )
    for name, setup in cases:
        page.evaluate(setup)
        page.wait_for_timeout(500)
        page.evaluate('window.scrollTo(0, document.documentElement.scrollHeight)')
        page.wait_for_timeout(200)
        box = page.locator('.ui-sidebar').bounding_box()
        if abs(box['y']) > 2 or abs(box['y'] + box['height'] - vh) > 2:
            failures.append(f'sidebar moved after scroll to bottom ({name}): {box}')
        shot(page, f'p1-shell-scrolled-{name}')
    finish(failures, errors)
