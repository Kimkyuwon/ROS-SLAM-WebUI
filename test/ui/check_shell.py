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
    shot(page, 'p1-shell')
    finish(failures, errors)
