"""8화면을 차례로 열고 스크린샷 + 새 JS 에러 0을 확인한다."""
import sys

from ui_check_common import SCREEN_TABS, finish, go_screen, open_page, shot

prefix = sys.argv[1] if len(sys.argv) > 1 else 'smoke'
failures = []
with open_page() as (page, errors):
    for subtab_id in SCREEN_TABS:
        go_screen(page, subtab_id)
        page.wait_for_timeout(600)
        if not page.locator(f'[id="{subtab_id}"]').is_visible():
            failures.append(f'{subtab_id} not visible')
        shot(page, f'{prefix}-{subtab_id}')
    finish(failures, errors)
