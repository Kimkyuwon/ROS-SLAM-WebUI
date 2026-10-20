"""status_summary를 page.route로 흉내 내 사이드바 실행 표시(REC)가 3초 안에 뜨는지 확인한다."""
import json

from ui_check_common import finish, open_page, shot

SUMMARY = {
    'slam': {'running': False, 'saving_map': False},
    'localization': {'running': False},
    'optimization': {'running': False},
    'bag_player': {'loaded': False, 'playing': False, 'paused': False},
    'recorder': {'recording': True, 'mode': 'all'},
    'file_player': {'loaded': False, 'playing': False, 'paused': False, 'converting': False, 'saving_bag': False},
}


def fulfill_summary(route):
    route.fulfill(status=200, content_type='application/json',
                  body=json.dumps({'success': True, 'data': SUMMARY}))


failures = []
with open_page() as (page, errors):
    page.route('**/api/system/status_summary', fulfill_summary)
    tag = page.locator('[data-screen="bag-recorder-subtab"] [data-run-tag]')
    try:
        tag.wait_for(state='visible', timeout=3000)
    except Exception:
        failures.append('REC tag not visible within 3s')
    if tag.count() and tag.inner_text().strip() != 'REC':
        failures.append(f'REC tag text {tag.inner_text()!r}')
    if page.locator('[data-run-tag]:visible').count() != 1:
        failures.append('unexpected visible run tags')
    shot(page, 'p1-sidebar-markers')
    finish(failures, errors)
