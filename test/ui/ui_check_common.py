"""ros_slam_webui UI 검증 공용 헬퍼 (Playwright, 헤드리스 Chromium).

서버는 미리 띄워 둔다. networkidle은 기다리지 않는다 (WS·폴링 때문에 끝나지 않음).
"""
import os
import sys
from contextlib import contextmanager

from playwright.sync_api import sync_playwright

WEBUI_URL = os.environ.get('WEBUI_URL', 'http://localhost:8080')
SHOT_DIR = os.environ.get('WEBUI_SHOT_DIR', '/tmp/webui_redesign_shots')
IGNORED_ERROR_PATTERNS = ('WebSocket', 'rosbridge', 'ERR_CONNECTION_REFUSED', ':9090', ':8081')

SCREEN_TABS = {
    'lidar-slam-subtab': 'slam-tab', 'localization-subtab': 'slam-tab',
    'multi-session-slam-subtab': 'slam-tab', 'bag-player-subtab': 'player-tab',
    'bag-recorder-subtab': 'player-tab', 'file-player-subtab': 'player-tab',
    'plot-subtab': 'visualization-tab', '3d-viewer-subtab': 'visualization-tab',
}


@contextmanager
def open_page(width=1440, height=900):
    os.makedirs(SHOT_DIR, exist_ok=True)
    errors = []
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        page = browser.new_page(viewport={'width': width, 'height': height})
        page.on('console', lambda m: errors.append(m.text) if m.type == 'error' else None)
        page.on('pageerror', lambda e: errors.append(f'pageerror: {e}'))
        page.goto(WEBUI_URL, wait_until='load')
        page.wait_for_selector('#slam-tab')
        try:
            yield page, errors
        finally:
            browser.close()


def go_screen(page, subtab_id):
    sidebar_item = page.locator(f'.ui-sidebar-item[data-screen="{subtab_id}"]')
    if sidebar_item.count() and sidebar_item.first.is_visible():
        sidebar_item.first.click()
    else:
        page.locator(f'.tab-button[onclick*="{SCREEN_TABS[subtab_id]}"]').click()
        page.locator(f'.subtab-button[onclick*="{subtab_id}"]').click()
    page.wait_for_selector(f'[id="{subtab_id}"].active')


def shot(page, name):
    path = os.path.join(SHOT_DIR, f'{name}.png')
    page.screenshot(path=path)
    return path


def new_js_errors(errors):
    return [e for e in errors if not any(p in e for p in IGNORED_ERROR_PATTERNS)]


def finish(failures, errors):
    js_errors = new_js_errors(errors)
    for e in js_errors:
        print('JS ERROR:', e)
    for f in failures:
        print('FAIL:', f)
    if failures or js_errors:
        sys.exit(1)
    print('PASS')
