"""openTab/openSubTab를 클릭 이벤트 없이 호출해도 동작하는지."""
from ui_check_common import open_page, finish

failures = []

with open_page() as (page, errors):
    page.evaluate("openTab('player-tab')")
    if not page.locator('[id="player-tab"]').evaluate("el => el.classList.contains('active')"):
        failures.append('player-tab not active after openTab')
    if not page.locator('[id="bag-player-subtab"]').evaluate("el => el.classList.contains('active')"):
        failures.append('bag-player-subtab not active after openTab')
    active_main = page.locator('.tab-button.active').evaluate_all("els => els.map(e => e.getAttribute('onclick'))")
    if len(active_main) != 1 or "'player-tab'" not in active_main[0]:
        failures.append(f'active tab buttons = {active_main}')

    page.evaluate("openSubTab('bag-recorder-subtab')")
    if not page.locator('[id="bag-recorder-subtab"]').evaluate("el => el.classList.contains('active')"):
        failures.append('bag-recorder-subtab not active after openSubTab')
    active_sub = page.locator('.subtab-button.active').evaluate_all("els => els.map(e => e.getAttribute('onclick'))")
    if not any("'bag-recorder-subtab'" in o for o in active_sub):
        failures.append(f'active subtab buttons = {active_sub}')

    page.evaluate("openTab('slam-tab')")
    if not page.locator('[id="lidar-slam-subtab"]').evaluate("el => el.classList.contains('active')"):
        failures.append('lidar-slam-subtab not active after openTab(slam-tab)')

finish(failures, errors)
