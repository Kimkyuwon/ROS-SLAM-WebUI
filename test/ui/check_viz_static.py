"""Plot 탭 바·메뉴·현재값·Plotly 색 정적 검사 (서버 없이 file:// 페이지에서 실행).

사용: python3 test/ui/check_viz_static.py  (워크트리 루트에서)
"""
import os
import sys
import tempfile

from playwright.sync_api import sync_playwright

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
STATIC = os.path.join(ROOT, 'web', 'static')
SHOT_DIR = '/tmp/webui_redesign_shots'

PAGE = f"""<!DOCTYPE html><html><head><meta charset="utf-8">
<link rel="stylesheet" href="file://{STATIC}/ui_theme.css">
</head><body>
<div style="width: 900px;">
  <div id="plot-tab-bar-container"></div>
  <div id="plot-area-container" style="height: 360px;"></div>
</div>
<script src="file://{STATIC}/vendor/plotly-2.27.0.min.js"></script>
<script src="file://{STATIC}/plot_manager.js"></script>
<script src="file://{STATIC}/plot_tab_manager.js"></script>
</body></html>"""

failures = []


def check(cond, msg):
    if not cond:
        failures.append(msg)


os.makedirs(SHOT_DIR, exist_ok=True)
with tempfile.NamedTemporaryFile('w', suffix='.html', delete=False, encoding='utf-8') as f:
    f.write(PAGE)
    page_path = f.name

with sync_playwright() as pw:
    browser = pw.chromium.launch(headless=True, args=['--allow-file-access-from-files'])
    page = browser.new_page(viewport={'width': 1000, 'height': 560})
    errors = []
    page.on('pageerror', lambda e: errors.append(str(e)))
    page.on('console', lambda m: errors.append(m.text) if m.type == 'error' else None)
    page.goto('file://' + page_path)

    ok = page.evaluate("""() => {
        window.mgr = new PlotTabManager('plot-tab-bar-container', 'plot-area-container', 30);
        return mgr.init();
    }""")
    check(ok, 'PlotTabManager.init() 실패')
    page.evaluate("() => mgr.createTab()")
    page.wait_for_timeout(300)

    bar_html = page.evaluate("document.getElementById('plot-tab-bar-container').outerHTML")
    check('124, 111, 255' not in bar_html, '탭 바에 보라 인라인 색이 남음')
    check('style=' not in page.evaluate("document.getElementById('plot-tab-list').innerHTML"),
          '탭 요소에 인라인 style 잔존')
    check(page.locator('.ui-plot-tab').count() == 2, f'ui-plot-tab 개수={page.locator(".ui-plot-tab").count()}')
    check(page.locator('.ui-plot-tab-active').count() == 1, 'ui-plot-tab-active는 1개여야 함')
    check(page.locator('.ui-plot-tab-close').count() == 2, 'ui-plot-tab-close 개수')
    check(page.locator('#plot-tab-add-btn.ui-plot-tab-add').count() == 1, '+ 버튼 클래스')
    page.evaluate("() => mgr.switchTab('tab-1')")
    active_id = page.evaluate("document.querySelector('.ui-plot-tab-active').id")
    check(active_id == 'plot-tab-ui-tab-1', f'활성 탭 전환 실패: {active_id}')
    check(page.locator('.ui-plot-empty').count() >= 1, '빈 상태에 ui-plot-empty 없음')
    check('rgb(255, 255, 255)' not in page.evaluate(
        "getComputedStyle(document.querySelector('.ui-plot-empty')).backgroundColor"), '빈 상태 배경이 흰색')
    page.screenshot(path=f'{SHOT_DIR}/viz_static_empty.png')

    page.evaluate("""() => {
        const tab = mgr.tabs[0];
        window.pm = tab.plotManager;
        pm.createPlot(['odom/pose/x', 'odom/pose/y']);
        for (let i = 0; i < 20; i++) {
            pm.updatePlot('odom/pose/x', 100 + i * 0.1, Math.sin(i / 3));
            pm.updatePlot('odom/pose/y', 100 + i * 0.1, Math.cos(i / 3));
        }
    }""")
    page.wait_for_timeout(300)

    layout = page.evaluate("({p: pm.container.layout.paper_bgcolor, g: pm.container.layout.xaxis.gridcolor,"
                           " t: pm.container.layout.xaxis.tickfont.color})")
    check(layout['p'].lower() == '#11151a', f"paper_bgcolor={layout['p']}")
    check(layout['g'].lower() == '#252b33', f"gridcolor={layout['g']}")
    check(layout['t'].lower() == '#8b95a1', f"tickfont={layout['t']}")

    page.evaluate("() => pm.setCurrentValues(new Map([['odom/pose/x', 4.7123], ['odom/pose/y', 1]]))")
    cur = page.evaluate("document.querySelector('.ui-plot-current-value').textContent")
    check(cur == '4.712', f'현재값 표시={cur}')
    paused_hidden = page.evaluate("document.querySelector('.ui-plot-current-paused').hidden")
    check(paused_hidden is True, '재생 중 일시정지 글자가 보임')

    page.evaluate("() => pm.togglePause(true)")
    page.wait_for_timeout(100)
    check(page.evaluate("document.querySelector('.ui-plot-current-paused').hidden") is False, '일시정지 글자 안 보임')
    page.evaluate("() => pm.setCurrentValues(new Map([['odom/pose/x', 9.9]]))")
    cur = page.evaluate("document.querySelector('.ui-plot-current-value').textContent")
    check(cur == '4.712', f'일시정지 중 값이 바뀜: {cur}')
    page.screenshot(path=f'{SHOT_DIR}/viz_static_paused.png')

    box = page.locator('#plot-area-tab-1').bounding_box()
    page.mouse.click(box['x'] + 300, box['y'] + 150, button='right')
    page.wait_for_timeout(200)
    menu = page.locator('.ui-context-menu')
    check(menu.count() >= 1, '컨텍스트 메뉴(ui-context-menu) 없음')
    if menu.count():
        bg = menu.first.evaluate('el => getComputedStyle(el).backgroundColor')
        check(bg == 'rgb(23, 28, 34)', f'메뉴 배경={bg}')
        n_inline = menu.first.evaluate('el => el.style.length')
        check(n_inline == 2, f'메뉴 인라인 속성이 left/top 2개여야 함: {n_inline}')
        check(page.locator('.ui-context-menu-item').count() >= 3, '메뉴 항목 클래스')
    page.screenshot(path=f'{SHOT_DIR}/viz_static_menu.png')

    check(not errors, f'콘솔/페이지 에러: {errors}')
    browser.close()

os.unlink(page_path)
if failures:
    print('FAIL')
    for m in failures:
        print(' -', m)
    sys.exit(1)
print('PASS')
