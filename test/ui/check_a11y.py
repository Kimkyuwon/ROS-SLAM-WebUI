"""P5 검사: 토스트·확인창·모달·반응형·포커스.

서버 필요. ROS 노드는 실행하지 않는다.
"""
import re

from playwright.sync_api import TimeoutError as PlaywrightTimeout
from ui_check_common import finish, open_page, shot

TONE_CASES = (
    ('Failed to start SLAM mapping: x', 'stop'),
    ('저장 실패', 'stop'),
    ('Topic not found', 'stop'),
    ('Please select a bag file first', 'warn'),
    ('토픽을 선택하세요', 'warn'),
    ('Map saved', 'go'),
    ('Recording started', 'go'),
    ('hello', 'info'),
)


def toast_tones(page):
    return page.evaluate("() => [...document.querySelectorAll('.ui-toast')].map(t => t.dataset.tone)")


def check_alert_shim(page, failures, native_dialogs):
    page.evaluate("() => document.querySelectorAll('.ui-toast').forEach(t => t.remove())")
    page.evaluate("() => alert('Failed to start SLAM mapping: x')")
    if native_dialogs:
        failures.append(f'a11y: native dialog shown {native_dialogs}')
    tones = toast_tones(page)
    if tones != ['stop']:
        failures.append(f'a11y: alert -> toast tones {tones} != [stop]')
    text = page.evaluate("() => document.querySelector('.ui-toast p').textContent")
    if text != 'Failed to start SLAM mapping: x':
        failures.append(f'a11y: toast text {text!r}')
    page.wait_for_timeout(300)
    shot(page, 'p5-alert-toast')
    page.evaluate("() => document.querySelectorAll('.ui-toast').forEach(t => t.remove())")
    for text, tone in TONE_CASES:
        got = page.evaluate('(t) => uiKit.inferToastTone(t)', text)
        if got != tone:
            failures.append(f'a11y: inferToastTone({text!r}) = {got!r} != {tone!r}')
    page.evaluate("() => alert(undefined)")
    if page.evaluate("() => document.querySelector('.ui-toast p').textContent") != '':
        failures.append('a11y: alert(undefined) should show empty text')
    page.evaluate("() => document.querySelectorAll('.ui-toast').forEach(t => t.remove())")


def check_confirm_dialog(page, failures):
    page.evaluate(
        """() => {
            const opener = document.createElement('button');
            opener.id = 'a11y-opener';
            document.body.appendChild(opener);
            opener.focus();
            window._a11yResult = 'pending';
            uiKit.confirmDialog({ title: 'T', message: 'line1\\nline2', okText: 'OK' })
                .then(v => { window._a11yResult = v; });
        }""")
    dialog = page.locator('.ui-dialog[role="alertdialog"]')
    if dialog.count() != 1:
        failures.append('a11y: confirmDialog did not open')
        return
    if dialog.get_attribute('aria-modal') != 'true':
        failures.append('a11y: dialog aria-modal != true')
    focused = page.evaluate("() => document.activeElement.textContent")
    if focused != '취소':
        failures.append(f'a11y: initial focus {focused!r} != 취소')
    shot(page, 'p5-confirm-dialog')
    page.keyboard.press('Escape')
    page.wait_for_function("() => window._a11yResult !== 'pending'")
    if page.evaluate('() => window._a11yResult') is not False:
        failures.append('a11y: Escape should resolve false')
    if page.evaluate("() => document.activeElement && document.activeElement.id") != 'a11y-opener':
        failures.append('a11y: focus not restored after close')
    page.evaluate(
        """() => { window._a11yResult = 'pending';
            uiKit.confirmDialog({ title: 'T', message: 'm' }).then(v => { window._a11yResult = v; }); }""")
    page.locator('.ui-dialog .ui-btn:not(.ui-btn-ghost)').click()
    page.wait_for_function("() => window._a11yResult !== 'pending'")
    if page.evaluate('() => window._a11yResult') is not True:
        failures.append('a11y: OK should resolve true')
    page.evaluate("() => document.getElementById('a11y-opener').remove()")


def check_explicit_toasts(page, failures):
    page.evaluate("() => document.querySelectorAll('.ui-toast').forEach(t => t.remove())")
    page.evaluate("() => updateBufferTime('500')")
    if toast_tones(page) != ['warn']:
        failures.append(f'a11y: invalid buffer time toast {toast_tones(page)} != [warn]')
    if page.evaluate("() => document.getElementById('buffer-time-input').value") != '5':
        failures.append('a11y: invalid buffer time did not restore 5')
    page.evaluate("() => document.querySelectorAll('.ui-toast').forEach(t => t.remove())")
    page.evaluate(
        """() => {
            const mgr = new PlotTabManager('a11y-tab-bar', 'a11y-plot-area', 5.0);
            mgr.plotAreaContainer = document.createElement('div');
            mgr.tabs.length = mgr.maxTabs;
            window._a11yTabResult = mgr.createTab();
        }""")
    if page.evaluate('() => window._a11yTabResult') is not None:
        failures.append('a11y: createTab over max should return null')
    if toast_tones(page) != ['warn']:
        failures.append(f'a11y: max tab toast {toast_tones(page)} != [warn]')
    page.evaluate("() => document.querySelectorAll('.ui-toast').forEach(t => t.remove())")


MODAL_IDS = (
    'file-browser-modal', 'bag-name-browser-modal', 'topic-selection-modal', 'plot-topic-selection-modal',
    'recorder-topic-modal', 'add-display-modal', 'save-map-modal', 'plot-settings-modal', 'filter-dialog-modal',
)


def check_modals(page, failures):
    info = page.evaluate(
        """() => [...document.querySelectorAll('.modal')].map(m => {
            const label = document.getElementById(m.getAttribute('aria-labelledby') || '');
            const close = m.querySelector('.modal-header .close');
            return { id: m.id, role: m.getAttribute('role'), modal: m.getAttribute('aria-modal'),
                label: label ? label.textContent.trim() : '',
                close: close ? [close.getAttribute('role'), close.getAttribute('aria-label'), close.tabIndex] : null };
        })""")
    ids = [m['id'] for m in info]
    if sorted(ids) != sorted(MODAL_IDS):
        failures.append(f'a11y: modal set changed {ids}')
    for m in info:
        if m['role'] != 'dialog' or m['modal'] != 'true':
            failures.append(f"a11y: #{m['id']} role={m['role']} aria-modal={m['modal']}")
        if not m['label']:
            failures.append(f"a11y: #{m['id']} aria-labelledby missing")
        if m['close'] != ['button', '닫기', 0]:
            failures.append(f"a11y: #{m['id']} close {m['close']}")

    page.evaluate(
        """() => { const b = document.createElement('button'); b.id = 'a11y-modal-opener'; b.textContent = 'open';
            document.querySelector('.ui-main').prepend(b); b.focus(); openFileBrowser(() => {}, '~'); }""")
    page.wait_for_function("() => document.getElementById('file-browser-modal').style.display !== 'none'")
    page.wait_for_timeout(100)
    if not page.evaluate("() => document.getElementById('file-browser-modal').contains(document.activeElement)"):
        failures.append('a11y: focus did not move into #file-browser-modal on open')
    page.keyboard.press('Escape')
    page.wait_for_timeout(100)
    if page.evaluate("() => document.getElementById('file-browser-modal').style.display") != 'none':
        failures.append('a11y: Escape did not close #file-browser-modal')
    if page.evaluate("() => document.activeElement && document.activeElement.id") != 'a11y-modal-opener':
        failures.append('a11y: focus not restored to opener after modal close')
    page.evaluate("() => document.getElementById('a11y-modal-opener').remove()")

    page.evaluate("() => openFileBrowser(() => {}, '~')")
    page.wait_for_function("() => document.getElementById('file-browser-modal').style.display !== 'none'")
    page.locator('#file-browser-modal .modal-header .close').focus()
    page.keyboard.press('Enter')
    page.wait_for_timeout(100)
    if page.evaluate("() => document.getElementById('file-browser-modal').style.display") != 'none':
        failures.append('a11y: Enter on close did not close #file-browser-modal')

    for modal_id in MODAL_IDS:
        page.evaluate("(id) => { document.getElementById(id).style.display = 'block'; }", modal_id)
        width = page.evaluate(
            "(id) => document.querySelector(`#${id} .modal-content`).getBoundingClientRect().width", modal_id)
        if width < 300:
            failures.append(f'a11y: #{modal_id} content width {width}')
        shot(page, f'p5-modal-{modal_id}')
        page.evaluate("(id) => { document.getElementById(id).style.display = 'none'; }", modal_id)


VIEWPORTS = ((1440, 900), (1280, 800), (1024, 768), (900, 700))
CANVAS_SCREENS = (('lidar-slam-subtab', '#lidar-slam-viewer-empty'),
                  ('3d-viewer-subtab', '[id="3d-viewer-container"] canvas'))


def sidebar_box(page):
    return page.evaluate(
        """() => {
            const s = document.getElementById('ui-sidebar').getBoundingClientRect();
            const t = document.getElementById('ui-sidebar-toggle');
            const tr = t.getBoundingClientRect();
            const visible = (r) => r.width > 0 && r.height > 0 && r.right > 0;
            return { width: s.width, sidebarVisible: visible(s), toggleVisible: visible(tr),
                expanded: t.getAttribute('aria-expanded'),
                docOverflow: document.documentElement.scrollWidth - window.innerWidth };
        }""")


def check_canvas_fill(page, failures, width, screen, selector):
    page.evaluate('(id) => showScreen(id)', screen)
    page.wait_for_timeout(400)
    try:
        page.wait_for_selector(selector, state='visible', timeout=10000)
    except PlaywrightTimeout:
        pass
    box = page.evaluate(
        """(sel) => { const el = [...document.querySelectorAll(sel)].find(e => e.offsetParent !== null);
            if (!el) return null; const r = el.getBoundingClientRect();
            const main = document.querySelector('.ui-main').getBoundingClientRect();
            return [r.left, r.width, main.left, main.width]; }""", selector)
    if not box:
        failures.append(f'a11y: {width}px {screen} canvas not visible')
        return
    left, w, main_left, main_w = box
    if w < main_w * 0.4 or left + w > main_left + main_w + 1:
        failures.append(f'a11y: {width}px {screen} canvas box {box}')
    shot(page, f'p5-responsive-{width}-{screen}')


def check_responsive(page, failures):
    for width, height in VIEWPORTS:
        page.set_viewport_size({'width': width, 'height': height})
        page.wait_for_timeout(300)
        box = sidebar_box(page)
        if width >= 1280 and (box['width'] != 244 or box['toggleVisible']):
            failures.append(f'a11y: {width}px sidebar {box}')
        if width == 1024 and (box['width'] != 64 or box['toggleVisible']):
            failures.append(f'a11y: 1024px sidebar rail {box}')
        if width < 1024 and (box['sidebarVisible'] or not box['toggleVisible']):
            failures.append(f'a11y: {width}px sidebar should hide behind toggle {box}')
        if box['docOverflow'] > 0:
            failures.append(f'a11y: {width}px horizontal overflow {box["docOverflow"]}px')
        for screen, selector in CANVAS_SCREENS:
            check_canvas_fill(page, failures, width, screen, selector)
        if width == 1024:
            names = page.evaluate(
                """() => [...document.querySelectorAll('.ui-sidebar-item')].map(b => [b.title, b.innerText.trim()])""")
            if any(not title for title, _ in names):
                failures.append(f'a11y: rail items without title {names}')
    page.locator('#ui-sidebar-toggle').click()
    page.wait_for_timeout(200)
    box = sidebar_box(page)
    if not box['sidebarVisible'] or box['expanded'] != 'true':
        failures.append(f'a11y: toggle did not open sidebar {box}')
    shot(page, 'p5-responsive-900-open')
    page.locator('.ui-sidebar-item[data-screen="plot-subtab"]').click()
    page.wait_for_timeout(200)
    box = sidebar_box(page)
    if box['sidebarVisible'] or box['expanded'] != 'false':
        failures.append(f'a11y: sidebar item click did not close overlay {box}')
    if not page.evaluate("() => document.getElementById('plot-subtab').classList.contains('active')"):
        failures.append('a11y: sidebar item in overlay did not switch screen')
    page.locator('#ui-sidebar-toggle').click()
    page.keyboard.press('Escape')
    page.wait_for_timeout(200)
    if sidebar_box(page)['sidebarVisible']:
        failures.append('a11y: Escape did not close sidebar overlay')
    page.set_viewport_size({'width': 1440, 'height': 900})


SCREENS = ('lidar-slam-subtab', 'localization-subtab', 'multi-session-slam-subtab', 'bag-player-subtab',
           'bag-recorder-subtab', 'file-player-subtab', 'plot-subtab', '3d-viewer-subtab')


def unnamed_buttons(page):
    return page.evaluate(
        """() => [...document.querySelectorAll('button, [role="button"]')]
            .filter(el => el.offsetParent !== null || getComputedStyle(el).position === 'fixed')
            .filter(el => el.getBoundingClientRect().width > 0)
            .filter(el => !((el.getAttribute('aria-label') || '').trim() || (el.title || '').trim()
                || (el.innerText || el.textContent || '').trim()
                || (el.getAttribute('aria-labelledby') && document.getElementById(el.getAttribute('aria-labelledby')))))
            .map(el => el.id ? '#' + el.id : el.outerHTML.slice(0, 100))""")


def symbol_only_buttons(page):
    # 글자가 ▶ × ⛶ 같은 기호뿐이면 화면 읽기 프로그램이 의미 없는 이름을 읽는다
    return page.evaluate(
        """() => [...document.querySelectorAll('button, [role="button"]')]
            .filter(el => el.offsetParent !== null && el.getBoundingClientRect().width > 0)
            .filter(el => !(el.getAttribute('aria-label') || '').trim())
            .filter(el => { const t = (el.innerText || '').trim(); return t && !/[\\p{L}\\p{N}]/u.test(t); })
            .map(el => (el.id ? '#' + el.id : el.outerHTML.slice(0, 100))
                + ' ' + JSON.stringify(el.innerText.trim()))""")


def check_button_names(page, failures):
    for screen in SCREENS:
        page.evaluate('(id) => showScreen(id)', screen)
        page.wait_for_timeout(500)
        bad = unnamed_buttons(page)
        if bad:
            failures.append(f'a11y: {screen} unnamed buttons {bad}')
        symbols = symbol_only_buttons(page)
        if symbols:
            failures.append(f'a11y: {screen} symbol-only button names {symbols}')


def check_focus_ring(page, failures):
    page.evaluate("() => showScreen('plot-subtab')")
    page.wait_for_timeout(300)
    page.locator('#plot-window-seg [data-sec="10"]').focus()
    page.mouse.click(5, 5)
    page.locator('.ui-sidebar-item[data-screen="plot-subtab"]').focus()
    for _ in range(3):
        page.keyboard.press('Tab')
    style = page.evaluate(
        """() => { const el = document.activeElement; const cs = getComputedStyle(el);
            const name = el.tagName + (el.id ? '#' + el.id : '') + '.' + el.className;
            return [name, cs.outlineStyle, cs.outlineWidth]; }""")
    if style[1] == 'none' or style[2] in ('0px', ''):
        failures.append(f'a11y: no focus-visible outline on {style}')
    shot(page, 'p5-focus-ring')


def check_reduced_motion(page, failures):
    page.emulate_media(reduced_motion='reduce')
    anim = page.evaluate(
        """() => {
            const tag = document.querySelector('.ui-run-tag');
            const dot = document.createElement('span');
            dot.className = 'ui-dot';
            tag.hidden = false;
            tag.appendChild(dot);
            const probes = [dot, ...document.querySelectorAll('.ui-status-badge .ui-dot, .ui-toast')];
            const toast = uiKit.showToast('motion probe', 'info');
            probes.push(toast);
            const names = probes.map(el => getComputedStyle(el).animationName).filter(n => n && n !== 'none');
            dot.remove(); tag.hidden = true; toast.remove();
            return names;
        }""")
    if anim:
        failures.append(f'a11y: animations under prefers-reduced-motion {sorted(set(anim))}')
    page.emulate_media(reduced_motion='no-preference')


def check_no_native_confirm(page, failures):
    source = page.evaluate("async () => (await fetch('/static/script.js')).text()")
    calls = [m.group(0) for m in re.finditer(r'(?<![\w.])confirm\(', source)]
    if calls:
        failures.append(f'a11y: script.js still calls native confirm() {len(calls)}x')


def main():
    failures = []
    native_dialogs = []
    with open_page() as (page, errors):
        page.on('dialog', lambda d: (native_dialogs.append(d.message), d.dismiss()))
        check_alert_shim(page, failures, native_dialogs)
        check_confirm_dialog(page, failures)
        check_no_native_confirm(page, failures)
        check_explicit_toasts(page, failures)
        check_modals(page, failures)
        check_responsive(page, failures)
        check_button_names(page, failures)
        check_focus_ring(page, failures)
        check_reduced_motion(page, failures)
        if native_dialogs:
            failures.append(f'a11y: native dialogs {native_dialogs}')
        # check_explicit_toasts가 일부러 넣은 잘못된 버퍼 값 로그
        errors[:] = [e for e in errors if '[updateBufferTime] Invalid' not in e]
        finish(failures, errors)


if __name__ == '__main__':
    main()
