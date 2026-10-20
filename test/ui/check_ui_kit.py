"""uiKit 헬퍼 검증: 토스트 개수·자동 닫힘, 단계 표시줄, 배지, 버튼 라벨, 잠금."""
from ui_check_common import open_page, finish

failures = []

with open_page() as (page, errors):
    page.wait_for_function('typeof uiKit !== "undefined" && typeof UiStatusLogic !== "undefined"', timeout=5000)

    page.evaluate("uiKit.showToast('저장 완료', 'go', { timeoutMs: 500 })")
    n = page.locator('.ui-toast').count()
    if n != 1:
        failures.append(f'toast count after go = {n}')
    page.wait_for_timeout(900)
    n = page.locator('.ui-toast').count()
    if n != 0:
        failures.append(f'go toast should auto close, count = {n}')

    page.evaluate("uiKit.showToast('실패', 'stop')")
    page.evaluate("uiKit.showToast('안내', 'info', { timeoutMs: 60000 })")
    page.wait_for_timeout(6000)
    n = page.locator('.ui-toast').count()
    if n != 2:
        failures.append(f'stop toast must stay (expected 2), count = {n}')
    stack_attrs = page.evaluate(
        "(() => { const s = document.getElementById('ui-toast-stack');"
        " return [s.getAttribute('aria-live'), s.getAttribute('role')]; })()")
    if stack_attrs != ['polite', 'status']:
        failures.append(f'toast stack aria = {stack_attrs}')
    if page.locator('.ui-toast[data-tone="stop"]').get_attribute('role') != 'alert':
        failures.append('stop toast role != alert')

    page.evaluate("for (let i = 0; i < 6; i++) uiKit.showToast('t' + i, 'stop')")
    n = page.locator('.ui-toast').count()
    if n != 4:
        failures.append(f'toast limit 4, count = {n}')
    page.locator('.ui-toast-close').first.click()
    if page.locator('.ui-toast').count() != 3:
        failures.append('toast close button did not remove toast')

    page.evaluate("""(() => {
        const ol = document.createElement('ol');
        ol.id = 'kit-stepper';
        document.body.appendChild(ol);
        uiKit.renderStepper(ol, ['A', 'B', 'C', 'D'], 2);
    })()""")
    cur = page.locator('#kit-stepper [aria-current="step"]')
    if cur.count() != 1 or 'C' not in cur.inner_text():
        failures.append('stepper current step is not the 3rd item')
    states = page.eval_on_selector_all('#kit-stepper li', 'els => els.map(e => e.dataset.state)')
    if states != ['done', 'done', 'current', 'todo']:
        failures.append(f'stepper states = {states}')

    result = page.evaluate("""(() => {
        const badge = document.createElement('span');
        badge.innerHTML = '<span class="ui-status-text">IDLE</span>';
        uiKit.setStatusBadge(badge, 'go', 'RUN');
        const btn = document.createElement('button');
        btn.innerHTML = '<span class="ui-btn-label">Start</span>';
        uiKit.setBtnLabel(btn, 'Stop');
        const plain = document.createElement('button');
        plain.textContent = 'x';
        uiKit.setBtnLabel(plain, 'y');
        uiKit.setEnabled(plain, false, '이유');
        const box = document.createElement('div');
        box.innerHTML = '<input><select></select>';
        uiKit.setReadonly(box, true);
        return {
            tone: badge.dataset.tone, text: badge.textContent, label: btn.textContent,
            plain: plain.textContent, disabled: plain.disabled, title: plain.title,
            ro: box.dataset.readonly, inputDisabled: box.querySelector('input').disabled,
        };
    })()""")
    expected = {'tone': 'go', 'text': 'RUN', 'label': 'Stop', 'plain': 'y', 'disabled': True,
                'title': '이유', 'ro': 'true', 'inputDisabled': True}
    if result != expected:
        failures.append(f'helpers = {result}')

finish(failures, errors)
