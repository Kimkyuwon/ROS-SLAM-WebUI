"""ui-* 컴포넌트 클래스 스타일 확인 (샘플 마크업 주입)."""
from ui_check_common import finish, open_page, shot

SAMPLE = '''
<div id="ui-sample" style="position:fixed;top:8px;left:8px;z-index:9999;display:flex;gap:8px;padding:8px;background:#0B0E11">
  <button class="ui-btn ui-btn-go">Start</button>
  <button class="ui-btn ui-btn-stop">Stop</button>
  <button class="ui-btn" disabled>Save Map</button>
  <button class="ui-icon-btn" aria-label="Reset" aria-pressed="true">R</button>
  <span class="ui-status-badge" data-tone="go"><span class="ui-dot"></span>Running</span>
</div>'''

# 나머지 클래스 모음. 스크린샷으로 눈 확인하고, 아래 수치로 핵심 값을 검사한다.
SHOWCASE = '''
<div id="ui-showcase" style="position:fixed;top:72px;left:8px;right:8px;z-index:9998;display:grid;
     grid-template-columns:repeat(3,minmax(0,1fr));gap:12px;padding:12px;background:#0B0E11;align-items:start">
  <section class="ui-panel" id="sc-panel">
    <header class="ui-panel-header"><h3 class="ui-panel-title">Analytics</h3><span class="ui-tag" data-tone="info">ROS2</span></header>
    <div class="ui-panel-body">
      <div class="ui-kpi-grid">
        <div class="ui-kpi" data-tone="go"><span class="ui-kpi-label">IMU Hz</span><span class="ui-kpi-hint">OK</span>
          <span class="ui-kpi-value" id="sc-kpi-value">199.8</span><span class="ui-kpi-unit">Hz</span>
          <svg class="ui-sparkline" viewBox="0 0 100 30" preserveAspectRatio="none"><polyline points="0,20 30,10 60,18 100,5"/></svg></div>
        <div class="ui-kpi"><span class="ui-kpi-label">CPU</span><span class="ui-kpi-value">42.1</span><span class="ui-kpi-unit">%</span>
          <span class="ui-kpi-sub">3.2 cores</span></div>
      </div>
      <ol class="ui-stepper" id="sc-stepper" style="margin-top:12px">
        <li class="ui-stepper-item" data-state="done"><span class="ui-stepper-mark">&#10003;</span>Config</li>
        <li class="ui-stepper-item" data-state="current" aria-current="step"><span class="ui-stepper-mark">2</span>Run</li>
        <li class="ui-stepper-item" data-state="todo"><span class="ui-stepper-mark">3</span>Save</li>
      </ol>
      <div style="margin-top:12px;display:flex;gap:8px;align-items:center">
        <span class="ui-status-badge" data-tone="stop"><span class="ui-dot"></span>REC 00:12</span>
        <span class="ui-status-badge" data-tone="warn"><span class="ui-dot"></span>Delay</span>
        <span class="ui-status-badge" data-tone="idle"><span class="ui-dot"></span>Idle</span>
        <span class="ui-run-tag" data-tone="go">RUN</span>
      </div>
    </div>
    <footer class="ui-panel-footer"><button class="ui-btn ui-btn-ghost">Select all</button><button class="ui-btn ui-btn-record">Record</button></footer>
  </section>
  <section class="ui-panel">
    <div class="ui-panel-body" style="display:flex;flex-direction:column;gap:12px">
      <div class="ui-seg" id="sc-seg" role="radiogroup"><button class="ui-seg-option" role="radio" aria-checked="true">10 s</button><button class="ui-seg-option" role="radio" aria-checked="false">30 s</button></div>
      <label class="toggle-switch ui-toggle" id="sc-toggle"><input type="checkbox" checked><span class="toggle-track"><span class="toggle-thumb"></span></span>Loop</label>
      <div class="ui-field"><span class="ui-field-label">Output</span><input class="ui-input" id="sc-input" value="/data/result"><span class="ui-hint">mapping_config.yaml에 적용됩니다</span></div>
      <select class="ui-select" id="sc-select"><option>PCD</option><option>BAG</option></select>
      <div class="ui-progress" id="sc-progress"><div class="ui-progress-fill" style="width:40%"></div></div>
      <div class="ui-progress ui-progress-indeterminate"></div>
      <ul class="ui-progress-list"><li data-state="done">Load</li><li data-state="current">Optimize</li><li data-state="todo">Write</li></ul>
      <details class="ui-more" open><summary>세밀 조절</summary><span class="ui-hint">slider</span></details>
      <input type="checkbox" class="ui-eye-toggle" id="sc-eye" checked><input type="checkbox" class="ui-eye-toggle" id="sc-eye-off">
    </div>
  </section>
  <section class="ui-panel">
    <table class="ui-table" id="sc-table"><thead><tr><th scope="col">Topic</th><th scope="col" class="ui-num">Hz</th></tr></thead>
      <tbody><tr><td>/lidar</td><td class="ui-num">10.0</td></tr><tr data-disabled><td>/imu</td><td class="ui-num">200.0</td></tr></tbody></table>
    <div class="ui-viewer-empty" id="sc-empty" style="min-height:140px"><p class="ui-viewer-empty-eyebrow">LIVE VIEWER</p>
      <p class="ui-viewer-empty-title">Start SLAM을 누르면 지도가 그려집니다</p></div>
    <div class="ui-plot-tabs"><button class="ui-plot-tab ui-plot-tab-active" id="sc-plot-tab">Plot 1</button><button class="ui-plot-tab">Plot 2</button><button class="ui-plot-tab-add">+</button></div>
  </section>
</div>
<div class="ui-toast-stack" id="sc-toast-stack"><div class="ui-toast" data-tone="go" id="sc-toast"><p>저장 완료</p><button class="ui-toast-close" aria-label="알림 닫기">&times;</button></div>
  <div class="ui-toast" data-tone="stop"><p>저장 실패</p><button class="ui-toast-close" aria-label="알림 닫기">&times;</button></div></div>'''


def style(page, selector, prop):
    return page.locator(selector).first.evaluate(f"e => getComputedStyle(e).{prop}")


def expect(failures, label, actual, expected):
    if actual != expected:
        failures.append(f'{label}: {actual!r} != {expected!r}')


failures = []
with open_page() as (page, errors):
    page.evaluate(f"document.body.insertAdjacentHTML('beforeend', `{SAMPLE}`)")
    go = page.locator('#ui-sample .ui-btn-go')
    if go.evaluate("e => getComputedStyle(e).backgroundColor") != 'rgb(63, 208, 122)':
        failures.append('ui-btn-go bg')
    if go.evaluate("e => getComputedStyle(e).height") != '40px':
        failures.append('ui-btn height')
    if page.locator('#ui-sample .ui-btn[disabled]').evaluate("e => getComputedStyle(e).opacity") != '0.35':
        failures.append('disabled opacity')

    page.evaluate(f"document.body.insertAdjacentHTML('beforeend', `{SHOWCASE}`)")
    expect(failures, 'panel border-radius', style(page, '#sc-panel', 'borderTopLeftRadius'), '6px')
    expect(failures, 'kpi value font', 'Fira Code' in style(page, '#sc-kpi-value', 'fontFamily'), True)
    expect(failures, 'kpi value size', style(page, '#sc-kpi-value', 'fontSize'), '20px')
    expect(failures, 'kpi value tone', style(page, '#sc-kpi-value', 'color'), 'rgb(63, 208, 122)')
    expect(failures, 'badge height', style(page, '.ui-status-badge', 'height'), '28px')
    expect(failures, 'dot size', style(page, '.ui-status-badge .ui-dot', 'width'), '8px')
    expect(failures, 'dot color', style(page, '.ui-status-badge .ui-dot', 'backgroundColor'), 'rgb(63, 208, 122)')
    expect(failures, 'dot pulse', style(page, '.ui-status-badge[data-tone=go] .ui-dot', 'animationName'), 'ui-pulse')
    expect(failures, 'stepper current color', style(page, '#sc-stepper [data-state=current]', 'color'), 'rgb(108, 182, 255)')
    expect(failures, 'stepper item height', style(page, '#sc-stepper [data-state=current]', 'height'), '28px')
    expect(failures, 'stepper done mark bg', style(page, '#sc-stepper [data-state=done] .ui-stepper-mark', 'backgroundColor'),
           'rgb(63, 208, 122)')
    expect(failures, 'seg option height', style(page, '#sc-seg [aria-checked=true]', 'height'), '32px')
    expect(failures, 'seg checked bg', style(page, '#sc-seg [aria-checked=true]', 'backgroundColor'), 'rgb(23, 28, 34)')
    expect(failures, 'toggle on bg', style(page, '#sc-toggle .toggle-track', 'backgroundColor'), 'rgb(108, 182, 255)')
    expect(failures, 'input height', style(page, '#sc-input', 'height'), '40px')
    expect(failures, 'input font', 'Fira Code' in style(page, '#sc-input', 'fontFamily'), True)
    expect(failures, 'select height', style(page, '#sc-select', 'height'), '40px')
    expect(failures, 'progress height', style(page, '#sc-progress', 'height'), '4px')
    expect(failures, 'table row height', style(page, '#sc-table tbody tr:first-child td', 'height'), '40px')
    expect(failures, 'table disabled row', style(page, '#sc-table tr[data-disabled]', 'opacity'), '0.45')
    expect(failures, 'empty border', style(page, '#sc-empty', 'borderTopStyle'), 'dashed')
    expect(failures, 'toast bg', style(page, '#sc-toast', 'backgroundColor'), 'rgb(23, 28, 34)')
    expect(failures, 'toast stack width', style(page, '#sc-toast-stack', 'width'), '320px')
    expect(failures, 'plot tab height', style(page, '#sc-plot-tab', 'height'), '32px')
    expect(failures, 'plot tab active line', style(page, '#sc-plot-tab', 'borderBottomColor'), 'rgb(108, 182, 255)')
    expect(failures, 'eye on mask', 'data:image' in style(page, '#sc-eye', 'webkitMaskImage'), True)
    expect(failures, 'eye off mask differs',
           style(page, '#sc-eye', 'webkitMaskImage') != style(page, '#sc-eye-off', 'webkitMaskImage'), True)

    page.locator('#sc-input').focus()
    expect(failures, 'input focus border', style(page, '#sc-input', 'borderTopColor'), 'rgb(108, 182, 255)')
    page.locator('#ui-sample .ui-btn-go').focus()
    page.keyboard.press('Tab')
    page.keyboard.press('Shift+Tab')
    expect(failures, 'focus-visible outline', style(page, '#ui-sample .ui-btn-go', 'outlineStyle'), 'solid')
    page.wait_for_timeout(500)
    shot(page, 'p0-components')
    finish(failures, errors)
