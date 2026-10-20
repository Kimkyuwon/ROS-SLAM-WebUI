"""PlotJugglerTree.filterByText 검사 (서버 없이 file:// 페이지로 plot_tree.js만 로드).

사용: python3 test/ui/check_plot_tree_filter.py
"""
import os
import sys

from playwright.sync_api import sync_playwright

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TREE_JS = os.path.join(ROOT, 'web', 'static', 'plot_tree.js')
PAGE = '/tmp/check_plot_tree_filter.html'

with open(PAGE, 'w', encoding='utf-8') as f:
    f.write(f'''<!doctype html><html><body>
<div id="plot-tree"></div>
<script src="file://{TREE_JS}"></script>
</body></html>''')

PATHS = [
    'imu/data/linear_acceleration/x',
    'imu/data/linear_acceleration/y',
    'imu/data/angular_velocity/x',
    'odom/pose/position/x',
    'odom/twist/linear/x',
]

# 보이는 리프 / 보이는 노드 경로 목록 (조상이 hidden이면 보이지 않는 것으로 계산)
SNAP_JS = '''() => {
    const tree = window.__tree;
    const isVisible = (el) => {
        for (let n = el; n && n !== tree.rootNode; n = n.parentElement) {
            if (n.hidden) return false;
        }
        return true;
    };
    const nodes = Array.from(tree.container.querySelectorAll('.plot-tree-node'));
    const expanded = nodes.filter(n => n.classList.contains('plot-tree-expanded'))
        .map(n => n.dataset.path).sort();
    const leaves = nodes.filter(n => n.dataset.isLeaf === 'true' && isVisible(n))
        .map(n => n.dataset.path).sort();
    const visible = nodes.filter(isVisible).map(n => n.dataset.path).sort();
    return { leaves, visible, expanded };
}'''

errors = []
failures = []


def check(cond, msg):
    if not cond:
        failures.append(msg)


with sync_playwright() as p:
    browser = p.chromium.launch(headless=True)
    page = browser.new_page()
    page.on('pageerror', lambda e: errors.append(f'pageerror: {e}'))
    page.on('console', lambda m: errors.append(f'console.error: {m.text}') if m.type == 'error' else None)
    page.goto(f'file://{PAGE}')

    page.evaluate('''(paths) => {
        const tree = new PlotJugglerTree('plot-tree');
        tree.init();
        paths.forEach(p => tree.addItem(p, 1));
        tree.toggleExpand(tree.nodeMap.get('imu'));  // 검색 전 상태: imu만 펼침
        window.__tree = tree;
    }''', PATHS)

    before = page.evaluate(SNAP_JS)
    check(len(before['leaves']) == 5, f'초기 리프 5개 기대, 실제 {before["leaves"]}')
    check(before['expanded'] == ['imu'], f'초기 펼침 imu만 기대, 실제 {before["expanded"]}')

    page.evaluate("() => window.__tree.filterByText('linear')")
    found = page.evaluate(SNAP_JS)
    check(found['leaves'] == ['imu/data/linear_acceleration/x', 'imu/data/linear_acceleration/y',
                              'odom/twist/linear/x'], f'linear 리프 불일치: {found["leaves"]}')
    check('imu/data/angular_velocity' not in found['visible'], 'angular_velocity 가지가 보임')
    check('odom/pose' not in found['visible'], 'odom/pose 가지가 보임')
    for anc in ('imu', 'imu/data', 'imu/data/linear_acceleration', 'odom', 'odom/twist', 'odom/twist/linear'):
        check(anc in found['visible'], f'조상 {anc} 안 보임')
        check(anc in found['expanded'], f'조상 {anc} 펼쳐지지 않음')

    page.evaluate("() => window.__tree.filterByText('LINEAR')")  # 대소문자 무시
    check(page.evaluate(SNAP_JS)['leaves'] == found['leaves'], '대문자 질의 결과 불일치')

    page.evaluate("() => window.__tree.filterByText('zzz')")
    check(page.evaluate(SNAP_JS)['leaves'] == [], '매칭 없음인데 리프가 보임')

    page.evaluate("() => window.__tree.filterByText('')")
    after = page.evaluate(SNAP_JS)
    check(after['leaves'] == before['leaves'], f'복원 후 리프 불일치: {after["leaves"]}')
    check(after['visible'] == before['visible'], '복원 후 보이는 노드 불일치')
    check(after['expanded'] == before['expanded'], f'복원 후 펼침 상태 불일치: {after["expanded"]}')
    browser.close()

for e in errors:
    failures.append(e)
if failures:
    print('FAIL: ' + ' | '.join(failures))
    sys.exit(1)
print('PASS: plot_tree filterByText')
