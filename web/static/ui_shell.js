// ui_shell.js — 사이드바·화면 헤더·전역 상태 표시 (script.js 다음에 로드)
const uiShell = (() => {
    const SCREENS = {
        'lidar-slam-subtab':         { tab: 'slam-tab', group: 'SLAM / Localization', title: 'LiDAR SLAM' },
        'localization-subtab':       { tab: 'slam-tab', group: 'SLAM / Localization', title: 'Localization' },
        'multi-session-slam-subtab': { tab: 'slam-tab', group: 'SLAM / Localization', title: 'Multi-Session SLAM' },
        'bag-player-subtab':   { tab: 'player-tab', group: 'Data Player / Recorder', title: 'Bag Player' },
        'bag-recorder-subtab': { tab: 'player-tab', group: 'Data Player / Recorder', title: 'Bag Recorder' },
        'file-player-subtab':  { tab: 'player-tab', group: 'Data Player / Recorder', title: 'File Player' },
        'plot-subtab':         { tab: 'visualization-tab', group: 'Visualization', title: 'Plot' },
        '3d-viewer-subtab':    { tab: 'visualization-tab', group: 'Visualization', title: '3D Viewer' },
    };
    const LEGACY_NAV_KEY = 'webui.legacyNav';

    function isLegacyNav() {
        try { return localStorage.getItem(LEGACY_NAV_KEY) === '1'; } catch (e) { return false; }
    }

    function onScreenChanged(subtabId) {
        document.querySelectorAll('.ui-sidebar-item').forEach(item => {
            if (item.dataset.screen === subtabId) item.setAttribute('aria-current', 'page');
            else item.removeAttribute('aria-current');
        });
        refresh();
    }

    function showScreen(subtabId) {
        const screen = SCREENS[subtabId];
        if (!screen) return;
        lastActiveSubtab[screen.tab] = subtabId;
        openTab(screen.tab);
    }

    function refresh() {
        // Task 1.8 이후 화면별 헤더 렌더를 여기서 부른다
    }

    function init() {
        if (isLegacyNav()) return;
        document.body.classList.add('ui-has-sidebar');
        // 초기 HTML은 메인 탭마다 서브탭 하나씩 active라 보이는 탭 안의 것을 고른다
        const active = document.querySelector('.tab-content.active .subtab-content.active');
        onScreenChanged(active ? active.id : 'lidar-slam-subtab');
    }

    return { SCREENS, showScreen, onScreenChanged, refresh, init };
})();
window.uiShell = uiShell;
window.showScreen = uiShell.showScreen;
document.addEventListener('DOMContentLoaded', uiShell.init);
