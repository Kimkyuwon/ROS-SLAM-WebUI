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

    // 사이드바 표시 → 헤더 배지 글자 (P2·P3가 화면별 정밀 렌더로 교체)
    const HEADER_BADGE_TEXT = {
        RUN: 'Running', REC: 'Recording', PLAY: 'Playing', PAUSE: 'Paused', SAVE: 'Saving', OPT: 'Optimizing',
    };

    function refresh() {
        const screenEl = document.querySelector('.tab-content.active .subtab-content.active');
        if (!screenEl) return;
        const badge = screenEl.querySelector('.ui-screen-header .ui-status-badge');
        const markers = lastSummary ? UiStatusLogic.deriveSidebarMarkers(lastSummary) : {};
        const marker = markers[screenEl.id];
        if (marker) uiKit.setStatusBadge(badge, marker.tone, HEADER_BADGE_TEXT[marker.text] || marker.text);
        else uiKit.setStatusBadge(badge, 'idle', 'Ready');
    }

    const ROSBRIDGE_TEXT = {
        connected: ['go', 'rosbridge 연결됨'],
        disconnected: ['stop', 'rosbridge 오류'],
        reconnecting: ['warn', 'rosbridge 재연결 중…'],
    };

    function onRosbridgeState(state) {
        const conf = ROSBRIDGE_TEXT[state];
        if (conf) uiKit.setStatusBadge(document.getElementById('ui-conn-rosbridge'), conf[0], conf[1]);
    }

    function onLatency(ms) {
        const el = document.getElementById('ui-conn-latency');
        if (!el) return;
        el.textContent = (ms == null || !isFinite(ms)) ? 'N/A' : `${Math.round(ms)} ms`;
        el.dataset.tone = (ms == null) ? 'idle' : (ms < 50 ? 'go' : (ms < 150 ? 'warn' : 'stop'));
    }

    function onDomainId(id) {
        const el = document.getElementById('ui-conn-domain');
        if (el) el.textContent = String(id);
    }

    function onBackendWsState(state) {
        const el = document.getElementById('ui-conn-port-pc2');
        const ports = window.ROS_SLAM_WEBUI || {};
        if (el) el.textContent = `${ports.pc2WsPort || '-'}${state === 'connected' ? '' : ' (끊김)'}`;
    }

    function renderPorts() {
        const ports = window.ROS_SLAM_WEBUI || {};
        const set = (id, v) => { const el = document.getElementById(id); if (el) el.textContent = v ?? '-'; };
        set('ui-conn-port-web', ports.webPort);
        set('ui-conn-port-pc2', ports.pc2WsPort);
        set('ui-conn-port-rosbridge', ports.rosbridgePort);
    }

    const SUMMARY_POLL_MS = 2000;
    let lastSummary = null;

    function renderSidebarMarkers(data) {
        const markers = UiStatusLogic.deriveSidebarMarkers(data);
        document.querySelectorAll('.ui-sidebar-item').forEach(item => {
            const tag = item.querySelector('[data-run-tag]');
            const marker = markers[item.dataset.screen];
            tag.hidden = !marker;
            tag.textContent = marker ? marker.text : '';
            tag.dataset.tone = marker ? marker.tone : 'idle';
        });
    }

    async function pollSummary() {
        const result = await apiCall('/api/system/status_summary');
        if (result && result.success && result.data) {
            lastSummary = result.data;
            renderSidebarMarkers(lastSummary);
            refresh();
        }
    }

    function init() {
        if (isLegacyNav()) return;
        document.body.classList.add('ui-has-sidebar');
        // 초기 HTML은 메인 탭마다 서브탭 하나씩 active라 보이는 탭 안의 것을 고른다
        const active = document.querySelector('.tab-content.active .subtab-content.active');
        onScreenChanged(active ? active.id : 'lidar-slam-subtab');
        renderPorts();
        // webui_ports.js가 document에 bubbles 없이 보내므로 window가 아니라 document에서 받는다
        document.addEventListener('ros-slam-webui-ports-ready', renderPorts);
        pollSummary();
        setInterval(pollSummary, SUMMARY_POLL_MS);
    }

    return {
        SCREENS, showScreen, onScreenChanged, refresh, init,
        onRosbridgeState, onLatency, onDomainId, onBackendWsState,
        get lastSummary() { return lastSummary; },
    };
})();
window.uiShell = uiShell;
window.showScreen = uiShell.showScreen;
document.addEventListener('DOMContentLoaded', uiShell.init);
