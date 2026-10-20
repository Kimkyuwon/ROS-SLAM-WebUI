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

    const SLAM_KF_COUNT_ID = 'ad-kf_count';
    const ANALYTICS_STALE_MS = 3000;
    const lastAnalyticsAt = { slam: 0, loc: 0 };
    let lastSlamState = null;

    function readNumber(id) {
        const el = document.getElementById(id);
        if (!el) return null;
        const n = parseFloat(el.textContent);
        return Number.isFinite(n) ? n : null;
    }

    function noteAnalytics(kind) { lastAnalyticsAt[kind] = performance.now(); }

    // 첫 메시지 전(0)에는 stale로 보지 않는다 — Start 직후 "데이터 없음"이 바로 뜨지 않게
    function isAnalyticsStale(kind) {
        return lastAnalyticsAt[kind] > 0 && performance.now() - lastAnalyticsAt[kind] > ANALYTICS_STALE_MS;
    }

    function onSlamState(state) {
        lastSlamState = state;
        refresh();
    }

    function saveMapLockReason(input) {
        if (!input.isRunning) return 'SLAM 실행 중에만 저장할 수 있습니다';
        if (input.stopping) return 'SLAM을 종료하는 중입니다';
        if (input.saving) return '지도를 저장하는 중입니다';
        if (!(input.kfCount > 0)) return '키프레임이 아직 없습니다';
        return '데이터 수신이 멈춰 저장할 수 없습니다';
    }

    const SLAM_EMPTY_TITLE_STARTING = 'rosbridge 연결 대기 중…';
    let slamEmptyTitleDefault = null;
    let saveStartedAt = 0;
    let saveElapsedTimer = null;

    function renderSaveElapsed() {
        const el = document.getElementById('slam-save-map-elapsed');
        if (el) el.textContent = UiStatusLogic.formatElapsed((performance.now() - saveStartedAt) / 1000);
    }

    // 화면과 상관없이 돈다 — 저장 중 다른 화면에 갔다 와도 경과 시간이 이어지게
    function syncSaveElapsed() {
        const saving = Boolean(window._slamSaving);
        if (saving && !saveElapsedTimer) {
            saveStartedAt = performance.now();
            renderSaveElapsed();
            saveElapsedTimer = setInterval(renderSaveElapsed, 1000);
        } else if (!saving && saveElapsedTimer) {
            clearInterval(saveElapsedTimer);
            saveElapsedTimer = null;
        }
    }

    function renderSlamEmpty(empty, input, liveEl, resultEl) {
        empty.hidden = (liveEl && liveEl.style.display !== 'none')
            || (resultEl && resultEl.style.display !== 'none');
        // 저장 중에는 같은 자리의 저장 카드가 글자를 덮으므로 격자만 남긴다
        empty.dataset.state = input.saving ? 'saving' : '';
        const title = empty.querySelector('.ui-viewer-empty-title');
        if (!title) return;
        if (slamEmptyTitleDefault === null) slamEmptyTitleDefault = title.textContent;
        title.textContent = input.starting ? SLAM_EMPTY_TITLE_STARTING : slamEmptyTitleDefault;
    }

    function renderSlam() {
        const resultEl = document.getElementById('savemap-result-viewer');
        const liveEl = document.getElementById('slam-live-viewer');
        const runTime = document.getElementById('analytics-run-time');
        const input = {
            isRunning: Boolean(lastSlamState && lastSlamState.is_running),
            starting: Boolean(window._slamLiveViewerHoldOpen),
            stopping: Boolean(window._slamStopping),
            saving: Boolean(window._slamSaving),
            justSaved: Boolean(window._slamMapJustSaved),
            resultVisible: Boolean(resultEl && resultEl.style.display !== 'none'),
            elapsed: runTime && /\d/.test(runTime.textContent) ? runTime.textContent.trim() : '',
            kfCount: readNumber(SLAM_KF_COUNT_ID),
            stale: isAnalyticsStale('slam'),
            imuHz: readNumber('analytics-imu-hz'),
            lidHz: readNumber('analytics-lid-hz'),
        };
        const v = UiStatusLogic.deriveSlamView(input);
        uiKit.setStatusBadge(document.getElementById('lidar-slam-header-badge'), v.tone, v.text);
        uiKit.renderStepper(document.getElementById('lidar-slam-stepper'), UiStatusLogic.SLAM_STEPS, v.step);
        uiKit.setEnabled(document.getElementById('slam-start-button'), v.buttons.start);
        uiKit.setEnabled(document.getElementById('slam-stop-button'), v.buttons.stop);
        uiKit.setEnabled(document.getElementById('slam-save-map-button'), v.buttons.saveMap, saveMapLockReason(input));
        const empty = document.getElementById('lidar-slam-viewer-empty');
        if (empty) renderSlamEmpty(empty, input, liveEl, resultEl);
    }

    function refresh() {
        const screenEl = document.querySelector('.tab-content.active .subtab-content.active');
        syncSaveElapsed();
        if (!screenEl) return;
        if (screenEl.id === 'lidar-slam-subtab') {
            renderSlam();
            return;
        }
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
        SCREENS, showScreen, onScreenChanged, refresh, init, onSlamState, noteAnalytics,
        onRosbridgeState, onLatency, onDomainId, onBackendWsState,
        get lastSummary() { return lastSummary; },
    };
})();
window.uiShell = uiShell;
window.showScreen = uiShell.showScreen;
document.addEventListener('DOMContentLoaded', uiShell.init);
