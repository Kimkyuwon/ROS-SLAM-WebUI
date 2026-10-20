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
    let lastLocState = null;
    // 시작 실패 감시는 Task 2.11이 이 값을 바꾼다
    let locStartFailed = false;
    let locNotInitSince = 0;

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

    function onLocState(state) {
        lastLocState = state;
        refresh();
    }

    function saveMapLockReason(input) {
        if (!input.isRunning) return 'SLAM 실행 중에만 저장할 수 있습니다';
        if (input.stopping) return 'SLAM을 종료하는 중입니다';
        if (input.saving) return '지도를 저장하는 중입니다';
        if (!(input.kfCount > 0)) return '키프레임이 아직 없습니다';
        return '데이터 수신이 멈춰 저장할 수 없습니다';
    }

    const SLAM_EMPTY_PARTS = {
        eyebrow: '.ui-viewer-empty-eyebrow', title: '.ui-viewer-empty-title', body: '.ui-viewer-empty-body',
    };
    const SLAM_EMPTY_TEXT = {
        starting: { title: 'rosbridge 연결 대기 중…' },
        'restore-failed': {
            eyebrow: 'SAVED RESULT',
            title: '서버가 다시 시작되어 이전 결과를 열 수 없습니다',
            body: '새로고침 뒤 저장된 지도를 복원하려 했지만 서버가 마지막 저장 폴더를 기억하지 못합니다. '
                + '오른쪽 위 Start SLAM으로 새로 시작할 수 있습니다.',
        },
    };
    let slamEmptyDefault = null;
    let slamRestoreFailed = false;
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

    function onSaveMapRestoreFailed() {
        slamRestoreFailed = true;
        refresh();
    }

    function renderSlamEmpty(empty, input, liveEl, resultEl) {
        const resultShown = Boolean(resultEl && resultEl.style.display !== 'none');
        empty.hidden = (liveEl && liveEl.style.display !== 'none') || resultShown;
        if (input.starting || input.isRunning || resultShown) slamRestoreFailed = false;
        // 저장 중에는 같은 자리의 저장 카드가 글자를 덮으므로 격자만 남긴다
        let state = '';
        if (input.saving) state = 'saving';
        else if (input.starting) state = 'starting';
        else if (slamRestoreFailed) state = 'restore-failed';
        empty.dataset.state = state;
        if (!slamEmptyDefault) {
            slamEmptyDefault = {};
            Object.entries(SLAM_EMPTY_PARTS).forEach(([key, sel]) => {
                const el = empty.querySelector(sel);
                slamEmptyDefault[key] = el ? el.textContent : '';
            });
        }
        const text = { ...slamEmptyDefault, ...(SLAM_EMPTY_TEXT[state] || {}) };
        Object.entries(SLAM_EMPTY_PARTS).forEach(([key, sel]) => {
            const el = empty.querySelector(sel);
            if (el) el.textContent = text[key];
        });
    }

    function renderSaveMapResultDir() {
        const el = document.getElementById('savemap-result-dir');
        if (!el || typeof saveMapResultViewer === 'undefined' || !saveMapResultViewer._directory) return;
        el.textContent = `pose_graph_optimization/${saveMapResultViewer._directory}/`;
    }

    // 새로고침으로 실행 중 화면에 들어온 경우도 잠근다. 입력은 상태가 바뀔 때만 다시 써서 0.5초 폴링 부담을 피한다
    function renderConfigLock(containerId, locked) {
        const cfgEl = document.getElementById(containerId);
        if (!cfgEl) return;
        if (cfgEl.dataset.readonly !== String(locked)) uiKit.setReadonly(cfgEl, locked);
        cfgEl.querySelectorAll('.ui-panel-footer .ui-btn').forEach((btn) => {
            uiKit.setEnabled(btn, !locked, '실행 중에는 바꿀 수 없습니다');
        });
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
        if (input.resultVisible) renderSaveMapResultDir();
        renderConfigLock('slam-config-container', input.isRunning || input.starting || input.stopping);
    }

    // 노드는 원점에서 스스로 초기화를 시도하므로, 이 시간이 지나도 Not Init이면 안내를 보인다 (임의 값)
    const LOC_INIT_HINT_MS = 10000;

    function renderLoc() {
        const isRunning = Boolean(lastLocState && lastLocState.is_running);
        const badge = document.getElementById('loc-header-init-badge');
        const summaryLoc = lastSummary ? lastSummary.localization.running : null;
        const input = {
            isRunning,
            starting: Boolean(window._locStarting),
            stopping: Boolean(window._locStopping),
            initialized: Boolean(badge && badge.classList.contains('ok')),
            external: isRunning && summaryLoc === false,
            stale: isAnalyticsStale('loc'),
            lastStartFailed: locStartFailed,
        };
        const v = UiStatusLogic.deriveLocView(input);
        uiKit.setStatusBadge(document.getElementById('localization-header-badge'), v.tone, v.text);
        uiKit.renderStepper(document.getElementById('localization-stepper'), UiStatusLogic.LOC_STEPS, v.step);
        uiKit.setEnabled(document.getElementById('localization-start-button'), v.buttons.start);
        uiKit.setEnabled(document.getElementById('localization-stop-button'), v.buttons.stop,
            v.text.includes('외부') ? '웹 UI가 시작하지 않은 Localization입니다' : '');
        const mapInput = document.querySelector('#localization-config-parameters [data-config-key$="map_file_path"]');
        const mapPath = document.getElementById('localization-map-path');
        // map_file_path는 FAST_LIO 소스 폴더 기준 상대 경로 (laserLocalization.cpp:1544)
        if (mapPath) mapPath.textContent = mapInput ? `FAST_LIO/${mapInput.value}` : '-';
        const live = document.getElementById('localization-live-viewer');
        const empty = document.getElementById('localization-viewer-empty');
        if (empty) empty.hidden = Boolean(live && live.style.display !== 'none');
        renderConfigLock('localization-config-container', input.isRunning || input.starting || input.stopping);

        const notInit = isRunning && !input.initialized && !input.external && !input.stale
            && !input.starting && !input.stopping;
        if (!notInit) locNotInitSince = 0;
        else if (!locNotInitSince) locNotInitSince = performance.now();
        const hint = document.getElementById('localization-init-hint');
        if (hint) hint.hidden = !(notInit && performance.now() - locNotInitSince > LOC_INIT_HINT_MS);
    }

    function refresh() {
        const screenEl = document.querySelector('.tab-content.active .subtab-content.active');
        syncSaveElapsed();
        if (!screenEl) return;
        if (screenEl.id === 'lidar-slam-subtab') {
            renderSlam();
            return;
        }
        if (screenEl.id === 'localization-subtab') {
            renderLoc();
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
        SCREENS, showScreen, onScreenChanged, refresh, init, onSlamState, onLocState, noteAnalytics, onSaveMapRestoreFailed,
        onRosbridgeState, onLatency, onDomainId, onBackendWsState,
        get lastSummary() { return lastSummary; },
    };
})();
window.uiShell = uiShell;
window.showScreen = uiShell.showScreen;
document.addEventListener('DOMContentLoaded', uiShell.init);
