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
    const HEALTH_TICK_MS = 1000;
    // Start 성공 응답 뒤 이 시간 안에 /loc_analytics가 한 번도 안 오면 시작 실패로 본다
    const LOC_START_TIMEOUT_MS = 10000;
    let locStartFailed = false;
    let locStartWatch = null;
    let locStartedAt = 0;
    let locNotInitSince = 0;

    function readNumber(id) {
        const el = document.getElementById(id);
        if (!el) return null;
        const n = parseFloat(el.textContent);
        return Number.isFinite(n) ? n : null;
    }

    function clearLocStartWatch() {
        if (locStartWatch) clearTimeout(locStartWatch);
        locStartWatch = null;
    }

    function noteAnalytics(kind) {
        lastAnalyticsAt[kind] = performance.now();
        if (kind === 'loc') {
            clearLocStartWatch();
            locStartFailed = false;
        }
    }

    function onLocStartRequested() {
        clearLocStartWatch();
        locStartFailed = false;
        locStartedAt = performance.now();
        locStartWatch = setTimeout(() => {
            locStartWatch = null;
            if (lastAnalyticsAt.loc < locStartedAt) {
                locStartFailed = true;
                refresh();
            }
        }, LOC_START_TIMEOUT_MS);
    }

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

    // 이번 실행에서 본 키프레임 수의 최댓값. 화면 숫자가 끊기면(—·0) 줄지 않는다.
    let slamKfMax = 0;

    function resetSlamKeyframeCount() {
        slamKfMax = 0;
    }

    function slamKeyframeCount(input) {
        if (input.starting || !input.isRunning) {
            slamKfMax = 0;
            return 0;
        }
        const live = readNumber(SLAM_KF_COUNT_ID);
        if (live != null && live > slamKfMax) slamKfMax = live;
        return slamKfMax;
    }

    function saveMapLockReason(input) {
        if (!input.isRunning) return 'SLAM 실행 중에만 저장할 수 있습니다';
        if (input.stopping) return 'SLAM을 종료하는 중입니다';
        if (input.saving) return '지도를 저장하는 중입니다';
        if (!(input.kfCount > 0)) return '키프레임이 아직 없습니다';
        return '';
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
    let slamSavedSinceStart = false;
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
            kfCount: 0,
            stale: isAnalyticsStale('slam'),
            imuHz: readNumber('analytics-imu-hz'),
            lidHz: readNumber('analytics-lid-hz'),
        };
        input.kfCount = slamKeyframeCount(input);
        const v = UiStatusLogic.deriveSlamView(input);
        if (input.starting) slamSavedSinceStart = false;
        if (input.justSaved) slamSavedSinceStart = true;
        uiKit.setStatusBadge(document.getElementById('lidar-slam-header-badge'), v.tone, v.text);
        uiKit.renderStepper(document.getElementById('lidar-slam-stepper'), UiStatusLogic.SLAM_STEPS, v.step);
        uiKit.setEnabled(document.getElementById('slam-start-button'), v.buttons.start);
        uiKit.setEnabled(document.getElementById('slam-stop-button'), v.buttons.stop);
        uiKit.setEnabled(document.getElementById('slam-save-map-button'), v.buttons.saveMap, saveMapLockReason(input));
        const startHint = document.getElementById('lidar-slam-start-hint');
        if (startHint) startHint.hidden = !(v.buttons.start && !input.resultVisible);
        const liveBadge = document.getElementById('slam-live-badge');
        if (liveBadge) {
            liveBadge.dataset.tone = input.stale ? 'warn' : 'go';
            liveBadge.textContent = input.stale ? 'NO DATA' : 'LIVE';
        }
        const empty = document.getElementById('lidar-slam-viewer-empty');
        if (empty) renderSlamEmpty(empty, input, liveEl, resultEl);
        if (input.resultVisible) renderSaveMapResultDir();
        renderConfigLock('slam-config-container', input.isRunning || input.starting || input.stopping);
    }

    function shouldConfirmSlamStop() {
        return !slamSavedSinceStart;
    }

    // 노드는 원점에서 스스로 초기화를 시도하므로, 이 시간이 지나도 Not Init이면 안내를 보인다 (임의 값)
    const LOC_INIT_HINT_MS = 10000;

    // 시작 실패일 때만 배지가 로그 패널을 여는 버튼이 된다
    function setBadgeLogLink(badge, active) {
        if (!badge) return;
        badge.setAttribute('role', active ? 'button' : 'status');
        if (active) badge.tabIndex = 0;
        else badge.removeAttribute('tabindex');
    }

    function initBadgeLogLink() {
        const badge = document.getElementById('localization-header-badge');
        if (!badge) return;
        const open = () => { if (locStartFailed) openNodeLog('loc'); };
        badge.addEventListener('click', open);
        badge.addEventListener('keydown', (e) => {
            if (e.key !== 'Enter' && e.key !== ' ') return;
            e.preventDefault();
            open();
        });
    }

    function renderLoc() {
        // launch는 octomap_server 때문에 살아 있어도 /loc_analytics가 없으면 시작 실패가 우선한다
        const isRunning = Boolean(lastLocState && lastLocState.is_running) && !locStartFailed;
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
        const headBadge = document.getElementById('localization-header-badge');
        uiKit.setStatusBadge(headBadge, v.tone, v.text);
        setBadgeLogLink(headBadge, locStartFailed);
        uiKit.renderStepper(document.getElementById('localization-stepper'), UiStatusLogic.LOC_STEPS, v.step);
        uiKit.setEnabled(document.getElementById('localization-start-button'), v.buttons.start);
        uiKit.setEnabled(document.getElementById('localization-stop-button'), v.buttons.stop,
            v.text.includes('외부') ? '웹 UI가 시작하지 않은 Localization입니다' : '');
        uiKit.setEnabled(document.getElementById('loc-initpose-btn'), isRunning,
            isRunning ? '' : 'Localization 실행 중에만 지정할 수 있습니다');
        if (!isRunning && window.locLiveViewer && locLiveViewer._posePick) locLiveViewer.cancelInitialPosePick();
        const mapInput = document.querySelector('#localization-config-parameters [data-config-key$="map_file_path"]');
        const mapPath = document.getElementById('localization-map-path');
        // map_file_path는 FAST_LIO 소스 폴더 기준 상대 경로 (laserLocalization.cpp:1544)
        if (mapPath) mapPath.textContent = mapInput ? `FAST_LIO/${mapInput.value}` : '-';
        const live = document.getElementById('localization-live-viewer');
        const empty = document.getElementById('localization-viewer-empty');
        if (empty) empty.hidden = Boolean(live && live.style.display !== 'none');
        const locBadge = document.getElementById('loc-live-badge');
        if (locBadge) {
            locBadge.dataset.tone = input.stale ? 'warn' : 'go';
            locBadge.textContent = input.stale ? 'NO DATA' : 'LIVE';
        }
        renderConfigLock('localization-config-container', input.isRunning || input.starting || input.stopping);

        const notInit = isRunning && !input.initialized && !input.external && !input.stale
            && !input.starting && !input.stopping;
        if (!notInit) locNotInitSince = 0;
        else if (!locNotInitSince) locNotInitSince = performance.now();
        const hint = document.getElementById('localization-init-hint');
        if (hint) hint.hidden = !(notInit && performance.now() - locNotInitSince > LOC_INIT_HINT_MS);
    }

    function plotTabs() {
        const manager = typeof plotState !== 'undefined' && plotState.plotTabManager;
        return manager ? manager.tabs.filter(tab => tab.plotManager) : [];
    }

    // 탭 modebar로 따로 멈출 수 있어, 모든 탭이 멈췄을 때만 Paused로 본다
    function isPlotAllPaused() {
        const tabs = plotTabs();
        return tabs.length > 0 && tabs.every(tab => tab.plotManager.isPaused);
    }

    function renderPlotHeader() {
        const paused = isPlotAllPaused();
        const btn = document.getElementById('plot-header-pause-btn');
        uiKit.setBtnLabel(btn, paused ? 'Resume' : 'Pause');
        const icon = btn && btn.querySelector('.ui-icon use');
        if (icon) icon.setAttribute('href', paused ? '#icon-play' : '#icon-pause');
        const hasData = plotTabs().some(tab => tab.plotManager.dataBuffers.size > 0);
        const badge = document.getElementById('plot-header-badge');
        if (paused) uiKit.setStatusBadge(badge, 'warn', 'Paused');
        else if (hasData) uiKit.setStatusBadge(badge, 'go', 'Streaming');
        else uiKit.setStatusBadge(badge, 'idle', 'Ready');
    }

    function setPlotPaused(paused) {
        plotTabs().forEach(tab => {
            if (tab.plotManager.isPaused !== paused) tab.plotManager.togglePause(paused);
        });
        renderPlotHeader();
    }

    function togglePlotPause() { setPlotPaused(!isPlotAllPaused()); }

    function syncPlotWindowSeg() {
        const input = document.getElementById('buffer-time-input');
        const value = input ? input.value : '';
        document.querySelectorAll('#plot-window-seg [data-sec]').forEach(option => {
            option.setAttribute('aria-checked', String(option.dataset.sec === value));
        });
    }

    function setPlotWindow(seconds) {
        const input = document.getElementById('buffer-time-input');
        if (input) input.value = String(seconds);
        updateBufferTime(seconds);
        syncPlotWindowSeg();
    }

    function onPlotPauseChanged() {
        const screenEl = document.querySelector('.tab-content.active .subtab-content.active');
        if (screenEl && screenEl.id === 'plot-subtab') renderPlotHeader();
    }

    function initPlotHeader() {
        const input = document.getElementById('buffer-time-input');
        if (input) input.addEventListener('change', syncPlotWindowSeg);
        syncPlotWindowSeg();
    }

    // 1024px 미만에서만 보이는 메뉴 버튼으로 사이드바를 겹쳐 연다 (CSS가 폭별 배치를 맡음)
    function setSidebarOpen(open) {
        document.body.classList.toggle('ui-sidebar-open', open);
        const toggle = document.getElementById('ui-sidebar-toggle');
        if (!toggle) return;
        toggle.setAttribute('aria-expanded', String(open));
        toggle.setAttribute('aria-label', open ? '메뉴 닫기' : '메뉴 열기');
        toggle.title = toggle.getAttribute('aria-label');
    }

    function initSidebarToggle() {
        // 아이콘 레일(1024~1279px)에서는 라벨이 숨으므로 툴팁으로 이름을 보인다
        document.querySelectorAll('.ui-sidebar-item').forEach(item => {
            const label = item.querySelector('.ui-sidebar-label');
            if (label && !item.title) item.title = label.textContent.trim();
            item.addEventListener('click', () => setSidebarOpen(false));
        });
        const toggle = document.getElementById('ui-sidebar-toggle');
        if (!toggle) return;
        toggle.addEventListener('click', () => setSidebarOpen(!document.body.classList.contains('ui-sidebar-open')));
        document.addEventListener('click', (e) => {
            if (!document.body.classList.contains('ui-sidebar-open')) return;
            if (e.target.closest('#ui-sidebar, #ui-sidebar-toggle')) return;
            setSidebarOpen(false);
        });
        document.addEventListener('keydown', (e) => {
            if (e.key === 'Escape' && document.body.classList.contains('ui-sidebar-open')) setSidebarOpen(false);
        });
    }

    let multiLoopCount = null;

    function onOptStatus(status) {
        if (status && status.loop_count != null) multiLoopCount = Number(status.loop_count);
    }
    let multiSawRunning = false;

    function multiBasename(path) {
        const parts = String(path || '').split(/[/\\]/).filter(Boolean);
        return parts.length ? parts[parts.length - 1] : '';
    }

    function multiOutputError(output, map1, map2) {
        if (!output) return '';
        if (!/^[A-Za-z0-9_-]{1,64}$/.test(output)) return '영문, 숫자, _, - 만 1~64자입니다';
        const base1 = multiBasename(map1);
        const base2 = multiBasename(map2);
        if ((base1 && output === base1) || (base2 && output === base2)) {
            return '입력 폴더와 같은 이름은 쓸 수 없습니다';
        }
        return '';
    }

    function renderMulti() {
        const map1 = (document.getElementById('slam-map1') || {}).value || '';
        const map2 = (document.getElementById('slam-map2') || {}).value || '';
        const outputEl = document.getElementById('slam-output');
        const output = outputEl ? outputEl.value.trim() : '';
        const err = multiOutputError(output, map1.trim(), map2.trim());
        const errEl = document.getElementById('multi-session-output-error');
        if (errEl) {
            errEl.hidden = !err;
            errEl.textContent = err;
        }
        const loaded = document.getElementById('multi-session-map1-loaded');
        if (loaded) {
            loaded.hidden = !map1.trim();
            loaded.textContent = map1.trim() ? `Map 1 loaded · ${map1.trim()}` : '';
        }
        const inputsReady = Boolean(map1.trim() && map2.trim() && output && !err);
        const running = Boolean(typeof _optRunning !== 'undefined' && _optRunning);
        if (running && !multiSawRunning) multiLoopCount = null;
        multiSawRunning = running;
        const msgEl = document.getElementById('slam-opt-msg');
        const message = msgEl ? msgEl.textContent : '';
        const parsedLoop = UiStatusLogic.parseLoopCount(message);
        if (parsedLoop !== null) multiLoopCount = parsedLoop;
        const area = document.getElementById('slam-opt-status-area');
        const failed = Boolean(area && area.classList.contains('error') && area.style.display !== 'none');
        const v = UiStatusLogic.deriveMultiView({
            running,
            complete: typeof _optComplete !== 'undefined' && _optComplete,
            failed,
            inputsReady,
            doneCount: UiStatusLogic.optDoneCount(message),
            loopCount: multiLoopCount,
        });
        uiKit.setStatusBadge(document.getElementById('multi-session-header-badge'), v.tone, v.text);
        uiKit.renderStepper(
            document.getElementById('multi-session-stepper'), UiStatusLogic.MULTI_STEPS, v.step
        );
        const runBtn = document.getElementById('slam-opt-run-btn');
        if (!running && runBtn) uiKit.setEnabled(runBtn, v.buttons.run);
        const cancel = document.getElementById('multi-session-header-cancel-btn');
        if (cancel) cancel.hidden = !v.buttons.cancel;
        const list = document.getElementById('multi-session-progress');
        if (list) {
            const doneCount = UiStatusLogic.optDoneCount(message);
            list.replaceChildren(...UiStatusLogic.OPT_PROGRESS_LABELS.map((label, i) => {
                const li = document.createElement('li');
                let state = 'todo';
                if (running) state = i < doneCount ? 'done' : (i === doneCount ? 'current' : 'todo');
                else if (typeof _optComplete !== 'undefined' && _optComplete) state = 'done';
                li.dataset.state = state;
                li.textContent = (i === 1 && multiLoopCount !== null)
                    ? `${label} · 루프 ${multiLoopCount}개` : label;
                return li;
            }));
        }
        const viewer = document.getElementById('slam-result-viewer');
        const empty = document.getElementById('multi-session-viewer-empty');
        if (empty) empty.hidden = Boolean(viewer && viewer.style.display !== 'none');
    }

    function syncBagRateSeg() {
        const slider = document.getElementById('bag-playback-rate');
        const rate = slider ? Number(slider.value) / 10 : 1;
        document.querySelectorAll('#bag-rate-seg [data-rate]').forEach((btn) => {
            btn.setAttribute('aria-checked', String(Number(btn.dataset.rate) === rate));
        });
    }

    function initBagRateSeg() {
        const seg = document.getElementById('bag-rate-seg');
        if (!seg || seg.dataset.bound) return;
        seg.dataset.bound = '1';
        seg.addEventListener('click', (e) => {
            const btn = e.target.closest('[data-rate]');
            if (!btn || btn.disabled) return;
            const slider = document.getElementById('bag-playback-rate');
            if (!slider) return;
            slider.value = String(Math.round(Number(btn.dataset.rate) * 10));
            if (typeof updatePlaybackRate === 'function') updatePlaybackRate(slider.value);
            syncBagRateSeg();
        });
    }

    // 숨은 #view-type-select 값을 바꿔 기존 switchViewType 핸들러를 그대로 부른다
    function initViewTypeSeg() {
        const seg = document.getElementById('view-type-seg');
        if (!seg || seg.dataset.bound) return;
        seg.dataset.bound = '1';
        seg.addEventListener('click', (e) => {
            const btn = e.target.closest('[data-view]');
            const select = document.getElementById('view-type-select');
            if (!btn || !select) return;
            select.value = btn.dataset.view;
            select.dispatchEvent(new Event('change'));
        });
    }

    function renderBag() {
        const dir = document.getElementById('bag-directory');
        const loaded = Boolean(dir && dir.value.trim());
        const play = document.getElementById('bag-play-button');
        const pause = document.getElementById('bag-pause-button');
        const playing = Boolean(play && play.textContent.trim() === 'Stop');
        const paused = Boolean(pause && pause.textContent.trim() === 'Resume');
        const slider = document.getElementById('bag-playback-rate');
        const rate = slider ? Number(slider.value) / 10 : 1;
        const convert = document.getElementById('convert-bag-btn');
        const converting = Boolean(convert && convert.textContent.includes('Converting'));
        const v = UiStatusLogic.deriveBagView({ loaded, playing, paused, rate, converting });
        uiKit.setStatusBadge(document.getElementById('bag-player-header-badge'), v.tone, v.text);
        if (play) uiKit.setEnabled(play, v.buttons.play);
        if (pause) uiKit.setEnabled(pause, v.buttons.pause);
        const seek = document.getElementById('bag-slider');
        if (seek) seek.disabled = !v.buttons.seek;
        document.querySelectorAll('#bag-rate-seg [data-rate]').forEach((btn) => {
            btn.disabled = !v.buttons.seek;
        });
        const loop = document.getElementById('bag-player-loop');
        if (loop) loop.disabled = !loaded;
        if (convert && !converting) uiKit.setEnabled(convert, v.buttons.convert);
        const panel = document.getElementById('bag-convert-controls');
        if (panel) panel.dataset.converting = String(converting);
        if (play) {
            play.classList.toggle('ui-btn-go', !playing);
            play.classList.toggle('ui-btn-stop', playing);
            play.dataset.active = String(playing);
        }
        document.querySelectorAll('#bag-player-subtab [onclick^="setAllBagTopics"]').forEach((btn) => {
            btn.disabled = !loaded || converting;
        });
        document.querySelectorAll('#bag-player-subtab [onclick="loadBagFile()"]').forEach((btn) => {
            btn.disabled = converting;
        });
        syncBagRateSeg();
    }

    let lastRecorder = null;

    function onRecorderState(state) {
        lastRecorder = state;
        const screenEl = document.querySelector('.tab-content.active .subtab-content.active');
        if (screenEl && screenEl.id === 'bag-recorder-subtab') renderRecorder();
    }

    function renderRecorder() {
        const nameEl = document.getElementById('recorder-bag-name');
        const elapsed = lastRecorder ? Number(lastRecorder.elapsed_sec) || 0 : 0;
        const bytes = lastRecorder ? Number(lastRecorder.size_bytes) || 0 : 0;
        const v = UiStatusLogic.deriveRecorderView({
            recording: Boolean(lastRecorder && lastRecorder.recording),
            hasName: Boolean(nameEl && nameEl.value.trim()),
            topicCount: bagRecorderState.selectedTopics.length,
            elapsedSec: elapsed,
        });
        uiKit.setStatusBadge(document.getElementById('bag-recorder-header-badge'), v.tone, v.text);
        const rec = document.getElementById('recorder-record-button');
        if (rec) uiKit.setEnabled(rec, v.buttons.record);
        const nameBtn = document.querySelector('#bag-recorder-subtab [onclick="enterBagName()"]');
        if (nameBtn) nameBtn.disabled = !v.buttons.edit;
        const fmt = document.getElementById('recorder-format-select');
        if (fmt) fmt.disabled = !v.buttons.edit;
        document.querySelectorAll('#recorder-topic-table input').forEach((el) => {
            el.disabled = !v.buttons.edit;
        });
        const sec = Math.max(0, Math.floor(elapsed));
        const time = document.getElementById('recorder-stat-time');
        if (time) time.textContent = `${Math.floor(sec / 60)}:${String(sec % 60).padStart(2, '0')}`;
        const size = document.getElementById('recorder-stat-size');
        if (size) {
            size.textContent = bytes >= 1073741824
                ? `${(bytes / 1073741824).toFixed(1)} GB`
                : `${(bytes / 1048576).toFixed(0)} MB`;
        }
        const topics = document.getElementById('recorder-stat-topics');
        if (topics) topics.textContent = String(bagRecorderState.selectedTopics.length);
        const title = document.getElementById('recorder-topic-table-title');
        const total = document.querySelectorAll('#recorder-topic-table tbody tr').length;
        if (title) title.textContent = `Selected Topics · ${bagRecorderState.selectedTopics.length}/${total}`;
        const refreshBtn = document.querySelector('#bag-recorder-subtab [onclick="refreshRecorderTopics()"]');
        if (refreshBtn) refreshBtn.disabled = !v.buttons.edit;
        // 녹화 중이 아니면 고른 형식을 미리 보여 준다 (녹화 중 표시는 서버 mode 기준으로 script.js가 쓴다)
        const modeBadge = document.getElementById('recorder-mode-badge');
        if (modeBadge && v.buttons.edit && fmt) {
            modeBadge.style.display = 'inline';
            modeBadge.textContent = fmt.options[fmt.selectedIndex].text;
        }
    }

    function initFileDatasetSeg() {
        const seg = document.getElementById('file-player-dataset-seg');
        if (!seg || seg.dataset.bound) return;
        seg.dataset.bound = '1';
        seg.addEventListener('click', (e) => {
            const btn = e.target.closest('[data-dataset]');
            if (!btn || btn.disabled) return;
            const select = document.getElementById('dataset-format-select');
            if (!select) return;
            select.value = btn.dataset.dataset;
            select.dispatchEvent(new Event('change'));
            renderFile();
        });
    }

    function renderFile() {
        const select = document.getElementById('dataset-format-select');
        const ds = select ? select.value : '';
        document.querySelectorAll('#file-player-dataset-seg [data-dataset]').forEach((btn) => {
            btn.setAttribute('aria-checked', String(btn.dataset.dataset === ds));
            btn.disabled = false;
        });
        const fp = lastSummary && lastSummary.file_player ? lastSummary.file_player : {};
        const play = document.getElementById('play-button');
        const pause = document.getElementById('pause-button');
        const playing = Boolean(play && play.textContent.trim() === 'End');
        const paused = Boolean(pause && pause.textContent.trim() === 'Resume');
        const converting = Boolean(fp.converting)
            || (typeof kittiState !== 'undefined' && kittiState.converting)
            || (typeof kaistState !== 'undefined' && kaistState.converting)
            || (typeof mulranState !== 'undefined' && mulranState.converting)
            || (typeof heliprState !== 'undefined' && heliprState.converting);
        const v = UiStatusLogic.deriveFileView({
            loaded: Boolean(fp.loaded),
            playing,
            paused,
            converting,
            savingBag: Boolean(fp.saving_bag),
            progress: parseFloat((document.getElementById(`${ds}-progress-fill`) || { style: {} }).style.width) || 0,
        });
        uiKit.setStatusBadge(document.getElementById('file-player-header-badge'), v.tone, v.text);
        if (play) {
            uiKit.setEnabled(play, v.buttons.play);
            play.classList.toggle('ui-btn-go', !playing);
            play.classList.toggle('ui-btn-stop', playing);
            play.dataset.active = String(playing);
        }
        if (pause) uiKit.setEnabled(pause, playing || paused);
        document.querySelectorAll('#file-player-dataset-seg [data-dataset]').forEach((btn) => {
            btn.disabled = !v.buttons.load;
        });
        const screen = document.getElementById('file-player-subtab');
        if (screen) screen.dataset.dataset = ds;
        const title = document.getElementById('file-player-load-title');
        if (title && select) title.textContent = `${select.options[select.selectedIndex].text} · 불러오기`;
        const load = document.querySelector('#file-player-subtab [onclick="loadPlayerPath()"]');
        if (load) load.disabled = !v.buttons.load;
    }

    function refresh() {
        const screenEl = document.querySelector('.tab-content.active .subtab-content.active');
        if (window._locStopping) {
            clearLocStartWatch();
            locStartFailed = false;
        }
        syncSaveElapsed();
        syncNodeLogPolling();
        if (!screenEl) return;
        if (screenEl.id === 'lidar-slam-subtab') {
            renderSlam();
            warnSidebarTag(screenEl);
            return;
        }
        if (screenEl.id === 'localization-subtab') {
            renderLoc();
            warnSidebarTag(screenEl);
            return;
        }
        if (screenEl.id === 'multi-session-slam-subtab') {
            renderMulti();
            return;
        }
        if (screenEl.id === 'bag-player-subtab') {
            renderBag();
            return;
        }
        if (screenEl.id === 'bag-recorder-subtab') {
            renderRecorder();
            return;
        }
        if (screenEl.id === 'file-player-subtab') {
            renderFile();
            return;
        }
        if (screenEl.id === 'plot-subtab') {
            renderPlotHeader();
            return;
        }
        // 3D 배지는 Fixed Frame 표시 전용 (threejs_display.js)
        if (screenEl.id === '3d-viewer-subtab') {
            if (typeof updateFixedFrameBadge === 'function') updateFixedFrameBadge();
            return;
        }
        const badge = screenEl.querySelector('.ui-screen-header .ui-status-badge');
        const markers = lastSummary ? UiStatusLogic.deriveSidebarMarkers(lastSummary) : {};
        const marker = markers[screenEl.id];
        if (marker) uiKit.setStatusBadge(badge, marker.tone, HEADER_BADGE_TEXT[marker.text] || marker.text);
        else uiKit.setStatusBadge(badge, 'idle', 'Ready');
    }

    // 사이드바 RUN 태그는 launch 프로세스만 보므로, 보이는 화면의 배지가 warn일 때만 그 태그 톤을 맞춘다
    function warnSidebarTag(screenEl) {
        const item = document.querySelector(`.ui-sidebar-item[data-screen="${screenEl.id}"]`);
        const tag = item && item.querySelector('[data-run-tag]');
        const badge = screenEl.querySelector('.ui-screen-header .ui-status-badge');
        if (!tag || tag.hidden || !badge) return;
        if (badge.dataset.tone === 'warn') {
            tag.dataset.tone = 'warn';
        } else if (lastSummary) {
            tag.dataset.tone = UiStatusLogic.deriveSidebarMarkers(lastSummary)[screenEl.id].tone;
        }
    }

    // 새 메시지가 없어도 데이터 끊김(3초) 판정이 갱신되게 한다
    function healthTick() {
        const screenEl = document.querySelector('.tab-content.active .subtab-content.active');
        if (screenEl && (screenEl.id === 'lidar-slam-subtab' || screenEl.id === 'localization-subtab')) refresh();
    }

    const ROSBRIDGE_TEXT = {
        connected: ['go', 'rosbridge 연결됨'],
        disconnected: ['stop', 'rosbridge 오류'],
        reconnecting: ['warn', 'rosbridge 재연결 중…'],
    };

    let rosbridgeTone = 'idle';

    function onRosbridgeState(state) {
        const conf = ROSBRIDGE_TEXT[state];
        if (conf) uiKit.setStatusBadge(document.getElementById('ui-conn-rosbridge'), conf[0], conf[1]);
        if (conf) rosbridgeTone = conf[0];
    }

    function onLatency(ms) {
        const text = (ms == null || !isFinite(ms)) ? 'N/A' : `${Math.round(ms)} ms`;
        const tone = (ms == null) ? rosbridgeTone : (ms < 50 ? 'go' : (ms < 150 ? 'warn' : 'stop'));
        for (const id of ['ui-conn-latency', 'latency-indicator']) {
            const el = document.getElementById(id);
            if (!el) continue;
            el.textContent = text;
            el.dataset.tone = tone;
        }
    }

    function onDomainId(id) {
        const text = String(id);
        for (const elId of ['ui-conn-domain', 'ros-domain-chip']) {
            const el = document.getElementById(elId);
            if (el) el.textContent = text;
        }
    }

    function onBackendWsState(state) {
        const el = document.getElementById('ui-conn-port-pc2');
        const legacy = document.getElementById('pc2-ws-port-chip');
        const ports = window.ROS_SLAM_WEBUI || {};
        const text = `${ports.pc2WsPort || '-'}${state === 'connected' ? '' : ' (끊김)'}`;
        const tone = state === 'connected' ? 'idle' : 'warn';
        for (const node of [el, legacy]) {
            if (!node) continue;
            node.textContent = text;
            node.dataset.tone = tone;
        }
    }

    function renderPorts() {
        const ports = window.ROS_SLAM_WEBUI || {};
        const set = (id, v) => { const el = document.getElementById(id); if (el) el.textContent = v ?? '-'; };
        set('ui-conn-port-web', ports.webPort);
        set('web-port-chip', ports.webPort);
        set('ui-conn-port-pc2', ports.pc2WsPort);
        set('pc2-ws-port-chip', ports.pc2WsPort);
        set('ui-conn-port-rosbridge', ports.rosbridgePort);
        set('rosbridge-status-chip', ports.rosbridgePort);
    }

    const NODE_LOG_POLL_MS = 3000;
    const NODE_LOG_LINES = 50;
    const NODE_LOGS = {
        slam: { process: 'slam', screen: 'lidar-slam-subtab', timer: null },
        loc: { process: 'localization', screen: 'localization-subtab', timer: null },
    };
    const NODE_LOG_TONES = [
        ['stop', ['Map file cannot open']],
        ['warn', ['No point, skip this scan', 'lidar loop back']],
    ];

    function nodeLogTone(line) {
        const hit = NODE_LOG_TONES.find(([, keys]) => keys.some(key => line.includes(key)));
        return hit ? hit[0] : '';
    }

    async function pollNodeLog(kind) {
        const conf = NODE_LOGS[kind];
        const pre = document.getElementById(`${kind}-log-pre`);
        const result = await apiCall(`/api/system/process_log?name=${conf.process}&lines=${NODE_LOG_LINES}`);
        if (!pre || !result || !result.success || !result.data) return;
        const atBottom = pre.scrollHeight - pre.scrollTop - pre.clientHeight < 8;
        pre.replaceChildren(...result.data.lines.map((line) => {
            const row = document.createElement('div');
            row.textContent = line;
            const tone = nodeLogTone(line);
            if (tone) row.dataset.tone = tone;
            return row;
        }));
        if (atBottom) pre.scrollTop = pre.scrollHeight;
    }

    function isNodeLogOpen(kind) {
        const panel = document.getElementById(`${kind}-log-panel`);
        return Boolean(panel && !panel.hidden);
    }

    function isScreenVisible(kind) {
        const el = document.getElementById(NODE_LOGS[kind].screen);
        return Boolean(el && el.offsetParent !== null);
    }

    // 패널이 열려 있고 그 화면이 보일 때만 3초 폴링
    function syncNodeLogPolling() {
        Object.entries(NODE_LOGS).forEach(([kind, conf]) => {
            const shouldPoll = isNodeLogOpen(kind) && isScreenVisible(kind);
            if (shouldPoll && !conf.timer) {
                pollNodeLog(kind);
                conf.timer = setInterval(() => pollNodeLog(kind), NODE_LOG_POLL_MS);
            } else if (!shouldPoll && conf.timer) {
                clearInterval(conf.timer);
                conf.timer = null;
            }
        });
    }

    function setNodeLogOpen(kind, open) {
        const panel = document.getElementById(`${kind}-log-panel`);
        const btn = document.getElementById(`${kind}-log-btn`);
        if (!panel) return;
        panel.hidden = !open;
        if (btn) btn.setAttribute('aria-pressed', String(open));
        syncNodeLogPolling();
    }

    function openNodeLog(kind) {
        if (NODE_LOGS[kind]) setNodeLogOpen(kind, true);
    }

    function initNodeLog() {
        Object.keys(NODE_LOGS).forEach((kind) => {
            const btn = document.getElementById(`${kind}-log-btn`);
            const close = document.querySelector(`#${kind}-log-panel .ui-log-close`);
            if (btn) btn.addEventListener('click', () => setNodeLogOpen(kind, !isNodeLogOpen(kind)));
            if (close) close.addEventListener('click', () => setNodeLogOpen(kind, false));
        });
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
            if (lastSummary.optimization && lastSummary.optimization.running
                && typeof _optPollTimer !== 'undefined' && !_optPollTimer && !_optRunning) {
                _optRunning = true;
                if (typeof _startOptPolling === 'function') _startOptPolling();
            }
            refresh();
        }
    }

    function init() {
        initNodeLog();
        initBadgeLogLink();
        initPlotHeader();
        initBagRateSeg();
        initViewTypeSeg();
        initFileDatasetSeg();
        setInterval(healthTick, HEALTH_TICK_MS);
        document.body.classList.add('ui-has-sidebar');
        initSidebarToggle();
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
        resetSlamKeyframeCount,
        onRosbridgeState, onLatency, onDomainId, onBackendWsState, openNodeLog, onLocStartRequested,
        onRecorderState, onOptStatus, shouldConfirmSlamStop,
        setPlotPaused, togglePlotPause, setPlotWindow, onPlotPauseChanged,
        get lastSummary() { return lastSummary; },
    };
})();
window.uiShell = uiShell;
window.showScreen = uiShell.showScreen;
document.addEventListener('DOMContentLoaded', uiShell.init);
