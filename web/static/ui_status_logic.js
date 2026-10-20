// ui_status_logic.js — DOM 없는 상태 파생 함수 (브라우저: window.UiStatusLogic, Node: module.exports)
(function (root) {
    'use strict';

    function formatElapsed(totalSec) {
        const sec = Math.max(0, Math.floor(totalSec || 0));
        const h = Math.floor(sec / 3600);
        const m = Math.floor((sec % 3600) / 60);
        const s = String(sec % 60).padStart(2, '0');
        return h > 0 ? `${h}:${String(m).padStart(2, '0')}:${s}` : `${m}:${s}`;
    }

    function playMarker(playing, paused) {
        if (paused) return { text: 'PAUSE', tone: 'warn' };
        if (playing) return { text: 'PLAY', tone: 'go' };
        return null;
    }

    function deriveSidebarMarkers(d) {
        const out = {};
        const put = (id, marker) => { if (marker) out[id] = marker; };
        put('lidar-slam-subtab', d.slam.saving_map ? { text: 'SAVE', tone: 'info' }
            : (d.slam.running ? { text: 'RUN', tone: 'go' } : null));
        put('localization-subtab', d.localization.running ? { text: 'RUN', tone: 'go' } : null);
        put('multi-session-slam-subtab', d.optimization.running ? { text: 'OPT', tone: 'info' } : null);
        put('bag-player-subtab', playMarker(d.bag_player.playing, d.bag_player.paused));
        put('bag-recorder-subtab', d.recorder.recording ? { text: 'REC', tone: 'stop' } : null);
        const fp = d.file_player;
        put('file-player-subtab', (fp.converting || fp.saving_bag) ? { text: 'SAVE', tone: 'info' }
            : playMarker(fp.playing, fp.paused));
        return out;
    }

    const SLAM_STEPS = ['Config', 'Mapping', 'Save Map', 'Result'];
    const LOC_STEPS = ['Config', 'Initialize', 'Tracking'];
    const view = (tone, text, step, buttons) => ({ tone, text, step, buttons });

    function deriveSlamView(s) {
        const locked = { start: false, stop: false, saveMap: false };
        if (s.stopping) return view('warn', 'Stopping…', 1, locked);
        if (s.starting) return view('info', 'Starting…', 1, locked);
        if (s.saving) return view('info', 'Saving Map…', 2, locked);
        const canSave = s.kfCount > 0;
        if (s.isRunning && s.stale) return view('warn', 'Running · 데이터 없음', 1, { start: false, stop: true, saveMap: false });
        if (s.isRunning && s.imuHz === 0) return view('warn', 'Running · IMU 끊김', 1, { start: false, stop: true, saveMap: canSave });
        if (s.isRunning && s.lidHz === 0) return view('warn', 'Running · LiDAR 끊김', 1, { start: false, stop: true, saveMap: canSave });
        if (s.isRunning && s.justSaved) {
            return view('info', 'Map Saved · SLAM 계속 실행 중', 3, { start: false, stop: true, saveMap: canSave });
        }
        if (s.isRunning) {
            const text = s.elapsed ? `Running · ${s.elapsed}` : 'Running';
            return view('go', text, 1, { start: false, stop: true, saveMap: canSave });
        }
        if (s.resultVisible) return view('idle', 'Ready · 저장된 지도 표시 중', 3, { start: true, stop: false, saveMap: false });
        return view('idle', 'Ready', 0, { start: true, stop: false, saveMap: false });
    }

    function deriveLocView(s) {
        const locked = { start: false, stop: false };
        if (s.stopping) return view('warn', 'Stopping…', 1, locked);
        if (s.starting) return view('info', 'Starting…', 1, locked);
        if (s.isRunning && s.external) return view('warn', 'Running (외부 실행)', s.initialized ? 2 : 1, locked);
        if (s.isRunning && s.stale) return view('warn', 'Running · 데이터 없음', 1, { start: false, stop: true });
        if (s.isRunning && s.initialized) return view('go', 'Initialized · Tracking', 2, { start: false, stop: true });
        if (s.isRunning) return view('warn', 'Running · Not Init', 1, { start: false, stop: true });
        if (s.lastStartFailed) return view('stop', '시작 실패 (로그 보기)', 0, { start: true, stop: false });
        return view('idle', 'Ready', 0, { start: true, stop: false });
    }

    const MULTI_STEPS = ['Inputs', 'Optimization', 'Result'];
    // long_term_mapping main 소스의 실행 순서 (long_term_mapping.cpp:1431-1463)
    const OPT_PROGRESS = [
        { label: '세션 불러오기', doneOn: ['Session Edge Loading Complete.'] },
        { label: '세션 정렬·루프 찾기', doneOn: ['Place Recognition Complete.', 'Loop Edge Generation Complete.'] },
        { label: '포즈 그래프 최적화', doneOn: ['Pose Factor loading Complete.', 'Graph Optimization Complete.'] },
        { label: '병합 지도 만들기', doneOn: ['Map Merging Complete.'] },
        { label: '변화 검출·정적 지도', doneOn: ['Map Update Complete.', 'Long Term SLAM Complete.'] },
    ];
    const OPT_PROGRESS_LABELS = OPT_PROGRESS.map(p => p.label);

    function optDoneCount(message) {
        const msg = String(message || '');
        for (let i = OPT_PROGRESS.length - 1; i >= 0; i--) {
            if (OPT_PROGRESS[i].doneOn.some(m => msg.startsWith(m))) return i + 1;
        }
        return 0;
    }

    function parseLoopCount(message) {
        const m = /size : (\d+)/.exec(String(message || ''));
        return m ? Number(m[1]) : null;
    }

    function deriveMultiView(s) {
        const total = OPT_PROGRESS.length;
        if (s.running) return view('info', `Optimizing · ${s.doneCount}/${total}`, 1, { run: false, cancel: true });
        if (s.complete && s.loopCount === 0) {
            return view('warn', 'Done · 세션 사이 루프 0개', 2, { run: true, cancel: false });
        }
        if (s.complete) {
            const text = s.loopCount === null ? 'Done' : `Done · 루프 ${s.loopCount}개`;
            return view('go', text, 2, { run: true, cancel: false });
        }
        if (s.failed) return view('stop', 'Failed', 0, { run: s.inputsReady, cancel: false });
        return view('idle', s.inputsReady ? 'Ready' : 'Inputs needed', 0, { run: s.inputsReady, cancel: false });
    }

    function deriveBagView(s) {
        const locked = { play: false, pause: false, seek: false, convert: false };
        if (!s.loaded) return view('idle', 'No bag', 0, locked);
        if (s.converting) return view('info', 'Converting…', 0, locked);
        if (s.paused) return view('warn', 'Paused', 0, { play: true, pause: true, seek: true, convert: false });
        if (s.playing) {
            return view('go', `Playing · ${s.rate}×`, 0, { play: true, pause: true, seek: true, convert: false });
        }
        return view('idle', 'Loaded', 0, { play: true, pause: false, seek: true, convert: true });
    }

    function deriveRecorderView(s) {
        if (s.recording) {
            return view('stop', `Recording · ${formatElapsed(s.elapsedSec)}`, 0, { record: true, edit: false });
        }
        const ready = s.hasName && s.topicCount > 0;
        return view('idle', ready ? 'Ready' : '이름·토픽 필요', 0, { record: ready, edit: true });
    }

    function deriveFileView(s) {
        if (s.converting || s.savingBag) {
            const text = `Saving bag · ${Math.round(s.progress || 0)}%`;
            return view('info', text, 0, { play: true, load: false, saveBag: false });
        }
        if (s.paused) return view('warn', 'Paused', 0, { play: true, load: false, saveBag: true });
        if (s.playing) return view('go', 'Playing', 0, { play: true, load: false, saveBag: false });
        if (s.loaded) return view('idle', 'Loaded', 0, { play: true, load: true, saveBag: true });
        return view('idle', 'Not loaded', 0, { play: false, load: true, saveBag: false });
    }

    const api = {
        formatElapsed, deriveSidebarMarkers, SLAM_STEPS, LOC_STEPS, deriveSlamView, deriveLocView,
        MULTI_STEPS, OPT_PROGRESS_LABELS, optDoneCount, parseLoopCount,
        deriveMultiView, deriveBagView, deriveRecorderView, deriveFileView,
    };
    if (typeof module !== 'undefined' && module.exports) {
        module.exports = api;
    } else {
        root.UiStatusLogic = api;
    }
})(typeof window !== 'undefined' ? window : globalThis);
