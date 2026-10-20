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

    const api = { formatElapsed, deriveSidebarMarkers, SLAM_STEPS, LOC_STEPS, deriveSlamView, deriveLocView };
    if (typeof module !== 'undefined' && module.exports) {
        module.exports = api;
    } else {
        root.UiStatusLogic = api;
    }
})(typeof window !== 'undefined' ? window : globalThis);
