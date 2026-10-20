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

    const api = { formatElapsed, deriveSidebarMarkers };
    if (typeof module !== 'undefined' && module.exports) {
        module.exports = api;
    } else {
        root.UiStatusLogic = api;
    }
})(typeof window !== 'undefined' ? window : globalThis);
