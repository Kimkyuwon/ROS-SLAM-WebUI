const assert = require('assert');
const L = require('../../web/static/ui_status_logic.js');

const idle = {
    slam: { running: false, saving_map: false }, localization: { running: false },
    optimization: { running: false }, bag_player: { loaded: false, playing: false, paused: false },
    recorder: { recording: false, mode: 'ros2' },
    file_player: { loaded: false, playing: false, paused: false, converting: false, saving_bag: false },
};
const withPatch = (patch) => JSON.parse(JSON.stringify(Object.assign({}, idle, patch)));

assert.deepStrictEqual(L.deriveSidebarMarkers(idle), {});
assert.deepStrictEqual(L.deriveSidebarMarkers(withPatch({ slam: { running: true, saving_map: false } }))['lidar-slam-subtab'],
    { text: 'RUN', tone: 'go' });
assert.strictEqual(L.deriveSidebarMarkers(withPatch({ slam: { running: true, saving_map: true } }))['lidar-slam-subtab'].text, 'SAVE');
assert.deepStrictEqual(L.deriveSidebarMarkers(withPatch({ recorder: { recording: true, mode: 'ros2_mcap' } }))['bag-recorder-subtab'],
    { text: 'REC', tone: 'stop' });
assert.strictEqual(L.deriveSidebarMarkers(withPatch({ optimization: { running: true } }))['multi-session-slam-subtab'].text, 'OPT');
assert.strictEqual(L.deriveSidebarMarkers(withPatch({ bag_player: { loaded: true, playing: true, paused: true } }))['bag-player-subtab'].text, 'PAUSE');
assert.strictEqual(L.formatElapsed(65), '1:05');
assert.strictEqual(L.formatElapsed(3725), '1:02:05');

const base = { isRunning: false, starting: false, stopping: false, saving: false, justSaved: false, resultVisible: false,
    elapsed: '', kfCount: 3, stale: false, imuHz: 200, lidHz: 10 };
const v = (p) => L.deriveSlamView(Object.assign({}, base, p));
assert.deepStrictEqual(v({}), { tone: 'idle', text: 'Ready', step: 0, buttons: { start: true, stop: false, saveMap: false } });
assert.deepStrictEqual(v({ isRunning: true, elapsed: '00:00:06' }).buttons, { start: false, stop: true, saveMap: true });
assert.strictEqual(v({ isRunning: true, kfCount: 0 }).buttons.saveMap, false);
assert.strictEqual(v({ isRunning: true, stale: true }).text, 'Running · 데이터 없음');
assert.strictEqual(v({ isRunning: true, stale: true }).buttons.saveMap, false);
assert.strictEqual(v({ isRunning: true, imuHz: 0 }).text, 'Running · IMU 끊김');
assert.strictEqual(v({ isRunning: true, imuHz: null, lidHz: null }).tone, 'go');
assert.strictEqual(v({ isRunning: true, elapsed: '00:00:06' }).text, 'Running · 00:00:06');
assert.strictEqual(v({ isRunning: true, saving: true }).step, 2);
assert.deepStrictEqual(v({ isRunning: true, saving: true }).buttons, { start: false, stop: false, saveMap: false });
assert.strictEqual(v({ isRunning: true, justSaved: true }).step, 3);
assert.strictEqual(v({ stopping: true, isRunning: true }).tone, 'warn');
assert.strictEqual(v({ resultVisible: true }).step, 3);
const lb = { isRunning: false, starting: false, stopping: false, initialized: false, external: false,
    stale: false, lastStartFailed: false };
const lv = (p) => L.deriveLocView(Object.assign({}, lb, p));
assert.strictEqual(lv({ lastStartFailed: true }).tone, 'stop');
assert.strictEqual(lv({ isRunning: true, stale: true }).text, 'Running · 데이터 없음');
assert.strictEqual(lv({ isRunning: true }).text, 'Running · Not Init');
assert.strictEqual(lv({ isRunning: true, initialized: true }).step, 2);
assert.deepStrictEqual(lv({ isRunning: true, external: true }).buttons, { start: false, stop: false });
assert.strictEqual(lv({ starting: true }).text, 'Starting…');
assert.deepStrictEqual(L.SLAM_STEPS, ['Config', 'Mapping', 'Save Map', 'Result']);
assert.deepStrictEqual(L.LOC_STEPS, ['Config', 'Initialize', 'Tracking']);
assert.strictEqual(L.optDoneCount(''), 0);
assert.strictEqual(L.optDoneCount('Session Edge Loading Complete.'), 1);
assert.strictEqual(L.optDoneCount('Loop Edge Generation Complete. size : 12'), 2);
assert.strictEqual(L.optDoneCount('Graph Optimization Complete.'), 3);
assert.strictEqual(L.optDoneCount('Map Merging Complete.'), 4);
assert.strictEqual(L.optDoneCount('Long Term SLAM Complete.'), 5);
assert.strictEqual(L.parseLoopCount('Loop Edge Generation Complete. size : 12'), 12);
assert.strictEqual(L.parseLoopCount('Map Merging Complete.'), null);
const mb = { running: false, complete: false, failed: false, inputsReady: true, doneCount: 0, loopCount: null };
assert.strictEqual(L.deriveMultiView(Object.assign({}, mb, { running: true, doneCount: 2 })).text, 'Optimizing · 2/5');
assert.deepStrictEqual(L.deriveMultiView(Object.assign({}, mb, { running: true })).buttons, { run: false, cancel: true });
assert.strictEqual(L.deriveMultiView(Object.assign({}, mb, { inputsReady: false })).buttons.run, false);
assert.strictEqual(L.deriveMultiView(Object.assign({}, mb, { complete: true })).step, 2);
assert.strictEqual(L.deriveMultiView(Object.assign({}, mb, { complete: true, loopCount: 0 })).tone, 'warn');
assert.strictEqual(L.deriveMultiView(Object.assign({}, mb, { complete: true, loopCount: 12 })).text, 'Done · 루프 12개');
const bb = { loaded: true, playing: false, paused: false, rate: 1, converting: false };
assert.strictEqual(L.deriveBagView(Object.assign({}, bb, { loaded: false })).buttons.play, false);
assert.strictEqual(L.deriveBagView(Object.assign({}, bb, { playing: true, rate: 2 })).text, 'Playing · 2×');
assert.strictEqual(L.deriveBagView(Object.assign({}, bb, { playing: true, paused: true })).tone, 'warn');
assert.strictEqual(L.deriveRecorderView({ recording: true, hasName: true, topicCount: 3, elapsedSec: 3 }).text, 'Recording · 0:03');
assert.strictEqual(L.deriveRecorderView({ recording: false, hasName: true, topicCount: 0, elapsedSec: 0 }).buttons.record, false);
assert.strictEqual(L.deriveFileView({ loaded: true, playing: false, paused: false, converting: true, savingBag: false, progress: 28 }).text, 'Saving bag · 28%');
console.log('PASS check_status_logic');
