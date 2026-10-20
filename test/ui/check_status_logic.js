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
console.log('PASS check_status_logic');
