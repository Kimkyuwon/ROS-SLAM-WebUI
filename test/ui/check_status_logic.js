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
console.log('PASS check_status_logic');
