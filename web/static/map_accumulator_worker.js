'use strict';

const VOXEL_SIZE = 0.15;
const MAX_POINTS = 500000;
const UPDATE_INTERVAL_MS = 1000;

const voxelMap = new Map();
const posBuffer = new Float32Array(MAX_POINTS * 3);
const colBuffer = new Float32Array(MAX_POINTS * 3);
let writeHead = 0;
let totalCount = 0;
let lastFlushTime = 0;

function voxelKey(x, y, z) {
    return `${Math.floor(x / VOXEL_SIZE)}_${Math.floor(y / VOXEL_SIZE)}_${Math.floor(z / VOXEL_SIZE)}`;
}

self.onmessage = function (e) {
    const { cmd } = e.data;

    if (cmd === 'addPoints') {
        const { positions, colors } = e.data;
        const n = positions.length / 3;
        for (let i = 0; i < n; i++) {
            const x = positions[i * 3];
            const y = positions[i * 3 + 1];
            const z = positions[i * 3 + 2];
            const key = voxelKey(x, y, z);
            if (voxelMap.has(key)) continue;

            const slot = writeHead % MAX_POINTS;
            voxelMap.set(key, slot);
            posBuffer[slot * 3]     = x;
            posBuffer[slot * 3 + 1] = y;
            posBuffer[slot * 3 + 2] = z;
            colBuffer[slot * 3]     = colors[i * 3];
            colBuffer[slot * 3 + 1] = colors[i * 3 + 1];
            colBuffer[slot * 3 + 2] = colors[i * 3 + 2];
            writeHead++;
            totalCount = Math.min(totalCount + 1, MAX_POINTS);
        }

        const now = Date.now();
        if (now - lastFlushTime >= UPDATE_INTERVAL_MS) {
            lastFlushTime = now;
            const count = totalCount;
            if (count === 0) return;
            const pos = posBuffer.slice(0, count * 3);
            const col = colBuffer.slice(0, count * 3);
            self.postMessage({ cmd: 'flush', positions: pos, colors: col, count },
                [pos.buffer, col.buffer]);
        }
    }

    if (cmd === 'clear') {
        voxelMap.clear();
        writeHead = 0;
        totalCount = 0;
    }
};
