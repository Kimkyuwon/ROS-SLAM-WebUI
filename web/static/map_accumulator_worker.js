'use strict';

// ─────────────────────────────────────────────────────────────────────────
//  누적 점군(map accumulator) 워커
//  - PCL VoxelGrid 방식: floor(x/leaf) 인덱스별 centroid(평균) 누적
//  - FAST_LIO publish_map crop과 동일: 원본(globalVoxels)은 유지, flush 시에만
//    pose 기준 ACCUM_RANGE_M 범위로 crop해 표시용 점만 전송
//  - 영구 삭제는 MAX_POINTS 초과 시 pose에서 먼 복셀부터 (메모리 상한)
// ─────────────────────────────────────────────────────────────────────────

const VOXEL_SIZE = 0.3;           // PCL VoxelGrid leaf size (m)
const ACCUM_RANGE_M = 60.0;       // 시각화 crop 반경 (m) — 저장소 삭제에 사용하지 않음
const UPDATE_INTERVAL_MS = 1000;  // flush 주기 (ms)
const MAX_POINTS = 400000;        // 메모리 상한 (초과 시 먼 복셀부터 강제 제거)

const RANGE_SQ = ACCUM_RANGE_M * ACCUM_RANGE_M;

// vk -> { sx, sy, sz, count, scr, scg, scb }  (centroid 누적)
const globalVoxels = new Map();
let lastFlushTime = 0;
let lastPose = null;   // [x, y] 마지막으로 알려진 수평 pose

function voxelKey(x, y, z) {
    return Math.floor(x / VOXEL_SIZE) + '_' +
           Math.floor(y / VOXEL_SIZE) + '_' +
           Math.floor(z / VOXEL_SIZE);
}

function pointInRange(x, y) {
    if (!lastPose) return true;
    const dx = x - lastPose[0];
    const dy = y - lastPose[1];
    return dx * dx + dy * dy <= RANGE_SQ;
}

// PCL VoxelGrid::filter — 동일 복셀 내 점들의 centroid·색상 평균 누적
function accumulatePoint(x, y, z, cr, cg, cb) {
    const vk = voxelKey(x, y, z);
    let v = globalVoxels.get(vk);
    if (!v) {
        globalVoxels.set(vk, { sx: x, sy: y, sz: z, count: 1, scr: cr, scg: cg, scb: cb });
        return;
    }
    v.sx += x;
    v.sy += y;
    v.sz += z;
    v.scr += cr;
    v.scg += cg;
    v.scb += cb;
    v.count++;
}

// 메모리 상한 초과 시 pose에서 먼 복셀(centroid 기준)부터 제거
function enforceCap() {
    if (globalVoxels.size <= MAX_POINTS || !lastPose) return;
    const px = lastPose[0], py = lastPose[1];
    const arr = [];
    for (const [vk, v] of globalVoxels) {
        const cx = v.sx / v.count;
        const cy = v.sy / v.count;
        const dx = cx - px, dy = cy - py;
        arr.push([dx * dx + dy * dy, vk]);
    }
    arr.sort((a, b) => b[0] - a[0]);
    for (let i = 0; i < arr.length && globalVoxels.size > MAX_POINTS; i++) {
        globalVoxels.delete(arr[i][1]);
    }
}

// flush: 저장소는 유지한 채, pose 기준 crop + centroid 출력
function flush() {
    enforceCap();

    const entries = [];
    for (const v of globalVoxels.values()) {
        const x = v.sx / v.count;
        const y = v.sy / v.count;
        const z = v.sz / v.count;
        if (!pointInRange(x, y)) continue;
        entries.push({
            pos: [x, y, z],
            col: [v.scr / v.count, v.scg / v.count, v.scb / v.count]
        });
    }

    const outCount = entries.length;
    if (outCount === 0) return;

    const positions = new Float32Array(outCount * 3);
    const colors = new Float32Array(outCount * 3);
    let o = 0;
    for (const p of entries) {
        positions[o]     = p.pos[0];
        positions[o + 1] = p.pos[1];
        positions[o + 2] = p.pos[2];
        colors[o]     = p.col[0];
        colors[o + 1] = p.col[1];
        colors[o + 2] = p.col[2];
        o += 3;
    }

    self.postMessage({
        cmd: 'flush',
        positions: positions,
        colors: colors,
        count: outCount
    });
}

self.onmessage = function (e) {
    const { cmd } = e.data;

    if (cmd === 'addPoints') {
        const { positions, colors, pose } = e.data;
        if (pose) lastPose = [pose[0], pose[1]];

        const n = positions.length / 3;
        if (!lastPose && n > 0) {
            let sx = 0, sy = 0;
            for (let i = 0; i < n; i++) {
                sx += positions[i * 3];
                sy += positions[i * 3 + 1];
            }
            lastPose = [sx / n, sy / n];
        }

        for (let i = 0; i < n; i++) {
            accumulatePoint(
                positions[i * 3],
                positions[i * 3 + 1],
                positions[i * 3 + 2],
                colors[i * 3],
                colors[i * 3 + 1],
                colors[i * 3 + 2]
            );
        }

        enforceCap();

        const now = Date.now();
        if (now - lastFlushTime >= UPDATE_INTERVAL_MS) {
            lastFlushTime = now;
            flush();
        }
        return;
    }

    if (cmd === 'setPose') {
        const { pose } = e.data;
        if (pose) {
            lastPose = [pose[0], pose[1]];
            const now = Date.now();
            if (now - lastFlushTime >= UPDATE_INTERVAL_MS) {
                lastFlushTime = now;
                flush();
            }
        }
        return;
    }

    if (cmd === 'clear') {
        globalVoxels.clear();
        lastPose = null;
        lastFlushTime = 0;
        return;
    }
};
