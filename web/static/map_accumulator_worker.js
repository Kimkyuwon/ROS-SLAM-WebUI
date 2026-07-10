'use strict';

// ─────────────────────────────────────────────────────────────────────────
//  누적 점군(map accumulator) 워커
//  - PCL VoxelGrid 방식: floor(x/leaf) 인덱스별 centroid(평균) 누적
//  - sliding window는 **cloud centroid만** 사용 (odom setPose 무시)
//  - **100초 age window**: 복셀 lastSeen 기준 오래된 것부터 삭제 (메모리 상한 주력)
//  - pose 60m crop-on-flush 유지 (출력 crop + 저장소 trim)
//  - flush 시 z축 elevation rainbow 색상 적용 (입력 색상 무시)
// ─────────────────────────────────────────────────────────────────────────

const VOXEL_SIZE = 1.0;           // PCL VoxelGrid leaf size (m)
const ACCUM_RANGE_M = 60.0;       // cloud centroid 기준 sliding window 반경 (m)
const UPDATE_INTERVAL_MS = 1000;  // flush 주기 (ms) — 메인 addPoints 1Hz와 맞춤
const AGE_WINDOW_MS = 100000;     // 100초 — 이보다 오래된 복셀 삭제
const MAX_POINTS = 120000;        // 안전 상한 (age window가 주력, 초과 시 먼 복셀 제거)

const RANGE_SQ = ACCUM_RANGE_M * ACCUM_RANGE_M;

// vk -> { sx, sy, sz, count, t }  (centroid 누적 + lastSeen ms)
const globalVoxels = new Map();
let lastFlushTime = 0;
let lastPose = null;   // [x, y] cloud centroid 기반 sliding window 중심
let lastPruneTime = 0;

function voxelKey(x, y, z) {
    return Math.floor(x / VOXEL_SIZE) + '_' +
           Math.floor(y / VOXEL_SIZE) + '_' +
           Math.floor(z / VOXEL_SIZE);
}

// script.js SlamLiveViewer._rainbowColor 와 동일 (z elevation map)
function rainbowColor(t) {
    const h = (1 - t) * 240;
    const c = 1;
    const x = c * (1 - Math.abs(((h / 60) % 2) - 1));
    let r = 0, g = 0, b = 0;
    if      (h < 60)  { r = c; g = x; b = 0; }
    else if (h < 120) { r = x; g = c; b = 0; }
    else if (h < 180) { r = 0; g = c; b = x; }
    else if (h < 240) { r = 0; g = x; b = c; }
    else if (h < 300) { r = x; g = 0; b = c; }
    else              { r = c; g = 0; b = x; }
    return [r, g, b];
}

function asFloat32(positions) {
    if (!positions) return null;
    if (positions instanceof Float32Array) return positions;
    if (positions instanceof ArrayBuffer) return new Float32Array(positions);
    if (ArrayBuffer.isView(positions)) {
        return new Float32Array(
            positions.buffer,
            positions.byteOffset,
            Math.floor(positions.byteLength / 4)
        );
    }
    try {
        return new Float32Array(positions);
    } catch (e) {
        return null;
    }
}

function cloudCentroidXY(positions, n) {
    let sx = 0, sy = 0;
    for (let i = 0; i < n; i++) {
        sx += positions[i * 3];
        sy += positions[i * 3 + 1];
    }
    return [sx / n, sy / n];
}

function distSq2(ax, ay, bx, by) {
    const dx = ax - bx, dy = ay - by;
    return dx * dx + dy * dy;
}

function pointInRange(x, y) {
    if (!lastPose) return true;
    return distSq2(x, y, lastPose[0], lastPose[1]) <= RANGE_SQ;
}

// PCL VoxelGrid::filter — 동일 복셀 내 점들의 centroid 누적 + lastSeen 갱신
function accumulatePoint(x, y, z, nowMs) {
    if (!isFinite(x) || !isFinite(y) || !isFinite(z)) return;
    const vk = voxelKey(x, y, z);
    let v = globalVoxels.get(vk);
    if (!v) {
        globalVoxels.set(vk, { sx: x, sy: y, sz: z, count: 1, t: nowMs });
        return;
    }
    v.sx += x;
    v.sy += y;
    v.sz += z;
    v.count++;
    v.t = nowMs;
}

/** 100초보다 오래된 복셀 삭제 (age sliding window) */
function pruneByAge(nowMs) {
    const cutoff = nowMs - AGE_WINDOW_MS;
    for (const [vk, v] of globalVoxels) {
        if ((v.t || 0) < cutoff) {
            globalVoxels.delete(vk);
        }
    }
}

// 메모리 상한 초과 시 window 중심에서 먼 복셀부터 제거 (age window 보조)
function enforceCap() {
    if (globalVoxels.size <= MAX_POINTS || !lastPose) return;
    const px = lastPose[0], py = lastPose[1];
    const arr = [];
    for (const [vk, v] of globalVoxels) {
        const cx = v.sx / v.count;
        const cy = v.sy / v.count;
        arr.push([distSq2(cx, cy, px, py), vk]);
    }
    arr.sort((a, b) => b[0] - a[0]);
    for (let i = 0; i < arr.length && globalVoxels.size > MAX_POINTS; i++) {
        globalVoxels.delete(arr[i][1]);
    }
}

// 저장소에서 window 밖 복셀 정리 (flush 시에만)
function pruneOutOfRange() {
    if (!lastPose) return;
    if (!isFinite(lastPose[0]) || !isFinite(lastPose[1])) return;
    const px = lastPose[0], py = lastPose[1];
    for (const [vk, v] of globalVoxels) {
        const cx = v.sx / v.count;
        const cy = v.sy / v.count;
        if (distSq2(cx, cy, px, py) > RANGE_SQ) {
            globalVoxels.delete(vk);
        }
    }
}

function maybeFlush(nowMs) {
    if (nowMs - lastFlushTime < UPDATE_INTERVAL_MS) return;
    flush(nowMs);
}

// flush: window 내 centroid만 출력 + z elevation rainbow
// 빈 flush여도 lastFlushTime 갱신 → 매 addPoints마다 전체 순회 폭주 방지
// (빈 결과는 메인에 보내지 않아 기존 geometry 유지)
function flush(nowMs) {
    pruneByAge(nowMs);
    enforceCap();

    const nVox = globalVoxels.size;
    if (nVox === 0) {
        lastFlushTime = nowMs;
        return false;
    }

    // 1-pass: 범위 내 개수·z범위 (임시 객체 배열 없이 typed 버퍼에 직접 기록)
    let inCount = 0;
    let minZ = Infinity, maxZ = -Infinity;
    for (const v of globalVoxels.values()) {
        const x = v.sx / v.count;
        const y = v.sy / v.count;
        if (!pointInRange(x, y)) continue;
        const z = v.sz / v.count;
        inCount++;
        if (z < minZ) minZ = z;
        if (z > maxZ) maxZ = z;
    }

    lastFlushTime = nowMs;
    if (inCount === 0) {
        pruneOutOfRange();
        return false;
    }

    const zRange = (maxZ - minZ) || 1;
    const positions = new Float32Array(inCount * 3);
    const colors = new Float32Array(inCount * 3);
    let o = 0;
    for (const v of globalVoxels.values()) {
        const x = v.sx / v.count;
        const y = v.sy / v.count;
        if (!pointInRange(x, y)) continue;
        const z = v.sz / v.count;
        positions[o]     = x;
        positions[o + 1] = y;
        positions[o + 2] = z;
        const t = (z - minZ) / zRange;
        const [r, g, b] = rainbowColor(t);
        colors[o]     = r;
        colors[o + 1] = g;
        colors[o + 2] = b;
        o += 3;
    }

    self.postMessage(
        { cmd: 'flush', positions: positions, colors: colors, count: inCount },
        [positions.buffer, colors.buffer]
    );
    pruneOutOfRange();
    return true;
}

self.onmessage = function (e) {
    const { cmd } = e.data;

    if (cmd === 'addPoints') {
        const positions = asFloat32(e.data.positions);
        if (!positions || positions.length < 3) return;

        const n = Math.floor(positions.length / 3);
        if (n < 1) return;

        const nowMs = Date.now();
        const centroid = cloudCentroidXY(positions, n);
        if (isFinite(centroid[0]) && isFinite(centroid[1])) {
            lastPose = centroid;
        }

        for (let i = 0; i < n; i++) {
            accumulatePoint(
                positions[i * 3],
                positions[i * 3 + 1],
                positions[i * 3 + 2],
                nowMs
            );
        }

        // age prune은 flush마다 + 가끔 add 직후에도 (메모리 조기 회수)
        if (nowMs - lastPruneTime >= UPDATE_INTERVAL_MS) {
            pruneByAge(nowMs);
            lastPruneTime = nowMs;
        }
        enforceCap();
        maybeFlush(nowMs);
        return;
    }

    // setPose: odom 보조 pose는 sliding window에 반영하지 않음 (호환용 no-op)
    if (cmd === 'setPose') {
        return;
    }

    if (cmd === 'clear') {
        globalVoxels.clear();
        lastPose = null;
        lastFlushTime = 0;
        lastPruneTime = 0;
        return;
    }
};
