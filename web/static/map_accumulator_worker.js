'use strict';

// ─────────────────────────────────────────────────────────────────────────
//  누적 점군(map accumulator) 워커
//  - VOXEL_SIZE 해상도로 중복 제거하며 /cloud_registered 프레임을 누적
//  - 공간을 BLOCK_SIZE 격자 블록으로 나눠 관리
//  - 현재 pose 기준 ACCUM_RANGE_M 반경을 벗어난 블록은 제거(sliding window)
//    → 누적 점 수가 궤적 길이에 비례해 무한 증가하지 않고 상한이 생겨
//      GPU 업로드/렌더 latency 가 일정하게 유지됨
// ─────────────────────────────────────────────────────────────────────────

// ── 조정 가능한 파라미터 ────────────────────────────────────────────────
const VOXEL_SIZE = 0.15;          // 중복 제거(다운샘플) 해상도 (m)
const BLOCK_SIZE = 10.0;          // 공간 블록 한 변 길이 (m)
const ACCUM_RANGE_M = 80.0;       // 현재 pose 기준 누적 유지 반경 (m)
const UPDATE_INTERVAL_MS = 1000;  // flush + 정리(prune) 주기 (ms)
const MAX_POINTS = 1500000;       // 안전 상한 (초과 시 먼 블록부터 강제 제거)

// blockKey -> { cx, cy, voxels:Set<string>, pos:number[], col:number[] }
const blocks = new Map();
let totalPoints = 0;
let lastFlushTime = 0;
let lastPose = null;   // [x, y] 마지막으로 알려진 수평 pose

function blockKey(bx, by) {
    return bx + '_' + by;
}

function voxelKey(x, y, z) {
    return Math.floor(x / VOXEL_SIZE) + '_' +
           Math.floor(y / VOXEL_SIZE) + '_' +
           Math.floor(z / VOXEL_SIZE);
}

function getBlock(x, y) {
    const bx = Math.floor(x / BLOCK_SIZE);
    const by = Math.floor(y / BLOCK_SIZE);
    const key = blockKey(bx, by);
    let b = blocks.get(key);
    if (!b) {
        b = {
            cx: (bx + 0.5) * BLOCK_SIZE,   // 블록 중심 (수평 거리 계산용)
            cy: (by + 0.5) * BLOCK_SIZE,
            voxels: new Set(),
            pos: [],
            col: []
        };
        blocks.set(key, b);
    }
    return b;
}

// 현재 pose 기준 반경 밖(블록 중심 거리 > ACCUM_RANGE_M + 여유) 블록 제거
function pruneBlocks() {
    if (!lastPose) return;
    const px = lastPose[0], py = lastPose[1];
    // 블록 중심 기준이므로 블록 크기만큼 여유를 둬서 경계 점 손실 방지
    const limit = ACCUM_RANGE_M + BLOCK_SIZE;
    const limitSq = limit * limit;
    for (const [key, b] of blocks) {
        const dx = b.cx - px, dy = b.cy - py;
        if (dx * dx + dy * dy > limitSq) {
            totalPoints -= b.pos.length / 3;
            blocks.delete(key);
        }
    }
}

// 안전 상한 초과 시 pose 에서 먼 블록부터 제거
function enforceCap() {
    if (totalPoints <= MAX_POINTS || !lastPose) return;
    const px = lastPose[0], py = lastPose[1];
    const arr = [];
    for (const [key, b] of blocks) {
        const dx = b.cx - px, dy = b.cy - py;
        arr.push([dx * dx + dy * dy, key, b]);
    }
    arr.sort((a, b) => b[0] - a[0]); // 먼 것부터
    for (let i = 0; i < arr.length && totalPoints > MAX_POINTS; i++) {
        const b = arr[i][2];
        totalPoints -= b.pos.length / 3;
        blocks.delete(arr[i][1]);
    }
}

function flush() {
    let count = 0;
    for (const b of blocks.values()) count += b.pos.length / 3;
    if (count === 0) return;
    const positions = new Float32Array(count * 3);
    const colors = new Float32Array(count * 3);
    let o = 0;
    for (const b of blocks.values()) {
        positions.set(b.pos, o);
        colors.set(b.col, o);
        o += b.pos.length;
    }
    self.postMessage({ cmd: 'flush', positions, colors, count },
        [positions.buffer, colors.buffer]);
}

self.onmessage = function (e) {
    const { cmd } = e.data;

    if (cmd === 'addPoints') {
        const { positions, colors, pose } = e.data;
        if (pose) lastPose = [pose[0], pose[1]];

        const n = positions.length / 3;
        // pose 미확보 시 이번 프레임 중심을 임시 pose 로 사용 (tf 없이도 window 동작)
        if (!lastPose && n > 0) {
            let sx = 0, sy = 0;
            for (let i = 0; i < n; i++) { sx += positions[i * 3]; sy += positions[i * 3 + 1]; }
            lastPose = [sx / n, sy / n];
        }

        for (let i = 0; i < n; i++) {
            const x = positions[i * 3];
            const y = positions[i * 3 + 1];
            const z = positions[i * 3 + 2];
            const b = getBlock(x, y);
            const vk = voxelKey(x, y, z);
            if (b.voxels.has(vk)) continue;   // 중복 voxel 제거
            b.voxels.add(vk);
            b.pos.push(x, y, z);
            b.col.push(colors[i * 3], colors[i * 3 + 1], colors[i * 3 + 2]);
            totalPoints++;
        }

        const now = Date.now();
        if (now - lastFlushTime >= UPDATE_INTERVAL_MS) {
            lastFlushTime = now;
            pruneBlocks();
            enforceCap();
            flush();
        }
        return;
    }

    if (cmd === 'setPose') {
        const { pose } = e.data;
        if (pose) lastPose = [pose[0], pose[1]];
        return;
    }

    if (cmd === 'clear') {
        blocks.clear();
        totalPoints = 0;
        lastPose = null;
        lastFlushTime = 0;
        return;
    }
};
