'use strict';

// 메인 스레드(Three.js 렌더)와 분리해 /api/ping RTT 측정
const PING_SAMPLES = 3;

async function measurePing() {
    let minLatency = Infinity;
    for (let i = 0; i < PING_SAMPLES; i++) {
        try {
            const t0 = performance.now();
            const response = await fetch('/api/ping', { cache: 'no-store' });
            const dt = performance.now() - t0;
            if (response.ok && dt < minLatency) minLatency = dt;
        } catch (_) { /* 개별 실패는 무시하고 나머지 샘플 계속 */ }
    }

    if (isFinite(minLatency)) {
        postMessage({ type: 'latency', ms: minLatency });
    } else {
        postMessage({ type: 'latency', ms: null });
    }
}

self.onmessage = (e) => {
    if (e.data === 'ping') {
        measurePing();
    }
};
