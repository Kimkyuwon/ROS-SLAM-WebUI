/**
 * pc2_stream_worker.js  —  v2  (Python Backend Binary WebSocket)
 *
 * ┌──────────────────────────────────────────────────────────────────────────┐
 * │  v1 (구): Worker → rosbridge(9090) → JSON/base64 → 10 MB+/메시지         │
 * │  v2 (신): Worker → Python Backend(8081) → Binary → ~600 KB/메시지        │
 * │                                                                          │
 * │  변경점                                                                   │
 * │  ✔ JSON.parse / base64 atob 완전 제거                                     │
 * │  ✔ ArrayBuffer 직접 수신 (ws.binaryType = 'arraybuffer')                 │
 * │  ✔ XYZ + colorField(intensity/rgb)는 Python에서 이미 추출된 float32      │
 * │  ✔ 색상 계산(rainbow / solid / rgb)은 Worker에서 수행 (기존 로직 유지)  │
 * └──────────────────────────────────────────────────────────────────────────┘
 *
 * Binary 패킷 포맷 (Python → Worker, little-endian):
 *   [3B]  magic   = 'PC2'
 *   [1B]  version = 1
 *   [1B]  flags   (bit0=has_intensity, bit1=has_rgb)
 *   [4B]  uint32  topic_name 바이트 길이
 *   [4B]  uint32  frame_id 바이트 길이
 *   [4B]  uint32  point_count
 *   [N B] topic_name  (UTF-8)
 *   [M B] frame_id    (UTF-8)
 *   [count × 12 B]  XYZ float32 interleaved  (x0,y0,z0, x1,y1,z1, …)
 *   [count ×  4 B]  colorField float32        (intensity 또는 0.0)
 *   [count ×  4 B]  rgb uint32               (has_rgb=1 일 때만)
 *
 * Commands from main thread:
 *   { cmd: 'connect',        url: string }
 *   { cmd: 'subscribe',      topicName, colorMode, colorField, solidColor }
 *   { cmd: 'unsubscribe',    topicName }
 *   { cmd: 'unsubscribeAll' }   — 서버·Worker 구독 맵 전부 해제 (데이터셋 전환 시)
 *   { cmd: 'updateSettings', topicName, colorMode?, colorField?, solidColor? }
 *
 * Messages to main thread:
 *   { type: 'pc2frame', topicName, frameId, pos, col, count, newMin, newMax }
 *   { type: 'connected' }
 *   { type: 'disconnected' }
 */

const PC2_MAX_POINTS = 50_000;

// ── 사전 할당 ping-pong 버퍼 ─────────────────────────────────────────────────
const _bufPos = [
    new Float32Array(PC2_MAX_POINTS * 3),
    new Float32Array(PC2_MAX_POINTS * 3),
];
const _bufCol = [
    new Float32Array(PC2_MAX_POINTS * 3),
    new Float32Array(PC2_MAX_POINTS * 3),
];
let _bufIdx = 0;

// ── WebSocket 상태 ────────────────────────────────────────────────────────────
let ws      = null;
let wsUrl   = null;
let wsReady = false;

// ── 토픽별 설정 ──────────────────────────────────────────────────────────────
// topicName → { colorMode, colorField, solidR, solidG, solidB, adaptiveMin, adaptiveMax }
const subscriptions = new Map();

// TextDecoder 재사용 (Worker 스레드에서 GC 방지)
const _decoder = new TextDecoder('utf-8');

// ─────────────────────────────────────────────────────────────────────────────
// 메인 스레드 명령 처리
// ─────────────────────────────────────────────────────────────────────────────
self.onmessage = function (e) {
    const { cmd } = e.data;

    if (cmd === 'connect') {
        wsUrl = e.data.url;
        _connectWs();

    } else if (cmd === 'subscribe') {
        const { topicName, colorMode, colorField, solidColor } = e.data;
        const solid = solidColor || '#ffffff';
        const hex   = solid.replace('#', '');
        subscriptions.set(topicName, {
            colorMode:   colorMode  || 'rainbow',
            colorField:  colorField || 'intensity',
            solidColor:  solid,
            solidR: parseInt(hex.substring(0, 2), 16) / 255,
            solidG: parseInt(hex.substring(2, 4), 16) / 255,
            solidB: parseInt(hex.substring(4, 6), 16) / 255,
            adaptiveMin: 0,
            adaptiveMax: 1,
        });
        if (wsReady) _sendSubscribe(topicName);

    } else if (cmd === 'unsubscribe') {
        const { topicName } = e.data;
        subscriptions.delete(topicName);
        if (wsReady) {
            ws.send(JSON.stringify({ cmd: 'unsubscribe', topic: topicName }));
        }

    } else if (cmd === 'unsubscribeAll') {
        const topics = Array.from(subscriptions.keys());
        subscriptions.clear();
        if (wsReady) {
            for (let i = 0; i < topics.length; i++) {
                try {
                    ws.send(JSON.stringify({ cmd: 'unsubscribe', topic: topics[i] }));
                } catch (err) { /* ignore */ }
            }
        }

    } else if (cmd === 'updateSettings') {
        const { topicName, colorMode, colorField, solidColor } = e.data;
        const entry = subscriptions.get(topicName);
        if (!entry) return;
        if (colorMode  !== undefined) entry.colorMode  = colorMode;
        if (colorField !== undefined) entry.colorField = colorField;
        if (solidColor !== undefined) {
            const hex        = solidColor.replace('#', '');
            entry.solidColor = solidColor;
            entry.solidR     = parseInt(hex.substring(0, 2), 16) / 255;
            entry.solidG     = parseInt(hex.substring(2, 4), 16) / 255;
            entry.solidB     = parseInt(hex.substring(4, 6), 16) / 255;
        }
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// WebSocket 연결 관리
// ─────────────────────────────────────────────────────────────────────────────
function _connectWs() {
    ws = new WebSocket(wsUrl);
    ws.binaryType = 'arraybuffer';   // ← binary 수신의 핵심

    ws.onopen = function () {
        wsReady = true;
        self.postMessage({ type: 'connected' });
        // 연결 전에 요청됐던 구독을 모두 전송
        for (const topicName of subscriptions.keys()) {
            _sendSubscribe(topicName);
        }
    };

    // Python Backend는 text(JSON 메타데이터)와 binary(XYZ 데이터) 두 종류를 전송
    ws.onmessage = function(evt) {
        if (typeof evt.data === 'string') {
            _handleJsonMessage(evt);
        } else {
            _handleBinaryMessage(evt);
        }
    };

    ws.onerror = function () {
        self.postMessage({ type: 'error' });
    };

    ws.onclose = function () {
        wsReady = false;
        self.postMessage({ type: 'disconnected' });
        setTimeout(_connectWs, 3000);   // 자동 재연결
    };
}

function _sendSubscribe(topicName) {
    // Python Backend WebSocket 서버에 JSON 명령 전송
    ws.send(JSON.stringify({ cmd: 'subscribe', topic: topicName }));
}

// ─────────────────────────────────────────────────────────────────────────────
// JSON 메타데이터 메시지 처리 (text 수신 경로)
// Python Backend가 PointCloud2 수신마다 전송:
//   { type:'pc2meta', topic, stamp_sec, stamp_nanosec, frame_id, point_count }
// → 메인 스레드에 그대로 forward → CustomEvent로 Plot 탭에 공급
// ─────────────────────────────────────────────────────────────────────────────
function _handleJsonMessage(evt) {
    try {
        const msg = JSON.parse(evt.data);
        if (msg.type === 'pc2meta') {
            // 메인 스레드로 forward (Plot 탭의 CustomEvent 발생에 사용)
            self.postMessage({ type: 'pc2meta', ...msg });
        }
    } catch (e) {
        // JSON 파싱 실패는 무시
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Alignment-safe TypedArray 생성 helpers
// Float32Array / Uint32Array 는 byteOffset이 4의 배수여야 함.
// topic/frame_id 길이 합에 따라 offset이 비정렬될 수 있으므로
// 비정렬이면 Uint8Array로 복사 후 생성한다.
// ─────────────────────────────────────────────────────────────────────────────
function _safeFloat32(buf, byteOffset, elementCount) {
    const byteLen = elementCount * 4;
    if ((byteOffset & 3) === 0) {
        return new Float32Array(buf, byteOffset, elementCount);
    }
    // byteOffset이 4의 배수 아님 → 복사 후 생성 (zero-copy 불가)
    const tmp = new ArrayBuffer(byteLen);
    new Uint8Array(tmp).set(new Uint8Array(buf, byteOffset, byteLen));
    return new Float32Array(tmp);
}

function _safeUint32(buf, byteOffset, elementCount) {
    const byteLen = elementCount * 4;
    if ((byteOffset & 3) === 0) {
        return new Uint32Array(buf, byteOffset, elementCount);
    }
    const tmp = new ArrayBuffer(byteLen);
    new Uint8Array(tmp).set(new Uint8Array(buf, byteOffset, byteLen));
    return new Uint32Array(tmp);
}

// ─────────────────────────────────────────────────────────────────────────────
// Binary 메시지 파싱
// ─────────────────────────────────────────────────────────────────────────────
function _handleBinaryMessage(evt) {
    const buf = evt.data;
    if (!(buf instanceof ArrayBuffer) || buf.byteLength < 17) return;

    try {
        const view = new DataView(buf);

        // magic 'PC2' 확인
        if (view.getUint8(0) !== 0x50 ||   // 'P'
            view.getUint8(1) !== 0x43 ||   // 'C'
            view.getUint8(2) !== 0x32)     // '2'
            return;

        /* version */ view.getUint8(3);   // 현재 미사용 (version=1)
        const flags        = view.getUint8(4);
        const hasIntensity = (flags & 0x01) !== 0;
        const hasRgb       = (flags & 0x02) !== 0;

        const topicLen  = view.getUint32(5,  true);
        const frameLen  = view.getUint32(9,  true);
        const count     = view.getUint32(13, true);

        let offset = 17;
        const topicName = _decoder.decode(new Uint8Array(buf, offset, topicLen));
        offset += topicLen;
        const frameId   = _decoder.decode(new Uint8Array(buf, offset, frameLen));
        offset += frameLen;

        const entry = subscriptions.get(topicName);
        if (!entry) return;

        // XYZ float32 — alignment-safe 생성
        // offset = 17 + topicLen + frameLen 이 4의 배수가 아닐 수 있음
        const xyzRaw = _safeFloat32(buf, offset, count * 3);
        offset += count * 12;

        // colorField float32
        const colorRaw = _safeFloat32(buf, offset, count);
        offset += count * 4;

        // rgb uint32 (옵션)
        let rgbRaw = null;
        if (hasRgb) {
            rgbRaw = _safeUint32(buf, offset, count);
        }

        _colorAndPost(topicName, frameId, xyzRaw, colorRaw, rgbRaw,
                      hasIntensity, hasRgb, count, entry);

    } catch (e) {
        console.error('[PC2Worker] _handleBinaryMessage error:', e);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 색상 계산 + 메인 스레드에 postMessage
// ─────────────────────────────────────────────────────────────────────────────
function _colorAndPost(topicName, frameId,
                       xyzRaw, colorRaw, rgbRaw,
                       hasIntensity, hasRgb,
                       count, entry) {

    const n      = Math.min(count, PC2_MAX_POINTS);
    const bufIdx = _bufIdx;
    _bufIdx      = 1 - _bufIdx;

    const pos = _bufPos[bufIdx];
    const col = _bufCol[bufIdx];

    const { colorMode, colorField, solidR, solidG, solidB } = entry;
    let rangeMin  = entry.adaptiveMin;
    let rangeMax  = entry.adaptiveMax;
    const rangeSpan = (rangeMax > rangeMin) ? (rangeMax - rangeMin) : 1.0;
    let newMin = Infinity, newMax = -Infinity;

    for (let i = 0; i < n; i++) {
        const i3 = i * 3;

        // XYZ 복사
        pos[i3]     = xyzRaw[i3];
        pos[i3 + 1] = xyzRaw[i3 + 1];
        pos[i3 + 2] = xyzRaw[i3 + 2];

        // 색상 계산
        if (colorMode === 'rgb' && hasRgb && rgbRaw) {
            const rgb    = rgbRaw[i];
            col[i3]     = ((rgb >> 16) & 0xFF) / 255;
            col[i3 + 1] = ((rgb >>  8) & 0xFF) / 255;
            col[i3 + 2] = ( rgb        & 0xFF) / 255;

        } else if (colorMode === 'rainbow') {
            // colorField 값 결정
            let fv;
            if (colorField === 'intensity' && hasIntensity) {
                fv = colorRaw[i];
            } else if (colorField === 'x') {
                fv = xyzRaw[i3];
            } else if (colorField === 'y') {
                fv = xyzRaw[i3 + 1];
            } else if (colorField === 'z') {
                fv = xyzRaw[i3 + 2];
            } else {
                fv = colorRaw[i];   // fallback: intensity
            }
            if (!isFinite(fv)) fv = rangeMin;
            if (fv < newMin) newMin = fv;
            if (fv > newMax) newMax = fv;

            const t   = Math.min(1.0, Math.max(0.0, (fv - rangeMin) / rangeSpan));
            const hue = t * 0.75;                    // 빨강(0.0) → 보라(0.75)
            const h6  = hue * 6.0;
            const hi  = h6 | 0;
            const f_  = h6 - hi;
            let cr, cg, cb;
            switch (hi % 6) {
                case 0: cr = 1;       cg = f_;      cb = 0;       break;
                case 1: cr = 1 - f_;  cg = 1;       cb = 0;       break;
                case 2: cr = 0;       cg = 1;       cb = f_;      break;
                case 3: cr = 0;       cg = 1 - f_;  cb = 1;       break;
                case 4: cr = f_;      cg = 0;       cb = 1;       break;
                default: cr = 1;      cg = 0;       cb = 1 - f_;  break;
            }
            col[i3] = cr; col[i3 + 1] = cg; col[i3 + 2] = cb;

        } else {
            // solid color
            col[i3] = solidR; col[i3 + 1] = solidG; col[i3 + 2] = solidB;
        }
    }

    // adaptive 범위 갱신
    if (isFinite(newMin)) entry.adaptiveMin = newMin;
    if (isFinite(newMax)) entry.adaptiveMax = newMax;

    self.postMessage({
        type: 'pc2frame',
        topicName,
        frameId,
        pos: pos.subarray(0, n * 3),
        col: col.subarray(0, n * 3),
        count: n,
        newMin: isFinite(newMin) ? newMin : rangeMin,
        newMax: isFinite(newMax) ? newMax : rangeMax,
    });
}
