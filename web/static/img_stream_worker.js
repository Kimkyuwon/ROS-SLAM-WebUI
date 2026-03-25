/**
 * img_stream_worker.js — Image Binary WebSocket Worker
 *
 * Python 백엔드(포트 8081)에서 JPEG 바이너리 스트림을 수신하여
 * createImageBitmap()으로 GPU 가속 디코딩 후 메인 스레드에 전달한다.
 *
 * Binary 패킷 포맷 (Python → Worker, little-endian):
 *   [3B]  magic   = 'IMG'
 *   [1B]  version = 1
 *   [4B]  uint32  topic_name 바이트 길이
 *   [4B]  uint32  jpeg_data 바이트 길이
 *   [N B] topic_name (UTF-8)
 *   [L B] JPEG data
 *
 * Commands from main thread:
 *   { cmd: 'connect',          url: string }
 *   { cmd: 'subscribe',        topicName: string }
 *   { cmd: 'unsubscribe',      topicName: string }
 *   { cmd: 'unsubscribeAll' }  — Image WS 구독 전부 해제
 *
 * Messages to main thread:
 *   { type: 'imgframe',   topicName, bitmap }  — ImageBitmap (transferable)
 *   { type: 'connected' }
 *   { type: 'disconnected' }
 */

// ── WebSocket 상태 ────────────────────────────────────────────────────────────
let ws      = null;
let wsUrl   = null;
let wsReady = false;

// ── 구독 중인 토픽 집합 ───────────────────────────────────────────────────────
const subscriptions = new Set();

// TextDecoder 재사용
const _decoder = new TextDecoder('utf-8');

// ── 디코딩 중인 토픽별 Promise (중복 디코딩 방지) ───────────────────────────
// topicName → true (디코딩 진행 중)
const _decoding = new Map();

// ─────────────────────────────────────────────────────────────────────────────
// 메인 스레드 명령 처리
// ─────────────────────────────────────────────────────────────────────────────
self.onmessage = function (e) {
    const { cmd } = e.data;

    if (cmd === 'connect') {
        wsUrl = e.data.url;
        _connectWs();

    } else if (cmd === 'subscribe') {
        subscriptions.add(e.data.topicName);
        if (wsReady) {
            ws.send(JSON.stringify({ cmd: 'subscribe_image', topic: e.data.topicName }));
        }

    } else if (cmd === 'unsubscribe') {
        subscriptions.delete(e.data.topicName);
        _decoding.delete(e.data.topicName);
        if (wsReady) {
            ws.send(JSON.stringify({ cmd: 'unsubscribe_image', topic: e.data.topicName }));
        }

    } else if (cmd === 'unsubscribeAll') {
        const topics = Array.from(subscriptions);
        subscriptions.clear();
        _decoding.clear();
        if (wsReady) {
            for (let i = 0; i < topics.length; i++) {
                try {
                    ws.send(JSON.stringify({ cmd: 'unsubscribe_image', topic: topics[i] }));
                } catch (err) { /* ignore */ }
            }
        }
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// WebSocket 연결 관리
// ─────────────────────────────────────────────────────────────────────────────
function _connectWs() {
    ws = new WebSocket(wsUrl);
    ws.binaryType = 'arraybuffer';

    ws.onopen = function () {
        wsReady = true;
        self.postMessage({ type: 'connected' });
        // 연결 전에 요청됐던 구독을 모두 전송
        for (const topicName of subscriptions) {
            ws.send(JSON.stringify({ cmd: 'subscribe_image', topic: topicName }));
        }
    };

    ws.onmessage = function (evt) {
        if (evt.data instanceof ArrayBuffer) {
            _handleBinaryMessage(evt.data);
        }
        // text 메시지는 이 Worker에서는 사용하지 않음
    };

    ws.onerror = function () {
        wsReady = false;
    };

    ws.onclose = function () {
        wsReady = false;
        self.postMessage({ type: 'disconnected' });
        setTimeout(_connectWs, 3000);   // 자동 재연결
    };
}

// ─────────────────────────────────────────────────────────────────────────────
// Binary 패킷 파싱 및 ImageBitmap 디코딩
// ─────────────────────────────────────────────────────────────────────────────
function _handleBinaryMessage(buf) {
    if (buf.byteLength < 12) return;

    const view = new DataView(buf);

    // magic 'IMG' 확인
    if (view.getUint8(0) !== 0x49 ||  // 'I'
        view.getUint8(1) !== 0x4D ||  // 'M'
        view.getUint8(2) !== 0x47)    // 'G'
        return;

    // version = getUint8(3), 현재 미사용

    const topicLen = view.getUint32(4, true);
    const jpegLen  = view.getUint32(8, true);

    const headerSize = 12;
    if (buf.byteLength < headerSize + topicLen + jpegLen) return;

    const topicName = _decoder.decode(new Uint8Array(buf, headerSize, topicLen));

    // 구독하지 않은 토픽이면 무시
    if (!subscriptions.has(topicName)) return;

    // 이미 같은 토픽을 디코딩 중이면 스킵 (백프레셔 방지)
    if (_decoding.get(topicName)) return;
    _decoding.set(topicName, true);

    // JPEG bytes → Blob → ImageBitmap (GPU 가속 디코딩)
    const jpegBytes = new Uint8Array(buf, headerSize + topicLen, jpegLen);
    const blob      = new Blob([jpegBytes], { type: 'image/jpeg' });

    createImageBitmap(blob).then(function (bitmap) {
        _decoding.delete(topicName);
        // ImageBitmap은 transferable이므로 zero-copy로 전달
        self.postMessage({ type: 'imgframe', topicName, bitmap }, [bitmap]);
    }).catch(function (err) {
        _decoding.delete(topicName);
        // JPEG 디코딩 실패는 조용히 무시 (손상된 프레임)
    });
}
