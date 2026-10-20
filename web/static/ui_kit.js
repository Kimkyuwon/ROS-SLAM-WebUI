// ui_kit.js — 도메인을 모르는 작은 DOM 헬퍼
const uiKit = (() => {
    const TOAST_LIMIT = 4;
    const TOAST_TIMEOUT_MS = 4000;

    function setStatusBadge(el, tone, text) {
        if (!el) return;
        el.dataset.tone = tone;
        const label = el.querySelector('.ui-status-text');
        if (label) label.textContent = text;
    }

    function renderStepper(olEl, steps, current) {
        if (!olEl) return;
        olEl.replaceChildren(...steps.map((name, i) => {
            const li = document.createElement('li');
            li.className = 'ui-stepper-item';
            li.dataset.state = i < current ? 'done' : (i === current ? 'current' : 'todo');
            if (i === current) li.setAttribute('aria-current', 'step');
            const mark = document.createElement('span');
            mark.className = 'ui-stepper-mark';
            mark.textContent = i < current ? '✓' : String(i + 1);
            mark.setAttribute('aria-hidden', 'true');
            li.append(mark, document.createTextNode(name));
            return li;
        }));
    }

    function setEnabled(el, enabled, reasonText = '') {
        if (!el) return;
        el.disabled = !enabled;
        if (reasonText) el.title = enabled ? '' : reasonText;
    }

    function setBtnLabel(btn, text) {
        if (!btn) return;
        const label = btn.querySelector('.ui-btn-label');
        if (label) label.textContent = text; else btn.textContent = text;
    }

    const SPARKLINE_POINTS = 60;

    // viewBox 0 0 100 30 기준. 최근 값이 오른쪽 끝에 오도록 60칸에 맞춘다
    function renderSparkline(svgEl, values, max) {
        if (!svgEl) return;
        let line = svgEl.querySelector('polyline');
        if (!values || !values.length) {
            if (line) line.remove();
            return;
        }
        if (!line) {
            line = document.createElementNS('http://www.w3.org/2000/svg', 'polyline');
            line.setAttribute('vector-effect', 'non-scaling-stroke');
            svgEl.appendChild(line);
        }
        const recent = values.slice(-SPARKLINE_POINTS);
        const top = max > 0 ? max : (Math.max(...recent) * 1.2 || 1);
        const step = 100 / (SPARKLINE_POINTS - 1);
        const offset = SPARKLINE_POINTS - recent.length;
        line.setAttribute('points', recent.map((v, i) => {
            const y = 29 - Math.min(Math.max(v / top, 0), 1) * 28;
            return `${((offset + i) * step).toFixed(2)},${y.toFixed(2)}`;
        }).join(' '));
    }

    function setReadonly(container, readonly) {
        if (!container) return;
        container.dataset.readonly = readonly ? 'true' : 'false';
        container.querySelectorAll('input, select, textarea').forEach(el => { el.disabled = readonly; });
    }

    function toastStack() {
        let stack = document.getElementById('ui-toast-stack');
        if (!stack) {
            stack = document.createElement('div');
            stack.id = 'ui-toast-stack';
            stack.className = 'ui-toast-stack';
            stack.setAttribute('aria-live', 'polite');
            stack.setAttribute('role', 'status');
            document.body.appendChild(stack);
        }
        return stack;
    }

    function showToast(message, tone = 'info', { timeoutMs = TOAST_TIMEOUT_MS } = {}) {
        const stack = toastStack();
        while (stack.children.length >= TOAST_LIMIT) stack.firstElementChild.remove();
        const toast = document.createElement('div');
        toast.className = 'ui-toast';
        toast.dataset.tone = tone;
        toast.setAttribute('role', tone === 'stop' ? 'alert' : 'status');
        const text = document.createElement('p');
        text.textContent = String(message);
        const close = document.createElement('button');
        close.type = 'button';
        close.className = 'ui-toast-close';
        close.setAttribute('aria-label', '알림 닫기');
        close.textContent = '×';
        close.onclick = () => toast.remove();
        toast.append(text, close);
        stack.appendChild(toast);
        if (tone !== 'stop') setTimeout(() => toast.remove(), timeoutMs);
        return toast;
    }

    return { setStatusBadge, renderStepper, setEnabled, setBtnLabel, setReadonly, showToast, renderSparkline };
})();
window.uiKit = uiKit;
