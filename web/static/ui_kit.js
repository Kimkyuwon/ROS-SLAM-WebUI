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

    let dialogSeq = 0;

    function confirmDialog({ title, message, okText = '확인', cancelText = '취소', tone = 'warn' }) {
        return new Promise((resolve) => {
            const seq = ++dialogSeq;
            const prevFocus = document.activeElement;
            const backdrop = document.createElement('div');
            backdrop.className = 'ui-dialog-backdrop';
            const dialog = document.createElement('div');
            dialog.className = 'ui-dialog';
            dialog.dataset.tone = tone;
            dialog.setAttribute('role', 'alertdialog');
            dialog.setAttribute('aria-modal', 'true');
            dialog.setAttribute('aria-labelledby', `ui-dialog-title-${seq}`);
            dialog.setAttribute('aria-describedby', `ui-dialog-msg-${seq}`);
            const heading = document.createElement('h3');
            heading.id = `ui-dialog-title-${seq}`;
            heading.className = 'ui-dialog-title';
            heading.textContent = title;
            const body = document.createElement('p');
            body.id = `ui-dialog-msg-${seq}`;
            body.className = 'ui-hint ui-dialog-message';
            body.textContent = message;
            const actions = document.createElement('div');
            actions.className = 'ui-dialog-actions';
            const cancel = document.createElement('button');
            cancel.type = 'button';
            cancel.className = 'ui-btn ui-btn-ghost';
            cancel.textContent = cancelText;
            const ok = document.createElement('button');
            ok.type = 'button';
            ok.className = 'ui-btn';
            ok.dataset.tone = tone;
            ok.textContent = okText;
            actions.append(cancel, ok);
            dialog.append(heading, body, actions);
            backdrop.appendChild(dialog);

            const close = (value) => {
                document.removeEventListener('keydown', onKey, true);
                backdrop.remove();
                if (prevFocus && prevFocus.focus) prevFocus.focus();
                resolve(value);
            };
            const onKey = (e) => {
                if (e.key === 'Escape') {
                    e.preventDefault();
                    e.stopPropagation();
                    close(false);
                } else if (e.key === 'Tab') {
                    e.preventDefault();
                    (document.activeElement === cancel ? ok : cancel).focus();
                }
            };
            cancel.onclick = () => close(false);
            ok.onclick = () => close(true);
            document.addEventListener('keydown', onKey, true);
            document.body.appendChild(backdrop);
            cancel.focus();
        });
    }

    return { setStatusBadge, renderStepper, setEnabled, setBtnLabel, setReadonly, showToast, renderSparkline,
        confirmDialog };
})();
window.uiKit = uiKit;
