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

    return { setStatusBadge, renderStepper, setEnabled, setBtnLabel, setReadonly, showToast };
})();
window.uiKit = uiKit;
