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

    function isModalOpen(modal) {
        return Boolean(modal.style.display) && modal.style.display !== 'none';
    }

    const MODAL_FIRST_FIELD = '.modal-body input:not([type="hidden"]):not(:disabled), .modal-body select, .modal-body textarea';

    // 여는 함수가 9개라 style 변화를 보고 연 순간 첫 입력으로, 닫힌 순간 연 버튼으로 포커스를 옮긴다
    // (여는 코드가 먼저 입력에 focus()를 줄 수 있어 연 버튼은 모달 밖 마지막 포커스로 기억한다)
    let lastOutsideFocus = null;

    function trackModalFocus(modal) {
        let opener = null;
        let wasOpen = isModalOpen(modal);
        new MutationObserver(() => {
            const open = isModalOpen(modal);
            if (open === wasOpen) return;
            wasOpen = open;
            const active = document.activeElement;
            if (open) {
                opener = lastOutsideFocus;
                if (modal.contains(active)) return;
                const target = modal.querySelector(MODAL_FIRST_FIELD) || modal.querySelector('.modal-header .close');
                if (target) target.focus();
                return;
            }
            const focusLost = !active || active === document.body || modal.contains(active);
            if (opener && opener.isConnected && focusLost) opener.focus();
            opener = null;
        }).observe(modal, { attributes: true, attributeFilter: ['style'] });
    }

    // 기존 .modal은 JS가 style.display로 여닫으므로 마크업·핸들러는 그대로 두고 역할·ESC·포커스만 붙인다
    function enhanceModals() {
        document.querySelectorAll('.modal').forEach(modal => {
            modal.setAttribute('role', 'dialog');
            modal.setAttribute('aria-modal', 'true');
            const heading = modal.querySelector('.modal-header h2, .modal-header h3');
            if (heading) {
                if (!heading.id) heading.id = `${modal.id}-title`;
                modal.setAttribute('aria-labelledby', heading.id);
            }
            modal.querySelectorAll('.modal-header .close').forEach(btn => {
                if (btn.tagName === 'BUTTON') return;
                btn.setAttribute('role', 'button');
                btn.setAttribute('aria-label', '닫기');
                btn.tabIndex = 0;
                btn.addEventListener('keydown', (e) => {
                    if (e.key !== 'Enter' && e.key !== ' ') return;
                    e.preventDefault();
                    btn.click();
                });
            });
            trackModalFocus(modal);
        });
        document.addEventListener('focusin', (e) => {
            if (!e.target.closest('.modal, .ui-dialog-backdrop')) lastOutsideFocus = e.target;
        });
        document.addEventListener('keydown', (e) => {
            if (e.key !== 'Escape' || document.querySelector('.ui-dialog-backdrop')) return;
            const open = [...document.querySelectorAll('.modal')].filter(isModalOpen).pop();
            if (!open) return;
            const btn = open.querySelector('.modal-header .close, [data-dismiss]');
            if (btn) btn.click();
        });
    }

    return { setStatusBadge, renderStepper, setEnabled, setBtnLabel, setReadonly, showToast, renderSparkline,
        confirmDialog, enhanceModals };
})();
document.addEventListener('DOMContentLoaded', uiKit.enhanceModals);
window.uiKit = uiKit;

uiKit.inferToastTone = (text) => {
    if (/fail|error|실패|오류|cannot|not found|invalid/i.test(text)) return 'stop';
    if (/please|먼저|선택하세요|enter |no topics|없습니다/i.test(text)) return 'warn';
    if (/complete|saved|success|완료|started|stopped/i.test(text)) return 'go';
    return 'info';
};
window.alert = (message) => {
    const text = String(message ?? '');
    uiKit.showToast(text, uiKit.inferToastTone(text));
};
