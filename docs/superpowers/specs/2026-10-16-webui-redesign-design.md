# Field Console UI 리디자인 스펙 (디자인/UI 사양)

- 승인된 목업: `docs/design/redesign-mock/` (승인일 2026-10-16). 목업 원본(스크린샷·번들·소스)은 저장소에 커밋하지 않았다. 메인 체크아웃 절대 경로 `/home/kkw/localization_ws/src/ros_slam_webui/docs/design/redesign-mock/`에 있다.
- 구현 계획: `docs/superpowers/plans/2026-10-16-webui-redesign.md`
- 이 문서는 계획서 부록 A(A.1~A.6)를 그대로 옮긴 것이다.

## 부록 A. 디자인/UI 사양

### A.1 CSS 변수 (`ui_theme.css` 첫 부분 — Task 0.4)

```css
/* ui_theme.css — Field Console 디자인 토큰과 공용 컴포넌트 (style.css 다음에 로드) */
@font-face { font-family: 'Fira Sans'; src: url('vendor/fonts/FiraSans-Regular.woff2') format('woff2'); font-weight: 400; font-display: swap; }
@font-face { font-family: 'Fira Sans'; src: url('vendor/fonts/FiraSans-Medium.woff2') format('woff2'); font-weight: 500; font-display: swap; }
@font-face { font-family: 'Fira Sans'; src: url('vendor/fonts/FiraSans-SemiBold.woff2') format('woff2'); font-weight: 600; font-display: swap; }
@font-face { font-family: 'Fira Code'; src: url('vendor/fonts/FiraCode-Regular.woff2') format('woff2'); font-weight: 400; font-display: swap; }
@font-face { font-family: 'Fira Code'; src: url('vendor/fonts/FiraCode-Medium.woff2') format('woff2'); font-weight: 500; font-display: swap; }

:root {
    /* 색 — 뜻이 있을 때만 색을 쓴다 */
    --color-bg: #0B0E11;
    --color-surface: #11151A;
    --color-raised: #171C22;
    --color-line: #252B33;
    --color-line2: #323A45;
    --color-ink: #E8ECF0;
    --color-mute: #8B95A1;
    --color-dim: #5D6773;
    --color-viewer: #07090C;
    --color-go: #3FD07A;
    --color-go-ink: #04210F;
    --color-stop: #F0616D;
    --color-warn: #E8B04B;
    --color-info: #6CB6FF;
    /* 글꼴 — Fira에 한글이 없어 Noto로 이어 받는다 */
    --font-sans: 'Fira Sans', 'Noto Sans KR', 'Noto Sans CJK KR', system-ui, sans-serif;
    --font-mono: 'Fira Code', 'Noto Sans Mono CJK KR', ui-monospace, monospace;
    /* 모양 — 반경은 섞어 쓴다 (균일 14px 금지) */
    --radius-xs: 3px;
    --radius-sm: 4px;
    --radius-md: 6px;
    --control-h: 40px;
    --control-h-lg: 44px;
    --sidebar-w: 244px;
    --sidebar-w-rail: 64px;
    --focus-ring: 2px solid var(--color-info);
    --dur-fast: 150ms;

    /* 기존 변수 별칭 (style.css의 두 :root 블록을 덮어써 옛 화면도 새 색이 됨) */
    --bg: var(--color-bg);
    --panel: var(--color-surface);
    --panel-strong: var(--color-raised);
    --border: var(--color-line);
    --border-light: var(--color-line2);
    --text: var(--color-ink);
    --text-primary: var(--color-ink);
    --text-secondary: var(--color-mute);
    --text-muted: var(--color-dim);
    --muted: var(--color-mute);
    --accent: var(--color-info);
    --accent-2: var(--color-go);
    --accent-dim: rgba(108, 182, 255, 0.12);
    --teal: var(--color-go);
    --teal-dim: rgba(63, 208, 122, 0.12);
    --success: var(--color-go);
    --danger: var(--color-stop);
    --warning: var(--color-warn);
    --primary: var(--color-info);
    --bg-base: var(--color-bg);
    --bg-card: var(--color-surface);
    --bg-section: var(--color-surface);
    --bg-input: var(--color-bg);
    --bg-hover: var(--color-raised);
    --radius: var(--radius-md);
    --mono-font: var(--font-mono);
    color-scheme: dark;
}

body {
    background: var(--color-bg);
    color: var(--color-ink);
}

::selection { background: rgba(108, 182, 255, 0.35); }

.ui-mono, .ui-num { font-family: var(--font-mono); font-variant-numeric: tabular-nums; }
.ui-sr-only { position: absolute; width: 1px; height: 1px; overflow: hidden; clip: rect(0 0 0 0); white-space: nowrap; }

@media (prefers-reduced-motion: reduce) {
    *, *::before, *::after { animation-duration: 0.01ms !important; animation-iteration-count: 1 !important; transition-duration: 0.01ms !important; }
}
```

`body`의 글꼴은 C:3655의 `!important` 규칙(Task 0.4에서 `var(--font-sans)`로 교체)이 정한다.

### A.2 컴포넌트 클래스 (Task 0.5)

| 클래스 | 용도 | 핵심 값 (목업 `kit.tsx` 기준) |
|---|---|---|
| `.ui-btn` | 기본(보조) 버튼 | 높이 40px, 패딩 0 14px, 테두리 `--color-line2`, 반경 4px, 14px/500, hover `--color-raised`, disabled 투명도 0.35 + `not-allowed` |
| `.ui-btn-go` | Start·실행 | 배경 `--color-go`, 글자 `--color-go-ink`, 600, hover `brightness(1.1)` |
| `.ui-btn-stop` | Stop·Cancel | 테두리 `rgba(240,97,109,.55)`, 글자 `--color-stop`, hover 배경 10% |
| `.ui-btn-ghost` | 약한 동작(모두 선택 등) | 테두리 없음, 글자 `--color-mute` |
| `.ui-btn-record` | 녹화 | 배경 `--color-stop`, 글자 흰색, 높이 44px |
| `.ui-btn-label` | 버튼 글자 칸 | `setBtnLabel` 대상 |
| `.ui-icon-btn` | 뷰어 툴바·폴더 선택 | 40×40, 배경 `rgba(11,14,17,.85)` + `backdrop-filter: blur(4px)`, `.active`/`[aria-pressed="true"]`는 테두리 `rgba(108,182,255,.6)` + 글자 `--color-info` |
| `.ui-icon` | SVG 아이콘 | 16×16, `stroke: currentColor`, `fill: none` |
| `.ui-divider` | 헤더 버튼 사이 세로선 | 1×24px, `--color-line2` |
| `.ui-panel` / `.ui-panel-header` / `.ui-panel-title` / `.ui-panel-body` / `.ui-panel-footer` | 카드 | 테두리 `--color-line`, 배경 `--color-surface`, 반경 6px, 머리 최소 40px + 아래 테두리, 제목 11px 대문자 자간 0.09em `--color-mute` |
| `.ui-kpi-grid` / `.ui-kpi` / `.ui-kpi-label` / `.ui-kpi-value` / `.ui-kpi-unit` / `.ui-kpi-hint` / `.ui-kpi-sub` | 숫자 카드 | 2열, 카드 반경 4px, 배경 `rgba(11,14,17,.6)`, 값 Fira Code 20px 500, `[data-tone]`이면 값 색 |
| `.ui-sparkline` | 작은 선 그래프 | 높이 28px, `stroke: currentColor` 1.5px |
| `.ui-status-badge` + `[data-tone]` | 상태 배지 | 높이 28px, 반경 3px, 테두리 `--color-line2`, 배경 `--color-bg`, Fira Code 12px, 점 8px. 톤 색: go/info/warn/stop/idle(`--color-mute`, 점 `--color-dim`) |
| `.ui-dot` | 상태 점 | 8px 원, 배지 톤 색. `[data-tone="go"]` 또는 녹화 중이면 `pulse` 애니메이션 |
| `.ui-tag` + `[data-tone]` | 작은 태그(ROS1/ROS2, 길이) | 반경 3px, 테두리 `--color-line2`, Fira Code 11px |
| `.ui-run-tag` | 사이드바 RUN/REC… | Fira Code 10px, 앞에 6px 점, 톤 색 |
| `.ui-stepper` / `.ui-stepper-item[data-state]` / `.ui-stepper-mark` | 단계 표시줄 | 항목 높이 28px 반경 3px. current: 배경 `rgba(108,182,255,.1)` + 안쪽 링 `rgba(108,182,255,.4)` + 글자 `--color-info`. done: 원 `--color-go` 채움 + ✓. todo: `--color-dim`. 항목 사이 20px 선(done이면 `rgba(63,208,122,.6)`) |
| `.ui-seg` / `.ui-seg-option[aria-checked]` | 분할 버튼 | 바깥 반경 4px 테두리 `--color-line2` 패딩 2px, 옵션 높이 32px 12px 글자, 선택 배경 `--color-raised` + 안쪽 테두리 `--color-line2` |
| `.ui-toggle` | 스위치 | 기존 `.toggle-switch` 재스타일: 켜짐 `--color-info`, 꺼짐 `--color-line2`, 라벨 글자 항상 표시 |
| `.ui-field` / `.ui-field-label` / `.ui-hint` | 폼 줄 | 라벨 12px `--color-mute`, 힌트 11px `--color-dim` |
| `.ui-input` / `.ui-select` | 입력 | 높이 40px, 반경 4px, 테두리 `--color-line2`, 배경 `--color-bg`, Fira Code 13px, focus 테두리 `--color-info` + 1px 링, readonly·disabled 글자 `--color-mute` |
| `.ui-table` | 토픽 표 | 머리 11px 대문자 `--color-dim`, 행 높이 40px, 행 구분 `--color-line`, `.ui-num` 오른쪽 정렬, `tr[data-disabled]` 투명도 0.45 |
| `.ui-progress` / `.ui-progress-fill` / `.ui-progress-indeterminate` | 진행 바 | 높이 4px, 트랙 `--color-line2`, 채움 `--color-info`, 무한형은 40% 막대가 1.4s 왕복(모션 줄이기 시 정지) |
| `.ui-progress-list` | Multi 진행 5단계 | `li[data-state]` done ✓ 초록 / current 원 테두리 info / todo dim |
| `.ui-toast-stack` / `.ui-toast[data-tone]` / `.ui-toast-close` | 토스트 | 오른쪽 아래 16px, 폭 320px, 반경 6px, 배경 `--color-raised`, 왼쪽 4px 톤 막대, 그림자 `0 8px 24px rgba(0,0,0,.45)` |
| `.ui-dialog-backdrop` / `.ui-dialog` | 확인 대화상자 | 배경 `rgba(0,0,0,.6)`, 상자 최대 480px, 반경 6px, `--color-raised` |
| `.ui-viewer-empty` | Start 전 빈 지도 칸 | `--color-viewer` 바탕 + 48px 격자(`rgba(139,149,161,.08)` 1px), 점선 테두리 `--color-line2`, 왼쪽 아래 정렬 글자 |
| `.ui-viewer-overlay` | 저장·최적화 진행 카드 | 뷰어 칸 안 `position: absolute; left: 16px; bottom: 16px`, 폭 320px, `--color-raised`, 반경 6px |
| `.ui-playbar` | 하단 재생 바 | `position: sticky; bottom: 0`, 높이 64px, 배경 `rgba(11,14,17,.95)`, 위 테두리 `--color-line`, 재생 버튼 44px 원형 `--color-go` |
| `.ui-more` | "세밀 조절"·"직접 입력" 접기 | `<details>` 기본 모양 + summary 12px `--color-mute` |
| `.ui-eye-toggle` | 3D Displays 표시 체크박스 | 20×20 mask 아이콘(eye / eye-off) |
| `.ui-plot-tab*`, `.ui-context-menu`, `.ui-plot-empty` | Plot 탭 바·메뉴·빈 상태 | 탭 높이 32px 아래 2px 선택선 `--color-info`, 메뉴 `--color-raised` |

포커스: 모든 `ui-*` 상호작용 요소에 `:focus-visible { outline: var(--focus-ring); outline-offset: 2px; }`.

### A.3 셸·화면 레이아웃 (Task 1.5, 2.3)

```css
.ui-sidebar { display: none; }
.ui-has-sidebar .ui-app-shell { display: grid; grid-template-columns: var(--sidebar-w) minmax(0, 1fr); min-height: 100vh; }
.ui-has-sidebar .ui-sidebar {
    display: flex; flex-direction: column; position: sticky; top: 0; height: 100vh;
    border-right: 1px solid var(--color-line); background: var(--color-surface);
}
.ui-has-sidebar .topbar, .ui-has-sidebar .tab-navigation, .ui-has-sidebar .subtab-navigation { display: none; }
body.ui-has-sidebar { padding: 0; }
.ui-has-sidebar .page-shell { max-width: none; margin: 0; padding: 0; }
.ui-sidebar-brand { display: flex; align-items: center; gap: 10px; height: 56px; padding: 0 16px; border-bottom: 1px solid var(--color-line); }
.ui-brand-mark { display: grid; place-items: center; width: 28px; height: 28px; border: 1px solid var(--color-line2); border-radius: var(--radius-sm); font: 500 11px var(--font-mono); color: var(--color-info); }
.ui-sidebar-nav { flex: 1; overflow-y: auto; padding: 12px 8px; }
.ui-sidebar-group-title { padding: 0 8px 6px; font-size: 10px; font-weight: 500; letter-spacing: 0.12em; text-transform: uppercase; color: var(--color-dim); }
.ui-sidebar-item { position: relative; display: flex; align-items: center; gap: 10px; width: 100%; height: 40px; padding: 0 10px; border: 0; border-radius: var(--radius-sm); background: transparent; color: var(--color-mute); text-align: left; cursor: pointer; }
.ui-sidebar-item:hover { background: rgba(23, 28, 34, 0.6); color: var(--color-ink); }
.ui-sidebar-item[aria-current="page"] { background: var(--color-raised); color: var(--color-ink); }
.ui-sidebar-item[aria-current="page"]::before { content: ''; position: absolute; left: 0; top: 8px; bottom: 8px; width: 2px; border-radius: 2px; background: var(--color-info); }
.ui-conn-block { padding: 12px; border-top: 1px solid var(--color-line); }
.ui-conn-list { display: grid; grid-template-columns: 1fr auto; gap: 4px 12px; font: 11px var(--font-mono); color: var(--color-dim); }
.ui-screen-header { position: sticky; top: 0; z-index: 20; display: flex; flex-wrap: wrap; align-items: center; justify-content: space-between; gap: 12px; padding: 16px 20px 12px; border-bottom: 1px solid var(--color-line); background: rgba(11, 14, 17, 0.95); backdrop-filter: blur(6px); }
.ui-screen-header .ui-stepper { flex-basis: 100%; padding-top: 8px; border-top: 1px solid rgba(37, 43, 51, 0.7); }
.ui-screen-group { font-size: 11px; letter-spacing: 0.09em; text-transform: uppercase; color: var(--color-dim); }
.ui-screen-title { margin: 2px 0 0; font-size: 20px; font-weight: 600; }
.ui-screen-grid { display: grid; grid-template-columns: minmax(0, 8fr) minmax(320px, 4fr); gap: 16px; padding: 16px 20px; }
.ui-col-main { position: relative; min-width: 0; }
.ui-col-side { display: flex; flex-direction: column; gap: 12px; min-width: 0; }
```

### A.4 반응형 브레이크포인트 (Task 5.4)

| 폭 | 사이드바 | 화면 격자 | 기타 |
|---|---|---|---|
| ≥ 1440px | 244px, 글자 + RUN 표시 | 8/4 (오른쪽 최소 360px) | 기준 디자인 |
| 1280~1439px | 244px | 8/4 (오른쪽 최소 320px), KPI 2열 유지 | 기존 `@media (max-width:1280px)`(C:3225)는 그대로 |
| 1024~1279px | 64px 아이콘 레일. 라벨은 `aria-label`·툴팁, RUN 표시는 아이콘 아래 9px 글자 | 1열(지도 → 정보) | 헤더 버튼 글자 유지 |
| < 1024px | 숨김, `#ui-sidebar-toggle`로 겹쳐 열기(배경 어둡게) | 1열 | 재생 바 전체 폭, 표 가로 스크롤 |

지원 목표는 데스크톱(≥1024px)이다. 1024px 미만은 "깨지지 않음"까지만 확인한다.

### A.5 접근성 체크리스트

- [ ] 모든 아이콘 버튼에 `aria-label`(+ 같은 글자 `title`). 토글형은 `aria-pressed`.
- [ ] 상태는 색만으로 알리지 않는다: 배지·RUN 표시·지연·토스트·범례 모두 글자 병기.
- [ ] `:focus-visible` 2px `--color-info` 외곽선, 오프셋 2px. 마우스 클릭에는 안 보임.
- [ ] 사이드바 현재 항목 `aria-current="page"`, 단계 표시줄 현재 단계 `aria-current="step"`, 단계 목록 `aria-label="진행 단계"`.
- [ ] 분할 버튼 `role="radiogroup"` + 옵션 `role="radio"`·`aria-checked`.
- [ ] 토스트 묶음은 `role="status"`/`aria-live="polite"`, 오류 토스트 `role="alert"`, 닫기 버튼 `aria-label="알림 닫기"`.
- [ ] 모달 `role="dialog"`·`aria-modal="true"`·`aria-labelledby`, ESC 닫기, 열면 첫 입력에 포커스, 닫으면 원래 버튼으로 포커스.
- [ ] 표 머리 `<th scope="col">`, 체크박스 `aria-label="<토픽> 선택"`.
- [ ] `prefers-reduced-motion: reduce`에서 점 깜빡임·무한 진행 바·토스트 슬라이드 정지.
- [ ] 대비: 본문 `--color-ink`/`--color-bg` 약 15:1, 보조 `--color-mute`/`--color-bg` 약 6:1(4.5:1 이상), `--color-dim`은 장식·힌트에만(약 3.3:1, 미측정 값 — 실행 시 대비 도구로 확인).
- [ ] 버튼 높이 40~44px(클릭 영역).

### A.6 Three.js 뷰어 색 (의미 색 유지 결정)

| 대상 | 지금 색 | 결정 | 이유 |
|---|---|---|---|
| 씬 배경 4개 뷰어 + 3D Viewer | `0x0a0a18`(Live 2개), `0x1a1a2e`(Result), `0x2a2a2a`(3D) | **`#07090C`로 통일** (Task 2.9) | 목업 `viewer` 토큰. 흰 점·선 대비가 오름 |
| SLAM Live LIO 선 | `#00ff44` | 유지 | 사용자가 익힌 의미 색 |
| SLAM Live Pose Graph 선 | `#ffffff` | 유지 | 〃 |
| 현재 스캔 | 흰색 | 유지 | 〃 |
| `/PGO_map`·`/Laser_map` | z-rainbow | 유지 | 높이 정보 |
| Save Map 결과 | LIO Map `#ffdd00`, Optimized `#ff3333`, Dynamic Object Removal `#44ff88`, LIO `#ff8800`, PGO `#fff`, Loop Closure 파랑 | 유지 (목업의 LIO 초록·PGO 파랑·루프 노랑은 예시 색이라 채택 안 함) | Phase 4.14 결정 유지, 범례와 일치 |
| Multi-Session 12레이어 | Phase 4.15 색상환 30° 팔레트 | 유지 | 겹침 없는 팔레트를 이미 재설계함 |
| Height Clip 핸들 | 최대 파랑·최소 주황 | 토큰으로만 보정(`--color-info`·`--color-warn`) | 의미 같음 |
| 범례 스와치 | HTML 인라인 색 | 값 유지, P6에서 `--swatch` 변수로 이동 | — |

