# ROS SLAM WEBUI "Field Console" 리디자인 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**디자인 기준: docs/design/redesign-mock/ (항상 참고)**

**Goal:** 승인된 목업(사이드바 셸 + 화면별 헤더·단계 표시줄 + 지도 8/12·정보 4/12 배치 + 토스트)을 지금의 바닐라 JS 웹 UI에 기능 손실 없이 적용한다.

**Architecture:** 빌드 없는 바닐라 JS를 유지한다. CSS 토큰 파일 하나와 작은 UI 헬퍼 JS 세 개를 추가한다. 기존 DOM id·`onclick` 핸들러 이름·상태 플래그 로직은 그대로 두고, DOM을 옮기고 다시 꾸미는 방식으로 바꾼다.

**Tech Stack:** 바닐라 JS(ES2020) · Three.js r128 · Plotly 2.27 · roslib 1.1 · Python rclpy(ROS 2 Jazzy) · Playwright(Python 1.64) · Node(`node --check`, 순수 함수 테스트).

**Spec:** 별도 스펙 문서 없음. 승인된 목업이 스펙이다.
- 목업 소스(시각 참고만, 이식하지 않음): `docs/design/redesign-mock/source/src/` (`components/kit.tsx`, `components/Shell.tsx`, `screens/*.tsx`, `tailwind.config.js`)
- 번들·스크린샷: `docs/design/redesign-mock/bundle/index.html`, `docs/design/redesign-mock/shots/` (`01~47-*.png`, `notes/`)
- Task 0.1에서 이 계획의 부록 A를 `docs/superpowers/specs/2026-10-16-webui-redesign-design.md`로 옮겨 스펙으로 남긴다(룰 `01` §7.6).

**기존 Phase 소속:** 기존 Phase에 속하지 않는다. Phase 4.12(다크 테마 디자인 시스템)를 대체하는 새 **Phase 8.0~8.6**으로 `03`에 등록한다(부록 B.3 제안 문구). 이 계획의 단계 이름 **P0~P6**은 Phase 8.0~8.6과 1:1이다.

**파일 약칭:** `H`=`web/index.html`, `J`=`web/static/script.js`, `C`=`web/static/style.css`, `P`=`ros_slam_webui/web_server.py`, `T`=`web/static/threejs_display.js`. 줄 번호는 2026-10-16 `dev`(67c9fbd) 기준 ±5줄이다. 실행 전에 `rg`로 다시 확인한다.

---

## 목차

1. 아키텍처 결정
2. 예시(목업)에 없는 실제 동작 목록과 처리 방법
3. 상태 기계 정리와 UI 매핑
4. 사이드바 실행 표시(RUN/REC/PLAY/OPT)와 전역 상태 API
5. 단계별 로드맵 요약 (P0~P6)
6. P0 태스크 — 토큰·글꼴·컴포넌트 CSS
7. P1 태스크 — 셸 (사이드바·헤더·전역 상태)
8. P2 태스크 — LiDAR SLAM / Localization
9. P3 태스크 — Multi-Session / Bag Player / Bag Recorder / File Player
10. P4 태스크 — Plot / 3D Viewer
11. P5 태스크 — 토스트·접근성·반응형
12. P6 태스크 — 정리
13. ROS 패키지 대조 결과 (FAST_LIO · pose_graph_optimization · long_term_mapping, 2026-10-16)
- 부록 A. 디자인/UI 사양 (CSS 변수, 컴포넌트 클래스, 뷰어 색, 반응형, 접근성)
- 부록 B. 브랜치·검증·문서 (워크트리, 커밋 계획, 룰 갱신 제안 문구)
- 부록 C. 리스크 Top 5와 완화책
- 부록 D. 미확인 항목, 결정 사항(확정), 셀프 리뷰
- 부록 E. 백로그 (이 계획 밖 별도 작업)

---

## Global Constraints

- 빌드 단계 추가 금지. 새 JS·CSS는 `web/static/` 바로 아래(`setup.py`의 `glob('web/static/*.*')`가 자동 설치). 글꼴은 `web/static/vendor/fonts/`(재귀 설치됨).
- **id 변경 금지.** DOM 이동은 허용. 삭제는 P6에서 JS·CSS 참조가 0인 것만. 근거: `getElementById`/`domCache.get` 약 208곳, `SlamResultViewer`의 `ids` 맵(J:10827-10945), 풀스크린 핸들러 id 매칭(J:11048-11090), `data-layer` 이름.
- **`onclick` 핸들러 이름 변경 금지.** `button[onclick*="…"]` CSS 규칙 28개(C:3732-3814, 4154-4184)와 `openSubTab`의 문자열 매칭(J:183)이 이름에 걸려 있다.
- 상태 플래그(`_slamSaving`, `_slamMapJustSaved`, `_slamLiveViewerHoldOpen`, `_slamStopping`, `_optRunning`, `_optComplete`, `_optViewerDismissed`)는 **읽기만** 한다. 새 플래그는 Localization의 `_locStarting`/`_locStopping`/`_locStoppedAt` 3개와 Multi의 `_multiOutputDirty` 1개뿐이다.
- 구독 시점 불변. Live Viewer `show()`는 Start 뒤에만 부른다. 빈 지도 칸은 순수 DOM이다.
- JS: 4 spaces, 세미콜론, `const`/`let`(`var` 금지), `camelCase`, 파일 `snake_case.js`. CSS 클래스·id `kebab-case`. **새 클래스는 모두 `ui-` 접두사**(기존 `btn`/`panel`/`field` 부분 문자열과 충돌 방지).
- API: `/api/{module}/{action}`, GET=조회. 새 응답은 `{"success": true, "data": {...}}`. 내부 예외 문자열은 응답에 넣지 않고 로그에만.
- 새 엔드포인트는 같은 태스크 안에서 `02` §6에 `✅`로 기록한다.
- 커밋: Conventional Commits, 설명은 한국어. 이 계획 작성 단계에서는 커밋하지 않는다.
- `.cursor/`는 gitignore다. 룰은 메인 체크아웃 절대 경로에서 Edit로만 고친다(통째 Write 금지). 워크트리에는 `.cursor/`가 없다.
- green 기준(룰 `01` §7.6): 워크트리 colcon 빌드 성공 + 바꾼 파일 `node --check` / `python3 -m py_compile` 통과 + Playwright 검증(콘솔 새 에러 0, 스크린샷 확인). 실데이터가 필요한 항목은 "사용자 확인 필요"로 보고한다.
- 서버 기동·정리 절차는 룰 `01` §7.2와 스킬 §5(4)를 따른다. 테스트 전후로 `pkill -9 -f "ros_slam_webui/web_server"`와 rosbridge 정리를 한다.

---

## 1. 아키텍처 결정

### 1.1 선택안과 탈락안

| 안 | 판정 | 근거 |
|---|---|---|
| **A. 바닐라 유지 + CSS 토큰 파일 + 작은 UI 헬퍼 JS** | **선택** | 빌드 단계가 없고 colcon이 `web/` 파일을 그대로 설치한다. J 12,284줄이 id로 DOM을 직접 잡고, Three.js 뷰어 5개(SLAM Live, Loc Live, Save Map Result, Multi Result, 3D Viewer)와 Plotly 차트가 그 DOM 위에서 돈다. DOM을 옮기고 클래스만 바꾸면 이 결합이 깨지지 않는다. 단계마다 롤백이 쉽다. |
| B. React/Vue + Vite 재작성 | 탈락 | 빌드 단계와 Node 의존성이 생기고 `setup.py` 설치 경로도 바뀐다. 뷰어·대시보드·폴링·Save Map 플래그를 컴포넌트로 다시 써야 해서 사실상 전면 재작성이다. 회귀 위험이 가장 크다. 목업 React 코드는 가짜 데이터용이라 재사용 가치가 낮다. |
| C. Web Components / Lit (ESM vendor) | 탈락 | 빌드는 없어도 된다. 하지만 Shadow DOM 안의 요소는 `document.getElementById`로 안 잡히고 전역 CSS도 안 먹는다. Light DOM으로 쓰면 이점이 거의 없다. |
| D. Tailwind (CDN 런타임 또는 빌드) | 탈락 | CDN은 인터넷 없는 현장에서 깨진다. 빌드형은 B와 같은 문제다. 목업의 토큰 값만 CSS 변수로 옮기면 충분하다. |
| E. Alpine.js / petite-vue | 탈락 | 반응형 상태 층이 하나 더 생겨 기존 명령형 상태와 이중 관리가 된다. 얻는 것보다 헷갈림이 크다. |

### 1.2 핵심 설계 원칙

1. **옮기기만 하고 이름은 그대로.** 버튼·입력·뷰어는 같은 id·같은 `onclick`으로 새 위치(헤더, 오른쪽 칸, 하단 재생 바)로 옮긴다.
2. **상태는 읽어서 파생한다.** 헤더 배지·버튼 잠금·단계 표시줄은 순수 함수(`ui_status_logic.js`)가 서버 응답 필드 + 기존 플래그 + 기존 버튼 글자를 읽어 계산한다. 흐름 코드는 바꾸지 않는다.
3. **훅은 한 줄.** 기존 폴링 함수 끝과 클릭 핸들러 끝에 `if (window.uiShell) uiShell.onXxx(...)` 한 줄만 넣는다.
4. **옛 UI는 숨겨 두고 마지막에 지운다.** 옛 탭 버튼·상단 칩·`Status:` 라벨은 P1~P5 동안 CSS로 숨기고(JS가 아직 씀), P6에서 참조를 정리한 뒤 지운다.
5. **되돌리기 스위치.** P1~P5 동안 `localStorage['webui.legacyNav'] = '1'`이면 사이드바를 끄고 옛 탭을 보여 준다. P6에서 제거한다.

### 1.3 파일 구조

**새 파일 (`web/static/`)**

| 파일 | 책임 | 예상 크기 |
|---|---|---|
| `ui_theme.css` | 디자인 토큰, 기존 변수 별칭, `ui-*` 컴포넌트, 셸 레이아웃, 반응형. `style.css` 다음에 로드 | 600~800줄 |
| `ui_kit.js` | 도메인을 모르는 DOM 헬퍼: `showToast`, `confirmDialog`, `setStatusBadge`, `renderStepper`, `setEnabled`, `setBtnLabel`, `renderSparkline`, 모달 ESC·aria | 250~350줄 |
| `ui_status_logic.js` | DOM 없는 순수 함수: `deriveSlamView`, `deriveLocView`, `deriveMultiView`, `deriveBagView`, `deriveRecorderView`, `deriveFileView`, `deriveSidebarMarkers`, `optDoneCount`, `formatElapsed`. 브라우저에서는 `window.UiStatusLogic`, Node에서는 `module.exports` | 200~250줄 |
| `ui_shell.js` | `SCREENS` 표, `showScreen()`, 사이드바 활성 표시, 화면 헤더 렌더, 전역 상태 폴링(2초), 연결 블록, `onXxx` 훅 | 300~400줄 |
| `vendor/fonts/*.woff2` + `vendor/fonts/OFL.txt` | Fira Sans 400/500/600, Fira Code 400/500 — **확정(2026-10-16): 저장소에 포함, 오프라인 동작** | 바이너리 5개 + 라이선스 |

**새 테스트 파일**

| 파일 | 종류 |
|---|---|
| `test/ui/ui_check_common.py` | Playwright 공용 헬퍼(페이지 열기, 콘솔 에러 수집, 스크린샷) |
| `test/ui/check_ids.py` | `dev` 대비 사라진 id·중복 id 검사 |
| `test/ui/check_smoke.py` | 8화면 이동 + 스크린샷 + 새 JS 에러 0 |
| `test/ui/check_*.py` | 단계별 UI 동작 검사 (이름이 `test_`로 시작하지 않아 `colcon test`/pytest가 수집하지 않음) |
| `test/ui/check_status_logic.js` | `ui_status_logic.js` Node 단위 테스트 (`node test/ui/check_status_logic.js`) |
| `test/test_status_summary.py`, `test/test_bag_info_parse.py`, `test/test_recorder_stats.py` | 백엔드 pytest |

**수정 파일**: H(셸·화면 구조 이동), C(글꼴 규칙·속성 선택자 정리), J(훅 한 줄씩, `openTab`/`openSubTab` 안전화, Loc 플래그, 소수 버그 수정), `plot_tree.js`·`plot_manager.js`·`plot_tab_manager.js`(P4), T(P4), P(전역 상태·bag 토픽 상세·Recorder 통계).

**로드 순서 (H 끝부분)**: `webui_ports.js` → `ui_kit.js` → `ui_status_logic.js` → `script.js` → 기존 module 블록 → `threejs_display.js` → `ui_shell.js`. `ui_shell.js`는 `openTab`, `lastActiveSubtab` 같은 J 전역을 쓰므로 마지막이다. J의 최상위 `let _optRunning` 등은 classic script 전역 렉시컬 바인딩이라 `ui_shell.js`에서 이름으로 읽을 수 있다.

---

## 2. 예시(목업)에 없는 실제 동작 목록과 처리 방법

위험도: **낮** = CSS·마크업 이동만 / **중** = 작은 JS 수정 / **상** = 흐름·백엔드·여러 파일 연동.

### 2.1 전역 셸·공통

| # | 동작 | 현재 위치 | 새 UI 사양 | 구현 방식 | 위험 | 단계 |
|---|---|---|---|---|---|---|
| G1 | rosbridge 연결 칩 3상태(connected/error/reconnecting), 3초 고정 재연결 | J:4621-4640, J:4885-4901, C:73-105 | 사이드바 하단 연결 블록: 점 + 글자 "rosbridge 연결됨 / 오류 / 재연결 중…" | `updateRosbridgeStatusChip` 끝에 `uiShell.onRosbridgeState(state)` 한 줄. 옛 칩은 숨김 | 낮 | P1 |
| G2 | 지연(latency) = HTTP `/api/ping` 3회 최소 RTT, 3초 주기, 인라인 색 | J:4094-4132, `latency_ping_worker.js` | 연결 블록 오른쪽 `12 ms`(Fira Code). 색 대신 `data-tone` + 숫자 | worker `onmessage`에 `uiShell.onLatency(ms)` 한 줄 | 낮 | P1 |
| G3 | ROS_DOMAIN_ID 1회 조회, 포트 칩(Web/PC2 WS/rosbridge) | J:4138-4150, J:11093-11105, `webui_ports.js` | 연결 블록 `DOMAIN / Web / PC2 WS / rosbridge` 4줄 | `updateRosDomainId`에 훅 한 줄. 포트는 `window.ROS_SLAM_WEBUI`를 `ros-slam-webui-ports-ready` 뒤 읽음 | 낮 | P1 |
| G4 | 8081 백엔드 WS 연결 상태가 UI에 전혀 없음(변환 진행률·Plot 공용 채널) | J:4253-4291 | 연결 블록 `PC2 WS` 줄에 "연결됨/끊김" 글자 | `_initBackendWs` `onopen`/`onclose`에 `uiShell.onBackendWsState(...)` | 낮 | P1 |
| G5 | `openTab`/`openSubTab`이 전역 `event`에 의존(프로그램 호출 시 TypeError), 전역 smooth 스크롤 | J:131-213 | 사이드바 `showScreen(subtabId)`가 두 함수를 호출 | `event.target` 대신 `onclick` 문자열로 버튼 찾기(이미 `openSubTab`에 있는 방식). 끝에 `uiShell.onScreenChanged(id)` | 중 | P1 |
| G6 | 상태 폴링은 보이는 서브탭만(500ms). Recorder는 폴링 없음(`updateRecorderState` J:2645 호출처 0) | J:4175-4193 | 사이드바 RUN/REC/PLAY/OPT 표시 | 새 `GET /api/system/status_summary` 2초 폴링(4장). 기존 500ms 폴링은 그대로 | 중 | P1 |
| G7 | 9개 HTML 모달 + JS 오버레이(`showYamlErrorModal` J:3572). 바깥 클릭 닫기는 3개만, ESC 없음, `block`/`flex` 혼용 | H:1343-1635, J:4203-4216 | 공통 다이얼로그 모양(raised 배경, 6px 반경, 제목·본문·버튼 줄), ESC 닫기, `role="dialog"` | CSS로 `.modal` 재정의 + `ui_kit.enhanceModals()`가 ESC·aria 부여(열기/닫기 함수는 그대로) | 중 | P5 |
| G8 | `alert(` 98곳(J 91, T 6 — T 6개는 죽은 코드, `plot_tab_manager.js` 1), `confirm(` 1곳(J:977) | 각 파일 | 오른쪽 아래 토스트. 오류 토스트는 자동으로 안 닫힘. `confirm`은 확인 대화상자 | 1단계 `window.alert` shim → 2단계 성공 알림부터 명시적 `showToast`. `playBag`(async)의 `confirm`은 `await uiKit.confirmDialog()` | 중 | P5 |
| G9 | 파일/폴더 브라우저 모달(모드 구분 없음: 폴더 클릭=진입, 파일 클릭=즉시 선택, 푸터=현재 폴더 선택) | H:1343-1362, J:350-412 | 같은 동작, 다이얼로그 모양만 새로. 경로 줄 Fira Code, 폴더/파일 아이콘 | CSS + `.dir-entry` 재스타일 | 낮 | P5 |
| G10 | 렌더 루프(rAF)가 숨은 화면에서도 계속 돎, 각 뷰어가 자체 `ROSLIB.Ros`를 만들 수 있음 | J:6679, 7957, 9387, T:998 | 변경 없음(범위 밖) | 기록만. 사이드바 전환 비용은 지금과 같음 | — | — |
| G11 | 노드 건강 신호가 없음: 크래시 heartbeat·IMU 타임아웃 로그 없음, `is_running`은 launch 프로세스만 봄, stderr 로그(`/tmp/web_gui_slam_stderr.log`, `…_localization_…`)는 쓰기만 하고 아무도 안 읽음, launch stdout은 `DEVNULL` | P:3501-3505, 3664-3668, 3932; 13.1절 | 헤더 배지에 "데이터 없음 / IMU 끊김 / LiDAR 끊김"(3.1·3.2절), 헤더 오른쪽 "로그" 아이콘 버튼 → 마지막 50줄 패널 | analytics 수신 시각·`imu_freq`/`lid_freq`로 판정(프론트만) + 새 `GET /api/system/process_log`(Task 2.10) | 중 | P2 |
| G12 | Start SLAM이 `pkill -9 -f rviz2`로 사용자가 띄운 rviz까지 죽임 | P:8209, 3478 | Start 버튼 툴팁 "실행 중인 rviz2도 함께 종료됩니다" | 문구만(동작 수정은 범위 밖, 부록 D) | 낮 | P2 |

### 2.2 LiDAR SLAM / Save Map 결과

| # | 동작 | 현재 위치 | 새 UI 사양 | 구현 방식 | 위험 | 단계 |
|---|---|---|---|---|---|---|
| S1 | Config Load(yaml만, 아니면 `showYamlErrorModal`), "Set Config File"은 **기본 `mapping_config.yaml`에 저장**(Load한 파일이 아님) | H:57-58, J:3107-3111, J:2791 | Config 패널 아래 버튼 두 개. "Set Config File" 옆 작은 글자 "→ mapping_config.yaml에 적용" | 버튼 이동 + 안내 글자. 동작 불변 | 낮 | P2 |
| S2 | 페이지 로드 시 기본 config 자동 로드, 패널 접힘 상태로 표시. `update_config` 즉시 반영 호출은 **서버 핸들러 없음**(no-op) | J:4161-4167, J:2992-3010 | Config 패널 오른쪽 칸, 파일명 배지 | 이동만. no-op 호출은 손대지 않음(부록 D 기록) | 낮 | P2 |
| S3 | Start 시 Config 패널을 **숨기고** Stop 시 복원(`_slamConfigVisibleBeforeStart`) | J:3372-3377, J:3423-3428 | 실행 중에도 보이되 읽기 전용(자물쇠 아이콘 + "바뀐 값은 다음 Start부터 적용") | 숨김 2줄을 `uiKit.setReadonly(container, true/false)`로 교체. 플래그 변수는 남겨 둠 | 중 | P2 |
| S4 | Save Map 이름 모달(빈 값 alert, Enter 없음, 바깥 클릭 안 닫힘) | H:1474-1491, J:3138-3203 | 같은 모달, 새 모양. Enter = 확인 | CSS + keydown 한 줄(P5) | 낮 | P2/P5 |
| S5 | 저장 진행 배너(`#slam-save-map-status-area`, 스피너·메시지·Cancel, 성공 3초 뒤 사라짐, 오류 유지). 진행률 % 없음. **실제**: PGO의 단계별 진행(`Void mapping i/N` 등)은 printf라 `stdout=DEVNULL`로 버려지고, 서비스 응답은 끝에 한 번뿐. 서버는 10초마다 "Saving map... (N s elapsed)"만 씀(P:3589) | H:326-333, J:3206-3358, `saveMapData.cpp:560-779` | 지도 칸 아래 왼쪽 오버레이 카드 "지도 저장 중" + 경로 + **무한 진행 바 + 경과 시간** + "저장 중 새 키프레임 추가 안 됨" 안내 + "기다리기 중단" 버튼 | 배너 요소를 뷰어 칸 안으로 옮기고 `ui-viewer-overlay`로 재스타일. 경과 시간은 프론트 타이머. id 유지. 단계별 진행률은 PGO 수정 없이는 불가(부록 D) | 중 | P2 |
| S6 | 결과 뷰어 범례 클릭 토글(LIO Map·Optimized·Dynamic Object Removal / LIO·PGO·Loop Closure) | H:356-367, J:10961 | 뷰어 안 왼쪽 아래 칩(`aria-pressed`). 목업처럼 위로 빼지 않음 — 풀스크린에서도 보여야 해서 | `.slam-legend-row` 재스타일 + `aria-pressed` 동기화(`data-active` 읽음) | 낮 | P2 |
| S7 | Height Clip 세로 이중 핸들(최대/최소, 더블클릭 리셋) | H:345-355, J:9311, J:10620 | 그대로 유지(목업의 가로 슬라이더보다 기능이 많음). 토큰 색으로 재스타일 | CSS만 | 낮 | P2 |
| S8 | Point Size 슬라이더(0.01~0.5, 기본 0.05) | H:368-373 | 뷰어 왼쪽 위 작은 패널 | CSS만 | 낮 | P2 |
| S9 | 툴바 5개(Reset·Top View·EDL Depth Filter·Snapshot 2x PNG·Fullscreen), 숨은 체크박스 상태 미러 | H:374-391 | `ui-icon-btn` 40×40, `aria-label`·`aria-pressed`, 오른쪽 위 | 클래스·aria만. 숨은 체크박스 유지 | 낮 | P2 |
| S10 | `SlamResultViewer`에 ResizeObserver 없음(크기는 `show()`·풀스크린 때만) | J:9228 | 사이드바 접힘·창 크기 변경 시 캔버스 자동 맞춤 | `_init()`에 ResizeObserver 추가(라이브 뷰어 J:7950 패턴 복사) | 중 | P2 |
| S11 | Live 툴바(Reset·Follow·Top View·Snapshot·Fullscreen). **버그**: Follow 기본 ON인데 버튼은 꺼진 모양, SLAM Live 풀스크린 아이콘 안 바뀜, Reset이 Top View 버튼 `active`를 안 지움 | H:99-146, J:7658, J:9012, J:11048-11090 | 같은 버튼, 새 모양 | `show()`에서 Follow `active` 동기화, 풀스크린 핸들러에 slam-live 분기 추가, Reset에서 버튼 상태 동기화 | 중 | P2 |
| S12 | Analytics: Hz·궤적·CPU·RAM 카드, Processing Time 차트(10초), **DOP 차트**, **누적 타이밍 표**, **Full Statistics 27항목(8분류)**, `GET /api/system/info` 1회 | H:153-324, J:11158-11784 | 오른쪽 칸: KPI 2×2(+스파크라인) + 거리·평균 속도 + Processing Time + DOP + 누적 표(접기 가능) + Full Statistics(기본 접힘, "27 항목") | DOM 이동 + CSS. 스파크라인은 대시보드 update에 링버퍼 60개 + `uiKit.renderSparkline` | 중 | P2 |
| S13 | Start 실패·Stop 실패 alert, Start 전 뷰어 미리 `show()`(hold-open) | J:3364-3444 | 실패는 오류 토스트, 배지는 Ready로 복귀 | 훅만(alert은 P5 shim) | 낮 | P2 |
| S14 | Save 시작 실패 시 `_visualHide` 뒤 `_visualShow` 안 부름 → Live Viewer가 숨은 채 남음 | J:3201-3202 | 실패 시 지도 다시 보임 | 실패 분기에 `slamLiveViewer._visualShow()` 한 줄(버그 수정, 사용자 승인 필요 항목으로 표시) | 중 | P2 |
| S15 | **저장 이름이 검증 없이 `rm -r <pkg>/<이름>/`에 들어감.** `..`·쉘 문자 가능, 이름이 `Map`이면 라이브 스테이징 폴더가 지워짐, 같은 이름이면 경고 없이 덮어씀 | P:8706, `saveMapData.cpp:855-857` | Save Map 모달: 이름 규칙 안내(`영문·숫자·_·-`, `Map` 금지), 규칙 위반이면 확인 버튼 잠금 + 빨간 글자, 이미 있는 이름이면 "덮어쓰기" 확인 대화상자 | 프론트 검사 + 서버 검사(Task 2.10: `^[A-Za-z0-9_-]{1,64}$`, `Map` 예약, 존재 여부 `exists` 응답) | 상 | P2 |
| S16 | **Start가 이전 미저장 지도를 지움**: PGO가 시작할 때 `pose_graph_optimization/Map/`(Scans·Debug)을 `rm -r` | `poseGraphOptimization.cpp:819-856` | Ready 상태 Start 아래 힌트 "Start하면 저장하지 않은 이전 지도는 지워집니다", 이번 실행에서 저장 없이 Stop하면 확인 대화상자 | 문구 + `slamSavedSinceStart`(3.1절) | 중 | P2 |
| S17 | 키프레임 0개에서 저장하면 PGO에서 빈 벡터 접근(정의되지 않은 동작) | `saveMapData.cpp:433` | 키프레임이 1개 이상일 때만 Save Map 열림, 잠금 툴팁 "키프레임이 아직 없습니다" | `kfCount`(3.1절) | 중 | P2 |
| S18 | `/PGO_map`은 전체 지도가 아니라 **최신 키프레임 50 m 안의 로컬 지도**, `/kf_node`·`/loopLine`은 최적화 전 원래 자세라 루프 선이 PGO 선과 어긋나 보일 수 있음. 프레임은 모두 `odom`(`map` 프레임 없음) | `poseGraphOptimization.cpp:773-815, 552-556, 712-720` | Live 범례: "LIO (원래 자세)", "Pose Graph (최적화)", "지도: 최근 50 m". 빈 칸 설명에도 "주변 50 m만 보입니다" | 범례 글자만 | 낮 | P2 |
| S19 | 결과 폴더에 `StaticNGMap.pcd`(지면 제외)·`StaticGroundMap.pcd`(지면)·`session_meta.yaml`도 생기지만 UI에 없음. `StaticMap.pcd`("Dynamic Object Removal")는 동적 객체 제거 + 지면 분리 + 이상치 제거 결과이고 지면 포함 | `saveMapData.cpp:806-807, 836-866, 894-897`; P:3998-4009 | 레이어는 지금 3개 유지, `StaticMap` 칩 툴팁 "동적 객체·이상치 제거(지면 포함)". 두 파일 추가 표시는 선택 | 툴팁만. 레이어 추가는 범위 밖(부록 D) | 낮 | P2 |
| S20 | 결과 경로 허용 목록이 `slam_last_saved_dir` 하나뿐 → **서버 재시작 뒤** 이전 저장 결과 복원은 막힘. `startswith` 비교라 접두가 같은 다른 폴더도 통과 | P:9192-9199 | 복원 실패 시 결과 칸에 "서버가 다시 시작되어 이전 결과를 열 수 없습니다" | 메시지만(허용 목록 수정은 범위 밖, 부록 D에 보안 메모) | 낮 | P2 |
| S21 | Save Map 결과는 `pose_graph_optimization/<이름>/`에 저장되지만 Localization은 `FAST_LIO_Localization_and_Mapping/` 기준 상대 경로(`map_file_path`)로 지도를 읽음 → 저장한 지도를 바로 Localization에 쓸 수 없음 | `saveMapData.cpp:855`, `laserLocalization.cpp:1544` | 결과 화면 안내 "Localization에 쓰려면 StaticMap.pcd를 FAST_LIO Map/ 폴더로 복사하고 map_file_path를 바꾸세요" | 안내만. "이 지도로 Localization" 버튼은 범위 밖(부록 D 제안) | 낮 | P2 |

### 2.3 Localization

| # | 동작 | 현재 위치 | 새 UI 사양 | 구현 방식 | 위험 | 단계 |
|---|---|---|---|---|---|---|
| L1 | Initialized 배지 2곳(`#loc-header-init-badge`, `#loc-init-status-dot`)이 `/loc_analytics.is_initialized`로만 갱신. `_resetWidgets`가 배지를 안 지워 재시작 직후 옛 "Initialized"가 보일 수 있음 | J:12189-12198, J:12244 | 화면 헤더 배지 "Running · Not Init" / "Initialized · Tracking", 단계 Initialize→Tracking | 헤더는 `#loc-header-init-badge`의 `ok` 클래스를 읽음. `_resetWidgets`에 배지 초기화 2줄 | 중 | P2 |
| L2 | 초기 위치 지정 UI 없음(Initialize는 관찰만) | 화면 전체 | 단계 표시줄 "Initialize"는 상태 표시일 뿐 버튼 없음 | 매핑만 | 낮 | P2 |
| L3 | 사용 지도 경로가 config 안(`map_file_path`, `/**/ros__parameters` 바로 아래)에만 보임 | FAST_LIO `localization_config.yaml`:9 | 헤더 오른쪽 "사용 지도 Map/gsic_map.pcd" | `#localization-config-parameters [data-config-key$="map_file_path"]`의 값을 읽음(J:2918, 2946에서 `data-config-key` 확인) | 낮 | P2 |
| L4 | Stop 직후 재출현 위험: 진행 중이던 상태 요청이 `is_running=true`로 늦게 도착, `/loc_analytics` 퍼블리셔 수가 DDS 반영 지연 | J:3475-3513, P:4011-4024 | 깜빡임 없음 | `_locStopping` + `_locStoppedAt` 3초 보류(3.2절) | 중 | P2 |
| L5 | 터미널에서 띄운 Localization도 `is_running=true`(퍼블리셔 검사). Stop은 `localization_process=None`이라 동작 미확인 | P:4011-4024 | 배지 "Running (외부 실행)", Stop 잠금 + 툴팁 | `/api/localization/state`에 `external` 필드 추가는 범위 밖. 전역 요약의 `localization.running`(자기 프로세스)과 화면 폴링 `is_running` 차이로 판정 | 중 | P2 |
| L6 | 대시보드: Hz·궤적·CPU·RAM, Processing Time(점 상한 없음), DOP + **Map Update 러그(200칸)**·업데이트 수·dop ratio·마지막 업데이트, 누적 표, **Full Statistics 31항목(10분류)** | H:493-696, J:11804-12284 | 오른쪽 칸: KPI(Matching score=`match_ratio`, LiDAR Hz, Wheel Odometry=`linear_velo`, CPU) + Map Update 러그 + 나머지 접기 | DOM 이동 + KPI 1개 신설(`match_ratio`는 `LocAnalytics.msg`:27에 있음) + 러그 ResizeObserver | 중 | P2 |
| L7 | 고정 프레임 함수(`onLocFixedFrame*`)에 DOM 없음 | J:7790-7840, J:10999 | 변경 없음 | 기록만(죽은 코드) | — | — |
| L8 | 지도 경로 `map_file_path`는 **FAST_LIO 소스 폴더 기준 상대 경로**(`ROOT_DIR + map_file_path`). 파일이 없으면 "Map file cannot open." 후 노드 종료 | `laserLocalization.cpp:1507, 1544-1549` | 헤더 "사용 지도 FAST_LIO/Map/gsic_map.pcd"(접두 표시), 시작 실패 시 배지 "시작 실패 (로그 보기)" | 접두 문자열 + 3.2절 `lastStartFailed` + 로그 API(Task 2.10) | 중 | P2 |
| L9 | 휠 오도메트리 `linear_velo`는 `odom_mode`가 1·2일 때만 채워짐. 지금 yaml은 `odom_mode: 0` → 항상 0 | `laserLocalization.cpp:831-913, 880-882`; `localization_config.yaml` | `odom_mode`가 0이면 Wheel Odometry KPI 대신 "휠 오도메트리 꺼짐 (odom_mode 0)" 글자 | Config 입력 `[data-config-key$="odom_mode"]` 값으로 분기 | 낮 | P2 |
| L10 | `map_updated` 판정은 `initial_flag && dop_ratio < 0.6`(코드). `LocAnalytics.msg` 주석의 "> 1.4"와 `dop_ratio` 식은 낡은 설명 | `laserLocalization.cpp:1785-1790` | 러그 범례 툴팁 "지도와 덜 맞는 프레임(dop_ratio < 0.6)에서 지도 갱신" | 툴팁 글자만 | 낮 | P2 |
| L11 | `/initialpose`로 재초기화 가능(움직이는 중이면 "Cannot init Position."으로 거절, 단 `odom_mode 0`이면 속도가 0이라 거절 안 됨). 웹 UI는 보낼 방법이 없음 | `laserLocalization.cpp:752-829, 1588` | Loc 뷰어 "초기 위치 지정" 버튼 + 지도 드래그 (확정 2026-10-16) | Task 2.12 (rosbridge `geometry_msgs/PoseWithCovarianceStamped`, frame `odom`) | 중 | P2 |
| L12 | Localization launch는 `octomap_server`도 같이 띄움(`/cloud_registered_body` 입력) | `localization.launch.py` | 변경 없음 | 기록만 | — | — |

### 2.4 Multi-Session SLAM

| # | 동작 | 현재 위치 | 새 UI 사양 | 구현 방식 | 위험 | 단계 |
|---|---|---|---|---|---|---|
| M1 | Load Map 1/2는 폴더 선택 즉시 `set_map1/2` POST, Output은 입력 후 Set Output POST(서버는 basename만 저장) | H:711-727, J:415-450, P:3460 | 한 폼: 입력칸 + 폴더 아이콘 버튼, 실행 버튼 하나가 Output 저장 → 실행 | `runMultiSession()` 래퍼: Output이 바뀌었으면 `set_output` 후 `handleOptBtnClick()`. Load 버튼은 아이콘 버튼으로 이동 | 중 | P3 |
| M2 | 500ms 폴링이 Output 칸을 포커스 없을 때 덮어씀 → 입력 후 바로 Run 누르면 옛 값으로 바뀔 수 있음 | J:652-655 | 입력한 값 유지 | `_multiOutputDirty` 플래그(입력 이벤트에서 true, set_output 성공 후 false). 폴링은 dirty면 안 덮음 | 중 | P3 |
| M3 | 실행 버튼 하나가 Run ↔ Exit(Exit = cancel API + 뷰어 닫기), 실행 중 disabled, Cancel 버튼은 배너 안 | J:457-639 | 헤더: Run/Exit 버튼(`#slam-opt-run-btn` 이동) + 실행 중 Cancel(새 `#multi-header-cancel-btn` → `cancelOptimization()`) | 버튼 이동 + 새 버튼 1개 | 중 | P3 |
| M4 | 진행 상태는 서버 `message` 화이트리스트 8종(`Session Edge Loading Complete.` … `Long Term SLAM Complete.`). **실제**: `main` 소스에서 순서 확인됨(`long_term_mapping.cpp:1431-1463`). 단, 지금 설치된 바이너리(ELite 변종)에는 이 문장이 0건 | P:3773-3782, J:567-603 | Progress **5단계** 목록 + 헤더 배지 "Optimizing · n/5" + 완료 후 "루프 N개" | `optDoneCount(message)`로 8개 메시지를 5단계에 묶음(3.3절), `size : N` 파싱. 기준 코드는 `main` 재빌드(확정 2026-10-16, Task 3.0) | 중 | P3 |
| M5 | 재진입 자동 복원 경로가 `_optComplete`를 안 세워 버튼이 Exit로 안 바뀜 | J:662-668 | 결과가 보이면 항상 Exit | 복원 분기에서 `_optComplete = true` + `_resetOptBtn` 반대 처리(버튼 글자 Exit) | 중 | P3 |
| M6 | 새로고침 시 진행 중 최적화 폴링이 다시 안 켜짐 | J:464-481 | 새로고침해도 진행 표시 | 전역 요약 `optimization.running && !_optPollTimer`이면 `_startOptPolling()` | 중 | P3 |
| M7 | 결과 레이어 12종 범례(Map1·Map2·MergeMap1/2·PD·ND·FirstUE·SecondUE·3궤적·Loop Closure), Check Difference 첫 활성화 시 diff PCD 4개 로드, Point Size 0.01~1.0, EDL, Height Clip | H:749-820, J:10518-10553 | 뷰어 안 칩 줄. Check Difference 켜면 PD/ND/FirstUE/SecondUE 칩이 이어서 나옴 | 범례 재스타일. 색은 Phase 4.15 팔레트 유지 | 낮 | P3 |
| M8 | `slam-status` 자유 문구("Map 1 loaded - path", "Error: …") | H:731, J:641 | 폼 아래 작은 글자 줄 | 이동만 | 낮 | P3 |
| M9 | **노드가 죽어도 성공으로 표시**: `ros2 launch`는 노드 비정상 종료에도 0을 반환, 서버는 반환 코드만 봄. `process has died` 줄과 `std::cerr` 오류는 버려짐 | P:3812, 3831-3847; `long_term_mapping.cpp:1424` | 실패 배지 + 원인 한 줄("입력 파일을 찾을 수 없음" 등) | Task 3.11: `process has died` 감지 + 결과 파일 확인 + 오류 줄 화이트리스트 확장 | 상 | P3 |
| M10 | **Output 이름으로 `rm -r`**: 입력 폴더와 같은 이름·`.`·`..`이면 입력이나 패키지가 지워짐(경로 가드 없음). 서버는 빈 값만 막음 | `long_term_mapping.cpp:630-641`, P:3701 | Output 칸 규칙 안내 + 위반 시 Run 잠금 + 빨간 글자 | 프론트 + 서버 검사(Task 3.11: `^[A-Za-z0-9_-]{1,64}$`, Map1/Map2 폴더 이름과 같으면 거부) | 상 | P3 |
| M11 | Map1/Map2 폴더 필수 내용물: `Scans/<k>.pcd`·`_ground`·`_nonground`, `optimized_poses.txt`(8열), `edges.txt`(14열), `StaticMap.pcd` = Save Map 결과 폴더 그대로 | `long_term_mapping.cpp:660-670` | 폴더 선택 직후 내용 검사, 없으면 칸 아래 "필요한 파일 없음: …" | Task 3.11의 서버 검사를 `set_map1/2` 응답에 포함 | 중 | P3 |
| M12 | 실행마다 `long_term_mapping/config/params.yaml`(git 추적 파일)을 `yaml.dump`로 덮어씀 → 주석 사라짐 | P:3896-3923 | 변경 없음 | 기록만(부록 D) | — | — |
| M13 | `StaticMap.pcd`(병합 정적 지도)·`Debug/KissMatchedMap.pcd`(전역 정렬 확인용)는 Multi 결과 뷰어에 없음 | J:10863-10874 | 선택: "Static Map" 칩 추가 | 범위 밖(부록 D 제안) | — | — |

### 2.5 Bag Player

| # | 동작 | 현재 위치 | 새 UI 사양 | 구현 방식 | 위험 | 단계 |
|---|---|---|---|---|---|---|
| B1 | Select Topic 모달(체크박스, ROS1은 타입·publishable, 0개 확정 시 alert). 선택은 프론트 상태(`bagPlayerState.selectedTopics`), 다음 Play 때 서버로 감 | J:845-927, H:1391-1406 | 토픽 표(Topic·Type·Hz·Msgs, 체크박스), "모두 선택/모두 해제", 재생 중이면 "다음 Play부터 적용" 안내 | `renderBagTopicTable()`가 같은 상태를 읽고 씀 → `updateSelectedTopicsDisplay()` 호출. 토픽 이름은 `textContent`(지금은 innerHTML이라 이스케이프 안 됨) | 중 | P3 |
| B2 | ROS2 bag 토픽은 이름만, Hz·메시지 수 없음 (`ros2 bag info` 줄의 `Count:`·`Type:`을 버림) | P:6350-6421 | 표의 Type·Hz·Msgs | 백엔드: 파서를 순수 함수로 빼고 `topic_details:[{name,type,count}]` 추가(`topics`는 유지). Hz = count/duration을 프론트에서 계산. ROS1 count는 미확인 | 상 | P3 |
| B3 | ROS1/ROS2 배지, Convert(포맷 3종, 같은 포맷 차단, 선택 토픽만, **동기 HTTP, 진행률 없음**), 변환 후 자동 로드 | H:842-854, J:1237-1314 | 오른쪽 "형식 변환" 패널, 배지는 `ui-tag`, 변환 중 무한 진행 바 + 버튼 잠금 | 재스타일 + 버튼 잠금은 기존 disabled 사용 | 낮 | P3 |
| B4 | 재생 속도 = 연속 슬라이더(0.1~4.0×, `value/10`), 재생 중이면 매 입력마다 `/api/bag/set_rate` | H:879-882, J:1193-1231 | 하단 바 0.5/1/2/4× 분할 버튼 + 기존 0.1 단위 슬라이더를 "세밀 조절"로 접어 유지 (확정 2026-10-16) | 분할 버튼이 슬라이더 값을 바꾸고 `updatePlaybackRate()` 호출 | 중 | P3 |
| B5 | Loop·속도 컨트롤은 로드 후에만 나타남(`#ros1-playback-controls`, 이름과 달리 ROS2에도 사용) | J:766-767 | 하단 재생 바는 항상 보이고, 로드 전엔 잠김 | 컨트롤을 바로 옮기고 잠금은 `deriveBagView` | 중 | P3 |
| B6 | Play↔Stop, Pause↔Resume **글자가 곧 상태**(J:1203-1206 비교), ROS1은 별도 API·별도 상태 폴링 | J:947-1186 | 아이콘 + 글자 버튼 | `uiKit.setBtnLabel(btn, text)`: 아이콘 `<svg>` + `<span class="ui-btn-label">`. `textContent` 비교는 svg 글자가 없어 그대로 동작. 라벨 대입 `btn.textContent = …`만 헬퍼로 교체 | 중 | P3 |
| B7 | 시크 슬라이더 드래그 가드(`_bagSliderDragging`, pointer 리스너 J:11114-11120), `_bagSeeking` | J:1041-1050 | 하단 바 슬라이더 | 같은 요소 이동 | 낮 | P3 |
| B8 | 새로고침 복원(`restoreBagPlayerFromServer`) | J:777-803 | 그대로 | 변경 없음 | — | — |

### 2.6 Bag Recorder

| # | 동작 | 현재 위치 | 새 UI 사양 | 구현 방식 | 위험 | 단계 |
|---|---|---|---|---|---|---|
| R1 | Enter Bag Name = Save-As 모달(폴더 탐색 + 이름 입력, 새 폴더 없음, 덮어쓰기 경고 없음) | H:1365-1388, J:2413-2493 | Bag Name 줄(읽기 전용) + 폴더 아이콘 버튼 → 같은 모달 | 버튼 이동·재스타일. 인라인 직접 입력은 하지 않음(폴더 선택 흐름 유지) | 낮 | P3 |
| R2 | Select Topic 모달은 라이브 그래프 토픽(`/api/recorder/get_topics`, 2초 캐시) | J:2505-2585 | 토픽 표(체크, Type). Hz 칸 없음 (확정 2026-10-16: Hz는 Bag Player만) | 표 렌더 함수가 같은 상태(`bagRecorderState.selectedTopics`, `{name,type}`)를 씀 | 중 | P3 |
| R3 | 경과 시간·크기 없음. `/api/recorder/state`는 `bag_name, recording, mode`만 | P:4212 | 큰 Record 버튼 + 시간·크기·토픽 수 | 백엔드 `elapsed_sec`·`size_bytes` 추가(3.5절) | 상 | P3 |
| R4 | 녹화 시작/중지가 성공 alert로 막힘, 새로고침 시 녹화 상태 복원 안 됨 | J:2636-2638, J:2645 | 토스트, 녹화 중 이름·형식 잠금, 새로고침 후 REC 복원 | Recorder 화면에서도 500ms 폴링에 `updateRecorderState()` 추가 + 훅 | 중 | P3 |
| R5 | 녹화 프로세스가 죽어도 `recorder_recording`이 안 풀림(poll 없음) | P:4085-4115 | — | 기록만(범위 밖, 부록 D) | — | — |

### 2.7 File Player

| # | 동작 | 현재 위치 | 새 UI 사양 | 구현 방식 | 위험 | 단계 |
|---|---|---|---|---|---|---|
| F1 | 데이터셋 select 5종 + 블록 5벌 복붙(ConPR 저장 행, kitti/kaist/mulran/helipr-ui) | H:951-1109 | 분할 버튼 + 하나처럼 보이는 폼 | 분할 버튼이 숨은 `#dataset-format-select` 값을 바꾸고 `change` 발생 → 기존 `onDatasetFormatChange`. 블록 DOM 통합은 하지 않음(5벌을 같은 모양으로) | 중 | P3 |
| F2 | KITTI는 Drive(+`calib_dir`), KAIST/MulRan/HeLiPR는 Sequence, MulRan·HeLiPR은 1개면 자동 선택, 선택 즉시 Direct Play 로드 | J:1480-2096 | 라벨 "Drive"/"Sequence"만 다름 | 변경 없음 | 낮 | P3 |
| F3 | 데이터셋별 진행 바: KITTI/KAIST/MulRan/HeLiPR는 8081 WS `*_convert_progress/done/error`, ConPR은 HTTP 500ms 폴링(`save_bag_*`) | J:4293-4431, J:2276-2354 | "Bag으로 저장" 패널 하나처럼: 형식 select + Save bag + 진행 바 + 메시지 | 5벌을 같은 클래스로. KITTI 핸들러 null 가드 추가(지금 없음) | 중 | P3 |
| F4 | Loop·Auto Start 토글(서버 값으로 매번 덮음), Auto Start는 로드 성공 후 `playPlayer()` | H:1124-1143 | 오른쪽 "재생 옵션" 패널 | 이동·재스타일 | 낮 | P3 |
| F5 | Play↔End, Pause↔Resume 글자 = 상태, 시크 슬라이더 드래그 가드 없음, 프레임 카운터 없음(Timestamp만) | J:2260-2410, H:1111-1150 | 하단 재생 바(아이콘 + 글자), 오른쪽에 Timestamp | `setBtnLabel` 적용. 프레임 카운터는 만들지 않음(목업의 "frame n/N"은 Timestamp로 대체) | 중 | P3 |
| F6 | `/api/player/set_rate`(0.1~2.0)가 백엔드에만 있고 UI 없음 | P:9169 | 변경 없음(목업에도 없음) | 기록만 | — | — |
| F7 | HeLiPR LiDAR 선택 UI 없음(`sensors` 미전송, 서버 기본값) | P:6035 | 변경 없음 | 기록만, 기본값 미확인 | — | — |

### 2.8 Plot / 3D Viewer

| # | 동작 | 현재 위치 | 새 UI 사양 | 구현 방식 | 위험 | 단계 |
|---|---|---|---|---|---|---|
| V1 | Pause/Resume은 **탭별 Plotly modebar 버튼**(`togglePause(forceState)` plot_manager.js:2576), 전역 없음 | plot_manager.js:590-598 | 헤더 Pause/Resume 하나가 모든 탭에 적용 + 그래프에 "일시정지" 글자 | `plotState.plotTabManager.tabs.forEach(t => t.plotManager.togglePause(paused))` | 중 | P4 |
| V2 | Buffer 입력(1~100초, 기본 5) = 시간 창 | H:1191-1202, J:5537 | 헤더 10/30/60 s 분할 버튼(+ 기존 입력은 "직접 입력"으로 유지) | 분할 버튼이 `updateBufferTime(sec)` 호출 | 낮 | P4 |
| V3 | 필드 검색 없음. "Filter"는 데이터 변환 모달(7종) | — , H:1579-1635 | 토픽 목록 위 "필드 찾기" 입력. Filter 모달은 "Apply Filter"로 이름 유지 | `plot_tree.js`에 `filterByText(query)` 추가 | 중 | P4 |
| V4 | 트리 값 칸은 첫 메시지 1회만(실시간 아님). `updateValue(path, value)`는 있음(plot_tree.js:281) | J:5417, 5434 | 각 그래프 카드 오른쪽 위 현재값 + 트리 값 실시간(4Hz) | `plot_data` 수신부(J:4294-4311)에서 throttle 후 `plotState.tree.updateValue`. payload 필드명은 실행 시 확인 | 중 | P4 |
| V5 | `+ Add` 토픽 선택 모달, 드래그&드롭, Ctrl 다중 선택 → XY Plot 우클릭 메뉴, Plot 탭(최대 10, 이름 편집, `sessionStorage` 복원), 우클릭 메뉴(Settings·Auto Scale·Clear·Flip·PNG·Apply Filter·Delete), Plot Settings 모달 | plot_*.js, H:1410-1575 | 동작 그대로. 탭 바·메뉴·빈 상태를 다크 토큰으로 | `plot_tab_manager.js` 인라인 `cssText`를 클래스로, 메뉴·빈 상태 하드코딩 흰색 제거, Plotly 레이아웃 색을 CSS 변수에서 읽음 | 중 | P4 |
| V6 | 분할 패널 4개는 미구현("Coming soon" 메뉴, 호출 안 됨) | plot_manager.js:1023 | 변경 없음 | 기록만 | — | — |
| V7 | 3D Displays는 체크박스(눈 아이콘 아님), ✕는 LaserScan·TF·Livox에만, 타입별 설정(PC2 6스타일·색 모드·Decay 등) | T:3234-4468 | 체크박스를 눈 아이콘 모양으로(`aria-label="표시"`) | CSS(`appearance:none` + mask 아이콘). ✕ 일관화는 하지 않음 | 낮 | P4 |
| V8 | Reset(구독은 유지, 오브젝트만 제거 — title 문구와 다름)/Snapshot이 캔버스 위 절대 위치 | H:1216-1233, T:1847-1862 | 화면 헤더로 이동, Reset 툴팁을 실제 동작대로 "씬 초기화(구독 유지)" | 이동 + title 수정 | 낮 | P4 |
| V9 | Views: Orbit/TopDown, Zero, Near Clip, Target Frame은 동작. **Distance/Yaw/Pitch/Scale은 값만 저장, 카메라 미반영** | T:574-935, T:908-912 | 동작하는 항목만 보이고 나머지는 숨김 | `renderViewsPanel`에서 4개 입력에 `hidden` | 낮 | P4 |
| V10 | Image 패널(8081 바이너리, 접기·드래그 리사이즈 80~600px), 배경 `#2a2a2a` 고정, Fixed Frame 콤보, Add Display 모달 7타입 | T:2745-2980, T:462, T:6213-6455, T:6728-6855 | 동작 그대로, 새 모양. 배경 `#07090C` | CSS + 배경 상수 1줄 | 낮 | P4 |

---

## 3. 상태 기계 정리와 UI 매핑

**공통 규칙**
- 화면마다 입력(서버 필드 + 기존 플래그 + 기존 버튼 글자)을 모아 `UiStatusLogic.deriveXxxView(input)`에 넣는다. 결과 `{ tone, text, step, buttons }`를 `ui_shell.js`가 헤더 배지·버튼 `disabled`·단계 표시줄에 그린다.
- `tone`은 `go | info | warn | stop | idle` 중 하나다. 배지는 항상 점 + 글자를 같이 보여 준다(색만으로 표시 금지).
- 렌더 시점: ① 기존 500ms 폴링 함수 끝의 훅 ② 클릭 핸들러가 플래그를 바꾼 직후 `uiShell.refresh()` ③ 전역 요약 2초 폴링.
- 버튼 잠금은 **화면이 보일 때만** 의미가 있다. 숨은 화면은 사이드바 표시만 갱신한다.

### 3.1 LiDAR SLAM

**입력**

| 입력 | 출처 | 비고 |
|---|---|---|
| `isRunning` | `/api/slam/state`.`is_running` (P:3930, `slam_process.poll()`) | 500ms, 화면 보일 때. **launch 프로세스만 본다** — `fastlio_mapping`이나 PGO 노드만 죽으면 계속 true(13.1절) |
| `kfCount` | `/lio_analytics`의 키프레임 수 필드(`LioAnalytics.msg` Keyframe 절, 필드 이름은 `kf_count`로 추정 — 미확인) → 대시보드 Full Statistics 값 요소 `#ad-<필드>` 글자 | 0이면 Save Map 잠금(키프레임 0개 저장은 PGO에서 정의되지 않은 동작, 13.2절) |
| `stale` | 마지막 `/lio_analytics` 수신 후 3초 이상 지남(대시보드 메시지 핸들러에 `uiShell.noteAnalytics('slam')` 한 줄) | 실행 중인데 데이터가 끊긴 상태 = 노드 죽음·rosbridge 끊김 의심 |
| `imuHz`·`lidHz` | `/lio_analytics`.`imu_freq`·`lid_freq` (1초 벽시계 카운터) | 실행 중 0이면 센서 끊김 경고. bag 배속이 1×가 아니면 값이 배속만큼 달라진다 |
| `starting` | `window._slamLiveViewerHoldOpen` (J:3381-3395) | Start API 응답 전까지 true |
| `stopping` | `window._slamStopping` (J:3413-3434) | Stop API 응답 전까지 true |
| `saving` | `window._slamSaving` (J:3190 등) | 저장 요청~완료 |
| `justSaved` | `window._slamMapJustSaved` (J:3316) | 저장 완료 후 SLAM 계속 실행 중 |
| `resultVisible` | `#savemap-result-viewer`의 `style.display !== 'none'` | 새로고침 후 `_maybeRestoreSaveMapViewer` 복원 포함 |
| `elapsed` | `#analytics-run-time` 글자(HH:MM:SS) | 대시보드가 채움, 없으면 생략 |

**파생 (우선순위 위→아래, 먼저 맞는 줄)**

| 조건 | 배지(tone · 글자) | 단계 (Config→Mapping→Save Map→Result) | Start | Stop | Save Map |
|---|---|---|---|---|---|
| `stopping` | warn · Stopping… | Mapping | 잠금 | 잠금 | 잠금 |
| `starting` | info · Starting… | Mapping | 잠금 | 잠금 | 잠금 |
| `saving` | info · Saving Map… | Save Map | 잠금 | 잠금 ※ | 잠금(글자 "Saving…") |
| `isRunning && stale` | warn · Running · 데이터 없음 | Mapping | 잠금 | 열림 | 잠금 |
| `isRunning && (imuHz === 0 \|\| lidHz === 0)` | warn · Running · IMU 끊김 / LiDAR 끊김 | Mapping | 잠금 | 열림 | `kfCount > 0`이면 열림 |
| `isRunning && justSaved` | info · Map Saved · SLAM 계속 실행 중 | Result | 잠금 | 열림 | `kfCount > 0`이면 열림 |
| `isRunning` | go · Running · `elapsed` | Mapping | 잠금 | 열림 | `kfCount > 0`이면 열림 |
| `!isRunning && resultVisible` | idle · Ready · 저장된 지도 표시 중 | Result | 열림 | 잠금 | 잠금 |
| 그 외 | idle · Ready | Config | 열림 | 잠금 | 잠금 |

※ 목업은 저장 중에도 Stop이 열려 있다. 서비스 호출 중 프로세스를 죽이면 저장이 깨질 수 있어 **잠금으로 바꾼다**. 저장 중 Cancel은 오버레이에 두되, PGO가 단일 스레드라 노드 쪽 저장은 끝까지 진행된다(13.2절). 그래서 버튼 글자는 "기다리기 중단"이고, 누르면 토스트 "저장은 노드에서 계속됩니다. 끝나면 결과 폴더가 만들어집니다"를 띄운다(확정 2026-10-16. PGO 진짜 취소는 부록 E 백로그). 사용자에게 알릴 동작 변경이다.

"Map Saved · SLAM 계속 실행 중"은 실제와 맞다. 저장 뒤에도 FAST_LIO와 PGO 노드는 계속 돈다. 다만 저장하는 동안 PGO가 `/key_frame`을 받지 못해(단일 스레드 executor) 키프레임이 빠질 수 있다(미확인, 큐 30개). 저장 오버레이에 "저장 중에는 새 키프레임이 그래프에 추가되지 않습니다. 로봇을 멈추고 기다리세요"를 적는다.

Save Map 잠금은 새 동작이다(지금은 SLAM이 꺼져 있어도 모달이 열림). Stop 확인: 이번 실행에서 저장 성공이 한 번도 없으면 Stop을 누를 때 `uiKit.confirmDialog`로 "저장하지 않고 멈추면 이번 지도는 다음 Start 때 지워집니다"를 묻는다(PGO가 시작할 때 `pose_graph_optimization/Map/`을 `rm -r`, 13.2절). "저장 성공 여부"는 `ui_shell.js`가 `saving → justSaved` 전이를 보고 `slamSavedSinceStart`로 따로 기억한다(기존 플래그는 건드리지 않음). 결과 뷰어·빈 지도 칸 표시: `#slam-live-viewer`도 결과 뷰어도 안 보이면 `#lidar-slam-viewer-empty`(새 DOM)를 보인다. `starting`이면 빈 칸 글자를 "rosbridge 연결 대기 중…"으로 바꾼다.

### 3.2 Localization (깜빡임 방지 포함)

**입력**: `isRunning` = `/api/localization/state`.`is_running`(자기 프로세스 **또는** `/loc_analytics` 퍼블리셔 존재, P:4011-4024) / `starting` = 새 `window._locStarting` / `stopping` = 새 `window._locStopping` / `initialized` = `#loc-header-init-badge.classList.contains('ok')` / `external` = 화면 폴링 `isRunning === true` 이고 전역 요약 `localization.running === false` / `stale` = 마지막 `/loc_analytics` 수신 후 3초 이상(`uiShell.noteAnalytics('loc')`) / `lastStartFailed` = Start 성공 응답 뒤 10초 안에 `/loc_analytics`가 한 번도 오지 않음(`ui_shell.js`가 기억).

| 조건 | 배지 | 단계 (Config→Initialize→Tracking) | Start | Stop |
|---|---|---|---|---|
| `stopping` | warn · Stopping… | Initialize | 잠금 | 잠금 |
| `starting` | info · Starting… | Initialize | 잠금 | 잠금 |
| `isRunning && external` | warn · Running (외부 실행) | Initialize/Tracking | 잠금 | 잠금(툴팁 "웹 UI가 시작하지 않은 Localization") |
| `isRunning && stale` | warn · Running · 데이터 없음 | Initialize | 잠금 | 열림 |
| `isRunning && initialized` | go · Initialized · Tracking | Tracking | 잠금 | 열림 |
| `isRunning` | warn · Running · Not Init | Initialize | 잠금 | 열림 |
| `!isRunning && lastStartFailed` | stop · 시작 실패 (로그 보기) | Config | 열림 | 잠금 |
| 그 외 | idle · Ready | Config | 열림 | 잠금 |

**초기화 판정의 실제 (13.3절)**: `is_initialized`는 노드의 `initial_flag`다. `dop_ratio = down_dop / matching_dop`가 0.8 이상이 되면 true가 된다(`laserLocalization.cpp:1367, 1377-1390`). 노드는 **지도 원점(항등 자세)에서 스스로 맞추기를 시도**하고, 맞을 때까지는 EKF 갱신을 버리고 속도를 0으로 둔다. `/initialpose`를 받으면 그 자세로 다시 초기화한다(`laserLocalization.cpp:752-829`). 그래서 "Running · Not Init"이 오래가면 헤더에 힌트 "지도 원점 근처에서 시작하거나, 초기 위치를 지정하세요"를 보여 준다(10초 이상, 값은 임의). 초기 위치 지정 버튼은 Task 2.12(확정 2026-10-16).

**시작 실패**: 지도 파일을 못 열면 노드가 "Map file cannot open."을 남기고 꺼진다(`laserLocalization.cpp:1546-1549`). launch 프로세스는 octomap_server 때문에 살아 있을 수 있어 `is_running`만으로는 안 보인다(미확인). Start 뒤 10초 안에 `/loc_analytics`가 한 번도 안 오면 `lastStartFailed`로 보고 stderr 로그 마지막 줄들을 보여 준다(Task 2.10의 로그 API).

**깜빡임 방지 (지금 J에 없는 것)**

1. Start: `startLocalizationMapping()` 처음에 `window._locStarting = true`, `try { … } finally { window._locStarting = false; }`. 뷰어 `show()`는 지금처럼 **API 성공 뒤**에만 부른다(Start 직후 깜빡임은 이미 낮음).
2. Stop: 처음에 `window._locStopping = true`. `hide()` 직후 `window._locStoppedAt = performance.now()`. API 응답 뒤 `window._locStopping = false`.
3. `updateLocalizationState()`:
   - 요청 전에 `const reqAt = performance.now();`
   - 응답 뒤 `if (reqAt < (window._locStoppedAt || 0)) return;` → Stop 전에 보낸 요청의 늦은 응답을 버린다.
   - 복원 분기 조건에 `!window._locStopping && performance.now() - (window._locStoppedAt || 0) > 3000` 추가 → DDS 퍼블리셔 수 지연(최대 3초) 동안 재출현을 막는다. 3초 값은 미확인(실측 필요).
4. `LocAnalyticsDashboard._resetWidgets()`(J:12244)에서 두 배지를 `— Not Init` + `ok`/`bad` 클래스 제거로 초기화한다.

### 3.3 Multi-Session SLAM

**입력**: `running` = `_optRunning` / `complete` = `_optComplete` / `failed` = `#slam-opt-status-area.classList.contains('error')` / `inputsReady` = `#slam-map1`·`#slam-map2`·`#slam-output` 값이 모두 비어 있지 않음 / `message` = 마지막 `optimization_status.message`(`_pollOptimizationStatus`가 `#slam-opt-msg`에 씀 → 그 글자를 읽음).

**진행 5단계 묶음** (`optDoneCount(message)`). `long_term_mapping` `main` 소스의 실제 실행 순서(`long_term_mapping.cpp:1431→1463`)로 확인했다. 목업은 4단계였지만 마지막 칸의 "Map Update"는 저장이 아니라 **변화 검출(PD/ND/UE)과 정적 지도 생성**이고 무거운 단계라 둘로 나눴다(13.4절). 기준 코드는 `main` 소스 재빌드로 확정했다(2026-10-16, Task 3.0). 재빌드 전 설치 바이너리로는 이 메시지가 하나도 안 나온다.

| 단계 | 끝났다고 보는 메시지 (`startsWith`) | 하는 일 |
|---|---|---|
| 1 세션 불러오기 | `Session Edge Loading Complete.` | 두 세션의 포즈·엣지 파일 읽기 |
| 2 세션 정렬·루프 찾기 | `Place Recognition Complete.`, `Loop Edge Generation Complete.` | SOLiD 디스크립터, KISS-Matcher 전역 정렬, 루프 정합. 메시지 끝 `size : N`의 N = 세션 사이 루프 수 |
| 3 포즈 그래프 최적화 | `Pose Factor loading Complete.`, `Graph Optimization Complete.` | iSAM2, `optimized_poses.txt` 완성 |
| 4 병합 지도 만들기 | `Map Merging Complete.` | `FirstMap/SecondMap.pcd` |
| 5 변화 검출·정적 지도 | `Map Update Complete.`, `Long Term SLAM Complete.` | `Debug/PD·ND·FirstUE·SecondUE.pcd`(비어 있으면 파일 없음), `StaticMap.pcd`, 엣지 저장 |

**새 입력**: `loopCount` = 마지막 `Loop Edge Generation Complete. size : N`에서 꺼낸 N(`/size : (\d+)/`) / `outputsOk` = 완료 후 결과 파일 확인 결과(Task 3.11의 서버 검사, `optimization_status`에 `outputs_ok` 추가).

| 조건 | 배지 | 단계 (Inputs→Optimization→Result) | Run/Exit (`#slam-opt-run-btn`) | Cancel (새 헤더 버튼) |
|---|---|---|---|---|
| `running` | info · Optimizing · n/5 | Optimization | 잠금(지금도 disabled) | 열림 |
| `complete && loopCount === 0` | warn · Done · 세션 사이 루프 0개 | Result | 열림, 글자 "Exit" | 잠금 |
| `complete` | go · Done · 루프 N개 | Result | 열림, 글자 "Exit" | 잠금 |
| `failed` | stop · Failed | Inputs | `inputsReady`면 열림 | 잠금 |
| 그 외 | idle · Ready (입력 부족이면 "Inputs needed") | Inputs | `inputsReady`면 열림 | 잠금 |

**실패가 성공으로 보이는 문제 (13.4절)**: 웹 서버는 `ros2 launch` 반환 코드가 0이 아닐 때만 실패로 본다(P:3831-3847). 그런데 `ros2 launch`는 노드가 죽어도 0을 돌려준다. 그래서 입력 파일이 없어 노드가 `return -1`로 끝나도 지금은 "Optimization complete"가 된다(코드로만 확인, 실행 안 함). Task 3.11에서 서버가 ① `process has died` 줄을 실패로 처리하고 ② 끝난 뒤 `optimized_poses.txt`·`FirstMap.pcd`·`SecondMap.pcd`가 있고 비어 있지 않은지 확인한 뒤에만 성공으로 정한다.

**Cancel 뒤 남는 파일**: 부분 결과 폴더가 남고, 다음 실행이 같은 이름이면 `rm -r`로 지운다. UI는 취소 뒤 결과 뷰어를 열지 않는다(지금과 같음). 취소 토스트에 "부분 결과가 `long_term_mapping/<Output>/`에 남아 있습니다"를 적는다.

### 3.4 Bag Player

**입력**: `loaded` = `#bag-directory.value !== ''` / `playing` = `#bag-play-button` 글자가 `Stop` / `paused` = `#bag-pause-button` 글자가 `Resume` / `rate` = `bagPlayerState.playbackRate` / `converting` = `#convert-bag-btn.disabled`. 글자가 곧 상태인 기존 방식(J:1071-1180)을 그대로 읽는다. ROS1·ROS2 분기는 J가 이미 버튼 글자에 반영한다.

| 조건 | 배지 | Play | Pause | 시크·속도·Loop | Convert |
|---|---|---|---|---|---|
| `!loaded` | idle · No bag | 잠금 | 잠금 | 잠금 | 잠금 |
| `converting` | info · Converting… | 잠금 | 잠금 | 잠금 | 잠금 |
| `paused` | warn · Paused | 열림(Stop) | 열림(Resume) | 열림 | 잠금 |
| `playing` | go · Playing · `rate`× | 열림(Stop) | 열림 | 열림 | 잠금 |
| 그 외 | idle · Loaded | 열림 | 잠금 | 열림 | 열림 |

### 3.5 Bag Recorder

**입력**: `recording` = `#recorder-record-button` 글자 `Stop` 또는 `/api/recorder/state`.`recording` / `hasName` = `#recorder-bag-name.value` / `topicCount` = `bagRecorderState.selectedTopics.length` / `mode` = 서버 `mode` / `elapsedSec`·`sizeBytes` = 새 서버 필드.

| 조건 | 배지 | Record | 이름·형식·토픽 선택 |
|---|---|---|---|
| `recording` | stop · Recording · m:ss | 열림(글자 "녹화 멈추고 저장") | 잠금 |
| `hasName && topicCount > 0` | idle · Ready | 열림 | 열림 |
| 그 외 | idle · 이름·토픽 필요 | 잠금 | 열림 |

녹화 배지는 목업대로 stop(빨강) 톤이지만 글자 "Recording"을 같이 쓴다.

### 3.6 File Player

**입력**: `loaded` = 서버 `/api/player/state`.`data_loaded`(지금 프론트가 안 읽는 필드) / `playing` = `#play-button` 글자 `End` / `paused` = `#pause-button` 글자 `Resume` / `converting` = `kittiState|kaistState|mulranState|heliprState.converting` 중 하나 / `savingBag` = 서버 `save_bag_saving` / `progress` = 해당 진행 바 %.

| 조건 | 배지 | Play/Pause/시크 | Load·데이터셋 전환 | Save bag |
|---|---|---|---|---|
| `converting` or `savingBag` | info · Saving bag · n% | 열림 | 잠금 | 잠금 |
| `playing` | go · Playing | 열림 | 잠금 | 잠금 |
| `paused` | warn · Paused | 열림 | 잠금 | 열림 |
| `loaded` | idle · Loaded | 열림 | 열림 | 열림 |
| 그 외 | idle · Not loaded | 잠금 | 열림 | 잠금 |

---

## 4. 사이드바 실행 표시(RUN/REC/PLAY/OPT)와 전역 상태 API

### 4.1 문제

- 500ms 폴링은 보이는 서브탭 하나만 돈다(J:4175-4193). Recorder는 폴링이 아예 없다.
- 여러 모듈 상태를 한 번에 주는 엔드포인트가 없다. 지금 API로 하려면 약 9개를 두드려야 하고, ROS1 bag 재생은 `bag_playing`을 세우지 않아 `/api/bag/state`만으로는 안 보인다(P:6519-6577).
- 사이드바 표시를 위해 기존 상태 API 9개를 묶음 폴링하는 안은 요청 수가 많고 `/api/localization/state`가 DDS 그래프 조회(`count_publishers`)를 할 수 있어 **탈락**.

### 4.2 결정: 새 엔드포인트 `GET /api/system/status_summary` (백엔드 변경 필요)

- 위치: `do_GET`의 `/api/system/info` 분기(P:8551-8560) 바로 뒤. 로직은 `WebGUINode.get_status_summary()`로 `get_localization_state`(P:4011) 근처에 둔다.
- 비용: 메모리 속성 읽기 + `poll()` 4번 + ROS1 스레드 상태 1번. DDS 조회·subprocess·파일 IO 없음. 2초 폴링이면 부담 무시 가능.
- 형식: 룰 `02` §6.2를 따른다(이웃 상태 API는 평평한 필드라 다르다는 점을 주석으로 남김).

```json
{
  "success": true,
  "data": {
    "slam":         {"running": true,  "saving_map": false},
    "localization": {"running": false},
    "optimization": {"running": false},
    "bag_player":   {"loaded": true,  "playing": true,  "paused": false},
    "recorder":     {"recording": false, "mode": "ros2"},
    "file_player":  {"loaded": false, "playing": false, "paused": false, "converting": false, "saving_bag": false}
  }
}
```

| 필드 | 읽는 속성 (탐색에서 확인) |
|---|---|
| `slam.running` | `slam_process` + `poll() is None` (P:3932과 같은 식) |
| `slam.saving_map` | `slam_map_saving` (P:2586) |
| `localization.running` | `localization_process` + `poll() is None` (퍼블리셔 검사는 **안 함** — 외부 실행 Localization은 사이드바에 안 뜸, 화면 폴링에서만 보임) |
| `optimization.running` | `slam_opt_running` (P:2582) |
| `bag_player.*` | `bag_path`, `bag_playing`, `bag_paused` + ROS1: `ros1_player_thread.is_alive()`와 `get_status()['status']` |
| `recorder.*` | `recorder_recording`, `recorder_mode` |
| `file_player.*` | `player_data_loaded`, `player_playing`, `player_paused`, `player_is_ros2_bag`/`player_is_ros1_bag`(bag 모드면 bag 상태 사용), `kitti/kaist/mulran/helipr_converter_running`, `save_bag_saving` |

File Player가 bag 모드로 재생하면 `bag_playing`이 서기 때문에 Bag Player 표시도 같이 켜진다. 서버 상태가 하나라 생기는 현상이며 그대로 둔다(부록 D).

### 4.3 프론트

- `ui_shell.js`가 2초마다 `apiCall('/api/system/status_summary')`를 부른다. 실패하면 표시를 지우지 않고 마지막 값을 유지하며, 연결 블록에 "상태 조회 실패"를 띄우지 않는다(latency가 이미 서버 상태를 보여 줌).
- `deriveSidebarMarkers(data)` 결과를 각 `.ui-sidebar-item[data-screen=…] [data-run-tag]`에 넣는다.

| 사이드바 항목 | 조건 → 표시(글자 + 점) |
|---|---|
| LiDAR SLAM | `slam.saving_map` → SAVE(info) / `slam.running` → RUN(go) |
| Localization | `localization.running` → RUN(go) |
| Multi-Session SLAM | `optimization.running` → OPT(info) |
| Bag Player | `paused` → PAUSE(warn) / `playing` → PLAY(go) |
| Bag Recorder | `recording` → REC(stop) |
| File Player | `converting || saving_bag` → SAVE(info) / `paused` → PAUSE(warn) / `playing` → PLAY(go) |
| Plot / 3D Viewer | 없음 |

- 같은 요약으로 두 가지를 더 한다: Multi 진행 중 새로고침 시 `_startOptPolling()` 재개(M6), Recorder 녹화 중 새로고침 시 `updateRecorderState()` 1회 호출(R4).

### 4.4 문서

`02` §6에 새 소절 "시스템 API"를 만들고 다음 줄을 넣는다(Task 1.1 Step 6). 탐색에서 찾은 문서 누락(`/api/localization/*`, `/api/bag/*`, `/api/recorder/*`, `/api/player/*` 등)은 이 계획 범위 밖이며 부록 D에 기록한다.

```
GET  /api/system/status_summary         # 사이드바 실행 표시용 전역 상태 요약 ✅ (🆕 Phase 8.1)
GET  /api/system/process_log            # 노드 stderr 로그 마지막 N줄 (name=slam|localization) ✅ (🆕 Phase 8.2)
GET  /api/slam/save_map_check           # Save Map 이름 검증·존재 확인 ✅ (🆕 Phase 8.2)
```

응답 필드만 늘어나는 기존 엔드포인트(P2·P3): `/api/bag/load`·`/api/bag/get_info`(`topic_details`), `/api/recorder/state`(`elapsed_sec`, `size_bytes`), `/api/slam/set_map1|2`(`missing`), `/api/slam/optimization_status`(`outputs_ok`, `loop_count`), `/api/slam/save_map`·`set_output`·`optimize`(이름 검증 실패 시 `error`).

---

## 5. 단계별 로드맵 요약 (P0~P6)

| 단계 | 목표 | 변경 파일 | id 보존 전략 | 검증 | 롤백 | 규모 |
|---|---|---|---|---|---|---|
| **P0** 토큰·글꼴·컴포넌트 CSS | 색·글꼴·버튼/패널 모양만 바꿈. 배치 그대로 | `ui_theme.css`(신규), C(글꼴 4규칙·`@import`), H 1줄, 글꼴 파일, 검증 도구 | 마크업 변경 없음 | 빌드 + `check_smoke.py` 8화면 스크린샷(전/후 비교) + `check_ids.py` | `<link ui_theme.css>` 한 줄 제거 또는 브랜치 폐기 | +800줄, 0.5~1일 |
| **P1** 셸 | 사이드바·연결 블록·화면 헤더 뼈대·전역 상태 API, 탭 함수 안전화 | H +150, `ui_kit.js`·`ui_status_logic.js`·`ui_shell.js`(신규), J ±60, P +40, 테스트 | 옛 탭·칩은 숨김(JS 계속 씀). 새 요소만 새 id | pytest + Node + 빌드 + `check_shell.py`(8항목 클릭, 표시 갱신, 콘솔) | `localStorage webui.legacyNav=1`로 즉시 옛 UI, 또는 revert | +1,000줄, 1.5~2일 |
| **P2** SLAM / Loc | 헤더 액션·단계 표시줄·8/4 배치·빈 지도 칸·저장 오버레이·Config 읽기 전용·대시보드 재배치, Loc 깜빡임 방지, **저장 이름 검증·로그 API·노드 건강 표시(2.10·2.11), 초기 위치 지정(2.12)** | H(약 650줄 이동), `ui_theme.css` +300, J +200, `ui_status_logic.js` +100, P +70, 테스트 | 버튼·뷰어·대시보드 DOM은 같은 id로 이동만. 속성 선택자 규칙은 버튼 클래스 바꿀 때 지움 | pytest + Node + 빌드 + `check_slam_loc.py` + **실데이터 체크리스트 8항목(사용자)** | 태스크 커밋 revert | **3.5~4.5일** |
| **P3** Multi / Player / Recorder / File | 단일 폼, 토픽 표, 하단 재생 바, Recorder 통계, 데이터셋 분할 버튼, **Multi 실패 감지·입력/출력 검증(3.11), long_term_mapping 기준 코드 맞추기(3.0)** | H(약 500줄 이동), J +330, P +150, `ui_status_logic.js` +100, 테스트 | 숨은 select·모달은 남김(P6 정리). 새 요소만 새 id | pytest + Node + 빌드 + `check_data.py` + 실데이터(사용자) | revert | **3.5~5일** |
| **P4** Plot / 3D | 전역 Pause·시간 창·필드 검색·현재값, 탭 바/메뉴 다크화, Displays 눈 아이콘, 헤더 버튼 | `plot_tree.js`·`plot_manager.js`·`plot_tab_manager.js`, T, H, J 소량 | 이동만 | 빌드 + `check_viz.py` + 실데이터(사용자: Plot 값 갱신) | revert | +250줄, 1.5일 |
| **P5** 알림·접근성·반응형 | alert→토스트, confirm→대화상자, 모달 ESC·aria, 4단 반응형, 축소 사이드바 | `ui_kit.js`, `ui_theme.css`, J 다수(기계적), T, `plot_tab_manager.js` | 없음 | 빌드 + `check_a11y.py`(아이콘 버튼 이름표, alert 0) + 1024/1280/1440 스크린샷 | shim 한 줄 제거로 alert 복귀 | +300 / J ±150, 1.5~2일 |
| **P6** 정리 | SVG 스프라이트, 인라인 style 정리, 죽은 CSS·옛 탭·되돌리기 스위치 제거, 문서 | H, C, J | `check_ids.py`에 "삭제 허용 목록"을 넘겨 참조 0인 id만 지움 | 빌드 + 전체 `check_*.py` | revert | −800줄 내외, 2일 |

합계 대략 **14~18일**(페어 프로그래밍, P0 0.5~1 + P1 1.5~2 + P2 3.5~4.5 + P3 3.5~5 + P4 1.5 + P5 1.5~2 + P6 2). ROS 패키지 대조(13장)와 결정 확정(2026-10-16) 뒤 P2 +1.5일(초기 위치 지정 포함), P3 +0.5~1일이 늘었다. 부록 E 백로그는 포함하지 않는다. 줄 수·기간은 예상치다.

**공통 검증 도구 실행법** (P0 Task 0.2에서 만듦, 이후 모든 태스크가 씀)

```bash
# 1) 워크트리 빌드 (단계별 브랜치 이름으로 바꿔 쓴다)
cd /home/kkw/localization_ws && colcon build \
  --base-paths src/ros_slam_webui/.worktrees/feature/webui-redesign-p0 \
  --packages-select ros_slam_webui \
  --build-base build_wt/webui-redesign-p0 --install-base install_wt/webui-redesign-p0
# 2) 기존 프로세스 정리 후 서버 기동 (백그라운드)
pkill -9 -f "ros_slam_webui/web_server"; ps aux | grep rosbridge_websocket | grep -v grep | awk '{print $2}' | xargs -r kill -9
bash -c 'source /opt/ros/jazzy/setup.bash && source /home/kkw/localization_ws/install_wt/webui-redesign-p0/setup.bash && ros2 launch ros_slam_webui ros_slam_webui.launch.py' &
# 3) 검사 (워크트리 루트에서)
python3 test/ui/check_smoke.py && python3 test/ui/check_ids.py dev
# 4) 끝나면 다시 정리
pkill -9 -f "ros_slam_webui/web_server"; ps aux | grep rosbridge_websocket | grep -v grep | awk '{print $2}' | xargs -r kill -9
```

이후 태스크의 "검증" Step은 위 1~4를 "표준 검증"이라고 부른다.

---

## 6. P0 태스크 — 토큰·글꼴·컴포넌트 CSS (Phase 8.0)

### Task 0.1: 작업 브랜치·워크트리와 스펙 문서

**Files:**
- Create: `docs/superpowers/specs/2026-10-16-webui-redesign-design.md`

- [ ] **Step 1: 워크트리 생성**

```bash
cd /home/kkw/localization_ws/src/ros_slam_webui
git status --short   # 깨끗한지 확인 (이 계획 파일만 untracked면 정상)
git worktree add .worktrees/feature/webui-redesign-p0 -b feature/webui-redesign-p0 dev
```

- [ ] **Step 2: 스펙 문서 작성** — 이 계획의 부록 A 전체와 "승인된 목업 경로(`docs/design/redesign-mock/`), 승인일 2026-10-16"을 그대로 옮긴다. 내용을 새로 만들지 않는다.
- [ ] **Step 3: 커밋**

```bash
git add docs/superpowers/specs/2026-10-16-webui-redesign-design.md docs/superpowers/plans/2026-10-16-webui-redesign.md
git commit -m "docs: Field Console UI 리디자인 스펙·구현 계획 추가"
```

### Task 0.2: 검증 도구 (Playwright 공용 헬퍼, id 검사, 스모크)

**Files:**
- Create: `test/ui/ui_check_common.py`, `test/ui/check_ids.py`, `test/ui/check_smoke.py`

**Interfaces:**
- Produces: `open_page(width, height) -> (page, errors)` 컨텍스트, `shot(page, name) -> path`, `finish(failures, errors)` (실패·새 JS 에러가 있으면 exit 1), `go_screen(page, subtab_id)` (P1 전에는 옛 탭 버튼, P1 뒤에는 사이드바 클릭).

- [ ] **Step 1: 공용 헬퍼 작성**

```python
"""ros_slam_webui UI 검증 공용 헬퍼 (Playwright, 헤드리스 Chromium).

서버는 미리 띄워 둔다. networkidle은 기다리지 않는다 (WS·폴링 때문에 끝나지 않음).
"""
import os
import sys
from contextlib import contextmanager

from playwright.sync_api import sync_playwright

WEBUI_URL = os.environ.get('WEBUI_URL', 'http://localhost:8080')
SHOT_DIR = os.environ.get('WEBUI_SHOT_DIR', '/tmp/webui_redesign_shots')
IGNORED_ERROR_PATTERNS = ('WebSocket', 'rosbridge', 'ERR_CONNECTION_REFUSED', ':9090', ':8081')

SCREEN_TABS = {
    'lidar-slam-subtab': 'slam-tab', 'localization-subtab': 'slam-tab',
    'multi-session-slam-subtab': 'slam-tab', 'bag-player-subtab': 'player-tab',
    'bag-recorder-subtab': 'player-tab', 'file-player-subtab': 'player-tab',
    'plot-subtab': 'visualization-tab', '3d-viewer-subtab': 'visualization-tab',
}


@contextmanager
def open_page(width=1440, height=900):
    os.makedirs(SHOT_DIR, exist_ok=True)
    errors = []
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        page = browser.new_page(viewport={'width': width, 'height': height})
        page.on('console', lambda m: errors.append(m.text) if m.type == 'error' else None)
        page.on('pageerror', lambda e: errors.append(f'pageerror: {e}'))
        page.goto(WEBUI_URL, wait_until='load')
        page.wait_for_selector('#slam-tab')
        try:
            yield page, errors
        finally:
            browser.close()


def go_screen(page, subtab_id):
    sidebar_item = page.locator(f'.ui-sidebar-item[data-screen="{subtab_id}"]')
    if sidebar_item.count() and sidebar_item.first.is_visible():
        sidebar_item.first.click()
    else:
        page.locator(f'.tab-button[onclick*="{SCREEN_TABS[subtab_id]}"]').click()
        page.locator(f'.subtab-button[onclick*="{subtab_id}"]').click()
    page.wait_for_selector(f'#{subtab_id}.active')


def shot(page, name):
    path = os.path.join(SHOT_DIR, f'{name}.png')
    page.screenshot(path=path)
    return path


def new_js_errors(errors):
    return [e for e in errors if not any(p in e for p in IGNORED_ERROR_PATTERNS)]


def finish(failures, errors):
    js_errors = new_js_errors(errors)
    for e in js_errors:
        print('JS ERROR:', e)
    for f in failures:
        print('FAIL:', f)
    if failures or js_errors:
        sys.exit(1)
    print('PASS')
```

- [ ] **Step 2: id 검사 작성**

```python
"""index.html id 보존 검사: 기준 브랜치 대비 사라진 id와 중복 id를 출력한다.

사용: python3 test/ui/check_ids.py dev [삭제 허용 id ...]
"""
import re
import subprocess
import sys
from collections import Counter

ID_RE = re.compile(r'\bid="([^"]+)"')

base_ref = sys.argv[1] if len(sys.argv) > 1 else 'dev'
allowed = set(sys.argv[2:])
base_html = subprocess.run(['git', 'show', f'{base_ref}:web/index.html'],
                           capture_output=True, text=True, check=True).stdout
with open('web/index.html', encoding='utf-8') as f:
    cur_html = f.read()

removed = sorted(set(ID_RE.findall(base_html)) - set(ID_RE.findall(cur_html)))
dupes = sorted(i for i, n in Counter(ID_RE.findall(cur_html)).items() if n > 1)
bad = [i for i in removed if i not in allowed]
for i in removed:
    print(('ALLOWED ' if i in allowed else 'REMOVED ') + i)
for i in dupes:
    print('DUPLICATE ' + i)
sys.exit(1 if bad or dupes else 0)
```

- [ ] **Step 3: 스모크 작성**

```python
"""8화면을 차례로 열고 스크린샷 + 새 JS 에러 0을 확인한다."""
import sys

from ui_check_common import SCREEN_TABS, finish, go_screen, open_page, shot

prefix = sys.argv[1] if len(sys.argv) > 1 else 'smoke'
failures = []
with open_page() as (page, errors):
    for subtab_id in SCREEN_TABS:
        go_screen(page, subtab_id)
        page.wait_for_timeout(600)
        if not page.locator(f'#{subtab_id}').is_visible():
            failures.append(f'{subtab_id} not visible')
        shot(page, f'{prefix}-{subtab_id}')
    finish(failures, errors)
```

- [ ] **Step 4: 기준선 확보** — `dev` 그대로 표준 검증을 돌려 `python3 test/ui/check_smoke.py baseline`. 기대: PASS, `/tmp/webui_redesign_shots/baseline-*.png` 8장. 새 JS 에러가 이미 있으면 목록을 "기존 에러"로 기록하고 `IGNORED_ERROR_PATTERNS`에 넣지 말고 보고한다.
- [ ] **Step 5: 커밋** — `git add test/ui && git commit -m "test(ui): Playwright 스모크·id 보존 검사 도구 추가"`

### Task 0.3: Fira 글꼴 파일·라이선스·설치 경로 (확정 2026-10-16: 저장소에 포함)

**Files:**
- Create: `web/static/vendor/fonts/FiraSans-Regular.woff2`, `FiraSans-Medium.woff2`, `FiraSans-SemiBold.woff2`, `FiraCode-Regular.woff2`, `FiraCode-Medium.woff2`, `web/static/vendor/fonts/OFL.txt`, `web/static/vendor/fonts/README.md`(출처·버전·라이선스 3줄)
- Verify (수정 없을 것으로 예상): `setup.py` (`_collect_vendor_data_files`, `glob('web/static/*.*')`), `package.xml`

- [ ] **Step 1: 내려받기** — Fira Sans는 `https://github.com/mozilla/Fira` 릴리스, Fira Code는 `https://github.com/tonsky/FiraCode` 릴리스의 woff2를 쓴다. 정확한 다운로드 URL·버전은 실행 시 확인한다(미확인). 두 글꼴 모두 SIL OFL 1.1이다. 원문을 `OFL.txt`로, 출처 URL·버전·저작권 줄을 `README.md`로 같이 넣는다.
- [ ] **Step 2: 설치 경로 확인** — `setup.py`의 `_collect_vendor_data_files()`가 `web/static/vendor/`를 재귀로 `share/ros_slam_webui/web/static/vendor/...`에 설치한다(현재 Three.js·Plotly·roslib이 이 경로로 설치됨). 그래서 **`setup.py` 수정은 필요 없다**. 빌드 후 `ls install_wt/webui-redesign-p0/share/ros_slam_webui/web/static/vendor/fonts/`에 7개 파일이 있어야 한다. 없으면 그때만 `data_files`에 `(os.path.join('share', package_name, 'web/static/vendor/fonts'), glob('web/static/vendor/fonts/*'))`를 추가한다.
- [ ] **Step 3: 서빙 확인** — 서버 기동 후 `curl -sI localhost:8080/static/vendor/fonts/FiraSans-Regular.woff2`가 200이고 `Content-Type`이 `font/woff2` 또는 `application/octet-stream`인지 확인한다. 정적 파일 핸들러(P:8670 `else` 분기)의 MIME 처리는 미확인이다. 브라우저가 잘못된 MIME으로 글꼴을 거부하면 콘솔 경고로 보이므로 `check_theme.py`(Task 0.4)의 `Fira Sans` 확인이 같이 잡는다.
- [ ] **Step 4: 라이선스 표기** — `package.xml`의 `<license>`는 패키지 코드 라이선스라 바꾸지 않는다. 대신 `README.md` "Third-party" 항목(없으면 새로 만듦)에 "Fira Sans / Fira Code — SIL OFL 1.1, `web/static/vendor/fonts/OFL.txt`" 한 줄을 추가한다.
- [ ] **Step 5: 커밋** — `feat(ui): Fira Sans·Fira Code 글꼴 로컬 vendor 추가 (OFL 동봉)`

### Task 0.4: 디자인 토큰과 기존 변수 별칭

**Files:**
- Create: `web/static/ui_theme.css`
- Modify: `web/index.html:10` (link 추가), `web/static/style.css:1` (`@import` 삭제), `style.css:3655-3677` (글꼴 4규칙)

- [ ] **Step 1: 실패 확인 스크립트** — `test/ui/check_theme.py`

```python
"""토큰이 적용됐는지: body 배경·글꼴, 숫자 요소 글꼴."""
from ui_check_common import finish, open_page

failures = []
with open_page() as (page, errors):
    bg = page.evaluate("getComputedStyle(document.body).backgroundColor")
    font = page.evaluate("getComputedStyle(document.body).fontFamily")
    token = page.evaluate("getComputedStyle(document.documentElement).getPropertyValue('--color-bg').trim()")
    if token.lower() != '#0b0e11':
        failures.append(f'--color-bg={token}')
    if bg != 'rgb(11, 14, 17)':
        failures.append(f'body bg={bg}')
    if 'Fira Sans' not in font:
        failures.append(f'body font={font}')
    finish(failures, errors)
```

실행: 표준 검증 후 `python3 test/ui/check_theme.py`. 기대: FAIL(`--color-bg=` 빈 값).

- [ ] **Step 2: `ui_theme.css` 첫 부분 작성** — 부록 A.1의 `@font-face`·`:root` 블록 전체를 그대로 넣는다(토큰 + 기존 변수 별칭 + `body` 배경/글꼴). 별칭 덕분에 `var(--accent)` 등을 쓰는 기존 화면이 한 번에 새 색으로 바뀐다.
- [ ] **Step 3: H에 link 추가** — `<link rel="stylesheet" href="static/style.css">` 다음 줄에 `<link rel="stylesheet" href="static/ui_theme.css">`.
- [ ] **Step 4: C 글꼴 규칙 교체** — 1줄 `@import …Space+Grotesk…` 삭제. 3655, 3664, 3677줄의 `'Space Grotesk', …` 값을 `var(--font-sans)`로, 3674줄 `'JetBrains Mono', …`를 `var(--font-mono)`로 바꾼다(`!important`는 유지). 3666-3675 mono 규칙 선택자 목록 끝에 `, .ui-mono, .ui-kpi-value, .ui-status-badge, .ui-tag` 추가 — `span { … !important }`(3662)보다 클래스가 구체적이라 숫자가 Fira Code로 나온다.
- [ ] **Step 5: 검증** — 표준 검증 + `check_theme.py` PASS + `check_smoke.py p0-tokens`. 스크린샷을 기준선과 비교해 **배치는 같고 색·글꼴만** 바뀌었는지 직접 본다. `.brand-title`의 그라데이션 글자(3678-3686)가 새 accent로 보기 흉하면 그대로 두고 P1에서 사이드바가 대체한다.
- [ ] **Step 6: 커밋** — `git add web/static/ui_theme.css web/index.html web/static/style.css test/ui/check_theme.py && git commit -m "feat(ui): 디자인 토큰 ui_theme.css 추가 및 기존 CSS 변수 별칭 연결"`

롤백: H의 link 한 줄 제거 + C 두 곳 revert.

### Task 0.5: 공용 컴포넌트 클래스

**Files:**
- Modify: `web/static/ui_theme.css` (부록 A.2 컴포넌트 CSS 추가)
- Create: `test/ui/check_components.py`

- [ ] **Step 1: 실패 확인 스크립트** — 샘플 마크업을 페이지에 넣고 계산된 스타일을 본다.

```python
"""ui-* 컴포넌트 클래스 스타일 확인 (샘플 마크업 주입)."""
from ui_check_common import finish, open_page, shot

SAMPLE = '''
<div id="ui-sample" style="position:fixed;top:8px;left:8px;z-index:9999;display:flex;gap:8px;padding:8px;background:#0B0E11">
  <button class="ui-btn ui-btn-go">Start</button>
  <button class="ui-btn ui-btn-stop">Stop</button>
  <button class="ui-btn" disabled>Save Map</button>
  <button class="ui-icon-btn" aria-label="Reset" aria-pressed="true">R</button>
  <span class="ui-status-badge" data-tone="go"><span class="ui-dot"></span>Running</span>
</div>'''

failures = []
with open_page() as (page, errors):
    page.evaluate(f"document.body.insertAdjacentHTML('beforeend', `{SAMPLE}`)")
    go = page.locator('#ui-sample .ui-btn-go')
    if go.evaluate("e => getComputedStyle(e).backgroundColor") != 'rgb(63, 208, 122)':
        failures.append('ui-btn-go bg')
    if go.evaluate("e => getComputedStyle(e).height") != '40px':
        failures.append('ui-btn height')
    if page.locator('#ui-sample .ui-btn[disabled]').evaluate("e => getComputedStyle(e).opacity") != '0.35':
        failures.append('disabled opacity')
    shot(page, 'p0-components')
    finish(failures, errors)
```

기대: FAIL(`ui-btn-go bg`).

- [ ] **Step 2: 부록 A.2 CSS를 `ui_theme.css`에 추가**(버튼·아이콘 버튼·패널·KPI·배지·태그·단계 표시줄·분할 버튼·토글·필드·입력·토스트·다이얼로그·뷰어 오버레이).
- [ ] **Step 3: 검증** — `check_components.py` PASS, 스크린샷 `p0-components.png`를 목업 `01-slam-ready.png` 헤더 버튼과 눈으로 비교. `check_smoke.py` PASS(기존 화면 변화 없음 — 새 클래스는 아직 아무 요소에도 안 붙음).
- [ ] **Step 4: 커밋** — `feat(ui): ui-* 공용 컴포넌트 클래스(버튼·패널·KPI·배지·단계 표시줄 등) 추가`

### P0 완료 처리

- [ ] 메인 체크아웃 `03`에 Phase 8 블록(부록 B.3) 추가, 8.0 체크. finishing 메뉴에서 사용자 선택에 따라 `dev` 통합.

---

## 7. P1 태스크 — 셸 (Phase 8.1)

브랜치: `feature/webui-redesign-p1` (P0가 `dev`에 들어간 뒤 생성).

### Task 1.1: 전역 상태 API `GET /api/system/status_summary`

**Files:**
- Modify: `ros_slam_webui/web_server.py` (메서드 추가: `get_localization_state` P:4011 근처 / 분기 추가: P:8560 뒤)
- Create: `test/test_status_summary.py`
- Modify (메인 체크아웃): `.cursor/rules/02.SW_architecture_rules.mdc` §6

**Interfaces:**
- Produces: `WebGUINode.get_status_summary(self) -> dict` (4.2절 JSON).

- [ ] **Step 1: 속성 존재 확인** — `rg -n "self\.(slam_map_saving|slam_opt_running|player_data_loaded|player_is_ros2_bag|player_is_ros1_bag|save_bag_saving|kitti_converter_running|helipr_converter_running)\s*=" ros_slam_webui/web_server.py`. 모두 `__init__`에 있어야 한다. 없으면 이 태스크를 멈추고 보고.
- [ ] **Step 2: 실패 테스트**

```python
"""get_status_summary: 메모리 속성만 읽어 모듈별 실행 상태를 돌려준다."""
from types import SimpleNamespace

import pytest

web_server = pytest.importorskip('ros_slam_webui.web_server')


class _Proc:
    def __init__(self, alive):
        self._alive = alive

    def poll(self):
        return None if self._alive else 0


class _Ros1Thread:
    def __init__(self, status):
        self._status = status

    def is_alive(self):
        return True

    def get_status(self):
        return {'status': self._status}


def _node(**over):
    base = dict(
        slam_process=None, slam_map_saving=False, localization_process=None, slam_opt_running=False,
        ros1_player_thread=None, bag_path='', bag_playing=False, bag_paused=False,
        recorder_recording=False, recorder_mode='ros2',
        player_data_loaded=False, player_playing=False, player_paused=False,
        player_is_ros2_bag=False, player_is_ros1_bag=False,
        kitti_converter_running=False, kaist_converter_running=False,
        mulran_converter_running=False, helipr_converter_running=False, save_bag_saving=False)
    base.update(over)
    return SimpleNamespace(**base)


def _summary(**over):
    return web_server.WebGUINode.get_status_summary(_node(**over))


def test_idle():
    r = _summary()
    assert r['success'] is True
    assert r['data']['slam'] == {'running': False, 'saving_map': False}
    assert r['data']['bag_player'] == {'loaded': False, 'playing': False, 'paused': False}


def test_slam_running_and_saving():
    r = _summary(slam_process=_Proc(True), slam_map_saving=True)
    assert r['data']['slam'] == {'running': True, 'saving_map': True}


def test_dead_process_is_not_running():
    assert _summary(localization_process=_Proc(False))['data']['localization']['running'] is False


def test_ros1_bag_playing_counts_as_playing():
    r = _summary(bag_path='/x.bag', ros1_player_thread=_Ros1Thread('playing'))
    assert r['data']['bag_player'] == {'loaded': True, 'playing': True, 'paused': False}


def test_file_player_converting():
    assert _summary(mulran_converter_running=True)['data']['file_player']['converting'] is True
```

실행: `cd <워크트리> && source /opt/ros/jazzy/setup.bash && python3 -m pytest test/test_status_summary.py -v`. 기대: `AttributeError: … get_status_summary`. 모듈 import 자체가 실패하면(전역 부작용, 미확인) 원인을 보고하고, 필요하면 `get_status_summary`의 계산부를 모듈 함수 `build_status_summary(node)`로 빼서 그것만 테스트한다.

- [ ] **Step 3: 구현** (`get_localization_state` 뒤)

```python
    def get_status_summary(self):
        """사이드바 실행 표시용 전역 상태 요약. 메모리 속성만 읽는다 (DDS 조회 없음)."""
        ros1_status = 'stopped'
        if self.ros1_player_thread and self.ros1_player_thread.is_alive():
            ros1_status = self.ros1_player_thread.get_status().get('status', 'stopped')
        bag_playing = bool(self.bag_playing) or ros1_status == 'playing'
        bag_paused = bool(self.bag_paused) or ros1_status == 'paused'
        fp_bag_mode = bool(self.player_is_ros2_bag or self.player_is_ros1_bag)
        converting = any((self.kitti_converter_running, self.kaist_converter_running,
                          self.mulran_converter_running, self.helipr_converter_running))
        return {
            'success': True,
            'data': {
                'slam': {'running': self._is_alive(self.slam_process),
                         'saving_map': bool(self.slam_map_saving)},
                'localization': {'running': self._is_alive(self.localization_process)},
                'optimization': {'running': bool(self.slam_opt_running)},
                'bag_player': {'loaded': bool(self.bag_path), 'playing': bag_playing, 'paused': bag_paused},
                'recorder': {'recording': bool(self.recorder_recording), 'mode': self.recorder_mode},
                'file_player': {
                    'loaded': bool(self.player_data_loaded),
                    'playing': bool(self.player_playing) or (fp_bag_mode and bag_playing),
                    'paused': bool(self.player_paused) or (fp_bag_mode and bag_paused),
                    'converting': converting,
                    'saving_bag': bool(self.save_bag_saving),
                },
            },
        }

    @staticmethod
    def _is_alive(process):
        return bool(process) and process.poll() is None
```

`_is_alive`는 staticmethod라 테스트의 `SimpleNamespace`에는 없다 → 테스트 `_node()`에 `_is_alive=web_server.WebGUINode._is_alive`를 추가한다(Step 2 코드에 반영해 둘 것).

- [ ] **Step 4: 라우트** — P:8560(`/api/system/info` 분기 끝) 다음에:

```python
            elif parsed_path.path == '/api/system/status_summary':
                self.send_json_response(self.node.get_status_summary())
```

- [ ] **Step 5: 검증** — pytest PASS, `python3 -m py_compile ros_slam_webui/web_server.py`, 빌드, 서버 기동 후 `curl -s localhost:8080/api/system/status_summary` 결과가 4.2절 모양인지.
- [ ] **Step 6: 문서** — 메인 체크아웃 `02` §6.3 끝(SLAM Result Viewer 블록 뒤, P:589 부근 문서 줄)에 "### 6.3.1 시스템 API" 소절과 4.4절 줄을 Edit로 추가.
- [ ] **Step 7: 커밋** — `feat(api): 사이드바 실행 표시용 GET /api/system/status_summary 추가` (`.cursor/`는 커밋 대상 아님)

### Task 1.2: 순수 상태 로직 모듈 (사이드바 표시·경과 시간)

**Files:**
- Create: `web/static/ui_status_logic.js`, `test/ui/check_status_logic.js`

**Interfaces:**
- Produces: `UiStatusLogic.deriveSidebarMarkers(data) -> { [subtabId]: { text, tone } }`, `UiStatusLogic.formatElapsed(sec) -> 'm:ss' | 'h:mm:ss'`. 이후 태스크가 같은 파일에 `deriveSlamView` 등을 추가한다.

- [ ] **Step 1: 실패 테스트**

```javascript
const assert = require('assert');
const L = require('../../web/static/ui_status_logic.js');

const idle = {
    slam: { running: false, saving_map: false }, localization: { running: false },
    optimization: { running: false }, bag_player: { loaded: false, playing: false, paused: false },
    recorder: { recording: false, mode: 'ros2' },
    file_player: { loaded: false, playing: false, paused: false, converting: false, saving_bag: false },
};
const withPatch = (patch) => JSON.parse(JSON.stringify(Object.assign({}, idle, patch)));

assert.deepStrictEqual(L.deriveSidebarMarkers(idle), {});
assert.deepStrictEqual(L.deriveSidebarMarkers(withPatch({ slam: { running: true, saving_map: false } }))['lidar-slam-subtab'],
    { text: 'RUN', tone: 'go' });
assert.strictEqual(L.deriveSidebarMarkers(withPatch({ slam: { running: true, saving_map: true } }))['lidar-slam-subtab'].text, 'SAVE');
assert.deepStrictEqual(L.deriveSidebarMarkers(withPatch({ recorder: { recording: true, mode: 'ros2_mcap' } }))['bag-recorder-subtab'],
    { text: 'REC', tone: 'stop' });
assert.strictEqual(L.deriveSidebarMarkers(withPatch({ optimization: { running: true } }))['multi-session-slam-subtab'].text, 'OPT');
assert.strictEqual(L.deriveSidebarMarkers(withPatch({ bag_player: { loaded: true, playing: true, paused: true } }))['bag-player-subtab'].text, 'PAUSE');
assert.strictEqual(L.formatElapsed(65), '1:05');
assert.strictEqual(L.formatElapsed(3725), '1:02:05');
console.log('PASS check_status_logic');
```

실행: `node test/ui/check_status_logic.js` → 기대: `Cannot find module`.

- [ ] **Step 2: 구현**

```javascript
// ui_status_logic.js — DOM 없는 상태 파생 함수 (브라우저: window.UiStatusLogic, Node: module.exports)
(function (root) {
    'use strict';

    function formatElapsed(totalSec) {
        const sec = Math.max(0, Math.floor(totalSec || 0));
        const h = Math.floor(sec / 3600);
        const m = Math.floor((sec % 3600) / 60);
        const s = String(sec % 60).padStart(2, '0');
        return h > 0 ? `${h}:${String(m).padStart(2, '0')}:${s}` : `${m}:${s}`;
    }

    function playMarker(playing, paused) {
        if (paused) return { text: 'PAUSE', tone: 'warn' };
        if (playing) return { text: 'PLAY', tone: 'go' };
        return null;
    }

    function deriveSidebarMarkers(d) {
        const out = {};
        const put = (id, marker) => { if (marker) out[id] = marker; };
        put('lidar-slam-subtab', d.slam.saving_map ? { text: 'SAVE', tone: 'info' }
            : (d.slam.running ? { text: 'RUN', tone: 'go' } : null));
        put('localization-subtab', d.localization.running ? { text: 'RUN', tone: 'go' } : null);
        put('multi-session-slam-subtab', d.optimization.running ? { text: 'OPT', tone: 'info' } : null);
        put('bag-player-subtab', playMarker(d.bag_player.playing, d.bag_player.paused));
        put('bag-recorder-subtab', d.recorder.recording ? { text: 'REC', tone: 'stop' } : null);
        const fp = d.file_player;
        put('file-player-subtab', (fp.converting || fp.saving_bag) ? { text: 'SAVE', tone: 'info' }
            : playMarker(fp.playing, fp.paused));
        return out;
    }

    const api = { formatElapsed, deriveSidebarMarkers };
    if (typeof module !== 'undefined' && module.exports) {
        module.exports = api;
    } else {
        root.UiStatusLogic = api;
    }
})(typeof window !== 'undefined' ? window : globalThis);
```

- [ ] **Step 3: 검증** — `node test/ui/check_status_logic.js` PASS, `node --check web/static/ui_status_logic.js`.
- [ ] **Step 4: 커밋** — `feat(ui): 상태 파생 순수 함수 ui_status_logic.js 추가 (사이드바 표시)`

### Task 1.3: UI 헬퍼 `ui_kit.js` (배지·단계 표시줄·잠금·라벨·토스트)

**Files:**
- Create: `web/static/ui_kit.js`
- Modify: `web/index.html` (스크립트 태그: `webui_ports.js` 다음에 `ui_kit.js`, `ui_status_logic.js`)

**Interfaces:**
- Produces (전역 `window.uiKit`):
  - `setStatusBadge(el, tone, text)` — `el.dataset.tone = tone`, `.ui-status-text` 글자 교체
  - `renderStepper(olEl, steps: string[], current: number)` — 완료(✓)·현재(`aria-current="step"`)·대기
  - `setEnabled(el, enabled: boolean, reasonText?)` — `disabled` + `title`
  - `setBtnLabel(btn, text)` — `.ui-btn-label`이 있으면 그 글자만, 없으면 `textContent`
  - `showToast(message, tone = 'info', { timeoutMs } = {})` — `stop` 톤은 자동으로 안 닫힘, 최대 4개
  - `setReadonly(container, readonly: boolean)` — 안의 input/select/textarea `disabled`, 컨테이너 `data-readonly`

- [ ] **Step 1: 실패 확인 스크립트** `test/ui/check_ui_kit.py` — `page.evaluate`로 `uiKit.showToast('저장 완료','go')` 호출 후 `.ui-toast` 1개, `uiKit.showToast('실패','stop')` 후 6초 기다려도 2개 남는지, `renderStepper`에 4단계·current 2를 넣고 `[aria-current="step"]` 글자가 3번째인지 확인. 기대: FAIL(`uiKit is not defined`).
- [ ] **Step 2: 구현** — 아래 코드.

```javascript
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
```

- [ ] **Step 3: 검증** — `node --check web/static/ui_kit.js`, 빌드, `check_ui_kit.py` PASS, `check_smoke.py` PASS.
- [ ] **Step 4: 커밋** — `feat(ui): ui_kit.js 헬퍼(상태 배지·단계 표시줄·버튼 잠금·토스트) 추가`

### Task 1.4: `openTab`/`openSubTab` 전역 `event` 의존 제거

**Files:**
- Modify: `web/static/script.js:131-213`

- [ ] **Step 1: 실패 확인** `test/ui/check_tabs_programmatic.py` — `page.evaluate("openTab('player-tab')")` 후 `#bag-player-subtab.active`가 보이는지, `pageerror`가 없는지. 기대: FAIL(`event is not defined` 또는 `Cannot read properties of undefined`).
- [ ] **Step 2: 수정** — J:143-144의 `event.target.classList.add('active');`를 다음으로 바꾼다.

```javascript
    // Activate corresponding button (event 없이도 동작 — 사이드바·프로그램 호출 대응)
    const tabButton = Array.from(buttons).find(btn =>
        (btn.getAttribute('onclick') || '').includes(`'${tabId}'`)
    );
    if (tabButton) tabButton.classList.add('active');
```

J:179의 `if (!skipEvent && event && event.target)`를 `if (!skipEvent && typeof event !== 'undefined' && event && event.target)`로 바꾼다. `openSubTab` 끝(J:213 `}` 바로 앞)에 `if (window.uiShell) uiShell.onScreenChanged(subtabId);` 한 줄 추가.

- [ ] **Step 3: 검증** — `node --check web/static/script.js`, 빌드, `check_tabs_programmatic.py` PASS, `check_smoke.py` PASS(옛 탭 클릭 경로).
- [ ] **Step 4: 커밋** — `fix(ui): openTab/openSubTab 전역 event 의존 제거 (프로그램 호출 지원)`

### Task 1.5: 사이드바와 앱 셸 레이아웃, `showScreen()`

**Files:**
- Modify: `web/index.html` (`<body>` 바로 안: 아이콘 스프라이트 + 셸 래퍼, 끝: `ui_shell.js` 태그)
- Create: `web/static/ui_shell.js`
- Modify: `web/static/ui_theme.css` (부록 A.3 셸 레이아웃)

**Interfaces:**
- Consumes: `openTab`, `openSubTab`, `lastActiveSubtab`(J), `uiKit`, `UiStatusLogic`.
- Produces (전역 `window.uiShell`, 전역 함수 `showScreen`): `showScreen(subtabId)`, `uiShell.onScreenChanged(subtabId)`, `uiShell.refresh()`(이후 태스크가 채움), `uiShell.SCREENS`.

- [ ] **Step 1: 실패 확인** `test/ui/check_shell.py`

```python
"""사이드바 8항목 클릭 → 해당 서브탭 활성 + aria-current, 옛 탭 숨김."""
from ui_check_common import SCREEN_TABS, finish, open_page, shot

failures = []
with open_page() as (page, errors):
    if page.locator('.ui-sidebar-item').count() != 8:
        failures.append('sidebar item count')
    if page.locator('.tab-navigation').is_visible():
        failures.append('old tab nav visible')
    for subtab_id in SCREEN_TABS:
        page.locator(f'.ui-sidebar-item[data-screen="{subtab_id}"]').click()
        page.wait_for_selector(f'#{subtab_id}.active')
        cur = page.locator('.ui-sidebar-item[aria-current="page"]').get_attribute('data-screen')
        if cur != subtab_id:
            failures.append(f'aria-current {cur} != {subtab_id}')
    shot(page, 'p1-shell')
    finish(failures, errors)
```

기대: FAIL(`sidebar item count`).

- [ ] **Step 2: HTML** — `<body>` 직후, 기존 `<div class="page-shell">`를 `<main class="ui-main">`로 감싸고 그 앞에 사이드바를 둔다. 아이콘은 `<svg class="ui-icon"><use href="#icon-…"></use></svg>`. 스프라이트 `<svg id="ui-icon-sprite" hidden>`에 `icon-radar, icon-crosshair, icon-layers, icon-play-circle, icon-record, icon-folder, icon-chart, icon-box` 8개 `<symbol viewBox="0 0 24 24">`를 넣는다(lucide 아이콘 path를 그대로 복사, ISC 라이선스 — 출처 주석 1줄).

```html
<body>
    <svg id="ui-icon-sprite" hidden><!-- lucide icons (ISC): symbol 8개 --></svg>
    <div class="ui-app-shell">
        <aside class="ui-sidebar" id="ui-sidebar">
            <div class="ui-sidebar-brand">
                <span class="ui-brand-mark" aria-hidden="true">RS</span>
                <div><p class="ui-brand-title">ROS SLAM WEBUI</p><p class="ui-brand-sub">Field Console</p></div>
            </div>
            <nav class="ui-sidebar-nav" aria-label="화면">
                <p class="ui-sidebar-group-title">SLAM / Localization</p>
                <ul>
                    <li><button type="button" class="ui-sidebar-item" data-screen="lidar-slam-subtab"
                            onclick="showScreen('lidar-slam-subtab')">
                        <svg class="ui-icon" aria-hidden="true"><use href="#icon-radar"></use></svg>
                        <span class="ui-sidebar-label">LiDAR SLAM</span>
                        <span class="ui-run-tag" data-run-tag hidden></span>
                    </button></li>
                    <!-- localization-subtab(icon-crosshair), multi-session-slam-subtab(icon-layers) 같은 형식 -->
                </ul>
                <p class="ui-sidebar-group-title">Data Player / Recorder</p>
                <ul><!-- bag-player-subtab(icon-play-circle), bag-recorder-subtab(icon-record), file-player-subtab(icon-folder) --></ul>
                <p class="ui-sidebar-group-title">Visualization</p>
                <ul><!-- plot-subtab(icon-chart), 3d-viewer-subtab(icon-box) --></ul>
            </nav>
            <div class="ui-conn-block" id="ui-conn-block"><!-- Task 1.6 --></div>
        </aside>
        <main class="ui-main">
            <div class="page-shell"> … 기존 내용 그대로 … </div>
        </main>
    </div>
```

주석으로 줄인 7개 항목은 실제 파일에서 위 LiDAR SLAM `<li>`와 같은 마크업으로 `data-screen`·`onclick`·아이콘·라벨만 바꿔 모두 적는다. 라벨: Localization / Multi-Session SLAM / Bag Player / Bag Recorder / File Player / Plot / 3D Viewer.

- [ ] **Step 3: `ui_shell.js` 첫 버전**

```javascript
// ui_shell.js — 사이드바·화면 헤더·전역 상태 표시 (script.js 다음에 로드)
const uiShell = (() => {
    const SCREENS = {
        'lidar-slam-subtab':         { tab: 'slam-tab', group: 'SLAM / Localization', title: 'LiDAR SLAM' },
        'localization-subtab':       { tab: 'slam-tab', group: 'SLAM / Localization', title: 'Localization' },
        'multi-session-slam-subtab': { tab: 'slam-tab', group: 'SLAM / Localization', title: 'Multi-Session SLAM' },
        'bag-player-subtab':   { tab: 'player-tab', group: 'Data Player / Recorder', title: 'Bag Player' },
        'bag-recorder-subtab': { tab: 'player-tab', group: 'Data Player / Recorder', title: 'Bag Recorder' },
        'file-player-subtab':  { tab: 'player-tab', group: 'Data Player / Recorder', title: 'File Player' },
        'plot-subtab':         { tab: 'visualization-tab', group: 'Visualization', title: 'Plot' },
        '3d-viewer-subtab':    { tab: 'visualization-tab', group: 'Visualization', title: '3D Viewer' },
    };
    const LEGACY_NAV_KEY = 'webui.legacyNav';

    function isLegacyNav() {
        try { return localStorage.getItem(LEGACY_NAV_KEY) === '1'; } catch (e) { return false; }
    }

    function onScreenChanged(subtabId) {
        document.querySelectorAll('.ui-sidebar-item').forEach(item => {
            if (item.dataset.screen === subtabId) item.setAttribute('aria-current', 'page');
            else item.removeAttribute('aria-current');
        });
        refresh();
    }

    function showScreen(subtabId) {
        const screen = SCREENS[subtabId];
        if (!screen) return;
        lastActiveSubtab[screen.tab] = subtabId;
        openTab(screen.tab);
    }

    function refresh() {
        // Task 1.8 이후 화면별 헤더 렌더를 여기서 부른다
    }

    function init() {
        if (isLegacyNav()) return;
        document.body.classList.add('ui-has-sidebar');
        const active = document.querySelector('.subtab-content.active');
        onScreenChanged(active ? active.id : 'lidar-slam-subtab');
    }

    return { SCREENS, showScreen, onScreenChanged, refresh, init };
})();
window.uiShell = uiShell;
window.showScreen = uiShell.showScreen;
document.addEventListener('DOMContentLoaded', uiShell.init);
```

`init`은 초기 HTML에 서브탭 3개가 동시에 `active`인 점(H:52, 833, 1162)을 그대로 둔다. `.tab-content.active` 안의 첫 `.subtab-content.active`를 고르도록 `document.querySelector('.tab-content.active .subtab-content.active')`로 쓴다(위 코드의 선택자를 이것으로 교체해 적용).

- [ ] **Step 4: CSS** — 부록 A.3의 셸 레이아웃 블록을 `ui_theme.css`에 추가(`.ui-has-sidebar .tab-navigation, .ui-has-sidebar .subtab-navigation, .ui-has-sidebar .topbar { display: none; }` 포함). `ui-has-sidebar`가 없으면 `.ui-sidebar { display: none; }` — 되돌리기 스위치.
- [ ] **Step 5: 검증** — `node --check web/static/ui_shell.js`, 빌드, `check_shell.py` PASS, `check_ids.py dev` PASS, `check_smoke.py p1-shell`(헬퍼가 사이드바 경로 사용). 브라우저 콘솔에서 `localStorage.setItem('webui.legacyNav','1')` 후 새로고침하면 옛 탭이 보이는지 수동 확인. 3D Viewer 진입 후 캔버스가 0px이 아닌지 스크린샷으로 확인(지연 초기화, 리스크 4).
- [ ] **Step 6: 커밋** — `feat(ui): 왼쪽 사이드바 셸과 showScreen 추가 (옛 탭은 숨김, legacyNav 스위치)`

### Task 1.6: 연결 블록 (rosbridge·지연·DOMAIN·포트·PC2 WS)

**Files:**
- Modify: `web/index.html` (`#ui-conn-block` 내용), `web/static/ui_shell.js`, `web/static/script.js` (훅 5줄)

**Interfaces:**
- Produces: `uiShell.onRosbridgeState('connected'|'disconnected'|'reconnecting')`, `uiShell.onLatency(ms|null)`, `uiShell.onDomainId(id)`, `uiShell.onBackendWsState('connected'|'disconnected')`.

- [ ] **Step 1: 실패 확인** `check_shell.py`에 추가: `#ui-conn-rosbridge [data-tone]` 존재, `#ui-conn-port-web` 글자가 `8080`, 4초 기다린 뒤 `#ui-conn-latency` 글자가 `ms`로 끝남. 기대: FAIL.
- [ ] **Step 2: HTML**

```html
<div class="ui-conn-block" id="ui-conn-block">
    <div class="ui-conn-head">
        <span class="ui-conn-state" id="ui-conn-rosbridge" data-tone="idle">
            <span class="ui-dot" aria-hidden="true"></span><span class="ui-status-text">rosbridge 확인 중…</span>
        </span>
        <span class="ui-mono" id="ui-conn-latency">-- ms</span>
    </div>
    <dl class="ui-conn-list">
        <dt>DOMAIN</dt><dd id="ui-conn-domain">-</dd>
        <dt>Web</dt><dd id="ui-conn-port-web">-</dd>
        <dt>PC2 WS</dt><dd id="ui-conn-port-pc2">-</dd>
        <dt>rosbridge</dt><dd id="ui-conn-port-rosbridge">-</dd>
    </dl>
</div>
```

- [ ] **Step 3: `ui_shell.js`에 추가**

```javascript
    const ROSBRIDGE_TEXT = {
        connected: ['go', 'rosbridge 연결됨'],
        disconnected: ['stop', 'rosbridge 오류'],
        reconnecting: ['warn', 'rosbridge 재연결 중…'],
    };

    function onRosbridgeState(state) {
        const conf = ROSBRIDGE_TEXT[state];
        if (conf) uiKit.setStatusBadge(document.getElementById('ui-conn-rosbridge'), conf[0], conf[1]);
    }

    function onLatency(ms) {
        const el = document.getElementById('ui-conn-latency');
        if (!el) return;
        el.textContent = (ms == null || !isFinite(ms)) ? 'N/A' : `${Math.round(ms)} ms`;
        el.dataset.tone = (ms == null) ? 'idle' : (ms < 50 ? 'go' : (ms < 150 ? 'warn' : 'stop'));
    }

    function onDomainId(id) {
        const el = document.getElementById('ui-conn-domain');
        if (el) el.textContent = String(id);
    }

    function onBackendWsState(state) {
        const el = document.getElementById('ui-conn-port-pc2');
        const ports = window.ROS_SLAM_WEBUI || {};
        if (el) el.textContent = `${ports.pc2WsPort || '-'}${state === 'connected' ? '' : ' (끊김)'}`;
    }

    function renderPorts() {
        const ports = window.ROS_SLAM_WEBUI || {};
        const set = (id, v) => { const el = document.getElementById(id); if (el) el.textContent = v ?? '-'; };
        set('ui-conn-port-web', ports.webPort);
        set('ui-conn-port-pc2', ports.pc2WsPort);
        set('ui-conn-port-rosbridge', ports.rosbridgePort);
    }
```

`init()`에 `renderPorts(); window.addEventListener('ros-slam-webui-ports-ready', renderPorts);` 추가, 반환 객체에 네 훅 추가.

- [ ] **Step 4: J 훅** (각 한 줄)
  - `updateRosbridgeStatusChip` 끝(J:4638 `chip.textContent = …` 다음): `if (window.uiShell) uiShell.onRosbridgeState(state);` — 함수 첫 줄 `if (!chip) return;`보다 **앞**에 두어 칩이 없어도 호출되게 한다.
  - latency worker `onmessage`(J:4100) 안 `if (type !== 'latency') return;` 다음: `if (window.uiShell) uiShell.onLatency(ms);`
  - `updateRosDomainId`(J:4141) 성공 분기: `if (window.uiShell) uiShell.onDomainId(result.domain_id);`
  - `_initBackendWs` `onopen`(J:4267)·`onclose`(J:4285) 첫 줄: `if (window.uiShell) uiShell.onBackendWsState('connected');` / `('disconnected')`
- [ ] **Step 5: 검증** — `node --check` 2파일, 빌드, `check_shell.py` PASS. rosbridge를 끄고(`kill`) 3초 뒤 연결 블록이 "재연결 중…"으로 바뀌는지 스크린샷.
- [ ] **Step 6: 커밋** — `feat(ui): 사이드바 연결 블록(rosbridge·지연·DOMAIN·포트·PC2 WS 상태) 추가`

### Task 1.7: 전역 상태 폴링 → 사이드바 실행 표시

**Files:**
- Modify: `web/static/ui_shell.js`

**Interfaces:**
- Consumes: `GET /api/system/status_summary`(Task 1.1), `UiStatusLogic.deriveSidebarMarkers`(Task 1.2).
- Produces: `uiShell.lastSummary`(마지막 `data` 또는 `null`) — P2·P3가 읽음.

- [ ] **Step 1: 실패 확인** `test/ui/check_sidebar_markers.py` — `page.route('**/api/system/status_summary', …)`로 `recorder.recording=true` 응답을 돌려준 뒤 3초 안에 `[data-screen="bag-recorder-subtab"] [data-run-tag]`가 보이고 글자가 `REC`인지. 기대: FAIL.
- [ ] **Step 2: 구현**

```javascript
    const SUMMARY_POLL_MS = 2000;
    let lastSummary = null;

    function renderSidebarMarkers(data) {
        const markers = UiStatusLogic.deriveSidebarMarkers(data);
        document.querySelectorAll('.ui-sidebar-item').forEach(item => {
            const tag = item.querySelector('[data-run-tag]');
            const marker = markers[item.dataset.screen];
            tag.hidden = !marker;
            tag.textContent = marker ? marker.text : '';
            tag.dataset.tone = marker ? marker.tone : 'idle';
        });
    }

    async function pollSummary() {
        const result = await apiCall('/api/system/status_summary');
        if (result && result.success && result.data) {
            lastSummary = result.data;
            renderSidebarMarkers(lastSummary);
            refresh();
        }
    }
```

`init()` 끝에 `pollSummary(); setInterval(pollSummary, SUMMARY_POLL_MS);`. 반환 객체에 `get lastSummary() { return lastSummary; }`.

- [ ] **Step 3: 검증** — 빌드, `check_sidebar_markers.py` PASS, `check_smoke.py` PASS.
- [ ] **Step 4: 커밋** — `feat(ui): 전역 상태 2초 폴링으로 사이드바 RUN/REC/PLAY/OPT 표시`

### Task 1.8: 화면 헤더 뼈대 (8화면)

**Files:**
- Modify: `web/index.html` (각 `.subtab-content` 첫 자식으로 헤더 삽입), `web/static/ui_theme.css`(A.2의 `.ui-screen-header`), `web/static/ui_shell.js`

**Interfaces:**
- Produces: 각 화면 `#<screen>-header-badge`(예: `lidar-slam-header-badge`), `#<screen>-header-actions`, `#<screen>-stepper`(SLAM·Loc·Multi만). `<screen>` 접두사: `lidar-slam`, `localization`, `multi-session`, `bag-player`, `bag-recorder`, `file-player`, `plot`, `viewer3d`.

- [ ] **Step 1: 실패 확인** `check_shell.py`에 추가: 각 화면 진입 후 `.subtab-content.active .ui-screen-header h1` 글자가 `SCREENS[id].title`과 같고 `.tab-content h2`가 안 보이는지. 기대: FAIL.
- [ ] **Step 2: HTML** — 예: LiDAR SLAM(H:52 `<div id="lidar-slam-subtab" …>` 바로 다음 줄).

```html
<header class="ui-screen-header" data-screen-header>
    <div class="ui-screen-heading">
        <p class="ui-screen-group">SLAM / Localization</p>
        <div class="ui-screen-title-row">
            <h1 class="ui-screen-title">LiDAR SLAM</h1>
            <span class="ui-status-badge" id="lidar-slam-header-badge" data-tone="idle" role="status">
                <span class="ui-dot" aria-hidden="true"></span><span class="ui-status-text">Ready</span>
            </span>
        </div>
    </div>
    <div class="ui-screen-actions" id="lidar-slam-header-actions"></div>
    <ol class="ui-stepper" id="lidar-slam-stepper" aria-label="진행 단계"></ol>
</header>
```

8화면 모두 같은 형식으로 넣는다. Plot·3D·Player·Recorder·File은 `<ol class="ui-stepper">`를 넣지 않는다. 기존 `<h2>`는 지우지 않고 CSS `.ui-has-sidebar .subtab-content > .section > h2 { display: none; }`로 숨긴다(P6 삭제).

- [ ] **Step 3: 기본 렌더** — `refresh()`가 보이는 화면의 헤더 배지를 `lastSummary`로 대충 채운다(P2·P3가 화면별 정밀 렌더로 교체): 해당 화면 표시가 있으면 `go`/`stop` 톤 + "Running"/"Recording"/"Playing", 없으면 `idle` + "Ready".
- [ ] **Step 4: 검증** — 빌드, `check_shell.py` PASS, `check_ids.py dev` PASS(새 id만 추가), 8화면 스크린샷을 목업 헤더와 비교.
- [ ] **Step 5: 커밋** — `feat(ui): 8개 화면 sticky 헤더 뼈대(그룹·제목·상태 배지) 추가`

### P1 완료 처리

- [ ] 메인 체크아웃 `02` §2.2를 부록 B.4 문구로 Edit, `03` 8.1 체크. finishing 메뉴.

---

## 8. P2 태스크 — LiDAR SLAM / Localization (Phase 8.2)

브랜치: `feature/webui-redesign-p2`.

### Task 2.1: SLAM·Localization 파생 함수

**Files:**
- Modify: `web/static/ui_status_logic.js`, `test/ui/check_status_logic.js`

**Interfaces:**
- Produces: `SLAM_STEPS = ['Config','Mapping','Save Map','Result']`, `LOC_STEPS = ['Config','Initialize','Tracking']`, `deriveSlamView({isRunning, starting, stopping, saving, justSaved, resultVisible, elapsed, kfCount, stale, imuHz, lidHz}) -> {tone, text, step, buttons:{start, stop, saveMap}}`, `deriveLocView({isRunning, starting, stopping, initialized, external, stale, lastStartFailed}) -> {tone, text, step, buttons:{start, stop}}`. `imuHz`·`lidHz`가 `null`이면(아직 메시지 없음) 센서 경고를 내지 않는다.

- [ ] **Step 1: 실패 테스트 추가** (3.1·3.2절 표를 그대로 단언)

```javascript
const base = { isRunning: false, starting: false, stopping: false, saving: false, justSaved: false, resultVisible: false,
    elapsed: '', kfCount: 3, stale: false, imuHz: 200, lidHz: 10 };
const v = (p) => L.deriveSlamView(Object.assign({}, base, p));
assert.deepStrictEqual(v({}), { tone: 'idle', text: 'Ready', step: 0, buttons: { start: true, stop: false, saveMap: false } });
assert.deepStrictEqual(v({ isRunning: true, elapsed: '00:00:06' }).buttons, { start: false, stop: true, saveMap: true });
assert.strictEqual(v({ isRunning: true, kfCount: 0 }).buttons.saveMap, false);
assert.strictEqual(v({ isRunning: true, stale: true }).text, 'Running · 데이터 없음');
assert.strictEqual(v({ isRunning: true, stale: true }).buttons.saveMap, false);
assert.strictEqual(v({ isRunning: true, imuHz: 0 }).text, 'Running · IMU 끊김');
assert.strictEqual(v({ isRunning: true, imuHz: null, lidHz: null }).tone, 'go');
assert.strictEqual(v({ isRunning: true, elapsed: '00:00:06' }).text, 'Running · 00:00:06');
assert.strictEqual(v({ isRunning: true, saving: true }).step, 2);
assert.deepStrictEqual(v({ isRunning: true, saving: true }).buttons, { start: false, stop: false, saveMap: false });
assert.strictEqual(v({ isRunning: true, justSaved: true }).step, 3);
assert.strictEqual(v({ stopping: true, isRunning: true }).tone, 'warn');
assert.strictEqual(v({ resultVisible: true }).step, 3);
const lb = { isRunning: false, starting: false, stopping: false, initialized: false, external: false,
    stale: false, lastStartFailed: false };
const lv = (p) => L.deriveLocView(Object.assign({}, lb, p));
assert.strictEqual(lv({ lastStartFailed: true }).tone, 'stop');
assert.strictEqual(lv({ isRunning: true, stale: true }).text, 'Running · 데이터 없음');
assert.strictEqual(lv({ isRunning: true }).text, 'Running · Not Init');
assert.strictEqual(lv({ isRunning: true, initialized: true }).step, 2);
assert.deepStrictEqual(lv({ isRunning: true, external: true }).buttons, { start: false, stop: false });
assert.strictEqual(lv({ starting: true }).text, 'Starting…');
```

실행 `node test/ui/check_status_logic.js` → FAIL(`deriveSlamView is not a function`).

- [ ] **Step 2: 구현** (IIFE 안, `api`에 추가)

```javascript
    const SLAM_STEPS = ['Config', 'Mapping', 'Save Map', 'Result'];
    const LOC_STEPS = ['Config', 'Initialize', 'Tracking'];
    const view = (tone, text, step, buttons) => ({ tone, text, step, buttons });

    function deriveSlamView(s) {
        const locked = { start: false, stop: false, saveMap: false };
        if (s.stopping) return view('warn', 'Stopping…', 1, locked);
        if (s.starting) return view('info', 'Starting…', 1, locked);
        if (s.saving) return view('info', 'Saving Map…', 2, locked);
        const canSave = s.kfCount > 0;
        if (s.isRunning && s.stale) return view('warn', 'Running · 데이터 없음', 1, { start: false, stop: true, saveMap: false });
        if (s.isRunning && s.imuHz === 0) return view('warn', 'Running · IMU 끊김', 1, { start: false, stop: true, saveMap: canSave });
        if (s.isRunning && s.lidHz === 0) return view('warn', 'Running · LiDAR 끊김', 1, { start: false, stop: true, saveMap: canSave });
        if (s.isRunning && s.justSaved) {
            return view('info', 'Map Saved · SLAM 계속 실행 중', 3, { start: false, stop: true, saveMap: canSave });
        }
        if (s.isRunning) {
            const text = s.elapsed ? `Running · ${s.elapsed}` : 'Running';
            return view('go', text, 1, { start: false, stop: true, saveMap: canSave });
        }
        if (s.resultVisible) return view('idle', 'Ready · 저장된 지도 표시 중', 3, { start: true, stop: false, saveMap: false });
        return view('idle', 'Ready', 0, { start: true, stop: false, saveMap: false });
    }

    function deriveLocView(s) {
        const locked = { start: false, stop: false };
        if (s.stopping) return view('warn', 'Stopping…', 1, locked);
        if (s.starting) return view('info', 'Starting…', 1, locked);
        if (s.isRunning && s.external) return view('warn', 'Running (외부 실행)', s.initialized ? 2 : 1, locked);
        if (s.isRunning && s.stale) return view('warn', 'Running · 데이터 없음', 1, { start: false, stop: true });
        if (s.isRunning && s.initialized) return view('go', 'Initialized · Tracking', 2, { start: false, stop: true });
        if (s.isRunning) return view('warn', 'Running · Not Init', 1, { start: false, stop: true });
        if (s.lastStartFailed) return view('stop', '시작 실패 (로그 보기)', 0, { start: true, stop: false });
        return view('idle', 'Ready', 0, { start: true, stop: false });
    }
```

- [ ] **Step 3: 검증** — Node PASS, `node --check`.
- [ ] **Step 4: 커밋** — `feat(ui): SLAM·Localization 상태 파생 함수 추가`

### Task 2.2: LiDAR SLAM 헤더 액션·단계 표시줄·렌더 훅

**Files:**
- Modify: `web/index.html:56-62` (버튼 이동), `web/static/style.css:3768-3814` (start/stop 속성 선택자 규칙 중 `startSlamMapping`·`stopSlamMapping` 항목 삭제), `web/static/ui_shell.js`, `web/static/script.js` (훅)

- [ ] **Step 1: 실패 확인** `test/ui/check_slam_loc.py` — LiDAR SLAM 진입 후: `#lidar-slam-header-actions #slam-start-button` 존재·활성, `#slam-stop-button`·`#slam-save-map-button` 비활성, `#lidar-slam-stepper [aria-current="step"]` 글자 `Config`, `#slam-start-button` 배경 `rgb(63, 208, 122)`. 기대: FAIL.
- [ ] **Step 2: HTML 이동** — H:56-62의 `.bag-control-buttons`에서 `#slam-save-map-button`, `#slam-stop-button`, `#slam-start-button`을 이 순서로 `#lidar-slam-header-actions`로 옮긴다. 클래스: Save Map `ui-btn`, Stop `ui-btn ui-btn-stop`, Start `ui-btn ui-btn-go`. 각 버튼 안을 `<svg class="ui-icon" aria-hidden="true"><use href="#icon-…"></use></svg><span class="ui-btn-label">Start SLAM</span>` 형식으로(스프라이트에 `icon-play`, `icon-square`, `icon-save` 추가). Save Map과 Stop 사이에 `<span class="ui-divider" aria-hidden="true"></span>`. Config Load·Set Config File은 Task 2.3에서 Config 패널로 옮긴다.
- [ ] **Step 3: CSS 정리** — C:3768-3814에서 `button[onclick*="startSlamMapping"]`·`button[onclick*="stopSlamMapping"]`(+`:hover`) 선택자만 목록에서 지운다(Localization 것은 Task 2.8에서). `#slam-start-button`/`#slam-stop-button` id 색 규칙도 같은 범위에서 찾아 지운다(`rg -n "#slam-(start|stop)-button" web/static/style.css`).
- [ ] **Step 4: 렌더** — `ui_shell.js`에 추가하고 `refresh()`에서 보이는 화면이 `lidar-slam-subtab`이면 호출.

```javascript
    let lastSlamState = null;

    function onSlamState(state) {
        lastSlamState = state;
        refresh();
    }

    function renderSlam() {
        const resultEl = document.getElementById('savemap-result-viewer');
        const liveEl = document.getElementById('slam-live-viewer');
        const runTime = document.getElementById('analytics-run-time');
        const v = UiStatusLogic.deriveSlamView({
            isRunning: Boolean(lastSlamState && lastSlamState.is_running),
            starting: Boolean(window._slamLiveViewerHoldOpen),
            stopping: Boolean(window._slamStopping),
            saving: Boolean(window._slamSaving),
            justSaved: Boolean(window._slamMapJustSaved),
            resultVisible: Boolean(resultEl && resultEl.style.display !== 'none'),
            elapsed: runTime && /\d/.test(runTime.textContent) ? runTime.textContent.trim() : '',
            kfCount: readNumber(SLAM_KF_COUNT_ID),
            stale: isAnalyticsStale('slam'),
            imuHz: readNumber('analytics-imu-hz'),
            lidHz: readNumber('analytics-lid-hz'),
        });
        uiKit.setStatusBadge(document.getElementById('lidar-slam-header-badge'), v.tone, v.text);
        uiKit.renderStepper(document.getElementById('lidar-slam-stepper'), UiStatusLogic.SLAM_STEPS, v.step);
        uiKit.setEnabled(document.getElementById('slam-start-button'), v.buttons.start);
        uiKit.setEnabled(document.getElementById('slam-stop-button'), v.buttons.stop);
        uiKit.setEnabled(document.getElementById('slam-save-map-button'), v.buttons.saveMap, 'SLAM 실행 중에만 저장할 수 있습니다');
        const empty = document.getElementById('lidar-slam-viewer-empty');
        if (empty) empty.hidden = (liveEl && liveEl.style.display !== 'none') || (resultEl && resultEl.style.display !== 'none');
    }
```

(`lidar-slam-viewer-empty`는 Task 2.3에서 생긴다. 없으면 건너뜀.)

같은 파일에 두는 보조 함수. `SLAM_KF_COUNT_ID`는 Full Statistics의 키프레임 수 값 요소 id다. `rg -n "kf_count|ad-kf" web/index.html web/static/script.js`로 실제 id를 확인해 상수에 넣는다(`ad-kf_count`로 추정, 미확인). 대시보드가 아직 메시지를 못 받았으면 `null`이 돌아와 Save Map은 잠긴다.

```javascript
    const SLAM_KF_COUNT_ID = 'ad-kf_count';
    const ANALYTICS_STALE_MS = 3000;
    const lastAnalyticsAt = { slam: 0, loc: 0 };

    function readNumber(id) {
        const el = document.getElementById(id);
        if (!el) return null;
        const n = parseFloat(el.textContent);
        return Number.isFinite(n) ? n : null;
    }

    function noteAnalytics(kind) { lastAnalyticsAt[kind] = performance.now(); }

    function isAnalyticsStale(kind) {
        return lastAnalyticsAt[kind] > 0 && performance.now() - lastAnalyticsAt[kind] > ANALYTICS_STALE_MS;
    }
```

`noteAnalytics`는 반환 객체에 넣는다. 첫 메시지 전(`0`)에는 stale로 보지 않는다. Start 직후 "데이터 없음"이 바로 뜨지 않게 하려는 것이다. 메시지가 끝내 안 오는 경우는 Localization의 `lastStartFailed`(Task 2.11)가 맡는다.

- [ ] **Step 5: J 훅** — `updateSlamState()`의 `if (state) {` 블록 끝(J:724 `}` 앞): `if (window.uiShell) uiShell.onSlamState(state);`. 다음 함수의 플래그 변경 직후에 `if (window.uiShell) uiShell.refresh();`: `startSlamMapping`(J:3382 뒤, J:3395 `finally` 안), `stopSlamMapping`(J:3413 뒤, J:3434 뒤), `confirmSaveMap`(J:3190 뒤), `_pollSaveMapStatus` done 분기(J:3316 뒤), `cancelSaveMap`(J:3346 뒤), `closeSaveMapModal`(J:3160 뒤).
- [ ] **Step 6: 검증** — `node --check` 2파일, 빌드, `check_slam_loc.py` PASS, `check_ids.py dev` PASS. 데이터 없이 Start를 눌러 배지가 Starting… → (실패 시) Ready로 돌아오는지 스크린샷.
- [ ] **Step 7: 커밋** — `feat(slam): LiDAR SLAM 헤더 액션·상태 배지·단계 표시줄과 버튼 잠금`

### Task 2.3: LiDAR SLAM 8/4 배치·빈 지도 칸·저장 오버레이·Config 버튼 이동

**Files:**
- Modify: `web/index.html:63-400` (구조 이동), `web/static/ui_theme.css`

- [ ] **Step 1: 실패 확인** `check_slam_loc.py`에 추가: `.ui-screen-grid .ui-col-main #lidar-slam-viewer-empty` 보임(SLAM 미실행), `.ui-col-side #slam-config-container` 존재, `#slam-config-container .ui-btn[onclick*="loadSlamConfig"]` 존재, 1440px에서 `.ui-col-main` 너비가 `.ui-col-side`의 약 2배(±15%). 기대: FAIL.
- [ ] **Step 2: 마크업 이동** — `.section.card` 안을 다음 구조로 재배치한다. 모든 기존 요소는 id 그대로 **이동만** 한다.

```html
<div class="ui-screen-grid">
    <div class="ui-col-main">
        <div class="ui-viewer-empty" id="lidar-slam-viewer-empty">
            <p class="ui-viewer-empty-eyebrow">LIVE VIEWER</p>
            <p class="ui-viewer-empty-title">Start SLAM을 누르면 실시간 지도가 여기에 그려집니다</p>
            <p class="ui-viewer-empty-body">Configuration 패널에서 값을 확인하고 시작하세요. rosbridge 구독은 Start 이후에만 연결됩니다.</p>
        </div>
        <!-- #slam-live-viewer (H:79-150) 이동 -->
        <!-- #savemap-result-viewer (H:335-392) 이동 -->
        <!-- #slam-save-map-status-area (H:326-333) 이동: class에 ui-viewer-overlay 추가 -->
    </div>
    <aside class="ui-col-side">
        <!-- #slam-analytics-dashboard (H:153-324) 이동 -->
        <!-- #slam-config-container (H:65-76) 이동 -->
    </aside>
</div>
<div class="status" hidden><!-- #lidar-slam-status 라벨 유지 (P6 삭제) --></div>
```

빈 칸의 Start 버튼은 목업에 있지만 **넣지 않는다**(같은 동작 버튼이 두 개면 잠금 동기화가 늘어남). 대신 헤더 Start를 가리키는 글자만 둔다.

- [ ] **Step 3: Config 버튼** — H:57-58의 Config Load·Set Config File을 `#slam-config-container` 맨 아래 `<div class="ui-panel-footer">`로 옮기고 `ui-btn`을 붙인다. Set Config File 아래 `<p class="ui-hint">mapping_config.yaml에 적용됩니다</p>`(S1). 빈 `.bag-control-buttons`는 지운다(id 없음).
- [ ] **Step 4: 저장 오버레이** — `#slam-save-map-status-area`에 `ui-viewer-overlay` 클래스 추가, 안에 `<div class="ui-progress ui-progress-indeterminate" aria-label="저장 진행 중"></div>`를 `#slam-save-map-msg` 다음에 넣는다. 그 아래 새 `<p class="ui-mono" id="slam-save-map-elapsed">0:00</p>`(경과 시간 — `ui_shell.js`가 `saving`인 동안 1초마다 `formatElapsed`로 갱신)와 `<p class="ui-hint">저장 중에는 새 키프레임이 그래프에 추가되지 않습니다. 로봇을 멈추고 기다리세요.</p>`. `#slam-save-map-cancel-btn`의 글자를 "기다리기 중단"으로 바꾸고 `title`에 "노드의 저장은 끝까지 진행됩니다"(확정 2026-10-16, 진짜 취소는 부록 E 백로그). PGO가 단계별 진행을 웹 서버로 보내지 않아(printf → `DEVNULL`) 진행률 %는 만들 수 없다. 성공·오류 시 진행 바 숨김은 CSS: `.status-banner.success .ui-progress, .status-banner.error .ui-progress { display: none; }`.
- [ ] **Step 5: CSS** — 부록 A.3의 `.ui-screen-grid`, `.ui-col-main`, `.ui-col-side`, `.ui-viewer-empty`, `.ui-viewer-overlay`, `.ui-progress` 블록. 뷰어 칸 높이는 `min-height: 420px; height: calc(100vh - 220px)`.
- [ ] **Step 6: 검증** — 빌드, `check_slam_loc.py` PASS, `check_ids.py dev` PASS, 스크린샷을 목업 `01-slam-ready.png`와 비교. 풀스크린 버튼이 여전히 뷰어 컨테이너 안에 있는지(`#slam-live-canvas-container` 자식) 확인.
- [ ] **Step 7: 커밋** — `feat(slam): LiDAR SLAM 지도 8/정보 4 배치·빈 지도 칸·저장 오버레이 적용`

### Task 2.4: SLAM Analytics 오른쪽 칸 재배치·스파크라인·차트 리사이즈

**Files:**
- Modify: `web/index.html:153-324`, `web/static/script.js` (`SlamAnalyticsDashboard` J:11158-11784), `web/static/ui_kit.js` (`renderSparkline`), `web/static/ui_theme.css`

- [ ] **Step 1: 실패 확인** — `check_slam_loc.py`에 `page.evaluate("slamAnalyticsDashboard.show()")` 후: `#slam-analytics-dashboard .ui-kpi` 4개 이상, `#slam-analytics-detail` 접힘, `#analytics-chart-proctime .main-svg` 너비 ≤ `.ui-col-side` 너비. 기대: FAIL.
- [ ] **Step 2: 마크업** — 네 카드(Hz·궤적·CPU·RAM)를 `.ui-kpi-grid`(2열) 안 `.ui-kpi` 6개(IMU Hz, LiDAR Hz, CPU, RAM, Distance, Avg Speed)로 재구성. **값 요소 id는 그대로** 쓴다(`analytics-imu-hz`, `analytics-lid-hz`, `analytics-cpu-percent`, `analytics-ram-value`, `analytics-traj-dist`, `analytics-avg-speed`). 나머지 보조 요소(`analytics-run-time`, `analytics-cpu-cores-equiv`, `analytics-cpu-bar-fill`, `analytics-ram-sub`, `analytics-ram-bar-fill`)는 각 KPI의 `.ui-kpi-sub`에 넣어 유지. IMU/LiDAR/CPU KPI에 `<svg class="ui-sparkline" id="analytics-spark-imu|lid|cpu" viewBox="0 0 100 30" preserveAspectRatio="none" aria-hidden="true"></svg>`. Processing Time·DOP 차트와 누적 표는 그 아래 순서대로, Full Statistics 토글 글자에 "27 항목"(UI가 보여 주는 수. 메시지 필드는 47개 — 13.1절).

라벨은 실제 계산에 맞춘다(`laserMapping.cpp` 근거, 13.1절):
- CPU KPI 라벨 "CPU (fast_lio)" — 시스템 전체가 아니라 이 프로세스 CPU 시간 ÷ 코어 수. 첫 값은 0.
- RAM KPI 라벨 "RAM (fast_lio)" — 프로세스 VmRSS MB.
- IMU/LiDAR Hz 툴팁 "1초 벽시계 기준. bag을 1×가 아닌 속도로 재생하면 그만큼 달라집니다."
- Distance 툴팁 "속도 0.2 m/s 이상일 때만 누적". Run time은 첫 스캔부터의 벽시계 시간.
- Processing Time의 `total`은 세 단계 합이 아니라 한 주기 전체 시간이다. 범례 글자 "Total (한 주기)".
- DOP 차트는 `dop_flag`가 꺼져 있으면 0만 나온다. 값이 계속 0이면 차트 위에 "DOP 계산 꺼짐 (dop_flag)" 글자.

`SlamAnalyticsDashboard`의 메시지 처리 시작 부분(J:11260 rAF 처리 직전)에 `if (window.uiShell) uiShell.noteAnalytics('slam');` 한 줄을 넣는다(Task 2.2의 stale 판정용).
- [ ] **Step 3: `uiKit.renderSparkline(svgEl, values, max)`** — `polyline` 하나를 만들거나 갱신(점 60개, `vector-effect: non-scaling-stroke`). 값 0개면 비움.
- [ ] **Step 4: 대시보드 연결** — `SlamAnalyticsDashboard`의 Hz 갱신(J:11510)·CPU 갱신(J:11565)에서 `this._spark = this._spark || { imu: [], lid: [], cpu: [] }`에 값을 넣고 60개 넘으면 `shift()`, `uiKit.renderSparkline(...)` 호출. `_resetWidgets`(J:11735)에서 `this._spark = { imu: [], lid: [], cpu: [] }`.
- [ ] **Step 5: 차트 리사이즈** — 생성자에서 `new ResizeObserver(() => { ['analytics-chart-proctime','analytics-chart-dop'].forEach(id => { const el = document.getElementById(id); if (el && el.data) Plotly.Plots.resize(el); }); }).observe(document.getElementById('slam-analytics-dashboard'));`
- [ ] **Step 6: 검증** — `node --check`, 빌드, `check_slam_loc.py` PASS, 1280px·1440px 스크린샷에서 차트가 칸을 넘지 않는지.
- [ ] **Step 7: 커밋** — `feat(slam): Analytics를 오른쪽 칸 KPI·스파크라인 배치로 재구성, 차트 자동 리사이즈`

### Task 2.5: 실행 중 Config 읽기 전용 (SLAM·Localization)

**Files:**
- Modify: `web/static/script.js:3372-3377, 3423-3428, 3520-3525, 3542-3547`, `web/index.html` (Config 패널 머리에 자물쇠 아이콘·안내 글자)

- [ ] **Step 1: 실패 확인** — `check_slam_loc.py`: `page.route('**/api/slam/start_mapping', …)`로 성공 응답을 주고 Start 클릭 → `#slam-config-container`가 **보이고** 안의 input이 모두 `disabled`. 기대: FAIL(패널이 숨음).
- [ ] **Step 2: 수정** — 네 곳의 `_cfgEl.style.display = 'none'` / `_cfgEl.style.display = ''`를 각각 `uiKit.setReadonly(_cfgEl, true)` / `uiKit.setReadonly(_cfgEl, false)`로 바꾼다. `_slamConfigVisibleBeforeStart`·`_locConfigVisibleBeforeStart` 대입 줄은 남겨 둔다(다른 곳 참조 0인지 `rg`로 확인 후 P6에서 정리). 새로고침으로 실행 중 화면에 들어온 경우를 위해 `renderSlam`/`renderLoc`에서 `isRunning`이면 `setReadonly(…, true)`도 호출.
- [ ] **Step 3: HTML** — Config 패널 머리에 `<svg class="ui-icon ui-readonly-only" aria-hidden="true"><use href="#icon-lock"></use></svg>`와 `<p class="ui-hint ui-readonly-only">실행 중에는 읽기 전용입니다. 바뀐 값은 다음 Start부터 적용됩니다.</p>`. CSS: `[data-readonly="false"] .ui-readonly-only { display: none; }`.
- [ ] **Step 4: 검증** — 빌드, `check_slam_loc.py` PASS. Stop 뒤 입력이 다시 열리는지.
- [ ] **Step 5: 커밋** — `feat(slam): 실행 중 Config 패널을 숨기지 않고 읽기 전용으로 표시`

### Task 2.6: Save Map 결과 뷰어 재스타일 + ResizeObserver

**Files:**
- Modify: `web/index.html:335-392`, `web/static/script.js` (`SlamResultViewer._init` J:9228 부근, `toggleSaveMapLayer` J:10961), `web/static/ui_theme.css`

- [ ] **Step 1: 실패 확인** — `check_slam_loc.py`: `#savemap-viewer-actions button`마다 `aria-label`이 비어 있지 않음, `.slam-legend-row`에 `aria-pressed` 존재. 기대: FAIL.
- [ ] **Step 2: 툴바** — 5개 버튼에 `ui-icon-btn` 클래스, 기존 `title` 값을 `aria-label`에도 복사. 토글형(Top View, EDL)은 `_syncControlStates`가 `active` 클래스를 바꾸므로 CSS에서 `.ui-icon-btn.active`를 눌린 모양으로(A.2). `aria-pressed`는 Step 3 함수가 맞춘다.
- [ ] **Step 3: 범례 칩** — `.slam-legend-row`를 `button` 모양 CSS로(점 + 이름, 꺼지면 흐림). JS: `toggleSaveMapLayer`/`toggleSlamLayer`와 `_syncControlStates` 끝에 `root.querySelectorAll('.slam-legend-row, .slam-action-btn').forEach(el => el.setAttribute('aria-pressed', String(el.dataset.active !== 'false' && !el.classList.contains('inactive'))))` 같은 동기화 한 줄을 넣는다. `data-active`/`inactive` 중 실제 쓰는 표시는 J:10961에서 확인해 그 하나만 쓴다.
- [ ] **Step 4: ResizeObserver** — `_init()`의 렌더러 생성 직후:

```javascript
        this._resizeObserver = new ResizeObserver(() => {
            if (this._visible !== false) this._resizeRenderer();
        });
        this._resizeObserver.observe(document.getElementById(this._ids.container));
```

메서드 이름(`_resizeRenderer`, `_visible`)은 `SlamLiveViewer`(J:7950) 기준이다. `SlamResultViewer`에서 크기 맞춤 메서드 이름이 다르면(미확인) `rg -n "setSize\(" web/static/script.js`로 J:9148-10826 안의 것을 찾아 그 메서드를 호출한다.

- [ ] **Step 5: Height Clip·Point Size** — CSS만: 트랙 `--color-line2`, 최대 핸들 `--color-info`, 최소 핸들 `--color-warn`, 라벨 Fira Code.
- [ ] **Step 6: 검증** — `node --check`, 빌드, `check_slam_loc.py` PASS. 실데이터: 저장된 지도 폴더가 있으면 `page.evaluate("saveMapResultViewer.show({directory: '<경로>'})")`로 열어 스크린샷(경로는 사용자에게 받음, 없으면 사용자 확인 목록으로).
- [ ] **Step 7: 커밋** — `feat(slam): Save Map 결과 뷰어 툴바·범례 칩 재스타일, 크기 자동 맞춤 추가`

### Task 2.7: Live 뷰어 툴바 접근성과 작은 버그 3개

**Files:**
- Modify: `web/index.html:99-146, 439-486`, `web/static/script.js` (J:6858, J:8169 `show()`, J:7625·J:8968 Reset, J:11048-11090 풀스크린 핸들러)

- [ ] **Step 1: 실패 확인** — `check_slam_loc.py`: `slamLiveViewer.show()` 호출 직후 `#slam-live-follow-btn`에 `active` 클래스(Follow 기본 ON과 일치). 기대: FAIL.
- [ ] **Step 2: 수정**
  - 두 툴바 버튼에 `ui-icon-btn` + `aria-label`(title 복사).
  - `show()` 두 곳 끝에 `document.getElementById('<slam-live|loc>-follow-btn')?.classList.toggle('active', this._followMode);` (없는 요소는 건너뜀 — 지금처럼 `if (el)` 형태로 쓴다. 옵셔널 체이닝 대입은 쓰지 않는다).
  - Reset 두 곳: Top View 버튼 `active` 제거 줄 추가.
  - 풀스크린 핸들러에 `slam-live-canvas-container` 분기를 다른 분기와 같은 형식으로 추가해 `slam-live-fullscreen-icon-expand/collapse`를 바꾼다.
- [ ] **Step 3: 검증** — `node --check`, 빌드, `check_slam_loc.py` PASS. 풀스크린 아이콘 전환은 헤드리스에서 확인이 어려우면 사용자 확인 목록에 넣는다.
- [ ] **Step 4: 커밋** — `fix(slam): Live 뷰어 Follow 초기 표시·풀스크린 아이콘·Reset 버튼 상태 동기화`

### Task 2.8: Localization 화면 (헤더·깜빡임 방지·배치·지도 경로·KPI)

**Files:**
- Modify: `web/index.html:405-702`, `web/static/script.js:3475-3566, 12189-12198, 12244`, `web/static/style.css` (`startLocalization`·`stopLocalization` 속성 선택자 삭제), `web/static/ui_shell.js`

- [ ] **Step 1: 실패 확인** — `check_slam_loc.py`에 Localization 시나리오:
  1. 화면 진입 → `#localization-stepper [aria-current]` = `Config`, `#localization-start-button` 활성.
  2. `page.route`로 `/api/localization/state`가 처음엔 `{"is_running": true}`를 1.5초 지연해 돌려주게 하고, 그 사이 `stopLocalizationMapping()`을 부른다 → 2초 뒤 `#localization-live-viewer`가 숨은 상태여야 함(늦은 응답 무시).
  3. `#localization-map-path` 글자에 `.pcd` 포함(기본 config 로드 후).
  기대: 2·3 FAIL.
- [ ] **Step 2: 깜빡임 방지 수정** (3.2절)

```javascript
// updateLocalizationState() 처음
    const reqAt = performance.now();
    const state = await apiCall('/api/localization/state');
    if (reqAt < (window._locStoppedAt || 0)) return;   // Stop 전에 보낸 요청의 늦은 응답은 버림
```

복원 분기(J:3500)의 `if (state.is_running) {`를 `if (state.is_running && !window._locStopping && performance.now() - (window._locStoppedAt || 0) > LOC_RESTORE_HOLDOFF_MS) {`로 바꾸고 파일 상단 상수 영역에 `const LOC_RESTORE_HOLDOFF_MS = 3000;  // DDS 퍼블리셔 수 반영 지연 동안 재출현 방지 (실측 필요)`. `startLocalizationMapping`은 처음에 `window._locStarting = true;`, API 호출을 `try { … } finally { window._locStarting = false; if (window.uiShell) uiShell.refresh(); }`로 감싼다. `stopLocalizationMapping`은 처음에 `window._locStopping = true;`, `locLiveViewer.hide()` 다음 줄에 `window._locStoppedAt = performance.now();`, API 응답 뒤 `window._locStopping = false;` + `uiShell.refresh()`. 함수 끝 `if (state) {` 블록 끝에 `if (window.uiShell) uiShell.onLocState(state);`.

- [ ] **Step 3: 배지 초기화** — `LocAnalyticsDashboard._resetWidgets()`(J:12244)에 두 배지(`loc-header-init-badge`, `loc-init-status-dot`)의 글자를 `— Not Init`, 클래스 `ok`·`bad` 제거.
- [ ] **Step 4: 헤더·배치** — Task 2.2·2.3과 같은 방식: Start/Stop을 `#localization-header-actions`로(`ui-btn-go`/`ui-btn-stop`, 아이콘+라벨), Config 버튼은 Config 패널 아래로, `.ui-screen-grid`에 `#localization-viewer-empty`(새) + `#localization-live-viewer` / 오른쪽에 `#loc-analytics-dashboard` + `#localization-config-container`. 헤더 액션 왼쪽에 `<span class="ui-hint">사용 지도 <span class="ui-mono" id="localization-map-path">-</span></span>`. C의 `startLocalization`·`stopLocalization` 속성 선택자 삭제.
- [ ] **Step 5: 렌더** — `ui_shell.js`

```javascript
    let lastLocState = null;
    function onLocState(state) { lastLocState = state; refresh(); }

    function renderLoc() {
        const isRunning = Boolean(lastLocState && lastLocState.is_running);
        const badge = document.getElementById('loc-header-init-badge');
        const summaryLoc = lastSummary ? lastSummary.localization.running : null;
        const v = UiStatusLogic.deriveLocView({
            isRunning,
            starting: Boolean(window._locStarting),
            stopping: Boolean(window._locStopping),
            initialized: Boolean(badge && badge.classList.contains('ok')),
            external: isRunning && summaryLoc === false,
            stale: isAnalyticsStale('loc'),
            lastStartFailed: locStartFailed,
        });
        uiKit.setStatusBadge(document.getElementById('localization-header-badge'), v.tone, v.text);
        uiKit.renderStepper(document.getElementById('localization-stepper'), UiStatusLogic.LOC_STEPS, v.step);
        uiKit.setEnabled(document.getElementById('localization-start-button'), v.buttons.start);
        uiKit.setEnabled(document.getElementById('localization-stop-button'), v.buttons.stop,
            v.text.includes('외부') ? '웹 UI가 시작하지 않은 Localization입니다' : '');
        const mapInput = document.querySelector('#localization-config-parameters [data-config-key$="map_file_path"]');
        const mapPath = document.getElementById('localization-map-path');
        // map_file_path는 FAST_LIO 소스 폴더 기준 상대 경로 (laserLocalization.cpp:1544)
        if (mapPath) mapPath.textContent = mapInput ? `FAST_LIO/${mapInput.value}` : '-';
        const live = document.getElementById('localization-live-viewer');
        const empty = document.getElementById('localization-viewer-empty');
        if (empty) empty.hidden = Boolean(live && live.style.display !== 'none');
    }
```

`initialized`는 `/loc_analytics` 메시지가 올 때만 바뀌므로, `LocAnalyticsDashboard`의 배지 갱신(J:12189-12198) 끝에도 `if (window.uiShell) uiShell.refresh();`를 넣는다. 같은 메시지 처리 시작 부분에 `if (window.uiShell) uiShell.noteAnalytics('loc');`. `ui_shell.js` 모듈 안에 `let locStartFailed = false;`를 이 태스크에서 선언한다. 이 값을 true/false로 바꾸는 감시 로직은 Task 2.11이 넣는다.

"Running · Not Init"이 10초 넘게 이어지면 헤더 배지 옆에 `<span class="ui-hint" id="localization-init-hint">지도 원점 근처에서 시작하거나 초기 위치를 지정하세요</span>`를 보인다(노드는 원점에서 스스로 초기화를 시도, 13.3절). 10초는 임의 값이다.

- [ ] **Step 6: 대시보드 재배치** — Task 2.4와 같은 KPI 그리드: Matching score(새 `#loc-analytics-match-ratio`, `msg.match_ratio`를 `toFixed(2)`, 0.5 미만이면 warn 톤 + 글자 "낮음". 0.5 기준은 임의 값 — 실데이터로 조정), LiDAR Hz(`loc-analytics-lid-hz`), Wheel Odometry(새 `#loc-analytics-wheel-velo`, `msg.linear_velo` m/s — **단, Config의 `odom_mode`가 0이면 노드가 이 값을 채우지 않으므로**(`laserLocalization.cpp:880-882`) 값 대신 "휠 오도메트리 꺼짐 (odom_mode 0)"), CPU(`loc-analytics-cpu-percent`, 라벨 "CPU (fast_lio)"). 러그 범례 툴팁 "dop_ratio < 0.6인 프레임에서 지도 갱신"(코드 기준, `LocAnalytics.msg` 주석의 "> 1.4"는 낡음). 기존 IMU Hz·RAM·거리·속도는 KPI 아래 작은 줄로 유지. 그다음 Map Update 러그(제목 "MAP UPDATE · 최근 200 프레임", 범례 "■ 업데이트됨 ■ 건너뜀" — 색+글자), Processing Time·DOP·누적 표·Full Statistics("31 항목"). 러그 캔버스는 `ResizeObserver`로 `_initRug()` 다시 호출. Processing Time·DOP `extendTraces`에 SLAM과 같은 점 상한(400) 인자 추가.
- [ ] **Step 7: 검증** — `node --check`, 빌드, `check_slam_loc.py` PASS, `check_ids.py dev` PASS, 스크린샷을 목업 `05-localization.png`와 비교.
- [ ] **Step 8: 커밋** — `feat(slam): Localization 헤더·단계 표시줄·8/4 배치, Stop 직후 뷰어 재출현 방지`

### Task 2.9: Three.js 뷰어 배경 색 통일

**Files:**
- Modify: `web/static/script.js:6659, 7937, 9244`, `web/static/threejs_display.js:462`, `web/index.html:1274`(인라인 `#2a2a2a`)

- [ ] **Step 1: 수정** — J 상수 영역에 `const VIEWER_BG_COLOR = 0x07090c;  // ui_theme.css --color-viewer` 추가. 세 곳의 `new THREE.Color(0x0a0a18)`/`0x1a1a2e`를 `new THREE.Color(VIEWER_BG_COLOR)`로. T:462는 `0x07090c` 직접(T는 J 상수를 쓰지 않음). H:1274 인라인 배경을 `var(--color-viewer)`로.
- [ ] **Step 2: 검증** — 빌드, 스모크 스크린샷에서 네 뷰어 배경이 같은지. **실데이터(사용자)**: 흰 현재 스캔·흰 PGO 선이 더 잘 보이는지, z-rainbow 맵 대비가 괜찮은지 확인 요청.
- [ ] **Step 3: 커밋** — `style(viewer): Three.js 뷰어 배경을 디자인 토큰 #07090C로 통일`

### Task 2.10: 백엔드 — Save Map 이름 검증·존재 확인, 프로세스 로그 API

**Files:**
- Modify: `ros_slam_webui/web_server.py` (`save_slam_map` P:3540 부근, `/api/slam/save_map` 분기 P:8705, GET 분기 추가)
- Create: `test/test_save_name_and_log.py`
- Modify (메인 체크아웃): `02` §6

**Interfaces:**
- Produces: 모듈 함수 `validate_map_dir_name(name: str) -> str | None`(문제가 없으면 `None`, 있으면 사용자용 문구), `tail_text_file(path: str, max_lines: int) -> list[str]`. `GET /api/slam/save_map_check?name=…` → `{"success": true, "data": {"valid": bool, "reason": str, "exists": bool}}`. `GET /api/system/process_log?name=slam|localization&lines=50` → `{"success": true, "data": {"lines": [...]}}`. `POST /api/slam/save_map`은 이름이 규칙에 어긋나면 `{"success": false, "error": "<문구>"}`.

- [ ] **Step 1: 실패 테스트**

```python
import pytest

web_server = pytest.importorskip('ros_slam_webui.web_server')


@pytest.mark.parametrize('name', ['parking05', 'b2_run-3', 'A' * 64])
def test_valid_names(name):
    assert web_server.validate_map_dir_name(name) is None


@pytest.mark.parametrize('name', ['', 'Map', '..', 'a/b', 'a b', 'x;rm', 'A' * 65, '../etc'])
def test_invalid_names(name):
    assert web_server.validate_map_dir_name(name)


def test_tail_text_file(tmp_path):
    log = tmp_path / 'x.log'
    log.write_text('\n'.join(f'line{i}' for i in range(100)))
    assert web_server.tail_text_file(str(log), 3) == ['line97', 'line98', 'line99']
    assert web_server.tail_text_file(str(tmp_path / 'none.log'), 3) == []
```

- [ ] **Step 2: 구현**

```python
MAP_DIR_NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,64}$')
RESERVED_MAP_DIR_NAMES = {'Map'}  # pose_graph_optimization 라이브 스테이징 폴더 (시작 시 rm -r 됨)
PROCESS_LOG_PATHS = {
    'slam': '/tmp/web_gui_slam_stderr.log',
    'localization': '/tmp/web_gui_localization_stderr.log',
}


def validate_map_dir_name(name):
    """지도 폴더 이름 검사. 문제 없으면 None, 있으면 사용자용 문구."""
    if not MAP_DIR_NAME_RE.match(name or ''):
        return '영문·숫자·_·- 만 쓸 수 있습니다 (1~64자)'
    if name in RESERVED_MAP_DIR_NAMES:
        return f'"{name}"은(는) 예약된 이름입니다'
    return None


def tail_text_file(path, max_lines):
    """파일 마지막 max_lines줄. 없으면 빈 목록."""
    try:
        with open(path, 'r', errors='replace') as f:
            return [line.rstrip('\n') for line in f.readlines()[-max_lines:]]
    except OSError:
        return []
```

`re`가 이미 import돼 있는지 확인한다. `/tmp/web_gui_*_stderr.log` 경로는 P:3501·3664에서 같은 문자열인지 확인해 상수로 바꿔 쓴다. `save_map_check`의 `exists`는 `_find_sibling_package_dir('pose_graph_optimization')`(P:3990 패턴) 아래 `<name>/` 존재 여부. `process_log`의 `name`은 사전 키만 허용, `lines`는 1~200으로 자른다. `save_slam_map` 처음에 `validate_map_dir_name`을 호출해 실패면 바로 반환.

- [ ] **Step 3: 프론트** — Save Map 모달(`#save-map-directory`)에 `oninput` 300ms 디바운스로 `save_map_check` 호출: `valid=false`면 확인 버튼 잠금 + 입력 아래 빨간 글자 `reason`. `confirmSaveMap`(J:3172) 처음에 `exists`이면 `await uiKit.confirmDialog({ title: '같은 이름의 지도가 있습니다', message: '덮어쓰면 이전 지도가 지워집니다.', okText: '덮어쓰기' })`. 모달 안내 글자 "저장 위치: pose_graph_optimization/<이름>/".
- [ ] **Step 4: 로그 패널** — LiDAR SLAM·Localization 헤더 액션 맨 앞에 `ui-icon-btn`(`aria-label="노드 로그"`) → 오른쪽 칸 맨 위에 접히는 `ui-panel` "NODE LOG (stderr 마지막 50줄)", 열려 있는 동안 3초마다 `process_log` 호출, `ui-mono` 11px `<pre>`. 안내 한 줄: "stdout(cout) 출력은 기록되지 않습니다"(P:3505가 stdout을 버림). 로그에 `Map file cannot open`·`No point, skip this scan`·`lidar loop back`이 있으면 해당 줄을 warn/stop 색으로.
- [ ] **Step 5: 검증·커밋** — pytest PASS, `py_compile`, `node --check`, 빌드, `curl -s 'localhost:8080/api/slam/save_map_check?name=Map'` → `valid:false`. 메인 체크아웃 `02` §6.3에 두 엔드포인트 `✅` 추가. `feat(api): Save Map 이름 검증·존재 확인과 노드 stderr 로그 조회 API 추가`

### Task 2.11: 노드 건강 표시 (데이터 끊김·센서 끊김·Localization 시작 실패)

**Files:**
- Modify: `web/static/ui_shell.js`, `web/static/ui_status_logic.js`(Task 2.1에서 이미 입력 추가됨), `test/ui/check_slam_loc.py`

- [ ] **Step 1: 실패 확인** — `check_slam_loc.py`: `page.evaluate`로 `uiShell.noteAnalytics('slam')` 호출, `onSlamState({is_running: true})`, 3.5초 기다림 → `#lidar-slam-header-badge` 글자 "Running · 데이터 없음". Localization: `startLocalizationMapping` 응답을 `page.route`로 성공 처리, `/loc_analytics` 없이 10.5초 → 배지 "시작 실패 (로그 보기)". 기대: FAIL.
- [ ] **Step 2: 구현** — `ui_shell.js`에 1초 간격 타이머 하나: 보이는 화면이 SLAM/Loc이면 `refresh()`(stale 판정 갱신). Localization 시작 감시: `onLocStartRequested()`(J `startLocalizationMapping` 성공 분기에서 호출)에서 `locStartFailed = false; locStartWatch = setTimeout(() => { if (lastAnalyticsAt.loc < startedAt) { locStartFailed = true; refresh(); } }, 10000)`. `noteAnalytics('loc')`가 오면 `locStartFailed = false`. 배지가 "시작 실패"일 때 클릭하면 로그 패널(Task 2.10)을 연다. SLAM도 같은 감시를 두되 배지만 "데이터 없음"으로 둔다(SLAM은 데이터가 늦게 올 수 있어 실패로 단정하지 않음).
- [ ] **Step 3: 사이드바** — 전역 요약은 launch 프로세스만 보므로 사이드바 RUN은 그대로 둔다. 보이는 화면의 배지가 warn이면 사이드바 해당 항목 RUN 태그 톤도 warn으로 바꾼다(글자 RUN 유지).
- [ ] **Step 4: 검증·커밋** — `node --check`, 빌드, `check_slam_loc.py` PASS. `feat(slam): 데이터 끊김·센서 끊김·Localization 시작 실패를 헤더 배지로 표시`

### Task 2.12: Localization 초기 위치 지정 (확정 2026-10-16)

**Files:**
- Modify: `web/index.html` (`#loc-viewer-actions`에 버튼), `web/static/script.js` (`LocalizationLiveViewer`에 지정 모드), `web/static/ui_theme.css`

**Interfaces:**
- Produces: `locLiveViewer.startInitialPosePick()` / `cancelInitialPosePick()`. 확정 시 rosbridge로 `/initialpose`(`geometry_msgs/PoseWithCovarianceStamped`, `header.frame_id = 'odom'`) 1회 발행.

- [ ] **Step 1: 근거 확인** — `laserLocalization.cpp:752-829`에서 구독 QoS·메시지 타입·사용 필드(위치·쿼터니언만, 공분산 무시 여부)를 읽어 발행 메시지를 맞춘다. frame_id를 검사하는지 미확인 → 검사하면 그 값을 쓴다.
- [ ] **Step 2: 실패 확인** — `check_slam_loc.py`: `#loc-initpose-btn` 클릭 → 뷰어에 `.ui-viewer-overlay` 안내 "지도에서 위치를 누르고 방향으로 끄세요 (Esc 취소)"가 보이고, rosbridge 없이 확정하면 오류 토스트 "rosbridge 연결이 없어 보낼 수 없습니다". 기대: FAIL.
- [ ] **Step 3: 구현** — 탑뷰로 전환 → `pointerdown` 위치를 `Raycaster`로 z=0 평면 교점(위치) → 드래그 방향(yaw) → 화살표 미리보기(`THREE.ArrowHelper`, 재사용) → `pointerup`에 확인 대화상자 "이 위치로 초기화할까요? (x, y, yaw°)" → 발행 → 토스트 "초기 위치를 보냈습니다. 로봇이 멈춰 있어야 적용됩니다". 발행에는 뷰어가 쓰는 `ROSLIB.Ros`(`plotState.ros` 또는 자체 연결)를 쓴다. Localization이 실행 중이 아니면 버튼 잠금.
- [ ] **Step 4: 검증·커밋** — `node --check`, 빌드, `check_slam_loc.py` PASS. **실데이터(사용자)**: 원점이 아닌 곳에서 시작한 bag으로 지정 → 배지가 Initialized로 바뀌는지. `feat(slam): Localization 뷰어에서 초기 위치를 찍어 /initialpose 발행`

### P2 실데이터 체크리스트 (사용자 확인 요청)

Ctrl+Shift+R 뒤 실제 bag 재생 + SLAM으로 확인한다.

1. Start SLAM → 배지 Starting… → Running · 시간, 지도·KPI 갱신, 사이드바 RUN. 키프레임이 생기기 전에는 Save Map 잠금.
2. Save Map → 이름 입력(`Map`·공백 입력 시 거부 확인) → 오버레이 "지도 저장 중"(무한 진행 바·경과 시간) → 완료 → 결과 뷰어 + 배지 "Map Saved · SLAM 계속 실행 중", 단계 Result. **저장에 걸린 시간을 알려 주세요**(계획의 오버레이 문구 조정용).
3. 저장 중 "기다리기 중단" → 라이브 지도와 대시보드 복원, 토스트 "저장은 노드에서 계속됩니다", 잠시 뒤 `pose_graph_optimization/<이름>/` 폴더가 생기는지.
4. 같은 이름으로 다시 저장 → 덮어쓰기 확인 대화상자.
5. 저장 없이 Stop → 확인 대화상자. Stop 뒤 배지 Stopping… → Ready, 빈 지도 칸, Config 입력 다시 열림.
6. 실행 중 fastlio 노드만 `kill` → 3초 뒤 배지 "Running · 데이터 없음", 로그 패널에 마지막 오류.
7. Localization Start → Running · Not Init → Initialized · Tracking, 사용 지도 경로 표시, Stop 직후 뷰어가 다시 나타나지 않음. `map_file_path`를 없는 파일로 바꿔 Start → 10초 뒤 "시작 실패 (로그 보기)".
8. 새로고침(실행 중) → 화면·배지·Config 읽기 전용 복원.

### P2 완료 처리

- [ ] 체크리스트 결과를 받은 뒤 `03` 8.2 체크. finishing 메뉴.

---

## 9. P3 태스크 — Multi-Session / Bag Player / Bag Recorder / File Player (Phase 8.3)

브랜치: `feature/webui-redesign-p3`.

### Task 3.1: 파생 함수 (Multi·Bag·Recorder·File)

**Files:**
- Modify: `web/static/ui_status_logic.js`, `test/ui/check_status_logic.js`

**Interfaces:**
- Produces: `MULTI_STEPS = ['Inputs','Optimization','Result']`, `OPT_PROGRESS_LABELS`(5개), `optDoneCount(message) -> 0..5`, `parseLoopCount(message) -> number | null`, `deriveMultiView({running, complete, failed, inputsReady, doneCount, loopCount})`, `deriveBagView({loaded, playing, paused, rate, converting})`, `deriveRecorderView({recording, hasName, topicCount, elapsedSec})`, `deriveFileView({loaded, playing, paused, converting, savingBag, progress})`. 반환 버튼 키: Multi `{run, cancel}`, Bag `{play, pause, seek, convert}`, Recorder `{record, edit}`, File `{play, load, saveBag}`.

- [ ] **Step 1: 실패 테스트 추가**

```javascript
assert.strictEqual(L.optDoneCount(''), 0);
assert.strictEqual(L.optDoneCount('Session Edge Loading Complete.'), 1);
assert.strictEqual(L.optDoneCount('Loop Edge Generation Complete. size : 12'), 2);
assert.strictEqual(L.optDoneCount('Graph Optimization Complete.'), 3);
assert.strictEqual(L.optDoneCount('Map Merging Complete.'), 4);
assert.strictEqual(L.optDoneCount('Long Term SLAM Complete.'), 5);
assert.strictEqual(L.parseLoopCount('Loop Edge Generation Complete. size : 12'), 12);
assert.strictEqual(L.parseLoopCount('Map Merging Complete.'), null);
const mb = { running: false, complete: false, failed: false, inputsReady: true, doneCount: 0, loopCount: null };
assert.strictEqual(L.deriveMultiView(Object.assign({}, mb, { running: true, doneCount: 2 })).text, 'Optimizing · 2/5');
assert.deepStrictEqual(L.deriveMultiView(Object.assign({}, mb, { running: true })).buttons, { run: false, cancel: true });
assert.strictEqual(L.deriveMultiView(Object.assign({}, mb, { inputsReady: false })).buttons.run, false);
assert.strictEqual(L.deriveMultiView(Object.assign({}, mb, { complete: true })).step, 2);
assert.strictEqual(L.deriveMultiView(Object.assign({}, mb, { complete: true, loopCount: 0 })).tone, 'warn');
assert.strictEqual(L.deriveMultiView(Object.assign({}, mb, { complete: true, loopCount: 12 })).text, 'Done · 루프 12개');
const bb = { loaded: true, playing: false, paused: false, rate: 1, converting: false };
assert.strictEqual(L.deriveBagView(Object.assign({}, bb, { loaded: false })).buttons.play, false);
assert.strictEqual(L.deriveBagView(Object.assign({}, bb, { playing: true, rate: 2 })).text, 'Playing · 2×');
assert.strictEqual(L.deriveBagView(Object.assign({}, bb, { playing: true, paused: true })).tone, 'warn');
assert.strictEqual(L.deriveRecorderView({ recording: true, hasName: true, topicCount: 3, elapsedSec: 3 }).text, 'Recording · 0:03');
assert.strictEqual(L.deriveRecorderView({ recording: false, hasName: true, topicCount: 0, elapsedSec: 0 }).buttons.record, false);
assert.strictEqual(L.deriveFileView({ loaded: true, playing: false, paused: false, converting: true, savingBag: false, progress: 28 }).text, 'Saving bag · 28%');
```

- [ ] **Step 2: 구현** (3.3~3.6절 표를 코드로)

```javascript
    const MULTI_STEPS = ['Inputs', 'Optimization', 'Result'];
    // long_term_mapping main 소스의 실행 순서 (long_term_mapping.cpp:1431-1463)
    const OPT_PROGRESS = [
        { label: '세션 불러오기', doneOn: ['Session Edge Loading Complete.'] },
        { label: '세션 정렬·루프 찾기', doneOn: ['Place Recognition Complete.', 'Loop Edge Generation Complete.'] },
        { label: '포즈 그래프 최적화', doneOn: ['Pose Factor loading Complete.', 'Graph Optimization Complete.'] },
        { label: '병합 지도 만들기', doneOn: ['Map Merging Complete.'] },
        { label: '변화 검출·정적 지도', doneOn: ['Map Update Complete.', 'Long Term SLAM Complete.'] },
    ];
    const OPT_PROGRESS_LABELS = OPT_PROGRESS.map(p => p.label);

    function optDoneCount(message) {
        const msg = String(message || '');
        for (let i = OPT_PROGRESS.length - 1; i >= 0; i--) {
            if (OPT_PROGRESS[i].doneOn.some(m => msg.startsWith(m))) return i + 1;
        }
        return 0;
    }

    function parseLoopCount(message) {
        const m = /size : (\d+)/.exec(String(message || ''));
        return m ? Number(m[1]) : null;
    }

    function deriveMultiView(s) {
        const total = OPT_PROGRESS.length;
        if (s.running) return view('info', `Optimizing · ${s.doneCount}/${total}`, 1, { run: false, cancel: true });
        if (s.complete && s.loopCount === 0) return view('warn', 'Done · 세션 사이 루프 0개', 2, { run: true, cancel: false });
        if (s.complete) {
            const text = s.loopCount === null ? 'Done' : `Done · 루프 ${s.loopCount}개`;
            return view('go', text, 2, { run: true, cancel: false });
        }
        if (s.failed) return view('stop', 'Failed', 0, { run: s.inputsReady, cancel: false });
        return view('idle', s.inputsReady ? 'Ready' : 'Inputs needed', 0, { run: s.inputsReady, cancel: false });
    }

    function deriveBagView(s) {
        if (!s.loaded) return view('idle', 'No bag', 0, { play: false, pause: false, seek: false, convert: false });
        if (s.converting) return view('info', 'Converting…', 0, { play: false, pause: false, seek: false, convert: false });
        if (s.paused) return view('warn', 'Paused', 0, { play: true, pause: true, seek: true, convert: false });
        if (s.playing) return view('go', `Playing · ${s.rate}×`, 0, { play: true, pause: true, seek: true, convert: false });
        return view('idle', 'Loaded', 0, { play: true, pause: false, seek: true, convert: true });
    }

    function deriveRecorderView(s) {
        if (s.recording) return view('stop', `Recording · ${formatElapsed(s.elapsedSec)}`, 0, { record: true, edit: false });
        const ready = s.hasName && s.topicCount > 0;
        return view('idle', ready ? 'Ready' : '이름·토픽 필요', 0, { record: ready, edit: true });
    }

    function deriveFileView(s) {
        if (s.converting || s.savingBag) {
            return view('info', `Saving bag · ${Math.round(s.progress || 0)}%`, 0, { play: true, load: false, saveBag: false });
        }
        if (s.paused) return view('warn', 'Paused', 0, { play: true, load: false, saveBag: true });
        if (s.playing) return view('go', 'Playing', 0, { play: true, load: false, saveBag: false });
        if (s.loaded) return view('idle', 'Loaded', 0, { play: true, load: true, saveBag: true });
        return view('idle', 'Not loaded', 0, { play: false, load: true, saveBag: false });
    }
```

- [ ] **Step 3: 검증·커밋** — Node PASS, `node --check`. `feat(ui): Multi·Bag·Recorder·File 상태 파생 함수 추가`

### Task 3.0: 선행 — `long_term_mapping` `main` 소스 재빌드 (확정 2026-10-16)

**Files:** 없음(빌드·확인만, 웹 UI 저장소 밖)

- [ ] **Step 1: 차이 확인** — `strings /home/kkw/localization_ws/install/long_term_mapping/lib/long_term_mapping/LTmapping | grep -c "Long Term SLAM Complete"`가 0이면 설치 바이너리가 소스(`main`)와 다른 상태다(2026-10-16 대조 시 0건, 13.4절).
- [ ] **Step 2: 재빌드** — `long_term_mapping`이 `main` 브랜치인지 `git -C /home/kkw/localization_ws/src/long_term_mapping branch --show-current`로 확인한 뒤 `cd /home/kkw/localization_ws && colcon build --packages-select long_term_mapping`. 다시 Step 1 결과가 1 이상인지 확인한다. 설치 `share/.../config`의 끊어진 symlink(`b2_*`, `kaist_params.yaml`, `parking_*`)가 재빌드로 정리되는지 `ls -l`로 본다.
- [ ] **Step 3: 스모크** — `strings`로 `Session Edge Loading Complete.`·`Map Update Complete.`도 있는지 확인. 실제 실행 확인은 Task 3.11 Step 5(사용자 실데이터)에서 한다. ELite 변종 바이너리는 이후 쓰지 않는다(`seam` 패키지로 분리된 코드).
- 커밋 없음(다른 패키지 빌드).

### Task 3.2: Multi-Session 단일 폼·헤더 버튼·진행 5단계

**Files:**
- Modify: `web/index.html:706-747`, `web/static/script.js:415-481, 652-655, 662-668`, `web/static/ui_shell.js`, `web/static/style.css` (`runOptimization`·`optimizeSlam` 속성 선택자)

- [ ] **Step 1: 실패 확인** `test/ui/check_data.py` (Multi 부분): 화면 진입 → `#multi-session-header-actions #slam-opt-run-btn` 존재·`Map1/Map2` 비어 있으면 비활성, `#multi-session-progress li` 5개. Output에 `..`를 넣으면 Run 비활성 + `#multi-session-output-error` 글자. `#slam-output`에 글자 입력 후 1초(폴링 2번) 기다려도 값 유지. 기대: FAIL.
- [ ] **Step 2: 폼 마크업** — 왼쪽 칸 `INPUTS` 패널: `Map 1`/`Map 2`/`Output` 각 `ui-field` = 라벨 + 입력(`#slam-map1` 읽기 전용, `#slam-map2` 읽기 전용, `#slam-output` 편집) + 폴더 아이콘 버튼(`ui-icon-btn`, `onclick="loadMap1()"`/`"loadMap2()"`, `aria-label="Map 1 폴더 선택"`). 기존 "Set Output" 버튼은 숨긴다(`hidden`, P6 삭제) — 실행 버튼이 대신 저장한다. 도움말 `<p class="ui-hint">`: "Save Map으로 만든 세션 폴더 (Scans/·optimized_poses.txt·edges.txt·StaticMap.pcd 필요)" / "합칠 두 번째 세션" / "결과 폴더 이름 — long_term_mapping/<이름>/에 만들어지고, 같은 이름의 폴더는 지워집니다". Output 아래 `<p class="ui-hint ui-error" id="multi-session-output-error" hidden>`: 프론트 검사(`^[A-Za-z0-9_-]{1,64}$`, Map1·Map2 폴더 이름과 같으면 "입력 폴더와 같은 이름은 쓸 수 없습니다") 실패 시 보이고 Run 잠금. Map1/Map2 칸 아래 `#multi-session-map1-check`/`-map2-check`: Task 3.11의 `set_map1/2` 응답 `missing` 목록이 있으면 "필요한 파일 없음: …". 그 아래 `PROGRESS` 패널 `<ol class="ui-progress-list" id="multi-session-progress"></ol>`, `#slam-status`는 폼 아래 `ui-hint`로 이동. 오른쪽 칸에 `#slam-result-viewer`(+ 새 `#multi-session-viewer-empty`)와 `#slam-opt-status-area`(`ui-viewer-overlay`).
- [ ] **Step 3: 헤더 버튼** — `#slam-opt-run-btn`을 `#multi-session-header-actions`로 옮기고 `onclick="runMultiSession()"`로 **바꾸지 않는다**(핸들러 이름 보존). 대신 `handleOptBtnClick()`(J:457) 처음에 Output 저장을 넣는다:

```javascript
    if (!_optComplete && _multiOutputDirty) {
        const output = domCache.get('slam-output').value.trim();
        const saved = await apiCall('/api/slam/set_output', { path: output });
        if (!saved.success) { alert('Failed to set output: ' + (saved.error || saved.message || '')); return; }
        _multiOutputDirty = false;
    }
```

`handleOptBtnClick`가 async가 아니면 `async`를 붙인다(호출처는 onclick 하나라 영향 없음 — `rg -n "handleOptBtnClick" web/` 로 확인). 파일 상단 `_opt*` 선언 옆에 `let _multiOutputDirty = false;`, `#slam-output`에 `oninput="_multiOutputDirty = true"`. `updateSlamState`의 Output 덮어쓰기 조건(J:653)을 `if (document.activeElement !== outputField && !_multiOutputDirty)`로. 헤더에 새 `<button id="multi-session-header-cancel-btn" class="ui-btn ui-btn-stop" onclick="cancelOptimization()" hidden>`. `_resetOptBtn`의 글자 `'Multi Session Optimization'`은 그대로 둔다.
- [ ] **Step 4: 렌더** — `ui_shell.js` `renderMulti()`: 입력값으로 `inputsReady`, `_optRunning`·`_optComplete`·`#slam-opt-status-area.classList.contains('error')`, `#slam-opt-msg` 글자로 `optDoneCount`. 배지·단계 표시줄·Run 잠금(`running`이면 J가 이미 disabled — `setEnabled`는 `running`이 아닐 때만 호출해 J와 다투지 않게 한다)·Cancel `hidden = !v.buttons.cancel`. 진행 목록은 `OPT_PROGRESS_LABELS`로 `li`를 만들고 `data-state` = done/current/todo(`running`일 때 current = doneCount 인덱스). 메시지가 다음 단계로 넘어가면 `size : N`이 사라지므로 `parseLoopCount`가 숫자를 돌려준 마지막 값을 `ui_shell.js`의 `multiLoopCount`에 기억하고(새 실행 시작 때 `null`로 초기화) 2단계 항목 옆에 "루프 N개"로 보인다. 훅: `_showOptStatus`/`_showOptSuccess`/`_showOptError`/`_resetOptBtn` 끝과 `_pollOptimizationStatus`의 메시지 갱신 뒤에 `uiShell.refresh()`.
- [ ] **Step 5: CSS** — `runOptimization`/`optimizeSlam` 속성 선택자 규칙 삭제, `#slam-opt-run-btn`에 `ui-btn ui-btn-go`.
- [ ] **Step 6: 검증** — `node --check`, 빌드, `check_data.py` PASS, `check_ids.py dev` PASS, 스크린샷 vs 목업 `06-multi-running.png`.
- [ ] **Step 7: 커밋** — `feat(slam): Multi-Session 단일 입력 폼·헤더 실행 버튼·진행 5단계 표시`

### Task 3.3: Multi-Session 결과 칩·자동 복원 버그·새로고침 진행 재개

**Files:**
- Modify: `web/index.html:749-820`, `web/static/script.js:662-668`, `web/static/ui_shell.js`

- [ ] **Step 1: 실패 확인** — `check_data.py`: `page.route('/api/slam/state')`로 `status: 'Optimization complete!'`를 주고 진입 → 결과 뷰어 표시 시도 + `#slam-opt-run-btn` 글자 `Exit`. `/api/system/status_summary`에 `optimization.running=true`를 주면 3초 안에 `/api/slam/optimization_status` 요청이 발생. 기대: FAIL.
- [ ] **Step 2: 자동 복원 수정** — J:665-667 `slamResultViewer.show();` 앞에 `_optComplete = true; const runBtn = domCache.get('slam-opt-run-btn'); runBtn.textContent = 'Exit';`(J:512-538 `_showOptSuccess`가 쓰는 것과 같은 글자). 사용자 승인 필요 버그 수정으로 보고.
- [ ] **Step 3: 진행 재개** — `pollSummary()`에서 `if (lastSummary.optimization.running && typeof _optPollTimer !== 'undefined' && !_optPollTimer && !_optRunning) { _optRunning = true; _startOptPolling(); }`.
- [ ] **Step 4: 칩** — `#slam-pcd-legend`/`#slam-result-legend` 행을 Task 2.6과 같은 칩 CSS로. 색 스와치는 Phase 4.15 팔레트 그대로. diff 칩 `title`(코드 기준 의미, 13.4절): PD "세션2에만 있는 구조 (새로 생김)", ND "세션1에만 있는 구조 (사라짐)", FirstUE "세션1만 본 영역 (세션2 미관측)", SecondUE "세션2만 본 영역 (세션1 미관측)". diff 파일이 비어 있어 없으면 칩에 "(없음)"을 붙이고 흐리게. `#slam-fs-diff-btn`은 `ui-icon-btn` + `aria-label="Check Difference"`.
- [ ] **Step 5: 검증·커밋** — `node --check`, 빌드, `check_data.py` PASS. `fix(slam): Multi-Session 결과 자동 복원 시 Exit 표시·새로고침 후 진행 폴링 재개, 결과 레이어 칩`

### Task 3.4: 백엔드 — bag 토픽 상세(Type·Count)

**Files:**
- Modify: `ros_slam_webui/web_server.py:6350-6421` (`get_bag_info`)
- Create: `test/test_bag_info_parse.py`
- Modify (메인 체크아웃): `02` §6 (응답 필드 메모)

**Interfaces:**
- Produces: 모듈 함수 `parse_ros2_bag_info(stdout: str) -> dict` = `{'topics': [str], 'topic_details': [{'name', 'type', 'count'}], 'duration': float}`. `/api/bag/load`·`/api/bag/get_info` 응답에 `topic_details` 추가(기존 `topics` 유지).

- [ ] **Step 1: 실패 테스트**

```python
import pytest

web_server = pytest.importorskip('ros_slam_webui.web_server')

SAMPLE = """
Files:             b2_1_0.mcap
Bag size:          4.2 GiB
Storage id:        mcap
Duration:          190.123456789s
Topic information: Topic: /livox/lidar | Type: livox_ros_driver2/msg/CustomMsg | Count: 1902 | Serialization Format: cdr
                   Topic: /livox/imu | Type: sensor_msgs/msg/Imu | Count: 38040 | Serialization Format: cdr
"""


def test_parse_topics_and_counts():
    r = web_server.parse_ros2_bag_info(SAMPLE)
    assert r['topics'] == ['/livox/lidar', '/livox/imu']
    assert r['topic_details'][1] == {'name': '/livox/imu', 'type': 'sensor_msgs/msg/Imu', 'count': 38040}
    assert abs(r['duration'] - 190.123456789) < 1e-6


def test_missing_fields_are_tolerated():
    r = web_server.parse_ros2_bag_info('Topic: /x | Count: abc')
    assert r['topic_details'] == [{'name': '/x', 'type': '', 'count': None}]
```

Jazzy `ros2 bag info` 실제 출력 형식은 미확인 → 실행 전에 실제 bag 하나로 `ros2 bag info <bag>`를 돌려 SAMPLE을 그 출력으로 바꾼다.

- [ ] **Step 2: 구현** — 기존 루프(P:6380-6404)의 파싱을 모듈 함수로 옮기고 `get_bag_info`는 그 결과를 쓴다. 각 `Topic:` 줄을 `|`로 나눠 `키: 값` 사전으로 만들고 `count`는 `int` 변환 실패 시 `None`. `Duration` 줄은 기존 방식. `get_bag_info` 반환에 `'topic_details': parsed['topic_details']` 추가, `self.bag_topics`는 `parsed['topics']`. 기존 줄 단위 `info` 로그(P:6381)는 그대로 둔다.
- [ ] **Step 3: ROS1** — `_get_ros1_bag_info`(P:6423~)가 rosbags로 연결 정보를 읽을 때 메시지 수(`connection.msgcount`, 이름 미확인)를 얻을 수 있으면 `count`를, 아니면 `None`을 넣어 `topic_details`를 같은 모양으로 반환.
- [ ] **Step 4: 검증·커밋** — pytest PASS, `py_compile`, 빌드, `curl -s -X POST localhost:8080/api/bag/load -d '{"path":"<bag>"}'`에 `topic_details`. `feat(api): bag 정보에 토픽별 Type·메시지 수(topic_details) 추가`

### Task 3.5: Bag Player 토픽 표 (Select Topic 모달 대체)

**Files:**
- Modify: `web/index.html:856-868`, `web/static/script.js` (`applyBagPlayerInfo` J:734 끝, 새 함수 `renderBagTopicTable`·`setAllBagTopics`), `web/static/ui_theme.css` (`.ui-table`)

**Interfaces:**
- Consumes: `bagPlayerState.availableTopics`(문자열 또는 `{name,type,publishable}`), `bagPlayerState.selectedTopics`, `bagPlayerState.bagDuration`, 새 `bagPlayerState.topicDetails`(Task 3.4).
- Produces: `renderBagTopicTable()`, `setAllBagTopics(checked: boolean)`.

- [ ] **Step 1: 실패 확인** — `check_data.py`: `page.route('/api/bag/load')`로 토픽 3개(`topic_details` 포함, duration 10) 응답을 주고 `loadBagFile` 흐름 대신 `page.evaluate("applyBagPlayerInfo('/x', {topics:[...], topic_details:[...], duration:10, bag_type:'ros2'}, {resetSlider:true})")` 호출 → `#bag-topic-table tbody tr` 3개, Hz 칸 `count/duration` 값, 체크 해제 시 `bagPlayerState.selectedTopics` 길이 감소. 기대: FAIL. (`applyBagPlayerInfo` 인자 모양은 J:734에서 확인해 맞춘다.)
- [ ] **Step 2: 마크업** — `SELECTED TOPICS · n/m` 패널: 머리에 `<button class="ui-btn ui-btn-ghost" onclick="setAllBagTopics(true)">모두 선택</button>`, `…(false)">모두 해제</button>`, 본문 `<table class="ui-table" id="bag-topic-table"><thead><tr><th scope="col"><span class="ui-sr-only">선택</span></th><th scope="col">Topic</th><th scope="col">Type</th><th scope="col" class="ui-num">Hz</th><th scope="col" class="ui-num">Msgs</th></tr></thead><tbody></tbody></table>`, 아래 `<p class="ui-hint" id="bag-topic-table-hint" hidden>재생 중 바꾼 선택은 다음 Play부터 적용됩니다</p>`. 기존 "Select Topic" 버튼과 `#bag-selected-topics-display`는 `hidden`(P6 삭제).
- [ ] **Step 3: 구현** (J, `updateSelectedTopicsDisplay` 다음)

```javascript
function renderBagTopicTable() {
    const tbody = document.querySelector('#bag-topic-table tbody');
    if (!tbody) return;
    const details = new Map((bagPlayerState.topicDetails || []).map(d => [d.name, d]));
    const duration = bagPlayerState.bagDuration || 0;
    tbody.replaceChildren(...bagPlayerState.availableTopics.map(entry => {
        const name = typeof entry === 'object' ? entry.name : entry;
        const publishable = typeof entry === 'object' ? entry.publishable !== false : true;
        const info = details.get(name) || {};
        const type = info.type || (typeof entry === 'object' ? entry.type : '') || '';
        const count = Number.isFinite(info.count) ? info.count : null;
        const tr = document.createElement('tr');
        const box = document.createElement('input');
        box.type = 'checkbox';
        box.checked = publishable && bagPlayerState.selectedTopics.includes(name);
        box.disabled = !publishable;
        box.setAttribute('aria-label', `${name} 선택`);
        box.onchange = () => {
            const set = new Set(bagPlayerState.selectedTopics);
            if (box.checked) set.add(name); else set.delete(name);
            bagPlayerState.selectedTopics = bagPlayerState.availableTopics
                .map(t => (typeof t === 'object' ? t.name : t)).filter(t => set.has(t));
            updateSelectedTopicsDisplay();
            renderBagTopicTableMeta();
        };
        const cells = [name + (publishable ? '' : ' (not publishable)'), type,
            count !== null && duration > 0 ? (count / duration).toFixed(1) : '–',
            count !== null ? count.toLocaleString() : '–'];
        const tdBox = document.createElement('td');
        tdBox.appendChild(box);
        tr.appendChild(tdBox);
        cells.forEach((text, i) => {
            const td = document.createElement('td');
            td.textContent = text;
            if (i >= 2) td.className = 'ui-num';
            tr.appendChild(td);
        });
        if (!publishable) tr.dataset.disabled = 'true';
        return tr;
    }));
    renderBagTopicTableMeta();
}

function renderBagTopicTableMeta() {
    const title = document.getElementById('bag-topic-table-title');
    if (title) title.textContent = `SELECTED TOPICS · ${bagPlayerState.selectedTopics.length}/${bagPlayerState.availableTopics.length}`;
    const hint = document.getElementById('bag-topic-table-hint');
    if (hint) hint.hidden = domCache.get('bag-play-button').textContent.trim() !== 'Stop';
}

function setAllBagTopics(checked) {
    bagPlayerState.selectedTopics = checked
        ? bagPlayerState.availableTopics.filter(t => typeof t !== 'object' || t.publishable !== false)
            .map(t => (typeof t === 'object' ? t.name : t))
        : [];
    updateSelectedTopicsDisplay();
    renderBagTopicTable();
}
```

`applyBagPlayerInfo` 끝에 `bagPlayerState.topicDetails = info.topic_details || [];`와 `renderBagTopicTable();`. 0개 선택 경고는 Play 때(`playBag`)에 이미 서버가 처리하는지 미확인 → 0개면 표 제목을 warn 톤으로만 표시하고 alert는 내지 않는다.

- [ ] **Step 4: 검증·커밋** — `node --check`, 빌드, `check_data.py` PASS. `feat(player): Bag Player 토픽 선택을 모달 대신 표(Type·Hz·Msgs)로 표시`

### Task 3.6: 하단 고정 재생 바 (Bag Player)

**Files:**
- Modify: `web/index.html:868-898`, `web/static/script.js` (Play/Pause 라벨 대입), `web/static/ui_shell.js`, `web/static/ui_theme.css` (`.ui-playbar`)

- [ ] **Step 1: 라벨 대입 목록 확인** — `rg -n "(bag-play-button|bag-pause-button)|playBtn\.textContent\s*=|pauseBtn\.textContent\s*=" web/static/script.js`로 J:947-1231 안의 `textContent =` 대입을 모두 찾는다(비교 `===`는 그대로 둠).
- [ ] **Step 2: 실패 확인** — `check_data.py`: Bag Player에서 `.ui-playbar`가 뷰포트 아래쪽에 붙어 있고(스크롤 후에도 `boundingBox().y + height ≈ viewport height`), bag 미로드 시 `#bag-play-button` disabled, 버튼 안에 `svg`가 있으면서 `textContent.trim() === 'Play'`. 기대: FAIL.
- [ ] **Step 3: 마크업** — 서브탭 끝에 `<div class="ui-playbar" id="bag-playbar">` = `#bag-play-button`(아이콘+`.ui-btn-label`), `#bag-pause-button`, `#bag-time-label`(Fira Code), `#bag-slider`, 속도 분할 버튼 `<div class="ui-seg" role="radiogroup" aria-label="재생 속도" id="bag-rate-seg">`(0.5/1/2/4, `data-rate`), `#bag-playback-rate` 슬라이더와 `#playback-rate-label`은 `<details class="ui-more"><summary>세밀 조절</summary>…</details>` 안으로 옮긴다(확정 2026-10-16). 분할 버튼에 없는 값(예: 1.3×)이면 분할 버튼은 모두 `aria-checked="false"`, 라벨은 슬라이더 값. Loop 토글 `#bag-player-loop`. 모두 **이동**이다.
- [ ] **Step 4: 라벨 헬퍼 적용** — Step 1 목록의 `btn.textContent = 'X'`를 `uiKit.setBtnLabel(btn, 'X')`로 바꾼다. 아이콘은 CSS가 아니라 버튼 안 `<svg>`라 라벨만 바뀐다.
- [ ] **Step 5: 속도 분할 버튼** — `ui_shell.js`에서 `#bag-rate-seg [data-rate]` 클릭 시 `const slider = document.getElementById('bag-playback-rate'); slider.value = String(Math.round(rate * 10)); updatePlaybackRate();` 후 `aria-checked` 갱신. 렌더 때 슬라이더 값(`value/10`)과 같은 버튼을 `aria-checked="true"`로.
- [ ] **Step 6: 렌더** — `renderBag()`: 3.4절 입력으로 `deriveBagView` → 배지, Play/Pause/슬라이더/속도/Loop/Convert 잠금. J `updateBagState()` 끝과 `playBag`/`pauseBag`/`convertBag` 끝에 `uiShell.refresh()`.
- [ ] **Step 7: 검증·커밋** — `node --check`, 빌드, `check_data.py` PASS, 스크린샷 vs `08-bag-player.png`. **실데이터(사용자)**: ROS1·ROS2 bag 각각 Play/Pause/Seek/속도 변경/Loop. `feat(player): Bag Player 하단 고정 재생 바·속도 분할 버튼·상태별 잠금`

### Task 3.7: Bag 형식 변환 패널

**Files:**
- Modify: `web/index.html:842-854`, `web/static/ui_theme.css`

- [ ] **Step 1:** 오른쪽 칸 `형식 변환 (CONVERT)` 패널로 `#player-format-select`·`#convert-bag-btn` 이동, `#bag-ros1-badge`/`#bag-ros2-badge`는 bag 경로 아래 태그 줄로(`ui-tag`), 같은 줄에 `길이 m:ss`·`n topics` 태그(새 `#bag-meta-duration`, `#bag-meta-topics`, `applyBagPlayerInfo` 끝에서 채움). 변환 중(`#convert-bag-btn.disabled`) 버튼 아래 `ui-progress-indeterminate` 표시(CSS `#convert-bag-btn:disabled + .ui-progress { display: block; }`). 도움말 "선택한 토픽만 변환됩니다".
- [ ] **Step 2: 검증·커밋** — 빌드, 스크린샷. `style(player): Bag 형식 변환 패널·bag 정보 태그 재배치`

### Task 3.8: 백엔드 — Recorder 경과 시간·크기

**Files:**
- Modify: `ros_slam_webui/web_server.py` (`__init__` recorder 속성 옆, `record_bag` P:4085-4210, `get_recorder_state` P:4212)
- Create: `test/test_recorder_stats.py`

**Interfaces:**
- Produces: `/api/recorder/state`에 `elapsed_sec: float`, `size_bytes: int` 추가. 모듈 함수 `path_size_bytes(path: str) -> int`(파일이면 크기, 폴더면 하위 파일 합, 없으면 0).

- [ ] **Step 1: 출력 경로 확인** — P:4125-4210에서 ROS2(`ros2 bag record -o …`)와 ROS1(`.bag`) 출력 경로를 만드는 식을 읽고, 녹화 시작 시 그 경로를 `self.recorder_output_path`에 저장할 위치를 정한다.
- [ ] **Step 2: 실패 테스트**

```python
import pytest

web_server = pytest.importorskip('ros_slam_webui.web_server')


def test_path_size_file_and_dir(tmp_path):
    f = tmp_path / 'a.bag'
    f.write_bytes(b'x' * 10)
    d = tmp_path / 'b_bag'
    d.mkdir()
    (d / 'b_0.mcap').write_bytes(b'y' * 7)
    (d / 'metadata.yaml').write_bytes(b'z' * 3)
    assert web_server.path_size_bytes(str(f)) == 10
    assert web_server.path_size_bytes(str(d)) == 10
    assert web_server.path_size_bytes(str(tmp_path / 'none')) == 0
```

- [ ] **Step 3: 구현** — 모듈 함수:

```python
def path_size_bytes(path):
    """파일이면 크기, 폴더면 하위 파일 크기 합, 없으면 0."""
    if not path or not os.path.exists(path):
        return 0
    if os.path.isfile(path):
        return os.path.getsize(path)
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(root, name))
            except OSError:
                pass
    return total
```

`__init__`에 `self.recorder_started_at = None`, `self.recorder_output_path = ''`. 시작 성공 시 둘 다 설정, 중지 분기(P:4112)에서 `None`/`''`. `get_recorder_state`에 `'elapsed_sec': (time.time() - self.recorder_started_at) if self.recorder_recording and self.recorder_started_at else 0.0`, `'size_bytes': path_size_bytes(self.recorder_output_path) if self.recorder_recording else 0`. `os.walk`는 큰 mcap 1~2개라 2초 폴링에 부담이 작다(미측정).

- [ ] **Step 4: 검증·커밋** — pytest PASS, `py_compile`, 빌드, 녹화 중 `curl -s localhost:8080/api/recorder/state`. 메인 체크아웃 `02` §6에 `/api/recorder/state` 필드 메모. `feat(api): Recorder 상태에 경과 시간·파일 크기 추가`

### Task 3.9: Bag Recorder 화면

**Files:**
- Modify: `web/index.html:905-942`, `web/static/script.js` (폴링 J:4184-4190, `updateRecorderState` J:2645, `recordBag` J:2606, 새 `renderRecorderTopicTable`), `web/static/ui_shell.js`

- [ ] **Step 1: 실패 확인** — `check_data.py`: Recorder 진입 → `/api/recorder/state` 요청이 1초 안에 발생(지금은 0), `page.route`로 `recording:true, elapsed_sec:3, size_bytes:67108864`를 주면 `#recorder-stat-time`=`0:03`, `#recorder-stat-size`=`64 MB`, 배지 "Recording · 0:03", `#recorder-format-select` disabled. 기대: FAIL.
- [ ] **Step 2: 폴링 추가** — J:4188 `file-player-subtab` 분기 뒤에 `else if (activeSubTab && activeSubTab.id === 'bag-recorder-subtab') { updateRecorderState(); }`. 페이지 로드 초기 호출(J:4158 뒤)에도 `updateRecorderState();`. `updateRecorderState` 끝에 `if (window.uiShell) uiShell.onRecorderState(state);`(함수 안 응답 변수 이름은 J:2645에서 확인).
- [ ] **Step 3: 마크업** — 왼쪽 `RECORD` 패널: Bag Name(`#recorder-bag-name` + 폴더 아이콘 버튼 `onclick="enterBagName()"`), Format(`#recorder-format-select`), 큰 `#recorder-record-button`(`ui-btn ui-btn-record`, 아이콘+`.ui-btn-label`), 통계 3칸 `#recorder-stat-time`·`#recorder-stat-size`·`#recorder-stat-topics`(`ui-kpi`, 녹화 중 stop 톤). 오른쪽 `SELECTED TOPICS` 표 `#recorder-topic-table`(Topic·Type. Hz 칸 없음 — 확정 2026-10-16), "목록 새로고침" 버튼 = 기존 `selectRecorderTopics()`가 쓰는 `/api/recorder/get_topics` 결과로 표를 다시 그림. 모달·"Select Topic" 버튼은 `hidden`. `#recorder-mode-badge`는 헤더 배지 옆 `ui-tag`.
- [ ] **Step 4: 표 렌더** — Task 3.5와 같은 구조로 `renderRecorderTopicTable(topics)`를 J에 추가(체크 변경 시 `bagRecorderState.selectedTopics`를 `{name,type}` 배열로 갱신 → `updateRecorderTopicsDisplay` 대응 함수 호출 — 이름은 J:2587에서 확인). 녹화 중엔 체크박스 disabled. 라벨 대입(`recordBag` J:2629-2638)은 `uiKit.setBtnLabel`.
- [ ] **Step 5: 렌더** — `renderRecorder()`: `deriveRecorderView` → 배지·Record 잠금·`edit` 잠금(이름 버튼, 형식, 표 체크), 통계 칸(크기는 `(bytes/1048576).toFixed(0) + ' MB'`, 1 GB 이상이면 GB).
- [ ] **Step 6: 검증·커밋** — `node --check`, 빌드, `check_data.py` PASS, 스크린샷 vs `09-bag-recorder.png`. **실데이터(사용자)**: 실제 녹화 시작·중지, 새로고침 후 REC 복원. `feat(recorder): Bag Recorder 큰 Record 버튼·경과 시간·크기·토픽 표, 새로고침 복원`

### Task 3.10: File Player 데이터셋 분할 버튼·통일된 폼·재생 바

**Files:**
- Modify: `web/index.html:945-1152`, `web/static/script.js` (Play/Pause 라벨 대입 J:2260-2410, KITTI WS 핸들러 null 가드 J:4316-4346), `web/static/ui_shell.js`, `web/static/ui_theme.css`

- [ ] **Step 1: 실패 확인** — `check_data.py`: File Player에서 `#file-player-dataset-seg [data-dataset="kitti"]` 클릭 → `#dataset-format-select.value === 'kitti'`, `#kitti-ui` 보임, `#conpr-save-row` 숨김, 세그먼트 `aria-checked`. 하단 `.ui-playbar` 존재. 기대: FAIL.
- [ ] **Step 2: 분할 버튼** — `#dataset-format-select`를 `hidden`으로 두고 위에 `<div class="ui-seg" role="radiogroup" aria-label="데이터셋" id="file-player-dataset-seg">` + 버튼 5개(`data-dataset` = conpr/kitti/kaist/mulran/helipr, 라벨 ConPR / KITTI Raw / KAIST Complex Urban / MulRan / HeLiPR). `ui_shell.js`: 클릭 시 `select.value = ds; select.dispatchEvent(new Event('change'));` → 기존 `onDatasetFormatChange`. 렌더 때 select 값으로 `aria-checked` 동기화(변환 중이면 세그먼트 잠금 — `deriveFileView.buttons.load`).
- [ ] **Step 3: 통일된 폼** — 5개 블록의 DOM은 합치지 않는다. 같은 클래스로 같은 모양을 만든다: 각 블록을 `ui-panel` "BAG으로 저장" 안에 `ui-field`(Drive/Sequence select), 형식 select, Save bag 버튼(`ui-btn`), `ui-progress`(기존 `*-progress-fill`이 `width%`를 바꾸므로 그 요소에 `ui-progress-fill` 클래스 추가), 메시지. ConPR "Save bag" 글자를 "Save Bag"으로 통일(대소문자만). 경로·Load 줄은 `KITTI RAW · 불러오기` 식 제목의 공통 패널. Loop·Auto Start는 오른쪽 `재생 옵션` 패널(`ui-toggle`).
- [ ] **Step 4: 재생 바** — `#play-button`, `#pause-button`, `#player-slider`, `#player-timestamp-label`을 `.ui-playbar`로 이동. 라벨 대입(`textContent = 'Play'|'End'|'Pause'|'Resume'` 및 초기 `▶ Play` 등)을 `uiKit.setBtnLabel`로. 프레임 카운터는 만들지 않는다.
- [ ] **Step 5: KITTI null 가드** — J:4316-4346 진행 처리에서 다른 데이터셋처럼 `if (fill && text) { … }` 가드를 넣는다.
- [ ] **Step 6: 렌더** — `renderFile()`: 3.6절 입력(`lastSummary.file_player.loaded`, 버튼 글자, `*State.converting`, `save_bag_saving`은 `lastSummary.file_player.saving_bag`, 진행률은 보이는 블록의 `*-progress-text` 숫자)으로 `deriveFileView` → 배지·잠금. `updatePlayerState` 끝과 각 `convert*`/`_on*ConvertDone`/`saveBag` 끝에 `uiShell.refresh()`.
- [ ] **Step 7: 검증·커밋** — `node --check`, 빌드, `check_data.py` PASS, `check_ids.py dev` PASS, 스크린샷 vs `10-file-player.png`. **실데이터(사용자)**: 데이터셋 하나로 Load→Play→Save Bag 진행률. `feat(player): File Player 데이터셋 분할 버튼·통일된 저장 폼·하단 재생 바`

### Task 3.11: 백엔드 — Multi-Session 실패 감지·입력/출력 검증

**Files:**
- Modify: `ros_slam_webui/web_server.py` (`set_slam_map1/2`·`set_slam_output` P:3440-3465 부근, `run_slam_optimization` P:3696-3760, 출력 읽기 스레드 P:3800-3850, `_set_opt_success` P:3784)
- Create: `test/test_multi_session_checks.py`

**Interfaces:**
- Produces: 모듈 함수 `check_session_dir(path: str) -> list[str]`(없는 필수 항목 이름 목록), `check_multi_outputs(out_dir: str) -> list[str]`(없거나 빈 결과 파일 목록), `validate_output_name(name: str, map1: str, map2: str) -> str | None`. `set_map1/2` 응답에 `missing: [...]` 추가, `set_output`·`optimize`는 이름이 틀리면 `{"success": false, "error": …}`, `optimization_status`에 `outputs_ok: bool`, `loop_count: int | null` 추가.

- [ ] **Step 1: 실패 테스트**

```python
import pytest

web_server = pytest.importorskip('ros_slam_webui.web_server')


def _make_session(root, with_static=True):
    (root / 'Scans').mkdir(parents=True)
    (root / 'Scans' / '0.pcd').write_bytes(b'x')
    (root / 'optimized_poses.txt').write_text('0 0 0 0 0 0 0 1\n')
    (root / 'edges.txt').write_text('0 1 ' + ' '.join(['0'] * 12) + '\n')
    if with_static:
        (root / 'StaticMap.pcd').write_bytes(b'x')


def test_session_dir_ok(tmp_path):
    _make_session(tmp_path / 's1')
    assert web_server.check_session_dir(str(tmp_path / 's1')) == []


def test_session_dir_missing_static(tmp_path):
    _make_session(tmp_path / 's1', with_static=False)
    assert web_server.check_session_dir(str(tmp_path / 's1')) == ['StaticMap.pcd']


@pytest.mark.parametrize('name', ['', '.', '..', 'a/b', 'Campus1'])
def test_bad_output_names(name):
    assert web_server.validate_output_name(name, '/x/Campus1', '/x/Campus2')


def test_outputs_missing(tmp_path):
    (tmp_path / 'optimized_poses.txt').write_text('')
    assert set(web_server.check_multi_outputs(str(tmp_path))) == {
        'optimized_poses.txt', 'FirstMap.pcd', 'SecondMap.pcd'}
```

- [ ] **Step 2: 구현** — 필수 항목은 `long_term_mapping.cpp:660-670` 기준: `Scans/`(폴더, 비어 있지 않음), `optimized_poses.txt`, `edges.txt`, `StaticMap.pcd`. `_ground`/`_nonground` 스캔은 `Scans/0_ground.pcd` 하나만 표본 확인(전체 확인은 수천 장이라 생략). 출력 이름 규칙은 Task 2.10의 `MAP_DIR_NAME_RE` 재사용 + `os.path.basename(map1|map2)`와 같으면 거부. 결과 확인은 `optimized_poses.txt`·`FirstMap.pcd`·`SecondMap.pcd`가 있고 크기 > 0.
- [ ] **Step 3: 실패 감지** — 출력 읽기 스레드(P:3804~)에 `if 'LTmapping' in line and 'process has died' in line:` 분기를 추가해 `slam_opt_status = {'running': False, 'done': True, 'success': False, 'message': 'LTmapping 노드가 비정상 종료했습니다'}`(`if self.slam_optimization_process is process` 가드 유지). `Error:`·`Failed to load`로 시작하는 줄은 화이트리스트와 별도로 마지막 오류 문구로 기억해 실패 메시지에 붙인다. `_set_opt_success`는 성공 확정 전에 `check_multi_outputs(<lt_dir>/<output>)`가 빈 목록인지 보고, 아니면 실패로 바꾼다("결과 파일 없음: …"). `Loop Edge Generation Complete. size : N`에서 N을 `slam_opt_status['loop_count']`로 저장.
- [ ] **Step 4: 프론트** — `_pollOptimizationStatus`가 `loop_count`를 `#slam-opt-msg` 옆 데이터로 남기도록 `uiShell.onOptStatus(status)` 훅 한 줄(Task 3.2 `multiLoopCount`는 이 값을 우선). `loadMap1/2` 성공 처리에서 응답 `missing`을 Task 3.2의 확인 칸에 보인다.
- [ ] **Step 5: 검증·커밋** — pytest PASS, `py_compile`, 빌드. **실데이터(사용자)**: Map1에 빈 폴더를 넣고 실행 → "Failed"와 원인 문구(지금은 성공으로 보임). 메인 체크아웃 `02` §6.3에 응답 필드 메모. `fix(api): Multi-Session 노드 비정상 종료·결과 누락을 실패로 판정하고 입력·출력 폴더 검사 추가`

### P3 완료 처리

- [ ] 사용자 실데이터 확인 후 `03` 8.3 체크, `02` §6에 `topic_details`·Recorder 필드 반영 확인. finishing 메뉴.

---

## 10. P4 태스크 — Plot / 3D Viewer (Phase 8.4)

브랜치: `feature/webui-redesign-p4`.

### Task 4.1: Plot 헤더 — 전역 Pause/Resume·시간 창

**Files:**
- Modify: `web/index.html:1162-1209` (헤더 액션), `web/static/ui_shell.js`

**Interfaces:**
- Consumes: `plotState.plotTabManager.tabs[*].plotManager.togglePause(forceState)`(plot_manager.js:2576), `.isPaused`, `updateBufferTime(seconds)`(J:5537).
- Produces: `uiShell.setPlotPaused(paused: boolean)`.

- [ ] **Step 1: 실패 확인** `test/ui/check_viz.py` — Plot 진입 후 `#plot-header-pause-btn` 클릭 → 모든 탭 `plotManager.isPaused === true`, 버튼 글자 `Resume`, 헤더 배지 `Paused`. `#plot-window-seg [data-sec="30"]` 클릭 → `#buffer-time-input.value === '30'`. 기대: FAIL.
- [ ] **Step 2: 마크업** — `#plot-header-actions`에 `<div class="ui-seg" role="radiogroup" aria-label="시간 창" id="plot-window-seg">`(10 s / 30 s / 60 s, `data-sec`)와 `<button class="ui-btn" id="plot-header-pause-btn"><svg …icon-pause…></svg><span class="ui-btn-label">Pause</span></button>`. 기존 Buffer 입력(H:1191-1202)은 `<details class="ui-more"><summary>직접 입력</summary>` 안으로.
- [ ] **Step 3: 구현**

```javascript
    function setPlotPaused(paused) {
        const manager = window.plotState && plotState.plotTabManager;
        if (!manager) return;
        manager.tabs.forEach(tab => {
            if (tab.plotManager && tab.plotManager.isPaused !== paused) tab.plotManager.togglePause(paused);
        });
        uiKit.setBtnLabel(document.getElementById('plot-header-pause-btn'), paused ? 'Resume' : 'Pause');
        uiKit.setStatusBadge(document.getElementById('plot-header-badge'),
            paused ? 'warn' : 'go', paused ? 'Paused' : 'Streaming');
    }
```

Pause 버튼 클릭 → `setPlotPaused(!(현재 활성 탭의 isPaused))`. 탭별 modebar로 따로 멈춘 경우는 렌더 때 "모든 탭이 멈췄으면 Paused, 하나라도 돌면 Streaming"으로 표시. 시간 창 클릭 → `updateBufferTime(sec)` 후 `#buffer-time-input` 값과 `aria-checked` 동기화. 새 탭에도 적용되는지는 `PlotTabManager` 생성 인자(5.0)와 `updateBufferTime` 동작에 달림(미확인) — 검증에서 확인.

- [ ] **Step 4: 검증·커밋** — `node --check`, 빌드, `check_viz.py` PASS. `feat(plot): Plot 헤더 전역 Pause/Resume·시간 창 분할 버튼`

### Task 4.2: Plot 필드 검색

**Files:**
- Modify: `web/static/plot_tree.js` (새 메서드), `web/index.html:1167-1180` (입력칸)

**Interfaces:**
- Produces: `PlotJugglerTree.prototype.filterByText(query: string)` — 경로(소문자)에 `query`가 들어 있는 리프와 그 조상만 보이고 조상은 펼침. 빈 문자열이면 모두 보이고 펼침 상태는 원래대로.

- [ ] **Step 1: 실패 확인** — `check_viz.py`: `page.evaluate`로 `plotState.tree`에 가짜 토픽 구조를 넣을 수 있으면 그것으로, 아니면 `plot_tree.js`를 로드한 빈 페이지에서 `new PlotJugglerTree(container)` + `updateMessageTree`로 두 토픽을 만들고 `filterByText('linear')` 후 보이는 리프 수 확인. 트리 노드 클래스·`dataset.path` 이름은 plot_tree.js:96-150에서 확인해 맞춘다. 기대: FAIL.
- [ ] **Step 2: 구현** — 리프 노드 요소(`dataset.path`가 있는 것)를 순회해 `hidden` 토글, 보이는 리프의 조상 노드는 `hidden=false` + 펼침 클래스. 검색 전 펼침 상태를 `this._expandSnapshot`에 저장해 빈 검색 때 복원.
- [ ] **Step 3: 입력칸** — 토픽 목록 머리 아래 `<input class="ui-input" type="search" id="plot-field-search" placeholder="필드 찾기" aria-label="필드 찾기" oninput="plotState.tree && plotState.tree.filterByText(this.value)">`.
- [ ] **Step 4: 검증·커밋** — `node --check web/static/plot_tree.js`, 빌드, `check_viz.py` PASS. `feat(plot): 토픽 트리 필드 검색 추가`

### Task 4.3: Plot 현재값·탭 바/메뉴 다크화·Plotly 색

**Files:**
- Modify: `web/static/script.js:4294-4311` (plot_data 수신), `web/static/plot_tab_manager.js` (인라인 `cssText` → 클래스), `web/static/plot_manager.js` (컨텍스트 메뉴·빈 상태·레이아웃 색, 카드 현재값), `web/static/ui_theme.css`

- [ ] **Step 1: 현재값** — plot_data 처리부에서 payload의 필드 경로·값 이름을 확인(미확인)한 뒤, 경로별 최신값을 `plotState._latestValues`(Map)에 넣고 250ms 간격으로 `plotState.tree.updateValue(path, value)`를 일괄 호출(타이머 하나). 각 그래프 카드 머리(PlotManager가 만드는 제목 영역)에 오른쪽 정렬 `ui-mono` 현재값 요소를 만들고 같은 타이머에서 첫 trace 최신값으로 갱신. 일시정지 중엔 갱신하지 않고 "일시정지" 글자 표시.
- [ ] **Step 2: 다크화** — `plot_tab_manager.js`의 탭 요소 `style.cssText`(보라 `rgba(124,111,255,…)`)를 `ui-plot-tab`, `ui-plot-tab-active`, `ui-plot-tab-close`, `ui-plot-tab-add` 클래스로 바꾸고 CSS를 `ui_theme.css`에. 같은 파일 407줄의 미정의 `var(--primary)`는 Task 0.4 별칭으로 해결됨. `plot_manager.js` 컨텍스트 메뉴·XY 메뉴의 흰 배경 인라인 스타일과 빈 상태 안내(413-446)를 `ui-context-menu`, `ui-plot-empty` 클래스로.
- [ ] **Step 3: Plotly 색** — 레이아웃 생성부(plot_manager.js:545-556 부근)의 하드코딩 색을 `getComputedStyle(document.documentElement).getPropertyValue('--color-…')`로 한 번 읽은 값으로(배경 `--color-surface`, 그리드 `--color-line`, 글자 `--color-mute`). trace 색 팔레트는 바꾸지 않는다(첫 trace `--color-info`만 맞춤은 선택).
- [ ] **Step 4: 검증·커밋** — `node --check` 3파일, 빌드, `check_viz.py` + 스크린샷 vs `11-plot.png`. **실데이터(사용자)**: Plot 값 실시간 갱신, 일시정지 중 줌. `feat(plot): Plot 현재값 표시·탭 바와 메뉴 다크 토큰 적용`

### Task 4.4: 3D Viewer — 헤더 버튼·Displays 눈 아이콘·Views 정리

**Files:**
- Modify: `web/index.html:1213-1335`, `web/static/threejs_display.js` (Views 렌더 T:764-806, Displays 렌더 T:3234~), `web/static/ui_theme.css`

- [ ] **Step 1: 실패 확인** — `check_viz.py`: 3D 진입 → `#viewer3d-header-actions` 안에 Reset·Snapshot 버튼, Views 패널에 Distance/Yaw/Pitch/Scale 입력이 안 보임, Displays 체크박스에 `aria-label`. 기대: FAIL.
- [ ] **Step 2: 헤더** — 캔버스 위 절대 위치 Reset/Snapshot(H:1216-1233)을 `#viewer3d-header-actions`로 이동(`ui-btn`, 아이콘+라벨). Reset `title`을 "씬 초기화 (구독은 유지)"로 고친다(실제 동작, T:1845-1861). C:4174-4184 `resetViewCamera` 속성 선택자 삭제. 헤더 배지는 Fixed Frame 값("Fixed Frame · map").
- [ ] **Step 3: Displays** — `renderDisplayPanel`이 만드는 표시 체크박스에 `ui-eye-toggle` 클래스와 `aria-label="${topic} 표시"`를 추가(만드는 줄을 T:3261 부근 섹션마다 찾음). CSS `.ui-eye-toggle { appearance: none; width: 20px; height: 20px; mask: url(…eye…) }` + `:not(:checked)`면 eye-off 아이콘. 동작 변화 없음.
- [ ] **Step 3b: Fixed Frame 경고** — FAST_LIO는 `odom → base_link` TF만 내고 `map` 프레임이 없다(13.1절). 3D Viewer 기본 Fixed Frame은 `map`이라 SLAM·Localization 데이터가 안 그려질 수 있다(지금 동작 미확인). 헤더 배지를 "Fixed Frame · map"으로 쓰되, 수집된 TF 프레임 목록(Fixed Frame 콤보가 쓰는 목록, T:6213-6455)에 현재 Fixed Frame이 없고 `odom`이 있으면 배지를 warn 톤 + 글자 "Fixed Frame · map (TF에 없음 — odom 권장)", 클릭하면 `setFixedFrame('odom')`. 기본값 자체는 바꾸지 않는다.
- [ ] **Step 4: Views** — `renderViewsPanel`에서 Distance·Yaw·Pitch·Scale 입력 줄에 `hidden`(T:908-912 값 저장만 하는 항목). 주석 한 줄: `// 카메라에 반영되지 않는 항목은 숨김 (미구현)`.
- [ ] **Step 5: 검증·커밋** — `node --check`, 빌드, `check_viz.py` PASS, 스크린샷 vs `12-3d-viewer.png`. `feat(viewer): 3D Viewer 헤더 Reset/Snapshot·Displays 눈 아이콘·미동작 Views 항목 숨김`

### P4 완료 처리

- [ ] `03` 8.4 체크. finishing 메뉴.

---

## 11. P5 태스크 — 토스트·접근성·반응형 (Phase 8.5)

브랜치: `feature/webui-redesign-p5`.

### Task 5.1: `alert()` shim과 확인 대화상자

**Files:**
- Modify: `web/static/ui_kit.js` (`confirmDialog`, `inferToastTone`, alert shim), `web/static/script.js:977` (`confirm` → `await uiKit.confirmDialog`)

**Interfaces:**
- Produces: `uiKit.confirmDialog({ title, message, okText = '확인', cancelText = '취소', tone = 'warn' }) -> Promise<boolean>`, `uiKit.inferToastTone(text) -> tone`.

- [ ] **Step 1: 실패 확인** `test/ui/check_a11y.py` — `page.on('dialog')`로 네이티브 대화상자가 뜨면 실패 기록. `page.evaluate("alert('Failed to start SLAM mapping: x')")` → 네이티브 대화상자 없음 + `.ui-toast[data-tone="stop"]` 1개. 기대: FAIL.
- [ ] **Step 2: 구현** (`ui_kit.js` 끝)

```javascript
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
```

`confirmDialog`는 `<div class="ui-dialog-backdrop"><div class="ui-dialog" role="alertdialog" aria-modal="true" aria-labelledby=… aria-describedby=…>`를 만들고, 확인/취소/ESC로 `resolve(true/false)` 후 제거. 열릴 때 취소 버튼에 포커스, 닫힐 때 이전 포커스로 복귀.

- [ ] **Step 3: confirm 교체** — J:977 `if (!confirm(msg)) return;` 형태를 `if (!(await uiKit.confirmDialog({ title: '재생할 수 없는 토픽', message: msg, okText: '나머지만 재생' }))) return;`로(정확한 원문은 J:970-980에서 확인). `playBag`은 이미 async(J:947).
- [ ] **Step 4: 검증·커밋** — `node --check` 2파일, 빌드, `check_a11y.py` PASS, `check_smoke.py` PASS. `feat(ui): alert를 토스트로 바꾸는 shim과 확인 대화상자 추가`

롤백: shim 4줄 삭제로 네이티브 alert 복귀.

### Task 5.2: 알림 문구 정리 (명시적 토스트)

**Files:**
- Modify: `web/static/script.js` (성공 알림·블로킹이 불편한 곳부터), `web/static/threejs_display.js`(6개 — 죽은 코드라 손대지 않음), `web/static/plot_tab_manager.js:90`

- [ ] **Step 1: 목록** — `rg -n "alert\(" web/static/script.js web/static/plot_tab_manager.js > /tmp/alerts.txt`. 우선 교체 대상: Recorder 시작/중지(J:2636, 2638), Config 저장(J:2791), Bag 변환 완료(J:1289), 데이터셋 변환 오류(J:4346, 4374, 4402, 4430), 탭 10개 초과(`plot_tab_manager.js:90`).
- [ ] **Step 2: 교체** — 각각 `uiKit.showToast(문구, 'go'|'stop'|'warn')`. 한국어·영어 혼용 문구는 그대로 둔다(문구 번역은 범위 밖). 나머지 alert는 shim이 처리하므로 기계적 교체는 선택 — 하면 `composer` 워커로 일괄.
- [ ] **Step 3: 검증·커밋** — `node --check`, 빌드, `check_smoke.py`. `refactor(ui): 주요 성공·오류 알림을 명시적 토스트로 교체`

### Task 5.3: 모달 공통 모양·ESC·aria

**Files:**
- Modify: `web/static/ui_kit.js` (`enhanceModals`), `web/static/ui_theme.css` (`.modal` 재정의), `web/static/script.js:3149` 부근(Save Map Enter)

- [ ] **Step 1: 실패 확인** — `check_a11y.py`: `openFileBrowser(()=>{}, '~')` 후 `Escape` → `#file-browser-modal` 숨김. 9개 모달에 `role="dialog"`. 기대: FAIL.
- [ ] **Step 2: 구현** — `enhanceModals()`: `document.querySelectorAll('.modal')`에 `role="dialog"`, `aria-modal="true"`, 제목 요소에 id를 붙이고 `aria-labelledby`. `document.addEventListener('keydown', e => { if (e.key !== 'Escape') return; const open = [...document.querySelectorAll('.modal')].find(m => m.style.display && m.style.display !== 'none'); if (open) open.querySelector('.close, [data-dismiss]')?.click(); })` — 옵셔널 체이닝 호출 대신 `const btn = …; if (btn) btn.click();`로 쓴다. Save Map 입력 `#save-map-directory`에 Enter → `confirmSaveMap()`. 바깥 클릭 닫기는 지금 3개만 되는 상태를 유지(변경하면 실수로 닫힘이 늘어남).
- [ ] **Step 3: CSS** — 세 군데 중복 `.modal` 정의(C:418-508, 952-1060, 4070-4100)를 건드리지 않고 `ui_theme.css`에서 `.modal-content`를 raised 배경·6px 반경·`--color-line2` 테두리·최대 폭 560px로 재정의. `block`/`flex` 혼용 보정 규칙(C:~4080)은 그대로.
- [ ] **Step 4: 검증·커밋** — 빌드, `check_a11y.py` PASS, 모달 9개 스크린샷. `feat(ui): 모달 공통 모양·ESC 닫기·dialog 역할 부여`

### Task 5.4: 반응형 4단계와 축소 사이드바

**Files:**
- Modify: `web/static/ui_theme.css` (부록 A.4), `web/index.html` (작은 화면 메뉴 버튼), `web/static/ui_shell.js`

- [ ] **Step 1: 실패 확인** — `check_a11y.py`에 뷰포트 1024×768·1280×800·1440×900·900×700 스크린샷. 1024에서 `.ui-sidebar` 너비 64px, 900에서 사이드바 숨김 + `#ui-sidebar-toggle` 보임. 기대: FAIL.
- [ ] **Step 2: 구현** — 부록 A.4 브레이크포인트 CSS. `<button id="ui-sidebar-toggle" class="ui-icon-btn" aria-label="메뉴 열기" aria-controls="ui-sidebar" aria-expanded="false">`를 `.ui-main` 맨 위에. `ui_shell.js`: 클릭 시 `body.ui-sidebar-open` 토글 + `aria-expanded`, 항목 클릭 시 닫기.
- [ ] **Step 3: 캔버스 확인** — 각 폭에서 SLAM·3D 뷰어 캔버스 크기가 칸을 채우는지(Task 2.6 ResizeObserver, T:551-556).
- [ ] **Step 4: 검증·커밋** — 빌드, 스크린샷 4장 확인. `feat(ui): 1440/1280/1024/작은 화면 반응형과 아이콘 사이드바`

### Task 5.5: 접근성 점검

**Files:**
- Modify: 점검에서 나온 곳(H 위주), `web/static/ui_theme.css`

- [ ] **Step 1: 점검 스크립트** — `check_a11y.py`에 추가: 8화면마다 `button`, `[role=button]` 중 보이는 요소의 접근 이름(글자·`aria-label`·`title`)이 빈 것 목록, `:focus-visible` 외곽선이 있는지(Tab 3번 누른 뒤 `document.activeElement` 스타일), `prefers-reduced-motion: reduce` 에뮬레이션에서 `.ui-run-tag .ui-dot` 애니메이션 없음.
- [ ] **Step 2: 수정** — 나온 목록에 `aria-label` 추가. 부록 A.5 체크리스트를 하나씩 확인.
- [ ] **Step 3: 검증·커밋** — `check_a11y.py` PASS. `fix(ui): 아이콘 버튼 이름표·포커스 링·모션 줄이기 점검 반영`

### Task 5.6 (선택): 디자인 원칙 다이얼로그

- [ ] 사이드바 연결 블록 아래 "디자인 원칙 보기" 버튼 → `uiKit.confirmDialog`가 아닌 단순 `ui-dialog`로 색 토큰 7개·글꼴·배치·상태·알림 원칙(목업 `Shell.tsx` DesignDialog 문구)을 보여 준다. 사용자가 원할 때만. 커밋 `feat(ui): 디자인 원칙 다이얼로그 추가`.

### P5 완료 처리

- [ ] 메인 체크아웃 `01` §12.3을 부록 B.5 문구로 Edit, `03` 8.5 체크. finishing 메뉴.

---

## 12. P6 태스크 — 정리 (Phase 8.6)

브랜치: `feature/webui-redesign-p6`.

### Task 6.1: 뷰어 툴바 SVG → 스프라이트

**Files:**
- Modify: `web/index.html` (H:99-146, 374-391, 439-486, 798-819, 1280-1286의 인라인 `<svg>` 약 25개), 스프라이트에 `icon-home, icon-pin, icon-topview, icon-camera, icon-expand, icon-collapse, icon-layers-edl, icon-diff` 추가

- [ ] **Step 1:** 각 인라인 `<svg>…</svg>`를 `<svg class="ui-icon" aria-hidden="true"><use href="#icon-…"></use></svg>`로. **풀스크린 아이콘처럼 id가 붙은 `<svg>`는 id를 그대로 남긴다**(`savemap-fullscreen-icon-expand` 등 — 핸들러가 `style.display`로 바꿈).
- [ ] **Step 2: 검증** — `check_ids.py dev` PASS(id 그대로), 스모크 스크린샷에서 아이콘 모양 동일, 풀스크린 아이콘 전환(사용자 확인).
- [ ] **Step 3: 커밋** — `refactor(ui): 뷰어 툴바 인라인 SVG를 아이콘 스프라이트로 통합`

### Task 6.2: 인라인 `style=` 정리

**Files:**
- Modify: `web/index.html`, `web/static/ui_theme.css`, `web/static/script.js`(JS 문자열 인라인 스타일: 누적 표 행 J:11478-11492·J:11975-11990, `updateSelectedTopicsDisplay` J:938-941, `showYamlErrorModal` J:3572-3615)

- [ ] **Step 1: 목록** — `rg -n 'style="' web/index.html | wc -l`(기준 146). 남길 것: JS가 `style.display`로 켜고 끄는 요소의 초기 `display:none`(지우면 첫 화면에 보임). 나머지(margin, color, font-size, legend 스와치 색 등)는 클래스로.
- [ ] **Step 2: 옮기기** — 화면 단위로 한 커밋씩(SLAM·Loc·Multi / Data / Viz / 모달). 범례 스와치 색처럼 값마다 다른 것은 `style="--swatch: #ff3333"` + `.slam-legend-dot { background: var(--swatch); }` 형식으로 줄인다.
- [ ] **Step 3: 검증·커밋** — 화면마다 스모크 스크린샷 비교, `check_ids.py dev` PASS. `refactor(ui): 인라인 style을 클래스로 정리 (<화면>)`

### Task 6.3: 죽은 CSS·옛 탭·되돌리기 스위치 제거

**Files:**
- Modify: `web/index.html` (옛 `.topbar`, `.tab-navigation`, `.subtab-navigation`, 숨긴 `<h2>`, 숨긴 `.status` 라벨, 숨긴 Select Topic 버튼·`Set Output` 버튼·`#dataset-format-select`), `web/static/style.css`, `web/static/script.js`, `web/static/ui_shell.js`

- [ ] **Step 1: 참조 확인** — 지울 id마다 `rg -n "<id>" web/`가 HTML 정의 1곳만 나와야 한다. JS가 아직 쓰는 id(예: `lidar-slam-status`, `rosbridge-status-chip`, `latency-indicator`, `ros-domain-chip`, `#dataset-format-select`)는 **JS 쓰기 줄을 먼저 지우거나 uiShell 훅으로 대체한 뒤** 지운다. `#dataset-format-select`는 `onDatasetFormatChange`가 값을 읽으므로 **남긴다**(숨김 유지).
- [ ] **Step 2: 옛 탭** — `.tab-navigation`·`.subtab-navigation` 버튼을 지우면 `openSubTab`의 버튼 찾기(J:183)와 `openTab`의 버튼 찾기(Task 1.4)가 아무것도 못 찾을 뿐 동작은 유지된다. 두 찾기 코드도 같이 지운다. `ui_shell.js`의 `LEGACY_NAV_KEY`·`isLegacyNav` 제거, `ui-has-sidebar` 클래스는 H `<body>`에 직접.
- [ ] **Step 3: CSS** — 남은 `button[onclick*=…]` 규칙 전부, `.tab-button`·`.subtab-*`·`.topbar`·`.chip*`·`.brand-*` 규칙, 첫 `:root`(C:9-22)에서 별칭으로 덮이는 변수, `DESIGN OVERRIDE`의 글꼴 강제 규칙 중 `ui_theme.css`와 겹치는 것을 지운다. 지울 때마다 스모크 스크린샷 비교.
- [ ] **Step 4: 검증·커밋** — `check_ids.py dev <삭제 허용 id들>` PASS, 전체 `check_*.py` PASS. `refactor(ui): 옛 탭·상단 칩·속성 선택자 CSS 등 리디자인 이전 코드 제거`

### Task 6.4 (선택, 권장하지 않음): File Player 5개 블록 DOM 통합

- 이점: 마크업 약 120줄 감소. 위험: id 접두사(`kitti-`·`kaist-`·`mulran-`·`helipr-`·`conpr-`)를 쓰는 JS 함수·WS 핸들러·alert 약 40곳을 함께 바꿔야 한다. P3에서 이미 같은 모양이라 사용자에게 보이는 차이가 없다. **하지 않는 것을 기본으로** 하고, 사용자가 원할 때 별도 계획으로 뺀다.

### Task 6.5: 문서 마무리

- [ ] 메인 체크아웃 `02` §9.1 파일 구조에 `ui_theme.css`, `ui_kit.js`, `ui_status_logic.js`, `ui_shell.js`, `test/ui/` 추가(줄 수 숫자는 적지 않음), §2.1.1의 "Stop SLAM / 맵 저장 시 뷰어 숨김" 설명에 "Config는 읽기 전용 유지" 반영.
- [ ] `03` 8.6 체크, Phase 8 전체 ✅, "현재 구현 상태 요약" 표와 마지막 업데이트 날짜·문서 버전 갱신.
- [ ] (선택) README 스크린샷 교체 — 사용자 요청 시.
- [ ] 커밋(코드 저장소에 들어가는 것만): `docs: Field Console 리디자인 완료 반영`

---

## 13. ROS 패키지 대조 결과 (2026-10-16)

웹 UI가 띄우고 보여 주는 세 패키지의 README·launch·config·노드 소스를 읽고, `web_server.py`·`script.js`가 이들을 어떻게 실행·구독·해석하는지 맞춰 봤다. 노드를 실행하지 않고 코드만 읽었다.

경로 약칭: `FL/` = `FAST_LIO_Localization_and_Mapping/`, `LM` = `FL/src/laserMapping.cpp`, `LL` = `FL/src/laserLocalization.cpp`, `PGO` = `pose_graph_optimization/src/poseGraphOptimization.cpp`, `SAVE` = `pose_graph_optimization/src/saveMapData.cpp`, `LT` = `long_term_mapping/src/long_term_mapping.cpp`(`main`, 커밋 a52ddc9). `P`·`J`는 위와 같다.

### 13.1 SLAM 실행·Live Viewer·Analytics (FAST_LIO + PGO)

| 항목 | 계획의 가정 | 실제 (근거) | 반영한 수정 | 미확인 |
|---|---|---|---|---|
| 실행되는 노드 | `mapping.launch.py` = FAST_LIO | FAST_LIO `fastlio_mapping` + **`pose_graph_optimization` `posegraphoptimization`** 두 노드. rviz는 주석 처리(`FL/launch/mapping.launch.py:58-75`). Localization launch는 FAST_LIO + `octomap_server` | 2.3절 L12, Start 툴팁(G12) | — |
| `is_running` 의미 | SLAM이 돌고 있음 | launch 프로세스 생존만 봄(P:3932). 노드 하나만 죽어도 true | 3.1절 `stale`·센서 입력, Task 2.11, 리스크 7 | 노드만 죽였을 때 launch가 정말 살아 있는지 |
| Live 토픽 출처 | FAST_LIO | `/cloud_registered`·`/Odometry`·`/tf`는 FAST_LIO(`LM:1234,1237,925`). **`/PGO_map`·`/PGO_path`·`/kf_node`·`/loopLine`은 PGO**(`PGO:998-1004`) | 2.2절 S18 | — |
| `/PGO_map` | 누적 전체 지도 | 최신 키프레임 **50 m 안** 로컬 지도, voxel 0.4, 디스크에서 Scans를 다시 읽어 계속 재발행(`PGO:773-815`) | S18 범례 "지도: 최근 50 m" | 실제 발행 주기 |
| LIO 선·루프 선 | 최적화 반영 | `/kf_node`·`/loopLine`은 최적화 **전** 자세(`PGO:552-556, 712-720`), `/PGO_path`만 최적화(0.5 Hz 전체 재발행) | S18 범례 "LIO (원래 자세)" | — |
| 프레임 | `map` 존재 | 모든 토픽 `odom`, TF는 `odom → base_link`뿐 | Task 4.4 Step 3b(3D Viewer Fixed Frame 경고) | 3D Viewer가 `map`일 때 실제로 안 그려지는지 |
| Analytics 토픽 | `/lio_analytics`, `/loc_analytics` | 맞음(`LM:1240`, `LL:1599`). 브라우저가 rosbridge로 구독, 스캔마다 발행 → UI가 10 Hz throttle | 변경 없음 | — |
| 필드 수 | Full Statistics 27/31 | 메시지 필드는 **47(Lio)/49(Loc)**, UI가 쓰는 건 약 20/23. 27·31은 UI 표시 항목 수 | Task 2.4 문구 "27 항목(필드 47개)" | — |
| CPU·RAM | 시스템 CPU | **fast_lio 프로세스** CPU ÷ 코어 수(첫 값 0), RAM은 VmRSS MB(`LM:1099-1137`) | Task 2.4 라벨 "CPU (fast_lio)" | — |
| Hz | 센서 주기 | 콜백 수 / **1초 벽시계** → bag 배속이 1×가 아니면 달라짐(`LM:1485-1495`) | Task 2.4 툴팁, 3.1절 "IMU/LiDAR 끊김"(0일 때) | — |
| 거리·시간·처리 시간 | 단순 누적 | 거리는 속도 ≥ 0.2 m/s일 때만, run_time은 벽시계, `total_time`은 한 주기 전체(세 단계 합 아님), mean/max는 실행 전체 누적 | Task 2.4 툴팁·범례 | — |
| DOP | 항상 계산 | `dop_flag`가 꺼지면 0 고정(`LM:1538`) | Task 2.4 "DOP 계산 꺼짐" 글자 | 현재 yaml의 `dop_flag` 값 |
| 키프레임 수 | 계획에 없음 | `LioAnalytics` Keyframe 절에 있음 | 3.1절 `kfCount` → Save Map 잠금 | 필드 이름(`kf_count` 추정)과 UI 값 요소 id |
| 건강 신호 | 계획에 없음 | 크래시 heartbeat·"IMU lost" 로그 없음. 경고는 `No point, skip this scan!`(`LM:1304`), `lidar loop back`(`LL:694`), `Map file cannot open.`(`LL:1548`) 등. stderr 로그 파일은 쓰기만 하고 안 읽음, stdout은 `DEVNULL`(P:3501-3505) | G11, Task 2.10 로그 API, Task 2.11 | `cout`/`cerr` 줄이 어느 파일로 가는지 |
| Start의 부작용 | 없음 | `pkill -9 -f rviz2`(P:8209) | G12 툴팁 | — |
| 지도 저장 on exit | — | `pcd_save_en`은 yaml에서 안 읽혀 종료 시 저장 안 함(`LM:99, 1617-1623`) | 기록만 | — |

### 13.2 Save Map (pose_graph_optimization)

| 항목 | 계획의 가정 | 실제 (근거) | 반영한 수정 | 미확인 |
|---|---|---|---|---|
| 서비스 제공자 | FAST_LIO | **PGO**의 `save_trajectory`(`pose_graph_optimization/srv/SaveMap`, 요청 `directory_name`) + `cancel_save_trajectory`(Trigger)(`PGO:1049-1057`) | 13장 기록, 저장 위치 안내 | — |
| 저장이 하는 일 | 최적화 + 동적 객체 제거 | 최적화는 **다시 안 돌림**(그 시점 iSAM2 결과 스냅샷, `SAVE:175-195`, 미연결 `runBatchLoopClosureAndOptimization`). DUFOMap 방식 동적 제거 → PatchWork++ 지면 분리 → cell-plane 이상치 제거 4단계(`SAVE:510-781`) | 오버레이 문구(최적화 언급 빼고 "지도 생성 중") | 저장 직전 몇 초 기다려야 최신 루프가 반영되는지 |
| 결과 폴더 | 계획 레이어 3개 | `pose_graph_optimization/<이름>/`: `LioMap/OptimizedMap/StaticMap/StaticNGMap/StaticGroundMap.pcd`, `odom_poses.txt`, `optimized_poses.txt`, `edges.txt`(오도메트리+루프), `session_meta.yaml`, `Scans/`(프레임당 4파일). 예시 `parking05` 308프레임 523 MB | S19(툴팁), S21(Localization 안내) | — |
| 레이어 의미 | StaticMap = 동적 제거 | 동적 제거 + 지면 분리 + 이상치 제거, **지면 포함** | S19 칩 툴팁 | — |
| 진행률 | 얻을 수 없음 추정 | 단계별 `Void mapping i/N` 등은 **printf → `DEVNULL`**, 서비스 응답은 끝에 한 번. ROS 로그는 시작·끝뿐 | 무한 진행 바 + 경과 시간(Task 2.3) | 진행률이 꼭 필요하면 PGO에 진행 토픽 추가(별도 작업) |
| 저장 중 SLAM | 계속 실행 | FAST_LIO는 계속 돎. PGO는 **단일 스레드 executor**라 저장 동안 `/key_frame` 콜백이 멈춤(LCD·최적화·viz 스레드는 계속)(`PGO:1059`) | 3.1절 안내 문구 "새 키프레임이 추가되지 않습니다" | KeepLast(30)를 넘어 실제로 키프레임이 빠지는지 |
| 저장 후 | "Map Saved · SLAM 계속 실행 중" | 맞음. 두 번째 저장 가능. 같은 이름이면 `rm -r` 후 재생성, 다른 이름이면 Scans 전체 재복사 | S15 덮어쓰기 확인 | — |
| Cancel | 저장 취소 | 플래그·서비스는 있지만 **단일 스레드라 저장이 끝난 뒤에야 처리**, 다음 저장 시작 때 리셋되어 무효(`SAVE:166-168, 826, 918-925`). UI Cancel은 Python 대기만 끝냄 | 버튼 "기다리기 중단"(확정), 진짜 취소는 부록 E 백로그 B1, 리스크 9 | 저장 중 Cancel 뒤 새 저장이 앞 저장 뒤에 줄 서는 실제 동작 |
| 이름 검증 | 없음 가정 | 이름이 그대로 `system("exec rm -r <root><name>/")`(`SAVE:855-857`). `..`·쉘 문자·`Map`(라이브 스테이징) 위험 | S15, Task 2.10, 리스크 6 | — |
| 키프레임 0개 | — | `keyframeTimesSnapshot_.back()` 빈 벡터 접근(`SAVE:433`) | S17 Save Map 잠금 | — |
| Start 시 | — | PGO가 시작마다 `<pkg>/Map/` `rm -r`(`PGO:819-856`) | S16 힌트·Stop 확인 | — |
| 결과 복원 | 새로고침 시 복원 | 허용 목록이 `slam_last_saved_dir` 하나(P:9192-9199) → 서버 재시작 뒤 복원 불가, `startswith` 비교 | S20 안내, 부록 D 보안 메모 | — |

### 13.3 Localization (FAST_LIO `laserLocalization`)

| 항목 | 계획의 가정 | 실제 (근거) | 반영한 수정 | 미확인 |
|---|---|---|---|---|
| 초기화 판정 | `is_initialized` 배지를 읽음 | `initial_flag` = `dop_ratio(= down_dop / matching_dop) ≥ 0.8`이면 true. 그 전엔 EKF 갱신을 버리고 속도 0(`LL:1367, 1377-1390`) | 3.2절 설명, "Not Init" 10초 힌트 | 실제 초기화까지 걸리는 시간 |
| 초기 위치 | 화면 동작 필요 여부 미정 | **지도 원점(항등 자세)에서 스스로 시도**. `/initialpose`를 받으면 그 자세로 재초기화(`LL:752-829`), 웹 UI는 보낼 방법 없음. `init_pose` 파라미터·재초기화 서비스 없음 | Task 2.12(확정 2026-10-16) | `/initialpose`의 frame_id 검사 여부 |
| 지도 경로 | `map_file_path` 표시 | 맞음. 단 **FAST_LIO 소스 폴더 기준 상대 경로**(`ROOT_DIR + map_file_path`, `LL:1544`). 없으면 "Map file cannot open." 후 종료 | L8, Task 2.8 렌더(`FAST_LIO/` 접두), `lastStartFailed` | 노드 종료 뒤 launch(octomap 포함) 생존 여부 |
| Map Update 러그 | `map_updated` | `initial_flag && dop_ratio < 0.6`(`LL:1785-1790`). msg 주석 "> 1.4"는 낡음 | L10 툴팁 | — |
| 매칭 점수 | `match_ratio` | 맞음 = `num_feats / down_size` | 변경 없음(KPI 기준 0.5는 임의) | 적절한 경고 기준 |
| 휠 오도메트리 | `linear_velo` | `common.odom_topic`(yaml `/scout_base_controller/odom`) 구독, 하지만 `odom_mode: 0`이면 값을 안 채움(`LL:880-882`) → 항상 0 | L9, Task 2.8 KPI 분기 | — |
| 외부 입력 | — | `/tf_static` 첫 변환을 외부 파라미터로 씀(`LL:1591`) | 기록만 | — |

### 13.4 Multi-Session (long_term_mapping)

| 항목 | 계획의 가정 | 실제 (근거) | 반영한 수정 | 미확인 |
|---|---|---|---|---|
| **코드 상태** | 하나의 코드 | 디스크 소스 `main`(2026-10-16 16:49 `dev2→main` 체크아웃) ≠ **설치 바이너리(2026-10-14 빌드, ELite 변종)** ≠ `dev2`. 설치 바이너리에는 화이트리스트 8문장이 0건이고 결과도 `LifelongMap.pcd` 등 | Task 3.0(`main` 재빌드 확정), 리스크 8 | 설치 바이너리를 `main` params로 돌렸을 때 동작 |
| 실행 방식 | `map1/map2/output` 인자 | 인자 없음. 웹 서버가 `long_term_mapping/config/params.yaml`의 `directory1/2`, `output_directory`를 `yaml.dump`로 덮어씀(P:3896-3923, 주석 사라짐) | M12 기록 | — |
| 입력 | 폴더 | 폴더 = Save Map 결과 그대로: `Scans/<k>.pcd`·`_ground`·`_nonground`, `optimized_poses.txt`(8열), `edges.txt`(14열), `StaticMap.pcd`(`LT:660-670`) | M11, Task 3.11 `check_session_dir` | — |
| 출력 | 폴더 이름 | `long_term_mapping/<이름>/`, 시작 때 `rm -r <out>`, `/Debug`, `/Scans`(`LT:630-641`). 가드 없음 → 입력과 같은 이름·`.`·`..` 위험 | M10, Task 3.11, 리스크 6 | — |
| 진행 메시지 순서 | 미확인 | `main`에서 1→8 순서 확인(`LT:1431-1463`). 그 뒤 `Completion message published.`, 노드 정상 종료 | 3.3절 | 단계별 소요 시간 |
| 4단계 묶음 | 목업 4단계 | "Map Update"는 저장이 아니라 **변화 검출(PD/ND/UE) + 정적 지도** → 5단계로 나눔. `Loop Edge Generation Complete. size : N`에 루프 수 | 3.3절, Task 3.1(5단계·`parseLoopCount`) | — |
| 실패 판정 | 반환 코드로 충분 | `ros2 launch`는 노드가 죽어도 0 → 서버가 성공 처리(P:3831-3847). `process has died`·`std::cerr` 오류는 버려짐 | M9, Task 3.11, 리스크 7 | 실제 실패 시 UI 표시(실행 확인 필요) |
| 루프 0개 | — | 오류 없이 진행, 세션2는 전역 정렬 위치에 머묾. KISS-Matcher 실패도 조용히 항등 변환 | 3.3절 "루프 0개" warn | `num_exclude_recent: 0`이라 세션 사이 루프가 적게 나오는지 |
| 결과 파일 | 계획 레이어 | `FirstMap/SecondMap.pcd`(+Ground/NonGround), `StaticMap.pcd`, `optimized_poses.txt`, `edges.txt`, `Scans/`, `Debug/PD·ND·FirstUE·SecondUE.pcd`(**비어 있으면 파일 없음**), `Debug/KissMatchedMap.pcd` | M7 유지, M13 제안 | — |
| PD/ND/UE 의미 | 레이어 이름만 | FirstUE = 세션1 점 중 세션2가 못 본 1.5 m 셀(README의 2 m와 다름), ND = 세션1에만 있는 구조(거리² > voxel → 약 0.63 m), PD = 세션2에만 있는 구조. 항상 계산 | 칩 툴팁에 한 줄씩(Task 3.3 Step 4에 추가 적용) | — |
| Check Difference | 노드 기능 | UI 토글일 뿐(이미 생성된 Debug PCD를 읽음, J:10518) | 변경 없음 | — |
| Cancel | 부분 파일 | SIGKILL, 정리 로직 없음. 단계별로 `optimized_poses.txt`·`FirstMap*`·Debug 일부가 남음. 다음 같은 이름 실행이 지움 | 3.3절 취소 토스트 문구 | — |
| Localization 연결 | — | 없음. 결과를 쓰려면 경로 수동 지정 필요 | 기록만(S21과 같은 문제) | — |

### 13.5 계획 밖이지만 별도 작업으로 제안하는 것

아래 항목은 이 계획의 단계(P0~P6)에 넣지 않는다. 우선순위·담당은 부록 E 백로그에서 관리한다.

| 제안 | 이유 | 대상 패키지 |
|---|---|---|
| PGO에 MultiThreadedExecutor + 저장/취소용 callback group | 저장 중 키프레임 수신·진짜 취소 | pose_graph_optimization |
| PGO 저장 진행을 토픽(예: `/save_map_progress`)으로 발행 | 진행률 % 표시 | pose_graph_optimization |
| PGO·LTmapping의 폴더 이름 가드(노드 쪽) | 웹 UI 밖에서 호출돼도 데이터 보호 | 두 패키지 |
| 결과 경로 허용 목록을 "패키지 아래 하위 폴더"로 바꾸고 `os.path.commonpath`로 비교 | 서버 재시작 뒤 복원, 접두 비교 허점 | ros_slam_webui |
| "이 지도로 Localization" 버튼(StaticMap 복사 + `map_file_path` 갱신) | Save Map → Localization 흐름 끊김 | ros_slam_webui + FAST_LIO |
| `recorder_recording`을 프로세스 `poll()`로 갱신 | 녹화 프로세스가 죽어도 REC로 남음 | ros_slam_webui |
| `kill_slam_processes`의 `pkill -9 -f rviz2` 범위 축소 | 사용자 rviz까지 종료 | ros_slam_webui |

### 13.6 미확인 (실행해 봐야 아는 것)

1. Save Map 실제 소요 시간과, 저장 중 FAST_LIO 키프레임(KeepLast 30)이 실제로 빠지는지.
2. 저장 중 Cancel 뒤 새 저장을 누르면 앞 저장 뒤에 줄 서는지, UI 상태가 어떻게 보이는지.
3. `fastlio_mapping`이나 PGO 노드만 죽었을 때 launch 프로세스가 살아 있는지(`is_running` 값).
4. FAST_LIO의 `cout`/`cerr` 줄과 RCLCPP 로그가 `ros2 launch` 아래에서 stdout(버려짐)과 stderr 로그 중 어디로 가는지 → 로그 패널에 무엇이 보일지.
5. `LioAnalytics`의 키프레임 수 필드 이름과 대시보드 값 요소 id.
6. Localization 지도 파일이 없을 때 octomap_server 때문에 launch가 살아 있는지.
7. `/initialpose`의 frame_id 검사 여부, `odom_mode 0`에서 재초기화 결과.
8. 3D Viewer Fixed Frame이 `map`일 때 SLAM 데이터가 실제로 안 그려지는지.
9. `long_term_mapping` 재빌드 뒤 화이트리스트 메시지·`[LTmapping]:` 로거 이름이 그대로 나오는지, 단계별 소요 시간.
10. 입력 폴더가 비었을 때 LTmapping이 크래시하는지(빈 클라우드가 KISS-Matcher·SOLiD에 들어가는 경우).
11. 매칭 점수 경고 기준(0.5)과 "Not Init" 힌트 시간(10초)의 적절한 값.

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

---

## 부록 B. 브랜치·검증·문서

### B.1 브랜치와 워크트리 (룰 `01` §7.6, §8.2)

- 단계마다 `dev`에서 새 브랜치: `feature/webui-redesign-p0` … `feature/webui-redesign-p6`. 앞 단계가 `dev`에 들어간 뒤 다음 단계를 만든다. 단계별 브랜치라 한 단계만 되돌리기 쉽다.
- 워크트리: `.worktrees/feature/webui-redesign-pN` (이미 `.git/info/exclude`에 `.worktrees/` 있음, `.gitignore` 수정 금지).
- 빌드: 5장 "표준 검증" 1번 명령에서 `p0`을 바꿔 쓴다. 산출물은 `build_wt/`·`install_wt/`.
- finishing: 기준 `dev`. merge·push·PR은 사용자가 고른 것만. `main` 반영은 명시적 요청 때만(§8.3). force push 금지.
- `.cursor/` 룰 갱신은 워크트리가 아니라 메인 체크아웃 `/home/kkw/localization_ws/src/ros_slam_webui/.cursor/rules/`에서 Edit로.

### B.2 태스크 단위 커밋 계획 (실행하지 않음)

| 단계 | 커밋 메시지 (순서대로) |
|---|---|
| P0 | `docs: Field Console UI 리디자인 스펙·구현 계획 추가` / `test(ui): Playwright 스모크·id 보존 검사 도구 추가` / `feat(ui): Fira Sans·Fira Code 글꼴 로컬 vendor 추가 (OFL)` / `feat(ui): 디자인 토큰 ui_theme.css 추가 및 기존 CSS 변수 별칭 연결` / `feat(ui): ui-* 공용 컴포넌트 클래스(버튼·패널·KPI·배지·단계 표시줄 등) 추가` |
| P1 | `feat(api): 사이드바 실행 표시용 GET /api/system/status_summary 추가` / `feat(ui): 상태 파생 순수 함수 ui_status_logic.js 추가 (사이드바 표시)` / `feat(ui): ui_kit.js 헬퍼(상태 배지·단계 표시줄·버튼 잠금·토스트) 추가` / `fix(ui): openTab/openSubTab 전역 event 의존 제거 (프로그램 호출 지원)` / `feat(ui): 왼쪽 사이드바 셸과 showScreen 추가 (옛 탭은 숨김, legacyNav 스위치)` / `feat(ui): 사이드바 연결 블록(rosbridge·지연·DOMAIN·포트·PC2 WS 상태) 추가` / `feat(ui): 전역 상태 2초 폴링으로 사이드바 RUN/REC/PLAY/OPT 표시` / `feat(ui): 8개 화면 sticky 헤더 뼈대(그룹·제목·상태 배지) 추가` |
| P2 | `feat(ui): SLAM·Localization 상태 파생 함수 추가` / `feat(slam): LiDAR SLAM 헤더 액션·상태 배지·단계 표시줄과 버튼 잠금` / `feat(slam): LiDAR SLAM 지도 8/정보 4 배치·빈 지도 칸·저장 오버레이 적용` / `feat(slam): Analytics를 오른쪽 칸 KPI·스파크라인 배치로 재구성, 차트 자동 리사이즈` / `feat(slam): 실행 중 Config 패널을 숨기지 않고 읽기 전용으로 표시` / `feat(slam): Save Map 결과 뷰어 툴바·범례 칩 재스타일, 크기 자동 맞춤 추가` / `fix(slam): Live 뷰어 Follow 초기 표시·풀스크린 아이콘·Reset 버튼 상태 동기화` / `feat(slam): Localization 헤더·단계 표시줄·8/4 배치, Stop 직후 뷰어 재출현 방지` / `style(viewer): Three.js 뷰어 배경을 디자인 토큰 #07090C로 통일` / `feat(api): Save Map 이름 검증·존재 확인과 노드 stderr 로그 조회 API 추가` / `feat(slam): 데이터 끊김·센서 끊김·Localization 시작 실패를 헤더 배지로 표시` / `feat(slam): Localization 뷰어에서 초기 위치를 찍어 /initialpose 발행` |
| P3 | `feat(ui): Multi·Bag·Recorder·File 상태 파생 함수 추가` / `feat(slam): Multi-Session 단일 입력 폼·헤더 실행 버튼·진행 5단계 표시` / `fix(slam): Multi-Session 결과 자동 복원 시 Exit 표시·새로고침 후 진행 폴링 재개, 결과 레이어 칩` / `feat(api): bag 정보에 토픽별 Type·메시지 수(topic_details) 추가` / `feat(player): Bag Player 토픽 선택을 모달 대신 표(Type·Hz·Msgs)로 표시` / `feat(player): Bag Player 하단 고정 재생 바·속도 분할 버튼·상태별 잠금` / `style(player): Bag 형식 변환 패널·bag 정보 태그 재배치` / `feat(api): Recorder 상태에 경과 시간·파일 크기 추가` / `feat(recorder): Bag Recorder 큰 Record 버튼·경과 시간·크기·토픽 표, 새로고침 복원` / `feat(player): File Player 데이터셋 분할 버튼·통일된 저장 폼·하단 재생 바` / `fix(api): Multi-Session 노드 비정상 종료·결과 누락을 실패로 판정하고 입력·출력 폴더 검사 추가` |
| P4 | `feat(plot): Plot 헤더 전역 Pause/Resume·시간 창 분할 버튼` / `feat(plot): 토픽 트리 필드 검색 추가` / `feat(plot): Plot 현재값 표시·탭 바와 메뉴 다크 토큰 적용` / `feat(viewer): 3D Viewer 헤더 Reset/Snapshot·Displays 눈 아이콘·미동작 Views 항목 숨김` |
| P5 | `feat(ui): alert를 토스트로 바꾸는 shim과 확인 대화상자 추가` / `refactor(ui): 주요 성공·오류 알림을 명시적 토스트로 교체` / `feat(ui): 모달 공통 모양·ESC 닫기·dialog 역할 부여` / `feat(ui): 1440/1280/1024/작은 화면 반응형과 아이콘 사이드바` / `fix(ui): 아이콘 버튼 이름표·포커스 링·모션 줄이기 점검 반영` / (선택) `feat(ui): 디자인 원칙 다이얼로그 추가` |
| P6 | `refactor(ui): 뷰어 툴바 인라인 SVG를 아이콘 스프라이트로 통합` / `refactor(ui): 인라인 style을 클래스로 정리 (<화면>)` × 4 / `refactor(ui): 옛 탭·상단 칩·속성 선택자 CSS 등 리디자인 이전 코드 제거` / `docs: Field Console 리디자인 완료 반영` |

### B.3 `03.task_plan_rules.mdc` 추가 제안 문구 (P0 완료 때 Edit)

"현재 구현 상태 요약" 표 Phase 7 줄 다음:

```markdown
| Phase 8.0~8.6 | Field Console UI 리디자인 (사이드바 셸·화면 헤더·상태 배지/버튼 잠금·토스트·반응형) | ⚠️ 진행 중 |
```

"## 미시작 Phase" 앞:

```markdown
### ⚠️ Phase 8: Field Console UI 리디자인

**목표**: 승인된 목업 디자인을 바닐라 JS 구조 그대로 적용한다 (Phase 4.12 다크 테마를 대체).
**계획**: `docs/superpowers/plans/2026-10-16-webui-redesign.md` / **스펙**: `docs/superpowers/specs/2026-10-16-webui-redesign-design.md`
**원칙**: id·onclick 핸들러 이름 보존, 상태 플래그는 읽기 전용, 구독 시점 불변.

- [ ] **8.0** 디자인 토큰·Fira 글꼴·`ui-*` 컴포넌트 CSS (`ui_theme.css`)
- [ ] **8.1** 사이드바 셸·연결 블록·화면 헤더, `GET /api/system/status_summary`
- [ ] **8.2** LiDAR SLAM / Localization 헤더·단계 표시줄·8/4 배치, Localization Stop 직후 재출현 방지
- [ ] **8.3** Multi-Session 단일 폼, Bag Player 토픽 표·재생 바, Recorder 경과 시간·크기, File Player 분할 버튼
- [ ] **8.4** Plot 전역 Pause·필드 검색·현재값, 3D Viewer Displays·헤더 버튼
- [ ] **8.5** alert → 토스트, 모달 접근성, 반응형
- [ ] **8.6** SVG 스프라이트·인라인 style 정리·리디자인 이전 코드 제거

**완료 기준**: 단계마다 빌드 성공 + Playwright 8화면 스크린샷·콘솔 새 에러 0 + 실데이터 항목 사용자 확인.
**의존성**: 없음 (Phase 4.12 디자인을 대체)
```

"전체 일정 요약" 표에도 `| Phase 8.0~8.6 | Field Console UI 리디자인 | 높음 | ⚠️ 진행 중 |` 한 줄.

### B.4 `02` §2.2 갱신 제안 (P1 완료 때)

```markdown
### 2.2 탭 구조 규칙
- **화면 전환**: 왼쪽 사이드바(그룹 3개·항목 8개)에서 `showScreen(subtabId)`를 호출한다.
  `showScreen`은 내부에서 `openTab()` → `openSubTab()`을 부르므로 Plot/3D 초기화 등 부수효과는 그대로다.
- `openTab()`/`openSubTab()`은 전역 `event`에 의존하지 않는다 (프로그램 호출 가능).
- 메인/서브 탭 DOM(`.tab-content`/`.subtab-content`)과 id는 유지한다.
- 화면 헤더(배지·버튼 잠금·단계 표시줄)는 `ui_status_logic.js`의 순수 함수가 서버 상태 + 기존 플래그를 읽어 계산한다. 플래그 흐름은 바꾸지 않는다.
```

### B.5 `01` §12.3 갱신 제안 (P5 완료 때)

```markdown
#### 에러 피드백
- `alert()`를 쓰지 않는다. `uiKit.showToast(message, tone)`를 쓴다 (tone: go / info / warn / stop).
- `stop` 톤 토스트는 자동으로 닫히지 않는다. 사용자 선택이 필요하면 `await uiKit.confirmDialog({...})`.
- 버튼 글자를 바꿀 때는 `uiKit.setBtnLabel(btn, text)`를 쓴다 (아이콘 유지).
```

`01` §1.3 HTML 예시(`tab-button`)와 §2.3 예시는 P6 뒤에 사이드바 예시로 바꾸자고 제안만 한다.

---

## 부록 C. 리스크 Top 5와 완화책

| # | 리스크 | 영향 | 완화책 |
|---|---|---|---|
| 1 | **id·onclick 문자열 결합이 깨짐.** id 참조 약 208곳, `button[onclick*=]` 28규칙, 풀스크린 핸들러 id 매칭, `data-layer` 이름, 숨은 체크박스 7개 | 버튼 색 사라짐, 뷰어·대시보드가 조용히 멈춤 | id·핸들러 이름 보존 원칙. `check_ids.py`를 모든 태스크 검증에 포함. DOM은 이동만. 버튼 클래스를 바꾸는 태스크에서 해당 속성 선택자 규칙만 지움 |
| 2 | **SLAM Save Map 플래그 흐름 회귀** (`_slamSaving`·`_slamMapJustSaved`·`_slamLiveViewerHoldOpen`·`_visualHide`) | Live Viewer 재출현, 구독 누수, 결과 뷰어 미표시 | 플래그는 읽기 전용 파생(3.1절). 흐름 코드는 훅 한 줄과 Save 시작 실패 버그 1줄(S14, 승인 후)만. P2 실데이터 체크리스트 6항목을 사용자에게 요청 |
| 3 | **CSS 캐스케이드 충돌** (`!important` 257개, `:root` 2벌, 전역 `span`/`button` 글꼴 강제) | 새 클래스가 안 먹거나 숫자가 Fira Code로 안 나옴 | `ui_theme.css`를 마지막에 로드. P0에서 글꼴 강제 규칙을 변수로 바꾸고 `ui-*` 숫자 클래스를 mono 목록에 추가. 단계마다 8화면 스크린샷 비교 |
| 4 | **레이아웃 변경으로 캔버스 크기 오류** (`SlamResultViewer`에 ResizeObserver 없음, 3D 지연 초기화, 대시보드 Plotly 고정 크기, 러그 캔버스 크기 1회 계산) | 뷰어가 찌그러지거나 0px | Task 2.4·2.6·2.8에서 ResizeObserver 추가. 1024/1280/1440 폭 스크린샷(Task 5.4) |
| 5 | **실데이터 없는 검증 한계 + alert 일괄 교체로 놓치는 오류** | 토픽이 있어야 보이는 회귀를 늦게 발견, 오류가 토스트로 묻힘 | 오류 토스트는 자동으로 안 닫힘. 단계마다 "사용자 확인 필요" 목록 보고. alert은 shim 먼저(되돌리기 4줄) → 주요 문구만 명시적 교체 |

**ROS 패키지 대조로 추가된 리스크 (13장)** — 리디자인이 만든 것은 아니지만, 새 UI가 "상태를 정확히 보여 준다"고 약속하는 순간 드러나는 것들이다. 위 Top 5와 같은 무게로 다룬다.

| # | 리스크 | 영향 | 완화책 |
|---|---|---|---|
| 6 | **이름 입력으로 `rm -r`** — Save Map 이름(PGO), Multi Output 이름(LTmapping)이 검증 없이 삭제 명령에 들어감. `Map`·입력 폴더 이름·`..`이면 데이터 손실 | 실험 데이터 영구 손실 | Task 2.10·3.11의 프론트+서버 이름 검사, 덮어쓰기 확인 대화상자. 노드 쪽 가드는 별도 작업 제안(부록 D) |
| 7 | **실패가 성공으로 보임** — Multi는 `ros2 launch`가 0을 돌려줘 노드 크래시도 "complete", SLAM `is_running`은 launch만 봐 노드가 죽어도 Running | 사용자가 빈 결과를 정상으로 믿음 | Task 3.11(`process has died`·결과 파일 확인), Task 2.11(analytics 끊김 → "데이터 없음") |
| 8 | **Multi 코드 불일치** — 설치 바이너리가 소스와 달라 진행 표시·결과 레이어가 전부 비어 보임 | P3 검증이 엉뚱한 이유로 실패 | Task 3.0(`main` 재빌드)을 P3 첫 태스크로 |
| 9 | **Save Map Cancel이 노드를 멈추지 못함** — 단일 스레드 executor | UI는 "취소됨"인데 폴더가 나중에 생김, 다음 저장이 뒤에 줄 섬 | 버튼 글자 "기다리기 중단" + 안내(확정), PGO 진짜 취소는 부록 E 백로그 B1 |

---

## 부록 D. 미확인 항목, 결정 사항(확정), 셀프 리뷰

### D.1 미확인 항목 (실행 중 확인)

- `ros_slam_webui.web_server`를 pytest에서 import할 때 부작용 유무(Task 1.1 Step 2).
- Jazzy `ros2 bag info`의 실제 줄 형식, ROS1 bag 메시지 수를 rosbags에서 얻는 속성 이름(Task 3.4).
- Multi-Session 단계별 소요 시간, 설치 바이너리를 재빌드했을 때 화이트리스트 메시지가 실제로 출력되는지(소스 순서는 확인됨, 13.4절).
- ROS 패키지 대조에서 생긴 미확인 항목은 13.6절에 따로 모았다.
- Localization 퍼블리셔 수가 Stop 뒤 0이 되기까지 걸리는 시간(보류 3초 값, 3.2절).
- `SlamResultViewer`의 크기 맞춤 메서드 이름(Task 2.6), plot_data payload 필드 이름(Task 4.3), `updateBufferTime`이 새 Plot 탭에도 적용되는지(Task 4.1).
- Recorder 출력 경로를 만드는 정확한 식(Task 3.8), 녹화 프로세스가 죽었을 때 `recorder_recording`이 안 풀리는 문제(범위 밖, R5).
- 변환된 bag을 File Player가 `load_data`로 재생하는지, HeLiPR `sensors` 기본값(범위 밖).
- `update_config` 프론트 호출에 서버 핸들러가 없는 문제(no-op, S2 — 범위 밖, 사용자에게 알림).
- 외부에서 띄운 Localization에 Stop을 누르면 서버가 무엇을 하는지(L5 — 잠금으로 회피).
- `--color-dim` 대비 값, Fira 글꼴 정확한 다운로드 URL.
- `02` §6 문서에 빠진 기존 엔드포인트(`/api/localization/*`, `/api/bag/*`, `/api/recorder/*`, `/api/player/*`, `/api/browse` 등)와 문서에만 있는 `/api/viewer/get_topics` — 이 계획 범위 밖, 별도 문서 정리 작업으로 제안.

### D.2 결정 사항

**확정(2026-10-16)** — 사용자가 권장안으로 확정했다. 갈리던 태스크는 확정안만 남겼다.

| 항목 | 확정 내용 | 반영 위치 |
|---|---|---|
| 글꼴 | Fira Sans / Fira Code woff2를 `web/static/vendor/fonts/`에 넣고 OFL 원문 동봉, 오프라인 동작 | 1.3절 파일 표, Task 0.3, 부록 A.1 |
| Hz 칸 | Bag Player만 bag 정보(`topic_details`의 count / duration)로 Hz 계산. Recorder는 경과 시간·크기만, Hz 칸 없음 | 2.6절 R2, Task 3.4·3.5·3.9 |
| 재생 속도 | 0.5/1/2/4× 분할 버튼 + 기존 0.1 단위 슬라이더를 "세밀 조절"로 접어 유지 | 2.5절 B4, Task 3.6 |

**ROS 패키지 대조에서 생긴 결정 — 확정(2026-10-16)** (사용자가 권장안으로 확정, 탈락안 태스크는 지움)

| 항목 | 확정 내용 | 반영 위치 |
|---|---|---|
| Multi-Session 기준 코드 | `long_term_mapping` 지금 소스(`main`)를 다시 빌드해 기준으로 삼는다. 화이트리스트 8문장·`FirstMap/PD/ND/UE` 결과 그대로 사용, ELite 변종 바이너리는 쓰지 않음 | 3.3절, 2.4절 M4, Task 3.0, 13.4절, 리스크 8 |
| Save Map 취소 | 버튼 글자 "기다리기 중단" + "저장은 노드에서 끝까지 진행됩니다" 안내만. PGO 멀티스레드 진짜 취소는 백로그 | 3.1절, Task 2.3 Step 4, 13.2절, 리스크 9, 부록 E B1 |
| Localization 초기 위치 지정 | 넣는다: Loc 뷰어 "초기 위치 지정" → 위치·방향 드래그 → rosbridge로 `/initialpose` 발행 | 3.2절, 2.3절 L11, Task 2.12, 13.3절 |

남은 사용자 결정 질문: 없음.

함께 알릴 동작 변경(승인 필요): SLAM 저장 중 Stop 잠금, SLAM 미실행 시 Save Map 잠금, 실행 중 Config 숨김 → 읽기 전용, 버그 수정 3건(S14 Save 시작 실패 시 지도 복원, M5 Multi 자동 복원 시 Exit 표시, S11 Live 툴바 표시 동기화), 3D Views의 미동작 입력 4개 숨김.

### D.3 셀프 리뷰 결과

- **2026-10-16 갱신(ROS 패키지 대조 반영)**: 결정 3건 확정, 13장 추가, 상태 매핑(3.1~3.3)·누락 동작(G11-12, S15-21, L8-12, M9-13)·태스크(0.3, 2.1, 2.2, 2.3, 2.4, 2.8, 2.10-2.12, 3.0, 3.1, 3.2, 3.3, 3.11, 4.4)·리스크(6-9)·일정 수정. 새 이름 `noteAnalytics`/`isAnalyticsStale`/`readNumber`(Task 2.2), `locStartFailed`(Task 2.8 선언·2.11 갱신), `parseLoopCount`/`multiLoopCount`(Task 3.1·3.2), `validate_map_dir_name`/`tail_text_file`(Task 2.10), `check_session_dir`/`check_multi_outputs`/`validate_output_name`(Task 3.11)은 정의한 태스크와 쓰는 태스크가 맞는지 확인했다.

- **요구 범위 대조**: ① 아키텍처 결정 → 1장 ② 예시에 없는 동작 → 2장(셸 10·SLAM 14·Loc 7·Multi 8·Bag 8·Recorder 5·File 7·Viz 10) ③ 상태 기계 → 3장 ④ 사이드바 RUN·백엔드 → 4장 + Task 1.1·1.7 ⑤ 로드맵 → 5장 + 6~12장 ⑥ 디자인 사양 → 부록 A ⑦ 브랜치·검증·문서·리스크 → 부록 B·C. 빠진 항목 없음.
- **이름 일관성**: `deriveSlamView`/`deriveLocView`/`deriveMultiView`/`deriveBagView`/`deriveRecorderView`/`deriveFileView`/`deriveSidebarMarkers`/`optDoneCount`/`formatElapsed`, `uiKit.setStatusBadge`/`renderStepper`/`setEnabled`/`setBtnLabel`/`setReadonly`/`showToast`/`renderSparkline`/`confirmDialog`/`inferToastTone`, `uiShell.onScreenChanged`/`refresh`/`onSlamState`/`onLocState`/`onRecorderState`/`onRosbridgeState`/`onLatency`/`onDomainId`/`onBackendWsState`/`setPlotPaused`/`lastSummary` — 정의한 태스크와 쓰는 태스크가 같은 이름을 쓴다. `uiShell.onRecorderState`는 Task 3.9 Step 2에서 쓰고 Step 5 `renderRecorder`와 함께 추가한다.
- **남은 "실행 시 확인" 지시**: 줄 번호·변수 이름처럼 코드에서 바로 확인 가능한 것만 남겼고, 모두 확인 명령(`rg …`)을 같이 적었다. 값을 정하지 못한 것은 D.1에 모았다.
- **2026-10-16 2차 갱신(결정 3건 확정)**: Multi 기준 코드 `main` 재빌드, Save Map "기다리기 중단", 초기 위치 지정 포함으로 확정. 갈리던 문구를 3.1·3.2·3.3절, 2.3절 L11, 2.4절 M4, 5장(P2·P3 규모, 합계 14~18일), Task 2.3·2.12·3.0, 13.2~13.4절, 부록 B.2, 부록 C(리스크 8·9), 부록 D.2에서 확정안으로 바꿨다. 부록 E 백로그를 새로 만들었다.

---

## 부록 E. 백로그 (이 계획 밖 별도 작업)

이 계획의 단계(P0~P6)와 일정(14~18일)에 포함하지 않는다. 진행하려면 항목마다 별도 계획(`docs/superpowers/plans/`)을 쓴다. 근거는 13장.

| # | 항목 | 이유 | 대상 패키지 | 웹 UI 쪽 후속 | 규모(대략) |
|---|---|---|---|---|---|
| B1 | **PGO 진짜 저장 취소** — `rclcpp::spin(nh)`를 MultiThreadedExecutor로 바꾸고 `save_trajectory`·`cancel_save_trajectory`·`/key_frame` 구독을 서로 다른 callback group에 둔다. 취소 시 부분 저장 폴더 정리 (확정 2026-10-16: 백로그) | 지금은 저장 중 취소 요청이 저장이 끝난 뒤에야 처리되어 무효이고, 저장 동안 키프레임 수신도 멈춤(`poseGraphOptimization.cpp:1059`, `saveMapData.cpp:166-168, 826, 918-925`) | pose_graph_optimization | 버튼 글자를 "기다리기 중단" → "Cancel"로 되돌리고, 취소 완료 응답을 기다려 배지를 "Cancelled"로 | 1일 + 웹 UI 0.5일 |
| B2 | PGO 저장 진행을 토픽(예: `/save_map_progress`, 단계·i/N)으로 발행 | 진행률 % 표시 불가(printf → `DEVNULL`) | pose_graph_optimization | 저장 오버레이를 무한 진행 바 → 단계·% 바로 | 0.5일 + 0.5일 |
| B3 | PGO·LTmapping의 폴더 이름 가드(노드 쪽) | 웹 UI 밖에서 서비스를 호출해도 `rm -r` 데이터 손실 방지(Task 2.10·3.11은 웹 UI 경로만 막음) | pose_graph_optimization, long_term_mapping | 없음 | 0.5일 |
| B4 | 결과 경로 허용 목록을 "패키지 아래 하위 폴더"로, `os.path.commonpath` 비교 | 서버 재시작 뒤 결과 복원, 접두 비교 허점(P:9192-9199) | ros_slam_webui | S20 안내 문구 제거 | 0.5일 |
| B5 | "이 지도로 Localization" 버튼(StaticMap.pcd를 FAST_LIO `Map/`으로 복사 + `map_file_path` 갱신) | Save Map → Localization 흐름 끊김(S21) | ros_slam_webui (+ FAST_LIO 경로 규칙) | S21 안내를 버튼으로 | 1일 |
| B6 | `recorder_recording`을 녹화 프로세스 `poll()`로 갱신 | 녹화 프로세스가 죽어도 REC로 남음 | ros_slam_webui | 없음 | 0.25일 |
| B7 | `kill_slam_processes`의 `pkill -9 -f rviz2` 범위 축소 | 사용자 rviz까지 종료(G12) | ros_slam_webui | G12 툴팁 제거 | 0.25일 |
| B8 | Multi 결과 뷰어에 `StaticMap.pcd`·`Debug/KissMatchedMap.pcd` 레이어, Save Map 결과에 `StaticNGMap`·`StaticGroundMap` 레이어 | 산출물이 있는데 안 보임(M13, S19) | ros_slam_webui | 칩 추가 | 0.5일 |
| B9 | `02` §6 API 문서 전면 정리(빠진 `/api/localization/*`·`/api/bag/*`·`/api/recorder/*`·`/api/player/*`·`/api/browse`, 없는 `/api/viewer/get_topics`) | 문서·코드 불일치(부록 D.1) | ros_slam_webui (`.cursor/` 룰) | 없음 | 0.5일 |

