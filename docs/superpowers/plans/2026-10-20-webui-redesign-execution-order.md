# Field Console 리디자인 — 실행 순서표 (2026-10-20)

정본 계획: `docs/superpowers/plans/2026-10-16-webui-redesign.md` (이 파일은 계획서 본문을 고치지 않는다).
계획서 부록 B에 순서표 위치 지정이 없어 이 경로에 새로 만들었다.

## 1. 작업 공간

| 항목 | 값 |
|---|---|
| 워크트리 | `/home/kkw/localization_ws/src/ros_slam_webui/.worktrees/feature/webui-redesign-p0` |
| 브랜치 | 단계마다 앞 단계 끝에서 이어 만든다: `feature/webui-redesign-p0` → `-p1` → … → `-p6` (한 워크트리 안에서 `git checkout -b`) |
| 빌드 | `colcon build --base-paths src/ros_slam_webui/.worktrees/feature/webui-redesign-p0 --packages-select ros_slam_webui --build-base build_wt/webui-redesign-p0 --install-base install_wt/webui-redesign-p0` |
| 목업 | 저장소에 넣지 않음. 절대 경로 `/home/kkw/localization_ws/src/ros_slam_webui/docs/design/redesign-mock/`로만 참조 (부록 B에 shots/bundle 커밋 지시 없음) |

## 2. rg 재확인 결과 (HEAD 67c9fbd = 계획 기준 커밋)

계획서의 J/H/C/P 줄 번호를 표본 30여 곳 다시 확인했다. **어긋난 곳 없음**(±0줄).
확인한 곳: `openTab` J:131, `event.target` J:144·179, `handleOptBtnClick` J:457, `updateSlamState` J:645,
`confirm(` J:977, `updateRecorderState` J:2645, `confirmSaveMap` J:3172, `startSlamMapping` J:3364,
`updateLocalizationState` J:3475, `showYamlErrorModal` J:3572, latency worker J:4099, `updateRosDomainId` J:4138,
`_initBackendWs` J:4253, `updateRosbridgeStatusChip` J:4621, `updateBufferTime` J:5537, 뷰어 배경 J:6659·7937·9244,
`SlamResultViewer` J:9148, `toggleSaveMapLayer` J:10961, 대시보드 J:11158·11804, `_resetWidgets` J:11735·12244,
H 서브탭 8개(52·405·706·834·905·945·1162·1213), C `@import` 1줄·글꼴 3655/3664/3674/3677, `button[onclick*=` 28개,
P 3501·3534·3664·3696·3784·4011·4212·6350·6423·8551, `import re` P:123.

계획과 실제가 다른 점:

| # | 내용 | 처리 |
|---|---|---|
| D1 | `handleOptBtnClick`(J:457)은 async가 아님 | 계획 Task 3.2 지시대로 `async` 추가 (호출처 onclick 1곳) |
| D2 | 계획서에 `## 부록 A` 제목 줄이 없음 (13.6 다음에 바로 `### A.1`) | 스펙 파일에 제목을 붙여 A.1~A.6을 옮김 |
| D3 | 계획이 말하는 목업 파일 이름(`01-slam-ready.png`, `05-localization.png`, `06-multi-running.png`, `08-bag-player.png`, `09-bag-recorder.png`, `10-file-player.png`, `11-plot.png`, `12-3d-viewer.png`)이 실제 shots와 다름 (실제는 47장, 예: `16-loc-tracking.png`, `28-multi-running.png`, `34-bag-player-playing.png`, `39-bag-recorder-recording.png`, `41-file-player-not-loaded.png`, `43-plot-streaming.png`, `45-3d-viewer-frame-warning.png`) | 같은 화면의 실제 파일로 비교 |
| D4 | `long_term_mapping` 소스는 계획대로 `main` a52ddc9(메시지 8종 있음), 설치 바이너리는 `build/` symlink로 메시지 0건 | Task 3.0 보류(아래 4절) |
| D5 | 룰 03(ros_slam_webui)에 "노드 실행 승인" 절이 없음. 해당 절은 `long_term_mapping` 룰 03 §4에 있음 | 무거운 노드(SLAM·Localization·LTmapping) 실행은 하지 않음. 웹 서버+rosbridge만 실행 |

## 3. 실행 순서와 의존

| 순서 | 태스크 | 의존 | green 기준 (공통: 워크트리 빌드 + 바꾼 파일 `node --check`/`py_compile` + 해당 Playwright 검사 PASS·새 JS 에러 0 + `check_ids.py dev`) | 사용자 확인 |
|---|---|---|---|---|
| 1 | 0.1 스펙·계획 커밋 | — | 파일 2개 커밋 | — |
| 2 | 0.2 검증 도구 + 기준선 | 0.1 | `check_smoke.py baseline` PASS, 스크린샷 8장 | 기존 JS 에러 있으면 보고 |
| 3 | 0.3 Fira 글꼴 | — | 설치 경로에 7파일, curl 200 | — |
| 4 | 0.4 토큰 | 0.3 | `check_theme.py` PASS | 배치 동일·색만 변경 육안 |
| 5 | 0.5 컴포넌트 CSS | 0.4 | `check_components.py` PASS | — |
| 6 | 1.1 status_summary API | — | pytest PASS, curl 모양 | — |
| 7 | 1.2 ui_status_logic.js | — | `node test/ui/check_status_logic.js` PASS | — |
| 8 | 1.3 ui_kit.js | 0.5 | `check_ui_kit.py` PASS | — |
| 9 | 1.4 openTab 안전화 | — | `check_tabs_programmatic.py` PASS | — |
| 10 | 1.5 사이드바 셸 | 1.2·1.3·1.4 | `check_shell.py` PASS | legacyNav 수동 확인 |
| 11 | 1.6 연결 블록 | 1.5 | `check_shell.py` PASS | rosbridge 끊김 표시 |
| 12 | 1.7 사이드바 표시 | 1.1·1.2·1.5 | `check_sidebar_markers.py` PASS | — |
| 13 | 1.8 화면 헤더 | 1.5 | `check_shell.py` PASS | — |
| 14 | 2.1 SLAM/Loc 파생 | 1.2 | Node PASS | — |
| 15 | 2.2 SLAM 헤더 액션 | 1.8·2.1 | `check_slam_loc.py` PASS | 실데이터 1 |
| 16 | 2.3 SLAM 8/4 배치 | 2.2 | 〃 | — |
| 17 | 2.4 SLAM Analytics | 2.3 | 〃 | 실데이터 |
| 18 | 2.5 Config 읽기 전용 | 2.3·1.3 | 〃 | 실데이터 5 |
| 19 | 2.6 결과 뷰어 | 2.3 | 〃 | 저장된 지도 폴더 |
| 20 | 2.7 Live 툴바 버그 3 | 2.3 | 〃 | 풀스크린 아이콘 |
| 21 | 2.8 Localization 화면 | 2.1·2.2 패턴 | 〃 | 실데이터 7 |
| 22 | 2.9 뷰어 배경 | — | 스모크 | 대비 육안 |
| 23 | 2.10 이름 검증·로그 API | 2.3 | pytest + curl | 실데이터 2·4 |
| 24 | 2.11 노드 건강 표시 | 2.2·2.8·2.10 | `check_slam_loc.py` PASS | 실데이터 6·7 |
| 25 | 2.12 초기 위치 지정 | 2.8·5.1(confirmDialog) | 〃 | 실데이터(원점 밖 bag) |
| 26 | 3.0 long_term_mapping 재빌드 | — | **보류** | **사용자 승인 필요** |
| 27 | 3.1 Multi/Bag/Rec/File 파생 | 1.2 | Node PASS | — |
| 28 | 3.2 Multi 단일 폼 | 3.1 | `check_data.py` PASS | — |
| 29 | 3.3 Multi 결과 칩·복원 | 3.2 | 〃 | 실데이터 |
| 30 | 3.4 bag topic_details API | — | pytest PASS | 실제 bag으로 SAMPLE 확인 |
| 31 | 3.5 Bag 토픽 표 | 3.4 | `check_data.py` PASS | — |
| 32 | 3.6 Bag 재생 바 | 3.5 | 〃 | ROS1·ROS2 bag |
| 33 | 3.7 변환 패널 | 3.6 | 스크린샷 | — |
| 34 | 3.8 Recorder 통계 API | — | pytest PASS | 실제 녹화 |
| 35 | 3.9 Recorder 화면 | 3.8 | `check_data.py` PASS | 새로고침 REC 복원 |
| 36 | 3.10 File Player | 3.1 | 〃 | 데이터셋 1종 |
| 37 | 3.11 Multi 실패 감지 API | 3.2 | pytest PASS | Map1 빈 폴더 실행(LTmapping 실행 승인 필요) |
| 38 | 4.1~4.4 Plot·3D | 1.8 | `check_viz.py` PASS | Plot 값 갱신 |
| 39 | 5.1~5.5 토스트·모달·반응형·접근성 | 1.3 | `check_a11y.py` PASS | — |
| 40 | 6.1·6.2·6.3·6.5 정리·문서 | P1~P5 | 전체 `check_*.py` PASS | 풀스크린 아이콘 |

순서 조정(Ruling):
- **2.10·2.12의 `uiKit.confirmDialog` 의존**: 계획은 `confirmDialog`를 Task 5.1에서 만든다. 2.10(덮어쓰기 확인)·2.12(초기 위치 확인)이 먼저 쓰므로, `confirmDialog`를 **처음 쓰는 태스크에서 `ui_kit.js`에 먼저 추가**하고 5.1은 shim·`inferToastTone`·J:977 교체만 한다. 인터페이스는 5.1 정의(`{title, message, okText, cancelText, tone} -> Promise<boolean>`) 그대로.
- **3.0 보류**: 다른 패키지(`long_term_mapping`) 빌드 산출물(`build/long_term_mapping/LTmapping`)을 덮어써 지금 설치된 ELite 변종 바이너리를 잃는다. 워크트리 밖 부작용이고 그 패키지 룰(03 §4, 00.core)이 승인·git 상태 보존을 요구하므로 사용자 승인 전에는 하지 않는다. 3.1~3.3·3.11은 코드만 구현하고, 진행 5단계 실표시는 재빌드 뒤 사용자 확인으로 넘긴다.
- **단계별 dev 통합 없음**: 계획 B.1은 "앞 단계가 dev에 들어간 뒤 다음 단계"지만, 이번 작업은 dev/main을 건드리지 않는다. 그래서 다음 단계 브랜치를 앞 단계 브랜치 끝에서 만든다. 통합은 사용자가 finishing에서 고른다.
- **Task 5.6(선택)·6.4(권장 안 함)는 하지 않는다.**

## 4. 사용자 확인이 필요한 항목 (실행하지 않음)

- Task 3.0 `long_term_mapping` 재빌드 승인.
- P2 실데이터 체크리스트 8항목(계획서 8장 끝), P3 실데이터(ROS1/ROS2 bag 재생, 녹화, 데이터셋 Save Bag, Multi 실패 표시), P4 Plot 값 실시간 갱신.
- 동작 변경 승인(계획서 D.2): 저장 중 Stop 잠금, SLAM 미실행 시 Save Map 잠금, Config 숨김 → 읽기 전용, 버그 수정 3건(S14·M5·S11), 3D Views 미동작 입력 4개 숨김.
