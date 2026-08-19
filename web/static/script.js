// ROS 버전 감지 및 messageType 헬퍼 (다중 스크립트 간 전역 공유)
window._rosVersion = (typeof window._rosVersion === 'number') ? window._rosVersion : 2;
window.getMsgType = window.getMsgType || function getMsgType(ros1, ros2) {
    return window._rosVersion === 1 ? ros1 : ros2;
};
var getMsgType = window.getMsgType;
fetch('/api/ros_version')
    .then((r) => r.json())
    .then((d) => { window._rosVersion = d.version || 2; })
    .catch(() => {});

// webui_ports.js 로드 실패 시에도 기본 포트로 동작하도록 안전장치
if (typeof window.ensureWebuiPortsReady !== 'function') {
    window.ROS_SLAM_WEBUI = window.ROS_SLAM_WEBUI || {
        webPort: 8080,
        pc2WsPort: 8081,
        rosbridgePort: 9090,
        ready: true,
    };
    window.ROS_SLAM_WEBUI_READY = Promise.resolve(window.ROS_SLAM_WEBUI);
    window.ensureWebuiPortsReady = () => Promise.resolve(window.ROS_SLAM_WEBUI);
    window.getPc2WsPort = () => window.ROS_SLAM_WEBUI.pc2WsPort || 8081;
    window.getWebSocketHost = () => window.location.hostname || 'localhost';
    window.getPc2WsUrl = (host) => {
        const h = host || window.getWebSocketHost();
        return `ws://${h}:${window.getPc2WsPort()}`;
    };
    window.getRosbridgePort = () => window.ROS_SLAM_WEBUI.rosbridgePort || 9090;
    window.getRosbridgeUrl = (host) => {
        const h = host || window.getWebSocketHost();
        return `ws://${h}:${window.getRosbridgePort()}`;
    };
    window.getRosNotConnectedHint = () => {
        const rosVersion = (typeof window._rosVersion === 'number') ? window._rosVersion : 2;
        if (rosVersion === 1) {
            return 'Not connected to ROS. Make sure rosbridge_server is running:\n\n'
                + 'roslaunch ros_slam_webui ros_slam_webui.launch\n'
                + '# or\n'
                + 'roslaunch rosbridge_server rosbridge_websocket.launch port:=9090';
        }
        return 'Not connected to ROS. Make sure rosbridge_server is running:\n\n'
            + 'ros2 launch rosbridge_server rosbridge_websocket_launch.xml';
    };
}

// Global state - grouped by functionality
const fileBrowserState = {
    currentPath: '/home',
    callback: null
};

const bagPlayerState = {
    selectedTopics: [],
    availableTopics: [],
    bagDuration: 0.0,
    bagType: 'ros2',   // 'ros1' or 'ros2'
    bagFormat: 'ros2_db3', // 'ros1' | 'ros2_db3' | 'ros2_mcap'
    playbackRate: 1.0, // ROS1 재생 속도 배율
    wasPlaying: false  // 재생 종료 시 슬라이더 리셋 감지용
};

// bag 슬라이더 드래그 중 여부 (드래그 중에만 폴링 업데이트 차단)
let _bagSliderDragging = false;
// seek 처리 중 여부 (ROS2 bag seek 시 잠깐 playing=false → 슬라이더 0 리셋 방지)
let _bagSeeking = false;

const bagRecorderState = {
    bagName: '',
    selectedTopics: []
};

const siblingPackagePaths = {
    longTermMapping: null,
    poseGraphOptimization: null,
};

const kittiState = {
    baseDir: null,   // 사용자가 선택한 KITTI 최상위 디렉토리
    calibDir: null,  // calib 파일이 있는 실제 경로
    drives: [],      // drive 목록 [{name, drive_type, drive_id, data_path}]
    converting: false, // 변환 중 여부
    // 진행률/완료/오류는 8081 WebSocket kitti_convert_* 메시지로 수신
};

const kaistState = {
    baseDir: null,   // 사용자가 선택한 KAIST 최상위 디렉토리
    sequences: [],  // 시퀀스 목록 [{name, path}]
    converting: false, // 변환 중 여부
    // 진행률/완료/오류는 8081 WebSocket kaist_convert_* 메시지로 수신
};

const mulranState = {
    baseDir: null,    // 사용자가 선택한 MulRan 최상위 디렉토리
    sequences: [],    // 시퀀스 목록 [{name, path}]
    converting: false, // 변환 중 여부
    // 진행률/완료/오류는 8081 WebSocket mulran_convert_* 메시지로 수신
};

const heliprState = {
    baseDir: null,    // 사용자가 선택한 HeLiPR 최상위 디렉토리
    sequences: [],    // 시퀀스 목록 [{name, path}]
    converting: false, // 변환 중 여부
    // 진행률/완료/오류는 8081 WebSocket helipr_convert_* 메시지로 수신
};

// bag-format select 공통 라벨 (Bag Recorder, File Player 데이터셋 변환 공통 사용)
const BAG_FORMAT_LABELS = { ros1: 'ROS1 .bag', ros2_db3: 'ROS2 db3', ros2_mcap: 'ROS2 mcap' };

// Cached DOM elements
const domCache = {
    elements: {},
    get(id) {
        if (!this.elements[id]) {
            this.elements[id] = document.getElementById(id);
        }
        return this.elements[id];
    },
    clear() {
        this.elements = {};
    }
};

// Last active subtab state per main tab (persists across tab switches)
const lastActiveSubtab = {
    'slam-tab': 'lidar-slam-subtab',
    'player-tab': 'bag-player-subtab',
    'visualization-tab': 'plot-subtab'
};

// Tab Management
function openTab(tabId) {
    // Hide all tabs
    const tabs = document.querySelectorAll('.tab-content');
    tabs.forEach(tab => tab.classList.remove('active'));

    // Remove active class from all buttons
    const buttons = document.querySelectorAll('.tab-button');
    buttons.forEach(btn => btn.classList.remove('active'));

    // Show selected tab
    domCache.get(tabId).classList.add('active');

    // Activate corresponding button
    event.target.classList.add('active');

    // Scroll to top
    window.scrollTo({ top: 0, behavior: 'smooth' });

    // Restore last active sub-tab for this main tab (fallback to default)
    const subtabToRestore = lastActiveSubtab[tabId];
    if (subtabToRestore) {
        openSubTab(subtabToRestore, true);
    }
}

// Sub-Tab Management (consolidated function)
function openSubTab(subtabId, skipEvent = false) {
    // Hide all sub-tabs
    const subtabs = document.querySelectorAll('.subtab-content');
    subtabs.forEach(subtab => subtab.classList.remove('active'));

    // Remove active class from all sub-tab buttons
    const buttons = document.querySelectorAll('.subtab-button');
    buttons.forEach(btn => btn.classList.remove('active'));

    // Show selected sub-tab
    const selectedSubtab = domCache.get(subtabId);
    if (selectedSubtab) {
        selectedSubtab.classList.add('active');

        // Save last active subtab for the parent main tab
        const parentTab = selectedSubtab.closest('.tab-content');
        if (parentTab && parentTab.id && parentTab.id in lastActiveSubtab) {
            lastActiveSubtab[parentTab.id] = subtabId;
        }
    }

    // Activate corresponding button
    if (!skipEvent && event && event.target) {
        event.target.classList.add('active');
    } else {
        // Find and activate corresponding button
        const correspondingButton = Array.from(buttons).find(btn =>
            btn.getAttribute('onclick') && btn.getAttribute('onclick').includes(subtabId)
        );
        if (correspondingButton) {
            correspondingButton.classList.add('active');
        }
    }

    // Scroll to top only if not called internally
    if (!skipEvent) {
        window.scrollTo({ top: 0, behavior: 'smooth' });
    }

    // Initialize Plot subtab
    if (subtabId === 'plot-subtab') {
        console.log('[openSubTab] Initializing Plot subtab');
        initPlotSubtab();
    }
    
    // Initialize 3D Viewer if switching to that subtab
    if (subtabId === '3d-viewer-subtab') {
        // Wait for DOM update, then initialize
        setTimeout(() => {
            if (typeof initialize3DViewer === 'function') {
                console.log('Calling initialize3DViewer from openSubTab');
                initialize3DViewer();
            } else {
                console.warn('initialize3DViewer function not found');
            }
        }, 300);
    }
}

/** Plot 영역 크기 변경 시 rAF로 한 번만 Plotly 리사이즈 (ResizeObserver 콜백 폭주 완화) */
let _plotAreaResizeRafId = null;

/**
 * 현재 표시 중인 Plot 탭의 Plotly 그래프를 컨테이너에 맞게 리사이즈
 * (좌측 토픽 패널 접기/창 크기 변경 등)
 */
function resizeVisiblePlotlyPlots() {
    if (!plotState.plotTabManager || typeof Plotly === 'undefined' || !Plotly.Plots || typeof Plotly.Plots.resize !== 'function') {
        return;
    }
    for (const tab of plotState.plotTabManager.tabs) {
        if (!tab.plotDiv || tab.plotDiv.style.display === 'none') continue;
        if (!tab.plotManager || !tab.plotManager.isInitialized) continue;
        try {
            Plotly.Plots.resize(tab.plotDiv);
        } catch (err) {
            console.warn('[resizeVisiblePlotlyPlots]', err);
        }
    }
}

function scheduleResizeVisiblePlotlyPlots() {
    if (_plotAreaResizeRafId !== null) cancelAnimationFrame(_plotAreaResizeRafId);
    _plotAreaResizeRafId = requestAnimationFrame(() => {
        _plotAreaResizeRafId = null;
        resizeVisiblePlotlyPlots();
    });
}

function setupPlotAreaPlotlyResizeObserver() {
    const el = document.getElementById('plot-area-container');
    if (!el || plotState._plotAreaResizeObserver) return;
    if (typeof ResizeObserver === 'undefined') return;
    plotState._plotAreaResizeObserver = new ResizeObserver(() => {
        scheduleResizeVisiblePlotlyPlots();
    });
    plotState._plotAreaResizeObserver.observe(el);
}

window.resizeVisiblePlotlyPlots = resizeVisiblePlotlyPlots;

// Plot subtab 초기화
function initPlotSubtab() {
    initPlotTree();

    // PlotTabManager 초기화 (처음 한 번만)
    if (!plotState.plotTabManager) {
        console.log('[initPlotSubtab] Initializing PlotTabManager');
        plotState.plotTabManager = new PlotTabManager('plot-tab-bar-container', 'plot-area-container', 5.0);
        plotState.plotTabManager.init();
        setupPlotAreaPlotlyResizeObserver();

        // 드롭 존 설정 (PlotTabManager 초기화 후)
        setupPlotDropZone();
    }
    
    if (!plotState.ros) {
        console.log('[initPlotSubtab] Connecting to rosbridge');
        initRosbridge();
    } else if (plotState.ros.isConnected) {
        _verifyRosbridgeAlive(plotState.ros, 2500).then((alive) => {
            if (alive) {
                if (plotState.topics.length === 0) {
                    console.log('[initPlotSubtab] rosbridge verified, loading topics');
                    loadPlotTopics();
                }
            } else {
                console.warn('[initPlotSubtab] rosbridge stale — reconnecting');
                try { plotState.ros.close(); } catch (e) { /* ignore */ }
                plotState.ros = null;
                initRosbridge();
            }
        });
    } else {
        initRosbridge();
    }

    // Python 백엔드 WebSocket (8081) 연결 — throttle 없이 원래 주기로 plot
    _initBackendWs();

    // 주기적으로 토픽 목록 갱신 시작
    startTopicRefresh();
}

// 주기적으로 토픽 목록 갱신
function startTopicRefresh() {
    // 기존 인터벌이 있으면 정리
    if (plotState.topicRefreshInterval) {
        clearInterval(plotState.topicRefreshInterval);
    }
    
    plotState.topicRefreshInterval = setInterval(() => {
        if (plotState.ros && plotState.ros.isConnected) {
            console.log('[startTopicRefresh] Refreshing topic list...');
            loadPlotTopics();
        }
    }, plotState.topicRefreshRate);
    
    console.log(`[startTopicRefresh] Started topic refresh every ${plotState.topicRefreshRate}ms`);
}

// 토픽 갱신 중지
function stopTopicRefresh() {
    if (plotState.topicRefreshInterval) {
        clearInterval(plotState.topicRefreshInterval);
        plotState.topicRefreshInterval = null;
        console.log('[stopTopicRefresh] Stopped topic refresh');
    }
}

// API Helper Functions
async function apiCall(endpoint, data = null) {
    const options = {
        method: data ? 'POST' : 'GET',
        headers: {
            'Content-Type': 'application/json',
        }
    };

    if (data) {
        options.body = JSON.stringify(data);
    }

    try {
        const response = await fetch(endpoint, options);
        return await response.json();
    } catch (error) {
        console.error('API call failed:', error);
        return { success: false, error: error.message };
    }
}

// File Browser Functions
async function openFileBrowser(callback, startPath = '~') {
    fileBrowserState.callback = callback;
    fileBrowserState.currentPath = startPath;
    await loadDirectoryList(fileBrowserState.currentPath);
    domCache.get('file-browser-modal').style.display = 'block';
}

function closeFileBrowser() {
    domCache.get('file-browser-modal').style.display = 'none';
    fileBrowserState.callback = null;
}

async function loadDirectoryList(path) {
    try {
        const response = await fetch(`/api/browse?path=${encodeURIComponent(path)}`);
        const result = await response.json();

        if (result.success) {
            fileBrowserState.currentPath = result.current_path;
            domCache.get('current-path-display').textContent = result.current_path;

            const listElement = domCache.get('directory-list');
            listElement.innerHTML = '';

            result.entries.forEach(entry => {
                const div = document.createElement('div');
                div.className = 'directory-entry';

                // Add icon for directories and files
                if (entry.is_dir) {
                    div.classList.add('dir-entry');
                    div.textContent = '📁 ' + entry.name;
                    div.onclick = () => loadDirectoryList(entry.path);
                } else {
                    div.classList.add('dir-entry', 'file-item');
                    div.textContent = '📄 ' + entry.name;
                    div.onclick = () => selectFile(entry.path);
                }

                listElement.appendChild(div);
            });
        } else {
            alert('Failed to load directory: ' + (result.error || 'Unknown error'));
        }
    } catch (error) {
        console.error('Failed to load directory:', error);
        alert('Failed to load directory');
    }
}

function selectFile(filePath) {
    if (fileBrowserState.callback) {
        fileBrowserState.callback(filePath);
    }
    closeFileBrowser();
}

function selectCurrentDirectory() {
    if (fileBrowserState.callback) {
        fileBrowserState.callback(fileBrowserState.currentPath);
    }
    closeFileBrowser();
}

// SLAM GUI Functions
async function loadMap1() {
    const defaultDir = siblingPackagePaths.longTermMapping || '~';
    openFileBrowser(async (path) => {
        domCache.get('slam-map1').value = path;
        const result = await apiCall('/api/slam/set_map1', { path });
        if (result.success) {
            updateSlamStatus(result.status);
        }
    }, defaultDir);
}

async function loadMap2() {
    const defaultDir = siblingPackagePaths.longTermMapping || '~';
    openFileBrowser(async (path) => {
        domCache.get('slam-map2').value = path;
        const result = await apiCall('/api/slam/set_map2', { path });
        if (result.success) {
            updateSlamStatus(result.status);
        }
    }, defaultDir);
}

async function setOutput() {
    const outputField = domCache.get('slam-output');
    const directoryName = outputField.value.trim();
    if (!directoryName) {
        alert('Please enter an output directory name');
        return;
    }
    const result = await apiCall('/api/slam/set_output', { path: directoryName });
    if (result.success) {
        updateSlamStatus(result.status);
        // Keep the value in the field after setting
        outputField.value = directoryName;
    }
}

let _optPollTimer = null;
let _optComplete = false;
let _optRunning = false;
let _optViewerDismissed = false;  // Exit로 결과 뷰어를 닫은 경우 폴링 자동 복원 방지

function handleOptBtnClick() {
    if (_optComplete) {
        exitOptimization();
    } else {
        runOptimization();
    }
}

async function runOptimization() {
    _optRunning = true;
    _optViewerDismissed = false;
    slamResultViewer.hideAndReset();
    const runBtn = domCache.get('slam-opt-run-btn');
    runBtn.disabled = true;
    const result = await apiCall('/api/slam/optimize', {});
    if (result.success) {
        updateSlamStatus(result.status || 'Running...');
        _showOptStatus('Starting optimization...', true);
        _startOptPolling();
    } else {
        _optRunning = false;
        runBtn.disabled = false;
        alert('Failed to start optimization: ' + (result.message || result.status || 'Unknown error'));
    }
}

function _setOptAreaState(state) {
    const area = domCache.get('slam-opt-status-area');
    area.classList.remove('success', 'warn', 'error');
    if (state) area.classList.add(state);
}

function _showOptStatus(message, running) {
    const area = domCache.get('slam-opt-status-area');
    const msgEl = domCache.get('slam-opt-msg');
    const spinner = domCache.get('slam-opt-spinner');
    const cancelBtn = domCache.get('slam-opt-cancel-btn');

    msgEl.textContent = message;
    spinner.style.display = running ? 'inline-block' : 'none';
    cancelBtn.style.display = running ? 'inline-block' : 'none';
    cancelBtn.disabled = false;
    cancelBtn.textContent = 'Cancel';
    area.style.display = 'block';
    area.style.opacity = '1';
    _setOptAreaState(null);
}

function _resetOptBtn() {
    _optComplete = false;
    const runBtn = domCache.get('slam-opt-run-btn');
    runBtn.disabled = false;
    runBtn.textContent = 'Multi Session Optimization';
}

function _showOptSuccess() {
    const area = domCache.get('slam-opt-status-area');
    const cancelBtn = domCache.get('slam-opt-cancel-btn');
    const spinner = domCache.get('slam-opt-spinner');
    const msgEl = domCache.get('slam-opt-msg');

    cancelBtn.style.display = 'none';
    // 뷰어가 완전히 준비될 때까지 스피너/상태 문구를 유지해, 빈 뷰어 대신 이 표시로 진행 상황을 알린다.
    msgEl.textContent = 'Loading results...';

    _optRunning = false;
    _optComplete = true;
    const runBtn = domCache.get('slam-opt-run-btn');
    runBtn.disabled = false;
    runBtn.textContent = 'Exit';

    slamResultViewer.show().finally(() => {
        spinner.style.display = 'none';
        area.style.transition = 'opacity 0.6s ease';
        area.style.opacity = '0';
        setTimeout(() => {
            area.style.display = 'none';
            area.style.opacity = '1';
            area.style.transition = '';
        }, 620);
    });
}

function _showOptError(message, autoHide = false) {
    const area = domCache.get('slam-opt-status-area');
    const msgEl = domCache.get('slam-opt-msg');
    const spinner = domCache.get('slam-opt-spinner');
    const cancelBtn = domCache.get('slam-opt-cancel-btn');

    _optRunning = false;
    msgEl.textContent = message;
    spinner.style.display = 'none';
    cancelBtn.style.display = 'none';
    area.style.display = 'block';
    area.style.opacity = '1';
    area.style.transition = '';
    _setOptAreaState('error');
    _resetOptBtn();

    if (autoHide) {
        setTimeout(() => {
            area.style.transition = 'opacity 0.6s ease';
            area.style.opacity = '0';
            setTimeout(() => {
                area.style.display = 'none';
                area.style.opacity = '1';
                area.style.transition = '';
            }, 620);
        }, 3000);
    }
}

function _startOptPolling() {
    if (_optPollTimer) clearTimeout(_optPollTimer);
    _optPollTimer = null;
    _scheduleOptPoll();
}

function _scheduleOptPoll() {
    _optPollTimer = setTimeout(_pollOptStatus, 2000);
}

async function _pollOptStatus() {
    _optPollTimer = null;
    try {
        const status = await apiCall('/api/slam/optimization_status');

        if (status.done) {
            if (status.success) {
                updateSlamStatus('Optimization complete!');
                _showOptSuccess();
            } else if (status.message && status.message.includes('Cancelled')) {
                updateSlamStatus('Optimization cancelled');
                _showOptError('Cancelled: ' + status.message, true);
            } else {
                updateSlamStatus('Optimization failed');
                _showOptError('✗ ' + (status.message || 'Optimization failed'));
            }
        } else if (status.running) {
            _showOptStatus(status.message || 'Running...', true);
            _scheduleOptPoll();
        }
    } catch (e) {
        console.error('Failed to poll optimization status:', e);
        _scheduleOptPoll();
    }
}

async function cancelOptimization() {
    const cancelBtn = domCache.get('slam-opt-cancel-btn');
    cancelBtn.disabled = true;
    cancelBtn.textContent = 'Cancelling...';

    const result = await apiCall('/api/slam/cancel_optimization', {});

    if (result.success) {
        clearTimeout(_optPollTimer);
        _optPollTimer = null;
        updateSlamStatus('Optimization cancelled');
        _showOptError('Cancelled by user', true);
    } else {
        cancelBtn.disabled = false;
        cancelBtn.textContent = 'Cancel';
        console.warn('Cancel failed:', result.message);
    }
}

async function exitOptimization() {
    const runBtn = domCache.get('slam-opt-run-btn');
    runBtn.disabled = true;
    runBtn.textContent = 'Exiting...';

    await apiCall('/api/slam/cancel_optimization', {});

    clearTimeout(_optPollTimer);
    _optPollTimer = null;

    _optRunning = false;
    _optViewerDismissed = true;
    updateSlamStatus('Optimization exited');
    slamResultViewer.hideAndReset();
    _resetOptBtn();
}

function updateSlamStatus(status) {
    domCache.get('slam-status').textContent = 'Status: ' + status;
}

async function updateSlamState() {
    const state = await apiCall('/api/slam/state');
    if (state) {
        domCache.get('slam-map1').value = state.map1 || '';
        domCache.get('slam-map2').value = state.map2 || '';

        // Only update output field if it's not currently focused (user is not typing)
        const outputField = domCache.get('slam-output');
        if (document.activeElement !== outputField) {
            outputField.value = state.output || '';
        }

        // Update Multi-Session SLAM status
        updateSlamStatus(state.status || 'Ready');

        // Show result viewer if optimization is already complete (e.g. on subtab re-entry)
        // _optRunning이 true이면 최적화 진행 중, _optViewerDismissed면 사용자가 Exit로 닫은 상태
        // → 두 경우 모두 뷰어를 강제 복원하지 않음
        if (!_optRunning && !_optViewerDismissed && state.status === 'Optimization complete!') {
            const viewerEl = document.getElementById('slam-result-viewer');
            if (viewerEl && viewerEl.style.display === 'none') {
                slamResultViewer.show();
            }
        }

        // 서버 실행 상태 기준 Live Viewer·Analytics 복원 (새로고침/다른 기기, 탭 무관)
        if (!window._slamStopping && !window._slamSaving && !window._slamMapJustSaved) {
            if (state.is_running) {
                if (typeof slamLiveViewer !== 'undefined' && !slamLiveViewer._visible) {
                    slamLiveViewer.show();
                }
                if (typeof slamAnalyticsDashboard !== 'undefined') {
                    const dashEl = document.getElementById('slam-analytics-dashboard');
                    if (dashEl && dashEl.style.display === 'none') {
                        slamAnalyticsDashboard.show();
                        slamAnalyticsDashboard.subscribe();
                    }
                }
            } else {
                if (typeof slamLiveViewer !== 'undefined' && slamLiveViewer._visible && !window._slamLiveViewerHoldOpen) {
                    slamLiveViewer.hide();
                }
            }
        }

        // Update LiDAR SLAM status (only if LiDAR SLAM tab is active)
        const lidarSlamStatus = domCache.get('lidar-slam-status');
        if (lidarSlamStatus) {
            const lidarSlamTab = document.getElementById('lidar-slam-subtab');
            if (lidarSlamTab && lidarSlamTab.classList.contains('active')) {
                // 페이지 재진입 시 Save Map 결과가 이미 완료된 상태면 뷰어 복원
                _maybeRestoreSaveMapViewer();
                // SLAM 미실행 상태면 _slamSaving 플래그 자동 해제 (stale flag 방지)
                if (!state.is_running) {
                    window._slamSaving = false;
                    window._slamMapJustSaved = false; // SLAM 중단 시 플래그 해제
                }
                // Determine status based on SLAM state
                let statusText = 'Ready';
                if (state.is_running !== undefined) {
                    if (state.is_running) {
                        statusText = 'Running';
                    } else {
                        statusText = 'Ready';
                    }
                } else if (state.status && state.status !== 'Ready') {
                    statusText = state.status;
                }
                lidarSlamStatus.textContent = 'Status: ' + statusText;
                // Add red color for Stopping status
                if (statusText.includes('Stopping')) {
                    lidarSlamStatus.style.color = '#F44336'; // Red
                } else {
                    lidarSlamStatus.style.color = ''; // Reset to default
                }
            }
        }
        
        // Update Localization status will be handled by updateLocalizationState()
    }
}

// Bag Player Functions

/** bag load/get_info 응답을 UI·bagPlayerState에 반영 */
function applyBagPlayerInfo(path, result, opts = {}) {
    const { resetSlider = false, selectedTopicsOverride = null } = opts;

    if (path) {
        const bagDir = domCache.get('bag-directory');
        if (bagDir) bagDir.value = path;
    }

    bagPlayerState.availableTopics = result.topics || [];
    bagPlayerState.bagDuration = result.duration || 0.0;
    bagPlayerState.bagType = result.bag_type
        || (path && path.endsWith('.bag') ? 'ros1' : 'ros2');
    bagPlayerState.bagFormat = result.bag_format
        || (bagPlayerState.bagType === 'ros1' ? 'ros1' : 'ros2_db3');

    if (selectedTopicsOverride && selectedTopicsOverride.length > 0) {
        bagPlayerState.selectedTopics = selectedTopicsOverride.slice();
    } else if (bagPlayerState.bagType === 'ros1' && bagPlayerState.availableTopics.length > 0
            && typeof bagPlayerState.availableTopics[0] === 'object') {
        bagPlayerState.selectedTopics = bagPlayerState.availableTopics
            .filter(t => t.publishable)
            .map(t => t.name);
    } else {
        bagPlayerState.selectedTopics = bagPlayerState.availableTopics.map(
            t => (typeof t === 'object' ? t.name : t)
        );
    }

    const isRos1 = bagPlayerState.bagType === 'ros1';
    const ros1Badge = domCache.get('bag-ros1-badge');
    const ros2Badge = domCache.get('bag-ros2-badge');
    if (ros1Badge) ros1Badge.style.display = isRos1 ? 'inline' : 'none';
    if (ros2Badge) ros2Badge.style.display = !isRos1 ? 'inline' : 'none';
    const convertControls = domCache.get('bag-convert-controls');
    if (convertControls) convertControls.style.display = path ? '' : 'none';
    updatePlayerFormatSelectDefault();
    const rateControls = domCache.get('ros1-playback-controls');
    if (rateControls) rateControls.style.display = 'block';

    if (resetSlider) {
        updatePlaybackRate(document.getElementById('bag-playback-rate')?.value ?? 10);
        updateBagTimeLabel(0, bagPlayerState.bagDuration);
    }
    updateSelectedTopicsDisplay();
}

/** 페이지 로드·다른 기기 접속 시 서버 bag player 상태 복원 */
async function restoreBagPlayerFromServer() {
    try {
        const state = await apiCall('/api/bag/state');
        if (!state || !state.path) return;

        const info = await apiCall('/api/bag/get_info');
        if (!info || !info.success) return;

        applyBagPlayerInfo(state.path, info, {
            selectedTopicsOverride: state.selected_topics,
            resetSlider: false
        });

        if (state.playback_rate) {
            bagPlayerState.playbackRate = state.playback_rate;
            const rateSlider = document.getElementById('bag-playback-rate');
            if (rateSlider) {
                rateSlider.value = Math.round(state.playback_rate * 10);
                updatePlaybackRate(rateSlider.value);
            }
        }

        await updateBagState();
        console.log('[BagPlayer] Restored from server:', state.path);
    } catch (e) {
        console.warn('[BagPlayer] restore from server failed:', e);
    }
}

async function loadBagFile() {
    openFileBrowser(async (path) => {
        domCache.get('bag-directory').value = path;
        const result = await apiCall('/api/bag/load', { path });
        if (result.success) {
            console.log('Bag file loaded successfully:', path);
            // ConPR → ROS1/ROS2 bag 전환 시 3D Viewer 토픽 구독 리셋 (CustomMsg↔PointCloud2 충돌 방지)
            if (typeof resetViewerTopicSubscriptions === 'function') {
                resetViewerTopicSubscriptions();
            }
            if (typeof resetBagFrameAndTFState === 'function') {
                resetBagFrameAndTFState();
            }
            applyBagPlayerInfo(path, result, { resetSlider: true });

            console.log('Loaded topics:', bagPlayerState.availableTopics);
            console.log('Duration:', bagPlayerState.bagDuration, 'seconds');
            console.log('Bag type:', bagPlayerState.bagType);

            if (bagPlayerState.availableTopics.length === 0) {
                alert('No topics found in the bag file. The bag might be empty or corrupted.');
            }
        } else {
            alert('Failed to load bag file: ' + (result.message || 'Unknown error'));
        }
    }, '~');
}

function formatTime(seconds) {
    const mins = Math.floor(seconds / 60);
    const secs = Math.floor(seconds % 60);
    return `${mins}:${secs.toString().padStart(2, '0')}`;
}

function updateBagTimeLabel(current, total) {
    const label = domCache.get('bag-time-label');
    label.textContent = `${formatTime(current)} / ${formatTime(total)}`;
}

async function selectTopics() {
    const bagPath = domCache.get('bag-directory').value;
    if (!bagPath) {
        alert('Please load a bag file first');
        return;
    }

    if (bagPlayerState.availableTopics.length === 0) {
        alert('No topics found in the bag file');
        return;
    }

    // Display topic selection modal
    const topicList = domCache.get('topic-list');
    topicList.innerHTML = '';

    bagPlayerState.availableTopics.forEach(topicEntry => {
        // topicEntry: string (ROS2) 또는 {name, type, publishable} (ROS1)
        const topicName = typeof topicEntry === 'object' ? topicEntry.name : topicEntry;
        const topicType = typeof topicEntry === 'object' ? topicEntry.type : '';
        const publishable = typeof topicEntry === 'object' ? topicEntry.publishable : true;

        const div = document.createElement('div');
        div.className = 'topic-item';

        const checkbox = document.createElement('input');
        checkbox.type = 'checkbox';
        checkbox.id = `topic-${topicName}`;
        checkbox.value = topicName;
        checkbox.checked = bagPlayerState.selectedTopics.includes(topicName);

        // publish 불가 토픽은 비활성화 처리
        if (!publishable) {
            checkbox.disabled = true;
            checkbox.checked = false;
        }

        const label = document.createElement('label');
        label.htmlFor = `topic-${topicName}`;

        // 토픽 타입 표시 (있는 경우)
        if (topicType) {
            label.innerHTML = `<span style="font-weight:600;">${topicName}</span>`
                + ` <span style="color:#888; font-size:0.85em;">${topicType}</span>`
                + (!publishable ? ' <span style="color:#f66; font-size:0.82em;">(not publishable)</span>' : '');
        } else {
            label.textContent = topicName;
        }

        if (!publishable) {
            div.style.opacity = '0.45';
        }

        div.appendChild(checkbox);
        div.appendChild(label);
        topicList.appendChild(div);
    });

    domCache.get('topic-selection-modal').style.display = 'block';
}

function closeTopicSelection() {
    domCache.get('topic-selection-modal').style.display = 'none';
}

function confirmTopicSelection() {
    // Get all checked topics
    bagPlayerState.selectedTopics = [];
    const checkboxes = document.querySelectorAll('#topic-list input[type="checkbox"]:checked');
    checkboxes.forEach(checkbox => {
        bagPlayerState.selectedTopics.push(checkbox.value);
    });

    console.log('Selected topics:', bagPlayerState.selectedTopics);

    // Update display
    updateSelectedTopicsDisplay();

    closeTopicSelection();

    if (bagPlayerState.selectedTopics.length === 0) {
        alert('Please select at least one topic');
    }
}

function updateSelectedTopicsDisplay() {
    const display = domCache.get('bag-selected-topics-display');
    if (!display) return;
    if (!display.classList.contains('topics-display-styled')) {
        display.classList.add('topics-display-styled');
    }

    if (bagPlayerState.selectedTopics.length === 0) {
        display.innerHTML = '<span style="color: #888;">No topics selected</span>';
    } else {
        const topicsHtml = bagPlayerState.selectedTopics.map(topic =>
            `<div style="display: inline-block; background: #2a5a8a; padding: 3px 8px; margin: 2px; border-radius: 3px; font-size: 0.9em;">${topic}</div>`
        ).join('');
        display.innerHTML = topicsHtml;
    }
}

async function playBag() {
    const bagPath = domCache.get('bag-directory').value;
    if (!bagPath) {
        alert('Please load a bag file first');
        return;
    }

    // ROS1 bag: /api/bag/play_ros1 또는 /api/bag/stop_ros1 경로로 분기
    if (bagPlayerState.bagType === 'ros1') {
        const playButton = domCache.get('bag-play-button');

        // 이미 재생 중이면 정지
        if (playButton && playButton.textContent === 'Stop') {
            const stopResult = await apiCall('/api/bag/stop_ros1', {});
            if (stopResult.success) {
                playButton.textContent = 'Play';
                domCache.get('bag-pause-button').textContent = 'Pause';
                console.log('ROS1 bag playback stopped');
            } else {
                console.error('Failed to stop ROS1 playback');
            }
            return;
        }

        // publish 불가 토픽이 있는 경우 경고 다이얼로그 표시
        const unpublishable = bagPlayerState.availableTopics.filter(
            t => typeof t === 'object' && !t.publishable
        );
        if (unpublishable.length > 0) {
            const names = unpublishable.map(t => t.name).join('\n  - ');
            const proceed = confirm(
                `다음 토픽은 ROS2에서 지원되지 않아 publish되지 않습니다:\n  - ${names}\n\n계속하시겠습니까?`
            );
            if (!proceed) {
                return;
            }
        }

        const result = await apiCall('/api/bag/play_ros1', {
            bag_path: bagPath,
            topics: bagPlayerState.selectedTopics,
            playback_rate: bagPlayerState.playbackRate
        });
        if (result.success) {
            if (playButton) {
                playButton.textContent = 'Stop';
            }
            console.log('ROS1 bag playback started');
        } else {
            alert('Failed to start ROS1 playback: ' + (result.message || 'Unknown error'));
        }
        return;
    }

    // ROS2 bag: topics + rate 전달
    const result = await apiCall('/api/bag/play', {
        topics: bagPlayerState.selectedTopics,
        rate: bagPlayerState.playbackRate
    });
    if (result.success) {
        const button = domCache.get('bag-play-button');
        button.textContent = result.playing ? 'Stop' : 'Play';
        console.log('Bag playback:', result.playing ? 'started' : 'stopped',
                    `(rate=${bagPlayerState.playbackRate}x)`);
    } else {
        alert('Failed to play bag file: ' + (result.message || 'Unknown error'));
    }
}

async function pauseBag() {
    // ROS1 bag: /api/bag/pause_ros1 경로로 분기
    if (bagPlayerState.bagType === 'ros1') {
        const result = await apiCall('/api/bag/pause_ros1', {});
        if (result.success) {
            const button = domCache.get('bag-pause-button');
            button.textContent = result.paused ? 'Resume' : 'Pause';
            console.log('ROS1 bag playback:', result.paused ? 'paused' : 'resumed');
        } else {
            console.error('Failed to pause/resume ROS1 bag');
        }
        return;
    }

    // ROS2 bag: 기존 경로 유지
    const result = await apiCall('/api/bag/pause', {});
    if (result.success) {
        const button = domCache.get('bag-pause-button');
        button.textContent = result.paused ? 'Resume' : 'Pause';
        console.log('Bag playback:', result.paused ? 'paused' : 'resumed');
    } else {
        console.error('Failed to pause/resume bag');
    }
}

async function setBagPosition(position) {
    console.log('Setting bag position:', position);
    _bagSeeking = true;  // ROS2 bag seek 중 잠깐 playing=false 구간에서 슬라이더 리셋 방지
    await apiCall('/api/bag/set_position', { position: parseInt(position) });

    // Update time label
    const ratio = position / 10000.0;
    const currentTime = ratio * bagPlayerState.bagDuration;
    updateBagTimeLabel(currentTime, bagPlayerState.bagDuration);
}

async function updateBagState() {
    const state = await apiCall('/api/bag/state');
    if (state?.path && state.duration > 0 && bagPlayerState.bagDuration <= 0) {
        bagPlayerState.bagDuration = state.duration;
    }
    const isRos1 = bagPlayerState.bagType === 'ros1'
        || !!(state?.path && state.path.endsWith('.bag'));
    if (isRos1) bagPlayerState.bagType = 'ros1';

    // ROS1 bag 재생 중이면 /api/bag/ros1_play_status 폴링
    if (isRos1) {
        const ros1State = await apiCall('/api/bag/ros1_play_status');
        if (ros1State) {
            const { status, elapsed_sec, total_sec } = ros1State;

            // 버튼 상태 업데이트 (슬라이더 리셋 전에 먼저 처리)
            const playButton = domCache.get('bag-play-button');
            const pauseButton = domCache.get('bag-pause-button');

            if (status === 'stopped') {
                // 재생 완료 → 버튼 초기화
                if (playButton) {
                    playButton.textContent = 'Play';
                }
                if (pauseButton) {
                    pauseButton.textContent = 'Pause';
                }
                // 슬라이더가 0이 아니거나 방금 재생 중이었던 경우 처음으로 리셋
                const slider = domCache.get('bag-slider');
                if (slider && (bagPlayerState.wasPlaying || parseInt(slider.value, 10) > 0)) {
                    slider.value = 0;
                    updateBagTimeLabel(0, bagPlayerState.bagDuration);
                }
                bagPlayerState.wasPlaying = false;
            } else if (status === 'playing') {
                bagPlayerState.wasPlaying = true;
                if (playButton) {
                    playButton.textContent = 'Stop';
                }
                if (pauseButton) {
                    pauseButton.textContent = 'Pause';
                }
            } else if (status === 'paused') {
                if (playButton) {
                    playButton.textContent = 'Stop';
                }
                if (pauseButton) {
                    pauseButton.textContent = 'Resume';
                }
            }

            // Progress bar(슬라이더) 업데이트: 재생/일시정지 중일 때만 current_time으로 덮어씀
            if (status === 'playing' || status === 'paused') {
                const duration = total_sec || bagPlayerState.bagDuration;
                if (duration > 0 && elapsed_sec !== undefined) {
                    const ratio = elapsed_sec / duration;
                    const sliderValue = Math.floor(ratio * 10000);
                    const slider = domCache.get('bag-slider');
                    if (slider && !_bagSliderDragging) {
                        // 루프 감지: elapsed가 높은 값에서 0 근처로 떨어지면 강제 업데이트
                        const loopDetected = (sliderValue < 500 && parseInt(slider.value, 10) > 9500);
                        if (loopDetected) {
                            slider.value = sliderValue;
                        } else {
                            slider.value = sliderValue;
                        }
                    }
                    updateBagTimeLabel(elapsed_sec, duration);
                }
            }
        }
        // Loop 체크박스 동기화
        if (state && state.loop !== undefined) {
            const loopCb = domCache.get('bag-player-loop');
            if (loopCb) {
                loopCb.checked = state.loop;
            }
        }
        return;
    }

    // ROS2 bag: 기존 폴링 유지
    if (state) {
        // Update play button state (슬라이더 리셋 전에 먼저 처리)
        const playButton = domCache.get('bag-play-button');
        if (state.playing) {
            bagPlayerState.wasPlaying = true;
            _bagSeeking = false;  // 재생 재개 확인 → seek 플래그 해제
            playButton.textContent = 'Stop';
        } else {
            playButton.textContent = 'Play';
            // seek 처리 중(ROS2 stop→restart 과도 구간)에는 슬라이더 리셋 금지
            if (!_bagSeeking) {
                const slider = domCache.get('bag-slider');
                if (slider && (bagPlayerState.wasPlaying || parseInt(slider.value, 10) > 0)) {
                    slider.value = 0;
                    updateBagTimeLabel(0, bagPlayerState.bagDuration);
                }
                bagPlayerState.wasPlaying = false;
            }
        }

        // Update slider position: 재생/일시정지 중일 때만 current_time으로 덮어씀 (정지 후 리셋 위치를 보존)
        if ((state.playing || state.paused) && bagPlayerState.bagDuration > 0 && state.current_time !== undefined) {
            const ratio = state.current_time / bagPlayerState.bagDuration;
            const sliderValue = Math.floor(ratio * 10000);

            const slider = domCache.get('bag-slider');
            if (slider && !_bagSliderDragging) {
                // ROS2 루프 감지: current_time이 0 근처로 떨어지면 강제 업데이트
                const loopDetected = (sliderValue < 500 && parseInt(slider.value, 10) > 9500);
                if (loopDetected) {
                    slider.value = sliderValue;
                } else {
                    slider.value = sliderValue;
                }
            }

            updateBagTimeLabel(state.current_time, bagPlayerState.bagDuration);
        }

        // Update pause button state
        const pauseButton = domCache.get('bag-pause-button');
        if (state.paused) {
            pauseButton.textContent = 'Resume';
        } else {
            pauseButton.textContent = 'Pause';
        }

        // Loop 체크박스 동기화
        const loopCb = domCache.get('bag-player-loop');
        if (loopCb && state.loop !== undefined) {
            loopCb.checked = state.loop;
        }
    }
}

/**
 * 재생 속도 슬라이더 변경 핸들러 (ROS1/ROS2 공용)
 * 재생 중이거나 일시정지 중이면 즉시 API를 호출해 배속을 반영한다.
 * @param {string|number} sliderValue - 슬라이더 값 (1~40, 실제 속도 = value / 10)
 */
function updatePlaybackRate(sliderValue) {
    const rate = parseFloat(sliderValue) / 10.0;
    bagPlayerState.playbackRate = rate;
    const label = domCache.get('playback-rate-label');
    if (label) {
        label.textContent = `${rate.toFixed(1)}x`;
    }

    // 재생 중(Play → Stop 버튼) 또는 일시정지 중이면 즉시 배속 변경 API 호출
    const playButton = domCache.get('bag-play-button');
    const pauseButton = domCache.get('bag-pause-button');
    const isActive = playButton && playButton.textContent === 'Stop';
    const isPaused = pauseButton && pauseButton.textContent === 'Resume';

    if (isActive || isPaused) {
        applyPlaybackRateLive(rate);
    }
}

/**
 * 재생/일시정지 중 배속을 서버에 즉시 반영
 * @param {number} rate - 재생 속도 배율
 */
async function applyPlaybackRateLive(rate) {
    try {
        const result = await apiCall('/api/bag/set_rate', {
            rate: rate,
            bag_type: bagPlayerState.bagType  // 'ros1' or 'ros2'
        });
        if (result.success) {
            console.log(`[Playback Rate] Applied ${rate.toFixed(1)}x live (${bagPlayerState.bagType})`);
        } else {
            console.warn('[Playback Rate] Live rate change failed:', result.message);
        }
    } catch (e) {
        console.warn('[Playback Rate] Live rate change error:', e);
    }
}

/**
 * bag player 변환 포맷 select 기본값 설정 (현재 포맷과 다른 타깃)
 */
function updatePlayerFormatSelectDefault() {
    const select = domCache.get('player-format-select');
    if (!select) return;

    const currentFormat = bagPlayerState.bagFormat
        || (bagPlayerState.bagType === 'ros1' ? 'ros1' : 'ros2_db3');
    const defaultTargets = {
        ros1: 'ros2_mcap',
        ros2_db3: 'ros1',
        ros2_mcap: 'ros1',
    };
    select.value = defaultTargets[currentFormat] || 'ros2_mcap';
}

/**
 * 선택한 포맷으로 bag 변환
 * Selected Topics만 변환된 bag에 저장
 * POST /api/bag/convert 호출 후 변환된 bag 자동 로드
 */
async function convertBag() {
    const bagPath = domCache.get('bag-directory').value;
    if (!bagPath) {
        alert('Please load a bag file first');
        return;
    }

    if (!bagPlayerState.selectedTopics || bagPlayerState.selectedTopics.length === 0) {
        alert('Please select at least one topic before convert');
        return;
    }

    const targetFormat = domCache.get('player-format-select').value;
    const currentFormat = bagPlayerState.bagFormat
        || (bagPlayerState.bagType === 'ros1' ? 'ros1' : 'ros2_db3');
    if (targetFormat === currentFormat) {
        alert('Bag is already in the selected format');
        return;
    }

    const btn = domCache.get('convert-bag-btn');
    const originalText = btn.textContent;
    btn.disabled = true;
    btn.textContent = 'Converting...';

    try {
        const result = await apiCall('/api/bag/convert', {
            format: targetFormat,
            topics: bagPlayerState.selectedTopics,
        });
        btn.disabled = false;
        btn.textContent = originalText;

        if (result.success) {
            alert(`Conversion complete!\nOutput: ${result.output_path}`);

            const outputPath = result.output_path;
            if (outputPath) {
                domCache.get('bag-directory').value = outputPath;
                const loadResult = await apiCall('/api/bag/load', { path: outputPath });
                if (loadResult.success) {
                    applyBagPlayerInfo(outputPath, loadResult, { resetSlider: true });
                    if (typeof resetViewerTopicSubscriptions === 'function') {
                        resetViewerTopicSubscriptions();
                    }
                    if (typeof resetBagFrameAndTFState === 'function') {
                        resetBagFrameAndTFState();
                    }
                    console.log('Converted bag loaded:', outputPath);
                }
            }
        } else {
            alert('Conversion failed: ' + (result.error || 'Unknown error'));
        }
    } catch (error) {
        console.error('convertBag error:', error);
        alert('Conversion failed: ' + error.message);
        btn.disabled = false;
        btn.textContent = originalText;
    }
}

// File Player Functions

/**
 * 데이터셋 형식 변경 핸들러 (ConPR / KITTI Raw / KAIST Complex Urban / MulRan)
 * @param {string} format - 선택된 형식 ('conpr', 'kitti', 'kaist', 'mulran')
 */
function onDatasetFormatChange(format) {
    const kittiUi = domCache.get('kitti-ui');
    const kaistUi = domCache.get('kaist-ui');
    const mulranUi = domCache.get('mulran-ui');
    const heliprUi = domCache.get('helipr-ui');
    const conprSaveRow = domCache.get('conpr-save-row');

    // 모든 데이터셋 전용 UI를 우선 숨김 처리 (분기마다 반복 방지)
    const hideAll = () => {
        if (kittiUi) { kittiUi.style.display = 'none'; }
        if (kaistUi) { kaistUi.style.display = 'none'; }
        if (mulranUi) { mulranUi.style.display = 'none'; }
        if (heliprUi) { heliprUi.style.display = 'none'; }
    };

    if (format === 'kitti') {
        hideAll();
        kittiUi.style.display = 'block';
        if (conprSaveRow) { conprSaveRow.style.display = 'none'; }
        kittiState.baseDir = null;
        kittiState.calibDir = null;
        kittiState.drives = [];
        domCache.get('player-path-label').textContent = '—';
        _resetKittiDriveSelect();
        _resetKittiProgressBar();
    } else if (format === 'kaist') {
        hideAll();
        if (kaistUi) { kaistUi.style.display = 'block'; }
        if (conprSaveRow) { conprSaveRow.style.display = 'none'; }
        kaistState.baseDir = null;
        kaistState.sequences = [];
        domCache.get('player-path-label').textContent = '—';
        _resetKaistSequenceSelect();
        _resetKaistProgressBar();
    } else if (format === 'mulran') {
        hideAll();
        if (mulranUi) { mulranUi.style.display = 'block'; }
        if (conprSaveRow) { conprSaveRow.style.display = 'none'; }
        mulranState.baseDir = null;
        mulranState.sequences = [];
        domCache.get('player-path-label').textContent = '—';
        _resetMulranSequenceSelect();
        _resetMulranProgressBar();
    } else if (format === 'helipr') {
        hideAll();
        if (heliprUi) { heliprUi.style.display = 'block'; }
        if (conprSaveRow) { conprSaveRow.style.display = 'none'; }
        heliprState.baseDir = null;
        heliprState.sequences = [];
        domCache.get('player-path-label').textContent = '—';
        _resetHeliprSequenceSelect();
        _resetHeliprProgressBar();
    } else {
        hideAll();
        if (conprSaveRow) { conprSaveRow.style.display = ''; }
    }
}

/**
 * KITTI 드라이브 선택 셀렉트를 초기 상태로 리셋
 */
function _resetKittiDriveSelect() {
    const sel = domCache.get('kitti-drive-select');
    sel.innerHTML = '<option value="">— Select a drive —</option>';
}

/**
 * KITTI 변환 진행바 리셋
 */
function _resetKittiProgressBar() {
    const bar = domCache.get('kitti-progress-bar');
    const fill = domCache.get('kitti-progress-fill');
    const text = domCache.get('kitti-progress-text');
    const msg = domCache.get('kitti-progress-msg');
    if (bar) { bar.style.display = 'none'; }
    if (fill) { fill.style.width = '0%'; }
    if (text) { text.textContent = '0%'; }
    if (msg) { msg.textContent = ''; }
}

/**
 * KAIST 시퀀스 선택 셀렉트를 초기 상태로 리셋
 */
function _resetKaistSequenceSelect() {
    const sel = domCache.get('kaist-sequence-select');
    if (sel) { sel.innerHTML = '<option value="">— Select a sequence —</option>'; }
}

/**
 * KAIST 변환 진행바 리셋
 */
function _resetKaistProgressBar() {
    const bar = domCache.get('kaist-progress-bar');
    const fill = domCache.get('kaist-progress-fill');
    const text = domCache.get('kaist-progress-text');
    const msg = domCache.get('kaist-progress-msg');
    if (bar) { bar.style.display = 'none'; }
    if (fill) { fill.style.width = '0%'; }
    if (text) { text.textContent = '0%'; }
    if (msg) { msg.textContent = ''; }
}

/**
 * MulRan 시퀀스 선택 셀렉트를 초기 상태로 리셋
 */
function _resetMulranSequenceSelect() {
    const sel = domCache.get('mulran-sequence-select');
    if (sel) { sel.innerHTML = '<option value="">— Select a sequence —</option>'; }
}

/**
 * MulRan 변환 진행바 리셋
 */
function _resetMulranProgressBar() {
    const bar = domCache.get('mulran-progress-bar');
    const fill = domCache.get('mulran-progress-fill');
    const text = domCache.get('mulran-progress-text');
    const msg = domCache.get('mulran-progress-msg');
    if (bar) { bar.style.display = 'none'; }
    if (fill) { fill.style.width = '0%'; }
    if (text) { text.textContent = '0%'; }
    if (msg) { msg.textContent = ''; }
}

/**
 * HeLiPR 시퀀스 선택 셀렉트를 초기 상태로 리셋
 */
function _resetHeliprSequenceSelect() {
    const sel = domCache.get('helipr-sequence-select');
    if (sel) { sel.innerHTML = '<option value="">— Select a sequence —</option>'; }
}

/**
 * HeLiPR 변환 진행바 리셋
 */
function _resetHeliprProgressBar() {
    const bar = domCache.get('helipr-progress-bar');
    const fill = domCache.get('helipr-progress-fill');
    const text = domCache.get('helipr-progress-text');
    const msg = domCache.get('helipr-progress-msg');
    if (bar) { bar.style.display = 'none'; }
    if (fill) { fill.style.width = '0%'; }
    if (text) { text.textContent = '0%'; }
    if (msg) { msg.textContent = ''; }
}

/**
 * KITTI 디렉토리 탐색: scan_kitti API 호출 후 drive 목록 업데이트
 * 파일 브라우저에서 KITTI date 디렉토리 선택 후 호출됨
 */
async function loadKittiDirectory() {
    openFileBrowser(async (path) => {
        domCache.get('player-path-label').textContent = 'Scanning...';
        _resetKittiDriveSelect();
        _resetKittiProgressBar();

        const result = await apiCall('/api/player/scan_kitti', { path });
        if (!result.success) {
            domCache.get('player-path-label').textContent = 'Scan failed';
            alert('KITTI scan failed: ' + (result.error || result.message || 'Unknown error'));
            return;
        }

        const scan = result.scan_result;
        kittiState.baseDir = path;
        kittiState.calibDir = scan.calib_dir || null;
        kittiState.drives = scan.drive_dirs || [];

        domCache.get('player-path-label').textContent = path;

        // drive 목록을 select에 채우기
        const sel = domCache.get('kitti-drive-select');
        sel.innerHTML = '<option value="">— Select a drive —</option>';
        kittiState.drives.forEach((drive, idx) => {
            const opt = document.createElement('option');
            opt.value = idx;
            opt.textContent = `${drive.name} [${drive.drive_type}]`;
            sel.appendChild(opt);
        });

        if (kittiState.drives.length === 0) {
            alert('No drive directories found in the selected KITTI directory.');
        } else {
            // 항상 "Select a drive" 기본값 유지 - 사용자가 직접 선택
            console.log(`[KITTI] Found ${kittiState.drives.length} drive(s) in ${path}`);
        }
    }, '~');
}

/** File Player load_data 성공 시 이전 백/뷰어 상태 전부 비우고 서버 PC2 목록만 다시 연결 */
function applyPlayerLoadDataViewerSync(result) {
    if (!result || !result.success) return;

    // 데이터 전환 전에 현재 이미지 구독 토픽 저장 (리셋 후 자동 재구독용)
    const prevImageTopics = (typeof viewer3DState !== 'undefined' && viewer3DState.imageSubscriptions)
        ? Array.from(viewer3DState.imageSubscriptions.keys())
        : [];

    if (typeof resetViewerTopicSubscriptions === 'function') {
        resetViewerTopicSubscriptions();  // _detachAllStreamWorkers → imageSubscriptions 초기화
    }
    if (typeof syncPlayerFilePointCloudSubscriptions === 'function') {
        syncPlayerFilePointCloudSubscriptions(result.player_pc2_topics);
    }

    // 이전 이미지 구독 복원: 데이터 전환 후에도 이미지 패널이 자동으로 이어짐
    if (prevImageTopics.length > 0 && typeof subscribeToImage === 'function') {
        prevImageTopics.forEach(function(topicName) {
            subscribeToImage(topicName);
        });
    }

    if (typeof resetBagFrameAndTFState === 'function') {
        resetBagFrameAndTFState();
    }
    if (typeof resetAll3DViewer === 'function') {
        resetAll3DViewer();
    }

    // 데이터 전환 후 /tf · /tf_static 백그라운드 구독 재시작:
    // resetViewerTopicSubscriptions 내부의 restartBackgroundTfPipeline이
    // rosConnected=false 타이밍 경쟁으로 실패했을 경우를 대비한 보강.
    // TRANSIENT_LOCAL /tf_static 재수신 → MulRan·KAIST 좌표 변환 보장.
    if (typeof window.startBackgroundFrameCollection === 'function') {
        window.startBackgroundFrameCollection();
    }
}

/**
 * Drive 드롭다운 선택 변경 시 자동 호출.
 * 선택된 drive를 load_data API로 바로 로드 → data_stamp 구축 → Play 버튼 활성.
 */
async function onKittiDriveChange(driveIdx) {
    if (driveIdx === '' || driveIdx === null || !kittiState.baseDir) return;
    const drive = kittiState.drives[parseInt(driveIdx)];
    if (!drive) return;

    domCache.get('player-path-label').textContent = 'Loading...';
    const result = await apiCall('/api/player/load_data', { path: drive.data_path });
    if (result && result.success) {
        domCache.get('player-path-label').textContent = drive.data_path;
        console.log('[KITTI] Drive auto-loaded:', drive.data_path);
        applyPlayerLoadDataViewerSync(result);

        // Auto-start: 체크박스가 켜져 있으면 로드 직후 자동 재생
        const autoStartCheck = domCache.get('player-auto-start');
        if (autoStartCheck && autoStartCheck.checked) {
            console.log('[KITTI] Auto start enabled — starting playback');
            await playPlayer();
        }
    } else {
        const errMsg = result ? (result.message || result.error || 'Unknown') : 'No response';
        domCache.get('player-path-label').textContent = 'Load failed';
        console.error('[KITTI] Drive auto-load failed:', errMsg);
    }
}

/**
 * KITTI drive 디렉토리를 File Player에 직접 로드한다 (변환 없이 파일에서 직접 재생).
 * drive의 data_path를 load_data API에 전달 → 백엔드가 timestamps를 읽어 data_stamp 구축.
 */
async function loadKittiDrive() {
    const sel = domCache.get('kitti-drive-select');
    const driveIdx = sel.value;
    if (driveIdx === '' || driveIdx === null) {
        alert('Please select a drive first.');
        return;
    }
    if (!kittiState.baseDir) {
        alert('Please load a KITTI directory first.');
        return;
    }

    const drive = kittiState.drives[parseInt(driveIdx)];
    if (!drive) {
        alert('Invalid drive selection.');
        return;
    }

    const btn = domCache.get('kitti-convert-btn');
    btn.disabled = true;
    btn.textContent = 'Loading…';

    domCache.get('player-path-label').textContent = 'Loading...';

    const result = await apiCall('/api/player/load_data', { path: drive.data_path });

    btn.disabled = false;
    btn.textContent = 'Load';

    if (result && result.success) {
        applyPlayerLoadDataViewerSync(result);
        domCache.get('player-path-label').textContent = drive.data_path;
        console.log('[KITTI] Drive loaded:', drive.data_path);
    } else {
        const errMsg = result ? (result.message || result.error || 'Unknown error') : 'No response';
        domCache.get('player-path-label').textContent = 'Load failed';
        alert('Failed to load KITTI drive: ' + errMsg);
    }
}

/**
 * KITTI 변환 완료 후 처리: 진행바 완료 표시 → load_data로 재생 시작
 * @param {string} bagPath - 생성된 ROS2 bag 파일 경로
 * @param {HTMLElement} btn - Convert 버튼 엘리먼트
 * @param {HTMLElement} bar - 진행바 컨테이너 엘리먼트
 * @param {HTMLElement} fill - 진행바 fill 엘리먼트
 * @param {HTMLElement} text - 진행바 텍스트 엘리먼트
 * @param {HTMLElement} msg - 상태 메시지 엘리먼트
 */
/**
 * KITTI 데이터를 ROS2 bag으로 변환 (Save Bag).
 * 현재 선택된 drive를 /api/player/convert_kitti 로 전송.
 * 진행률은 WebSocket(8081)을 통해 수신.
 */
async function convertKitti() {
    const sel = domCache.get('kitti-drive-select');
    const driveIdx = sel ? sel.value : '';
    if (driveIdx === '' || driveIdx === null) {
        alert('먼저 드라이브를 선택하세요.');
        return;
    }
    if (!kittiState.baseDir) {
        alert('KITTI 디렉토리를 먼저 로드하세요.');
        return;
    }

    const drive = kittiState.drives[parseInt(driveIdx)];
    if (!drive) {
        alert('유효하지 않은 드라이브 선택입니다.');
        return;
    }

    const calibDir = drive.calib_dir || kittiState.calibDir;
    if (!calibDir) {
        alert('Calibration 디렉토리를 찾을 수 없습니다.\n날짜 디렉토리(예: 2011_09_30)에 *_calib 폴더가 있어야 합니다.');
        return;
    }

    if (kittiState.converting) {
        alert('이미 변환 중입니다.');
        return;
    }

    const bagFormatSel = domCache.get('kitti-bag-format-select');
    const bagFormat = bagFormatSel ? bagFormatSel.value : 'ros2_db3';
    kittiState.bagFormat = bagFormat;

    const btn   = domCache.get('kitti-convert-btn');
    const bar   = domCache.get('kitti-progress-bar');
    const fill  = domCache.get('kitti-progress-fill');
    const text  = domCache.get('kitti-progress-text');
    const msgEl = domCache.get('kitti-progress-msg');

    kittiState.converting = true;
    btn.disabled = true;
    btn.textContent = `Saving (${BAG_FORMAT_LABELS[bagFormat] || bagFormat})…`;

    if (bar)   { bar.style.display = 'block'; }
    if (fill)  { fill.style.width = '0%'; }
    if (text)  { text.textContent = '0%'; }
    if (msgEl) { msgEl.textContent = 'Starting conversion...'; }

    const result = await apiCall('/api/player/convert_kitti', {
        base_dir:   kittiState.baseDir,
        calib_dir:  calibDir,
        data_path:  drive.data_path,
        drive_name: drive.name,
        bag_format: bagFormat,
    });

    if (!result || !result.success) {
        kittiState.converting = false;
        btn.disabled = false;
        btn.textContent = 'Save Bag';
        if (bar) bar.style.display = 'none';
        const errMsg = result ? (result.error || result.message || 'Unknown') : 'No response';
        alert('변환 시작 실패: ' + errMsg);
    }
    // 진행률·완료·오류는 _handleBackendWsMessage의 WebSocket 핸들러에서 처리
}

/**
 * KAIST 디렉토리 탐색: scan_kaist API 호출 후 시퀀스 목록 업데이트
 * 파일 브라우저에서 KAIST base 디렉토리 선택 후 호출됨
 */
async function loadKaistDirectory() {
    openFileBrowser(async (path) => {
        domCache.get('player-path-label').textContent = 'Scanning...';
        _resetKaistSequenceSelect();
        _resetKaistProgressBar();

        const result = await apiCall('/api/player/scan_kaist', { path });
        if (!result.success) {
            domCache.get('player-path-label').textContent = 'Scan failed';
            alert('KAIST scan failed: ' + (result.error || result.message || 'Unknown error'));
            return;
        }

        const sequences = result.sequences || [];
        kaistState.baseDir = path;
        kaistState.sequences = sequences;

        domCache.get('player-path-label').textContent = path;

        const sel = domCache.get('kaist-sequence-select');
        if (sel) {
            sel.innerHTML = '<option value="">— Select a sequence —</option>';
            sequences.forEach((seq, idx) => {
                const opt = document.createElement('option');
                opt.value = idx;
                opt.textContent = seq.name || seq.path || `Sequence ${idx}`;
                sel.appendChild(opt);
            });
        }

        if (sequences.length === 0) {
            alert('No sequences found in the selected KAIST directory.');
        } else {
            console.log(`[KAIST] Found ${sequences.length} sequence(s) in ${path}`);
        }
    }, '~');
}

/**
 * KAIST 시퀀스 드롭다운 선택 변경 시 자동 호출.
 * 선택된 시퀀스를 load_data API로 바로 로드 → Direct Play 활성화.
 */
async function onKaistSequenceChange(seqIdx) {
    if (seqIdx === '' || seqIdx === null || !kaistState.baseDir) return;
    const seq = kaistState.sequences[parseInt(seqIdx)];
    if (!seq) return;

    domCache.get('player-path-label').textContent = 'Loading...';
    const sequencePath = seq.path || seq;
    const result = await apiCall('/api/player/load_data', { path: sequencePath });
    if (result && result.success) {
        domCache.get('player-path-label').textContent = sequencePath;
        console.log('[KAIST] Sequence auto-loaded:', sequencePath);
        applyPlayerLoadDataViewerSync(result);

        const autoStartCheck = domCache.get('player-auto-start');
        if (autoStartCheck && autoStartCheck.checked) {
            console.log('[KAIST] Auto start enabled — starting playback');
            await playPlayer();
        }
    } else {
        const errMsg = result ? (result.message || result.error || 'Unknown') : 'No response';
        domCache.get('player-path-label').textContent = 'Load failed';
        console.error('[KAIST] Sequence auto-load failed:', errMsg);
    }
}

/**
 * KAIST 시퀀스를 ROS2 bag으로 변환 (Save Bag).
 * 현재 선택된 시퀀스를 /api/player/convert_kaist로 전송.
 * 진행률은 WebSocket(8081)을 통해 수신.
 */
async function convertKaist() {
    const sel = domCache.get('kaist-sequence-select');
    const seqIdx = sel ? sel.value : '';
    if (seqIdx === '' || seqIdx === null) {
        alert('먼저 시퀀스를 선택하세요.');
        return;
    }
    if (!kaistState.baseDir) {
        alert('KAIST 디렉토리를 먼저 로드하세요.');
        return;
    }

    const seq = kaistState.sequences[parseInt(seqIdx)];
    if (!seq) {
        alert('유효하지 않은 시퀀스 선택입니다.');
        return;
    }

    const sequenceDir = seq.path || seq;
    if (kaistState.converting) {
        alert('이미 변환 중입니다.');
        return;
    }

    const bagFormatSel = domCache.get('kaist-bag-format-select');
    const bagFormat = bagFormatSel ? bagFormatSel.value : 'ros2_db3';

    const btn   = domCache.get('kaist-convert-btn');
    const bar   = domCache.get('kaist-progress-bar');
    const fill  = domCache.get('kaist-progress-fill');
    const text  = domCache.get('kaist-progress-text');
    const msgEl = domCache.get('kaist-progress-msg');

    kaistState.converting = true;
    if (btn) { btn.disabled = true; btn.textContent = `Saving (${BAG_FORMAT_LABELS[bagFormat] || bagFormat})…`; }
    if (bar) { bar.style.display = 'block'; }
    if (fill) { fill.style.width = '0%'; }
    if (text) { text.textContent = '0%'; }
    if (msgEl) { msgEl.textContent = 'Starting conversion...'; }

    // output_path: 시퀀스 디렉토리와 같은 위치에 _converted 추가 (백엔드가 확장자 처리)
    const outputPath = sequenceDir + '_converted';

    const result = await apiCall('/api/player/convert_kaist', {
        sequence_dir: sequenceDir,
        output_path: outputPath,
        bag_format: bagFormat
    });

    if (!result || !result.success) {
        kaistState.converting = false;
        if (btn) { btn.disabled = false; btn.textContent = 'Save Bag'; }
        if (bar) { bar.style.display = 'none'; }
        const errMsg = result ? (result.error || result.message || 'Unknown') : 'No response';
        alert('변환 시작 실패: ' + errMsg);
    }
    // 진행률·완료·오류는 _handleBackendWsMessage의 WebSocket 핸들러에서 처리
}

async function _onKaistConvertDone(bagPath, btn, bar, fill, text, msg) {
    if (fill) { fill.style.width = '100%'; }
    if (text) { text.textContent = '100%'; }
    if (msg) { msg.textContent = 'Conversion complete! Loading bag...'; }

    const loadResult = await apiCall('/api/player/load_data', { path: bagPath });
    if (loadResult && loadResult.success) {
        domCache.get('player-path-label').textContent = bagPath;
        if (msg) { msg.textContent = 'Ready to play'; }
        console.log('[KAIST] Bag loaded:', bagPath);
        applyPlayerLoadDataViewerSync(loadResult);
    } else {
        if (msg) { msg.textContent = 'Load failed'; }
        alert('Failed to load converted bag: ' + (loadResult ? (loadResult.message || loadResult.error || 'Unknown error') : 'No response'));
    }

    kaistState.converting = false;
    if (btn) { btn.disabled = false; btn.textContent = 'Save Bag'; }
}

// ── MulRan ────────────────────────────────────────────────────────────────────

/**
 * MulRan 디렉토리 탐색: scan_mulran API 호출 후 시퀀스 목록 업데이트
 * ``.../Mulran`` 상위만 고르면 ParkingLot·DCC01 등 하위 시퀀스가 드롭다운에 채워지고,
 * 시퀀스가 1개면 자동으로 load_data까지 수행한다.
 */
async function loadMulranDirectory() {
    openFileBrowser(async (path) => {
        domCache.get('player-path-label').textContent = 'Scanning...';
        _resetMulranSequenceSelect();
        _resetMulranProgressBar();

        const result = await apiCall('/api/player/scan_mulran', { path });
        if (!result.success) {
            domCache.get('player-path-label').textContent = 'Scan failed';
            alert('MulRan scan failed: ' + (result.error || result.message || 'Unknown error'));
            return;
        }

        const sequences = result.sequences || [];
        mulranState.baseDir = path;
        mulranState.sequences = sequences;

        domCache.get('player-path-label').textContent = path;

        const sel = domCache.get('mulran-sequence-select');
        if (sel) {
            sel.innerHTML = '<option value="">— Select a sequence —</option>';
            sequences.forEach((seq, idx) => {
                const opt = document.createElement('option');
                opt.value = String(idx);
                opt.textContent = seq.name || seq.path || `Sequence ${idx}`;
                sel.appendChild(opt);
            });
        }

        if (sequences.length === 0) {
            alert('No MulRan sequences found in the selected directory.');
        } else {
            console.log(`[MulRan] Found ${sequences.length} sequence(s) in ${path}`);
            // 시퀀스가 하나뿐이면 드롭다운 선택·load_data 까지 자동 (상위 Mulran 폴더만 고른 경우)
            if (sequences.length === 1 && sel) {
                sel.value = '0';
                await onMulranSequenceChange('0');
            }
        }
    }, '~');
}

/**
 * MulRan 시퀀스 드롭다운 선택 변경 시 자동 호출.
 * 선택된 시퀀스를 load_data API로 바로 로드 → Direct Play 활성화.
 */
async function onMulranSequenceChange(seqIdx) {
    if (seqIdx === '' || seqIdx === null || !mulranState.baseDir) return;
    const seq = mulranState.sequences[parseInt(seqIdx)];
    if (!seq) return;

    domCache.get('player-path-label').textContent = 'Loading...';
    const sequencePath = seq.path || seq;
    const result = await apiCall('/api/player/load_data', { path: sequencePath });
    if (result && result.success) {
        domCache.get('player-path-label').textContent = sequencePath;
        console.log('[MulRan] Sequence auto-loaded:', sequencePath);
        applyPlayerLoadDataViewerSync(result);

        const autoStartCheck = domCache.get('player-auto-start');
        if (autoStartCheck && autoStartCheck.checked) {
            console.log('[MulRan] Auto start enabled — starting playback');
            await playPlayer();
        }
    } else {
        const errMsg = result ? (result.message || result.error || 'Unknown') : 'No response';
        domCache.get('player-path-label').textContent = 'Load failed';
        console.error('[MulRan] Sequence auto-load failed:', errMsg);
    }
}

/**
 * MulRan 시퀀스를 ROS bag으로 변환 (Save Bag).
 * 현재 선택된 시퀀스를 /api/player/convert_mulran 으로 전송.
 * 진행률은 WebSocket(8081)을 통해 수신.
 */
async function convertMulran() {
    const sel = domCache.get('mulran-sequence-select');
    const seqIdx = sel ? sel.value : '';
    if (seqIdx === '' || seqIdx === null) {
        alert('먼저 시퀀스를 선택하세요.');
        return;
    }
    if (!mulranState.baseDir) {
        alert('MulRan 디렉토리를 먼저 로드하세요.');
        return;
    }

    const seq = mulranState.sequences[parseInt(seqIdx)];
    if (!seq) {
        alert('유효하지 않은 시퀀스 선택입니다.');
        return;
    }

    const sequenceDir = seq.path || seq;
    if (mulranState.converting) {
        alert('이미 변환 중입니다.');
        return;
    }

    const bagFormatSel = domCache.get('mulran-bag-format-select');
    const bagFormat = bagFormatSel ? bagFormatSel.value : 'ros2_db3';

    const btn   = domCache.get('mulran-convert-btn');
    const bar   = domCache.get('mulran-progress-bar');
    const fill  = domCache.get('mulran-progress-fill');
    const text  = domCache.get('mulran-progress-text');
    const msgEl = domCache.get('mulran-progress-msg');

    mulranState.converting = true;
    if (btn) { btn.disabled = true; btn.textContent = `Saving (${BAG_FORMAT_LABELS[bagFormat] || bagFormat})…`; }
    if (bar) { bar.style.display = 'block'; }
    if (fill) { fill.style.width = '0%'; }
    if (text) { text.textContent = '0%'; }
    if (msgEl) { msgEl.textContent = 'Starting conversion...'; }

    const outputPath = sequenceDir + '_converted';

    const result = await apiCall('/api/player/convert_mulran', {
        sequence_dir: sequenceDir,
        output_path: outputPath,
        bag_format: bagFormat
    });

    if (!result || !result.success) {
        mulranState.converting = false;
        if (btn) { btn.disabled = false; btn.textContent = 'Save Bag'; }
        if (bar) { bar.style.display = 'none'; }
        const errMsg = result ? (result.error || result.message || 'Unknown') : 'No response';
        alert('변환 시작 실패: ' + errMsg);
    }
    // 진행률·완료·오류는 _handleBackendWsMessage의 WebSocket 핸들러에서 처리
}

/**
 * MulRan 변환 완료 후 처리: 진행바 완료 표시 → load_data로 자동 로드
 */
async function _onMulranConvertDone(bagPath, btn, bar, fill, text, msg) {
    if (fill) { fill.style.width = '100%'; }
    if (text) { text.textContent = '100%'; }
    if (msg) { msg.textContent = 'Conversion complete! Loading bag...'; }

    const loadResult = await apiCall('/api/player/load_data', { path: bagPath });
    if (loadResult && loadResult.success) {
        domCache.get('player-path-label').textContent = bagPath;
        if (msg) { msg.textContent = 'Ready to play'; }
        console.log('[MulRan] Bag loaded:', bagPath);
        applyPlayerLoadDataViewerSync(loadResult);
    } else {
        if (msg) { msg.textContent = 'Load failed'; }
        alert('Failed to load converted bag: ' + (loadResult ? (loadResult.message || loadResult.error || 'Unknown error') : 'No response'));
    }

    mulranState.converting = false;
    if (btn) { btn.disabled = false; btn.textContent = 'Save Bag'; }
}

// ── HeLiPR ────────────────────────────────────────────────────────────────────

/**
 * HeLiPR 디렉토리 탐색: scan_helipr API 호출 후 시퀀스 목록 업데이트
 * ``.../HeLiPR`` 상위만 고르면 각 시퀀스가 드롭다운에 채워지고,
 * 시퀀스가 1개면 자동으로 load_data까지 수행한다.
 */
async function loadHeliprDirectory() {
    openFileBrowser(async (path) => {
        domCache.get('player-path-label').textContent = 'Scanning...';
        _resetHeliprSequenceSelect();
        _resetHeliprProgressBar();

        const result = await apiCall('/api/player/scan_helipr', { path });
        if (!result.success) {
            domCache.get('player-path-label').textContent = 'Scan failed';
            alert('HeLiPR scan failed: ' + (result.error || result.message || 'Unknown error'));
            return;
        }

        const sequences = result.sequences || [];
        heliprState.baseDir = path;
        heliprState.sequences = sequences;

        domCache.get('player-path-label').textContent = path;

        const sel = domCache.get('helipr-sequence-select');
        if (sel) {
            sel.innerHTML = '<option value="">— Select a sequence —</option>';
            sequences.forEach((seq, idx) => {
                const opt = document.createElement('option');
                opt.value = String(idx);
                opt.textContent = seq.name || seq.path || `Sequence ${idx}`;
                sel.appendChild(opt);
            });
        }

        if (sequences.length === 0) {
            alert('No HeLiPR sequences found in the selected directory.');
        } else {
            console.log(`[HeLiPR] Found ${sequences.length} sequence(s) in ${path}`);
            // 시퀀스가 하나뿐이면 드롭다운 선택·load_data 까지 자동 (상위 HeLiPR 폴더만 고른 경우)
            if (sequences.length === 1 && sel) {
                sel.value = '0';
                await onHeliprSequenceChange('0');
            }
        }
    }, '~');
}

/**
 * HeLiPR 시퀀스 드롭다운 선택 변경 시 자동 호출.
 * 선택된 시퀀스를 load_data API로 바로 로드 → Direct Play 활성화.
 */
async function onHeliprSequenceChange(seqIdx) {
    if (seqIdx === '' || seqIdx === null || !heliprState.baseDir) return;
    const seq = heliprState.sequences[parseInt(seqIdx)];
    if (!seq) return;

    domCache.get('player-path-label').textContent = 'Loading...';
    const sequencePath = seq.path || seq;
    const result = await apiCall('/api/player/load_data', { path: sequencePath });
    if (result && result.success) {
        domCache.get('player-path-label').textContent = sequencePath;
        console.log('[HeLiPR] Sequence auto-loaded:', sequencePath);
        applyPlayerLoadDataViewerSync(result);

        const autoStartCheck = domCache.get('player-auto-start');
        if (autoStartCheck && autoStartCheck.checked) {
            console.log('[HeLiPR] Auto start enabled — starting playback');
            await playPlayer();
        }
    } else {
        const errMsg = result ? (result.message || result.error || 'Unknown') : 'No response';
        domCache.get('player-path-label').textContent = 'Load failed';
        console.error('[HeLiPR] Sequence auto-load failed:', errMsg);
    }
}

/**
 * HeLiPR 시퀀스를 ROS bag으로 변환 (Save Bag).
 * 현재 선택된 시퀀스를 /api/player/convert_helipr 으로 전송.
 * 진행률은 WebSocket(8081)을 통해 수신.
 */
async function convertHelipr() {
    const sel = domCache.get('helipr-sequence-select');
    const seqIdx = sel ? sel.value : '';
    if (seqIdx === '' || seqIdx === null) {
        alert('먼저 시퀀스를 선택하세요.');
        return;
    }
    if (!heliprState.baseDir) {
        alert('HeLiPR 디렉토리를 먼저 로드하세요.');
        return;
    }

    const seq = heliprState.sequences[parseInt(seqIdx)];
    if (!seq) {
        alert('유효하지 않은 시퀀스 선택입니다.');
        return;
    }

    const sequenceDir = seq.path || seq;
    if (heliprState.converting) {
        alert('이미 변환 중입니다.');
        return;
    }

    const bagFormatSel = domCache.get('helipr-bag-format-select');
    const bagFormat = bagFormatSel ? bagFormatSel.value : 'ros2_db3';

    const btn   = domCache.get('helipr-convert-btn');
    const bar   = domCache.get('helipr-progress-bar');
    const fill  = domCache.get('helipr-progress-fill');
    const text  = domCache.get('helipr-progress-text');
    const msgEl = domCache.get('helipr-progress-msg');

    heliprState.converting = true;
    if (btn) { btn.disabled = true; btn.textContent = `Saving (${BAG_FORMAT_LABELS[bagFormat] || bagFormat})…`; }
    if (bar) { bar.style.display = 'block'; }
    if (fill) { fill.style.width = '0%'; }
    if (text) { text.textContent = '0%'; }
    if (msgEl) { msgEl.textContent = 'Starting conversion...'; }

    const outputPath = sequenceDir + '_converted';

    const result = await apiCall('/api/player/convert_helipr', {
        sequence_dir: sequenceDir,
        output_path: outputPath,
        bag_format: bagFormat
    });

    if (!result || !result.success) {
        heliprState.converting = false;
        if (btn) { btn.disabled = false; btn.textContent = 'Save Bag'; }
        if (bar) { bar.style.display = 'none'; }
        const errMsg = result ? (result.error || result.message || 'Unknown') : 'No response';
        alert('변환 시작 실패: ' + errMsg);
    }
    // 진행률·완료·오류는 _handleBackendWsMessage의 WebSocket 핸들러에서 처리
}

/**
 * HeLiPR 변환 완료 후 처리: 진행바 완료 표시 → load_data로 자동 로드
 */
async function _onHeliprConvertDone(bagPath, btn, bar, fill, text, msg) {
    if (fill) { fill.style.width = '100%'; }
    if (text) { text.textContent = '100%'; }
    if (msg) { msg.textContent = 'Conversion complete! Loading bag...'; }

    const loadResult = await apiCall('/api/player/load_data', { path: bagPath });
    if (loadResult && loadResult.success) {
        domCache.get('player-path-label').textContent = bagPath;
        if (msg) { msg.textContent = 'Ready to play'; }
        console.log('[HeLiPR] Bag loaded:', bagPath);
        applyPlayerLoadDataViewerSync(loadResult);
    } else {
        if (msg) { msg.textContent = 'Load failed'; }
        alert('Failed to load converted bag: ' + (loadResult ? (loadResult.message || loadResult.error || 'Unknown error') : 'No response'));
    }

    heliprState.converting = false;
    if (btn) { btn.disabled = false; btn.textContent = 'Save Bag'; }
}

async function _onKittiConvertDone(bagPath, btn, bar, fill, text, msg) {
    // 진행바 100% 완료 표시
    fill.style.width = '100%';
    text.textContent = '100%';
    msg.textContent = 'Conversion complete! Loading bag...';

    // load_data API 호출하여 생성된 ROS2 bag 로드 (재생은 사용자가 직접 Play 버튼으로)
    const loadResult = await apiCall('/api/player/load_data', { path: bagPath });
    if (loadResult && loadResult.success) {
        domCache.get('player-path-label').textContent = bagPath;
        msg.textContent = 'Ready to play';
        console.log('[KITTI] Bag loaded:', bagPath);
        applyPlayerLoadDataViewerSync(loadResult);
    } else {
        msg.textContent = 'Load failed';
        alert('Failed to load converted bag: ' + (loadResult ? (loadResult.message || loadResult.error || 'Unknown error') : 'No response'));
    }

    kittiState.converting = false;
    btn.disabled = false;
    btn.textContent = 'Save Bag';
}

/**
 * 데이터셋 형식에 따라 파일/디렉토리 로드
 * ConPR 형식이면 기존 로직, KITTI/KAIST 형식이면 각각 loadKittiDirectory/loadKaistDirectory() 호출
 */
async function loadPlayerPath() {
    const formatSel = domCache.get('dataset-format-select');
    const format = formatSel ? formatSel.value : 'conpr';

    if (format === 'kitti') {
        await loadKittiDirectory();
        return;
    }
    if (format === 'kaist') {
        await loadKaistDirectory();
        return;
    }
    if (format === 'mulran') {
        await loadMulranDirectory();
        return;
    }
    if (format === 'helipr') {
        await loadHeliprDirectory();
        return;
    }

    // ConPR 기존 로직
    openFileBrowser(async (path) => {
        domCache.get('player-path-label').textContent = 'Loading...';
        const result = await apiCall('/api/player/load_data', { path });
        if (result.success) {
            domCache.get('player-path-label').textContent = path;
            console.log('Player data loaded successfully');
            applyPlayerLoadDataViewerSync(result);

            // Auto start: 체크박스가 켜져 있으면 로드 직후 자동 재생
            const autoStartCheck = domCache.get('player-auto-start');
            if (autoStartCheck && autoStartCheck.checked) {
                console.log('[File Player] Auto start enabled — starting playback');
                await playPlayer();
            }
        } else {
            domCache.get('player-path-label').textContent = 'Failed to load';
            alert('Failed to load player data: ' + result.message);
        }
    }, '~');
}

async function playPlayer() {
    const result = await apiCall('/api/player/play', {});
    if (result.success) {
        const button = domCache.get('play-button');
        button.textContent = result.playing ? 'End' : 'Play';
    }
}

async function pausePlayer() {
    const result = await apiCall('/api/player/pause', {});
    if (result.success) {
        const button = domCache.get('pause-button');
        button.textContent = result.paused ? 'Resume' : 'Pause';
    }
}

async function saveBag() {
    const bar = domCache.get('conpr-progress-bar');
    const fill = domCache.get('conpr-progress-fill');
    const text = domCache.get('conpr-progress-text');
    const msgEl = domCache.get('conpr-progress-msg');
    const bagFormatSel = domCache.get('bag-format-select');
    const saveBagBtn = domCache.get('save-bag-btn');

    if (bar && bar.style.display === 'block') {
        return; // 이미 저장 중
    }

    const bagFormat = bagFormatSel ? bagFormatSel.value : 'ros2_db3';
    const originalBtnText = saveBagBtn ? saveBagBtn.textContent : 'Save bag';

    // KITTI와 완전 동일한 레이아웃: 진행바+메시지+format select+버튼 모두 표시, 버튼만 비활성화
    if (bar) { bar.style.display = 'block'; }
    if (fill) { fill.style.width = '0%'; }
    if (text) { text.textContent = '0%'; }
    if (msgEl) { msgEl.textContent = 'Starting conversion...'; }
    if (bagFormatSel) { bagFormatSel.disabled = true; }
    if (saveBagBtn) {
        saveBagBtn.disabled = true;
        saveBagBtn.textContent = `Saving (${BAG_FORMAT_LABELS[bagFormat] || bagFormat})…`;
    }

    function setProgress(pct) {
        if (fill) { fill.style.width = pct + '%'; }
        if (text) { text.textContent = pct + '%'; }
    }

    function restoreUi(success) {
        if (bar) { bar.style.display = 'none'; }
        if (fill) { fill.style.width = '0%'; }
        if (text) { text.textContent = '0%'; }
        if (msgEl) { msgEl.textContent = ''; }
        if (bagFormatSel) { bagFormatSel.disabled = false; }
        if (saveBagBtn) {
            saveBagBtn.disabled = false;
            saveBagBtn.textContent = originalBtnText;
        }
        if (!success) {
            alert('Bag save failed.');
        }
    }

    // 저장 시작 (백그라운드 스레드 실행 — 즉시 응답)
    const startResult = await apiCall('/api/player/save_bag', { bag_format: bagFormat });
    if (!startResult || !startResult.success) {
        restoreUi(false);
        alert('Failed to start bag save: ' + (startResult ? startResult.message : 'Unknown error'));
        return;
    }

    // save_bag_saving이 false가 될 때까지 500ms마다 폴링
    const success = await new Promise((resolve) => {
        const interval = setInterval(async () => {
            const state = await apiCall('/api/player/state');
            if (!state) { return; }

            if (state.save_bag_progress !== null && state.save_bag_progress !== undefined) {
                const pct = parseInt(state.save_bag_progress);
                if (!isNaN(pct)) { setProgress(pct); }
                if (msgEl && state.save_bag_message) {
                    msgEl.textContent = state.save_bag_message;
                }
            }

            if (!state.save_bag_saving) {
                clearInterval(interval);
                resolve(state.save_bag_success);
            }
        }, 500);
    });

    // 완료 시 100%로 채운 뒤 UI 복원
    setProgress(100);
    if (msgEl) { msgEl.textContent = 'Conversion complete!'; }
    setTimeout(() => restoreUi(success), 1200);
}

async function setLoop(loop) {
    await apiCall('/api/player/set_loop', { loop });
}

async function setBagPlayerLoop(loop) {
    await apiCall('/api/bag/set_loop', { loop });
}


async function setAutoStart(auto_start) {
    await apiCall('/api/player/set_auto_start', { auto_start });
}

async function setSliderPosition(position) {
    await apiCall('/api/player/set_slider', { position: parseInt(position) });
}

let _playerWasPlaying = false;

async function updatePlayerState() {
    const state = await apiCall('/api/player/state');
    if (state) {
        domCache.get('player-path-label').textContent = state.path || '';
        domCache.get('player-loop').checked = state.loop || false;
        domCache.get('player-auto-start').checked = state.auto_start || false;

        domCache.get('player-timestamp-label').textContent = state.timestamp || 0;

        // Update button states
        if (state.playing) {
            _playerWasPlaying = true;
            domCache.get('player-slider').value = state.slider_pos || 0;
            domCache.get('play-button').textContent = 'End';
        } else {
            // 재생이 끝난 직후 슬라이더를 처음으로 되돌림
            if (_playerWasPlaying) {
                domCache.get('player-slider').value = 0;
                _playerWasPlaying = false;
                // 서버 slider_pos도 0으로 동기화 (이후 폴링에서 끝 위치로 덮어쓰이는 것 방지)
                apiCall('/api/player/set_slider', { position: 0 });
            } else {
                domCache.get('player-slider').value = state.slider_pos || 0;
            }
            domCache.get('play-button').textContent = 'Play';
        }

        if (state.paused) {
            domCache.get('pause-button').textContent = 'Resume';
        } else {
            domCache.get('pause-button').textContent = 'Pause';
        }

    }
}

// Bag Recorder Functions

const bagNameBrowserState = {
    currentPath: '~'
};

async function enterBagName() {
    await openBagNameBrowser();
}

async function openBagNameBrowser() {
    bagNameBrowserState.currentPath = '~';
    domCache.get('bag-name-input').value = '';
    await loadBagNameDirectory(bagNameBrowserState.currentPath);
    domCache.get('bag-name-browser-modal').style.display = 'block';
}

function closeBagNameBrowser() {
    domCache.get('bag-name-browser-modal').style.display = 'none';
    domCache.get('bag-name-input').value = '';
}

async function loadBagNameDirectory(path) {
    try {
        const response = await fetch(`/api/browse?path=${encodeURIComponent(path)}`);
        const result = await response.json();

        if (result.success) {
            bagNameBrowserState.currentPath = result.current_path;
            domCache.get('bag-name-current-path').textContent = result.current_path;

            const listElement = domCache.get('bag-name-directory-list');
            listElement.innerHTML = '';

            result.entries.forEach(entry => {
                const div = document.createElement('div');
                div.className = 'directory-entry';

                if (entry.is_dir) {
                    div.classList.add('dir-entry');
                    div.textContent = '📁 ' + entry.name;
                    div.onclick = () => loadBagNameDirectory(entry.path);
                } else {
                    div.classList.add('dir-entry', 'file-item');
                    div.textContent = '📄 ' + entry.name;
                    div.onclick = () => {
                        domCache.get('bag-name-input').value = entry.name;
                    };
                }

                listElement.appendChild(div);
            });
        } else {
            alert('Failed to load directory: ' + (result.error || 'Unknown error'));
        }
    } catch (error) {
        console.error('Failed to load directory:', error);
        alert('Failed to load directory');
    }
}

async function confirmBagName() {
    const nameInput = domCache.get('bag-name-input');
    const name = nameInput.value.trim().replace(/\.bag$/, '');

    if (!name) {
        alert('Please enter a bag name');
        return;
    }

    const basePath = bagNameBrowserState.currentPath.replace(/\/+$/, '') + '/' + name;

    bagRecorderState.bagName = basePath;
    updateRecorderBagNameDisplay();
    console.log('Bag name set:', basePath);

    closeBagNameBrowser();

    const result = await apiCall('/api/recorder/set_bag_name', { bag_name: basePath });
    if (!result.success) {
        alert('Failed to set bag name: ' + (result.message || 'Unknown error'));
    }
}

function updateRecorderBagNameDisplay() {
    if (!bagRecorderState.bagName) {
        return;
    }
    const format = domCache.get('recorder-format-select').value;
    const displayPath = format === 'ros1' ? bagRecorderState.bagName + '.bag' : bagRecorderState.bagName;
    domCache.get('recorder-bag-name').value = displayPath;
}

async function selectRecorderTopics() {
    if (!bagRecorderState.bagName) {
        alert('Please enter bag name first');
        return;
    }

    // Get current ROS2 topics
    const result = await apiCall('/api/recorder/get_topics');

    if (!result.success || !result.topics || result.topics.length === 0) {
        const rosLabel = window._rosVersion === 1 ? 'ROS1' : 'ROS2';
        alert(`No ${rosLabel} topics found. Play a bag or start ROS nodes first.`);
        return;
    }

    // Display topic selection modal
    const topicList = domCache.get('recorder-topic-list');
    topicList.innerHTML = '';

    // 이미 선택된 토픽 이름 집합 (빠른 검색용)
    const selectedNames = new Set(
        bagRecorderState.selectedTopics.map(t => (typeof t === 'object' ? t.name : t))
    );

    result.topics.forEach(topicEntry => {
        // topicEntry는 {name, type} 객체 또는 문자열일 수 있음
        const topicName = (typeof topicEntry === 'object') ? topicEntry.name : topicEntry;
        const topicType = (typeof topicEntry === 'object') ? topicEntry.type : '';

        const div = document.createElement('div');
        div.className = 'topic-item';

        const checkbox = document.createElement('input');
        checkbox.type = 'checkbox';
        checkbox.id = `recorder-topic-${topicName}`;
        checkbox.value = topicName;
        checkbox.dataset.topicType = topicType;   // 타입 정보를 data 속성에 보존
        checkbox.checked = selectedNames.has(topicName);

        const label = document.createElement('label');
        label.htmlFor = `recorder-topic-${topicName}`;
        if (topicType) {
            label.innerHTML = `<span style="font-weight:600;">${topicName}</span>`
                + ` <span style="color:#888; font-size:0.85em;">${topicType}</span>`;
        } else {
            label.textContent = topicName;
        }

        div.appendChild(checkbox);
        div.appendChild(label);
        topicList.appendChild(div);
    });

    domCache.get('recorder-topic-modal').style.display = 'block';
}

function closeRecorderTopicSelection() {
    domCache.get('recorder-topic-modal').style.display = 'none';
}

function confirmRecorderTopicSelection() {
    // Get all checked topics — {name, type} 객체로 저장하여 ROS1 녹화 시 타입 정보 전달
    bagRecorderState.selectedTopics = [];
    const checkboxes = document.querySelectorAll('#recorder-topic-list input[type="checkbox"]:checked');
    checkboxes.forEach(checkbox => {
        bagRecorderState.selectedTopics.push({
            name: checkbox.value,
            type: checkbox.dataset.topicType || '',
        });
    });

    console.log('Selected topics for recording:', bagRecorderState.selectedTopics);

    // Update display
    updateRecorderSelectedTopicsDisplay();

    closeRecorderTopicSelection();

    if (bagRecorderState.selectedTopics.length === 0) {
        alert('Please select at least one topic');
    }
}

function updateRecorderSelectedTopicsDisplay() {
    const display = domCache.get('recorder-selected-topics-display');
    if (!display) return;
    if (!display.classList.contains('topics-display-styled')) {
        display.classList.add('topics-display-styled');
    }

    if (bagRecorderState.selectedTopics.length === 0) {
        display.innerHTML = '<span style="color: #888;">No topics selected</span>';
    } else {
        // selectedTopics는 {name, type} 객체 또는 문자열 모두 지원
        const topicsHtml = bagRecorderState.selectedTopics.map(topic => {
            const name = typeof topic === 'object' ? topic.name : topic;
            return `<div style="display: inline-block; background: #8a2a2a; padding: 3px 8px; margin: 2px; border-radius: 3px; font-size: 0.9em;">${name}</div>`;
        }).join('');
        display.innerHTML = topicsHtml;
    }
}

async function recordBag() {
    if (!bagRecorderState.bagName) {
        alert('Please enter bag name first');
        return;
    }

    if (bagRecorderState.selectedTopics.length === 0) {
        alert('Please select topics to record');
        return;
    }

    const format = domCache.get('recorder-format-select').value;
    const result = await apiCall('/api/recorder/record', {
        topics: bagRecorderState.selectedTopics,
        bag_format: format,
    });
    if (result.success) {
        const button = domCache.get('recorder-record-button');
        button.textContent = result.recording ? 'Stop' : 'Record';
        console.log('Recording:', result.recording ? 'started' : 'stopped');

        // 녹화 중 모드 배지 표시
        const badge = domCache.get('recorder-mode-badge');
        badge.style.display = result.recording ? 'inline' : 'none';
        const modeLabels = { ros1: 'ROS1 .bag', ros2_db3: 'ROS2 db3', ros2_mcap: 'ROS2 mcap' };
        badge.textContent = modeLabels[result.mode] || 'ROS2 mcap';

        if (result.recording) {
            const displayPath = format === 'ros1' ? bagRecorderState.bagName + '.bag' : bagRecorderState.bagName;
            alert(`Recording started:\n${displayPath}`);
        } else {
            alert('Recording stopped');
        }
    } else {
        alert('Failed to start/stop recording: ' + (result.message || 'Unknown error'));
    }
}

async function updateRecorderState() {
    const state = await apiCall('/api/recorder/state');
    if (state) {
        // Update button state
        const button = domCache.get('recorder-record-button');
        if (state.recording) {
            button.textContent = 'Stop';
        } else {
            button.textContent = 'Record';
        }

        // 모드 배지 업데이트
        const badge = domCache.get('recorder-mode-badge');
        if (badge) {
            badge.style.display = state.recording ? 'inline' : 'none';
            const modeLabels = { ros1: 'ROS1 .bag', ros2_db3: 'ROS2 db3', ros2_mcap: 'ROS2 mcap' };
            badge.textContent = modeLabels[state.mode] || 'ROS2 mcap';
        }
    }
}

// ==============================================================
// Generic Config Manager Class
// ==============================================================
class ConfigManager {
    constructor(name, defaultPath, containerIds, apiEndpoints) {
        this.name = name; // 'slam' or 'localization'
        this.defaultPath = defaultPath;
        this.currentPath = defaultPath;
        this.data = {};
        this.collapsed = true;
        this.containerIds = containerIds; // {parameters, container, toggleBtn}
        this.apiEndpoints = apiEndpoints; // {loadConfig, saveConfig, updateConfig}
    }

    setDefaultPath(defaultPath) {
        this.defaultPath = defaultPath;
        if (!this.currentPath) {
            this.currentPath = defaultPath;
        }
    }

    getDefaultDirectory() {
        if (!this.defaultPath) {
            return '/home';
        }
        const lastSlash = this.defaultPath.lastIndexOf('/');
        return lastSlash >= 0 ? this.defaultPath.slice(0, lastSlash) : '/home';
    }

    async loadDefault() {
        if (!this.defaultPath) {
            console.error(`Default ${this.name} config path is not available`);
            return;
        }

        this.currentPath = this.defaultPath;

        try {
            const result = await apiCall(this.apiEndpoints.loadConfig, { path: this.defaultPath });

            if (result.success && result.config) {
                console.log(`Default ${this.name} config loaded successfully`);
                this.data = result.config;
                this.display();

                // Show config container
                domCache.get(this.containerIds.container).style.display = 'block';

                // 파일명 배지 업데이트
                this._updateFileBadge(this.defaultPath);

                // 초기 상태: collapsed
                const parametersDiv = domCache.get(this.containerIds.parameters);
                const toggleBtn = domCache.get(this.containerIds.toggleBtn);
                parametersDiv.style.display = 'none';
                toggleBtn.classList.remove('open');
                this.collapsed = true;
            } else {
                console.error(`Failed to load default ${this.name} config:`, result.message);
            }
        } catch (error) {
            console.error(`Error loading default ${this.name} config:`, error);
        }
    }

    async load(startPath) {
        openFileBrowser(async (path) => {
            // Check if file has .yaml or .yml extension
            if (!path.endsWith('.yaml') && !path.endsWith('.yml')) {
                showYamlErrorModal();
                return;
            }

            // Load the selected yaml file
            const result = await apiCall(this.apiEndpoints.loadConfig, { path });

            if (result.success && result.config) {
                console.log(`${this.name} config loaded successfully from:`, path);
                this.currentPath = path;
                this.data = result.config;
                this.display();

                // Show config container & 파일명 배지 업데이트
                domCache.get(this.containerIds.container).style.display = 'block';
                this._updateFileBadge(path);

                // 새로 로드 시 펼침 상태로
                const parametersDiv = domCache.get(this.containerIds.parameters);
                const toggleBtn = domCache.get(this.containerIds.toggleBtn);
                parametersDiv.style.display = '';
                toggleBtn.classList.add('open');
                this.collapsed = false;
            } else {
                alert('Failed to load config file: ' + (result.message || 'Unknown error'));
            }
        }, startPath);
    }

    async save(targetPath = null, allowCurrentPathFallback = true) {
        try {
            this.syncFromInputs();
        } catch (error) {
            alert(error.message);
            return false;
        }

        if (targetPath === null) {
            targetPath = allowCurrentPathFallback ? (this.currentPath || this.defaultPath) : this.defaultPath;
        }
        if (!targetPath) {
            alert(`No ${this.name} config file path is available.`);
            return false;
        }

        console.log(`Saving ${this.name} config to:`, targetPath);
        console.log('Config data:', this.data);

        const result = await apiCall(this.apiEndpoints.saveConfig, {
            path: targetPath,
            config: this.data
        });

        if (result.success) {
            const savedPath = result.path || targetPath;
            this.currentPath = savedPath;
            alert('Config file saved successfully to:\n' + savedPath);
            console.log(`${this.name} config saved to:`, savedPath);
            return true;
        }

        alert('Failed to save config file: ' + (result.message || 'Unknown error'));
        return false;
    }

    syncFromInputs() {
        const container = domCache.get(this.containerIds.parameters);
        const inputs = container.querySelectorAll('[data-config-key]');

        inputs.forEach(input => {
            const key = input.dataset.configKey;
            const valueType = input.dataset.valueType;
            const value = this.parseInputValue(input, valueType);
            this.updateValue(key, value, false);
        });
    }

    parseInputValue(input, valueType) {
        if (valueType === 'boolean') {
            return input.checked;
        }

        if (valueType === 'array') {
            try {
                return JSON.parse(input.value.replace(/\s/g, ''));
            } catch (error) {
                throw new Error('Invalid array format. Use format: [1.0, 0.0, 0.0]');
            }
        }

        if (valueType === 'number') {
            const numValue = parseFloat(input.value);
            if (Number.isNaN(numValue)) {
                throw new Error(`Invalid number value for ${input.dataset.configKey}`);
            }
            return numValue;
        }

        return input.value;
    }

    display() {
        const container = domCache.get(this.containerIds.parameters);
        container.innerHTML = '';

        // Separate top-level primitive values and nested objects
        const topLevelParams = [];
        const nestedGroups = [];

        Object.keys(this.data).forEach(key => {
            const value = this.data[key];
            if (typeof value === 'object' && value !== null && !Array.isArray(value)) {
                nestedGroups.push({ key, data: value });
            } else {
                topLevelParams.push({ key, value });
            }
        });

        const makeGroupCard = (title, params, keyPrefix) => {
            const card = document.createElement('div');
            card.className = 'cfg-group';

            const titleEl = document.createElement('div');
            titleEl.className = 'cfg-group-title';
            titleEl.textContent = title;
            card.appendChild(titleEl);

            const grid = document.createElement('div');
            grid.className = 'cfg-group-grid';
            card.appendChild(grid);

            params.forEach(({ key, value, fullKey }) => {
                this.createParameterInput(grid, key, value, fullKey);
            });

            container.appendChild(card);
        };

        if (topLevelParams.length > 0) {
            makeGroupCard('General', topLevelParams.map(p => ({
                key: p.key, value: p.value, fullKey: p.key
            })));
        }

        nestedGroups.forEach(group => {
            const title = group.key.split('_').map(w =>
                w.charAt(0).toUpperCase() + w.slice(1)
            ).join(' ');
            const params = Object.keys(group.data).map(key => ({
                key,
                value: group.data[key],
                fullKey: `${group.key}.${key}`
            }));
            makeGroupCard(title, params);
        });
    }

    createParameterInput(grid, label, value, fullKey) {
        const row = document.createElement('div');
        row.className = 'cfg-param-row';

        // Array 타입은 2열 전체 사용
        const isArray = Array.isArray(value);
        const isWide = isArray || typeof value === 'string';
        if (isWide) row.classList.add('cfg-span2');

        const labelEl = document.createElement('span');
        labelEl.className = 'cfg-param-label';
        labelEl.textContent = label;
        row.appendChild(labelEl);

        if (typeof value === 'boolean') {
            // 토글 스위치
            const wrap = document.createElement('div');
            wrap.className = 'cfg-toggle-wrap';

            const sw = document.createElement('label');
            sw.className = 'cfg-switch';

            const cb = document.createElement('input');
            cb.type = 'checkbox';
            cb.checked = value;
            cb.id = `${this.name}-param-${fullKey}`;
            cb.dataset.configKey = fullKey;
            cb.dataset.valueType = 'boolean';

            const slider = document.createElement('span');
            slider.className = 'cfg-switch-slider';

            const valLabel = document.createElement('span');
            valLabel.className = 'cfg-switch-val' + (value ? ' cfg-val-on' : '');
            valLabel.textContent = value ? 'true' : 'false';

            cb.onchange = () => {
                valLabel.textContent = cb.checked ? 'true' : 'false';
                valLabel.className = 'cfg-switch-val' + (cb.checked ? ' cfg-val-on' : '');
                this.updateValue(fullKey, cb.checked);
            };

            sw.appendChild(cb);
            sw.appendChild(slider);
            wrap.appendChild(sw);
            wrap.appendChild(valLabel);
            row.appendChild(wrap);

        } else if (isArray) {
            const inp = document.createElement('input');
            inp.type = 'text';
            inp.className = 'cfg-param-input cfg-input-wide';
            inp.value = '[' + value.map(v => String(v)).join(', ') + ']';
            inp.id = `${this.name}-param-${fullKey}`;
            inp.dataset.configKey = fullKey;
            inp.dataset.valueType = 'array';
            inp.onchange = () => {
                try {
                    const parsed = JSON.parse(inp.value.replace(/\s/g, ''));
                    this.updateValue(fullKey, parsed);
                } catch (e) {
                    alert('Invalid array format. Use format: [1.0, 0.0, 0.0]');
                }
            };
            row.appendChild(inp);

        } else if (typeof value === 'number') {
            const inp = document.createElement('input');
            inp.type = 'number';
            inp.className = 'cfg-param-input';
            inp.value = value;
            inp.step = 'any';
            inp.id = `${this.name}-param-${fullKey}`;
            inp.dataset.configKey = fullKey;
            inp.dataset.valueType = 'number';
            inp.onchange = () => this.updateValue(fullKey, parseFloat(inp.value));
            row.appendChild(inp);

        } else if (typeof value === 'string') {
            const inp = document.createElement('input');
            inp.type = 'text';
            inp.className = 'cfg-param-input cfg-input-wide';
            inp.value = value;
            inp.id = `${this.name}-param-${fullKey}`;
            inp.dataset.configKey = fullKey;
            inp.dataset.valueType = 'string';
            inp.onchange = () => this.updateValue(fullKey, inp.value);
            row.appendChild(inp);

        } else {
            const span = document.createElement('span');
            span.className = 'cfg-param-label';
            span.style.color = '#c8cfe8';
            span.textContent = String(value);
            row.appendChild(span);
        }

        grid.appendChild(row);
    }

    updateValue(key, value, notifyBackend = true) {
        console.log(`Updated ${this.name} config: ${key} = ${value}`);

        // Update local config data
        const keys = key.split('.');
        let obj = this.data;

        for (let i = 0; i < keys.length - 1; i++) {
            if (!obj[keys[i]]) obj[keys[i]] = {};
            obj = obj[keys[i]];
        }

        obj[keys[keys.length - 1]] = value;

        // Send update to backend
        if (notifyBackend) {
            apiCall(this.apiEndpoints.updateConfig, { key, value });
        }
    }

    toggle() {
        const parametersDiv = domCache.get(this.containerIds.parameters);
        const toggleBtn = domCache.get(this.containerIds.toggleBtn);

        this.collapsed = !this.collapsed;

        if (this.collapsed) {
            parametersDiv.style.display = 'none';
            toggleBtn.classList.remove('open');
        } else {
            parametersDiv.style.display = '';
            toggleBtn.classList.add('open');
        }
    }

    _updateFileBadge(filePath) {
        const badgeId = this.containerIds.fileBadge;
        if (!badgeId) return;
        const badge = document.getElementById(badgeId);
        if (!badge) return;
        // 파일명만 추출 (경로 제거)
        const filename = filePath ? filePath.split('/').pop() : '';
        badge.textContent = filename;
        badge.style.display = filename ? '' : 'none';
    }
}

// ==============================================================
// Config Manager Instances
// ==============================================================
const slamConfig = new ConfigManager(
    'slam',
    '',
    {
        parameters: 'slam-config-parameters',
        container: 'slam-config-container',
        toggleBtn: 'slam-config-toggle-btn',
        fileBadge: 'slam-cfg-file-badge'
    },
    {
        loadConfig: '/api/slam/load_config_file',
        saveConfig: '/api/slam/save_config_file',
        updateConfig: '/api/slam/update_config'
    }
);

const localizationConfig = new ConfigManager(
    'localization',
    '',
    {
        parameters: 'localization-config-parameters',
        container: 'localization-config-container',
        toggleBtn: 'localization-config-toggle-btn',
        fileBadge: 'localization-cfg-file-badge'
    },
    {
        loadConfig: '/api/slam/load_config_file',
        saveConfig: '/api/slam/save_config_file',
        updateConfig: '/api/slam/update_config'
    }
);

async function initializeFastLioConfigPaths() {
    const result = await apiCall('/api/slam/default_config_paths');
    if (result.success) {
        slamConfig.setDefaultPath(result.mapping_config);
        localizationConfig.setDefaultPath(result.localization_config);
        console.log('FAST-LIO config directory:', result.config_dir);
    } else {
        console.error('Failed to resolve FAST-LIO config directory:', result.message || result.error);
    }
}

async function initializeSiblingPackagePaths() {
    const result = await apiCall('/api/slam/sibling_package_dirs');
    if (result.success) {
        if (result.long_term_mapping) {
            siblingPackagePaths.longTermMapping = result.long_term_mapping;
        }
        if (result.pose_graph_optimization) {
            siblingPackagePaths.poseGraphOptimization = result.pose_graph_optimization;
        }
        console.log('Sibling package paths:', siblingPackagePaths);
    } else {
        console.warn('Could not auto-detect sibling package paths');
    }
}

// ==============================================================
// Config Function Wrappers (for backwards compatibility with HTML)
// ==============================================================
async function loadDefaultSlamConfig() {
    await slamConfig.loadDefault();
}

async function loadSlamConfig() {
    await slamConfig.load(slamConfig.getDefaultDirectory());
}

async function saveSlamConfig() {
    await slamConfig.save(slamConfig.defaultPath, false);
}

function toggleSlamConfig() {
    slamConfig.toggle();
}

async function loadDefaultLocalizationConfig() {
    await localizationConfig.loadDefault();
}

async function loadLocalizationConfig() {
    await localizationConfig.load(localizationConfig.getDefaultDirectory());
}

async function saveLocalizationConfig() {
    await localizationConfig.save(localizationConfig.defaultPath, false);
}

function toggleLocalizationConfig() {
    localizationConfig.toggle();
}

// ==============================================================
// SLAM Map Functions
// ==============================================================
async function saveSlamMap() {
    // 새 Save Map 시작 시 이전 결과 뷰어 즉시 숨김
    if (typeof saveMapResultViewer !== 'undefined' && saveMapResultViewer) {
        saveMapResultViewer.hideAndReset();
    }
    // Phase 4.10: 대쉬보드 숨김 + 구독 해제
    if (typeof slamAnalyticsDashboard !== 'undefined') {
        slamAnalyticsDashboard.hide();
        slamAnalyticsDashboard.unsubscribe();
    }
    // Open save map modal
    domCache.get('save-map-modal').style.display = 'block';
    domCache.get('save-map-directory').value = '';
    domCache.get('save-map-directory').focus();
}

function closeSaveMapModal(skipRestore = false) {
    domCache.get('save-map-modal').style.display = 'none';
    // 모달 취소 시 플래그 해제 (confirmSaveMap 호출 전에 닫은 경우)
    // skipRestore=true이면 confirmSaveMap에서 호출한 것이므로 복원 생략
    if (!_saveMapPollTimer && !skipRestore) {
        window._slamSaving = false;
        if (typeof slamLiveViewer !== 'undefined') slamLiveViewer._visualShow();
        // 대시보드 복원 (saveSlamMap에서 숨겼으므로)
        if (typeof slamAnalyticsDashboard !== 'undefined') {
            slamAnalyticsDashboard.show();
            slamAnalyticsDashboard.subscribe();
        }
    }
}

let _saveMapPollTimer = null;
let _saveMapDirectory = null;

async function confirmSaveMap() {
    const directoryName = domCache.get('save-map-directory').value.trim();

    if (!directoryName) {
        alert('Please enter a directory name');
        return;
    }

    closeSaveMapModal(true); // 저장 확정 시 복원 생략

    // 새 Save Map 시작 시 이전 결과 뷰어 초기화 + 디렉토리명 보관
    _saveMapDirectory = directoryName;
    _saveMapRestoreChecked = false;
    if (typeof saveMapResultViewer !== 'undefined' && saveMapResultViewer) {
        saveMapResultViewer.hideAndReset();
    }

    // 저장 확정 시점에 Live Viewer 화면만 숨김 (WebSocket 유지 → DDS 재조회 없음)
    window._slamSaving = true;
    if (typeof slamLiveViewer !== 'undefined' && slamLiveViewer._visible) {
        slamLiveViewer._visualHide();
    }

    const result = await apiCall('/api/slam/save_map', { directory: directoryName });

    if (result.success) {
        _showSaveMapStatus('Saving map to "' + directoryName + '"...', true);
        _startSaveMapPolling();
    } else {
        window._slamSaving = false;
        alert('Failed to start map save: ' + (result.message || 'Unknown error'));
    }
}

function _setSaveMapAreaState(state) {
    const area = domCache.get('slam-save-map-status-area');
    area.classList.remove('success', 'warn', 'error');
    if (state) area.classList.add(state);
}

function _showSaveMapStatus(message, saving) {
    const area = domCache.get('slam-save-map-status-area');
    const msgEl = domCache.get('slam-save-map-msg');
    const spinner = domCache.get('slam-save-map-spinner');
    const cancelBtn = domCache.get('slam-save-map-cancel-btn');

    msgEl.textContent = message;
    spinner.style.display = saving ? 'inline-block' : 'none';
    cancelBtn.style.display = saving ? 'inline-block' : 'none';
    cancelBtn.disabled = false;
    cancelBtn.textContent = 'Cancel';
    area.style.display = 'block';
    area.style.opacity = '1';
    _setSaveMapAreaState(null);
}

function _showSaveMapSuccess(message) {
    const area = domCache.get('slam-save-map-status-area');
    const msgEl = domCache.get('slam-save-map-msg');
    const spinner = domCache.get('slam-save-map-spinner');
    const cancelBtn = domCache.get('slam-save-map-cancel-btn');

    msgEl.textContent = message;
    spinner.style.display = 'none';
    cancelBtn.style.display = 'none';
    area.style.display = 'block';
    area.style.opacity = '1';
    _setSaveMapAreaState('success');

    // 3초 후 페이드 아웃 후 숨김 (저장 전 초기 상태로 복귀)
    setTimeout(() => {
        area.style.transition = 'opacity 0.6s ease';
        area.style.opacity = '0';
        setTimeout(() => {
            area.style.display = 'none';
            area.style.opacity = '1';
            area.style.transition = '';
        }, 620);
    }, 3000);
}

function _showSaveMapError(message) {
    const area = domCache.get('slam-save-map-status-area');
    const msgEl = domCache.get('slam-save-map-msg');
    const spinner = domCache.get('slam-save-map-spinner');
    const cancelBtn = domCache.get('slam-save-map-cancel-btn');

    msgEl.textContent = message;
    spinner.style.display = 'none';
    cancelBtn.style.display = 'none';
    area.style.display = 'block';
    area.style.opacity = '1';
    _setSaveMapAreaState('error');
}

function _startSaveMapPolling() {
    if (_saveMapPollTimer) {
        clearInterval(_saveMapPollTimer);
    }
    _saveMapPollTimer = setInterval(_pollSaveMapStatus, 2000);
}

// 저장 성공 메시지("Map saved successfully to directory: {name}")에서 디렉토리명 추출
function _parseSaveMapDirectory(message) {
    if (!message) return null;
    const marker = 'directory:';
    const idx = message.indexOf(marker);
    if (idx === -1) return null;
    const name = message.slice(idx + marker.length).trim();
    return name || null;
}

let _saveMapRestoreChecked = false;

// 페이지/탭 재진입 시 직전 Save Map 결과가 완료(done+success)면 결과 뷰어 자동 복원
async function _maybeRestoreSaveMapViewer() {
    if (_saveMapRestoreChecked) return;
    if (_saveMapPollTimer) return;  // 진행 중이면 폴링이 처리
    if (typeof saveMapResultViewer === 'undefined' || !saveMapResultViewer) return;

    const viewerEl = document.getElementById('savemap-result-viewer');
    if (!viewerEl || viewerEl.style.display !== 'none') return;

    _saveMapRestoreChecked = true;
    try {
        const status = await apiCall('/api/slam/save_map_status');
        if (status && status.done && status.success) {
            const directory = _saveMapDirectory || _parseSaveMapDirectory(status.message);
            if (directory) {
                saveMapResultViewer.show({ directory: directory });
            }
        }
    } catch (e) {
        console.error('Failed to restore save map viewer:', e);
    }
}

async function _pollSaveMapStatus() {
    try {
        const status = await apiCall('/api/slam/save_map_status');

        if (status.done) {
            clearInterval(_saveMapPollTimer);
            _saveMapPollTimer = null;
            window._slamSaving = false;
            window._slamMapJustSaved = true; // 저장 완료 후 Live Viewer 재출현 차단

            if (status.success) {
                _showSaveMapSuccess('✓ ' + (status.message || 'Map saved successfully'));
            } else if (status.message && status.message.includes('Cancelled')) {
                _showSaveMapError('Cancelled: ' + status.message);
            } else {
                _showSaveMapError('✗ ' + (status.message || 'Map save failed'));
            }
        } else if (status.saving) {
            _showSaveMapStatus(status.message || 'Saving...', true);
        }
    } catch (e) {
        console.error('Failed to poll save map status:', e);
    }
}

async function cancelSaveMap() {
    const cancelBtn = domCache.get('slam-save-map-cancel-btn');
    cancelBtn.disabled = true;
    cancelBtn.textContent = 'Cancelling...';

    const result = await apiCall('/api/slam/cancel_save_map', {});

    if (result.success) {
        clearInterval(_saveMapPollTimer);
        _saveMapPollTimer = null;
        window._slamSaving = false;
        window._slamMapJustSaved = false; // 취소 시 Live Viewer 복원 허용
        // Live Viewer 복원
        if (typeof slamLiveViewer !== 'undefined') slamLiveViewer._visualShow();
        // 대시보드 복원
        if (typeof slamAnalyticsDashboard !== 'undefined') {
            slamAnalyticsDashboard.show();
            slamAnalyticsDashboard.subscribe();
        }
        _showSaveMapError('Cancelled by user');
    } else {
        cancelBtn.disabled = false;
        cancelBtn.textContent = 'Cancel';
        console.warn('Cancel failed:', result.message);
    }
}

// ==============================================================
// SLAM Start/Stop (terminal output removed)
// ==============================================================
async function startSlamMapping() {
    // SLAM 시작 시 stale 플래그 초기화
    window._slamSaving = false;
    window._slamMapJustSaved = false;
    // 새 SLAM 시작 시 이전 Save Map 결과 뷰어 숨김
    if (typeof saveMapResultViewer !== 'undefined' && saveMapResultViewer) {
        saveMapResultViewer.hideAndReset();
    }
    // Config 패널 숨기기 (Stop 시 복원용으로 상태 저장)
    const _cfgEl = document.getElementById('slam-config-container');
    if (_cfgEl) {
        window._slamConfigVisibleBeforeStart = (_cfgEl.style.display !== 'none');
        _cfgEl.style.display = 'none';
    }
    // Immediately update status to Running
    updateLidarSlamStatus('Running');
    // SLAM Live Viewer 표시 (API 응답 전 is_running=false 폴링이 hide()하지 않도록 hold)
    window._slamLiveViewerHoldOpen = true;
    if (typeof slamLiveViewer !== 'undefined') {
        slamLiveViewer.show();
    }
    // Phase 4.10: 대쉬보드 표시 + 구독 시작
    if (typeof slamAnalyticsDashboard !== 'undefined') {
        slamAnalyticsDashboard.show();
        slamAnalyticsDashboard.subscribe();
    }

    let result;
    try {
        result = await apiCall('/api/slam/start_mapping', {});
    } finally {
        window._slamLiveViewerHoldOpen = false;
    }
    if (result.success) {
        console.log('SLAM mapping started');
        // Status will be updated by periodic updateSlamState() calls
    } else {
        alert('Failed to start SLAM mapping: ' + (result.message || 'Unknown error'));
        console.error('Failed to start SLAM mapping');
        updateLidarSlamStatus('Ready');
    }
}

async function stopSlamMapping() {
    // SLAM 중지 시 이전 Save Map 결과 뷰어 숨김
    if (typeof saveMapResultViewer !== 'undefined' && saveMapResultViewer) {
        saveMapResultViewer.hideAndReset();
    }
    // 중지 중 updateSlamState가 뷰어를 다시 show하지 않도록 플래그 설정
    window._slamStopping = true;
    // SLAM Live Viewer 즉시 숨김 (locLiveViewer.hide와 동일하게 즉시 처리)
    if (typeof slamLiveViewer !== 'undefined') {
        slamLiveViewer.hide();
    }
    // Phase 4.10: 대쉬보드 숨김 + 구독 해제
    if (typeof slamAnalyticsDashboard !== 'undefined') {
        slamAnalyticsDashboard.hide();
        slamAnalyticsDashboard.unsubscribe();
    }
    // Start 전에 Config 패널이 열려 있었으면 복원
    const _cfgEl = document.getElementById('slam-config-container');
    if (_cfgEl && window._slamConfigVisibleBeforeStart) {
        _cfgEl.style.display = '';
        window._slamConfigVisibleBeforeStart = false;
    }
    // Immediately update status to Stopping
    updateLidarSlamStatus('Stopping...');

    const result = await apiCall('/api/slam/stop_mapping', {});
    // API 응답 후 플래그 해제
    window._slamStopping = false;
    if (result.success) {
        console.log('SLAM mapping stopped');
        setTimeout(() => {
            updateLidarSlamStatus('Ready');
        }, 500);
    } else {
        alert('Failed to stop SLAM mapping: ' + (result.message || 'Unknown error'));
        console.error('Failed to stop SLAM mapping');
        updateLidarSlamStatus('Ready');
    }
}

function updateLidarSlamStatus(status) {
    const lidarSlamStatus = domCache.get('lidar-slam-status');
    const lidarSlamTab = document.getElementById('lidar-slam-subtab');
    if (lidarSlamStatus && lidarSlamTab && lidarSlamTab.classList.contains('active')) {
        lidarSlamStatus.textContent = 'Status: ' + status;
        // Add red color for Stopping status
        if (status.includes('Stopping')) {
            lidarSlamStatus.style.color = '#F44336'; // Red
        } else {
            lidarSlamStatus.style.color = ''; // Reset to default
        }
    }
}

function updateLocalizationStatus(status) {
    const localizationStatus = domCache.get('localization-status');
    const localizationTab = document.getElementById('localization-subtab');
    if (localizationStatus && localizationTab && localizationTab.classList.contains('active')) {
        localizationStatus.textContent = 'Status: ' + status;
        // Add red color for Stopping status
        if (status.includes('Stopping')) {
            localizationStatus.style.color = '#F44336'; // Red
        } else {
            localizationStatus.style.color = ''; // Reset to default
        }
    }
}

async function updateLocalizationState() {
    const state = await apiCall('/api/localization/state');
    if (state) {
        const localizationStatus = domCache.get('localization-status');
        if (localizationStatus) {
            const localizationTab = document.getElementById('localization-subtab');
            if (localizationTab && localizationTab.classList.contains('active')) {
                let statusText = 'Ready';
                if (state.is_running !== undefined) {
                    if (state.is_running) {
                        statusText = 'Running';
                    } else {
                        statusText = 'Ready';
                    }
                }
                localizationStatus.textContent = 'Status: ' + statusText;
                if (statusText.includes('Stopping')) {
                    localizationStatus.style.color = '#F44336';
                } else {
                    localizationStatus.style.color = '';
                }
            }
        }
        // 재진입 복원: locLiveViewer + Analytics Dashboard
        if (typeof locLiveViewer !== 'undefined') {
            if (state.is_running) {
                if (!locLiveViewer._visible) locLiveViewer.show();
                if (typeof locAnalyticsDashboard !== 'undefined') {
                    const dashEl = document.getElementById('loc-analytics-dashboard');
                    if (dashEl && dashEl.style.display === 'none') {
                        locAnalyticsDashboard.show();
                        locAnalyticsDashboard.subscribe();
                    }
                }
            } else {
                if (locLiveViewer._visible) locLiveViewer.hide();
            }
        }
    }
}

// ==============================================================
// Localization Start/Stop (terminal output removed)
// ==============================================================
async function startLocalizationMapping() {
    // Config 패널 숨기기 (Stop 시 복원용으로 상태 저장)
    const _cfgEl = document.getElementById('localization-config-container');
    if (_cfgEl) {
        window._locConfigVisibleBeforeStart = (_cfgEl.style.display !== 'none');
        _cfgEl.style.display = 'none';
    }
    updateLocalizationStatus('Running');

    const result = await apiCall('/api/localization/start_mapping', {});
    if (result.success) {
        console.log('Localization mapping started');
        locLiveViewer.show();
        locAnalyticsDashboard.show();
        locAnalyticsDashboard.subscribe();
    } else {
        alert('Failed to start Localization mapping: ' + (result.message || 'Unknown error'));
        console.error('Failed to start Localization mapping');
        updateLocalizationStatus('Ready');
    }
}

async function stopLocalizationMapping() {
    // Start 전에 Config 패널이 열려 있었으면 복원
    const _cfgEl = document.getElementById('localization-config-container');
    if (_cfgEl && window._locConfigVisibleBeforeStart) {
        _cfgEl.style.display = '';
        window._locConfigVisibleBeforeStart = false;
    }
    updateLocalizationStatus('Stopping...');

    console.log('Stopping Localization mapping...');
    const result = await apiCall('/api/localization/stop_mapping', {});

    locLiveViewer.hide();
    locAnalyticsDashboard.hide();
    locAnalyticsDashboard.unsubscribe();

    if (result.success) {
        console.log('Localization mapping stopped');
        setTimeout(() => {
            updateLocalizationStatus('Ready');
        }, 500);
    } else {
        alert('Failed to stop Localization mapping: ' + (result.message || 'Unknown error'));
        console.error('Failed to stop Localization mapping');
        updateLocalizationStatus('Ready');
    }
}

// ==============================================================
// Utility Functions
// ==============================================================
function showYamlErrorModal() {
    // Create modal overlay
    const overlay = document.createElement('div');
    overlay.style.position = 'fixed';
    overlay.style.top = '0';
    overlay.style.left = '0';
    overlay.style.width = '100%';
    overlay.style.height = '100%';
    overlay.style.backgroundColor = 'rgba(0, 0, 0, 0.5)';
    overlay.style.display = 'flex';
    overlay.style.alignItems = 'center';
    overlay.style.justifyContent = 'center';
    overlay.style.zIndex = '10000';

    // Create modal content
    const modal = document.createElement('div');
    modal.style.backgroundColor = '#2a2a2a';
    modal.style.padding = '30px';
    modal.style.borderRadius = '8px';
    modal.style.boxShadow = '0 4px 6px rgba(0, 0, 0, 0.3)';
    modal.style.textAlign = 'center';
    modal.style.minWidth = '300px';

    // Error message
    const message = document.createElement('p');
    message.textContent = 'yaml 파일을 선택하세요.';
    message.style.color = '#ffffff';
    message.style.fontSize = '16px';
    message.style.marginBottom = '20px';

    // OK button
    const okButton = document.createElement('button');
    okButton.textContent = 'OK';
    okButton.style.padding = '8px 30px';
    okButton.style.fontSize = '14px';
    okButton.style.cursor = 'pointer';
    okButton.onclick = () => {
        document.body.removeChild(overlay);
    };

    modal.appendChild(message);
    modal.appendChild(okButton);
    overlay.appendChild(modal);
    document.body.appendChild(overlay);
}

// ==============================================================
// Latency Measurement
// ==============================================================
// Worker(latency_ping_worker.js)에서 순차 ping 3회 최소값 측정 — 메인스레드 렌더 부하와 분리

/** Path Group/Mesh dispose (증분 tube 누적 정리용) */
function _disposePathObject(scene, obj) {
    if (!obj) return;
    if (scene) scene.remove(obj);
    obj.traverse((child) => {
        if (child.geometry) child.geometry.dispose();
        if (child.material) {
            if (Array.isArray(child.material)) child.material.forEach(m => m.dispose());
            else child.material.dispose();
        }
    });
}

/**
 * 동적 점군 갱신 후 drawRange 동기화.
 * frustumCulled=false 이면 bounding sphere 생략 — 매 프레임 할당/GC 누적 방지.
 */
function _syncPointsGeometry(geo, count, options = {}) {
    geo.setDrawRange(0, count);
    if (!options.skipBoundingSphere) {
        geo.computeBoundingSphere();
    }
}

/**
 * 복셀 그리드 centroid 다운샘플링 (PCL VoxelGrid 유사)
 * @param {Float32Array} positions - N×3 XYZ
 * @param {Float32Array|null} colors - N×3 RGB (optional)
 * @param {number} voxelSize - 복셀 leaf size (m)
 * @param {{ whiteOutput?: boolean }} options - whiteOutput 시 (1,1,1) 고정
 */
function _voxelDownsample(positions, colors, voxelSize, options = {}) {
    const { whiteOutput = false } = options;
    const n = positions.length / 3;
    if (n === 0) return { positions: new Float32Array(0), colors: new Float32Array(0) };

    // whiteOutput: first-wins (centroid 불필요) — 메인스레드 부하 대폭 감소
    if (whiteOutput) {
        const seen = new Map();
        for (let i = 0; i < n; i++) {
            const x = positions[i * 3];
            const y = positions[i * 3 + 1];
            const z = positions[i * 3 + 2];
            const vk = (Math.floor(x / voxelSize) * 73856093)
                ^ (Math.floor(y / voxelSize) * 19349663)
                ^ (Math.floor(z / voxelSize) * 83492791);
            if (seen.has(vk)) continue;
            seen.set(vk, i);
        }
        const outCount = seen.size;
        const outPos = new Float32Array(outCount * 3);
        const outCol = new Float32Array(outCount * 3);
        let o = 0;
        for (const i of seen.values()) {
            outPos[o]     = positions[i * 3];
            outPos[o + 1] = positions[i * 3 + 1];
            outPos[o + 2] = positions[i * 3 + 2];
            outCol[o] = 1.0;
            outCol[o + 1] = 1.0;
            outCol[o + 2] = 1.0;
            o += 3;
        }
        return { positions: outPos, colors: outCol };
    }

    const voxels = new Map();
    for (let i = 0; i < n; i++) {
        const x = positions[i * 3];
        const y = positions[i * 3 + 1];
        const z = positions[i * 3 + 2];
        const vk = (Math.floor(x / voxelSize) * 73856093)
            ^ (Math.floor(y / voxelSize) * 19349663)
            ^ (Math.floor(z / voxelSize) * 83492791);
        let v = voxels.get(vk);
        if (!v) {
            v = { sx: x, sy: y, sz: z, count: 1 };
            if (colors) {
                v.scr = colors[i * 3];
                v.scg = colors[i * 3 + 1];
                v.scb = colors[i * 3 + 2];
            }
            voxels.set(vk, v);
        } else {
            v.sx += x;
            v.sy += y;
            v.sz += z;
            v.count++;
            if (colors) {
                v.scr += colors[i * 3];
                v.scg += colors[i * 3 + 1];
                v.scb += colors[i * 3 + 2];
            }
        }
    }

    const outCount = voxels.size;
    const outPos = new Float32Array(outCount * 3);
    const outCol = new Float32Array(outCount * 3);
    let o = 0;
    for (const v of voxels.values()) {
        const c = v.count;
        outPos[o]     = v.sx / c;
        outPos[o + 1] = v.sy / c;
        outPos[o + 2] = v.sz / c;
        if (colors) {
            outCol[o]     = v.scr / c;
            outCol[o + 1] = v.scg / c;
            outCol[o + 2] = v.scb / c;
        }
        o += 3;
    }
    return { positions: outPos, colors: outCol };
}

// Live path: throttle + pose sliding window
// CatmullRom 제어점은 MAX_CTRL로 서브샘플 — 대량 Vector3 매 rebuild는 메인스레드 스톨 원인.
const LIVE_PATH_MAX_POSES = 1000;
/** Localization Live Viewer: /Odometry path 포인트 최소 간격 (m, 3D) */
const LIVE_PATH_MIN_DIST_M = 1.0;
/** Live Viewer 점군 메인스레드 처리 상한 (cloud_registered 현재 스캔) */
const LIVE_CLOUD_UPDATE_MS = 500;
/** SLAM Live Viewer /PGO_map 갱신 상한 */
const LIVE_PGO_MAP_UPDATE_MS = 500;
/** Localization /Laser_map latched 대용량 갱신 상한 */
const LIVE_LASER_MAP_UPDATE_MS = 2000;
/** SLAM Live Viewer /kf_node InstancedMesh 갱신 상한 */
const LIVE_KF_NODE_UPDATE_MS = 500;
const LIVE_PATH_REBUILD_MS = 400;
const LIVE_PATH_MAX_CTRL_POINTS = 250;
const LIVE_PATH_MAX_TUBE_SEGMENTS = 200;

/** Path 바이너리 스냅샷을 버퍼에 병합 (증분 append 또는 reset) 후 FIFO trim */
function _mergePathSnapshot(bufferState, xyz, count, maxPoses) {
    if (count < 1) return bufferState;

    const lastMsgCount = bufferState.lastMsgCount || 0;
    let buf = bufferState.xyz;

    if (count < lastMsgCount) {
        buf = xyz.slice(0, count * 3);
        bufferState.lastMsgCount = count;
    } else if (count > lastMsgCount) {
        const newPart = xyz.subarray(lastMsgCount * 3, count * 3);
        if (!buf || buf.length === 0) {
            buf = xyz.slice(0, count * 3);
        } else if (newPart.length > 0) {
            const merged = new Float32Array(buf.length + newPart.length);
            merged.set(buf);
            merged.set(newPart, buf.length);
            buf = merged;
        }
        bufferState.lastMsgCount = count;
    } else if (!buf || buf.length === 0) {
        buf = xyz.slice(0, count * 3);
        bufferState.lastMsgCount = count;
    }

    let poseCount = buf.length / 3;
    if (poseCount > maxPoses) {
        const trim = poseCount - maxPoses;
        buf = buf.slice(trim * 3);
        poseCount = maxPoses;
    }

    bufferState.xyz = buf;
    bufferState.count = poseCount;
    return bufferState;
}

/**
 * 키프레임 전체 스냅샷(예: /kf_node PointCloud2)으로 path 버퍼를 교체.
 * 최근 maxPoses개만 유지 — 시각화 tube rebuild 시 오래된 구간 자동 제거.
 */
function _setPathFromKeyframeSnapshot(bufferState, positions, maxPoses) {
    const count = (positions && positions.length) ? (positions.length / 3) : 0;
    if (count < 1) {
        bufferState.xyz = null;
        bufferState.count = 0;
        bufferState.lastMsgCount = 0;
        return bufferState;
    }
    const keep = Math.min(count, maxPoses);
    const start = (count - keep) * 3;
    bufferState.xyz = positions.slice(start, count * 3);
    bufferState.count = keep;
    bufferState.lastMsgCount = count;
    return bufferState;
}

function _clearPathBufferState(viewer, bufferKey, timerKey) {
    viewer[bufferKey] = { xyz: null, count: 0, lastMsgCount: 0 };
    if (viewer[timerKey]) {
        clearTimeout(viewer[timerKey]);
        viewer[timerKey] = null;
    }
}

function _schedulePathTubeRebuild(viewer, timerKey, rebuildFn) {
    if (viewer[timerKey]) return;
    viewer[timerKey] = setTimeout(() => {
        viewer[timerKey] = null;
        rebuildFn();
    }, LIVE_PATH_REBUILD_MS);
}

/** 버퍼 XYZ → TubeGeometry Mesh 생성 (제어점 서브샘플로 CPU 스톨 방지) */
function _buildPathTubeFromBuffer(THREE, bufferState, color, tubeRadius, radialSegments = 5) {
    const count = bufferState.count;
    if (count < 2) return null;

    const xyz = bufferState.xyz;
    const step = Math.max(1, Math.ceil(count / LIVE_PATH_MAX_CTRL_POINTS));
    const points3d = [];
    for (let i = 0; i < count; i += step) {
        points3d.push(new THREE.Vector3(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]));
    }
    const last = count - 1;
    if (last % step !== 0) {
        points3d.push(new THREE.Vector3(xyz[last * 3], xyz[last * 3 + 1], xyz[last * 3 + 2]));
    }
    if (points3d.length < 2) return null;

    const curve = new THREE.CatmullRomCurve3(points3d);
    const segments = Math.min(points3d.length * 2, LIVE_PATH_MAX_TUBE_SEGMENTS);
    const geo = new THREE.TubeGeometry(curve, segments, tubeRadius, radialSegments, false);
    const mat = new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.visible = true;
    mesh.frustumCulled = false;
    return mesh;
}

/** SLAM Live path 선 두께 (px). LineBasicMaterial linewidth는 WebGL에서 무시되므로 Line2 사용 */
const LIVE_PATH_LINEWIDTH = 5;

/** 로봇 pose AXIS (/Odometry) — 기존 AxesHelper(0.5) 대비 길이·두께 2배 */
const LIVE_ROBOT_AXIS_LENGTH = 1.0;
const LIVE_ROBOT_AXIS_LINEWIDTH = 6;

/**
 * RGB 축 Group (Line2 굵은 선, fallback AxesHelper).
 * resolution: LineMaterial용 Vector2 (없으면 600x380).
 */
function _createRobotAxesGroup(THREE, length, linewidth, resolution) {
    const len = length || LIVE_ROBOT_AXIS_LENGTH;
    const lw = linewidth || LIVE_ROBOT_AXIS_LINEWIDTH;
    const group = new THREE.Group();
    const res = (resolution && resolution.isVector2)
        ? resolution.clone()
        : new THREE.Vector2(600, 380);

    if (window.Line2 && window.LineGeometry && window.LineMaterial) {
        const axes = [
            { positions: [0, 0, 0, len, 0, 0], color: 0xff0000 },
            { positions: [0, 0, 0, 0, len, 0], color: 0x00ff00 },
            { positions: [0, 0, 0, 0, 0, len], color: 0x0000ff },
        ];
        for (const a of axes) {
            const geo = new window.LineGeometry();
            geo.setPositions(a.positions);
            delete geo._maxInstanceCount;
            geo.instanceCount = 1;
            const mat = new window.LineMaterial({
                color: a.color,
                linewidth: lw,
                resolution: res.clone(),
            });
            const line = new window.Line2(geo, mat);
            line.frustumCulled = false;
            group.add(line);
        }
    } else {
        group.add(new THREE.AxesHelper(len));
    }
    return group;
}

function _disposeRobotAxesGroup(scene, group) {
    if (!group) return;
    if (scene) scene.remove(group);
    group.traverse((child) => {
        if (child.geometry) child.geometry.dispose();
        if (child.material) {
            if (Array.isArray(child.material)) child.material.forEach((m) => m.dispose());
            else child.material.dispose();
        }
    });
}

function _setGroupLineResolutions(group, w, h) {
    if (!group) return;
    group.traverse((child) => {
        if (child.material && child.material.resolution) {
            child.material.resolution.set(w, h);
        }
    });
}

/** SLAM Live path Line 상태 (TubeGeometry dispose/new 회피) */
function _createPathLineState() {
    return { line: null, drawnCount: 0, syncedMsgCount: 0, capacity: 0, fat: false };
}

function _disposePathLineState(scene, lineState) {
    if (!lineState || !lineState.line) return;
    if (scene) scene.remove(lineState.line);
    if (lineState.line.geometry) lineState.line.geometry.dispose();
    if (lineState.line.material) lineState.line.material.dispose();
    lineState.line = null;
    lineState.drawnCount = 0;
    lineState.syncedMsgCount = 0;
    lineState.capacity = 0;
    lineState.fat = false;
}

function _pathLineResolution(THREE, resolution) {
    if (resolution && resolution.isVector2) return resolution;
    return new THREE.Vector2(600, 380);
}

/** Line2(굵은 선) 확보. 미로드 시 THREE.Line fallback */
function _ensurePathLineObject(THREE, scene, lineState, needCount, color, resolution) {
    const useFat = !!(window.Line2 && window.LineGeometry && window.LineMaterial);
    if (lineState.line && lineState.fat === useFat) {
        if (useFat) {
            if (resolution && lineState.line.material && lineState.line.material.resolution) {
                lineState.line.material.resolution.copy(resolution);
            }
            return;
        }
        if (lineState.capacity >= needCount) return;
    }

    // fat ↔ thin 전환 또는 최초 생성 시 기존 라인 제거
    if (lineState.line) {
        scene.remove(lineState.line);
        if (lineState.line.geometry) lineState.line.geometry.dispose();
        if (lineState.line.material) lineState.line.material.dispose();
        lineState.line = null;
        lineState.capacity = 0;
        lineState.drawnCount = 0;
    }

    if (useFat) {
        const geo = new window.LineGeometry();
        const mat = new window.LineMaterial({
            color,
            linewidth: LIVE_PATH_LINEWIDTH,
            resolution: _pathLineResolution(THREE, resolution).clone(),
        });
        const line = new window.Line2(geo, mat);
        line.frustumCulled = false;
        line.visible = true;
        scene.add(line);
        lineState.line = line;
        lineState.fat = true;
        lineState.capacity = 0;
        return;
    }

    const cap = Math.max(needCount | 0, Math.ceil((needCount | 0) * 1.5), 256);
    const pos = new Float32Array(cap * 3);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setDrawRange(0, 0);
    const mat = new THREE.LineBasicMaterial({ color });
    const line = new THREE.Line(geo, mat);
    line.frustumCulled = false;
    line.visible = true;
    scene.add(line);
    lineState.line = line;
    lineState.fat = false;
    lineState.capacity = cap;
    lineState.drawnCount = 0;
    lineState.syncedMsgCount = 0;
}

/** thin Line 버퍼 grow (Line2는 setPositions로 처리) */
function _ensureThinPathLineCapacity(THREE, scene, lineState, needCount, color) {
    const need = Math.max(2, needCount | 0);
    if (!lineState.line || lineState.fat || lineState.capacity >= need) return;

    const cap = Math.max(need, Math.ceil(need * 1.5), 256);
    const oldLine = lineState.line;
    const pos = new Float32Array(cap * 3);
    const oldAttr = oldLine.geometry.getAttribute('position');
    const keep = Math.min(lineState.drawnCount | 0, lineState.capacity | 0, cap);
    if (oldAttr && keep > 0) {
        pos.set(oldAttr.array.subarray(0, keep * 3));
    }
    scene.remove(oldLine);
    oldLine.geometry.dispose();
    const mat = oldLine.material;
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setDrawRange(0, keep);
    const line = new THREE.Line(geo, mat);
    line.frustumCulled = false;
    line.visible = true;
    scene.add(line);
    lineState.line = line;
    lineState.capacity = cap;
}

/**
 * kf_node 구 중심 좌표와 동일한 XYZ로 Line 동기화.
 * - Line2: 굵은 선 (setPositions). 키프레임 수·0.5s 주기면 부담 무시 가능
 * - fallback THREE.Line: 끝점 검증 append / 전체 복사
 */
function _syncPathLineFromNodeCenters(THREE, scene, lineState, xyz, count, color, resolution) {
    const n = count | 0;
    if (!scene || !THREE || !xyz || n < 2) return null;

    const res = _pathLineResolution(THREE, resolution);
    _ensurePathLineObject(THREE, scene, lineState, n, color, res);

    if (lineState.fat) {
        const slice = xyz.subarray ? xyz.subarray(0, n * 3) : xyz;
        const geo = lineState.line.geometry;
        // Line2: setPositions 재호출 시 _maxInstanceCount가 첫 개수로 고정되어
        // 이후 점이 늘어나도 선이 안 늘어남 → 캐시 삭제 필수
        geo.setPositions(slice);
        delete geo._maxInstanceCount;
        geo.instanceCount = Math.max(0, n - 1);
        if (lineState.line.material && lineState.line.material.resolution) {
            lineState.line.material.resolution.copy(res);
        }
        lineState.drawnCount = n;
        lineState.syncedMsgCount = n;
        return lineState.line;
    }

    _ensureThinPathLineCapacity(THREE, scene, lineState, n, color);
    const posAttr = lineState.line.geometry.getAttribute('position');
    const prevDrawn = lineState.drawnCount | 0;
    const arr = posAttr.array;

    let canAppend = false;
    if (prevDrawn >= 2 && n === prevDrawn + 1 && xyz.length >= n * 3) {
        const i = (prevDrawn - 1) * 3;
        canAppend = (
            arr[i] === xyz[i] &&
            arr[i + 1] === xyz[i + 1] &&
            arr[i + 2] === xyz[i + 2]
        );
    }

    if (canAppend) {
        arr[prevDrawn * 3]     = xyz[prevDrawn * 3];
        arr[prevDrawn * 3 + 1] = xyz[prevDrawn * 3 + 1];
        arr[prevDrawn * 3 + 2] = xyz[prevDrawn * 3 + 2];
    } else {
        arr.set(xyz.subarray(0, n * 3));
    }
    posAttr.needsUpdate = true;
    lineState.line.geometry.setDrawRange(0, n);
    lineState.drawnCount = n;
    lineState.syncedMsgCount = n;
    return lineState.line;
}

/**
 * Path 버퍼 → Line 동기화 (/PGO_path용).
 */
function _syncPathLineFromBuffer(THREE, scene, lineState, bufferState, color, resolution) {
    const count = bufferState.count | 0;
    if (!scene || !THREE || count < 2 || !bufferState.xyz) return null;
    return _syncPathLineFromNodeCenters(
        THREE, scene, lineState, bufferState.xyz, count, color, resolution);
}

let _latencyPingWorker = null;

function _ensureLatencyPingWorker() {
    if (_latencyPingWorker) return _latencyPingWorker;
    try {
        _latencyPingWorker = new Worker('/static/latency_ping_worker.js?v=' + Date.now());
        _latencyPingWorker.onmessage = (e) => {
            const { type, ms } = e.data || {};
            if (type !== 'latency') return;
            const latencyElement = document.getElementById('latency-indicator');
            if (!latencyElement) return;
            if (ms == null || !isFinite(ms)) {
                latencyElement.textContent = 'latency: N/A';
                latencyElement.style.color = '#888';
                return;
            }
            const latency = Math.round(ms);
            latencyElement.textContent = `latency: ${latency}ms`;
            if (latency < 50) {
                latencyElement.style.color = '#4CAF50';
            } else if (latency < 150) {
                latencyElement.style.color = '#FFC107';
            } else {
                latencyElement.style.color = '#F44336';
            }
        };
        _latencyPingWorker.onerror = (e) => {
            console.warn('[latency] ping worker error:', e);
        };
    } catch (e) {
        console.warn('[latency] ping worker not available:', e);
    }
    return _latencyPingWorker;
}

function measureLatency() {
    const worker = _ensureLatencyPingWorker();
    if (worker) worker.postMessage('ping');
}

// ==============================================================
// Initialize and periodic updates
// ==============================================================
// Update ROS DOMAIN ID display
async function updateRosDomainId() {
    try {
        const result = await apiCall('/api/ros_domain_id');
        if (result.success && result.domain_id !== undefined) {
            const chip = domCache.get('ros-domain-chip');
            if (chip) {
                chip.textContent = `ROS DOMAIN ID: ${result.domain_id}`;
            }
        }
    } catch (error) {
        console.error('Failed to get ROS DOMAIN ID:', error);
    }
}

window.addEventListener('load', async () => {
    // Initial state update
    await restoreBagPlayerFromServer();
    updateSlamState();
    updateLocalizationState();
    updatePlayerState();
    updateBagState();
    updateRosDomainId(); // Update ROS DOMAIN ID display

    // Resolve FAST-LIO config paths and sibling package paths from the current ROS workspace.
    await Promise.all([
        initializeFastLioConfigPaths(),
        initializeSiblingPackagePaths(),
    ]);
    loadDefaultSlamConfig();
    loadDefaultLocalizationConfig();

    // Start latency measurement (병렬 ping N회 → 중앙값, 단일 RTT 스파이크 완화)
    // KAIST 등 무거운 데이터셋 재생 시 서버 부하 완화를 위해 3초 간격 사용
    measureLatency();
    setInterval(measureLatency, 3000);

    // Periodic state updates (every 500ms for smoother updates)
    setInterval(() => {
        const activeTab = document.querySelector('.tab-content.active');
        if (activeTab.id === 'slam-tab') {
            const activeSubTab = document.querySelector('.subtab-content.active');
            if (activeSubTab && (activeSubTab.id === 'multi-session-slam-subtab' || activeSubTab.id === 'lidar-slam-subtab')) {
                updateSlamState();
            } else if (activeSubTab && activeSubTab.id === 'localization-subtab') {
                updateLocalizationState();
            }
        } else if (activeTab.id === 'player-tab') {
            const activeSubTab = document.querySelector('.subtab-content.active');
            if (activeSubTab && activeSubTab.id === 'bag-player-subtab') {
                updateBagState();
            } else if (activeSubTab && activeSubTab.id === 'file-player-subtab') {
                updatePlayerState();
            }
        } else if (activeTab.id === 'visualization-tab') {
            // Visualization tab - no periodic updates needed
        }
    }, 500);
});

// Simple status banner updater
// Simple status banner updater (deprecated - status banner removed)
function setRunStatus(message, level = 'success') {
    // Status banner removed - this function is kept for compatibility but does nothing
}
// Close modal when clicking outside
window.onclick = function(event) {
    const fileBrowserModal = domCache.get('file-browser-modal');
    const topicSelectionModal = domCache.get('topic-selection-modal');
    const recorderTopicModal = domCache.get('recorder-topic-modal');

    if (event.target === fileBrowserModal) {
        closeFileBrowser();
    }
    if (event.target === topicSelectionModal) {
        closeTopicSelection();
    }
    if (event.target === recorderTopicModal) {
        closeRecorderTopicSelection();
    }
}

// ==============================================================
// Plot 기능 관련 코드
// ==============================================================

// Plot 상태 관리
const plotState = {
    tree: null,
    ros: null,
    topics: [],
    topicTypes: new Map(), // topic name -> message type (Map)
    selectedTopics: new Set(), // 구독 중인 토픽들
    subscribers: new Map(), // topic -> subscriber
    messageTrees: new Map(), // topic -> message tree data
    topicNodes: new Map(), // topic -> topic node element (최상위 노드)
    topicRefreshInterval: null, // 토픽 목록 갱신 인터벌
    topicRefreshRate: 5000, // 5초마다 토픽 목록 갱신 (타임아웃 방지)
    plotTabManager: null, // PlotTabManager 인스턴스 (탭 관리)
    /** @type {ResizeObserver|null} */
    _plotAreaResizeObserver: null,
    /** Plot 왼쪽 패널에 표시할 토픽 (모달에서 선택, ROS 전체 목록과 별도) */
    addedPlotTopics: [],
    plottedPaths: [], // 현재 Plot에 표시된 path들 (모든 탭 공유)
    isLoadingTopics: false, // 토픽 로딩 중 플래그
    pathsRestored: false, // 저장된 paths 복원 여부 (최초 1회만)
    // ── Python 백엔드 WebSocket (포트 8081) ──────────────────────────────────
    // rosbridge를 우회하여 throttle 없이 원래 주기로 plot 데이터 수신
    backendWs: null,            // WebSocket 인스턴스
    _pendingPlotSubs: []        // WS 연결 전에 요청된 subscribe_plot 대기열
};

// ── Python 백엔드 WebSocket 클라이언트 (포트 8081) ──────────────────────────
// rosbridge 없이 원래 토픽 주기 그대로 plot 데이터 수신.
// PC2WebSocketServer의 subscribe_plot 명령을 사용한다.
// ─────────────────────────────────────────────────────────────────────────────
function _initBackendWs() {
    ensureWebuiPortsReady().then(() => {
        const url = getPc2WsUrl();

        if (plotState.backendWs &&
            (plotState.backendWs.readyState === WebSocket.OPEN ||
             plotState.backendWs.readyState === WebSocket.CONNECTING)) {
            return;
        }

        const ws = new WebSocket(url);
        ws.binaryType = 'arraybuffer';
        plotState.backendWs = ws;

        ws.onopen = () => {
            console.log('[BackendWs] 연결됨:', url);
            const pending = plotState._pendingPlotSubs.splice(0);
            for (const req of pending) {
                ws.send(JSON.stringify(req));
            }
        };

        ws.onmessage = (evt) => {
            if (typeof evt.data === 'string') {
                _handleBackendWsMessage(evt.data);
            }
        };

        ws.onerror = () => {
            console.warn('[BackendWs] 연결 오류');
        };

        ws.onclose = () => {
            console.log('[BackendWs] 연결 끊김, 3초 후 재연결...');
            plotState.backendWs = null;
            setTimeout(_initBackendWs, 3000);
        };
    });
}

function _handleBackendWsMessage(rawData) {
    let msg;
    try { msg = JSON.parse(rawData); } catch (e) { return; }

    if (msg.type === 'plot_data') {
        // { type:'plot_data', topic, stamp_sec, stamp_nanosec, values:{field:value,...} }
        const { topic, stamp_sec, stamp_nanosec, values } = msg;
        const timestamp = stamp_sec + stamp_nanosec / 1e9;
        const topicKey  = topic.startsWith('/') ? topic.substring(1) : topic;

        for (const [field, value] of Object.entries(values)) {
            const fullPath = `${topicKey}/${field}`;
            if (plotState.plotTabManager && plotState.plotTabManager.tabs.length > 0) {
                plotState.plotTabManager.tabs.forEach(tab => {
                    if (tab.plotManager && tab.plotManager.dataBuffers.has(fullPath)) {
                        tab.plotManager.updatePlot(fullPath, timestamp, value);
                    }
                });
            }
        }
    } else if (msg.type === 'pc2meta') {
        // PC2 메타데이터는 threejs_display.js가 dispatch하는 CustomEvent와 동일
        window.dispatchEvent(new CustomEvent('pc2_topic_meta', { detail: msg }));

    // ── KITTI 변환 진행률 / 완료 / 오류 ──────────────────────────────────────
    } else if (msg.type === 'kitti_convert_progress') {
        const fill = domCache.get('kitti-progress-fill');
        const text = domCache.get('kitti-progress-text');
        const msgEl = domCache.get('kitti-progress-msg');
        const pct = parseInt(msg.progress || 0);
        if (!isNaN(pct)) {
            fill.style.width = pct + '%';
            text.textContent = pct + '%';
        }
        if (msg.message) { msgEl.textContent = msg.message; }

    } else if (msg.type === 'kitti_convert_done') {
        const btn  = domCache.get('kitti-convert-btn');
        const bar  = domCache.get('kitti-progress-bar');
        const fill = domCache.get('kitti-progress-fill');
        const text = domCache.get('kitti-progress-text');
        const msgEl = domCache.get('kitti-progress-msg');
        _onKittiConvertDone(msg.bag_path, btn, bar, fill, text, msgEl).catch(console.error);

    } else if (msg.type === 'kitti_convert_error') {
        const btn  = domCache.get('kitti-convert-btn');
        const bar  = domCache.get('kitti-progress-bar');
        const msgEl = domCache.get('kitti-progress-msg');
        kittiState.converting = false;
        btn.disabled = false;
        btn.textContent = 'Save Bag';
        if (bar) bar.style.display = 'none';
        if (msgEl) { msgEl.textContent = 'Error: ' + (msg.error || 'Unknown'); }
        alert('Conversion error: ' + (msg.error || 'Unknown'));

    // ── KAIST 변환 진행률 / 완료 / 오류 ──────────────────────────────────────
    } else if (msg.type === 'kaist_convert_progress') {
        const fill = domCache.get('kaist-progress-fill');
        const text = domCache.get('kaist-progress-text');
        const msgEl = domCache.get('kaist-progress-msg');
        const pct = parseInt(msg.progress || 0);
        if (fill && !isNaN(pct)) { fill.style.width = pct + '%'; }
        if (text && !isNaN(pct)) { text.textContent = pct + '%'; }
        if (msgEl && msg.message) { msgEl.textContent = msg.message; }

    } else if (msg.type === 'kaist_convert_done') {
        const btn  = domCache.get('kaist-convert-btn');
        const bar  = domCache.get('kaist-progress-bar');
        const fill = domCache.get('kaist-progress-fill');
        const text = domCache.get('kaist-progress-text');
        const msgEl = domCache.get('kaist-progress-msg');
        _onKaistConvertDone(msg.bag_path, btn, bar, fill, text, msgEl).catch(console.error);

    } else if (msg.type === 'kaist_convert_error') {
        const btn  = domCache.get('kaist-convert-btn');
        const bar  = domCache.get('kaist-progress-bar');
        const msgEl = domCache.get('kaist-progress-msg');
        kaistState.converting = false;
        if (btn) { btn.disabled = false; btn.textContent = 'Save Bag'; }
        if (bar) { bar.style.display = 'none'; }
        if (msgEl) { msgEl.textContent = 'Error: ' + (msg.error || 'Unknown'); }
        alert('Conversion error: ' + (msg.error || 'Unknown'));

    // ── MulRan 변환 진행률 / 완료 / 오류 ─────────────────────────────────────
    } else if (msg.type === 'mulran_convert_progress') {
        const fill  = domCache.get('mulran-progress-fill');
        const text  = domCache.get('mulran-progress-text');
        const msgEl = domCache.get('mulran-progress-msg');
        const pct = parseInt(msg.progress || 0);
        if (fill && !isNaN(pct)) { fill.style.width = pct + '%'; }
        if (text && !isNaN(pct)) { text.textContent = pct + '%'; }
        if (msgEl && msg.message) { msgEl.textContent = msg.message; }

    } else if (msg.type === 'mulran_convert_done') {
        const btn   = domCache.get('mulran-convert-btn');
        const bar   = domCache.get('mulran-progress-bar');
        const fill  = domCache.get('mulran-progress-fill');
        const text  = domCache.get('mulran-progress-text');
        const msgEl = domCache.get('mulran-progress-msg');
        _onMulranConvertDone(msg.bag_path, btn, bar, fill, text, msgEl).catch(console.error);

    } else if (msg.type === 'mulran_convert_error') {
        const btn   = domCache.get('mulran-convert-btn');
        const bar   = domCache.get('mulran-progress-bar');
        const msgEl = domCache.get('mulran-progress-msg');
        mulranState.converting = false;
        if (btn) { btn.disabled = false; btn.textContent = 'Save Bag'; }
        if (bar) { bar.style.display = 'none'; }
        if (msgEl) { msgEl.textContent = 'Error: ' + (msg.error || 'Unknown'); }
        alert('Conversion error: ' + (msg.error || 'Unknown'));

    // ── HeLiPR 변환 진행률 / 완료 / 오류 ─────────────────────────────────────
    } else if (msg.type === 'helipr_convert_progress') {
        const fill  = domCache.get('helipr-progress-fill');
        const text  = domCache.get('helipr-progress-text');
        const msgEl = domCache.get('helipr-progress-msg');
        const pct = parseInt(msg.progress || 0);
        if (fill && !isNaN(pct)) { fill.style.width = pct + '%'; }
        if (text && !isNaN(pct)) { text.textContent = pct + '%'; }
        if (msgEl && msg.message) { msgEl.textContent = msg.message; }

    } else if (msg.type === 'helipr_convert_done') {
        const btn   = domCache.get('helipr-convert-btn');
        const bar   = domCache.get('helipr-progress-bar');
        const fill  = domCache.get('helipr-progress-fill');
        const text  = domCache.get('helipr-progress-text');
        const msgEl = domCache.get('helipr-progress-msg');
        _onHeliprConvertDone(msg.bag_path, btn, bar, fill, text, msgEl).catch(console.error);

    } else if (msg.type === 'helipr_convert_error') {
        const btn   = domCache.get('helipr-convert-btn');
        const bar   = domCache.get('helipr-progress-bar');
        const msgEl = domCache.get('helipr-progress-msg');
        heliprState.converting = false;
        if (btn) { btn.disabled = false; btn.textContent = 'Save Bag'; }
        if (bar) { bar.style.display = 'none'; }
        if (msgEl) { msgEl.textContent = 'Error: ' + (msg.error || 'Unknown'); }
        alert('Conversion error: ' + (msg.error || 'Unknown'));
    }
}

// 8081 WebSocket으로 subscribe_plot 명령 전송 (연결 전이면 대기열에 추가)
// msgType: 클라이언트가 이미 알고 있는 토픽 타입 → 서버에서 get_topic_names_and_types() 불필요
function _sendBackendSubscribePlot(topic, fieldPath, msgType) {
    const cmd = {
        cmd:      'subscribe_plot',
        topic:    topic,
        fields:   [fieldPath],
        msg_type: msgType || ''   // 서버에 전달하여 타이밍 문제 없이 즉시 subscription 생성
    };
    if (plotState.backendWs && plotState.backendWs.readyState === WebSocket.OPEN) {
        plotState.backendWs.send(JSON.stringify(cmd));
    } else {
        plotState._pendingPlotSubs.push(cmd);
        _initBackendWs(); // 연결 시도
    }
}

/**
 * 필드 경로(예: imu/data/angular_velocity/x)에서 ROS 토픽 이름(예: /imu/data) 추출
 * @param {string} fullPath
 * @returns {string|null}
 */
function extractRosTopicFromFieldPath(fullPath) {
    if (!plotState.topicTypes || plotState.topicTypes.size === 0) {
        return null;
    }
    const fp = fullPath.startsWith('/') ? fullPath.slice(1) : fullPath;
    let best = null;
    let maxLen = 0;
    for (const topicName of plotState.topicTypes.keys()) {
        const tn = topicName.startsWith('/') ? topicName.slice(1) : topicName;
        if (fp === tn || fp.startsWith(tn + '/')) {
            if (tn.length > maxLen) {
                maxLen = tn.length;
                best = topicName;
            }
        }
    }
    return best;
}

// Plot subscriber 키 생성 헬퍼 함수 (setupPlotDataUpdate와 동일한 형식)
function getPlotSubscriberKey(fullPath) {
    // plotState가 초기화되지 않았거나 topicTypes가 없으면 null 반환
    if (!plotState || !plotState.topicTypes) {
        return null;
    }
    
    // 토픽 목록에서 path와 매칭되는 가장 긴 토픽 찾기
    let topic = null;
    let fieldPath = null;
    let maxMatchLength = 0;
    
    for (const [topicName, topicType] of plotState.topicTypes.entries()) {
        // 토픽 이름에서 / 제거하여 비교
        const topicNameWithoutSlash = topicName.startsWith('/') ? topicName.substring(1) : topicName;
        
        // fullPath가 topicNameWithoutSlash로 시작하는지 확인
        if (fullPath.startsWith(topicNameWithoutSlash + '/') || fullPath === topicNameWithoutSlash) {
            const matchLength = topicNameWithoutSlash.length;
            if (matchLength > maxMatchLength) {
                maxMatchLength = matchLength;
                topic = topicName;
                fieldPath = fullPath.substring(matchLength + 1); // +1 for the '/'
            }
        }
    }
    
    if (!topic) {
        // topic을 찾지 못한 경우 null 반환 (setupPlotDataUpdate에서 처리)
        return null;
    }
    
    // setupPlotDataUpdate와 동일한 형식으로 키 생성
    return `${topic}_plot_${fieldPath.replace(/\//g, '_')}`;
}

/**
 * Plot 탭을 닫거나 비울 때: 해당 탭의 path에 대해 백엔드 구독 해제 및 전역 plottedPaths 정리.
 * 다른 탭이 동일 path를 쓰면 구독은 유지한다.
 * @param {PlotTabManager} tabManager
 * @param {object|null} plotManager — PlotlyPlotManager 인스턴스
 */
function releasePlotPathsFromPlotManager(tabManager, plotManager) {
    if (!plotManager || !plotManager.dataBuffers || typeof plotManager.dataBuffers.keys !== 'function') {
        return;
    }
    const paths = Array.from(plotManager.dataBuffers.keys());
    paths.forEach((fullPath) => {
        const usedElsewhere = tabManager.tabs.some(
            (t) => t.plotManager && t.plotManager !== plotManager && t.plotManager.dataBuffers.has(fullPath)
        );
        if (usedElsewhere) {
            return;
        }
        const key = getPlotSubscriberKey(fullPath);
        if (key && plotState.subscribers.has(key)) {
            const sub = plotState.subscribers.get(key);
            if (sub && typeof sub.unsubscribe === 'function') {
                sub.unsubscribe();
            }
            plotState.subscribers.delete(key);
        }
        plotState.plottedPaths = plotState.plottedPaths.filter((p) => p !== fullPath);
    });
}

window.releasePlotPathsFromPlotManager = releasePlotPathsFromPlotManager;

// PlotJugglerTree 초기화 및 토픽 노드 생성
function initPlotTree() {
    if (!plotState.tree) {
        plotState.tree = new PlotJugglerTree('plot-tree');
        console.log('[initPlotTree] PlotJugglerTree instance created');
    }
    plotState.tree.init();
}

// 토픽 노드를 트리 최상위에 추가 (모달에서 선택한 addedPlotTopics 만)
function createTopicNodes() {
    initPlotTree();

    const topicsToShow = Array.isArray(plotState.addedPlotTopics) ? plotState.addedPlotTopics.slice() : [];
    const newTopics = new Set(topicsToShow);
    const oldTopics = new Set(plotState.topicNodes.keys());

    oldTopics.forEach((topic) => {
        if (!newTopics.has(topic)) {
            unselectPlotTopic(topic);
            if (plotState.tree && typeof plotState.tree.pruneNodeMapForTopic === 'function') {
                plotState.tree.pruneNodeMapForTopic(topic);
            }
            const node = plotState.topicNodes.get(topic);
            if (node && node.parentElement) {
                node.parentElement.removeChild(node);
            }
            plotState.topicNodes.delete(topic);
            plotState.messageTrees.delete(topic);
            console.log(`[createTopicNodes] Removed topic node: ${topic}`);
        }
    });

    topicsToShow.forEach((topic) => {
        if (!plotState.topicNodes.has(topic)) {
            const topicName = topic.startsWith('/') ? topic.substring(1) : topic;
            const topicNode = plotState.tree.createNode(topic, topicName, false);

            topicNode.addEventListener('click', (e) => {
                if (e.target.classList.contains('plot-tree-expand-icon')) {
                    return;
                }
                e.stopPropagation();

                if (e.ctrlKey || e.metaKey) {
                    if (plotState.selectedTopics.has(topic)) {
                        unselectPlotTopic(topic);
                    } else {
                        selectPlotTopic(topic);
                    }
                } else {
                    if (plotState.selectedTopics.has(topic) && plotState.selectedTopics.size === 1) {
                        unselectPlotTopic(topic);
                    } else {
                        Array.from(plotState.selectedTopics).forEach((t) => unselectPlotTopic(t));
                        selectPlotTopic(topic);
                    }
                }
            });

            plotState.tree.rootNode.childrenContainer.appendChild(topicNode);
            plotState.topicNodes.set(topic, topicNode);
            console.log(`[createTopicNodes] Added new topic node: ${topic}`);
        }
    });

    const totalNodes = plotState.tree.rootNode.childrenContainer.children.length;
    console.log(`[createTopicNodes] Total topic nodes in DOM: ${totalNodes}`);
    if (totalNodes > 0) {
        console.log('[createTopicNodes] First node:', plotState.tree.rootNode.childrenContainer.children[0]);
    }
}

// rosbridge 연결
/**
 * rosbridge 연결 상태를 topbar chip에 반영
 * @param {'connected'|'disconnected'|'reconnecting'} state - 연결 상태
 */
function updateRosbridgeStatusChip(state) {
    const chip = document.getElementById('rosbridge-status-chip');
    if (!chip) return;

    // 상태별 클래스/텍스트 맵
    const stateMap = {
        connected:    { cls: 'chip-connected',    text: 'rosbridge: connected' },
        disconnected: { cls: 'chip-disconnected',  text: 'rosbridge: error' },
        reconnecting: { cls: 'chip-reconnecting',  text: 'rosbridge: reconnecting...' }
    };

    const config = stateMap[state];
    if (!config) return;

    // 기존 상태 클래스 제거 후 새 클래스 적용
    chip.classList.remove('chip-soft', 'chip-connected', 'chip-disconnected', 'chip-reconnecting');
    chip.classList.add(config.cls);
    chip.textContent = config.text;
}
window.updateRosbridgeStatusChip = updateRosbridgeStatusChip;

/**
 * rosbridge WebSocket이 실제로 동작하는지 getTopics로 검증.
 * isConnected=true 이지만 죽은(stale) 연결을 감지한다.
 * @param {object} ros - ROSLIB.Ros 인스턴스
 * @param {number} [timeoutMs=2500]
 * @returns {Promise<boolean>}
 */
function _verifyRosbridgeAlive(ros, timeoutMs = 2500) {
    if (!ros || !ros.isConnected) {
        return Promise.resolve(false);
    }
    return new Promise((resolve) => {
        let done = false;
        const finish = (alive) => {
            if (done) return;
            done = true;
            clearTimeout(timer);
            resolve(alive);
        };
        const timer = setTimeout(() => finish(false), timeoutMs);
        try {
            ros.getTopics(
                () => finish(true),
                () => finish(false)
            );
        } catch (e) {
            finish(false);
        }
    });
}
window._verifyRosbridgeAlive = _verifyRosbridgeAlive;

// rosbridge WebSocket URL 결정 헬퍼
// - IP 주소 또는 localhost: 그대로 사용 (원격 접속 지원, DNS 즉시)
// - 호스트명(예: 'kkw'): localhost로 대체 (DNS/프록시 지연 방지)
function _getRosbridgeUrl(port) {
    if (typeof getRosbridgeUrl === 'function') {
        return getRosbridgeUrl();
    }
    const host = (typeof getWebSocketHost === 'function')
        ? getWebSocketHost()
        : (window.location.hostname || 'localhost');
    const resolvedPort = (typeof port === 'number' && Number.isFinite(port) && port > 0)
        ? port
        : 9090;
    return `ws://${host}:${resolvedPort}`;
}

function _normalizeTopicTypeForUi(typeName) {
    if (!typeName || typeof typeName !== 'string') return 'unknown';
    // ROS1에서는 nav_msgs/Path 같은 원본 타입이 구독 messageType과 일치해야 한다.
    // 여기서 /msg/ 형식으로 바꾸면 subscribeToTopic()가 첫 메시지를 못 받아
    // Plot tree 하위 필드가 생성되지 않는다.
    if (window._rosVersion === 1) return typeName;
    if (typeName.includes('/msg/')) return typeName;
    const slashCount = (typeName.match(/\//g) || []).length;
    if (slashCount === 1) {
        const parts = typeName.split('/');
        return `${parts[0]}/msg/${parts[1]}`;
    }
    return typeName;
}

async function _fetchTopicsFromBackendApi() {
    try {
        const result = await apiCall('/api/recorder/get_topics');
        if (!result || !result.success || !Array.isArray(result.topics)) {
            return { topics: [], types: [] };
        }

        const topics = [];
        const types = [];
        result.topics.forEach((entry) => {
            if (typeof entry === 'string') {
                topics.push(entry);
                types.push('unknown');
                return;
            }
            if (!entry || typeof entry !== 'object' || !entry.name) {
                return;
            }
            topics.push(entry.name);
            types.push(_normalizeTopicTypeForUi(entry.type));
        });
        return { topics, types };
    } catch (error) {
        console.warn('[loadPlotTopics] Backend topic fallback failed:', error);
        return { topics: [], types: [] };
    }
}

function _applyPlotTopicsToState(topics, types) {
    const topicTypesMap = new Map();
    topics.forEach((name, index) => {
        topicTypesMap.set(name, types[index] || 'unknown');
    });
    plotState.topicTypes = topicTypesMap;

    const oldTopicsSet = new Set(plotState.topics);
    const addedTopics = topics.filter((t) => !oldTopicsSet.has(t));
    if (addedTopics.length > 0) {
        console.log('[loadPlotTopics] New topics detected:', addedTopics);
    }
    const removedTopics = plotState.topics.filter((t) => !new Set(topics).has(t));
    if (removedTopics.length > 0) {
        console.log('[loadPlotTopics] Removed topics:', removedTopics);
    }

    plotState.topics = topics;

    const rosSet = new Set(plotState.topics);
    const removedFromPanel = plotState.addedPlotTopics.filter((t) => !rosSet.has(t));
    removedFromPanel.forEach((t) => unselectPlotTopic(t));
    plotState.addedPlotTopics = plotState.addedPlotTopics.filter((t) => rosSet.has(t));

    displayTopicList();
}

async function _getPlotTopicsWithFallback(timeoutMs = 10000) {
    let rosapiError = null;

    try {
        const result = await new Promise((resolve, reject) => {
            let timeoutId = null;
            let completed = false;
            timeoutId = setTimeout(() => {
                if (!completed) {
                    completed = true;
                    reject(new Error('Topic loading timeout'));
                }
            }, timeoutMs);

            try {
                plotState.ros.getTopics((topicsResult) => {
                    if (completed) return;
                    completed = true;
                    clearTimeout(timeoutId);
                    resolve(topicsResult);
                }, (error) => {
                    if (completed) return;
                    completed = true;
                    clearTimeout(timeoutId);
                    reject(error);
                });
            } catch (error) {
                if (!completed) {
                    completed = true;
                    clearTimeout(timeoutId);
                    reject(error);
                }
            }
        });

        const topics = result.topics || [];
        const types = result.types || [];
        if (topics.length > 0) {
            return { topics, types, source: 'rosapi' };
        }
    } catch (error) {
        rosapiError = error;
    }

    const fallback = await _fetchTopicsFromBackendApi();
    if (fallback.topics.length > 0) {
        if (rosapiError) {
            console.warn(
                '[loadPlotTopics] rosapi unavailable, using backend topic fallback (rosgraph):',
                rosapiError.message || rosapiError
            );
        } else {
            console.warn('[loadPlotTopics] Using backend topic fallback (rosgraph)');
        }
        return { topics: fallback.topics, types: fallback.types, source: 'backend' };
    }

    if (rosapiError) {
        throw rosapiError;
    }
    return { topics: [], types: [], source: 'none' };
}

function initRosbridge() {
    if (typeof ROSLIB === 'undefined') {
        console.error('[rosbridge] ROSLIB not loaded');
        return;
    }

    // close→reconnect 타이머가 중복 쌓이지 않도록
    if (initRosbridge._reconnectTimer) {
        clearTimeout(initRosbridge._reconnectTimer);
        initRosbridge._reconnectTimer = null;
    }

    const doInit = async () => {
        try {
            const url = _getRosbridgeUrl();
            const pageHost = window.location.hostname || 'localhost';
            console.log('[rosbridge] Connecting to', url, '| page host:', pageHost);

            if (plotState.ros && plotState.ros.isConnected) {
                const alive = await _verifyRosbridgeAlive(plotState.ros, 2500);
                if (alive) {
                    console.log('[rosbridge] Verified alive, skipping duplicate init');
                    updateRosbridgeStatusChip('connected');
                    if (typeof _syncViewerRosFromPlotState === 'function') {
                        await _syncViewerRosFromPlotState();
                    }
                    return;
                }
                console.warn('[rosbridge] Stale connection (isConnected but dead) — reconnecting');
                try { plotState.ros.close(); } catch (e) { /* ignore */ }
                plotState.ros = null;
                if (typeof window.invalidateViewerRosConnection === 'function') {
                    window.invalidateViewerRosConnection();
                }
            } else if (plotState.ros) {
                try { plotState.ros.close(); } catch (e) { /* ignore */ }
                plotState.ros = null;
            }

            updateRosbridgeStatusChip('reconnecting');

            plotState.ros = new ROSLIB.Ros({ url });

            plotState.ros.on('connection', () => {
                console.log('[rosbridge] Connected:', url);
                updateRosbridgeStatusChip('connected');
                if (typeof _syncViewerRosFromPlotState === 'function') {
                    _syncViewerRosFromPlotState();
                }
                loadPlotTopics();
            });

            plotState.ros.on('error', (error) => {
                console.error('[rosbridge] Connection error:', url, error);
                updateRosbridgeStatusChip('disconnected');
                const container = domCache.get('plot-tree');
                if (container) {
                    plotState.tree = null;
                    container.innerHTML = '<div class="plot-tree-status-msg" style="color: var(--warning); padding: 12px; text-align: center;">rosbridge connection failed. Check rosbridge status and configured port.</div>';
                }
            });

            plotState.ros.on('close', () => {
                console.log('[rosbridge] Connection closed. Reconnecting in 3s:', url);
                updateRosbridgeStatusChip('reconnecting');
                const container = domCache.get('plot-tree');
                if (container) {
                    plotState.tree = null;
                    container.innerHTML = '<div class="plot-tree-status-msg" style="color: var(--muted); padding: 12px; text-align: center;">rosbridge disconnected. Reconnecting...</div>';
                }
                plotState.ros = null;
                if (initRosbridge._reconnectTimer) {
                    clearTimeout(initRosbridge._reconnectTimer);
                }
                initRosbridge._reconnectTimer = setTimeout(() => {
                    initRosbridge._reconnectTimer = null;
                    initRosbridge();
                }, 3000);
            });
        } catch (error) {
            console.error('[rosbridge] Failed to initialize:', error);
        }
    };

    if (typeof ensureWebuiPortsReady === 'function') {
        ensureWebuiPortsReady().then(doInit);
    } else {
        doInit();
    }
}

/**
 * Live Viewer용 백엔드 binary WebSocket (8881) — 재연결 포함.
 * PC2/Path는 rosbridge 없이 동작한다.
 * connectGen: hide() 시 증가하여 show/hide 경쟁 중 stale 연결 시도를 무효화.
 */
function _createLiveViewerBackendWs(viewer, label, onOpen, onBinaryMessage, connectGen) {
    let ws = null;
    let reconnectTimer = null;
    let stopped = false;
    // TCP/브라우저에 쌓인 바이너리를 하나씩 처리하지 않고 최신 프레임만 유지 (catch-up 방지)
    let pendingBinary = null;
    let rafId = null;
    const gen = (typeof connectGen === 'number') ? connectGen : (viewer._wsConnectGen || 0);

    const isStale = () => stopped || (viewer._wsConnectGen !== gen);

    const flushPending = () => {
        rafId = null;
        if (isStale()) {
            pendingBinary = null;
            return;
        }
        const buf = pendingBinary;
        pendingBinary = null;
        if (!buf) return;
        try {
            onBinaryMessage(buf);
        } catch (e) {
            console.error(`[${label}] Backend WS binary handler failed:`, e);
        }
        // 처리 중 새 프레임이 왔으면 다음 프레임에서 최신만 다시 처리
        if (pendingBinary != null && rafId == null) {
            rafId = requestAnimationFrame(flushPending);
        }
    };

    const connect = () => {
        if (isStale()) {
            console.log(`[${label}] Backend WS connect skipped (stale gen ${gen} vs ${viewer._wsConnectGen})`);
            return;
        }

        const startWs = () => {
            if (isStale()) return;
            const url = getPc2WsUrl();
            console.log(`[${label}] Backend WS connecting:`, url, `(gen=${gen})`);
            ws = new WebSocket(url);
            ws.binaryType = 'arraybuffer';

            ws.onopen = () => {
                if (isStale()) {
                    console.warn(`[${label}] Backend WS opened but stale — closing`);
                    try { ws.close(); } catch (e) { /* ignore */ }
                    return;
                }
                console.log(`[${label}] Backend WS connected:`, url);
                try {
                    onOpen(ws);
                    console.log(`[${label}] Backend WS subscribe sent`);
                } catch (e) {
                    console.error(`[${label}] Backend WS onOpen handler failed:`, e);
                }
            };

            ws.onmessage = (ev) => {
                if (!(ev.data instanceof ArrayBuffer)) return;
                // 중간 프레임 drop — 큐를 순서대로 비우지 않음
                pendingBinary = ev.data;
                if (rafId == null) {
                    rafId = requestAnimationFrame(flushPending);
                }
            };

            ws.onerror = () => {
                console.warn(`[${label}] Backend WS error:`, url);
            };

            ws.onclose = () => {
                console.log(`[${label}] Backend WS closed, retry in 3s`);
                ws = null;
                if (!isStale()) {
                    reconnectTimer = setTimeout(connect, 3000);
                }
            };
        };

        if (typeof ensureWebuiPortsReady === 'function') {
            ensureWebuiPortsReady().then(startWs);
        } else {
            startWs();
        }
    };

    connect();

    return {
        isConnected: () => ws && ws.readyState === WebSocket.OPEN,
        unsubscribe: () => {
            stopped = true;
            pendingBinary = null;
            if (rafId != null) {
                cancelAnimationFrame(rafId);
                rafId = null;
            }
            if (reconnectTimer) {
                clearTimeout(reconnectTimer);
                reconnectTimer = null;
            }
            try {
                if (ws) ws.close();
            } catch (e) { /* ignore */ }
            ws = null;
        }
    };
}

// 토픽 목록 로드 (rosbridge 사용)
async function loadPlotTopics() {
    console.log('[loadPlotTopics] Loading topics...');
    
    if (!plotState.ros || !plotState.ros.isConnected) {
        console.warn('[loadPlotTopics] rosbridge not connected');
        const container = domCache.get('plot-tree');
        if (container) {
            plotState.tree = null;
            container.innerHTML = '<div class="plot-tree-status-msg" style="color: var(--warning); padding: 12px; text-align: center;">rosbridge not connected. Waiting for connection...</div>';
        }
        return;
    }

    const alive = await _verifyRosbridgeAlive(plotState.ros, 2500);
    if (!alive) {
        console.warn('[loadPlotTopics] rosbridge stale — forcing reconnect');
        try { plotState.ros.close(); } catch (e) { /* ignore */ }
        plotState.ros = null;
        if (typeof window.invalidateViewerRosConnection === 'function') {
            window.invalidateViewerRosConnection();
        }
        initRosbridge();
        return;
    }

    // 이미 로딩 중이면 스킵
    if (plotState.isLoadingTopics) {
        console.log('[loadPlotTopics] Already loading topics, skipping...');
        return;
    }

    plotState.isLoadingTopics = true;

    try {
        const { topics, types, source } = await _getPlotTopicsWithFallback(10000);

        console.log('[loadPlotTopics] Received topics:', topics.length, `(source: ${source})`);
        console.log('[loadPlotTopics] Topic list:', topics);

        if (topics.length === 0) {
            if (plotState.topics && plotState.topics.length > 0) {
                console.warn('[loadPlotTopics] No topics returned, keeping existing list');
                return;
            }
            const container = domCache.get('plot-tree');
            if (container) {
                plotState.tree = null;
                container.innerHTML = '<div class="plot-tree-status-msg" style="color: var(--warning); padding: 12px; text-align: center;">No ROS topics found. Start ROS nodes or play a bag first.</div>';
            }
            return;
        }

        _applyPlotTopicsToState(topics, types);
    } catch (error) {
        console.error('[loadPlotTopics] Error:', error);

        // 타임아웃이 발생했지만 이미 토픽 목록이 있는 경우 (기존 플롯이 동작 중)
        if (plotState.topics && plotState.topics.length > 0) {
            console.warn('[loadPlotTopics] Error occurred, but keeping existing topics');
            return;
        }

        const container = domCache.get('plot-tree');
        if (container) {
            plotState.tree = null;
            container.innerHTML = `<div class="plot-tree-status-msg" style="color: var(--danger); padding: 12px; text-align: center;">Failed to load topics: ${error.message}</div>`;
        }
    } finally {
        plotState.isLoadingTopics = false;
        
        // 토픽 로딩 완료 후 저장된 paths 복원 (최초 1회만)
        if (plotState.plotTabManager && !plotState.pathsRestored) {
            console.log('[loadPlotTopics] Restoring saved paths...');
            restoreSavedPaths();
            plotState.pathsRestored = true;
        }
    }
}

// 저장된 paths 복원 (페이지 새로고침 후)
function restoreSavedPaths() {
    if (!plotState.plotTabManager || !plotState.plotTabManager.tabs) {
        console.warn('[restoreSavedPaths] PlotTabManager not initialized');
        return;
    }

    console.log('[restoreSavedPaths] Restoring saved paths for all tabs...');

    const tabsWithSaved = plotState.plotTabManager.tabs.filter((t) => t.savedPaths && t.savedPaths.length > 0);
    if (tabsWithSaved.length === 0) {
        if (plotState.plotTabManager.tabs.length > 0) {
            const activeTabId = plotState.plotTabManager.activeTabId || plotState.plotTabManager.tabs[0].id;
            plotState.plotTabManager.switchTab(activeTabId);
        }
        return;
    }

    const allTopics = new Set(plotState.addedPlotTopics);
    tabsWithSaved.forEach((tab) => {
        tab.savedPaths.forEach((p) => {
            const t = extractRosTopicFromFieldPath(p);
            if (t) allTopics.add(t);
        });
    });
    plotState.addedPlotTopics = Array.from(allTopics);
    displayTopicList();
    plotState.addedPlotTopics.forEach((t) => {
        if (!plotState.messageTrees.has(t)) {
            selectPlotTopic(t);
        }
    });

    setTimeout(() => {
        tabsWithSaved.forEach((tab) => {
            const paths = tab.savedPaths;
            if (!paths || paths.length === 0) return;

            console.log(`[restoreSavedPaths] Restoring ${paths.length} path(s) for tab ${tab.id}:`, paths);
            plotState.plotTabManager.switchTab(tab.id);

            const success = tab.plotManager.createPlot(paths);
            if (success) {
                const newPaths = paths.filter((p) => !plotState.plottedPaths.includes(p));
                plotState.plottedPaths = plotState.plottedPaths.concat(newPaths);
                paths.forEach((path) => {
                    const plotSubscriberKey = getPlotSubscriberKey(path);
                    if (!plotSubscriberKey || !plotState.subscribers.has(plotSubscriberKey)) {
                        setupPlotDataUpdate(path);
                    }
                });
            } else {
                console.error(`[restoreSavedPaths] Failed to create plot for tab ${tab.id}`);
            }
            delete tab.savedPaths;
        });

        if (plotState.plotTabManager.tabs.length > 0) {
            const activeTabId = plotState.plotTabManager.activeTabId || plotState.plotTabManager.tabs[0].id;
            plotState.plotTabManager.switchTab(activeTabId);
            console.log(`[restoreSavedPaths] Switched to active tab: ${activeTabId}`);
        }
    }, 450);
}

// 토픽 목록 표시 (PlotJuggler 스타일 - addedPlotTopics 만 트리에 표시)
function displayTopicList() {
    const container = domCache.get('plot-tree');
    if (!container) {
        console.error('[displayTopicList] Container not found');
        return;
    }

    container.querySelector('.plot-tree-empty-hint')?.remove();
    container.querySelector('.plot-tree-status-msg')?.remove();

    createTopicNodes();

    const cc = plotState.tree && plotState.tree.rootNode && plotState.tree.rootNode.childrenContainer;
    if (cc && plotState.addedPlotTopics.length === 0) {
        const hint = document.createElement('div');
        hint.className = 'plot-tree-empty-hint';
        hint.style.cssText = 'color: var(--muted); padding: 10px 8px; text-align: center; font-size: 12px; line-height: 1.45;';
        hint.textContent = (plotState.topics && plotState.topics.length === 0)
            ? '「+ Add」로 토픽을 선택하세요. 지금은 ROS에 publish된 토픽이 없어 목록이 비어 있을 수 있습니다.'
            : '「+ Add」에서 표시할 토픽을 선택하세요. 선택한 토픽만 아래 트리에 나타납니다.';
        cc.appendChild(hint);
    }

    console.log('[displayTopicList] addedPlotTopics:', plotState.addedPlotTopics.length);
}

// 토픽 선택 및 구독 (PlotJuggler 스타일)
function selectPlotTopic(topic) {
    // 이미 구독 중이면 무시
    if (plotState.selectedTopics.has(topic)) {
        console.log(`[selectPlotTopic] Topic already subscribed: ${topic}`);
        return;
    }

    plotState.selectedTopics.add(topic);
    console.log(`[selectPlotTopic] Subscribing to topic: ${topic}`);

    // 토픽 노드 강조 표시 및 확장
    const topicNode = plotState.topicNodes.get(topic);
    if (topicNode) {
        topicNode.classList.add('plot-tree-topic-subscribed');
        
        // 자동으로 토픽 노드 확장 (메시지 트리 보이도록)
        if (!topicNode.classList.contains('plot-tree-expanded')) {
            plotState.tree.toggleExpand(topicNode);
        }
    }

    // 토픽 구독
    subscribeToTopic(topic);
}

// 토픽 구독 해제
function unselectPlotTopic(topic) {
    if (!plotState.selectedTopics.has(topic)) {
        return;
    }

    plotState.selectedTopics.delete(topic);
    
    // 구독 해제
    if (plotState.subscribers.has(topic)) {
        plotState.subscribers.get(topic).unsubscribe();
        plotState.subscribers.delete(topic);
    }
    
    // 토픽 노드 강조 해제
    const topicNode = plotState.topicNodes.get(topic);
    if (topicNode) {
        topicNode.classList.remove('plot-tree-topic-subscribed');
    }
    
    console.log(`[unselectPlotTopic] Unsubscribed from topic: ${topic}`);
}

// 토픽 구독
function subscribeToTopic(topic) {
    if (!plotState.ros || !plotState.ros.isConnected) {
        console.error('[subscribeToTopic] rosbridge not connected');
        return;
    }

    // 기존 구독 해제
    if (plotState.subscribers.has(topic)) {
        console.log(`[subscribeToTopic] Unsubscribing from existing: ${topic}`);
        plotState.subscribers.get(topic).unsubscribe();
        plotState.subscribers.delete(topic);
    }

    // 토픽 타입 조회 (plotState.topicTypes에서 가져오기)
    const messageType = plotState.topicTypes.get(topic);
    
    if (!messageType) {
        console.error(`[subscribeToTopic] Topic type not found for: ${topic}`);
        console.log('[subscribeToTopic] Available types:', Array.from(plotState.topicTypes.keys()).slice(0, 5));
        return;
    }

    console.log(`[subscribeToTopic] Subscribing to ${topic} (${messageType})`);

    // 메시지 트리 표시 목적 — 구조 파악 후 즉시 unsubscribe.
    // throttle_rate:0 (원래 주기, rosbridge 측 throttle 없음) + queue_length:1.
    // PC2 여부와 무관하게 첫 메시지 1개 수신 후 바로 unsubscribe하므로 rosbridge 부하 없음.
    const isPC2 = (messageType === 'sensor_msgs/msg/PointCloud2' ||
                   messageType === 'sensor_msgs/PointCloud2');

    const listener = new ROSLIB.Topic({
        ros: plotState.ros,
        name: topic,
        messageType: messageType,
        throttle_rate: isPC2 ? 2000 : 0, // PC2는 여전히 2초 (10MB+ 보호), 나머지는 즉시
        queue_length: 1
    });

    listener.subscribe((message) => {
        if (!plotState.messageTrees.has(topic)) {
            console.log(`[subscribeToTopic] First message received for ${topic}`);
        }
        updateMessageTree(topic, message);
        // 첫 메시지로 구조 파악 완료 → 즉시 unsubscribe (rosbridge 부하 최소화)
        listener.unsubscribe();
        plotState.subscribers.delete(topic);
        console.log(`[subscribeToTopic] Tree captured, unsubscribed: ${topic}`);
    });

    plotState.subscribers.set(topic, listener);
    console.log(`[subscribeToTopic] Successfully subscribed to ${topic}`);
}

// 메시지 트리 업데이트 (PlotJuggler 스타일 - 토픽 하위에 추가)
function updateMessageTree(topic, message) {
    if (!plotState.tree) {
        initPlotTree();
    }

    // 토픽 노드 가져오기
    const topicNode = plotState.topicNodes.get(topic);
    if (!topicNode) {
        console.error(`[updateMessageTree] Topic node not found: ${topic}`);
        return;
    }

    // PlotJuggler 스타일로 메시지를 재귀적으로 flatten
    const flattenedData = new Map();
    
    function flattenMessage(obj, prefix = '') {
        if (obj === null || obj === undefined) {
            return;
        }

        if (Array.isArray(obj)) {
            // 배열인 경우: 각 요소를 인덱스로 접근
            if (obj.length > 0) {
                if (typeof obj[0] === 'object' && obj[0] !== null) {
                    // 객체 배열: 첫 번째 요소만 파싱 (PlotJuggler 스타일)
                    flattenMessage(obj[0], prefix ? `${prefix}[0]` : '[0]');
                } else {
                    // 기본 타입 배열: 첫 번째 값만 표시
                    flattenedData.set(prefix, obj[0]);
                }
            }
        } else if (typeof obj === 'object') {
            // 객체인 경우: 각 키를 재귀적으로 처리
            Object.keys(obj).forEach(key => {
                const value = obj[key];
                const newPath = prefix ? `${prefix}/${key}` : key;
                
                if (value === null || value === undefined) {
                    // null/undefined는 건너뛰기
                    return;
                } else if (Array.isArray(value)) {
                    // 배열 필드
                    if (value.length > 0) {
                        if (typeof value[0] === 'object' && value[0] !== null) {
                            // 객체 배열: 첫 번째 요소만 파싱
                            flattenMessage(value[0], `${newPath}[0]`);
                        } else {
                            // 기본 타입 배열: 첫 번째 값만 표시 (리프 노드)
                            flattenedData.set(newPath, value[0]);
                        }
                    } else {
                        // 빈 배열은 건너뛰기
                        return;
                    }
                } else if (typeof value === 'object') {
                    // 중첩 객체: 재귀적으로 처리
                    flattenMessage(value, newPath);
                } else {
                    // 리프 노드 (기본 타입: number, string, boolean)
                    flattenedData.set(newPath, value);
                }
            });
        } else {
            // 기본 타입 (number, string, boolean)
            flattenedData.set(prefix, obj);
        }
    }

    // 메시지 flatten (prefix는 빈 문자열로 시작, 나중에 토픽 이름 추가)
    const topicName = topic.startsWith('/') ? topic.substring(1) : topic;
    flattenMessage(message, '');

    console.log(`[updateMessageTree] Topic: ${topic}, Flattened items: ${flattenedData.size}`);
    if (flattenedData.size === 0) {
        console.warn(`[updateMessageTree] No flattened data for topic: ${topic}`);
        return;
    }

    // 트리 재구성 (첫 메시지인 경우에만)
    const isFirstMessage = plotState.messageTrees.get(topic) === undefined;
    
    if (isFirstMessage) {
        // 첫 메시지: 트리 구조 생성 (토픽 노드 하위에 추가)
        console.log(`[updateMessageTree] First message for ${topic}, building tree structure...`);
        
        flattenedData.forEach((value, path) => {
            // 전체 경로: topic/path
            const fullPath = `${topicName}/${path}`;
            
            // 경로를 /로 분리
            const parts = path.split('/').filter(p => p.length > 0);
            let currentParent = topicNode;
            let currentPath = topicName;

            for (let i = 0; i < parts.length; i++) {
                const part = parts[i];
                const isLeaf = (i === parts.length - 1);
                currentPath = `${currentPath}/${part}`;

                let child = plotState.tree.findChildByName(currentParent, part);

                if (!child) {
                    child = plotState.tree.createNode(part, currentPath, isLeaf);
                    currentParent.childrenContainer.appendChild(child);
                }

                currentParent = child;
            }

            // 리프 노드인 경우 값 업데이트
            if (currentParent && currentParent.valueElement) {
                plotState.tree.updateValue(currentPath, value);
            }
        });
        
        plotState.messageTrees.set(topic, true);
        
        // 토픽 노드 자동 확장
        if (topicNode.childrenContainer.style.display === 'none' || topicNode.childrenContainer.style.display === '') {
            plotState.tree.toggleExpand(topicNode);
        }
        
        // 디버깅: 트리 상태 확인
        console.log(`[updateMessageTree] First message processed for ${topic}`);
    } else {
        // 이후 메시지: 값만 업데이트
        flattenedData.forEach((value, path) => {
            const fullPath = `${topicName}/${path}`;
            plotState.tree.updateValue(fullPath, value);
        });
    }
    
    const leafNodeCount = Array.from(plotState.tree.nodeMap.values()).filter(n => n.dataset.isLeaf === 'true').length;
    console.log(`[updateMessageTree] Tree update complete. Total leaf nodes: ${leafNodeCount}`);
}

// 트리 전체 확장
function expandAllPlotTree() {
    if (plotState.tree) {
        plotState.tree.expandAll();
        console.log('[expandAllPlotTree] All nodes expanded');
    }
}

// 트리 전체 축소
function collapseAllPlotTree() {
    if (plotState.tree) {
        plotState.tree.collapseAll();
        console.log('[collapseAllPlotTree] All nodes collapsed');
    }
}

/**
 * Plot 패널: 현재 ROS에 publish된 토픽을 모달에서 선택 (Bag Player Select Topic과 유사)
 */
async function openPlotTopicSelectionModal() {
    if (!plotState.ros || !plotState.ros.isConnected) {
        alert('rosbridge에 연결된 뒤 토픽을 선택할 수 있습니다.');
        return;
    }
    if (plotState.isLoadingTopics) {
        const waitStart = Date.now();
        while (plotState.isLoadingTopics && (Date.now() - waitStart) < 1500) {
            await new Promise((resolve) => setTimeout(resolve, 60));
        }
    }
    await loadPlotTopics();
    if (!plotState.topics || plotState.topics.length === 0) {
        alert('현재 publish된 토픽이 없습니다.');
        return;
    }

    const topicList = document.getElementById('plot-modal-topic-list');
    if (!topicList) return;
    topicList.innerHTML = '';

    plotState.topics.forEach((topicName, index) => {
        const topicType = plotState.topicTypes.get(topicName) || '';

        const div = document.createElement('div');
        div.className = 'topic-item';

        const checkbox = document.createElement('input');
        checkbox.type = 'checkbox';
        const safeId = `plot-topic-${index}-${topicName.replace(/[^a-zA-Z0-9]/g, '_')}`;
        checkbox.id = safeId;
        checkbox.value = topicName;
        checkbox.checked = plotState.addedPlotTopics.includes(topicName);

        const label = document.createElement('label');
        label.htmlFor = safeId;
        if (topicType) {
            label.innerHTML = `<span style="font-weight:600;">${topicName}</span> <span style="color:#888; font-size:0.85em;">${topicType}</span>`;
        } else {
            label.textContent = topicName;
        }

        div.appendChild(checkbox);
        div.appendChild(label);
        topicList.appendChild(div);
    });

    const modal = document.getElementById('plot-topic-selection-modal');
    if (modal) modal.style.display = 'block';
}

function closePlotTopicSelectionModal() {
    const modal = document.getElementById('plot-topic-selection-modal');
    if (modal) modal.style.display = 'none';
}

function confirmPlotTopicSelectionModal() {
    const checkboxes = document.querySelectorAll('#plot-modal-topic-list input[type="checkbox"]:checked');
    const next = [];
    checkboxes.forEach((cb) => next.push(cb.value));

    const prevSet = new Set(plotState.addedPlotTopics);
    const added = next.filter((t) => !prevSet.has(t));

    plotState.addedPlotTopics = next;
    displayTopicList();
    added.forEach((t) => selectPlotTopic(t));

    closePlotTopicSelectionModal();
}

window.openPlotTopicSelectionModal = openPlotTopicSelectionModal;
window.closePlotTopicSelectionModal = closePlotTopicSelectionModal;
window.confirmPlotTopicSelectionModal = confirmPlotTopicSelectionModal;

// 버퍼 시간 업데이트
function updateBufferTime(seconds) {
    const bufferTime = parseFloat(seconds);
    
    // 유효성 검사
    if (isNaN(bufferTime) || bufferTime < 1 || bufferTime > 100) {
        console.error('[updateBufferTime] Invalid buffer time:', seconds);
        alert('Buffer time must be between 1 and 100 seconds');
        // 기본값으로 복원
        document.getElementById('buffer-time-input').value = 5;
        return;
    }
    
    console.log(`[updateBufferTime] Setting buffer time to ${bufferTime} seconds`);
    
    // PlotTabManager가 초기화되어 있으면 모든 탭의 버퍼 시간 업데이트
    if (plotState.plotTabManager) {
        plotState.plotTabManager.setBufferTime(bufferTime);
    }
}

// Plot 영역 드롭 이벤트 처리
let isPlotDropZoneSetup = false;  // 중복 등록 방지 플래그

function setupPlotDropZone() {
    const plotAreaContainer = document.getElementById('plot-area-container');
    if (!plotAreaContainer) {
        console.warn('plot-area-container element not found');
        return;
    }

    // 이미 설정되었으면 스킵
    if (isPlotDropZoneSetup) {
        console.log('[setupPlotDropZone] Already setup, skipping...');
        return;
    }

    console.log('[setupPlotDropZone] Setting up drop zone...');
    isPlotDropZoneSetup = true;

    plotAreaContainer.addEventListener('dragover', (e) => {
        e.preventDefault();
        e.stopPropagation();
        e.dataTransfer.dropEffect = 'move';
        plotAreaContainer.style.backgroundColor = 'rgba(74, 214, 255, 0.1)';
        plotAreaContainer.style.border = '2px dashed rgba(74, 214, 255, 0.5)';
    });

    plotAreaContainer.addEventListener('dragleave', (e) => {
        e.preventDefault();
        e.stopPropagation();
        // plot-area-container 내부의 자식 요소로 이동한 경우는 제외
        if (!plotAreaContainer.contains(e.relatedTarget)) {
            plotAreaContainer.style.backgroundColor = 'transparent';
            plotAreaContainer.style.border = 'none';
        }
    });

    plotAreaContainer.addEventListener('drop', (e) => {
        e.preventDefault();
        e.stopPropagation();
        plotAreaContainer.style.backgroundColor = 'transparent';
        plotAreaContainer.style.border = 'none';

        try {
            const data = e.dataTransfer.getData('text/plain');
            if (!data) {
                console.warn('No data in drop event');
                return;
            }

            // JSON 배열로 파싱 시도
            let paths = [];
            try {
                paths = JSON.parse(data);
                if (!Array.isArray(paths)) {
                    paths = [paths]; // 단일 값인 경우 배열로 변환
                }
            } catch (parseError) {
                // JSON이 아닌 경우 단일 문자열로 처리
                paths = [data];
            }

            console.log('[setupPlotDropZone] Dropped paths:', paths);
            console.log('[setupPlotDropZone] Current plotState.plottedPaths BEFORE:', plotState.plottedPaths);

            if (paths.length === 0) {
                console.warn('[setupPlotDropZone] No paths to plot');
                return;
            }

            // PlotTabManager가 초기화되어 있는지 확인
            if (!plotState.plotTabManager) {
                console.error('[setupPlotDropZone] PlotTabManager not initialized');
                return;
            }

            // 활성 탭의 PlotlyPlotManager 가져오기
            const plotManager = plotState.plotTabManager.getActivePlotManager();
            if (!plotManager) {
                console.error('[setupPlotDropZone] No active plot manager');
                return;
            }

            // Plot 생성 (모든 paths 전달 - createPlot이 내부에서 중복 처리)
            const success = plotManager.createPlot(paths);
            if (success) {
                // 기존 paths에 새로운 paths만 추가 (중복 제거)
                const newPaths = paths.filter(p => !plotState.plottedPaths.includes(p));
                console.log('[setupPlotDropZone] New paths to add:', newPaths);
                console.log('[setupPlotDropZone] Filtered out (already exists):', paths.filter(p => plotState.plottedPaths.includes(p)));
                
                plotState.plottedPaths = plotState.plottedPaths.concat(newPaths);
                console.log('[setupPlotDropZone] Plot created/updated. Total paths AFTER:', plotState.plottedPaths);
                
                // 새로운 path에 대해서만 실시간 데이터 업데이트 설정
                newPaths.forEach(path => {
                    // 이미 구독 중인지 확인 (setupPlotDataUpdate와 동일한 키 형식 사용)
                    const plotSubscriberKey = getPlotSubscriberKey(path);
                    if (!plotSubscriberKey || !plotState.subscribers.has(plotSubscriberKey)) {
                        setupPlotDataUpdate(path);
                    } else {
                        console.log(`[setupPlotDropZone] Already subscribed to: ${path}`);
                    }
                });
                
                // 탭 상태 저장
                if (plotState.plotTabManager) {
                    plotState.plotTabManager.saveState();
                }
            } else {
                console.error('[setupPlotDropZone] Failed to create plot');
            }
        } catch (error) {
            console.error('[setupPlotDropZone] Error handling drop event:', error);
        }
    });
}

// Plot 데이터 실시간 업데이트 설정
function setupPlotDataUpdate(fullPath) {
    console.log('[setupPlotDataUpdate] Setting up data update for:', fullPath);
    
    // fullPath에서 토픽과 필드 경로 분리
    // 토픽 목록에서 가장 긴 매칭을 찾음 (예: "imu/data/orientation/x" -> topic: "/imu/data", field: "orientation/x")
    const parts = fullPath.split('/').filter(p => p.length > 0);
    if (parts.length < 2) {
        console.warn('[setupPlotDataUpdate] Invalid path:', fullPath);
        return;
    }
    
    // 토픽 목록에서 path와 매칭되는 가장 긴 토픽 찾기
    let topic = null;
    let fieldPath = null;
    let maxMatchLength = 0;
    
    for (const [topicName, topicType] of plotState.topicTypes.entries()) {
        // 토픽 이름에서 / 제거하여 비교
        const topicNameWithoutSlash = topicName.startsWith('/') ? topicName.substring(1) : topicName;
        
        // fullPath가 topicNameWithoutSlash로 시작하는지 확인
        if (fullPath.startsWith(topicNameWithoutSlash + '/') || fullPath === topicNameWithoutSlash) {
            const matchLength = topicNameWithoutSlash.length;
            if (matchLength > maxMatchLength) {
                maxMatchLength = matchLength;
                topic = topicName;
                fieldPath = fullPath.substring(matchLength + 1); // +1 for the '/'
            }
        }
    }
    
    if (!topic) {
        console.error('[setupPlotDataUpdate] No matching topic found for path:', fullPath);
        console.log('[setupPlotDataUpdate] Available topics:', Array.from(plotState.topicTypes.keys()));
        return;
    }
    
    console.log('[setupPlotDataUpdate] Topic:', topic, 'Field path:', fieldPath);
    
    // Plot 전용 subscriber 키
    const plotSubscriberKey = `${topic}_plot_${fieldPath.replace(/\//g, '_')}`;
    
    if (plotState.subscribers.has(plotSubscriberKey)) {
        console.log('[setupPlotDataUpdate] Plot subscriber already exists for:', plotSubscriberKey);
        return;
    }
    
    // Topic 정보 조회 (메시지 타입 확인)
    const topicType = plotState.topicTypes.get(topic);
    if (!topicType) {
        console.error('[setupPlotDataUpdate] Topic type not found:', topic);
        console.log('[setupPlotDataUpdate] Available topics:', Array.from(plotState.topicTypes.keys()));
        return;
    }
    
    console.log('[setupPlotDataUpdate] Creating subscriber for topic:', topic, 'type:', topicType);

    // ── 모든 토픽 (PC2 포함): Python 백엔드 8081 WebSocket (throttle 없이 원래 주기) ─
    //
    // [이전 구조의 버그]
    //   PC2 타입 → pc2_topic_meta CustomEvent 방식 사용
    //   BUT: 이 이벤트는 3D Viewer의 pc2_stream_worker가 dispatch하므로
    //        3D Viewer에서 해당 PC2 토픽을 선택해야만 plot이 작동했음.
    //
    // [수정 후]
    //   PC2 포함 모든 토픽 → subscribe_plot 명령으로 통일.
    //   msg_type을 클라이언트에서 서버에 직접 전달하여 서버의
    //   get_topic_names_and_types() 의존성 제거 (타이밍 문제 해결).
    //
    // PC2의 point_count는 width*height 계산이 필요하므로 서버 특수 처리.
    // 나머지 header/stamp/sec 등은 _extract_nested()로 처리.
    // ─────────────────────────────────────────────────────────────────────────
    console.log(`[setupPlotDataUpdate] Backend WS 경로 사용: ${fullPath} (type: ${topicType})`);
    _sendBackendSubscribePlot(topic, fieldPath, topicType);

    plotState.subscribers.set(plotSubscriberKey, {
        unsubscribe: () => {
            if (plotState.backendWs && plotState.backendWs.readyState === WebSocket.OPEN) {
                plotState.backendWs.send(JSON.stringify({
                    cmd: 'unsubscribe_plot', topic: topic, fields: [fieldPath]
                }));
            }
        }
    });
    console.log('[setupPlotDataUpdate] Backend WS plot subscriber 등록:', plotSubscriberKey);
}

// 필드 경로를 따라가서 값 추출
function extractFieldValue(obj, fieldPath) {
    const fields = fieldPath.split('/');
    let value = obj;
    
    for (const field of fields) {
        if (value === null || value === undefined) {
            return undefined;
        }
        
        // 배열 인덱스 처리 (예: "covariance[0]")
        const arrayMatch = field.match(/^(\w+)\[(\d+)\]$/);
        if (arrayMatch) {
            const arrayName = arrayMatch[1];
            const index = parseInt(arrayMatch[2], 10);
            value = value[arrayName];
            if (Array.isArray(value)) {
                value = value[index];
            } else {
                return undefined;
            }
        } else {
            value = value[field];
        }
    }
    
    // 숫자 값만 반환 (Plot에 표시 가능)
    if (typeof value === 'number') {
        return value;
    } else if (typeof value === 'boolean') {
        return value ? 1 : 0;
    } else {
        console.warn('[extractFieldValue] Non-numeric value:', value);
        return undefined;
    }
}

// XY Plot 생성 함수 (PlotJugglerTree 컨텍스트 메뉴에서 호출)
function createXYPlot(xPath, yPath) {
    console.log('[createXYPlot] Creating XY Plot:', xPath, 'vs', yPath);
    
    // PlotTabManager가 초기화되어 있는지 확인
    if (!plotState.plotTabManager) {
        console.error('[createXYPlot] PlotTabManager not initialized');
        return;
    }
    
    // 활성 탭의 PlotlyPlotManager 가져오기
    const plotManager = plotState.plotTabManager.getActivePlotManager();
    if (!plotManager) {
        console.error('[createXYPlot] No active plot manager');
        return;
    }
    
    // XY Plot 생성
    const success = plotManager.createXYPlot(xPath, yPath);
    if (success) {
        console.log('[createXYPlot] XY Plot created successfully');
        
        // 전역 plottedPaths에 추가 (중복 제거)
        const paths = [xPath, yPath];
        const newPaths = paths.filter(p => !plotState.plottedPaths.includes(p));
        plotState.plottedPaths = plotState.plottedPaths.concat(newPaths);
        
        // 실시간 데이터 업데이트 설정
        paths.forEach(path => {
            const plotSubscriberKey = getPlotSubscriberKey(path);
            if (!plotSubscriberKey || !plotState.subscribers.has(plotSubscriberKey)) {
                setupPlotDataUpdate(path);
            }
        });
        
        // 탭 상태 저장
        plotState.plotTabManager.saveState();
    } else {
        console.error('[createXYPlot] Failed to create XY Plot');
    }
}

// ==============================================================
// Plot Settings 관련 전역 함수들
// ==============================================================
let currentPlotSettingsPlotId = null;

// Plot Settings 모달 열기
window.openPlotSettings = function(plotId) {
    console.log('[openPlotSettings] Opening settings for plot:', plotId);
    
    currentPlotSettingsPlotId = plotId;
    
    // 현재 플롯의 PlotlyPlotManager 가져오기
    const plotManager = plotState.plotTabManager.getPlotManager(plotId);
    if (!plotManager || !plotManager.isInitialized) {
        console.error('[openPlotSettings] Plot manager not found or not initialized:', plotId);
        return;
    }
    
    // Trace 선택 드롭다운 채우기
    const traceSelect = domCache.get('plot-settings-trace-select');
    if (!traceSelect) {
        console.error('[openPlotSettings] Trace select element not found');
        return;
    }
    
    traceSelect.innerHTML = '';
    plotManager.traces.forEach((trace, index) => {
        const option = document.createElement('option');
        option.value = index;
        option.textContent = trace.name || `Trace ${index + 1}`;
        traceSelect.appendChild(option);
    });
    
    // 첫 번째 trace가 있으면 선택
    if (plotManager.traces.length > 0) {
        traceSelect.value = 0;
        window.loadTraceSettings(0);
    }
    
    // Trace 선택 변경 시 현재 설정 로드
    traceSelect.onchange = () => {
        const selectedIndex = parseInt(traceSelect.value);
        window.loadTraceSettings(selectedIndex);
    };
    
    // 슬라이더 값 업데이트 이벤트
    const lineWidthSlider = domCache.get('plot-settings-line-width');
    const lineWidthValue = domCache.get('plot-settings-line-width-value');
    if (lineWidthSlider && lineWidthValue) {
        lineWidthSlider.oninput = () => {
            lineWidthValue.textContent = lineWidthSlider.value;
        };
    }
    
    const markerSizeSlider = domCache.get('plot-settings-marker-size');
    const markerSizeValue = domCache.get('plot-settings-marker-size-value');
    if (markerSizeSlider && markerSizeValue) {
        markerSizeSlider.oninput = () => {
            markerSizeValue.textContent = markerSizeSlider.value;
        };
    }
    
    // 모달 표시
    const modal = domCache.get('plot-settings-modal');
    if (modal) {
        modal.style.display = 'flex';
    }
};

// 현재 trace의 설정 로드 (전역 함수)
window.loadTraceSettings = function(traceIndex) {
    if (!currentPlotSettingsPlotId) {
        console.error('[loadTraceSettings] No plot ID set');
        return;
    }
    
    // 현재 플롯의 PlotlyPlotManager 가져오기
    const plotManager = plotState.plotTabManager.getPlotManager(currentPlotSettingsPlotId);
    if (!plotManager || !plotManager.isInitialized) {
        console.error('[loadTraceSettings] Plot manager not found or not initialized');
        return;
    }
    
    const trace = plotManager.traces[traceIndex];
    if (!trace) return;
    
    // 색상
    const colorInput = domCache.get('plot-settings-color');
    if (colorInput && trace.line && trace.line.color) {
        colorInput.value = trace.line.color;
    }
    
    // 선 스타일
    const lineStyleSelect = domCache.get('plot-settings-line-style');
    if (lineStyleSelect && trace.line && trace.line.dash) {
        lineStyleSelect.value = trace.line.dash;
    }
    
    // 마커 스타일
    const markerStyleSelect = domCache.get('plot-settings-marker-style');
    if (markerStyleSelect) {
        if (trace.mode === 'lines') {
            markerStyleSelect.value = 'none';
        } else if (trace.marker && trace.marker.symbol) {
            markerStyleSelect.value = trace.marker.symbol;
        }
    }
    
    // 선 두께
    const lineWidthSlider = domCache.get('plot-settings-line-width');
    const lineWidthValue = domCache.get('plot-settings-line-width-value');
    if (lineWidthSlider && trace.line && trace.line.width) {
        lineWidthSlider.value = trace.line.width;
        if (lineWidthValue) {
            lineWidthValue.textContent = trace.line.width;
        }
    }
    
    // 마커 크기
    const markerSizeSlider = domCache.get('plot-settings-marker-size');
    const markerSizeValue = domCache.get('plot-settings-marker-size-value');
    if (markerSizeSlider && trace.marker && trace.marker.size) {
        markerSizeSlider.value = trace.marker.size;
        if (markerSizeValue) {
            markerSizeValue.textContent = trace.marker.size;
        }
    }
    
    // 그리드 표시 (layout 설정)
    const showGridCheckbox = domCache.get('plot-settings-show-grid');
    if (showGridCheckbox && plotManager.layout) {
        const showGrid = plotManager.layout.xaxis?.showgrid !== false;
        showGridCheckbox.checked = showGrid;
    }
    
    // X축 라벨
    const xaxisLabelInput = domCache.get('plot-settings-xaxis-label');
    if (xaxisLabelInput && plotManager.layout && plotManager.layout.xaxis) {
        xaxisLabelInput.value = plotManager.layout.xaxis.title?.text || '';
    }
    
    // Y축 라벨
    const yaxisLabelInput = domCache.get('plot-settings-yaxis-label');
    if (yaxisLabelInput && plotManager.layout && plotManager.layout.yaxis) {
        yaxisLabelInput.value = plotManager.layout.yaxis.title?.text || '';
    }
};

// Plot Settings 모달 닫기
window.closePlotSettings = function() {
    console.log('[closePlotSettings] Closing settings modal');
    
    const modal = domCache.get('plot-settings-modal');
    if (modal) {
        modal.style.display = 'none';
    }
    
    currentPlotSettingsPlotId = null;
};

// Plot Settings 적용
window.applyPlotSettings = function() {
    console.log('[applyPlotSettings] Applying settings');
    
    if (!currentPlotSettingsPlotId) {
        console.error('[applyPlotSettings] No plot ID set');
        return;
    }
    
    // 현재 플롯의 PlotlyPlotManager 가져오기
    const plotManager = plotState.plotTabManager.getPlotManager(currentPlotSettingsPlotId);
    if (!plotManager || !plotManager.isInitialized) {
        console.error('[applyPlotSettings] Plot manager not found or not initialized');
        return;
    }
    
    // 모든 설정 값 읽기
    const traceIndex = parseInt(domCache.get('plot-settings-trace-select')?.value || 0);
    const color = domCache.get('plot-settings-color')?.value;
    const lineStyle = domCache.get('plot-settings-line-style')?.value;
    const markerStyle = domCache.get('plot-settings-marker-style')?.value;
    const lineWidth = parseFloat(domCache.get('plot-settings-line-width')?.value);
    const markerSize = parseFloat(domCache.get('plot-settings-marker-size')?.value);
    const showGrid = domCache.get('plot-settings-show-grid')?.checked;
    const xaxisLabel = domCache.get('plot-settings-xaxis-label')?.value;
    const yaxisLabel = domCache.get('plot-settings-yaxis-label')?.value;
    
    // 설정 객체 생성
    const settings = {
        traceIndex,
        color,
        lineStyle,
        markerStyle,
        lineWidth,
        markerSize,
        showGrid,
        xaxisLabel,
        yaxisLabel
    };
    
    // PlotlyPlotManager의 applyTraceSettings() 메서드 호출
    plotManager.applyTraceSettings(settings);
    
    // 모달 닫기
    window.closePlotSettings();
};

// 모달 외부 클릭 시 닫기
window.addEventListener('click', (event) => {
    const modal = domCache.get('plot-settings-modal');
    if (event.target === modal) {
        window.closePlotSettings();
    }
});

// ==============================================================
// Filter Dialog 관련 전역 함수들
// ==============================================================
let currentFilterPlotId = null;
let currentFilterTraceIndex = null;
let currentFilterType = null;

// filter-type-items의 data-filter 값 → PlotlyPlotManager.applyFilter() filterType 매핑
const FILTER_TYPE_MAP = {
    'no_transform':    'noTransform',
    'absolute':        'absolute',
    'derivative':      'derivative',
    'moving_average':  'movingAverage',
    'moving_rms':      'movingRMS',
    'moving_variance': 'movingVariance',
    'scale_offset':    'scaleOffset'
};

// filter 표시 레이블 매핑
const FILTER_LABEL_MAP = {
    'no_transform':    'No Transform',
    'absolute':        'Absolute Value',
    'derivative':      'Derivative',
    'moving_average':  'Moving Average',
    'moving_rms':      'Moving RMS',
    'moving_variance': 'Moving Variance',
    'scale_offset':    'Scale / Offset'
};

/**
 * 필터 종류에 맞는 파라미터 패널 HTML을 #filter-params-content에 렌더링한다.
 * 각 입력값 변경 시 updateFilterPreview()를 호출하여 미리보기를 실시간 갱신한다.
 * @param {string} filterType - 필터 종류 (HTML data-filter 값)
 */
function renderFilterParams(filterType) {
    const container = document.getElementById('filter-params-content');
    if (!container) return;

    switch (filterType) {
        case 'no_transform':
            container.innerHTML = '<p class="filter-params-placeholder">Removes any applied filter and restores the original raw data stream.</p>';
            break;
        case 'absolute':
            container.innerHTML = '<p class="filter-params-placeholder">No parameters required.</p>';
            break;
        case 'derivative':
            container.innerHTML = `
                <div class="filter-param-group">
                    <label><input type="checkbox" id="fp-use-actual" checked> Use actual dt</label>
                </div>
                <div class="filter-param-group">
                    <label>Custom dt (s):</label>
                    <input type="number" id="fp-custom-dt" value="1.0" step="0.001" min="0.0001">
                </div>`;
            {
                const useActualCb = document.getElementById('fp-use-actual');
                const customDtInput = document.getElementById('fp-custom-dt');
                if (useActualCb && customDtInput) {
                    customDtInput.disabled = useActualCb.checked;
                    useActualCb.onchange = () => {
                        customDtInput.disabled = useActualCb.checked;
                        updateFilterPreview();
                    };
                    customDtInput.oninput = () => updateFilterPreview();
                }
            }
            break;
        case 'moving_average':
            container.innerHTML = `
                <div class="filter-param-group">
                    <label>Samples count:</label>
                    <input type="number" id="fp-samples-count" value="10" step="1" min="1">
                </div>
                <div class="filter-param-group">
                    <label><input type="checkbox" id="fp-compensate-offset"> Compensate offset</label>
                </div>`;
            document.getElementById('fp-samples-count')?.addEventListener('input', updateFilterPreview);
            document.getElementById('fp-compensate-offset')?.addEventListener('change', updateFilterPreview);
            break;
        case 'moving_rms':
            container.innerHTML = `
                <div class="filter-param-group">
                    <label>Samples count:</label>
                    <input type="number" id="fp-samples-count" value="10" step="1" min="1">
                </div>`;
            document.getElementById('fp-samples-count')?.addEventListener('input', updateFilterPreview);
            break;
        case 'moving_variance':
            container.innerHTML = `
                <div class="filter-param-group">
                    <label>Window size:</label>
                    <input type="number" id="fp-window-size" value="10" step="1" min="1">
                </div>
                <div class="filter-param-group">
                    <label><input type="checkbox" id="fp-apply-square-root"> Apply square root (std dev)</label>
                </div>`;
            document.getElementById('fp-window-size')?.addEventListener('input', updateFilterPreview);
            document.getElementById('fp-apply-square-root')?.addEventListener('change', updateFilterPreview);
            break;
        case 'scale_offset':
            container.innerHTML = `
                <div class="filter-param-group">
                    <label>Time offset (s):</label>
                    <input type="number" id="fp-time-offset" value="0" step="0.001">
                </div>
                <div class="filter-param-group">
                    <label>Value offset:</label>
                    <input type="number" id="fp-value-offset" value="0" step="0.001">
                </div>
                <div class="filter-param-group">
                    <label>Value multiplier:</label>
                    <input type="number" id="fp-value-multiplier" value="1" step="0.001">
                </div>
                <div class="filter-param-group filter-conversion-btns">
                    <label>Quick convert:</label>
                    <div class="filter-btn-row">
                        <button id="fp-btn-rad2deg" class="filter-convert-btn" title="Radians → Degrees (×180/π)">Rad→Deg</button>
                        <button id="fp-btn-deg2rad" class="filter-convert-btn" title="Degrees → Radians (×π/180)">Deg→Rad</button>
                    </div>
                </div>`;
            document.getElementById('fp-time-offset')?.addEventListener('input', updateFilterPreview);
            document.getElementById('fp-value-offset')?.addEventListener('input', updateFilterPreview);
            document.getElementById('fp-value-multiplier')?.addEventListener('input', updateFilterPreview);
            document.getElementById('fp-btn-rad2deg')?.addEventListener('click', () => {
                const multiplierInput = document.getElementById('fp-value-multiplier');
                if (multiplierInput) {
                    multiplierInput.value = (180 / Math.PI).toFixed(6);
                    multiplierInput.dispatchEvent(new Event('input'));
                }
            });
            document.getElementById('fp-btn-deg2rad')?.addEventListener('click', () => {
                const multiplierInput = document.getElementById('fp-value-multiplier');
                if (multiplierInput) {
                    multiplierInput.value = (Math.PI / 180).toFixed(6);
                    multiplierInput.dispatchEvent(new Event('input'));
                }
            });
            break;
        default:
            container.innerHTML = '<p class="filter-params-placeholder">Select a filter to configure parameters.</p>';
    }
}

/**
 * 현재 파라미터 패널의 입력값을 읽어 params 객체로 반환한다.
 * @param {string} filterType - 필터 종류 (HTML data-filter 값)
 * @returns {object} 필터 파라미터 객체
 */
function readFilterParams(filterType) {
    switch (filterType) {
        case 'no_transform':
            return {};
        case 'derivative':
            return {
                useActual: document.getElementById('fp-use-actual')?.checked ?? true,
                customDT: parseFloat(document.getElementById('fp-custom-dt')?.value || 1.0)
            };
        case 'moving_average':
            return {
                samplesCount: parseInt(document.getElementById('fp-samples-count')?.value || 10),
                compensateOffset: document.getElementById('fp-compensate-offset')?.checked ?? false
            };
        case 'moving_rms':
            return {
                samplesCount: parseInt(document.getElementById('fp-samples-count')?.value || 10)
            };
        case 'moving_variance':
            return {
                windowSize: parseInt(document.getElementById('fp-window-size')?.value || 10),
                applySquareRoot: document.getElementById('fp-apply-square-root')?.checked ?? false
            };
        case 'scale_offset':
            return {
                timeOffset: parseFloat(document.getElementById('fp-time-offset')?.value || 0),
                valueOffset: parseFloat(document.getElementById('fp-value-offset')?.value || 0),
                valueMultiplier: parseFloat(document.getElementById('fp-value-multiplier')?.value || 1)
            };
        default:
            return {};
    }
}

/**
 * Alias 입력창을 현재 선택된 source trace 이름과 필터 레이블로 자동 갱신한다.
 */
function updateFilterAlias() {
    if (!currentFilterPlotId || currentFilterTraceIndex === null) return;

    const plotManager = plotState.plotTabManager.getPlotManager(currentFilterPlotId);
    if (!plotManager) return;

    const sourceTrace = plotManager.traces[currentFilterTraceIndex];
    const aliasInput = document.getElementById('filter-alias-input');
    if (aliasInput && sourceTrace) {
        // 필터 체인: 항상 원본 topic 이름(bufferKey)을 베이스로 사용
        const baseName = sourceTrace.bufferKey || sourceTrace.name;

        if (currentFilterType === 'no_transform') {
            // No Transform: 원본 이름으로 복원
            aliasInput.value = baseName;
        } else {
            const label = currentFilterType
                ? (FILTER_LABEL_MAP[currentFilterType] || currentFilterType)
                : 'filtered';
            aliasInput.value = `${baseName}[${label}]`;
        }
    }
}

/**
 * #filter-preview-plot Plotly 차트를 현재 필터/파라미터 상태로 갱신한다.
 * source trace 원본(회색)과 필터 결과(빨강)를 함께 표시한다.
 */
function updateFilterPreview() {
    if (!currentFilterPlotId || currentFilterTraceIndex === null || !currentFilterType) return;

    const plotManager = plotState.plotTabManager.getPlotManager(currentFilterPlotId);
    if (!plotManager) return;

    const sourceTrace = plotManager.traces[currentFilterTraceIndex];
    if (!sourceTrace) return;

    // bufferKey: 필터 적용된 trace의 원본 buffer 키
    const bufferKey = sourceTrace.bufferKey || sourceTrace.name;
    const buffer = plotManager.dataBuffers.get(bufferKey);
    if (!buffer || buffer.isEmpty()) return;

    const rawData = buffer.getData();
    const { timestamps, values } = rawData;

    const params = readFilterParams(currentFilterType);
    const mappedType = FILTER_TYPE_MAP[currentFilterType];

    let filteredData;
    try {
        switch (mappedType) {
            case 'noTransform':
                // 필터 없음: 원본 데이터 그대로 표시
                filteredData = { timestamps: [...timestamps], values: [...values] };
                break;
            case 'absolute':
                filteredData = PlotDataFilter.applyAbsolute(timestamps, values);
                break;
            case 'derivative':
                filteredData = PlotDataFilter.applyDerivative(timestamps, values, params);
                break;
            case 'movingAverage':
                filteredData = PlotDataFilter.applyMovingAverage(timestamps, values, params);
                break;
            case 'movingRMS':
                filteredData = PlotDataFilter.applyMovingRMS(timestamps, values, params);
                break;
            case 'movingVariance':
                filteredData = PlotDataFilter.applyMovingVariance(timestamps, values, params);
                break;
            case 'scaleOffset':
                filteredData = PlotDataFilter.applyScaleOffset(timestamps, values, params);
                break;
            default:
                return;
        }
    } catch (err) {
        console.warn('[updateFilterPreview] Filter calculation error:', err);
        return;
    }

    // t0 모드 적용: 상대 시간으로 변환
    let xOrig = timestamps;
    let xFiltered = filteredData.timestamps;
    if (plotManager.t0Mode && plotManager.firstTimestamp !== null) {
        xOrig = xOrig.map(t => t - plotManager.firstTimestamp);
        xFiltered = xFiltered.map(t => t - plotManager.firstTimestamp);
    }

    const previewLayout = {
        height: 200,
        margin: { t: 10, b: 30, l: 50, r: 10 },
        paper_bgcolor: '#1e1e2e',
        plot_bgcolor: '#1e1e2e',
        font: { color: '#cdd6f4', size: 11 },
        showlegend: true,
        legend: { x: 0, y: 1, font: { size: 10 } },
        xaxis: { gridcolor: '#313244', zerolinecolor: '#45475a' },
        yaxis: { gridcolor: '#313244', zerolinecolor: '#45475a' }
    };

    Plotly.react('filter-preview-plot', [
        {
            x: xOrig,
            y: values,
            name: sourceTrace.name,
            type: 'scattergl',
            mode: 'lines',
            line: { color: '#95a5a6', width: 1 },
            opacity: 0.5
        },
        {
            x: xFiltered,
            y: filteredData.values,
            name: FILTER_LABEL_MAP[currentFilterType] || currentFilterType,
            type: 'scattergl',
            mode: 'lines',
            line: { color: '#e74c3c', width: 2 }
        }
    ], previewLayout, { responsive: true, displayModeBar: false });
}

/**
 * Filter Dialog 모달을 열고 초기 상태를 설정한다.
 * plotManager로부터 trace 목록을 읽어 Source Curve 목록을 채우고,
 * 필터 타입 선택 클릭 핸들러를 설정한다.
 * @param {string} plotId      - 대상 Plot ID
 * @param {number} traceIndex  - 기본 선택 trace 인덱스
 */
window.openFilterDialog = function(plotId, traceIndex) {
    console.log('[openFilterDialog] Opening filter dialog for plot:', plotId, 'trace:', traceIndex);

    currentFilterPlotId = plotId;
    currentFilterTraceIndex = traceIndex;
    currentFilterType = null;

    const plotManager = plotState.plotTabManager.getPlotManager(plotId);
    if (!plotManager || !plotManager.isInitialized) {
        console.error('[openFilterDialog] Plot manager not found or not initialized:', plotId);
        return;
    }

    // Source curve 목록 채우기
    const sourceList = document.getElementById('filter-source-items');
    if (sourceList) {
        sourceList.innerHTML = '';
        plotManager.traces.forEach((trace, idx) => {
            const li = document.createElement('li');
            li.textContent = trace.name || `Trace ${idx + 1}`;
            li.className = 'filter-source-item';
            if (idx === traceIndex) {
                li.classList.add('active');
            }
            li.addEventListener('click', () => {
                document.querySelectorAll('#filter-source-items .filter-source-item').forEach(el => el.classList.remove('active'));
                li.classList.add('active');
                currentFilterTraceIndex = idx;
                updateFilterAlias();
                updateFilterPreview();
            });
            sourceList.appendChild(li);
        });
    }

    // Filter type 항목 클릭 핸들러 설정 (기존 active 초기화)
    document.querySelectorAll('#filter-type-items .filter-type-item').forEach(item => {
        item.classList.remove('active');
        item.onclick = () => {
            document.querySelectorAll('#filter-type-items .filter-type-item').forEach(el => el.classList.remove('active'));
            item.classList.add('active');
            currentFilterType = item.dataset.filter;
            renderFilterParams(currentFilterType);
            updateFilterAlias();
            updateFilterPreview();
        };
    });

    // 파라미터 패널 초기화
    const paramsContent = document.getElementById('filter-params-content');
    if (paramsContent) {
        paramsContent.innerHTML = '<p class="filter-params-placeholder">Select a filter to configure parameters.</p>';
    }

    // Alias 초기화
    const sourceTrace = plotManager.traces[traceIndex];
    const aliasInput = document.getElementById('filter-alias-input');
    if (aliasInput) {
        const baseName = sourceTrace ? (sourceTrace.bufferKey || sourceTrace.name) : '';
        aliasInput.value = baseName ? `${baseName}[filtered]` : '';
    }

    // 미리보기 플롯 초기화 (원본 trace만 표시)
    const previewDiv = document.getElementById('filter-preview-plot');
    if (previewDiv && sourceTrace) {
        // bufferKey: 필터 적용된 trace의 원본 buffer 키
        const bufferKey = sourceTrace.bufferKey || sourceTrace.name;
        const buffer = plotManager.dataBuffers.get(bufferKey);
        if (buffer && !buffer.isEmpty()) {
            const rawData = buffer.getData();
            let xData = rawData.timestamps;
            if (plotManager.t0Mode && plotManager.firstTimestamp !== null) {
                xData = xData.map(t => t - plotManager.firstTimestamp);
            }
            const initLayout = {
                height: 200,
                margin: { t: 10, b: 30, l: 50, r: 10 },
                paper_bgcolor: '#1e1e2e',
                plot_bgcolor: '#1e1e2e',
                font: { color: '#cdd6f4', size: 11 },
                showlegend: true,
                legend: { x: 0, y: 1, font: { size: 10 } },
                xaxis: { gridcolor: '#313244', zerolinecolor: '#45475a' },
                yaxis: { gridcolor: '#313244', zerolinecolor: '#45475a' }
            };
            Plotly.react('filter-preview-plot', [{
                x: xData,
                y: rawData.values,
                name: sourceTrace.name,
                type: 'scattergl',
                mode: 'lines',
                line: { color: '#95a5a6', width: 1 }
            }], initLayout, { responsive: true, displayModeBar: false });
        } else {
            // 데이터 없으면 빈 차트 표시
            Plotly.react('filter-preview-plot', [], {
                height: 200,
                margin: { t: 10, b: 30, l: 50, r: 10 },
                paper_bgcolor: '#1e1e2e',
                plot_bgcolor: '#1e1e2e',
                font: { color: '#cdd6f4', size: 11 },
                annotations: [{ text: 'No data', x: 0.5, y: 0.5, xref: 'paper', yref: 'paper', showarrow: false, font: { color: '#6c7086' } }]
            }, { responsive: true, displayModeBar: false });
        }
    }

    // 모달 표시
    const modal = document.getElementById('filter-dialog-modal');
    if (modal) {
        modal.style.display = 'flex';
    }
};

/**
 * Filter Dialog 모달을 닫고 상태 변수를 초기화한다.
 */
window.closeFilterDialog = function() {
    console.log('[closeFilterDialog] Closing filter dialog');

    const modal = document.getElementById('filter-dialog-modal');
    if (modal) {
        modal.style.display = 'none';
    }

    currentFilterPlotId = null;
    currentFilterTraceIndex = null;
    currentFilterType = null;
};

/**
 * 현재 선택된 필터를 대상 Plot에 적용하고 다이얼로그를 닫는다.
 * PlotlyPlotManager.applyFilter()를 호출하여 isFiltered=true 정적 trace를 생성한다.
 * Auto Zoom이 체크된 경우 적용 후 Plot 축을 자동 맞춤한다.
 */
window.saveFilter = function() {
    console.log('[saveFilter] Saving filter');

    if (!currentFilterPlotId || currentFilterTraceIndex === null) {
        console.error('[saveFilter] No plot/trace selected');
        return;
    }

    if (!currentFilterType) {
        alert('Please select a filter type.');
        return;
    }

    const plotManager = plotState.plotTabManager.getPlotManager(currentFilterPlotId);
    if (!plotManager || !plotManager.isInitialized) {
        console.error('[saveFilter] Plot manager not found or not initialized');
        return;
    }

    const alias = document.getElementById('filter-alias-input')?.value?.trim() || '';
    const autoZoom = document.getElementById('filter-autozoom')?.checked ?? true;
    const params = readFilterParams(currentFilterType);
    const mappedType = FILTER_TYPE_MAP[currentFilterType];

    const success = plotManager.applyFilter(currentFilterTraceIndex, mappedType, params, alias);

    if (success) {
        console.log('[saveFilter] ✓ Filter applied successfully');

        // Auto Zoom: 적용 후 축을 자동 맞춤
        if (autoZoom) {
            try {
                Plotly.relayout(plotManager.containerId, {
                    'xaxis.autorange': true,
                    'yaxis.autorange': true
                });
            } catch (err) {
                console.warn('[saveFilter] Auto zoom failed:', err);
            }
        }

        window.closeFilterDialog();
    } else {
        console.error('[saveFilter] Failed to apply filter');
        alert('Failed to apply filter. Make sure the trace has data.');
    }
};

// Filter 다이얼로그 모달 외부 클릭 시 닫기
window.addEventListener('click', (event) => {
    const filterModal = document.getElementById('filter-dialog-modal');
    if (event.target === filterModal) {
        window.closeFilterDialog();
    }
});

/**
 * Plot 탭 왼쪽 토픽 목록 패널 접기/펼치기 (Views 패널과 동일한 화살표 UX)
 */
function togglePlotDisplayPanel() {
    const panel = document.getElementById('plot-display-panel');
    const container = document.getElementById('plot-container');
    if (!panel || !container) return;
    const isCollapsed = panel.classList.toggle('collapsed');
    container.style.gridTemplateColumns = isCollapsed ? '28px 1fr' : '300px 1fr';
    const btn = document.getElementById('plot-display-collapse-btn');
    if (btn) btn.textContent = isCollapsed ? '◀' : '▶';
    // 그리드 transition(0.2s) 이후 Plotly가 실제 너비를 반영하도록 리사이즈
    setTimeout(resizeVisiblePlotlyPlots, 230);
}

window.togglePlotDisplayPanel = togglePlotDisplayPanel;

// ==============================================================
// LocalizationLiveViewer - Localization 실시간 3D 뷰어
// ==============================================================
class LocalizationLiveViewer {
    constructor() {
        this._scene = null;
        this._camera = null;
        this._perspCamera = null;
        this._orthoCamera = null;
        this._renderer = null;
        this._controls = null;
        this._animFrameId = null;
        this._initialized = false;
        this._visible = false;
        this._ros = null;
        this._subscriptions = [];
        this._cloudObj = null;
        this._mapObj = null;
        // Path: Slam Live와 동일 — Line2 + grow 버퍼 (/Odometry 샘플링)
        this._pathLineState = _createPathLineState();
        this._pathObj = null;
        this._pathLineRes = null;
        this._tfObjects = {};
        this._robotOdomGroup = null; // THREE.Group — robot axes from /Odometry
        this._odomTopic = '/Odometry';
        this._robotPos = null;   // THREE.Vector3 — updated from /Odometry
        this._followMode = true; // camera follow toggle
        this._lastRobotAxisScale = undefined;
        this._mapMesh = null;
        this._mapTexture = null;
        // 레이어 가시성은 메시지 수신 시 자동으로 활성화 (수동 체크박스 제거)
        this._fixedFrame = 'odom';
        this._topView = false;
        this._savedCameraPos = null;
        this._savedCameraUp = null;
        this._savedTarget = null;
        this._knownFrames = new Set();
        this._resizeObserver = null;
        this._backendSubscribed = false;
        this._wsConnectGen = 0;
        this._rosbridgeSubscribed = false;
        this._cloudBusy = false;
        this._cloudLastMs = 0;
        this._laserMapLastMs = 0;
        this._cloudBusyWatchdog = null;
        this._perspPointSizesDirty = false;
        this._lastOrthoScale = undefined;
        this._pathPoseBuffer = { xyz: null, count: 0, lastMsgCount: 0 };
        this._pathRebuildTimer = null;
        this._lastPathPose = null; // {x,y,z} — /Odometry path 샘플링용
    }

    _waitForThree() {
        return new Promise((resolve) => {
            const check = () => {
                if (window.THREE && window.OrbitControls) {
                    resolve();
                } else {
                    setTimeout(check, 100);
                }
            };
            check();
        });
    }

    async _init() {
        if (this._initialized) return;
        await this._waitForThree();
        const THREE = window.THREE;
        const canvas = document.getElementById('loc-viewer-canvas');
        if (!canvas) return;

        const container = document.getElementById('loc-viewer-canvas-container');
        const w = container.clientWidth || 600;
        const h = container.clientHeight || 480;

        this._renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
        this._renderer.setPixelRatio(window.devicePixelRatio);
        this._renderer.setSize(w, h);

        this._scene = new THREE.Scene();
        this._scene.background = new THREE.Color(0x0a0a18);

        this._camera = new THREE.PerspectiveCamera(60, w / h, 0.1, 10000);
        this._camera.position.set(0, 0, 20);
        this._camera.up.set(0, 0, 1);
        this._perspCamera = this._camera;

        this._controls = new window.OrbitControls(this._camera, this._renderer.domElement);
        this._controls.enableDamping = true;
        this._controls.dampingFactor = 0.1;

        this._scene.add(new THREE.AxesHelper(3));

        this._resizeObserver = new ResizeObserver(() => this._resizeRenderer());
        this._resizeObserver.observe(container);

        this._initialized = true;
        this._startRenderLoop();
    }

    _startRenderLoop() {
        const LERP = 0.08;
        const animate = () => {
            this._animFrameId = requestAnimationFrame(animate);

            // Camera follow: pan camera+target together toward robot (preserves orbit angle)
            if (this._followMode && this._robotPos && this._controls) {
                const rp = this._robotPos;
                if (this._topView && this._orthoCamera) {
                    // TopView: move orthoCamera + target together
                    const dx = (rp.x - this._orthoCamera.position.x) * LERP;
                    const dy = (rp.y - this._orthoCamera.position.y) * LERP;
                    this._orthoCamera.position.x += dx;
                    this._orthoCamera.position.y += dy;
                    this._controls.target.x += dx;
                    this._controls.target.y += dy;
                    this._orthoCamera.lookAt(
                        this._orthoCamera.position.x,
                        this._orthoCamera.position.y, 0
                    );
                } else {
                    // Orbit: move camera + target by same delta (preserves angle/distance)
                    const dx = (rp.x - this._controls.target.x) * LERP;
                    const dy = (rp.y - this._controls.target.y) * LERP;
                    const dz = (rp.z - this._controls.target.z) * LERP;
                    this._controls.target.x += dx;
                    this._controls.target.y += dy;
                    this._controls.target.z += dz;
                    this._perspCamera.position.x += dx;
                    this._perspCamera.position.y += dy;
                    this._perspCamera.position.z += dz;
                }
            }

            if (this._controls) this._controls.update();
            // OrthographicCamera 탑뷰 시: zoom 변화에 따라 포인트 픽셀 크기 갱신
            // Perspective: dirty일 때만 복원 (매 프레임 needsUpdate 금지)
            if (this._topView && this._orthoCamera) {
                this._updateOrthoPointSizes();
            } else if (this._perspPointSizesDirty) {
                this._ensurePerspectivePointSizes();
                this._perspPointSizesDirty = false;
            }
            // 로봇 AXIS: Slam Live와 동일 줌 스케일
            this._updateRobotAxisZoomScale();
            if (this._renderer && this._scene && this._camera) {
                this._renderer.render(this._scene, this._camera);
            }
        };
        animate();
    }

    _pathLineResolutionVec() {
        const THREE = window.THREE;
        if (!THREE) return null;
        const container = document.getElementById('loc-viewer-canvas-container');
        const w = container ? (container.clientWidth || 600) : 600;
        const h = container ? (container.clientHeight || 380) : 380;
        if (!this._pathLineRes) this._pathLineRes = new THREE.Vector2(w, h);
        else this._pathLineRes.set(w, h);
        return this._pathLineRes;
    }

    _updatePathLineResolutions(w, h) {
        const mat = this._pathLineState && this._pathLineState.line && this._pathLineState.line.material;
        if (mat && mat.resolution) mat.resolution.set(w, h);
        _setGroupLineResolutions(this._robotOdomGroup, w, h);
    }

    /** 로봇 AXIS: Slam Live와 동일 — 줌아웃 약한 확대, 상한 4 */
    _computeRobotAxisScale() {
        if (this._topView && this._orthoCamera) {
            const ppu = this._getOrthoPixelsPerUnit();
            const desiredLen = 28 / Math.max(ppu, 0.01);
            return Math.max(0.5, Math.min(desiredLen / LIVE_ROBOT_AXIS_LENGTH, 4));
        }
        const cam = this._perspCamera || this._camera;
        if (!cam || !this._controls) return 1;
        const dist = cam.position.distanceTo(this._controls.target);
        return Math.max(0.5, Math.min(dist / 40, 4));
    }

    _updateRobotAxisZoomScale() {
        if (!this._robotOdomGroup) return;
        const s = this._computeRobotAxisScale();
        if (this._lastRobotAxisScale !== undefined && Math.abs(this._lastRobotAxisScale - s) < 0.03) {
            return;
        }
        this._lastRobotAxisScale = s;
        this._robotOdomGroup.scale.setScalar(s);
    }

    /**
     * OrthographicCamera에서 1 월드단위 = 몇 픽셀인지 계산 (3D Viewer 방식 동일)
     * OrthographicCamera는 sizeAttenuation 미적용 → material.size를 픽셀 단위로 직접 제어해야 함
     */
    _getOrthoPixelsPerUnit() {
        if (!this._orthoCamera || !this._renderer) return 40;
        const frustumH = (this._orthoCamera.top - this._orthoCamera.bottom) / (this._orthoCamera.zoom || 1);
        const pixelH   = this._renderer.domElement.height || 480;
        return pixelH / frustumH;
    }

    /**
     * 탑뷰(OrthographicCamera) 시 모든 포인트 클라우드 material.size를
     * frustum 스케일 기준으로 갱신 (zoom 변화 반영)
     */
    _updateOrthoPointSizes() {
        const scale = this._getOrthoPixelsPerUnit();
        if (this._lastOrthoScale !== undefined && Math.abs(this._lastOrthoScale - scale) < 0.05) {
            return;
        }
        this._lastOrthoScale = scale;
        const update = (obj) => {
            if (!obj || !obj.material) return;
            if (!('size' in obj.material)) return;
            // sizeAttenuation 변경 시에만 needsUpdate (매 프레임 셰이더 재컴파일 방지)
            if (obj.material.sizeAttenuation !== false) {
                obj.material.sizeAttenuation = false;
                obj.material.needsUpdate = true;
            }
            const baseSize = obj.material._baseSize || 0.1;
            obj.material.size = Math.max(1, baseSize * scale);
        };
        update(this._cloudObj);
        update(this._mapObj);
    }

    /** Orbit(Perspective) 뷰: sizeAttenuation·월드 크기 복원 (탑뷰 전환 후 잔류 방지) */
    _ensurePerspectivePointSizes() {
        const fix = (obj) => {
            if (!obj || !obj.material || obj.material._baseSize === undefined) return;
            if (!('size' in obj.material)) return;
            let attenuationChanged = false;
            if (obj.material.sizeAttenuation !== true) {
                obj.material.sizeAttenuation = true;
                attenuationChanged = true;
            }
            if (obj.material.size !== obj.material._baseSize) {
                obj.material.size = obj.material._baseSize;
            }
            // sizeAttenuation 토글만 셰이더 define 변경 → needsUpdate
            if (attenuationChanged) {
                obj.material.needsUpdate = true;
            }
        };
        fix(this._cloudObj);
        fix(this._mapObj);
    }

    _resizeRenderer() {
        if (!this._renderer) return;
        const container = document.getElementById('loc-viewer-canvas-container');
        if (!container) return;
        const w = container.clientWidth;
        const h = container.clientHeight;
        if (w > 0 && h > 0) {
            this._renderer.setSize(w, h);
            this._updatePathLineResolutions(w, h);
            const aspect = w / h;
            if (this._topView && this._orthoCamera) {
                const halfH = this._orthoCamera.top;
                this._orthoCamera.left   = -halfH * aspect;
                this._orthoCamera.right  =  halfH * aspect;
                this._orthoCamera.updateProjectionMatrix();
            } else if (this._perspCamera) {
                this._perspCamera.aspect = aspect;
                this._perspCamera.updateProjectionMatrix();
            }
        }
    }

    async show() {
        const viewerEl = document.getElementById('localization-live-viewer');
        if (viewerEl) viewerEl.style.display = 'block';
        this._visible = true;
        await this._init();
        await new Promise(resolve => requestAnimationFrame(resolve));
        this._resizeRenderer();
        this.toggleTopView(true);
        await this._connectAndSubscribe();
    }

    hide() {
        this._visible = false;
        this._wsConnectGen++;
        this._backendSubscribed = false;
        this._rosbridgeSubscribed = false;
        this._cloudBusy = false;
        this._cloudLastMs = 0;
        this._laserMapLastMs = 0;
        if (this._cloudBusyWatchdog) {
            clearTimeout(this._cloudBusyWatchdog);
            this._cloudBusyWatchdog = null;
        }
        this._unsubscribeAll();
        this._clearLiveObjects();
        const viewerEl = document.getElementById('localization-live-viewer');
        if (viewerEl) viewerEl.style.display = 'none';
        const topViewToggle = document.getElementById('loc-viewer-topview-toggle');
        if (topViewToggle) topViewToggle.checked = false;
        this._topView = false;
        this._savedCameraPos = null;
        this._knownFrames.clear();
    }

    _clearLiveObjects() {
        const THREE = window.THREE;
        if (!this._scene || !THREE) return;

        const removeObj = (obj) => {
            if (!obj) return;
            this._scene.remove(obj);
            if (obj.geometry) obj.geometry.dispose();
            if (obj.material) {
                if (obj.material.map) obj.material.map.dispose();
                obj.material.dispose();
            }
        };

        removeObj(this._cloudObj);
        removeObj(this._mapObj);
        _disposePathLineState(this._scene, this._pathLineState);
        this._cloudObj = null;
        this._mapObj = null;
        this._pathObj = null;
        _clearPathBufferState(this, '_pathPoseBuffer', '_pathRebuildTimer');
        this._lastPathPose = null;

        for (const key of Object.keys(this._tfObjects)) {
            const entry = this._tfObjects[key];
            if (entry && entry.group) this._scene.remove(entry.group);
        }
        this._tfObjects = {};

        if (this._robotOdomGroup) {
            _disposeRobotAxesGroup(this._scene, this._robotOdomGroup);
            this._robotOdomGroup = null;
            this._lastRobotAxisScale = undefined;
        }

        if (this._mapMesh) {
            this._scene.remove(this._mapMesh);
            if (this._mapMesh.geometry) this._mapMesh.geometry.dispose();
            if (this._mapMesh.material) this._mapMesh.material.dispose();
            this._mapMesh = null;
        }
        if (this._mapTexture) {
            this._mapTexture.dispose();
            this._mapTexture = null;
        }
    }

    async _connectAndSubscribe() {
        if (this._backendSubscribed) return;

        const loadingEl = document.getElementById('loc-viewer-loading');
        if (loadingEl) {
            loadingEl.textContent = '백엔드 WebSocket 연결 중...';
            loadingEl.style.display = 'block';
        }
        if (typeof ensureWebuiPortsReady === 'function') {
            await ensureWebuiPortsReady();
        }

        // PC2는 Python 백엔드 WS(8881) — path는 /Odometry(rosbridge)에서 샘플링
        console.log('[LocalizationLiveViewer] Subscribing binary topics (backend WS 8881)');
        this._subscribeBinaryTopics();
        this._backendSubscribed = true;
        if (loadingEl) loadingEl.style.display = 'none';

        const onRosbridgeReady = () => {
            this._subscribeRosbridgeTopics();
        };

        if (window.plotState && plotState.ros && plotState.ros.isConnected) {
            _verifyRosbridgeAlive(plotState.ros, 2500).then((alive) => {
                if (alive) {
                    this._ros = plotState.ros;
                    onRosbridgeReady();
                } else {
                    console.warn('[LocalizationLiveViewer] plotState.ros stale — own rosbridge connection');
                    this._connectOwnRosbridge(onRosbridgeReady);
                }
            });
            return;
        }

        this._connectOwnRosbridge(onRosbridgeReady);
    }

    _connectOwnRosbridge(onRosbridgeReady) {
        try {
            const url = _getRosbridgeUrl();
            console.log('[LocalizationLiveViewer] rosbridge connecting (TF/map):', url);
            this._ros = new ROSLIB.Ros({ url });
            this._ros.on('connection', () => {
                console.log('[LocalizationLiveViewer] rosbridge connected:', url);
                onRosbridgeReady();
            });
            this._ros.on('error', (err) => {
                console.error('[LocalizationLiveViewer] rosbridge error:', url, err);
            });
            this._ros.on('close', () => {
                console.warn('[LocalizationLiveViewer] rosbridge connection closed:', url);
            });
        } catch (e) {
            console.error('[LocalizationLiveViewer] failed to init rosbridge:', e);
        }
    }

    _subscribeBinaryTopics() {
        this._subscribePointCloud('/cloud_registered', 'cloud_registered');
        this._subscribePointCloudLatched('/Laser_map', 'laser_map');
        // path는 /Odometry 위치 샘플링 (_subscribeOdometry) — /path 바이너리 구독 없음
    }

    _subscribeRosbridgeTopics() {
        if (!this._ros || this._rosbridgeSubscribed) return;
        this._rosbridgeSubscribed = true;
        this._subscribeOdometry(this._odomTopic);
        this._subscribeTF('/tf');
        this._subscribeMap('/map');
    }

    _subscribeAll() {
        this._subscribeBinaryTopics();
        this._subscribeRosbridgeTopics();
    }

    _unsubscribeAll() {
        for (const t of this._subscriptions) {
            try { t.unsubscribe(); } catch (e) { /* ignore */ }
        }
        this._subscriptions = [];
        this._rosbridgeSubscribed = false;
    }

    _parsePC2(msg) {
        const MAX_PTS = 80000;
        let binary;
        try {
            const raw = atob(msg.data);
            binary = new Uint8Array(raw.length);
            for (let i = 0; i < raw.length; i++) {
                binary[i] = raw.charCodeAt(i);
            }
        } catch (e) {
            console.error('[LocalizationLiveViewer] PC2 decode error:', e);
            return null;
        }
        const view = new DataView(binary.buffer);
        const pointStep = msg.point_step;
        const totalPts = msg.width * msg.height;
        const fields = {};
        for (const f of msg.fields) {
            fields[f.name] = f.offset;
        }
        const xOff = fields['x'] !== undefined ? fields['x'] : 0;
        const yOff = fields['y'] !== undefined ? fields['y'] : 4;
        const zOff = fields['z'] !== undefined ? fields['z'] : 8;

        const step = Math.max(1, Math.floor(totalPts / MAX_PTS));
        const outPts = Math.ceil(totalPts / step);
        const positions = new Float32Array(outPts * 3);
        const colors = new Float32Array(outPts * 3);
        const tempZ = new Float32Array(outPts);
        let minZ = Infinity;
        let maxZ = -Infinity;
        let idx = 0;

        for (let i = 0; i < totalPts; i += step) {
            const base = i * pointStep;
            if (base + zOff + 4 > binary.length) break;
            const x = view.getFloat32(base + xOff, true);
            const y = view.getFloat32(base + yOff, true);
            const z = view.getFloat32(base + zOff, true);
            if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
            positions[idx * 3]     = x;
            positions[idx * 3 + 1] = y;
            positions[idx * 3 + 2] = z;
            tempZ[idx] = z;
            if (z < minZ) minZ = z;
            if (z > maxZ) maxZ = z;
            idx++;
        }

        const range = (maxZ - minZ) || 1;
        for (let i = 0; i < idx; i++) {
            const t = (tempZ[i] - minZ) / range;
            const [r, g, b] = this._rainbowColor(t);
            colors[i * 3]     = r;
            colors[i * 3 + 1] = g;
            colors[i * 3 + 2] = b;
        }

        return {
            positions: positions.subarray(0, idx * 3),
            colors: colors.subarray(0, idx * 3)
        };
    }

    _rainbowColor(t) {
        const h = (1 - t) * 240;
        const c = 1;
        const x = c * (1 - Math.abs(((h / 60) % 2) - 1));
        let r = 0, g = 0, b = 0;
        if      (h < 60)  { r = c; g = x; b = 0; }
        else if (h < 120) { r = x; g = c; b = 0; }
        else if (h < 180) { r = 0; g = c; b = x; }
        else if (h < 240) { r = 0; g = x; b = c; }
        else if (h < 300) { r = x; g = 0; b = c; }
        else              { r = c; g = 0; b = x; }
        return [r, g, b];
    }

    _updatePointCloud(key, parsed) {
        const THREE = window.THREE;
        if (!this._scene || !THREE || !parsed) return;

        const newCount = parsed.positions.length / 3;
        const existing = (key === 'cloud_registered') ? this._cloudObj : this._mapObj;
        // Slam Live와 동일 계열: 현재 스캔 흰색·불투명·크게, 맵은 약간 투명
        const pointSize = (key === 'laser_map') ? 0.14 : 0.22;
        const opacity   = (key === 'laser_map') ? 0.55 : 1.0;
        const transparent = (key === 'laser_map');

        if (existing && existing.geometry) {
            const posAttr = existing.geometry.getAttribute('position');
            const colAttr = existing.geometry.getAttribute('color');
            if (posAttr && posAttr.array.length >= parsed.positions.length) {
                posAttr.array.set(parsed.positions);
                posAttr.needsUpdate = true;
                colAttr.array.set(parsed.colors);
                colAttr.needsUpdate = true;
                _syncPointsGeometry(existing.geometry, newCount, { skipBoundingSphere: true });
                if (existing.material) {
                    existing.material._baseSize = pointSize;
                    existing.material.opacity = opacity;
                    existing.material.transparent = transparent;
                    existing.material.depthWrite = !transparent;
                    if (this._topView && this._orthoCamera) {
                        const scale = this._getOrthoPixelsPerUnit();
                        existing.material.size = Math.max(1, pointSize * scale);
                    } else {
                        existing.material.size = pointSize;
                    }
                }
                return;
            }
            this._scene.remove(existing);
            existing.geometry.dispose();
            existing.material.dispose();
        }

        const MAX_PTS = 80000;
        const posArray = new Float32Array(MAX_PTS * 3);
        const colArray = new Float32Array(MAX_PTS * 3);
        posArray.set(parsed.positions);
        colArray.set(parsed.colors);

        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.BufferAttribute(posArray, 3));
        geo.setAttribute('color', new THREE.BufferAttribute(colArray, 3));
        _syncPointsGeometry(geo, newCount, { skipBoundingSphere: true });

        const mat = new THREE.PointsMaterial({
            size: pointSize,
            sizeAttenuation: !this._topView,
            vertexColors: true,
            transparent,
            opacity,
            depthWrite: !transparent
        });
        mat._baseSize = pointSize; // 탑뷰 ortho 보정용 기준 크기 저장
        // 탑뷰 활성 상태에서 새 mesh가 생성되는 경우 즉시 크기 보정
        if (this._topView && this._orthoCamera) {
            const scale = this._getOrthoPixelsPerUnit();
            mat.size = Math.max(1, pointSize * scale);
        }
        const points = new THREE.Points(geo, mat);
        points.visible = true;
        points.frustumCulled = false;

        if (key === 'cloud_registered') {
            // 스캔 클라우드는 맵 클라우드와 공간적으로 겹침 → z-파이팅 방지
            // renderOrder=2로 맵보다 나중에 그려 depth 우선권 부여 (Slam Live와 동일)
            points.renderOrder = 2;
            this._cloudObj = points;
        } else {
            points.renderOrder = 1;
            this._mapObj = points;
        }
        this._scene.add(points);
    }

    _subscribePointCloud(topic, key) {
        const viewer = this;
        const sub = _createLiveViewerBackendWs(
            viewer,
            'LocalizationLiveViewer',
            (ws) => { ws.send(JSON.stringify({ cmd: 'subscribe', topic })); },
            (buffer) => {
                if (key === 'cloud_registered') {
                    const now = performance.now();
                    if (viewer._cloudBusy) return;
                    if (now - (viewer._cloudLastMs || 0) < LIVE_CLOUD_UPDATE_MS) return;
                    viewer._cloudBusy = true;
                    viewer._cloudLastMs = now;
                    if (viewer._cloudBusyWatchdog) clearTimeout(viewer._cloudBusyWatchdog);
                    viewer._cloudBusyWatchdog = setTimeout(() => {
                        viewer._cloudBusy = false;
                        viewer._cloudBusyWatchdog = null;
                    }, 3000);
                    try {
                        let parsed = viewer._parseBinaryPC2(buffer, {
                            maxPts: 40000,
                            fillWhite: true
                        });
                        if (!parsed) return;
                        parsed = _voxelDownsample(parsed.positions, parsed.colors, 0.5, { whiteOutput: true });
                        viewer._updatePointCloud(key, parsed);
                    } finally {
                        viewer._cloudBusy = false;
                        if (viewer._cloudBusyWatchdog) {
                            clearTimeout(viewer._cloudBusyWatchdog);
                            viewer._cloudBusyWatchdog = null;
                        }
                    }
                    return;
                }
                let parsed = viewer._parseBinaryPC2(buffer, { maxPts: 80000 });
                if (!parsed) return;
                viewer._updatePointCloud(key, parsed);
            }
        );
        this._subscriptions.push(sub);
    }

    // /Laser_map 은 TRANSIENT_LOCAL + RELIABLE QoS 로 발행됨
    // rosbridge는 volatile QoS 구독이라 latched 메시지를 받지 못함
    // → Python 백엔드(8081)에 subscribe_latched 명령으로 직접 연결
    _subscribePointCloudLatched(topic, key) {
        const viewer = this;
        const sub = _createLiveViewerBackendWs(
            viewer,
            'LocalizationLiveViewer',
            (ws) => {
                ws.send(JSON.stringify({ cmd: 'subscribe_latched', topic }));
                console.log(`[LocalizationLiveViewer] subscribe_latched 전송: ${topic}`);
            },
            (buffer) => {
                if (key === 'laser_map') {
                    const now = performance.now();
                    if (now - (viewer._laserMapLastMs || 0) < LIVE_LASER_MAP_UPDATE_MS) return;
                    viewer._laserMapLastMs = now;
                }
                let parsed = viewer._parseBinaryPC2(buffer);
                if (!parsed) return;
                if (key === 'laser_map') {
                    parsed = _voxelDownsample(parsed.positions, parsed.colors, 1.0);
                }
                viewer._updatePointCloud(key, parsed);
            }
        );
        this._subscriptions.push(sub);
    }

    // Python 백엔드 binary PC2 패킷 파싱
    // 패킷 포맷: [3B]'PC2' [1B]version [1B]flags [4B]topicLen [4B]frameLen [4B]count
    //            [topicLen]topic [frameLen]frameId [count*12]XYZ [count*4]colorF32 ([count*4]rgb)
    // options: { maxPts, fillWhite } — fillWhite면 rainbow 생략(흰색 스캔용)
    _parseBinaryPC2(buffer, options = {}) {
        try {
            const view = new DataView(buffer);
            if (view.getUint8(0) !== 0x50 || view.getUint8(1) !== 0x43 || view.getUint8(2) !== 0x32) return null;
            let off = 3;
            /* version = */ view.getUint8(off++);
            const flags    = view.getUint8(off++);
            const topicLen = view.getUint32(off, true); off += 4;
            const frameLen = view.getUint32(off, true); off += 4;
            const count    = view.getUint32(off, true); off += 4;
            off += topicLen + frameLen; // skip names
            if (count === 0) return null;

            const MAX_PTS = options.maxPts || 80000;
            const fillWhite = !!options.fillWhite;
            const step    = Math.max(1, Math.floor(count / MAX_PTS));
            const outPts  = Math.ceil(count / step);
            const positions = new Float32Array(outPts * 3);
            const colors    = new Float32Array(outPts * 3);
            const tempZ     = fillWhite ? null : new Float32Array(outPts);
            let minZ = Infinity, maxZ = -Infinity, idx = 0;

            const xyzBase = off;
            for (let i = 0; i < count; i += step) {
                const b = xyzBase + i * 12;
                if (b + 12 > buffer.byteLength) break;
                const x = view.getFloat32(b,     true);
                const y = view.getFloat32(b + 4, true);
                const z = view.getFloat32(b + 8, true);
                if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
                positions[idx * 3]     = x;
                positions[idx * 3 + 1] = y;
                positions[idx * 3 + 2] = z;
                if (fillWhite) {
                    colors[idx * 3] = 1.0;
                    colors[idx * 3 + 1] = 1.0;
                    colors[idx * 3 + 2] = 1.0;
                } else {
                    tempZ[idx] = z;
                    if (z < minZ) minZ = z;
                    if (z > maxZ) maxZ = z;
                }
                idx++;
            }

            if (!fillWhite) {
                const range = (maxZ - minZ) || 1;
                for (let i = 0; i < idx; i++) {
                    const t = (tempZ[i] - minZ) / range;
                    const [r, g, b] = this._rainbowColor(t);
                    colors[i * 3]     = r;
                    colors[i * 3 + 1] = g;
                    colors[i * 3 + 2] = b;
                }
            }
            return {
                positions: positions.subarray(0, idx * 3),
                colors:    colors.subarray(0, idx * 3)
            };
        } catch (e) {
            console.error('[LocalizationLiveViewer] binary PC2 parse error:', e);
            return null;
        }
    }

    _subscribePath(topic) {
        const t = new ROSLIB.Topic({
            ros: this._ros,
            name: topic,
            messageType: getMsgType('nav_msgs/Path', 'nav_msgs/msg/Path'),
            throttle_rate: 200,
            queue_length: 1
        });
        t.subscribe((msg) => {
            const THREE = window.THREE;
            if (!this._scene || !THREE) return;

            const poses = msg.poses || [];
            if (poses.length < 2) return;

            const xyz = new Float32Array(poses.length * 3);
            for (let i = 0; i < poses.length; i++) {
                const p = poses[i].pose.position;
                xyz[i * 3] = p.x;
                xyz[i * 3 + 1] = p.y;
                xyz[i * 3 + 2] = p.z;
            }
            _mergePathSnapshot(this._pathPoseBuffer, xyz, poses.length, LIVE_PATH_MAX_POSES);
            this._syncPathLineFromBuffer();
        });
        this._subscriptions.push(t);
    }

    // ── Path 바이너리 WS 구독 ────────────────────────────────────────────────

    _parseBinaryPath(buffer) {
        try {
            const view = new DataView(buffer);
            if (view.getUint8(0) !== 0x50 || view.getUint8(1) !== 0x54 || view.getUint8(2) !== 0x48) return null;
            let off = 3;
            view.getUint8(off++); // version
            const topicLen = view.getUint32(off, true); off += 4;
            const frameLen = view.getUint32(off, true); off += 4;
            const count    = view.getUint32(off, true); off += 4;
            const dec = new TextDecoder();
            const topic = dec.decode(new Uint8Array(buffer, off, topicLen)); off += topicLen;
            off += frameLen;
            if (count === 0) return null;
            const xyzBuf = new Float32Array(buffer.slice(off), 0, count * 3);
            return { topic, count, xyz: xyzBuf };
        } catch (e) {
            console.error('[LocalizationLiveViewer] binary Path parse error:', e);
            return null;
        }
    }

    _subscribePathBinary(topic) {
        const viewer = this;
        const sub = _createLiveViewerBackendWs(
            viewer,
            'LocalizationLiveViewer',
            (ws) => { ws.send(JSON.stringify({ cmd: 'subscribe_path', topic })); },
            (buffer) => {
                const parsed = viewer._parseBinaryPath(buffer);
                if (parsed && viewer._scene) viewer._updatePathIncremental(parsed);
            }
        );
        this._subscriptions.push(sub);
    }

    _updatePathIncremental(parsed) {
        const THREE = window.THREE;
        if (!this._scene || !THREE) return;

        const count = parsed.count;
        if (count < 1) return;

        _mergePathSnapshot(this._pathPoseBuffer, parsed.xyz, count, LIVE_PATH_MAX_POSES);
        this._syncPathLineFromBuffer();
    }

    /** Slam Live와 동일: Path 버퍼 → Line2 (TubeGeometry dispose/new 제거) */
    _syncPathLineFromBuffer() {
        const THREE = window.THREE;
        if (!this._scene || !THREE) return;

        const count = this._pathPoseBuffer.count | 0;
        if (count < 2) {
            if (this._pathLineState && this._pathLineState.line) {
                this._pathLineState.line.visible = false;
            }
            this._pathObj = null;
            return;
        }

        this._pathObj = _syncPathLineFromBuffer(
            THREE, this._scene, this._pathLineState, this._pathPoseBuffer, 0x00ff44,
            this._pathLineResolutionVec());
        if (this._pathObj) this._pathObj.visible = true;
    }

    _rebuildPathFromBuffer() {
        this._syncPathLineFromBuffer();
    }

    /**
     * /Odometry 위치 → path 버퍼에 조건부 append (1m 이상 이동 시, 최대 LIVE_PATH_MAX_POSES).
     * append 후 Line2 동기화 (Slam Live path와 동일).
     */
    _appendPathFromOdometry(x, y, z) {
        const last = this._lastPathPose;
        if (last) {
            const dist = Math.hypot(x - last.x, y - last.y, z - last.z);
            if (dist < LIVE_PATH_MIN_DIST_M) return;
        }

        let buf = this._pathPoseBuffer.xyz;
        let count = this._pathPoseBuffer.count || 0;

        if (!buf || count === 0) {
            buf = new Float32Array(3);
            buf[0] = x;
            buf[1] = y;
            buf[2] = z;
            count = 1;
        } else if (count >= LIVE_PATH_MAX_POSES) {
            // sliding window: 가장 오래된 1개 제거 후 새 포인트 append
            const next = new Float32Array(LIVE_PATH_MAX_POSES * 3);
            next.set(buf.subarray(3, count * 3));
            const i = (LIVE_PATH_MAX_POSES - 1) * 3;
            next[i] = x;
            next[i + 1] = y;
            next[i + 2] = z;
            buf = next;
            count = LIVE_PATH_MAX_POSES;
        } else {
            const next = new Float32Array((count + 1) * 3);
            next.set(buf);
            next[count * 3] = x;
            next[count * 3 + 1] = y;
            next[count * 3 + 2] = z;
            buf = next;
            count += 1;
        }

        this._pathPoseBuffer.xyz = buf;
        this._pathPoseBuffer.count = count;
        this._pathPoseBuffer.lastMsgCount = count;
        this._lastPathPose = { x, y, z };
        this._syncPathLineFromBuffer();
    }

    _subscribeOdometry(topic) {
        const t = new ROSLIB.Topic({
            ros: this._ros,
            name: topic,
            messageType: getMsgType('nav_msgs/Odometry', 'nav_msgs/msg/Odometry'),
            throttle_rate: 200,
            queue_length: 1
        });
        t.subscribe((msg) => {
            const THREE = window.THREE;
            if (!this._scene || !THREE) return;

            const pos = msg.pose.pose.position;
            const rot = msg.pose.pose.orientation;
            const frameId = (msg.header && msg.header.frame_id) ? msg.header.frame_id : '';
            const childFrameId = msg.child_frame_id || 'base_link';

            if (frameId) this._knownFrames.add(frameId);
            if (childFrameId) this._knownFrames.add(childFrameId);

            if (!this._robotOdomGroup) {
                const group = _createRobotAxesGroup(
                    THREE, LIVE_ROBOT_AXIS_LENGTH, LIVE_ROBOT_AXIS_LINEWIDTH,
                    this._pathLineResolutionVec());
                this._robotOdomGroup = group;
                group.visible = true;
                this._scene.add(group);
                this._lastRobotAxisScale = undefined;
                this._updateRobotAxisZoomScale();
            }

            this._robotOdomGroup.position.set(pos.x, pos.y, pos.z);
            this._robotOdomGroup.quaternion.set(rot.x, rot.y, rot.z, rot.w);

            if (!this._robotPos) this._robotPos = new THREE.Vector3();
            this._robotPos.set(pos.x, pos.y, pos.z);

            // path: /Odometry 위치 샘플링 (Follow/axis와 동일 콜백, 기존 동작 유지)
            this._appendPathFromOdometry(pos.x, pos.y, pos.z);
        });
        this._subscriptions.push(t);
    }

    _subscribeTF(topic) {
        const t = new ROSLIB.Topic({
            ros: this._ros,
            name: topic,
            messageType: getMsgType('tf2_msgs/TFMessage', 'tf2_msgs/msg/TFMessage'),
            throttle_rate: 200,
            queue_length: 1
        });
        t.subscribe((msg) => {
            const THREE = window.THREE;
            if (!this._scene || !THREE) return;
            for (const transform of (msg.transforms || [])) {
                const childId = transform.child_frame_id;
                const parentId = transform.header.frame_id;
                const trans = transform.transform.translation;
                const rot = transform.transform.rotation;

                this._knownFrames.add(childId);
                this._knownFrames.add(parentId);

                // Robot pose/axis는 /Odometry — base_link·body TF axis는 중복 방지
                if (childId === 'base_link' || childId === 'body') continue;

                if (!this._tfObjects[childId]) {
                    const group = new THREE.Group();
                    group.add(new THREE.AxesHelper(0.5));
                    this._tfObjects[childId] = { group };
                    group.visible = true;
                    this._scene.add(group);
                }

                const entry = this._tfObjects[childId];
                entry.group.position.set(trans.x, trans.y, trans.z);
                entry.group.quaternion.set(rot.x, rot.y, rot.z, rot.w);
            }
        });
        this._subscriptions.push(t);
    }

    _subscribeMap(topic) {
        const t = new ROSLIB.Topic({
            ros: this._ros,
            name: topic,
            messageType: getMsgType('nav_msgs/OccupancyGrid', 'nav_msgs/msg/OccupancyGrid'),
            throttle_rate: 1000,
            queue_length: 1
        });
        t.subscribe((msg) => {
            const THREE = window.THREE;
            if (!this._scene || !THREE) return;
            const { resolution, width, height, origin } = msg.info;
            let raw;
            try {
                if (Array.isArray(msg.data)) {
                    raw = new Int8Array(msg.data);
                } else if (typeof msg.data === 'string') {
                    raw = new Int8Array(Uint8Array.from(atob(msg.data), c => c.charCodeAt(0)).buffer);
                } else {
                    console.error('[LocalizationLiveViewer] Unknown OccupancyGrid data type:', typeof msg.data);
                    return;
                }
            } catch (e) {
                console.error('[LocalizationLiveViewer] OccupancyGrid decode error:', e);
                return;
            }

            const cvs = document.createElement('canvas');
            cvs.width = width;
            cvs.height = height;
            const ctx = cvs.getContext('2d');
            const img = ctx.createImageData(width, height);
            for (let i = 0; i < raw.length; i++) {
                const v = raw[i];
                let r, g, b, a = 255;
                if (v < 0)       { r = 100; g = 100; b = 100; a = 160; }  // unknown: gray
                else if (v > 50) { r = 10;  g = 10;  b = 10;  }           // occupied: near-black
                else             { r = 255; g = 255; b = 255; a = 220; }  // free: bright white
                img.data[i * 4]     = r;
                img.data[i * 4 + 1] = g;
                img.data[i * 4 + 2] = b;
                img.data[i * 4 + 3] = a;
            }
            ctx.putImageData(img, 0, 0);

            if (this._mapMesh) {
                this._scene.remove(this._mapMesh);
                if (this._mapMesh.geometry) this._mapMesh.geometry.dispose();
                if (this._mapMesh.material) this._mapMesh.material.dispose();
                this._mapMesh = null;
            }
            if (this._mapTexture) {
                this._mapTexture.dispose();
                this._mapTexture = null;
            }

            this._mapTexture = new THREE.CanvasTexture(cvs);
            this._mapTexture.magFilter = THREE.NearestFilter;
            this._mapTexture.minFilter = THREE.NearestFilter;
            this._mapTexture.flipY = false;
            const geo = new THREE.PlaneGeometry(width * resolution, height * resolution);
            const mat = new THREE.MeshBasicMaterial({
                map: this._mapTexture,
                transparent: true,
                opacity: 0.7,
                depthWrite: false
            });
            this._mapMesh = new THREE.Mesh(geo, mat);
            this._mapMesh.position.set(
                origin.position.x + width * resolution / 2,
                origin.position.y + height * resolution / 2,
                0
            );
            this._mapMesh.visible = true;
            this._scene.add(this._mapMesh);
        });
        this._subscriptions.push(t);
    }


    resetView() {
        if (!this._camera || !this._controls) return;
        this._camera.position.set(0, 0, 20);
        this._camera.up.set(0, 0, 1);
        this._controls.target.set(0, 0, 0);
        this._restoreOrbitControls();
        this._controls.update();
        const toggle = document.getElementById('loc-viewer-topview-toggle');
        if (toggle && toggle.checked) {
            toggle.checked = false;
            this._topView = false;
            this._savedCameraPos = null;
        }
    }

    _restoreOrbitControls() {
        if (!this._controls) return;
        this._controls.enableDamping = true;
        this._controls.enableRotate = true;
        this._controls.minPolarAngle = 0;
        this._controls.maxPolarAngle = Math.PI;
        if (window.THREE) {
            this._controls.mouseButtons = {
                LEFT: window.THREE.MOUSE.ROTATE,
                MIDDLE: window.THREE.MOUSE.DOLLY,
                RIGHT: window.THREE.MOUSE.PAN
            };
        }
    }

    toggleFollow(force) {
        this._followMode = (force !== undefined) ? force : !this._followMode;
        const btn = document.getElementById('loc-follow-btn');
        if (btn) btn.classList.toggle('active', this._followMode);
    }

    toggleTopView(enable) {
        this._topView = enable;
        if (!this._perspCamera || !this._controls) return;
        const THREE = window.THREE;

        if (enable) {
            // PerspectiveCamera 현재 상태 저장
            this._savedCameraPos = this._perspCamera.position.clone();
            this._savedCameraUp  = this._perspCamera.up.clone();
            this._savedTarget    = this._controls.target.clone();

            const cx     = this._controls.target.x || 0;
            const cy     = this._controls.target.y || 0;
            const dist   = this._perspCamera.position.distanceTo(this._controls.target) || 100;
            const halfH  = Math.max(dist * 0.6, 30);

            const container = document.getElementById('loc-viewer-canvas-container');
            const cw = container ? container.clientWidth  : 600;
            const ch = container ? container.clientHeight : 480;
            const aspect = cw / ch;

            // OrthographicCamera: 완전 수직 XY 평면 탑뷰 (투시 왜곡 없음)
            if (!this._orthoCamera) {
                this._orthoCamera = new THREE.OrthographicCamera(
                    -halfH * aspect, halfH * aspect,
                    halfH, -halfH,
                    -10000, 10000
                );
            } else {
                this._orthoCamera.left   = -halfH * aspect;
                this._orthoCamera.right  =  halfH * aspect;
                this._orthoCamera.top    =  halfH;
                this._orthoCamera.bottom = -halfH;
            }
            // 카메라를 씬 정중앙 바로 위에 배치, Y-up으로 짐벌락 방지
            this._orthoCamera.position.set(cx, cy, 1000);
            this._orthoCamera.up.set(0, 1, 0);
            this._orthoCamera.lookAt(cx, cy, 0);
            this._orthoCamera.updateProjectionMatrix();

            // OrbitControls를 OrthographicCamera로 전환 (회전 불가, 팬/줌만 허용)
            this._controls.object = this._orthoCamera;
            this._controls.target.set(cx, cy, 0);
            this._controls.enableRotate = false;
            this._controls.enablePan    = true;
            this._controls.enableZoom   = true;
            this._controls.enableDamping = true;
            this._controls.update();

            this._camera = this._orthoCamera;
            // 전환 즉시 포인트 크기 보정 (렌더 루프 첫 프레임 전에 적용)
            this._lastOrthoScale = undefined;
            this._updateOrthoPointSizes();
        } else {
            // PerspectiveCamera 복원
            this._camera = this._perspCamera;
            this._controls.object = this._perspCamera;
            this._restoreOrbitControls();

            // Perspective 복원 시 포인트 크기를 원래 월드 단위 크기로 되돌림
            this._ensurePerspectivePointSizes();
            this._perspPointSizesDirty = false;

            if (this._savedCameraPos) {
                this._perspCamera.position.copy(this._savedCameraPos);
                this._perspCamera.up.copy(this._savedCameraUp);
                this._controls.target.copy(this._savedTarget);
                this._savedCameraPos = null;
            } else {
                this._perspCamera.position.set(0, -30, 20);
                this._perspCamera.up.set(0, 0, 1);
                this._controls.target.set(0, 0, 0);
            }
            this._controls.update();
        }

        // header 체크박스 + fullscreen 버튼 active 상태 동기화
        const headerToggle = document.getElementById('loc-viewer-topview-toggle');
        if (headerToggle) headerToggle.checked = enable;
        const fsBtn = document.getElementById('loc-fs-topview-btn');
        if (fsBtn) fsBtn.classList.toggle('active', enable);
    }

    toggleFullscreen() {
        const container = document.getElementById('loc-viewer-canvas-container');
        if (!container) return;
        const isFullscreen = !!(document.fullscreenElement || document.webkitFullscreenElement);
        if (isFullscreen) {
            (document.exitFullscreen || document.webkitExitFullscreen).call(document);
        } else {
            (container.requestFullscreen || container.webkitRequestFullscreen).call(container);
        }
    }

    takeSnapshot(scale = 2) {
        if (!this._renderer || !this._scene || !this._camera) return;
        const container = document.getElementById('loc-viewer-canvas-container');
        if (!container) return;

        const w = container.clientWidth;
        const h = container.clientHeight;
        const sw = w * scale;
        const sh = h * scale;
        const timestamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
        const filename = `loc_snapshot_${timestamp}.png`;

        this._renderer.setSize(sw, sh);
        if (this._perspCamera) {
            this._perspCamera.aspect = w / h;
            this._perspCamera.updateProjectionMatrix();
        }

        this._renderer.render(this._scene, this._camera);

        const canvas = this._renderer.domElement;
        const link = document.createElement('a');
        link.href = canvas.toDataURL('image/png');
        link.download = filename;
        link.click();

        // 원래 크기 복원
        this._renderer.setSize(w, h);
        if (this._perspCamera) {
            this._perspCamera.aspect = w / h;
            this._perspCamera.updateProjectionMatrix();
        }
        this._renderer.render(this._scene, this._camera);
    }

    _updateFixedFrameDropdown() {
        const dropdown = document.getElementById('loc-fixed-frame-dropdown');
        if (!dropdown || dropdown.style.display === 'none') return;
        const input = document.getElementById('loc-fixed-frame-input');
        const filter = input ? input.value.toLowerCase() : '';
        dropdown.innerHTML = '';
        for (const frame of this._knownFrames) {
            if (filter && !frame.toLowerCase().includes(filter)) continue;
            const item = document.createElement('div');
            item.className = 'fixed-frame-dropdown-item';
            item.textContent = frame;
            item.addEventListener('mousedown', (e) => {
                e.preventDefault();
                this._fixedFrame = frame;
                if (input) input.value = frame;
                dropdown.style.display = 'none';
            });
            dropdown.appendChild(item);
        }
    }

    onFixedFrameFocus() {
        const dropdown = document.getElementById('loc-fixed-frame-dropdown');
        if (dropdown) {
            dropdown.style.display = 'block';
            this._updateFixedFrameDropdown();
        }
    }

    onFixedFrameInput(value) {
        this._fixedFrame = value;
        this._updateFixedFrameDropdown();
    }

    onFixedFrameBlur(event) {
        setTimeout(() => {
            const dropdown = document.getElementById('loc-fixed-frame-dropdown');
            if (dropdown) dropdown.style.display = 'none';
        }, 150);
    }

    toggleFixedFrameDropdown() {
        const dropdown = document.getElementById('loc-fixed-frame-dropdown');
        if (!dropdown) return;
        const isVisible = dropdown.style.display !== 'none';
        dropdown.style.display = isVisible ? 'none' : 'block';
        if (!isVisible) this._updateFixedFrameDropdown();
    }
}

// ==============================================================
// SlamLiveViewer - LiDAR SLAM 실시간 3D 뷰어
// 구독 토픽: /cloud_registered(현재 스캔 흰색), /PGO_map, /kf_node, /PGO_path, /loopLine, /tf, /Odometry
// LIO path: /key_frame(fast_lio/Frame) → PGO가 발행하는 /kf_node 위치 시퀀스
// ==============================================================
class SlamLiveViewer {
    constructor() {
        this._scene = null;
        this._camera = null;
        this._perspCamera = null;
        this._orthoCamera = null;
        this._renderer = null;
        this._controls = null;
        this._animFrameId = null;
        this._initialized = false;
        this._visible = false;
        this._ros = null;
        this._subscriptions = [];

        // PointCloud2 객체
        this._cloudObj = null;        // /cloud_registered (현재 스캔, 흰색 — 누적 없음)
        this._pgoMapObj = null;       // /PGO_map (z-rainbow)
        this._kfNodeObj = null;       // /kf_node (키프레임 구)
        this._kfNodeCapacity = 0;     // InstancedMesh 재사용 용량
        this._kfNodePosXyz = null;    // 줌 스케일 재적용용 위치 버퍼
        this._kfNodeBaseRadius = 0.7; // 기본 반지름
        this._lastKfNodeScale = undefined;
        this._lastRobotAxisScale = undefined;

        // Path: THREE.Line + 사전할당 버퍼 (TubeGeometry 매 프레임 dispose/new 제거)
        this._pathLineState = _createPathLineState();      // LIO (kf_node)
        this._pgoPathLineState = _createPathLineState();   // PGO (/PGO_path)
        this._pathObj = null;         // line alias (호환)
        this._pgoPathObj = null;
        this._pathPoseCount = 0;
        this._pgoPathPoseCount = 0;
        this._pathPoseBuffer = { xyz: null, count: 0, lastMsgCount: 0 };
        this._pgoPathPoseBuffer = { xyz: null, count: 0, lastMsgCount: 0 };
        this._pathRebuildTimer = null;
        this._pgoPathRebuildTimer = null;

        // Marker 객체 (/loopLine)
        this._loopLineObj = null;

        // TF 객체 (/tf) — LocalizationLiveViewer와 동일한 구조
        this._tfObjects = {};         // childFrameId → { group: THREE.Group }
        this._robotOdomGroup = null; // THREE.Group — robot axes from /Odometry
        this._odomTopic = '/Odometry';
        this._robotPos   = null;   // THREE.Vector3 — updated from /Odometry
        this._followMode = true;   // camera follow toggle
        this._knownFrames = new Set();

        this._topView = false;
        this._savedCameraPos = null;
        this._savedCameraUp = null;
        this._savedTarget = null;
        this._resizeObserver = null;
        this._backendSubscribed = false;
        this._wsConnectGen = 0;
        this._rosbridgeSubscribed = false;
        this._cloudBusy = false;
        this._cloudLastMs = 0;
        this._cloudBusyWatchdog = null;
        this._pgoMapBusy = false;
        this._pgoMapLastMs = 0;
        this._pgoMapBusyWatchdog = null;
        this._kfNodeLastMs = 0;
        this._perspPointSizesDirty = false;
        this._lastOrthoScale = undefined;
    }

    _waitForThree() {
        return new Promise((resolve) => {
            const check = () => {
                if (window.THREE && window.OrbitControls) resolve();
                else setTimeout(check, 100);
            };
            check();
        });
    }

    async _init() {
        if (this._initialized) return;
        await this._waitForThree();
        const THREE = window.THREE;
        const canvas = document.getElementById('slam-live-canvas');
        if (!canvas) return;

        const container = document.getElementById('slam-live-canvas-container');
        const w = container.clientWidth || 600;
        const h = container.clientHeight || 480;

        this._renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
        this._renderer.setPixelRatio(window.devicePixelRatio);
        this._renderer.setSize(w, h);

        this._scene = new THREE.Scene();
        this._scene.background = new THREE.Color(0x0a0a18);

        this._camera = new THREE.PerspectiveCamera(60, w / h, 0.1, 10000);
        this._camera.position.set(0, 0, 20);
        this._camera.up.set(0, 0, 1);
        this._perspCamera = this._camera;

        this._controls = new window.OrbitControls(this._camera, this._renderer.domElement);
        this._controls.enableDamping = true;
        this._controls.dampingFactor = 0.1;

        this._scene.add(new THREE.AxesHelper(3));

        this._resizeObserver = new ResizeObserver(() => this._resizeRenderer());
        this._resizeObserver.observe(container);

        this._initialized = true;
        this._startRenderLoop();
    }

    _startRenderLoop() {
        const LERP = 0.08;
        const animate = () => {
            this._animFrameId = requestAnimationFrame(animate);

            // Camera follow: pan camera+target together toward robot (preserves orbit angle)
            if (this._followMode && this._robotPos && this._controls) {
                const rp = this._robotPos;
                if (this._topView && this._orthoCamera) {
                    // TopView: move orthoCamera + target together
                    const dx = (rp.x - this._orthoCamera.position.x) * LERP;
                    const dy = (rp.y - this._orthoCamera.position.y) * LERP;
                    this._orthoCamera.position.x += dx;
                    this._orthoCamera.position.y += dy;
                    this._controls.target.x += dx;
                    this._controls.target.y += dy;
                    this._orthoCamera.lookAt(
                        this._orthoCamera.position.x,
                        this._orthoCamera.position.y, 0
                    );
                } else {
                    // Orbit: move camera + target by same delta (preserves angle/distance)
                    const dx = (rp.x - this._controls.target.x) * LERP;
                    const dy = (rp.y - this._controls.target.y) * LERP;
                    const dz = (rp.z - this._controls.target.z) * LERP;
                    this._controls.target.x += dx;
                    this._controls.target.y += dy;
                    this._controls.target.z += dz;
                    this._perspCamera.position.x += dx;
                    this._perspCamera.position.y += dy;
                    this._perspCamera.position.z += dz;
                }
            }

            if (this._controls) this._controls.update();
            if (this._topView && this._orthoCamera) {
                this._updateOrthoPointSizes();
            } else if (this._perspPointSizesDirty) {
                this._ensurePerspectivePointSizes();
                this._perspPointSizesDirty = false;
            }
            // kf_node·로봇 AXIS: 줌 아웃 시 화면 크기 유지 (다른 점군은 sizeAttenuation 유지)
            this._updateKfNodeZoomScale();
            this._updateRobotAxisZoomScale();
            if (this._renderer && this._scene && this._camera) {
                this._renderer.render(this._scene, this._camera);
            }
        };
        animate();
    }

    _getOrthoPixelsPerUnit() {
        if (!this._orthoCamera || !this._renderer) return 40;
        const frustumH = (this._orthoCamera.top - this._orthoCamera.bottom) / (this._orthoCamera.zoom || 1);
        const pixelH   = this._renderer.domElement.height || 480;
        return pixelH / frustumH;
    }

    /** kf_node 구체: 줌아웃은 완만 확대, 줌인(확대) 시에는 더 작게 */
    _computeKfNodeInstanceScale() {
        const baseR = this._kfNodeBaseRadius || 0.7;
        if (this._topView && this._orthoCamera) {
            const ppu = this._getOrthoPixelsPerUnit();
            // 화면상 반지름 ~5px 목표. 확대 시 min을 낮춰 구가 과대 표시되지 않게
            const desiredR = 5 / Math.max(ppu, 0.01);
            return Math.max(0.12, Math.min(desiredR / baseR, 4));
        }
        const cam = this._perspCamera || this._camera;
        if (!cam || !this._controls) return 1;
        const dist = cam.position.distanceTo(this._controls.target);
        const refDist = 40;
        return Math.max(0.12, Math.min(dist / refDist, 4));
    }

    /** 로봇 AXIS: 줌 아웃 시 약한 스케일 */
    _computeRobotAxisScale() {
        if (this._topView && this._orthoCamera) {
            const ppu = this._getOrthoPixelsPerUnit();
            // 축 길이 ~28px on screen, 스케일 상한 4
            const desiredLen = 28 / Math.max(ppu, 0.01);
            return Math.max(0.5, Math.min(desiredLen / LIVE_ROBOT_AXIS_LENGTH, 4));
        }
        const cam = this._perspCamera || this._camera;
        if (!cam || !this._controls) return 1;
        const dist = cam.position.distanceTo(this._controls.target);
        return Math.max(0.5, Math.min(dist / 40, 4));
    }

    _updateKfNodeZoomScale() {
        if (!this._kfNodeObj || !this._kfNodePosXyz) return;
        const s = this._computeKfNodeInstanceScale();
        if (this._lastKfNodeScale !== undefined && Math.abs(this._lastKfNodeScale - s) < 0.03) {
            return;
        }
        this._lastKfNodeScale = s;
        const THREE = window.THREE;
        if (!THREE) return;
        const dummy = new THREE.Object3D();
        const count = this._kfNodeObj.count | 0;
        const xyz = this._kfNodePosXyz;
        for (let i = 0; i < count; i++) {
            dummy.position.set(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]);
            dummy.scale.setScalar(s);
            dummy.updateMatrix();
            this._kfNodeObj.setMatrixAt(i, dummy.matrix);
        }
        this._kfNodeObj.instanceMatrix.needsUpdate = true;
    }

    _updateRobotAxisZoomScale() {
        if (!this._robotOdomGroup) return;
        const s = this._computeRobotAxisScale();
        if (this._lastRobotAxisScale !== undefined && Math.abs(this._lastRobotAxisScale - s) < 0.03) {
            return;
        }
        this._lastRobotAxisScale = s;
        this._robotOdomGroup.scale.setScalar(s);
    }

    _updateOrthoPointSizes() {
        const scale = this._getOrthoPixelsPerUnit();
        if (this._lastOrthoScale !== undefined && Math.abs(this._lastOrthoScale - scale) < 0.05) {
            return;
        }
        this._lastOrthoScale = scale;
        const update = (obj) => {
            if (!obj || !obj.material) return;
            // InstancedMesh(kf_node 구체)는 sizeAttenuation 없으므로 스킵
            if (!('size' in obj.material)) return;
            if (obj.material.sizeAttenuation !== false) {
                obj.material.sizeAttenuation = false;
                obj.material.needsUpdate = true;
            }
            const baseSize = obj.material._baseSize || 0.1;
            obj.material.size = Math.max(1, baseSize * scale);
        };
        update(this._cloudObj);
        update(this._pgoMapObj);
    }

    /** Orbit(Perspective) 뷰: sizeAttenuation·월드 크기 복원 (탑뷰 전환 후 잔류 방지) */
    _ensurePerspectivePointSizes() {
        const fix = (obj) => {
            if (!obj || !obj.material || obj.material._baseSize === undefined) return;
            if (!('size' in obj.material)) return;
            let attenuationChanged = false;
            if (obj.material.sizeAttenuation !== true) {
                obj.material.sizeAttenuation = true;
                attenuationChanged = true;
            }
            if (obj.material.size !== obj.material._baseSize) {
                obj.material.size = obj.material._baseSize;
            }
            if (attenuationChanged) {
                obj.material.needsUpdate = true;
            }
        };
        fix(this._cloudObj);
        fix(this._pgoMapObj);
    }

    _pathLineResolutionVec() {
        const THREE = window.THREE;
        if (!THREE) return null;
        const container = document.getElementById('slam-live-canvas-container');
        const w = container ? (container.clientWidth || 600) : 600;
        const h = container ? (container.clientHeight || 380) : 380;
        if (!this._pathLineRes) this._pathLineRes = new THREE.Vector2(w, h);
        else this._pathLineRes.set(w, h);
        return this._pathLineRes;
    }

    _updatePathLineResolutions(w, h) {
        const apply = (lineState) => {
            const mat = lineState && lineState.line && lineState.line.material;
            if (mat && mat.resolution) mat.resolution.set(w, h);
        };
        apply(this._pathLineState);
        apply(this._pgoPathLineState);
        _setGroupLineResolutions(this._robotOdomGroup, w, h);
    }

    _resizeRenderer() {
        if (!this._renderer) return;
        const container = document.getElementById('slam-live-canvas-container');
        if (!container) return;
        const w = container.clientWidth;
        const h = container.clientHeight;
        if (w > 0 && h > 0) {
            this._renderer.setSize(w, h);
            this._updatePathLineResolutions(w, h);
            const aspect = w / h;
            if (this._topView && this._orthoCamera) {
                const halfH = this._orthoCamera.top;
                this._orthoCamera.left  = -halfH * aspect;
                this._orthoCamera.right =  halfH * aspect;
                this._orthoCamera.updateProjectionMatrix();
            } else if (this._perspCamera) {
                this._perspCamera.aspect = aspect;
                this._perspCamera.updateProjectionMatrix();
            }
        }
    }

    async show() {
        const viewerEl = document.getElementById('slam-live-viewer');
        if (viewerEl) viewerEl.style.display = 'block';
        this._visible = true;
        await this._init();
        // 브라우저 레이아웃 계산 완료 후 리사이즈 (display:none → block 직후 clientHeight가 0일 수 있음)
        await new Promise(resolve => requestAnimationFrame(resolve));
        this._resizeRenderer();
        this.toggleTopView(true);
        await this._connectAndSubscribe();
    }

    hide() {
        this._visible = false;
        this._wsConnectGen++;
        this._backendSubscribed = false;
        this._rosbridgeSubscribed = false;
        this._cloudBusy = false;
        this._cloudLastMs = 0;
        this._pgoMapBusy = false;
        this._pgoMapLastMs = 0;
        this._kfNodeLastMs = 0;
        if (this._cloudBusyWatchdog) {
            clearTimeout(this._cloudBusyWatchdog);
            this._cloudBusyWatchdog = null;
        }
        if (this._pgoMapBusyWatchdog) {
            clearTimeout(this._pgoMapBusyWatchdog);
            this._pgoMapBusyWatchdog = null;
        }
        this._unsubscribeAll();
        this._clearLiveObjects();
        const viewerEl = document.getElementById('slam-live-viewer');
        if (viewerEl) viewerEl.style.display = 'none';
        const topViewToggle = document.getElementById('slam-live-topview-toggle');
        if (topViewToggle) topViewToggle.checked = false;
        this._topView = false;
        this._savedCameraPos = null;
        this._knownFrames.clear();
    }

    _clearLiveObjects() {
        const THREE = window.THREE;
        if (!this._scene || !THREE) return;

        const removeObj = (obj) => {
            if (!obj) return;
            this._scene.remove(obj);
            if (obj.isGroup) {
                obj.traverse((child) => {
                    if (child.geometry) child.geometry.dispose();
                    if (child.material) {
                        if (Array.isArray(child.material)) child.material.forEach(m => m.dispose());
                        else child.material.dispose();
                    }
                });
            } else {
                if (obj.geometry) obj.geometry.dispose();
                if (obj.material) {
                    if (Array.isArray(obj.material)) {
                        obj.material.forEach(m => m.dispose());
                    } else {
                        if (obj.material.map) obj.material.map.dispose();
                        obj.material.dispose();
                    }
                }
            }
        };

        removeObj(this._cloudObj);
        removeObj(this._pgoMapObj);
        removeObj(this._kfNodeObj);
        removeObj(this._loopLineObj);
        _disposePathLineState(this._scene, this._pathLineState);
        _disposePathLineState(this._scene, this._pgoPathLineState);

        this._cloudObj = null;
        this._pgoMapObj = null;
        this._kfNodeObj = null;
        this._kfNodeCapacity = 0;
        this._kfNodePosXyz = null;
        this._lastKfNodeScale = undefined;
        this._pathObj = null;
        this._pgoPathObj = null;
        this._loopLineObj = null;
        this._pathPoseCount = 0;
        this._pgoPathPoseCount = 0;
        _clearPathBufferState(this, '_pathPoseBuffer', '_pathRebuildTimer');
        _clearPathBufferState(this, '_pgoPathPoseBuffer', '_pgoPathRebuildTimer');

        // TF 객체 정리
        for (const key of Object.keys(this._tfObjects)) {
            const entry = this._tfObjects[key];
            if (entry && entry.group) this._scene.remove(entry.group);
        }
        this._tfObjects = {};

        if (this._robotOdomGroup) {
            _disposeRobotAxesGroup(this._scene, this._robotOdomGroup);
            this._robotOdomGroup = null;
            this._lastRobotAxisScale = undefined;
        }

        this._knownFrames.clear();
    }

    async _connectAndSubscribe() {
        if (this._backendSubscribed) return;

        const loadingEl = document.getElementById('slam-live-viewer-loading');
        if (loadingEl) {
            loadingEl.textContent = '백엔드 WebSocket 연결 중...';
            loadingEl.style.display = 'block';
        }
        if (typeof ensureWebuiPortsReady === 'function') {
            await ensureWebuiPortsReady();
        }

        // PC2/Path는 Python 백엔드 WS(8881) — rosbridge 연결과 무관하게 즉시 구독
        console.log('[SlamLiveViewer] Subscribing binary topics (backend WS 8881)');
        this._subscribeBinaryTopics();
        this._backendSubscribed = true;
        if (loadingEl) loadingEl.style.display = 'none';

        const onRosbridgeReady = () => {
            this._subscribeRosbridgeTopics();
        };

        if (window.plotState && plotState.ros && plotState.ros.isConnected) {
            _verifyRosbridgeAlive(plotState.ros, 2500).then((alive) => {
                if (alive) {
                    this._ros = plotState.ros;
                    onRosbridgeReady();
                } else {
                    console.warn('[SlamLiveViewer] plotState.ros stale — own rosbridge connection');
                    this._connectOwnRosbridge(onRosbridgeReady);
                }
            });
            return;
        }

        this._connectOwnRosbridge(onRosbridgeReady);
    }

    _connectOwnRosbridge(onRosbridgeReady) {
        try {
            const url = _getRosbridgeUrl();
            console.log('[SlamLiveViewer] rosbridge connecting (TF/loopLine):', url);
            this._ros = new ROSLIB.Ros({ url });
            this._ros.on('connection', () => {
                console.log('[SlamLiveViewer] rosbridge connected:', url);
                onRosbridgeReady();
            });
            this._ros.on('error', (err) => {
                console.error('[SlamLiveViewer] rosbridge error:', url, err);
            });
            this._ros.on('close', () => {
                console.warn('[SlamLiveViewer] rosbridge connection closed:', url);
            });
        } catch (e) {
            console.error('[SlamLiveViewer] failed to init rosbridge:', e);
        }
    }

    _subscribeBinaryTopics() {
        // 현재 스캔(흰색) — 누적 없음, 프레임마다 교체
        this._subscribePC2Binary('/cloud_registered', 'cloud_registered');
        this._subscribePC2Binary('/PGO_map', 'pgo_map');
        // /kf_node: 키프레임 구 + LIO path (FAST_LIO /key_frame → PGO PointCloud2)
        this._subscribePC2Binary('/kf_node', 'kf_node');
        this._subscribePathBinary('/PGO_path', 'pgo_path', 0xffffff);
    }

    _subscribeRosbridgeTopics() {
        if (!this._ros || this._rosbridgeSubscribed) return;
        this._rosbridgeSubscribed = true;
        this._subscribeOdometry(this._odomTopic);
        this._subscribeMarker('/loopLine');
        this._subscribeTF('/tf');
    }

    _subscribeAll() {
        this._subscribeBinaryTopics();
        this._subscribeRosbridgeTopics();
    }

    _subscribeOdometry(topic) {
        const t = new ROSLIB.Topic({
            ros: this._ros,
            name: topic,
            messageType: getMsgType('nav_msgs/Odometry', 'nav_msgs/msg/Odometry'),
            throttle_rate: 200,
            queue_length: 1
        });
        t.subscribe((msg) => {
            const THREE = window.THREE;
            if (!this._scene || !THREE) return;

            const pos = msg.pose.pose.position;
            const rot = msg.pose.pose.orientation;
            const frameId = (msg.header && msg.header.frame_id) ? msg.header.frame_id : '';
            const childFrameId = msg.child_frame_id || 'base_link';

            if (frameId) this._knownFrames.add(frameId);
            if (childFrameId) this._knownFrames.add(childFrameId);

            if (!this._robotOdomGroup) {
                const group = _createRobotAxesGroup(
                    THREE, LIVE_ROBOT_AXIS_LENGTH, LIVE_ROBOT_AXIS_LINEWIDTH,
                    this._pathLineResolutionVec());
                this._robotOdomGroup = group;
                group.visible = true;
                this._scene.add(group);
                this._lastRobotAxisScale = undefined;
                this._updateRobotAxisZoomScale();
            }

            this._robotOdomGroup.position.set(pos.x, pos.y, pos.z);
            this._robotOdomGroup.quaternion.set(rot.x, rot.y, rot.z, rot.w);

            if (!this._robotPos) this._robotPos = new THREE.Vector3();
            this._robotPos.set(pos.x, pos.y, pos.z);
        });
        this._subscriptions.push(t);
    }

    _unsubscribeAll() {
        for (const t of this._subscriptions) {
            try { t.unsubscribe(); } catch (e) { /* ignore */ }
        }
        this._subscriptions = [];
        this._rosbridgeSubscribed = false;
    }

    _rainbowColor(t) {
        const h = (1 - t) * 240;
        const c = 1;
        const x = c * (1 - Math.abs(((h / 60) % 2) - 1));
        let r = 0, g = 0, b = 0;
        if      (h < 60)  { r = c; g = x; b = 0; }
        else if (h < 120) { r = x; g = c; b = 0; }
        else if (h < 180) { r = 0; g = c; b = x; }
        else if (h < 240) { r = 0; g = x; b = c; }
        else if (h < 300) { r = x; g = 0; b = c; }
        else              { r = c; g = 0; b = x; }
        return [r, g, b];
    }

    // 바이너리 PC2 패킷 파싱
    // flags bit0=has_intensity → intensity 배열도 함께 반환 (kf_node 색상에 활용)
    // options: { maxPts, fillWhite } — fillWhite면 rainbow 생략(흰색 스캔용)
    _parseBinaryPC2(buffer, options = {}) {
        try {
            const view = new DataView(buffer);
            if (view.getUint8(0) !== 0x50 || view.getUint8(1) !== 0x43 || view.getUint8(2) !== 0x32) return null;
            let off = 3;
            /* version    = */ view.getUint8(off++);
            const flags    = view.getUint8(off++);
            const topicLen = view.getUint32(off, true); off += 4;
            const frameLen = view.getUint32(off, true); off += 4;
            const count    = view.getUint32(off, true); off += 4;
            off += topicLen + frameLen;
            if (count === 0) return null;

            const hasIntensity = (flags & 1) !== 0;
            const fillWhite = !!options.fillWhite;

            const MAX_PTS   = options.maxPts || 80000;
            const step      = Math.max(1, Math.floor(count / MAX_PTS));
            const outPts    = Math.ceil(count / step);
            const positions = new Float32Array(outPts * 3);
            const colors    = new Float32Array(outPts * 3);
            const tempZ     = fillWhite ? null : new Float32Array(outPts);
            let minZ = Infinity, maxZ = -Infinity, idx = 0;

            const xyzBase = off;
            for (let i = 0; i < count; i += step) {
                const b = xyzBase + i * 12;
                if (b + 12 > buffer.byteLength) break;
                const x = view.getFloat32(b,     true);
                const y = view.getFloat32(b + 4, true);
                const z = view.getFloat32(b + 8, true);
                if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
                positions[idx * 3]     = x;
                positions[idx * 3 + 1] = y;
                positions[idx * 3 + 2] = z;
                if (fillWhite) {
                    colors[idx * 3] = 1.0;
                    colors[idx * 3 + 1] = 1.0;
                    colors[idx * 3 + 2] = 1.0;
                } else {
                    tempZ[idx] = z;
                    if (z < minZ) minZ = z;
                    if (z > maxZ) maxZ = z;
                }
                idx++;
            }

            if (!fillWhite) {
                const range = (maxZ - minZ) || 1;
                for (let i = 0; i < idx; i++) {
                    const t = (tempZ[i] - minZ) / range;
                    const [r, g, b] = this._rainbowColor(t);
                    colors[i * 3]     = r;
                    colors[i * 3 + 1] = g;
                    colors[i * 3 + 2] = b;
                }
            }

            // intensity 추출 (XYZ 블록 바로 뒤, float32 × count)
            let intensities = null;
            if (hasIntensity) {
                const intensBase = xyzBase + count * 12;
                if (intensBase + count * 4 <= buffer.byteLength) {
                    intensities = new Float32Array(outPts);
                    let minI = Infinity, maxI = -Infinity;
                    let iidx = 0;
                    for (let i = 0; i < count; i += step) {
                        const val = view.getFloat32(intensBase + i * 4, true);
                        const v   = isFinite(val) ? val : 0;
                        intensities[iidx] = v;
                        if (v < minI) minI = v;
                        if (v > maxI) maxI = v;
                        iidx++;
                    }
                    // 0~1 정규화
                    const iRange = (maxI - minI) || 1;
                    for (let i = 0; i < iidx; i++) {
                        intensities[i] = (intensities[i] - minI) / iRange;
                    }
                    intensities = intensities.subarray(0, idx);
                }
            }

            return {
                positions:   positions.subarray(0, idx * 3),
                colors:      colors.subarray(0, idx * 3),
                intensities  // null 이면 intensity 없음
            };
        } catch (e) {
            console.error('[SlamLiveViewer] binary PC2 parse error:', e);
            return null;
        }
    }

    _updatePointCloud(key, parsed) {
        const THREE = window.THREE;
        if (!this._scene || !THREE || !parsed) return;

        // /kf_node는 구(Sphere) 형태 InstancedMesh로 별도 렌더링
        if (key === 'kf_node') {
            this._updateKfNodeSpheres(parsed);
            return;
        }

        const newCount = parsed.positions.length / 3;
        const isCloud = (key === 'cloud_registered');
        const existing = isCloud ? this._cloudObj : this._pgoMapObj;
        // cloud: 불투명 흰색·크기 확대 / PGO_map: z-rainbow + 반투명
        const pointSize = isCloud ? 0.22 : 0.14;
        const opacity = isCloud ? 1.0 : 0.55;
        const isTransparent = !isCloud;

        // 현재 스캔은 항상 순백 (회색/intensity 잔상 방지)
        if (isCloud) {
            parsed.colors.fill(1.0);
        }

        if (existing && existing.geometry) {
            const posAttr = existing.geometry.getAttribute('position');
            const colAttr = existing.geometry.getAttribute('color');
            if (posAttr && posAttr.array.length >= parsed.positions.length) {
                posAttr.array.set(parsed.positions);
                posAttr.needsUpdate = true;
                colAttr.array.set(parsed.colors);
                colAttr.needsUpdate = true;
                // frustumCulled=false → bounding sphere 생략 (할당/GC 누적 방지)
                _syncPointsGeometry(existing.geometry, newCount, { skipBoundingSphere: true });
                if (existing.material) {
                    existing.material.transparent = isTransparent;
                    existing.material.opacity = opacity;
                    existing.material.depthWrite = !isTransparent;
                    existing.material._baseSize = pointSize;
                    if (this._topView && this._orthoCamera) {
                        const scale = this._getOrthoPixelsPerUnit();
                        existing.material.size = Math.max(1, pointSize * scale);
                    } else {
                        existing.material.size = pointSize;
                    }
                }
                return;
            }
            this._scene.remove(existing);
            existing.geometry.dispose();
            existing.material.dispose();
            if (isCloud) this._cloudObj = null;
            else this._pgoMapObj = null;
        }

        const MAX_PTS = isCloud ? 40000 : 80000;
        const posArray = new Float32Array(MAX_PTS * 3);
        const colArray = new Float32Array(MAX_PTS * 3);
        posArray.set(parsed.positions);
        colArray.set(parsed.colors);

        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.BufferAttribute(posArray, 3));
        geo.setAttribute('color', new THREE.BufferAttribute(colArray, 3));
        _syncPointsGeometry(geo, newCount, { skipBoundingSphere: true });

        const mat = new THREE.PointsMaterial({
            size: pointSize,
            sizeAttenuation: !this._topView,
            vertexColors: true,
            transparent: isTransparent,
            opacity,
            depthWrite: !isTransparent
        });
        mat._baseSize = pointSize;
        if (this._topView && this._orthoCamera) {
            const scale = this._getOrthoPixelsPerUnit();
            mat.size = Math.max(1, pointSize * scale);
        }

        const points = new THREE.Points(geo, mat);
        points.visible = true;
        // 현재 스캔을 맵 위에 그리도록 renderOrder 높게
        points.renderOrder = isCloud ? 2 : 1;
        points.frustumCulled = false;
        if (isCloud) this._cloudObj = points;
        else this._pgoMapObj = points;
        this._scene.add(points);
    }

    // /kf_node: 키프레임 구 + LIO path Line (Path 토픽보다 가볍고, append-only 가능)
    // InstancedMesh·Line 버퍼 재사용 — TubeGeometry dispose/new 제거
    _updateKfNodeSpheres(parsed) {
        const THREE = window.THREE;
        if (!this._scene || !THREE || !parsed) return;

        const count = parsed.positions.length / 3;
        if (count === 0) return;

        // 구·라인 동일 중심 좌표 (LIVE_PATH_MAX trim 제거 → 노드 누락/어긋남 방지)
        this._kfNodePosXyz = parsed.positions.slice(0, count * 3);
        this._pathObj = _syncPathLineFromNodeCenters(
            THREE, this._scene, this._pathLineState, this._kfNodePosXyz, count, 0x00ff44,
            this._pathLineResolutionVec());
        this._pathPoseCount = count;
        // pathPoseBuffer도 동일 좌표로 맞춤 (다른 경로 호환)
        this._pathPoseBuffer.xyz = this._kfNodePosXyz;
        this._pathPoseBuffer.count = count;
        this._pathPoseBuffer.lastMsgCount = count;

        const hasIntensity = parsed.intensities && parsed.intensities.length >= count;
        const dummy = new THREE.Object3D();
        const color = new THREE.Color();
        const instScale = this._computeKfNodeInstanceScale();
        this._lastKfNodeScale = instScale;

        const fillInstances = (mesh) => {
            for (let i = 0; i < count; i++) {
                dummy.position.set(
                    parsed.positions[i * 3],
                    parsed.positions[i * 3 + 1],
                    parsed.positions[i * 3 + 2]
                );
                dummy.scale.setScalar(instScale);
                dummy.updateMatrix();
                mesh.setMatrixAt(i, dummy.matrix);

                if (hasIntensity) {
                    const [r, g, b] = this._rainbowColor(parsed.intensities[i]);
                    color.setRGB(r, g, b);
                } else {
                    color.setRGB(
                        parsed.colors[i * 3],
                        parsed.colors[i * 3 + 1],
                        parsed.colors[i * 3 + 2]
                    );
                }
                mesh.setColorAt(i, color);
            }
            mesh.count = count;
            mesh.instanceMatrix.needsUpdate = true;
            if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
        };

        // 기존 mesh 용량이 충분하면 재사용 (dispose/new 회피)
        if (this._kfNodeObj && this._kfNodeCapacity >= count) {
            fillInstances(this._kfNodeObj);
            return;
        }

        if (this._kfNodeObj) {
            this._scene.remove(this._kfNodeObj);
            if (this._kfNodeObj.geometry) this._kfNodeObj.geometry.dispose();
            if (this._kfNodeObj.material) this._kfNodeObj.material.dispose();
            this._kfNodeObj = null;
            this._kfNodeCapacity = 0;
        }

        const SPHERE_RADIUS = this._kfNodeBaseRadius || 0.7;
        // 여유 용량으로 재할당 빈도 감소 (키프레임 증가 시)
        const capacity = Math.max(count, Math.ceil(count * 1.5));
        const sphereGeo = new THREE.SphereGeometry(SPHERE_RADIUS, 8, 6);
        const sphereMat = new THREE.MeshBasicMaterial({ vertexColors: false });
        const mesh = new THREE.InstancedMesh(sphereGeo, sphereMat, capacity);
        mesh.instanceColor = null;
        fillInstances(mesh);
        mesh.renderOrder = 3;
        mesh.visible = true;
        mesh.frustumCulled = false;

        this._kfNodeObj = mesh;
        this._kfNodeCapacity = capacity;
        this._scene.add(mesh);
    }

    _subscribePC2Binary(topic, key) {
        const viewer = this;
        const sub = _createLiveViewerBackendWs(
            viewer,
            'SlamLiveViewer',
            (ws) => { ws.send(JSON.stringify({ cmd: 'subscribe', topic })); },
            (buffer) => {
                if (key === 'cloud_registered') {
                    const now = performance.now();
                    if (viewer._cloudBusy) return;
                    if (now - (viewer._cloudLastMs || 0) < LIVE_CLOUD_UPDATE_MS) return;
                    viewer._cloudBusy = true;
                    viewer._cloudLastMs = now;
                    if (viewer._cloudBusyWatchdog) clearTimeout(viewer._cloudBusyWatchdog);
                    viewer._cloudBusyWatchdog = setTimeout(() => {
                        viewer._cloudBusy = false;
                        viewer._cloudBusyWatchdog = null;
                    }, 3000);
                    try {
                        let parsed = viewer._parseBinaryPC2(buffer, {
                            maxPts: 40000,
                            fillWhite: true
                        });
                        if (!parsed) return;
                        parsed = _voxelDownsample(
                            parsed.positions, parsed.colors, 0.5, { whiteOutput: true });
                        viewer._updatePointCloud(key, parsed);
                    } finally {
                        viewer._cloudBusy = false;
                        if (viewer._cloudBusyWatchdog) {
                            clearTimeout(viewer._cloudBusyWatchdog);
                            viewer._cloudBusyWatchdog = null;
                        }
                    }
                    return;
                }
                if (key === 'pgo_map') {
                    const now = performance.now();
                    if (viewer._pgoMapBusy) return;
                    if (now - (viewer._pgoMapLastMs || 0) < LIVE_PGO_MAP_UPDATE_MS) return;
                    viewer._pgoMapBusy = true;
                    viewer._pgoMapLastMs = now;
                    if (viewer._pgoMapBusyWatchdog) clearTimeout(viewer._pgoMapBusyWatchdog);
                    viewer._pgoMapBusyWatchdog = setTimeout(() => {
                        viewer._pgoMapBusy = false;
                        viewer._pgoMapBusyWatchdog = null;
                    }, 5000);
                    try {
                        let parsed = viewer._parseBinaryPC2(buffer, { maxPts: 80000 });
                        if (!parsed) return;
                        // PGO 쪽 0.4m 복셀과 맞춤 — 메인스레드 부하 완화
                        parsed = _voxelDownsample(parsed.positions, parsed.colors, 0.4);
                        viewer._updatePointCloud(key, parsed);
                    } finally {
                        viewer._pgoMapBusy = false;
                        if (viewer._pgoMapBusyWatchdog) {
                            clearTimeout(viewer._pgoMapBusyWatchdog);
                            viewer._pgoMapBusyWatchdog = null;
                        }
                    }
                    return;
                }
                if (key === 'kf_node') {
                    const now = performance.now();
                    if (now - (viewer._kfNodeLastMs || 0) < LIVE_KF_NODE_UPDATE_MS) return;
                    viewer._kfNodeLastMs = now;
                }
                let parsed = viewer._parseBinaryPC2(buffer);
                if (!parsed) return;
                viewer._updatePointCloud(key, parsed);
            }
        );
        this._subscriptions.push(sub);
    }

    _subscribePath(topic, key, color) {
        const t = new ROSLIB.Topic({
            ros: this._ros,
            name: topic,
            messageType: getMsgType('nav_msgs/Path', 'nav_msgs/msg/Path'),
            throttle_rate: 200,
            queue_length: 1
        });
        t.subscribe((msg) => {
            const THREE = window.THREE;
            if (!this._scene || !THREE) return;

            const poses = msg.poses || [];
            if (poses.length < 2) return;

            const objKey = (key === 'path') ? '_pathObj' : '_pgoPathObj';
            _disposePathObject(this._scene, this[objKey]);
            this[objKey] = null;

            const points3d = poses.map(p => new THREE.Vector3(
                p.pose.position.x, p.pose.position.y, p.pose.position.z
            ));
            const curve = new THREE.CatmullRomCurve3(points3d);
            const segments = Math.min(poses.length * 2, 400);
            const geo = new THREE.TubeGeometry(curve, segments, 0.025, 4, false);
            const mat = new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide });
            const mesh = new THREE.Mesh(geo, mat);
            mesh.visible = true;
            mesh.frustumCulled = false;
            this[objKey] = mesh;
            this._scene.add(mesh);

            if (key === 'path') this._pathPoseCount = poses.length;
            else this._pgoPathPoseCount = poses.length;
        });
        this._subscriptions.push(t);
    }

    // ── PTH 바이너리 패킷 파싱 ──────────────────────────────────────────────

    _parseBinaryPath(buffer) {
        try {
            const view = new DataView(buffer);
            // magic: 'PTH' = 0x50 0x54 0x48
            if (view.getUint8(0) !== 0x50 || view.getUint8(1) !== 0x54 || view.getUint8(2) !== 0x48) return null;
            let off = 3;
            /* version */ view.getUint8(off++);
            const topicLen = view.getUint32(off, true); off += 4;
            const frameLen = view.getUint32(off, true); off += 4;
            const count    = view.getUint32(off, true); off += 4;
            const dec = new TextDecoder();
            const topic = dec.decode(new Uint8Array(buffer, off, topicLen)); off += topicLen;
            /* frameId */ off += frameLen;
            if (count === 0) return null;
            // offset이 4바이트 정렬이 아닐 수 있으므로 slice()로 복사 후 파싱
            const xyzBuf = new Float32Array(buffer.slice(off), 0, count * 3);
            return { topic, count, xyz: xyzBuf };
        } catch (e) {
            console.error('[SlamLiveViewer] binary Path parse error:', e);
            return null;
        }
    }

    // ── Path 바이너리 WS 구독 ────────────────────────────────────────────────

    _subscribePathBinary(topic, key, color) {
        const viewer = this;
        const sub = _createLiveViewerBackendWs(
            viewer,
            'SlamLiveViewer',
            (ws) => { ws.send(JSON.stringify({ cmd: 'subscribe_path', topic })); },
            (buffer) => {
                const parsed = viewer._parseBinaryPath(buffer);
                if (parsed && viewer._scene) {
                    viewer._updatePathIncremental(key, color, parsed);
                }
            }
        );
        this._subscriptions.push(sub);
    }

    // ── Path Line 동기화 (/PGO_path: 최적화된 Pose Graph용, LIO는 kf_node에서 처리) ─

    _updatePathIncremental(key, color, parsed) {
        const THREE = window.THREE;
        if (!this._scene || !THREE) return;

        const count = parsed.count;
        if (count < 1) return;

        // LIO path는 /kf_node에서 그림 — /PGO_path만 여기서 처리
        if (key === 'path') return;

        _mergePathSnapshot(this._pgoPathPoseBuffer, parsed.xyz, count, LIVE_PATH_MAX_POSES);
        this._pgoPathObj = _syncPathLineFromBuffer(
            THREE, this._scene, this._pgoPathLineState, this._pgoPathPoseBuffer, color,
            this._pathLineResolutionVec());
        this._pgoPathPoseCount = this._pgoPathPoseBuffer.count;
    }

    _rebuildPathFromBuffer(key, color) {
        const THREE = window.THREE;
        if (!this._scene || !THREE) return;
        const res = this._pathLineResolutionVec();
        if (key === 'path') {
            this._pathObj = _syncPathLineFromBuffer(
                THREE, this._scene, this._pathLineState, this._pathPoseBuffer, color, res);
            this._pathPoseCount = this._pathPoseBuffer.count;
            return;
        }
        this._pgoPathObj = _syncPathLineFromBuffer(
            THREE, this._scene, this._pgoPathLineState, this._pgoPathPoseBuffer, color, res);
        this._pgoPathPoseCount = this._pgoPathPoseBuffer.count;
    }

    _subscribeMarker(topic) {
        const t = new ROSLIB.Topic({
            ros: this._ros,
            name: topic,
            messageType: getMsgType('visualization_msgs/Marker', 'visualization_msgs/msg/Marker'),
            throttle_rate: 500,
            queue_length: 1
        });
        t.subscribe((msg) => {
            const THREE = window.THREE;
            if (!this._scene || !THREE) return;

            if (this._loopLineObj) {
                this._scene.remove(this._loopLineObj);
                this._loopLineObj.traverse((child) => {
                    if (child.geometry) child.geometry.dispose();
                    if (child.material) child.material.dispose();
                });
                this._loopLineObj = null;
            }

            const points = msg.points || [];
            if (points.length < 2) return;

            const TUBE_RADIUS = 0.06;
            const TUBE_RADIAL_SEGS = 5;
            const mat = new THREE.MeshBasicMaterial({ color: 0x2288ff });
            const group = new THREE.Group();

            // Marker type 5 = LINE_LIST, type 4 = LINE_STRIP
            const markerType = msg.type || 4;
            if (markerType === 5) {
                // LINE_LIST: points 배열이 쌍(pair)으로 선분을 이룸
                for (let i = 0; i + 1 < points.length; i += 2) {
                    const p0 = new THREE.Vector3(points[i].x, points[i].y, points[i].z);
                    const p1 = new THREE.Vector3(points[i + 1].x, points[i + 1].y, points[i + 1].z);
                    if (p0.distanceTo(p1) < 1e-6) continue;
                    const curve = new THREE.LineCurve3(p0, p1);
                    const geo = new THREE.TubeGeometry(curve, 1, TUBE_RADIUS, TUBE_RADIAL_SEGS, false);
                    group.add(new THREE.Mesh(geo, mat));
                }
            } else {
                // LINE_STRIP: 연속 포인트 → CatmullRomCurve3 + TubeGeometry
                const vecs = points.map(pt => new THREE.Vector3(pt.x, pt.y, pt.z));
                const curve = new THREE.CatmullRomCurve3(vecs);
                const tubularSegs = Math.min(Math.max(vecs.length * 2, 10), LIVE_PATH_MAX_TUBE_SEGMENTS);
                const geo = new THREE.TubeGeometry(curve, tubularSegs, TUBE_RADIUS, TUBE_RADIAL_SEGS, false);
                group.add(new THREE.Mesh(geo, mat));
            }

            if (group.children.length === 0) return;
            group.visible = true;
            this._loopLineObj = group;
            this._scene.add(group);
        });
        this._subscriptions.push(t);
    }

    // /tf 구독 — LocalizationLiveViewer와 동일 (로봇 pose/axis는 /Odometry)
    _subscribeTF(topic) {
        const t = new ROSLIB.Topic({
            ros: this._ros,
            name: topic,
            messageType: getMsgType('tf2_msgs/TFMessage', 'tf2_msgs/msg/TFMessage'),
            throttle_rate: 200,
            queue_length: 1
        });
        t.subscribe((msg) => {
            const THREE = window.THREE;
            if (!this._scene || !THREE) return;
            for (const transform of (msg.transforms || [])) {
                const childId  = transform.child_frame_id;
                const parentId = transform.header.frame_id;
                const trans    = transform.transform.translation;
                const rot      = transform.transform.rotation;

                this._knownFrames.add(childId);
                this._knownFrames.add(parentId);

                // Robot pose/axis는 /Odometry — base_link·body TF axis는 중복 방지
                if (childId === 'base_link' || childId === 'body') continue;

                if (!this._tfObjects[childId]) {
                    const group = new THREE.Group();
                    group.add(new THREE.AxesHelper(0.5));
                    this._tfObjects[childId] = { group };
                    group.visible = true;
                    this._scene.add(group);
                }

                const entry = this._tfObjects[childId];
                entry.group.position.set(trans.x, trans.y, trans.z);
                entry.group.quaternion.set(rot.x, rot.y, rot.z, rot.w);
            }
        });
        this._subscriptions.push(t);
    }

    resetView() {
        if (!this._camera || !this._controls) return;
        this._camera.position.set(0, 0, 20);
        this._camera.up.set(0, 0, 1);
        this._controls.target.set(0, 0, 0);
        this._restoreOrbitControls();
        this._controls.update();
        const toggle = document.getElementById('slam-live-topview-toggle');
        if (toggle && toggle.checked) {
            toggle.checked = false;
            this._topView = false;
            this._savedCameraPos = null;
        }
    }

    _restoreOrbitControls() {
        if (!this._controls) return;
        this._controls.enableDamping = true;
        this._controls.enableRotate = true;
        this._controls.minPolarAngle = 0;
        this._controls.maxPolarAngle = Math.PI;
        if (window.THREE) {
            this._controls.mouseButtons = {
                LEFT: window.THREE.MOUSE.ROTATE,
                MIDDLE: window.THREE.MOUSE.DOLLY,
                RIGHT: window.THREE.MOUSE.PAN
            };
        }
    }

    // Save Map 중 화면만 숨김 (WebSocket 연결 유지 → DDS 재조회 방지)
    _visualHide() {
        const viewerEl = document.getElementById('slam-live-viewer');
        if (viewerEl) viewerEl.style.display = 'none';
    }

    _visualShow() {
        const viewerEl = document.getElementById('slam-live-viewer');
        if (viewerEl && this._visible) viewerEl.style.display = 'block';
    }

    toggleFollow(force) {
        this._followMode = (force !== undefined) ? force : !this._followMode;
        const btn = document.getElementById('slam-live-follow-btn');
        if (btn) btn.classList.toggle('active', this._followMode);
    }

    toggleTopView(enable) {
        this._topView = enable;
        if (!this._perspCamera || !this._controls) return;
        const THREE = window.THREE;

        if (enable) {
            this._savedCameraPos = this._perspCamera.position.clone();
            this._savedCameraUp  = this._perspCamera.up.clone();
            this._savedTarget    = this._controls.target.clone();

            const cx    = this._controls.target.x || 0;
            const cy    = this._controls.target.y || 0;
            const dist  = this._perspCamera.position.distanceTo(this._controls.target) || 100;
            const halfH = Math.max(dist * 0.6, 30);

            const container = document.getElementById('slam-live-canvas-container');
            const cw = container ? container.clientWidth  : 600;
            const ch = container ? container.clientHeight : 480;
            const aspect = cw / ch;

            if (!this._orthoCamera) {
                this._orthoCamera = new THREE.OrthographicCamera(
                    -halfH * aspect, halfH * aspect,
                    halfH, -halfH,
                    -10000, 10000
                );
            } else {
                this._orthoCamera.left   = -halfH * aspect;
                this._orthoCamera.right  =  halfH * aspect;
                this._orthoCamera.top    =  halfH;
                this._orthoCamera.bottom = -halfH;
            }
            this._orthoCamera.position.set(cx, cy, 1000);
            this._orthoCamera.up.set(0, 1, 0);
            this._orthoCamera.lookAt(cx, cy, 0);
            this._orthoCamera.updateProjectionMatrix();

            this._controls.object = this._orthoCamera;
            this._controls.target.set(cx, cy, 0);
            this._controls.enableRotate = false;
            this._controls.enablePan    = true;
            this._controls.enableZoom   = true;
            this._controls.enableDamping = true;
            this._controls.update();

            this._camera = this._orthoCamera;
            this._lastOrthoScale = undefined;
            this._updateOrthoPointSizes();
        } else {
            this._camera = this._perspCamera;
            this._controls.object = this._perspCamera;
            this._restoreOrbitControls();

            // Perspective 복원: 월드 단위 size + sizeAttenuation (needsUpdate 포함)
            // kf_node는 InstancedMesh(MeshBasicMaterial)이므로 size 복원 불필요
            this._ensurePerspectivePointSizes();
            this._perspPointSizesDirty = false;

            if (this._savedCameraPos) {
                this._perspCamera.position.copy(this._savedCameraPos);
                this._perspCamera.up.copy(this._savedCameraUp);
                this._controls.target.copy(this._savedTarget);
                this._savedCameraPos = null;
            } else {
                this._perspCamera.position.set(0, -30, 20);
                this._perspCamera.up.set(0, 0, 1);
                this._controls.target.set(0, 0, 0);
            }
            this._controls.update();
        }

        const headerToggle = document.getElementById('slam-live-topview-toggle');
        if (headerToggle) headerToggle.checked = enable;
        const fsBtn = document.getElementById('slam-live-fs-topview-btn');
        if (fsBtn) fsBtn.classList.toggle('active', enable);
    }

    toggleFullscreen() {
        const container = document.getElementById('slam-live-canvas-container');
        if (!container) return;
        const isFullscreen = !!(document.fullscreenElement || document.webkitFullscreenElement);
        if (isFullscreen) {
            (document.exitFullscreen || document.webkitExitFullscreen).call(document);
        } else {
            (container.requestFullscreen || container.webkitRequestFullscreen).call(container);
        }
    }

    takeSnapshot(scale = 2) {
        if (!this._renderer || !this._scene || !this._camera) return;
        const container = document.getElementById('slam-live-canvas-container');
        if (!container) return;

        const w = container.clientWidth;
        const h = container.clientHeight;
        const sw = w * scale;
        const sh = h * scale;
        const timestamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
        const filename = `slam_live_snapshot_${timestamp}.png`;

        this._renderer.setSize(sw, sh);
        if (this._perspCamera) {
            this._perspCamera.aspect = w / h;
            this._perspCamera.updateProjectionMatrix();
        }
        this._renderer.render(this._scene, this._camera);

        const canvas = this._renderer.domElement;
        const link = document.createElement('a');
        link.href = canvas.toDataURL('image/png');
        link.download = filename;
        link.click();

        this._renderer.setSize(w, h);
        if (this._perspCamera) {
            this._perspCamera.aspect = w / h;
            this._perspCamera.updateProjectionMatrix();
        }
        this._renderer.render(this._scene, this._camera);
    }
}

// SlamLiveViewer 전역 인스턴스 및 래퍼 함수
const slamLiveViewer = new SlamLiveViewer();

function resetSlamLiveViewer()          { slamLiveViewer.resetView(); }
function toggleSlamLiveTopView(checked) { slamLiveViewer.toggleTopView(checked); }
function toggleSlamLiveViewerFullscreen() { slamLiveViewer.toggleFullscreen(); }
function takeSlamLiveSnapshot()         { slamLiveViewer.takeSnapshot(2); }

// ==============================================================
// SLAM Result Viewer
// ==============================================================
class SlamResultViewer {
    constructor(opts) {
        opts = opts || {};
        // 주입형 element-id 매핑 (인스턴스별로 다른 DOM 사용)
        this._ids = opts.ids || {};
        // 로드 사양 (pathsEndpoint, pcdLayers, trajLayers, edges, diff)
        this._spec = opts.spec || {};
        // show(context)로 전달되는 디렉토리 (Save Map 전용)
        this._directory = null;

        this._scene = null;
        this._camera = null;
        this._perspCamera = null;
        this._orthoCamera = null;
        this._renderer = null;
        this._controls = null;
        this._animFrameId = null;
        this._initialized = false;
        this._loaded = false;
        this._loading = false;
        this._pcdObjects = [];
        this._allObjects = [];
        this._pcdPointSize = 0.05;
        this._topView = false;
        this._savedCameraPos = null;
        this._savedCameraUp = null;
        this._savedTarget = null;
        this._diffObjects = [];
        this._diffLoaded = false;
        this._diffEnabled = false;  // 풀스크린 버튼 상태 동기화용
        this._diffPaths = null;
        this._layers = {};
        this._lcObjects = [];
        // 탑뷰 yaw 회전(커스텀 roll) 상태
        this._topViewYaw = 0;
        this._yawDragging = false;
        this._yawLastX = 0;

        // EDL(Eye-Dome Lighting) 2-pass 렌더 파이프라인 상태
        this._edlEnabled = false;
        this._edlTarget = null;
        this._edlScene = null;
        this._edlQuad = null;
        this._edlCamera = null;

        // Height Clip(Z 높이 기준 단면) 상태 — 왼쪽 세로 바(핸들 2개)로 Z 범위 조절
        this._heightClipPlaneHigh = null;   // z <= high 만 유지
        this._heightClipPlaneLow = null;    // z >= low 만 유지
        this._heightClipLow = 0;
        this._heightClipHigh = 0;
        this._heightClipZMin = 0;           // 맵 포인트 실제 최소 Z (슬라이더 하한)
        this._heightClipZMax = 0;           // 맵 포인트 실제 최대 Z (슬라이더 상한)
        this._heightClipDragging = null;    // 'high' | 'low' | null
    }

    _legendRow(name) {
        const root = this._ids.viewer ? document.getElementById(this._ids.viewer) : null;
        if (!root) return null;
        return root.querySelector(`.slam-legend-row[data-layer="${name}"]`);
    }

    _allLegendRows() {
        const root = this._ids.viewer ? document.getElementById(this._ids.viewer) : null;
        if (!root) return [];
        return root.querySelectorAll('.slam-legend-row[data-layer]');
    }

    _waitForThree() {
        return new Promise((resolve) => {
            const check = () => {
                if (window.THREE && window.OrbitControls && window.PCDLoader) {
                    resolve();
                } else {
                    setTimeout(check, 100);
                }
            };
            check();
        });
    }

    async _init() {
        if (this._initialized) return;
        await this._waitForThree();
        const THREE = window.THREE;
        const canvas = document.getElementById(this._ids.canvas);
        if (!canvas) return;

        const container = document.getElementById(this._ids.container);
        const w = container.clientWidth || 600;
        const h = container.clientHeight || 380;

        this._renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
        this._renderer.setPixelRatio(window.devicePixelRatio);
        this._renderer.setSize(w, h);

        this._scene = new THREE.Scene();
        this._scene.background = new THREE.Color(0x1a1a2e);

        this._camera = new THREE.PerspectiveCamera(60, w / h, 0.1, 10000);
        this._camera.position.set(0, -100, 80);
        this._camera.up.set(0, 0, 1);
        this._perspCamera = this._camera;

        this._controls = new window.OrbitControls(this._camera, this._renderer.domElement);
        this._controls.enableDamping = true;
        this._controls.dampingFactor = 0.1;
        this._controls.zoomSpeed = 2.0;
        // 좌: 회전 / 우·휠클릭: 이동(pan) / 휠 스크롤: 줌
        this._controls.mouseButtons = {
            LEFT: THREE.MOUSE.ROTATE,
            MIDDLE: THREE.MOUSE.PAN,
            RIGHT: THREE.MOUSE.PAN
        };

        this._scene.add(new THREE.AxesHelper(5));

        // Height Clip: material.clippingPlanes를 사용하므로 렌더러에서 활성화 필요
        this._renderer.localClippingEnabled = true;
        // high: normal(0,0,-1) → z <= constant 인 포인트만 유지
        this._heightClipPlaneHigh = new THREE.Plane(new THREE.Vector3(0, 0, -1), Infinity);
        // low: normal(0,0,1) → z >= -constant 인 포인트만 유지
        this._heightClipPlaneLow = new THREE.Plane(new THREE.Vector3(0, 0, 1), Infinity);

        this._initialized = true;
        this._bindYawDrag();
        this._bindHeightClipDrag();
        this._startRenderLoop();
    }

    /**
     * 탑뷰에서 마우스 좌드래그 → yaw 회전(커스텀 roll) 핸들러 바인딩.
     * 우드래그(pan)·휠클릭(pan)·휠(zoom)은 OrbitControls가 처리하고, 좌버튼만 전담한다.
     */
    _bindYawDrag() {
        const dom = this._renderer ? this._renderer.domElement : null;
        if (!dom) return;
        const YAW_SENSITIVITY = 0.005; // rad/px

        // OrbitControls가 pointerdown 시 캔버스에 pointer capture를 걸므로,
        // 이후 pointermove는 window가 아닌 캔버스(dom)로 리타게팅된다.
        // → window가 아닌 dom에 pointer 이벤트를 바인딩해야 yaw 드래그가 동작한다.
        dom.addEventListener('pointerdown', (e) => {
            if (!this._topView || e.button !== 0) return;
            this._yawDragging = true;
            this._yawLastX = e.clientX;
        });
        dom.addEventListener('pointermove', (e) => {
            if (!this._yawDragging) return;
            const dx = e.clientX - this._yawLastX;
            this._topViewYaw += dx * YAW_SENSITIVITY;
            this._yawLastX = e.clientX;
        });
        const endDrag = () => { this._yawDragging = false; };
        dom.addEventListener('pointerup', endDrag);
        dom.addEventListener('pointercancel', endDrag);
        window.addEventListener('pointerup', endDrag);
    }

    /**
     * 왼쪽 세로 Height Clip 바 드래그 바인딩 (상단 핸들=최대값, 하단 핸들=최소값).
     * 두 핸들 사이 구간(low <= z <= high)만 렌더링되며, 두 핸들은 서로 교차할 수 없다.
     * 이 뷰어 인스턴스에 해당 DOM(ids.heightClipTrack)이 없으면 아무 것도 하지 않는다.
     */
    _bindHeightClipDrag() {
        const track = document.getElementById(this._ids.heightClipTrack);
        const handleHigh = document.getElementById(this._ids.heightClipHandleMax);
        const handleLow = document.getElementById(this._ids.heightClipHandleMin);
        if (!track || !handleHigh || !handleLow) return;

        const zFromClientY = (clientY) => {
            const rect = track.getBoundingClientRect();
            let frac = rect.height > 0 ? 1 - (clientY - rect.top) / rect.height : 1;
            frac = Math.max(0, Math.min(1, frac));
            return this._heightClipZMin + frac * (this._heightClipZMax - this._heightClipZMin);
        };

        const applyDrag = (which, z) => {
            if (which === 'high') this.setHeightClipHigh(z);
            else this.setHeightClipLow(z);
        };

        // 핸들 각각에 독립적인 드래그 리스너를 바인딩한다(자기 자신에 포인터 캡처).
        // 트랙 리스너 하나에 두 핸들을 모두 위임하면, 실제 마우스 드래그 중 커서가
        // 8px 폭의 트랙 밖으로 살짝 벗어나거나 두 핸들이 겹칠 때 이벤트 타깃 판별이
        // 어긋나 "눌리기만 하고 움직이지 않는" 현상이 발생할 수 있어, 각 핸들이
        // 자신의 포인터 이벤트를 직접 캡처하도록 하여 이를 원천적으로 방지한다.
        const bindHandle = (handle, which) => {
            handle.addEventListener('pointerdown', (e) => {
                e.preventDefault();
                e.stopPropagation();
                this._heightClipDragging = which;
                handle.classList.add('height-clip-handle-active');
                try { handle.setPointerCapture(e.pointerId); } catch (err) { /* noop */ }
                applyDrag(which, zFromClientY(e.clientY));
            });
            handle.addEventListener('pointermove', (e) => {
                if (this._heightClipDragging !== which) return;
                e.preventDefault();
                applyDrag(which, zFromClientY(e.clientY));
            });
            const endDrag = (e) => {
                if (this._heightClipDragging !== which) return;
                this._heightClipDragging = null;
                handle.classList.remove('height-clip-handle-active');
                try { handle.releasePointerCapture(e.pointerId); } catch (err) { /* noop */ }
            };
            handle.addEventListener('pointerup', endDrag);
            handle.addEventListener('pointercancel', endDrag);
        };
        bindHandle(handleHigh, 'high');
        bindHandle(handleLow, 'low');

        // 트랙 배경 클릭(핸들 자체 클릭은 stopPropagation으로 여기까지 오지 않음):
        // 더 가까운 핸들을 해당 위치로 이동시키고 그대로 드래그를 이어간다.
        track.addEventListener('pointerdown', (e) => {
            const z = zFromClientY(e.clientY);
            const which = Math.abs(z - this._heightClipHigh) <= Math.abs(z - this._heightClipLow) ? 'high' : 'low';
            this._heightClipDragging = which;
            try { track.setPointerCapture(e.pointerId); } catch (err) { /* noop */ }
            applyDrag(which, z);
        });
        track.addEventListener('pointermove', (e) => {
            if (!this._heightClipDragging) return;
            applyDrag(this._heightClipDragging, zFromClientY(e.clientY));
        });
        const endTrackDrag = (e) => {
            this._heightClipDragging = null;
            try { track.releasePointerCapture(e.pointerId); } catch (err) { /* noop */ }
        };
        track.addEventListener('pointerup', endTrackDrag);
        track.addEventListener('pointercancel', endTrackDrag);

        // 더블클릭 시 전체 높이 표시로 리셋
        track.addEventListener('dblclick', () => {
            this.setHeightClipHigh(this._heightClipZMax);
            this.setHeightClipLow(this._heightClipZMin);
        });
    }

    _startRenderLoop() {
        const animate = () => {
            this._animFrameId = requestAnimationFrame(animate);
            if (this._controls) this._controls.update();
            // OrthographicCamera 탑뷰 시: zoom 변화에 따라 포인트 픽셀 크기 갱신 + yaw(roll) 적용
            if (this._topView && this._orthoCamera) {
                this._updateOrthoPointSizes();
                const THREE = window.THREE;
                // 시선축(-Z) 기준으로 기준 up(0,1,0)을 yaw만큼 회전 → 화면 roll
                const up = new THREE.Vector3(0, 1, 0).applyAxisAngle(
                    new THREE.Vector3(0, 0, -1), this._topViewYaw);
                this._orthoCamera.up.copy(up);
                this._orthoCamera.lookAt(this._controls.target);
            }
            if (this._renderer && this._scene && this._camera) {
                if (this._edlEnabled) {
                    this._renderEDL();
                } else {
                    this._renderer.render(this._scene, this._camera);
                }
            }
        };
        animate();
    }

    /**
     * EDL 렌더 타깃 및 풀스크린 쿼드를 최초 1회 생성한다.
     *
     * CloudCompare qEDL 플러그인(edl_shade.frag + edl_mix.frag) 알고리즘 충실 구현:
     *  CloudCompare EDL(edl_shade.frag + ccEDLFilter.cpp) 완전 재현:
     *  - fixDepth : NDC → 선형 깊이 → [Zm,ZM] 범위 정규화 → 반전(near=1, far=0)
     *    · perspective: (2·near·far)/(far+near - z_ndc·(far-near)) 역변환
     *    · orthographic: rawDepth·(far-near)+near  (선형 역변환)
     *    · Zm/ZM = 씬 바운딩 박스 8코너를 카메라 공간 투영 → 실제 min/max 깊이
     *  - computeObscurance : Znp = Zn - depth  (CC Light_dir=(0,0,1) 특수 경우)
     *    화면 밖 이웃은 기여 0 (clamp 아닌 범위 체크 → 가장자리 아티팩트 방지)
     *  - 3스케일(1x, 2x, 4x) 각각 계산 후 가중 평균 합성 (edl_mix: A0=1, A1=0.5, A2=0.25)
     *  - Exp_scale(uStr) = 100.0, Dist_to_neighbor_pix = 2.0 (perspective) / 1.2 (ortho)
     *  - 렌더 타깃을 devicePixelRatio 기반 물리 픽셀로 생성 (엣지 깨짐 방지)
     */
    _ensureEDL() {
        if (this._edlTarget) return;
        const THREE = window.THREE;
        const container = document.getElementById(this._ids.container);
        const cssW = container ? (container.clientWidth  || 600) : 600;
        const cssH = container ? (container.clientHeight || 380) : 380;
        // devicePixelRatio 적용: 실제 캔버스 물리 픽셀로 렌더 타깃 생성
        // CSS 픽셀 크기로 만들면 dpr>1 환경에서 UV가 [0,1]을 초과 → 가장자리 깨짐 발생
        const dpr   = this._renderer ? this._renderer.getPixelRatio() : 1;
        const physW = Math.round(cssW * dpr);
        const physH = Math.round(cssH * dpr);

        // depthTexture 타입을 FloatType(32-bit)으로 지정:
        //   기본 UnsignedShortType(16-bit)은 near=0.1/far=10000 구성에서
        //   100m 거리의 0.3m LiDAR 포인트 간격이 0.25 LSB → 구분 불가 → EDL 대비 제로.
        //   FloatType(32-bit)이면 같은 조건에서 약 60 LSB → 명확히 구분 가능.
        const depthTex  = new THREE.DepthTexture(physW, physH);
        depthTex.type   = THREE.FloatType;
        this._edlTarget = new THREE.WebGLRenderTarget(physW, physH, {
            minFilter:    THREE.NearestFilter,
            magFilter:    THREE.NearestFilter,
            format:       THREE.RGBAFormat,
            depthBuffer:  true,
            depthTexture: depthTex,
        });

        this._edlCamera = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);
        this._edlScene  = new THREE.Scene();

        const edlMat = new THREE.ShaderMaterial({
            uniforms: {
                uColor:       { value: this._edlTarget.texture },
                uDepth:       { value: this._edlTarget.depthTexture },
                uSx:          { value: physW },      // 물리 픽셀 너비
                uSy:          { value: physH },      // 물리 픽셀 높이
                uStr:         { value: 200.0 },      // CC Exp_scale (100 기본값 × 2 = 더 강한 대비)
                uRadius:      { value: 3.0 },        // CC Dist_to_neighbor_pix (perspective 기본 3.0, ortho 1.2)
                uCameraNear:  { value: 0.1 },        // 카메라 near (선형화용, 매 프레임 갱신)
                uCameraFar:   { value: 10000.0 },    // 카메라 far  (선형화용, 매 프레임 갱신)
                uZm:          { value: 1.0 },        // 씬 최소 선형 깊이 (정규화용, 매 프레임 갱신)
                uZM:          { value: 100.0 },      // 씬 최대 선형 깊이 (정규화용, 매 프레임 갱신)
                uPerspective: { value: 1.0 },        // 1.0=perspective, 0.0=orthographic
                uA0:          { value: 1.0 },        // 1x 스케일 가중치 (CC A0)
                uA1:          { value: 0.5 },        // 2x 스케일 가중치 (CC A1)
                uA2:          { value: 0.25 },       // 4x 스케일 가중치 (CC A2)
            },
            vertexShader: /* glsl */`
                void main() {
                    gl_Position = vec4(position.xy, 0.0, 1.0);
                }
            `,
            fragmentShader: /* glsl */`
                precision highp float;

                uniform sampler2D uColor;
                uniform sampler2D uDepth;
                uniform float uSx;
                uniform float uSy;
                uniform float uStr;
                uniform float uRadius;
                uniform float uCameraNear;
                uniform float uCameraFar;
                uniform float uZm;
                uniform float uZM;
                uniform float uPerspective;
                uniform float uA0;
                uniform float uA1;
                uniform float uA2;

                // CloudCompare edl_shade.frag 완전 재현
                // fixDepth(): NDC 깊이 → 뷰-스페이스 거리(m) → [Zm,ZM] 정규화 → 반전
                //   perspective : (2·near·far)/(far+near - z_ndc·(far-near))
                //   orthographic: rawDepth·(far-near) + near  (Three.js 부호 규칙)
                //   배경(rawDepth≥0.9999999)은 0 반환 → main()에서 원본 출력
                //   임계값 근거: near=0.1, far=10000 기준 909m→rawDepth≈0.9999,
                //   float DepthTexture 배경(cleared)=정확히 1.0이므로 0.9999999 사용.
                //   0.9999 기준이면 909m 이상 지오메트리(diff 클라우드, 원거리 궤적)가
                //   배경으로 오판정되어 EDL이 전혀 적용 안 됨.
                float fixDepth(float rawDepth) {
                    if (rawDepth >= 0.9999999) return 0.0;
                    float d;
                    if (uPerspective > 0.5) {
                        // 원근 역변환: NDC → 양수 뷰-스페이스 거리(m)
                        float z_ndc = rawDepth * 2.0 - 1.0;
                        d = (2.0 * uCameraFar * uCameraNear)
                          / (uCameraFar + uCameraNear - z_ndc * (uCameraFar - uCameraNear));
                    } else {
                        // 직교 역변환: rawDepth·(far-near)+near = -z_eye → 양수 거리
                        // Three.js 카메라 Z 부호: 앞쪽 z_eye < 0, distance = -z_eye
                        // = rawDepth·(far-near)+near  (near<0 포함 올바른 공식)
                        d = rawDepth * (uCameraFar - uCameraNear) + uCameraNear;
                    }
                    // [Zm, ZM] 정규화 후 반전 (CC: clamp(1-depth, 0, 1))
                    d = clamp((d - uZm) / max(uZM - uZm, 1e-4), 0.0, 1.0);
                    return 1.0 - d;
                }

                // 이웃 한 방향의 obscurance 기여값 계산.
                // CloudCompare Light_dir=(0,0,1) 특수 경우:
                //   P = (0,0,1,-depth), Znp = dot((N_rel,Zn,1),P) = Zn - depth
                // 화면 밖 이웃은 기여 0 (clamp 대신 범위 체크 → 가장자리 아티팩트 방지)
                float neighborObs(vec2 uv, float depth, vec2 nRelPos, float scale) {
                    vec2 nAbs = uv + nRelPos;
                    if (nAbs.x < 0.0 || nAbs.x > 1.0 || nAbs.y < 0.0 || nAbs.y > 1.0)
                        return 0.0;
                    float Zn  = fixDepth(texture2D(uDepth, nAbs).r);
                    return max(0.0, Zn - depth) / scale;
                }

                // CloudCompare computeObscurance: 8방향 이웃 합산
                // Neigh_pos_2D[c] = (cos(c·π/4), sin(c·π/4)) for c=0..7
                float computeObscurance(vec2 uv, float depth, float scale) {
                    float px = scale * uRadius / uSx;
                    float py = scale * uRadius / uSy;
                    float d  = 0.7071;
                    float s  = 0.0;
                    s += neighborObs(uv, depth, vec2( px,    0.0 ), scale);
                    s += neighborObs(uv, depth, vec2( d*px,  d*py), scale);
                    s += neighborObs(uv, depth, vec2( 0.0,   py  ), scale);
                    s += neighborObs(uv, depth, vec2(-d*px,  d*py), scale);
                    s += neighborObs(uv, depth, vec2(-px,    0.0 ), scale);
                    s += neighborObs(uv, depth, vec2(-d*px, -d*py), scale);
                    s += neighborObs(uv, depth, vec2( 0.0,  -py  ), scale);
                    s += neighborObs(uv, depth, vec2( d*px, -d*py), scale);
                    return s;
                }

                void main() {
                    vec2  uv       = gl_FragCoord.xy / vec2(uSx, uSy);
                    vec4  color    = texture2D(uColor, uv);
                    float rawDepth = texture2D(uDepth, uv).r;
                    float depth    = fixDepth(rawDepth);

                    // 배경 판정: rawDepth로 체크 (float DepthTexture cleared=1.0 정확)
                    // 0.9999999 사용 → 909m 이상 지오메트리를 배경으로 오판정하던 버그 수정
                    if (rawDepth >= 0.9999999) {
                        gl_FragColor = vec4(color.rgb, 1.0);
                        return;
                    }

                    // 3스케일 obscurance → 각 스케일에 CC Exp_scale 적용 → 가중 평균
                    // CC edl_mix: C = (A0·C1 + A1·C2 + A2·C4) / (A0+A1+A2)
                    float f1 = exp(-uStr * computeObscurance(uv, depth, 1.0));
                    float f2 = exp(-uStr * computeObscurance(uv, depth, 2.0));
                    float f4 = exp(-uStr * computeObscurance(uv, depth, 4.0));
                    float totalW = uA0 + uA1 + uA2;
                    float shade  = (uA0 * f1 + uA1 * f2 + uA2 * f4) / totalW;

                    // 대비 강화: pow(shade, 2.0) → 중간 음영을 제곱으로 압축
                    // shade=0.7 → 0.49, shade=0.5 → 0.25, shade=0.1 → 0.01
                    shade = pow(clamp(shade, 0.0, 1.0), 2.0);

                    // CC: gl_FragData[0] = vec4(shade * rgb, 1.0)
                    gl_FragColor = vec4(color.rgb * shade, 1.0);
                }
            `,
            depthTest:  false,
            depthWrite: false,
        });

        this._edlQuad = new THREE.Mesh(new THREE.PlaneGeometry(2, 2), edlMat);
        this._edlScene.add(this._edlQuad);
    }

    /**
     * EDL 2-pass 렌더. CloudCompare ccEDLFilter::shade() 완전 재현.
     * Pass1: 씬 → _edlTarget (color + depth)
     * Pass2: EDL 풀스크린 쿼드 → 스크린
     */
    _renderEDL() {
        if (!this._edlTarget || !this._edlScene || !this._edlCamera) return;
        const renderer = this._renderer;
        const u = this._edlQuad.material.uniforms;

        // 카메라 타입: perspective vs orthographic
        const isPerspective = this._camera.isPerspectiveCamera || false;
        u.uPerspective.value = isPerspective ? 1.0 : 0.0;
        // CC 기본값: perspective=3.0, ortho=1.2
        u.uRadius.value = isPerspective ? 3.0 : 1.2;

        // 카메라 near/far (깊이 선형화용)
        u.uCameraNear.value = this._camera.near;
        u.uCameraFar.value  = this._camera.far;

        // Zm/ZM: 씬 구(sphere) 기반 깊이 범위 (뷰포인트 독립적, stale 없음)
        //   bbox 8코너 방식은 카메라가 씬에 가깝거나 내부일 때 뒤쪽 코너가 제외되어
        //   dMin=near(0.1), dMax=150m+ 처럼 range가 15배 이상 과대해져 대비가 소실됨.
        //   구 방식(카메라-씬 중심 거리 ± 씬 반경)은 항상 안정적인 범위를 보장.
        //   또한 matrixWorldInverse를 쓰지 않으므로 1프레임 lag 버그도 없음.
        {
            const { center, maxDim } = this._computeBounds();
            const sceneRadius = Math.max(maxDim * 0.5, 1.0);
            const camDist     = this._camera.position.distanceTo(center);
            const zmRaw = camDist - sceneRadius;
            const zMRaw = camDist + sceneRadius;
            // near 아래로 내려가지 않도록 보정 (near가 음수인 ortho 포함)
            const nearFloor = Math.max(0.01, this._camera.near);
            const zm = Math.max(nearFloor, zmRaw);
            const zM = Math.max(zm + 0.1, Math.min(this._camera.far, zMRaw));
            u.uZm.value = zm;
            u.uZM.value = zM;
        }

        // Pass 1: scene → off-screen render target
        renderer.setRenderTarget(this._edlTarget);
        renderer.render(this._scene, this._camera);

        // Pass 2: EDL quad → screen
        renderer.setRenderTarget(null);
        renderer.render(this._edlScene, this._edlCamera);
    }

    /**
     * EDL ON/OFF 토글. 최초 활성화 시 리소스를 생성한다.
     * @param {boolean} enabled
     */
    setEDL(enabled) {
        this._edlEnabled = enabled;
        if (enabled) {
            this._ensureEDL();
        }
        this._syncControlStates();
    }

    /**
     * 헤더 체크박스와 풀스크린 오버레이 버튼 등 컨트롤 UI 상태를 현재 뷰어 상태와 동기화.
     * 풀스크린 오버레이 버튼은 추후 Task 3에서 ids에 추가된 후 활성화된다.
     */
    _syncControlStates() {
        const edlToggle = document.getElementById(this._ids.edlToggle);
        if (edlToggle) edlToggle.checked = this._edlEnabled;

        const topViewToggle = document.getElementById(this._ids.topViewToggle);
        if (topViewToggle) topViewToggle.checked = this._topView;

        // 풀스크린 오버레이 버튼 동기화 (DOM이 존재할 때만)
        const fsEdlBtn = document.getElementById(this._ids.fsEdlBtn);
        if (fsEdlBtn) fsEdlBtn.classList.toggle('active', this._edlEnabled);

        const fsTopViewBtn = document.getElementById(this._ids.fsTopViewBtn);
        if (fsTopViewBtn) fsTopViewBtn.classList.toggle('active', this._topView);

        const fsDiffBtn = document.getElementById(this._ids.fsDiffBtn);
        if (fsDiffBtn) fsDiffBtn.classList.toggle('active', this._diffEnabled);
    }

    /**
     * OrthographicCamera에서 1 월드단위 = 몇 픽셀인지 계산.
     * OrthographicCamera는 sizeAttenuation 미적용 → material.size를 픽셀 단위로 직접 제어해야 함.
     */
    _getOrthoPixelsPerUnit() {
        if (!this._orthoCamera || !this._renderer) return 40;
        const frustumH = (this._orthoCamera.top - this._orthoCamera.bottom) / (this._orthoCamera.zoom || 1);
        const pixelH = this._renderer.domElement.height || 380;
        if (frustumH <= 0) return 40;
        return pixelH / frustumH;
    }

    /**
     * 탑뷰(OrthographicCamera) 시 모든 PCD material.size를 현재 zoom 기준 픽셀 크기로 갱신.
     * baseSize(_baseSize, 월드 단위 = Pt Size 슬라이더 값)에 비례 → 줌에 따라 자연스럽게 확대/축소.
     */
    _updateOrthoPointSizes() {
        const scale = this._getOrthoPixelsPerUnit();
        const apply = (obj, fallback) => {
            if (!obj || !obj.material) return;
            const baseSize = (obj.material._baseSize !== undefined) ? obj.material._baseSize : fallback;
            obj.material.size = Math.max(1, Math.min(baseSize * scale, 64));
        };
        for (const obj of this._pcdObjects) apply(obj, this._pcdPointSize);
        for (const obj of this._diffObjects) apply(obj, this._pcdPointSize * 1.2);
    }

    _clearScene() {
        for (const obj of this._allObjects) {
            if (this._scene) this._scene.remove(obj);
            if (obj.geometry) obj.geometry.dispose();
            if (obj.material) {
                if (Array.isArray(obj.material)) {
                    obj.material.forEach(m => m.dispose());
                } else {
                    obj.material.dispose();
                }
            }
        }
        this._allObjects = [];
        this._pcdObjects = [];
        this._lcObjects = [];
        this._layers = {};
    }

    _addToScene(obj, layerName) {
        this._scene.add(obj);
        this._allObjects.push(obj);
        if (layerName) {
            if (!this._layers[layerName]) this._layers[layerName] = [];
            this._layers[layerName].push(obj);
        }
    }

    _makeLine(positions, color) {
        const THREE = window.THREE;
        if (window.Line2 && window.LineGeometry && window.LineMaterial) {
            const geo = new window.LineGeometry();
            geo.setPositions(positions);
            const c = document.getElementById(this._ids.container);
            const mat = new window.LineMaterial({
                color,
                linewidth: 3,
                resolution: new THREE.Vector2(c ? c.clientWidth : 600, c ? c.clientHeight : 380),
            });
            return new window.Line2(geo, mat);
        }
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
        return new THREE.Line(geo, new THREE.LineBasicMaterial({ color }));
    }

    _makeNodes(positions, color) {
        const THREE = window.THREE;
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));

        if (!SlamResultViewer._circleTexture) {
            const canvas = document.createElement('canvas');
            canvas.width = 64;
            canvas.height = 64;
            const ctx = canvas.getContext('2d');
            ctx.beginPath();
            ctx.arc(32, 32, 30, 0, Math.PI * 2);
            ctx.fillStyle = '#ffffff';
            ctx.fill();
            SlamResultViewer._circleTexture = new THREE.CanvasTexture(canvas);
        }

        // renderOrder=2: PCD(0) → diff 클라우드(1) → trajectory(2) 순서로 렌더링
        // depthTest: true(기본, LEQUAL) — diff(renderOrder=1)가 먼저 depth 기록,
        //   trajectory(renderOrder=2)는 같은 깊이라도 LEQUAL(<=)로 통과 → 항상 diff 위에 표시
        // transparent:true + depthWrite:true(기본) → EDL depth 텍스처 정상 기록
        const pts = new THREE.Points(geo, new THREE.PointsMaterial({
            color,
            size: 16,
            sizeAttenuation: false,
            map: SlamResultViewer._circleTexture,
            alphaTest: 0.5,
            transparent: true,
            // depthTest: true (기본, LEQUAL) → EDL 정확한 깊이 기록 + trajectory 항상 위에 표시
        }));
        pts.renderOrder = 2;
        return pts;
    }

    _posesToFlat(poses) {
        const flat = [];
        for (const p of poses) {
            flat.push(p.x, p.y, p.z);
        }
        return flat;
    }

    async _fetchPoses(path) {
        try {
            const resp = await fetch('/api/slam/poses?path=' + encodeURIComponent(path));
            if (!resp.ok) return [];
            const data = await resp.json();
            return data.poses || [];
        } catch (e) {
            console.error('SlamResultViewer: failed to fetch poses:', e);
            return [];
        }
    }

    async _loadPCD(path) {
        const PCDLoaderCls = window.PCDLoader;
        if (!PCDLoaderCls) {
            console.error('SlamResultViewer: PCDLoader not available on window');
            return null;
        }
        return new Promise((resolve) => {
            const loader = new PCDLoaderCls();
            loader.load(
                '/api/slam/pcd?path=' + encodeURIComponent(path),
                (points) => resolve(points),
                undefined,
                (err) => { console.error('SlamResultViewer: PCD load error:', err); resolve(null); }
            );
        });
    }

    // pose({x,y,z,qx,qy,qz,qw}) -> tf 변환 행렬 (long_term_mapping.cpp의 createTransformMatrix와 동일)
    _poseToMatrix(pose) {
        const THREE = window.THREE;
        const q = new THREE.Quaternion(pose.qx, pose.qy, pose.qz, pose.qw);
        const m = new THREE.Matrix4();
        m.makeRotationFromQuaternion(q);
        m.setPosition(pose.x, pose.y, pose.z);
        return m;
    }

    /**
     * pcl::io::savePCDFileBinary가 생성하는 uncompressed binary PCD를 빠르게 파싱한다.
     * 벤더 PCDLoader와 달리 (1) 헤더 영역만 텍스트로 디코딩하고(전체 바이너리 포인트
     * 데이터를 문자열로 변환하지 않음), (2) 값을 배열에 push하는 대신 DataView로 미리
     * 할당한 Float32Array에 직접 기록한다. 다중 스캔(수백 개) 파일을 반복 로드하는
     * Map1/Map2 누적 시 병목이 되는 부분이라 별도로 최적화함.
     * ascii/binary_compressed 등 비표준 포맷이면 null을 반환한다(호출측에서 폴백).
     */
    _parsePCDFast(buf) {
        try {
            const headLen = Math.min(buf.byteLength, 4096);
            const headText = new TextDecoder('utf-8').decode(new Uint8Array(buf, 0, headLen));

            // 벤더 PCDLoader.parseHeader()와 동일한 헤더 길이 계산 방식(오프셋 산출 방식을 그대로 따라야 함)
            const dataSearch = headText.search(/[\r\n]DATA\s(\S*)\s/i);
            if (dataSearch < 0) return null;
            const dataMatch = /[\r\n]DATA\s(\S*)\s/i.exec(headText.substr(dataSearch - 1));
            if (!dataMatch) return null;
            if (dataMatch[1].toLowerCase() !== 'binary') return null; // ascii/binary_compressed는 폴백

            const headerLen = dataMatch[0].length + dataSearch;
            const headerStr = headText.substr(0, headerLen).replace(/#.*/gi, '');

            const fieldsM = /FIELDS (.*)/i.exec(headerStr);
            const sizeM = /SIZE (.*)/i.exec(headerStr);
            const countM = /COUNT (.*)/i.exec(headerStr);
            const widthM = /WIDTH (.*)/i.exec(headerStr);
            const heightM = /HEIGHT (.*)/i.exec(headerStr);
            const pointsM = /POINTS (.*)/i.exec(headerStr);
            if (!fieldsM || !sizeM) return null;

            const fields = fieldsM[1].trim().split(/\s+/);
            const sizes = sizeM[1].trim().split(/\s+/).map(Number);
            const counts = countM ? countM[1].trim().split(/\s+/).map(Number) : fields.map(() => 1);
            const width = widthM ? parseInt(widthM[1], 10) : 0;
            const height = heightM ? parseInt(heightM[1], 10) : 1;
            const numPoints = pointsM ? parseInt(pointsM[1], 10) : (width * (height || 1));
            if (!numPoints || numPoints <= 0) return null;

            const offsets = {};
            let rowSize = 0;
            for (let i = 0; i < fields.length; i++) {
                offsets[fields[i]] = rowSize;
                rowSize += sizes[i] * (counts[i] || 1);
            }
            if (offsets.x === undefined || offsets.y === undefined || offsets.z === undefined) return null;

            const dv = new DataView(buf, headerLen);
            const positions = new Float32Array(numPoints * 3);
            const hasIntensity = offsets.intensity !== undefined;
            const intensities = hasIntensity ? new Float32Array(numPoints) : null;

            const offX = offsets.x, offY = offsets.y, offZ = offsets.z, offI = offsets.intensity;
            for (let i = 0, row = 0; i < numPoints; i++, row += rowSize) {
                positions[i * 3] = dv.getFloat32(row + offX, true);
                positions[i * 3 + 1] = dv.getFloat32(row + offY, true);
                positions[i * 3 + 2] = dv.getFloat32(row + offZ, true);
                if (hasIntensity) intensities[i] = dv.getFloat32(row + offI, true);
            }

            return { positions, intensities, count: numPoints };
        } catch (e) {
            console.warn('SlamResultViewer: fast PCD parse failed:', e);
            return null;
        }
    }

    /** path의 PCD를 { positions: Float32Array, intensities: Float32Array|null, count } 형태로 가져온다.
     * 빠른 경로(binary) 실패 시 벤더 PCDLoader(ascii/binary_compressed 등)로 폴백한다. */
    async _fetchPCDRaw(path) {
        try {
            const resp = await fetch('/api/slam/pcd?path=' + encodeURIComponent(path));
            if (!resp.ok) return null;
            const buf = await resp.arrayBuffer();
            const fast = this._parsePCDFast(buf);
            if (fast) return fast;
        } catch (e) {
            console.warn('SlamResultViewer: PCD fetch failed:', path, e);
            return null;
        }
        // 폴백: 표준 PCDLoader (ascii/binary_compressed 등 비표준 케이스)
        const pts = await this._loadPCD(path);
        if (!pts || !pts.geometry || !pts.geometry.attributes.position) return null;
        const posAttr = pts.geometry.attributes.position;
        const intAttr = pts.geometry.attributes.intensity;
        const result = {
            positions: new Float32Array(posAttr.array),
            intensities: intAttr ? new Float32Array(intAttr.array) : null,
            count: posAttr.count,
        };
        pts.geometry.dispose();
        return result;
    }

    // 3D 격자 인덱스(ix,iy,iz)를 문자열 concat 없이 하나의 정수 키로 패킹 (Map 해싱 비용 절감)
    // BASE=2^17(131072), OFFSET=2^16 → voxel_size=0.4m 기준 ±약 26km 범위 커버
    _voxelKeyNum(ix, iy, iz) {
        const BASE = 131072;
        const OFFSET = 65536;
        return ((ix + OFFSET) * BASE + (iy + OFFSET)) * BASE + (iz + OFFSET);
    }

    /**
     * Map1/Map2 서버사이드 누적 맵 요청. 백엔드(Python/numpy)가 스캔 로드 + tf 변환 +
     * 누적 + 복셀화를 전부 처리한 뒤, 완성된 point cloud를 raw float32(x,y,z 반복)
     * 바이너리 하나로 응답한다(요청 1회로 끝남). numpy 미설치 등으로 실패하면 서버가
     * 빈 응답을 주므로 null을 반환해 호출측이 클라이언트 사이드 폴백을 쓰도록 한다.
     */
    async _fetchAccumulatedMapFromServer(posesPath, scansDir, voxelSize) {
        try {
            const url = '/api/slam/accumulated_map'
                + '?poses_path=' + encodeURIComponent(posesPath)
                + '&scans_dir=' + encodeURIComponent(scansDir)
                + '&voxel_size=' + encodeURIComponent(voxelSize);
            const resp = await fetch(url);
            if (!resp.ok) return null;
            const buf = await resp.arrayBuffer();
            if (!buf || buf.byteLength === 0) return null;
            return new Float32Array(buf);
        } catch (e) {
            console.warn('SlamResultViewer: server-side accumulated map fetch failed, falling back:', e);
            return null;
        }
    }

    /**
     * 클라이언트 사이드 폴백: posesPath(optimized_poses.txt)의 각 pose로 scansDir/{i}.pcd
     * 스캔을 tf 변환한 뒤 누적하고, voxelSize 간격의 격자 중심(centroid) 다운샘플을 적용해
     * Float32Array(x,y,z 반복)를 만든다. (pcl::transformPointCloud + accumulate +
     * pcl::VoxelGrid와 동등한 처리) 스캔 파일들은 CONCURRENCY개씩 병렬로 fetch하고,
     * 각 스캔은 도착 즉시 tf 변환 후 voxel 그리드에 누적(=즉시 복셀화)하여 원본 포인트를
     * 따로 쌓아두지 않는다. 서버사이드 처리(_fetchAccumulatedMapFromServer)가 실패했을
     * 때만 사용된다.
     */
    async _loadAccumulatedScanMapClientSide(posesPath, scansDir, voxelSize) {
        const poses = await this._fetchPoses(posesPath);
        if (!poses || poses.length === 0) return null;

        const inv = 1.0 / Math.max(voxelSize, 1e-6);
        const voxelMap = new Map();
        const CONCURRENCY = 16;

        for (let start = 0; start < poses.length; start += CONCURRENCY) {
            const end = Math.min(start + CONCURRENCY, poses.length);
            const batch = [];
            for (let i = start; i < end; i++) batch.push(this._fetchPCDRaw(scansDir + i + '.pcd'));
            const results = await Promise.all(batch);

            for (let bi = 0; bi < results.length; bi++) {
                const raw = results[bi];
                if (!raw || !raw.positions || raw.count === 0) continue;

                const m = this._poseToMatrix(poses[start + bi]);
                const e = m.elements;
                const pos = raw.positions;
                for (let k = 0; k < raw.count; k++) {
                    const bx = pos[k * 3], by = pos[k * 3 + 1], bz = pos[k * 3 + 2];
                    const x = e[0] * bx + e[4] * by + e[8] * bz + e[12];
                    const y = e[1] * bx + e[5] * by + e[9] * bz + e[13];
                    const z = e[2] * bx + e[6] * by + e[10] * bz + e[14];
                    const key = this._voxelKeyNum(Math.floor(x * inv), Math.floor(y * inv), Math.floor(z * inv));
                    let acc = voxelMap.get(key);
                    if (!acc) {
                        acc = { x: 0, y: 0, z: 0, n: 0 };
                        voxelMap.set(key, acc);
                    }
                    acc.x += x; acc.y += y; acc.z += z; acc.n += 1;
                }
            }
        }

        if (voxelMap.size === 0) return null;

        const positions = new Float32Array(voxelMap.size * 3);
        let idx = 0;
        for (const acc of voxelMap.values()) {
            positions[idx++] = acc.x / acc.n;
            positions[idx++] = acc.y / acc.n;
            positions[idx++] = acc.z / acc.n;
        }
        return positions;
    }

    /**
     * Map1/Map2 누적 맵을 로드하여 THREE.Points로 만든다. 우선 백엔드(numpy)가 전체를
     * 처리한 결과(요청 1회)를 시도하고, 실패한 경우에만 브라우저에서 스캔 파일들을
     * 병렬로 fetch·복셀화하는 클라이언트 사이드 방식으로 폴백한다.
     */
    async _loadAccumulatedScanMap(posesPath, scansDir, voxelSize, color) {
        const THREE = window.THREE;
        let positions = await this._fetchAccumulatedMapFromServer(posesPath, scansDir, voxelSize);
        if (!positions) {
            positions = await this._loadAccumulatedScanMapClientSide(posesPath, scansDir, voxelSize);
        }
        if (!positions || positions.length === 0) return null;

        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        const pcd = new THREE.Points(geo, new THREE.PointsMaterial({
            color, size: this._pcdPointSize, sizeAttenuation: true, vertexColors: false,
            clippingPlanes: [this._heightClipPlaneHigh, this._heightClipPlaneLow],
        }));
        pcd.material._baseSize = this._pcdPointSize;
        return pcd;
    }

    /**
     * intensity 필드를 가진 단일 PCD(StaticMap.pcd)를 splits 정의(value별 색상/레이어)에 따라
     * 여러 개의 THREE.Points로 분리하여 씬에 추가한다.
     * splits: [{ value, color, layer }]
     */
    async _loadIntensitySplitPCD(path, splits) {
        const THREE = window.THREE;
        const raw = await this._fetchPCDRaw(path);
        if (!raw || !raw.positions || raw.count === 0) return;
        if (!raw.intensities) {
            console.warn('SlamResultViewer: intensity field not found in', path);
            return;
        }

        const valueToBucket = new Map();
        splits.forEach((s, bi) => valueToBucket.set(s.value, bi));

        // 1차 패스: 버킷별 포인트 개수를 먼저 센다 (push 기반 동적 배열 확장/박싱 비용 제거를 위해
        // 정확한 크기의 Float32Array를 미리 할당하기 위함)
        const counts = new Array(splits.length).fill(0);
        const bucketOf = new Int8Array(raw.count).fill(-1);
        for (let i = 0; i < raw.count; i++) {
            const bi = valueToBucket.get(Math.round(raw.intensities[i]));
            if (bi === undefined) continue;
            bucketOf[i] = bi;
            counts[bi]++;
        }

        const bucketArrays = counts.map(c => new Float32Array(c * 3));
        const cursors = new Array(splits.length).fill(0);
        // 2차 패스: 미리 할당된 배열에 좌표를 직접 기록
        for (let i = 0; i < raw.count; i++) {
            const bi = bucketOf[i];
            if (bi < 0) continue;
            const dst = bucketArrays[bi];
            const c = cursors[bi];
            const base = i * 3;
            dst[c] = raw.positions[base];
            dst[c + 1] = raw.positions[base + 1];
            dst[c + 2] = raw.positions[base + 2];
            cursors[bi] = c + 3;
        }

        splits.forEach((s, bi) => {
            if (bucketArrays[bi].length === 0) return;
            const geo = new THREE.BufferGeometry();
            geo.setAttribute('position', new THREE.BufferAttribute(bucketArrays[bi], 3));
            const pts = new THREE.Points(geo, new THREE.PointsMaterial({
                color: s.color, size: this._pcdPointSize, sizeAttenuation: true, vertexColors: false,
                clippingPlanes: [this._heightClipPlaneHigh, this._heightClipPlaneLow],
            }));
            pts.material._baseSize = this._pcdPointSize;
            this._addToScene(pts, s.layer);
            this._pcdObjects.push(pts);
        });
    }

    _computeBounds() {
        const THREE = window.THREE;
        const center = new THREE.Vector3(0, 0, 0);
        let maxDim = 100;
        if (this._allObjects.length > 0) {
            const box = new THREE.Box3();
            for (const obj of this._allObjects) {
                box.expandByObject(obj);
            }
            if (!box.isEmpty()) {
                const size = new THREE.Vector3();
                box.getSize(size);
                maxDim = Math.max(size.x, size.y, size.z) || maxDim;
                box.getCenter(center);
            }
        }
        return { center, maxDim };
    }

    _fitCamera() {
        if (!this._scene) return;
        const { center, maxDim } = this._computeBounds();
        // FOV 기반으로 맵이 화면에 가깝게 들어오도록 거리 계산 (factor < 1 → 더 가까이)
        const fov = (this._camera.fov || 60) * Math.PI / 180;
        const dist = ((maxDim / 2) / Math.tan(fov / 2)) * 0.6;
        this._camera.position.set(center.x, center.y - dist * 0.85, center.z + dist * 0.5);
        this._camera.up.set(0, 0, 1);
        this._controls.target.copy(center);
        this._controls.update();
    }

    // OrthographicCamera 기반 탑뷰: 완전 수직 XY 평면 투영(투시 왜곡 없음)
    _applyTopView() {
        const THREE = window.THREE;
        this._topViewYaw = 0;
        const { center, maxDim } = this._computeBounds();
        const halfH = Math.max((maxDim * 0.5) * 0.55, 5);

        const container = document.getElementById(this._ids.container);
        const cw = container ? (container.clientWidth || 600) : 600;
        const ch = container ? (container.clientHeight || 380) : 380;
        const aspect = cw / ch;

        if (!this._orthoCamera) {
            this._orthoCamera = new THREE.OrthographicCamera(
                -halfH * aspect, halfH * aspect, halfH, -halfH, -10000, 10000
            );
        } else {
            this._orthoCamera.left = -halfH * aspect;
            this._orthoCamera.right = halfH * aspect;
            this._orthoCamera.top = halfH;
            this._orthoCamera.bottom = -halfH;
            this._orthoCamera.zoom = 1;
        }
        // 씬 정중앙 바로 위에 배치, Y-up으로 짐벌락 방지 (loc 뷰어 방식)
        this._orthoCamera.position.set(center.x, center.y, center.z + 1000);
        this._orthoCamera.up.set(0, 1, 0);
        this._orthoCamera.lookAt(center.x, center.y, center.z);
        this._orthoCamera.updateProjectionMatrix();

        // OrbitControls를 OrthographicCamera로 전환 (수직 탑뷰 → 회전 비활성, 팬/줌만)
        this._controls.object = this._orthoCamera;
        this._controls.target.set(center.x, center.y, center.z);
        this._controls.enableRotate = false;
        this._controls.enablePan = true;
        this._controls.enableZoom = true;
        this._controls.panSpeed = 1.5;
        // 좌버튼은 자체 yaw 핸들러가 전담 → OrbitControls는 좌버튼 무시. 우/휠클릭은 이동(pan)
        this._controls.mouseButtons = {
            LEFT: null,
            MIDDLE: THREE.MOUSE.PAN,
            RIGHT: THREE.MOUSE.PAN
        };
        this._controls.update();

        this._camera = this._orthoCamera;
        // PCD 머티리얼을 픽셀 단위 제어로 전환 후 즉시 보정
        for (const obj of this._pcdObjects) {
            if (obj.material) obj.material.sizeAttenuation = false;
        }
        for (const obj of this._diffObjects) {
            if (obj.material) obj.material.sizeAttenuation = false;
        }
        this._updateOrthoPointSizes();
    }

    async load() {
        if (this._loaded || this._loading) return;
        this._loading = true;
        const loadingEl = document.getElementById(this._ids.loading);
        if (loadingEl) loadingEl.style.display = 'block';
        this._clearScene();

        try {
            await this._loadAndRender();
            this._setupHeightClip();
            this._loaded = true;
        } catch (e) {
            console.error('SlamResultViewer: load failed:', e);
        } finally {
            this._loading = false;
            if (loadingEl) loadingEl.style.display = 'none';
        }
    }

    _buildPathsEndpoint() {
        const spec = this._spec;
        let endpoint = spec.pathsEndpoint || '/api/slam/result_paths';
        if (this._directory) {
            endpoint += (endpoint.indexOf('?') >= 0 ? '&' : '?') + 'directory=' + encodeURIComponent(this._directory);
        }
        return endpoint;
    }

    async _loadAndRender() {
        const THREE = window.THREE;
        const spec = this._spec;
        const paths = await apiCall(this._buildPathsEndpoint());
        if (!paths) throw new Error('Failed to fetch result paths');

        // PCD 레이어 로드
        for (const pl of (spec.pcdLayers || [])) {
            const path = paths[pl.pathKey];
            if (!path) continue;
            const pcd = await this._loadPCD(path);
            if (!pcd) continue;
            pcd.material = new THREE.PointsMaterial({
                color: pl.color, size: this._pcdPointSize, sizeAttenuation: true, vertexColors: false,
                clippingPlanes: [this._heightClipPlaneHigh, this._heightClipPlaneLow],
            });
            pcd.material._baseSize = this._pcdPointSize;
            this._addToScene(pcd, pl.layer);
            this._pcdObjects.push(pcd);
        }

        // Map1/Map2: 각 맵 디렉토리의 optimized_poses.txt(궤적) + Scans/의 개별 스캔을
        // pose로 tf 변환 후 누적, voxel_size로 복셀화하여 시각화
        const voxelSize = parseFloat(paths[spec.voxelSizeKey || 'voxel_size']) || 0.4;
        for (const al of (spec.accumulatedLayers || [])) {
            const posesPath = paths[al.posesKey];
            const scansDir = paths[al.scansDirKey];
            if (!posesPath || !scansDir) continue;
            const pcd = await this._loadAccumulatedScanMap(posesPath, scansDir, voxelSize, al.color);
            if (!pcd) continue;
            this._addToScene(pcd, al.layer);
            this._pcdObjects.push(pcd);
        }

        // 병합 정적맵(StaticMap.pcd)을 intensity 값으로 분리하여 시각화 (1=Map1 출신, 2=Map2 출신)
        for (const isl of (spec.intensitySplitLayers || [])) {
            const path = paths[isl.pathKey];
            if (!path) continue;
            await this._loadIntensitySplitPCD(path, isl.splits);
        }

        // 궤적 레이어 로드 (라인 또는 노드)
        const posesByKey = {};
        for (const tl of (spec.trajLayers || [])) {
            const path = paths[tl.pathKey];
            if (!path) continue;
            const poses = await this._fetchPoses(path);
            posesByKey[tl.pathKey] = poses;
            const flat = this._posesToFlat(poses);
            if (flat.length >= 3) {
                if (tl.asNodes) {
                    this._addToScene(this._makeNodes(flat, tl.color), tl.layer);
                } else {
                    this._addToScene(this._makeLine(flat, tl.color), tl.layer);
                    // 라인 + 노드(구) 동시 표시: 노드 색은 궤적 색과 동일, 크기는 라인과 구별
                    if (tl.withNodes) {
                        this._addToScene(this._makeNodes(flat, tl.color), tl.layer);
                    }
                }
            }
        }

        // 루프 클로저 (edges)
        if (spec.edges) {
            const edgesPath = paths[spec.edges.pathKey];
            const poses = posesByKey[spec.edges.posesFromKey];
            if (edgesPath && poses && poses.length > 0) {
                await this._loadLoopClosures(edgesPath, poses, spec.edges.color);
            }
        }

        this._fitCamera();
    }

    async _loadLoopClosures(edgesPath, poses, color = 0xff2266) {
        try {
            const resp = await fetch('/api/slam/edges?path=' + encodeURIComponent(edgesPath));
            if (!resp.ok) {
                console.warn('SlamResultViewer: edges API error', resp.status);
                return;
            }
            const data = await resp.json();
            if (!data.success || !data.loop_closures || data.loop_closures.length === 0) {
                console.warn('SlamResultViewer: no loop closures', data);
                return;
            }
            console.log(`SlamResultViewer: ${data.loop_closures.length} loop closures found`);

            const THREE = window.THREE;
            const maxIdx = poses.length - 1;
            const positions = [];
            for (const edge of data.loop_closures) {
                const fi = edge.from_idx;
                const ti = edge.to_idx;
                if (fi < 0 || fi > maxIdx || ti < 0 || ti > maxIdx) continue;
                const from = poses[fi];
                const to   = poses[ti];
                positions.push(from.x, from.y, from.z, to.x, to.y, to.z);
            }
            if (positions.length === 0) {
                console.warn('SlamResultViewer: all loop closure indices out of range');
                return;
            }

            let lcLines;
            if (window.LineSegments2 && window.LineSegmentsGeometry && window.LineMaterial) {
                const lsGeo = new window.LineSegmentsGeometry();
                lsGeo.setPositions(positions);
                const container = document.getElementById(this._ids.container);
                const lsMat = new window.LineMaterial({
                    color,
                    linewidth: 3,
                    transparent: true,
                    opacity: 0.85,
                    depthTest: false,
                    resolution: new THREE.Vector2(
                        container ? container.clientWidth : 600,
                        container ? container.clientHeight : 380
                    ),
                });
                lcLines = new window.LineSegments2(lsGeo, lsMat);
            } else {
                const geo = new THREE.BufferGeometry();
                geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
                lcLines = new THREE.LineSegments(
                    geo,
                    new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.85, depthTest: false })
                );
            }
            lcLines.renderOrder = 2;
            this._addToScene(lcLines, 'loopclosure');
            this._lcObjects.push(lcLines);
        } catch (e) {
            console.warn('SlamResultViewer: loop closure load failed:', e);
        }
    }

    _resizeRenderer() {
        if (!this._renderer) return;
        const container = document.getElementById(this._ids.container);
        if (!container) return;
        const w = container.clientWidth;
        const h = container.clientHeight;
        if (w > 0 && h > 0) {
            this._renderer.setSize(w, h);
            const aspect = w / h;
            if (this._topView && this._orthoCamera) {
                const halfH = this._orthoCamera.top;
                this._orthoCamera.left = -halfH * aspect;
                this._orthoCamera.right = halfH * aspect;
                this._orthoCamera.updateProjectionMatrix();
            } else if (this._perspCamera) {
                this._perspCamera.aspect = aspect;
                this._perspCamera.updateProjectionMatrix();
            }
            // EDL 렌더 타깃: devicePixelRatio 반영한 물리 픽셀 크기로 동기화
            if (this._edlTarget) {
                const dpr   = this._renderer.getPixelRatio();
                const physW = Math.round(w * dpr);
                const physH = Math.round(h * dpr);
                this._edlTarget.setSize(physW, physH);
                if (this._edlQuad) {
                    const u = this._edlQuad.material.uniforms;
                    u.uSx.value = physW;
                    u.uSy.value = physH;
                }
            }
            // Line2/LineSegments2(LIO·PGO 궤적, Loop Closure) 머티리얼은 생성 시점의
            // container 크기로 screen-space 두께를 계산하는 resolution uniform을 갖는다.
            // show() 중 컨테이너가 display:none 상태에서 로드되면 0x0으로 캡처되어 선이
            // 보이지 않게 되므로, 실제 표시 크기를 알게 되는 매 리사이즈마다 갱신한다.
            for (const obj of this._allObjects) {
                if (obj.material && obj.material.resolution && obj.material.resolution.isVector2) {
                    obj.material.resolution.set(w, h);
                }
            }
        }
    }

    async show(context) {
        if (context && context.directory) {
            this._directory = context.directory;
        }
        // 이미 준비 중인 show()가 있으면 그 결과를 그대로 기다린다.
        // (폴링 등으로 show()가 중복 호출돼도 로드 작업이 중복 실행되지 않도록 방지)
        if (this._showPromise) return this._showPromise;

        const gen = (this._showGen = (this._showGen || 0) + 1);
        this._showPromise = (async () => {
            // 컨테이너를 숨긴 채로 초기화·로드를 모두 마친 뒤에야 화면에 표시한다.
            // → 빈 화면/로딩 표시가 보이는 대신, 시각화 준비가 끝나면 한 번에 뷰어가 나타난다.
            await this._init();
            await this.load();
            // 로드하는 동안 hide()/hideAndReset()이 호출됐다면(예: Exit) 뒤늦게 다시 표시하지 않는다.
            if (this._showGen !== gen) return;
            const viewerEl = document.getElementById(this._ids.viewer);
            if (viewerEl) viewerEl.style.display = 'block';
            this._resizeRenderer();
        })();
        try {
            await this._showPromise;
        } finally {
            this._showPromise = null;
        }
    }

    hide() {
        // 진행 중인 show()가 있다면 완료되더라도 뷰어를 다시 표시하지 않도록 무효화한다.
        this._showGen = (this._showGen || 0) + 1;
        const viewerEl = document.getElementById(this._ids.viewer);
        if (viewerEl) viewerEl.style.display = 'none';
    }

    _restoreOrbitControls() {
        // 탑뷰 yaw(roll) 상태 초기화
        this._topViewYaw = 0;
        this._yawDragging = false;
        // PerspectiveCamera로 복귀 + PCD 머티리얼 sizeAttenuation(월드 크기) 복원
        if (this._perspCamera) {
            this._camera = this._perspCamera;
            this._controls.object = this._perspCamera;
        }
        const restoreMat = (obj, fallback) => {
            if (!obj || !obj.material) return;
            obj.material.sizeAttenuation = true;
            obj.material.size = (obj.material._baseSize !== undefined) ? obj.material._baseSize : fallback;
            obj.material.needsUpdate = true;
        };
        for (const obj of this._pcdObjects) restoreMat(obj, this._pcdPointSize);
        for (const obj of this._diffObjects) restoreMat(obj, this._pcdPointSize * 1.2);

        this._controls.enableRotate = true;
        this._controls.enablePan = true;
        this._controls.enableZoom = true;
        this._controls.panSpeed = 3.0;
        this._controls.zoomSpeed = 2.0;
        this._controls.minPolarAngle = 0;
        this._controls.maxPolarAngle = Math.PI;
        // 좌: 회전 / 우·휠클릭: 이동(pan) / 휠 스크롤: 줌
        this._controls.mouseButtons = {
            LEFT: window.THREE.MOUSE.ROTATE,
            MIDDLE: window.THREE.MOUSE.PAN,
            RIGHT: window.THREE.MOUSE.PAN
        };
    }

    resetView() {
        const toggle = document.getElementById(this._ids.topViewToggle);
        if (toggle && toggle.checked) {
            toggle.checked = false;
            this._topView = false;
            this._restoreOrbitControls();
        }
        this._savedCameraPos = null;
        this._fitCamera();
    }

    toggleTopView(enabled) {
        this._topView = enabled;
        if (enabled) {
            // 전환 직전의 PerspectiveCamera 상태 저장
            this._savedCameraPos = this._perspCamera.position.clone();
            this._savedCameraUp  = this._perspCamera.up.clone();
            this._savedTarget    = this._controls.target.clone();
            this._applyTopView();
        } else {
            this._restoreOrbitControls();
            if (this._savedCameraPos) {
                this._perspCamera.position.copy(this._savedCameraPos);
                this._perspCamera.up.copy(this._savedCameraUp);
                this._controls.target.copy(this._savedTarget);
                this._savedCameraPos = null;
                this._controls.update();
            } else {
                this._fitCamera();
            }
        }
        this._syncControlStates();
    }

    togglePCDs(visible) {
        for (const obj of this._pcdObjects) {
            obj.visible = visible;
        }
    }

    async _loadDiffPCD(path, color, layerName) {
        const THREE = window.THREE;
        if (!path) return;
        const points = await this._loadPCD(path);
        if (!points) return;
        // Z-파이팅 방지: depthTest:false는 EDL depth 텍스처 기록을 방해하므로 사용하지 않음.
        //   대신 renderOrder=1 + Three.js 기본 LEQUAL depth test 활용:
        //   base PCD(renderOrder=0) 이후 렌더링 시 same_depth(50m) <= same_depth(50m) = true
        //   → diff PCD가 항상 base PCD를 덮어써 z-파이팅 제거.
        //   transparent:false(기본) → opaque 큐, depth 정상 기록 → EDL 정상 동작.
        points.material = new THREE.PointsMaterial({
            color,
            size: this._pcdPointSize * 1.2,
            sizeAttenuation: true,
            vertexColors: false,
            clippingPlanes: [this._heightClipPlaneHigh, this._heightClipPlaneLow],
            // depthTest: true (기본, LEQUAL) — renderOrder=1로 base 이후 렌더링, 같은 깊이 LEQUAL 통과
            // transparent: false (기본) — opaque 큐, EDL depth 텍스처 올바르게 기록
        });
        points.material._baseSize = this._pcdPointSize * 1.2;
        points.renderOrder = 1;   // base 클라우드(renderOrder=0) 이후 렌더링
        this._scene.add(points);
        this._diffObjects.push(points);
        if (layerName) {
            if (!this._layers[layerName]) this._layers[layerName] = [];
            this._layers[layerName].push(points);
        }
    }

    async toggleDiffPCDs(enabled) {
        this._diffEnabled = enabled;
        const diffLegend = document.getElementById(this._ids.diffLegend);
        if (enabled) {
            if (!this._diffLoaded) {
                const loadingEl = document.getElementById(this._ids.loading);
                if (loadingEl) loadingEl.style.display = 'block';
                try {
                    if (!this._diffPaths) {
                        this._diffPaths = await apiCall(this._buildPathsEndpoint());
                    }
                    const paths = this._diffPaths;
                    // PD / ND — 없을 수 있으므로 각각 독립 try (색상은 slamResultViewer 12색 팔레트 참조)
                    try { if (paths && paths.pd_pcd) await this._loadDiffPCD(paths.pd_pcd, 0xff1919, 'pd'); } catch (e) { console.warn('PD.pcd not available'); }
                    try { if (paths && paths.nd_pcd) await this._loadDiffPCD(paths.nd_pcd, 0x8c19ff, 'nd'); } catch (e) { console.warn('ND.pcd not available'); }
                    // FirstUE / SecondUE — 없을 수 있으므로 각각 독립 try
                    try { if (paths && paths.first_ue_pcd) await this._loadDiffPCD(paths.first_ue_pcd, 0xff198c, 'firstue'); } catch (e) { console.warn('FirstUE.pcd not available'); }
                    try { if (paths && paths.second_ue_pcd) await this._loadDiffPCD(paths.second_ue_pcd, 0x8cff19, 'secondue'); } catch (e) { console.warn('SecondUE.pcd not available'); }
                    this._diffLoaded = true;
                } finally {
                    if (loadingEl) loadingEl.style.display = 'none';
                }
            } else {
                for (const obj of this._diffObjects) {
                    obj.visible = true;
                }
                this._restoreDiffLayerVisuals();
            }
            if (diffLegend) diffLegend.style.display = 'block';
        } else {
            for (const obj of this._diffObjects) {
                obj.visible = false;
            }
            if (diffLegend) diffLegend.style.display = 'none';
        }
        this._syncControlStates();
    }

    _restoreDiffLayerVisuals() {
        for (const name of ['pd', 'nd', 'firstue', 'secondue']) {
            const row = this._legendRow(name);
            if (row) {
                const active = row.dataset.active !== 'false';
                const objs = this._layers[name] || [];
                for (const obj of objs) { obj.visible = active; }
            }
        }
    }

    _clearDiff() {
        for (const obj of this._diffObjects) {
            if (this._scene) this._scene.remove(obj);
            if (obj.geometry) obj.geometry.dispose();
            if (obj.material) obj.material.dispose();
        }
        this._diffObjects = [];
        this._diffLoaded = false;
        this._diffEnabled = false;
        this._diffPaths = null;
        ['pd', 'nd', 'firstue', 'secondue'].forEach(k => delete this._layers[k]);
    }

    toggleLayer(name) {
        const objs = this._layers[name];
        const row = this._legendRow(name);
        if (!objs || objs.length === 0) return;
        const nowVisible = objs[0].visible;
        const newVisible = !nowVisible;
        for (const obj of objs) { obj.visible = newVisible; }
        if (row) row.dataset.active = String(newVisible);
    }

    setPointSize(size) {
        this._pcdPointSize = size;
        // baseSize(월드 단위)는 항상 갱신. 퍼스펙티브일 때만 size 직접 반영
        // (탑뷰 ortho는 렌더 루프 _updateOrthoPointSizes가 zoom 기준으로 매 프레임 보정)
        for (const obj of this._pcdObjects) {
            if (obj.material) {
                obj.material._baseSize = size;
                if (!this._topView) {
                    obj.material.size = size;
                    obj.material.needsUpdate = true;
                }
            }
        }
        for (const obj of this._diffObjects) {
            if (obj.material) {
                obj.material._baseSize = size * 1.2;
                if (!this._topView) {
                    obj.material.size = size * 1.2;
                    obj.material.needsUpdate = true;
                }
            }
        }
        const label = document.getElementById(this._ids.pointSizeLabel);
        if (label) label.textContent = size.toFixed(2) + ' m';
    }

    /**
     * 로드된 PCD 맵 포인트들의 Z(높이) 최소/최대값을 계산하여 Height Clip 바 UI를 초기화한다.
     * 해당 뷰어 인스턴스에 Height Clip DOM(ids.heightClipOverlay)이 없으면 아무 것도 하지 않는다.
     */
    _setupHeightClip() {
        const overlay = document.getElementById(this._ids.heightClipOverlay);
        if (!overlay) return;

        let zMin = Infinity;
        let zMax = -Infinity;
        for (const obj of this._pcdObjects) {
            if (!obj.geometry) continue;
            obj.geometry.computeBoundingBox();
            const bb = obj.geometry.boundingBox;
            if (!bb) continue;
            zMin = Math.min(zMin, bb.min.z);
            zMax = Math.max(zMax, bb.max.z);
        }

        if (!isFinite(zMin) || !isFinite(zMax) || zMax - zMin < 1e-3) {
            overlay.style.display = 'none';
            return;
        }

        this._heightClipZMin = zMin;
        this._heightClipZMax = zMax;
        overlay.style.display = 'flex';

        const maxLabel = document.getElementById(this._ids.heightClipMaxLabel);
        const minLabel = document.getElementById(this._ids.heightClipMinLabel);
        if (maxLabel) maxLabel.textContent = zMax.toFixed(2);
        if (minLabel) minLabel.textContent = zMin.toFixed(2);

        // 기본값: 전체 범위(low=zMin, high=zMax)에서 시작 — 전체 맵 표시
        // (직접 대입: setHeightClipHigh/Low의 상호 clamp 로직이 이전 값(0 등)을 참조하지 않도록)
        this._heightClipLow = zMin;
        this._heightClipHigh = zMax;
        if (this._heightClipPlaneHigh) this._heightClipPlaneHigh.constant = zMax;
        if (this._heightClipPlaneLow) this._heightClipPlaneLow.constant = -zMin;
        this._updateHeightClipHandles();
        this._updateHeightClipLabel();
    }

    /**
     * Height Clip 상한(High) 값을 설정한다. 이 값보다 높은(z > high) 포인트는 클리핑되어 숨겨진다.
     * 최소값 핸들(low)보다 아래로 내려갈 수 없다 (두 핸들 교차 방지).
     * @param {number} z
     */
    setHeightClipHigh(z) {
        z = Math.max(this._heightClipZMin, Math.min(this._heightClipZMax, z));
        z = Math.max(z, this._heightClipLow);
        this._heightClipHigh = z;
        if (this._heightClipPlaneHigh) this._heightClipPlaneHigh.constant = z;
        this._updateHeightClipHandles();
        this._updateHeightClipLabel();
    }

    /**
     * Height Clip 하한(Low) 값을 설정한다. 이 값보다 낮은(z < low) 포인트는 클리핑되어 숨겨진다.
     * 최대값 핸들(high)보다 위로 올라갈 수 없다 (두 핸들 교차 방지).
     * @param {number} z
     */
    setHeightClipLow(z) {
        z = Math.max(this._heightClipZMin, Math.min(this._heightClipZMax, z));
        z = Math.min(z, this._heightClipHigh);
        this._heightClipLow = z;
        if (this._heightClipPlaneLow) this._heightClipPlaneLow.constant = -z;
        this._updateHeightClipHandles();
        this._updateHeightClipLabel();
    }

    _updateHeightClipHandles() {
        const handleHigh = document.getElementById(this._ids.heightClipHandleMax);
        const handleLow = document.getElementById(this._ids.heightClipHandleMin);
        const fill = document.getElementById(this._ids.heightClipFill);
        const range = this._heightClipZMax - this._heightClipZMin;
        const fracFor = (z) => range > 1e-6 ? Math.max(0, Math.min(1, (z - this._heightClipZMin) / range)) : 1;
        const highFrac = fracFor(this._heightClipHigh);
        const lowFrac = fracFor(this._heightClipLow);
        const highTopPct = (1 - highFrac) * 100;
        const lowTopPct = (1 - lowFrac) * 100;
        if (handleHigh) handleHigh.style.top = highTopPct + '%';
        if (handleLow) handleLow.style.top = lowTopPct + '%';
        if (fill) {
            fill.style.top = highTopPct + '%';
            fill.style.height = (lowTopPct - highTopPct) + '%';
        }
    }

    _updateHeightClipLabel() {
        const current = document.getElementById(this._ids.heightClipLabel);
        if (current) {
            current.textContent = this._heightClipLow.toFixed(2) + ' ~ ' + this._heightClipHigh.toFixed(2) + ' m';
        }
    }

    _resetAllLegendRows() {
        this._allLegendRows().forEach(row => {
            row.dataset.active = 'true';
        });
    }

    takeSnapshot(scale = 2) {
        if (!this._renderer || !this._scene || !this._camera) return;
        const container = document.getElementById(this._ids.container);
        if (!container) return;

        const w = container.clientWidth;
        const h = container.clientHeight;
        const sw = w * scale;
        const sh = h * scale;
        const timestamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
        const filename = `slam_snapshot_${timestamp}.png`;

        this._renderer.setSize(sw, sh);
        if (this._perspCamera) {
            this._perspCamera.aspect = w / h;
            this._perspCamera.updateProjectionMatrix();
        }

        if (this._edlEnabled && this._edlTarget) {
            // EDL ON: 고해상도 타깃으로 확장 후 2-pass 렌더
            this._edlTarget.setSize(sw, sh);
            if (this._edlQuad) {
                const u = this._edlQuad.material.uniforms;
                u.uSx.value = sw;
                u.uSy.value = sh;
            }
            this._renderEDL();
        } else {
            this._renderer.render(this._scene, this._camera);
        }

        const canvas = this._renderer.domElement;
        canvas.toBlob((blob) => {
            // 원래 해상도로 복원
            this._renderer.setSize(w, h);
            if (this._perspCamera) {
                this._perspCamera.aspect = w / h;
                this._perspCamera.updateProjectionMatrix();
            }
            if (this._edlTarget) {
                const dpr   = this._renderer.getPixelRatio();
                const physW = Math.round(w * dpr);
                const physH = Math.round(h * dpr);
                this._edlTarget.setSize(physW, physH);
                if (this._edlQuad) {
                    const u = this._edlQuad.material.uniforms;
                    u.uSx.value = physW;
                    u.uSy.value = physH;
                }
            }

            if (!blob) return;
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = filename;
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
            URL.revokeObjectURL(url);
        }, 'image/png');
    }

    hideAndReset() {
        this.hide();
        this._loaded = false;
        this._loading = false;
        this._clearScene();
        this._resetAllLegendRows();
        this._topView = false;
        this._topViewYaw = 0;
        this._yawDragging = false;
        this._savedCameraPos = null;
        this._savedCameraUp = null;
        this._savedTarget = null;
        if (this._controls) {
            this._restoreOrbitControls();
        }
        const topViewToggle = document.getElementById(this._ids.topViewToggle);
        if (topViewToggle) topViewToggle.checked = false;
        this._clearDiff();
        const diffToggle = document.getElementById(this._ids.diffToggle);
        if (diffToggle) diffToggle.checked = false;
        const diffLegend = document.getElementById(this._ids.diffLegend);
        if (diffLegend) diffLegend.style.display = 'none';
        const slider = document.getElementById(this._ids.pointSizeSlider);
        if (slider) {
            slider.value = '0.05';
            this._pcdPointSize = 0.05;
            const label = document.getElementById(this._ids.pointSizeLabel);
            if (label) label.textContent = '0.05 m';
        }
        // EDL 상태 초기화 (리소스는 재사용을 위해 보존)
        this._edlEnabled = false;
        // Height Clip 상태 초기화
        this._heightClipZMin = 0;
        this._heightClipZMax = 0;
        this._heightClipLow = 0;
        this._heightClipHigh = 0;
        this._heightClipDragging = null;
        if (this._heightClipPlaneHigh) this._heightClipPlaneHigh.constant = Infinity;
        if (this._heightClipPlaneLow) this._heightClipPlaneLow.constant = Infinity;
        const heightClipOverlay = document.getElementById(this._ids.heightClipOverlay);
        if (heightClipOverlay) heightClipOverlay.style.display = 'none';
        this._syncControlStates();
    }
}

// Multi-Session SLAM Optimization 결과 뷰어
const slamResultViewer = new SlamResultViewer({
    ids: {
        viewer: 'slam-result-viewer',
        canvas: 'slam-result-canvas',
        container: 'slam-result-canvas-container',
        loading: 'slam-result-loading',
        topViewToggle: 'slam-viewer-topview-toggle',
        edlToggle: 'slam-viewer-edl-toggle',
        fsTopViewBtn: 'slam-fs-topview-btn',
        fsEdlBtn: 'slam-fs-edl-btn',
        fsDiffBtn: 'slam-fs-diff-btn',
        pointSizeSlider: 'slam-point-size-slider',
        pointSizeLabel: 'slam-point-size-label',
        diffToggle: 'slam-viewer-diff-toggle',
        diffLegend: 'slam-diff-legend-rows',
        heightClipOverlay: 'slam-height-clip-overlay',
        heightClipTrack: 'slam-height-clip-track',
        heightClipHandleMax: 'slam-height-clip-handle-max',
        heightClipHandleMin: 'slam-height-clip-handle-min',
        heightClipFill: 'slam-height-clip-fill',
        heightClipLabel: 'slam-height-clip-current-label',
        heightClipMaxLabel: 'slam-height-clip-max-label',
        heightClipMinLabel: 'slam-height-clip-min-label',
    },
    spec: {
        pathsEndpoint: '/api/slam/result_paths',
        // Map1/Map2: 각 맵 디렉토리의 optimized_poses.txt(궤적) + Scans/ 개별 스캔을
        // pose로 tf 변환 후 누적, voxel_size로 복셀화하여 시각화 (long_term_mapping과 동일 처리)
        // 색상 팔레트 규칙: 12개 레이어(지도 점군 4·diff 점군 4·궤적 3·Loop Closure 1)가
        // 서로 겹치지 않도록 색상환(Hue) 360°를 12등분(30° 간격)하여 배정 (모두 고유 색상)
        accumulatedLayers: [
            { posesKey: 'map1_poses', scansDirKey: 'map1_scans_dir', color: 0x198cff, layer: 'map1' },
            { posesKey: 'map2_poses', scansDirKey: 'map2_scans_dir', color: 0x19ff8c, layer: 'map2' },
        ],
        voxelSizeKey: 'voxel_size',
        // Merge Map1/Map2: long_term_mapping이 세션별로 생성한 FirstMap.pcd/SecondMap.pcd를
        // 그대로 로드하여 시각화 (병합 좌표계로 정렬된 세션별 원본 맵, optimize() 실행 시 생성됨)
        pcdLayers: [
            { pathKey: 'map1_pcd', color: 0xff19ff, layer: 'mergemap1' },
            { pathKey: 'map2_pcd', color: 0xff8c19, layer: 'mergemap2' },
        ],
        trajLayers: [
            { pathKey: 'map1_poses', color: 0xffff19, layer: 'map1traj', asNodes: true },
            { pathKey: 'map2_poses', color: 0x1919ff, layer: 'map2traj', asNodes: true },
            { pathKey: 'output_poses', color: 0x19ffff, layer: 'outputtraj', asNodes: true },
        ],
        edges: { pathKey: 'output_edges', posesFromKey: 'output_poses', color: 0x19ff19 },
        diff: true,
    },
});

function resetSlamResultView() {
    slamResultViewer.resetView();
}

function toggleSlamPCDs(visible) {
    slamResultViewer.togglePCDs(visible);
}

function toggleSlamTopView(checked) {
    slamResultViewer.toggleTopView(checked);
}

function toggleSlamDiff(checked) {
    slamResultViewer.toggleDiffPCDs(checked);
}

function toggleSlamEDL(checked) {
    slamResultViewer.setEDL(checked);
}

function setSlamPointSize(size) {
    slamResultViewer.setPointSize(size);
}

function toggleSlamLayer(name) {
    slamResultViewer.toggleLayer(name);
}

function takeSlamSnapshot() {
    slamResultViewer.takeSnapshot(2);
}

// ==============================================================
// Save Map 결과 뷰어 (LiDAR SLAM 서브탭)
// ==============================================================
const saveMapResultViewer = new SlamResultViewer({
    ids: {
        viewer: 'savemap-result-viewer',
        canvas: 'savemap-result-canvas',
        container: 'savemap-result-canvas-container',
        loading: 'savemap-result-loading',
        topViewToggle: 'savemap-viewer-topview-toggle',
        edlToggle: 'savemap-viewer-edl-toggle',
        fsTopViewBtn: 'savemap-fs-topview-btn',
        fsEdlBtn: 'savemap-fs-edl-btn',
        pointSizeSlider: 'savemap-point-size-slider',
        pointSizeLabel: 'savemap-point-size-label',
        heightClipOverlay: 'savemap-height-clip-overlay',
        heightClipTrack: 'savemap-height-clip-track',
        heightClipHandleMax: 'savemap-height-clip-handle-max',
        heightClipHandleMin: 'savemap-height-clip-handle-min',
        heightClipFill: 'savemap-height-clip-fill',
        heightClipLabel: 'savemap-height-clip-current-label',
        heightClipMaxLabel: 'savemap-height-clip-max-label',
        heightClipMinLabel: 'savemap-height-clip-min-label',
    },
    spec: {
        pathsEndpoint: '/api/slam/save_map_result',
        pcdLayers: [
            { pathKey: 'lio_map_pcd', color: 0xffdd00, layer: 'liomap' },
            { pathKey: 'optimized_map_pcd', color: 0xff3333, layer: 'optimized' },
            { pathKey: 'static_map_pcd', color: 0x44ff88, layer: 'static' },
        ],
        trajLayers: [
            { pathKey: 'lio_poses', color: 0xff8800, layer: 'lio', asNodes: false, withNodes: true },
            { pathKey: 'pgo_poses', color: 0xffffff, layer: 'pgo', asNodes: false, withNodes: true },
        ],
        edges: { pathKey: 'edges', posesFromKey: 'pgo_poses', color: 0x2288ff },
    },
});

function resetSaveMapResultView() {
    saveMapResultViewer.resetView();
}

function toggleSaveMapTopView(checked) {
    saveMapResultViewer.toggleTopView(checked);
}

function toggleSaveMapEDL(checked) {
    saveMapResultViewer.setEDL(checked);
}

function toggleSaveMapLayer(name) {
    saveMapResultViewer.toggleLayer(name);
}

function setSaveMapPointSize(size) {
    saveMapResultViewer.setPointSize(size);
}

function takeSaveMapSnapshot() {
    saveMapResultViewer.takeSnapshot(2);
}

function toggleSaveMapResultFullscreen() {
    const container = document.getElementById('savemap-result-canvas-container');
    if (!container) return;
    const isFullscreen = !!(document.fullscreenElement || document.webkitFullscreenElement);
    if (isFullscreen) {
        (document.exitFullscreen || document.webkitExitFullscreen).call(document);
    } else {
        (container.requestFullscreen || container.webkitRequestFullscreen).call(container);
    }
}

function _updateSaveMapFullscreenIcon(isFullscreen) {
    const expand = document.getElementById('savemap-fullscreen-icon-expand');
    const collapse = document.getElementById('savemap-fullscreen-icon-collapse');
    if (expand) expand.style.display = isFullscreen ? 'none' : '';
    if (collapse) collapse.style.display = isFullscreen ? '' : 'none';
}

// ==============================================================
// LocalizationLiveViewer 전역 인스턴스 및 래퍼 함수
// ==============================================================
const locLiveViewer = new LocalizationLiveViewer();

function resetLocViewer()              { locLiveViewer.resetView(); }
function toggleLocTopView(checked)     { locLiveViewer.toggleTopView(checked); }
function toggleLocViewerFullscreen()   { locLiveViewer.toggleFullscreen(); }
function takeLocSnapshot()             { locLiveViewer.takeSnapshot(2); }
function onLocFixedFrameFocus()        { locLiveViewer.onFixedFrameFocus(); }
function onLocFixedFrameInput(value)   { locLiveViewer.onFixedFrameInput(value); }
function onLocFixedFrameBlur(event)    { locLiveViewer.onFixedFrameBlur(event); }
function toggleLocFixedFrameDropdown() { locLiveViewer.toggleFixedFrameDropdown(); }

function _updateLocFullscreenIcon(isFullscreen) {
    const expand   = document.getElementById('loc-fullscreen-icon-expand');
    const collapse = document.getElementById('loc-fullscreen-icon-collapse');
    if (expand)   expand.style.display   = isFullscreen ? 'none' : '';
    if (collapse) collapse.style.display = isFullscreen ? '' : 'none';
}

function toggleSlamFullscreen() {
    const container = document.getElementById('slam-result-canvas-container');
    if (!container) return;
    const isFullscreen = !!(document.fullscreenElement || document.webkitFullscreenElement);
    if (isFullscreen) {
        (document.exitFullscreen || document.webkitExitFullscreen).call(document);
    } else {
        (container.requestFullscreen || container.webkitRequestFullscreen).call(container);
    }
}

function _updateSlamFullscreenIcon(isFullscreen) {
    const expand = document.getElementById('slam-fullscreen-icon-expand');
    const collapse = document.getElementById('slam-fullscreen-icon-collapse');
    if (expand) expand.style.display = isFullscreen ? 'none' : '';
    if (collapse) collapse.style.display = isFullscreen ? '' : 'none';
}

function toggle3DViewerFullscreen() {
    const container = document.getElementById('3d-viewer-container');
    if (!container) return;
    const isFullscreen = !!(document.fullscreenElement || document.webkitFullscreenElement);
    if (isFullscreen) {
        (document.exitFullscreen || document.webkitExitFullscreen).call(document);
    } else {
        (container.requestFullscreen || container.webkitRequestFullscreen).call(container);
    }
}

function _updateViewerFullscreenIcon(isFullscreen) {
    const expand = document.getElementById('viewer-fullscreen-icon-expand');
    const collapse = document.getElementById('viewer-fullscreen-icon-collapse');
    if (expand) expand.style.display = isFullscreen ? 'none' : '';
    if (collapse) collapse.style.display = isFullscreen ? '' : 'none';
}

document.addEventListener('fullscreenchange', () => {
    const isFullscreen = !!document.fullscreenElement;
    const isSlamFullscreen    = isFullscreen && document.fullscreenElement?.id === 'slam-result-canvas-container';
    const isSaveMapFullscreen = isFullscreen && document.fullscreenElement?.id === 'savemap-result-canvas-container';
    const isViewerFullscreen  = isFullscreen && document.fullscreenElement?.id === '3d-viewer-container';
    const isLocFullscreen     = isFullscreen && document.fullscreenElement?.id === 'loc-viewer-canvas-container';
    _updateSlamFullscreenIcon(isSlamFullscreen);
    _updateSaveMapFullscreenIcon(isSaveMapFullscreen);
    _updateViewerFullscreenIcon(isViewerFullscreen);
    _updateLocFullscreenIcon(isLocFullscreen);
    if (slamResultViewer) {
        slamResultViewer._resizeRenderer();
        slamResultViewer._syncControlStates();
    }
    if (saveMapResultViewer) {
        saveMapResultViewer._resizeRenderer();
        saveMapResultViewer._syncControlStates();
    }
    if (locLiveViewer) locLiveViewer._resizeRenderer();
    if (typeof onWindowResize === 'function') onWindowResize();
});

document.addEventListener('webkitfullscreenchange', () => {
    const isFullscreen = !!document.webkitFullscreenElement;
    const isSlamFullscreen    = isFullscreen && document.webkitFullscreenElement?.id === 'slam-result-canvas-container';
    const isSaveMapFullscreen = isFullscreen && document.webkitFullscreenElement?.id === 'savemap-result-canvas-container';
    const isViewerFullscreen  = isFullscreen && document.webkitFullscreenElement?.id === '3d-viewer-container';
    const isLocFullscreen     = isFullscreen && document.webkitFullscreenElement?.id === 'loc-viewer-canvas-container';
    _updateSlamFullscreenIcon(isSlamFullscreen);
    _updateSaveMapFullscreenIcon(isSaveMapFullscreen);
    _updateViewerFullscreenIcon(isViewerFullscreen);
    _updateLocFullscreenIcon(isLocFullscreen);
    if (slamResultViewer) {
        slamResultViewer._resizeRenderer();
        slamResultViewer._syncControlStates();
    }
    if (saveMapResultViewer) {
        saveMapResultViewer._resizeRenderer();
        saveMapResultViewer._syncControlStates();
    }
    if (locLiveViewer) locLiveViewer._resizeRenderer();
    if (typeof onWindowResize === 'function') onWindowResize();
});

// ==============================================================
// 페이지 로드 시 초기화
// ==============================================================
document.addEventListener('DOMContentLoaded', () => {
    console.log('[DOMContentLoaded] Page loaded');

    ensureWebuiPortsReady().then((cfg) => {
        const webChip = document.getElementById('web-port-chip');
        const pc2Chip = document.getElementById('pc2-ws-port-chip');
        const rosChip = document.getElementById('rosbridge-status-chip');
        if (webChip) webChip.textContent = `Web: ${cfg.webPort}`;
        if (pc2Chip) pc2Chip.textContent = `PC2 WS: ${cfg.pc2WsPort}`;
        if (rosChip) rosChip.textContent = `rosbridge: ${cfg.rosbridgePort || 9090}`;
    });

    // PC2 WebSocket은 Plot 탭 여부와 무관하게 항상 연결 유지
    // (KITTI 변환 진행률 등 전역 백엔드 이벤트 수신에 필요)
    _initBackendWs();

    // rosbridge도 페이지 로드 시 즉시 연결 시도 (SLAM/3D Viewer 탭에서도 사용)
    initRosbridge();

    // bag 슬라이더 드래그 중에는 폴링 업데이트가 썸 위치를 덮어쓰지 않도록 플래그 관리
    const bagSlider = document.getElementById('bag-slider');
    if (bagSlider) {
        bagSlider.addEventListener('pointerdown', () => { _bagSliderDragging = true; });
        bagSlider.addEventListener('pointerup',   () => { _bagSliderDragging = false; });
        bagSlider.addEventListener('pointercancel', () => { _bagSliderDragging = false; });
    }

    // 포맷 선택 변경 시 bag 이름 표시 업데이트
    const formatSelect = document.getElementById('recorder-format-select');
    if (formatSelect) {
        formatSelect.addEventListener('change', () => {
            updateRecorderBagNameDisplay();
        });
    }

    // Visualization 탭의 Plot subtab이 기본 활성화되어 있으면 초기화
    setTimeout(() => {
        const visualizationTab = domCache.get('visualization-tab');
        const plotSubtab = domCache.get('plot-subtab');
        
        if (visualizationTab && visualizationTab.classList.contains('active') &&
            plotSubtab && plotSubtab.classList.contains('active')) {
            console.log('[DOMContentLoaded] Plot subtab is active, initializing');
            initPlotSubtab();
        }
    }, 300);
});

// ═══════════════════════════════════════════════════════════
// Phase 4.10 — SlamAnalyticsDashboard
// ═══════════════════════════════════════════════════════════

function _formatAnalyticsRamPrimary(usedMb) {
    if (usedMb < 1024) {
        return { value: usedMb.toFixed(1), unit: ' MB' };
    }
    return { value: (usedMb / 1024).toFixed(2), unit: ' GB' };
}

function _formatAnalyticsSysRamLabel(sysTotalMb) {
    return sysTotalMb < 1024 ? `${sysTotalMb} MB` : `${(sysTotalMb / 1024).toFixed(0)} GB`;
}

class SlamAnalyticsDashboard {
    constructor() {
        this._ros = null;
        this._subscription = null;
        this._plotsInitialized = false;
        this._ringBuffer = {
            timestamps: [],
            imu_time: [],
            state_time: [],
            map_time: [],
            total_time: [],
            scan_dop: [],
            matching_dop: [],
            traj_dist: []
        };
        this._WINDOW_SEC = 10;
        this._SPEED_WINDOW_SEC = 1; // Avg Speed: 1초 이동평균 윈도우
        this._MAX_TRACE_POINTS = 400; // Plotly trace 무한 누적 방지 (y축 auto-range가 과거 최대값에 고정되는 것 방지)
        this._sysInfo = { total_ram_mb: 0, cpu_cores: 1 };
        // latest-only coalesce: rosbridge/TCP 적체 시 큐를 순서대로 비우지 않음
        this._pendingMsg = null;
        this._rafId = null;
        this._fetchSysInfo();
    }

    async _fetchSysInfo() {
        try {
            const r = await fetch('/api/system/info');
            const d = await r.json();
            this._sysInfo = d;
        } catch (e) {
            console.warn('SlamAnalyticsDashboard: /api/system/info 실패', e);
        }
    }

    show() {
        const el = document.getElementById('slam-analytics-dashboard');
        if (el) {
            el.style.display = 'block';
        }
        this._ensurePlotsInitialized();
    }

    hide() {
        const el = document.getElementById('slam-analytics-dashboard');
        if (el) {
            el.style.display = 'none';
        }
    }

    subscribe() {
        if (this._subscription) {
            return;
        }
        // plotState.ros 연결된 경우 재사용, 없으면 자체 연결 생성 (LocalizationLiveViewer 패턴 동일)
        if (window.plotState && plotState.ros && plotState.ros.isConnected) {
            this._ros = plotState.ros;
            this._doSubscribe();
        } else {
            try {
                this._ros = new ROSLIB.Ros({ url: _getRosbridgeUrl() });
                this._ros.on('connection', () => {
                    console.log('[SlamAnalyticsDashboard] rosbridge connected');
                    this._doSubscribe();
                });
                this._ros.on('error', (err) => {
                    console.error('[SlamAnalyticsDashboard] rosbridge error:', err);
                });
                this._ros.on('close', () => {
                    console.warn('[SlamAnalyticsDashboard] rosbridge connection closed');
                });
            } catch (e) {
                console.error('[SlamAnalyticsDashboard] failed to init rosbridge:', e);
            }
        }
    }

    _doSubscribe() {
        if (this._subscription) {
            return;
        }
        if (!this._ros) {
            console.warn('[SlamAnalyticsDashboard] _doSubscribe: ros not ready');
            return;
        }
        this._subscription = new ROSLIB.Topic({
            ros: this._ros,
            name: '/lio_analytics',
            messageType: getMsgType('fast_lio/LioAnalytics', 'fast_lio/msg/LioAnalytics'),
            // 소형 메시지 + latest-only coalesce → 10Hz 안전 (큐 적체 없음)
            throttle_rate: 100,
            queue_length: 1
        });
        this._subscription.subscribe((msg) => {
            this._pendingMsg = msg;
            if (this._rafId == null) {
                this._rafId = requestAnimationFrame(() => this._flushPending());
            }
        });
        console.log('[SlamAnalyticsDashboard] subscribed to /lio_analytics');
    }

    _flushPending() {
        this._rafId = null;
        const msg = this._pendingMsg;
        this._pendingMsg = null;
        if (!msg) return;
        this._onMessage(msg);
        // 처리 중 도착한 최신 프레임만 이어서 반영
        if (this._pendingMsg != null && this._rafId == null) {
            this._rafId = requestAnimationFrame(() => this._flushPending());
        }
    }

    unsubscribe() {
        if (this._subscription) {
            this._subscription.unsubscribe();
            this._subscription = null;
        }
        this._pendingMsg = null;
        if (this._rafId != null) {
            cancelAnimationFrame(this._rafId);
            this._rafId = null;
        }
        // 자체 생성한 ros 연결이면 닫기 (plotState.ros는 닫지 않음)
        if (this._ros && this._ros !== (window.plotState && plotState.ros)) {
            try { this._ros.close(); } catch (e) { /* ignore */ }
        }
        this._ros = null;
        this._resetWidgets();
        this._ringBuffer = {
            timestamps: [],
            imu_time: [],
            state_time: [],
            map_time: [],
            total_time: [],
            scan_dop: [],
            matching_dop: [],
            traj_dist: []
        };
    }

    _onMessage(msg) {
        const now = Date.now() / 1000;
        const buf = this._ringBuffer;

        buf.timestamps.push(now);
        buf.imu_time.push((msg.imu_time || 0) * 1000);
        buf.state_time.push((msg.state_time || 0) * 1000);
        buf.map_time.push((msg.map_time || 0) * 1000);
        buf.total_time.push((msg.total_time || 0) * 1000);
        buf.scan_dop.push(msg.scan_dop || 0);
        buf.matching_dop.push(msg.matching_dop || 0);
        buf.traj_dist.push(msg.traj_dist || 0);

        // 10초 초과 항목 제거
        while (buf.timestamps.length > 0 && (now - buf.timestamps[0]) > this._WINDOW_SEC) {
            buf.timestamps.shift();
            buf.imu_time.shift();
            buf.state_time.shift();
            buf.map_time.shift();
            buf.total_time.shift();
            buf.scan_dop.shift();
            buf.matching_dop.shift();
            buf.traj_dist.shift();
        }

        this._updateWidgets(msg);
    }

    _ensurePlotsInitialized() {
        if (this._plotsInitialized) {
            return;
        }
        this._plotsInitialized = true;
        this._initProcTimePlot();
        this._initDopPlot();
        this._resetCumulTable();
    }

    _initProcTimePlot() {
        const layout = {
            paper_bgcolor: 'transparent',
            plot_bgcolor: 'rgba(13,13,26,0.5)',
            margin: { t: 4, b: 30, l: 42, r: 8 },
            xaxis: {
                showgrid: false,
                color: '#556',
                tickfont: { size: 9, color: '#556' },
                nticks: 5,
                type: 'date'
            },
            yaxis: {
                gridcolor: '#2d2d4e',
                color: '#556',
                tickfont: { size: 9, color: '#8899aa' },
                title: { text: 'ms', font: { size: 10, color: '#556' } }
            },
            legend: {
                orientation: 'h',
                y: -0.22,
                x: 0,
                font: { size: 9, color: '#c8d6e5' },
                bgcolor: 'transparent'
            },
            hovermode: 'x unified',
            hoverlabel: {
                bgcolor: '#1a1a35',
                bordercolor: '#4a4a6e',
                font: { color: '#e8f0fe', size: 11, family: 'Segoe UI, sans-serif' },
                align: 'left'
            }
        };
        Plotly.newPlot('analytics-chart-proctime', [
            {
                x: [],
                y: [],
                name: 'IMU Undistort',
                type: 'scatter',
                mode: 'lines',
                fill: 'tozeroy',
                fillcolor: 'rgba(0,210,106,0.35)',
                line: { color: '#00d26a', width: 1.5 }
            },
            {
                x: [],
                y: [],
                name: 'EKF State Update',
                type: 'scatter',
                mode: 'lines',
                fill: 'tonexty',
                fillcolor: 'rgba(126,207,244,0.35)',
                line: { color: '#7ecff4', width: 1.5 }
            },
            {
                x: [],
                y: [],
                name: 'Map Increment',
                type: 'scatter',
                mode: 'lines',
                fill: 'tonexty',
                fillcolor: 'rgba(233,69,96,0.35)',
                line: { color: '#e94560', width: 1.5 }
            },
            {
                x: [],
                y: [],
                name: 'Total Pipeline',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#ffd32a', width: 2, dash: 'dot' }
            }
        ], layout, { responsive: true, displayModeBar: false });
    }

    _initDopPlot() {
        const layout = {
            paper_bgcolor: 'transparent',
            plot_bgcolor: 'rgba(13,13,26,0.5)',
            margin: { t: 4, b: 30, l: 36, r: 8 },
            xaxis: {
                showgrid: false,
                color: '#556',
                tickfont: { size: 9, color: '#556' },
                nticks: 5,
                type: 'date'
            },
            yaxis: {
                gridcolor: '#2d2d4e',
                color: '#556',
                tickfont: { size: 9, color: '#8899aa' },
                title: { text: 'DOP', font: { size: 10, color: '#556' } },
                range: [0, 10]
            },
            legend: {
                orientation: 'h',
                y: -0.22,
                x: 0,
                font: { size: 9, color: '#c8d6e5' },
                bgcolor: 'transparent'
            },
            hovermode: 'x unified',
            hoverlabel: {
                bgcolor: '#1a1a35',
                bordercolor: '#4a4a6e',
                font: { color: '#e8f0fe', size: 11, family: 'Segoe UI, sans-serif' },
                align: 'left'
            }
        };
        Plotly.newPlot('analytics-chart-dop', [
            {
                x: [],
                y: [],
                name: 'Scan PDOP',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#00d26a', width: 2 }
            },
            {
                x: [],
                y: [],
                name: 'Matching PDOP',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#a29bfe', width: 2 }
            }
        ], layout, { responsive: true, displayModeBar: false });
    }

    _resetCumulTable() {
        const tbody = document.getElementById('analytics-cumul-tbody');
        if (!tbody) {
            return;
        }
        const rows = [
            { id: 'cumul-imu', label: 'IMU Undistort' },
            { id: 'cumul-state', label: 'EKF State Update' },
            { id: 'cumul-map', label: 'Map Increment' },
            { id: 'cumul-total', label: 'Total Pipeline' }
        ];
        tbody.innerHTML = rows.map((r) => `
            <tr>
                <td class="analytics-cumul-row-label">${r.label}</td>
                <td class="analytics-right analytics-cumul-mean" id="${r.id}-mean">—</td>
                <td class="analytics-right analytics-cumul-max" id="${r.id}-max">—</td>
                <td style="width:140px; padding: 0 8px;">
                    <div style="position:relative;">
                        <div style="height:8px; background:#1a1a35; border-radius:4px; overflow:hidden; margin-top:3px;">
                            <div id="${r.id}-bar-mean" style="height:100%; border-radius:4px; background:#7ecff4; transition:width 0.4s; width:0%;"></div>
                        </div>
                        <div id="${r.id}-bar-max" style="position:absolute;top:3px;left:0%;width:2px;height:8px;background:#e94560;border-radius:1px;transform:translateX(-1px);"></div>
                    </div>
                    <div style="display:flex;justify-content:space-between;font-size:8px;color:#556;margin-top:2px;">
                        <span style="color:#7ecff4">■ Mean</span><span style="color:#e94560">▎Max</span>
                    </div>
                </td>
            </tr>
        `).join('');
    }

    _updateWidgets(msg) {
        this._ensurePlotsInitialized();
        this._updateHzCard(msg);
        this._updateTrajCard(msg);
        this._updateCpuCard(msg);
        this._updateRamCard(msg);
        this._updateProcTimePlot(msg);
        this._updateDopPlot(msg);
        this._updateCumulTable(msg);
        this._updateDetailSection(msg);
    }

    _updateHzCard(msg) {
        const imuEl = document.getElementById('analytics-imu-hz');
        const lidEl = document.getElementById('analytics-lid-hz');
        if (!imuEl || !lidEl) {
            return;
        }
        const setHz = (el, val) => {
            el.textContent = val;
            el.className = 'analytics-hz-value';
            if (val <= 0) {
                el.classList.add('analytics-hz-err');
            } else if (val < 10) {
                el.classList.add('analytics-hz-warn');
            } else {
                el.classList.add('analytics-hz-ok');
            }
        };
        setHz(imuEl, msg.imu_freq || 0);
        setHz(lidEl, msg.lid_freq || 0);
    }

    _updateTrajCard(msg) {
        const distEl = document.getElementById('analytics-traj-dist');
        const timeEl = document.getElementById('analytics-run-time');
        const speedEl = document.getElementById('analytics-avg-speed');
        if (!distEl || !timeEl || !speedEl) {
            return;
        }

        const dist = msg.traj_dist || 0;
        const runSec = msg.run_time || 0;
        distEl.textContent = (typeof dist === 'number' ? dist.toFixed(2) : dist);

        const h = Math.floor(runSec / 3600);
        const m = Math.floor((runSec % 3600) / 60);
        const s = runSec % 60;
        timeEl.textContent = `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;

        // Avg Speed: 전체 누적평균 대신 최근 1초간 이동거리/이동시간 기준 이동평균으로 계산
        const buf = this._ringBuffer;
        const n = buf.timestamps.length;
        let speed = 0;
        if (n > 0) {
            const nowTs = buf.timestamps[n - 1];
            let idx = n - 1;
            while (idx > 0 && (nowTs - buf.timestamps[idx - 1]) <= this._SPEED_WINDOW_SEC) {
                idx--;
            }
            const dt = nowTs - buf.timestamps[idx];
            const dd = dist - buf.traj_dist[idx];
            speed = (dt > 0) ? (dd / dt) : 0;
        }
        speedEl.textContent = speed.toFixed(2);
    }

    _updateCpuCard(msg) {
        const pctEl = document.getElementById('analytics-cpu-percent');
        const coreEl = document.getElementById('analytics-cpu-cores-equiv');
        const barEl = document.getElementById('analytics-cpu-bar-fill');
        if (!pctEl || !barEl) {
            return;
        }

        // cpu_usage = 시스템 대비 CPU % (0-100, 코어 수로 정규화됨)
        const cpuPercent = msg.cpu_usage || 0;
        const totalCores = this._sysInfo.cpu_cores || 1;
        const usedCores = (cpuPercent / 100) * totalCores;
        const barPct = Math.min(cpuPercent, 100);

        pctEl.innerHTML = `${cpuPercent.toFixed(1)}<span class="analytics-cpu-pct-sym">%</span>`;
        if (coreEl) {
            coreEl.textContent = `${usedCores.toFixed(1)} of ${totalCores} cores`;
        }
        barEl.style.width = `${barPct}%`;
    }

    _updateRamCard(msg) {
        const valEl = document.getElementById('analytics-ram-value');
        const subEl = document.getElementById('analytics-ram-sub');
        const barEl = document.getElementById('analytics-ram-bar-fill');
        if (!valEl || !barEl) {
            return;
        }

        const usedMb = Number(msg.ram_usage) || 0;
        const sysTotal = Number(this._sysInfo.total_ram_mb) || 32768;
        const sysPct = sysTotal > 0 ? (usedMb / sysTotal * 100) : 0;
        const barPct = Math.min(sysPct, 100);

        const { value, unit } = _formatAnalyticsRamPrimary(usedMb);
        valEl.innerHTML = `${sysPct.toFixed(1)}<span class="analytics-cpu-pct-sym">%</span>`;
        if (subEl) {
            subEl.textContent = `${value}${unit} of ${_formatAnalyticsSysRamLabel(sysTotal)}`;
        }
        barEl.style.width = `${barPct}%`;
    }

    _updateProcTimePlot(msg) {
        const now = Date.now();
        const x = new Date(now);
        const imu = (msg.imu_time || 0) * 1000;
        const stateStacked = imu + ((msg.state_time || 0) * 1000);
        const mapStacked = stateStacked + ((msg.map_time || 0) * 1000);
        const total = (msg.total_time || 0) * 1000;

        Plotly.extendTraces('analytics-chart-proctime', {
            x: [[x], [x], [x], [x]],
            y: [[imu], [stateStacked], [mapStacked], [total]]
        }, [0, 1, 2, 3], this._MAX_TRACE_POINTS);

        // 창(WINDOW_SEC) 내 최근 데이터의 최대값 기준으로 y축 범위 재계산 (과거 한때의 최대값에 고정되지 않도록)
        const buf = this._ringBuffer;
        let procMax = 0;
        for (let i = 0; i < buf.timestamps.length; i++) {
            const stackedTop = buf.imu_time[i] + buf.state_time[i] + buf.map_time[i];
            procMax = Math.max(procMax, stackedTop, buf.total_time[i]);
        }
        const procYmax = procMax > 0 ? procMax * 1.15 : 10;

        Plotly.relayout('analytics-chart-proctime', {
            'xaxis.range': [new Date(now - (this._WINDOW_SEC * 1000)), x],
            'yaxis.range': [0, procYmax]
        });
    }

    _updateDopPlot(msg) {
        const buf = this._ringBuffer;
        const now = Date.now();
        const x = new Date(now);
        const scan = msg.scan_dop || 0;
        const matching = msg.matching_dop || 0;

        Plotly.extendTraces('analytics-chart-dop', {
            x: [[x], [x]],
            y: [[scan], [matching]]
        }, [0, 1], this._MAX_TRACE_POINTS);

        const allDop = [...buf.scan_dop, ...buf.matching_dop].filter((v) => Number.isFinite(v));
        const dopMax = allDop.length > 0 ? Math.max(...allDop) : 10;
        const dopYmax = Math.min(dopMax * 1.15, 100);
        Plotly.relayout('analytics-chart-dop', {
            'xaxis.range': [new Date(now - (this._WINDOW_SEC * 1000)), x],
            'yaxis.range': [0, dopYmax]
        });
    }

    _updateCumulTable(msg) {
        const rows = [
            { id: 'cumul-imu', mean: (msg.imu_mean || 0) * 1000, max: (msg.imu_max || 0) * 1000 },
            { id: 'cumul-state', mean: (msg.state_mean || 0) * 1000, max: (msg.state_max || 0) * 1000 },
            { id: 'cumul-map', mean: (msg.map_mean || 0) * 1000, max: (msg.map_max || 0) * 1000 },
            { id: 'cumul-total', mean: (msg.total_mean || 0) * 1000, max: (msg.total_max || 0) * 1000 }
        ];
        const maxVal = Math.max(...rows.map((r) => r.max), 1);

        rows.forEach((r) => {
            const meanEl = document.getElementById(`${r.id}-mean`);
            const maxEl = document.getElementById(`${r.id}-max`);
            const barMeanEl = document.getElementById(`${r.id}-bar-mean`);
            const barMaxEl = document.getElementById(`${r.id}-bar-max`);

            if (meanEl) {
                meanEl.textContent = r.mean.toFixed(2);
            }
            if (maxEl) {
                maxEl.textContent = r.max.toFixed(2);
            }
            if (barMeanEl) {
                barMeanEl.style.width = `${(r.mean / maxVal * 100).toFixed(1)}%`;
            }
            if (barMaxEl) {
                barMaxEl.style.left = `${(r.max / maxVal * 100).toFixed(1)}%`;
            }
        });
    }

    _updateDetailSection(msg) {
        const fields = [
            ['scan_size', (v) => `${v.toLocaleString()} pts`],
            ['down_size', (v) => `${v.toLocaleString()} pts`],
            ['map_size', (v) => `${v.toLocaleString()} pts`],
            ['map_valid_size', (v) => `${v.toLocaleString()} pts`],
            ['new_idxs', (v) => `${v.toLocaleString()} pts`],
            ['map_delete_size', (v) => `${v.toLocaleString()} pts`],
            ['buffer_size', (v) => `${v}`],
            ['imu_buffer_size', (v) => `${v}`],
            ['scan_time', (v) => `${v.toFixed(4)} s`],
            // Adaptive Downsampling Parameters
            ['filter_size_surf_ad', (v) => `${v.toFixed(3)} m`],
            ['point_filter_num_ad', (v) => `${v}`],
            ['num_feats', (v) => `${v.toLocaleString()}`],
            ['num_reject', (v) => `${v.toLocaleString()}`],
            ['match_ratio', (v) => `${v.toFixed(4)}`],
            ['res_mean', (v) => `${v.toFixed(5)} m`],
            ['res_std', (v) => `${v.toFixed(5)} m`],
            ['kf_iterations', (v) => `${v}`],
            ['pos_cov', (v) => `${v.toExponential(3)}`],
            ['rot_cov', (v) => `${v.toExponential(3)}`],
            ['lidar_meas_cov', (v) => `${v.toExponential(3)}`],
            ['vel_norm', (v) => `${v.toFixed(3)} m/s`],
            ['acc_bias_norm', (v) => `${v.toFixed(4)}`],
            ['gyr_bias_norm', (v) => `${v.toFixed(5)}`],
            ['search_time', (v) => `${(v * 1000).toFixed(1)} ms`],
            ['delete_time', (v) => `${(v * 1000).toFixed(1)} ms`],
            ['kf_count', (v) => `${v}`],
            ['kf_dist_last', (v) => `${v.toFixed(2)} m`]
        ];

        fields.forEach(([key, fmt]) => {
            const el = document.getElementById(`ad-${key}`);
            if (!el) {
                return;
            }
            const val = msg[key];
            if (val === undefined || val === null) {
                return;
            }
            try {
                el.textContent = fmt(val);
            } catch (e) {
                el.textContent = val;
            }
        });
    }

    _resetWidgets() {
        const textIds = [
            'analytics-imu-hz',
            'analytics-lid-hz',
            'analytics-traj-dist',
            'analytics-run-time',
            'analytics-avg-speed',
            'analytics-cpu-percent',
            'analytics-cpu-cores-equiv'
        ];
        textIds.forEach((id) => {
            const el = document.getElementById(id);
            if (el) {
                el.textContent = '—';
            }
        });
        const barFill = document.getElementById('analytics-cpu-bar-fill');
        if (barFill) {
            barFill.style.width = '0%';
        }
        const ramVal = document.getElementById('analytics-ram-value');
        if (ramVal) {
            ramVal.innerHTML = '—<span class="analytics-cpu-pct-sym"> MB</span>';
        }
        const ramSub = document.getElementById('analytics-ram-sub');
        if (ramSub) {
            ramSub.textContent = '';
        }
        const ramBar = document.getElementById('analytics-ram-bar-fill');
        if (ramBar) {
            ramBar.style.width = '0%';
        }

        // 상세 섹션 닫기 및 버튼 텍스트 초기화
        const detailSection = document.getElementById('slam-analytics-detail');
        const detailBtn = document.getElementById('slam-analytics-detail-toggle');
        if (detailSection) {
            detailSection.classList.remove('open');
        }
        if (detailBtn) {
            detailBtn.classList.remove('open');
            detailBtn.textContent = 'Full Statistics (Scan/Map · Feature Matching · Residual · IESEKF · IMU State · Time Sync …)';
        }

        this._plotsInitialized = false;
    }
}

// SlamAnalyticsDashboard 전역 인스턴스
const slamAnalyticsDashboard = new SlamAnalyticsDashboard();

// Phase 4.10.6 — detail toggle
function toggleSlamAnalyticsDetail() {
    const section = document.getElementById('slam-analytics-detail');
    const btn = document.getElementById('slam-analytics-detail-toggle');
    if (!section || !btn) {
        return;
    }
    const isOpen = section.classList.toggle('open');
    btn.classList.toggle('open', isOpen);
    // 버튼 텍스트: arrow는 CSS ::after로 처리
    btn.textContent = isOpen
        ? 'Collapse Statistics'
        : 'Full Statistics (Scan/Map · Feature Matching · Residual · IESEKF · IMU State · Time Sync …)';
}

// ══════════════════════════════════════════════════════════════════════
// LocAnalyticsDashboard — Localization Analytics Dashboard
// ══════════════════════════════════════════════════════════════════════
class LocAnalyticsDashboard {
    constructor() {
        this._ros          = null;
        this._subscription = null;
        this._plotsInit    = false;
        this._WINDOW_SEC   = 10;
        this._SPEED_WINDOW_SEC = 1; // Avg Speed: 1초 이동평균 윈도우
        this._speedBuf     = { timestamps: [], traj_dist: [] };
        this._sysInfo      = {};
        this._rugHistory   = new Array(200).fill(false);
        this._rugCanvas    = null;
        this._updateCount  = 0;
        this._lastUpdateMs = null;
        this._pendingMsg   = null;
        this._rafId        = null;
        this._fetchSysInfo();
    }

    async _fetchSysInfo() {
        try {
            const r = await fetch('/api/system/info');
            this._sysInfo = await r.json();
        } catch (e) { /* ignore */ }
    }

    show() {
        const el = document.getElementById('loc-analytics-dashboard');
        if (el) el.style.display = 'block';
        this._ensurePlotsInit();
        this._initRug();
    }

    hide() {
        const el = document.getElementById('loc-analytics-dashboard');
        if (el) el.style.display = 'none';
    }

    subscribe() {
        if (this._subscription) return;
        if (window.plotState && plotState.ros && plotState.ros.isConnected) {
            this._ros = plotState.ros;
            this._doSubscribe();
        } else {
            try {
                this._ros = new ROSLIB.Ros({ url: _getRosbridgeUrl() });
                this._ros.on('connection', () => this._doSubscribe());
                this._ros.on('error', (e) => console.error('[LocAnalytics] rosbridge error:', e));
                this._ros.on('close', ()  => console.warn('[LocAnalytics] rosbridge closed'));
            } catch (e) {
                console.error('[LocAnalytics] failed to init rosbridge:', e);
            }
        }
    }

    _doSubscribe() {
        if (this._subscription || !this._ros) return;
        this._subscription = new ROSLIB.Topic({
            ros: this._ros,
            name: '/loc_analytics',
            messageType: getMsgType('fast_lio/LocAnalytics', 'fast_lio/msg/LocAnalytics'),
            throttle_rate: 100,
            queue_length: 1
        });
        this._subscription.subscribe((msg) => {
            this._pendingMsg = msg;
            if (this._rafId == null) {
                this._rafId = requestAnimationFrame(() => this._flushPending());
            }
        });
        console.log('[LocAnalytics] subscribed to /loc_analytics');
    }

    _flushPending() {
        this._rafId = null;
        const msg = this._pendingMsg;
        this._pendingMsg = null;
        if (!msg) return;
        this._onMessage(msg);
        if (this._pendingMsg != null && this._rafId == null) {
            this._rafId = requestAnimationFrame(() => this._flushPending());
        }
    }

    unsubscribe() {
        if (this._subscription) {
            this._subscription.unsubscribe();
            this._subscription = null;
        }
        this._pendingMsg = null;
        if (this._rafId != null) {
            cancelAnimationFrame(this._rafId);
            this._rafId = null;
        }
        if (this._ros && this._ros !== (window.plotState && plotState.ros)) {
            try { this._ros.close(); } catch (e) { /* ignore */ }
        }
        this._ros = null;
        this._resetWidgets();
        this._speedBuf = { timestamps: [], traj_dist: [] };
    }

    _onMessage(msg) {
        this._ensurePlotsInit();

        // Avg Speed 1초 이동평균 계산용 이력 버퍼 갱신
        const now = Date.now() / 1000;
        const sbuf = this._speedBuf;
        sbuf.timestamps.push(now);
        sbuf.traj_dist.push(msg.traj_dist || 0);
        while (sbuf.timestamps.length > 0 && (now - sbuf.timestamps[0]) > this._WINDOW_SEC) {
            sbuf.timestamps.shift();
            sbuf.traj_dist.shift();
        }

        this._updateWidgets(msg);
    }

    _ensurePlotsInit() {
        if (this._plotsInit) return;
        this._plotsInit = true;
        this._initProcTimePlot();
        this._initDopPlot();
        this._resetCumulTable();
    }

    // ── Plotly 초기화 (SLAM과 동일 스펙) ────────────────────────
    _initProcTimePlot() {
        const layout = {
            paper_bgcolor: 'transparent', plot_bgcolor: 'rgba(13,13,26,0.5)',
            margin: { t: 4, b: 30, l: 42, r: 8 },
            xaxis: { showgrid:false, color:'#556', tickfont:{size:9,color:'#556'}, nticks:5, type:'date' },
            yaxis: { gridcolor:'#2d2d4e', color:'#556', tickfont:{size:9,color:'#8899aa'}, title:{text:'ms',font:{size:10,color:'#556'}} },
            legend: { orientation:'h', y:-0.22, x:0, font:{size:9,color:'#c8d6e5'}, bgcolor:'transparent' },
            hovermode: 'x unified',
            hoverlabel: { bgcolor:'#1a1a35', bordercolor:'#4a4a6e', font:{color:'#e8f0fe',size:11}, align:'left' }
        };
        Plotly.newPlot('loc-analytics-chart-proctime', [
            { x:[], y:[], name:'IMU Undistort',    type:'scatter', mode:'lines', fill:'tozeroy',  fillcolor:'rgba(0,210,106,0.35)',  line:{color:'#00d26a',width:1.5} },
            { x:[], y:[], name:'EKF State Update', type:'scatter', mode:'lines', fill:'tonexty',  fillcolor:'rgba(126,207,244,0.35)',line:{color:'#7ecff4',width:1.5} },
            { x:[], y:[], name:'Map Increment',    type:'scatter', mode:'lines', fill:'tonexty',  fillcolor:'rgba(233,69,96,0.35)',  line:{color:'#e94560',width:1.5} },
            { x:[], y:[], name:'Total Pipeline',   type:'scatter', mode:'lines', line:{color:'#ffd32a',width:2,dash:'dot'} }
        ], layout, { responsive:true, displayModeBar:false });
    }

    _initDopPlot() {
        const layout = {
            paper_bgcolor: 'transparent', plot_bgcolor: 'rgba(13,13,26,0.5)',
            margin: { t: 4, b: 28, l: 36, r: 8 },
            xaxis: { showgrid:false, color:'#556', tickfont:{size:9,color:'#556'}, nticks:5, type:'date' },
            yaxis: { gridcolor:'#2d2d4e', color:'#556', tickfont:{size:9,color:'#8899aa'}, title:{text:'DOP',font:{size:10,color:'#556'}}, range:[0,10] },
            legend: { orientation:'h', y:-0.28, x:0, font:{size:9,color:'#c8d6e5'}, bgcolor:'transparent' },
            hovermode: 'x unified',
            hoverlabel: { bgcolor:'#1a1a35', bordercolor:'#4a4a6e', font:{color:'#e8f0fe',size:11}, align:'left' }
        };
        Plotly.newPlot('loc-analytics-chart-dop', [
            { x:[], y:[], name:'Scan PDOP',     type:'scatter', mode:'lines', line:{color:'#00d26a',width:2} },
            { x:[], y:[], name:'Matching PDOP', type:'scatter', mode:'lines', line:{color:'#a29bfe',width:2} }
        ], layout, { responsive:true, displayModeBar:false });
    }

    _resetCumulTable() {
        const tbody = document.getElementById('loc-analytics-cumul-tbody');
        if (!tbody) return;
        const rows = [
            { id:'loc-cumul-imu',   label:'IMU Undistort' },
            { id:'loc-cumul-state', label:'EKF State Update' },
            { id:'loc-cumul-map',   label:'Map Increment' },
            { id:'loc-cumul-total', label:'Total Pipeline' }
        ];
        tbody.innerHTML = rows.map((r) => `
            <tr>
                <td class="analytics-cumul-row-label">${r.label}</td>
                <td class="analytics-right analytics-cumul-mean" id="${r.id}-mean">—</td>
                <td class="analytics-right analytics-cumul-max"  id="${r.id}-max">—</td>
                <td style="width:180px;padding:0 8px;">
                    <div style="position:relative;">
                        <div style="height:8px;background:#1a1a35;border-radius:4px;overflow:hidden;margin-top:3px;">
                            <div id="${r.id}-bar-mean" style="height:100%;border-radius:4px;background:#7ecff4;transition:width .4s;width:0%"></div>
                        </div>
                        <div id="${r.id}-bar-max" style="position:absolute;top:3px;left:0%;width:2px;height:8px;background:#e94560;border-radius:1px;transform:translateX(-1px)"></div>
                    </div>
                    <div style="display:flex;justify-content:space-between;font-size:8px;color:#556;margin-top:2px;">
                        <span style="color:#7ecff4">■ Mean</span><span style="color:#e94560">▎Max</span>
                    </div>
                </td>
            </tr>
        `).join('');
    }

    // ── Rug canvas ───────────────────────────────────────────────
    _initRug() {
        this._rugCanvas = document.getElementById('loc-rug-canvas');
        if (!this._rugCanvas) return;
        const track = this._rugCanvas.parentElement;
        this._rugCanvas.width  = track.clientWidth  || 400;
        this._rugCanvas.height = track.clientHeight || 12;
        this._drawRug();
    }

    _drawRug() {
        if (!this._rugCanvas) return;
        const ctx  = this._rugCanvas.getContext('2d');
        const W    = this._rugCanvas.width;
        const H    = this._rugCanvas.height;
        const N    = this._rugHistory.length;
        const cell = W / N;
        ctx.clearRect(0, 0, W, H);
        this._rugHistory.forEach((updated, i) => {
            ctx.fillStyle = updated ? '#ffd32a' : '#1a1a35';
            ctx.fillRect(Math.floor(i * cell), 0, Math.max(Math.ceil(cell), 1), H);
        });
    }

    // ── Widget updates ───────────────────────────────────────────
    _updateWidgets(msg) {
        this._updateHzCard(msg);
        this._updateTrajCard(msg);
        this._updateCpuCard(msg);
        this._updateRamCard(msg);
        this._updateProcTimePlot(msg);
        this._updateDopPlot(msg);
        this._updateCumulTable(msg);
        this._updateRug(msg);
        this._updateInitBadge(msg);
        this._updateDetailSection(msg);
    }

    _updateHzCard(msg) {
        const imuEl = document.getElementById('loc-analytics-imu-hz');
        const lidEl = document.getElementById('loc-analytics-lid-hz');
        if (!imuEl || !lidEl) return;
        const setHz = (el, val) => {
            el.textContent = val;
            el.className = 'analytics-hz-value ' +
                (val <= 0 ? 'analytics-hz-err' : val < 10 ? 'analytics-hz-warn' : 'analytics-hz-ok');
        };
        setHz(imuEl, msg.imu_freq || 0);
        setHz(lidEl, msg.lid_freq || 0);
    }

    _updateTrajCard(msg) {
        const distEl  = document.getElementById('loc-analytics-traj-dist');
        const timeEl  = document.getElementById('loc-analytics-run-time');
        const speedEl = document.getElementById('loc-analytics-avg-speed');
        if (!distEl) return;
        const dist   = msg.traj_dist || 0;
        const runSec = msg.run_time  || 0;
        distEl.textContent = typeof dist === 'number' ? dist.toFixed(2) : dist;
        const h = Math.floor(runSec / 3600);
        const m = Math.floor((runSec % 3600) / 60);
        const s = runSec % 60;
        if (timeEl) timeEl.textContent = `${String(h).padStart(2,'0')}:${String(m).padStart(2,'0')}:${String(s).padStart(2,'0')}`;

        // Avg Speed: 전체 누적평균 대신 최근 1초간 이동거리/이동시간 기준 이동평균으로 계산
        const sbuf = this._speedBuf;
        const n = sbuf.timestamps.length;
        let speed = 0;
        if (n > 0) {
            const nowTs = sbuf.timestamps[n - 1];
            let idx = n - 1;
            while (idx > 0 && (nowTs - sbuf.timestamps[idx - 1]) <= this._SPEED_WINDOW_SEC) {
                idx--;
            }
            const dt = nowTs - sbuf.timestamps[idx];
            const dd = dist - sbuf.traj_dist[idx];
            speed = (dt > 0) ? (dd / dt) : 0;
        }
        if (speedEl) speedEl.textContent = speed.toFixed(2);
    }

    _updateCpuCard(msg) {
        const pctEl  = document.getElementById('loc-analytics-cpu-percent');
        const coreEl = document.getElementById('loc-analytics-cpu-cores-equiv');
        const barEl  = document.getElementById('loc-analytics-cpu-bar-fill');
        if (!pctEl || !barEl) return;
        const cpuPercent = msg.cpu_usage || 0;
        const total      = this._sysInfo.cpu_cores || 1;
        const usedCores  = (cpuPercent / 100) * total;
        const barPct     = Math.min(cpuPercent, 100);
        pctEl.innerHTML = `${cpuPercent.toFixed(1)}<span class="analytics-cpu-pct-sym">%</span>`;
        if (coreEl) coreEl.textContent = `${usedCores.toFixed(1)} of ${total} cores`;
        barEl.style.width = `${barPct}%`;
    }

    _updateRamCard(msg) {
        const valEl = document.getElementById('loc-analytics-ram-value');
        const subEl = document.getElementById('loc-analytics-ram-sub');
        const barEl = document.getElementById('loc-analytics-ram-bar-fill');
        if (!valEl || !barEl) {
            return;
        }

        const usedMb = Number(msg.ram_usage) || 0;
        const sysTotal = Number(this._sysInfo.total_ram_mb) || 32768;
        const sysPct = sysTotal > 0 ? (usedMb / sysTotal * 100) : 0;
        const barPct = Math.min(sysPct, 100);

        const { value, unit } = _formatAnalyticsRamPrimary(usedMb);
        valEl.innerHTML = `${sysPct.toFixed(1)}<span class="analytics-cpu-pct-sym">%</span>`;
        if (subEl) {
            subEl.textContent = `${value}${unit} of ${_formatAnalyticsSysRamLabel(sysTotal)}`;
        }
        barEl.style.width = `${barPct}%`;
    }

    _updateProcTimePlot(msg) {
        const now = Date.now();
        const x   = new Date(now);
        const imu   = (msg.imu_time   || 0) * 1000;
        const state = imu + (msg.state_time || 0) * 1000;
        const map   = state + (msg.map_time  || 0) * 1000;
        const total = (msg.total_time || 0) * 1000;
        Plotly.extendTraces('loc-analytics-chart-proctime', {
            x: [[x],[x],[x],[x]], y: [[imu],[state],[map],[total]]
        }, [0,1,2,3]);
        Plotly.relayout('loc-analytics-chart-proctime', {
            'xaxis.range': [new Date(now - this._WINDOW_SEC * 1000), x]
        });
    }

    _updateDopPlot(msg) {
        const now  = Date.now();
        const x    = new Date(now);
        const scan = msg.scan_dop     || 0;
        const match= msg.matching_dop || 0;
        Plotly.extendTraces('loc-analytics-chart-dop', {
            x: [[x],[x]], y: [[scan],[match]]
        }, [0,1]);
        const dopMax = Math.max(scan, match, 1);
        Plotly.relayout('loc-analytics-chart-dop', {
            'xaxis.range': [new Date(now - this._WINDOW_SEC * 1000), x],
            'yaxis.range': [0, Math.min(dopMax * 1.15, 100)]
        });
    }

    _updateCumulTable(msg) {
        const rows = [
            { id:'loc-cumul-imu',   mean:(msg.imu_mean   ||0)*1000, max:(msg.imu_max   ||0)*1000 },
            { id:'loc-cumul-state', mean:(msg.state_mean ||0)*1000, max:(msg.state_max ||0)*1000 },
            { id:'loc-cumul-map',   mean:(msg.map_mean   ||0)*1000, max:(msg.map_max   ||0)*1000 },
            { id:'loc-cumul-total', mean:(msg.total_mean ||0)*1000, max:(msg.total_max ||0)*1000 }
        ];
        const maxVal = Math.max(...rows.map((r) => r.max), 1);
        rows.forEach((r) => {
            const mEl  = document.getElementById(`${r.id}-mean`);
            const xEl  = document.getElementById(`${r.id}-max`);
            const bMEl = document.getElementById(`${r.id}-bar-mean`);
            const bXEl = document.getElementById(`${r.id}-bar-max`);
            if (mEl)  mEl.textContent  = r.mean.toFixed(2);
            if (xEl)  xEl.textContent  = r.max.toFixed(2);
            if (bMEl) bMEl.style.width = `${(r.mean / maxVal * 100).toFixed(1)}%`;
            if (bXEl) bXEl.style.left  = `${(r.max  / maxVal * 100).toFixed(1)}%`;
        });
    }

    _updateRug(msg) {
        const updated  = !!msg.map_updated;
        const dopRatio = msg.dop_ratio || 0;

        this._rugHistory.push(updated);
        this._rugHistory.shift();
        this._drawRug();

        // dop_ratio 표시
        const ratioEl = document.getElementById('loc-dop-ratio-val');
        if (ratioEl) ratioEl.textContent = dopRatio.toFixed(2);

        if (updated) {
            this._updateCount++;
            this._lastUpdateMs = Date.now();
            // 플래시 도트
            const dot = document.getElementById('loc-map-update-dot');
            if (dot) { dot.classList.remove('flash'); void dot.offsetWidth; dot.classList.add('flash'); }
        }

        // 카운트 & 마지막 업데이트
        const countEl = document.getElementById('loc-map-update-count');
        if (countEl) countEl.textContent = this._updateCount;
        const lastEl  = document.getElementById('loc-map-update-last');
        if (lastEl && this._lastUpdateMs !== null) {
            const sec = ((Date.now() - this._lastUpdateMs) / 1000).toFixed(1);
            lastEl.textContent = `${sec}s ago`;
        }
    }

    _updateInitBadge(msg) {
        const inited = !!msg.is_initialized;
        const text   = inited ? '✓ Initialized' : '✕ Not Init';
        const cls    = inited ? 'ok' : 'bad';

        const headerBadge = document.getElementById('loc-header-init-badge');
        if (headerBadge) { headerBadge.textContent = text; headerBadge.className = `loc-init-badge ${cls}`; }

        const rugBadge = document.getElementById('loc-init-status-dot');
        if (rugBadge) { rugBadge.textContent = text; rugBadge.className = `loc-init-status-dot ${cls}`; }
    }

    _updateDetailSection(msg) {
        const fields = [
            ['scan_size',          (v) => `${v.toLocaleString()} pts`],
            ['down_size',          (v) => `${v.toLocaleString()} pts`],
            ['map_size',           (v) => `${v.toLocaleString()} pts`],
            ['map_valid_size',     (v) => `${v.toLocaleString()} pts`],
            ['new_idxs',           (v) => `${v.toLocaleString()} pts`],
            ['buffer_size',        (v) => `${v}`],
            ['imu_buffer_size',    (v) => `${v}`],
            ['scan_time',          (v) => `${v.toFixed(4)} s`],
            ['filter_size_surf_ad',(v) => `${v.toFixed(3)} m`],
            ['point_filter_num_ad',(v) => `${v}`],
            ['num_feats',          (v) => `${v.toLocaleString()}`],
            ['num_reject',         (v) => `${v.toLocaleString()}`],
            ['match_ratio',        (v) => `${v.toFixed(4)}`],
            ['res_mean',           (v) => `${v.toFixed(5)} m`],
            ['res_std',            (v) => `${v.toFixed(5)} m`],
            ['pos_cov',            (v) => `${v.toExponential(3)}`],
            ['rot_cov',            (v) => `${v.toExponential(3)}`],
            ['lidar_meas_cov',     (v) => `${v.toExponential(3)}`],
            ['scan_dop',           (v) => `${v.toFixed(3)}`],
            ['down_dop',           (v) => `${v.toFixed(3)}`],
            ['matching_dop',       (v) => `${v.toFixed(3)}`],
            ['dop_ratio',          (v) => `${v.toFixed(3)}`],
            ['vel_norm',           (v) => `${v.toFixed(3)} m/s`],
            ['acc_bias_norm',      (v) => `${v.toFixed(4)}`],
            ['gyr_bias_norm',      (v) => `${v.toFixed(5)}`],
            ['kf_count',           (v) => `${v}`],
            ['kf_dist_last',       (v) => `${v.toFixed(2)} m`],
            ['is_initialized',     (v) => v ? 'true' : 'false'],
            ['map_updated',        (v) => v ? 'true' : 'false'],
            ['linear_velo',        (v) => `${v.toFixed(3)} m/s`],
            ['angular_velo',       (v) => `${v.toFixed(3)} rad/s`]
        ];
        fields.forEach(([key, fmt]) => {
            const el  = document.getElementById(`lad-${key}`);
            if (!el) return;
            const val = msg[key];
            if (val === undefined || val === null) return;
            try { el.textContent = fmt(val); } catch (e) { el.textContent = val; }
        });
    }

    _resetWidgets() {
        ['loc-analytics-imu-hz','loc-analytics-lid-hz','loc-analytics-traj-dist',
         'loc-analytics-run-time','loc-analytics-avg-speed','loc-analytics-cpu-percent',
         'loc-analytics-cpu-cores-equiv','loc-dop-ratio-val','loc-map-update-last']
            .forEach((id) => { const el = document.getElementById(id); if (el) el.textContent = '—'; });
        const bar = document.getElementById('loc-analytics-cpu-bar-fill');
        if (bar) bar.style.width = '0%';
        const ramVal = document.getElementById('loc-analytics-ram-value');
        if (ramVal) ramVal.innerHTML = '—<span class="analytics-cpu-pct-sym"> MB</span>';
        const ramSub = document.getElementById('loc-analytics-ram-sub');
        if (ramSub) ramSub.textContent = '';
        const ramBar = document.getElementById('loc-analytics-ram-bar-fill');
        if (ramBar) ramBar.style.width = '0%';
        this._rugHistory.fill(false);
        this._drawRug();
        this._speedBuf = { timestamps: [], traj_dist: [] };
        this._updateCount  = 0;
        this._lastUpdateMs = null;
        const countEl = document.getElementById('loc-map-update-count');
        if (countEl) countEl.textContent = '0';
        const section = document.getElementById('loc-analytics-detail');
        const btn     = document.getElementById('loc-analytics-detail-toggle');
        if (section) section.classList.remove('open');
        if (btn)     btn.classList.remove('open');
        this._plotsInit = false;
    }
}

// LocAnalyticsDashboard 전역 인스턴스
const locAnalyticsDashboard = new LocAnalyticsDashboard();

function toggleLocAnalyticsDetail() {
    const section = document.getElementById('loc-analytics-detail');
    const btn     = document.getElementById('loc-analytics-detail-toggle');
    if (!section || !btn) return;
    const isOpen = section.classList.toggle('open');
    btn.classList.toggle('open', isOpen);
    btn.textContent = isOpen
        ? 'Collapse Statistics'
        : 'Full Statistics (Scan/Map · Feature Matching · Residual · IESEKF · DOP · Loc Status · Odometry …)';
}
