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
    } else if (plotState.ros.isConnected && plotState.topics.length === 0) {
        console.log('[initPlotSubtab] rosbridge already connected, loading topics');
        loadPlotTopics();
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
                    div.textContent = '📁 ' + entry.name;
                    div.onclick = () => loadDirectoryList(entry.path);
                } else {
                    div.textContent = '📄 ' + entry.name;
                    div.onclick = () => selectFile(entry.path);
                    div.style.color = '#aaaaaa';
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

function handleOptBtnClick() {
    if (_optComplete) {
        exitOptimization();
    } else {
        runOptimization();
    }
}

async function runOptimization() {
    slamResultViewer.hideAndReset();
    const runBtn = domCache.get('slam-opt-run-btn');
    runBtn.disabled = true;
    const result = await apiCall('/api/slam/optimize', {});
    if (result.success) {
        updateSlamStatus(result.status || 'Running...');
        _showOptStatus('Starting optimization...', true);
        _startOptPolling();
    } else {
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

    spinner.style.display = 'none';
    cancelBtn.style.display = 'none';

    area.style.transition = 'opacity 0.6s ease';
    area.style.opacity = '0';
    setTimeout(() => {
        area.style.display = 'none';
        area.style.opacity = '1';
        area.style.transition = '';
    }, 620);

    _optComplete = true;
    const runBtn = domCache.get('slam-opt-run-btn');
    runBtn.disabled = false;
    runBtn.textContent = 'Exit';

    slamResultViewer.show();
}

function _showOptError(message, autoHide = false) {
    const area = domCache.get('slam-opt-status-area');
    const msgEl = domCache.get('slam-opt-msg');
    const spinner = domCache.get('slam-opt-spinner');
    const cancelBtn = domCache.get('slam-opt-cancel-btn');

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
        if (state.status === 'Optimization complete!') {
            const viewerEl = document.getElementById('slam-result-viewer');
            if (viewerEl && viewerEl.style.display === 'none') {
                slamResultViewer.show();
            }
        }

        // Update LiDAR SLAM status (only if LiDAR SLAM tab is active)
        const lidarSlamStatus = domCache.get('lidar-slam-status');
        if (lidarSlamStatus) {
            const lidarSlamTab = document.getElementById('lidar-slam-subtab');
            if (lidarSlamTab && lidarSlamTab.classList.contains('active')) {
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
            // Get topics, duration and bag_type from result
            // topics는 string[] (ROS2) 또는 {name, type, publishable}[] (ROS1) 형태일 수 있음
            bagPlayerState.availableTopics = result.topics || [];
            bagPlayerState.bagDuration = result.duration || 0.0;
            bagPlayerState.bagType = result.bag_type || 'ros2';

            // ROS1 bag의 경우 선택 가능한(publishable) 토픽만 기본 선택
            if (bagPlayerState.bagType === 'ros1' && bagPlayerState.availableTopics.length > 0
                    && typeof bagPlayerState.availableTopics[0] === 'object') {
                bagPlayerState.selectedTopics = bagPlayerState.availableTopics
                    .filter(t => t.publishable)
                    .map(t => t.name);
            } else {
                bagPlayerState.selectedTopics = bagPlayerState.availableTopics.map(
                    t => (typeof t === 'object' ? t.name : t)
                );
            }

            console.log('Loaded topics:', bagPlayerState.availableTopics);
            console.log('Duration:', bagPlayerState.bagDuration, 'seconds');
            console.log('Bag type:', bagPlayerState.bagType);

            // Show/hide ROS1/ROS2 badge, convert button, and playback rate controls
            const isRos1 = bagPlayerState.bagType === 'ros1';
            domCache.get('bag-ros1-badge').style.display = isRos1 ? 'inline' : 'none';
            domCache.get('bag-ros2-badge').style.display = !isRos1 ? 'inline' : 'none';
            domCache.get('convert-to-ros2-btn').style.display = isRos1 ? 'inline-block' : 'none';
            domCache.get('convert-to-ros1-btn').style.display = !isRos1 ? 'inline-block' : 'none';
            // Rate 슬라이더: ROS1 / ROS2 bag 모두 표시
            const rateControls = domCache.get('ros1-playback-controls');
            if (rateControls) {
                rateControls.style.display = 'block';
            }
            // 슬라이더 레이블 업데이트 (bag 로드 시 초기화)
            updatePlaybackRate(document.getElementById('bag-playback-rate')?.value ?? 10);

            // Update time label
            updateBagTimeLabel(0, bagPlayerState.bagDuration);

            // Update selected topics display
            updateSelectedTopicsDisplay();

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
    // ROS1 bag 재생 중이면 /api/bag/ros1_play_status 폴링
    if (bagPlayerState.bagType === 'ros1') {
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
        // Loop 체크박스 동기화 (ROS1: /api/bag/state에서 loop 조회)
        const bagState = await apiCall('/api/bag/state');
        if (bagState && bagState.loop !== undefined) {
            const loopCb = domCache.get('bag-player-loop');
            if (loopCb) {
                loopCb.checked = bagState.loop;
            }
        }
        return;
    }

    // ROS2 bag: 기존 폴링 유지
    const state = await apiCall('/api/bag/state');
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
 * ROS1 bag 파일을 ROS2 포맷으로 변환
 * POST /api/bag/convert_ros1 호출 후 변환된 ROS2 bag 자동 로드
 */
async function convertToRos2() {
    const bagPath = domCache.get('bag-directory').value;
    if (!bagPath) {
        alert('Please load a ROS1 bag file first');
        return;
    }

    const btn = domCache.get('convert-to-ros2-btn');
    const originalText = btn.textContent;
    btn.disabled = true;
    btn.textContent = 'Converting...';

    try {
        const result = await apiCall('/api/bag/convert_ros1', { path: bagPath });
        if (result.success) {
            // 버튼 상태 항상 복원 (재사용 가능하도록)
            btn.disabled = false;
            btn.textContent = originalText;

            alert(`Conversion complete!\nOutput: ${result.output_path}`);

            // 변환된 ROS2 bag 자동 로드
            const outputPath = result.output_path;
            domCache.get('bag-directory').value = outputPath;
            const loadResult = await apiCall('/api/bag/load', { path: outputPath });
            if (loadResult.success) {
                bagPlayerState.availableTopics = loadResult.topics || [];
                bagPlayerState.selectedTopics = [...bagPlayerState.availableTopics];
                bagPlayerState.bagDuration = loadResult.duration || 0.0;
                bagPlayerState.bagType = loadResult.bag_type || 'ros2';

                // ROS1/ROS2 배지, Convert 버튼 업데이트; 속도 슬라이더는 유지
                const isRos1 = bagPlayerState.bagType === 'ros1';
                domCache.get('bag-ros1-badge').style.display = isRos1 ? 'inline' : 'none';
                domCache.get('bag-ros2-badge').style.display = !isRos1 ? 'inline' : 'none';
                domCache.get('convert-to-ros2-btn').style.display = isRos1 ? 'inline-block' : 'none';
                domCache.get('convert-to-ros1-btn').style.display = !isRos1 ? 'inline-block' : 'none';
                // 변환 후에도 rate 슬라이더는 표시 유지
                const ros1Controls = domCache.get('ros1-playback-controls');
                if (ros1Controls) {
                    ros1Controls.style.display = 'block';
                }

                updateBagTimeLabel(0, bagPlayerState.bagDuration);
                updateSelectedTopicsDisplay();
                if (typeof resetViewerTopicSubscriptions === 'function') {
                    resetViewerTopicSubscriptions();
                }
                if (typeof resetBagFrameAndTFState === 'function') {
                    resetBagFrameAndTFState();
                }
                console.log('Converted ROS2 bag loaded:', outputPath);
            }
        } else {
            alert('Conversion failed: ' + (result.error || 'Unknown error'));
            btn.disabled = false;
            btn.textContent = originalText;
        }
    } catch (error) {
        console.error('convertToRos2 error:', error);
        alert('Conversion failed: ' + error.message);
        btn.disabled = false;
        btn.textContent = originalText;
    }
}

async function convertToRos1() {
    const bagPath = domCache.get('bag-directory').value;
    if (!bagPath || bagPlayerState.bagType !== 'ros2') {
        alert('Please load a ROS2 bag first');
        return;
    }

    const btn = domCache.get('convert-to-ros1-btn');
    const originalText = btn.textContent;
    btn.disabled = true;
    btn.textContent = 'Converting...';

    try {
        const result = await apiCall('/api/bag/convert_to_ros1', {});
        btn.disabled = false;
        btn.textContent = originalText;
        if (result.success) {
            alert(`Conversion complete!\nOutput: ${result.output_path}`);
        } else {
            alert('Conversion failed: ' + (result.error || 'Unknown error'));
        }
    } catch (e) {
        btn.disabled = false;
        btn.textContent = originalText;
        alert('Conversion error: ' + e.message);
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
    const conprSaveRow = domCache.get('conpr-save-row');

    if (format === 'kitti') {
        kittiUi.style.display = 'block';
        if (kaistUi) { kaistUi.style.display = 'none'; }
        if (mulranUi) { mulranUi.style.display = 'none'; }
        if (conprSaveRow) { conprSaveRow.style.display = 'none'; }
        kittiState.baseDir = null;
        kittiState.calibDir = null;
        kittiState.drives = [];
        domCache.get('player-path-label').textContent = '—';
        _resetKittiDriveSelect();
        _resetKittiProgressBar();
    } else if (format === 'kaist') {
        if (kittiUi) { kittiUi.style.display = 'none'; }
        if (kaistUi) { kaistUi.style.display = 'block'; }
        if (mulranUi) { mulranUi.style.display = 'none'; }
        if (conprSaveRow) { conprSaveRow.style.display = 'none'; }
        kaistState.baseDir = null;
        kaistState.sequences = [];
        domCache.get('player-path-label').textContent = '—';
        _resetKaistSequenceSelect();
        _resetKaistProgressBar();
    } else if (format === 'mulran') {
        if (kittiUi) { kittiUi.style.display = 'none'; }
        if (kaistUi) { kaistUi.style.display = 'none'; }
        if (mulranUi) { mulranUi.style.display = 'block'; }
        if (conprSaveRow) { conprSaveRow.style.display = 'none'; }
        mulranState.baseDir = null;
        mulranState.sequences = [];
        domCache.get('player-path-label').textContent = '—';
        _resetMulranSequenceSelect();
        _resetMulranProgressBar();
    } else {
        if (kittiUi) { kittiUi.style.display = 'none'; }
        if (kaistUi) { kaistUi.style.display = 'none'; }
        if (mulranUi) { mulranUi.style.display = 'none'; }
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
    const bagFormat = bagFormatSel ? bagFormatSel.value : 'ros2';
    kittiState.bagFormat = bagFormat;

    const btn   = domCache.get('kitti-convert-btn');
    const bar   = domCache.get('kitti-progress-bar');
    const fill  = domCache.get('kitti-progress-fill');
    const text  = domCache.get('kitti-progress-text');
    const msgEl = domCache.get('kitti-progress-msg');

    kittiState.converting = true;
    btn.disabled = true;
    btn.textContent = bagFormat === 'ros1' ? 'Saving ROS1…' : 'Saving…';

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
    const bagFormat = bagFormatSel ? bagFormatSel.value : 'ros2';

    const btn   = domCache.get('kaist-convert-btn');
    const bar   = domCache.get('kaist-progress-bar');
    const fill  = domCache.get('kaist-progress-fill');
    const text  = domCache.get('kaist-progress-text');
    const msgEl = domCache.get('kaist-progress-msg');

    kaistState.converting = true;
    if (btn) { btn.disabled = true; btn.textContent = bagFormat === 'ros1' ? 'Saving ROS1…' : 'Saving…'; }
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
    const bagFormat = bagFormatSel ? bagFormatSel.value : 'ros2';

    const btn   = domCache.get('mulran-convert-btn');
    const bar   = domCache.get('mulran-progress-bar');
    const fill  = domCache.get('mulran-progress-fill');
    const text  = domCache.get('mulran-progress-text');
    const msgEl = domCache.get('mulran-progress-msg');

    mulranState.converting = true;
    if (btn) { btn.disabled = true; btn.textContent = bagFormat === 'ros1' ? 'Saving ROS1…' : 'Saving…'; }
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

    const bagFormat = bagFormatSel ? bagFormatSel.value : 'ros2';
    const originalBtnText = saveBagBtn ? saveBagBtn.textContent : 'Save bag';

    // KITTI와 완전 동일한 레이아웃: 진행바+메시지+format select+버튼 모두 표시, 버튼만 비활성화
    if (bar) { bar.style.display = 'block'; }
    if (fill) { fill.style.width = '0%'; }
    if (text) { text.textContent = '0%'; }
    if (msgEl) { msgEl.textContent = 'Starting conversion...'; }
    if (bagFormatSel) { bagFormatSel.disabled = true; }
    if (saveBagBtn) {
        saveBagBtn.disabled = true;
        saveBagBtn.textContent = bagFormat === 'ros1' ? 'Saving ROS1…' : 'Saving…';
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
                    div.textContent = '📁 ' + entry.name;
                    div.onclick = () => loadBagNameDirectory(entry.path);
                } else {
                    div.textContent = '📄 ' + entry.name;
                    div.style.color = '#aaaaaa';
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
        alert('No ROS2 topics found. Make sure ROS2 nodes are running.');
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

                // Set initial collapsed state
                const parametersDiv = domCache.get(this.containerIds.parameters);
                const toggleBtn = domCache.get(this.containerIds.toggleBtn);
                const separators = document.querySelectorAll(`#${this.containerIds.container} .separator`);

                parametersDiv.style.display = 'none';
                separators.forEach(sep => sep.style.display = 'none');
                toggleBtn.textContent = '▼';
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

                // Show config container
                domCache.get(this.containerIds.container).style.display = 'block';
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
            return;
        }

        if (targetPath === null) {
            targetPath = allowCurrentPathFallback ? (this.currentPath || this.defaultPath) : this.defaultPath;
        }
        if (!targetPath) {
            alert(`No ${this.name} config file path is available.`);
            return;
        }

        console.log(`Saving ${this.name} config to:`, targetPath);
        console.log('Config data:', this.data);

        const result = await apiCall(this.apiEndpoints.saveConfig, {
            path: targetPath,
            config: this.data
        });

        if (result.success) {
            const savedPath = result.path || targetPath;
            alert('Config file saved successfully to:\n' + savedPath);
            console.log(`${this.name} config saved to:`, savedPath);
        } else {
            alert('Failed to save config file: ' + (result.message || 'Unknown error'));
        }
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
                // This is a nested group
                nestedGroups.push({ key, data: value });
            } else {
                // This is a top-level parameter
                topLevelParams.push({ key, value });
            }
        });

        // Display top-level parameters first
        if (topLevelParams.length > 0) {
            const groupHeader = document.createElement('h4');
            groupHeader.textContent = 'General';
            groupHeader.style.marginTop = '20px';
            groupHeader.style.marginBottom = '10px';
            groupHeader.style.color = '#4a9eff';
            container.appendChild(groupHeader);

            topLevelParams.forEach(param => {
                this.createParameterInput(container, param.key, param.value, param.key);
            });
        }

        // Display nested groups
        nestedGroups.forEach(group => {
            const groupHeader = document.createElement('h4');
            // Convert snake_case to Title Case
            const title = group.key.split('_').map(word =>
                word.charAt(0).toUpperCase() + word.slice(1)
            ).join(' ');
            groupHeader.textContent = title;
            groupHeader.style.marginTop = '20px';
            groupHeader.style.marginBottom = '10px';
            groupHeader.style.color = '#4a9eff';
            container.appendChild(groupHeader);

            Object.keys(group.data).forEach(key => {
                const value = group.data[key];
                const fullKey = `${group.key}.${key}`;
                this.createParameterInput(container, key, value, fullKey);
            });
        });
    }

    createParameterInput(container, label, value, fullKey) {
        const formGroup = document.createElement('div');
        formGroup.className = 'form-group';
        formGroup.style.display = 'grid';
        formGroup.style.gridTemplateColumns = '200px 1fr';
        formGroup.style.alignItems = 'center';
        formGroup.style.marginBottom = '8px';

        const labelElement = document.createElement('label');
        labelElement.textContent = label + ':';
        labelElement.style.fontSize = '0.9em';
        formGroup.appendChild(labelElement);

        let inputElement;

        // Handle different value types
        if (typeof value === 'boolean') {
            const checkboxContainer = document.createElement('div');
            checkboxContainer.style.textAlign = 'right';
            checkboxContainer.style.paddingRight = '10px';

            inputElement = document.createElement('input');
            inputElement.type = 'checkbox';
            inputElement.checked = value;
            inputElement.id = `${this.name}-param-${fullKey}`;
            inputElement.dataset.configKey = fullKey;
            inputElement.dataset.valueType = 'boolean';
            inputElement.onchange = () => this.updateValue(fullKey, inputElement.checked);

            checkboxContainer.appendChild(inputElement);
            formGroup.appendChild(checkboxContainer);
            container.appendChild(formGroup);
            return;
        } else if (Array.isArray(value)) {
            inputElement = document.createElement('input');
            inputElement.type = 'text';
            // Keep original number precision (don't round)
            const formattedValue = value.map(v => String(v));
            inputElement.value = '[' + formattedValue.join(', ') + ']';
            inputElement.id = `${this.name}-param-${fullKey}`;
            inputElement.dataset.configKey = fullKey;
            inputElement.dataset.valueType = 'array';
            inputElement.style.width = '100%';
            inputElement.onchange = () => {
                try {
                    // Parse the input, removing spaces for flexibility
                    const cleanedInput = inputElement.value.replace(/\s/g, '');
                    const parsedValue = JSON.parse(cleanedInput);
                    this.updateValue(fullKey, parsedValue);
                } catch (e) {
                    alert('Invalid array format. Use format: [1.0, 0.0, 0.0]');
                }
            };
        } else if (typeof value === 'number') {
            inputElement = document.createElement('input');
            inputElement.type = 'number';
            inputElement.value = value;
            inputElement.id = `${this.name}-param-${fullKey}`;
            inputElement.step = 'any';  // Allow any decimal precision
            inputElement.dataset.configKey = fullKey;
            inputElement.dataset.valueType = 'number';
            inputElement.style.width = '100%';
            inputElement.onchange = () => {
                const numValue = parseFloat(inputElement.value);
                this.updateValue(fullKey, numValue);
            };
        } else if (typeof value === 'string') {
            inputElement = document.createElement('input');
            inputElement.type = 'text';
            inputElement.value = value;
            inputElement.id = `${this.name}-param-${fullKey}`;
            inputElement.dataset.configKey = fullKey;
            inputElement.dataset.valueType = 'string';
            inputElement.style.width = '100%';
            inputElement.onchange = () => this.updateValue(fullKey, inputElement.value);
        } else {
            inputElement = document.createElement('span');
            inputElement.textContent = String(value);
        }

        formGroup.appendChild(inputElement);
        container.appendChild(formGroup);
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
        const separators = document.querySelectorAll(`#${this.containerIds.container} .separator`);

        this.collapsed = !this.collapsed;

        if (this.collapsed) {
            // Collapse
            parametersDiv.style.display = 'none';
            separators.forEach(sep => sep.style.display = 'none');
            toggleBtn.textContent = '▼';
        } else {
            // Expand
            parametersDiv.style.display = 'block';
            separators.forEach(sep => sep.style.display = 'block');
            toggleBtn.textContent = '▲';
        }
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
        toggleBtn: 'slam-config-toggle-btn'
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
        toggleBtn: 'localization-config-toggle-btn'
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
    // Open save map modal
    domCache.get('save-map-modal').style.display = 'block';
    domCache.get('save-map-directory').value = '';
    domCache.get('save-map-directory').focus();
}

function closeSaveMapModal() {
    domCache.get('save-map-modal').style.display = 'none';
}

let _saveMapPollTimer = null;

async function confirmSaveMap() {
    const directoryName = domCache.get('save-map-directory').value.trim();

    if (!directoryName) {
        alert('Please enter a directory name');
        return;
    }

    closeSaveMapModal();

    const result = await apiCall('/api/slam/save_map', { directory: directoryName });

    if (result.success) {
        _showSaveMapStatus('Saving map to "' + directoryName + '"...', true);
        _startSaveMapPolling();
    } else {
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

async function _pollSaveMapStatus() {
    try {
        const status = await apiCall('/api/slam/save_map_status');

        if (status.done) {
            clearInterval(_saveMapPollTimer);
            _saveMapPollTimer = null;

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
    // Immediately update status to Running
    updateLidarSlamStatus('Running');
    
    const result = await apiCall('/api/slam/start_mapping', {});
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
    // Immediately update status to Stopping
    updateLidarSlamStatus('Stopping...');
    
    const result = await apiCall('/api/slam/stop_mapping', {});
    if (result.success) {
        console.log('SLAM mapping stopped');
        // Wait a bit for process to fully stop, then update to Ready
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
        // 재진입 복원: locLiveViewer가 선언된 후에만 실행
        if (typeof locLiveViewer !== 'undefined') {
            if (state.is_running && !locLiveViewer._visible) {
                locLiveViewer.show();
            } else if (!state.is_running && locLiveViewer._visible) {
                locLiveViewer.hide();
            }
        }
    }
}

// ==============================================================
// Localization Start/Stop (terminal output removed)
// ==============================================================
async function startLocalizationMapping() {
    updateLocalizationStatus('Running');
    
    const result = await apiCall('/api/localization/start_mapping', {});
    if (result.success) {
        console.log('Localization mapping started');
        locLiveViewer.show();
    } else {
        alert('Failed to start Localization mapping: ' + (result.message || 'Unknown error'));
        console.error('Failed to start Localization mapping');
        updateLocalizationStatus('Ready');
    }
}

async function stopLocalizationMapping() {
    updateLocalizationStatus('Stopping...');
    
    console.log('Stopping Localization mapping...');
    const result = await apiCall('/api/localization/stop_mapping', {});

    locLiveViewer.hide();

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
// 병렬 요청 방식은 동시에 여러 HTTP 스레드가 경쟁해 측정값 왜곡.
// 순차 최소값 방식: 1회씩 차례로 보내고 가장 빠른 RTT를 표시한다.
// → 큐잉 지연을 제외한 실제 서버 응답 시간에 가장 가까운 값.
const _LATENCY_PING_SAMPLES = 3;

async function measureLatency() {
    const latencyElement = document.getElementById('latency-indicator');
    if (!latencyElement) return;

    try {
        let minLatency = Infinity;
        for (let i = 0; i < _LATENCY_PING_SAMPLES; i++) {
            try {
                const t0 = performance.now();
                const response = await fetch('/api/ping', { cache: 'no-store' });
                const dt = performance.now() - t0;
                if (response.ok && dt < minLatency) minLatency = dt;
            } catch (_) { /* 개별 실패는 무시하고 나머지 샘플 계속 */ }
        }

        if (!isFinite(minLatency)) {
            latencyElement.textContent = 'latency: N/A';
            latencyElement.style.color = '#888';
            return;
        }

        const latency = Math.round(minLatency);
        latencyElement.textContent = `latency: ${latency}ms`;

        if (latency < 50) {
            latencyElement.style.color = '#4CAF50';
        } else if (latency < 150) {
            latencyElement.style.color = '#FFC107';
        } else {
            latencyElement.style.color = '#F44336';
        }

    } catch (error) {
        latencyElement.textContent = 'latency: N/A';
        latencyElement.style.color = '#888';
    }
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
    const host = window.location.hostname;
    const url  = `ws://${host}:8081`;

    if (plotState.backendWs &&
        (plotState.backendWs.readyState === WebSocket.OPEN ||
         plotState.backendWs.readyState === WebSocket.CONNECTING)) {
        return; // 이미 연결 중
    }

    const ws = new WebSocket(url);
    ws.binaryType = 'arraybuffer'; // binary 메시지는 무시 (PC2 binary는 worker가 처리)
    plotState.backendWs = ws;

    ws.onopen = () => {
        console.log('[BackendWs] 연결됨:', url);
        // 대기 중이던 subscribe_plot 명령 전송
        const pending = plotState._pendingPlotSubs.splice(0);
        for (const req of pending) {
            ws.send(JSON.stringify(req));
        }
    };

    ws.onmessage = (evt) => {
        if (typeof evt.data === 'string') {
            _handleBackendWsMessage(evt.data);
        }
        // binary(PC2 포인트클라우드)는 pc2_stream_worker.js가 처리 — 여기서는 무시
    };

    ws.onerror = () => {
        console.warn('[BackendWs] 연결 오류');
    };

    ws.onclose = () => {
        console.log('[BackendWs] 연결 끊김, 3초 후 재연결...');
        plotState.backendWs = null;
        setTimeout(_initBackendWs, 3000);
    };
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

function initRosbridge() {
    if (typeof ROSLIB === 'undefined') {
        console.error('ROSLIB not loaded');
        return;
    }

    try {
        plotState.ros = new ROSLIB.Ros({
            url: 'ws://localhost:9090'
        });

        plotState.ros.on('connection', () => {
            console.log('[rosbridge] Connected to rosbridge');
            updateRosbridgeStatusChip('connected');
            loadPlotTopics();
        });

        plotState.ros.on('error', (error) => {
            console.error('[rosbridge] Connection error:', error);
            updateRosbridgeStatusChip('disconnected');
            const container = domCache.get('plot-tree');
            if (container) {
                plotState.tree = null;
                container.innerHTML = '<div class="plot-tree-status-msg" style="color: var(--warning); padding: 12px; text-align: center;">rosbridge connection failed. Make sure rosbridge is running on port 9090.</div>';
            }
        });

        plotState.ros.on('close', () => {
            console.log('[rosbridge] Connection closed. Attempting to reconnect...');
            updateRosbridgeStatusChip('reconnecting');
            const container = domCache.get('plot-tree');
            if (container) {
                plotState.tree = null;
                container.innerHTML = '<div class="plot-tree-status-msg" style="color: var(--muted); padding: 12px; text-align: center;">rosbridge disconnected. Reconnecting...</div>';
            }
            setTimeout(() => {
                initRosbridge(); // 재연결 시도
            }, 3000);
        });
    } catch (error) {
        console.error('[rosbridge] Failed to initialize:', error);
    }
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

    // 이미 로딩 중이면 스킵
    if (plotState.isLoadingTopics) {
        console.log('[loadPlotTopics] Already loading topics, skipping...');
        return;
    }

    plotState.isLoadingTopics = true;

    try {
        // 타임아웃 설정 (10초로 증가)
        const timeout = 10000;
        let timeoutId = null;
        let completed = false;

        // 타임아웃 Promise
        const timeoutPromise = new Promise((_, reject) => {
            timeoutId = setTimeout(() => {
                if (!completed) {
                    reject(new Error('Topic loading timeout'));
                }
            }, timeout);
        });

        // getTopics Promise
        const getTopicsPromise = new Promise((resolve, reject) => {
            try {
                plotState.ros.getTopics((result) => {
                    completed = true;
                    clearTimeout(timeoutId);
                    resolve(result);
                }, (error) => {
                    completed = true;
                    clearTimeout(timeoutId);
                    reject(error);
                });
            } catch (error) {
                completed = true;
                clearTimeout(timeoutId);
                reject(error);
            }
        });

        // 경쟁: getTopics vs timeout
        const result = await Promise.race([getTopicsPromise, timeoutPromise]);

        const topics = result.topics || [];
        const types = result.types || [];
        
        console.log('[loadPlotTopics] Received topics:', topics.length);
        console.log('[loadPlotTopics] Topic list:', topics);
        
        // topics와 types를 Map으로 저장 (별도 저장)
        const topicTypesMap = new Map();
        topics.forEach((name, index) => {
            topicTypesMap.set(name, types[index] || 'unknown');
        });
        plotState.topicTypes = topicTypesMap;

        const oldTopicsSet = new Set(plotState.topics);
        const newTopicsSet = new Set(topics);
        const addedTopics = topics.filter((t) => !oldTopicsSet.has(t));
        if (addedTopics.length > 0) {
            console.log('[loadPlotTopics] New topics detected:', addedTopics);
        }
        const removedTopics = plotState.topics.filter((t) => !newTopicsSet.has(t));
        if (removedTopics.length > 0) {
            console.log('[loadPlotTopics] Removed topics:', removedTopics);
        }

        plotState.topics = topics;

        // ROS 그래프에 없어진 토픽은 왼쪽 패널에서 자동 제거
        const rosSet = new Set(plotState.topics);
        const removedFromPanel = plotState.addedPlotTopics.filter((t) => !rosSet.has(t));
        removedFromPanel.forEach((t) => unselectPlotTopic(t));
        plotState.addedPlotTopics = plotState.addedPlotTopics.filter((t) => rosSet.has(t));

        displayTopicList();
    } catch (error) {
        console.error('[loadPlotTopics] Error:', error);
        
        // 타임아웃이 발생했지만 이미 토픽 목록이 있는 경우 (기존 플롯이 동작 중)
        if (plotState.topics && plotState.topics.length > 0) {
            console.warn('[loadPlotTopics] Timeout occurred, but keeping existing topics');
            // 기존 UI 유지, 에러 메시지는 콘솔에만 출력
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
        this._pathObj = null;
        this._tfObjects = {};
        this._mapMesh = null;
        this._mapTexture = null;
        this._layers = {
            cloud_registered: true,
            laser_map: true,
            path: true,
            tf: true,
            map: true
        };
        this._fixedFrame = 'odom';
        this._topView = false;
        this._savedCameraPos = null;
        this._savedCameraUp = null;
        this._savedTarget = null;
        this._knownFrames = new Set();
        this._resizeObserver = null;
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
        this._camera.position.set(0, -30, 20);
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
        const animate = () => {
            this._animFrameId = requestAnimationFrame(animate);
            if (this._controls) this._controls.update();
            // OrthographicCamera 탑뷰 시: zoom 변화에 따라 포인트 픽셀 크기 갱신
            if (this._topView && this._orthoCamera) {
                this._updateOrthoPointSizes();
            }
            if (this._renderer && this._scene && this._camera) {
                this._renderer.render(this._scene, this._camera);
            }
        };
        animate();
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
        const update = (obj) => {
            if (!obj || !obj.material) return;
            const baseSize = obj.material._baseSize || 0.1;
            obj.material.size = Math.max(1, baseSize * scale);
        };
        update(this._cloudObj);
        update(this._mapObj);
    }

    _resizeRenderer() {
        if (!this._renderer) return;
        const container = document.getElementById('loc-viewer-canvas-container');
        if (!container) return;
        const w = container.clientWidth;
        const h = container.clientHeight;
        if (w > 0 && h > 0) {
            this._renderer.setSize(w, h);
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
        this._resizeRenderer();
        this._connectAndSubscribe();
    }

    hide() {
        this._visible = false;
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
        removeObj(this._pathObj);
        this._cloudObj = null;
        this._mapObj = null;
        this._pathObj = null;

        for (const key of Object.keys(this._tfObjects)) {
            const entry = this._tfObjects[key];
            if (entry && entry.group) this._scene.remove(entry.group);
        }
        this._tfObjects = {};

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

    _connectAndSubscribe() {
        const loadingEl = document.getElementById('loc-viewer-loading');
        if (window.plotState && plotState.ros && plotState.ros.isConnected) {
            this._ros = plotState.ros;
            if (loadingEl) loadingEl.style.display = 'none';
            this._subscribeAll();
        } else {
            if (loadingEl) loadingEl.style.display = 'flex';
            try {
                this._ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
                this._ros.on('connection', () => {
                    console.log('[LocalizationLiveViewer] rosbridge connected');
                    if (loadingEl) loadingEl.style.display = 'none';
                    this._subscribeAll();
                });
                this._ros.on('error', (err) => {
                    console.error('[LocalizationLiveViewer] rosbridge error:', err);
                });
                this._ros.on('close', () => {
                    console.warn('[LocalizationLiveViewer] rosbridge connection closed');
                });
            } catch (e) {
                console.error('[LocalizationLiveViewer] failed to init rosbridge:', e);
            }
        }
    }

    _subscribeAll() {
        this._subscribePointCloud('/cloud_registered', 'cloud_registered');
        this._subscribePointCloudLatched('/Laser_map', 'laser_map');
        this._subscribePath('/path');
        this._subscribeTF('/tf');
        this._subscribeMap('/map');
    }

    _unsubscribeAll() {
        for (const t of this._subscriptions) {
            try { t.unsubscribe(); } catch (e) { /* ignore */ }
        }
        this._subscriptions = [];
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
        const pointSize = (key === 'laser_map') ? 0.08 : 0.12;
        const opacity   = (key === 'laser_map') ? 0.5  : 1.0;
        const transparent = (key === 'laser_map');

        if (existing && existing.geometry) {
            const posAttr = existing.geometry.getAttribute('position');
            const colAttr = existing.geometry.getAttribute('color');
            if (posAttr && posAttr.array.length >= parsed.positions.length) {
                posAttr.array.set(parsed.positions);
                posAttr.needsUpdate = true;
                colAttr.array.set(parsed.colors);
                colAttr.needsUpdate = true;
                existing.geometry.setDrawRange(0, newCount);
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
        geo.setDrawRange(0, newCount);

        const mat = new THREE.PointsMaterial({
            size: pointSize,
            sizeAttenuation: true,
            vertexColors: true,
            transparent,
            opacity
        });
        mat._baseSize = pointSize; // 탑뷰 ortho 보정용 기준 크기 저장
        // 탑뷰 활성 상태에서 새 mesh가 생성되는 경우 즉시 크기 보정
        if (this._topView && this._orthoCamera) {
            const scale = this._getOrthoPixelsPerUnit();
            mat.size = Math.max(1, pointSize * scale);
        }
        const points = new THREE.Points(geo, mat);
        points.visible = this._layers[key];

        if (key === 'cloud_registered') {
            this._cloudObj = points;
        } else {
            this._mapObj = points;
        }
        this._scene.add(points);
    }

    _subscribePointCloud(topic, key) {
        // Python 백엔드 binary WebSocket (8081) 사용
        // rosbridge JSON+base64 대비 전송 크기 ~4배 감소, JSON 파싱 오버헤드 제거 → 부드러운 실시간 시각화
        const hostname = window.location.hostname || 'localhost';
        const ws = new WebSocket(`ws://${hostname}:8081`);
        ws.binaryType = 'arraybuffer';

        ws.onopen = () => {
            ws.send(JSON.stringify({ cmd: 'subscribe', topic }));
        };

        ws.onmessage = (ev) => {
            if (!(ev.data instanceof ArrayBuffer)) return; // JSON meta 무시
            const parsed = this._parseBinaryPC2(ev.data);
            if (parsed) this._updatePointCloud(key, parsed);
        };

        ws.onerror = (e) => console.error('[LocalizationLiveViewer] PC2 WS error:', e);

        this._subscriptions.push({
            unsubscribe: () => {
                try {
                    if (ws.readyState === WebSocket.OPEN) {
                        ws.send(JSON.stringify({ cmd: 'unsubscribe', topic }));
                    }
                    ws.close();
                } catch (e) { /* ignore */ }
            }
        });
    }

    // /Laser_map 은 TRANSIENT_LOCAL + RELIABLE QoS 로 발행됨
    // rosbridge는 volatile QoS 구독이라 latched 메시지를 받지 못함
    // → Python 백엔드(8081)에 subscribe_latched 명령으로 직접 연결
    _subscribePointCloudLatched(topic, key) {
        const hostname = window.location.hostname || 'localhost';
        const ws = new WebSocket(`ws://${hostname}:8081`);
        ws.binaryType = 'arraybuffer';

        ws.onopen = () => {
            ws.send(JSON.stringify({ cmd: 'subscribe_latched', topic }));
            console.log(`[LocalizationLiveViewer] subscribe_latched 전송: ${topic}`);
        };

        ws.onmessage = (ev) => {
            if (!(ev.data instanceof ArrayBuffer)) return; // JSON meta 무시
            const parsed = this._parseBinaryPC2(ev.data);
            if (parsed) this._updatePointCloud(key, parsed);
        };

        ws.onerror = (e) => {
            console.error('[LocalizationLiveViewer] PC2 WS error:', e);
        };

        this._subscriptions.push({
            unsubscribe: () => {
                try {
                    if (ws.readyState === WebSocket.OPEN) {
                        ws.send(JSON.stringify({ cmd: 'unsubscribe_latched', topic }));
                    }
                    ws.close();
                } catch (e) { /* ignore */ }
            }
        });
    }

    // Python 백엔드 binary PC2 패킷 파싱
    // 패킷 포맷: [3B]'PC2' [1B]version [1B]flags [4B]topicLen [4B]frameLen [4B]count
    //            [topicLen]topic [frameLen]frameId [count*12]XYZ [count*4]colorF32 ([count*4]rgb)
    _parseBinaryPC2(buffer) {
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

            const MAX_PTS = 80000;
            const step    = Math.max(1, Math.floor(count / MAX_PTS));
            const outPts  = Math.ceil(count / step);
            const positions = new Float32Array(outPts * 3);
            const colors    = new Float32Array(outPts * 3);
            const tempZ     = new Float32Array(outPts);
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
            messageType: 'nav_msgs/msg/Path',
            throttle_rate: 200,
            queue_length: 1
        });
        t.subscribe((msg) => {
            const THREE = window.THREE;
            if (!this._scene || !THREE) return;
            if (this._pathObj) {
                this._scene.remove(this._pathObj);
                if (this._pathObj.geometry) this._pathObj.geometry.dispose();
                if (this._pathObj.material) this._pathObj.material.dispose();
                this._pathObj = null;
            }
            const poses = msg.poses || [];
            if (poses.length < 2) return;
            // TubeGeometry로 굵은 선 렌더링 (WebGL linewidth 제한 우회)
            const points3d = poses.map(p => new THREE.Vector3(
                p.pose.position.x, p.pose.position.y, p.pose.position.z
            ));
            const curve = new THREE.CatmullRomCurve3(points3d);
            const segments = Math.min(poses.length * 2, 400);
            const geo = new THREE.TubeGeometry(curve, segments, 0.08, 5, false);
            const mat = new THREE.MeshBasicMaterial({ color: 0x00ff44, side: THREE.DoubleSide });
            this._pathObj = new THREE.Mesh(geo, mat);
            this._pathObj.visible = this._layers.path;
            this._scene.add(this._pathObj);
        });
        this._subscriptions.push(t);
    }

    _subscribeTF(topic) {
        const t = new ROSLIB.Topic({
            ros: this._ros,
            name: topic,
            messageType: 'tf2_msgs/msg/TFMessage',
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

                if (!this._tfObjects[childId]) {
                    const group = new THREE.Group();
                    group.add(new THREE.AxesHelper(0.5));
                    this._tfObjects[childId] = { group };
                    group.visible = this._layers.tf;
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
            messageType: 'nav_msgs/msg/OccupancyGrid',
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
            this._mapMesh.visible = this._layers.map;
            this._scene.add(this._mapMesh);
        });
        this._subscriptions.push(t);
    }

    toggleLayer(key) {
        this._layers[key] = !this._layers[key];
        const visible = this._layers[key];
        if (key === 'cloud_registered' && this._cloudObj) {
            this._cloudObj.visible = visible;
        } else if (key === 'laser_map' && this._mapObj) {
            this._mapObj.visible = visible;
        } else if (key === 'path' && this._pathObj) {
            this._pathObj.visible = visible;
        } else if (key === 'tf') {
            for (const entry of Object.values(this._tfObjects)) {
                if (entry && entry.group) entry.group.visible = visible;
            }
        } else if (key === 'map' && this._mapMesh) {
            this._mapMesh.visible = visible;
        }
    }

    resetView() {
        if (!this._camera || !this._controls) return;
        this._camera.position.set(0, -30, 20);
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
            this._updateOrthoPointSizes();
        } else {
            // PerspectiveCamera 복원
            this._camera = this._perspCamera;
            this._controls.object = this._perspCamera;
            this._restoreOrbitControls();

            // Perspective 복원 시 포인트 크기를 원래 월드 단위 크기로 되돌림
            const restoreSize = (obj) => {
                if (obj && obj.material && obj.material._baseSize !== undefined) {
                    obj.material.size = obj.material._baseSize;
                }
            };
            restoreSize(this._cloudObj);
            restoreSize(this._mapObj);

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
// SLAM Result Viewer
// ==============================================================
class SlamResultViewer {
    constructor() {
        this._scene = null;
        this._camera = null;
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
        this._diffPaths = null;
        this._layers = {};
        this._lcObjects = [];
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
        const canvas = document.getElementById('slam-result-canvas');
        if (!canvas) return;

        const container = document.getElementById('slam-result-canvas-container');
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

        this._controls = new window.OrbitControls(this._camera, this._renderer.domElement);
        this._controls.enableDamping = true;
        this._controls.dampingFactor = 0.1;

        this._scene.add(new THREE.AxesHelper(5));
        this._initialized = true;
        this._startRenderLoop();
    }

    _startRenderLoop() {
        const animate = () => {
            this._animFrameId = requestAnimationFrame(animate);
            if (this._controls) this._controls.update();
            if (this._renderer && this._scene && this._camera) {
                this._renderer.render(this._scene, this._camera);
            }
        };
        animate();
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
            const c = document.getElementById('slam-result-canvas-container');
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

        return new THREE.Points(geo, new THREE.PointsMaterial({
            color,
            size: 16,
            sizeAttenuation: false,
            map: SlamResultViewer._circleTexture,
            alphaTest: 0.5,
            transparent: true,
        }));
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

    _fitCamera() {
        if (!this._scene) return;
        const THREE = window.THREE;
        let maxDim = 100;
        if (this._allObjects.length > 0) {
            const box = new THREE.Box3();
            for (const obj of this._allObjects) {
                box.expandByObject(obj);
            }
            if (!box.isEmpty()) {
                const size = new THREE.Vector3();
                box.getSize(size);
                maxDim = Math.max(size.x, size.y, size.z);
            }
        }
        this._camera.position.set(0, -maxDim * 0.8, maxDim * 0.4);
        this._camera.up.set(0, 0, 1);
        this._controls.target.set(0, 0, 0);
        this._controls.update();
    }

    _applyTopView() {
        let dist = 200;
        if (this._allObjects.length > 0) {
            const THREE = window.THREE;
            const box = new THREE.Box3();
            for (const obj of this._allObjects) {
                box.expandByObject(obj);
            }
            if (!box.isEmpty()) {
                const size = new THREE.Vector3();
                box.getSize(size);
                dist = Math.max(size.x, size.y, size.z) * 1.2;
            }
        }
        this._camera.position.set(0, 0, dist);
        this._camera.up.set(0, 1, 0);
        this._controls.target.set(0, 0, 0);
        this._controls.minPolarAngle = 0;
        this._controls.maxPolarAngle = 0.01;
        this._controls.mouseButtons = {
            LEFT: window.THREE.MOUSE.PAN,
            MIDDLE: window.THREE.MOUSE.DOLLY,
            RIGHT: window.THREE.MOUSE.PAN
        };
        this._controls.update();
    }

    async load() {
        if (this._loaded || this._loading) return;
        this._loading = true;
        const loadingEl = document.getElementById('slam-result-loading');
        if (loadingEl) loadingEl.style.display = 'block';
        this._clearScene();

        try {
            await this._loadAndRender();
            this._loaded = true;
        } catch (e) {
            console.error('SlamResultViewer: load failed:', e);
        } finally {
            this._loading = false;
            if (loadingEl) loadingEl.style.display = 'none';
        }
    }

    async _loadAndRender() {
        const THREE = window.THREE;
        const paths = await apiCall('/api/slam/result_paths');
        if (!paths) throw new Error('Failed to fetch result paths');

        const [poses1, poses2, posesOut] = await Promise.all([
            this._fetchPoses(paths.map1_poses),
            this._fetchPoses(paths.map2_poses),
            this._fetchPoses(paths.output_poses),
        ]);

        const pcd1 = paths.map1_pcd ? await this._loadPCD(paths.map1_pcd) : null;
        const pcd2 = paths.map2_pcd ? await this._loadPCD(paths.map2_pcd) : null;

        if (pcd1) {
            pcd1.material = new THREE.PointsMaterial({ color: 0x4488ff, size: this._pcdPointSize, sizeAttenuation: true, vertexColors: false });
            this._addToScene(pcd1, 'map1');
            this._pcdObjects.push(pcd1);
        }
        if (pcd2) {
            pcd2.material = new THREE.PointsMaterial({ color: 0x44dd88, size: this._pcdPointSize, sizeAttenuation: true, vertexColors: false });
            this._addToScene(pcd2, 'map2');
            this._pcdObjects.push(pcd2);
        }

        const flat1 = this._posesToFlat(poses1);
        const flat2 = this._posesToFlat(poses2);
        const flatOut = this._posesToFlat(posesOut);

        if (flat1.length >= 3) this._addToScene(this._makeNodes(flat1, 0xffee44), 'map1traj');
        if (flat2.length >= 3) this._addToScene(this._makeNodes(flat2, 0xff9900), 'map2traj');
        if (flatOut.length >= 3) this._addToScene(this._makeNodes(flatOut, 0x44ffff), 'outputtraj');

        if (paths.output_edges && posesOut.length > 0) {
            await this._loadLoopClosures(paths.output_edges, posesOut);
        }

        this._fitCamera();
    }

    async _loadLoopClosures(edgesPath, poses) {
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
                const container = document.getElementById('slam-result-canvas-container');
                const lsMat = new window.LineMaterial({
                    color: 0xff2266,
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
                    new THREE.LineBasicMaterial({ color: 0xff2266, transparent: true, opacity: 0.85, depthTest: false })
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
        const container = document.getElementById('slam-result-canvas-container');
        if (!container) return;
        const w = container.clientWidth;
        const h = container.clientHeight;
        if (w > 0 && h > 0) {
            this._renderer.setSize(w, h);
            this._camera.aspect = w / h;
            this._camera.updateProjectionMatrix();
        }
    }

    async show() {
        const viewerEl = document.getElementById('slam-result-viewer');
        if (viewerEl) viewerEl.style.display = 'block';
        await this._init();
        this._resizeRenderer();
        await this.load();
    }

    hide() {
        const viewerEl = document.getElementById('slam-result-viewer');
        if (viewerEl) viewerEl.style.display = 'none';
    }

    _restoreOrbitControls() {
        this._controls.enableRotate = true;
        this._controls.minPolarAngle = 0;
        this._controls.maxPolarAngle = Math.PI;
        this._controls.mouseButtons = {
            LEFT: window.THREE.MOUSE.ROTATE,
            MIDDLE: window.THREE.MOUSE.DOLLY,
            RIGHT: window.THREE.MOUSE.PAN
        };
    }

    resetView() {
        const toggle = document.getElementById('slam-viewer-topview-toggle');
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
            this._savedCameraPos = this._camera.position.clone();
            this._savedCameraUp  = this._camera.up.clone();
            this._savedTarget    = this._controls.target.clone();
            this._applyTopView();
        } else {
            this._restoreOrbitControls();
            if (this._savedCameraPos) {
                this._camera.position.copy(this._savedCameraPos);
                this._camera.up.copy(this._savedCameraUp);
                this._controls.target.copy(this._savedTarget);
                this._savedCameraPos = null;
            } else {
                this._fitCamera();
                return;
            }
            this._controls.update();
        }
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
        points.material = new THREE.PointsMaterial({
            color,
            size: this._pcdPointSize * 1.2,
            sizeAttenuation: true,
            vertexColors: false,
        });
        this._scene.add(points);
        this._diffObjects.push(points);
        if (layerName) {
            if (!this._layers[layerName]) this._layers[layerName] = [];
            this._layers[layerName].push(points);
        }
    }

    async toggleDiffPCDs(enabled) {
        const diffLegend = document.getElementById('slam-diff-legend-rows');
        if (enabled) {
            if (!this._diffLoaded) {
                const loadingEl = document.getElementById('slam-result-loading');
                if (loadingEl) loadingEl.style.display = 'block';
                try {
                    if (!this._diffPaths) {
                        this._diffPaths = await apiCall('/api/slam/result_paths');
                    }
                    const paths = this._diffPaths;
                    // PD / ND — 없을 수 있으므로 각각 독립 try
                    try { if (paths && paths.pd_pcd) await this._loadDiffPCD(paths.pd_pcd, 0xff6600, 'pd'); } catch (e) { console.warn('PD.pcd not available'); }
                    try { if (paths && paths.nd_pcd) await this._loadDiffPCD(paths.nd_pcd, 0xdd00ff, 'nd'); } catch (e) { console.warn('ND.pcd not available'); }
                    // FirstUE / SecondUE — 없을 수 있으므로 각각 독립 try
                    try { if (paths && paths.first_ue_pcd) await this._loadDiffPCD(paths.first_ue_pcd, 0xff0066, 'firstue'); } catch (e) { console.warn('FirstUE.pcd not available'); }
                    try { if (paths && paths.second_ue_pcd) await this._loadDiffPCD(paths.second_ue_pcd, 0xaaff00, 'secondue'); } catch (e) { console.warn('SecondUE.pcd not available'); }
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
    }

    _restoreDiffLayerVisuals() {
        for (const name of ['pd', 'nd', 'firstue', 'secondue']) {
            const row = document.querySelector(`.slam-legend-row[data-layer="${name}"]`);
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
        this._diffPaths = null;
        ['pd', 'nd', 'firstue', 'secondue'].forEach(k => delete this._layers[k]);
    }

    toggleLayer(name) {
        const objs = this._layers[name];
        const row = document.querySelector(`.slam-legend-row[data-layer="${name}"]`);
        if (!objs || objs.length === 0) return;
        const nowVisible = objs[0].visible;
        const newVisible = !nowVisible;
        for (const obj of objs) { obj.visible = newVisible; }
        if (row) row.dataset.active = String(newVisible);
    }

    setPointSize(size) {
        this._pcdPointSize = size;
        for (const obj of this._pcdObjects) {
            if (obj.material) {
                obj.material.size = size;
                obj.material.needsUpdate = true;
            }
        }
        for (const obj of this._diffObjects) {
            if (obj.material) {
                obj.material.size = size * 1.2;
                obj.material.needsUpdate = true;
            }
        }
        const label = document.getElementById('slam-point-size-label');
        if (label) label.textContent = size.toFixed(2) + ' m';
    }

    _resetAllLegendRows() {
        document.querySelectorAll('.slam-legend-row[data-layer]').forEach(row => {
            row.dataset.active = 'true';
        });
    }

    takeSnapshot(scale = 2) {
        if (!this._renderer || !this._scene || !this._camera) return;
        const container = document.getElementById('slam-result-canvas-container');
        if (!container) return;

        const w = container.clientWidth;
        const h = container.clientHeight;
        const timestamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
        const filename = `slam_snapshot_${timestamp}.png`;

        this._renderer.setSize(w * scale, h * scale);
        this._camera.aspect = w / h;
        this._camera.updateProjectionMatrix();
        this._renderer.render(this._scene, this._camera);

        const canvas = this._renderer.domElement;
        canvas.toBlob((blob) => {
            this._renderer.setSize(w, h);
            this._camera.aspect = w / h;
            this._camera.updateProjectionMatrix();

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
        this._savedCameraPos = null;
        this._savedCameraUp = null;
        this._savedTarget = null;
        if (this._controls) {
            this._restoreOrbitControls();
        }
        const topViewToggle = document.getElementById('slam-viewer-topview-toggle');
        if (topViewToggle) topViewToggle.checked = false;
        this._clearDiff();
        const diffToggle = document.getElementById('slam-viewer-diff-toggle');
        if (diffToggle) diffToggle.checked = false;
        const diffLegend = document.getElementById('slam-diff-legend-rows');
        if (diffLegend) diffLegend.style.display = 'none';
        const slider = document.getElementById('slam-point-size-slider');
        if (slider) {
            slider.value = '0.05';
            this._pcdPointSize = 0.05;
            const label = document.getElementById('slam-point-size-label');
            if (label) label.textContent = '0.05 m';
        }
    }
}

const slamResultViewer = new SlamResultViewer();

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
// LocalizationLiveViewer 전역 인스턴스 및 래퍼 함수
// ==============================================================
const locLiveViewer = new LocalizationLiveViewer();

function resetLocViewer()              { locLiveViewer.resetView(); }
function toggleLocTopView(checked)     { locLiveViewer.toggleTopView(checked); }
function toggleLocLayer(key)           { locLiveViewer.toggleLayer(key); }
function toggleLocViewerFullscreen()   { locLiveViewer.toggleFullscreen(); }
function onLocFixedFrameFocus()        { locLiveViewer.onFixedFrameFocus(); }
function onLocFixedFrameInput(value)   { locLiveViewer.onFixedFrameInput(value); }
function onLocFixedFrameBlur(event)    { locLiveViewer.onFixedFrameBlur(event); }
function toggleLocFixedFrameDropdown() { locLiveViewer.toggleFixedFrameDropdown(); }

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
    const isSlamFullscreen = isFullscreen && document.fullscreenElement?.id === 'slam-result-canvas-container';
    const isViewerFullscreen = isFullscreen && document.fullscreenElement?.id === '3d-viewer-container';
    _updateSlamFullscreenIcon(isSlamFullscreen);
    _updateViewerFullscreenIcon(isViewerFullscreen);
    if (slamResultViewer) slamResultViewer._resizeRenderer();
    if (locLiveViewer) locLiveViewer._resizeRenderer();
    if (typeof onWindowResize === 'function') onWindowResize();
});

document.addEventListener('webkitfullscreenchange', () => {
    const isFullscreen = !!document.webkitFullscreenElement;
    const isSlamFullscreen = isFullscreen && document.webkitFullscreenElement?.id === 'slam-result-canvas-container';
    const isViewerFullscreen = isFullscreen && document.webkitFullscreenElement?.id === '3d-viewer-container';
    _updateSlamFullscreenIcon(isSlamFullscreen);
    _updateViewerFullscreenIcon(isViewerFullscreen);
    if (slamResultViewer) slamResultViewer._resizeRenderer();
    if (locLiveViewer) locLiveViewer._resizeRenderer();
    if (typeof onWindowResize === 'function') onWindowResize();
});

// ==============================================================
// 페이지 로드 시 초기화
// ==============================================================
document.addEventListener('DOMContentLoaded', () => {
    console.log('[DOMContentLoaded] Page loaded');

    // 8081 WebSocket은 Plot 탭 여부와 무관하게 항상 연결 유지
    // (KITTI 변환 진행률 등 전역 백엔드 이벤트 수신에 필요)
    _initBackendWs();

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
