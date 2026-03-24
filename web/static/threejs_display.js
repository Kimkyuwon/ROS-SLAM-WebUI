// Three.js Direct Implementation for PointCloud2 Visualization

// =============================================
// RViz 스타일 색상 팔레트 피커
// =============================================

// 프리셋 색상 팔레트 (9열 × 5행 = 45색)
const SOLID_COLOR_PRESETS = [
    // Row 1 - Light
    '#7FC5F4', '#80E5A0', '#F5F59A', '#F5C580', '#F58080', '#D580F5', '#C5A080', '#F0F0F0', '#A8A8A8',
    // Row 2 - Medium-light
    '#4AA3E8', '#40C060', '#F0E040', '#F0A040', '#E84040', '#B040E8', '#A07040', '#D0D0D0', '#707070',
    // Row 3 - Medium
    '#1E90FF', '#00C000', '#FFD700', '#FF8C00', '#FF2020', '#9400D3', '#8B4513', '#909090', '#404040',
    // Row 4 - Medium-dark
    '#1060D0', '#008000', '#DAA520', '#FF4500', '#CC0000', '#7B00CC', '#6B3410', '#606060', '#202020',
    // Row 5 - Dark
    '#00008B', '#006400', '#B8860B', '#D2691E', '#8B0000', '#4B0082', '#3D1A08', '#303030', '#000000',
];

// 팔레트 피커 전역 상태
const colorPickerPopupState = {
    topicName: null,
    selectedColor: '#ffffff',
    swatchEl: null,
    hexLabel: null,
    onConfirm: null,    // 확정 시 호출할 콜백 (function(color) {}), null이면 PointCloud2 기본 동작
};

/**
 * 팔레트 팝업 DOM 생성 (최초 1회)
 */
function buildColorPickerPopup() {
    if (document.getElementById('solid-color-picker-popup')) return;

    const popup = document.createElement('div');
    popup.id = 'solid-color-picker-popup';
    popup.className = 'solid-color-picker-popup';
    popup.style.display = 'none';

    // 헤더
    const header = document.createElement('div');
    header.className = 'scp-header';

    const cancelBtn = document.createElement('button');
    cancelBtn.className = 'scp-btn scp-cancel';
    cancelBtn.textContent = '취소(C)';
    cancelBtn.addEventListener('click', closeColorPickerPopup);

    const title = document.createElement('span');
    title.className = 'scp-title';
    title.textContent = 'Select Color';

    const selectBtn = document.createElement('button');
    selectBtn.className = 'scp-btn scp-select';
    selectBtn.textContent = '선택(S)';
    selectBtn.addEventListener('click', confirmColorPickerPopup);

    header.appendChild(cancelBtn);
    header.appendChild(title);
    header.appendChild(selectBtn);

    // 색상 그리드
    const grid = document.createElement('div');
    grid.id = 'scp-grid';
    grid.className = 'scp-grid';

    SOLID_COLOR_PRESETS.forEach(color => {
        const cell = document.createElement('button');
        cell.className = 'scp-color-cell';
        cell.style.backgroundColor = color;
        cell.dataset.color = color;
        cell.title = color;
        cell.addEventListener('click', function() {
            selectColorInPicker(color);
        });
        grid.appendChild(cell);
    });

    // 사용자 지정 섹션
    const customSection = document.createElement('div');
    customSection.className = 'scp-custom-section';

    const customLabel = document.createElement('div');
    customLabel.className = 'scp-custom-label';
    customLabel.textContent = '사용자 지정';

    const customRow = document.createElement('div');
    customRow.className = 'scp-custom-row';

    const addBtn = document.createElement('button');
    addBtn.className = 'scp-add-btn';
    addBtn.textContent = '+';
    addBtn.title = '직접 색상 입력';

    const hiddenInput = document.createElement('input');
    hiddenInput.type = 'color';
    hiddenInput.id = 'scp-custom-input';
    hiddenInput.style.cssText = 'position:absolute;opacity:0;pointer-events:none;width:0;height:0;';

    const customSwatch = document.createElement('div');
    customSwatch.id = 'scp-custom-swatch';
    customSwatch.className = 'scp-custom-swatch';

    addBtn.addEventListener('click', function(e) {
        e.stopPropagation();
        hiddenInput.value = colorPickerPopupState.selectedColor;
        hiddenInput.click();
    });

    hiddenInput.addEventListener('input', function() {
        const val = this.value;
        customSwatch.style.backgroundColor = val;
        selectColorInPicker(val, true);  // 커스텀 컬러
    });

    customRow.appendChild(addBtn);
    customRow.appendChild(hiddenInput);
    customRow.appendChild(customSwatch);

    customSection.appendChild(customLabel);
    customSection.appendChild(customRow);

    popup.appendChild(header);
    popup.appendChild(grid);
    popup.appendChild(customSection);

    document.body.appendChild(popup);

    // 외부 클릭 시 닫기
    document.addEventListener('click', function(e) {
        const p = document.getElementById('solid-color-picker-popup');
        if (p && p.style.display !== 'none') {
            if (!p.contains(e.target) && e.target !== colorPickerPopupState.swatchEl) {
                closeColorPickerPopup();
            }
        }
    }, true);
}

/**
 * 팔레트에서 색상 선택 (체크마크 이동)
 * @param {string} color - '#rrggbb' 형태
 * @param {boolean} isCustom - 사용자 지정 색상 여부
 */
function selectColorInPicker(color, isCustom = false) {
    colorPickerPopupState.selectedColor = color;

    // 모든 셀 체크마크 제거
    document.querySelectorAll('#scp-grid .scp-color-cell').forEach(cell => {
        cell.classList.remove('scp-selected');
    });

    if (!isCustom) {
        // 프리셋 셀에 체크마크
        document.querySelectorAll('#scp-grid .scp-color-cell').forEach(cell => {
            if (cell.dataset.color.toLowerCase() === color.toLowerCase()) {
                cell.classList.add('scp-selected');
            }
        });
        // 커스텀 스와치 초기화
        const customSwatch = document.getElementById('scp-custom-swatch');
        if (customSwatch) customSwatch.style.backgroundColor = 'transparent';
    } else {
        // 커스텀 스와치에 반영
        const customSwatch = document.getElementById('scp-custom-swatch');
        if (customSwatch) customSwatch.style.backgroundColor = color;
    }
}

/**
 * 팔레트 팝업 열기
 * @param {string|null} topicName - PointCloud2 토픽명 (Path/Odometry는 null 가능)
 * @param {string} currentColor   - 현재 색상 (#rrggbb)
 * @param {HTMLElement} swatchEl  - 팝업을 열어준 스와치 버튼 요소
 * @param {HTMLElement|null} hexLabel - 색상 hex 텍스트 요소 (없으면 null)
 * @param {Function|null} onConfirm  - 확정 시 콜백 function(color), null이면 PointCloud2 기본 동작
 */
function openColorPickerPopup(topicName, currentColor, swatchEl, hexLabel, onConfirm = null) {
    buildColorPickerPopup();

    colorPickerPopupState.topicName = topicName;
    colorPickerPopupState.selectedColor = currentColor;
    colorPickerPopupState.swatchEl = swatchEl;
    colorPickerPopupState.hexLabel = hexLabel;
    colorPickerPopupState.onConfirm = onConfirm;

    // 커스텀 스와치 초기화
    const customSwatch = document.getElementById('scp-custom-swatch');
    if (customSwatch) customSwatch.style.backgroundColor = 'transparent';

    // 현재 색상 선택 표시
    const isPreset = SOLID_COLOR_PRESETS.some(c => c.toLowerCase() === currentColor.toLowerCase());
    selectColorInPicker(currentColor, !isPreset);
    if (!isPreset && customSwatch) customSwatch.style.backgroundColor = currentColor;

    // 팝업 위치 계산 (스와치 근처)
    const popup = document.getElementById('solid-color-picker-popup');
    popup.style.display = 'block';

    const rect = swatchEl.getBoundingClientRect();
    const pw = popup.offsetWidth || 290;
    const ph = popup.offsetHeight || 260;

    let left = rect.left;
    let top = rect.bottom + 6;

    if (left + pw > window.innerWidth - 8) left = window.innerWidth - pw - 8;
    if (left < 8) left = 8;
    if (top + ph > window.innerHeight - 8) top = rect.top - ph - 6;
    if (top < 8) top = 8;

    popup.style.left = `${left}px`;
    popup.style.top = `${top}px`;
}

/**
 * 팔레트 팝업 닫기 (취소)
 */
function closeColorPickerPopup() {
    const popup = document.getElementById('solid-color-picker-popup');
    if (popup) popup.style.display = 'none';
}

/**
 * 팔레트 팝업 확정 (선택)
 */
function confirmColorPickerPopup() {
    const { topicName, selectedColor, swatchEl, hexLabel, onConfirm } = colorPickerPopupState;

    if (typeof onConfirm === 'function') {
        // Path/Odometry 등 커스텀 콜백
        onConfirm(selectedColor);
        if (swatchEl) swatchEl.style.backgroundColor = selectedColor;
        if (hexLabel) hexLabel.textContent = selectedColor.toUpperCase();
    } else if (topicName) {
        // PointCloud2 기본 동작
        updateTopicSetting(topicName, 'solidColor', selectedColor);
        if (swatchEl) swatchEl.style.backgroundColor = selectedColor;
        if (hexLabel) hexLabel.textContent = selectedColor.toUpperCase();
    }
    closeColorPickerPopup();
}

// 3D Viewer 상태 - 객체로 그룹화
const viewer3DState = {
    scene: null,
    camera: null,
    renderer: null,
    controls: null,
    ros: null,
    rosConnected: false,
    pointCloudMeshes: {},
    displaySelectedTopics: [],
    currentDisplayContainer: null,
    threeJSInitialized: false,
    topicSubscriptions: new Map(),  // 토픽명 → ROSLIB.Topic 구독 객체
    topicSettings: new Map(),       // 토픽명 → 색상/표시 설정 객체
    pathObjects: new Map(),         // topicName → { line, visible }
    odomObjects: new Map(),         // topicName → { group, arrow, sphere, visible }
    trajectoryObjects: new Map(),   // topicName → { parentGroup, axesGroupList[], visible }
    selectedPathTopics: [],         // 선택된 Path 토픽 목록
    selectedOdomTopics: [],         // 선택된 Odometry 토픽 목록
    trajectoryMaxPoints: 1000,      // 궤적 최대 포인트 수
    decayObjects: new Map(),        // topicName → [{ points: THREE.Points, time: DOMHighResTimeStamp }]
    tfFrameTree: new Map(),         // childFrameId → { parentId, translation, quaternion, stamp }
    tfObjects: new Map(),           // topicName → { group: THREE.Group, visible: boolean }
    selectedTFTopics: [],           // 선택된 TF 토픽 목록
    fixedFrame: 'map',              // FrameIDFilter 기준 프레임 (기본값: 'map')
    pcFrameGroups: new Map(),       // topicName → THREE.Group (PC2 frame 래퍼)
    topicFrameIds: new Map(),       // topicName → last received ROS frame_id
    selectedLivoxTopics: [],        // 선택된 Livox 토픽 목록
    livoxTagFilter: new Map(),      // topicName → boolean (true = 노이즈 제외)

    // LaserScan (sensor_msgs/LaserScan) — RViz LaserScanDisplay 참조
    selectedLaserScanTopics: [],    // 선택된 LaserScan 토픽 목록
    laserScanObjects: new Map(),   // topicName → { frameGroup, points, visible }
    laserScanLastMessage: new Map(), // topicName → 마지막 수신 메시지 (색상 모드 변경 시 즉시 재처리용)

    // Phase 2.7: Image 패널 관련 상태
    selectedImageTopics: [],        // 선택된 Image 토픽 목록
    imageSubscriptions: new Map(),  // topicName → true  (Worker 구독 중 표시용)
    imagePanelCollapsed: false,     // 패널 접힘 상태
    _imagePanelHeight: 200,         // 패널 마지막 높이 저장 (접기/펼치기 복원용)

    // Phase 2.4: Views 패널 관련 상태
    orthoCam: null,                    // OrthographicCamera (TopDown View용)
    currentViewType: 'orbit',          // 'orbit' | 'topdown'
    activeCamera: null,                // animate() 루프에서 사용하는 현재 활성 카메라
    cameraTargetFrame: '<Fixed Frame>', // Views 패널 Target Frame
    viewSettings: {
        orbit:   { distance: 10, yaw: 0.785, pitch: 0.785, nearClip: 0.01, invertZ: false },
        topdown: { scale: 1.0, nearClip: 0.01 }
    },

    // Dirty 렌더링 플래그
    // true 일 때만 renderer.render() 호출 → 불필요한 60fps GPU 렌더 제거
    needsRender: true,   // 초기에 한 번은 렌더해서 빈 씬을 표시
};

// Get the active display container based on current subtab
function getActiveDisplayContainer() {
    const activeSubTab = document.querySelector('.subtab-content.active');
    console.log('getActiveDisplayContainer - activeSubTab:', activeSubTab);
    
    if (!activeSubTab) {
        console.warn('No active subtab found');
        // Fallback: try to get 3d-viewer-subtab directly
        const viewerSubtab = document.getElementById('3d-viewer-subtab');
        if (viewerSubtab) {
            console.log('Using 3d-viewer-subtab as fallback');
            return document.getElementById('3d-viewer-container');
        }
        return null;
    }

    if (activeSubTab.id === '3d-viewer-subtab') {
        const container = document.getElementById('3d-viewer-container');
        console.log('Found 3D viewer container:', container);
        return container;
    }
    
    console.warn('Active subtab is not 3d-viewer-subtab:', activeSubTab.id);
    return null;
}

// Move renderer to active container
function moveRendererToActiveContainer() {
    const newContainer = getActiveDisplayContainer();
    if (!newContainer || !viewer3DState.renderer) return;

    // Don't move if already in the correct container
    if (viewer3DState.currentDisplayContainer === newContainer) return;

    console.log('Moving renderer to:', newContainer.id);

    // Append renderer to new container
    newContainer.appendChild(viewer3DState.renderer.domElement);
    viewer3DState.currentDisplayContainer = newContainer;

    // Resize renderer to fit new container
    if (viewer3DState.camera) {
        viewer3DState.camera.aspect = newContainer.clientWidth / newContainer.clientHeight;
        viewer3DState.camera.updateProjectionMatrix();
    }
    viewer3DState.renderer.setSize(newContainer.clientWidth, newContainer.clientHeight);
}

// Initialize Three.js scene
function initThreeJSDisplay() {
    console.log('=== initThreeJSDisplay called ===');
    console.log('THREE available:', typeof THREE !== 'undefined');
    console.log('OrbitControls available:', typeof window.OrbitControls !== 'undefined');

    // Try multiple ways to get the container
    let container = getActiveDisplayContainer();
    
    // Fallback: directly get the container
    if (!container) {
        console.log('Trying direct container access...');
        container = document.getElementById('3d-viewer-container');
    }
    
    if (!container) {
        console.error('3D Viewer container not found. Make sure 3D Viewer tab is active.');
        console.error('Attempted to find: #3d-viewer-container');
        return;
    }
    
    console.log('Container found:', container.id);
    console.log('Container dimensions:', container.clientWidth, 'x', container.clientHeight);

    // Remove loading message if present
    const loadingMsg = document.getElementById('3d-viewer-loading');
    if (loadingMsg) {
        loadingMsg.remove();
    }

    // If already initialized, just move renderer to new container
    if (viewer3DState.threeJSInitialized) {
        console.log('Three.js already initialized, moving renderer...');
        moveRendererToActiveContainer();
        return;
    }

    // Check if THREE is available
    if (typeof THREE === 'undefined') {
        console.error('Three.js library not loaded!');
        container.innerHTML = '<p style="color: #fff; padding: 20px;">Error: Three.js library failed to load. Please check your internet connection.</p>';
        return;
    }

    viewer3DState.currentDisplayContainer = container;
    viewer3DState.threeJSInitialized = true;

    // Create scene
    viewer3DState.scene = new THREE.Scene();
    viewer3DState.scene.background = new THREE.Color(0x2a2a2a);

    // Get container dimensions
    const containerWidth = container.clientWidth || 800;
    const containerHeight = container.clientHeight || 600;
    const aspect = containerWidth / containerHeight || 1.33;
    
    // Create camera — ROS Z-up 좌표계: camera.up = (0,0,1) 이어야 OrbitControls가
    // Z축 기준으로 orbit하여 RViz Orbit 뷰와 동일하게 동작한다.
    viewer3DState.camera = new THREE.PerspectiveCamera(75, aspect, 0.1, 1000);
    viewer3DState.camera.up.set(0, 0, 1);           // ← Z-up (ROS/RViz 좌표계)
    viewer3DState.camera.position.set(0, -10, 7);   // 뒤쪽(-Y) + 위쪽(+Z) 비스듬한 Orbit 초기 뷰
    viewer3DState.camera.lookAt(0, 0, 0);

    // Phase 2.4: OrthographicCamera 초기화 (TopDown View용)
    const frustumSize = 50;  // ±50m 범위
    viewer3DState.orthoCam = new THREE.OrthographicCamera(
        frustumSize * aspect / -2,  frustumSize * aspect / 2,
        frustumSize / 2,            frustumSize / -2,
        0.1, 1000
    );
    viewer3DState.orthoCam.up.set(0, 0, 1);         // ← Z-up
    viewer3DState.orthoCam.position.set(0, 0, 100);
    viewer3DState.orthoCam.lookAt(0, 0, 0);
    viewer3DState.activeCamera = viewer3DState.camera;  // 기본: PerspectiveCamera

    // Create renderer
    viewer3DState.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    
    // Ensure container has valid dimensions
    if (containerWidth === 0 || containerHeight === 0) {
        console.warn('Container has zero dimensions, using default size');
        viewer3DState.renderer.setSize(800, 600);
    } else {
        viewer3DState.renderer.setSize(containerWidth, containerHeight);
    }
    
    container.appendChild(viewer3DState.renderer.domElement);

    // Add OrbitControls (ES6 module import)
    if (typeof window.OrbitControls !== 'undefined') {
        try {
            viewer3DState.controls = new window.OrbitControls(viewer3DState.camera, viewer3DState.renderer.domElement);
            viewer3DState.controls.enableDamping = true;
            viewer3DState.controls.dampingFactor = 0.25;
            // Dirty 렌더링: 카메라 이동(damping 포함)이 있을 때만 렌더
            viewer3DState.controls.addEventListener('change', function () {
                viewer3DState.needsRender = true;
            });
            console.log('OrbitControls initialized successfully');
        } catch (error) {
            console.error('Failed to initialize OrbitControls:', error);
        }
    } else {
        console.warn('OrbitControls not loaded. 3D navigation will be limited.');
        console.warn('Make sure OrbitControls is imported as ES6 module.');
    }

    // Add grid in XY plane (Z-up coordinate system)
    const gridHelper = new THREE.GridHelper(100, 100, 0x444444, 0x444444);
    gridHelper.rotation.x = Math.PI / 2;  // Rotate to XY plane
    viewer3DState.scene.add(gridHelper);

    // Add axes helper (X=red/forward, Y=green/left, Z=blue/up) — 격자 2칸 크기
    const axesHelper = new THREE.AxesHelper(2);
    viewer3DState.scene.add(axesHelper);

    // Add lights
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    viewer3DState.scene.add(ambientLight);
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.4);
    directionalLight.position.set(10, 10, 10);
    viewer3DState.scene.add(directionalLight);

    // Handle window resize
    window.addEventListener('resize', onWindowResize);

    // Start animation loop
    animate();

    // Connect to ROS
    connectToROS();

    // Views 패널 초기 렌더링 (orbit 기본값에 맞는 파라미터 UI 표시)
    renderViewsPanel();

    // Image 패널 리사이즈 핸들 설정
    setupImagePanelResize();

    // Phase 2.7: #3d-viewer-container ResizeObserver
    // 이미지 패널 접기/펼치기·리사이즈로 flex 높이가 바뀔 때마다 렌더러 자동 리사이즈
    const _viewerEl = document.getElementById('3d-viewer-container');
    if (_viewerEl && typeof ResizeObserver !== 'undefined') {
        new ResizeObserver(() => { onWindowResize(); }).observe(_viewerEl);
    }

    console.log('=== Three.js display initialized successfully ===');
    console.log('Scene:', viewer3DState.scene);
    console.log('Camera:', viewer3DState.camera);
    console.log('Renderer:', viewer3DState.renderer);
    console.log('Container:', container);
}

// =============================================
// Phase 2.4: Views 패널 — 뷰 타입 전환 & 카메라 추적
// =============================================

/**
 * switchViewType(type)
 * Orbit / TopDown 카메라 및 OrbitControls 전환
 * @param {string} type - 'orbit' | 'topdown'
 */
function switchViewType(type) {
    if (!viewer3DState.threeJSInitialized) return;
    viewer3DState.currentViewType = type;

    const controls = viewer3DState.controls;

    if (type === 'orbit') {
        // PerspectiveCamera 활성화 (Z-up 보장, invertZ 설정 반영)
        viewer3DState.activeCamera = viewer3DState.camera;
        const orbitZUp = viewer3DState.viewSettings.orbit.invertZ ? -1 : 1;
        viewer3DState.camera.up.set(0, 0, orbitZUp);
        if (controls) {
            // TopDown yaw 핸들러 해제 및 mouse button 복원
            if (viewer3DState.renderer) {
                _detachTopdownYawHandlers(viewer3DState.renderer.domElement);
            }
            controls.mouseButtons = {
                LEFT:   THREE.MOUSE.ROTATE,
                MIDDLE: THREE.MOUSE.DOLLY,
                RIGHT:  THREE.MOUSE.PAN
            };
            const tx = controls.target.x;
            const ty = controls.target.y;
            const tz = controls.target.z;
            viewer3DState.camera.position.set(tx, ty - 10, tz + 7);
            viewer3DState.camera.lookAt(tx, ty, tz);
            controls.object = viewer3DState.camera;
            controls.enableRotate = true;
            controls.enablePan    = true;
            controls.enableZoom   = true;
            controls.enabled      = true;
            controls.update();
        }
    } else if (type === 'topdown') {
        // OrthographicCamera 활성화
        viewer3DState.activeCamera = viewer3DState.orthoCam;
        if (controls) {
            controls.object = viewer3DState.orthoCam;
            controls.enableRotate = false;
            controls.enablePan    = true;
            controls.enableZoom   = true;
            controls.enabled      = true;
            // 좌클릭 = 커스텀 yaw 핸들러, 우클릭 = 팬 (orbit의 좌클릭 회전과 동일 UX)
            controls.mouseButtons = {
                LEFT:   -1,              // OrbitControls 좌클릭 비활성화 → 커스텀 yaw 핸들러가 처리
                MIDDLE: THREE.MOUSE.DOLLY,
                RIGHT:  THREE.MOUSE.PAN  // 우클릭 = 팬
            };
            viewer3DState.orthoCam.position.set(
                controls.target.x, controls.target.y, 100
            );
            // Yaw 상태 반영 (up 벡터를 먼저 설정해야 lookAt이 올바르게 동작)
            viewer3DState.orthoCam.up.set(
                -Math.sin(_topdownYaw), Math.cos(_topdownYaw), 0
            );
            viewer3DState.orthoCam.lookAt(controls.target);
            controls.update();
            // TopDown yaw 핸들러 등록
            if (viewer3DState.renderer) {
                _attachTopdownYawHandlers(viewer3DState.renderer.domElement);
            }
        }
    }

    // UI 동기화: view-type-select 드롭다운 값 업데이트
    const sel = document.getElementById('view-type-select');
    if (sel && sel.value !== type) sel.value = type;

    // 뷰 타입별 파라미터 UI 갱신
    renderViewsPanel();

    console.log('[Views] Switched to view type:', type);
}

/**
 * _getTopdownPixelsPerUnit()
 * TopDown(OrthographicCamera) 뷰에서 1 월드단위 = ? 픽셀 인지 계산
 * OrthographicCamera에서 Three.js는 sizeAttenuation 여부와 관계없이
 * gl_PointSize = size (픽셀 단위)를 그대로 사용하므로,
 * 월드단위 pointSize → 픽셀 변환 스케일이 필요하다.
 * @returns {number} pixels per world unit
 */
function _getTopdownPixelsPerUnit() {
    const cam      = viewer3DState.orthoCam;
    const renderer = viewer3DState.renderer;
    if (!cam || !renderer) return 40; // fallback
    // OrthographicCamera frustum 높이 (월드단위), zoom 적용
    const frustumH = (cam.top - cam.bottom) / (cam.zoom || 1);
    const pixelH   = renderer.domElement.height || 680;
    return pixelH / frustumH;  // pixels per world unit
}

/**
 * updateTopdownPointSizes()
 * TopDown 뷰에서 매 animate() 프레임 호출:
 * OrthographicCamera zoom이 변할 때 포인트 픽셀 크기가 자동 갱신되도록
 * 모든 포인트 mesh의 material.size를 frustum 스케일로 재계산한다.
 * (sizeAttenuation 변경 없이 size 값만 조정 → 셰이더 재컴파일 없음)
 */
function updateTopdownPointSizes() {
    const scale = _getTopdownPixelsPerUnit();

    Object.keys(viewer3DState.pointCloudMeshes).forEach(function(topicName) {
        const mesh = viewer3DState.pointCloudMeshes[topicName];
        if (!mesh || !mesh.material) return;
        if (mesh._isBoxMesh) return;  // Boxes: InstancedMesh → size 적용 불필요
        const settings  = viewer3DState.topicSettings.get(topicName) || {};
        const worldSize = settings.pointSize !== undefined ? settings.pointSize : 0.1;
        mesh.material.size = Math.max(1, worldSize * scale);
    });

    viewer3DState.decayObjects.forEach(function(arr, topicName) {
        const settings  = viewer3DState.topicSettings.get(topicName) || {};
        const worldSize = settings.pointSize !== undefined ? settings.pointSize : 0.1;
        const px = Math.max(1, worldSize * scale);
        arr.forEach(function(item) {
            if (item.points && item.points.material) {
                item.points.material.size = px;
            }
        });
    });
}
window.switchViewType = switchViewType;

// ── Target Frame 추적 내부 상태 ──
// 이전 프레임에서의 Target Frame 월드 좌표 (delta 계산용)
let _tfTrackId  = null;  // 현재 추적 중인 frame ID
let _tfLastPos  = null;  // THREE.Vector3 | null — 이전 프레임 Target Frame 위치

function _resetTfCameraAssistState() {
    _tfTrackId = null;
    _tfLastPos = null;
}

/**
 * updateOrbitTargetFrame()
 * Orbit / TopDown 뷰: Target Frame의 실제 이동량(delta)만큼 camera와 controls.target을
 * 같이 이동시켜 RViz처럼 카메라 오프셋(거리·방향)을 유지하며 추적.
 *
 * [핵심] lerp 방식 대신 delta 방식을 사용:
 *  - 처음 Target Frame 선택 시 → 위치만 기억, 카메라 즉각 이동 없음
 *  - 이후 매 프레임 → frame이 실제로 이동한 delta만큼만 camera·target 이동
 *  - 사용자의 pan/rotate/zoom → OrbitControls가 spherical 상태로 보존하므로 유지됨
 */
function updateOrbitTargetFrame() {
    const targetFrame = viewer3DState.cameraTargetFrame;
    if (targetFrame === '<Fixed Frame>') {
        // <Fixed Frame>으로 돌아오면 추적 상태 리셋
        _tfTrackId = null;
        return;
    }

    const controls = viewer3DState.controls;
    if (!controls) return;

    const transform = computeWorldTransform(targetFrame, viewer3DState.fixedFrame);
    if (!transform) return;

    const currPos = transform.position;

    // 처음 선택하거나 프레임이 바뀐 경우: 위치만 저장하고 이 프레임은 이동 없음
    if (_tfTrackId !== targetFrame || _tfLastPos === null) {
        _tfTrackId = targetFrame;
        _tfLastPos = new THREE.Vector3(currPos.x, currPos.y, currPos.z);
        return;
    }

    // Target Frame의 실제 이동 delta
    const dx = currPos.x - _tfLastPos.x;
    const dy = currPos.y - _tfLastPos.y;
    const dz = currPos.z - _tfLastPos.z;
    _tfLastPos.set(currPos.x, currPos.y, currPos.z);

    if (dx === 0 && dy === 0 && dz === 0) return;

    // controls.target과 camera 모두 delta만큼 이동 → 상대 오프셋 유지
    controls.target.x += dx;
    controls.target.y += dy;
    controls.target.z += dz;

    const cam = viewer3DState.activeCamera || viewer3DState.camera;
    cam.position.x += dx;
    cam.position.y += dy;
    cam.position.z += dz;
}

/**
 * renderViewsPanel()
 * 현재 뷰 타입에 맞는 파라미터 입력 UI를 #views-params-container에 동적 렌더링
 */
function renderViewsPanel() {
    const container = document.getElementById('views-params-container');
    if (!container) return;

    const type     = viewer3DState.currentViewType;
    const settings = viewer3DState.viewSettings[type] || {};
    let html = '';

    if (type === 'orbit') {
        const invertZChecked = settings.invertZ ? ' checked' : '';
        html = `
            <div class="views-row">
                <label>Distance:</label>
                <input type="number" value="${settings.distance || 10}" step="0.5" min="0.1"
                       onchange="onViewParamChange('orbit','distance', parseFloat(this.value))">
            </div>
            <div class="views-row">
                <label>Yaw:</label>
                <input type="number" value="${settings.yaw || 0.785}" step="0.05"
                       onchange="onViewParamChange('orbit','yaw', parseFloat(this.value))">
            </div>
            <div class="views-row">
                <label>Pitch:</label>
                <input type="number" value="${settings.pitch || 0.785}" step="0.05"
                       onchange="onViewParamChange('orbit','pitch', parseFloat(this.value))">
            </div>
            <div class="views-row">
                <label for="invert-z-checkbox">Invert Z Axis:</label>
                <input type="checkbox" id="invert-z-checkbox"
                       class="views-checkbox"${invertZChecked}
                       onchange="onViewInvertZToggle()"
                       title="Z축 반전 (RViz Invert Z Axis와 동일)">
            </div>`;
    } else if (type === 'topdown') {
        html = `
            <div class="views-row">
                <label>Scale:</label>
                <input type="number" value="${settings.scale || 1.0}" step="0.1" min="0.01"
                       onchange="onViewParamChange('topdown','scale', parseFloat(this.value))">
            </div>`;
    }

    container.innerHTML = html;
}
window.renderViewsPanel = renderViewsPanel;

/**
 * updateViewTargetFrameOptions()
 * _allKnownFrames를 사용해 #view-target-frame 드롭다운 갱신
 * updateFixedFrameOptions() 호출 시 함께 호출
 */
function updateViewTargetFrameOptions() {
    const sel = document.getElementById('view-target-frame');
    if (!sel) return;

    const current = viewer3DState.cameraTargetFrame || '<Fixed Frame>';
    const frames  = Array.from(_allKnownFrames).sort();

    // 기본 옵션 + 수집된 프레임 목록
    let html = '<option value="<Fixed Frame>">&lt;Fixed Frame&gt;</option>';
    frames.forEach(function(f) {
        const selected = (f === current) ? ' selected' : '';
        html += `<option value="${f}"${selected}>${f}</option>`;
    });
    sel.innerHTML = html;

    // 현재 선택 값 복원
    sel.value = current;
}
window.updateViewTargetFrameOptions = updateViewTargetFrameOptions;

/**
 * resetViewCamera()
 * Views 패널 "Zero" 버튼: 카메라를 초기 위치로 리셋
 */
function resetViewCamera() {
    const type = viewer3DState.currentViewType;
    if (type === 'orbit') {
        // RViz Orbit 초기화 시점: 뒤쪽(-Y) + 위쪽(+Z)에서 원점을 비스듬히 바라봄
        // camera.up Z 성분은 invertZ 설정을 반영
        const resetZUp = viewer3DState.viewSettings.orbit.invertZ ? -1 : 1;
        viewer3DState.camera.up.set(0, 0, resetZUp);
        viewer3DState.camera.position.set(0, -10, 7);
        viewer3DState.camera.lookAt(0, 0, 0);
        if (viewer3DState.controls) {
            viewer3DState.controls.target.set(0, 0, 0);
            viewer3DState.controls.update();
        }
    } else if (type === 'topdown') {
        // Yaw 초기화: Y축이 화면 위 (북쪽 = 위)
        _topdownYaw = 0;
        viewer3DState.orthoCam.up.set(0, 1, 0);
        viewer3DState.orthoCam.position.set(0, 0, 100);
        viewer3DState.orthoCam.lookAt(0, 0, 0);
        if (viewer3DState.controls) {
            viewer3DState.controls.target.set(0, 0, 0);
            viewer3DState.controls.update();
        }
    }
    console.log('[Views] Camera reset to zero for type:', type);
}
window.resetViewCamera = resetViewCamera;

/**
 * onViewNearClipChange(value)
 * Near Clip 입력 변경 핸들러 — 활성 카메라의 near 값 갱신
 */
function onViewNearClipChange(value) {
    const near = parseFloat(value);
    if (isNaN(near) || near <= 0) return;

    const type = viewer3DState.currentViewType;
    if (viewer3DState.viewSettings[type]) {
        viewer3DState.viewSettings[type].nearClip = near;
    }
    // PerspectiveCamera
    if (viewer3DState.camera) {
        viewer3DState.camera.near = near;
        viewer3DState.camera.updateProjectionMatrix();
    }
    // OrthographicCamera
    if (viewer3DState.orthoCam) {
        viewer3DState.orthoCam.near = near;
        viewer3DState.orthoCam.updateProjectionMatrix();
    }
}
window.onViewNearClipChange = onViewNearClipChange;

/**
 * onViewTargetFrameChange(value)
 * Target Frame 드롭다운 변경 핸들러
 */
function onViewTargetFrameChange(value) {
    viewer3DState.cameraTargetFrame = value;
    // 프레임 변경 시 delta 추적 상태 리셋 → 새 프레임의 첫 위치부터 추적 시작
    _tfTrackId = null;
    console.log('[Views] Target frame changed to:', value);
}
window.onViewTargetFrameChange = onViewTargetFrameChange;

/**
 * onViewParamChange(viewType, param, value)
 * 뷰 파라미터 입력 변경 공통 핸들러
 */
function onViewParamChange(viewType, param, value) {
    if (!viewer3DState.viewSettings[viewType]) return;
    viewer3DState.viewSettings[viewType][param] = value;
    console.log('[Views] Param changed:', viewType, param, value);
}
window.onViewParamChange = onViewParamChange;

/**
 * onViewInvertZToggle()
 * Orbit 뷰 전용: Z축 반전 토글 (RViz "Invert Z Axis"와 동일)
 * camera.up의 Z 성분을 +1 / -1 로 반전하여 씬을 뒤집는다.
 */
function onViewInvertZToggle() {
    const settings = viewer3DState.viewSettings.orbit;
    settings.invertZ = !settings.invertZ;

    // Orbit 뷰가 활성 상태일 때만 즉시 적용
    if (viewer3DState.currentViewType === 'orbit' && viewer3DState.camera) {
        const zUp = settings.invertZ ? -1 : 1;
        viewer3DState.camera.up.set(0, 0, zUp);
        if (viewer3DState.controls) {
            viewer3DState.controls.update();
        }
        viewer3DState.needsRender = true;
    }

    // UI 토글 버튼 갱신
    renderViewsPanel();
    console.log('[Views] Invert Z Axis:', settings.invertZ);
}
window.onViewInvertZToggle = onViewInvertZToggle;

function onWindowResize() {
    const container = getActiveDisplayContainer();
    if (!container || !viewer3DState.camera || !viewer3DState.renderer) return;

    const w = container.clientWidth;
    const h = container.clientHeight;

    viewer3DState.camera.aspect = w / h;
    viewer3DState.camera.updateProjectionMatrix();
    viewer3DState.renderer.setSize(w, h);

    // Phase 2.4: OrthographicCamera resize 처리
    if (viewer3DState.orthoCam) {
        const frustumSize = 50;
        const aspect = w / h;
        viewer3DState.orthoCam.left   = frustumSize * aspect / -2;
        viewer3DState.orthoCam.right  = frustumSize * aspect / 2;
        viewer3DState.orthoCam.top    = frustumSize / 2;
        viewer3DState.orthoCam.bottom = frustumSize / -2;
        viewer3DState.orthoCam.updateProjectionMatrix();
    }
}

/**
 * Decay Time: 만료된 PointCloud2 Points mesh를 scene에서 제거
 * animate() 루프에서 매 프레임 호출
 */
/**
 * Decay 오브젝트 만료 처리
 * @returns {boolean} 아직 살아있는 decay 오브젝트가 있으면 true (→ 다음 프레임도 렌더 필요)
 */
function tickDecayObjects() {
    const now = performance.now();
    let hasLiving = false;
    viewer3DState.decayObjects.forEach((arr, topicName) => {
        if (arr.length === 0) return;
        const settings  = viewer3DState.topicSettings.get(topicName) || {};
        const decayMs   = (settings.decayTime || 0) * 1000;
        if (decayMs <= 0) return;

        const pcFG = viewer3DState.pcFrameGroups.get(topicName);
        let removed = false;
        for (let i = arr.length - 1; i >= 0; i--) {
            if (now - arr[i].time > decayMs) {
                if (pcFG) pcFG.remove(arr[i].points);
                else viewer3DState.scene.remove(arr[i].points);
                arr[i].points.geometry.dispose();
                arr[i].points.material.dispose();
                arr.splice(i, 1);
                removed = true;
            }
        }
        if (removed) viewer3DState.decayObjects.set(topicName, arr);
        if (arr.length > 0) hasLiving = true;
    });
    return hasLiving;
}

function animate() {
    requestAnimationFrame(animate);

    if (viewer3DState.controls && viewer3DState.controls.update) {
        viewer3DState.controls.update();  // enableDamping 감쇠 처리 (내부에서 'change' 이벤트 발생)
    }

    // 카메라 Target Frame 추적 (Orbit / TopDown 공통)
    const vt = viewer3DState.currentViewType;
    // Target Frame이 <Fixed Frame> 이 아니면 카메라가 프레임을 따라 움직이므로 항상 렌더 필요
    if (viewer3DState.cameraTargetFrame && viewer3DState.cameraTargetFrame !== '<Fixed Frame>') {
        viewer3DState.needsRender = true;
    }
    updateOrbitTargetFrame();

    // TopDown 뷰: frustum zoom 변화에 맞춰 포인트 픽셀 크기 매 프레임 갱신
    if (vt === 'topdown') {
        updateTopdownPointSizes();
    }

    // Decay 오브젝트가 살아있으면 매 프레임 렌더 (fade-out 애니메이션)
    const hasDecay = tickDecayObjects();
    if (hasDecay) viewer3DState.needsRender = true;

    // ── Dirty 렌더링: needsRender 플래그가 설정됐을 때만 GPU 호출 ──
    // 변경사항 없으면 렌더 생략 → GPU 부하↓, PC2 프레임 업로드 경합 제거
    const cam = viewer3DState.activeCamera || viewer3DState.camera;
    if (viewer3DState.needsRender && viewer3DState.renderer && viewer3DState.scene && cam) {
        viewer3DState.renderer.render(viewer3DState.scene, cam);
        viewer3DState.needsRender = false;
    }
}

// Connect to ROS
function connectToROS() {
    // Get hostname dynamically for external access
    const hostname = window.location.hostname || 'localhost';
    const rosbridgeUrl = `ws://${hostname}:9090`;

    console.log('=== ROS Connection Debug ===');
    console.log('Attempting to connect to rosbridge...');
    console.log('WebSocket URL:', rosbridgeUrl);
    console.log('window.location.hostname:', window.location.hostname);
    console.log('window.location.href:', window.location.href);

    viewer3DState.ros = new ROSLIB.Ros({
        url: rosbridgeUrl
    });

    viewer3DState.ros.on('connection', function() {
        console.log('✓ Successfully connected to rosbridge:', rosbridgeUrl);
        viewer3DState.rosConnected = true;
        if (typeof updateRosbridgeStatusChip === 'function') {
            updateRosbridgeStatusChip('connected');
        }
        // ROS 연결 후 백그라운드 TF frame 수집 시작
        startBackgroundFrameCollection();
    });

    viewer3DState.ros.on('error', function(error) {
        console.error('✗ Error connecting to rosbridge:', rosbridgeUrl);
        console.error('Error details:', error);
        viewer3DState.rosConnected = false;
        if (typeof updateRosbridgeStatusChip === 'function') {
            updateRosbridgeStatusChip('disconnected');
        }
    });

    viewer3DState.ros.on('close', function() {
        console.log('✗ Connection to rosbridge closed');
        viewer3DState.rosConnected = false;
        console.log('Will retry in 3 seconds...');
        if (typeof updateRosbridgeStatusChip === 'function') {
            updateRosbridgeStatusChip('reconnecting');
        }
        // Try to reconnect after 3 seconds
        setTimeout(connectToROS, 3000);
    });
}

// =============================================
// PointCloud2 Style 텍스처 캐시
// RViz 스타일: Points / Squares / Flat Squares / Spheres / Boxes / Tiles
// =============================================
const _styleTextureCache = {};

/**
 * 스타일에 대응하는 Canvas 스프라이트 텍스처를 반환 (최초 생성 후 캐시)
 * 'squares': 텍스처 없음 (PointsMaterial 기본 정사각형)
 * 'boxes'  : InstancedMesh 사용 → 텍스처 없음
 * @param {string} style
 * @returns {THREE.CanvasTexture|null}
 */
function _getStyleTexture(style) {
    if (_styleTextureCache[style] !== undefined) return _styleTextureCache[style];

    if (style === 'squares' || style === 'boxes') {
        _styleTextureCache[style] = null;
        return null;
    }

    const SIZE = 64;
    const canvas = document.createElement('canvas');
    canvas.width = canvas.height = SIZE;
    const ctx = canvas.getContext('2d');
    const C = SIZE / 2, R = C - 2;

    ctx.clearRect(0, 0, SIZE, SIZE);

    if (style === 'points') {
        // 원형 점
        ctx.fillStyle = '#ffffff';
        ctx.beginPath();
        ctx.arc(C, C, R, 0, Math.PI * 2);
        ctx.fill();

    } else if (style === 'flat_squares') {
        // 평면 정사각형
        ctx.fillStyle = '#ffffff';
        ctx.fillRect(4, 4, SIZE - 8, SIZE - 8);

    } else if (style === 'spheres') {
        // 방사형 그라디언트로 구 표현
        const g = ctx.createRadialGradient(C * 0.7, C * 0.65, R * 0.05, C, C, R);
        g.addColorStop(0,   '#ffffff');
        g.addColorStop(0.4, '#cccccc');
        g.addColorStop(0.8, '#666666');
        g.addColorStop(1,   '#111111');
        ctx.fillStyle = g;
        ctx.beginPath();
        ctx.arc(C, C, R, 0, Math.PI * 2);
        ctx.fill();

    } else if (style === 'tiles') {
        // 테두리 있는 정사각형 타일
        ctx.fillStyle = '#ffffff';
        ctx.fillRect(3, 3, SIZE - 6, SIZE - 6);
        ctx.strokeStyle = 'rgba(0,0,0,0.45)';
        ctx.lineWidth = 3;
        ctx.strokeRect(3, 3, SIZE - 6, SIZE - 6);
    }

    const tex = new THREE.CanvasTexture(canvas);
    tex.needsUpdate = true;
    _styleTextureCache[style] = tex;
    return tex;
}

// Boxes 스타일 최대 인스턴스 수 (성능 보호)
const PC2_BOX_MAX_INSTANCES = 30000;

// =============================================
// PointCloud2 파싱 성능 최적화: 모듈 레벨 재사용 버퍼
// - Array.push 완전 제거 (GC 압력 제거)
// - 단일 패스 처리 (2패스 → 1패스)
// - 인라인 색상 계산 (함수 호출 / 객체 생성 제거)
// =============================================
const PC2_MAX_POINTS = 50000;
const _pc2PosBuffer = new Float32Array(PC2_MAX_POINTS * 3);
const _pc2ColBuffer = new Float32Array(PC2_MAX_POINTS * 3);

// ── PC2 스트리밍 워커 (단일 인스턴스, 모든 PC2 토픽 공유) ──
// rosbridge에 직접 WebSocket 연결 → JSON.parse 포함 전처리를 Worker 스레드에서 수행
// 메인 스레드는 GPU 업로드(~3ms)만 담당하여 render loop 60fps 유지
let _pc2StreamWorker = null;
const _pc2FrameCounts = new Map(); // topicName → frameCount (BoundingSphere 갱신 주기용)

// ── Image 스트리밍 워커 (단일 인스턴스, 모든 Image 토픽 공유) ──
// Python Backend(8081)에서 JPEG 바이너리를 수신 → createImageBitmap()으로 GPU 가속 디코딩
// 메인 스레드는 canvas.drawImage(bitmap) 만 수행 → 픽셀 루프 / atob 완전 제거
let _imgStreamWorker = null;

/**
 * PointCloud2 메시지 고성능 파싱 (단일 패스, 재사용 버퍼)
 * @param {Object} message       - rosbridge PointCloud2 메시지
 * @param {Object} [settings]    - 색상 설정 { colorMode, colorField, solidColor, pointSize }
 * @param {number} [prevMin]     - 이전 프레임 rainbow 최솟값 (적응형 범위)
 * @param {number} [prevMax]     - 이전 프레임 rainbow 최댓값
 * @returns {{ count: number, newMin: number, newMax: number }}
 *   결과는 _pc2PosBuffer / _pc2ColBuffer 에 0..count*3-1 범위로 기록됨
 */
function parsePointCloud2(message, settings, prevMin, prevMax) {
    const colorMode = (settings && settings.colorMode) || 'rainbow';
    const colorField = (settings && settings.colorField) || 'intensity';
    const solidColor = (settings && settings.solidColor) || '#ffffff';

    // Field 오프셋 탐색
    let xOffset = -1, yOffset = -1, zOffset = -1, rgbOffset = -1, colorFieldOffset = -1;
    for (const field of message.fields) {
        if (field.name === 'x')                              xOffset = field.offset;
        if (field.name === 'y')                              yOffset = field.offset;
        if (field.name === 'z')                              zOffset = field.offset;
        if (field.name === 'rgb' || field.name === 'rgba')  rgbOffset = field.offset;
        if (field.name === colorField)                       colorFieldOffset = field.offset;
    }

    if (xOffset === -1 || yOffset === -1 || zOffset === -1) {
        console.error('[PC2] Missing x/y/z fields in PointCloud2');
        return { count: 0, newMin: 0, newMax: 1 };
    }

    const pointStep = message.point_step;
    const numPoints = Math.min(message.width * message.height, PC2_MAX_POINTS);

    // ── 최적화 base64 디코딩: 필요한 바이트만 디코딩 ──
    // PC2_MAX_POINTS 캡으로 실제 사용하는 바이트는 numPoints × point_step 만큼이다.
    // base64는 3바이트 → 4문자 단위이므로, ceil(needed/3)*4 문자만 잘라서 atob().
    // 예) Ouster 131K pt: 7.6MB 전체 → 50K pt: 2.9MB = 디코딩 71% 절감.
    const bytesNeeded  = numPoints * pointStep;
    const charsNeeded  = Math.ceil(bytesNeeded / 3) * 4;  // 항상 4의 배수 → 유효한 base64
    const b64Slice     = (charsNeeded < message.data.length)
        ? message.data.substring(0, charsNeeded)
        : message.data;
    const binaryString = atob(b64Slice);
    const len          = binaryString.length;
    const data         = new Uint8Array(len);

    // 4× 루프 언롤링: charCodeAt 배치 처리 (JIT 최적화 유도)
    let _j = 0;
    for (; _j + 3 < len; _j += 4) {
        data[_j]     = binaryString.charCodeAt(_j);
        data[_j + 1] = binaryString.charCodeAt(_j + 1);
        data[_j + 2] = binaryString.charCodeAt(_j + 2);
        data[_j + 3] = binaryString.charCodeAt(_j + 3);
    }
    for (; _j < len; _j++) data[_j] = binaryString.charCodeAt(_j);

    const view = new DataView(data.buffer);

    // ── Solid 색상 사전 파싱 ──
    let solidR = 1, solidG = 1, solidB = 1;
    if (colorMode === 'solid') {
        const hex = solidColor.replace('#', '');
        solidR = parseInt(hex.substring(0, 2), 16) / 255;
        solidG = parseInt(hex.substring(2, 4), 16) / 255;
        solidB = parseInt(hex.substring(4, 6), 16) / 255;
    }

    // ── Rainbow 범위: 이전 프레임 값으로 단일 패스 실현 (적응형) ──
    const rangeMin  = (isFinite(prevMin) ? prevMin : 0);
    const rangeMax  = (isFinite(prevMax) && prevMax > rangeMin) ? prevMax : (rangeMin + 1);
    const rangeSpan = rangeMax - rangeMin;

    let newMin = Infinity, newMax = -Infinity;
    let count = 0;

    // ── 단일 패스: XYZ + 색상 동시 처리 ──
    for (let i = 0; i < numPoints; i++) {
        const base = i * pointStep;

        const x = view.getFloat32(base + xOffset, true);
        const y = view.getFloat32(base + yOffset, true);
        const z = view.getFloat32(base + zOffset, true);
        if (isNaN(x) || isNaN(y) || isNaN(z)) continue;

        const ptr3 = count * 3;
        _pc2PosBuffer[ptr3]     = x;
        _pc2PosBuffer[ptr3 + 1] = y;
        _pc2PosBuffer[ptr3 + 2] = z;

        if (colorMode === 'rainbow') {
            // 색상 필드 값 읽기
            let fv = (colorFieldOffset !== -1)
                ? view.getFloat32(base + colorFieldOffset, true)
                : z;
            if (isNaN(fv)) fv = rangeMin;

            // min/max 갱신 (다음 프레임 적응형 범위)
            if (fv < newMin) newMin = fv;
            if (fv > newMax) newMax = fv;

            // ── HSV Rainbow 인라인 (s=1,v=1 특수화 → 객체/함수 생성 없음) ──
            // t: 0.0(빨강) → 1.0(보라)
            const t   = rangeSpan > 0 ? Math.min(1.0, Math.max(0.0, (fv - rangeMin) / rangeSpan)) : 0.5;
            const hue = t * 0.75;                    // 빨강(0.0) → 보라(0.75)
            const h6  = hue * 6;
            const hi  = h6 | 0;                     // Math.floor 대체
            const f   = h6 - hi;
            let cr, cg, cb;
            switch (hi % 6) {
                case 0: cr = 1;     cg = f;     cb = 0;     break;
                case 1: cr = 1 - f; cg = 1;     cb = 0;     break;
                case 2: cr = 0;     cg = 1;     cb = f;     break;
                case 3: cr = 0;     cg = 1 - f; cb = 1;     break;
                case 4: cr = f;     cg = 0;     cb = 1;     break;
                default: cr = 1;   cg = 0;     cb = 1 - f; break;
            }
            _pc2ColBuffer[ptr3]     = cr;
            _pc2ColBuffer[ptr3 + 1] = cg;
            _pc2ColBuffer[ptr3 + 2] = cb;

        } else if (colorMode === 'rgb' && rgbOffset !== -1) {
            const rgb = view.getUint32(base + rgbOffset, true);
            _pc2ColBuffer[ptr3]     = ((rgb >> 16) & 0xFF) / 255;
            _pc2ColBuffer[ptr3 + 1] = ((rgb >>  8) & 0xFF) / 255;
            _pc2ColBuffer[ptr3 + 2] = ( rgb        & 0xFF) / 255;

        } else {
            // solid 또는 fallback
            _pc2ColBuffer[ptr3]     = solidR;
            _pc2ColBuffer[ptr3 + 1] = solidG;
            _pc2ColBuffer[ptr3 + 2] = solidB;
        }

        count++;
    }

    return {
        count,
        newMin: isFinite(newMin) ? newMin : 0,
        newMax: isFinite(newMax) ? newMax : 1,
    };
}

// =============================================
// Livox LiDAR CustomMsg 고성능 파싱
// =============================================

/**
 * By Line 색상 배열: 최대 16채널 Livox 라이다에 대해 균등 분산색 16색
 */
const LIVOX_LINE_COLORS = [
    [1.00, 0.20, 0.20],  // 0  빨강
    [1.00, 0.60, 0.10],  // 1  주황
    [1.00, 1.00, 0.10],  // 2  노랑
    [0.40, 1.00, 0.20],  // 3  연두
    [0.10, 0.90, 0.10],  // 4  초록
    [0.10, 0.90, 0.60],  // 5  청록
    [0.10, 0.80, 1.00],  // 6  하늘
    [0.10, 0.40, 1.00],  // 7  파랑
    [0.40, 0.10, 1.00],  // 8  남색
    [0.70, 0.10, 1.00],  // 9  보라
    [1.00, 0.10, 0.80],  // 10 분홍
    [1.00, 0.10, 0.40],  // 11 자홍
    [0.80, 0.80, 0.80],  // 12 밝은 회색
    [0.50, 0.50, 0.50],  // 13 중간 회색
    [1.00, 0.80, 0.60],  // 14 살구
    [0.60, 1.00, 1.00],  // 15 밝은 청록
];

/**
 * Livox CustomMsg 파싱 (단일 패스, 재사용 버퍼)
 * rosbridge가 message.points 배열을 JSON으로 자동 역직렬화하여 전달하므로 base64 디코딩 불필요
 * @param {Object} message    - rosbridge livox_ros_driver2/msg/CustomMsg 메시지
 * @param {Object} [settings] - 색상 설정 { colorMode, colorField, solidColor, tagFilter }
 *   - colorMode:  'rainbow' | 'byline' | 'solid'
 *   - colorField: 'reflectivity' | 'x' | 'y' | 'z'  (rainbow 모드에서 사용)
 *   - solidColor: '#rrggbb'
 *   - tagFilter:  boolean (true = tag bit[3:2] 노이즈 포인트 제외)
 * @param {number} [prevMin] - 이전 프레임 rainbow 최솟값 (적응형 범위, x/y/z 필드용)
 * @param {number} [prevMax] - 이전 프레임 rainbow 최댓값
 * @returns {{ count: number, newMin: number, newMax: number }}
 *   결과는 _pc2PosBuffer / _pc2ColBuffer 에 0..count*3-1 범위로 기록됨
 */
function parseLivoxCustomMsg(message, settings, prevMin, prevMax) {
    const colorMode  = (settings && settings.colorMode)  || 'rainbow';
    const colorField = (settings && settings.colorField) || 'reflectivity';
    const solidColor = (settings && settings.solidColor) || '#ffffff';
    const tagFilter  = (settings && settings.tagFilter)  || false;

    // ── Solid 색상 사전 파싱 ──
    let solidR = 1, solidG = 1, solidB = 1;
    if (colorMode === 'solid') {
        const hex = solidColor.replace('#', '');
        solidR = parseInt(hex.substring(0, 2), 16) / 255;
        solidG = parseInt(hex.substring(2, 4), 16) / 255;
        solidB = parseInt(hex.substring(4, 6), 16) / 255;
    }

    const points = message.points || [];
    const total  = Math.min(points.length, PC2_MAX_POINTS);

    // ── Rainbow 범위 설정 ──
    // reflectivity: 고정 0-255 (단일 패스 실현)
    // x/y/z: 적응형 (이전 프레임 min/max 사용)
    const useFixedRange = (colorField === 'reflectivity');
    const rangeMin  = useFixedRange ? 0 : (isFinite(prevMin) ? prevMin : 0);
    const rangeMax  = useFixedRange ? 255 : (isFinite(prevMax) && prevMax > rangeMin ? prevMax : rangeMin + 1);
    const rangeSpan = rangeMax - rangeMin;
    let newMin = Infinity, newMax = -Infinity;

    // ── 진단 로그: 최초 수신 메시지에서 고유 tag 값 출력 (디버깅용) ──
    if (parseLivoxCustomMsg._debugTagLogged !== message.header?.seq) {
        parseLivoxCustomMsg._debugTagLogged = message.header?.seq;
        const uniqueTags = new Set();
        for (let di = 0; di < Math.min(total, 200); di++) {
            if (points[di]) uniqueTags.add(points[di].tag);
        }
        console.log(`[Livox][Tag Debug] 고유 tag 값 (첫 200pt): [${[...uniqueTags].join(', ')}]  → hex: [${[...uniqueTags].map(t => '0x' + (t >>> 0).toString(16).padStart(2,'0')).join(', ')}]`);
    }

    let count = 0;

    for (let i = 0; i < total; i++) {
        const pt = points[i];
        if (!pt) continue;

        // Tag Filter: bit[3:2] (마스크 0x0C) = 신뢰도
        //   0x00=고신뢰, 0x04=고신뢰 노이즈, 0x08=중간 노이즈, 0x0C=저신뢰
        if (tagFilter && (pt.tag & 0x0C) !== 0) continue;

        const x = pt.x, y = pt.y, z = pt.z;
        if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;

        const ptr3 = count * 3;
        _pc2PosBuffer[ptr3]     = x;
        _pc2PosBuffer[ptr3 + 1] = y;
        _pc2PosBuffer[ptr3 + 2] = z;

        if (colorMode === 'rainbow') {
            // 선택된 필드 값 읽기
            let fv;
            switch (colorField) {
                case 'x':            fv = x; break;
                case 'y':            fv = y; break;
                case 'z':            fv = z; break;
                case 'reflectivity':
                default:             fv = (pt.reflectivity !== undefined) ? pt.reflectivity : 0; break;
            }
            // 적응형 min/max 갱신 (x/y/z 필드용, 다음 프레임에 사용)
            if (!useFixedRange) {
                if (fv < newMin) newMin = fv;
                if (fv > newMax) newMax = fv;
            }
            // HSV Rainbow 인라인 (빨강→보라)
            const t   = rangeSpan > 0 ? Math.min(1.0, Math.max(0.0, (fv - rangeMin) / rangeSpan)) : 0.5;
            const hue = t * 0.75;
            const h6  = hue * 6;
            const hi  = h6 | 0;
            const f   = h6 - hi;
            let cr, cg, cb;
            switch (hi % 6) {
                case 0: cr = 1;     cg = f;     cb = 0;     break;
                case 1: cr = 1 - f; cg = 1;     cb = 0;     break;
                case 2: cr = 0;     cg = 1;     cb = f;     break;
                case 3: cr = 0;     cg = 1 - f; cb = 1;     break;
                case 4: cr = f;     cg = 0;     cb = 1;     break;
                default: cr = 1;   cg = 0;     cb = 1 - f; break;
            }
            _pc2ColBuffer[ptr3]     = cr;
            _pc2ColBuffer[ptr3 + 1] = cg;
            _pc2ColBuffer[ptr3 + 2] = cb;

        } else if (colorMode === 'byline') {
            // line 값 기준 색상 배열 인덱싱 (최대 16채널)
            const lineIdx = (pt.line !== undefined) ? (pt.line % LIVOX_LINE_COLORS.length) : 0;
            const col = LIVOX_LINE_COLORS[lineIdx];
            _pc2ColBuffer[ptr3]     = col[0];
            _pc2ColBuffer[ptr3 + 1] = col[1];
            _pc2ColBuffer[ptr3 + 2] = col[2];

        } else {
            // solid 또는 fallback
            _pc2ColBuffer[ptr3]     = solidR;
            _pc2ColBuffer[ptr3 + 1] = solidG;
            _pc2ColBuffer[ptr3 + 2] = solidB;
        }

        count++;
    }

    return { count, newMin, newMax };
}

/** PC2/Image 스트림 워커에 남은 구독 해제 후 워커 종료 → 다음 구독 시 새 연결(전환 누적·상태 꼬임 방지) */
function _detachAllStreamWorkers() {
    if (_pc2StreamWorker) {
        try {
            _pc2StreamWorker.postMessage({ cmd: 'unsubscribeAll' });
        } catch (e) {
            console.warn('[Viewer] PC2 worker unsubscribeAll:', e);
        }
        try {
            _pc2StreamWorker.terminate();
        } catch (e) {
            console.warn('[Viewer] PC2 worker terminate:', e);
        }
        _pc2StreamWorker = null;
    }
    if (_imgStreamWorker) {
        try {
            _imgStreamWorker.postMessage({ cmd: 'unsubscribeAll' });
        } catch (e) {
            console.warn('[Viewer] Image worker unsubscribeAll:', e);
        }
        try {
            _imgStreamWorker.terminate();
        } catch (e) {
            console.warn('[Viewer] Image worker terminate:', e);
        }
        _imgStreamWorker = null;
    }
    _pc2FrameCounts.clear();
}

/**
 * 3D Viewer 토픽 구독 전체 리셋 (백·데이터셋 전환 시)
 * 구독이 없어도 Display/설정/워커/백그라운드 TF 보조 구독을 비운다.
 */
function resetViewerTopicSubscriptions() {
    const topicNames = Array.from(viewer3DState.topicSubscriptions.keys());
    topicNames.forEach(function(topicName) {
        try {
            unsubscribeFromTopic(topicName);
        } catch (e) {
            console.warn('[Viewer] Reset unsubscribe:', topicName, e);
        }
    });
    // 백그라운드 /tf 구독을 먼저 끊어 큐에 남은 이전 백 메시지가 tfFrameTree 를 다시 채우지 않게 함
    _unsubscribeAllBackgroundTf();
    _detachAllStreamWorkers();
    _resetTfCameraAssistState();
    _bumpTfSessionId();

    viewer3DState.displaySelectedTopics = [];
    viewer3DState.selectedLivoxTopics = [];
    viewer3DState.selectedPathTopics = [];
    viewer3DState.selectedOdomTopics = [];
    viewer3DState.selectedTFTopics = [];
    viewer3DState.selectedLaserScanTopics = [];
    viewer3DState.selectedImageTopics = [];
    viewer3DState.topicSettings.clear();
    viewer3DState.livoxTagFilter.clear();
    _lastPlayerFilePc2Topics = [];

    viewer3DState.tfFrameTree.clear();
    _allKnownFrames.clear();
    viewer3DState.topicFrameIds.clear();
    viewer3DState.laserScanLastMessage.clear();

    _tfLatestRosTimeNs = 0;
    _lastKnownFramesSig = '';

    resetFixedFrameAndViewTargetsToDefault();

    if (typeof renderDisplayPanel === 'function') {
        renderDisplayPanel();
    }
    if (typeof updateFixedFrameOptions === 'function') {
        updateFixedFrameOptions();
    }
    if (typeof updateViewTargetFrameOptions === 'function') {
        updateViewTargetFrameOptions();
    }
    viewer3DState.needsRender = true;
    restartBackgroundTfPipeline({ skipUnsubscribe: true });
    console.log('[Viewer] Full topic/display/TF-cache reset (re-select displays as needed)');
}

// 토픽 구독 해제 및 리소스 정리
function unsubscribeFromTopic(topicName) {
    // ROSLIB.Topic 구독 해제
    const topic = viewer3DState.topicSubscriptions.get(topicName);
    if (topic) {
        try {
            topic.unsubscribe();
            console.log('Unsubscribed from topic:', topicName);
        } catch (e) {
            console.warn('Failed to unsubscribe from topic:', topicName, e);
        }
        viewer3DState.topicSubscriptions.delete(topicName);
    }

    // PointCloud2 frameGroup 및 children 정리
    const pcFG = viewer3DState.pcFrameGroups.get(topicName);
    if (pcFG) {
        // 일반 모드 mesh
        if (viewer3DState.pointCloudMeshes[topicName]) {
            pcFG.remove(viewer3DState.pointCloudMeshes[topicName]);
            viewer3DState.pointCloudMeshes[topicName].geometry.dispose();
            viewer3DState.pointCloudMeshes[topicName].material.dispose();
            delete viewer3DState.pointCloudMeshes[topicName];
        }
        // Decay 모드 누적 mesh
        const decayArr = viewer3DState.decayObjects.get(topicName);
        if (decayArr && decayArr.length > 0) {
            decayArr.forEach(item => {
                pcFG.remove(item.points);
                item.points.geometry.dispose();
                item.points.material.dispose();
            });
            viewer3DState.decayObjects.delete(topicName);
        }
        viewer3DState.scene.remove(pcFG);
        viewer3DState.pcFrameGroups.delete(topicName);
        console.log('Removed PC2 frame group for topic:', topicName);
    } else {
        // fallback: frameGroup 없는 경우 (이전 버전 호환)
        if (viewer3DState.pointCloudMeshes[topicName]) {
            viewer3DState.scene.remove(viewer3DState.pointCloudMeshes[topicName]);
            viewer3DState.pointCloudMeshes[topicName].geometry.dispose();
            viewer3DState.pointCloudMeshes[topicName].material.dispose();
            delete viewer3DState.pointCloudMeshes[topicName];
        }
        const decayArr = viewer3DState.decayObjects.get(topicName);
        if (decayArr && decayArr.length > 0) {
            decayArr.forEach(item => {
                viewer3DState.scene.remove(item.points);
                item.points.geometry.dispose();
                item.points.material.dispose();
            });
            viewer3DState.decayObjects.delete(topicName);
        }
    }
    viewer3DState.topicFrameIds.delete(topicName);

    // Path 객체 정리 (frameGroup 래퍼 포함)
    const pathObj = viewer3DState.pathObjects.get(topicName);
    if (pathObj) {
        const rootObj = pathObj.frameGroup || pathObj.line;
        viewer3DState.scene.remove(rootObj);
        pathObj.line.geometry.dispose();
        pathObj.line.material.dispose();
        viewer3DState.pathObjects.delete(topicName);
        console.log('Removed path object for topic:', topicName);
    }

    // Odometry 객체 정리
    const odomObj = viewer3DState.odomObjects.get(topicName);
    if (odomObj) {
        viewer3DState.scene.remove(odomObj.group);
        // axes group 내 모든 Mesh의 geometry/material dispose (GPU 메모리 누수 방지)
        odomObj.group.traverse(function(child) {
            if (child.isMesh) {
                child.geometry.dispose();
                if (Array.isArray(child.material)) {
                    child.material.forEach(m => m.dispose());
                } else if (child.material) {
                    child.material.dispose();
                }
            }
        });
        viewer3DState.odomObjects.delete(topicName);
        console.log('Removed odometry object for topic:', topicName);
    }

    // Trajectory 객체 정리 (parentGroup + 모든 axes group dispose)
    const trajObj = viewer3DState.trajectoryObjects.get(topicName);
    if (trajObj) {
        trajObj.axesGroupList.forEach(ag => {
            trajObj.parentGroup.remove(ag);
            disposeAxesGroup(ag);
        });
        viewer3DState.scene.remove(trajObj.parentGroup);
        viewer3DState.trajectoryObjects.delete(topicName);
        console.log('Removed trajectory object for topic:', topicName);
    }

    // TF 객체 정리 (group 내 모든 Line/Sprite GPU 리소스 해제)
    const tfObj = viewer3DState.tfObjects.get(topicName);
    if (tfObj) {
        viewer3DState.scene.remove(tfObj.group);
        tfObj.group.traverse(function(child) {
            if (child.isLine || child.isSprite) {
                if (child.geometry) child.geometry.dispose();
                if (child.material) {
                    if (child.material.map) child.material.map.dispose();
                    child.material.dispose();
                }
            }
        });
        viewer3DState.tfObjects.delete(topicName);
        console.log('Removed TF object for topic:', topicName);
    }

    // Livox 선택 목록에서 제거
    const livoxIdx = viewer3DState.selectedLivoxTopics.indexOf(topicName);
    if (livoxIdx !== -1) {
        viewer3DState.selectedLivoxTopics.splice(livoxIdx, 1);
        viewer3DState.livoxTagFilter.delete(topicName);
    }

    // LaserScan 객체 정리 (RViz LaserScanDisplay 스타일)
    const laserScanObj = viewer3DState.laserScanObjects.get(topicName);
    if (laserScanObj) {
        const rootObj = laserScanObj.frameGroup || laserScanObj.points;
        viewer3DState.scene.remove(rootObj);
        laserScanObj.points.geometry.dispose();
        laserScanObj.points.material.dispose();
        viewer3DState.laserScanObjects.delete(topicName);
        console.log('Removed LaserScan object for topic:', topicName);
    }
    const lsIdx = viewer3DState.selectedLaserScanTopics.indexOf(topicName);
    if (lsIdx !== -1) viewer3DState.selectedLaserScanTopics.splice(lsIdx, 1);
    viewer3DState.laserScanLastMessage.delete(topicName);

    // Image 구독 정리 (Worker 기반)
    if (viewer3DState.imageSubscriptions.has(topicName)) {
        if (_imgStreamWorker) {
            _imgStreamWorker.postMessage({ cmd: 'unsubscribe', topicName });
        }
        viewer3DState.imageSubscriptions.delete(topicName);
        removeImageCell(topicName);
        const imgIdx = viewer3DState.selectedImageTopics.indexOf(topicName);
        if (imgIdx !== -1) viewer3DState.selectedImageTopics.splice(imgIdx, 1);
    }
}

// 단일 토픽의 시각화 오브젝트만 씬에서 제거 (구독/선택 상태는 유지)
function clearTopicVisualization(topicName) {
    // ── PointCloud2: pcFrameGroup은 클로저 참조 유지 → 내부 mesh/decay만 제거 ──
    const pcFG = viewer3DState.pcFrameGroups.get(topicName);
    if (pcFG) {
        if (viewer3DState.pointCloudMeshes[topicName]) {
            pcFG.remove(viewer3DState.pointCloudMeshes[topicName]);
            viewer3DState.pointCloudMeshes[topicName].geometry.dispose();
            viewer3DState.pointCloudMeshes[topicName].material.dispose();
            delete viewer3DState.pointCloudMeshes[topicName];
        }
        const decayArr = viewer3DState.decayObjects.get(topicName);
        if (decayArr && decayArr.length > 0) {
            decayArr.forEach(item => {
                pcFG.remove(item.points);
                item.points.geometry.dispose();
                item.points.material.dispose();
            });
            viewer3DState.decayObjects.set(topicName, []);
        }
    }

    // ── Path: scene에서 제거 + Map 삭제 → 다음 메시지에 콜백이 재생성 ──
    const pathObj = viewer3DState.pathObjects.get(topicName);
    if (pathObj) {
        const rootObj = pathObj.frameGroup || pathObj.line;
        viewer3DState.scene.remove(rootObj);
        pathObj.line.geometry.dispose();
        pathObj.line.material.dispose();
        viewer3DState.pathObjects.delete(topicName);
    }

    // ── Odometry: scene에서 제거 + Map 삭제 → 다음 메시지에 콜백이 재생성 ──
    const odomObj = viewer3DState.odomObjects.get(topicName);
    if (odomObj) {
        viewer3DState.scene.remove(odomObj.group);
        odomObj.group.traverse(function(child) {
            if (child.isMesh) {
                child.geometry.dispose();
                if (Array.isArray(child.material)) {
                    child.material.forEach(m => m.dispose());
                } else if (child.material) {
                    child.material.dispose();
                }
            }
        });
        viewer3DState.odomObjects.delete(topicName);
    }

    // ── Trajectory: scene에서 제거 + Map 삭제 → 다음 메시지에 콜백이 재생성 ──
    const trajObj = viewer3DState.trajectoryObjects.get(topicName);
    if (trajObj) {
        trajObj.axesGroupList.forEach(ag => {
            trajObj.parentGroup.remove(ag);
            disposeAxesGroup(ag);
        });
        viewer3DState.scene.remove(trajObj.parentGroup);
        viewer3DState.trajectoryObjects.delete(topicName);
    }

    // ── TF: scene에서 제거 + Map 삭제 → 다음 TF 메시지에 rebuildTFScene이 재생성 ──
    const tfObj = viewer3DState.tfObjects.get(topicName);
    if (tfObj) {
        viewer3DState.scene.remove(tfObj.group);
        tfObj.group.traverse(function(child) {
            if (child.isLine || child.isSprite) {
                if (child.geometry) child.geometry.dispose();
                if (child.material) {
                    if (child.material.map) child.material.map.dispose();
                    child.material.dispose();
                }
            }
        });
        viewer3DState.tfObjects.delete(topicName);
    }

    // ── LaserScan: scene에서 제거 + Map 삭제 → 다음 메시지에 콜백이 재생성 ──
    const laserScanObj = viewer3DState.laserScanObjects.get(topicName);
    if (laserScanObj) {
        const rootObj = laserScanObj.frameGroup || laserScanObj.points;
        viewer3DState.scene.remove(rootObj);
        laserScanObj.points.geometry.dispose();
        laserScanObj.points.material.dispose();
        viewer3DState.laserScanObjects.delete(topicName);
    }
}

// 시각화 데이터 리셋: 구독/선택 상태는 유지하고 씬의 시각화 오브젝트만 초기화
// → 다음 수신 메시지부터 각 콜백이 오브젝트를 자동 재생성하여 새 데이터부터 표시
function resetAll3DViewer() {
    const allTopics = [...new Set([
        ...viewer3DState.displaySelectedTopics,
        ...viewer3DState.selectedLivoxTopics,
        ...viewer3DState.selectedPathTopics,
        ...viewer3DState.selectedOdomTopics,
        ...viewer3DState.selectedTFTopics,
        ...viewer3DState.selectedLaserScanTopics,
    ])];

    // 각 토픽의 시각화 오브젝트만 씬에서 제거 (GPU 메모리 해제)
    allTopics.forEach(topicName => clearTopicVisualization(topicName));

    // tfFrameTree 는 건드리지 않음 — load_data 직후 TF 재구독과 맞물리면 점군 TF 미스로 전부 숨겨질 수 있음

    console.log('[resetAll3DViewer] Visualization cleared. Subscriptions maintained. Awaiting fresh data.');
}

/**
 * 백/데이터 재생 전환 시 이전 TF·프레임 캐시 제거 (KAIST→KITTI 등)
 * - tfFrameTree, _allKnownFrames, topicFrameIds, laserScanLastMessage 초기화
 * - TF 디스플레이는 빈 트리로 재빌드 후 다음 /tf 수신으로 갱신
 */
function resetBagFrameAndTFState() {
    _resetTfCameraAssistState();
    // tfFrameTree / _allKnownFrames 는 resetViewerTopicSubscriptions 에서만 비운다.
    // 여기서 다시 비우면 백그라운드 /tf 가 막 채운 직후 PC2 가 와서 transform 미스 → 점군 visible=false 가 된다.
    viewer3DState.topicFrameIds.clear();
    viewer3DState.laserScanLastMessage.clear();

    if (!viewer3DState.scene) {
        console.log('[Viewer] Bag/data switch: topic frame caches cleared (3D scene not initialized)');
        return;
    }

    viewer3DState.selectedTFTopics.forEach(function(topicName) {
        try {
            rebuildTFScene(topicName);
        } catch (e) {
            console.warn('[Viewer] resetBagFrameAndTFState rebuildTFScene:', topicName, e);
        }
    });

    reapplyAllFrameTransforms();
    if (typeof updateFixedFrameOptions === 'function') {
        updateFixedFrameOptions();
    }
    viewer3DState.needsRender = true;
    console.log('[Viewer] Bag/data switch: topic frame_ids / laserScan cache cleared (TF tree kept in sync with reset pipeline)');
}

/** 직전 load_data 가 알려 준 File Player PointCloud2 토픽 (구독 동기화용) */
let _lastPlayerFilePc2Topics = [];

/**
 * 서버 load_data 응답의 player_pc2_topics 로 PC2 구독·Display 를 맞춘다.
 * - 배열: 웹 노드가 발행하는 PC2 목록(백엔드와 동일 문자열). 이전 load 추적 토픽 중 여기 없으면 해제 후 목록 반영.
 * - null/undefined: ROS bag 등 자동 목록 없음 → 추적 중인 file-player PC2 만 해제.
 */
function syncPlayerFilePointCloudSubscriptions(playerPc2Topics) {
    const prev = _lastPlayerFilePc2Topics.slice();

    function removeTopic(t) {
        const ix = viewer3DState.displaySelectedTopics.indexOf(t);
        if (ix !== -1) {
            viewer3DState.displaySelectedTopics.splice(ix, 1);
        }
        if (viewer3DState.topicSubscriptions.has(t)) {
            try {
                unsubscribeFromTopic(t);
            } catch (e) {
                console.warn('[Viewer] sync PC2 off', t, e);
            }
        }
    }

    if (playerPc2Topics === undefined || playerPc2Topics === null) {
        prev.forEach(removeTopic);
        _lastPlayerFilePc2Topics = [];
        if (typeof renderDisplayPanel === 'function') {
            renderDisplayPanel();
        }
        return;
    }
    if (!Array.isArray(playerPc2Topics)) {
        playerPc2Topics = [];
    }
    const next = playerPc2Topics.slice();
    const nextSet = new Set(next);

    prev.forEach(function (t) {
        if (!nextSet.has(t)) {
            removeTopic(t);
        }
    });

    next.forEach(function (t) {
        if (viewer3DState.displaySelectedTopics.indexOf(t) === -1) {
            viewer3DState.displaySelectedTopics.push(t);
        }
        subscribeToPointCloud(t);
    });

    _lastPlayerFilePc2Topics = next;
    if (typeof renderDisplayPanel === 'function') {
        renderDisplayPanel();
    }
}

/**
 * File Player load_data 직후: 씬 메시 + TF·프레임 캐시만 초기화 (syncPlayerFilePointCloudSubscriptions 이후 호출).
 */
function resetViewerAfterPlayerLoad() {
    if (!viewer3DState.scene) {
        viewer3DState.tfFrameTree.clear();
        _allKnownFrames.clear();
        viewer3DState.topicFrameIds.clear();
        viewer3DState.laserScanLastMessage.clear();
        console.log('[Viewer] Player load: caches cleared (3D scene not initialized)');
        return;
    }
    resetAll3DViewer();
    resetBagFrameAndTFState();
}

/**
 * PC2 스트리밍 워커 초기화 (단일 인스턴스)
 * rosbridge에 Worker 전용 WebSocket 연결 → JSON.parse 포함 전처리를 Worker 스레드로 완전 분리
 */
function _getPC2StreamWorker() {
    if (_pc2StreamWorker) return _pc2StreamWorker;

    _pc2StreamWorker = new Worker('/static/pc2_stream_worker.js?v=' + Date.now());

    // ── 스트림 워커 결과 수신 (메인 스레드) ──
    _pc2StreamWorker.onmessage = function (ev) {
        const msg = ev.data;

        // ── JSON 메타데이터 패킷 (Plot 탭용) ──────────────────────────────────
        // Python Backend가 PointCloud2 수신마다 전송:
        //   { type:'pc2meta', topic, stamp_sec, stamp_nanosec, frame_id, point_count }
        // script.js(Plot 탭)에서 'pc2_topic_meta' CustomEvent를 구독하여
        // rosbridge 없이 PointCloud2 헤더 스탬프를 실시간으로 plot한다.
        if (msg.type === 'pc2meta') {
            window.dispatchEvent(new CustomEvent('pc2_topic_meta', { detail: msg }));
            return;
        }

        if (msg.type !== 'pc2frame') return;

        const { topicName, frameId, pos, col, count } = msg;

        // frame 변환 적용 (메인 스레드에서 수행 — Three.js 접근 필요)
        const pcFrameGroup = viewer3DState.pcFrameGroups.get(topicName);
        if (pcFrameGroup) {
            viewer3DState.topicFrameIds.set(topicName, frameId);
            applyFrameTransformToObject(pcFrameGroup, frameId);
        }

        // pos/col 은 이미 count 크기만큼의 뷰(subarray)로 전달됨
        _uploadPC2ToGPU(topicName, pos, col, count);
    };

    // Python Backend PC2 WebSocket 서버에 연결 (포트 8081)
    // rosbridge(9090) 대신 Python 백엔드가 직접 PointCloud2를 구독하여
    // binary 패킷으로 전달 → JSON/base64 오버헤드 없음
    const hostname = window.location.hostname || 'localhost';
    _pc2StreamWorker.postMessage({ cmd: 'connect', url: `ws://${hostname}:8081` });

    return _pc2StreamWorker;
}

/**
 * Image 스트리밍 Worker (단일 인스턴스, 모든 Image 토픽 공유)
 * Python Backend(8081)에서 JPEG 바이너리를 수신하여
 * createImageBitmap()으로 디코딩 후 해당 canvas에 drawImage() 한다.
 */
function _getImgStreamWorker() {
    if (_imgStreamWorker) return _imgStreamWorker;

    _imgStreamWorker = new Worker('/static/img_stream_worker.js?v=' + Date.now());

    _imgStreamWorker.onmessage = function (ev) {
        const msg = ev.data;
        if (msg.type !== 'imgframe') return;

        const { topicName, bitmap } = msg;
        const canvasId = 'image-canvas-' + topicName.replace(/\//g, '_');
        const canvas   = document.getElementById(canvasId);
        if (!canvas || !bitmap) {
            bitmap && bitmap.close && bitmap.close();
            return;
        }

        // canvas 크기를 이미지에 맞춤 (한 번만)
        if (canvas.width !== bitmap.width || canvas.height !== bitmap.height) {
            canvas.width  = bitmap.width;
            canvas.height = bitmap.height;
        }

        const ctx = canvas.getContext('2d');
        if (ctx) {
            ctx.drawImage(bitmap, 0, 0);
        }
        bitmap.close();   // GPU 메모리 즉시 해제
    };

    const hostname = window.location.hostname || 'localhost';
    _imgStreamWorker.postMessage({ cmd: 'connect', url: `ws://${hostname}:8081` });

    return _imgStreamWorker;
}

/**
 * PC2 GPU 업로드 (메인 스레드 전용)
 * Worker로부터 받은 Float32Array를 Three.js BufferGeometry에 업로드
 */
function _uploadPC2ToGPU(topicName, pos, col, count) {
    if (count === 0) return;

    const pcFrameGroup = viewer3DState.pcFrameGroups.get(topicName);
    if (!pcFrameGroup) return;

    // 새 데이터 도착 → 다음 animate()에서 렌더 트리거
    viewer3DState.needsRender = true;

    const settings  = viewer3DState.topicSettings.get(topicName) || {};
    const pointSize = settings.pointSize !== undefined ? settings.pointSize : 0.1;
    const alpha     = settings.alpha     !== undefined ? settings.alpha     : 1.0;
    const decayTime = settings.decayTime !== undefined ? settings.decayTime : 0;
    const pcStyle   = settings.style     || 'squares';  // RViz 스타일

    // ── 스프라이트(Points) 재질 생성 ──
    // TopDown(OrthographicCamera) 시 frustum 스케일 적용
    // Orbit(PerspectiveCamera) 시 sizeAttenuation:true → 원근감 자동 적용
    function makeMaterial(sz, a) {
        let renderSz = sz;
        if (viewer3DState.currentViewType === 'topdown') {
            renderSz = Math.max(1, sz * _getTopdownPixelsPerUnit());
        }
        const tex = _getStyleTexture(pcStyle);
        // transparent는 opacity 기준으로만 판단 (tex 유무는 transparent에 영향 X)
        // → Alpha=1이면 transparent=false → depth 정렬 변동 없이 완전 불투명으로 표시
        // alphaTest: 0.4 → 0.01 로 낮춤
        // → 텍스처 가장자리(alpha≈0)만 clip하고, opacity가 낮아도 포인트가 사라지지 않음
        // depthWrite: opacity=1이면 true, 반투명이면 false (Three.js 권장)
        return new THREE.PointsMaterial({
            size:            renderSz,
            vertexColors:    true,
            sizeAttenuation: true,
            transparent:     a < 1.0,
            opacity:         a,
            map:             tex,
            alphaTest:       tex ? 0.01 : 0,
            depthWrite:      a >= 1.0,
        });
    }

    // ── Boxes: InstancedMesh 생성 ──
    function makeBoxMesh(posArr, colArr, count, sz, a) {
        const n = Math.min(count, PC2_BOX_MAX_INSTANCES);
        const geo = new THREE.BoxGeometry(sz, sz, sz);
        const mat = new THREE.MeshLambertMaterial({
            transparent: a < 1.0,
            opacity:     a,
        });
        const im = new THREE.InstancedMesh(geo, mat, n);
        im.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
        // instanceColor 설정
        im.instanceColor = new THREE.InstancedBufferAttribute(new Float32Array(n * 3), 3);
        im.instanceColor.setUsage(THREE.DynamicDrawUsage);
        const dummy = new THREE.Object3D();
        const colBuf = im.instanceColor.array;
        for (let i = 0; i < n; i++) {
            const pi = i * 3;
            dummy.position.set(posArr[pi], posArr[pi + 1], posArr[pi + 2]);
            dummy.updateMatrix();
            im.setMatrixAt(i, dummy.matrix);
            colBuf[pi]     = colArr[pi];
            colBuf[pi + 1] = colArr[pi + 1];
            colBuf[pi + 2] = colArr[pi + 2];
        }
        im.count = n;
        im.instanceMatrix.needsUpdate = true;
        im.instanceColor.needsUpdate  = true;
        im._isBoxMesh = true;  // 식별 플래그
        return im;
    }

    const mesh = viewer3DState.pointCloudMeshes[topicName];
    const fc   = (_pc2FrameCounts.get(topicName) || 0) + 1;
    _pc2FrameCounts.set(topicName, fc);

    // ── 스타일 전환 감지: 이전 mesh 타입과 현재 스타일 불일치 시 강제 재생성 ──
    if (mesh) {
        const wasBox    = mesh._isBoxMesh === true;
        const isNowBox  = pcStyle === 'boxes';
        if (wasBox !== isNowBox) {
            pcFrameGroup.remove(mesh);
            mesh.geometry.dispose();
            mesh.material.dispose();
            viewer3DState.pointCloudMeshes[topicName] = null;
        }
    }

    if (decayTime > 0) {
        // ═══ DECAY 모드: 매 프레임 독립 오브젝트 생성·누적 (Boxes 포함) ═══
        const curMesh = viewer3DState.pointCloudMeshes[topicName];
        if (curMesh) {
            pcFrameGroup.remove(curMesh);
            curMesh.geometry.dispose();
            curMesh.material.dispose();
            viewer3DState.pointCloudMeshes[topicName] = null;
        }

        let decayObj;
        if (pcStyle === 'boxes') {
            // Boxes 스타일: InstancedMesh를 프레임마다 생성하여 누적
            decayObj = makeBoxMesh(pos, col, count, pointSize, alpha);
        } else {
            // Points 계열: pos/col을 slice()로 독립 복사하여 누적
            const posArr = pos.slice();
            const colArr = col.slice();
            const geo = new THREE.BufferGeometry();
            geo.setAttribute('position', new THREE.BufferAttribute(posArr, 3));
            geo.setAttribute('color',    new THREE.BufferAttribute(colArr, 3));
            decayObj = new THREE.Points(geo, makeMaterial(pointSize, alpha));
        }

        pcFrameGroup.add(decayObj);
        const arr = viewer3DState.decayObjects.get(topicName) || [];
        arr.push({ points: decayObj, time: performance.now() });
        viewer3DState.decayObjects.set(topicName, arr);

    } else {
        // ═══ 일반 모드: 단일 mesh in-place 업데이트 ═══
        const decayArr = viewer3DState.decayObjects.get(topicName);
        if (decayArr && decayArr.length > 0) {
            decayArr.forEach(item => {
                pcFrameGroup.remove(item.points);
                item.points.geometry.dispose();
                item.points.material.dispose();
            });
            viewer3DState.decayObjects.set(topicName, []);
        }

        const curMesh = viewer3DState.pointCloudMeshes[topicName];

        if (pcStyle === 'boxes') {
            // ── Boxes: 매 프레임 InstancedMesh 재생성 (수 변동 대응) ──
            if (curMesh) {
                pcFrameGroup.remove(curMesh);
                curMesh.geometry.dispose();
                curMesh.material.dispose();
            }
            const newBox = makeBoxMesh(pos, col, count, pointSize, alpha);
            viewer3DState.pointCloudMeshes[topicName] = newBox;
            pcFrameGroup.add(newBox);

        } else if (!curMesh) {
            // ── 첫 메시지(Points 계열): GPU 버퍼를 MAX_POINTS 크기로 미리 할당 ──
            const posArr = new Float32Array(PC2_MAX_POINTS * 3);
            const colArr = new Float32Array(PC2_MAX_POINTS * 3);
            posArr.set(pos);   // pos는 이미 count 크기
            colArr.set(col);
            const posAttr = new THREE.BufferAttribute(posArr, 3);
            const colAttr = new THREE.BufferAttribute(colArr, 3);
            posAttr.setUsage(THREE.DynamicDrawUsage);
            colAttr.setUsage(THREE.DynamicDrawUsage);
            const geometry = new THREE.BufferGeometry();
            geometry.setAttribute('position', posAttr);
            geometry.setAttribute('color',    colAttr);
            geometry.setDrawRange(0, count);
            geometry.computeBoundingSphere();
            const newMesh = new THREE.Points(geometry, makeMaterial(pointSize, alpha));
            viewer3DState.pointCloudMeshes[topicName] = newMesh;
            pcFrameGroup.add(newMesh);
            console.log('[PC2] Mesh created (StreamWorker):', topicName, '|', count, 'pts', '| style:', pcStyle);
        } else {
            // ── 업데이트: 기존 GPU 버퍼를 in-place 갱신 (할당 0회) ──
            const geometry = curMesh.geometry;
            const posAttr  = geometry.attributes.position;
            const colAttr  = geometry.attributes.color;
            posAttr.array.set(pos);   // pos는 이미 count 크기
            colAttr.array.set(col);
            posAttr.needsUpdate = true;
            colAttr.needsUpdate = true;
            geometry.setDrawRange(0, count);
            curMesh.material.size        = pointSize;
            curMesh.material.opacity     = alpha;
            curMesh.material.transparent = alpha < 1.0;
            curMesh.material.depthWrite  = alpha >= 1.0;
            curMesh.material.needsUpdate = true;
            if (fc % 30 === 0) geometry.computeBoundingSphere();
        }
    }
}

// Subscribe to PointCloud2 topic
function subscribeToPointCloud(topicName) {
    if (!viewer3DState.rosConnected) {
        console.warn('[PC2] Not connected to ROS');
        return;
    }

    // 재구독 방지: 이미 구독 중인 토픽이면 건너뜀
    if (viewer3DState.topicSubscriptions.has(topicName)) {
        return viewer3DState.topicSubscriptions.get(topicName);
    }

    console.log('[PC2] Subscribing to topic (StreamWorker):', topicName);

    // PC2 frame 래퍼 그룹 (frame_id → fixedFrame 변환 적용 대상)
    let pcFrameGroup = viewer3DState.pcFrameGroups.get(topicName);
    if (!pcFrameGroup) {
        pcFrameGroup = new THREE.Group();
        viewer3DState.scene.add(pcFrameGroup);
        viewer3DState.pcFrameGroups.set(topicName, pcFrameGroup);
    }

    // 현재 색상 설정을 Worker에 전달
    const settings = viewer3DState.topicSettings.get(topicName) || {};

    // Python Backend PC2 WebSocket Worker에 구독 요청
    // throttle(5Hz)는 Python 백엔드(PC2WebSocketServer)에서 처리하므로
    // Worker 측에는 전달할 필요 없음
    const worker = _getPC2StreamWorker();
    worker.postMessage({
        cmd:        'subscribe',
        topicName,
        colorMode:  settings.colorMode  || 'rainbow',
        colorField: settings.colorField || 'intensity',
        solidColor: settings.solidColor || '#ffffff',
    });

    // topicSubscriptions에 sentinel 저장 (unsubscribeFromTopic() 호환)
    // sentinel.unsubscribe() → 워커에 unsubscribe 명령 전송
    const sentinel = {
        unsubscribe: function () {
            if (_pc2StreamWorker) {
                _pc2StreamWorker.postMessage({ cmd: 'unsubscribe', topicName });
            }
            _pc2FrameCounts.delete(topicName);
        },
    };
    viewer3DState.topicSubscriptions.set(topicName, sentinel);

    return sentinel;
}

/**
 * Livox LiDAR 토픽 구독 (Python 백엔드 binary 스트리밍 — rosbridge 불필요)
 * PC2와 동일한 Worker 경로 사용. 서버가 CustomMsg → PC2 호환 binary로 변환하여 전송.
 */
function subscribeToLivox(topicName) {
    // 재구독 방지: 이미 구독 중인 토픽이면 건너뜀
    if (viewer3DState.topicSubscriptions.has(topicName)) {
        return viewer3DState.topicSubscriptions.get(topicName);
    }

    console.log('[Livox] Subscribing to topic (StreamWorker):', topicName);

    // PC2 frame 래퍼 그룹
    let pcFrameGroup = viewer3DState.pcFrameGroups.get(topicName);
    if (!pcFrameGroup) {
        pcFrameGroup = new THREE.Group();
        viewer3DState.scene.add(pcFrameGroup);
        viewer3DState.pcFrameGroups.set(topicName, pcFrameGroup);
    }

    const settings = viewer3DState.topicSettings.get(topicName) || {};
    const worker = _getPC2StreamWorker();
    worker.postMessage({
        cmd:        'subscribe',
        topicName,
        colorMode:  settings.colorMode  || 'rainbow',
        colorField: settings.colorField || 'intensity',  // binary의 colorField=reflectivity
        solidColor: settings.solidColor || '#ffffff',
    });

    const sentinel = {
        unsubscribe: function () {
            if (_pc2StreamWorker) {
                _pc2StreamWorker.postMessage({ cmd: 'unsubscribe', topicName });
            }
            _pc2FrameCounts.delete(topicName);
        },
    };
    viewer3DState.topicSubscriptions.set(topicName, sentinel);

    if (!viewer3DState.selectedLivoxTopics.includes(topicName)) {
        viewer3DState.selectedLivoxTopics.push(topicName);
    }

    return sentinel;
}

// Wait for ROS connection with timeout
async function waitForROSConnection(timeoutMs = 5000) {
    const startTime = Date.now();

    while (!viewer3DState.rosConnected && (Date.now() - startTime) < timeoutMs) {
        await new Promise(resolve => setTimeout(resolve, 100));
    }

    return viewer3DState.rosConnected;
}

// Get available PointCloud2 topics
// Python Backend HTTP API 사용 — rosbridge getTopics() 대신
// (PC2 데이터는 이제 포트 8081 Binary WS로 받으므로 rosbridge 조회 불필요)
async function getAvailablePointCloudTopics() {
    try {
        const res = await fetch('/api/viewer/pc2_topics');
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        const data = await res.json();
        const topics = data.topics || [];
        console.log('[PC2] Available PointCloud2 topics (via API):', topics);
        return topics;
    } catch (e) {
        console.error('[PC2] Failed to get PC2 topics from API:', e);
        return [];
    }
}

// Get available Livox LiDAR topics (livox_ros_driver2/msg/CustomMsg)
// Python 백엔드 API 사용 — rosbridge 불필요, PC2와 동일한 binary 스트리밍 경로
async function getAvailableLivoxTopics() {
    try {
        const res = await fetch('/api/viewer/livox_topics');
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        const data = await res.json();
        const topics = data.topics || [];
        console.log('[Livox] Available Livox topics (via API):', topics);
        return topics;
    } catch (e) {
        console.error('[Livox] Failed to get topics from API:', e);
        return [];
    }
}

// =============================================
// Phase 2.7: Image 패널 — sensor_msgs/Image 실시간 렌더링
// =============================================

/**
 * 사용 가능한 Image 토픽 목록 반환
 * @returns {Promise<string[]>} sensor_msgs/msg/Image 토픽 목록
 */
async function getAvailableImageTopics() {
    if (!viewer3DState.rosConnected) {
        console.log('[Image] Waiting for ROS connection...');
        const connected = await waitForROSConnection(5000);
        if (!connected) {
            console.warn('[Image] Not connected to ROS after timeout');
            return [];
        }
    }

    return new Promise((resolve) => {
        viewer3DState.ros.getTopics(function(topics) {
            const imageTopics = [];

            if (topics.topics && topics.types) {
                topics.topics.forEach((topic, index) => {
                    const type = topics.types[index];
                    if (type === 'sensor_msgs/msg/Image' || type === 'sensor_msgs/Image') {
                        imageTopics.push(topic);
                    }
                });
            }

            console.log('[Image] Available Image topics:', imageTopics);
            resolve(imageTopics);
        }, function(error) {
            console.error('[Image] Failed to get topics:', error);
            resolve([]);
        });
    });
}

/**
 * sensor_msgs/Image 토픽 구독 및 이미지 패널에 렌더링
 * Python Backend(8081) 바이너리 WebSocket → img_stream_worker.js → drawImage()
 * @param {string} topicName - 구독할 토픽 이름
 */
function subscribeToImage(topicName) {
    // 재구독 방지
    if (viewer3DState.imageSubscriptions.has(topicName)) return;

    console.log('[Image] Subscribing to topic (Worker):', topicName);

    // Worker 구독 등록 (true = 구독 중)
    viewer3DState.imageSubscriptions.set(topicName, true);

    // .image-cell DOM 추가 및 패널 자동 오픈
    addImageCell(topicName);

    // img_stream_worker에 구독 요청 (Python Backend 8081 → JPEG binary)
    const worker = _getImgStreamWorker();
    worker.postMessage({ cmd: 'subscribe', topicName });
}

/**
 * #image-panel-container에 .image-cell DOM 추가 및 패널 자동 오픈
 * @param {string} topicName - 토픽 이름
 */
function addImageCell(topicName) {
    const container = document.getElementById('image-panel-container');
    if (!container) return;

    // 이미 존재하는 경우 스킵
    const existingId = 'image-cell-' + topicName.replace(/\//g, '_');
    if (document.getElementById(existingId)) return;

    // .image-cell 생성
    const cell       = document.createElement('div');
    cell.className   = 'image-cell';
    cell.id          = existingId;

    // canvas
    const canvasId   = 'image-canvas-' + topicName.replace(/\//g, '_');
    const canvas     = document.createElement('canvas');
    canvas.id        = canvasId;

    // 라벨
    const label      = document.createElement('div');
    label.className  = 'image-cell-label';
    label.textContent = topicName;
    label.title      = topicName;

    // X 버튼
    const removeBtn       = document.createElement('button');
    removeBtn.className   = 'image-cell-remove-btn';
    removeBtn.textContent = '✕';
    removeBtn.title       = '토픽 제거';
    removeBtn.addEventListener('click', function() {
        unsubscribeFromImage(topicName);
    });

    cell.appendChild(canvas);
    cell.appendChild(label);
    cell.appendChild(removeBtn);
    container.appendChild(cell);

    // 패널 영역 자동 오픈 (첫 토픽 추가 시 리사이즈 핸들 + 토글 버튼도 표시)
    const area   = document.getElementById('image-panel-area');
    const handle = document.getElementById('image-panel-resize-handle');
    const btn    = document.getElementById('image-panel-toggle-btn');
    if (area) {
        const wasHidden = (area.style.display === 'none');
        area.style.display = '';
        viewer3DState.imagePanelCollapsed = false;
        if (wasHidden) {
            // 패널이 닫혀 있다가 새로 열리는 경우 → 저장된 높이로 복원
            container.style.height = (viewer3DState._imagePanelHeight || 200) + 'px';
        }
    }
    if (handle) handle.style.display = '';
    if (btn) {
        btn.style.display = 'flex';
        btn.textContent   = '▼';  // 버튼이 이미지 패널 위: ▼ = 패널 열림(접을 수 있음)
    }
    // viewer 높이 직접 재계산·적용
    requestAnimationFrame(_applyViewerHeight);
}

/**
 * .image-cell DOM 제거, 0개 시 패널 자동 닫기
 * @param {string} topicName - 토픽 이름
 */
function removeImageCell(topicName) {
    const cellId = 'image-cell-' + topicName.replace(/\//g, '_');
    const cell   = document.getElementById(cellId);
    if (cell) cell.remove();

    // 남은 셀이 없으면 패널 닫기
    const container = document.getElementById('image-panel-container');
    if (container && container.children.length === 0) {
        closeImagePanel();
    }
}

/**
 * Image 구독 해제 (X 버튼 클릭 시)
 * @param {string} topicName - 토픽 이름
 */
function unsubscribeFromImage(topicName) {
    if (viewer3DState.imageSubscriptions.has(topicName)) {
        viewer3DState.imageSubscriptions.delete(topicName);
        // Worker에 구독 해제 요청
        if (_imgStreamWorker) {
            _imgStreamWorker.postMessage({ cmd: 'unsubscribe', topicName });
        }
    }
    removeImageCell(topicName);
    const idx = viewer3DState.selectedImageTopics.indexOf(topicName);
    if (idx !== -1) viewer3DState.selectedImageTopics.splice(idx, 1);
    renderDisplayPanel();
}

/**
 * #3d-viewer-container 높이를 직접 계산해서 설정하고 Three.js 렌더러를 리사이즈한다.
 * flex 레이아웃에 의존하지 않고 명시적으로 계산·적용한다.
 */
function _applyViewerHeight() {
    const viewerPanel     = document.querySelector('.viewer-canvas-panel');
    const viewerContainer = document.getElementById('3d-viewer-container');
    const handle          = document.getElementById('image-panel-resize-handle');
    const btn             = document.getElementById('image-panel-toggle-btn');
    const panelContainer  = document.getElementById('image-panel-container');
    if (!viewerPanel || !viewerContainer) return;

    const totalH  = viewerPanel.clientHeight || 680;
    const btnH    = (btn    && btn.style.display    !== 'none') ? (btn.offsetHeight    || 20) : 0;
    const handleH = (handle && handle.style.display !== 'none') ? (handle.offsetHeight || 8)  : 0;
    const imgH    = panelContainer ? (panelContainer.offsetHeight || 0) : 0;
    const viewerH = Math.max(300, totalH - btnH - handleH - imgH);

    viewerContainer.style.height = viewerH + 'px';

    if (viewer3DState.renderer && viewer3DState.camera) {
        const w = viewerContainer.clientWidth;
        viewer3DState.renderer.setSize(w, viewerH);
        viewer3DState.camera.aspect = w / viewerH;
        viewer3DState.camera.updateProjectionMatrix();
        if (viewer3DState.orthoCam) {
            const aspect = w / viewerH, fs = 50;
            viewer3DState.orthoCam.left   = fs * aspect / -2;
            viewer3DState.orthoCam.right  = fs * aspect /  2;
            viewer3DState.orthoCam.top    = fs / 2;
            viewer3DState.orthoCam.bottom = fs / -2;
            viewer3DState.orthoCam.updateProjectionMatrix();
        }
    }
}

/**
 * Image 패널 접기/펼치기 토글
 * 접힘: 저장된 높이 → 80px (이미지 크기만 줄임)
 * 펼침: 80px → 저장된 높이 복원
 * #3d-viewer-container 높이를 직접 계산해서 설정 → renderer.setSize 호출
 */
function toggleImagePanel() {
    const panelContainer = document.getElementById('image-panel-container');
    const handle = document.getElementById('image-panel-resize-handle');
    const btn    = document.getElementById('image-panel-toggle-btn');
    if (!panelContainer) return;

    viewer3DState.imagePanelCollapsed = !viewer3DState.imagePanelCollapsed;

    const COLLAPSED_H = 80;

    if (viewer3DState.imagePanelCollapsed) {
        viewer3DState._imagePanelHeight = panelContainer.offsetHeight || 200;
        panelContainer.style.height = COLLAPSED_H + 'px';
        if (handle) handle.style.display = 'none';
        if (btn) btn.textContent = '▲';
    } else {
        panelContainer.style.height = (viewer3DState._imagePanelHeight || 200) + 'px';
        if (handle) handle.style.display = '';
        if (btn) btn.textContent = '▼';
    }

    // transition(0.2s) 구간 동안 매 프레임마다 viewer 높이 재계산·적용
    const t0 = performance.now();
    (function rafLoop() {
        _applyViewerHeight();
        if (performance.now() - t0 < 250) requestAnimationFrame(rafLoop);
    })();
}

/**
 * Image 패널 닫기 (토픽 0개 시 자동 호출)
 */
function closeImagePanel() {
    const area   = document.getElementById('image-panel-area');
    const handle = document.getElementById('image-panel-resize-handle');
    const btn    = document.getElementById('image-panel-toggle-btn');
    if (area)   { area.classList.remove('collapsed'); area.style.display = 'none'; }
    if (handle) handle.style.display = 'none';
    if (btn)    btn.style.display    = 'none';
    viewer3DState.imagePanelCollapsed = false;
    requestAnimationFrame(_applyViewerHeight);
}

/**
 * Image 패널 리사이즈 핸들 설정
 * mousedown/mousemove/mouseup으로 #image-panel-container 높이 조절 (80~600px)
 */
function setupImagePanelResize() {
    const handle    = document.getElementById('image-panel-resize-handle');
    const container = document.getElementById('image-panel-container');
    if (!handle || !container) return;

    let dragging    = false;
    let startY      = 0;
    let startHeight = 0;

    handle.addEventListener('mousedown', function(e) {
        dragging    = true;
        startY      = e.clientY;
        startHeight = container.offsetHeight;
        e.preventDefault();
    });

    document.addEventListener('mousemove', function(e) {
        if (!dragging) return;
        const delta = startY - e.clientY;   // 위로 드래그 → 높이 증가
        const newH  = Math.min(600, Math.max(80, startHeight + delta));
        container.style.height = newH + 'px';
        _applyViewerHeight();   // 드래그 중 viewer 높이 실시간 업데이트
    });

    document.addEventListener('mouseup', function() {
        if (dragging) {
            dragging = false;
            _applyViewerHeight();
        }
    });
}

// Handle topic selection
async function selectDisplayTopics() {
    console.log('Opening topic selection dialog...');

    // rosbridge 미연결 시 즉시 알림 (5초 대기 없이)
    if (!viewer3DState.rosConnected) {
        alert('Not connected to ROS. Make sure rosbridge_server is running:\n\nros2 launch rosbridge_server rosbridge_websocket_launch.xml');
        return;
    }

    // 버튼 로딩 표시
    const btn = document.getElementById('viewer-add-topic-btn');
    const originalText = btn ? btn.textContent : '';
    if (btn) {
        btn.textContent = 'Loading...';
        btn.disabled = true;
    }

    let topics = [];
    try {
        topics = await getAvailablePointCloudTopics();
    } finally {
        if (btn) {
            btn.textContent = originalText;
            btn.disabled = false;
        }
    }

    if (topics.length === 0) {
        alert('No PointCloud2 topics available. Make sure topics are being published.');
        return;
    }

    // Show topic selection modal
    const modal = document.getElementById('display-topic-modal');
    const topicList = document.getElementById('display-topic-list');

    if (!modal || !topicList) {
        console.error('Display topic modal not found in HTML');
        return;
    }

    topicList.innerHTML = '';

    topics.forEach(topic => {
        const div = document.createElement('div');
        div.className = 'topic-item';

        const checkbox = document.createElement('input');
        checkbox.type = 'checkbox';
        checkbox.id = `display-topic-${topic}`;
        checkbox.value = topic;
        checkbox.checked = viewer3DState.displaySelectedTopics.includes(topic);

        const label = document.createElement('label');
        label.htmlFor = `display-topic-${topic}`;
        label.textContent = topic;

        div.appendChild(checkbox);
        div.appendChild(label);
        topicList.appendChild(div);
    });

    modal.style.display = 'block';
}

function closeDisplayTopicSelection() {
    const modal = document.getElementById('display-topic-modal');
    if (modal) modal.style.display = 'none';
}

function confirmDisplayTopicSelection() {
    const checkboxes = document.querySelectorAll('#display-topic-list input[type="checkbox"]');
    const newTopics = [];

    checkboxes.forEach(cb => {
        if (cb.checked) {
            newTopics.push(cb.value);
        }
    });

    // 제거된 토픽: 구독 해제 + mesh dispose (unsubscribeFromTopic으로 통합 처리)
    viewer3DState.displaySelectedTopics.forEach(topic => {
        if (!newTopics.includes(topic)) {
            unsubscribeFromTopic(topic);
        }
    });

    // Subscribe to new topics
    newTopics.forEach(topic => {
        if (!viewer3DState.displaySelectedTopics.includes(topic)) {
            subscribeToPointCloud(topic);
        }
    });

    viewer3DState.displaySelectedTopics = newTopics;
    closeDisplayTopicSelection();

    // Display 패널 업데이트
    renderDisplayPanel();

    console.log('Display selected topics:', viewer3DState.displaySelectedTopics);
}

// Initialize when 3D Viewer tab is opened
function initialize3DViewer() {
    console.log('initialize3DViewer called');
    
    // Check if 3D Viewer subtab is active
    const viewerSubtab = document.getElementById('3d-viewer-subtab');
    if (!viewerSubtab) {
        console.warn('3D Viewer subtab not found');
        return;
    }
    
    // Wait for Three.js to be ready (required)
    const checkThree = (attempts = 0) => {
        if (typeof window.THREE === 'undefined' && !window.threeReady) {
            if (attempts < 50) { // Wait up to 5 seconds
                console.log('Waiting for Three.js...', attempts);
                window.waitingForThree = () => checkThree(attempts + 1);
                setTimeout(() => checkThree(attempts + 1), 100);
                return;
            } else {
                console.error('Three.js failed to load after 5 seconds');
                return;
            }
        }
        
        // Three.js is ready, OrbitControls is optional - proceed with initialization
        console.log('Three.js ready, initializing display...');
        
        // Wait a bit for DOM to update, then initialize
        setTimeout(() => {
            // Double check that subtab is still active
            if (viewerSubtab.classList.contains('active')) {
                console.log('Initializing Three.js display...');
                initThreeJSDisplay();
            } else {
                console.log('3D Viewer subtab is not active, skipping initialization');
            }
        }, 200);
    };
    
    checkThree();
}

// Store initialization function for OrbitControls module
window.threejsDisplayReady = function() {
    const viewerSubtab = document.getElementById('3d-viewer-subtab');
    if (viewerSubtab && viewerSubtab.classList.contains('active')) {
        console.log('OrbitControls ready, initializing 3D Viewer...');
        initialize3DViewer();
    }
};

// Initialize on page load if 3D Viewer is active
window.addEventListener('load', () => {
    setTimeout(() => {
        const viewerSubtab = document.getElementById('3d-viewer-subtab');
        if (viewerSubtab && viewerSubtab.classList.contains('active')) {
            initialize3DViewer();
        }
    }, 500);
});

/**
 * 토픽 표시/숨기기 토글
 * @param {string} topicName - 토픽명
 * @param {boolean} visible - 표시 여부
 */
function toggleTopicVisible(topicName, visible) {
    if (viewer3DState.pointCloudMeshes[topicName]) {
        viewer3DState.pointCloudMeshes[topicName].visible = visible;
        console.log(`Topic ${topicName} visibility: ${visible}`);
    }
}

/**
 * 토픽별 색상/크기 설정 업데이트
 * 다음 메시지 수신 시 새 설정이 자동 적용됨
 * @param {string} topicName - 토픽명
 * @param {string} key - 설정 키 (colorMode | colorField | solidColor | pointSize)
 * @param {*} value - 설정 값
 */
function updateTopicSetting(topicName, key, value) {
    const current = viewer3DState.topicSettings.get(topicName) || {};
    current[key] = value;
    viewer3DState.topicSettings.set(topicName, current);
    console.log(`Updated setting for ${topicName}: ${key} = ${value}`);

    // PC2 스트리밍 워커에 색상 관련 설정 동기화
    // (워커가 색상 계산을 수행하므로 메인 스레드의 설정 변경을 워커에도 반영)
    if (_pc2StreamWorker && (key === 'colorMode' || key === 'colorField' || key === 'solidColor')) {
        _pc2StreamWorker.postMessage({
            cmd: 'updateSettings',
            topicName,
            [key]: value,
        });
    }

    // LaserScan: 색상 모드/단색 변경 시 마지막 메시지로 즉시 재처리 (Solid 지정색 즉시 반영)
    if (viewer3DState.selectedLaserScanTopics.includes(topicName) &&
        (key === 'colorMode' || key === 'solidColor' || key === 'colorField')) {
        reprocessLaserScanFromLastMessage(topicName);
    }
}

/**
 * LaserScan 마지막 수신 메시지를 현재 설정으로 재처리 (색상 모드 변경 시 즉시 반영)
 * @param {string} topicName - LaserScan 토픽명
 */
function reprocessLaserScanFromLastMessage(topicName) {
    const lastMsg = viewer3DState.laserScanLastMessage.get(topicName);
    if (!lastMsg || !lastMsg.ranges || lastMsg.ranges.length === 0) return;

    const existingObj = viewer3DState.laserScanObjects.get(topicName);
    if (!existingObj || existingObj.visible === false) return;

    const settings = viewer3DState.topicSettings.get(topicName) || {};
    const frameId = (lastMsg.header && lastMsg.header.frame_id) ? lastMsg.header.frame_id : '';
    const pointSize = settings.pointSize !== undefined ? settings.pointSize : 0.1;
    const alpha = settings.alpha !== undefined ? settings.alpha : 1.0;

    const { positions, colors, count } = laserScanToPointsAndColors(lastMsg, settings);
    if (count === 0) return;

    const colorMode = settings.colorMode || 'rainbow';
    const useVertexColors = colorMode === 'rainbow';

    existingObj.points.geometry.dispose();
    existingObj.points.geometry = new THREE.BufferGeometry();
    existingObj.points.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    if (useVertexColors) {
        existingObj.points.geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    } else {
        existingObj.points.geometry.deleteAttribute('color');
    }
    existingObj.points.geometry.attributes.position.needsUpdate = true;
    existingObj.points.material.size = pointSize;
    existingObj.points.material.opacity = alpha;
    existingObj.points.material.transparent = alpha < 1.0;
    existingObj.points.material.depthWrite = alpha >= 1.0;
    existingObj.points.material.vertexColors = useVertexColors;
    if (useVertexColors) {
        existingObj.points.material.color.setHex(0xffffff);  // vertex color 그대로 표시 (곱셈 방지)
    } else {
        existingObj.points.material.color.setStyle(settings.solidColor || '#ffffff');
    }
    existingObj.points.material.needsUpdate = true;
    applyFrameTransformToObject(existingObj.frameGroup, frameId);
    viewer3DState.needsRender = true;
}

/**
 * Display 패널 동적 렌더링
 * 구독 중인 PointCloud2 / Path / Odometry 토픽 목록을 #viewer-display-panel-content 영역에 렌더링
 */
function renderDisplayPanel() {
    const container = document.getElementById('viewer-display-panel-content');
    if (!container) return;

    const pcTopics       = viewer3DState.displaySelectedTopics;
    const pathTopics     = viewer3DState.selectedPathTopics;
    const odomTopics     = viewer3DState.selectedOdomTopics;
    const tfTopics       = viewer3DState.selectedTFTopics;
    const livoxTopics    = viewer3DState.selectedLivoxTopics;
    const imageTopics    = viewer3DState.selectedImageTopics;
    const laserScanTopics = viewer3DState.selectedLaserScanTopics;

    const hasAny = pcTopics.length > 0 || pathTopics.length > 0 || odomTopics.length > 0 ||
                   tfTopics.length > 0 || livoxTopics.length > 0 || imageTopics.length > 0 ||
                   laserScanTopics.length > 0;

    if (!hasAny) {
        container.innerHTML = '<div style="font-size:12px; color: var(--muted); padding: 4px 2px;">구독 중인 토픽 없음</div>';
        return;
    }

    container.innerHTML = '';

    // ── PointCloud2 섹션 ──
    if (pcTopics.length > 0) {
        const pcLabel = document.createElement('div');
        pcLabel.className = 'display-panel-section-label';
        pcLabel.textContent = 'PointCloud2';
        container.appendChild(pcLabel);

        pcTopics.forEach(topicName => {
            const settings   = viewer3DState.topicSettings.get(topicName) || {};
            const colorMode  = settings.colorMode  || 'rainbow';
            const colorField = settings.colorField || 'intensity';
            const solidColor = settings.solidColor || '#ffffff';
            const pointSize  = settings.pointSize  !== undefined ? settings.pointSize  : 0.1;
            const alpha      = settings.alpha      !== undefined ? settings.alpha      : 1.0;
            const decayTime  = settings.decayTime  !== undefined ? settings.decayTime  : 0;
            const pcStyle    = settings.style      || 'squares';
            const visible = viewer3DState.pointCloudMeshes[topicName]
                ? viewer3DState.pointCloudMeshes[topicName].visible !== false
                : true;

            // 토픽 항목 컨테이너
            const item = document.createElement('div');
            item.className = 'display-topic-item';
            item.dataset.topicName = topicName;

            // 헤더: 체크박스 + 토픽명
            const header = document.createElement('div');
            header.className = 'display-topic-item-header';

            const checkbox = document.createElement('input');
            checkbox.type = 'checkbox';
            checkbox.checked = visible;
            checkbox.title = '표시/숨기기';
            checkbox.addEventListener('change', function() {
                toggleTopicVisible(topicName, this.checked);
            });

            const nameSpan = document.createElement('span');
            nameSpan.className = 'display-topic-item-name';
            nameSpan.textContent = topicName;
            nameSpan.title = topicName;

            header.appendChild(checkbox);
            header.appendChild(nameSpan);

            // 설정 영역: Style + 색상 모드 + 필드 선택
            const settingsDiv = document.createElement('div');
            settingsDiv.className = 'display-topic-item-settings';

            // ── Style 선택 (RViz 스타일) ──
            const styleLabel = document.createElement('label');
            styleLabel.textContent = 'Style';

            const styleSelect = document.createElement('select');
            styleSelect.title = '렌더링 스타일 선택 (RViz 스타일)';
            [
                { value: 'points',       label: 'Points' },
                { value: 'squares',      label: 'Squares' },
                { value: 'flat_squares', label: 'Flat Squares' },
                { value: 'spheres',      label: 'Spheres' },
                { value: 'boxes',        label: 'Boxes' },
                { value: 'tiles',        label: 'Tiles' },
            ].forEach(opt => {
                const option = document.createElement('option');
                option.value = opt.value;
                option.textContent = opt.label;
                if (opt.value === pcStyle) option.selected = true;
                styleSelect.appendChild(option);
            });
            styleSelect.addEventListener('change', function() {
                updatePointStyle(topicName, this.value);
            });

            settingsDiv.appendChild(styleLabel);
            settingsDiv.appendChild(styleSelect);

            // ── 색상 모드 선택 ──
            const colorModeLabel = document.createElement('label');
            colorModeLabel.textContent = '색상 모드';

            const colorModeSelect = document.createElement('select');
            colorModeSelect.title = '색상 모드 선택';
            [
                { value: 'rainbow', label: 'Rainbow' },
                { value: 'rgb',     label: 'RGB' },
                { value: 'solid',   label: 'Solid' }
            ].forEach(opt => {
                const option = document.createElement('option');
                option.value = opt.value;
                option.textContent = opt.label;
                if (opt.value === colorMode) option.selected = true;
                colorModeSelect.appendChild(option);
            });
            colorModeSelect.addEventListener('change', function() {
                updateTopicSetting(topicName, 'colorMode', this.value);
                colorFieldRow.style.display = this.value === 'rainbow' ? 'flex' : 'none';
                solidColorRow.style.display = this.value === 'solid'   ? 'flex' : 'none';
            });

            // 색상 필드 선택 (Rainbow 모드)
            const colorFieldRow = document.createElement('div');
            colorFieldRow.style.display = colorMode === 'rainbow' ? 'flex' : 'none';
            colorFieldRow.style.flexDirection = 'column';
            colorFieldRow.style.gap = '2px';

            const colorFieldLabel = document.createElement('label');
            colorFieldLabel.textContent = '색상 필드';

            const colorFieldSelect = document.createElement('select');
            colorFieldSelect.title = '색상 매핑 필드 선택';
            ['intensity', 'z', 'x', 'y'].forEach(fieldName => {
                const option = document.createElement('option');
                option.value = fieldName;
                option.textContent = fieldName;
                if (fieldName === colorField) option.selected = true;
                colorFieldSelect.appendChild(option);
            });
            colorFieldSelect.addEventListener('change', function() {
                updateTopicSetting(topicName, 'colorField', this.value);
            });

            colorFieldRow.appendChild(colorFieldLabel);
            colorFieldRow.appendChild(colorFieldSelect);

            // 단색 색상 선택 (Solid 모드) - RViz 스타일 팔레트 팝업
            const solidColorRow = document.createElement('div');
            solidColorRow.style.display = colorMode === 'solid' ? 'flex' : 'none';
            solidColorRow.style.flexDirection = 'column';
            solidColorRow.style.gap = '2px';

            const solidColorLabel = document.createElement('label');
            solidColorLabel.textContent = '색상';

            const solidColorPickerRow = document.createElement('div');
            solidColorPickerRow.className = 'solid-color-picker-row';

            const colorSwatch = document.createElement('button');
            colorSwatch.className = 'solid-color-swatch';
            colorSwatch.title = '색상 선택';
            colorSwatch.style.backgroundColor = solidColor;

            const colorHexLabel = document.createElement('span');
            colorHexLabel.className = 'solid-color-hex';
            colorHexLabel.textContent = solidColor.toUpperCase();

            colorSwatch.addEventListener('click', function(e) {
                e.stopPropagation();
                openColorPickerPopup(topicName, solidColor, colorSwatch, colorHexLabel);
            });

            solidColorPickerRow.appendChild(colorSwatch);
            solidColorPickerRow.appendChild(colorHexLabel);

            solidColorRow.appendChild(solidColorLabel);
            solidColorRow.appendChild(solidColorPickerRow);

            settingsDiv.appendChild(colorModeLabel);
            settingsDiv.appendChild(colorModeSelect);
            settingsDiv.appendChild(colorFieldRow);
            settingsDiv.appendChild(solidColorRow);

            // ── Point Size 슬라이더 ──
            const sizeRow = document.createElement('div');
            sizeRow.className = 'pc2-setting-row';

            const sizeLabelEl = document.createElement('label');
            sizeLabelEl.textContent = `Size: ${pointSize.toFixed(2)}`;

            const sizeSlider = document.createElement('input');
            sizeSlider.type  = 'range';
            sizeSlider.min   = '0.01';
            sizeSlider.max   = '1.0';
            sizeSlider.step  = '0.01';
            sizeSlider.value = pointSize;
            sizeSlider.className = 'display-item-slider';
            sizeSlider.title = '포인트 크기 (m)';
            sizeSlider.addEventListener('input', function() {
                const v = parseFloat(this.value);
                sizeLabelEl.textContent = `Size: ${v.toFixed(2)}`;
                updatePointSize(topicName, v);
            });

            sizeRow.appendChild(sizeLabelEl);
            sizeRow.appendChild(sizeSlider);
            settingsDiv.appendChild(sizeRow);

            // ── Alpha (투명도) 슬라이더 ──
            const alphaRow = document.createElement('div');
            alphaRow.className = 'pc2-setting-row';

            const alphaLabelEl = document.createElement('label');
            alphaLabelEl.textContent = `Alpha: ${alpha.toFixed(2)}`;

            const alphaSlider = document.createElement('input');
            alphaSlider.type  = 'range';
            alphaSlider.min   = '0.0';
            alphaSlider.max   = '1.0';
            alphaSlider.step  = '0.05';
            alphaSlider.value = alpha;
            alphaSlider.className = 'display-item-slider';
            alphaSlider.title = '투명도 (0=완전투명, 1=불투명)';
            alphaSlider.addEventListener('input', function() {
                const v = parseFloat(this.value);
                alphaLabelEl.textContent = `Alpha: ${v.toFixed(2)}`;
                updatePointAlpha(topicName, v);
            });

            alphaRow.appendChild(alphaLabelEl);
            alphaRow.appendChild(alphaSlider);
            settingsDiv.appendChild(alphaRow);

            // ── Decay Time 슬라이더 ──
            const decayRow = document.createElement('div');
            decayRow.className = 'pc2-setting-row';

            const decayLabelEl = document.createElement('label');
            decayLabelEl.textContent = `Decay Time: ${decayTime.toFixed(1)}s`;

            const decaySlider = document.createElement('input');
            decaySlider.type  = 'range';
            decaySlider.min   = '0';
            decaySlider.max   = '30';
            decaySlider.step  = '0.5';
            decaySlider.value = decayTime;
            decaySlider.className = 'display-item-slider';
            decaySlider.title = 'Decay Time (0=최신 프레임만, >0=n초간 누적 표시)';
            decaySlider.addEventListener('input', function() {
                const v = parseFloat(this.value);
                decayLabelEl.textContent = `Decay Time: ${v.toFixed(1)}s`;
                updateTopicSetting(topicName, 'decayTime', v);
                // decay time이 0이면 누적 mesh 전부 즉시 제거
                // 값이 줄어든 경우도 만료된 항목을 즉시 제거 (tickDecayObjects 다음 프레임까지 기다리지 않음)
                if (v <= 0) {
                    clearDecayObjects(topicName);
                } else {
                    // 새 decay time보다 오래된 decay 오브젝트 즉시 제거
                    _pruneExpiredDecayObjects(topicName, v);
                }
            });

            decayRow.appendChild(decayLabelEl);
            decayRow.appendChild(decaySlider);
            settingsDiv.appendChild(decayRow);

            item.appendChild(header);
            item.appendChild(settingsDiv);
            container.appendChild(item);
        });
    }

    // ── Path 섹션 ──
    if (pathTopics.length > 0) {
        const pathLabel = document.createElement('div');
        pathLabel.className = 'display-panel-section-label';
        pathLabel.textContent = 'Path';
        container.appendChild(pathLabel);

        pathTopics.forEach(topicName => {
            const settings      = viewer3DState.topicSettings.get(topicName) || {};
            const pathColor     = settings.pathColor     || '#00ff00';
            const pathLineWidth = settings.pathLineWidth || 2;
            const pathAlpha     = settings.pathAlpha     !== undefined ? settings.pathAlpha : 1.0;
            const pathObj       = viewer3DState.pathObjects.get(topicName);
            const _pathRoot     = pathObj ? (pathObj.frameGroup || pathObj.line) : null;
            const visible       = _pathRoot ? _pathRoot.visible !== false : true;

            const item = document.createElement('div');
            item.className = 'display-topic-item display-path-item';
            item.dataset.topicName = topicName;

            // 헤더: 체크박스 + 토픽명
            const header = document.createElement('div');
            header.className = 'display-topic-item-header';

            const checkbox = document.createElement('input');
            checkbox.type = 'checkbox';
            checkbox.checked = visible;
            checkbox.title = '경로 표시/숨기기';
            checkbox.addEventListener('change', function() {
                togglePathVisible(topicName, this.checked);
            });

            const nameSpan = document.createElement('span');
            nameSpan.className = 'display-topic-item-name';
            nameSpan.textContent = topicName;
            nameSpan.title = topicName;

            header.appendChild(checkbox);
            header.appendChild(nameSpan);

            // 설정: 경로 색상
            const settingsDiv = document.createElement('div');
            settingsDiv.className = 'display-topic-item-settings';

            const colorRow = document.createElement('div');
            colorRow.className = 'display-item-color-row';

            const colorLabel = document.createElement('label');
            colorLabel.textContent = '색상';

            // RViz 스타일 팔레트 팝업 (PointCloud2 Solid와 동일한 UI)
            const colorPickerRow = document.createElement('div');
            colorPickerRow.className = 'solid-color-picker-row';

            const colorSwatch = document.createElement('button');
            colorSwatch.className = 'solid-color-swatch';
            colorSwatch.title = '경로 색상 선택';
            colorSwatch.style.backgroundColor = pathColor;

            const colorHexLabel = document.createElement('span');
            colorHexLabel.className = 'solid-color-hex';
            colorHexLabel.textContent = pathColor.toUpperCase();

            colorSwatch.addEventListener('click', function(e) {
                e.stopPropagation();
                const curColor = viewer3DState.topicSettings.get(topicName)?.pathColor || pathColor;
                openColorPickerPopup(null, curColor, colorSwatch, colorHexLabel, function(newColor) {
                    updateTopicSetting(topicName, 'pathColor', newColor);
                    updatePathColor(topicName, newColor);
                });
            });

            colorPickerRow.appendChild(colorSwatch);
            colorPickerRow.appendChild(colorHexLabel);

            colorRow.appendChild(colorLabel);
            colorRow.appendChild(colorPickerRow);
            settingsDiv.appendChild(colorRow);

            // Line Width 슬라이더
            const lineWidthRow = document.createElement('div');
            lineWidthRow.className = 'display-item-maxpts-row';

            const lineWidthLabel = document.createElement('label');
            lineWidthLabel.textContent = `Line Width: ${pathLineWidth}`;

            const lineWidthSlider = document.createElement('input');
            lineWidthSlider.type = 'range';
            lineWidthSlider.min = '1';
            lineWidthSlider.max = '10';
            lineWidthSlider.step = '1';
            lineWidthSlider.value = pathLineWidth;
            lineWidthSlider.className = 'display-item-slider';
            lineWidthSlider.title = '경로 선 굵기';
            lineWidthSlider.addEventListener('input', function() {
                const w = parseInt(this.value);
                lineWidthLabel.textContent = `Line Width: ${w}`;
                updatePathLineWidth(topicName, w);
            });

            lineWidthRow.appendChild(lineWidthLabel);
            lineWidthRow.appendChild(lineWidthSlider);
            settingsDiv.appendChild(lineWidthRow);

            // Alpha (투명도) 슬라이더
            const alphaRow = document.createElement('div');
            alphaRow.className = 'pc2-setting-row';

            const alphaLabel = document.createElement('label');
            alphaLabel.textContent = `Alpha: ${pathAlpha.toFixed(2)}`;

            const alphaSlider = document.createElement('input');
            alphaSlider.type  = 'range';
            alphaSlider.min   = '0.0';
            alphaSlider.max   = '1.0';
            alphaSlider.step  = '0.05';
            alphaSlider.value = pathAlpha;
            alphaSlider.className = 'display-item-slider';
            alphaSlider.title = '경로 투명도 (0=완전투명, 1=불투명)';
            alphaSlider.addEventListener('input', function() {
                const v = parseFloat(this.value);
                alphaLabel.textContent = `Alpha: ${v.toFixed(2)}`;
                updatePathAlpha(topicName, v);
            });

            alphaRow.appendChild(alphaLabel);
            alphaRow.appendChild(alphaSlider);
            settingsDiv.appendChild(alphaRow);

            item.appendChild(header);
            item.appendChild(settingsDiv);
            container.appendChild(item);
        });
    }

    // ── Odometry 섹션 ──
    if (odomTopics.length > 0) {
        const odomLabel = document.createElement('div');
        odomLabel.className = 'display-panel-section-label';
        odomLabel.textContent = 'Odometry';
        container.appendChild(odomLabel);

        odomTopics.forEach(topicName => {
            const settings    = viewer3DState.topicSettings.get(topicName) || {};
            const odomObj     = viewer3DState.odomObjects.get(topicName);
            const visible     = odomObj ? odomObj.group.visible !== false : true;
            const axesLength  = settings.axesLength  !== undefined ? settings.axesLength  : 0.5;
            const axesRadius  = settings.axesRadius  !== undefined ? settings.axesRadius  : 0.01;
            const maxPts      = settings.trajectoryMaxPoints !== undefined
                                    ? settings.trajectoryMaxPoints
                                    : viewer3DState.trajectoryMaxPoints;

            const item = document.createElement('div');
            item.className = 'display-topic-item display-odom-item';
            item.dataset.topicName = topicName;

            // 헤더: 체크박스 + 토픽명
            const header = document.createElement('div');
            header.className = 'display-topic-item-header';

            const checkbox = document.createElement('input');
            checkbox.type = 'checkbox';
            checkbox.checked = visible;
            checkbox.title = '로봇 마커 표시/숨기기';
            checkbox.addEventListener('change', function() {
                toggleOdomVisible(topicName, this.checked);
            });

            const nameSpan = document.createElement('span');
            nameSpan.className = 'display-topic-item-name';
            nameSpan.textContent = topicName;
            nameSpan.title = topicName;

            header.appendChild(checkbox);
            header.appendChild(nameSpan);

            // 설정 영역
            const settingsDiv = document.createElement('div');
            settingsDiv.className = 'display-topic-item-settings';

            // ── Axes Length ──
            const axesLenRow = document.createElement('div');
            axesLenRow.className = 'display-item-maxpts-row';

            const axesLenLabel = document.createElement('label');
            axesLenLabel.textContent = 'Axes Length';

            const axesLenInput = document.createElement('input');
            axesLenInput.type  = 'number';
            axesLenInput.min   = '0.01';
            axesLenInput.max   = '10';
            axesLenInput.step  = '0.01';
            axesLenInput.value = axesLength;
            axesLenInput.className = 'display-item-number-input';
            axesLenInput.title = 'Axes 길이 (m)';
            axesLenInput.addEventListener('change', function() {
                const val = Math.max(0.01, parseFloat(this.value) || 0.5);
                this.value = val;
                updateTopicSetting(topicName, 'axesLength', val);
                const rVal = parseFloat(axesRadInput.value) || 0.01;
                updateOdomAxes(topicName, val, rVal);
                rebuildTrajectoryAxes(topicName);   // 기존 궤적 axes도 즉시 재생성
            });

            axesLenRow.appendChild(axesLenLabel);
            axesLenRow.appendChild(axesLenInput);

            // ── Axes Radius ──
            const axesRadRow = document.createElement('div');
            axesRadRow.className = 'display-item-maxpts-row';

            const axesRadLabel = document.createElement('label');
            axesRadLabel.textContent = 'Axes Radius';

            const axesRadInput = document.createElement('input');
            axesRadInput.type  = 'number';
            axesRadInput.min   = '0.001';
            axesRadInput.max   = '1';
            axesRadInput.step  = '0.001';
            axesRadInput.value = axesRadius;
            axesRadInput.className = 'display-item-number-input';
            axesRadInput.title = 'Axes 반경 (m)';
            axesRadInput.addEventListener('change', function() {
                const val = Math.max(0.001, parseFloat(this.value) || 0.01);
                this.value = val;
                updateTopicSetting(topicName, 'axesRadius', val);
                const lVal = parseFloat(axesLenInput.value) || 0.5;
                updateOdomAxes(topicName, lVal, val);
                rebuildTrajectoryAxes(topicName);   // 기존 궤적 axes도 즉시 재생성
            });

            axesRadRow.appendChild(axesRadLabel);
            axesRadRow.appendChild(axesRadInput);

            // ── 궤적 최대 포인트 ──
            const maxPtsRow = document.createElement('div');
            maxPtsRow.className = 'display-item-maxpts-row';

            const maxPtsLabelEl = document.createElement('label');
            maxPtsLabelEl.textContent = `Max Points: ${maxPts}`;

            const maxPtsSlider = document.createElement('input');
            maxPtsSlider.type  = 'range';
            maxPtsSlider.min   = '1';
            maxPtsSlider.max   = '1000';
            maxPtsSlider.step  = '1';
            maxPtsSlider.value = maxPts;
            maxPtsSlider.className = 'display-item-slider';
            maxPtsSlider.title = '궤적 axes 최대 표시 개수 (1~1000)';
            maxPtsSlider.addEventListener('input', function() {
                const v = parseInt(this.value);
                maxPtsLabelEl.textContent = `Max Points: ${v}`;
                updateTopicSetting(topicName, 'trajectoryMaxPoints', v);
                // 현재 궤적 버퍼가 새 한도를 초과하면 앞부분 FIFO 제거
                const trajObj = viewer3DState.trajectoryObjects.get(topicName);
                if (trajObj) {
                    while (trajObj.axesGroupList.length > v) {
                        const oldest = trajObj.axesGroupList.shift();
                        trajObj.parentGroup.remove(oldest);
                        disposeAxesGroup(oldest);
                    }
                }
            });

            maxPtsRow.appendChild(maxPtsLabelEl);
            maxPtsRow.appendChild(maxPtsSlider);

            settingsDiv.appendChild(axesLenRow);
            settingsDiv.appendChild(axesRadRow);
            settingsDiv.appendChild(maxPtsRow);

            item.appendChild(header);
            item.appendChild(settingsDiv);
            container.appendChild(item);
        });
    }

    // ── LaserScan 섹션 (PointCloud2와 동일 레이아웃) ──
    if (laserScanTopics.length > 0) {
        const lsLabel = document.createElement('div');
        lsLabel.className = 'display-panel-section-label';
        lsLabel.textContent = 'LaserScan';
        container.appendChild(lsLabel);

        laserScanTopics.forEach(topicName => {
            const settings = viewer3DState.topicSettings.get(topicName) || {};
            const lsObj = viewer3DState.laserScanObjects.get(topicName);
            const visible = lsObj ? lsObj.visible !== false : true;
            const colorMode = settings.colorMode || 'rainbow';
            const colorField = settings.colorField || 'intensity';
            const solidColor = settings.solidColor || '#ffffff';
            const pointSize = settings.pointSize !== undefined ? settings.pointSize : 0.1;
            const alpha = settings.alpha !== undefined ? settings.alpha : 1.0;

            const item = document.createElement('div');
            item.className = 'display-topic-item display-laserscan-item';
            item.dataset.topicName = topicName;

            const header = document.createElement('div');
            header.className = 'display-topic-item-header';

            const checkbox = document.createElement('input');
            checkbox.type = 'checkbox';
            checkbox.checked = visible;
            checkbox.title = 'LaserScan 표시/숨기기';
            checkbox.addEventListener('change', function() {
                toggleLaserScanVisible(topicName, this.checked);
            });

            const nameSpan = document.createElement('span');
            nameSpan.className = 'display-topic-item-name';
            nameSpan.textContent = topicName;
            nameSpan.title = topicName;

            const removeBtn = document.createElement('button');
            removeBtn.className = 'display-topic-remove-btn';
            removeBtn.textContent = '✕';
            removeBtn.title = '토픽 제거';
            removeBtn.addEventListener('click', function() {
                unsubscribeFromTopic(topicName);
                viewer3DState.selectedLaserScanTopics = viewer3DState.selectedLaserScanTopics.filter(t => t !== topicName);
                renderDisplayPanel();
            });

            header.appendChild(checkbox);
            header.appendChild(nameSpan);
            header.appendChild(removeBtn);

            const settingsDiv = document.createElement('div');
            settingsDiv.className = 'display-topic-item-settings';

            // ── 색상 모드 (Rainbow / Solid) ──
            const colorModeLabel = document.createElement('label');
            colorModeLabel.textContent = '색상 모드';

            const colorModeSelect = document.createElement('select');
            colorModeSelect.title = '색상 모드 선택';
            [
                { value: 'rainbow', label: 'Rainbow' },
                { value: 'solid', label: 'Solid' }
            ].forEach(opt => {
                const option = document.createElement('option');
                option.value = opt.value;
                option.textContent = opt.label;
                if (opt.value === colorMode) option.selected = true;
                colorModeSelect.appendChild(option);
            });
            colorModeSelect.addEventListener('change', function() {
                updateTopicSetting(topicName, 'colorMode', this.value);
                colorFieldRow.style.display = this.value === 'rainbow' ? 'flex' : 'none';
                solidColorRow.style.display = this.value === 'solid' ? 'flex' : 'none';
            });

            // 색상 필드 선택 (Rainbow 모드) - intensity, range
            const colorFieldRow = document.createElement('div');
            colorFieldRow.style.display = colorMode === 'rainbow' ? 'flex' : 'none';
            colorFieldRow.style.flexDirection = 'column';
            colorFieldRow.style.gap = '2px';

            const colorFieldLabel = document.createElement('label');
            colorFieldLabel.textContent = '색상 필드';

            const colorFieldSelect = document.createElement('select');
            colorFieldSelect.title = '색상 매핑 필드 선택';
            ['intensity', 'range'].forEach(fieldName => {
                const option = document.createElement('option');
                option.value = fieldName;
                option.textContent = fieldName;
                if (fieldName === colorField) option.selected = true;
                colorFieldSelect.appendChild(option);
            });
            colorFieldSelect.addEventListener('change', function() {
                updateTopicSetting(topicName, 'colorField', this.value);
            });

            colorFieldRow.appendChild(colorFieldLabel);
            colorFieldRow.appendChild(colorFieldSelect);

            // 단색 색상 선택 (Solid 모드) - RViz 스타일 팔레트
            const solidColorRow = document.createElement('div');
            solidColorRow.style.display = colorMode === 'solid' ? 'flex' : 'none';
            solidColorRow.style.flexDirection = 'column';
            solidColorRow.style.gap = '2px';

            const solidColorLabel = document.createElement('label');
            solidColorLabel.textContent = '색상';

            const solidColorPickerRow = document.createElement('div');
            solidColorPickerRow.className = 'solid-color-picker-row';

            const colorSwatch = document.createElement('button');
            colorSwatch.className = 'solid-color-swatch';
            colorSwatch.title = '색상 선택';
            colorSwatch.style.backgroundColor = solidColor;

            const colorHexLabel = document.createElement('span');
            colorHexLabel.className = 'solid-color-hex';
            colorHexLabel.textContent = solidColor.toUpperCase();

            colorSwatch.addEventListener('click', function(e) {
                e.stopPropagation();
                openColorPickerPopup(topicName, solidColor, colorSwatch, colorHexLabel, function(newColor) {
                    updateLaserScanColor(topicName, newColor);
                });
            });

            solidColorPickerRow.appendChild(colorSwatch);
            solidColorPickerRow.appendChild(colorHexLabel);

            solidColorRow.appendChild(solidColorLabel);
            solidColorRow.appendChild(solidColorPickerRow);

            settingsDiv.appendChild(colorModeLabel);
            settingsDiv.appendChild(colorModeSelect);
            settingsDiv.appendChild(colorFieldRow);
            settingsDiv.appendChild(solidColorRow);

            // ── Size (PointCloud2와 동일: min=0.01, max=1.0, step=0.01) ──
            const sizeRow = document.createElement('div');
            sizeRow.className = 'pc2-setting-row';

            const sizeLabelEl = document.createElement('label');
            sizeLabelEl.textContent = `Size: ${pointSize.toFixed(2)}`;

            const sizeSlider = document.createElement('input');
            sizeSlider.type = 'range';
            sizeSlider.min = '0.01';
            sizeSlider.max = '1.0';
            sizeSlider.step = '0.01';
            sizeSlider.value = pointSize;
            sizeSlider.className = 'display-item-slider';
            sizeSlider.title = '포인트 크기 (m)';
            sizeSlider.addEventListener('input', function() {
                const v = parseFloat(this.value);
                sizeLabelEl.textContent = `Size: ${v.toFixed(2)}`;
                updateLaserScanPointSize(topicName, v);
            });

            sizeRow.appendChild(sizeLabelEl);
            sizeRow.appendChild(sizeSlider);
            settingsDiv.appendChild(sizeRow);

            // ── Alpha (투명도) ──
            const alphaRow = document.createElement('div');
            alphaRow.className = 'pc2-setting-row';

            const alphaLabelEl = document.createElement('label');
            alphaLabelEl.textContent = `Alpha: ${alpha.toFixed(2)}`;

            const alphaSlider = document.createElement('input');
            alphaSlider.type = 'range';
            alphaSlider.min = '0.0';
            alphaSlider.max = '1.0';
            alphaSlider.step = '0.01';
            alphaSlider.value = alpha;
            alphaSlider.className = 'display-item-slider';
            alphaSlider.title = '투명도 (0=완전투명, 1=불투명)';
            alphaSlider.addEventListener('input', function() {
                const v = parseFloat(this.value);
                alphaLabelEl.textContent = `Alpha: ${v.toFixed(2)}`;
                updateLaserScanAlpha(topicName, v);
            });

            alphaRow.appendChild(alphaLabelEl);
            alphaRow.appendChild(alphaSlider);
            settingsDiv.appendChild(alphaRow);

            item.appendChild(header);
            item.appendChild(settingsDiv);
            container.appendChild(item);
        });
    }

    // ── TF 섹션 ──
    if (tfTopics.length > 0) {
        const tfLabel = document.createElement('div');
        tfLabel.className = 'display-panel-section-label';
        tfLabel.textContent = 'TF';
        container.appendChild(tfLabel);

        tfTopics.forEach(function(topicName) {
            const tfObj   = viewer3DState.tfObjects.get(topicName);
            const visible = tfObj ? tfObj.visible !== false : true;
            const tfSettings = viewer3DState.topicSettings.get(topicName) || {};
            const curAxesSize = tfSettings.tfAxesSize !== undefined ? tfSettings.tfAxesSize : 0.3;

            const item = document.createElement('div');
            item.className = 'display-topic-item display-tf-item';
            item.dataset.topicName = topicName;

            // 헤더: 체크박스 + 토픽명 + 제거 버튼
            const header = document.createElement('div');
            header.className = 'display-topic-item-header';

            const checkbox = document.createElement('input');
            checkbox.type    = 'checkbox';
            checkbox.checked = visible;
            checkbox.title   = 'TF 표시/숨기기';
            checkbox.addEventListener('change', function() {
                toggleTFVisible(topicName, this.checked);
            });

            const nameSpan = document.createElement('span');
            nameSpan.className = 'display-topic-item-name';
            nameSpan.textContent = topicName;
            nameSpan.title = topicName;

            // 제거 버튼
            const removeBtn = document.createElement('button');
            removeBtn.className = 'display-topic-remove-btn';
            removeBtn.textContent = '✕';
            removeBtn.title = '토픽 제거';
            removeBtn.addEventListener('click', function() {
                unsubscribeFromTopic(topicName);
                viewer3DState.selectedTFTopics = viewer3DState.selectedTFTopics.filter(t => t !== topicName);
                renderDisplayPanel();
            });

            header.appendChild(checkbox);
            header.appendChild(nameSpan);
            header.appendChild(removeBtn);
            item.appendChild(header);

            // ── 설정 영역: Axes Size / Axes Radius / Text Size ──
            const settingsDiv = document.createElement('div');
            settingsDiv.className = 'display-topic-item-settings';

            // ── Axes Size ──
            const axesSizeRow = document.createElement('div');
            axesSizeRow.className = 'display-item-maxpts-row';

            const axesSizeLabel = document.createElement('label');
            axesSizeLabel.textContent = 'Axes Size';

            const axesSizeInput = document.createElement('input');
            axesSizeInput.type      = 'number';
            axesSizeInput.min       = '0.01';
            axesSizeInput.max       = '10';
            axesSizeInput.step      = '0.01';
            axesSizeInput.value     = curAxesSize;
            axesSizeInput.className = 'display-item-number-input';
            axesSizeInput.title     = 'TF Axes 길이 (m)';
            axesSizeInput.addEventListener('change', function() {
                const val = Math.max(0.01, parseFloat(this.value) || 0.3);
                this.value = val;
                updateTopicSetting(topicName, 'tfAxesSize', val);
                viewer3DState.selectedTFTopics.forEach(function(tn) { rebuildTFScene(tn); });
            });

            axesSizeRow.appendChild(axesSizeLabel);
            axesSizeRow.appendChild(axesSizeInput);
            settingsDiv.appendChild(axesSizeRow);

            // ── Axes Radius (화살표 원통 반경 — Odometry 동일) ──
            const curAxesRadius = tfSettings.tfAxesRadius !== undefined ? tfSettings.tfAxesRadius : 0.01;

            const axesRadRow = document.createElement('div');
            axesRadRow.className = 'display-item-maxpts-row';

            const axesRadLabel = document.createElement('label');
            axesRadLabel.textContent = 'Axes Radius';

            const axesRadInput = document.createElement('input');
            axesRadInput.type      = 'number';
            axesRadInput.min       = '0.001';
            axesRadInput.max       = '1';
            axesRadInput.step      = '0.001';
            axesRadInput.value     = curAxesRadius;
            axesRadInput.className = 'display-item-number-input';
            axesRadInput.title     = 'TF 화살표 원통 반경 (m)';
            axesRadInput.addEventListener('change', function() {
                const val = Math.max(0.001, parseFloat(this.value) || 0.01);
                this.value = val;
                updateTopicSetting(topicName, 'tfAxesRadius', val);
                viewer3DState.selectedTFTopics.forEach(function(tn) { rebuildTFScene(tn); });
            });

            axesRadRow.appendChild(axesRadLabel);
            axesRadRow.appendChild(axesRadInput);
            settingsDiv.appendChild(axesRadRow);

            // ── Text Size (라벨 크기 — Axes Size 와 독립) ──
            const curTextSize = tfSettings.tfTextSize !== undefined ? tfSettings.tfTextSize : 0.2;

            const textSizeRow = document.createElement('div');
            textSizeRow.className = 'display-item-maxpts-row';

            const textSizeLabel = document.createElement('label');
            textSizeLabel.textContent = 'Text Size';

            const textSizeInput = document.createElement('input');
            textSizeInput.type      = 'number';
            textSizeInput.min       = '0.01';
            textSizeInput.max       = '5';
            textSizeInput.step      = '0.01';
            textSizeInput.value     = curTextSize;
            textSizeInput.className = 'display-item-number-input';
            textSizeInput.title     = '프레임 이름 라벨 높이 (m)';
            textSizeInput.addEventListener('change', function() {
                const val = Math.max(0.01, parseFloat(this.value) || 0.2);
                this.value = val;
                updateTopicSetting(topicName, 'tfTextSize', val);
                viewer3DState.selectedTFTopics.forEach(function(tn) { rebuildTFScene(tn); });
            });

            textSizeRow.appendChild(textSizeLabel);
            textSizeRow.appendChild(textSizeInput);
            settingsDiv.appendChild(textSizeRow);

            item.appendChild(settingsDiv);

            container.appendChild(item);
        });
    }

    // ── Livox LiDAR 섹션 ──
    if (livoxTopics.length > 0) {
        // 섹션 라벨: "Livox LiDAR" + LIVOX 배지
        const livoxLabel = document.createElement('div');
        livoxLabel.className = 'display-panel-section-label';
        livoxLabel.textContent = 'Livox LiDAR';

        const livoxBadge = document.createElement('span');
        livoxBadge.className = 'livox-badge';
        livoxBadge.textContent = 'LIVOX';
        livoxLabel.appendChild(livoxBadge);
        container.appendChild(livoxLabel);

        livoxTopics.forEach(topicName => {
            const settings   = viewer3DState.topicSettings.get(topicName) || {};
            const colorMode  = settings.colorMode  || 'rainbow';
            const colorField = settings.colorField || 'reflectivity';
            const solidColor = settings.solidColor || '#ffffff';
            const pointSize  = settings.pointSize  !== undefined ? settings.pointSize  : 0.1;
            const alpha      = settings.alpha      !== undefined ? settings.alpha      : 1.0;
            const decayTime  = settings.decayTime  !== undefined ? settings.decayTime  : 0;
            const tagFilter  = settings.tagFilter  !== undefined ? settings.tagFilter  : false;

            const meshEntry = viewer3DState.pointCloudMeshes[topicName];
            const visible = meshEntry ? meshEntry.visible !== false : true;

            // 토픽 항목 컨테이너
            const item = document.createElement('div');
            item.className = 'display-topic-item';
            item.dataset.topicName = topicName;

            // 헤더: 체크박스 + 토픽명 + 제거 버튼
            const header = document.createElement('div');
            header.className = 'display-topic-item-header';

            const checkbox = document.createElement('input');
            checkbox.type = 'checkbox';
            checkbox.checked = visible;
            checkbox.title = '표시/숨기기';
            checkbox.addEventListener('change', function() {
                toggleTopicVisible(topicName, this.checked);
            });

            const nameSpan = document.createElement('span');
            nameSpan.className = 'display-topic-item-name';
            nameSpan.textContent = topicName;
            nameSpan.title = topicName;

            const removeBtn = document.createElement('button');
            removeBtn.className = 'display-topic-remove-btn';
            removeBtn.textContent = '✕';
            removeBtn.title = '토픽 제거';
            removeBtn.addEventListener('click', function() {
                unsubscribeFromTopic(topicName);
                renderDisplayPanel();
            });

            header.appendChild(checkbox);
            header.appendChild(nameSpan);
            header.appendChild(removeBtn);

            // 설정 영역
            const settingsDiv = document.createElement('div');
            settingsDiv.className = 'display-topic-item-settings';

            // ── Style 선택 (RViz 스타일) ──
            const pcStyle = settings.style || 'squares';

            const styleLabel = document.createElement('label');
            styleLabel.textContent = 'Style';

            const styleSelect = document.createElement('select');
            styleSelect.title = '렌더링 스타일 선택 (RViz 스타일)';
            [
                { value: 'points',       label: 'Points' },
                { value: 'squares',      label: 'Squares' },
                { value: 'flat_squares', label: 'Flat Squares' },
                { value: 'spheres',      label: 'Spheres' },
                { value: 'boxes',        label: 'Boxes' },
                { value: 'tiles',        label: 'Tiles' },
            ].forEach(opt => {
                const option = document.createElement('option');
                option.value = opt.value;
                option.textContent = opt.label;
                if (opt.value === pcStyle) option.selected = true;
                styleSelect.appendChild(option);
            });
            styleSelect.addEventListener('change', function() {
                updatePointStyle(topicName, this.value);
            });

            settingsDiv.appendChild(styleLabel);
            settingsDiv.appendChild(styleSelect);

            // ── 색상 모드 선택 (Rainbow / By Line / Solid) ──
            const colorModeLabel = document.createElement('label');
            colorModeLabel.textContent = '색상 모드';

            const colorModeSelect = document.createElement('select');
            colorModeSelect.title = '색상 모드 선택';
            [
                { value: 'rainbow',  label: 'Rainbow' },
                { value: 'byline',   label: 'By Line' },
                { value: 'solid',    label: 'Solid' }
            ].forEach(opt => {
                const option = document.createElement('option');
                option.value = opt.value;
                option.textContent = opt.label;
                if (opt.value === colorMode) option.selected = true;
                colorModeSelect.appendChild(option);
            });

            // 단색 색상 선택 행 (Solid 모드)
            const solidColorRow = document.createElement('div');
            solidColorRow.style.display = colorMode === 'solid' ? 'flex' : 'none';
            solidColorRow.style.flexDirection = 'column';
            solidColorRow.style.gap = '2px';

            const solidColorLabel = document.createElement('label');
            solidColorLabel.textContent = '색상';

            const solidColorPickerRow = document.createElement('div');
            solidColorPickerRow.className = 'solid-color-picker-row';

            const colorSwatch = document.createElement('button');
            colorSwatch.className = 'solid-color-swatch';
            colorSwatch.title = '색상 선택';
            colorSwatch.style.backgroundColor = solidColor;

            const colorHexLabel = document.createElement('span');
            colorHexLabel.className = 'solid-color-hex';
            colorHexLabel.textContent = solidColor.toUpperCase();

            colorSwatch.addEventListener('click', function(e) {
                e.stopPropagation();
                openColorPickerPopup(topicName, solidColor, colorSwatch, colorHexLabel);
            });

            solidColorPickerRow.appendChild(colorSwatch);
            solidColorPickerRow.appendChild(colorHexLabel);

            solidColorRow.appendChild(solidColorLabel);
            solidColorRow.appendChild(solidColorPickerRow);

            // ── 색상 필드 선택 (Rainbow 모드에서만 표시) ──
            const colorFieldRow = document.createElement('div');
            colorFieldRow.style.display = colorMode === 'rainbow' ? 'flex' : 'none';
            colorFieldRow.style.flexDirection = 'column';
            colorFieldRow.style.gap = '2px';

            const colorFieldLabel = document.createElement('label');
            colorFieldLabel.textContent = '색상 필드';

            const colorFieldSelect = document.createElement('select');
            colorFieldSelect.title = '색상 매핑 필드 선택';
            [
                { value: 'reflectivity', label: 'reflectivity' },
                { value: 'z',            label: 'z' },
                { value: 'x',            label: 'x' },
                { value: 'y',            label: 'y' }
            ].forEach(opt => {
                const option = document.createElement('option');
                option.value = opt.value;
                option.textContent = opt.label;
                if (opt.value === colorField) option.selected = true;
                colorFieldSelect.appendChild(option);
            });
            colorFieldSelect.addEventListener('change', function() {
                updateTopicSetting(topicName, 'colorField', this.value);
            });

            colorFieldRow.appendChild(colorFieldLabel);
            colorFieldRow.appendChild(colorFieldSelect);

            colorModeSelect.addEventListener('change', function() {
                updateTopicSetting(topicName, 'colorMode', this.value);
                colorFieldRow.style.display = this.value === 'rainbow' ? 'flex' : 'none';
                solidColorRow.style.display = this.value === 'solid'   ? 'flex' : 'none';
            });

            settingsDiv.appendChild(colorModeLabel);
            settingsDiv.appendChild(colorModeSelect);
            settingsDiv.appendChild(colorFieldRow);
            settingsDiv.appendChild(solidColorRow);

            // ── Tag Filter 체크박스 (Livox 전용: 노이즈 제외) ──
            const tagFilterRow = document.createElement('div');
            tagFilterRow.className = 'pc2-setting-row';
            tagFilterRow.style.flexDirection = 'row';
            tagFilterRow.style.alignItems = 'center';
            tagFilterRow.style.justifyContent = 'flex-start';
            tagFilterRow.style.gap = '6px';

            const tagFilterCheckbox = document.createElement('input');
            tagFilterCheckbox.type    = 'checkbox';
            tagFilterCheckbox.checked = tagFilter;
            tagFilterCheckbox.title   = '노이즈 포인트 제외 (tag bit[3:2] = 0x0C): 0=고신뢰, 1=노이즈';
            tagFilterCheckbox.id      = `livox-tag-filter-${topicName.replace(/\//g, '_')}`;
            tagFilterCheckbox.addEventListener('change', function() {
                updateTopicSetting(topicName, 'tagFilter', this.checked);
            });

            const tagFilterLabel = document.createElement('label');
            tagFilterLabel.htmlFor     = tagFilterCheckbox.id;
            tagFilterLabel.textContent = '노이즈 제외';
            tagFilterLabel.style.cursor = 'pointer';

            tagFilterRow.appendChild(tagFilterCheckbox);
            tagFilterRow.appendChild(tagFilterLabel);
            settingsDiv.appendChild(tagFilterRow);

            // ── Point Size 슬라이더 ──
            const sizeRow = document.createElement('div');
            sizeRow.className = 'pc2-setting-row';

            const sizeLabelEl = document.createElement('label');
            sizeLabelEl.textContent = `Size: ${pointSize.toFixed(2)}`;

            const sizeSlider = document.createElement('input');
            sizeSlider.type  = 'range';
            sizeSlider.min   = '0.01';
            sizeSlider.max   = '1.0';
            sizeSlider.step  = '0.01';
            sizeSlider.value = pointSize;
            sizeSlider.className = 'display-item-slider';
            sizeSlider.title = '포인트 크기 (m)';
            sizeSlider.addEventListener('input', function() {
                const v = parseFloat(this.value);
                sizeLabelEl.textContent = `Size: ${v.toFixed(2)}`;
                updatePointSize(topicName, v);
            });

            sizeRow.appendChild(sizeLabelEl);
            sizeRow.appendChild(sizeSlider);
            settingsDiv.appendChild(sizeRow);

            // ── Alpha (투명도) 슬라이더 ──
            const alphaRow = document.createElement('div');
            alphaRow.className = 'pc2-setting-row';

            const alphaLabelEl = document.createElement('label');
            alphaLabelEl.textContent = `Alpha: ${alpha.toFixed(2)}`;

            const alphaSlider = document.createElement('input');
            alphaSlider.type  = 'range';
            alphaSlider.min   = '0.0';
            alphaSlider.max   = '1.0';
            alphaSlider.step  = '0.05';
            alphaSlider.value = alpha;
            alphaSlider.className = 'display-item-slider';
            alphaSlider.title = '투명도 (0=완전투명, 1=불투명)';
            alphaSlider.addEventListener('input', function() {
                const v = parseFloat(this.value);
                alphaLabelEl.textContent = `Alpha: ${v.toFixed(2)}`;
                updatePointAlpha(topicName, v);
            });

            alphaRow.appendChild(alphaLabelEl);
            alphaRow.appendChild(alphaSlider);
            settingsDiv.appendChild(alphaRow);

            // ── Decay Time 슬라이더 ──
            const decayRow = document.createElement('div');
            decayRow.className = 'pc2-setting-row';

            const decayLabelEl = document.createElement('label');
            decayLabelEl.textContent = `Decay Time: ${decayTime.toFixed(1)}s`;

            const decaySlider = document.createElement('input');
            decaySlider.type  = 'range';
            decaySlider.min   = '0';
            decaySlider.max   = '30';
            decaySlider.step  = '0.5';
            decaySlider.value = decayTime;
            decaySlider.className = 'display-item-slider';
            decaySlider.title = 'Decay Time (0=최신 프레임만, >0=n초간 누적 표시)';
            decaySlider.addEventListener('input', function() {
                const v = parseFloat(this.value);
                decayLabelEl.textContent = `Decay Time: ${v.toFixed(1)}s`;
                updateTopicSetting(topicName, 'decayTime', v);
                if (v <= 0) {
                    clearDecayObjects(topicName);
                } else {
                    _pruneExpiredDecayObjects(topicName, v);
                }
            });

            decayRow.appendChild(decayLabelEl);
            decayRow.appendChild(decaySlider);
            settingsDiv.appendChild(decayRow);

            item.appendChild(header);
            item.appendChild(settingsDiv);
            container.appendChild(item);
        });
    }

    // ── Image 섹션 ──
    if (imageTopics.length > 0) {
        const imageLabel      = document.createElement('div');
        imageLabel.className  = 'display-panel-section-label';
        imageLabel.textContent = 'Image';

        const imageBadge      = document.createElement('span');
        imageBadge.className  = 'image-badge';
        imageBadge.textContent = 'IMG';
        imageLabel.appendChild(imageBadge);
        container.appendChild(imageLabel);

        imageTopics.forEach(function(topicName) {
            const item       = document.createElement('div');
            item.className   = 'display-topic-item';
            item.dataset.topicName = topicName;

            const header     = document.createElement('div');
            header.className = 'display-topic-item-header';

            // 체크박스: image cell 표시/숨기기
            const cellId  = 'image-cell-' + topicName.replace(/\//g, '_');
            const cell    = document.getElementById(cellId);
            const visible = !cell || cell.style.display !== 'none';

            const checkbox   = document.createElement('input');
            checkbox.type    = 'checkbox';
            checkbox.checked = visible;
            checkbox.title   = '표시/숨기기';
            checkbox.addEventListener('change', function() {
                const c = document.getElementById(cellId);
                if (c) c.style.display = this.checked ? '' : 'none';
            });

            const nameSpan     = document.createElement('span');
            nameSpan.className = 'display-topic-item-name';
            nameSpan.textContent = topicName;
            nameSpan.title     = topicName;

            header.appendChild(checkbox);
            header.appendChild(nameSpan);
            item.appendChild(header);
            container.appendChild(item);
        });
    }
}

// =============================================
// PathRenderer: nav_msgs/Path → THREE.Line (녹색)
// =============================================

/**
 * Path 토픽 구독 및 THREE.Line으로 시각화
 * @param {string} topicName - nav_msgs/msg/Path 토픽명
 * @returns {ROSLIB.Topic} 구독 객체
 */
function subscribeToPath(topicName) {
    if (!viewer3DState.rosConnected) {
        console.warn('[Path] Not connected to ROS');
        return;
    }

    // 재구독 방지
    if (viewer3DState.topicSubscriptions.has(topicName)) {
        return viewer3DState.topicSubscriptions.get(topicName);
    }

    console.log('[Path] Subscribing to topic:', topicName);

    const topic = new ROSLIB.Topic({
        ros: viewer3DState.ros,
        name: topicName,
        messageType: 'nav_msgs/msg/Path',
        throttle_rate: 500,   // 2Hz
        queue_length: 1
    });

    viewer3DState.topicSubscriptions.set(topicName, topic);

    topic.subscribe(function(message) {
        if (!message.poses || message.poses.length === 0) return;

        // frame_id 저장
        const frameId = (message.header && message.header.frame_id) ? message.header.frame_id : '';
        viewer3DState.topicFrameIds.set(topicName, frameId);
        viewer3DState.needsRender = true;

        const existingObj = viewer3DState.pathObjects.get(topicName);

        // 숨겨진 경우 스킵
        if (existingObj && existingObj.visible === false) return;

        // poses → THREE.Vector3 배열 변환 (로컬 frame 좌표 그대로)
        const positions = message.poses.map(poseStamped => {
            const p = poseStamped.pose.position;
            return new THREE.Vector3(p.x, p.y, p.z);
        });

        if (existingObj) {
            // 기존 라인 geometry 업데이트
            existingObj.line.geometry.dispose();
            existingObj.line.geometry = new THREE.BufferGeometry().setFromPoints(positions);
            // frame transform 재적용
            applyFrameTransformToObject(existingObj.frameGroup, frameId);
        } else {
            // 신규 THREE.Line 생성 (저장된 설정 반영)
            const s        = viewer3DState.topicSettings.get(topicName) || {};
            const color    = s.pathColor     ? parseInt(s.pathColor.replace('#', ''), 16) : 0x00ff00;
            const width    = s.pathLineWidth || 2;
            const a        = s.pathAlpha     !== undefined ? s.pathAlpha : 1.0;
            const geometry = new THREE.BufferGeometry().setFromPoints(positions);
            const material = new THREE.LineBasicMaterial({
                color, linewidth: width,
                transparent: a < 1.0, opacity: a,
            });
            const line       = new THREE.Line(geometry, material);
            const frameGroup = new THREE.Group();
            frameGroup.add(line);
            viewer3DState.scene.add(frameGroup);
            // frame transform 적용
            applyFrameTransformToObject(frameGroup, frameId);
            viewer3DState.pathObjects.set(topicName, { frameGroup, line, visible: true });
            console.log('[Path] Line added for topic:', topicName);
        }
    });

    return topic;
}

// =============================================
// TrajectoryTracker: Odometry 기반 궤적 추적
// =============================================

/**
 * Odometry 위치를 궤적 버퍼에 추가하고 THREE.Line 업데이트
 * @param {string} topicName - Odometry 토픽명 (궤적 키)
 * @param {number} x
 * @param {number} y
 * @param {number} z
 */
/**
 * 단일 axes group을 dispose (geometry + material)
 * @param {THREE.Group} axesGroup
 */
function disposeAxesGroup(axesGroup) {
    axesGroup.traverse(function(child) {
        if (child.isMesh) {
            child.geometry.dispose();
            if (child.material) child.material.dispose();
        }
    });
}

/**
 * Odometry 위치/자세(world 기준)를 궤적 버퍼에 추가하고 axes group으로 시각화 (FIFO).
 * x,y,z,qx,qy,qz,qw 는 이미 fixedFrame 기준 world 좌표여야 함.
 * 각 포인트마다 독립적인 axes group을 생성하여 parentGroup에 추가.
 * maxPoints 초과 시 가장 오래된 axes group 제거(FIFO).
 *
 * @param {string} topicName
 * @param {number} x
 * @param {number} y
 * @param {number} z
 * @param {number} qx - Quaternion x
 * @param {number} qy - Quaternion y
 * @param {number} qz - Quaternion z
 * @param {number} qw - Quaternion w
 * @param {string} [frameId] - 원본 ROS frame_id (topicFrameIds 저장용)
 */
function updateTrajectory(topicName, x, y, z, qx, qy, qz, qw, frameId) {
    // per-topic 설정
    const settings  = viewer3DState.topicSettings.get(topicName) || {};
    const maxPoints = Math.min(
        settings.trajectoryMaxPoints !== undefined
            ? settings.trajectoryMaxPoints
            : viewer3DState.trajectoryMaxPoints,
        1000   // 하드 상한
    );
    const axesLength = settings.axesLength !== undefined ? settings.axesLength : 0.5;
    const axesRadius = settings.axesRadius !== undefined ? settings.axesRadius : 0.01;

    // traj용 frameId 키 저장 (reapplyAllFrameTransforms에서는 사용하지 않음 — world 좌표로 직접 저장)
    if (frameId !== undefined) {
        viewer3DState.topicFrameIds.set(topicName + '_traj', frameId);
    }

    let trajObj = viewer3DState.trajectoryObjects.get(topicName);

    if (!trajObj) {
        // 최초 생성: parentGroup을 scene에 추가
        const parentGroup = new THREE.Group();
        viewer3DState.scene.add(parentGroup);
        trajObj = { parentGroup, axesGroupList: [], visible: true };
        viewer3DState.trajectoryObjects.set(topicName, trajObj);
        console.log('[Trajectory] Axes parent group created for topic:', topicName);
    }

    // 새 axes group 생성 (world position + quaternion 직접 적용)
    const axesGroup = createOdomAxesGroup(axesLength, axesRadius);
    axesGroup.position.set(x, y, z);
    axesGroup.setRotationFromQuaternion(new THREE.Quaternion(qx, qy, qz, qw));
    trajObj.parentGroup.add(axesGroup);
    trajObj.axesGroupList.push(axesGroup);

    // 최대 포인트 초과 시 FIFO: 가장 오래된 axes group 제거
    while (trajObj.axesGroupList.length > maxPoints) {
        const oldest = trajObj.axesGroupList.shift();
        trajObj.parentGroup.remove(oldest);
        disposeAxesGroup(oldest);
    }
}

// =============================================
// OdometryRenderer: nav_msgs/Odometry → THREE.Group (RViz 스타일 Axes)
// =============================================

/**
 * RViz 스타일 Axes THREE.Group 생성
 * X축: 빨강, Y축: 초록, Z축: 파랑 (cylinder + cone 화살표)
 * @param {number} length - 전체 축 길이 (m)
 * @param {number} radius - cylinder 반경 (m)
 * @returns {THREE.Group}
 */
function createOdomAxesGroup(length, radius) {
    const group = new THREE.Group();
    const radSegs  = 6;
    const stemLen  = length * 0.8;
    const headLen  = length * 0.2;
    const headRad  = radius * 2.5;

    // [색상, axisGroup.rotation(x,z)] – CylinderGeometry는 Y방향 기본
    const axesDefs = [
        { color: 0xff2020, rx: 0,           rz: -Math.PI / 2 },  // X축: 빨강
        { color: 0x20ff20, rx: 0,           rz: 0            },  // Y축: 초록
        { color: 0x2020ff, rx: Math.PI / 2, rz: 0            },  // Z축: 파랑
    ];

    axesDefs.forEach(({ color, rx, rz }) => {
        const axisGroup = new THREE.Group();
        axisGroup.rotation.set(rx, 0, rz);

        const mat = new THREE.MeshLambertMaterial({ color });

        // stem: Y축 방향 cylinder, 중심을 stemLen/2 위치로 이동
        const stemGeo = new THREE.CylinderGeometry(radius, radius, stemLen, radSegs);
        const stem    = new THREE.Mesh(stemGeo, mat);
        stem.position.y = stemLen / 2;
        axisGroup.add(stem);

        // cone (화살촉): stem 끝에서 headLen/2 추가
        const coneGeo = new THREE.ConeGeometry(headRad, headLen, radSegs);
        const cone    = new THREE.Mesh(coneGeo, mat);
        cone.position.y = stemLen + headLen / 2;
        axisGroup.add(cone);

        group.add(axisGroup);
    });

    return group;
}

/**
 * Odometry 토픽 구독 및 THREE.Group으로 로봇 위치/방향 시각화
 * 수신마다 updateTrajectory() 호출
 * @param {string} topicName - nav_msgs/msg/Odometry 토픽명
 * @returns {ROSLIB.Topic} 구독 객체
 */
function subscribeToOdometry(topicName) {
    if (!viewer3DState.rosConnected) {
        console.warn('[Odom] Not connected to ROS');
        return;
    }

    // 재구독 방지
    if (viewer3DState.topicSubscriptions.has(topicName)) {
        return viewer3DState.topicSubscriptions.get(topicName);
    }

    console.log('[Odom] Subscribing to topic:', topicName);

    const topic = new ROSLIB.Topic({
        ros: viewer3DState.ros,
        name: topicName,
        messageType: 'nav_msgs/msg/Odometry',
        throttle_rate: 100,   // 10Hz
        queue_length: 1
    });

    viewer3DState.topicSubscriptions.set(topicName, topic);

    topic.subscribe(function(message) {
        viewer3DState.needsRender = true;
        const pos = message.pose.pose.position;
        const ori = message.pose.pose.orientation;

        const x  = pos.x,  y  = pos.y,  z  = pos.z;
        const qx = ori.x, qy = ori.y, qz = ori.z, qw = ori.w;

        // frame_id 저장 및 world pose 계산
        const frameId = (message.header && message.header.frame_id) ? message.header.frame_id : '';
        viewer3DState.topicFrameIds.set(topicName, frameId);
        const worldPose = computeOdomWorldPose(x, y, z, qx, qy, qz, qw, frameId);

        let existingObj = viewer3DState.odomObjects.get(topicName);

        if (!existingObj) {
            // RViz 스타일 axes 그룹 최초 생성
            const group      = new THREE.Group();
            const settings   = viewer3DState.topicSettings.get(topicName) || {};
            const axesLength = settings.axesLength !== undefined ? settings.axesLength : 0.5;
            const axesRadius = settings.axesRadius !== undefined ? settings.axesRadius : 0.01;
            const axesGroup  = createOdomAxesGroup(axesLength, axesRadius);
            group.add(axesGroup);
            viewer3DState.scene.add(group);
            existingObj = { group, axesGroup, visible: true, lastPose: null };
            viewer3DState.odomObjects.set(topicName, existingObj);
            console.log('[Odom] Axes group added for topic:', topicName);
        } else {
            // 숨겨진 경우 스킵
            if (existingObj.visible === false) return;
        }

        // lastPose 갱신 (fixedFrame 변경 시 재계산에 사용)
        existingObj.lastPose = { x, y, z, qx, qy, qz, qw, frameId };

        if (!worldPose) {
            // TF 없음 → 숨김
            existingObj.group.visible = false;
        } else {
            existingObj.group.visible = true;
            existingObj.group.position.copy(worldPose.position);
            existingObj.group.setRotationFromQuaternion(worldPose.quaternion);
        }

        // 궤적 업데이트 (world pose 전달)
        if (worldPose) {
            updateTrajectory(topicName, worldPose.position.x, worldPose.position.y, worldPose.position.z,
                worldPose.quaternion.x, worldPose.quaternion.y, worldPose.quaternion.z, worldPose.quaternion.w,
                frameId);
        }
    });

    return topic;
}

// =============================================
// LaserScanRenderer: sensor_msgs/LaserScan → THREE.Points (RViz LaserScanDisplay 참조)
// =============================================

/**
 * LaserScan 메시지를 3D 포인트+색상으로 변환 (RViz laser_geometry + intensity 기반 rainbow)
 * angle_min + i*angle_increment → x=range*cos(angle), y=range*sin(angle), z=0
 * @param {Object} message - rosbridge sensor_msgs/msg/LaserScan 메시지
 * @param {Object} settings - { colorMode, colorField, solidColor }
 * @returns {{ positions: Float32Array, colors: Float32Array, count: number }}
 */
function laserScanToPointsAndColors(message, settings) {
    const ranges = message.ranges || [];
    const intensities = message.intensities || [];
    const angleMin = message.angle_min ?? 0;
    const angleIncrement = message.angle_increment ?? 0;
    const rangeMin = message.range_min ?? 0;
    const rangeMax = message.range_max ?? Infinity;

    const colorMode = (settings && settings.colorMode) || 'rainbow';
    const colorField = (settings && settings.colorField) || 'intensity';
    const solidColor = (settings && settings.solidColor) || '#ffffff';

    let solidR = 1, solidG = 1, solidB = 1;
    if (colorMode === 'solid') {
        const hex = solidColor.replace('#', '');
        solidR = parseInt(hex.substring(0, 2), 16) / 255;
        solidG = parseInt(hex.substring(2, 4), 16) / 255;
        solidB = parseInt(hex.substring(4, 6), 16) / 255;
    }

    const positions = [];
    const colors = [];
    let fieldMin = Infinity, fieldMax = -Infinity;

    for (let i = 0; i < ranges.length; i++) {
        const r = ranges[i];
        if (typeof r !== 'number' || !Number.isFinite(r) || r < rangeMin || r > rangeMax) continue;

        const angle = angleMin + i * angleIncrement;
        const x = r * Math.cos(angle);
        const y = r * Math.sin(angle);
        positions.push(x, y, 0);

        if (colorMode === 'rainbow') {
            let fv;
            if (colorField === 'intensity' && i < intensities.length && typeof intensities[i] === 'number' && Number.isFinite(intensities[i])) {
                fv = intensities[i];
            } else if (colorField === 'range') {
                fv = r;
            } else {
                fv = (i < intensities.length && Number.isFinite(intensities[i])) ? intensities[i] : r;
            }
            if (!Number.isFinite(fv)) fv = 0;
            if (fv < fieldMin) fieldMin = fv;
            if (fv > fieldMax) fieldMax = fv;
            colors.push(fv);
        }
    }

    const count = positions.length / 3;
    const posArr = new Float32Array(positions);
    const colArr = new Float32Array(count * 3);

    if (colorMode === 'rainbow' && count > 0) {
        const rangeSpan = Math.max(1e-6, fieldMax - fieldMin);
        for (let i = 0; i < count; i++) {
            const fv = colors[i];
            const t = Math.min(1.0, Math.max(0.0, (fv - fieldMin) / rangeSpan));
            const hue = t * 0.75;
            const h6 = hue * 6.0;
            const hi = h6 | 0;
            const f_ = h6 - hi;
            let cr, cg, cb;
            switch (hi % 6) {
                case 0: cr = 1; cg = f_; cb = 0; break;
                case 1: cr = 1 - f_; cg = 1; cb = 0; break;
                case 2: cr = 0; cg = 1; cb = f_; break;
                case 3: cr = 0; cg = 1 - f_; cb = 1; break;
                case 4: cr = f_; cg = 0; cb = 1; break;
                default: cr = 1; cg = 0; cb = 1 - f_; break;
            }
            colArr[i * 3] = cr;
            colArr[i * 3 + 1] = cg;
            colArr[i * 3 + 2] = cb;
        }
    } else if (colorMode === 'solid') {
        for (let i = 0; i < count; i++) {
            colArr[i * 3] = solidR;
            colArr[i * 3 + 1] = solidG;
            colArr[i * 3 + 2] = solidB;
        }
    }

    return { positions: posArr, colors: colArr, count };
}

/**
 * LaserScan 토픽 구독 및 THREE.Points로 시각화 (RViz LaserScanDisplay 스타일)
 * @param {string} topicName - sensor_msgs/msg/LaserScan 토픽명
 * @returns {ROSLIB.Topic} 구독 객체
 */
function subscribeToLaserScan(topicName) {
    if (!viewer3DState.rosConnected) {
        console.warn('[LaserScan] Not connected to ROS');
        return;
    }

    if (viewer3DState.topicSubscriptions.has(topicName)) {
        return viewer3DState.topicSubscriptions.get(topicName);
    }

    console.log('[LaserScan] Subscribing to topic:', topicName);

    const topic = new ROSLIB.Topic({
        ros: viewer3DState.ros,
        name: topicName,
        messageType: 'sensor_msgs/msg/LaserScan',
        throttle_rate: 100,   // 10Hz
        queue_length: 1
    });

    viewer3DState.topicSubscriptions.set(topicName, topic);

    topic.subscribe(function(message) {
        if (!message.ranges || message.ranges.length === 0) return;

        viewer3DState.laserScanLastMessage.set(topicName, message);

        const frameId = (message.header && message.header.frame_id) ? message.header.frame_id : '';
        viewer3DState.topicFrameIds.set(topicName, frameId);
        viewer3DState.needsRender = true;

        const existingObj = viewer3DState.laserScanObjects.get(topicName);
        if (existingObj && existingObj.visible === false) return;

        const settings = viewer3DState.topicSettings.get(topicName) || {};
        const pointSize = settings.pointSize !== undefined ? settings.pointSize : 0.1;
        const alpha = settings.alpha !== undefined ? settings.alpha : 1.0;

        const { positions, colors, count } = laserScanToPointsAndColors(message, settings);
        if (count === 0) return;

        const colorMode = settings.colorMode || 'rainbow';
        const useVertexColors = colorMode === 'rainbow';

        if (existingObj) {
            existingObj.points.geometry.dispose();
            existingObj.points.geometry = new THREE.BufferGeometry();
            existingObj.points.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
            if (useVertexColors) {
                existingObj.points.geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
            } else {
                existingObj.points.geometry.deleteAttribute('color');
            }
            existingObj.points.geometry.attributes.position.needsUpdate = true;
            existingObj.points.material.size = pointSize;
            existingObj.points.material.opacity = alpha;
            existingObj.points.material.transparent = alpha < 1.0;
            existingObj.points.material.depthWrite = alpha >= 1.0;
            existingObj.points.material.vertexColors = useVertexColors;
            if (useVertexColors) {
                existingObj.points.material.color.setHex(0xffffff);  // vertex color 그대로 표시 (곱셈 방지)
            } else {
                existingObj.points.material.color.setStyle(settings.solidColor || '#ffffff');
            }
            existingObj.points.material.needsUpdate = true;
            applyFrameTransformToObject(existingObj.frameGroup, frameId);
        } else {
            const geometry = new THREE.BufferGeometry();
            geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
            if (useVertexColors) {
                geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
            }
            const material = new THREE.PointsMaterial({
                size: pointSize,
                sizeAttenuation: true,
                vertexColors: useVertexColors,
                opacity: alpha,
                transparent: alpha < 1.0,
                depthWrite: alpha >= 1.0
            });
            if (useVertexColors) {
                material.color.setHex(0xffffff);  // vertex color 그대로 표시 (곱셈 방지)
            } else {
                material.color.setStyle(settings.solidColor || '#ffffff');
            }
            const points = new THREE.Points(geometry, material);
            const frameGroup = new THREE.Group();
            frameGroup.add(points);
            viewer3DState.scene.add(frameGroup);
            applyFrameTransformToObject(frameGroup, frameId);
            viewer3DState.laserScanObjects.set(topicName, { frameGroup, points, visible: true });
            console.log('[LaserScan] Points added for topic:', topicName);
        }
    });

    return topic;
}

// =============================================
// 토픽 조회 함수: Path / Odometry / LaserScan
// =============================================

/**
 * 사용 가능한 nav_msgs/Path 토픽 목록 조회
 * @returns {Promise<string[]>} Path 토픽 이름 배열
 */
async function getAvailablePathTopics() {
    if (!viewer3DState.rosConnected) {
        console.log('[Path] Waiting for ROS connection...');
        const connected = await waitForROSConnection(5000);
        if (!connected) {
            console.warn('[Path] Not connected to ROS after timeout');
            return [];
        }
    }

    return new Promise((resolve) => {
        viewer3DState.ros.getTopics(function(topics) {
            const pathTopics = [];
            if (topics.topics && topics.types) {
                topics.topics.forEach((topic, index) => {
                    const type = topics.types[index];
                    if (type === 'nav_msgs/Path' || type === 'nav_msgs/msg/Path') {
                        pathTopics.push(topic);
                    }
                });
            }
            console.log('[Path] Available Path topics:', pathTopics);
            resolve(pathTopics);
        }, function(error) {
            console.error('[Path] Failed to get topics:', error);
            resolve([]);
        });
    });
}

/**
 * 사용 가능한 nav_msgs/Odometry 토픽 목록 조회
 * @returns {Promise<string[]>} Odometry 토픽 이름 배열
 */
async function getAvailableOdometryTopics() {
    if (!viewer3DState.rosConnected) {
        console.log('[Odom] Waiting for ROS connection...');
        const connected = await waitForROSConnection(5000);
        if (!connected) {
            console.warn('[Odom] Not connected to ROS after timeout');
            return [];
        }
    }

    return new Promise((resolve) => {
        viewer3DState.ros.getTopics(function(topics) {
            const odomTopics = [];
            if (topics.topics && topics.types) {
                topics.topics.forEach((topic, index) => {
                    const type = topics.types[index];
                    if (type === 'nav_msgs/Odometry' || type === 'nav_msgs/msg/Odometry') {
                        odomTopics.push(topic);
                    }
                });
            }
            console.log('[Odom] Available Odometry topics:', odomTopics);
            resolve(odomTopics);
        }, function(error) {
            console.error('[Odom] Failed to get topics:', error);
            resolve([]);
        });
    });
}

/**
 * 사용 가능한 sensor_msgs/LaserScan 토픽 목록 조회
 * @returns {Promise<string[]>} LaserScan 토픽 이름 배열
 */
async function getAvailableLaserScanTopics() {
    if (!viewer3DState.rosConnected) {
        console.log('[LaserScan] Waiting for ROS connection...');
        const connected = await waitForROSConnection(5000);
        if (!connected) {
            console.warn('[LaserScan] Not connected to ROS after timeout');
            return [];
        }
    }

    return new Promise((resolve) => {
        viewer3DState.ros.getTopics(function(topics) {
            const lsTopics = [];
            if (topics.topics && topics.types) {
                topics.topics.forEach((topic, index) => {
                    const type = topics.types[index];
                    if (type === 'sensor_msgs/LaserScan' || type === 'sensor_msgs/msg/LaserScan') {
                        lsTopics.push(topic);
                    }
                });
            }
            console.log('[LaserScan] Available LaserScan topics:', lsTopics);
            resolve(lsTopics);
        }, function(error) {
            console.error('[LaserScan] Failed to get topics:', error);
            resolve([]);
        });
    });
}

// =============================================
// Path / Odometry / LaserScan 가시성 토글 및 색상 업데이트
// =============================================

/**
 * Path 라인 표시/숨기기
 * @param {string} topicName
 * @param {boolean} visible
 */
function togglePathVisible(topicName, visible) {
    const pathObj = viewer3DState.pathObjects.get(topicName);
    if (pathObj) {
        const rootObj = pathObj.frameGroup || pathObj.line;
        rootObj.visible = visible;
        pathObj.visible = visible;
        console.log(`[Path] ${topicName} visibility: ${visible}`);
    }
}

/**
 * Odometry 로봇 마커 그룹 표시/숨기기
 * @param {string} topicName
 * @param {boolean} visible
 */
function toggleOdomVisible(topicName, visible) {
    const odomObj = viewer3DState.odomObjects.get(topicName);
    if (odomObj) {
        odomObj.group.visible = visible;
        odomObj.visible = visible;
        console.log(`[Odom] ${topicName} visibility: ${visible}`);
    }
    // 궤적 axes parentGroup도 함께 show/hide
    const trajObj = viewer3DState.trajectoryObjects.get(topicName);
    if (trajObj) {
        trajObj.parentGroup.visible = visible;
        trajObj.visible = visible;
    }
}

/**
 * Path 라인 선 굵기 업데이트 및 설정 저장
 * @param {string} topicName
 * @param {number} width - 1~10
 */
function updatePathLineWidth(topicName, width) {
    const pathObj = viewer3DState.pathObjects.get(topicName);
    if (pathObj) {
        pathObj.line.material.linewidth = width;
        pathObj.line.material.needsUpdate = true;
    }
    updateTopicSetting(topicName, 'pathLineWidth', width);
}

/**
 * Path 라인 투명도(Alpha) 즉시 업데이트
 * @param {string} topicName
 * @param {number} alpha - 0.0(완전투명) ~ 1.0(불투명)
 */
function updatePathAlpha(topicName, alpha) {
    const pathObj = viewer3DState.pathObjects.get(topicName);
    if (pathObj) {
        pathObj.line.material.opacity     = alpha;
        pathObj.line.material.transparent = alpha < 1.0;
        pathObj.line.material.needsUpdate = true;
    }
    updateTopicSetting(topicName, 'pathAlpha', alpha);
}

/**
 * Odometry axes 크기(length, radius) 업데이트
 * 기존 axesGroup을 dispose하고 새로운 크기로 재생성
 * @param {string} topicName
 * @param {number} length
 * @param {number} radius
 */
function updateOdomAxes(topicName, length, radius) {
    const odomObj = viewer3DState.odomObjects.get(topicName);
    if (!odomObj) return;

    // 기존 axesGroup 제거 및 dispose
    if (odomObj.axesGroup) {
        odomObj.group.remove(odomObj.axesGroup);
        odomObj.axesGroup.traverse(function(child) {
            if (child.isMesh) {
                child.geometry.dispose();
                if (child.material) child.material.dispose();
            }
        });
    }

    // 새 axesGroup 생성 및 추가
    const newAxes = createOdomAxesGroup(length, radius);
    odomObj.group.add(newAxes);
    odomObj.axesGroup = newAxes;

    console.log(`[Odom] Axes updated for ${topicName}: length=${length}, radius=${radius}`);
}

/**
 * 기존 궤적 axes group 전체를 현재 설정(axesLength/axesRadius)으로 재생성
 * Axes Length 또는 Axes Radius 변경 시 호출하여 즉시 반영
 * @param {string} topicName
 */
function rebuildTrajectoryAxes(topicName) {
    const trajObj = viewer3DState.trajectoryObjects.get(topicName);
    if (!trajObj || trajObj.axesGroupList.length === 0) return;

    const settings   = viewer3DState.topicSettings.get(topicName) || {};
    const axesLength = settings.axesLength !== undefined ? settings.axesLength : 0.5;
    const axesRadius = settings.axesRadius !== undefined ? settings.axesRadius : 0.01;

    // 각 axes group의 position/quaternion 보존하며 재생성
    const rebuilt = trajObj.axesGroupList.map(oldGroup => {
        const pos  = oldGroup.position.clone();
        const quat = oldGroup.quaternion.clone();

        trajObj.parentGroup.remove(oldGroup);
        disposeAxesGroup(oldGroup);

        const newGroup = createOdomAxesGroup(axesLength, axesRadius);
        newGroup.position.copy(pos);
        newGroup.setRotationFromQuaternion(quat);
        trajObj.parentGroup.add(newGroup);
        return newGroup;
    });

    trajObj.axesGroupList = rebuilt;
    console.log(`[Trajectory] Axes rebuilt for ${topicName}: length=${axesLength}, radius=${axesRadius}, count=${rebuilt.length}`);
}

/**
 * Path 라인 색상 업데이트 및 설정 저장
 * @param {string} topicName
 * @param {string} colorHex - '#rrggbb' 형태
 */
function updatePathColor(topicName, colorHex) {
    const pathObj = viewer3DState.pathObjects.get(topicName);
    if (pathObj) {
        const colorInt = parseInt(colorHex.replace('#', ''), 16);
        pathObj.line.material.color.setHex(colorInt);
    }
    updateTopicSetting(topicName, 'pathColor', colorHex);
}

/**
 * LaserScan 포인트 표시/숨기기
 * @param {string} topicName
 * @param {boolean} visible
 */
function toggleLaserScanVisible(topicName, visible) {
    const lsObj = viewer3DState.laserScanObjects.get(topicName);
    if (lsObj) {
        const rootObj = lsObj.frameGroup || lsObj.points;
        rootObj.visible = visible;
        lsObj.visible = visible;
        console.log(`[LaserScan] ${topicName} visibility: ${visible}`);
    }
}

/**
 * LaserScan 포인트 색상 업데이트 (Solid 모드용)
 * @param {string} topicName
 * @param {string} colorHex - '#rrggbb' 형태
 */
function updateLaserScanColor(topicName, colorHex) {
    const lsObj = viewer3DState.laserScanObjects.get(topicName);
    if (lsObj && !lsObj.points.material.vertexColors) {
        lsObj.points.material.color.setStyle(colorHex);
    }
    updateTopicSetting(topicName, 'solidColor', colorHex);
}

/**
 * LaserScan 포인트 크기 업데이트 (PointCloud2와 동일: pointSize)
 * @param {string} topicName
 * @param {number} size - 포인트 크기(m)
 */
function updateLaserScanPointSize(topicName, size) {
    const lsObj = viewer3DState.laserScanObjects.get(topicName);
    if (lsObj) {
        lsObj.points.material.size = size;
    }
    updateTopicSetting(topicName, 'pointSize', size);
}

/**
 * LaserScan Alpha(투명도) 업데이트
 * @param {string} topicName
 * @param {number} alpha - 0.0~1.0
 */
function updateLaserScanAlpha(topicName, alpha) {
    const lsObj = viewer3DState.laserScanObjects.get(topicName);
    if (lsObj) {
        lsObj.points.material.opacity = alpha;
        lsObj.points.material.transparent = alpha < 1.0;
    }
    updateTopicSetting(topicName, 'alpha', alpha);
}

// updateTrajectoryColor 제거: axes 방식에서는 X/Y/Z 고정 색상 사용 (RViz 스타일)

// =============================================
// PointCloud2 런타임 속성 업데이트
// =============================================

/**
 * PointCloud2 렌더링 스타일 변경 (Points / Squares / Flat Squares / Spheres / Boxes / Tiles)
 * 스타일 전환 시 현재 mesh를 제거하고 다음 데이터 프레임에서 재생성.
 * @param {string} topicName
 * @param {string} style - 'points' | 'squares' | 'flat_squares' | 'spheres' | 'boxes' | 'tiles'
 */
function updatePointStyle(topicName, style) {
    updateTopicSetting(topicName, 'style', style);

    // 현재 mesh 제거 → 다음 _uploadPC2ToGPU 호출 시 새 스타일로 재생성
    const pcFG = viewer3DState.pcFrameGroups.get(topicName);
    const mesh = viewer3DState.pointCloudMeshes[topicName];
    if (mesh && pcFG) {
        pcFG.remove(mesh);
        mesh.geometry.dispose();
        mesh.material.dispose();
        viewer3DState.pointCloudMeshes[topicName] = null;
    }
    // decay 누적 mesh도 초기화
    clearDecayObjects(topicName);

    viewer3DState.needsRender = true;
    console.log(`[PC2 Style] ${topicName} → ${style}`);
}

/**
 * PointCloud2 포인트 사이즈 즉시 업데이트
 * @param {string} topicName
 * @param {number} size - 포인트 크기(m)
 */
function updatePointSize(topicName, size) {
    updateTopicSetting(topicName, 'pointSize', size);
    // TopDown 뷰: frustum 기반 픽셀 스케일 적용
    // Orbit: 월드단위 그대로 사용 (sizeAttenuation:true → 원근감 자동 처리)
    const isTopdown = viewer3DState.currentViewType === 'topdown';
    const renderSize = isTopdown ? Math.max(1, size * _getTopdownPixelsPerUnit()) : size;

    const mesh = viewer3DState.pointCloudMeshes[topicName];
    if (mesh) {
        if (mesh._isBoxMesh) {
            // Boxes: 지오메트리 재생성이 필요하므로 다음 프레임에서 자동 처리
            // (updatePointStyle 호출 없이 다음 _uploadPC2ToGPU에서 새 BoxGeometry 사용)
        } else {
            mesh.material.size = renderSize;
        }
    }
    // decay 모드의 누적 mesh들도 즉시 반영
    const arr = viewer3DState.decayObjects.get(topicName);
    if (arr) arr.forEach(item => {
        if (item.points && item.points.material && !item.points._isBoxMesh) {
            item.points.material.size = renderSize;
        }
    });
}

/**
 * PointCloud2 투명도(Alpha) 즉시 업데이트
 * @param {string} topicName
 * @param {number} alpha - 0.0(완전투명) ~ 1.0(불투명)
 */
function updatePointAlpha(topicName, alpha) {
    updateTopicSetting(topicName, 'alpha', alpha);
    const mesh = viewer3DState.pointCloudMeshes[topicName];
    if (mesh) {
        const hasMap = !!(mesh.material.map);
        mesh.material.opacity     = alpha;
        mesh.material.transparent = alpha < 1.0;
        mesh.material.depthWrite  = alpha >= 1.0;
        mesh.material.alphaTest   = hasMap ? 0.01 : 0;  // 스타일 전환 후에도 올바른 값 유지
        mesh.material.needsUpdate = true;
    }
    const arr = viewer3DState.decayObjects.get(topicName);
    if (arr) arr.forEach(item => {
        if (item.points && item.points.material) {
            const hasMap = !!(item.points.material.map);
            item.points.material.opacity     = alpha;
            item.points.material.transparent = alpha < 1.0;
            item.points.material.depthWrite  = alpha >= 1.0;
            item.points.material.alphaTest   = hasMap ? 0.01 : 0;
            item.points.material.needsUpdate = true;
        }
    });
}

/**
 * Decay Time 변경 시 해당 토픽의 누적 decay mesh 전체 제거
 * decay 포인트들은 pcFrameGroup에 추가되므로 pcFrameGroup에서 제거해야 한다.
 * @param {string} topicName
 */
function clearDecayObjects(topicName) {
    const arr = viewer3DState.decayObjects.get(topicName);
    if (!arr || arr.length === 0) return;

    // decay 오브젝트는 pcFrameGroup 에 추가된 것 → 그곳에서 제거
    const pcFG = viewer3DState.pcFrameGroups.get(topicName);
    arr.forEach(item => {
        if (pcFG) pcFG.remove(item.points);
        else      viewer3DState.scene.remove(item.points); // fallback
        item.points.geometry.dispose();
        item.points.material.dispose();
    });
    viewer3DState.decayObjects.set(topicName, []);
}

/**
 * _pruneExpiredDecayObjects(topicName, newDecaySec)
 * decay time이 줄어들 때, 새 decay time 기준보다 오래된 항목을 즉시 제거
 * @param {string} topicName
 * @param {number} newDecaySec - 새 decay time (초)
 */
function _pruneExpiredDecayObjects(topicName, newDecaySec) {
    const arr = viewer3DState.decayObjects.get(topicName);
    if (!arr || arr.length === 0) return;

    const pcFG  = viewer3DState.pcFrameGroups.get(topicName);
    const now   = performance.now();
    const limitMs = newDecaySec * 1000;

    for (let i = arr.length - 1; i >= 0; i--) {
        if (now - arr[i].time > limitMs) {
            if (pcFG) pcFG.remove(arr[i].points);
            else      viewer3DState.scene.remove(arr[i].points);
            arr[i].points.geometry.dispose();
            arr[i].points.material.dispose();
            arr.splice(i, 1);
        }
    }
    viewer3DState.decayObjects.set(topicName, arr);
}

// =============================================
// Path 토픽 선택 UI (selectPathTopics)
// =============================================

/**
 * Path 토픽 선택 다이얼로그 열기
 */
async function selectPathTopics() {
    if (!viewer3DState.rosConnected) {
        alert('Not connected to ROS. Make sure rosbridge_server is running:\n\nros2 launch rosbridge_server rosbridge_websocket_launch.xml');
        return;
    }

    const btn = document.getElementById('viewer-add-path-btn');
    const originalText = btn ? btn.textContent : '';
    if (btn) { btn.textContent = 'Loading...'; btn.disabled = true; }

    let topics = [];
    try {
        topics = await getAvailablePathTopics();
    } finally {
        if (btn) { btn.textContent = originalText; btn.disabled = false; }
    }

    if (topics.length === 0) {
        alert('No nav_msgs/Path topics available.\nMake sure path topics are being published.');
        return;
    }

    const modal     = document.getElementById('path-topic-modal');
    const topicList = document.getElementById('path-topic-list');
    if (!modal || !topicList) {
        console.error('[Path] Topic modal element not found in HTML');
        return;
    }

    topicList.innerHTML = '';
    topics.forEach(topic => {
        const div = document.createElement('div');
        div.className = 'topic-item';

        const checkbox = document.createElement('input');
        checkbox.type    = 'checkbox';
        checkbox.id      = `path-topic-cb-${topic}`;
        checkbox.value   = topic;
        checkbox.checked = viewer3DState.selectedPathTopics.includes(topic);

        const label = document.createElement('label');
        label.htmlFor     = `path-topic-cb-${topic}`;
        label.textContent = topic;

        div.appendChild(checkbox);
        div.appendChild(label);
        topicList.appendChild(div);
    });

    modal.style.display = 'block';
}

/**
 * Path 토픽 선택 모달 닫기
 */
function closePathTopicSelection() {
    const modal = document.getElementById('path-topic-modal');
    if (modal) modal.style.display = 'none';
}

/**
 * Path 토픽 선택 확인: 체크된 토픽 구독, 해제된 토픽 정리
 */
function confirmPathTopicSelection() {
    const checkboxes = document.querySelectorAll('#path-topic-list input[type="checkbox"]');
    const newTopics  = [];
    checkboxes.forEach(cb => { if (cb.checked) newTopics.push(cb.value); });

    // 제거된 토픽: 구독 해제 및 Three.js 리소스 정리
    viewer3DState.selectedPathTopics.forEach(topic => {
        if (!newTopics.includes(topic)) {
            unsubscribeFromTopic(topic);
        }
    });

    // 신규 토픽: 구독 시작
    newTopics.forEach(topic => {
        if (!viewer3DState.selectedPathTopics.includes(topic)) {
            subscribeToPath(topic);
        }
    });

    viewer3DState.selectedPathTopics = newTopics;
    closePathTopicSelection();
    renderDisplayPanel();
    console.log('[Path] Selected topics:', viewer3DState.selectedPathTopics);
}

// =============================================
// Odometry 토픽 선택 UI (selectOdometryTopics)
// =============================================

/**
 * Odometry 토픽 선택 다이얼로그 열기
 */
async function selectOdometryTopics() {
    if (!viewer3DState.rosConnected) {
        alert('Not connected to ROS. Make sure rosbridge_server is running:\n\nros2 launch rosbridge_server rosbridge_websocket_launch.xml');
        return;
    }

    const btn = document.getElementById('viewer-add-odom-btn');
    const originalText = btn ? btn.textContent : '';
    if (btn) { btn.textContent = 'Loading...'; btn.disabled = true; }

    let topics = [];
    try {
        topics = await getAvailableOdometryTopics();
    } finally {
        if (btn) { btn.textContent = originalText; btn.disabled = false; }
    }

    if (topics.length === 0) {
        alert('No nav_msgs/Odometry topics available.\nMake sure odometry topics are being published.');
        return;
    }

    const modal     = document.getElementById('odom-topic-modal');
    const topicList = document.getElementById('odom-topic-list');
    if (!modal || !topicList) {
        console.error('[Odom] Topic modal element not found in HTML');
        return;
    }

    topicList.innerHTML = '';
    topics.forEach(topic => {
        const div = document.createElement('div');
        div.className = 'topic-item';

        const checkbox = document.createElement('input');
        checkbox.type    = 'checkbox';
        checkbox.id      = `odom-topic-cb-${topic}`;
        checkbox.value   = topic;
        checkbox.checked = viewer3DState.selectedOdomTopics.includes(topic);

        const label = document.createElement('label');
        label.htmlFor     = `odom-topic-cb-${topic}`;
        label.textContent = topic;

        div.appendChild(checkbox);
        div.appendChild(label);
        topicList.appendChild(div);
    });

    modal.style.display = 'block';
}

/**
 * Odometry 토픽 선택 모달 닫기
 */
function closeOdometryTopicSelection() {
    const modal = document.getElementById('odom-topic-modal');
    if (modal) modal.style.display = 'none';
}

/**
 * Odometry 토픽 선택 확인: 체크된 토픽 구독, 해제된 토픽 정리
 */
function confirmOdometryTopicSelection() {
    const checkboxes = document.querySelectorAll('#odom-topic-list input[type="checkbox"]');
    const newTopics  = [];
    checkboxes.forEach(cb => { if (cb.checked) newTopics.push(cb.value); });

    // 제거된 토픽: 구독 해제 및 Three.js 리소스 정리
    viewer3DState.selectedOdomTopics.forEach(topic => {
        if (!newTopics.includes(topic)) {
            unsubscribeFromTopic(topic);
        }
    });

    // 신규 토픽: 구독 시작
    newTopics.forEach(topic => {
        if (!viewer3DState.selectedOdomTopics.includes(topic)) {
            subscribeToOdometry(topic);
        }
    });

    viewer3DState.selectedOdomTopics = newTopics;
    closeOdometryTopicSelection();
    renderDisplayPanel();
    console.log('[Odom] Selected topics:', viewer3DState.selectedOdomTopics);
}

// =============================================
// TFRenderer: tf2_msgs/TFMessage → Three.js 시각화
// =============================================

/**
 * Canvas 기반 텍스트 Sprite 생성 (frame 이름 라벨)
 * @param {string} text - 표시할 텍스트
 * @param {Object} options - {fontSize, textColor, bgColor, labelWidth, labelHeight}
 * @returns {THREE.Sprite}
 */
function createTextSprite(text, options = {}) {
    const fontSize    = options.fontSize    || 14;
    const textColor   = options.textColor   || 'rgba(255,255,255,1)';
    const bgColor     = options.bgColor;          // undefined = 배경 없음
    const labelWidth  = options.labelWidth  !== undefined ? options.labelWidth  : 2.0;
    const labelHeight = options.labelHeight !== undefined ? options.labelHeight : 0.5;

    const canvas = document.createElement('canvas');
    canvas.width  = 256;
    canvas.height = 64;
    const ctx = canvas.getContext('2d');

    // 배경 (bgColor 지정 시에만 그림)
    if (bgColor && bgColor !== 'transparent' && bgColor !== 'rgba(0,0,0,0)') {
        ctx.fillStyle = bgColor;
        ctx.beginPath();
        ctx.roundRect(4, 4, canvas.width - 8, canvas.height - 8, 6);
        ctx.fill();
    }

    // 텍스트 (그림자로 가독성 확보)
    ctx.font         = `bold ${fontSize}px Arial`;
    ctx.textAlign    = 'center';
    ctx.textBaseline = 'middle';
    ctx.shadowColor  = 'rgba(0,0,0,0.9)';
    ctx.shadowBlur   = 5;
    ctx.shadowOffsetX = 1;
    ctx.shadowOffsetY = 1;
    ctx.fillStyle    = textColor;
    ctx.fillText(text, canvas.width / 2, canvas.height / 2);

    const texture  = new THREE.CanvasTexture(canvas);
    const material = new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false });
    const sprite   = new THREE.Sprite(material);
    sprite.scale.set(labelWidth, labelHeight, 1);

    return sprite;
}

/**
 * RViz/tf2 Buffer 와 유사: 동적 /tf 는 스트림 상대 시각으로 오래된 변환 제거,
 * ROS 시각이 크게 뒤로 가면(다른 백·재시작) 트리를 비우고 다시 쌓음.
 * /tf_static 은 만료하지 않음(정적 변환).
 */
const TF_DYNAMIC_TIMEOUT_NS = 10e9;
const TF_BACK_JUMP_NS = 1e9;
let _tfLatestRosTimeNs = 0;
let _lastKnownFramesSig = '';

function _rosStampToNs(stamp) {
    if (!stamp) return 0;
    const sec = stamp.sec !== undefined ? stamp.sec : 0;
    const nsec = stamp.nanosec !== undefined ? stamp.nanosec : (stamp.nsec !== undefined ? stamp.nsec : 0);
    return sec * 1e9 + nsec;
}

function _maxStampNsFromTransforms(transforms) {
    let m = 0;
    if (!transforms || transforms.length === 0) return 0;
    transforms.forEach(function(ts) {
        const ns = _rosStampToNs(ts.header && ts.header.stamp);
        if (ns > m) m = ns;
    });
    return m;
}

/** rosbridge 토픽명: /tf_static, /ns1/tf_static 등 */
function _tfSourceTopicIsStatic(topicName) {
    return typeof topicName === 'string' &&
        (topicName === '/tf_static' || /(^|\/)tf_static$/.test(topicName));
}

function _rebuildKnownFramesFromTfTree() {
    _allKnownFrames.clear();
    viewer3DState.tfFrameTree.forEach(function(entry, childId) {
        _allKnownFrames.add(childId);
        if (entry.parentId && entry.parentId !== '') {
            _allKnownFrames.add(entry.parentId);
        }
    });
}

/**
 * TF 메시지 파싱: viewer3DState.tfFrameTree 에 병합
 * @param {Object} message - tf2_msgs/msg/TFMessage
 * @param {boolean} isStatic - /tf_static 여부
 */
function parseTFMessage(message, isStatic) {
    if (!message.transforms || message.transforms.length === 0) return;

    message.transforms.forEach(function(transformStamped) {
        const childId  = transformStamped.child_frame_id;
        const parentId = transformStamped.header.frame_id;
        const t        = transformStamped.transform.translation;
        const r        = transformStamped.transform.rotation;

        viewer3DState.tfFrameTree.set(childId, {
            parentId:    parentId,
            translation: { x: t.x, y: t.y, z: t.z },
            quaternion:  { x: r.x, y: r.y, z: r.z, w: r.w },
            stamp:       transformStamped.header.stamp,
            isStatic:    !!isStatic
        });
    });
}

/**
 * @param {Object} message
 * @param {string} sourceTopicName - 콜백이 붙은 TF 토픽 이름
 * @returns {{ frameChanged: boolean }}
 */
function applyTfMessageWithRvStyleLifecycle(message, sourceTopicName) {
    if (!message.transforms || message.transforms.length === 0) {
        return { frameChanged: false };
    }

    const isStatic = _tfSourceTopicIsStatic(sourceTopicName);
    const maxNs    = _maxStampNsFromTransforms(message.transforms);

    if (!isStatic && maxNs > 0 && _tfLatestRosTimeNs > 0 && maxNs < _tfLatestRosTimeNs - TF_BACK_JUMP_NS) {
        viewer3DState.tfFrameTree.clear();
        _tfLatestRosTimeNs = 0;
    }

    parseTFMessage(message, isStatic);

    if (!isStatic && maxNs > 0) {
        if (maxNs > _tfLatestRosTimeNs) {
            _tfLatestRosTimeNs = maxNs;
        }
        const cutoff = _tfLatestRosTimeNs - TF_DYNAMIC_TIMEOUT_NS;
        Array.from(viewer3DState.tfFrameTree.entries()).forEach(function(pair) {
            const childId = pair[0];
            const entry   = pair[1];
            if (entry.isStatic) return;
            const en = _rosStampToNs(entry.stamp);
            if (en < cutoff) {
                viewer3DState.tfFrameTree.delete(childId);
            }
        });
    }

    _rebuildKnownFramesFromTfTree();
    const sig = Array.from(_allKnownFrames).sort().join('|');
    const frameChanged = sig !== _lastKnownFramesSig;
    _lastKnownFramesSig = sig;
    return { frameChanged: frameChanged };
}

/**
 * frame을 루트까지 역추적하여 경로(frames 배열)와 각 구간 변환(transforms)을 반환.
 * transforms[i] = frames[i](child)가 frames[i+1](parent) 기준에서 가진 변환.
 * @param {string} startFrame
 * @returns {{ frames: string[], transforms: {translation, quaternion}[] }}
 */
function _pathToRoot(startFrame) {
    const frames     = [startFrame];
    const transforms = [];
    let current      = startFrame;
    const visited    = new Set([startFrame]);

    while (viewer3DState.tfFrameTree.has(current)) {
        const entry    = viewer3DState.tfFrameTree.get(current);
        const parentId = entry.parentId;
        if (!parentId || parentId === '' || visited.has(parentId)) break;
        transforms.push({ translation: entry.translation, quaternion: entry.quaternion });
        frames.push(parentId);
        visited.add(parentId);
        current = parentId;
    }
    return { frames, transforms };
}

/**
 * chain(child→parent 방향 transforms 배열)을 역순으로 누적 적용하여
 * 시작 frame 원점의 위치/방향을 parent 기준 좌표계로 산출.
 * chainLen 개 변환만 사용(0이면 identity).
 */
function _accumulateTransform(transforms, chainLen) {
    let pos  = new THREE.Vector3(0, 0, 0);
    let quat = new THREE.Quaternion(0, 0, 0, 1);
    for (let i = chainLen - 1; i >= 0; i--) {
        const t  = transforms[i];
        const lp = new THREE.Vector3(t.translation.x, t.translation.y, t.translation.z);
        const lq = new THREE.Quaternion(t.quaternion.x, t.quaternion.y, t.quaternion.z, t.quaternion.w);
        lp.applyQuaternion(quat);
        pos.add(lp);
        quat.multiply(lq);
    }
    return { pos, quat };
}

/**
 * frameId → fixedFrame 기준의 world 변환 계산 (위치 + 회전).
 * RViz tf2 방식의 LCA(최소 공통 조상) 알고리즘 적용 —
 * fixedFrame이 frameId의 조상/자손/형제 등 어떤 관계여도 정확하게 처리.
 *
 * @param {string} frameId    - 대상 frame ID
 * @param {string} fixedFrame - 기준 frame ID
 * @returns {{position: THREE.Vector3, quaternion: THREE.Quaternion}|null} 변환 불가 시 null
 */
function computeWorldTransform(frameId, fixedFrame) {
    if (frameId === fixedFrame) {
        return {
            position:   new THREE.Vector3(0, 0, 0),
            quaternion: new THREE.Quaternion(0, 0, 0, 1)
        };
    }

    // frameId, fixedFrame 각각 루트까지 경로 수집
    const fromPath = _pathToRoot(frameId);
    const toPath   = _pathToRoot(fixedFrame);

    // LCA 탐색: fromPath.frames 중 toPath.frames에 있는 첫 번째 공통 frame
    const toFrameIndex = new Map();
    toPath.frames.forEach(function(f, i) { toFrameIndex.set(f, i); });

    let fromToLca = -1;   // fromPath.frames 에서 LCA 인덱스
    let toToLca   = -1;   // toPath.frames   에서 LCA 인덱스
    for (let i = 0; i < fromPath.frames.length; i++) {
        if (toFrameIndex.has(fromPath.frames[i])) {
            fromToLca = i;
            toToLca   = toFrameIndex.get(fromPath.frames[i]);
            break;
        }
    }
    if (fromToLca < 0) return null;  // 공통 조상 없음 → 변환 불가

    // T_LCA_frameId  : frameId 원점의 LCA 기준 위치/방향
    const A = _accumulateTransform(fromPath.transforms, fromToLca);

    // T_LCA_fixedFrame : fixedFrame 원점의 LCA 기준 위치/방향
    const B = _accumulateTransform(toPath.transforms,   toToLca);

    // T_fixedFrame_frameId = inv(T_LCA_fixedFrame) ∘ T_LCA_frameId
    //   invQuat = B.quat.conjugate()
    //   resultPos  = invQuat * (A.pos - B.pos)
    //   resultQuat = invQuat * A.quat
    const invQuat    = B.quat.clone().invert();
    const resultPos  = A.pos.clone().sub(B.pos).applyQuaternion(invQuat);
    const resultQuat = invQuat.clone().multiply(A.quat);

    return { position: resultPos, quaternion: resultQuat };
}

/**
 * Three.js 오브젝트에 frameId → fixedFrame 좌표 변환을 적용.
 * TF 경로가 없으면 원점에 두고 표시 유지(부분 TF·데이터 전환 직후에도 점군 등이 사라지지 않게).
 * @param {THREE.Object3D} object - position/quaternion 변경 대상
 * @param {string} frameId        - 오브젝트가 표현된 ROS frame ID
 * @returns {boolean} 변환 적용(표시) 성공 여부
 */
function applyFrameTransformToObject(object, frameId) {
    const fixedFrame = viewer3DState.fixedFrame;
    if (!frameId || frameId === fixedFrame) {
        object.position.set(0, 0, 0);
        object.quaternion.set(0, 0, 0, 1);
        object.visible = true;
        return true;
    }
    const transform = computeWorldTransform(frameId, fixedFrame);
    if (!transform) {
        // 경로 없을 때는 숨기지 않고 원점에 표시 (부분 TF·전환 직후에도 점군이 사라지지 않게)
        object.position.set(0, 0, 0);
        object.quaternion.set(0, 0, 0, 1);
        object.visible = true;
        return false;
    }
    object.position.copy(transform.position);
    object.quaternion.copy(transform.quaternion);
    object.visible = true;
    return true;
}

/**
 * fixedFrame 변경 또는 TF 업데이트 시 PointCloud2 / Path / Odometry / Trajectory
 * 시각화 오브젝트에 새 frame transform 재적용.
 */
function reapplyAllFrameTransforms() {
    // ── PointCloud2 frame groups ──
    viewer3DState.pcFrameGroups.forEach(function(fg, topicName) {
        const frameId = viewer3DState.topicFrameIds.get(topicName);
        if (frameId) applyFrameTransformToObject(fg, frameId);
    });

    // ── Path frame groups ──
    viewer3DState.pathObjects.forEach(function(pathObj, topicName) {
        if (pathObj.frameGroup) {
            const frameId = viewer3DState.topicFrameIds.get(topicName);
            if (frameId) applyFrameTransformToObject(pathObj.frameGroup, frameId);
        }
    });

    // ── Odometry: lastPose 재계산 ──
    viewer3DState.odomObjects.forEach(function(odomObj, topicName) {
        if (!odomObj.lastPose) return;
        const { x, y, z, qx, qy, qz, qw, frameId } = odomObj.lastPose;
        const worldPose = computeOdomWorldPose(x, y, z, qx, qy, qz, qw, frameId);
        if (!worldPose) {
            odomObj.group.visible = false;
        } else {
            odomObj.group.position.copy(worldPose.position);
            odomObj.group.setRotationFromQuaternion(worldPose.quaternion);
            if (odomObj.visible !== false) odomObj.group.visible = true;
        }
    });

    // ── Trajectory frame groups ──
    viewer3DState.trajectoryObjects.forEach(function(trajObj, topicName) {
        if (trajObj.frameGroup) {
            const frameId = viewer3DState.topicFrameIds.get(topicName + '_traj');
            if (frameId) applyFrameTransformToObject(trajObj.frameGroup, frameId);
        }
    });
}

/**
 * Odometry 메시지의 (x,y,z,quat) in frameId를 fixedFrame 기준 world pose로 변환.
 * @returns {{position: THREE.Vector3, quaternion: THREE.Quaternion}|null}
 */
function computeOdomWorldPose(x, y, z, qx, qy, qz, qw, frameId) {
    const fixedFrame = viewer3DState.fixedFrame;
    if (!frameId || frameId === fixedFrame) {
        return {
            position:   new THREE.Vector3(x, y, z),
            quaternion: new THREE.Quaternion(qx, qy, qz, qw)
        };
    }
    const T = computeWorldTransform(frameId, fixedFrame);
    if (!T) return null;

    const localPos = new THREE.Vector3(x, y, z);
    localPos.applyQuaternion(T.quaternion);
    const worldPos  = T.position.clone().add(localPos);
    const worldQuat = T.quaternion.clone().multiply(new THREE.Quaternion(qx, qy, qz, qw));
    return { position: worldPos, quaternion: worldQuat };
}

/**
 * TF 씬 재빌드: tfFrameTree의 모든 frame(child + root)에 대해 world 변환 계산 후
 * createOdomAxesGroup(화살표), THREE.Line(연결선), THREE.Sprite(라벨)로 구성
 * @param {string} topicName - TF 토픽명 (tfObjects Map의 키)
 */
function rebuildTFScene(topicName) {
    const fixedFrame = viewer3DState.fixedFrame;

    // 기존 group 완전 정리 (GPU 리소스 포함)
    const prevObj = viewer3DState.tfObjects.get(topicName);
    if (prevObj) {
        viewer3DState.scene.remove(prevObj.group);
        prevObj.group.traverse(function(child) {
            if (child.isLine || child.isSprite || child.isMesh) {
                if (child.geometry) child.geometry.dispose();
                if (child.material) {
                    if (child.material.map) child.material.map.dispose();
                    child.material.dispose();
                }
            }
        });
    }

    // 설정 읽기
    const group      = new THREE.Group();
    const visible    = prevObj ? prevObj.visible : true;
    const settings   = viewer3DState.topicSettings.get(topicName) || {};
    const axesSize   = settings.tfAxesSize   !== undefined ? settings.tfAxesSize   : 0.3;
    const axesRadius = settings.tfAxesRadius !== undefined ? settings.tfAxesRadius : 0.01;
    const textSize   = settings.tfTextSize   !== undefined ? settings.tfTextSize   : 0.2;

    // ── 렌더링 대상: child 프레임 + parent(루트 포함) 모든 프레임 수집 ──
    const allFrames = new Set();
    viewer3DState.tfFrameTree.forEach(function(entry, childId) {
        allFrames.add(childId);
        if (entry.parentId && entry.parentId !== '') allFrames.add(entry.parentId);
    });

    allFrames.forEach(function(frameId) {
        const worldTransform = computeWorldTransform(frameId, fixedFrame);
        if (!worldTransform) return;  // fixedFrame 까지 경로 없음 → 스킵

        // 이 frame이 child인 경우 parent 정보 조회 (루트 frame은 entry 없음)
        const entry = viewer3DState.tfFrameTree.get(frameId);
        const parentTransform = (entry && entry.parentId && entry.parentId !== '')
            ? computeWorldTransform(entry.parentId, fixedFrame)
            : null;

        // 1. 화살표 Axes (X=빨강, Y=초록, Z=파랑) — Odometry 동일 스타일 cylinder+cone
        const axesGroup = createOdomAxesGroup(axesSize, axesRadius);
        axesGroup.position.copy(worldTransform.position);
        axesGroup.setRotationFromQuaternion(worldTransform.quaternion);
        group.add(axesGroup);

        // 2. 연결선: parent origin → child origin (노란색)
        if (parentTransform) {
            const linePoints = [
                parentTransform.position.clone(),
                worldTransform.position.clone()
            ];
            const lineGeo = new THREE.BufferGeometry().setFromPoints(linePoints);
            const lineMat = new THREE.LineBasicMaterial({ color: 0xffff00, linewidth: 1 });
            const line    = new THREE.Line(lineGeo, lineMat);
            group.add(line);
        }

        // 3. frame 이름 라벨 (Sprite) — textSize로 독립 제어, 로컬 Z 방향 오프셋
        const labelH       = Math.max(0.04, textSize);
        const labelW       = labelH * 5;
        const label        = createTextSprite(frameId, { fontSize: 13, labelWidth: labelW, labelHeight: labelH });
        const localZOffset = new THREE.Vector3(0, 0, axesSize + labelH * 0.6);
        localZOffset.applyQuaternion(worldTransform.quaternion);
        label.position.copy(worldTransform.position).add(localZOffset);
        group.add(label);
    });

    group.visible = visible;
    viewer3DState.scene.add(group);
    viewer3DState.tfObjects.set(topicName, { group, visible });
    console.log('[TF] Scene rebuilt for topic:', topicName, '| frames:', viewer3DState.tfFrameTree.size);

    // Fixed Frame datalist 업데이트 (알려진 프레임 ID 목록)
    updateFixedFrameOptions();

    // TF 업데이트 → PointCloud2 / Path / Odometry frame transform 재적용
    reapplyAllFrameTransforms();
}

/**
 * 현재 알려진 모든 TF 프레임 ID 수집
 * - tfFrameTree: TF 구독 시 파싱된 child/parent 프레임
 * - topicFrameIds: PointCloud2/Path/Odometry 메시지 헤더에서 수집된 frame_id
 */
// ── 백그라운드 TF frame 수집 (토픽 구독 없이도 프레임 목록 제공) ──
// 시각화 구독(topicSubscriptions)과 완전히 분리된 별도 구독 Map
const _bgFrameSubs = new Map();           // topicName → ROSLIB.Topic
const _allKnownFrames = new Set();        // 수집된 모든 frame ID

/** Fixed Frame 기본값 — 데이터 로드·전환 시 입력·상태를 이 값으로 맞춤 */
const DEFAULT_FIXED_FRAME = 'map';

/**
 * 데이터셋/백 전환 시마다 증가. 구독 콜백이 캡처한 세션과 다르면 TF 파싱 스킵(이전 백 큐 잔여·섞임 방지).
 */
let _tfSessionId = 0;

function _bumpTfSessionId() {
    _tfSessionId++;
}

/** 백그라운드 TF 콜백에서 공유하는 reapply 스로틀 */
let _bgTfReapplyTimer = null;
let _discoverExtraTfTimerId = null;

function _clearBgTfReapplyTimer() {
    if (_bgTfReapplyTimer !== null) {
        clearTimeout(_bgTfReapplyTimer);
        _bgTfReapplyTimer = null;
    }
}

/** 모든 백그라운드 TF 구독 해제 (데이터셋·백 전환 시 이전 TF 재유입 방지) */
function _unsubscribeAllBackgroundTf() {
    _clearBgTfReapplyTimer();
    _bgFrameSubs.forEach(function(topic) {
        if (topic) {
            try {
                topic.unsubscribe();
            } catch (e) { /* ignore */ }
        }
    });
    _bgFrameSubs.clear();
    if (_discoverExtraTfTimerId !== null) {
        clearTimeout(_discoverExtraTfTimerId);
        _discoverExtraTfTimerId = null;
    }
}

function _handleBackgroundTfMessage(message, sourceTopicName) {
    if (!message.transforms) return;

    const { frameChanged } = applyTfMessageWithRvStyleLifecycle(message, sourceTopicName);

    if (_bgTfReapplyTimer === null) {
        _bgTfReapplyTimer = setTimeout(function() {
            _bgTfReapplyTimer = null;
            reapplyAllFrameTransforms();
        }, 100);
    }

    if (frameChanged) {
        const dd = document.getElementById('fixed-frame-dropdown');
        if (dd && dd.style.display !== 'none') {
            if (_ffShowAll) {
                renderFixedFrameDropdown('');
            } else {
                const inp = document.getElementById('fixed-frame-input');
                renderFixedFrameDropdown(inp ? inp.value : '');
            }
        }
        updateViewTargetFrameOptions();
    }
}

/**
 * @param {{ name: string, throttle?: number, qos?: object, queue_length?: number }} config
 */
function _subscribeOneBackgroundTfTopic(config) {
    if (_bgFrameSubs.has(config.name)) return;
    if (!viewer3DState.ros || !viewer3DState.rosConnected) return;
    try {
        const topicOpts = {
            ros:           viewer3DState.ros,
            name:          config.name,
            messageType:   'tf2_msgs/msg/TFMessage',
            throttle_rate: config.throttle !== undefined ? config.throttle : 100,
            queue_length:  config.queue_length !== undefined ? config.queue_length : 3
        };
        if (config.qos) topicOpts.qos = config.qos;
        const topic = new ROSLIB.Topic(topicOpts);
        const tfSession = _tfSessionId;
        const srcName   = config.name;
        topic.subscribe(function(message) {
            if (tfSession !== _tfSessionId) return;
            _handleBackgroundTfMessage(message, srcName);
        });
        _bgFrameSubs.set(config.name, topic);
        console.log('[BG-TF] Background TF transform collection started for', config.name);
    } catch (e) {
        console.warn('[BG-TF] Failed to start background TF collection for', config.name, e);
    }
}

const _BG_CORE_TF_TOPICS = [
    // queue_length 작게: 전환 직후 rosbridge 큐에 쌓인 구 TF 버스트 유입 완화
    { name: '/tf',        throttle: 100, qos: null, queue_length: 3 },
    { name: '/tf_static', throttle: 0,   qos: { durability: 'transient_local' }, queue_length: 3 },
];

function _subscribeCoreBackgroundTfTopics() {
    _BG_CORE_TF_TOPICS.forEach(_subscribeOneBackgroundTfTopic);
}

function _discoverExtraTfTopicsImpl() {
    if (!viewer3DState.rosConnected) return;
    viewer3DState.ros.getTopics(function(topics) {
        if (!topics.topics || !topics.types) return;
        const toSubscribe = [];
        topics.topics.forEach(function(topicName, idx) {
            const type = topics.types[idx];
            if ((type === 'tf2_msgs/TFMessage' || type === 'tf2_msgs/msg/TFMessage') &&
                !_bgFrameSubs.has(topicName)) {
                toSubscribe.push(topicName);
            }
        });
        toSubscribe.forEach(function(name) {
            if (_bgFrameSubs.has(name)) return;
            try {
                const topic = new ROSLIB.Topic({
                    ros: viewer3DState.ros,
                    name: name,
                    messageType: 'tf2_msgs/msg/TFMessage',
                    throttle_rate: 100,
                    queue_length: 3
                });
                const tfSession = _tfSessionId;
                topic.subscribe(function(message) {
                    if (tfSession !== _tfSessionId) return;
                    _handleBackgroundTfMessage(message, name);
                });
                _bgFrameSubs.set(name, topic);
                console.log('[BG-TF] Discovered and subscribed to TF topic:', name);
            } catch (e) {
                console.warn('[BG-TF] Failed to subscribe to', name, e);
            }
        });
    }, function() {});
}

function _scheduleDiscoverExtraTfTopics(delayMs) {
    if (_discoverExtraTfTimerId !== null) {
        clearTimeout(_discoverExtraTfTimerId);
        _discoverExtraTfTimerId = null;
    }
    const d = delayMs !== undefined ? delayMs : 2000;
    _discoverExtraTfTimerId = setTimeout(function() {
        _discoverExtraTfTimerId = null;
        _discoverExtraTfTopicsImpl();
    }, d);
}

/**
 * /tf·/tf_static 재구독 + 네임스페이스 TF 지연 탐색
 * @param {{ skipUnsubscribe?: boolean }} [opts] true면 기존 구독 해제 생략(이미 _unsubscribeAllBackgroundTf 직후일 때)
 */
function restartBackgroundTfPipeline(opts) {
    if (!viewer3DState.ros || !viewer3DState.rosConnected) return;
    if (!opts || !opts.skipUnsubscribe) {
        _unsubscribeAllBackgroundTf();
    }
    _subscribeCoreBackgroundTfTopics();
    _scheduleDiscoverExtraTfTopics(2000);
}

// ── TopDown 뷰 Yaw 회전 상태 ──
let _topdownYaw = 0;                       // 현재 yaw 각도 (라디안, 0 = Y축이 화면 위 = 북쪽)
let _topdownYawDragActive = false;
let _topdownYawLastX = 0;
const _TOPDOWN_YAW_SENSITIVITY = 0.004;   // 픽셀당 회전 라디안

/**
 * _applyTopdownYaw()
 * 현재 _topdownYaw 값을 orthoCam.up 에 반영하고 lookAt 업데이트.
 * camera.up = (-sin(yaw), cos(yaw), 0) → look(-Z)과 항상 직교 보장.
 */
function _applyTopdownYaw() {
    const cam = viewer3DState.orthoCam;
    if (!cam) return;
    cam.up.set(-Math.sin(_topdownYaw), Math.cos(_topdownYaw), 0);
    const target = viewer3DState.controls
        ? viewer3DState.controls.target
        : new THREE.Vector3();
    cam.lookAt(target);
    if (viewer3DState.controls) viewer3DState.controls.update();
}

// pointer 이벤트 + 캡처 페이즈 사용:
// OrbitControls는 버블 페이즈로 등록되어 있으므로, 캡처 페이즈에서
// stopImmediatePropagation() 을 호출하면 OrbitControls가 좌클릭을 받기 전에
// 우리 핸들러가 먼저 처리하여 yaw 회전이 정상 동작한다.
function _topdownYawPointerdown(e) {
    if (e.button === 0) {
        _topdownYawDragActive = true;
        _topdownYawLastX = e.clientX;
        e.stopImmediatePropagation();  // OrbitControls의 pointerdown 차단
    }
}

function _topdownYawPointermove(e) {
    if (!_topdownYawDragActive) return;
    if (!(e.buttons & 1)) { _topdownYawDragActive = false; return; }
    const dx = e.clientX - _topdownYawLastX;
    _topdownYawLastX = e.clientX;
    _topdownYaw += dx * _TOPDOWN_YAW_SENSITIVITY;
    _applyTopdownYaw();
    e.stopImmediatePropagation();  // OrbitControls의 pointermove 차단
}

function _topdownYawPointerup(e) {
    if (e.button === 0) _topdownYawDragActive = false;
    // pointerup은 전파 허용 → OrbitControls 내부 정리가 진행되도록
}

/** TopDown yaw 이벤트 리스너 등록 (캡처 페이즈) */
function _attachTopdownYawHandlers(domEl) {
    domEl.addEventListener('pointerdown', _topdownYawPointerdown, true);
    domEl.addEventListener('pointermove', _topdownYawPointermove, true);
    domEl.addEventListener('pointerup',   _topdownYawPointerup,   true);
}

/** TopDown yaw 이벤트 리스너 해제 (캡처 페이즈) */
function _detachTopdownYawHandlers(domEl) {
    domEl.removeEventListener('pointerdown', _topdownYawPointerdown, true);
    domEl.removeEventListener('pointermove', _topdownYawPointermove, true);
    domEl.removeEventListener('pointerup',   _topdownYawPointerup,   true);
}

/**
 * ROS 연결 직후: 백그라운드 /tf·/tf_static + 네임스페이스 TF 탐색
 */
function startBackgroundFrameCollection() {
    if (!viewer3DState.rosConnected) return;
    restartBackgroundTfPipeline();
}

/**
 * 현재 알려진 모든 TF 프레임 ID 수집
 * - _allKnownFrames : 백그라운드 /tf, /tf_static 구독으로 수집
 * - tfFrameTree     : 사용자가 구독한 TF 토픽 파싱 결과
 * - topicFrameIds   : PC2 / Path / Odom 메시지 헤더의 frame_id
 */
function getAvailableFrames() {
    const frames = new Set(_allKnownFrames);

    viewer3DState.tfFrameTree.forEach(function(entry, childId) {
        frames.add(childId);
        if (entry.parentId && entry.parentId !== '') frames.add(entry.parentId);
    });
    viewer3DState.topicFrameIds.forEach(function(frameId, key) {
        if (frameId && frameId !== '' && !key.endsWith('_traj')) {
            frames.add(frameId);
        }
    });
    return Array.from(frames).sort();
}

// ── 내부 상태: 드롭다운 키보드 탐색 + 모드 플래그 ──
let _ffHighlightIdx = -1;
let _ffSelecting    = false;   // mousedown 선택 직후 oninput 버그 차단
let _ffShowAll      = false;   // true = ▾ 버튼으로 연 '전체 목록' 모드
                               // false = 사용자 타이핑으로 연 '필터' 모드
let _ffBlurTimer    = null;    // onblur setTimeout 핸들 (clearTimeout용)

/**
 * Fixed Frame 드롭다운 패널 렌더링
 * @param {string} [filterText] - 필터 문자열 (빈 문자열/undefined → 전체 표시)
 */
function renderFixedFrameDropdown(filterText) {
    const dropdown = document.getElementById('fixed-frame-dropdown');
    if (!dropdown) return;

    const frames  = getAvailableFrames();
    const filter  = (filterText !== undefined && filterText !== null)
                    ? filterText.toLowerCase().trim() : '';
    const filtered = filter
                    ? frames.filter(f => f.toLowerCase().includes(filter))
                    : frames;

    _ffHighlightIdx = -1;
    dropdown.innerHTML = '';

    if (filtered.length === 0) {
        const empty = document.createElement('div');
        empty.className = 'fixed-frame-dropdown-empty';
        empty.textContent = frames.length === 0
            ? '/tf 또는 /tf_static 데이터 대기 중...'
            : '일치하는 프레임 없음';
        dropdown.appendChild(empty);
        dropdown.style.display = 'block';
        return;
    }

    filtered.forEach(function(frameId) {
        const item = document.createElement('div');
        item.className = 'fixed-frame-dropdown-item'
            + (frameId === viewer3DState.fixedFrame ? ' active' : '');
        item.textContent = frameId;
        item.dataset.frame = frameId;

        item.addEventListener('mousedown', function(e) {
            e.preventDefault();  // input blur 방지
            _ffSelecting = true; // oninput 이벤트 차단 시작

            const input = document.getElementById('fixed-frame-input');
            if (input) input.value = frameId;
            setFixedFrame(frameId);
            closeFixedFrameDropdown();

            // 다음 렌더 프레임 이후 플래그 해제 (브라우저 이벤트 큐 처리 후)
            requestAnimationFrame(function() { _ffSelecting = false; });
        });
        dropdown.appendChild(item);
    });

    dropdown.style.display = 'block';

    // 현재 fixedFrame 항목으로 스크롤
    const activeEl = dropdown.querySelector('.active');
    if (activeEl) activeEl.scrollIntoView({ block: 'nearest' });
}

/**
 * 드롭다운 열기/닫기 토글
 * - ▾ 버튼 클릭 시: _ffShowAll=true로 설정하고 전체 목록 표시
 *   → TF 데이터 수신으로 updateFixedFrameOptions() 가 호출되어도
 *     전체 목록 모드를 유지하므로 필터링되지 않음
 */
function toggleFixedFrameDropdown() {
    const dropdown = document.getElementById('fixed-frame-dropdown');
    if (!dropdown) return;

    // 열려 있는 경우: 닫기
    if (dropdown.style.display !== 'none') {
        closeFixedFrameDropdown();
        return;
    }

    // 닫혀 있는 경우: 전체 목록 표시
    // 진행 중인 blur 닫기 타이머 취소 (▾ 클릭이 blur보다 뒤에 처리될 때 방지)
    if (_ffBlurTimer !== null) {
        clearTimeout(_ffBlurTimer);
        _ffBlurTimer = null;
    }

    _ffShowAll = true;  // 전체 목록 모드
    renderFixedFrameDropdown('');

    const input = document.getElementById('fixed-frame-input');
    if (input) input.focus();
}

/**
 * 드롭다운 닫기
 */
function closeFixedFrameDropdown() {
    if (_ffBlurTimer !== null) {
        clearTimeout(_ffBlurTimer);
        _ffBlurTimer = null;
    }
    const dropdown = document.getElementById('fixed-frame-dropdown');
    if (dropdown) dropdown.style.display = 'none';
    _ffHighlightIdx = -1;
    _ffShowAll = false;
}

/**
 * 키보드 하이라이트 항목 갱신
 */
function _ffUpdateHighlight(dropdown) {
    const items = dropdown.querySelectorAll('.fixed-frame-dropdown-item');
    items.forEach((el, i) => {
        el.classList.toggle('highlighted', i === _ffHighlightIdx);
        if (i === _ffHighlightIdx) el.scrollIntoView({ block: 'nearest' });
    });
}

/* ── HTML 이벤트 핸들러 (index.html에서 호출) ── */

/**
 * input oninput 핸들러: 입력 글자로 필터링 후 드롭다운 표시
 * _ffSelecting이 true면 mousedown 선택 직후이므로 무시
 */
function onFixedFrameInput(value) {
    if (_ffSelecting) return;
    _ffShowAll = false;  // 사용자가 직접 타이핑 → 필터 모드로 전환
    renderFixedFrameDropdown(value);
}

/**
 * input onkeydown 핸들러: 화살표 / Enter / Escape 처리
 */
function onFixedFrameKeydown(event) {
    const dropdown = document.getElementById('fixed-frame-dropdown');
    const input    = document.getElementById('fixed-frame-input');
    const isOpen   = dropdown && dropdown.style.display !== 'none';

    if (event.key === 'ArrowDown') {
        event.preventDefault();
        if (!isOpen) { renderFixedFrameDropdown(''); return; }
        const items = dropdown.querySelectorAll('.fixed-frame-dropdown-item');
        if (items.length === 0) return;
        _ffHighlightIdx = Math.min(_ffHighlightIdx + 1, items.length - 1);
        _ffUpdateHighlight(dropdown);

    } else if (event.key === 'ArrowUp') {
        event.preventDefault();
        if (!isOpen) return;
        const items = dropdown.querySelectorAll('.fixed-frame-dropdown-item');
        _ffHighlightIdx = Math.max(_ffHighlightIdx - 1, 0);
        _ffUpdateHighlight(dropdown);

    } else if (event.key === 'Enter') {
        event.preventDefault();
        if (isOpen && _ffHighlightIdx >= 0) {
            const items = dropdown.querySelectorAll('.fixed-frame-dropdown-item');
            if (items[_ffHighlightIdx]) {
                const frameId = items[_ffHighlightIdx].dataset.frame;
                _ffSelecting = true;
                if (input) input.value = frameId;
                setFixedFrame(frameId);
                requestAnimationFrame(function() { _ffSelecting = false; });
            }
        } else if (input) {
            setFixedFrame(input.value.trim());
        }
        closeFixedFrameDropdown();

    } else if (event.key === 'Escape') {
        closeFixedFrameDropdown();
        if (input) input.blur();
    }
}

/**
 * input onblur 핸들러: 드롭다운 외부 클릭 시 닫기
 * _ffSelecting 중이거나 combo 내부로 포커스 이동 시 닫지 않음
 * toggleFixedFrameDropdown()에서 clearTimeout으로 취소 가능하도록
 * _ffBlurTimer에 핸들을 저장
 */
function onFixedFrameBlur(event) {
    if (_ffSelecting) return;
    const combo = document.getElementById('fixed-frame-combo');
    if (combo && combo.contains(event.relatedTarget)) return;
    _ffBlurTimer = setTimeout(function() {
        _ffBlurTimer = null;
        closeFixedFrameDropdown();
    }, 120);
}

/**
 * Fixed Frame input의 datalist를 현재 tfFrameTree 프레임 ID로 업데이트.
 * TF 구독 중일 때 rebuildTFScene에서 호출되어 자동으로 최신 목록 유지.
 * 드롭다운이 열려 있으면 실시간 갱신.
 */
function updateFixedFrameOptions() {
    // 드롭다운이 열려 있을 때만 갱신
    const dropdown = document.getElementById('fixed-frame-dropdown');
    if (!dropdown || dropdown.style.display === 'none') return;

    if (_ffShowAll) {
        // ▾ 버튼으로 연 전체 목록 모드 → 필터 없이 전체 재렌더
        renderFixedFrameDropdown('');
    } else {
        // 타이핑 필터 모드 → 현재 입력값 기준 재렌더
        const input = document.getElementById('fixed-frame-input');
        renderFixedFrameDropdown(input ? input.value : '');
    }
}

/**
 * TF 토픽 구독 (tf2_msgs/msg/TFMessage)
 * 메시지 수신마다 applyTfMessageWithRvStyleLifecycle() → throttle 100ms → rebuildTFScene()
 * @param {string} topicName - TF 토픽명 ('/tf' 또는 '/tf_static')
 * @returns {ROSLIB.Topic} 구독 객체
 */
function subscribeToTF(topicName) {
    if (!viewer3DState.rosConnected) {
        console.warn('[TF] Not connected to ROS');
        return;
    }

    // 재구독 방지
    if (viewer3DState.topicSubscriptions.has(topicName)) {
        return viewer3DState.topicSubscriptions.get(topicName);
    }

    console.log('[TF] Subscribing to topic:', topicName);

    const topic = new ROSLIB.Topic({
        ros:          viewer3DState.ros,
        name:         topicName,
        messageType:  'tf2_msgs/msg/TFMessage',
        throttle_rate: 100,   // 최대 10Hz
        queue_length:  1
    });

    viewer3DState.topicSubscriptions.set(topicName, topic);

    // throttle 변수: 100ms 이내 중복 rebuildTFScene 방지
    let _tfThrottleTimer = null;
    const tfSession = _tfSessionId;

    topic.subscribe(function(message) {
        if (tfSession !== _tfSessionId) return;
        // 숨겨진 경우 파싱 스킵 (CPU 절약)
        const tfObj = viewer3DState.tfObjects.get(topicName);
        if (tfObj && tfObj.visible === false) return;

        viewer3DState.needsRender = true;
        applyTfMessageWithRvStyleLifecycle(message, topicName);

        if (_tfThrottleTimer) return;
        _tfThrottleTimer = setTimeout(function() {
            _tfThrottleTimer = null;
            if (tfSession !== _tfSessionId) return;
            rebuildTFScene(topicName);
        }, 100);
    });

    return topic;
}

/**
 * 데이터 로드·백 전환 후 Fixed Frame 입력·상태를 기본값으로, Target Frame은 &lt;Fixed Frame&gt;으로.
 * (사용자가 이전에 고른 프레임이 남아 다른 데이터 TF와 섞여 보이는 것 방지)
 */
function resetFixedFrameAndViewTargetsToDefault() {
    viewer3DState.fixedFrame = DEFAULT_FIXED_FRAME;
    const fixedFrameInput = document.getElementById('fixed-frame-input');
    if (fixedFrameInput) {
        fixedFrameInput.value = DEFAULT_FIXED_FRAME;
    }
    closeFixedFrameDropdown();

    viewer3DState.cameraTargetFrame = '<Fixed Frame>';
    const viewTargetSel = document.getElementById('view-target-frame');
    if (viewTargetSel) {
        viewTargetSel.value = '<Fixed Frame>';
    }

    viewer3DState.selectedTFTopics.forEach(function(topicName) {
        try {
            rebuildTFScene(topicName);
        } catch (e) {
            console.warn('[Viewer] resetFixedFrame rebuildTFScene:', topicName, e);
        }
    });
    reapplyAllFrameTransforms();
    console.log('[Viewer] Fixed frame →', DEFAULT_FIXED_FRAME, '| Target frame → <Fixed Frame>');
}

/**
 * Fixed Frame 변경 → 구독 중인 모든 TF 토픽에 대해 rebuildTFScene() 재호출
 * @param {string} newFrame - 새 기준 frame ID (예: 'map', 'odom', 'base_link')
 */
function setFixedFrame(newFrame) {
    newFrame = (newFrame || '').trim();
    if (!newFrame || newFrame === viewer3DState.fixedFrame) return;

    viewer3DState.fixedFrame = newFrame;
    console.log('[TF] Fixed frame changed to:', newFrame);

    // input 값 동기화 (외부에서 호출 시 UI 반영)
    const fixedFrameInput = document.getElementById('fixed-frame-input');
    if (fixedFrameInput && fixedFrameInput.value !== newFrame) {
        fixedFrameInput.value = newFrame;
    }

    // 구독 중인 모든 TF 토픽 씬 재빌드
    viewer3DState.selectedTFTopics.forEach(function(topicName) {
        rebuildTFScene(topicName);
    });

    // PointCloud2 / Path / Odometry 오브젝트 frame transform 재적용
    reapplyAllFrameTransforms();
}

/**
 * TF 가시성 토글 (그룹 show/hide)
 * @param {string} topicName
 * @param {boolean} visible
 */
function toggleTFVisible(topicName, visible) {
    const tfObj = viewer3DState.tfObjects.get(topicName);
    if (tfObj) {
        tfObj.group.visible = visible;
        tfObj.visible = visible;
        console.log('[TF]', topicName, 'visibility:', visible);
    }
}

/**
 * 사용 가능한 TF 토픽 목록 조회 (tf2_msgs/msg/TFMessage)
 * @returns {Promise<string[]>}
 */
async function getAvailableTFTopics() {
    if (!viewer3DState.rosConnected) {
        console.log('[TF] Waiting for ROS connection...');
        const connected = await waitForROSConnection(5000);
        if (!connected) {
            console.warn('[TF] Not connected to ROS after timeout');
            return [];
        }
    }

    return new Promise(function(resolve) {
        viewer3DState.ros.getTopics(function(topics) {
            const tfTopics = [];
            if (topics.topics && topics.types) {
                topics.topics.forEach(function(topic, index) {
                    const type = topics.types[index];
                    if (type === 'tf2_msgs/TFMessage' || type === 'tf2_msgs/msg/TFMessage') {
                        tfTopics.push(topic);
                    }
                });
            }
            // /tf, /tf_static가 발행되지 않더라도 기본값으로 포함
            ['/tf', '/tf_static'].forEach(function(defaultTopic) {
                if (!tfTopics.includes(defaultTopic)) {
                    tfTopics.push(defaultTopic);
                }
            });
            console.log('[TF] Available TF topics:', tfTopics);
            resolve(tfTopics);
        }, function(error) {
            console.error('[TF] Failed to get topics:', error);
            resolve(['/tf', '/tf_static']);
        });
    });
}

/**
 * TF 토픽 선택 다이얼로그 열기
 */
async function selectTFTopics() {
    if (!viewer3DState.rosConnected) {
        alert('Not connected to ROS. Make sure rosbridge_server is running:\n\nros2 launch rosbridge_server rosbridge_websocket_launch.xml');
        return;
    }

    const btn = document.getElementById('viewer-add-tf-btn');
    const originalText = btn ? btn.textContent : '';
    if (btn) { btn.textContent = 'Loading...'; btn.disabled = true; }

    let topics = [];
    try {
        topics = await getAvailableTFTopics();
    } finally {
        if (btn) { btn.textContent = originalText; btn.disabled = false; }
    }

    const modal     = document.getElementById('tf-topic-modal');
    const topicList = document.getElementById('tf-topic-list');
    if (!modal || !topicList) {
        console.error('[TF] Topic modal element not found in HTML');
        return;
    }

    topicList.innerHTML = '';
    topics.forEach(function(topic) {
        const div = document.createElement('div');
        div.className = 'topic-item';

        const checkbox = document.createElement('input');
        checkbox.type    = 'checkbox';
        checkbox.id      = `tf-topic-cb-${topic}`;
        checkbox.value   = topic;
        checkbox.checked = viewer3DState.selectedTFTopics.includes(topic);

        const label = document.createElement('label');
        label.htmlFor     = `tf-topic-cb-${topic}`;
        label.textContent = topic;

        div.appendChild(checkbox);
        div.appendChild(label);
        topicList.appendChild(div);
    });

    modal.style.display = 'block';
}

/**
 * TF 토픽 선택 모달 닫기
 */
function closeTFTopicSelection() {
    const modal = document.getElementById('tf-topic-modal');
    if (modal) modal.style.display = 'none';
}

/**
 * TF 토픽 선택 확인: 체크된 토픽 구독, 해제된 토픽 정리
 */
function confirmTFTopicSelection() {
    const checkboxes = document.querySelectorAll('#tf-topic-list input[type="checkbox"]');
    const newTopics  = [];
    checkboxes.forEach(function(cb) { if (cb.checked) newTopics.push(cb.value); });

    // 제거된 토픽: 구독 해제 및 Three.js 리소스 정리
    viewer3DState.selectedTFTopics.forEach(function(topic) {
        if (!newTopics.includes(topic)) {
            unsubscribeFromTopic(topic);
        }
    });

    // 신규 토픽: 구독 시작
    newTopics.forEach(function(topic) {
        if (!viewer3DState.selectedTFTopics.includes(topic)) {
            subscribeToTF(topic);
        }
    });

    viewer3DState.selectedTFTopics = newTopics;
    closeTFTopicSelection();
    renderDisplayPanel();
    console.log('[TF] Selected topics:', viewer3DState.selectedTFTopics);
}

// ============================================================
// Views 패널 접기/펼치기
// ============================================================

/**
 * 3D Viewer 그리드 열 너비: Displays / 캔버스 / Views 접힘 상태 조합
 */
function updateViewerLayoutColumns() {
    const layout = document.querySelector('.viewer-layout');
    if (!layout) return;
    const displayPanel = document.getElementById('viewer-display-panel');
    const viewsPanel = document.getElementById('viewer-views-panel');
    const displayCollapsed = displayPanel && displayPanel.classList.contains('collapsed');
    const viewsCollapsed = viewsPanel && viewsPanel.classList.contains('collapsed');
    const left = displayCollapsed ? '28px' : '260px';
    const right = viewsCollapsed ? '28px' : '200px';
    layout.style.gridTemplateColumns = `${left} 1fr ${right}`;
}

/**
 * Displays 패널 접기/펼치기 (Views 패널과 동일한 화살표 UX)
 */
function toggleViewerDisplayPanel() {
    const panel = document.getElementById('viewer-display-panel');
    if (!panel) return;
    const isCollapsed = panel.classList.toggle('collapsed');
    const btn = document.getElementById('viewer-display-collapse-btn');
    if (btn) btn.textContent = isCollapsed ? '◀' : '▶';
    updateViewerLayoutColumns();
    setTimeout(onWindowResize, 220);
}
window.toggleViewerDisplayPanel = toggleViewerDisplayPanel;

/**
 * Views 패널 접기/펼치기 토글
 * - 패널 본문(#views-body)을 숨기거나 표시
 * - .viewer-layout의 3번째 열 너비를 28px(접힘) / 200px(펼침)로 전환
 * - 화살표 버튼 방향을 ◀(접힘) / ▶(펼침)로 전환
 */
function toggleViewsPanel() {
    const panel = document.getElementById('viewer-views-panel');
    if (!panel) return;
    const isCollapsed = panel.classList.toggle('collapsed');
    const btn = document.getElementById('views-collapse-btn');
    if (btn) btn.textContent = isCollapsed ? '◀' : '▶';
    updateViewerLayoutColumns();
    // CSS transition(0.2s) 완료 후 Three.js 렌더러 크기 재조정
    setTimeout(onWindowResize, 220);
}
window.toggleViewsPanel = toggleViewsPanel;

// =============================================
// 통합 Add Display 다이얼로그 (RViz 스타일)
// =============================================

/**
 * 각 메시지 타입별 카테고리 정의
 */
const ADD_DISPLAY_CATEGORIES = [
    {
        type: 'PointCloud2',
        msgType: 'sensor_msgs/PointCloud2',
        description: 'Displays PointCloud2 messages as a set of colored points in 3D space.',
        color: '#4ad6ff',
        stateKey: 'displaySelectedTopics',
        fetchTopics: null,   // 초기화 후 할당
        subscribeFn: null
    },
    {
        type: 'Path',
        msgType: 'nav_msgs/Path',
        description: 'Displays a nav_msgs/Path message as a series of connected line segments in 3D space.',
        color: '#4dff7c',
        stateKey: 'selectedPathTopics',
        fetchTopics: null,
        subscribeFn: null
    },
    {
        type: 'Odometry',
        msgType: 'nav_msgs/Odometry',
        description: 'Displays odometry data from a nav_msgs/Odometry message as an arrow showing pose and optional trajectory trail.',
        color: '#ff9f4d',
        stateKey: 'selectedOdomTopics',
        fetchTopics: null,
        subscribeFn: null
    },
    {
        type: 'TF',
        msgType: 'tf2_msgs/TFMessage',
        description: 'Displays the TF transform tree, showing coordinate frames and their relationships as axes in 3D space.',
        color: '#c0a0ff',
        stateKey: 'selectedTFTopics',
        fetchTopics: null,
        subscribeFn: null
    },
    {
        type: 'LivoxLidar',
        msgType: 'livox_ros_driver2/msg/CustomMsg',
        description: 'Displays Livox LiDAR CustomMsg as colored points. Supports By Line color mode and Tag Filter for noise removal.',
        color: '#29b6f6',            // 하늘색 배지
        stateKey: 'selectedLivoxTopics',
        fetchTopics: null,           // 런타임 할당
        subscribeFn: null
    },
    {
        type: 'Image',
        msgType: 'sensor_msgs/msg/Image',
        description: 'Displays sensor_msgs/Image topics as live video panels below the 3D view.',
        color: '#ab47bc',
        stateKey: 'selectedImageTopics',
        fetchTopics: null,
        subscribeFn: null
    },
    {
        type: 'LaserScan',
        msgType: 'sensor_msgs/msg/LaserScan',
        description: 'Displays 2D laser scan data as points in 3D space (RViz LaserScanDisplay style).',
        color: '#00e676',
        stateKey: 'selectedLaserScanTopics',
        fetchTopics: null,
        subscribeFn: null
    }
];

// 카테고리별 현재 로드된 토픽 목록 (모달 내부 상태)
let _addDisplayTopics = [[], [], [], [], [], [], []];

/**
 * Add Display 통합 다이얼로그 열기 (RViz 스타일)
 */
async function openAddDisplayModal() {
    if (!viewer3DState.rosConnected) {
        alert('Not connected to ROS. Make sure rosbridge_server is running:\n\nros2 launch rosbridge_server rosbridge_websocket_launch.xml');
        return;
    }

    // fetchTopics 함수 할당 (순환 참조 방지를 위해 런타임에 할당)
    ADD_DISPLAY_CATEGORIES[0].fetchTopics = getAvailablePointCloudTopics;
    ADD_DISPLAY_CATEGORIES[0].subscribeFn = subscribeToPointCloud;
    ADD_DISPLAY_CATEGORIES[1].fetchTopics = getAvailablePathTopics;
    ADD_DISPLAY_CATEGORIES[1].subscribeFn = subscribeToPath;
    ADD_DISPLAY_CATEGORIES[2].fetchTopics = getAvailableOdometryTopics;
    ADD_DISPLAY_CATEGORIES[2].subscribeFn = subscribeToOdometry;
    ADD_DISPLAY_CATEGORIES[3].fetchTopics = getAvailableTFTopics;
    ADD_DISPLAY_CATEGORIES[3].subscribeFn = subscribeToTF;
    ADD_DISPLAY_CATEGORIES[4].fetchTopics = getAvailableLivoxTopics;
    ADD_DISPLAY_CATEGORIES[4].subscribeFn = subscribeToLivox;
    ADD_DISPLAY_CATEGORIES[5].fetchTopics = getAvailableImageTopics;
    ADD_DISPLAY_CATEGORIES[5].subscribeFn = subscribeToImage;
    ADD_DISPLAY_CATEGORIES[6].fetchTopics = getAvailableLaserScanTopics;
    ADD_DISPLAY_CATEGORIES[6].subscribeFn = subscribeToLaserScan;

    const modal = document.getElementById('add-display-modal');
    const tree  = document.getElementById('add-display-tree');
    if (!modal || !tree) return;

    // 버튼 로딩 표시
    const btn = document.getElementById('viewer-add-display-btn');
    if (btn) { btn.textContent = 'Loading...'; btn.disabled = true; }

    tree.innerHTML = '<div class="add-display-loading">Loading topics...</div>';
    modal.style.display = 'block';

    // 모든 토픽 타입 병렬 로딩
    try {
        const results = await Promise.all(
            ADD_DISPLAY_CATEGORIES.map(cat => cat.fetchTopics())
        );
        _addDisplayTopics = results;
    } catch (err) {
        console.error('[AddDisplay] Failed to load topics:', err);
        _addDisplayTopics = [[], [], [], [], [], [], []];
    } finally {
        if (btn) { btn.textContent = '+ Add'; btn.disabled = false; }
    }

    renderAddDisplayTree();
}

/**
 * Add Display 트리 렌더링
 */
function renderAddDisplayTree() {
    const tree = document.getElementById('add-display-tree');
    if (!tree) return;

    tree.innerHTML = '';

    ADD_DISPLAY_CATEGORIES.forEach(function(cat, catIdx) {
        const currentTopics = viewer3DState[cat.stateKey] || [];
        const availableTopics = _addDisplayTopics[catIdx] || [];

        // 카테고리 컨테이너
        const catDiv = document.createElement('div');
        catDiv.className = 'add-display-category';

        // 카테고리 헤더
        const catHeader = document.createElement('div');
        catHeader.className = 'add-display-cat-header';
        catHeader.innerHTML =
            '<span class="add-display-cat-arrow" id="add-display-cat-arrow-' + catIdx + '">▾</span>' +
            '<span class="add-display-cat-dot" style="background:' + cat.color + '"></span>' +
            '<span class="add-display-cat-name">' + cat.type + '</span>' +
            '<span class="add-display-cat-type">' + cat.msgType + '</span>';
        catHeader.onclick = function() { toggleAddDisplayCategory(catIdx); };
        catDiv.appendChild(catHeader);

        // 토픽 목록
        const topicList = document.createElement('div');
        topicList.className = 'add-display-topic-list';
        topicList.id = 'add-display-topics-' + catIdx;

        if (availableTopics.length === 0) {
            const noTopic = document.createElement('div');
            noTopic.className = 'add-display-no-topics';
            noTopic.textContent = 'No topics available';
            topicList.appendChild(noTopic);
        } else {
            availableTopics.forEach(function(topic) {
                const topicDiv = document.createElement('div');
                topicDiv.className = 'add-display-topic-item';

                const cb = document.createElement('input');
                cb.type    = 'checkbox';
                cb.id      = 'add-display-cb-' + catIdx + '-' + topic;
                cb.value   = topic;
                cb.checked = currentTopics.includes(topic);
                cb.onchange = function() { updateAddDisplayDescription(cat.description); };

                const lbl = document.createElement('label');
                lbl.htmlFor     = cb.id;
                lbl.textContent = topic;
                lbl.onclick = function() { updateAddDisplayDescription(cat.description); };

                topicDiv.appendChild(cb);
                topicDiv.appendChild(lbl);
                topicList.appendChild(topicDiv);
            });
        }

        catDiv.appendChild(topicList);
        tree.appendChild(catDiv);
    });

    // 설명 초기화
    updateAddDisplayDescription('');
}

/**
 * 카테고리 접기/펼치기
 */
function toggleAddDisplayCategory(catIdx) {
    const list  = document.getElementById('add-display-topics-' + catIdx);
    const arrow = document.getElementById('add-display-cat-arrow-' + catIdx);
    if (!list) return;
    const isCollapsed = list.style.display === 'none';
    list.style.display = isCollapsed ? 'block' : 'none';
    if (arrow) arrow.textContent = isCollapsed ? '▾' : '▸';
}

/**
 * 설명 텍스트 업데이트
 */
function updateAddDisplayDescription(text) {
    const el = document.getElementById('add-display-desc-text');
    if (el) el.textContent = text;
}

/**
 * Add Display 다이얼로그 닫기
 */
function closeAddDisplayModal() {
    const modal = document.getElementById('add-display-modal');
    if (modal) modal.style.display = 'none';
}

/**
 * Add Display 선택 확인: 모든 카테고리의 체크된 토픽 구독/해제 처리
 */
function confirmAddDisplaySelection() {
    ADD_DISPLAY_CATEGORIES.forEach(function(cat, catIdx) {
        const checkboxes = document.querySelectorAll(
            '#add-display-topics-' + catIdx + ' input[type="checkbox"]'
        );
        const newTopics = [];
        checkboxes.forEach(function(cb) { if (cb.checked) newTopics.push(cb.value); });

        const currentTopics = viewer3DState[cat.stateKey] || [];

        // Image 카테고리: 상태(selectedImageTopics)에 없더라도
        // imageSubscriptions에 남아있는 고아 구독(이전 bag 잔재)까지 제거 대상에 포함
        const topicsToRemove = (cat.stateKey === 'selectedImageTopics')
            ? new Set([...currentTopics, ...viewer3DState.imageSubscriptions.keys()])
            : new Set(currentTopics);

        // 제거된 토픽: 구독 해제
        topicsToRemove.forEach(function(topic) {
            if (!newTopics.includes(topic)) {
                unsubscribeFromTopic(topic);
            }
        });

        // 신규 토픽: 구독 시작
        newTopics.forEach(function(topic) {
            if (!currentTopics.includes(topic)) {
                cat.subscribeFn(topic);
            }
        });

        viewer3DState[cat.stateKey] = newTopics;
    });

    closeAddDisplayModal();
    renderDisplayPanel();
    console.log('[AddDisplay] Confirmed. State:', {
        pointcloud: viewer3DState.displaySelectedTopics,
        path: viewer3DState.selectedPathTopics,
        odometry: viewer3DState.selectedOdomTopics,
        tf: viewer3DState.selectedTFTopics,
        image: viewer3DState.selectedImageTopics,
        laserScan: viewer3DState.selectedLaserScanTopics
    });
}

// Expose functions to global scope for onclick handlers
window.initThreeJSDisplay = initThreeJSDisplay;
window.resetViewerTopicSubscriptions = resetViewerTopicSubscriptions;
window.resetBagFrameAndTFState = resetBagFrameAndTFState;
window.resetViewerAfterPlayerLoad = resetViewerAfterPlayerLoad;
window.resetAll3DViewer = resetAll3DViewer;
window.syncPlayerFilePointCloudSubscriptions = syncPlayerFilePointCloudSubscriptions;
window.initialize3DViewer = initialize3DViewer;
// 통합 Add Display 다이얼로그
window.openAddDisplayModal = openAddDisplayModal;
window.closeAddDisplayModal = closeAddDisplayModal;
window.confirmAddDisplaySelection = confirmAddDisplaySelection;
window.toggleAddDisplayCategory = toggleAddDisplayCategory;
// 기존 개별 함수 (하위 호환성 유지)
window.selectDisplayTopics = selectDisplayTopics;
window.closeDisplayTopicSelection = closeDisplayTopicSelection;
window.confirmDisplayTopicSelection = confirmDisplayTopicSelection;
window.moveRendererToActiveContainer = moveRendererToActiveContainer;
window.renderDisplayPanel = renderDisplayPanel;
window.toggleTopicVisible = toggleTopicVisible;
window.updateTopicSetting = updateTopicSetting;
window.closeColorPickerPopup = closeColorPickerPopup;
window.confirmColorPickerPopup = confirmColorPickerPopup;
window.subscribeToPath = subscribeToPath;
window.subscribeToOdometry = subscribeToOdometry;
window.updateTrajectory = updateTrajectory;
window.getAvailablePathTopics = getAvailablePathTopics;
window.getAvailableOdometryTopics = getAvailableOdometryTopics;
// Path / Odometry 가시성 토글 및 색상 업데이트
window.togglePathVisible = togglePathVisible;
window.toggleOdomVisible = toggleOdomVisible;
window.updatePathColor = updatePathColor;
// Path / Odometry 토픽 선택 UI
window.selectPathTopics = selectPathTopics;
window.closePathTopicSelection = closePathTopicSelection;
window.confirmPathTopicSelection = confirmPathTopicSelection;
window.selectOdometryTopics = selectOdometryTopics;
window.closeOdometryTopicSelection = closeOdometryTopicSelection;
window.confirmOdometryTopicSelection = confirmOdometryTopicSelection;
// TF 관련 함수
window.subscribeToTF = subscribeToTF;
window.setFixedFrame = setFixedFrame;
window.resetFixedFrameAndViewTargetsToDefault = resetFixedFrameAndViewTargetsToDefault;
window.toggleTFVisible = toggleTFVisible;
window.selectTFTopics = selectTFTopics;
window.closeTFTopicSelection = closeTFTopicSelection;
window.confirmTFTopicSelection = confirmTFTopicSelection;
window.rebuildTFScene = rebuildTFScene;
window.updateFixedFrameOptions = updateFixedFrameOptions;
// Fixed Frame 커스텀 드롭다운 핸들러
window.getAvailableFrames = getAvailableFrames;
window.startBackgroundFrameCollection = startBackgroundFrameCollection;
window.toggleFixedFrameDropdown = toggleFixedFrameDropdown;
window.closeFixedFrameDropdown = closeFixedFrameDropdown;
window.onFixedFrameInput = onFixedFrameInput;
window.onFixedFrameKeydown = onFixedFrameKeydown;
window.onFixedFrameBlur = onFixedFrameBlur;

/**
 * Fixed Frame input onfocus 핸들러
 * 입력창 클릭(포커스) 시 드롭다운 자동 열기
 */
function onFixedFrameFocus() {
    if (_ffSelecting) return;
    const dropdown = document.getElementById('fixed-frame-dropdown');
    if (dropdown && dropdown.style.display === 'none') {
        if (_ffBlurTimer !== null) {
            clearTimeout(_ffBlurTimer);
            _ffBlurTimer = null;
        }
        _ffShowAll = true;
        renderFixedFrameDropdown('');
    }
}
window.onFixedFrameFocus = onFixedFrameFocus;

// ─── Snapshot ────────────────────────────────────────────────────────────────
function take3DSnapshot() {
    if (!viewer3DState.renderer || !viewer3DState.scene) return;
    const cam = viewer3DState.activeCamera || viewer3DState.camera;
    viewer3DState.renderer.render(viewer3DState.scene, cam); // 최신 프레임 보장
    const ts = new Date().toISOString().replace(/[-:.TZ]/g, '').slice(0, 15);
    const dataURL = viewer3DState.renderer.domElement.toDataURL('image/png');
    const a = document.createElement('a');
    a.href = dataURL;
    a.download = `snapshot_${ts}.png`;
    a.click();
}
window.take3DSnapshot = take3DSnapshot;
// Image 패널 관련
window.toggleImagePanel = toggleImagePanel;
window.unsubscribeFromImage = unsubscribeFromImage;

console.log('=== Three.js Display script loaded ===');
console.log('Functions exposed:', {
    initThreeJSDisplay: typeof initThreeJSDisplay,
    initialize3DViewer: typeof initialize3DViewer,
    selectDisplayTopics: typeof selectDisplayTopics
});
