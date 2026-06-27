// PlotTabManager 클래스 - 브라우저 스타일 Plot 탭 관리
class PlotTabManager {
    constructor(tabBarContainerId, plotAreaContainerId, bufferTime = 5.0) {
        this.tabBarContainerId = tabBarContainerId;
        this.plotAreaContainerId = plotAreaContainerId;
        this.tabBarContainer = null;
        this.plotAreaContainer = null;
        this.bufferTime = bufferTime;
        
        this.tabs = [];  // { id, title, plotManager, plotDiv }
        this.activeTabId = null;
        this.nextTabId = 1;
        this.maxTabs = 10;  // 최대 탭 개수 제한
        
        console.log('[PlotTabManager] Constructor called');
    }

    init() {
        this.tabBarContainer = document.getElementById(this.tabBarContainerId);
        this.plotAreaContainer = document.getElementById(this.plotAreaContainerId);
        
        if (!this.tabBarContainer || !this.plotAreaContainer) {
            console.error('[PlotTabManager] Required containers not found');
            return false;
        }
        
        // 탭 바 UI 생성
        this.createTabBar();
        
        // 저장된 상태 복원 시도
        const stateLoaded = this.loadState();
        
        // 저장된 상태가 없으면 디폴트 탭 1개 생성
        if (!stateLoaded) {
            this.createTab();
            console.log('[PlotTabManager] Initialized with default tab');
        } else {
            console.log('[PlotTabManager] Initialized with restored state');
            
            // 복원된 탭들의 paths 복원 (rosbridge 연결 후)
            // 이는 setupPlotDataUpdate가 호출된 후에 실행되어야 함
            // 따라서 여기서는 savedPaths를 보관만 하고, 나중에 복원
        }
        
        // beforeunload 이벤트 등록 - 페이지 떠나기 직전에 상태 저장
        window.addEventListener('beforeunload', () => {
            console.log('[PlotTabManager] beforeunload: saving state');
            this.saveState();
        });
        
        return true;
    }

    createTabBar() {
        // 탭 바 레이아웃: 왼쪽에 탭 목록, 오른쪽에 + 버튼
        this.tabBarContainer.innerHTML = `
            <div style="display: flex; align-items: center; gap: 6px; border-bottom: 2px solid rgba(124, 111, 255, 0.2); margin-bottom: 8px; padding-bottom: 2px; flex-wrap: nowrap; min-height: 40px; max-height: 40px; overflow: hidden;">
                <div id="plot-tab-list" style="display: flex; align-items: center; gap: 6px; flex: 1; overflow-x: hidden; flex-wrap: nowrap;"></div>
                <button id="plot-tab-add-btn" 
                        style="padding: 8px 14px; background: linear-gradient(135deg, rgba(124, 111, 255, 0.2), rgba(124, 111, 255, 0.2)); border: 1px solid rgba(124, 111, 255, 0.4); border-radius: 6px; color: #b0a8ff; font-size: 16px; font-weight: 600; cursor: pointer; min-width: 36px; transition: all 0.25s ease; box-shadow: 0 2px 6px rgba(0, 0, 0, 0.2); flex-shrink: 0;"
                        title="Add new plot tab">+</button>
            </div>
        `;
        
        // + 버튼 이벤트 리스너
        const addBtn = document.getElementById('plot-tab-add-btn');
        addBtn.addEventListener('click', () => {
            this.createTab();
        });
        
        // + 버튼 호버 효과
        addBtn.addEventListener('mouseenter', () => {
            addBtn.style.background = 'linear-gradient(135deg, rgba(124, 111, 255, 0.35), rgba(124, 111, 255, 0.35))';
            addBtn.style.borderColor = 'rgba(124, 111, 255, 0.7)';
            addBtn.style.transform = 'scale(1.05)';
            addBtn.style.boxShadow = '0 4px 12px rgba(124, 111, 255, 0.3)';
        });
        addBtn.addEventListener('mouseleave', () => {
            addBtn.style.background = 'linear-gradient(135deg, rgba(124, 111, 255, 0.2), rgba(124, 111, 255, 0.2))';
            addBtn.style.borderColor = 'rgba(124, 111, 255, 0.4)';
            addBtn.style.transform = 'scale(1)';
            addBtn.style.boxShadow = '0 2px 6px rgba(0, 0, 0, 0.2)';
        });
    }

    createTab(title = null) {
        // 최대 탭 개수 체크
        if (this.tabs.length >= this.maxTabs) {
            console.warn(`[PlotTabManager] Maximum number of tabs reached (${this.maxTabs})`);
            alert(`최대 ${this.maxTabs}개의 Plot 탭만 생성할 수 있습니다.`);
            return null;
        }
        
        const tabId = `tab-${this.nextTabId++}`;
        const defaultTitle = title || `Plot ${this.nextTabId - 1}`;
        
        // Plot div 생성 (각 탭별 독립적인 div)
        const plotDivId = `plot-area-${tabId}`;
        const plotDiv = document.createElement('div');
        plotDiv.id = plotDivId;
        plotDiv.style.width = '100%';
        plotDiv.style.height = '100%';
        plotDiv.style.display = 'none';  // 초기에는 숨김
        plotDiv.style.backgroundColor = 'rgba(0, 0, 0, 0.2)';
        plotDiv.style.borderRadius = '8px';
        this.plotAreaContainer.appendChild(plotDiv);
        
        // PlotlyPlotManager 인스턴스 생성
        const plotManager = new PlotlyPlotManager(plotDivId, this.bufferTime);
        
        // 탭 객체 생성
        const tab = {
            id: tabId,
            title: defaultTitle,
            plotManager: plotManager,
            plotDiv: plotDiv,
            plotDivId: plotDivId
        };
        
        this.tabs.push(tab);
        
        // 탭 UI 생성
        this.renderTabUI(tab);
        
        // 새 탭을 즉시 활성화
        this.switchTab(tabId);
        
        // 빈 plot 초기화 (탭 활성화 후)
        setTimeout(() => {
            plotManager.initEmptyPlot();
        }, 100);  // switchTab 후 DOM이 업데이트될 시간 확보
        
        console.log(`[PlotTabManager] Created tab: ${tabId} (${defaultTitle})`);
        return tab;
    }

    renderTabUI(tab) {
        const tabList = document.getElementById('plot-tab-list');
        if (!tabList) return;
        
        const tabElement = document.createElement('div');
        tabElement.id = `plot-tab-ui-${tab.id}`;
        tabElement.style.cssText = `
            display: flex;
            align-items: center;
            gap: 4px;
            padding: 8px 6px;
            background: linear-gradient(135deg, rgba(30, 30, 40, 0.95), rgba(25, 25, 35, 0.95));
            border: 1px solid rgba(124, 111, 255, 0.15);
            border-bottom: none;
            border-radius: 8px 8px 0 0;
            cursor: pointer;
            position: relative;
            flex: 1 1 0;
            min-width: 30px;
            max-width: 180px;
            overflow: hidden;
            transition: all 0.25s cubic-bezier(0.4, 0, 0.2, 1);
            box-shadow: 0 2px 6px rgba(0, 0, 0, 0.2);
        `;
        
        // 탭 제목
        const titleSpan = document.createElement('span');
        titleSpan.id = `plot-tab-title-${tab.id}`;
        titleSpan.textContent = tab.title;
        titleSpan.style.cssText = `
            flex: 1 1 0;
            min-width: 0;
            overflow: hidden;
            text-overflow: ellipsis;
            white-space: nowrap;
            font-size: 11px;
            font-weight: 500;
            color: rgba(255, 255, 255, 0.85);
            transition: color 0.2s ease;
        `;
        
        // 탭 제목 더블클릭 이벤트 (편집 모드)
        titleSpan.addEventListener('dblclick', (e) => {
            e.stopPropagation();
            this.editTabTitle(tab.id);
        });
        
        tabElement.appendChild(titleSpan);

        // 닫기 버튼 (탭이 1개일 때도 표시 — 마지막 탭은 플롯만 초기화)
        const closeBtn = document.createElement('button');
        closeBtn.id = `plot-tab-close-${tab.id}`;
        closeBtn.textContent = '×';
        closeBtn.style.cssText = `
            padding: 0;
            width: 16px;
            height: 16px;
            background: transparent;
            border: none;
            color: rgba(255, 255, 255, 0.6);
            font-size: 16px;
            cursor: pointer;
            display: flex;
            align-items: center;
            justify-content: center;
            border-radius: 3px;
            transition: all 0.2s ease;
            flex-shrink: 0;
        `;
        closeBtn.title = 'Close tab';

        closeBtn.addEventListener('mouseenter', () => {
            closeBtn.style.background = 'rgba(255, 70, 85, 0.25)';
            closeBtn.style.color = 'rgba(255, 255, 255, 0.95)';
            closeBtn.style.transform = 'scale(1.1)';
        });
        closeBtn.addEventListener('mouseleave', () => {
            closeBtn.style.background = 'transparent';
            closeBtn.style.color = 'rgba(255, 255, 255, 0.6)';
            closeBtn.style.transform = 'scale(1)';
        });
        closeBtn.addEventListener('click', (e) => {
            e.stopPropagation();
            this.closeTab(tab.id);
        });

        tabElement.appendChild(closeBtn);
        
        // 탭 클릭 이벤트 (전환)
        tabElement.addEventListener('click', () => {
            this.switchTab(tab.id);
        });
        
        // 탭 호버 이벤트
        tabElement.addEventListener('mouseenter', () => {
            if (this.activeTabId !== tab.id) {
                tabElement.style.background = 'linear-gradient(135deg, rgba(40, 45, 60, 0.95), rgba(35, 40, 55, 0.95))';
                tabElement.style.borderColor = 'rgba(124, 111, 255, 0.3)';
                tabElement.style.transform = 'translateY(-2px)';
                tabElement.style.boxShadow = '0 4px 12px rgba(124, 111, 255, 0.15)';
                titleSpan.style.color = 'rgba(255, 255, 255, 0.95)';
            }
        });
        tabElement.addEventListener('mouseleave', () => {
            if (this.activeTabId !== tab.id) {
                tabElement.style.background = 'linear-gradient(135deg, rgba(30, 30, 40, 0.95), rgba(25, 25, 35, 0.95))';
                tabElement.style.borderColor = 'rgba(124, 111, 255, 0.15)';
                tabElement.style.transform = 'translateY(0)';
                tabElement.style.boxShadow = '0 2px 6px rgba(0, 0, 0, 0.2)';
                titleSpan.style.color = 'rgba(255, 255, 255, 0.85)';
            }
        });
        
        tabList.appendChild(tabElement);
        
        // 닫기 버튼 표시/숨기기 업데이트
        this.updateCloseButtons();
        
        // 탭 너비 동적 조정
        this.updateTabWidths();
        
        // + 버튼 상태 업데이트
        this.updateAddButton();
    }

    switchTab(tabId) {
        const tab = this.tabs.find(t => t.id === tabId);
        if (!tab) {
            console.warn(`[PlotTabManager] Tab not found: ${tabId}`);
            return;
        }
        
        // 이전 활성 탭 숨기기
        if (this.activeTabId) {
            const prevTab = this.tabs.find(t => t.id === this.activeTabId);
            if (prevTab) {
                prevTab.plotDiv.style.display = 'none';
                const prevTabUI = document.getElementById(`plot-tab-ui-${prevTab.id}`);
                if (prevTabUI) {
                    prevTabUI.style.background = 'linear-gradient(135deg, rgba(30, 30, 40, 0.95), rgba(25, 25, 35, 0.95))';
                    prevTabUI.style.borderColor = 'rgba(124, 111, 255, 0.15)';
                    prevTabUI.style.borderBottom = 'none';
                    prevTabUI.style.transform = 'translateY(0)';
                    prevTabUI.style.boxShadow = '0 2px 6px rgba(0, 0, 0, 0.2)';
                    
                    const prevTitleSpan = document.getElementById(`plot-tab-title-${prevTab.id}`);
                    if (prevTitleSpan) {
                        prevTitleSpan.style.color = 'rgba(255, 255, 255, 0.85)';
                    }
                }
            }
        }
        
        // 새 활성 탭 표시
        this.activeTabId = tabId;
        tab.plotDiv.style.display = 'block';
        
        const tabUI = document.getElementById(`plot-tab-ui-${tabId}`);
        if (tabUI) {
            tabUI.style.background = 'linear-gradient(135deg, rgba(124, 111, 255, 0.25), rgba(124, 111, 255, 0.25))';
            tabUI.style.borderColor = 'rgba(124, 111, 255, 0.6)';
            tabUI.style.borderBottom = '3px solid rgba(124, 111, 255, 0.9)';
            tabUI.style.transform = 'translateY(0)';
            tabUI.style.boxShadow = '0 4px 16px rgba(124, 111, 255, 0.3), inset 0 1px 0 rgba(255, 255, 255, 0.1)';
            
            const titleSpan = document.getElementById(`plot-tab-title-${tabId}`);
            if (titleSpan) {
                titleSpan.style.color = 'rgba(255, 255, 255, 1)';
            }
        }
        
        console.log(`[PlotTabManager] Switched to tab: ${tabId}`);

        // 탭 전환 직후 표시되는 Plot div 크기에 맞게 Plotly 리사이즈
        if (typeof window.resizeVisiblePlotlyPlots === 'function') {
            requestAnimationFrame(() => {
                window.resizeVisiblePlotlyPlots();
            });
        }
    }

    closeTab(tabId) {
        const tabIndex = this.tabs.findIndex((t) => t.id === tabId);
        if (tabIndex === -1) {
            console.warn(`[PlotTabManager] Tab not found: ${tabId}`);
            return;
        }

        const tab = this.tabs[tabIndex];

        // 마지막 탭: 탭 UI는 유지하고 드래그 안내 초기 화면만 복원
        if (this.tabs.length === 1) {
            if (typeof window.releasePlotPathsFromPlotManager === 'function') {
                window.releasePlotPathsFromPlotManager(this, tab.plotManager);
            }
            if (tab.plotManager) {
                tab.plotManager.destroy();
                tab.plotManager.initEmptyPlot();
            }
            if (typeof window.resizeVisiblePlotlyPlots === 'function') {
                requestAnimationFrame(() => window.resizeVisiblePlotlyPlots());
            }
            try {
                this.saveState();
            } catch (e) {
                console.warn('[PlotTabManager] saveState after reset:', e);
            }
            console.log('[PlotTabManager] Last tab cleared to empty plot placeholder');
            return;
        }

        if (typeof window.releasePlotPathsFromPlotManager === 'function') {
            window.releasePlotPathsFromPlotManager(this, tab.plotManager);
        }
        if (tab.plotManager) {
            tab.plotManager.destroy();
        }

        // Plot div 제거
        if (tab.plotDiv && tab.plotDiv.parentNode) {
            tab.plotDiv.parentNode.removeChild(tab.plotDiv);
        }
        
        // 탭 UI 제거
        const tabUI = document.getElementById(`plot-tab-ui-${tabId}`);
        if (tabUI && tabUI.parentNode) {
            tabUI.parentNode.removeChild(tabUI);
        }
        
        // 탭 배열에서 제거
        this.tabs.splice(tabIndex, 1);
        
        // 삭제된 탭이 활성 탭이었으면 다른 탭 활성화
        if (this.activeTabId === tabId) {
            // 이전 탭 또는 다음 탭 활성화
            const nextTabIndex = Math.max(0, tabIndex - 1);
            if (this.tabs.length > 0) {
                this.switchTab(this.tabs[nextTabIndex].id);
            }
        }
        
        // 닫기 버튼 표시/숨기기 업데이트
        this.updateCloseButtons();
        
        // 탭 너비 동적 조정
        this.updateTabWidths();
        
        // + 버튼 상태 업데이트
        this.updateAddButton();
        
        console.log(`[PlotTabManager] Closed tab: ${tabId}`);
    }

    editTabTitle(tabId) {
        const tab = this.tabs.find(t => t.id === tabId);
        if (!tab) return;
        
        const titleSpan = document.getElementById(`plot-tab-title-${tabId}`);
        if (!titleSpan) return;
        
        const currentTitle = tab.title;
        
        // 인라인 편집 UI 생성
        const input = document.createElement('input');
        input.type = 'text';
        input.value = currentTitle;
        input.style.cssText = `
            flex: 1;
            padding: 2px 4px;
            background: rgba(0, 0, 0, 0.5);
            border: 1px solid var(--primary);
            border-radius: 3px;
            color: var(--text);
            font-size: 13px;
            outline: none;
        `;
        
        // Enter 키 또는 포커스 아웃 시 저장
        const saveTitle = () => {
            const newTitle = input.value.trim();
            if (newTitle === '') {
                // 빈 제목은 디폴트 제목으로 복원
                tab.title = `Plot ${tabId.replace('tab-', '')}`;
            } else {
                tab.title = newTitle;
            }
            
            // 원래 span으로 복원
            titleSpan.textContent = tab.title;
            titleSpan.style.display = 'block';
            input.remove();
            
            console.log(`[PlotTabManager] Tab title updated: ${tabId} -> ${tab.title}`);
        };
        
        input.addEventListener('keydown', (e) => {
            if (e.key === 'Enter') {
                saveTitle();
            } else if (e.key === 'Escape') {
                // ESC 키 시 취소
                titleSpan.style.display = 'block';
                input.remove();
            }
        });
        
        input.addEventListener('blur', saveTitle);
        
        // Span 숨기고 input 추가
        titleSpan.style.display = 'none';
        titleSpan.parentNode.insertBefore(input, titleSpan.nextSibling);
        input.focus();
        input.select();
    }

    updateCloseButtons() {
        this.tabs.forEach((tab) => {
            const closeBtn = document.getElementById(`plot-tab-close-${tab.id}`);
            if (closeBtn) {
                closeBtn.style.display = 'flex';
            }
        });
    }

    updateTabWidths() {
        // CSS flex로 자동 조정되므로 별도 작업 불필요
        // 탭이 많아지면 자동으로 flex: 1 1 0으로 균등 분배됨
        console.log(`[PlotTabManager] Tab widths auto-adjusted by CSS flex (${this.tabs.length} tabs)`);
    }

    updateAddButton() {
        // + 버튼 상태 업데이트 (최대 탭 개수 체크)
        const addBtn = document.getElementById('plot-tab-add-btn');
        if (!addBtn) return;
        
        if (this.tabs.length >= this.maxTabs) {
            // 최대 개수에 도달하면 버튼 비활성화
            addBtn.disabled = true;
            addBtn.style.opacity = '0.4';
            addBtn.style.cursor = 'not-allowed';
            addBtn.title = `최대 ${this.maxTabs}개의 Plot 탭만 생성할 수 있습니다`;
        } else {
            // 정상 상태로 복구
            addBtn.disabled = false;
            addBtn.style.opacity = '1';
            addBtn.style.cursor = 'pointer';
            addBtn.title = 'Add new plot tab';
        }
    }

    getActiveTab() {
        return this.tabs.find(t => t.id === this.activeTabId);
    }

    getActivePlotManager() {
        const tab = this.getActiveTab();
        return tab ? tab.plotManager : null;
    }

    getPlotManager(plotId) {
        // plotId는 containerId와 동일 (예: "plot-container-tab-1")
        const tab = this.tabs.find(t => t.plotManager && t.plotManager.containerId === plotId);
        return tab ? tab.plotManager : null;
    }

    setBufferTime(bufferTime) {
        this.bufferTime = bufferTime;
        // 모든 탭의 PlotlyPlotManager에 버퍼 시간 설정
        this.tabs.forEach(tab => {
            if (tab.plotManager) {
                tab.plotManager.setBufferTime(bufferTime);
            }
        });
        console.log(`[PlotTabManager] Buffer time updated: ${bufferTime}s`);
    }

    // 페이지 로드 타입 확인 (일반 새로고침 vs 일반 로드)
    getNavigationType() {
        try {
            // Modern API (PerformanceNavigationTiming)
            const navEntries = performance.getEntriesByType('navigation');
            if (navEntries && navEntries.length > 0) {
                const navType = navEntries[0].type;
                console.log('[PlotTabManager.getNavigationType] Modern API:', navType);
                // 'reload': F5 새로고침
                // 'navigate': 일반 로드 (새 탭, 주소 입력 등)
                // 'back_forward': 뒤로/앞으로 가기
                return navType;
            }
            
            // Fallback to deprecated API
            const perfNav = performance.navigation;
            if (perfNav) {
                const navType = perfNav.type;
                console.log('[PlotTabManager.getNavigationType] Deprecated API:', navType);
                // 0: TYPE_NAVIGATE (일반 로드)
                // 1: TYPE_RELOAD (새로고침)
                // 2: TYPE_BACK_FORWARD (뒤로/앞으로 가기)
                if (navType === 1) return 'reload';
                if (navType === 2) return 'back_forward';
                return 'navigate';
            }
            
            // API를 사용할 수 없으면 기본값 (안전하게 'navigate' 반환)
            console.warn('[PlotTabManager.getNavigationType] Performance API not available');
            return 'navigate';
        } catch (error) {
            console.error('[PlotTabManager.getNavigationType] Error:', error);
            return 'navigate';  // 안전하게 'navigate' 반환
        }
    }

    // 탭 상태 저장 (beforeunload 이벤트에서만 저장 - F5 새로고침만 복원)
    saveState() {
        try {
            const state = {
                activeTabId: this.activeTabId,
                nextTabId: this.nextTabId,
                bufferTime: this.bufferTime,
                timestamp: Date.now(),  // 저장 시간 기록
                tabs: this.tabs.map(tab => {
                    const plottedPaths = tab.plotManager && tab.plotManager.dataBuffers 
                        ? Array.from(tab.plotManager.dataBuffers.keys()) 
                        : [];
                    console.log(`[PlotTabManager.saveState] Tab ${tab.id} has ${plottedPaths.length} path(s):`, plottedPaths);
                    return {
                        id: tab.id,
                        title: tab.title,
                        plottedPaths: plottedPaths
                    };
                })
            };
            
            sessionStorage.setItem('plotTabManagerState', JSON.stringify(state));
            console.log('[PlotTabManager.saveState] State saved with timestamp:', state.timestamp);
        } catch (error) {
            console.error('[PlotTabManager.saveState] Failed to save state:', error);
        }
    }

    // 탭 상태 복원 (SessionStorage - 일반 새로고침 시에만 복원)
    loadState() {
        try {
            // ⚠️ 중요: Navigation Type 확인 - 일반 새로고침(reload)일 때만 복원
            const navType = this.getNavigationType();
            console.log('[PlotTabManager.loadState] Navigation type:', navType);
            
            // 'reload'가 아니면 즉시 초기화
            if (navType !== 'reload') {
                console.log('[PlotTabManager] Not a reload, starting fresh (clearing saved state)');
                sessionStorage.removeItem('plotTabManagerState');
                return false;
            }
            
            const savedState = sessionStorage.getItem('plotTabManagerState');
            
            // 저장된 상태가 없으면 기본 상태
            if (!savedState) {
                console.log('[PlotTabManager] No saved state found');
                return false;
            }

            const state = JSON.parse(savedState);
            
            // ⚠️ timestamp 체크: 1초 이내의 새로고침만 복원
            // F5 새로고침은 매우 빠르게 발생 (100-500ms)
            // 하드 리프레시나 런치 재실행은 1초 이상 소요
            const now = Date.now();
            const timeDiff = now - (state.timestamp || 0);
            const MAX_RESTORE_TIME = 1000;  // 1초
            
            if (timeDiff > MAX_RESTORE_TIME) {
                console.log(`[PlotTabManager] State too old (${timeDiff}ms), starting fresh`);
                sessionStorage.removeItem('plotTabManagerState');
                return false;
            }
            
            console.log(`[PlotTabManager] Loading saved state (${timeDiff}ms ago):`, state);
            
            // 복원 직후 즉시 SessionStorage 삭제
            sessionStorage.removeItem('plotTabManagerState');

            // nextTabId 복원
            this.nextTabId = state.nextTabId || 1;
            this.bufferTime = state.bufferTime || 5.0;

            // 기존 탭 제거 (디폴트 탭)
            this.tabs.forEach(tab => {
                if (tab.plotDiv && tab.plotDiv.parentNode) {
                    tab.plotDiv.parentNode.removeChild(tab.plotDiv);
                }
                const tabUI = document.getElementById(`plot-tab-ui-${tab.id}`);
                if (tabUI && tabUI.parentNode) {
                    tabUI.parentNode.removeChild(tabUI);
                }
            });
            this.tabs = [];

            // 저장된 탭 복원
            if (state.tabs && state.tabs.length > 0) {
                state.tabs.forEach(tabInfo => {
                    const tab = this.createTabFromState(tabInfo);
                    // 탭 생성 시 saveState 호출을 방지하기 위해 직접 추가
                });

                // 활성 탭 복원
                if (state.activeTabId) {
                    this.switchTab(state.activeTabId);
                } else if (this.tabs.length > 0) {
                    this.switchTab(this.tabs[0].id);
                }

                console.log(`[PlotTabManager] Restored ${this.tabs.length} tab(s)`);
                return true;
            }

            return false;
        } catch (error) {
            console.error('[PlotTabManager] Failed to load state:', error);
            return false;
        }
    }

    // 저장된 상태에서 탭 생성
    createTabFromState(tabInfo) {
        const tabId = tabInfo.id;
        const title = tabInfo.title;
        
        // Plot div 생성
        const plotDivId = `plot-area-${tabId}`;
        const plotDiv = document.createElement('div');
        plotDiv.id = plotDivId;
        plotDiv.style.width = '100%';
        plotDiv.style.height = '100%';
        plotDiv.style.display = 'none';
        plotDiv.style.backgroundColor = 'rgba(0, 0, 0, 0.2)';
        plotDiv.style.borderRadius = '8px';
        this.plotAreaContainer.appendChild(plotDiv);
        
        // PlotlyPlotManager 인스턴스 생성
        const plotManager = new PlotlyPlotManager(plotDivId, this.bufferTime);
        
        // 탭 객체 생성
        const tab = {
            id: tabId,
            title: title,
            plotManager: plotManager,
            plotDiv: plotDiv,
            plotDivId: plotDivId,
            savedPaths: tabInfo.plottedPaths || []  // 저장된 paths (나중에 복원)
        };
        
        this.tabs.push(tab);
        
        // 탭 UI 생성
        this.renderTabUI(tab);
        
        console.log(`[PlotTabManager] Restored tab: ${tabId} (${title})`);
        return tab;
    }

    // 저장된 paths를 활성 탭에 복원 (드래그 앤 드롭 시뮬레이션)
    restorePlottedPaths(tabId) {
        const tab = this.tabs.find(t => t.id === tabId);
        if (!tab || !tab.savedPaths || tab.savedPaths.length === 0) {
            console.log(`[PlotTabManager.restorePlottedPaths] No saved paths for tab ${tabId}`);
            return;
        }

        console.log(`[PlotTabManager.restorePlottedPaths] Restoring ${tab.savedPaths.length} path(s) for tab ${tabId}:`, tab.savedPaths);
        
        // PlotlyPlotManager에 paths 추가 (createPlot 호출)
        if (tab.plotManager) {
            const success = tab.plotManager.createPlot(tab.savedPaths);
            console.log(`[PlotTabManager.restorePlottedPaths] createPlot success:`, success);
            
            // plotState.plottedPaths에도 추가 (중복 제거)
            if (window.plotState) {
                const newPaths = tab.savedPaths.filter(p => !window.plotState.plottedPaths.includes(p));
                window.plotState.plottedPaths = window.plotState.plottedPaths.concat(newPaths);
                console.log(`[PlotTabManager.restorePlottedPaths] Added ${newPaths.length} new paths to global plottedPaths`);
                
                // 각 path에 대해 실시간 데이터 업데이트 설정
                newPaths.forEach(path => {
                    if (window.setupPlotDataUpdate) {
                        console.log(`[PlotTabManager.restorePlottedPaths] Setting up data update for path: ${path}`);
                        window.setupPlotDataUpdate(path);
                    }
                });
            }
        } else {
            console.error(`[PlotTabManager.restorePlottedPaths] plotManager is null for tab ${tabId}`);
        }
        
        // savedPaths 제거 (이미 복원됨)
        delete tab.savedPaths;
    }

    // 상태 초기화 (SessionStorage 삭제)
    clearState() {
        try {
            sessionStorage.removeItem('plotTabManagerState');
            console.log('[PlotTabManager] State cleared from SessionStorage');
        } catch (error) {
            console.error('[PlotTabManager] Failed to clear state:', error);
        }
    }
}
