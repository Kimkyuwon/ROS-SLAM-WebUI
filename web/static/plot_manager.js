// DataBuffer 클래스 - 시계열 데이터 버퍼 관리 (ROS time 기준)
class DataBuffer {
    constructor(bufferTime = 5.0) {
        this.bufferTime = bufferTime;  // 버퍼 시간 (초 단위)
        this.timestamps = [];          // X축 데이터 (ROS time, offset 적용 전)
        this.values = [];              // Y축 데이터
        this.lastTimestamp = null;     // 마지막 timestamp (시간 역행 감지용)
        this.timestampContinuityOffset = 0;  // timestamp 연속성 offset (bag 일시정지 후 재생 시)
    }

    addData(timestamp, value) {
        // 시간 역행 감지: 새로운 timestamp가 이전보다 1초 이상 작으면 버퍼 초기화
        // (작은 시간 오차는 무시, bag 파일 재시작 등 큰 역행만 감지)
        if (this.lastTimestamp !== null && timestamp < this.lastTimestamp - 1.0) {
            console.warn(`[DataBuffer] Time reversal detected: ${this.lastTimestamp.toFixed(3)} -> ${timestamp.toFixed(3)} (diff: ${(timestamp - this.lastTimestamp).toFixed(3)}s)`);
            this.clear();
            return { timeReversal: true };
        }
        
        // bag 일시정지 후 재생 감지: 큰 시간 gap (1.0초 이상, 시간 역행은 아님)
        // 이 경우 timestamp continuity offset을 적용하여 연속적으로 보이게 함
        if (this.lastTimestamp !== null && timestamp > this.lastTimestamp + 1.0) {
            const gap = timestamp - this.lastTimestamp;
            console.log(`[DataBuffer] Large time gap detected: ${gap.toFixed(3)}s (likely bag pause/resume)`);
            console.log(`[DataBuffer] Applying continuity offset to maintain plot continuity`);
            // offset 누적: 현재 gap만큼 빼서 연속적으로 보이게 함
            this.timestampContinuityOffset += gap;
            console.log(`[DataBuffer] Cumulative continuity offset: ${this.timestampContinuityOffset.toFixed(3)}s`);
        }
        
        this.lastTimestamp = timestamp;
        // offset을 적용한 timestamp 저장 (연속성 유지)
        const adjustedTimestamp = timestamp - this.timestampContinuityOffset;
        this.timestamps.push(adjustedTimestamp);
        this.values.push(value);

        // ROS time 기준: 현재 adjustedTimestamp - bufferTime보다 오래된 데이터 삭제
        const cutoffTime = adjustedTimestamp - this.bufferTime;
        while (this.timestamps.length > 0 && this.timestamps[0] < cutoffTime) {
            this.timestamps.shift();
            this.values.shift();
        }
        
        return { timeReversal: false };
    }

    getData() {
        return {
            timestamps: [...this.timestamps],
            values: [...this.values]
        };
    }

    /**
     * 다운샘플링된 데이터 반환 (성능 최적화)
     * 최대/최소 값을 항상 포함하여 스케일 안정성 유지
     * @param {number} maxPoints - 최대 포인트 수 (기본: 500, 10ms 이하 지연)
     * @param {boolean} forceFullData - 강제로 모든 데이터 반환 (일시정지 시)
     * @returns {object} - {timestamps, values}
     */
    getDownsampledData(maxPoints = 500, forceFullData = false) {
        const length = this.timestamps.length;
        
        // 강제 전체 데이터 모드 (일시정지 시)
        if (forceFullData) {
            return {
                timestamps: this.timestamps,  // 모든 데이터 직접 참조
                values: this.values
            };
        }
        
        // 데이터가 maxPoints 이하면 복사 없이 직접 반환
        if (length <= maxPoints) {
            return {
                timestamps: this.timestamps,  // 복사 없이 직접 참조
                values: this.values
            };
        }
        
        // 🎯 최대/최소 값 찾기 (스케일 안정성 위해)
        let minIndex = 0;
        let maxIndex = 0;
        let minValue = this.values[0];
        let maxValue = this.values[0];
        
        for (let i = 1; i < length; i++) {
            if (this.values[i] < minValue) {
                minValue = this.values[i];
                minIndex = i;
            }
            if (this.values[i] > maxValue) {
                maxValue = this.values[i];
                maxIndex = i;
            }
        }
        
        // Uniform downsampling (균등 샘플링) + 최대/최소 값 포함
        const step = length / maxPoints;
        const downsampledIndices = new Set();  // 중복 방지
        
        // 첫 번째 포인트
        downsampledIndices.add(0);
        
        // 최대/최소 값 (항상 포함)
        downsampledIndices.add(minIndex);
        downsampledIndices.add(maxIndex);
        
        // Uniform sampling
        for (let i = 1; i < maxPoints - 1; i++) {
            const index = Math.floor(i * step);
            downsampledIndices.add(index);
        }
        
        // 마지막 포인트 (최신 데이터)
        downsampledIndices.add(length - 1);
        
        // 인덱스 정렬 후 데이터 추출 (시간 순서 유지)
        const sortedIndices = Array.from(downsampledIndices).sort((a, b) => a - b);
        const downsampledTimestamps = sortedIndices.map(i => this.timestamps[i]);
        const downsampledValues = sortedIndices.map(i => this.values[i]);
        
        return {
            timestamps: downsampledTimestamps,
            values: downsampledValues
        };
    }

    getLength() {
        return this.timestamps.length;
    }

    clear() {
        this.timestamps = [];
        this.values = [];
        this.lastTimestamp = null;
        this.timestampContinuityOffset = 0;  // offset도 리셋
    }

    isEmpty() {
        return this.timestamps.length === 0;
    }

    setBufferTime(bufferTime) {
        this.bufferTime = bufferTime;
        
        // 현재 가장 최신 timestamp 기준으로 오래된 데이터 삭제
        if (this.timestamps.length > 0) {
            const latestTimestamp = this.timestamps[this.timestamps.length - 1];
            const cutoffTime = latestTimestamp - this.bufferTime;
            
            while (this.timestamps.length > 0 && this.timestamps[0] < cutoffTime) {
                this.timestamps.shift();
                this.values.shift();
            }
        }
    }
}

// PlotDataFilter 클래스 - 시계열 데이터 필터 적용 (정적 메서드 모음)
class PlotDataFilter {

    /**
     * 타임스탬프 배열에서 중앙값 dt(샘플 간격)를 추정한다.
     * 연속한 두 타임스탬프 간 차이들의 중앙값을 반환한다.
     * @param {number[]} timestamps - 타임스탬프 배열 (초 단위)
     * @returns {number} 중앙값 dt (초 단위). 데이터가 2개 미만이면 1.0 반환.
     */
    static guessMedianDT(timestamps) {
        if (!timestamps || timestamps.length < 2) {
            return 1.0;
        }
        const diffs = [];
        for (let i = 1; i < timestamps.length; i++) {
            const dt = timestamps[i] - timestamps[i - 1];
            if (dt > 0) {
                diffs.push(dt);
            }
        }
        if (diffs.length === 0) {
            return 1.0;
        }
        diffs.sort((a, b) => a - b);
        const mid = Math.floor(diffs.length / 2);
        return diffs.length % 2 === 0
            ? (diffs[mid - 1] + diffs[mid]) / 2
            : diffs[mid];
    }

    /**
     * 절댓값 필터: 각 값에 Math.abs() 적용
     * @param {number[]} timestamps - 타임스탬프 배열
     * @param {number[]} values - 값 배열
     * @returns {{timestamps: number[], values: number[]}}
     */
    static applyAbsolute(timestamps, values) {
        return {
            timestamps: [...timestamps],
            values: values.map(v => Math.abs(v))
        };
    }

    /**
     * 미분 필터: 연속 샘플 간 변화율 (dy/dt) 계산
     * @param {number[]} timestamps - 타임스탬프 배열
     * @param {number[]} values - 값 배열
     * @param {object} params
     * @param {boolean} params.useActual - true면 실제 dt 사용, false면 customDT 사용
     * @param {number}  params.customDT  - useActual=false 일 때 사용할 고정 dt (초)
     * @returns {{timestamps: number[], values: number[]}}
     */
    static applyDerivative(timestamps, values, { useActual = true, customDT = 1.0 } = {}) {
        if (timestamps.length < 2) {
            return { timestamps: [...timestamps], values: [...values] };
        }
        const outTimestamps = [];
        const outValues = [];
        for (let i = 1; i < timestamps.length; i++) {
            const dy = values[i] - values[i - 1];
            const dt = useActual
                ? (timestamps[i] - timestamps[i - 1]) || customDT
                : customDT;
            // dt가 0이면 스킵 (중복 타임스탬프)
            if (dt === 0) continue;
            outTimestamps.push(timestamps[i]);
            outValues.push(dy / dt);
        }
        return { timestamps: outTimestamps, values: outValues };
    }

    /**
     * 이동 평균 필터
     * @param {number[]} timestamps - 타임스탬프 배열
     * @param {number[]} values - 값 배열
     * @param {object} params
     * @param {number}  params.samplesCount     - 윈도우 샘플 수 (홀수 권장)
     * @param {boolean} params.compensateOffset - true면 윈도우 중앙 타임스탬프 사용, false면 현재 타임스탬프
     * @returns {{timestamps: number[], values: number[]}}
     */
    static applyMovingAverage(timestamps, values, { samplesCount = 10, compensateOffset = false } = {}) {
        const n = Math.max(1, Math.round(samplesCount));
        const outTimestamps = [];
        const outValues = [];
        for (let i = n - 1; i < timestamps.length; i++) {
            let sum = 0;
            for (let j = i - n + 1; j <= i; j++) {
                sum += values[j];
            }
            outValues.push(sum / n);
            // compensateOffset: 윈도우 시작과 끝의 중간 타임스탬프 사용
            if (compensateOffset) {
                const startIdx = i - n + 1;
                outTimestamps.push((timestamps[startIdx] + timestamps[i]) / 2);
            } else {
                outTimestamps.push(timestamps[i]);
            }
        }
        return { timestamps: outTimestamps, values: outValues };
    }

    /**
     * 이동 RMS (Root Mean Square) 필터
     * @param {number[]} timestamps - 타임스탬프 배열
     * @param {number[]} values - 값 배열
     * @param {object} params
     * @param {number}  params.samplesCount - 윈도우 샘플 수
     * @returns {{timestamps: number[], values: number[]}}
     */
    static applyMovingRMS(timestamps, values, { samplesCount = 10 } = {}) {
        const n = Math.max(1, Math.round(samplesCount));
        const outTimestamps = [];
        const outValues = [];
        for (let i = n - 1; i < timestamps.length; i++) {
            let sumSq = 0;
            for (let j = i - n + 1; j <= i; j++) {
                sumSq += values[j] * values[j];
            }
            outValues.push(Math.sqrt(sumSq / n));
            outTimestamps.push(timestamps[i]);
        }
        return { timestamps: outTimestamps, values: outValues };
    }

    /**
     * 이동 분산 / 표준편차 필터
     * @param {number[]} timestamps - 타임스탬프 배열
     * @param {number[]} values - 값 배열
     * @param {object} params
     * @param {number}  params.windowSize      - 윈도우 샘플 수
     * @param {boolean} params.applySquareRoot - true면 표준편차(sqrt(variance)) 반환, false면 분산 반환
     * @returns {{timestamps: number[], values: number[]}}
     */
    static applyMovingVariance(timestamps, values, { windowSize = 10, applySquareRoot = false } = {}) {
        const n = Math.max(1, Math.round(windowSize));
        const outTimestamps = [];
        const outValues = [];
        for (let i = n - 1; i < timestamps.length; i++) {
            let sum = 0;
            for (let j = i - n + 1; j <= i; j++) {
                sum += values[j];
            }
            const mean = sum / n;
            let sumSqDiff = 0;
            for (let j = i - n + 1; j <= i; j++) {
                const diff = values[j] - mean;
                sumSqDiff += diff * diff;
            }
            const variance = sumSqDiff / n;
            outValues.push(applySquareRoot ? Math.sqrt(variance) : variance);
            outTimestamps.push(timestamps[i]);
        }
        return { timestamps: outTimestamps, values: outValues };
    }

    /**
     * 스케일 & 오프셋 필터: value = (원래값 × valueMultiplier) + valueOffset, timestamp += timeOffset
     * @param {number[]} timestamps - 타임스탬프 배열
     * @param {number[]} values - 값 배열
     * @param {object} params
     * @param {number} params.timeOffset      - 타임스탬프에 더할 오프셋 (초)
     * @param {number} params.valueOffset     - 값에 더할 오프셋
     * @param {number} params.valueMultiplier - 값에 곱할 스케일
     * @returns {{timestamps: number[], values: number[]}}
     */
    static applyScaleOffset(timestamps, values, { timeOffset = 0.0, valueOffset = 0.0, valueMultiplier = 1.0 } = {}) {
        return {
            timestamps: timestamps.map(t => t + timeOffset),
            values: values.map(v => v * valueMultiplier + valueOffset)
        };
    }

    /**
     * filterConfig 객체를 기반으로 적절한 필터를 디스패치한다.
     * trace.filterConfig = { type: '...', params: {...} } 형식을 사용한다.
     * @param {number[]} timestamps - 타임스탬프 배열
     * @param {number[]} values - 값 배열
     * @param {{type: string, params: object}} filterConfig - 필터 설정
     * @returns {{timestamps: number[], values: number[]}}
     */
    static applyByConfig(timestamps, values, filterConfig) {
        if (!filterConfig) return { timestamps: [...timestamps], values: [...values] };
        switch (filterConfig.type) {
            case 'absolute':      return PlotDataFilter.applyAbsolute(timestamps, values);
            case 'derivative':    return PlotDataFilter.applyDerivative(timestamps, values, filterConfig.params);
            case 'movingAverage': return PlotDataFilter.applyMovingAverage(timestamps, values, filterConfig.params);
            case 'movingRMS':     return PlotDataFilter.applyMovingRMS(timestamps, values, filterConfig.params);
            case 'movingVariance':return PlotDataFilter.applyMovingVariance(timestamps, values, filterConfig.params);
            case 'scaleOffset':   return PlotDataFilter.applyScaleOffset(timestamps, values, filterConfig.params);
            default:              return { timestamps: [...timestamps], values: [...values] };
        }
    }
}

// PlotlyPlotManager 클래스 - Plotly.js를 이용한 Plot 생성 및 업데이트
class PlotlyPlotManager {
    constructor(containerId, bufferTime = 5.0) {  // 기본 5초 (ROS time 기준)
        this.containerId = containerId;
        this.plotId = containerId;  // plotId는 containerId와 동일
        this.container = null;
        this.bufferTime = bufferTime;  // 버퍼 시간 (초 단위, ROS time 기준)
        this.dataBuffers = new Map();  // path -> DataBuffer 매핑
        this.traces = [];              // Plotly traces
        this.isInitialized = false;
        this.updateThrottleInterval = 0.1;  // 10Hz throttling (ROS time 기준, 초 단위)
        this.lastUpdateRosTime = null;      // ROS time 기준 마지막 업데이트 시간
        this.pendingUpdate = false;
        this.firstTimestamp = null;    // 첫 데이터 포인트의 timestamp (t0 기준)
        this.t0Mode = true;            // t0 모드 (상대 시간 표시) - 디폴트 true로 변경
        this.isPaused = false;         // 일시정지 상태
        this.autoScaleRange = null;    // 오토 스케일 범위 (줌 제한용)
        this.panMode = false;          // Pan 모드 활성화 여부
        this.panStartPos = null;        // Pan 시작 위치
        this.panHandlers = [];
        this.panLastUpdateTime = 0;     // Pan 업데이트 throttling용
        this.panThrottleMs = 16;        // ~60Hz (약 16ms)
        this.panMousemoveHandler = null; // Pan mousemove 핸들러 (인스턴스 변수로 변경하여 제거 가능하도록)
        this.plotDeletionSetup = false;  // Plot 삭제 기능 설정 여부
        this.hoveredTraceIndex = null;   // 현재 hover된 trace index         // Pan 이벤트 핸들러 저장
        this.isXYPlot = false;          // XY Plot 여부
        this.xPath = null;              // X축 데이터 경로
        this.yPath = null;              // Y축 데이터 경로
        this.isXFlipped = false;        // X축 반전 상태
        this.isYFlipped = false;        // Y축 반전 상태
    }

    init() {
        this.container = document.getElementById(this.containerId);
        if (!this.container) {
            console.error(`[PlotlyPlotManager] Container ${this.containerId} not found`);
            return false;
        }

        // Plotly.js 로드 확인
        if (typeof Plotly === 'undefined') {
            console.error('[PlotlyPlotManager] Plotly.js not loaded');
            return false;
        }

        console.log('[PlotlyPlotManager] Initialized');
        return true;
    }

    initEmptyPlot() {
        if (!this.init()) {
            return false;
        }

        // Plotly 대신 간단한 HTML로 안내 메시지 표시
        const container = document.getElementById(this.containerId);
        if (!container) {
            console.error('[PlotlyPlotManager] Container not found:', this.containerId);
            return false;
        }

        container.innerHTML = `
            <div style="
                width: 100%;
                height: 100%;
                display: flex;
                flex-direction: column;
                align-items: center;
                justify-content: center;
                background: linear-gradient(135deg, #f8f8f8 0%, #ffffff 100%);
                border: 2px dashed #cccccc;
                border-radius: 8px;
                font-family: Arial, sans-serif;
            ">
                <div style="
                    font-size: 24px;
                    font-weight: 500;
                    color: #666666;
                    margin-bottom: 16px;
                    text-align: center;
                ">
                    📊 Drag and drop topics to plot
                </div>
                <div style="
                    font-size: 14px;
                    color: #999999;
                    text-align: center;
                    max-width: 400px;
                    line-height: 1.6;
                ">
                    Use 「+ Add」in the left panel to choose topics, then drag fields here to plot
                </div>
            </div>
        `;

        this.isInitialized = false;  // false로 설정하여 createPlot()이 제대로 동작하도록
        console.log('[PlotlyPlotManager] Empty plot placeholder initialized (HTML only)');
        return true;
    }

    createPlot(paths) {
        if (!this.init()) {
            return false;
        }

        // paths가 배열이 아니면 배열로 변환
        if (!Array.isArray(paths)) {
            paths = [paths];
        }

        if (paths.length === 0) {
            console.warn('[PlotlyPlotManager] No paths provided');
            return false;
        }

        console.log('[PlotlyPlotManager] Creating plot for paths:', paths);

        // 이미 초기화된 경우 trace만 추가
        if (this.isInitialized) {
            console.log('[PlotlyPlotManager] Plot already exists, adding traces...');
            return this.addTraces(paths);
        }

        // 기존 Plot 초기화 (처음 생성 시에만)
        this.clear();

        // 각 path별로 DataBuffer와 trace 생성
        paths.forEach((path, index) => {
            // DataBuffer 생성 (ROS time 기준 버퍼)
            const buffer = new DataBuffer(this.bufferTime);
            this.dataBuffers.set(path, buffer);
            console.log(`[PlotlyPlotManager] Created buffer for path: "${path}" (bufferTime: ${this.bufferTime}s)`);

            // Plotly trace 생성
            const trace = {
                x: [],
                y: [],
                mode: 'lines+markers',
                name: path,
                type: 'scatter',
                line: {
                    width: 2,       // 선 두께 증가 (1.5 → 2)
                    shape: 'linear'
                },
                marker: {
                    size: 4,        // 마커 크기 증가 (2 → 4, 더 잘 보임)
                    opacity: 0.8    // 마커 투명도 증가 (0.6 → 0.8, 더 선명)
                },
                connectgaps: false,  // gap이 있으면 선 끊기
                hovertemplate: '%{y:.6f}<extra></extra>'  // Y값만 표시 (시간 제거)
            };
            this.traces.push(trace);
        });

        // Plotly layout 설정 (흰색 배경, 검은색 격자선)
        const layout = {
            title: {
                text: paths.length === 1 ? `Plot: ${paths[0]}` : `Plot: ${paths.length} items`,
                font: {
                    color: '#dde1f5',
                    size: 14
                }
            },
            xaxis: {
                title: {
                    text: 'Time (seconds, relative to t0)',  // 디폴트로 상대 시간 표시
                    font: { color: '#7b82ab' }
                },
                showgrid: true,
                gridcolor: '#1e2238',
                gridwidth: 1,
                zeroline: true,
                zerolinecolor: '#2e3358',
                zerolinewidth: 1,
                tickfont: { color: '#7b82ab' },
                exponentformat: 'e',  // 과학적 표기법 (1.23e+08)
                showexponent: 'all'
            },
            yaxis: {
                title: {
                    text: 'Value',
                    font: { color: '#7b82ab' }
                },
                showgrid: true,
                gridcolor: '#1e2238',
                gridwidth: 1,
                zeroline: true,
                zerolinecolor: '#2e3358',
                zerolinewidth: 1,
                tickfont: { color: '#7b82ab' },
                exponentformat: 'e',  // 과학적 표기법 (1.23e+08)
                showexponent: 'all'
            },
            showlegend: true,
            legend: {
                x: 1,
                xanchor: 'right',
                y: 1,
                yanchor: 'top',
                bgcolor: 'rgba(17,20,38,0.9)',
                bordercolor: '#2e3358',
                borderwidth: 1,
                font: { color: '#dde1f5' }
            },
            margin: {
                l: 60,
                r: 120,
                b: 50,
                t: 50
            },
            paper_bgcolor: '#0b0d18',
            plot_bgcolor: '#0e1020',
            font: {
                color: '#dde1f5',
                family: 'Arial, sans-serif'
            }
        };

            // Plotly config 설정
        const config = {
            responsive: true,
            displayModeBar: true,
            modeBarPosition: 'top right',  // 모드바를 오른쪽 상단에 위치
            modeBarButtonsToRemove: ['lasso2d', 'select2d', 'zoomIn2d', 'zoomOut2d', 'autoScale2d', 'resetScale2d'],  // +/-, reset axes 버튼 제거
            displaylogo: false,
            scrollZoom: true,  // 마우스 휠 줌 활성화 (일시정지 시에만 작동, dragmode로 제어)
            hovermode: 'closest',  // Hover 모드 설정
            hoverlabel: {
                bgcolor: 'rgba(255, 255, 255, 0.9)',
                bordercolor: '#000',
                font: { color: '#000', size: 12 }
            },
            modeBarButtonsToAdd: [
                {
                    name: 'Pause/Play',
                    icon: {
                        width: 1000,
                        height: 1000,
                        // 일시정지 아이콘 (두 개의 수직 막대)
                        path: 'M300,200 L300,800 L400,800 L400,200 Z M600,200 L600,800 L700,800 L700,200 Z',
                        transform: 'matrix(1 0 0 -1 0 1000)'
                    },
                    click: (gd) => {
                        this.togglePause();
                    }
                },
                {
                    name: 'ROS Time (Absolute Time)',  // 버튼 이름 변경 (t0 모드가 기본이므로)
                    icon: {
                        width: 1000,
                        height: 1000,
                        // 시계 모양 아이콘
                        path: 'M500,100 A400,400 0 1,1 500,900 A400,400 0 1,1 500,100 M500,300 L500,500 L650,650',
                        transform: 'matrix(1 0 0 -1 0 1000)'
                    },
                    click: (gd) => {
                        this.toggleT0Mode();
                    }
                },
                {
                    name: 'Zoom Out (Auto Scale)',
                    icon: {
                        width: 1000,
                        height: 1000,
                        // 돋보기 - (줌아웃) 아이콘
                        path: 'M450,200 A250,250 0 1,1 450,700 A250,250 0 1,1 450,200 M350,450 L550,450 M600,650 L800,850',
                        transform: 'matrix(1 0 0 -1 0 1000)'
                    },
                    click: (gd) => {
                        this.zoomOutAutoScale();
                    }
                },
                {
                    name: 'Clear Plot (Reset)',
                    icon: {
                        width: 1000,
                        height: 1000,
                        // 쓰레기통 모양 아이콘
                        path: 'M300,200 L300,800 L700,800 L700,200 Z M250,200 L750,200 M350,150 L650,150 M400,350 L400,700 M500,350 L500,700 M600,350 L600,700',
                        transform: 'matrix(1 0 0 -1 0 1000)'
                    },
                    click: (gd) => {
                        this.clearPlot();
                    }
                }
            ]
        };

        // Plotly Plot 생성
        try {
            Plotly.newPlot(this.containerId, this.traces, layout, config);
            this.isInitialized = true;
            
            // 초기 상태 설정: 재생 중이면 dragmode를 false로 설정 (줌/팬 비활성화)
            if (!this.isPaused) {
                const plotDiv = document.getElementById(this.containerId);
                Plotly.relayout(this.containerId, { dragmode: false });
                console.log('[PlotlyPlotManager] Initial dragmode set to false (playing mode)');
            }
            
            // 모드바 위치를 오른쪽 상단으로 강제 설정
            setTimeout(() => {
                const plotDiv = document.getElementById(this.containerId);
                if (plotDiv) {
                    const modebar = plotDiv.querySelector('.modebar-container') || 
                                   plotDiv.querySelector('.modebar');
                    if (modebar) {
                        modebar.style.position = 'absolute';
                        modebar.style.top = '0';
                        modebar.style.right = '0';
                        modebar.style.left = 'auto';
                        console.log('[PlotlyPlotManager] Modebar position forced to top-right');
                    }
                }
            }, 100);
            
            // 타이틀 더블클릭 이벤트 추가
            this.setupTitleEditor();
            
            
            // 줌 제한 설정
            this.setupZoomLimiter();
            
            // 마우스 휠 제어 (일시정지 시에만 줌 가능)
            this.setupWheelControl();

            // 모드바(Plotly 툴바) 가드: 재생 중에는 Zoom/Pan 관련 버튼이 동작하지 않도록 차단
            this.setupModeBarGuards();
            // 초기 상태(재생)에서는 버튼 비활성화 상태로 표시
            setTimeout(() => this.updateModeBarButtonStates(), 200);
            
            // Hover tooltip 커스터마이징
            this.setupCustomHover();
            
            // Plot 데이터 및 Legend에서 plot 삭제 기능
            this.setupPlotDeletion();
            
            console.log('[PlotlyPlotManager] Plot created successfully');
            return true;
        } catch (error) {
            console.error('[PlotlyPlotManager] Failed to create plot:', error);
            return false;
        }
    }

    createXYPlot(xPath, yPath) {
        if (!this.init()) {
            return false;
        }

        console.log('[PlotlyPlotManager] Creating XY Plot:', xPath, 'vs', yPath);

        // 기존 Plot 초기화 (처음 생성 시에만)
        this.clear();

        // XY Plot 플래그 설정
        this.isXYPlot = true;
        this.xPath = xPath;
        this.yPath = yPath;

        // X, Y 각각 DataBuffer 생성
        const xBuffer = new DataBuffer(this.bufferTime);
        const yBuffer = new DataBuffer(this.bufferTime);
        this.dataBuffers.set(xPath, xBuffer);
        this.dataBuffers.set(yPath, yBuffer);
        console.log(`[PlotlyPlotManager] Created buffers for X: "${xPath}", Y: "${yPath}"`);

        // Plotly trace 생성 (XY Plot용)
        const trace = {
            x: [],
            y: [],
            mode: 'markers',  // XY Plot은 마커로 표시
            name: this._buildXYLegendName(xPath, yPath),
            type: 'scatter',
            marker: {
                size: 6,
                opacity: 0.8
            },
            line: {
                width: 2
            },
            connectgaps: false
        };
        this.traces.push(trace);

        // Plotly layout 설정
        const layout = {
            title: {
                text: `XY Plot: ${yPath} vs ${xPath}`,
                font: {
                    color: '#dde1f5',
                    size: 14
                }
            },
            xaxis: {
                title: {
                    text: xPath,  // X축 라벨은 데이터명
                    font: { color: '#7b82ab' }
                },
                showgrid: true,
                gridcolor: '#1e2238',
                gridwidth: 1,
                zeroline: true,
                zerolinecolor: '#2e3358',
                zerolinewidth: 1,
                tickfont: { color: '#7b82ab' }
            },
            yaxis: {
                title: {
                    text: yPath,  // Y축 라벨은 데이터명
                    font: { color: '#7b82ab' }
                },
                showgrid: true,
                gridcolor: '#1e2238',
                gridwidth: 1,
                zeroline: true,
                zerolinecolor: '#2e3358',
                zerolinewidth: 1,
                tickfont: { color: '#7b82ab' }
            },
            showlegend: true,
            legend: {
                x: 1,
                xanchor: 'right',
                y: 1,
                yanchor: 'top',
                bgcolor: 'rgba(17,20,38,0.9)',
                bordercolor: '#2e3358',
                borderwidth: 1,
                font: { color: '#dde1f5' }
            },
            margin: {
                l: 60,
                r: 120,
                b: 50,
                t: 50
            },
            paper_bgcolor: '#0b0d18',
            plot_bgcolor: '#0e1020',
            font: {
                color: '#dde1f5',
                family: 'Arial, sans-serif'
            }
        };

        // Plotly config 설정 (createPlot과 동일)
        const config = {
            responsive: true,
            displayModeBar: true,
            modeBarPosition: 'top right',
            modeBarButtonsToRemove: ['lasso2d', 'select2d', 'zoomIn2d', 'zoomOut2d', 'autoScale2d', 'resetScale2d'],
            displaylogo: false,
            scrollZoom: true,
            hovermode: 'closest',
            hoverlabel: {
                bgcolor: 'rgba(255, 255, 255, 0.9)',
                bordercolor: '#000',
                font: { color: '#000', size: 12 }
            },
            modeBarButtonsToAdd: [
                {
                    name: 'Pause/Play',
                    icon: {
                        width: 1000,
                        height: 1000,
                        path: 'M300,200 L300,800 L400,800 L400,200 Z M600,200 L600,800 L700,800 L700,200 Z',
                        transform: 'matrix(1 0 0 -1 0 1000)'
                    },
                    click: (gd) => {
                        this.togglePause();
                    }
                },
                {
                    name: 'Zoom Out (Auto Scale)',
                    icon: {
                        width: 1000,
                        height: 1000,
                        path: 'M450,200 A250,250 0 1,1 450,700 A250,250 0 1,1 450,200 M350,450 L550,450 M600,650 L800,850',
                        transform: 'matrix(1 0 0 -1 0 1000)'
                    },
                    click: (gd) => {
                        this.zoomOutAutoScale();
                    }
                },
                {
                    name: 'Clear Plot (Reset)',
                    icon: {
                        width: 1000,
                        height: 1000,
                        path: 'M300,200 L300,800 L700,800 L700,200 Z M250,200 L750,200 M350,150 L650,150 M400,350 L400,700 M500,350 L500,700 M600,350 L600,700',
                        transform: 'matrix(1 0 0 -1 0 1000)'
                    },
                    click: (gd) => {
                        this.clearPlot();
                    }
                },
            ]
        };

        // Plotly Plot 생성
        try {
            Plotly.newPlot(this.containerId, this.traces, layout, config);
            this.isInitialized = true;
            
            // 초기 상태: 재생 중
            if (!this.isPaused) {
                Plotly.relayout(this.containerId, { dragmode: false });
            }
            
            // 타이틀 더블클릭 이벤트 추가
            this.setupTitleEditor();
            
            
            // 줌 제한 설정
            this.setupZoomLimiter();
            
            // 마우스 휠 제어
            this.setupWheelControl();
            
            // 모드바 가드
            this.setupModeBarGuards();
            setTimeout(() => this.updateModeBarButtonStates(), 200);
            
            // Hover tooltip 커스터마이징
            this.setupCustomHover();
            
            // Plot 삭제 기능
            this.setupPlotDeletion();
            
            console.log('[PlotlyPlotManager] XY Plot created successfully');
            return true;
        } catch (error) {
            console.error('[PlotlyPlotManager] Failed to create XY plot:', error);
            return false;
        }
    }

    flipXAxis() {
        this.isXFlipped = !this.isXFlipped;
        const plotDiv = document.getElementById(this.containerId);
        const xRange = plotDiv && plotDiv.layout && plotDiv.layout.xaxis && plotDiv.layout.xaxis.range;
        if (xRange && xRange.length === 2) {
            // 현재 range를 swap하여 반전 (매 클릭마다 반전)
            Plotly.relayout(this.containerId, {
                'xaxis.range': [xRange[1], xRange[0]],
                'xaxis.autorange': false
            });
        } else {
            Plotly.relayout(this.containerId, {
                'xaxis.autorange': this.isXFlipped ? 'reversed' : true
            });
        }
        console.log(`[PlotlyPlotManager] X axis flipped: ${this.isXFlipped}`);
    }

    flipYAxis() {
        this.isYFlipped = !this.isYFlipped;
        const plotDiv = document.getElementById(this.containerId);
        const yRange = plotDiv && plotDiv.layout && plotDiv.layout.yaxis && plotDiv.layout.yaxis.range;
        if (yRange && yRange.length === 2) {
            // 현재 range를 swap하여 반전 (매 클릭마다 반전)
            Plotly.relayout(this.containerId, {
                'yaxis.range': [yRange[1], yRange[0]],
                'yaxis.autorange': false
            });
        } else {
            Plotly.relayout(this.containerId, {
                'yaxis.autorange': this.isYFlipped ? 'reversed' : true
            });
        }
        console.log(`[PlotlyPlotManager] Y axis flipped: ${this.isYFlipped}`);
    }

    synchronizeData(xBuffer, yBuffer) {
        // 두 버퍼의 timestamp를 기준으로 데이터 동기화
        // 같은 timestamp를 가진 데이터끼리 매칭 (허용 오차: ±0.01초)
        const tolerance = 0.01;  // 10ms
        
        const xData = xBuffer.getData();
        const yData = yBuffer.getData();
        
        const syncedX = [];
        const syncedY = [];
        
        let yIndex = 0;
        
        // X 데이터를 기준으로 Y 데이터 매칭
        for (let i = 0; i < xData.timestamps.length; i++) {
            const xTimestamp = xData.timestamps[i];
            
            // Y 데이터에서 가장 가까운 timestamp 찾기
            while (yIndex < yData.timestamps.length && 
                   yData.timestamps[yIndex] < xTimestamp - tolerance) {
                yIndex++;
            }
            
            // 매칭되는 Y 데이터가 있으면 추가
            if (yIndex < yData.timestamps.length && 
                Math.abs(yData.timestamps[yIndex] - xTimestamp) <= tolerance) {
                syncedX.push(xData.values[i]);
                syncedY.push(yData.values[yIndex]);
            }
        }
        
        return {
            x: syncedX,
            y: syncedY
        };
    }

    setupContextMenu() {
        const plotDiv = document.getElementById(this.containerId);
        let contextMenuPosition = { x: 0, y: 0 };  // 마우스 클릭 위치 저장
        
        // 기존 컨텍스트 메뉴 제거
        plotDiv.addEventListener('contextmenu', (e) => {
            // delete plot 메뉴가 표시되는 경우 기존 메뉴는 표시하지 않음
            if (this.hoveredTraceIndex !== null) {
                // Plot 데이터 위에 마우스가 있으면 기존 메뉴 표시 안함
                return;
            }
            
            // Legend 영역 확인
            const legend = plotDiv.querySelector('.legend');
            if (legend) {
                const legendRect = legend.getBoundingClientRect();
                const mouseX = e.clientX;
                const mouseY = e.clientY;
                
                if (mouseX >= legendRect.left && mouseX <= legendRect.right &&
                    mouseY >= legendRect.top && mouseY <= legendRect.bottom) {
                    // Legend 위에 있으면 기존 메뉴 표시 안함
                    return;
                }
            }
            
            e.preventDefault();
            
            // 마우스 위치 저장 (plot 영역 기준)
            const rect = plotDiv.getBoundingClientRect();
            contextMenuPosition.x = e.clientX - rect.left;
            contextMenuPosition.y = e.clientY - rect.top;
            
            // 기존 메뉴 제거
            const existingMenu = document.getElementById('plot-context-menu');
            if (existingMenu) {
                existingMenu.remove();
            }
            
            // 커스텀 메뉴 생성
            const menu = document.createElement('div');
            menu.id = 'plot-context-menu';
            menu.style.cssText = `
                position: absolute;
                left: ${e.pageX}px;
                top: ${e.pageY}px;
                background: #ffffff;
                border: 1px solid #ccc;
                border-radius: 4px;
                box-shadow: 0 2px 10px rgba(0,0,0,0.2);
                z-index: 10000;
                font-size: 13px;
                min-width: 180px;
            `;
            
            const menuItems = [
                { label: '📷 Export as PNG', action: () => this.exportToPNG() },
                { label: '↔️ Auto Scale', action: () => this.zoomOutAutoScale() },
                { label: '⚙️ Plot Settings', action: () => this.openPlotSettings() },
                { separator: true },
                { label: '➗ Split Horizontally (Coming soon)', action: () => console.log('Coming soon'), disabled: true },
                { label: '➗ Split Vertically (Coming soon)', action: () => console.log('Coming soon'), disabled: true }
            ];
            
            menuItems.forEach(item => {
                if (item.separator) {
                    const separator = document.createElement('div');
                    separator.style.cssText = 'height: 1px; background: #ddd; margin: 4px 0;';
                    menu.appendChild(separator);
                } else {
                    const menuItem = document.createElement('div');
                    menuItem.textContent = item.label;
                    menuItem.style.cssText = `
                        padding: 8px 16px;
                        cursor: ${item.disabled ? 'not-allowed' : 'pointer'};
                        color: ${item.disabled ? '#999' : '#000'};
                        background: transparent;
                    `;
                    
                    if (!item.disabled) {
                        menuItem.onmouseenter = () => {
                            menuItem.style.background = '#f0f0f0';
                        };
                        menuItem.onmouseleave = () => {
                            menuItem.style.background = 'transparent';
                        };
                        menuItem.onclick = () => {
                            item.action();
                            menu.remove();
                        };
                    }
                    
                    menu.appendChild(menuItem);
                }
            });
            
            document.body.appendChild(menu);
            
            // 외부 클릭 시 메뉴 닫기
            const closeMenu = (event) => {
                if (!menu.contains(event.target)) {
                    menu.remove();
                    document.removeEventListener('click', closeMenu);
                }
            };
            setTimeout(() => {
                document.addEventListener('click', closeMenu);
            }, 100);
        });
    }

    setupCustomHover() {
        const plotDiv = document.getElementById(this.containerId);
        if (!plotDiv) {
            return;
        }
        
        // Hover 시 Y값만 표시 - plotly_hover 이벤트로 직접 수정
        let hoverTimeout = null;
        
        plotDiv.on('plotly_hover', (data) => {
            if (!data || !data.points || data.points.length === 0) {
                return;
            }
            
            if (hoverTimeout) {
                clearTimeout(hoverTimeout);
            }
            
            // Plotly가 tooltip을 생성한 후 수정
            hoverTimeout = setTimeout(() => {
                const hoverLayer = plotDiv.querySelector('.hoverlayer');
                if (hoverLayer) {
                    const tooltips = hoverLayer.querySelectorAll('g.hovertext');
                    
                    // Y값 저장 (클로저 문제 해결)
                    const yValues = data.points.map(p => p.y !== undefined ? p.y : 0);
                    
                    // tooltip 수정 함수 (매번 tooltips를 다시 찾아서 최신 상태 유지)
                    const modifyTooltips = () => {
                        const currentHoverLayer = plotDiv.querySelector('.hoverlayer');
                        if (!currentHoverLayer) return;
                        
                        const currentTooltips = currentHoverLayer.querySelectorAll('g.hovertext');
                        currentTooltips.forEach((tooltipGroup, index) => {
                            if (yValues[index] !== undefined) {
                                const yValue = yValues[index];
                                
                                // tooltip의 모든 text 요소 찾기
                                const textElements = tooltipGroup.querySelectorAll('text, tspan');
                                textElements.forEach((textEl) => {
                                    const text = textEl.textContent || '';
                                    // time이 포함된 경우 (괄호, 쉼표, 과학적 표기법, 또는 숫자,쉼표,숫자 형식) Y값만 표시
                                    const hasComma = text.includes(',');
                                    const hasParen = text.includes('(');
                                    const hasScientific = text.match(/^\d+\.\d+e[+-]\d+/i) !== null;
                                    const hasComplexFormat = text.match(/^\d+\.\d+e\+\d+.*,.*-?\d/) !== null;
                                    const shouldModify = hasComma || hasParen || hasScientific || hasComplexFormat;
                                    
                                    if (shouldModify) {
                                        const newText = (typeof yValue === 'number') ? yValue.toFixed(6) : String(yValue);
                                        textEl.textContent = newText;
                                    }
                                });
                            }
                        });
                    };
                    
                    // 즉시 수정
                    modifyTooltips();
                    
                    // Plotly가 tooltip을 계속 업데이트하므로 MutationObserver로 감시
                    // 기존 observer가 있으면 disconnect
                    if (this.hoverObserver) {
                        this.hoverObserver.disconnect();
                    }
                    
                    // hoverLayer 전체를 관찰 (더 확실함)
                    this.hoverObserver = new MutationObserver(() => {
                        modifyTooltips();
                    });
                    
                    this.hoverObserver.observe(hoverLayer, {
                        childList: true,
                        subtree: true,
                        characterData: true
                    });
                    
                    // 주기적으로도 체크 (MutationObserver가 놓칠 수 있음)
                    if (this.hoverCheckInterval) {
                        clearInterval(this.hoverCheckInterval);
                    }
                    this.hoverCheckInterval = setInterval(() => {
                        modifyTooltips();
                    }, 100);
                }
            }, 50);
        });
        
        plotDiv.on('plotly_unhover', () => {
            if (hoverTimeout) {
                clearTimeout(hoverTimeout);
                hoverTimeout = null;
            }
            
            // MutationObserver 정리
            if (this.hoverObserver) {
                this.hoverObserver.disconnect();
                this.hoverObserver = null;
            }
            
            // Interval 정리
            if (this.hoverCheckInterval) {
                clearInterval(this.hoverCheckInterval);
                this.hoverCheckInterval = null;
            }
        });
    }

    setupPlotDeletion() {
        // 이미 설정되어 있으면 스킵 (중복 방지)
        if (this.plotDeletionSetup) {
            return;
        }
        
        const plotDiv = document.getElementById(this.containerId);
        if (!plotDiv) {
            return;
        }
        
        this.plotDeletionSetup = true;
        
        // Plot 데이터 위 마우스 오버레이 감지
        const hoverHandler = (data) => {
            if (data && data.points && data.points.length > 0) {
                this.hoveredTraceIndex = data.points[0].curveNumber;
            }
        };
        
        const unhoverHandler = () => {
            this.hoveredTraceIndex = null;
        };
        
        plotDiv.on('plotly_hover', hoverHandler);
        plotDiv.on('plotly_unhover', unhoverHandler);
        
        // 통합 contextmenu 핸들러 (Plot 데이터 + Legend)
        const contextMenuHandler = (e) => {
            e.preventDefault();
            e.stopPropagation();
            e.stopImmediatePropagation();
            
            // 기존 컨텍스트 메뉴 제거 (delete plot이 표시되면 기존 메뉴는 표시하지 않음)
            const existingContextMenu = document.getElementById('plot-context-menu');
            if (existingContextMenu) {
                existingContextMenu.remove();
            }
            
            const existingDeleteMenu = document.getElementById('plot-delete-menu');
            if (existingDeleteMenu) {
                existingDeleteMenu.remove();
            }
            
            let targetTraceIndex = null;
            let isPlotData = false;
            let isLegend = false;
            
            // 1. Plot 데이터 위에 마우스가 있는지 확인
            if (this.hoveredTraceIndex !== null) {
                targetTraceIndex = this.hoveredTraceIndex;
                isPlotData = true;
            } else {
                // 2. Legend 영역 확인
                const legend = plotDiv.querySelector('.legend');
                if (legend) {
                    const legendRect = legend.getBoundingClientRect();
                    const mouseX = e.clientX;
                    const mouseY = e.clientY;
                    
                    if (mouseX >= legendRect.left && mouseX <= legendRect.right &&
                        mouseY >= legendRect.top && mouseY <= legendRect.bottom) {
                        isLegend = true;
                        
                        // Legend의 rect 요소들 확인
                        const legendRects = legend.querySelectorAll('rect');
                        for (let i = 0; i < legendRects.length; i++) {
                            const rect = legendRects[i];
                            const rectBounds = rect.getBoundingClientRect();
                            if (mouseX >= rectBounds.left && mouseX <= rectBounds.right &&
                                mouseY >= rectBounds.top && mouseY <= rectBounds.bottom) {
                                // 부모 요소에서 trace index 찾기
                                let parent = rect.parentElement;
                                while (parent && parent !== legend) {
                                    if (parent.classList.contains('traces')) {
                                        const traceIndex = Array.from(legend.querySelectorAll('.traces')).indexOf(parent);
                                        if (traceIndex !== -1) {
                                            targetTraceIndex = traceIndex;
                                            break;
                                        }
                                    }
                                    parent = parent.parentElement;
                                }
                                break;
                            }
                        }
                        
                        // Fallback: 마우스 위치 기반으로 가장 가까운 trace 찾기
                        if (targetTraceIndex === null) {
                            let minDist = Infinity;
                            const legendItems = legend.querySelectorAll('.traces');
                            legendItems.forEach((item, index) => {
                                const itemRect = item.getBoundingClientRect();
                                const centerY = itemRect.top + itemRect.height / 2;
                                const dist = Math.abs(mouseY - centerY);
                                if (dist < minDist) {
                                    minDist = dist;
                                    targetTraceIndex = index;
                                }
                            });
                        }
                    }
                }
            }
            
            // 1. Plot 데이터 또는 Legend 위: Delete plot 메뉴 표시
            if ((isPlotData || isLegend) && targetTraceIndex !== null && targetTraceIndex < this.traces.length) {
                e.preventDefault();
                e.stopPropagation();
                e.stopImmediatePropagation(); // 다른 이벤트 리스너 차단
                
                // Legend 위에서 오른쪽 클릭 후 발생하는 자동 토글 방지
                if (isLegend) {
                    // Plotly의 legend 더블클릭 이벤트 차단
                    const blockLegendDoubleClick = (event) => {
                        event.preventDefault();
                        event.stopPropagation();
                        event.stopImmediatePropagation();
                        plotDiv.off('plotly_legenddoubleclick', blockLegendDoubleClick);
                    };
                    
                    // 짧은 시간 동안 legend 더블클릭 차단
                    plotDiv.on('plotly_legenddoubleclick', blockLegendDoubleClick);
                    setTimeout(() => {
                        plotDiv.off('plotly_legenddoubleclick', blockLegendDoubleClick);
                    }, 500);
                    
                    // Legend 클릭 이벤트도 capture phase에서 차단
                    const legend = plotDiv.querySelector('.legend');
                    if (legend) {
                        // 모든 마우스 이벤트 차단
                        const blockAllMouseEvents = (mouseEvent) => {
                            // 오른쪽 클릭 직후 발생하는 모든 마우스 이벤트 차단
                            mouseEvent.preventDefault();
                            mouseEvent.stopPropagation();
                            mouseEvent.stopImmediatePropagation();
                        };
                        
                        // capture phase에서 차단 (Plotly보다 먼저 실행)
                        legend.addEventListener('click', blockAllMouseEvents, true);
                        legend.addEventListener('mousedown', blockAllMouseEvents, true);
                        legend.addEventListener('mouseup', blockAllMouseEvents, true);
                        legend.addEventListener('dblclick', blockAllMouseEvents, true);
                        
                        // 500ms 후 자동 제거
                        setTimeout(() => {
                            legend.removeEventListener('click', blockAllMouseEvents, true);
                            legend.removeEventListener('mousedown', blockAllMouseEvents, true);
                            legend.removeEventListener('mouseup', blockAllMouseEvents, true);
                            legend.removeEventListener('dblclick', blockAllMouseEvents, true);
                        }, 500);
                    }
                }
                
                const menu = document.createElement('div');
                menu.id = 'plot-delete-menu';
                menu.style.cssText = `
                    position: absolute;
                    left: ${e.pageX}px;
                    top: ${e.pageY}px;
                    background: #ffffff;
                    border: 1px solid #ccc;
                    border-radius: 4px;
                    box-shadow: 0 2px 10px rgba(0,0,0,0.2);
                    z-index: 10001;
                    font-size: 13px;
                    min-width: 150px;
                `;
                
                // 메뉴 항목 목록: 🗑️ Delete plot
                const traceMenuItems = [
                    {
                        label: '🗑️ Delete plot',
                        action: () => {
                            this.deleteTrace(targetTraceIndex);
                            menu.remove();
                        }
                    }
                ];

                traceMenuItems.forEach(itemDef => {
                    const menuItem = document.createElement('div');
                    menuItem.textContent = itemDef.label;
                    menuItem.style.cssText = `
                        padding: 8px 16px;
                        cursor: pointer;
                        color: #000;
                        background: transparent;
                    `;

                    menuItem.onmouseenter = () => {
                        menuItem.style.background = '#f0f0f0';
                    };
                    menuItem.onmouseleave = () => {
                        menuItem.style.background = 'transparent';
                    };
                    menuItem.onclick = itemDef.action;

                    menu.appendChild(menuItem);
                });

                document.body.appendChild(menu);
                
                // 외부 클릭 시 메뉴 닫기
                const closeMenu = (event) => {
                    if (!menu.contains(event.target)) {
                        menu.remove();
                        document.removeEventListener('click', closeMenu);
                    }
                };
                setTimeout(() => {
                    document.addEventListener('click', closeMenu);
                }, 100);
                
                return false; // 이벤트 전파 차단
            } else {
                // 2. 빈 plot 영역: 일반 컨텍스트 메뉴 표시 (Plot Settings, Auto Scale 등)
                e.preventDefault();
                e.stopPropagation();
                
                const menu = document.createElement('div');
                menu.id = 'plot-context-menu';
                menu.style.cssText = `
                    position: absolute;
                    left: ${e.pageX}px;
                    top: ${e.pageY}px;
                    background: #ffffff;
                    border: 1px solid #ccc;
                    border-radius: 4px;
                    box-shadow: 0 2px 10px rgba(0,0,0,0.2);
                    z-index: 10000;
                    font-size: 13px;
                    min-width: 180px;
                `;
                
                const menuItems = [
                    { label: '⚙️ Plot Settings', action: () => this.openPlotSettings() },
                    { label: '↔️ Auto Scale', action: () => this.zoomOutAutoScale() },
                    { label: '🗑️ Clear Plot', action: () => this.clearPlot() },
                    { separator: true },
                    ...(this.isXYPlot ? [
                        { label: '⇔ Flip Horizontal Axis', action: () => this.flipXAxis() }
                    ] : []),
                    { label: '⇕ Flip Vertical Axis', action: () => this.flipYAxis() },
                    { separator: true },
                    { label: '📷 Export as PNG', action: () => this.exportToPNG() },
                    { separator: true },
                    { label: '🔧 Apply Filter', action: () => this.openFilterDialog(0) }
                ];
                
                menuItems.forEach(item => {
                    if (item.separator) {
                        const separator = document.createElement('div');
                        separator.style.cssText = 'height: 1px; background: #ddd; margin: 4px 0;';
                        menu.appendChild(separator);
                    } else {
                        const menuItem = document.createElement('div');
                        menuItem.textContent = item.label;
                        menuItem.style.cssText = `
                            padding: 8px 16px;
                            cursor: pointer;
                            color: #000;
                            background: transparent;
                        `;
                        
                        menuItem.onmouseenter = () => {
                            menuItem.style.background = '#f0f0f0';
                        };
                        menuItem.onmouseleave = () => {
                            menuItem.style.background = 'transparent';
                        };
                        menuItem.onclick = () => {
                            item.action();
                            menu.remove();
                        };
                        
                        menu.appendChild(menuItem);
                    }
                });
                
                document.body.appendChild(menu);
                
                // 메뉴 외부 클릭 시 닫기
                const closeMenu = (event) => {
                    if (!menu.contains(event.target)) {
                        menu.remove();
                        document.removeEventListener('click', closeMenu);
                    }
                };
                setTimeout(() => {
                    document.addEventListener('click', closeMenu);
                }, 100);
                
                return false; // 이벤트 전파 차단
            }
        };
        
        // contextmenu 이벤트는 capture phase에서 먼저 처리하여 setupContextMenu보다 우선
        plotDiv.addEventListener('contextmenu', contextMenuHandler, true);
    }

    deleteTrace(traceIndex) {
        if (traceIndex === null || traceIndex < 0 || traceIndex >= this.traces.length) {
            console.warn(`[PlotlyPlotManager] Invalid trace index: ${traceIndex}`);
            return;
        }
        
        const trace = this.traces[traceIndex];
        // bufferKey: 필터 적용 후 실제 buffer 키 (없으면 name 사용)
        const path = trace.bufferKey || trace.name;
        
        console.log(`[PlotlyPlotManager] Deleting trace: ${trace.name} (bufferKey: ${path}, index: ${traceIndex})`);
        
        // DataBuffer에서 제거
        if (this.dataBuffers.has(path)) {
            this.dataBuffers.delete(path);
        }
        
        // Trace 제거
        this.traces.splice(traceIndex, 1);
        
        // Plotly에서 trace 제거
        try {
            Plotly.deleteTraces(this.containerId, traceIndex);
            console.log(`[PlotlyPlotManager] Trace deleted successfully`);
        } catch (error) {
            console.error('[PlotlyPlotManager] Failed to delete trace:', error);
            // Plotly.react로 전체 다시 렌더링
            const plotDiv = document.getElementById(this.containerId);
            const currentLayout = plotDiv.layout;
            if (currentLayout) {
                currentLayout.paper_bgcolor = '#0b0d18';
                currentLayout.plot_bgcolor = '#0e1020';
            }
            Plotly.react(this.containerId, this.traces, currentLayout);
        }
        
        // addTraces 후 plot 삭제 기능 재설정 (이벤트 리스너 재등록)
        this.plotDeletionSetup = false;
        setTimeout(() => {
            this.setupPlotDeletion();
        }, 100);
    }

    setupPanControl() {
        const plotDiv = document.getElementById(this.containerId);
        if (!plotDiv) {
            return;
        }
        
        // 이미 활성화되어 있으면 기존 핸들러 제거 후 재등록 (안전장치)
        if (this.panMode) {
            // 기존 핸들러 제거
            this.panHandlers.forEach(({ element, event, handler, useCapture }) => {
                element.removeEventListener(event, handler, useCapture || false);
            });
            if (this.panMousemoveHandler) {
                document.removeEventListener('mousemove', this.panMousemoveHandler);
                this.panMousemoveHandler = null;
            }
            this.panHandlers = [];
        }
        
        this.panMode = true;
        
        // H2: 기존 mousemove 핸들러가 남아있으면 먼저 제거 (안전장치)
        if (this.panMousemoveHandler) {
            document.removeEventListener('mousemove', this.panMousemoveHandler);
            this.panMousemoveHandler = null;
        }
        
        // 중간 버튼(휠 클릭) mousedown 이벤트 (먼저 정의하고 등록하여 panStartPos를 먼저 설정)
        const mousedownHandler = (e) => {
            // 중간 버튼이 아니면 아무것도 하지 않음 (Plotly 기본 동작 허용)
            if (e.button !== 1) {
                return;
            }
            
            // 일시정지 상태가 아니면 아무것도 하지 않음
            if (!this.isPaused) {
                return;
            }
            
            // 중간 버튼이고 일시정지 상태일 때만 처리 (줌 모드에서도 작동)
            e.preventDefault();
            e.stopPropagation();
            e.stopImmediatePropagation();
            
            // Plotly의 dragmode를 false로 설정하여 모든 드래그 동작 차단
            const currentDragmode = plotDiv.layout.dragmode || 'zoom';
            this.savedDragmode = currentDragmode;
            
            console.log(`[PlotlyPlotManager] Middle button pan started (saved dragmode: ${currentDragmode})`);
            
            // panStartPos를 설정 (globalDragBlockers가 이미 등록되어 있으므로 즉시 차단 시작)
            this.panStartPos = {
                x: e.clientX,
                y: e.clientY
            };
            
            // dragmode를 false로 설정 (Plotly가 드래그를 완전히 비활성화)
            Plotly.relayout(plotDiv, { dragmode: false });
            
            // Plotly의 드래그 핸들러를 직접 비활성화
            // plotDiv의 _fullLayout에서 dragmode 강제 설정
            if (plotDiv._fullLayout) {
                plotDiv._fullLayout.dragmode = false;
            }
                
                // 현재 plot 범위 저장
                const layout = plotDiv.layout;
                this.panStartRange = {
                    x: layout.xaxis && layout.xaxis.range ? [...layout.xaxis.range] : null,
                    y: layout.yaxis && layout.yaxis.range ? [...layout.yaxis.range] : null
                };
                
                // 범위가 없으면 autorange 상태이므로 현재 범위를 가져와야 함
                if (!this.panStartRange.x || !this.panStartRange.y) {
                    const xaxis = plotDiv._fullLayout.xaxis;
                    const yaxis = plotDiv._fullLayout.yaxis;
                    if (xaxis && yaxis) {
                        this.panStartRange = {
                            x: [xaxis._rl[0], xaxis._rl[1]],
                            y: [yaxis._rl[0], yaxis._rl[1]]
                        };
                    }
                }
                
                plotDiv.style.cursor = 'grabbing';
                
                // mousemove 핸들러 동적 등록 (Pan 시작 시에만)
                // H2: 인스턴스 변수에 저장하여 removePanControl에서 제거 가능하도록
                this.panMousemoveHandler = (moveEvent) => {
                    if (!this.panStartPos || moveEvent.buttons !== 4) {
                        return;
                    }
                    
                    if (!this.isPaused) {
                        return;
                    }
                    
                    // Throttling: 너무 자주 업데이트하지 않음
                    const now = Date.now();
                    if (now - this.panLastUpdateTime < this.panThrottleMs) {
                        return;
                    }
                    this.panLastUpdateTime = now;
                    
                    moveEvent.preventDefault();
                    moveEvent.stopPropagation();
                    
                    const deltaX = moveEvent.clientX - this.panStartPos.x;
                    const deltaY = moveEvent.clientY - this.panStartPos.y;
                    
                    // Plot 영역 크기
                    const rect = plotDiv.getBoundingClientRect();
                    const plotWidth = rect.width;
                    const plotHeight = rect.height;
                    
                    // 범위 변화량 계산
                    if (this.panStartRange && this.panStartRange.x && this.panStartRange.y) {
                        const xRange = this.panStartRange.x[1] - this.panStartRange.x[0];
                        const yRange = this.panStartRange.y[1] - this.panStartRange.y[0];
                        
                        const deltaXRange = -(deltaX / plotWidth) * xRange;
                        const deltaYRange = (deltaY / plotHeight) * yRange; // Y축은 반대
                        
                        const newXRange = [
                            this.panStartRange.x[0] + deltaXRange,
                            this.panStartRange.x[1] + deltaXRange
                        ];
                        const newYRange = [
                            this.panStartRange.y[0] + deltaYRange,
                            this.panStartRange.y[1] + deltaYRange
                        ];
                        
                        Plotly.relayout(this.containerId, {
                            'xaxis.range': newXRange,
                            'yaxis.range': newYRange,
                            'xaxis.autorange': false,
                            'yaxis.autorange': false
                        });
                    }
                };
                
                document.addEventListener('mousemove', this.panMousemoveHandler);
                console.log('[PlotlyPlotManager] Pan started');
        };
        
        // mouseup 이벤트 (전역)
        const mouseupHandler = (e) => {
            if (e.button === 1 && this.panStartPos !== null) { // 중간 버튼이 눌려있었던 경우
                // mousemove 핸들러 제거
                if (this.panMousemoveHandler) {
                    document.removeEventListener('mousemove', this.panMousemoveHandler);
                    this.panMousemoveHandler = null;
                }
                
                // dragmode 복원
                const restoreDragmode = this.savedDragmode || 'zoom';
                Plotly.relayout(plotDiv, { dragmode: restoreDragmode });
                
                // _fullLayout도 복원
                if (plotDiv._fullLayout) {
                    plotDiv._fullLayout.dragmode = restoreDragmode;
                }
                
                this.panStartPos = null;
                this.panStartRange = null;
                this.panLastUpdateTime = 0;
                if (plotDiv) {
                    plotDiv.style.cursor = '';
                }
                console.log('[PlotlyPlotManager] Pan ended');
            }
        };
        
        // 이벤트 리스너 등록
        // 1. mousedownHandler를 capture phase로 먼저 등록하여 panStartPos를 먼저 설정
        plotDiv.addEventListener('mousedown', mousedownHandler, true);
        
        // 2. 왼쪽 버튼 드래그 차단 핸들러 정의 및 등록 (mousedownHandler 다음에 실행되도록)
        const blockLeftButtonDrag = (dragEvent) => {
            // pan 중일 때만 왼쪽 버튼 차단
            if (dragEvent.button === 0 && this.panStartPos !== null) {
                dragEvent.preventDefault();
                dragEvent.stopPropagation();
                dragEvent.stopImmediatePropagation();
                return false;
            }
        };
        
        // blockLeftButtonDrag를 capture phase로 등록 (mousedownHandler 다음에 등록)
        plotDiv.addEventListener('mousedown', blockLeftButtonDrag, true);
        document.addEventListener('mousedown', blockLeftButtonDrag, true);
        window.addEventListener('mousedown', blockLeftButtonDrag, true);
        
        // mouseup 이벤트 등록
        document.addEventListener('mouseup', mouseupHandler, false);
        
        // 핸들러 저장 (나중에 제거하기 위해)
        this.panHandlers = [
            { element: plotDiv, event: 'mousedown', handler: mousedownHandler, useCapture: true },
            { element: document, event: 'mouseup', handler: mouseupHandler, useCapture: false }
        ];
        
        this.globalDragBlockers = [
            { element: plotDiv, event: 'mousedown', handler: blockLeftButtonDrag, useCapture: true },
            { element: document, event: 'mousedown', handler: blockLeftButtonDrag, useCapture: true },
            { element: window, event: 'mousedown', handler: blockLeftButtonDrag, useCapture: true }
        ];
        
        console.log('[PlotlyPlotManager] Pan control enabled (middle button)');
    }

    removePanControl() {
        if (!this.panMode) {
            return; // 이미 비활성화됨
        }
        
        // 이벤트 리스너 제거
        this.panHandlers.forEach(({ element, event, handler, useCapture }) => {
            element.removeEventListener(event, handler, useCapture || false);
        });
        
        // H2: mousemove 핸들러 제거 (인스턴스 변수로 변경했으므로 여기서 제거 가능)
        if (this.panMousemoveHandler) {
            document.removeEventListener('mousemove', this.panMousemoveHandler);
            this.panMousemoveHandler = null;
        }
        
        // globalDragBlockers 제거
        if (this.globalDragBlockers) {
            this.globalDragBlockers.forEach(({ element, event, handler, useCapture }) => {
                element.removeEventListener(event, handler, useCapture);
            });
            this.globalDragBlockers = null;
        }
        
        this.panHandlers = [];
        this.panMode = false;
        this.panStartPos = null;
        this.panStartRange = null;
        this.panLastUpdateTime = 0;
        
        const plotDiv = document.getElementById(this.containerId);
        if (plotDiv) {
            plotDiv.style.cursor = '';
        }
        
        console.log('[PlotlyPlotManager] Pan control disabled');
    }

    exportToPNG() {
        console.log('[PlotlyPlotManager] Exporting plot as PNG...');
        
        // 탭 제목 가져오기: containerId에서 탭 ID 추출 후 DOM에서 탭 제목 조회
        const tabId = this.containerId.replace('plot-area-', '');
        const titleEl = document.getElementById(`plot-tab-title-${tabId}`);
        const plotTitle = titleEl ? titleEl.textContent.trim() : tabId;
        
        // 파일명: plot_{plotTitle}_{yyyymmdd_HHmmss}.png
        const now = new Date();
        const pad = (n) => String(n).padStart(2, '0');
        const timestamp = `${now.getFullYear()}${pad(now.getMonth() + 1)}${pad(now.getDate())}_${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}`;
        const safeTitle = plotTitle.replace(/[^a-zA-Z0-9가-힣_\-]/g, '_');
        const filename = `plot_${safeTitle}_${timestamp}`;
        
        Plotly.downloadImage(this.containerId, {
            format: 'png',
            width: 1200,
            height: 800,
            filename: filename
        });
        
        console.log(`[PlotlyPlotManager] PNG export triggered: ${filename}.png`);
    }

    setupZoomLimiter() {
        const plotDiv = document.getElementById(this.containerId);
        
        // relayout 이벤트 감지
        plotDiv.on('plotly_relayout', (eventData) => {
            // 오토스케일 버튼이나 더블클릭 시
            if (eventData['xaxis.autorange'] || eventData['yaxis.autorange']) {
                // autorange가 true가 되면 Plotly가 알아서 범위를 맞춤 -> 우리가 저장할 필요는 없음 (updatePlot에서 계속 저장 중)
                return;
            }
            
            // 일시정지 상태가 아니면 줌 제한하지 않음
            if (!this.isPaused || !this.autoScaleRange) {
                return;
            }
            
            // 현재 범위 확인
            let currentXRange = null;
            let currentYRange = null;
            
            if (eventData['xaxis.range[0]'] !== undefined && eventData['xaxis.range[1]'] !== undefined) {
                currentXRange = [eventData['xaxis.range[0]'], eventData['xaxis.range[1]']];
            } else if (eventData['xaxis.range']) {
                currentXRange = eventData['xaxis.range'];
            }
            
            if (eventData['yaxis.range[0]'] !== undefined && eventData['yaxis.range[1]'] !== undefined) {
                currentYRange = [eventData['yaxis.range[0]'], eventData['yaxis.range[1]']];
            } else if (eventData['yaxis.range']) {
                currentYRange = eventData['yaxis.range'];
            }
            
            // 범위 제한 체크
            const needsCorrection = {};
            
            if (currentXRange) {
                const xSpan = currentXRange[1] - currentXRange[0];
                const autoXSpan = this.autoScaleRange.x[1] - this.autoScaleRange.x[0];
                
                // 오토스케일 범위보다 5% 이상 더 커지면 (줌 아웃 과도)
                if (xSpan > autoXSpan * 1.05) {
                    needsCorrection['xaxis.range'] = this.autoScaleRange.x;
                    console.log('[PlotlyPlotManager] X-axis zoom limited to auto scale');
                }
            }
            
            if (currentYRange) {
                const ySpan = currentYRange[1] - currentYRange[0];
                const autoYSpan = this.autoScaleRange.y[1] - this.autoScaleRange.y[0];
                
                if (ySpan > autoYSpan * 1.05) {
                    needsCorrection['yaxis.range'] = this.autoScaleRange.y;
                    console.log('[PlotlyPlotManager] Y-axis zoom limited to auto scale');
                }
            }
            
            if (Object.keys(needsCorrection).length > 0) {
                Plotly.relayout(this.containerId, needsCorrection);
            }
        });
    }

    setupWheelControl() {
        const plotDiv = document.getElementById(this.containerId);
        
        // wheel 이벤트를 가로채서 일시정지 상태에서만 허용
        plotDiv.addEventListener('wheel', (e) => {
            if (!this.isPaused) {
                // 재생 중에는 줌 방지
                e.preventDefault();
                e.stopPropagation();
                return false;
            }
            // 일시정지 상태에서는 기본 줌 동작 허용
        }, { passive: false, capture: true });
        
        // 오른쪽 클릭 시 줌 방지 (일시정지 상태에서 줌 버튼이 활성화되어 있을 때)
        plotDiv.addEventListener('contextmenu', (e) => {
            // 일시정지 상태에서 dragmode가 zoom이면 오른쪽 클릭 차단
            if (this.isPaused && plotDiv.layout && plotDiv.layout.dragmode === 'zoom') {
                // Plotly의 오른쪽 클릭 줌 방지
                // 하지만 contextmenu는 허용 (setupContextMenu에서 처리)
                // 여기서는 Plotly의 내부 줌 핸들러만 차단
                const target = e.target;
                const isPlotArea = target.closest('.plot-container') && !target.closest('.modebar');
                
                if (isPlotArea) {
                    // Plotly의 오른쪽 클릭 줌 핸들러를 차단하기 위해
                    // mousedown 이벤트에서 차단하는 것이 더 효과적
                    // 여기서는 contextmenu만 통과시키고 Plotly 내부 줌은 차단
                }
            }
        }, { passive: false, capture: true });
        
        // mousedown 이벤트에서 오른쪽 클릭 줌 차단
        plotDiv.addEventListener('mousedown', (e) => {
            // 오른쪽 버튼 (button === 2) 및 일시정지 상태에서 zoom 모드인 경우
            if (e.button === 2 && this.isPaused && plotDiv.layout && plotDiv.layout.dragmode === 'zoom') {
                // Plotly의 줌 핸들러 차단
                e.stopPropagation();
                e.stopImmediatePropagation();
                // contextmenu는 허용하기 위해 preventDefault는 호출하지 않음
                console.log('[PlotlyPlotManager] Right-click zoom blocked (zoom mode)');
            }
        }, { passive: false, capture: true });
        
        console.log('[PlotlyPlotManager] Wheel control setup (paused mode only, right-click zoom blocked)');
    }

    _isZoomPanModeBarButtonTitle(title) {
        if (!title) return false;
        const t = String(title).toLowerCase();
        // Plotly 기본 버튼 타이틀들 (브라우저/버전에 따라 약간 다를 수 있음)
        // - Zoom / Pan / Box Select / Lasso Select / Zoom in / Zoom out 등
        // + 우리가 추가한 'Zoom Out (Auto Scale)' 도 재생 중에는 막는다.
        return (
            t.includes('zoom') ||
            t.includes('pan') ||
            t.includes('box select') ||
            t.includes('lasso')
        );
    }

    setupModeBarGuards() {
        const plotDiv = document.getElementById(this.containerId);
        if (!plotDiv) return;

        // 캡처 단계에서 클릭을 막아 Plotly 내부 핸들러보다 먼저 처리
        plotDiv.addEventListener('click', (e) => {
            const btn = e.target && e.target.closest ? e.target.closest('.modebar-btn') : null;
            if (!btn) return;

            const title = btn.getAttribute('data-title') || '';
            if (!this.isPaused && this._isZoomPanModeBarButtonTitle(title)) {
                e.preventDefault();
                e.stopPropagation();

                // 재생 중에는 dragmode가 바뀌지 않도록 즉시 복원
                Plotly.relayout(this.containerId, { dragmode: false });
                return false;
            }
        }, true);
    }

    updateModeBarButtonStates() {
        const plotDiv = document.getElementById(this.containerId);
        const modeBar = plotDiv ? plotDiv.querySelector('.modebar') : null;
        if (!modeBar) return;

        const buttons = modeBar.querySelectorAll('.modebar-btn[data-title]');
        buttons.forEach((btn) => {
            const title = btn.getAttribute('data-title') || '';
            if (!this._isZoomPanModeBarButtonTitle(title)) return;

            // 재생 중: 비활성화(클릭 불가) + 흐리게 표시
            if (!this.isPaused) {
                btn.style.pointerEvents = 'none';
                btn.style.opacity = '0.35';
                btn.style.filter = 'grayscale(1)';
            } else {
                // 일시정지: 활성화
                btn.style.pointerEvents = 'auto';
                btn.style.opacity = '1';
                btn.style.filter = 'none';
            }
        });
    }

    setupTitleEditor() {
        // Plot 타이틀 요소 찾기 (Plotly의 title text 요소)
        // 약간의 지연을 두고 DOM이 완전히 생성된 후 찾기
        setTimeout(() => {
            const titleElement = this.container.querySelector('.gtitle') || 
                                 this.container.querySelector('text.gtitle') ||
                                 this.container.querySelector('.g-gtitle');
            
            if (!titleElement) {
                console.warn('[PlotlyPlotManager] Title element not found, trying SVG text...');
                // SVG text 요소 중 타이틀 찾기
                const svgTexts = this.container.querySelectorAll('text');
                for (let text of svgTexts) {
                    if (text.textContent && text.textContent.startsWith('Plot:')) {
                        this.attachTitleClickHandler(text);
                        console.log('[PlotlyPlotManager] Title editor setup complete (via SVG text)');
                        return;
                    }
                }
                console.warn('[PlotlyPlotManager] Could not find title element');
                return;
            }

            this.attachTitleClickHandler(titleElement);
            console.log('[PlotlyPlotManager] Title editor setup complete');
        }, 100);
    }

    attachTitleClickHandler(titleElement) {
        titleElement.style.cursor = 'pointer';
        titleElement.addEventListener('dblclick', () => {
            const currentTitle = titleElement.textContent;
            this.showTitleEditDialog(currentTitle);
        });
    }

    showTitleEditDialog(currentTitle) {
        // 다이얼로그 생성
        const dialog = document.createElement('div');
        dialog.style.cssText = `
            position: fixed;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            background: white;
            padding: 20px;
            border: 2px solid #333;
            border-radius: 8px;
            box-shadow: 0 4px 12px rgba(0,0,0,0.3);
            z-index: 10000;
            min-width: 300px;
        `;

        // 제목
        const dialogTitle = document.createElement('h3');
        dialogTitle.textContent = 'Edit Plot Title';
        dialogTitle.style.cssText = 'margin: 0 0 15px 0; color: #333;';
        dialog.appendChild(dialogTitle);

        // 입력창
        const input = document.createElement('input');
        input.type = 'text';
        input.value = currentTitle;
        input.style.cssText = `
            width: 100%;
            padding: 8px;
            margin-bottom: 15px;
            border: 1px solid #ccc;
            border-radius: 4px;
            font-size: 14px;
            box-sizing: border-box;
        `;
        dialog.appendChild(input);

        // 버튼 컨테이너
        const buttonContainer = document.createElement('div');
        buttonContainer.style.cssText = 'display: flex; gap: 10px; justify-content: flex-end;';

        // OK 버튼
        const okButton = document.createElement('button');
        okButton.textContent = 'OK';
        okButton.style.cssText = `
            padding: 8px 16px;
            background: #007bff;
            color: white;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            font-size: 14px;
        `;
        okButton.onmouseover = () => okButton.style.background = '#0056b3';
        okButton.onmouseout = () => okButton.style.background = '#007bff';
        okButton.onclick = () => {
            const newTitle = input.value.trim();
            if (newTitle) {
                Plotly.relayout(this.containerId, {
                    'title.text': newTitle
                });
            }
            document.body.removeChild(overlay);
            document.body.removeChild(dialog);
        };

        // Cancel 버튼
        const cancelButton = document.createElement('button');
        cancelButton.textContent = 'Cancel';
        cancelButton.style.cssText = `
            padding: 8px 16px;
            background: #6c757d;
            color: white;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            font-size: 14px;
        `;
        cancelButton.onmouseover = () => cancelButton.style.background = '#5a6268';
        cancelButton.onmouseout = () => cancelButton.style.background = '#6c757d';
        cancelButton.onclick = () => {
            document.body.removeChild(overlay);
            document.body.removeChild(dialog);
        };

        buttonContainer.appendChild(cancelButton);
        buttonContainer.appendChild(okButton);
        dialog.appendChild(buttonContainer);

        // 배경 오버레이
        const overlay = document.createElement('div');
        overlay.style.cssText = `
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background: rgba(0, 0, 0, 0.5);
            z-index: 9999;
        `;
        overlay.onclick = () => {
            document.body.removeChild(overlay);
            document.body.removeChild(dialog);
        };

        // DOM에 추가
        document.body.appendChild(overlay);
        document.body.appendChild(dialog);

        // 입력창에 포커스
        input.focus();
        input.select();

        // Enter 키로 OK
        input.onkeydown = (e) => {
            if (e.key === 'Enter') {
                okButton.click();
            } else if (e.key === 'Escape') {
                cancelButton.click();
            }
        };
    }

    updatePlot(path, timestamp, value) {
        if (!this.isInitialized) {
            console.warn('[PlotlyPlotManager] Plot not initialized');
            return;
        }

        // DataBuffer에 데이터 추가 (항상 추가, ROS time 기준)
        const buffer = this.dataBuffers.get(path);
        if (!buffer) {
            console.warn(`[PlotlyPlotManager] Buffer not found for path: ${path}`);
            console.warn('[PlotlyPlotManager] Available buffers:', Array.from(this.dataBuffers.keys()));
            return;
        }

        // buffer.addData()에서 시간 역행 감지
        const result = buffer.addData(timestamp, value);
        
        // 시간 역행이 감지되면 전체 plot 초기화
        if (result && result.timeReversal) {
            console.warn(`[PlotlyPlotManager] ⚠️ Time reversal detected for ${path}! Clearing ALL plots and restarting...`);
            this.clearPlot();
            // 시간 역행 후 ROS time 리셋
            this.lastUpdateRosTime = null;
            this.firstTimestamp = null;
            // clearPlot()이 모든 버퍼를 초기화했으므로 현재 데이터 다시 추가
            buffer.addData(timestamp, value);
            console.log(`[PlotlyPlotManager] ✓ Plot cleared and restarted with new data.`);
            // 첫 timestamp 저장
            this.firstTimestamp = timestamp;
            // 즉시 업데이트
            this.lastUpdateRosTime = timestamp;
            this._updatePlotly();
            return;
        }
        
        // 첫 timestamp 저장 (t0 기준)
        if (this.firstTimestamp === null) {
            this.firstTimestamp = timestamp;
            console.log(`[PlotlyPlotManager] First timestamp set: ${this.firstTimestamp}`);
        }
        
        // 첫 10개 데이터만 로그
        if (buffer.getLength() <= 10) {
            console.log(`[PlotlyPlotManager] Data added for ${path}: t=${timestamp.toFixed(3)}, v=${value.toFixed(3)}, buffer size=${buffer.getLength()}`);
        }

        // 일시정지 상태면 화면 업데이트만 스킵 (데이터는 버퍼에 저장됨)
        if (this.isPaused) {
            return;
        }

        // Throttling: ROS time 기준 10Hz로 업데이트 제한
        if (this.lastUpdateRosTime !== null && 
            timestamp - this.lastUpdateRosTime < this.updateThrottleInterval) {
            // ROS time 기준으로 throttling (메시지가 너무 빠르게 들어올 때)
            return;
        }

        // 마지막 업데이트 시간 기록 (ROS time)
        this.lastUpdateRosTime = timestamp;
        
        this._updatePlotly();
    }

    _updatePlotly() {
        if (!this.isInitialized) {
            return;
        }

        try {
            // XY Plot인 경우
            if (this.isXYPlot) {
                this._updateXYPlot();
                return;
            }
            
            // 시계열 Plot인 경우 (기존 로직)
            // 각 trace의 데이터 업데이트 (isFiltered trace 제외)
            const traceIndices = [];  // 업데이트할 trace 인덱스 목록
            const updateData = {
                x: [],
                y: []
            };

            let totalPoints = 0;
            const t0 = this.firstTimestamp;  // t0 값을 미리 저장 (반복 참조 최소화)
            
            // 🎯 일시정지 시: 모든 데이터 표시, 재생 중: 강력한 다운샘플링 (최대 500 포인트, 10ms 이하 지연)
            const forceFullData = this.isPaused;
            const maxPoints = forceFullData ? Infinity : 500;  // 재생: 500, 일시정지: 모든 데이터
            
            this.traces.forEach((trace, index) => {
                // bufferKey: 필터 적용된 trace는 원본 경로, 없으면 name 사용
                const path = trace.bufferKey || trace.name;
                const buffer = this.dataBuffers.get(path);

                // 버퍼가 없는 trace는 정적 데이터(legacy isFiltered) → 업데이트 제외
                if (!buffer) {
                    return;
                }

                traceIndices.push(index);

                if (!buffer.isEmpty()) {
                    // 🚀 하이브리드 전략: 일시정지 시 모든 데이터, 재생 중 강력한 다운샘플링
                    const data = buffer.getDownsampledData(maxPoints, forceFullData);

                    let timestamps = data.timestamps;
                    let values = data.values;

                    // 🔧 filterConfig가 있으면 실시간으로 필터 적용
                    if (trace.filterConfig) {
                        try {
                            const filtered = PlotDataFilter.applyByConfig(timestamps, values, trace.filterConfig);
                            timestamps = filtered.timestamps;
                            values = filtered.values;
                        } catch (filterErr) {
                            console.warn('[_updatePlotly] Filter application error:', filterErr);
                        }
                    }

                    // t0 모드: 첫 timestamp를 0으로 만들어 상대 시간 표시
                    if (this.t0Mode && t0 !== null) {
                        const length = timestamps.length;
                        const converted = new Array(length);
                        for (let i = 0; i < length; i++) {
                            converted[i] = timestamps[i] - t0;
                        }
                        timestamps = converted;
                    }

                    updateData.x.push(timestamps);
                    updateData.y.push(values);
                    totalPoints += timestamps.length;
                } else {
                    // 버퍼가 비어있으면 빈 배열
                    updateData.x.push([]);
                    updateData.y.push([]);
                }
            });

            // 🚀 최적화: 비-필터 trace만 인덱스 지정하여 업데이트 (isFiltered trace 데이터 보존)
            if (traceIndices.length > 0) {
                Plotly.update(this.containerId, {
                    x: updateData.x,
                    y: updateData.y
                }, {}, traceIndices);
            }

            // 오토스케일 범위 업데이트 (줌 제한을 위해 데이터 범위 저장)
            this._updateAutoScaleRange(updateData);
            
            // 디버깅: 첫 10번만 로그 (성능 확인용)
            if (this.updateCount === undefined) this.updateCount = 0;
            if (this.updateCount < 10) {
                console.log(`[PlotlyPlotManager] Update #${this.updateCount}: isPaused=${this.isPaused}, totalPoints=${totalPoints}, forceFullData=${forceFullData}`);
                this.updateCount++;
            }

        } catch (error) {
            console.error('[PlotlyPlotManager] Failed to update plot:', error);
        }
    }
    
    _updateXYPlot() {
        // XY Plot 전용 업데이트 로직
        const xBuffer = this.dataBuffers.get(this.xPath);
        const yBuffer = this.dataBuffers.get(this.yPath);
        
        if (!xBuffer || !yBuffer || xBuffer.isEmpty() || yBuffer.isEmpty()) {
            return;
        }
        
        // timestamp 기준으로 동기화
        const syncedData = this.synchronizeData(xBuffer, yBuffer);
        
        if (syncedData.x.length === 0 || syncedData.y.length === 0) {
            return;
        }
        
        // Plotly.update() 호출
        Plotly.update(this.containerId, {
            x: [syncedData.x],
            y: [syncedData.y]
        }, {});
        
        // 오토스케일 범위 업데이트
        this._updateAutoScaleRange({
            x: [syncedData.x],
            y: [syncedData.y]
        });
        
        // 디버깅: 첫 10번만 로그
        if (this.updateCount === undefined) this.updateCount = 0;
        if (this.updateCount < 10) {
            console.log(`[PlotlyPlotManager] XY Plot Update #${this.updateCount}: points=${syncedData.x.length}`);
            this.updateCount++;
        }
    }

    _updateAutoScaleRange(updateData) {
        if (!updateData.x || updateData.x.length === 0) return;

        let minX = Infinity, maxX = -Infinity;
        let minY = Infinity, maxY = -Infinity;

        updateData.x.forEach((xArr, i) => {
            if (!xArr || xArr.length === 0) return;
            const yArr = updateData.y[i];
            
            // 🚀 최적화: spread operator 대신 반복문 사용 (메모리 효율)
            // X 범위
            for (let j = 0; j < xArr.length; j++) {
                if (xArr[j] < minX) minX = xArr[j];
                if (xArr[j] > maxX) maxX = xArr[j];
            }

            // Y 범위
            if (yArr && yArr.length > 0) {
                for (let j = 0; j < yArr.length; j++) {
                    if (yArr[j] < minY) minY = yArr[j];
                    if (yArr[j] > maxY) maxY = yArr[j];
                }
            }
        });

        if (minX !== Infinity && maxX !== -Infinity) {
            // 여유분 5% 추가
            const xPadding = (maxX - minX) * 0.05;
            const yPadding = (maxY - minY) * 0.05;

            this.autoScaleRange = {
                x: [minX - xPadding, maxX + xPadding],
                y: [minY - yPadding, maxY + yPadding]
            };
        }
    }

    clear() {
        this.dataBuffers.clear();
        this.traces = [];
        this.isInitialized = false;
        this.lastUpdateRosTime = null;
        this.pendingUpdate = false;
        this.firstTimestamp = null;

        if (this.container) {
            this.container.innerHTML = '';
        }
    }

    addTraces(paths) {
        if (!this.isInitialized) {
            console.warn('[PlotlyPlotManager] Plot not initialized, cannot add traces');
            return false;
        }

        // paths가 배열이 아니면 배열로 변환
        if (!Array.isArray(paths)) {
            paths = [paths];
        }

        console.log('[PlotlyPlotManager] Adding traces to existing plot:', paths);

        // 새로운 trace 생성
        const newTraces = [];
        paths.forEach((path) => {
            // 이미 존재하는 path면 스킵
            if (this.dataBuffers.has(path)) {
                console.log(`[PlotlyPlotManager] Path already exists: ${path}`);
                return;
            }

            // DataBuffer 생성 (ROS time 기준 버퍼)
            const buffer = new DataBuffer(this.bufferTime);
            this.dataBuffers.set(path, buffer);
            console.log(`[PlotlyPlotManager] Created buffer for path: "${path}" (bufferTime: ${this.bufferTime}s)`);

            // Plotly trace 생성
            const trace = {
                x: [],
                y: [],
                mode: 'lines+markers',
                name: path,
                type: 'scatter',
                line: {
                    width: 2,       // 선 두께 증가 (1.5 → 2)
                    shape: 'linear'
                },
                marker: {
                    size: 4,        // 마커 크기 증가 (2 → 4, 더 잘 보임)
                    opacity: 0.8    // 마커 투명도 증가 (0.6 → 0.8, 더 선명)
                },
                connectgaps: false,  // gap이 있으면 선 끊기
                hovertemplate: '%{y:.6f}<extra></extra>'  // Y값만 표시 (시간 제거)
            };
            // this.traces에는 나중에 추가 (Plotly.addTraces 성공 후)
            newTraces.push(trace);
        });

        if (newTraces.length === 0) {
            console.log('[PlotlyPlotManager] No new traces to add');
            return true;
        }

        // Plotly에 trace 추가
        try {
            const plotDiv = document.getElementById(this.containerId);
            const beforeCount = plotDiv.data ? plotDiv.data.length : 0;
            
            console.log(`[PlotlyPlotManager] BEFORE adding: Plotly has ${beforeCount} traces`);
            console.log(`[PlotlyPlotManager] this.traces.length = ${this.traces.length}`);
            console.log(`[PlotlyPlotManager] Adding traces:`, newTraces.map(t => t.name));
            
            // this.traces에 추가
            newTraces.forEach(trace => {
                this.traces.push(trace);
            });
            
            // ⚠️ 중요: Plotly.react를 사용하여 전체 traces를 다시 렌더링
            // Plotly.addTraces는 내부적으로 이상한 동작을 하므로 사용하지 않음
            const currentLayout = plotDiv.layout;
            
            // 제목 업데이트 (드래그한 메시지 제목으로 변경)
            const totalPaths = Array.from(this.dataBuffers.keys());
            currentLayout.title = {
                text: totalPaths.length === 1 ? `Plot: ${totalPaths[0]}` : `Plot: ${totalPaths.length} items`,
                font: {
                    color: '#000000',
                    size: 14
                }
            };
            
            Plotly.react(this.containerId, this.traces, currentLayout);
            
            const afterCount = plotDiv.data ? plotDiv.data.length : 0;
            console.log(`[PlotlyPlotManager] AFTER adding: Plotly has ${afterCount} traces (expected ${this.traces.length})`);
            console.log(`[PlotlyPlotManager] Added ${newTraces.length} traces successfully`);
            console.log(`[PlotlyPlotManager] Updated title to: ${currentLayout.title.text}`);
            
            // addTraces 후 plot 삭제 기능 재설정 (legend hover 이벤트 재설정)
            setTimeout(() => {
                this.setupPlotDeletion();
            }, 100);
            
            return true;
        } catch (error) {
            console.error('[PlotlyPlotManager] Failed to add traces:', error);
            return false;
        }
    }

    toggleT0Mode() {
        this.t0Mode = !this.t0Mode;
        console.log(`[PlotlyPlotManager] t0 mode: ${this.t0Mode ? 'ON (Relative)' : 'OFF (Absolute ROS time)'}`);
        
        if (this.t0Mode && this.firstTimestamp === null) {
            console.warn('[PlotlyPlotManager] No data yet, t0 mode will activate when data arrives');
            return;
        }
        
        // X축 라벨 변경 (t0 모드가 디폴트이므로)
        const xAxisTitle = this.t0Mode ? 'Time (seconds, relative to t0)' : 'Time (seconds, ROS time)';
        
        const plotDiv = document.getElementById(this.containerId);
        const layout = plotDiv && plotDiv.layout ? plotDiv.layout : null;

        // 재생 중: autorange 유지 (실시간 데이터 따라감)
        if (!this.isPaused) {
            Plotly.relayout(this.containerId, {
                'xaxis.title.text': xAxisTitle,
                'xaxis.autorange': true
            });
            // 즉시 업데이트
            this._updatePlotly();
        } 
        // 일시정지 중: 현재 보고 있는 뷰 유지하면서 좌표 변환
        else if (layout && layout.xaxis && Array.isArray(layout.xaxis.range)) {
            const currentXRange = layout.xaxis.range;
            let newXRange;
            
            if (this.firstTimestamp !== null) {
                if (this.t0Mode) {
                    // 절대 -> 상대: 값에서 firstTimestamp 빼기
                    // 단, 이미 상대값인 경우(버그 방지) 체크는 어렵지만, 
                    // toggle이므로 이전 상태가 절대값이면 현재값들은 절대값임.
                    newXRange = [currentXRange[0] - this.firstTimestamp, currentXRange[1] - this.firstTimestamp];
                } else {
                    // 상대 -> 절대: 값에 firstTimestamp 더하기
                    newXRange = [currentXRange[0] + this.firstTimestamp, currentXRange[1] + this.firstTimestamp];
                }
            }

            const relayoutArgs = { 'xaxis.title.text': xAxisTitle };
            if (newXRange) {
                relayoutArgs['xaxis.autorange'] = false; // 줌 상태 유지
                relayoutArgs['xaxis.range'] = newXRange;
            }
            
            // 뷰 업데이트
            Plotly.relayout(this.containerId, relayoutArgs).then(() => {
                // 데이터도 변환된 좌표계로 다시 그려야 함
                // 일시정지 상태에서 _updatePlotly() 호출 허용을 위해 잠시 플래그 조작 또는 강제 실행
                // 하지만 _updatePlotly는 this.isPaused 체크를 하지 않으므로 그냥 호출하면 됨
                // 단, updatePlot 메서드 내부에서만 체크함. _updatePlotly 자체는 체크 안함.
                this._updatePlotly();
            });
        } else {
            // 레이아웃 정보 없으면 라벨만 변경
            Plotly.relayout(this.containerId, { 'xaxis.title.text': xAxisTitle });
            this._updatePlotly();
        }
        
        // 버튼 이름 업데이트 (디폴트가 t0 모드이므로 버튼 이름 반대로)
        this.updateT0ButtonName();
        
        console.log(`[PlotlyPlotManager] t0 mode toggled, X-axis ${this.t0Mode ? 'relative' : 'absolute'}`);
    }

    updateT0ButtonName() {
        const plotDiv = document.getElementById(this.containerId);
        const modeBar = plotDiv ? plotDiv.querySelector('.modebar') : null;
        
        if (modeBar) {
            // 모든 버튼을 순회하여 t0/ROS Time 버튼 찾기
            const buttons = modeBar.querySelectorAll('[data-title]');
            let t0Button = null;
            
            for (const btn of buttons) {
                const title = btn.getAttribute('data-title');
                if (title && (title.includes('ROS Time') || title.includes('t0') || title.includes('Relative Time'))) {
                    t0Button = btn;
                    break;
                }
            }
            
            if (t0Button) {
                // t0 모드 ON: 버튼 이름을 "ROS Time (Absolute Time)"으로
                // t0 모드 OFF: 버튼 이름을 "t0 (Relative Time)"으로
                const newTitle = this.t0Mode ? 'ROS Time (Absolute Time)' : 't0 (Relative Time)';
                t0Button.setAttribute('data-title', newTitle);
                console.log(`[PlotlyPlotManager] t0 button title changed to: ${newTitle}`);
            } else {
                console.warn('[PlotlyPlotManager] t0/ROS Time button not found');
            }
        }
    }

    togglePause(forceState = null) {
        // forceState: true=일시정지, false=재생, null=토글
        if (forceState !== null) {
            this.isPaused = forceState;
        } else {
            this.isPaused = !this.isPaused;
        }
        
        console.log(`[PlotlyPlotManager] Plot ${this.isPaused ? 'PAUSED' : 'RESUMED'}`);
        
        const plotDiv = document.getElementById(this.containerId);
        
        if (this.isPaused) {
            // 일시정지: 줌 및 팬 모드 활성화
            Plotly.relayout(this.containerId, {
                'dragmode': 'zoom'  // 박스 선택으로 줌, 팬은 shift+드래그
            });
            
            // Pan 컨트롤 활성화
            this.setupPanControl();
            
            // 🎯 일시정지 시 즉시 모든 데이터를 표시
            console.log('[PlotlyPlotManager] Paused: Loading full data for detailed analysis...');
            this._updatePlotly();
            
            console.log('[PlotlyPlotManager] Zoom/Pan enabled (paused mode)');
        } else {
            // Pan 컨트롤 비활성화
            this.removePanControl();
            // 재생: 자동 오토스케일 후 모든 상호작용 비활성화
            this.zoomOutAutoScale();  // 자동 오토스케일
            
            // dragmode를 false로 설정하여 드래그 비활성화
            Plotly.relayout(this.containerId, {
                'dragmode': false
            });
            
            console.log('[PlotlyPlotManager] All interactions disabled (playing mode), auto scaled');
        }

        // 재생/일시정지 상태에 따라 모드바 버튼 활성/비활성 업데이트
        setTimeout(() => this.updateModeBarButtonStates(), 50);
        
        // 버튼 아이콘 업데이트
        const modeBar = plotDiv.querySelector('.modebar');
        
        if (modeBar) {
            // 모든 버튼을 순회하여 Pause/Play 버튼 찾기
            const buttons = modeBar.querySelectorAll('[data-title]');
            let pauseButton = null;
            
            for (const btn of buttons) {
                const title = btn.getAttribute('data-title');
                if (title && (title.includes('Pause') || title.includes('Play'))) {
                    pauseButton = btn;
                    break;
                }
            }
            
            if (pauseButton) {
                const path = pauseButton.querySelector('path');
                if (path) {
                    if (this.isPaused) {
                        // 재생 아이콘 (삼각형)
                        path.setAttribute('d', 'M300,200 L300,800 L800,500 Z');
                        pauseButton.setAttribute('data-title', 'Play (Zoom enabled)');
                        console.log('[PlotlyPlotManager] Button changed to PLAY icon');
                    } else {
                        // 일시정지 아이콘 (두 개의 수직 막대)
                        path.setAttribute('d', 'M300,200 L300,800 L400,800 L400,200 Z M600,200 L600,800 L700,800 L700,200 Z');
                        pauseButton.setAttribute('data-title', 'Pause/Play');
                        console.log('[PlotlyPlotManager] Button changed to PAUSE icon');
                    }
                }
            } else {
                console.warn('[PlotlyPlotManager] Pause button not found');
            }
        }
        
        // 일시정지 해제 시 즉시 plot 업데이트
        if (!this.isPaused) {
            this._updatePlotly();
        }
    }

    zoomOutAutoScale() {
        console.log('[PlotlyPlotManager] Zoom out (auto scale)...');
        
        // Plot을 오토 스케일 (축 범위를 데이터에 맞게 자동 조정)
        Plotly.relayout(this.containerId, {
            'xaxis.autorange': true,
            'yaxis.autorange': true
        });
        
        console.log('[PlotlyPlotManager] Plot auto scaled.');
    }

    clearPlot() {
        console.log('[PlotlyPlotManager] Clearing plot data...');
        
        if (!this.isInitialized) {
            console.warn('[PlotlyPlotManager] Plot not initialized, skipping clear');
            return;
        }
        
        // 모든 버퍼 초기화 (버퍼 객체는 유지, 데이터만 삭제)
        this.dataBuffers.forEach((buffer, path) => {
            buffer.clear();
        });
        
        // firstTimestamp 리셋
        this.firstTimestamp = null;
        
        // t0 모드는 유지 (사용자 요청)
        
        // 즉시 모든 trace를 빈 데이터로 업데이트 (화면에서 즉시 사라짐)
        try {
            // 모든 trace를 한 번에 업데이트 (더 빠름)
            const emptyData = {
                x: this.traces.map(() => []),
                y: this.traces.map(() => [])
            };
            
            Plotly.update(this.containerId, emptyData, {
                'xaxis.autorange': true,
                'yaxis.autorange': true
            });
            
            console.log('[PlotlyPlotManager] Plot immediately cleared.');
        } catch (error) {
            console.error('[PlotlyPlotManager] Failed to clear plot:', error);
        }
    }

    openPlotSettings() {
        console.log('[PlotlyPlotManager] Opening plot settings...');
        
        // 전역 함수 호출 (script.js에서 구현)
        if (typeof window.openPlotSettings === 'function') {
            window.openPlotSettings(this.plotId);
        } else {
            console.error('[PlotlyPlotManager] Global openPlotSettings() function not found');
        }
    }

    /**
     * Filter 다이얼로그를 열기 위해 script.js의 전역 함수를 호출한다.
     * @param {number} traceIndex - Filter를 적용할 source trace 인덱스
     */
    openFilterDialog(traceIndex) {
        console.log(`[PlotlyPlotManager] Opening filter dialog for trace index: ${traceIndex}`);

        if (typeof window.openFilterDialog === 'function') {
            window.openFilterDialog(this.plotId, traceIndex);
        } else {
            console.error('[PlotlyPlotManager] Global openFilterDialog() function not found');
        }
    }

    /**
     * 선택한 trace에 필터를 in-place로 적용한다.
     * 기존 trace를 새로운 trace로 교체하는 것이 아니라, 해당 trace를 직접 수정한다.
     * - trace.bufferKey : 실제 DataBuffer 키 (원본 topic 경로), 필터 체이닝 시 유지
     * - trace.filterConfig : { type, params } → _updatePlotly()에서 실시간 적용
     * - 'noTransform' filterType : 필터를 제거하고 원본으로 복원
     *
     * @param {number} traceIndex - Source trace 인덱스
     * @param {string} filterType - 필터 종류 ('absolute'|'derivative'|'movingAverage'|'movingRMS'|'movingVariance'|'scaleOffset'|'noTransform')
     * @param {object} params     - 필터 파라미터
     * @param {string} alias      - 결과 trace 이름 (legend에 표시)
     * @returns {boolean} 성공 여부
     */
    applyFilter(traceIndex, filterType, params, alias) {
        if (!this.isInitialized) {
            console.warn('[PlotlyPlotManager] Plot not initialized');
            return false;
        }

        if (traceIndex < 0 || traceIndex >= this.traces.length) {
            console.error('[PlotlyPlotManager] Invalid trace index for applyFilter:', traceIndex);
            return false;
        }

        const sourceTrace = this.traces[traceIndex];

        // bufferKey: 필터 체인 시 항상 원본 buffer를 가리킴
        const bufferKey = sourceTrace.bufferKey || sourceTrace.name;
        const buffer = this.dataBuffers.get(bufferKey);

        if (!buffer || buffer.isEmpty()) {
            console.warn('[PlotlyPlotManager] No data in buffer for trace:', bufferKey);
            return false;
        }

        // ── No Transform: 필터 제거, 원본 이름 복원 ──────────────────────
        if (filterType === 'noTransform') {
            sourceTrace.name = bufferKey;
            delete sourceTrace.bufferKey;
            delete sourceTrace.filterConfig;

            try {
                const plotDiv = document.getElementById(this.containerId);
                Plotly.react(this.containerId, this.traces, plotDiv.layout);
                this.plotDeletionSetup = false;
                setTimeout(() => this.setupPlotDeletion(), 100);
                console.log(`[PlotlyPlotManager] ✓ Filter removed (No Transform): trace restored to "${bufferKey}"`);
                return true;
            } catch (error) {
                console.error('[PlotlyPlotManager] Failed to remove filter:', error);
                return false;
            }
        }

        // ── 필터 적용: trace를 in-place로 수정 ──────────────────────────
        const filteredName = alias || `${bufferKey}[${filterType}]`;

        // trace 메타 업데이트
        sourceTrace.name = filteredName;
        sourceTrace.bufferKey = bufferKey;       // 원본 buffer 키 보존
        sourceTrace.filterConfig = { type: filterType, params };

        // 혹시 남아있는 legacy 필드 정리
        delete sourceTrace.isFiltered;
        sourceTrace.x = [];
        sourceTrace.y = [];

        try {
            const plotDiv = document.getElementById(this.containerId);
            Plotly.react(this.containerId, this.traces, plotDiv.layout);

            this.plotDeletionSetup = false;
            setTimeout(() => this.setupPlotDeletion(), 100);

            console.log(`[PlotlyPlotManager] ✓ Filter applied in-place: type=${filterType}, name="${filteredName}"`);
            return true;
        } catch (error) {
            console.error('[PlotlyPlotManager] Failed to apply filter:', error);
            return false;
        }
    }

    applyTraceSettings(settings) {
        console.log('[PlotlyPlotManager] Applying trace settings:', settings);
        
        if (!this.isInitialized) {
            console.warn('[PlotlyPlotManager] Plot not initialized');
            return;
        }
        
        const { traceIndex, color, lineStyle, markerStyle, lineWidth, markerSize, showGrid, xaxisLabel, yaxisLabel } = settings;
        
        // trace가 유효한지 확인
        if (traceIndex < 0 || traceIndex >= this.traces.length) {
            console.error('[PlotlyPlotManager] Invalid trace index:', traceIndex);
            return;
        }
        
        // trace 업데이트 객체 준비
        const traceUpdate = {};
        
        if (color) {
            traceUpdate['line.color'] = color;
            traceUpdate['marker.color'] = color;
        }
        
        if (lineStyle) {
            const dashMap = {
                'solid': 'solid',
                'dash': 'dash',
                'dot': 'dot',
                'dashdot': 'dashdot'
            };
            traceUpdate['line.dash'] = dashMap[lineStyle] || 'solid';
        }
        
        if (markerStyle) {
            const markerMap = {
                'circle': 'circle',
                'square': 'square',
                'diamond': 'diamond',
                'cross': 'cross',
                'x': 'x',
                'none': 'none'
            };
            traceUpdate['marker.symbol'] = markerMap[markerStyle] || 'circle';
            
            // 마커가 'none'이면 모드를 'lines'로, 아니면 'lines+markers'로 설정
            if (markerStyle === 'none') {
                traceUpdate['mode'] = 'lines';
            } else {
                traceUpdate['mode'] = 'lines+markers';
            }
        }
        
        if (lineWidth !== undefined) {
            traceUpdate['line.width'] = lineWidth;
        }
        
        if (markerSize !== undefined) {
            traceUpdate['marker.size'] = markerSize;
        }
        
        // layout 업데이트 객체 준비
        const layoutUpdate = {};
        
        if (showGrid !== undefined) {
            layoutUpdate['xaxis.showgrid'] = showGrid;
            layoutUpdate['yaxis.showgrid'] = showGrid;
        }
        
        if (xaxisLabel !== undefined) {
            layoutUpdate['xaxis.title.text'] = xaxisLabel;
        }
        
        if (yaxisLabel !== undefined) {
            layoutUpdate['yaxis.title.text'] = yaxisLabel;
        }
        
        try {
            // Plotly.update()를 사용하여 trace와 layout을 동시에 업데이트
            Plotly.update(this.containerId, traceUpdate, layoutUpdate, [traceIndex]);
            console.log('[PlotlyPlotManager] ✓ Trace settings applied successfully');
        } catch (error) {
            console.error('[PlotlyPlotManager] Failed to apply trace settings:', error);
        }
    }

    setBufferTime(seconds) {
        console.log(`[PlotlyPlotManager] Setting buffer time to ${seconds} seconds (ROS time based)`);
        
        // PlotManager의 bufferTime 업데이트
        this.bufferTime = seconds;
        
        // 모든 DataBuffer의 bufferTime 업데이트 (기존 데이터는 시간 기준으로 유지/삭제)
        this.dataBuffers.forEach((buffer, path) => {
            const beforeLength = buffer.getLength();
            buffer.setBufferTime(seconds);
            const afterLength = buffer.getLength();
            console.log(`[PlotlyPlotManager] Updated buffer for ${path}: ${beforeLength} → ${afterLength} points`);
        });
        
        console.log(`[PlotlyPlotManager] Buffer time updated. All buffers now use ${seconds}s window (ROS time based)`);
    }

    destroy() {
        if (this.isInitialized && this.container) {
            try {
                Plotly.purge(this.containerId);
            } catch (error) {
                console.error('[PlotlyPlotManager] Failed to purge plot:', error);
            }
        }
        this.clear();
    }

    /**
     * XY Plot legend 이름을 PlotJuggler 스타일로 생성한다.
     * xPath와 yPath의 공통 prefix를 추출하여
     * "/공통경로/[xField, yField]" 형식으로 반환한다.
     *
     * 예)
     *   xPath: "/odom/pose/pose/position/x"
     *   yPath: "/odom/pose/pose/position/y"
     *   → "/odom/pose/pose/position/[x, y]"
     *
     * 공통 prefix가 없으면 단순히 "[xField, yField]" 형식으로 반환한다.
     *
     * @param {string} xPath - X축 토픽 경로
     * @param {string} yPath - Y축 토픽 경로
     * @returns {string} legend 표시용 이름
     */
    _buildXYLegendName(xPath, yPath) {
        const xParts = xPath.split('/').filter(p => p.length > 0);
        const yParts = yPath.split('/').filter(p => p.length > 0);

        // 공통 prefix 길이 계산 (마지막 세그먼트 제외)
        const minLen = Math.min(xParts.length, yParts.length);
        let commonLen = 0;
        for (let i = 0; i < minLen - 1; i++) {
            if (xParts[i] === yParts[i]) {
                commonLen = i + 1;
            } else {
                break;
            }
        }

        const xField = xParts[xParts.length - 1] || xPath;
        const yField = yParts[yParts.length - 1] || yPath;

        if (commonLen > 0) {
            const commonPrefix = '/' + xParts.slice(0, commonLen).join('/');
            return `${commonPrefix}/[${xField}, ${yField}]`;
        }

        return `[${xField}, ${yField}]`;
    }
}