const BATTERY_THRESHOLDS = {
  good: 70,
  caution: 35,
};

const STORAGE_THRESHOLDS = {
  good: 70,
  caution: 85,
};

const CARD_CLASSES = [
  'is-good',
  'is-live',
  'is-caution',
  'is-low',
  'is-waiting',
  'is-stale',
  'is-offline',
];

const CONTROL_SERIES = {
  throttle: { color: '#3987e5', label: 'Throttle' },
  steering: { color: '#d95926', label: 'Steering' },
};

const config = window.MONITOR_CONFIG || {
  statusEndpoint: '/api/status',
  graphEndpoint: '/api/graph',
  debugFrameCompressedEndpoint: '/api/frame',
  debugFrameLaneEndpoint: '/api/frame/lane',
  debugFrameYoloEndpoint: '/api/frame/yolo',
  debugFrameLidarEndpoint: '/api/frame/lidar',
  debugFrameArucoEndpoint: '/api/frame/lidar',
  controlHistoryEndpoint: '/api/control_history',
  controlHistoryWindowSec: 20.0,
  topicEchoEndpoint: '/api/topic_echo',
  paramsEndpoint: '/api/params',
  debugImageEnabled: false,
  placeholderUrl: '/api/frame/placeholder',
  refreshIntervalMs: 1000,
  imageRefreshIntervalMs: 150,
};

const elements = {
  batteryCard: document.getElementById('battery-card'),
  batteryChip: document.getElementById('battery-chip'),
  batteryValue: document.getElementById('battery-value'),
  batteryUpdated: document.getElementById('battery-updated'),
  batteryMeterFill: document.getElementById('battery-meter-fill'),
  imageCard: document.getElementById('image-card'),
  imageChip: document.getElementById('image-chip'),
  imageUpdated: document.getElementById('image-updated'),
  imageResolution: document.getElementById('image-resolution'),
  debugFrameCompressed: document.getElementById('debug-frame-compressed'),
  debugFrameLane: document.getElementById('debug-frame-lane'),
  debugFrameYolo: document.getElementById('debug-frame-yolo'),
  debugFrameLidar: document.getElementById('debug-frame-lidar') || document.getElementById('debug-frame-aruco'),
  debugFrameAruco: document.getElementById('debug-frame-lidar') || document.getElementById('debug-frame-aruco'),
  controlCard: document.getElementById('control-card'),
  controlChip: document.getElementById('control-chip'),
  controlUpdated: document.getElementById('control-updated'),
  throttleValue: document.getElementById('throttle-value'),
  steeringValue: document.getElementById('steering-value'),
  controlChartCanvas: document.getElementById('control-chart'),
  controlChartEmpty: document.getElementById('control-chart-empty'),
  controlChartTooltip: document.getElementById('control-chart-tooltip'),
  storageCard: document.getElementById('storage-card'),
  storageChip: document.getElementById('storage-chip'),
  storageValue: document.getElementById('storage-value'),
  storageUpdated: document.getElementById('storage-updated'),
  storageMeterFill: document.getElementById('storage-meter-fill'),
  storageDetail: document.getElementById('storage-detail'),
  graphCard: document.getElementById('graph-card'),
  graphChip: document.getElementById('graph-chip'),
  graphCanvas: document.getElementById('ros-graph-canvas'),
  graphUpdated: document.getElementById('graph-updated'),
  graphSummary: document.getElementById('graph-summary'),
  topicEchoCard: document.getElementById('topic-echo-card'),
  topicEchoChip: document.getElementById('topic-echo-chip'),
  topicEchoSelect: document.getElementById('topic-echo-select'),
  topicEchoStop: document.getElementById('topic-echo-stop'),
  topicEchoLog: document.getElementById('topic-echo-log'),
  paramsCard: document.getElementById('params-card'),
  paramsChip: document.getElementById('params-chip'),
  paramsNodeSelect: document.getElementById('params-node-select'),
  paramsRefresh: document.getElementById('params-refresh'),
  paramsTableBody: document.getElementById('params-table-body'),
};

let debugImageRequestInFlight = false;
let currentEchoTopic = '';
let topicEchoOptionsSignature = '';

function clampPercent(value) {
  return Math.max(0, Math.min(100, Number(value) || 0));
}

function clampControl(value) {
  return Math.max(-1, Math.min(1, Number(value) || 0));
}

function clamp01(value) {
  return Math.max(0, Math.min(1, value));
}

function formatUpdatedAt(updatedAt) {
  if (!updatedAt) {
    return 'Waiting for message';
  }

  const date = new Date(updatedAt);

  if (Number.isNaN(date.getTime())) {
    return 'Invalid timestamp';
  }

  return date.toLocaleString();
}

function escapeHtml(value) {
  return String(value ?? '').replace(/[&<>"']/g, (character) => ({
    '&': '&amp;',
    '<': '&lt;',
    '>': '&gt;',
    '"': '&quot;',
    "'": '&#39;',
  }[character]));
}

function getGraphLabel(state) {
  switch (state) {
    case 'live':
      return 'LIVE';
    case 'stale':
      return 'STALE';
    default:
      return 'WAITING';
  }
}

function renderGraphNodeCard(node, role) {
  if (!node) {
    return '<span class="ros-graph__node-card ros-graph__node-card--placeholder">--</span>';
  }

  return `
    <span class="ros-graph__node-card ros-graph__node-card--${role}" title="${escapeHtml(node.label)}">
      ${escapeHtml(node.label || node.id)}
    </span>
  `;
}

function renderGraphEdge(edge, topicNode) {
  const topicLabel = topicNode?.label || edge.topic || 'topic';
  return `
    <div class="ros-graph__row">
      <div class="ros-graph__column ros-graph__column--source">
        ${renderGraphNodeCard(edge.source_node, 'source')}
      </div>
      <div class="ros-graph__edge" aria-label="${escapeHtml(topicLabel)}">
        <span
          class="ros-graph__edge-label ros-graph__edge-label--clickable"
          title="${escapeHtml(topicLabel)} 에코 보기"
          data-topic="${escapeHtml(edge.topic || topicLabel)}"
          role="button"
          tabindex="0"
        >
          ${escapeHtml(topicLabel)}
        </span>
      </div>
      <div class="ros-graph__column ros-graph__column--target">
        ${renderGraphNodeCard(edge.target_node, 'target')}
      </div>
    </div>
  `;
}

function renderGraphPublisherTopicRow(edge) {
  const topicLabel = edge.topic_node?.label || edge.topic || 'topic';
  return `
    <div class="ros-graph__publisher-topic-row">
      <div class="ros-graph__edge" aria-label="${escapeHtml(topicLabel)}">
        <span
          class="ros-graph__edge-label ros-graph__edge-label--clickable"
          title="${escapeHtml(topicLabel)} 에코 보기"
          data-topic="${escapeHtml(edge.topic || topicLabel)}"
          role="button"
          tabindex="0"
        >
          ${escapeHtml(topicLabel)}
        </span>
      </div>
      <div class="ros-graph__column ros-graph__column--target">
        ${renderGraphNodeCard(edge.target_node, 'target')}
      </div>
    </div>
  `;
}

function renderGraphPublisherGroup(group) {
  return `
    <div class="ros-graph__row ros-graph__publisher-group">
      <div class="ros-graph__column ros-graph__column--source">
        ${renderGraphNodeCard(group.source_node, 'source')}
      </div>
      <div class="ros-graph__publisher-topic-list">
        ${group.edges.map(renderGraphPublisherTopicRow).join('')}
      </div>
    </div>
  `;
}

function buildGraphRenderItems(rows) {
  const items = [];
  let currentPublisherGroup = null;

  rows.forEach((edge) => {
    if (edge.direction === 'publishes') {
      if (!currentPublisherGroup || currentPublisherGroup.source !== edge.source) {
        currentPublisherGroup = {
          kind: 'publisherGroup',
          source: edge.source,
          source_node: edge.source_node,
          edges: [],
        };
        items.push(currentPublisherGroup);
      }

      currentPublisherGroup.edges.push(edge);
      return;
    }

    currentPublisherGroup = null;
    items.push({ kind: 'edge', edge });
  });

  return items;
}

function renderGraph(payload) {
  if (!elements.graphCard || !elements.graphCanvas) {
    return;
  }

  const nodes = Array.isArray(payload.nodes) ? payload.nodes : [];
  const edges = Array.isArray(payload.edges) ? payload.edges : [];
  const nodeById = new Map(nodes.map((node) => [node.id, node]));
  const rows = edges.map((edge) => ({
    ...edge,
    source_node: nodeById.get(edge.source),
    target_node: nodeById.get(edge.target),
    topic_node: nodeById.get(`topic:${edge.topic}`),
  }));
  const graphState = rows.length > 0 ? 'live' : 'waiting';
  const renderItems = buildGraphRenderItems(rows);
  const rowMarkup = renderItems.map((item) => (
    item.kind === 'publisherGroup'
      ? renderGraphPublisherGroup(item)
      : renderGraphEdge(item.edge, item.edge.topic_node)
  )).join('');

  setCardState(elements.graphCard, graphState);
  elements.graphChip.textContent = getGraphLabel(graphState);
  elements.graphUpdated.textContent = formatUpdatedAt(payload.updated_at);
  elements.graphSummary.textContent = `${nodes.length} nodes / ${edges.length} edges`;
  elements.graphCanvas.innerHTML = rowMarkup || '<p class="ros-graph__placeholder">Waiting for ROS graph</p>';

  updateTopicEchoOptions(nodes);
}

function updateTopicEchoOptions(nodes) {
  const select = elements.topicEchoSelect;
  if (!select) {
    return;
  }

  const topicNames = Array.from(new Set(
    nodes.filter((node) => node.kind === 'topic').map((node) => node.label)
  )).sort();

  const optionsHtml = ['<option value="">토픽 선택…</option>']
    .concat(topicNames.map((name) => `<option value="${escapeHtml(name)}">${escapeHtml(name)}</option>`))
    .join('');

  if (topicEchoOptionsSignature === optionsHtml) {
    return;
  }

  const previousValue = select.value;
  select.innerHTML = optionsHtml;
  topicEchoOptionsSignature = optionsHtml;
  if (topicNames.includes(previousValue)) {
    select.value = previousValue;
  }
}

function setCardState(cardElement, state) {
  cardElement.classList.remove(...CARD_CLASSES);
  cardElement.classList.add(`is-${state}`);
}

function getBatteryState(data) {
  if (!data.has_data) {
    return 'waiting';
  }

  if (data.is_stale) {
    return 'stale';
  }

  const battery = clampPercent(data.battery_status);

  if (battery >= BATTERY_THRESHOLDS.good) {
    return 'good';
  }

  if (battery >= BATTERY_THRESHOLDS.caution) {
    return 'caution';
  }

  return 'low';
}

function getBatteryLabel(state) {
  switch (state) {
    case 'good':
      return 'GOOD';
    case 'caution':
      return 'CAUTION';
    case 'low':
      return 'LOW';
    case 'stale':
      return 'STALE';
    default:
      return 'WAITING';
  }
}

function getImageState(data) {
  if (!data.has_data) {
    return 'waiting';
  }

  if (data.is_stale) {
    return 'stale';
  }

  return 'live';
}

function getImageLabel(state) {
  switch (state) {
    case 'live':
      return 'LIVE';
    case 'stale':
      return 'STALE';
    default:
      return 'WAITING';
  }
}

function getControlState(data) {
  if (!data.has_data) {
    return 'waiting';
  }

  if (data.is_stale) {
    return 'stale';
  }

  return 'live';
}

function getControlLabel(state) {
  switch (state) {
    case 'live':
      return 'LIVE';
    case 'stale':
      return 'STALE';
    default:
      return 'WAITING';
  }
}

function getStorageState(data) {
  if (!data.has_data) {
    return 'waiting';
  }

  if (data.is_stale) {
    return 'stale';
  }

  const storageUsed = clampPercent(data.used_percentage);

  if (storageUsed >= STORAGE_THRESHOLDS.caution) {
    return 'low';
  }

  if (storageUsed >= STORAGE_THRESHOLDS.good) {
    return 'caution';
  }

  return 'good';
}

function getStorageLabel(state) {
  switch (state) {
    case 'good':
      return 'GOOD';
    case 'caution':
      return 'CAUTION';
    case 'low':
      return 'HIGH';
    case 'stale':
      return 'STALE';
    default:
      return 'WAITING';
  }
}

function formatControlValue(value, hasData) {
  if (!hasData || value === null || value === undefined) {
    return '--.--';
  }

  return Number(value).toFixed(2);
}

function renderBattery(data) {
  const state = getBatteryState(data);
  const batteryPercent = data.has_data ? clampPercent(data.battery_status) : 0;

  setCardState(elements.batteryCard, state);
  elements.batteryChip.textContent = getBatteryLabel(state);
  elements.batteryValue.textContent = data.battery_display || '--.-%';
  elements.batteryUpdated.textContent = formatUpdatedAt(data.updated_at);
  elements.batteryMeterFill.style.width = `${batteryPercent}%`;
}

function renderImage(data) {
  const state = getImageState(data);

  setCardState(elements.imageCard, state);
  elements.imageChip.textContent = getImageLabel(state);
  elements.imageUpdated.textContent = formatUpdatedAt(data.updated_at);
  elements.imageResolution.textContent = data.resolution_display || 'Waiting for frame';
}

function renderControl(data) {
  const state = getControlState(data);
  const hasData = Boolean(data.has_data);

  setCardState(elements.controlCard, state);
  elements.controlChip.textContent = getControlLabel(state);
  elements.controlUpdated.textContent = formatUpdatedAt(data.updated_at);
  elements.throttleValue.textContent = formatControlValue(data.throttle, hasData);
  elements.steeringValue.textContent = formatControlValue(data.steering, hasData);
}

function renderStorage(data) {
  const state = getStorageState(data);
  const usedPercent = data.has_data ? clampPercent(data.used_percentage) : 0;

  setCardState(elements.storageCard, state);
  elements.storageChip.textContent = getStorageLabel(state);
  elements.storageValue.textContent = data.used_display || '--.-%';
  elements.storageUpdated.textContent = formatUpdatedAt(data.updated_at);
  elements.storageMeterFill.style.width = `${usedPercent}%`;
  elements.storageDetail.textContent = `${data.used_space_display || '--'} / ${data.total_space_display || '--'} used`;
}

function renderOffline() {
  setCardState(elements.batteryCard, 'offline');
  setCardState(elements.imageCard, 'offline');
  setCardState(elements.controlCard, 'offline');
  setCardState(elements.storageCard, 'offline');
  if (elements.graphCard) {
    setCardState(elements.graphCard, 'offline');
  }
  elements.batteryChip.textContent = 'OFFLINE';
  elements.imageChip.textContent = 'OFFLINE';
  elements.controlChip.textContent = 'OFFLINE';
  elements.storageChip.textContent = 'OFFLINE';
  if (elements.graphChip) {
    elements.graphChip.textContent = 'OFFLINE';
  }
  elements.batteryUpdated.textContent = 'Unable to reach monitor server';
  elements.imageUpdated.textContent = 'Unable to reach monitor server';
  elements.controlUpdated.textContent = 'Unable to reach monitor server';
  elements.storageUpdated.textContent = 'Unable to reach monitor server';
  if (elements.graphUpdated) {
    elements.graphUpdated.textContent = 'Unable to reach monitor server';
  }
}

async function fetchGraph() {
  if (!config.graphEndpoint) {
    return;
  }

  try {
    const response = await fetch(config.graphEndpoint, { cache: 'no-store' });

    if (!response.ok) {
      throw new Error(`Unexpected response: ${response.status}`);
    }

    const payload = await response.json();
    renderGraph(payload || {});
  } catch (error) {
    console.error('Failed to fetch ROS graph', error);
    if (elements.graphCard) {
      setCardState(elements.graphCard, 'stale');
      elements.graphChip.textContent = 'STALE';
      elements.graphUpdated.textContent = 'Unable to reach graph endpoint';
    }
  }
}

async function fetchStatus() {
  try {
    const response = await fetch(config.statusEndpoint, { cache: 'no-store' });

    if (!response.ok) {
      throw new Error(`Unexpected response: ${response.status}`);
    }

    const payload = await response.json();
    renderBattery(payload.battery || {});
    renderImage(payload.image || {});
    renderControl(payload.control || {});
    renderStorage(payload.storage || {});
  } catch (error) {
    console.error('Failed to fetch monitor status', error);
    renderOffline();
  }
}

function refreshImageByEndpoint(targetElement, endpoint) {
  if (!targetElement) {
    return;
  }

  const image = new Image();
  image.onload = () => {
    targetElement.src = image.src;
  };
  image.onerror = () => {
    targetElement.src = config.placeholderUrl;
  };
  image.src = `${endpoint}?t=${Date.now()}`;
}

function refreshDebugFrames() {
  if (!config.debugImageEnabled || debugImageRequestInFlight) {
    return;
  }

  debugImageRequestInFlight = true;
  refreshImageByEndpoint(elements.debugFrameCompressed, config.debugFrameCompressedEndpoint);
  refreshImageByEndpoint(elements.debugFrameLane, config.debugFrameLaneEndpoint);
  refreshImageByEndpoint(elements.debugFrameYolo, config.debugFrameYoloEndpoint);
  refreshImageByEndpoint(elements.debugFrameLidar, config.debugFrameLidarEndpoint || config.debugFrameArucoEndpoint);
  debugImageRequestInFlight = false;
}

const CONTROL_CHART_PADDING = { top: 10, right: 12, bottom: 20, left: 32 };

let lastControlHistory = [];

function resizeCanvasForDPR(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(1, Math.round(rect.width));
  const height = Math.max(1, Math.round(rect.height));
  const targetWidth = Math.round(width * dpr);
  const targetHeight = Math.round(height * dpr);

  if (canvas.width !== targetWidth || canvas.height !== targetHeight) {
    canvas.width = targetWidth;
    canvas.height = targetHeight;
  }

  return { width, height, dpr };
}

function controlChartXForAge(ageSec, plotWidth) {
  const windowSec = Math.max(1, config.controlHistoryWindowSec);
  return CONTROL_CHART_PADDING.left + (1 - clamp01(ageSec / windowSec)) * plotWidth;
}

function controlChartYForValue(value, plotHeight) {
  return CONTROL_CHART_PADDING.top + ((1 - clampControl(value)) / 2) * plotHeight;
}

function drawControlChartSeries(ctx, history, key, color, plotWidth, plotHeight) {
  ctx.beginPath();
  history.forEach((sample, index) => {
    const x = controlChartXForAge(sample.age_sec, plotWidth);
    const y = controlChartYForValue(sample[key], plotHeight);
    if (index === 0) {
      ctx.moveTo(x, y);
    } else {
      ctx.lineTo(x, y);
    }
  });
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.lineJoin = 'round';
  ctx.lineCap = 'round';
  ctx.stroke();
}

function drawControlChartHoverMarker(ctx, sample, key, color, plotWidth, plotHeight) {
  const x = controlChartXForAge(sample.age_sec, plotWidth);
  const y = controlChartYForValue(sample[key], plotHeight);

  ctx.beginPath();
  ctx.arc(x, y, 5, 0, Math.PI * 2);
  ctx.fillStyle = 'rgba(30, 30, 30, 0.96)';
  ctx.fill();

  ctx.beginPath();
  ctx.arc(x, y, 3, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
}

function drawControlChart(history, hoverAgeSec) {
  const canvas = elements.controlChartCanvas;
  if (!canvas) {
    return;
  }

  if (elements.controlChartEmpty) {
    elements.controlChartEmpty.hidden = history.length > 0;
  }

  const { width, height, dpr } = resizeCanvasForDPR(canvas);
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);

  if (history.length === 0) {
    return;
  }

  const plotWidth = Math.max(1, width - CONTROL_CHART_PADDING.left - CONTROL_CHART_PADDING.right);
  const plotHeight = Math.max(1, height - CONTROL_CHART_PADDING.top - CONTROL_CHART_PADDING.bottom);
  const windowSec = Math.max(1, config.controlHistoryWindowSec);

  ctx.font = '10px system-ui, -apple-system, "Segoe UI", sans-serif';
  ctx.fillStyle = 'rgba(157, 161, 166, 0.85)';
  ctx.textBaseline = 'middle';

  [1, 0, -1].forEach((tick) => {
    const y = controlChartYForValue(tick, plotHeight);
    ctx.strokeStyle = tick === 0 ? 'rgba(212, 212, 212, 0.28)' : 'rgba(157, 161, 166, 0.16)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(CONTROL_CHART_PADDING.left, y);
    ctx.lineTo(width - CONTROL_CHART_PADDING.right, y);
    ctx.stroke();
    ctx.textAlign = 'right';
    ctx.fillText(tick.toFixed(1), CONTROL_CHART_PADDING.left - 6, y);
  });

  ctx.textAlign = 'left';
  ctx.fillText(`-${windowSec.toFixed(0)}s`, CONTROL_CHART_PADDING.left, height - 6);
  ctx.textAlign = 'right';
  ctx.fillText('now', width - CONTROL_CHART_PADDING.right, height - 6);

  drawControlChartSeries(ctx, history, 'throttle', CONTROL_SERIES.throttle.color, plotWidth, plotHeight);
  drawControlChartSeries(ctx, history, 'steering', CONTROL_SERIES.steering.color, plotWidth, plotHeight);

  if (hoverAgeSec === null || hoverAgeSec === undefined) {
    return;
  }

  const nearest = findNearestControlSample(history, hoverAgeSec);
  if (!nearest) {
    return;
  }

  const crosshairX = controlChartXForAge(nearest.age_sec, plotWidth);
  ctx.strokeStyle = 'rgba(212, 212, 212, 0.35)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(crosshairX, CONTROL_CHART_PADDING.top);
  ctx.lineTo(crosshairX, height - CONTROL_CHART_PADDING.bottom);
  ctx.stroke();

  drawControlChartHoverMarker(ctx, nearest, 'throttle', CONTROL_SERIES.throttle.color, plotWidth, plotHeight);
  drawControlChartHoverMarker(ctx, nearest, 'steering', CONTROL_SERIES.steering.color, plotWidth, plotHeight);
}

function findNearestControlSample(history, ageSec) {
  if (history.length === 0) {
    return null;
  }

  let nearest = history[0];
  let bestDiff = Math.abs(history[0].age_sec - ageSec);
  for (let i = 1; i < history.length; i += 1) {
    const diff = Math.abs(history[i].age_sec - ageSec);
    if (diff < bestDiff) {
      bestDiff = diff;
      nearest = history[i];
    }
  }
  return nearest;
}

function showControlChartTooltip(sample, x, y) {
  const tooltip = elements.controlChartTooltip;
  const canvas = elements.controlChartCanvas;
  if (!tooltip || !canvas) {
    return;
  }

  tooltip.hidden = false;
  tooltip.innerHTML = `
    <div class="control-chart__tooltip-row control-chart__tooltip-row--age">T-${sample.age_sec.toFixed(1)}s</div>
    <div class="control-chart__tooltip-row control-chart__tooltip-row--throttle">Throttle ${sample.throttle.toFixed(2)}</div>
    <div class="control-chart__tooltip-row control-chart__tooltip-row--steering">Steering ${sample.steering.toFixed(2)}</div>
  `;

  const tooltipWidth = tooltip.offsetWidth || 120;
  const left = Math.max(0, Math.min(x + 12, canvas.clientWidth - tooltipWidth));
  const top = Math.max(0, y - tooltip.offsetHeight - 12);
  tooltip.style.left = `${left}px`;
  tooltip.style.top = `${top}px`;
}

function hideControlChartTooltip() {
  const tooltip = elements.controlChartTooltip;
  if (tooltip) {
    tooltip.hidden = true;
  }
}

function handleControlChartMouseMove(event) {
  const canvas = elements.controlChartCanvas;
  if (!canvas || lastControlHistory.length === 0) {
    return;
  }

  const rect = canvas.getBoundingClientRect();
  const x = event.clientX - rect.left;
  const plotWidth = Math.max(1, rect.width - CONTROL_CHART_PADDING.left - CONTROL_CHART_PADDING.right);
  const windowSec = Math.max(1, config.controlHistoryWindowSec);
  const ratio = clamp01((x - CONTROL_CHART_PADDING.left) / plotWidth);
  const ageSec = (1 - ratio) * windowSec;

  const nearest = findNearestControlSample(lastControlHistory, ageSec);
  if (!nearest) {
    return;
  }

  drawControlChart(lastControlHistory, nearest.age_sec);
  showControlChartTooltip(nearest, x, event.clientY - rect.top);
}

function handleControlChartMouseLeave() {
  drawControlChart(lastControlHistory, null);
  hideControlChartTooltip();
}

function setupControlChartInteractions() {
  const canvas = elements.controlChartCanvas;
  if (!canvas) {
    return;
  }

  canvas.addEventListener('mousemove', handleControlChartMouseMove);
  canvas.addEventListener('mouseleave', handleControlChartMouseLeave);
}

async function fetchControlHistory() {
  try {
    const response = await fetch(config.controlHistoryEndpoint, { cache: 'no-store' });

    if (!response.ok) {
      throw new Error(`Unexpected response: ${response.status}`);
    }

    lastControlHistory = await response.json();
  } catch (error) {
    console.error('Failed to fetch control history', error);
    lastControlHistory = [];
  }

  drawControlChart(lastControlHistory, null);
}

async function startTopicEcho(topicName) {
  currentEchoTopic = topicName;

  if (!elements.topicEchoLog || !elements.topicEchoChip || !elements.topicEchoStop) {
    return;
  }

  elements.topicEchoStop.disabled = !topicName;

  if (!topicName) {
    elements.topicEchoChip.textContent = 'WAITING';
    elements.topicEchoLog.textContent = '토픽을 선택하면 메시지가 표시됩니다.';
    return;
  }

  elements.topicEchoChip.textContent = 'WAITING';
  elements.topicEchoLog.textContent = `${topicName} 연결 중…`;

  try {
    const response = await fetch(config.topicEchoEndpoint, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ topic: topicName }),
    });
    const payload = await response.json().catch(() => ({}));

    if (!response.ok || !payload.ok) {
      throw new Error(payload.error || `Unexpected response: ${response.status}`);
    }
  } catch (error) {
    console.error('Failed to start topic echo', error);
    elements.topicEchoChip.textContent = 'STALE';
    elements.topicEchoLog.textContent = `${topicName} 에코 시작 실패: ${error.message}`;
  }
}

function renderTopicEcho(payload) {
  if (!elements.topicEchoLog || !elements.topicEchoChip) {
    return;
  }

  const entries = Array.isArray(payload.entries) ? payload.entries : [];
  if (entries.length === 0) {
    return;
  }

  const text = entries
    .map((entry) => `# ${formatUpdatedAt(entry.stamp)}\n${entry.text}`)
    .join('\n---\n');

  elements.topicEchoLog.textContent = text;
  elements.topicEchoChip.textContent = 'LIVE';
  elements.topicEchoLog.scrollTop = elements.topicEchoLog.scrollHeight;
}

async function fetchTopicEcho() {
  if (!currentEchoTopic) {
    return;
  }

  try {
    const response = await fetch(config.topicEchoEndpoint, { cache: 'no-store' });

    if (!response.ok) {
      throw new Error(`Unexpected response: ${response.status}`);
    }

    const payload = await response.json();
    if (payload.topic !== currentEchoTopic) {
      return;
    }

    renderTopicEcho(payload);
  } catch (error) {
    console.error('Failed to fetch topic echo', error);
  }
}

function selectTopicFromGraph(topicName) {
  if (!topicName) {
    return;
  }

  if (elements.topicEchoSelect) {
    elements.topicEchoSelect.value = topicName;
  }

  startTopicEcho(topicName);

  if (elements.topicEchoCard) {
    elements.topicEchoCard.scrollIntoView({ behavior: 'smooth', block: 'start' });
  }
}

function setupTopicEcho() {
  if (elements.topicEchoSelect) {
    elements.topicEchoSelect.addEventListener('change', (event) => {
      startTopicEcho(event.target.value);
    });
  }

  if (elements.topicEchoStop) {
    elements.topicEchoStop.addEventListener('click', () => {
      if (elements.topicEchoSelect) {
        elements.topicEchoSelect.value = '';
      }
      startTopicEcho('');
    });
  }

  if (elements.graphCanvas) {
    elements.graphCanvas.addEventListener('click', (event) => {
      const target = event.target.closest('.ros-graph__edge-label--clickable');
      if (!target) {
        return;
      }
      selectTopicFromGraph(target.dataset.topic);
    });

    elements.graphCanvas.addEventListener('keydown', (event) => {
      if (event.key !== 'Enter' && event.key !== ' ') {
        return;
      }
      const target = event.target.closest('.ros-graph__edge-label--clickable');
      if (!target) {
        return;
      }
      event.preventDefault();
      selectTopicFromGraph(target.dataset.topic);
    });
  }
}

// rcl_interfaces/msg/ParameterType constants (no ROS msg import on the JS
// side, so hardcoded -- these are part of the stable ROS2 interface, not
// this project's own code).
const PARAM_TYPE = { BOOL: 1, INTEGER: 2, DOUBLE: 3, STRING: 4 };

function paramInputHtml(param) {
  const idSafe = escapeHtml(param.name);
  if (param.value === null || param.value === undefined) {
    return `<span class="params-readonly" title="이 UI에서 편집 불가한 타입(배열 등)">${escapeHtml(String(param.value))} (읽기전용)</span>`;
  }
  if (param.type === PARAM_TYPE.BOOL) {
    return `<input type="checkbox" class="params-input" data-param-name="${idSafe}" data-param-type="${param.type}" ${param.value ? 'checked' : ''}>`;
  }
  if (param.type === PARAM_TYPE.INTEGER) {
    return `<input type="number" step="1" class="params-input params-input--number" data-param-name="${idSafe}" data-param-type="${param.type}" value="${Number(param.value)}">`;
  }
  if (param.type === PARAM_TYPE.DOUBLE) {
    return `<input type="number" step="any" class="params-input params-input--number" data-param-name="${idSafe}" data-param-type="${param.type}" value="${Number(param.value)}">`;
  }
  return `<input type="text" class="params-input params-input--text" data-param-name="${idSafe}" data-param-type="${param.type}" value="${escapeHtml(String(param.value))}">`;
}

function renderParamsTable(params) {
  if (!elements.paramsTableBody) {
    return;
  }
  if (params.length === 0) {
    elements.paramsTableBody.innerHTML =
      '<tr><td colspan="3" class="params-table__empty">이 노드에는 파라미터가 없습니다.</td></tr>';
    return;
  }
  elements.paramsTableBody.innerHTML = params
    .map((param) => `
      <tr data-row-for="${escapeHtml(param.name)}">
        <td class="params-table__name">${escapeHtml(param.name)}</td>
        <td>${paramInputHtml(param)}</td>
        <td>${param.value === null || param.value === undefined ? '' :
          `<button type="button" class="params-apply-button" data-param-name="${escapeHtml(param.name)}">적용</button>`}</td>
      </tr>`)
    .join('');
}

async function fetchParams() {
  const nodeSelect = elements.paramsNodeSelect;
  if (!nodeSelect || !elements.paramsChip) {
    return;
  }
  const nodeName = nodeSelect.value;
  elements.paramsChip.textContent = 'WAITING';
  elements.paramsTableBody.innerHTML =
    '<tr><td colspan="3" class="params-table__empty">불러오는 중…</td></tr>';

  try {
    const response = await fetch(`${config.paramsEndpoint}?node=${encodeURIComponent(nodeName)}`, {
      cache: 'no-store',
    });
    const payload = await response.json().catch(() => ({}));
    if (!response.ok || !payload.ok) {
      throw new Error(payload.error || `Unexpected response: ${response.status}`);
    }
    renderParamsTable(payload.params);
    elements.paramsChip.textContent = 'LIVE';
  } catch (error) {
    console.error('Failed to fetch params', error);
    elements.paramsChip.textContent = 'STALE';
    elements.paramsTableBody.innerHTML =
      `<tr><td colspan="3" class="params-table__empty">불러오기 실패: ${escapeHtml(error.message)}</td></tr>`;
  }
}

async function applyParam(paramName) {
  const nodeSelect = elements.paramsNodeSelect;
  const row = elements.paramsTableBody.querySelector(`tr[data-row-for="${CSS.escape(paramName)}"]`);
  if (!nodeSelect || !row) {
    return;
  }
  const input = row.querySelector('.params-input');
  const button = row.querySelector('.params-apply-button');
  if (!input || !button) {
    return;
  }
  const type = Number(input.dataset.paramType);
  const value = type === PARAM_TYPE.BOOL ? input.checked : input.value;

  button.disabled = true;
  button.textContent = '적용 중…';
  try {
    const response = await fetch(config.paramsEndpoint, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ node: nodeSelect.value, name: paramName, type, value }),
    });
    const payload = await response.json().catch(() => ({}));
    if (!response.ok || !payload.ok) {
      throw new Error(payload.reason || payload.error || `Unexpected response: ${response.status}`);
    }
    button.textContent = '적용됨';
    row.classList.remove('params-row--error');
    row.classList.add('params-row--applied');
  } catch (error) {
    console.error(`Failed to set param ${paramName}`, error);
    button.textContent = '실패';
    row.classList.remove('params-row--applied');
    row.classList.add('params-row--error');
    row.title = error.message;
  } finally {
    button.disabled = false;
    window.setTimeout(() => {
      button.textContent = '적용';
      row.classList.remove('params-row--applied', 'params-row--error');
    }, 1500);
  }
}

function setupParamsTab() {
  if (elements.paramsNodeSelect) {
    elements.paramsNodeSelect.addEventListener('change', fetchParams);
  }
  if (elements.paramsRefresh) {
    elements.paramsRefresh.addEventListener('click', fetchParams);
  }
  if (elements.paramsTableBody) {
    elements.paramsTableBody.addEventListener('click', (event) => {
      const button = event.target.closest('.params-apply-button');
      if (!button) {
        return;
      }
      applyParam(button.dataset.paramName);
    });
  }
  // Params only load on tab-open/select-change/apply (not the fast poll
  // interval) -- auto-refreshing input fields every second would overwrite
  // whatever the user is mid-typing.
  const paramsTabButton = document.getElementById('tab-button-params');
  if (paramsTabButton) {
    paramsTabButton.addEventListener('click', () => {
      if (elements.paramsTableBody && elements.paramsTableBody.dataset.loaded !== 'true') {
        elements.paramsTableBody.dataset.loaded = 'true';
        fetchParams();
      }
    });
  }
}

function setupTabs() {
  const buttons = document.querySelectorAll('.tab-button');

  buttons.forEach((button) => {
    button.addEventListener('click', () => {
      const targetId = button.dataset.tabTarget;

      buttons.forEach((otherButton) => {
        const isActive = otherButton === button;
        otherButton.classList.toggle('is-active', isActive);
        otherButton.setAttribute('aria-selected', isActive ? 'true' : 'false');
      });

      document.querySelectorAll('.tab-panel').forEach((panel) => {
        panel.hidden = panel.id !== targetId;
      });
    });
  });
}

function startPolling() {
  fetchStatus();
  fetchGraph();
  fetchControlHistory();
  setupControlChartInteractions();
  setupTabs();
  setupTopicEcho();
  setupParamsTab();
  if (config.debugImageEnabled) {
    refreshDebugFrames();
  }
  window.setInterval(fetchStatus, config.refreshIntervalMs);
  window.setInterval(fetchGraph, config.refreshIntervalMs);
  window.setInterval(fetchControlHistory, config.refreshIntervalMs);
  window.setInterval(fetchTopicEcho, config.refreshIntervalMs);
  if (config.debugImageEnabled) {
    window.setInterval(refreshDebugFrames, config.imageRefreshIntervalMs);
  }
}

document.addEventListener('DOMContentLoaded', startPolling);
