const $ = (selector) => document.querySelector(selector);
const $$ = (selector) => [...document.querySelectorAll(selector)];

const elements = {
  modeButtons: $$('.mode-button'),
  liveTune: $('.live-tune'),
  scanTune: $('.scan-tune'),
  liveVisual: $('#live-visual'),
  scanVisual: $('#scan-visual'),
  protocol: $('#protocol-select'),
  frequency: $('#frequency-input'),
  frequencyEditor: $('#frequency-editor'),
  frequencyDigits: $('#frequency-digits'),
  channelFrequency: $('#channel-frequency-output'),
  copyChannelFrequency: $('#copy-channel-frequency'),
  span: $('#span-label'),
  start: $('#start-button'),
  startLabel: $('#start-button-label'),
  stop: $('#stop-button'),
  statusLabel: $('#status-label'),
  statusDetail: $('#status-detail'),
  connectionLight: $('#connection-light'),
  peak: $('#peak-value'),
  fpsValue: $('#fps-value'),
  fftValue: $('#fft-value'),
  spectrumCanvas: $('#spectrum-canvas'),
  waterfallCanvas: $('#waterfall-canvas'),
  filterPassband: $('#filter-passband'),
  filterPassbandLabel: $('#filter-passband-label'),
  filterHandleLeft: $('#filter-handle-left'),
  filterHandleRight: $('#filter-handle-right'),
  tuningMarker: $('#tuning-marker'),
  tuningTooltip: $('#tuning-tooltip'),
  scanCanvas: $('#scan-canvas'),
  scanStart: $('#scan-start'),
  scanEnd: $('#scan-end'),
  scanResolution: $('#scan-resolution'),
  scanProgressLabel: $('#scan-progress-label'),
  scanProgressDetail: $('#scan-progress-detail'),
  scanProgressValue: $('#scan-progress-value'),
  scanProgressBar: $('#scan-progress-bar'),
  device: $('#device-select'),
  refreshDevices: $('#refresh-devices'),
  gainAuto: $('#gain-auto'),
  gainInput: $('#gain-input'),
  gainOutput: $('#gain-output'),
  ppm: $('#ppm-input'),
  tunerBandwidth: $('#tuner-bandwidth'),
  protocolControls: $('#protocol-controls'),
  recordingControls: $('#recording-controls'),
  audioRecordingBlock: $('#audio-recording-block'),
  audioRecordingFormat: $('#audio-recording-format'),
  audioRecordingPath: $('#audio-recording-path'),
  audioRecordingButton: $('#audio-recording-button'),
  audioRecordingStatus: $('#audio-recording-status'),
  iqRecordingFormat: $('#iq-recording-format'),
  iqRecordingPath: $('#iq-recording-path'),
  iqRecordingButton: $('#iq-recording-button'),
  iqRecordingStatus: $('#iq-recording-status'),
  fftSize: $('#fft-size-select'),
  fpsInput: $('#fps-input'),
  fpsOutput: $('#fps-output'),
  decoderTitle: $('#decoder-title'),
  decoderContent: $('#decoder-content'),
  logContent: $('#log-content'),
  logCount: $('#log-count'),
  toast: $('#toast'),
};

const protocolDefaults = {
  voice: 145.5,
  adsb: 1090,
  ais: 162,
  sonde: 403,
  sstv: 145.8,
};

const state = {
  mode: 'live',
  processMode: 'idle',
  processStatus: 'stopped',
  activeSessionId: null,
  protocolSwitchPending: false,
  centerHz: 145_500_000,
  channelHz: 145_500_000,
  sampleRate: 2_400_000,
  latestBins: null,
  spectrumPending: false,
  framesThisSecond: 0,
  measuredFps: 0,
  lastFpsAt: performance.now(),
  receiverStatus: { channelDbfs: null, thresholdDbfs: null, squelchOpen: null },
  recordings: {
    audio: { active: false, path: '', format: '', bytes: 0 },
    iq: { active: false, path: '', format: '', bytes: 0 },
  },
  recordingPending: { audio: false, iq: false },
  appliedGainDb: null,
  liveTune: { dragging: false, rect: null, frequencyHz: null },
  filterResize: { dragging: false, rect: null },
  logs: [],
  adsb: new Map(),
  ais: new Map(),
  sonde: new Map(),
  sstv: { event: 'waiting', progress: 0, total: 0, artifactUrl: '', path: '' },
  scan: {
    points: new Map(),
    startHz: 50_000_000,
    endHz: 1_700_000_000,
    viewStartHz: 50_000_000,
    viewEndHz: 1_700_000_000,
    progress: 0,
    renderPending: false,
    dragging: false,
    dragStartX: 0,
    dragViewStart: 0,
    dragViewEnd: 0,
    hoverX: null,
  },
};

const FREQUENCY_MAX_HZ = 1_766_000_000;
const FREQUENCY_DIGIT_POWERS = [
  1_000_000_000, 100_000_000, 10_000_000, 1_000_000, 100_000,
  10_000, 1_000, 100, 10, 1,
];
const FREQUENCY_DIGIT_LABELS = [
  'gigahertz', 'centinaia di megahertz', 'decine di megahertz', 'megahertz',
  'centinaia di kilohertz', 'decine di kilohertz', 'kilohertz',
  'centinaia di hertz', 'decine di hertz', 'hertz',
];
const LIVE_PLOT_INSETS = Object.freeze({ left: 48, right: 15 });
const CHANNEL_WHEEL_STEP_HZ = 1_000;
let frequencyRetuneTimer = null;
let gainUpdateTimer = null;
let channelWheelCommitTimer = null;
let channelWheelFeedbackTimer = null;
let protocolSwitchGeneration = 0;

function protocolSampleRate(protocol) {
  if (protocol === 'voice') {
    return Number($('#voice-sample-rate')?.value || 2_400_000);
  }
  return {
    adsb: 2_000_000,
    ais: 2_400_000,
    sonde: 240_000,
    sstv: 240_000,
  }[protocol] || 2_400_000;
}

function livePlotMetrics(totalWidth) {
  const width = Math.max(1, Number(totalWidth) || 1);
  const left = Math.min(LIVE_PLOT_INSETS.left, Math.max(0, width - 1));
  const right = Math.min(LIVE_PLOT_INSETS.right, Math.max(0, width - left - 1));
  return { left, right, width: Math.max(1, width - left - right) };
}

function livePlotX(ratio, totalWidth) {
  const plot = livePlotMetrics(totalWidth);
  return plot.left + Math.max(0, Math.min(1, ratio)) * plot.width;
}

function livePlotRatio(clientX, rect) {
  const plot = livePlotMetrics(rect.width);
  return Math.max(0, Math.min(1,
    (clientX - rect.left - plot.left) / plot.width));
}

function clampFrequencyHz(frequencyHz) {
  return Math.max(0, Math.min(FREQUENCY_MAX_HZ, Math.round(Number(frequencyHz) || 0)));
}

function frequencyHzFromInput() {
  return clampFrequencyHz(Number(elements.frequency.value) * 1e6);
}

function buildFrequencyEditor() {
  const groupSizes = [1, 3, 3, 3];
  const units = ['GHz', 'MHz', 'kHz', 'Hz'];
  let digitIndex = 0;
  elements.frequencyDigits.innerHTML = groupSizes.map((size, groupIndex) => {
    const controls = Array.from({ length: size }, () => {
      const index = digitIndex++;
      const label = FREQUENCY_DIGIT_LABELS[index];
      return `<span class="frequency-digit-control" data-index="${index}">
        <button type="button" class="frequency-step" data-index="${index}" data-direction="1" tabindex="-1" aria-label="Aumenta ${label}">▲</button>
        <button type="button" class="frequency-digit" data-index="${index}" aria-label="${label}" title="Digita 0–9, usa ↑/↓ o la rotellina">0</button>
        <button type="button" class="frequency-step" data-index="${index}" data-direction="-1" tabindex="-1" aria-label="Diminuisci ${label}">▼</button>
      </span>`;
    }).join('');
    const separator = groupIndex < groupSizes.length - 1
      ? '<span class="frequency-separator" aria-hidden="true">.</span>' : '';
    return `<span class="frequency-cluster">${controls}<span class="frequency-unit">${units[groupIndex]}</span></span>${separator}`;
  }).join('');
}

function renderFrequencyEditor(frequencyHz = frequencyHzFromInput()) {
  const digits = String(clampFrequencyHz(frequencyHz)).padStart(10, '0');
  $$('.frequency-digit').forEach((button, index) => {
    button.textContent = digits[index];
    button.setAttribute('aria-label', `${FREQUENCY_DIGIT_LABELS[index]}: ${digits[index]}`);
  });
}

function syncFrequencyEditor(frequencyHz) {
  const normalizedHz = clampFrequencyHz(frequencyHz);
  elements.frequency.value = (normalizedHz / 1e6).toFixed(6);
  renderFrequencyEditor(normalizedHz);
  return normalizedHz;
}

function focusFrequencyDigit(index) {
  elements.frequencyDigits.querySelector(`.frequency-digit[data-index="${Math.max(0, Math.min(9, index))}"]`)?.focus({ preventScroll: true });
}

function commitFrequencyFromEditor(frequencyHz, focusIndex) {
  const normalizedHz = syncFrequencyEditor(frequencyHz);
  state.centerHz = normalizedHz;
  state.channelHz = normalizedHz;
  positionLiveChannelMarker();
  focusFrequencyDigit(focusIndex);
  clearTimeout(frequencyRetuneTimer);
  frequencyRetuneTimer = setTimeout(() => { void retuneLive(normalizedHz); }, 90);
}

function replaceFrequencyDigit(index, value) {
  const digits = String(frequencyHzFromInput()).padStart(10, '0').split('');
  digits[index] = value;
  commitFrequencyFromEditor(Number(digits.join('')), Math.min(9, index + 1));
}

function stepFrequencyDigit(index, direction) {
  commitFrequencyFromEditor(frequencyHzFromInput() + FREQUENCY_DIGIT_POWERS[index] * direction, index);
}

function escapeHtml(value) {
  return String(value ?? '')
    .replaceAll('&', '&amp;')
    .replaceAll('<', '&lt;')
    .replaceAll('>', '&gt;')
    .replaceAll('"', '&quot;')
    .replaceAll("'", '&#039;');
}

function formatFrequency(hz, digits = 3) {
  if (!Number.isFinite(hz)) return '—';
  if (Math.abs(hz) >= 1e9) return `${(hz / 1e9).toFixed(digits)} GHz`;
  if (Math.abs(hz) >= 1e6) return `${(hz / 1e6).toFixed(digits)} MHz`;
  if (Math.abs(hz) >= 1e3) return `${(hz / 1e3).toFixed(1)} kHz`;
  return `${hz.toFixed(0)} Hz`;
}

function formatChannelFrequency(hz) {
  return Number.isFinite(hz) ? `${(hz / 1e6).toFixed(6)} MHz` : '—';
}

function updateChannelFrequencyReadout(frequencyHz = state.channelHz) {
  elements.channelFrequency.textContent = formatChannelFrequency(frequencyHz);
  const isValid = Number.isFinite(frequencyHz);
  elements.copyChannelFrequency.disabled = !isValid;
  elements.copyChannelFrequency.title = isValid
    ? `Copia ${Math.round(frequencyHz)} Hz`
    : 'Frequenza non disponibile';
}

async function copyChannelFrequencyHz() {
  if (!Number.isFinite(state.channelHz)) return;
  const frequencyHz = String(Math.round(state.channelHz));
  try {
    if (navigator.clipboard?.writeText) {
      await navigator.clipboard.writeText(frequencyHz);
    } else {
      const fallback = document.createElement('textarea');
      fallback.value = frequencyHz;
      fallback.setAttribute('readonly', '');
      fallback.style.position = 'fixed';
      fallback.style.opacity = '0';
      document.body.appendChild(fallback);
      fallback.select();
      const copied = document.execCommand('copy');
      fallback.remove();
      if (!copied) throw new Error('Copia non supportata');
    }
    toast(`Copiata: ${frequencyHz} Hz`);
  } catch (error) {
    toast(`Impossibile copiare la frequenza: ${error.message}`, 'error');
  }
}

function formatValue(value, suffix = '', digits = 1) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? `${parsed.toFixed(digits)}${suffix}` : '—';
}

function toast(message, type = 'info') {
  elements.toast.textContent = message;
  elements.toast.className = `toast visible ${type}`;
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => { elements.toast.className = 'toast'; }, 3200);
}

async function api(path, options = {}) {
  const response = await fetch(path, {
    headers: { 'Content-Type': 'application/json', ...(options.headers || {}) },
    ...options,
  });
  const payload = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(payload.error || `HTTP ${response.status}`);
  return payload;
}

function fitCanvas(canvas) {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(1, Math.round(rect.width * dpr));
  const height = Math.max(1, Math.round(rect.height * dpr));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  const context = canvas.getContext('2d');
  context.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { context, width: rect.width, height: rect.height, dpr };
}

function powerColor(db) {
  const value = Math.max(0, Math.min(1, (db + 115) / 95));
  const stops = [
    [0.00, [5, 7, 17]],
    [0.18, [25, 18, 70]],
    [0.38, [71, 38, 143]],
    [0.57, [11, 139, 179]],
    [0.73, [81, 229, 195]],
    [0.88, [255, 218, 91]],
    [1.00, [255, 76, 123]],
  ];
  for (let i = 1; i < stops.length; i += 1) {
    if (value <= stops[i][0]) {
      const [p0, c0] = stops[i - 1];
      const [p1, c1] = stops[i];
      const mix = (value - p0) / (p1 - p0);
      return c0.map((channel, index) => Math.round(channel + (c1[index] - channel) * mix));
    }
  }
  return stops.at(-1)[1];
}

const waterfall = {
  width: 0,
  height: 260,
  pixels: null,
  canvas: document.createElement('canvas'),
  context: null,

  clear() {
    this.width = 0;
    this.pixels = null;
    this.context = null;
    this.canvas.width = 1;
    this.canvas.height = this.height;
  },

  reset(width) {
    this.width = width;
    this.canvas.width = width;
    this.canvas.height = this.height;
    this.context = this.canvas.getContext('2d');
    this.pixels = new Uint8ClampedArray(width * this.height * 4);
    for (let i = 3; i < this.pixels.length; i += 4) this.pixels[i] = 255;
  },

  push(bins) {
    if (this.width !== bins.length || !this.pixels) this.reset(bins.length);
    const rowBytes = this.width * 4;
    this.pixels.copyWithin(rowBytes, 0, rowBytes * (this.height - 1));
    for (let x = 0; x < this.width; x += 1) {
      const [red, green, blue] = powerColor(bins[x]);
      const offset = x * 4;
      this.pixels[offset] = red;
      this.pixels[offset + 1] = green;
      this.pixels[offset + 2] = blue;
      this.pixels[offset + 3] = 255;
    }
    this.context.putImageData(new ImageData(this.pixels, this.width, this.height), 0, 0);
  },

  draw() {
    const { context, width, height } = fitCanvas(elements.waterfallCanvas);
    const plot = livePlotMetrics(width);
    context.clearRect(0, 0, width, height);
    context.fillStyle = '#090c13';
    context.fillRect(0, 0, width, height);
    if (this.width > 1) {
      // Crop from the centre of the first bin to the centre of the last one,
      // matching the bin-to-pixel mapping used by the spectrum trace.
      context.drawImage(
        this.canvas,
        0.5, 0, this.width - 1, this.height,
        plot.left, 0, plot.width, height,
      );
    } else if (this.width === 1) {
      context.drawImage(this.canvas, plot.left, 0, plot.width, height);
    } else {
      const gradient = context.createLinearGradient(0, 0, 0, height);
      gradient.addColorStop(0, '#0d1220');
      gradient.addColorStop(1, '#07090f');
      context.fillStyle = gradient;
      context.fillRect(plot.left, 0, plot.width, height);
    }
  },
};

function drawSpectrum() {
  const { context, width, height } = fitCanvas(elements.spectrumCanvas);
  const bins = state.latestBins;
  const { left, right, width: graphWidth } = livePlotMetrics(width);
  const top = 10;
  const bottom = 27;
  const graphHeight = Math.max(1, height - top - bottom);
  const minimumDb = -120;
  const maximumDb = 0;

  context.clearRect(0, 0, width, height);
  context.fillStyle = '#090c13';
  context.fillRect(0, 0, width, height);
  context.strokeStyle = 'rgba(255,255,255,.055)';
  context.lineWidth = 1;
  context.font = '9px ui-monospace, SFMono-Regular, Menlo, monospace';
  context.fillStyle = '#606a7f';

  for (let db = minimumDb; db <= maximumDb; db += 20) {
    const y = top + ((maximumDb - db) / (maximumDb - minimumDb)) * graphHeight;
    context.beginPath();
    context.moveTo(left, Math.round(y) + 0.5);
    context.lineTo(width - right, Math.round(y) + 0.5);
    context.stroke();
    context.fillText(`${db}`, 12, y + 3);
  }
  for (let tick = 0; tick <= 8; tick += 1) {
    const ratio = tick / 8;
    const x = left + ratio * graphWidth;
    context.beginPath();
    context.moveTo(Math.round(x) + 0.5, top);
    context.lineTo(Math.round(x) + 0.5, top + graphHeight);
    context.stroke();
    const frequency = state.centerHz - state.sampleRate / 2 + ratio * state.sampleRate;
    const label = (frequency / 1e6).toFixed(state.sampleRate < 1e6 ? 3 : 2);
    context.fillText(label, Math.max(2, Math.min(width - 45, x - 17)), height - 9);
  }

  if (!bins?.length) {
    context.fillStyle = '#697287';
    context.textAlign = 'center';
    context.font = '11px Inter, ui-sans-serif, sans-serif';
    context.fillText('Avvia una sessione Live per ricevere lo spettro', width / 2, height / 2);
    context.textAlign = 'left';
    waterfall.draw();
    return;
  }

  const path = new Path2D();
  bins.forEach((db, index) => {
    const x = left + (index / Math.max(1, bins.length - 1)) * graphWidth;
    const y = top + ((maximumDb - Math.max(minimumDb, Math.min(maximumDb, db))) / (maximumDb - minimumDb)) * graphHeight;
    if (index === 0) path.moveTo(x, y);
    else path.lineTo(x, y);
  });
  const area = new Path2D(path);
  area.lineTo(width - right, top + graphHeight);
  area.lineTo(left, top + graphHeight);
  area.closePath();
  const fill = context.createLinearGradient(0, top, 0, top + graphHeight);
  fill.addColorStop(0, 'rgba(86,230,255,.38)');
  fill.addColorStop(.45, 'rgba(74,111,226,.16)');
  fill.addColorStop(1, 'rgba(73,43,150,.01)');
  context.fillStyle = fill;
  context.fill(area);
  context.strokeStyle = '#65e9ff';
  context.lineWidth = 1.25;
  context.shadowColor = 'rgba(85,230,255,.45)';
  context.shadowBlur = 5;
  context.stroke(path);
  context.shadowBlur = 0;
  waterfall.draw();
}

function scheduleSpectrumDraw() {
  if (state.spectrumPending) return;
  state.spectrumPending = true;
  requestAnimationFrame(() => {
    state.spectrumPending = false;
    drawSpectrum();
  });
}

function drawScan() {
  const scan = state.scan;
  const { context, width, height } = fitCanvas(elements.scanCanvas);
  const left = 54;
  const right = 18;
  const top = 18;
  const bottom = 35;
  const graphWidth = Math.max(1, width - left - right);
  const graphHeight = Math.max(1, height - top - bottom);
  const viewSpan = scan.viewEndHz - scan.viewStartHz;
  const points = [...scan.points.entries()]
    .filter(([frequency]) => frequency >= scan.viewStartHz && frequency <= scan.viewEndHz)
    .sort((a, b) => a[0] - b[0]);

  context.clearRect(0, 0, width, height);
  const background = context.createLinearGradient(0, 0, 0, height);
  background.addColorStop(0, '#0d111d');
  background.addColorStop(1, '#070910');
  context.fillStyle = background;
  context.fillRect(0, 0, width, height);
  context.strokeStyle = 'rgba(255,255,255,.055)';
  context.fillStyle = '#606a7f';
  context.font = '9px ui-monospace, SFMono-Regular, Menlo, monospace';

  for (let db = -120; db <= 0; db += 20) {
    const y = top + ((0 - db) / 120) * graphHeight;
    context.beginPath(); context.moveTo(left, y); context.lineTo(width - right, y); context.stroke();
    context.fillText(`${db}`, 15, y + 3);
  }
  const tickCount = width < 700 ? 5 : 10;
  for (let tick = 0; tick <= tickCount; tick += 1) {
    const ratio = tick / tickCount;
    const x = left + ratio * graphWidth;
    context.beginPath(); context.moveTo(x, top); context.lineTo(x, top + graphHeight); context.stroke();
    const frequency = scan.viewStartHz + ratio * viewSpan;
    context.fillText((frequency / 1e6).toFixed(viewSpan < 50e6 ? 2 : 0), Math.max(2, Math.min(width - 40, x - 14)), height - 12);
  }

  if (points.length === 0) {
    context.fillStyle = '#697287';
    context.textAlign = 'center';
    context.font = '11px Inter, ui-sans-serif, sans-serif';
    context.fillText('Avvia Scan per costruire la mappa wideband', width / 2, height / 2);
    context.textAlign = 'left';
  } else {
    const line = new Path2D();
    for (let index = 0; index < points.length; index += 1) {
      const [frequency, db] = points[index];
      const x = left + ((frequency - scan.viewStartHz) / viewSpan) * graphWidth;
      const y = top + ((0 - Math.max(-120, Math.min(0, db))) / 120) * graphHeight;
      if (index === 0) line.moveTo(x, y); else line.lineTo(x, y);
      const [red, green, blue] = powerColor(db);
      context.fillStyle = `rgba(${red},${green},${blue},.68)`;
      const barWidth = Math.max(1, graphWidth / Math.max(points.length, 1) + .4);
      context.fillRect(x, top + graphHeight - 34, barWidth, 34);
    }
    const area = new Path2D(line);
    area.lineTo(left + graphWidth, top + graphHeight);
    area.lineTo(left, top + graphHeight);
    area.closePath();
    const fill = context.createLinearGradient(0, top, 0, top + graphHeight);
    fill.addColorStop(0, 'rgba(140,112,255,.38)');
    fill.addColorStop(1, 'rgba(61,38,140,.015)');
    context.fillStyle = fill;
    context.fill(area);
    context.strokeStyle = '#9b85ff';
    context.lineWidth = 1.1;
    context.shadowColor = 'rgba(140,112,255,.55)';
    context.shadowBlur = 5;
    context.stroke(line);
    context.shadowBlur = 0;
  }

  if (scan.hoverX !== null) {
    const x = Math.max(left, Math.min(width - right, scan.hoverX));
    const frequency = scan.viewStartHz + ((x - left) / graphWidth) * viewSpan;
    context.strokeStyle = 'rgba(85,230,255,.6)';
    context.beginPath(); context.moveTo(x, top); context.lineTo(x, top + graphHeight); context.stroke();
    const label = formatFrequency(frequency, viewSpan < 30e6 ? 4 : 3);
    context.font = '10px ui-monospace, monospace';
    const textWidth = context.measureText(label).width;
    const labelX = Math.max(left, Math.min(width - right - textWidth - 14, x + 8));
    context.fillStyle = 'rgba(15,20,31,.94)';
    context.fillRect(labelX, top + 8, textWidth + 12, 22);
    context.fillStyle = '#6eeaff';
    context.fillText(label, labelX + 6, top + 23);
  }
}

function scheduleScanDraw() {
  if (state.scan.renderPending) return;
  state.scan.renderPending = true;
  requestAnimationFrame(() => {
    state.scan.renderPending = false;
    drawScan();
  });
}

function protocolControlMarkup(protocol) {
  if (protocol === 'voice') {
    return `
      <div class="section-title"><span>Audio Voice</span><small>AM / FM</small></div>
      <label class="field"><span>Demodulazione</span><select id="voice-demod"><option value="fm">FM</option><option value="am">AM</option></select></label>
      <label class="field"><span>Sample rate / span</span><select id="voice-sample-rate"><option value="240000">240 kHz</option><option value="960000">960 kHz</option><option value="2400000" selected>2.4 MHz</option></select></label>
      <label class="field"><span>Larghezza filtro <output id="voice-filter-width-output">15.0 kHz</output></span><input id="voice-filter-width" type="number" min="1000" max="500000" step="100" value="15000" /></label>
      <label class="field"><span>Tipo filtro</span><select id="voice-filter-type"><option value="iir" selected>IIR Butterworth</option><option value="fir">FIR Kaiser</option></select></label>
      <label class="toggle-row"><span><b>Squelch</b><small>Silenzia il rumore sotto soglia</small></span><input id="squelch-enabled" type="checkbox" /><i></i></label>
      <label class="field range-field"><span><b>Soglia squelch</b><output id="squelch-output">−30 dBFS</output></span><input id="squelch-input" type="range" min="-120" max="0" step="1" value="-30" /></label>`;
  }
  if (protocol === 'adsb') {
    return `
      <div class="section-title"><span>ADS-B / Mode S</span><small>1090ES</small></div>
      <label class="field"><span>Latitudine ricevitore</span><input id="adsb-latitude" type="number" min="-90" max="90" step="0.0001" placeholder="Opzionale" /></label>
      <label class="field"><span>Longitudine ricevitore</span><input id="adsb-longitude" type="number" min="-180" max="180" step="0.0001" placeholder="Opzionale" /></label>`;
  }
  if (protocol === 'ais') {
    return `
      <div class="section-title"><span>AIS marittimo</span><small>162 MHz</small></div>
      <label class="field"><span>Canale</span><select id="ais-channel"><option value="both">A + B simultanei</option><option value="A">Solo A — 161.975</option><option value="B">Solo B — 162.025</option></select></label>`;
  }
  if (protocol === 'sonde') {
    return `<div class="section-title"><span>Radiosonda</span><small>RS41</small></div><div class="empty-state"><div><strong>Decoder automatico</strong><small>La frequenza dipende dalla sonda osservata, normalmente nella banda 400–406 MHz.</small></div></div>`;
  }
  return `
    <div class="section-title"><span>Slow Scan TV</span><small>VIS autodetect</small></div>
    <label class="field"><span>Formato SSTV</span><select id="sstv-mode"><option value="auto">Automatico</option><option value="pd120">PD120</option><option value="martin-m1">Martin M1</option></select></label>
    <label class="field"><span>Cartella immagini</span><input id="save-directory" type="text" value="immagini-sstv" /></label>`;
}

function updateProtocolControls() {
  const protocol = elements.protocol.value;
  elements.protocolControls.innerHTML = protocolControlMarkup(protocol);
  const sampleRate = $('#voice-sample-rate');
  sampleRate?.addEventListener('change', updateSpanLabel);
  const filterWidth = $('#voice-filter-width');
  filterWidth?.addEventListener('input', updateFilterBand);
  filterWidth?.addEventListener('change', () => {
    setVoiceFilterWidth(filterWidth.value);
    void updateRuntimeFilter();
  });
  $('#voice-filter-type')?.addEventListener('change', updateRuntimeFilter);
  const squelch = $('#squelch-input');
  squelch?.addEventListener('input', () => { $('#squelch-output').textContent = `${squelch.value.replace('-', '−')} dBFS`; });
  squelch?.addEventListener('change', updateRuntimeSquelch);
  $('#squelch-enabled')?.addEventListener('change', updateRuntimeSquelch);
  renderDecoder();
  updateSpanLabel();
  updateFilterBand();
  renderRecordingControls();
}

function formatRecordingBytes(bytes) {
  const value = Number(bytes) || 0;
  if (value >= 1e9) return `${(value / 1e9).toFixed(2)} GB`;
  if (value >= 1e6) return `${(value / 1e6).toFixed(1)} MB`;
  if (value >= 1e3) return `${(value / 1e3).toFixed(1)} kB`;
  return `${value} B`;
}

function renderRecordingControls() {
  if (!elements.recordingControls) return;
  const sessionReady = state.mode === 'live'
    && state.processMode === 'live'
    && ['starting', 'receiving'].includes(state.processStatus);
  elements.audioRecordingBlock.classList.toggle(
    'hidden', elements.protocol.value !== 'voice');

  for (const kind of ['audio', 'iq']) {
    const recording = state.recordings[kind];
    const pending = state.recordingPending[kind];
    const button = kind === 'audio'
      ? elements.audioRecordingButton : elements.iqRecordingButton;
    const status = kind === 'audio'
      ? elements.audioRecordingStatus : elements.iqRecordingStatus;
    const format = kind === 'audio'
      ? elements.audioRecordingFormat : elements.iqRecordingFormat;
    const path = kind === 'audio'
      ? elements.audioRecordingPath : elements.iqRecordingPath;
    const available = sessionReady
      && (kind !== 'audio' || elements.protocol.value === 'voice');
    const busy = pending || Boolean(recording.stopping);
    button.disabled = !available || busy;
    button.classList.toggle('active', Boolean(recording.active));
    button.querySelector('span').textContent = recording.active
      ? `Ferma registrazione ${kind === 'audio' ? 'audio' : 'I/Q'}`
      : `Registra ${kind === 'audio' ? 'audio' : 'I/Q'}`;
    format.disabled = Boolean(recording.active) || busy;
    path.disabled = Boolean(recording.active) || busy;
    if (recording.path && (recording.active || recording.stopping)) {
      path.value = recording.path;
    }
    status.classList.toggle('active', Boolean(recording.active));
    if (recording.active) status.textContent = '● REC';
    else if (recording.stopping) status.textContent = 'Salvataggio…';
    else if (recording.error) status.textContent = 'Errore';
    else if (recording.path && recording.bytes > 0) {
      status.textContent = `Salvato · ${formatRecordingBytes(recording.bytes)}`;
    } else status.textContent = 'Fermo';
    status.title = recording.error || recording.path || '';
  }
}

async function toggleRecording(kind) {
  const recording = state.recordings[kind];
  const format = kind === 'audio'
    ? elements.audioRecordingFormat : elements.iqRecordingFormat;
  const path = kind === 'audio'
    ? elements.audioRecordingPath : elements.iqRecordingPath;
  const action = recording.active ? 'stop' : 'start';
  state.recordingPending[kind] = true;
  renderRecordingControls();
  try {
    const result = await api('/api/live/recording', {
      method: 'POST',
      body: JSON.stringify({
        kind,
        action,
        format: format.value,
        path: path.value,
      }),
    });
    if (result.path) path.value = result.path;
    toast(action === 'start'
      ? `Registrazione ${kind === 'audio' ? 'audio' : 'I/Q'} avviata`
      : `Registrazione ${kind === 'audio' ? 'audio' : 'I/Q'} salvata`);
  } catch (error) {
    toast(error.message, 'error');
  } finally {
    state.recordingPending[kind] = false;
    renderRecordingControls();
  }
}

function updateSpanLabel() {
  const protocol = elements.protocol.value;
  const sampleRate = protocolSampleRate(protocol);
  elements.span.textContent = formatFrequency(sampleRate, sampleRate >= 1e6 ? 3 : 0);
  updateFilterBand();
}

function resetLiveSpectrum(protocol, centerHz) {
  state.latestBins = null;
  state.centerHz = centerHz;
  state.channelHz = centerHz;
  state.sampleRate = protocolSampleRate(protocol);
  state.framesThisSecond = 0;
  state.measuredFps = 0;
  state.lastFpsAt = performance.now();
  state.receiverStatus = { channelDbfs: null, thresholdDbfs: null, squelchOpen: null };
  waterfall.clear();
  elements.peak.textContent = '— dBFS';
  elements.fpsValue.textContent = '— FPS';
  elements.fftValue.textContent = elements.fftSize.value;
  elements.span.textContent = formatFrequency(
    state.sampleRate, state.sampleRate >= 1e6 ? 3 : 0,
  );
  positionLiveChannelMarker();
  scheduleSpectrumDraw();
}

function renderGainOutput() {
  if (elements.gainAuto.checked) {
    elements.gainOutput.textContent = 'Auto';
    elements.gainOutput.title = 'Controllo automatico del tuner';
    return;
  }
  const requestedGainDb = Number(elements.gainInput.value);
  if (Number.isFinite(state.appliedGainDb)) {
    elements.gainOutput.textContent = `${requestedGainDb} dB · eff. ${state.appliedGainDb.toFixed(1)} dB`;
    elements.gainOutput.title = `Gain richiesto: ${requestedGainDb} dB; applicato dal tuner: ${state.appliedGainDb.toFixed(1)} dB`;
  } else {
    elements.gainOutput.textContent = `${requestedGainDb} dB`;
    elements.gainOutput.title = 'Gain manuale richiesto';
  }
}

async function updateRuntimeGain() {
  clearTimeout(gainUpdateTimer);
  renderGainOutput();
  if (state.processMode !== 'live'
      || !['starting', 'receiving'].includes(state.processStatus)) return;
  try {
    await api('/api/live/gain', {
      method: 'POST',
      body: JSON.stringify({
        manualGain: !elements.gainAuto.checked,
        gainDb: Number(elements.gainInput.value),
      }),
    });
  } catch (error) {
    toast(error.message, 'error');
  }
}

function scheduleRuntimeGain() {
  clearTimeout(gainUpdateTimer);
  gainUpdateTimer = setTimeout(() => { void updateRuntimeGain(); }, 120);
}

function selectedVoiceSampleRate() {
  return Number($('#voice-sample-rate')?.value || state.sampleRate || 2_400_000);
}

function filterWidthLimits() {
  const sampleRate = selectedVoiceSampleRate();
  return { min: 1000, max: Math.max(1000, Math.min(500000, sampleRate - 1)) };
}

function normalizeFilterWidth(widthHz) {
  const { min, max } = filterWidthLimits();
  return Math.max(min, Math.min(max, Math.round(Number(widthHz) / 100) * 100));
}

function formatFilterWidth(widthHz) {
  return widthHz >= 1000 ? `${(widthHz / 1000).toFixed(1)} kHz` : `${widthHz} Hz`;
}

function setVoiceFilterWidth(widthHz) {
  const input = $('#voice-filter-width');
  if (!input) return 0;
  const normalized = normalizeFilterWidth(widthHz);
  const { max } = filterWidthLimits();
  input.max = String(max);
  input.value = String(normalized);
  $('#voice-filter-width-output').textContent = formatFilterWidth(normalized);
  elements.filterPassbandLabel.textContent = formatFilterWidth(normalized);
  updateFilterBand();
  return normalized;
}

function updateFilterBand(channelOverrideHz) {
  const input = $('#voice-filter-width');
  const visible = state.mode === 'live'
    && elements.protocol.value === 'voice' && Boolean(input);
  elements.filterPassband.classList.toggle('hidden', !visible);
  if (!visible) return;
  const sampleRate = state.processMode === 'live'
    ? state.sampleRate : selectedVoiceSampleRate();
  const channelHz = Number.isFinite(channelOverrideHz)
    ? channelOverrideHz
    : Number.isFinite(state.channelHz) ? state.channelHz : state.centerHz;
  const ratio = sampleRate > 0
    ? Math.max(0, Math.min(1, 0.5 + (channelHz - state.centerHz) / sampleRate))
    : 0.5;
  const widthHz = normalizeFilterWidth(input.value);
  const plot = livePlotMetrics(elements.liveVisual.getBoundingClientRect().width);
  const centerX = plot.left + ratio * plot.width;
  const widthPx = sampleRate > 0 ? widthHz / sampleRate * plot.width : 0;
  elements.filterPassband.style.left = `${centerX - widthPx / 2}px`;
  elements.filterPassband.style.width = `${widthPx}px`;
  elements.filterPassbandLabel.textContent = formatFilterWidth(widthHz);
  $('#voice-filter-width-output').textContent = formatFilterWidth(widthHz);
}

async function updateRuntimeFilter() {
  if (state.processMode !== 'live'
      || !['starting', 'receiving'].includes(state.processStatus)
      || elements.protocol.value !== 'voice') return;
  const widthHz = setVoiceFilterWidth($('#voice-filter-width')?.value);
  try {
    await api('/api/live/filter', {
      method: 'POST',
      body: JSON.stringify({
        widthHz,
        type: $('#voice-filter-type')?.value || 'iir',
      }),
    });
  } catch (error) {
    toast(error.message, 'error');
  }
}

function previewFilterResize(clientX) {
  const resize = state.filterResize;
  if (!resize.dragging || !resize.rect) return;
  const sampleRate = state.processMode === 'live'
    ? state.sampleRate : selectedVoiceSampleRate();
  const channelHz = Number.isFinite(state.channelHz) ? state.channelHz : state.centerHz;
  const ratio = sampleRate > 0
    ? Math.max(0, Math.min(1, 0.5 + (channelHz - state.centerHz) / sampleRate))
    : 0.5;
  const plot = livePlotMetrics(resize.rect.width);
  const centerX = resize.rect.left + plot.left + ratio * plot.width;
  const halfWidthPx = Math.abs(clientX - centerX);
  setVoiceFilterWidth(2 * halfWidthPx / plot.width * sampleRate);
}

function beginFilterResize(event) {
  if (event.button !== 0 || elements.protocol.value !== 'voice') return;
  event.preventDefault();
  event.stopPropagation();
  state.filterResize.dragging = true;
  state.filterResize.rect = elements.spectrumCanvas.getBoundingClientRect();
  elements.filterPassband.classList.add('dragging');
  previewFilterResize(event.clientX);
}

function moveFilterResize(event) {
  if (!state.filterResize.dragging) return;
  previewFilterResize(event.clientX);
}

function finishFilterResize(event) {
  if (!state.filterResize.dragging) return;
  previewFilterResize(event.clientX);
  state.filterResize.dragging = false;
  state.filterResize.rect = null;
  elements.filterPassband.classList.remove('dragging');
  void updateRuntimeFilter();
}

function handleFilterResizeKey(event) {
  if (!['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown'].includes(event.key)) return;
  event.preventDefault();
  const grow = event.key === 'ArrowRight' || event.key === 'ArrowUp';
  const step = event.shiftKey ? 100 : 1000;
  setVoiceFilterWidth(Number($('#voice-filter-width')?.value || 15000)
    + (grow ? step : -step));
  void updateRuntimeFilter();
}

function collectCommonConfig() {
  return {
    device: elements.device.value,
    manualGain: !elements.gainAuto.checked,
    gainDb: Number(elements.gainInput.value),
    ppm: Number(elements.ppm.value),
    tunerBandwidth: Number(elements.tunerBandwidth.value),
  };
}

function collectLiveConfig(frequencyOverride) {
  const protocol = elements.protocol.value;
  const config = {
    ...collectCommonConfig(),
    protocol,
    frequencyMhz: frequencyOverride ?? Number(elements.frequency.value),
    fftSize: Number(elements.fftSize.value),
    fps: Number(elements.fpsInput.value),
  };
  if (protocol === 'voice') {
    Object.assign(config, {
      demod: $('#voice-demod').value,
      sampleRate: Number($('#voice-sample-rate').value),
      filterWidth: Number($('#voice-filter-width').value),
      filterType: $('#voice-filter-type').value,
      squelchEnabled: $('#squelch-enabled').checked,
      squelchDbfs: Number($('#squelch-input').value),
    });
  } else if (protocol === 'adsb') {
    config.latitude = $('#adsb-latitude').value;
    config.longitude = $('#adsb-longitude').value;
  } else if (protocol === 'ais') {
    config.aisChannel = $('#ais-channel').value;
  } else if (protocol === 'sstv') {
    config.sstvMode = $('#sstv-mode').value;
    config.saveDirectory = $('#save-directory').value;
  }
  return config;
}

function collectScanConfig() {
  return {
    ...collectCommonConfig(),
    startMhz: Number(elements.scanStart.value),
    endMhz: Number(elements.scanEnd.value),
    binHz: Number(elements.scanResolution.value),
    integrationSeconds: 1,
  };
}

async function updateRuntimeSquelch() {
  const enabledControl = $('#squelch-enabled');
  const thresholdControl = $('#squelch-input');
  if (!enabledControl || !thresholdControl
      || state.processMode !== 'live'
      || !['starting', 'receiving'].includes(state.processStatus)) {
    return;
  }
  try {
    await api('/api/live/squelch', {
      method: 'POST',
      body: JSON.stringify({
        enabled: enabledControl.checked,
        thresholdDbfs: Number(thresholdControl.value),
      }),
    });
    state.receiverStatus.thresholdDbfs = Number(thresholdControl.value);
    if (!enabledControl.checked) state.receiverStatus.squelchOpen = null;
    renderVoiceDecoder();
  } catch (error) {
    toast(error.message, 'error');
  }
}

function setMode(mode) {
  state.mode = mode;
  elements.modeButtons.forEach((button) => {
    const active = button.dataset.mode === mode;
    button.classList.toggle('active', active);
    button.setAttribute('aria-selected', String(active));
  });
  elements.liveTune.classList.toggle('hidden', mode !== 'live');
  elements.scanTune.classList.toggle('hidden', mode !== 'scan');
  elements.liveVisual.classList.toggle('hidden', mode !== 'live');
  elements.scanVisual.classList.toggle('hidden', mode !== 'scan');
  $$('.live-only-control').forEach((element) => element.classList.toggle('hidden', mode !== 'live'));
  elements.protocolControls.classList.toggle('hidden', mode !== 'live');
  $('#protocol-section-rule').classList.toggle('hidden', mode !== 'live');
  elements.startLabel.textContent = mode === 'live' ? 'Avvia Live' : 'Avvia Scan';
  if (mode === 'live') {
    scheduleSpectrumDraw();
  } else {
    scheduleScanDraw();
  }
  updateFilterBand();
  renderRecordingControls();
  renderDecoder();
}

function setBusy(busy) {
  elements.start.disabled = busy;
  elements.stop.disabled = !busy;
}

async function startCurrentMode() {
  try {
    setBusy(true);
    if (state.mode === 'live') {
      const config = collectLiveConfig();
      state.centerHz = config.frequencyMhz * 1e6;
      state.channelHz = state.centerHz;
      positionLiveChannelMarker();
      await api('/api/live/start', { method: 'POST', body: JSON.stringify(config) });
      toast('Sessione Live avviata');
    } else {
      const config = collectScanConfig();
      state.scan.points.clear();
      state.scan.startHz = config.startMhz * 1e6;
      state.scan.endHz = config.endMhz * 1e6;
      state.scan.viewStartHz = state.scan.startHz;
      state.scan.viewEndHz = state.scan.endHz;
      state.scan.progress = 0;
      updateScanProgress(0);
      scheduleScanDraw();
      await api('/api/scan/start', { method: 'POST', body: JSON.stringify(config) });
      toast('Scansione wideband avviata');
    }
  } catch (error) {
    setBusy(false);
    toast(error.message, 'error');
  }
}

async function stopSession() {
  try {
    elements.stop.disabled = true;
    await api('/api/stop', { method: 'POST', body: '{}' });
  } catch (error) {
    toast(error.message, 'error');
  }
}

async function openLiveAtFrequency(frequencyHz) {
  const normalizedHz = syncFrequencyEditor(frequencyHz);
  const frequencyMhz = normalizedHz / 1e6;
  state.centerHz = normalizedHz;
  state.channelHz = normalizedHz;
  positionLiveChannelMarker();
  setMode('live');
  try {
    setBusy(true);
    await api('/api/live/start', { method: 'POST', body: JSON.stringify(collectLiveConfig(frequencyMhz)) });
    toast(`Live centrato a ${frequencyMhz.toFixed(3)} MHz`);
  } catch (error) {
    setBusy(false);
    toast(error.message, 'error');
  }
}

async function retuneLive(frequencyHz) {
  const normalizedHz = syncFrequencyEditor(frequencyHz);
  const frequencyMhz = normalizedHz / 1e6;
  state.centerHz = normalizedHz;
  state.channelHz = normalizedHz;
  positionLiveChannelMarker();
  if (state.processMode !== 'live'
      || !['starting', 'receiving'].includes(state.processStatus)) {
    return;
  }
  try {
    await api('/api/live/tune', {
      method: 'POST',
      body: JSON.stringify({ frequencyMhz }),
    });
  } catch (error) {
    elements.tuningMarker.style.left = `${livePlotX(
      0.5, elements.liveVisual.getBoundingClientRect().width,
    )}px`;
    toast(error.message, 'error');
  }
}

function positionLiveChannelMarker() {
  const channelHz = Number.isFinite(state.channelHz)
    ? state.channelHz : state.centerHz;
  const ratio = state.sampleRate > 0
    ? Math.max(0, Math.min(1,
      0.5 + (channelHz - state.centerHz) / state.sampleRate))
    : 0.5;
  const visualWidth = elements.liveVisual.getBoundingClientRect().width;
  elements.tuningMarker.style.left = `${livePlotX(ratio, visualWidth)}px`;
  elements.tuningTooltip.textContent = formatFrequency(channelHz, 4);
  updateChannelFrequencyReadout(channelHz);
  updateFilterBand();
}

async function selectLiveChannel(frequencyHz) {
  state.channelHz = frequencyHz;
  positionLiveChannelMarker();
  if (state.mode === 'live' && elements.protocol.value === 'voice') {
    renderVoiceDecoder();
  }
  if (state.processMode !== 'live'
      || !['starting', 'receiving'].includes(state.processStatus)) return;
  try {
    await api('/api/live/channel', {
      method: 'POST',
      body: JSON.stringify({ frequencyMhz: frequencyHz / 1e6 }),
    });
  } catch (error) {
    state.channelHz = state.centerHz;
    positionLiveChannelMarker();
    toast(error.message, 'error');
  }
}

function tuneLiveChannelWithWheel(event) {
  if (state.mode !== 'live' || event.deltaY === 0) return;
  const spectrumRect = elements.spectrumCanvas.getBoundingClientRect();
  const waterfallRect = elements.waterfallCanvas.getBoundingClientRect();
  const overSpectrum = event.clientY >= spectrumRect.top && event.clientY <= spectrumRect.bottom;
  const overWaterfall = event.clientY >= waterfallRect.top && event.clientY <= waterfallRect.bottom;
  if (!overSpectrum && !overWaterfall) return;

  event.preventDefault();
  const direction = event.deltaY < 0 ? 1 : -1;
  const baseFrequencyHz = Number.isFinite(state.channelHz) ? state.channelHz : state.centerHz;
  const halfSpanHz = Math.max(0, state.sampleRate / 2);
  const minimumHz = state.centerHz - halfSpanHz;
  const maximumHz = state.centerHz + halfSpanHz;
  const frequencyHz = Math.min(maximumHz, Math.max(minimumHz,
    Math.round(baseFrequencyHz + direction * CHANNEL_WHEEL_STEP_HZ)));

  state.channelHz = frequencyHz;
  positionLiveChannelMarker();
  if (elements.protocol.value === 'voice') renderVoiceDecoder();

  elements.tuningMarker.classList.add('wheel-tuning');
  clearTimeout(channelWheelFeedbackTimer);
  channelWheelFeedbackTimer = setTimeout(() => {
    elements.tuningMarker.classList.remove('wheel-tuning');
  }, 650);

  clearTimeout(channelWheelCommitTimer);
  channelWheelCommitTimer = setTimeout(() => {
    void selectLiveChannel(frequencyHz);
  }, 80);
}

function previewLiveTune(clientX) {
  const tune = state.liveTune;
  if (!tune.rect) return;
  const ratio = livePlotRatio(clientX, tune.rect);
  tune.frequencyHz = tune.centerHz - tune.sampleRate / 2
                     + ratio * tune.sampleRate;
  elements.tuningMarker.style.left = `${livePlotX(ratio, tune.rect.width)}px`;
  elements.tuningTooltip.textContent = formatFrequency(tune.frequencyHz, 4);
  updateChannelFrequencyReadout(tune.frequencyHz);
  updateFilterBand(tune.frequencyHz);
}

function beginLiveTune(event) {
  if (event.button !== 0) return;
  event.preventDefault();
  const sourceIsMarker = event.currentTarget === elements.tuningMarker;
  state.liveTune.dragging = true;
  state.liveTune.rect = (sourceIsMarker ? elements.liveVisual : event.currentTarget)
    .getBoundingClientRect();
  state.liveTune.centerHz = state.centerHz;
  state.liveTune.sampleRate = state.sampleRate;
  elements.tuningMarker.classList.add('dragging');
  previewLiveTune(event.clientX);
}

function moveLiveTune(event) {
  if (!state.liveTune.dragging) return;
  previewLiveTune(event.clientX);
}

function finishLiveTune(event) {
  if (!state.liveTune.dragging) return;
  previewLiveTune(event.clientX);
  const frequencyHz = state.liveTune.frequencyHz;
  state.liveTune.dragging = false;
  state.liveTune.rect = null;
  elements.tuningMarker.classList.remove('dragging');
  if (Number.isFinite(frequencyHz)) void selectLiveChannel(frequencyHz);
}

function updateScanProgress(progress) {
  const percent = Math.round(Math.max(0, Math.min(1, progress)) * 100);
  elements.scanProgressValue.textContent = `${percent}%`;
  elements.scanProgressBar.style.width = `${percent}%`;
  elements.scanProgressLabel.textContent = percent >= 100 ? 'Mappa completata' : percent > 0 ? 'Scansione dello spettro in corso' : 'Mappa pronta per la scansione';
  elements.scanProgressDetail.textContent = percent > 0 && percent < 100
    ? `${formatFrequency(state.scan.startHz)} → ${formatFrequency(state.scan.endHz)}`
    : 'Usa la rotella per lo zoom e trascina per spostarti';
}

function renderVoiceDecoder() {
  elements.decoderTitle.textContent = 'Audio e attività';
  const squelchEnabled = $('#squelch-enabled')?.checked;
  const squelchLabel = !squelchEnabled
    ? 'Squelch disattivato'
    : state.receiverStatus.squelchOpen === true
      ? 'Squelch aperto'
      : state.receiverStatus.squelchOpen === false
        ? 'Squelch chiuso'
        : 'Squelch in attesa';
  const channelLevel = Number.isFinite(state.receiverStatus.channelDbfs)
    ? ` Livello canale ${state.receiverStatus.channelDbfs.toFixed(1)} dBFS.`
    : '';
  const tunedChannel = Number.isFinite(state.channelHz)
    ? ` Canale selezionato ${formatFrequency(state.channelHz, 4)}.`
    : '';
  elements.decoderContent.innerHTML = `
    <div class="voice-viz">
      <div><span class="signal-pill">● ${escapeHtml(squelchLabel)}</span><h3>${escapeHtml($('#voice-demod')?.value?.toUpperCase() || 'FM')} demodulation</h3><p>L’audio decodificato viene riprodotto dall’uscita predefinita del computer.${escapeHtml(tunedChannel)}${escapeHtml(channelLevel)} Il waterfall resta centrato sulla frequenza impostata in alto.</p></div>
      <div class="audio-meter" aria-hidden="true">${Array.from({ length: 24 }, (_, index) => `<i style="animation-duration:${1 + (index % 7) * .13}s"></i>`).join('')}</div>
    </div>`;
}

function tableMarkup(headers, rows) {
  if (rows.length === 0) return '<div class="empty-state"><div><strong>Nessun dato decodificato</strong><small>Le righe appariranno quando saranno ricevuti frame validi.</small></div></div>';
  return `<table class="data-table"><thead><tr>${headers.map((header) => `<th>${escapeHtml(header)}</th>`).join('')}</tr></thead><tbody>${rows.join('')}</tbody></table>`;
}

function renderAdsb() {
  elements.decoderTitle.textContent = 'Aeromobili ADS-B';
  const rows = [...state.adsb.values()].sort((a, b) => b.updatedAt - a.updatedAt).slice(0, 100).map((item) => `<tr>
    <td class="primary-cell">${escapeHtml(item.icao || '—')}</td><td>${escapeHtml(item.callsign || '—')}</td><td>${formatValue(item.altitude_ft, ' ft', 0)}</td><td>${formatValue(item.speed_kt, ' kt', 0)}</td><td>${formatValue(item.track_deg, '°', 0)}</td><td>${item.latitude !== undefined ? `${Number(item.latitude).toFixed(4)}, ${Number(item.longitude).toFixed(4)}` : '—'}</td><td>${Math.round((Date.now() - item.updatedAt) / 1000)}s</td></tr>`);
  elements.decoderContent.innerHTML = tableMarkup(['ICAO', 'Callsign', 'Quota', 'Velocità', 'Rotta', 'Posizione', 'Età'], rows);
}

function renderAis() {
  elements.decoderTitle.textContent = 'Traffico AIS';
  const rows = [...state.ais.values()].sort((a, b) => b.updatedAt - a.updatedAt).slice(0, 100).map((item) => `<tr>
    <td class="primary-cell">${escapeHtml(item.mmsi || '—')}</td><td>${escapeHtml(item.vessel_name || item.name || item.callsign || '—')}</td><td>${escapeHtml(item.channel || '—')}</td><td>${formatValue(item.speed_over_ground_kt, ' kt', 1)}</td><td>${formatValue(item.course_over_ground_deg, '°', 1)}</td><td>${item.latitude !== undefined ? `${Number(item.latitude).toFixed(4)}, ${Number(item.longitude).toFixed(4)}` : '—'}</td><td>${escapeHtml(item.message_name || '—')}</td></tr>`);
  elements.decoderContent.innerHTML = tableMarkup(['MMSI', 'Nave', 'Canale', 'SOG', 'COG', 'Posizione', 'Messaggio'], rows);
}

function renderSonde() {
  elements.decoderTitle.textContent = 'Radiosonde RS41';
  const rows = [...state.sonde.values()].sort((a, b) => b.updatedAt - a.updatedAt).slice(0, 100).map((item) => `<tr>
    <td class="primary-cell">${escapeHtml(item.serial || '—')}</td><td>${escapeHtml(item.frame ?? '—')}</td><td>${formatValue(item.altitude_m, ' m', 0)}</td><td>${formatValue(item.speed_mps, ' m/s', 1)}</td><td>${formatValue(item.climb_mps, ' m/s', 1)}</td><td>${escapeHtml(item.satellites ?? '—')}</td><td>${item.latitude !== undefined ? `${Number(item.latitude).toFixed(4)}, ${Number(item.longitude).toFixed(4)}` : '—'}</td></tr>`);
  elements.decoderContent.innerHTML = tableMarkup(['Seriale', 'Frame', 'Quota', 'Velocità', 'Salita', 'Sat', 'Posizione'], rows);
}

async function paintPpm(url, canvas) {
  const bytes = new Uint8Array(await (await fetch(url)).arrayBuffer());
  let position = 0;
  const readToken = () => {
    while (position < bytes.length) {
      if (bytes[position] === 35) {
        while (position < bytes.length && bytes[position] !== 10) position += 1;
      } else if (bytes[position] <= 32) position += 1;
      else break;
    }
    const start = position;
    while (position < bytes.length && bytes[position] > 32) position += 1;
    return new TextDecoder().decode(bytes.slice(start, position));
  };
  if (readToken() !== 'P6') throw new Error('Unsupported SSTV image format');
  const width = Number(readToken());
  const height = Number(readToken());
  const maximum = Number(readToken());
  while (position < bytes.length && bytes[position] <= 32) position += 1;
  if (!width || !height || maximum !== 255) throw new Error('Invalid SSTV image');
  canvas.width = width;
  canvas.height = height;
  const rgba = new Uint8ClampedArray(width * height * 4);
  for (let source = position, target = 0; target < rgba.length; source += 3, target += 4) {
    rgba[target] = bytes[source]; rgba[target + 1] = bytes[source + 1]; rgba[target + 2] = bytes[source + 2]; rgba[target + 3] = 255;
  }
  canvas.getContext('2d').putImageData(new ImageData(rgba, width, height), 0, 0);
}

function renderSstv() {
  elements.decoderTitle.textContent = 'Immagini SSTV';
  const progress = state.sstv.total ? Math.round((state.sstv.progress / state.sstv.total) * 100) : 0;
  elements.decoderContent.innerHTML = `<div class="sstv-layout"><div class="sstv-preview">${state.sstv.artifactUrl ? '<canvas id="sstv-canvas"></canvas>' : '<div class="empty-state"><div><strong>In attesa di un’immagine</strong><small>L’anteprima apparirà al termine della ricezione.</small></div></div>'}</div><div class="sstv-copy"><span class="signal-pill">${escapeHtml(state.sstv.event)}</span><h3>${progress ? `${progress}% ricevuto` : 'VIS autodetect attivo'}</h3><p>${escapeHtml(state.sstv.path || 'Le immagini complete vengono salvate nella cartella configurata.')}</p>${state.sstv.artifactUrl ? `<a class="text-link" href="${escapeHtml(state.sstv.artifactUrl)}" download>Salva una copia →</a>` : ''}</div></div>`;
  if (state.sstv.artifactUrl) paintPpm(state.sstv.artifactUrl, $('#sstv-canvas')).catch((error) => addLog(error.message, 'error'));
}

function renderScanPeaks() {
  elements.decoderTitle.textContent = 'Segnali rilevati';
  const points = [...state.scan.points.entries()]
    .sort((a, b) => b[1] - a[1])
    .slice(0, 12)
    .map(([frequency, power], index) => `<tr><td class="primary-cell">${index + 1}</td><td>${formatFrequency(frequency, 4)}</td><td>${Number(power).toFixed(1)} dB</td><td><span class="signal-pill">${power > -45 ? 'Forte' : power > -70 ? 'Medio' : 'Debole'}</span></td></tr>`);
  elements.decoderContent.innerHTML = tableMarkup(['#', 'Frequenza', 'Potenza', 'Livello'], points);
}

function renderDecoder() {
  if (state.mode === 'scan') {
    renderScanPeaks();
    return;
  }
  const protocol = elements.protocol.value;
  if (protocol === 'voice') renderVoiceDecoder();
  else if (protocol === 'adsb') renderAdsb();
  else if (protocol === 'ais') renderAis();
  else if (protocol === 'sonde') renderSonde();
  else renderSstv();
}

function handleDecoder(payload) {
  const protocol = payload.protocol;
  const updatedAt = Date.now();
  if (protocol === 'adsb' && payload.icao) {
    state.adsb.set(payload.icao, { ...(state.adsb.get(payload.icao) || {}), ...payload, updatedAt });
  } else if (protocol === 'ais' && payload.mmsi) {
    state.ais.set(payload.mmsi, { ...(state.ais.get(payload.mmsi) || {}), ...payload, updatedAt });
  } else if (protocol === 'rs41' && payload.serial) {
    state.sonde.set(payload.serial, { ...(state.sonde.get(payload.serial) || {}), ...payload, updatedAt });
  } else if (protocol === 'sstv') {
    state.sstv.event = payload.event || state.sstv.event;
    if (payload.line !== undefined) state.sstv.progress = payload.line;
    if (payload.total_lines !== undefined) state.sstv.total = payload.total_lines;
    if (payload.artifact_url) state.sstv.artifactUrl = payload.artifact_url;
    if (payload.path) state.sstv.path = payload.path;
  }
  if (elements.protocol.value === protocol || (protocol === 'rs41' && elements.protocol.value === 'sonde')) renderDecoder();
}

function addLog(message, level = 'info') {
  state.logs.unshift({ time: new Date(), message, level });
  if (state.logs.length > 200) state.logs.length = 200;
  elements.logCount.textContent = String(state.logs.length);
  elements.logContent.innerHTML = state.logs.map((line) => `<div class="log-line ${line.level}"><span>${line.time.toLocaleTimeString()}</span>  ${escapeHtml(line.message)}</div>`).join('');
}

function handleState(message) {
  state.processMode = message.mode;
  state.processStatus = message.status;
  state.activeSessionId = message.sessionId || null;
  if (message.mode === 'live' && message.sessionId && message.protocol
      && elements.protocol.value !== message.protocol) {
    elements.protocol.value = message.protocol;
    updateProtocolControls();
  }
  if (message.audioRecording) state.recordings.audio = { ...message.audioRecording };
  if (message.iqRecording) state.recordings.iq = { ...message.iqRecording };
  if (typeof message.manualGain === 'boolean') {
    elements.gainAuto.checked = !message.manualGain;
    elements.gainInput.disabled = !message.manualGain;
  }
  if (Number.isFinite(message.gainDb)) elements.gainInput.value = String(message.gainDb);
  if (Object.hasOwn(message, 'appliedGainDb')) {
    state.appliedGainDb = Number.isFinite(message.appliedGainDb)
      ? message.appliedGainDb : null;
  }
  renderGainOutput();
  if (message.mode === 'live' && Number.isFinite(message.frequencyMhz)) {
    state.centerHz = message.frequencyMhz * 1e6;
    state.channelHz = Number.isFinite(message.channelFrequencyMhz)
      ? message.channelFrequencyMhz * 1e6 : state.centerHz;
    syncFrequencyEditor(state.centerHz);
    positionLiveChannelMarker();
  }
  if (message.protocol === 'voice' && Number.isFinite(message.filterWidth)) {
    setVoiceFilterWidth(message.filterWidth);
    if ($('#voice-filter-type') && ['fir', 'iir'].includes(message.filterType)) {
      $('#voice-filter-type').value = message.filterType;
    }
  }
  const active = ['starting', 'receiving', 'scanning', 'stopping'].includes(message.status);
  setBusy(state.protocolSwitchPending || (active && message.status !== 'stopping'));
  elements.stop.disabled = !active || message.status === 'stopping';
  const labels = {
    stopped: ['Pronto', 'Ricevitore fermo'],
    starting: ['Connessione', 'Inizializzazione del tuner'],
    receiving: ['Live', `${message.protocol || elements.protocol.value} in ricezione`],
    scanning: ['Scan', `${Math.round((message.progress || 0) * 100)}% completato`],
    stopping: ['Arresto', 'Chiusura della sessione'],
    complete: ['Completato', 'Scansione terminata'],
    error: ['Errore', 'Controlla la diagnostica'],
  };
  const [label, detail] = labels[message.status] || [message.status, ''];
  elements.statusLabel.textContent = label;
  elements.statusDetail.textContent = detail;
  elements.connectionLight.className = `connection-light ${active ? 'active' : message.status === 'error' ? 'error' : ''}`;
  if (!state.protocolSwitchPending
      && (message.status === 'error' || message.status === 'complete'
          || message.status === 'stopped')) setBusy(false);
  if (message.progress !== undefined) updateScanProgress(message.progress);
  renderRecordingControls();
}

function handleSpectrum(message) {
  state.latestBins = message.bins;
  state.centerHz = message.center_hz;
  if (!Number.isFinite(state.channelHz)) state.channelHz = state.centerHz;
  state.sampleRate = message.sample_rate;
  state.framesThisSecond += 1;
  const now = performance.now();
  if (now - state.lastFpsAt >= 1000) {
    state.measuredFps = state.framesThisSecond * 1000 / (now - state.lastFpsAt);
    state.framesThisSecond = 0;
    state.lastFpsAt = now;
  }
  elements.span.textContent = formatFrequency(state.sampleRate, 3);
  elements.peak.textContent = `${Number(message.max_dbfs).toFixed(1)} dBFS`;
  elements.fpsValue.textContent = `${state.measuredFps.toFixed(0)} FPS`;
  elements.fftValue.textContent = String(message.fft_size);
  if (!state.liveTune.dragging) {
    positionLiveChannelMarker();
  }
  waterfall.push(message.bins);
  scheduleSpectrumDraw();
}

function isCurrentSessionEvent(message) {
  return Boolean(message.sessionId)
    && message.sessionId === state.activeSessionId;
}

function handleReceiverStatus(message) {
  state.receiverStatus.channelDbfs = message.channel_dbfs;
  state.receiverStatus.thresholdDbfs = message.squelch_threshold_dbfs;
  state.receiverStatus.squelchOpen = message.squelch_open;
  if (state.mode === 'live' && elements.protocol.value === 'voice') renderVoiceDecoder();
}

function handleTuned(message) {
  if (!Number.isFinite(message.frequency_hz)) return;
  state.centerHz = message.frequency_hz;
  state.channelHz = message.frequency_hz;
  syncFrequencyEditor(state.centerHz);
  positionLiveChannelMarker();
}

function handleChannelTuned(message) {
  if (!Number.isFinite(message.frequency_hz)) return;
  state.channelHz = message.frequency_hz;
  positionLiveChannelMarker();
  if (state.mode === 'live' && elements.protocol.value === 'voice') {
    renderVoiceDecoder();
  }
}

function handleScanTile(message) {
  for (let index = 0; index < message.bins.length; index += 1) {
    state.scan.points.set(Math.round(message.low_hz + index * message.step_hz), message.bins[index]);
  }
  state.scan.progress = Math.max(state.scan.progress, message.progress || 0);
  updateScanProgress(state.scan.progress);
  scheduleScanDraw();
  if (state.mode === 'scan') renderScanPeaks();
}

function connectEvents() {
  const events = new EventSource('/api/events');
  events.onmessage = ({ data }) => {
    try {
      const message = JSON.parse(data);
      if (message.type === 'state') handleState(message);
      else if (message.type === 'spectrum' && isCurrentSessionEvent(message)) handleSpectrum(message);
      else if (message.type === 'receiver_status' && isCurrentSessionEvent(message)) handleReceiverStatus(message);
      else if (message.type === 'tuned' && isCurrentSessionEvent(message)) handleTuned(message);
      else if (message.type === 'channel_tuned' && isCurrentSessionEvent(message)) handleChannelTuned(message);
      else if (message.type === 'gain_failed') toast(message.message || 'Modifica gain non riuscita', 'error');
      else if (message.type === 'scan_tile' && isCurrentSessionEvent(message)) handleScanTile(message);
      else if (message.type === 'decoder' && isCurrentSessionEvent(message)) handleDecoder(message.payload || {});
      else if (message.type === 'diagnostic') addLog(message.message, message.level);
      else if (message.type === 'process_exit' && !message.expected && message.code !== 0) addLog(`${message.kind} exited with code ${message.code ?? message.signal}`, 'error');
    } catch (error) {
      addLog(`Invalid event: ${error.message}`, 'error');
    }
  };
  events.onerror = () => {
    elements.statusLabel.textContent = 'Disconnesso';
    elements.statusDetail.textContent = 'Controller locale non raggiungibile';
    elements.connectionLight.className = 'connection-light error';
  };
}

async function refreshDevices() {
  elements.refreshDevices.disabled = true;
  try {
    const { devices } = await api('/api/devices');
    const current = elements.device.value;
    elements.device.innerHTML = devices.length
      ? devices.map((device) => `<option value="${escapeHtml(device.serial || device.index)}">${escapeHtml(`[${device.index}] ${device.name}${device.serial ? ` · ${device.serial}` : ''}`)}</option>`).join('')
      : '<option value="0">Nessun dongle rilevato · indice 0</option>';
    if ([...elements.device.options].some((option) => option.value === current)) elements.device.value = current;
  } catch (error) {
    toast(error.message, 'error');
  } finally {
    elements.refreshDevices.disabled = false;
  }
}

elements.modeButtons.forEach((button) => button.addEventListener('click', () => setMode(button.dataset.mode)));
elements.start.addEventListener('click', startCurrentMode);
elements.stop.addEventListener('click', stopSession);
elements.refreshDevices.addEventListener('click', refreshDevices);
elements.copyChannelFrequency.addEventListener('click', copyChannelFrequencyHz);
elements.audioRecordingButton.addEventListener('click', () => toggleRecording('audio'));
elements.iqRecordingButton.addEventListener('click', () => toggleRecording('iq'));
elements.protocol.addEventListener('change', async () => {
  const generation = ++protocolSwitchGeneration;
  const protocol = elements.protocol.value;
  const centerHz = protocolDefaults[protocol] * 1e6;
  syncFrequencyEditor(centerHz);
  updateProtocolControls();
  setMode('live');
  resetLiveSpectrum(protocol, centerHz);

  const sessionIsLive = state.processMode === 'live'
    && ['starting', 'receiving'].includes(state.processStatus);
  if (!sessionIsLive) return;

  // Reject any late FFT/decoder frames from the session being replaced.
  state.activeSessionId = null;
  state.protocolSwitchPending = true;
  elements.protocol.disabled = true;
  setBusy(true);
  try {
    await api('/api/live/start', {
      method: 'POST',
      body: JSON.stringify(collectLiveConfig()),
    });
    if (generation === protocolSwitchGeneration) {
      toast(`${protocol.toUpperCase()} attivo a ${(centerHz / 1e6).toFixed(3)} MHz`);
    }
  } catch (error) {
    if (generation === protocolSwitchGeneration) toast(error.message, 'error');
  } finally {
    if (generation === protocolSwitchGeneration) {
      state.protocolSwitchPending = false;
      elements.protocol.disabled = false;
      const active = ['starting', 'receiving'].includes(state.processStatus);
      setBusy(active);
      elements.stop.disabled = !active;
    }
  }
});
elements.frequencyDigits.addEventListener('pointerover', (event) => {
  const control = event.target.closest('.frequency-digit-control');
  control?.querySelector('.frequency-digit')?.focus({ preventScroll: true });
});
elements.frequencyDigits.addEventListener('click', (event) => {
  const step = event.target.closest('.frequency-step');
  if (!step) return;
  stepFrequencyDigit(Number(step.dataset.index), Number(step.dataset.direction));
});
elements.frequencyDigits.addEventListener('keydown', (event) => {
  const digit = event.target.closest('.frequency-digit');
  if (!digit) return;
  const index = Number(digit.dataset.index);
  if (/^[0-9]$/.test(event.key)) {
    event.preventDefault();
    replaceFrequencyDigit(index, event.key);
  } else if (event.key === 'ArrowUp' || event.key === 'ArrowDown') {
    event.preventDefault();
    stepFrequencyDigit(index, event.key === 'ArrowUp' ? 1 : -1);
  } else if (event.key === 'ArrowLeft' || event.key === 'ArrowRight') {
    event.preventDefault();
    focusFrequencyDigit(index + (event.key === 'ArrowRight' ? 1 : -1));
  }
});
elements.frequencyDigits.addEventListener('wheel', (event) => {
  const digit = event.target.closest('.frequency-digit-control')?.querySelector('.frequency-digit');
  if (!digit) return;
  event.preventDefault();
  const index = Number(digit.dataset.index);
  stepFrequencyDigit(index, event.deltaY < 0 ? 1 : -1);
}, { passive: false });
elements.scanStart.addEventListener('input', () => setMode('scan'));
elements.scanEnd.addEventListener('input', () => setMode('scan'));
elements.gainAuto.addEventListener('change', () => {
  elements.gainInput.disabled = elements.gainAuto.checked;
  state.appliedGainDb = null;
  void updateRuntimeGain();
});
elements.gainInput.addEventListener('input', () => {
  state.appliedGainDb = null;
  renderGainOutput();
  scheduleRuntimeGain();
});
elements.gainInput.addEventListener('change', updateRuntimeGain);
elements.fpsInput.addEventListener('input', () => { elements.fpsOutput.textContent = `${elements.fpsInput.value} FPS`; });

[elements.spectrumCanvas, elements.waterfallCanvas].forEach((canvas) => {
  canvas.addEventListener('pointerdown', beginLiveTune);
});
elements.liveVisual.addEventListener('wheel', tuneLiveChannelWithWheel, { passive: false });
elements.tuningMarker.addEventListener('pointerdown', beginLiveTune);
elements.filterHandleLeft.addEventListener('pointerdown', beginFilterResize);
elements.filterHandleRight.addEventListener('pointerdown', beginFilterResize);
elements.filterHandleLeft.addEventListener('keydown', handleFilterResizeKey);
elements.filterHandleRight.addEventListener('keydown', handleFilterResizeKey);
window.addEventListener('pointermove', moveLiveTune);
window.addEventListener('pointermove', moveFilterResize);
window.addEventListener('pointerup', finishLiveTune);
window.addEventListener('pointerup', finishFilterResize);

elements.scanCanvas.addEventListener('mousemove', (event) => {
  const rect = elements.scanCanvas.getBoundingClientRect();
  state.scan.hoverX = event.clientX - rect.left;
  if (state.scan.dragging) {
    const deltaRatio = (event.clientX - state.scan.dragStartX) / rect.width;
    const span = state.scan.dragViewEnd - state.scan.dragViewStart;
    let start = state.scan.dragViewStart - deltaRatio * span;
    let end = state.scan.dragViewEnd - deltaRatio * span;
    if (start < state.scan.startHz) { end += state.scan.startHz - start; start = state.scan.startHz; }
    if (end > state.scan.endHz) { start -= end - state.scan.endHz; end = state.scan.endHz; }
    state.scan.viewStartHz = start;
    state.scan.viewEndHz = end;
  }
  scheduleScanDraw();
});
elements.scanCanvas.addEventListener('mouseleave', () => { state.scan.hoverX = null; state.scan.dragging = false; scheduleScanDraw(); });
elements.scanCanvas.addEventListener('mousedown', (event) => {
  state.scan.dragging = true;
  state.scan.dragStartX = event.clientX;
  state.scan.dragViewStart = state.scan.viewStartHz;
  state.scan.dragViewEnd = state.scan.viewEndHz;
});
window.addEventListener('mouseup', () => { state.scan.dragging = false; });
elements.scanCanvas.addEventListener('wheel', (event) => {
  event.preventDefault();
  const rect = elements.scanCanvas.getBoundingClientRect();
  const ratio = Math.max(0, Math.min(1, (event.clientX - rect.left) / rect.width));
  const oldSpan = state.scan.viewEndHz - state.scan.viewStartHz;
  const newSpan = Math.max(1e6, Math.min(state.scan.endHz - state.scan.startHz, oldSpan * (event.deltaY > 0 ? 1.25 : 0.8)));
  const focus = state.scan.viewStartHz + ratio * oldSpan;
  let start = focus - ratio * newSpan;
  let end = start + newSpan;
  if (start < state.scan.startHz) { start = state.scan.startHz; end = start + newSpan; }
  if (end > state.scan.endHz) { end = state.scan.endHz; start = end - newSpan; }
  state.scan.viewStartHz = start;
  state.scan.viewEndHz = end;
  scheduleScanDraw();
}, { passive: false });
elements.scanCanvas.addEventListener('dblclick', (event) => {
  const rect = elements.scanCanvas.getBoundingClientRect();
  const ratio = Math.max(0, Math.min(1, (event.clientX - rect.left) / rect.width));
  openLiveAtFrequency(state.scan.viewStartHz + ratio * (state.scan.viewEndHz - state.scan.viewStartHz));
});

$$('.decoder-tab').forEach((tab) => tab.addEventListener('click', () => {
  $$('.decoder-tab').forEach((item) => item.classList.toggle('active', item === tab));
  const showLog = tab.dataset.decoderTab === 'log';
  elements.decoderContent.classList.toggle('hidden', showLog);
  elements.logContent.classList.toggle('hidden', !showLog);
}));

window.addEventListener('resize', () => {
  scheduleSpectrumDraw();
  scheduleScanDraw();
  positionLiveChannelMarker();
});

buildFrequencyEditor();
syncFrequencyEditor(state.centerHz);
updateProtocolControls();
setMode('live');
drawSpectrum();
drawScan();
connectEvents();
refreshDevices();
renderGainOutput();
