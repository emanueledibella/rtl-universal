import { createReadStream, existsSync, statSync } from 'node:fs';
import { createServer } from 'node:http';
import { spawn } from 'node:child_process';
import { randomUUID } from 'node:crypto';
import { extname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { createInterface } from 'node:readline';

const uiDirectory = fileURLToPath(new URL('.', import.meta.url));
const projectRoot = resolve(uiDirectory, '..');
const receiverBinary = process.env.RTL_UNIVERSAL_BIN || join(projectRoot, 'rtl-universal');
const rtlPowerBinary = process.env.RTL_POWER_BIN || 'rtl_power';
const host = process.env.RTL_UI_HOST || '127.0.0.1';
const port = Number.parseInt(process.env.RTL_UI_PORT || '4173', 10);

const clients = new Set();
const artifacts = new Map();
let activeSession = null;
let currentState = { mode: 'idle', status: 'stopped', sessionId: null };

const contentTypes = {
  '.html': 'text/html; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.ppm': 'image/x-portable-pixmap',
};

function sendJson(response, status, value) {
  const body = JSON.stringify(value);
  response.writeHead(status, {
    'Content-Type': 'application/json; charset=utf-8',
    'Content-Length': Buffer.byteLength(body),
    'Cache-Control': 'no-store',
  });
  response.end(body);
}

function broadcast(value) {
  const payload = `data: ${JSON.stringify(value)}\n\n`;
  for (const response of clients) response.write(payload);
}

function updateState(patch) {
  currentState = { ...currentState, ...patch };
  broadcast({ type: 'state', ...currentState });
}

async function readJsonBody(request) {
  const chunks = [];
  let length = 0;
  for await (const chunk of request) {
    length += chunk.length;
    if (length > 1024 * 1024) throw new Error('Request body too large');
    chunks.push(chunk);
  }
  if (chunks.length === 0) return {};
  return JSON.parse(Buffer.concat(chunks).toString('utf8'));
}

function finiteNumber(value, name, minimum, maximum) {
  const parsed = Number(value);
  if (!Number.isFinite(parsed) || parsed < minimum || parsed > maximum) {
    throw new Error(`${name} must be between ${minimum} and ${maximum}`);
  }
  return parsed;
}

function integerNumber(value, name, minimum, maximum) {
  const parsed = finiteNumber(value, name, minimum, maximum);
  if (!Number.isInteger(parsed)) throw new Error(`${name} must be an integer`);
  return parsed;
}

function optionalText(value, fallback = '') {
  if (value === undefined || value === null) return fallback;
  return String(value).trim();
}

function attachLineReader(stream, handler) {
  if (!stream) return;
  const lines = createInterface({ input: stream, crlfDelay: Infinity });
  lines.on('line', handler);
}

function registerArtifact(event) {
  if (event?.protocol !== 'sstv' || event?.event !== 'image-saved' || !event.path) return event;
  const path = resolve(String(event.path));
  if (!existsSync(path)) return event;
  const id = randomUUID();
  artifacts.set(id, path);
  return { ...event, artifact_url: `/api/artifacts/${id}` };
}

async function stopActive(reason = 'user') {
  const session = activeSession;
  if (!session) {
    updateState({ mode: 'idle', status: 'stopped', sessionId: null });
    return;
  }
  activeSession = null;
  session.stopping = true;
  updateState({ status: 'stopping' });
  broadcast({ type: 'diagnostic', level: 'info', message: `Stopping ${session.kind} session (${reason})` });

  if (session.child.exitCode === null) session.child.kill('SIGINT');
  await Promise.race([
    new Promise((resolveExit) => session.child.once('exit', resolveExit)),
    new Promise((resolveTimeout) => setTimeout(resolveTimeout, 1500)),
  ]);
  if (session.child.exitCode === null) session.child.kill('SIGTERM');
  updateState({ mode: 'idle', status: 'stopped', sessionId: null });
}

function trackChild(child, session) {
  child.on('error', (error) => {
    broadcast({ type: 'diagnostic', level: 'error', message: error.message });
    if (activeSession?.id === session.id) {
      activeSession = null;
      updateState({ mode: 'idle', status: 'error', sessionId: null });
    }
  });
  child.on('exit', (code, signal) => {
    broadcast({
      type: 'process_exit',
      sessionId: session.id,
      kind: session.kind,
      code,
      signal,
      expected: session.stopping,
    });
    if (activeSession?.id === session.id) {
      activeSession = null;
      updateState({
        mode: 'idle',
        status: code === 0 ? 'complete' : 'error',
        sessionId: null,
        ...(session.kind === 'scan' && code === 0 ? { progress: 1 } : {}),
      });
    }
  });
}

async function startLive(config) {
  if (!existsSync(receiverBinary)) {
    throw new Error(`Receiver binary not found at ${receiverBinary}; run make first`);
  }
  const protocol = optionalText(config.protocol, 'voice').toLowerCase();
  if (!['voice', 'adsb', 'ais', 'sonde', 'sstv'].includes(protocol)) {
    throw new Error('Unsupported live protocol');
  }
  const defaults = { voice: 145.5, adsb: 1090, ais: 162, sonde: 403, sstv: 145.8 };
  const frequencyMhz = finiteNumber(config.frequencyMhz ?? defaults[protocol], 'frequency', 10, 1766);
  const fftSize = integerNumber(config.fftSize ?? 1024, 'FFT size', 256, 4096);
  if ((fftSize & (fftSize - 1)) !== 0) throw new Error('FFT size must be a power of two');
  const fps = finiteNumber(config.fps ?? 30, 'spectrum FPS', 1, 60);
  const device = optionalText(config.device, '0');
  const args = [
    '--mode', protocol,
    '--freq', String(frequencyMhz),
    '--device', device,
    '--output', 'json',
    '--stats', '1',
    '--spectrum',
    '--spectrum-fd', '3',
    '--control-fd', '4',
    '--fft-size', String(fftSize),
    '--spectrum-fps', String(fps),
  ];

  if (config.manualGain) {
    args.push('--gain', String(integerNumber(config.gainDb ?? 20, 'gain', 0, 50)));
  }
  const ppm = integerNumber(config.ppm ?? 0, 'PPM', -200, 200);
  if (ppm !== 0) args.push('--ppm', String(ppm));
  const tunerBandwidth = integerNumber(config.tunerBandwidth ?? 0, 'tuner bandwidth', 0, 3200000);
  if (tunerBandwidth > 0) args.push('--bw', String(tunerBandwidth));

  if (protocol === 'voice') {
    const demod = optionalText(config.demod, 'fm').toLowerCase();
    if (!['fm', 'am'].includes(demod)) throw new Error('Voice demodulation must be FM or AM');
    const sampleRate = integerNumber(config.sampleRate ?? 2400000, 'sample rate', 225001, 3200000);
    const filterWidth = integerNumber(config.filterWidth ?? 15000, 'filter width', 1000, sampleRate - 1);
    args.push('--demod', demod, '--sample-rate', String(sampleRate));
    args.push('--filter-width', String(filterWidth), '--filter-type', optionalText(config.filterType, 'iir'));
    if (config.squelchEnabled) {
      args.push('--squelch', String(finiteNumber(config.squelchDbfs ?? -30, 'squelch', -120, 0)));
    } else {
      args.push('--squelch', 'off');
    }
  } else if (protocol === 'ais') {
    const channel = optionalText(config.aisChannel, 'both');
    if (!['both', 'A', 'B'].includes(channel)) throw new Error('AIS channel must be both, A or B');
    args.push('--ais-channel', channel);
  } else if (protocol === 'adsb') {
    const haveLatitude = config.latitude !== '' && config.latitude !== undefined;
    const haveLongitude = config.longitude !== '' && config.longitude !== undefined;
    if (haveLatitude !== haveLongitude) throw new Error('ADS-B latitude and longitude must be supplied together');
    if (haveLatitude) {
      args.push('--lat', String(finiteNumber(config.latitude, 'latitude', -90, 90)));
      args.push('--lon', String(finiteNumber(config.longitude, 'longitude', -180, 180)));
    }
  } else if (protocol === 'sstv') {
    const mode = optionalText(config.sstvMode, 'auto');
    if (!['auto', 'pd120', 'martin-m1'].includes(mode)) throw new Error('Invalid SSTV mode');
    args.push('--sstv-mode', mode);
    const saveDirectory = optionalText(config.saveDirectory, 'immagini-sstv');
    args.push('--save-dir', saveDirectory);
  }

  await stopActive('switch-to-live');
  const child = spawn(receiverBinary, args, {
    cwd: projectRoot,
    stdio: ['ignore', 'pipe', 'pipe', 'pipe', 'pipe'],
  });
  const session = {
    id: randomUUID(),
    kind: 'live',
    child,
    control: child.stdio[4],
    stopping: false,
    config: { ...config, protocol, frequencyMhz },
  };
  activeSession = session;
  trackChild(child, session);

  attachLineReader(child.stdout, (line) => {
    const trimmed = line.trim();
    if (!trimmed) return;
    try {
      const payload = registerArtifact(JSON.parse(trimmed));
      broadcast({ type: 'decoder', sessionId: session.id, payload });
    } catch {
      broadcast({ type: 'diagnostic', level: 'info', source: 'receiver', message: trimmed });
    }
  });
  attachLineReader(child.stderr, (line) => {
    const trimmed = line.trim();
    if (!trimmed) return;
    const squelchStatus = trimmed.match(/channel=([-+\d.]+) dBFS squelch_threshold=([-+\d.]+) dBFS squelch=(open|closed)/);
    if (squelchStatus) {
      broadcast({
        type: 'receiver_status',
        sessionId: session.id,
        channel_dbfs: Number(squelchStatus[1]),
        squelch_threshold_dbfs: Number(squelchStatus[2]),
        squelch_open: squelchStatus[3] === 'open',
      });
    }
    const tuned = trimmed.match(/\[CONTROL] event=tuned frequency_hz=(\d+)/);
    if (tuned) {
      broadcast({ type: 'tuned', sessionId: session.id, frequency_hz: Number(tuned[1]) });
    }
    broadcast({ type: 'diagnostic', level: /error|failed|invalid/i.test(trimmed) ? 'error' : 'info', source: 'receiver', message: trimmed });
  });
  attachLineReader(child.stdio[3], (line) => {
    try {
      const payload = JSON.parse(line);
      if (!session.seenSpectrum) {
        session.seenSpectrum = true;
        updateState({ mode: 'live', status: 'receiving', sessionId: session.id, protocol, frequencyMhz });
      }
      broadcast({ ...payload, sessionId: session.id });
    } catch {
      broadcast({ type: 'diagnostic', level: 'error', message: 'Invalid spectrum frame from receiver' });
    }
  });

  updateState({ mode: 'live', status: 'starting', sessionId: session.id, protocol, frequencyMhz });
  return { sessionId: session.id, args };
}

function writeLiveControl(command) {
  const session = activeSession;
  if (!session || session.kind !== 'live' || !session.control?.writable) {
    throw new Error('No active Live session');
  }
  session.control.write(`${command}\n`);
  return session;
}

function tuneLive(config) {
  const frequencyMhz = finiteNumber(config.frequencyMhz, 'frequency', 10, 1766);
  const session = writeLiveControl(`TUNE ${Math.round(frequencyMhz * 1e6)}`);
  session.config.frequencyMhz = frequencyMhz;
  updateState({
    mode: 'live',
    status: 'receiving',
    sessionId: session.id,
    protocol: session.config.protocol,
    frequencyMhz,
  });
  return { sessionId: session.id, frequencyMhz };
}

function setLiveSquelch(config) {
  const session = activeSession;
  if (!session || session.kind !== 'live' || session.config.protocol !== 'voice') {
    throw new Error('Squelch control requires an active Voice session');
  }
  const enabled = Boolean(config.enabled);
  const thresholdDbfs = finiteNumber(config.thresholdDbfs ?? -30, 'squelch', -120, 0);
  writeLiveControl(`SQUELCH ${enabled ? 1 : 0} ${thresholdDbfs}`);
  session.config.squelchEnabled = enabled;
  session.config.squelchDbfs = thresholdDbfs;
  return { sessionId: session.id, enabled, thresholdDbfs };
}

function parseRtlPowerLine(line, config, session) {
  const fields = line.split(',').map((field) => field.trim());
  if (fields.length < 7) return;
  const lowHz = Number(fields[2]);
  const highHz = Number(fields[3]);
  const stepHz = Number(fields[4]);
  const samples = Number(fields[5]);
  const bins = fields.slice(6).map(Number).filter(Number.isFinite);
  if (![lowHz, highHz, stepHz, samples].every(Number.isFinite) || bins.length === 0) return;
  const progress = Math.max(0, Math.min(1, (highHz - config.startHz) / (config.endHz - config.startHz)));
  broadcast({
    type: 'scan_tile',
    sessionId: session.id,
    low_hz: lowHz,
    high_hz: highHz,
    step_hz: stepHz,
    samples,
    bins,
    progress,
  });
  updateState({ mode: 'scan', status: 'scanning', sessionId: session.id, progress });
}

async function startScan(config) {
  const startMhz = finiteNumber(config.startMhz ?? 50, 'scan start', 24, 1765);
  const endMhz = finiteNumber(config.endMhz ?? 1700, 'scan end', 25, 1766);
  if (endMhz <= startMhz) throw new Error('Scan end must be greater than scan start');
  const binHz = integerNumber(config.binHz ?? 100000, 'scan resolution', 1000, 2800000);
  const integrationSeconds = integerNumber(config.integrationSeconds ?? 1, 'integration', 1, 60);
  const device = optionalText(config.device, '0');
  const startHz = Math.round(startMhz * 1e6);
  const endHz = Math.round(endMhz * 1e6);
  const args = [
    '-f', `${startHz}:${endHz}:${binHz}`,
    '-i', String(integrationSeconds),
    '-1',
    '-w', 'blackman-harris',
    '-c', '0.2',
    '-d', device,
  ];
  if (config.manualGain) args.push('-g', String(finiteNumber(config.gainDb ?? 20, 'gain', 0, 50)));
  const ppm = integerNumber(config.ppm ?? 0, 'PPM', -200, 200);
  if (ppm !== 0) args.push('-p', String(ppm));

  await stopActive('switch-to-scan');
  const child = spawn(rtlPowerBinary, args, { cwd: projectRoot, stdio: ['ignore', 'pipe', 'pipe'] });
  const normalizedConfig = { ...config, startMhz, endMhz, startHz, endHz, binHz };
  const session = { id: randomUUID(), kind: 'scan', child, stopping: false, config: normalizedConfig };
  activeSession = session;
  trackChild(child, session);
  attachLineReader(child.stdout, (line) => parseRtlPowerLine(line, normalizedConfig, session));
  attachLineReader(child.stderr, (line) => {
    const trimmed = line.trim();
    if (trimmed) broadcast({ type: 'diagnostic', level: /error|failed|no supported/i.test(trimmed) ? 'error' : 'info', source: 'rtl_power', message: trimmed });
  });
  updateState({ mode: 'scan', status: 'starting', sessionId: session.id, progress: 0, startMhz, endMhz });
  return { sessionId: session.id, args };
}

function listDevices() {
  return new Promise((resolveDevices) => {
    if (!existsSync(receiverBinary)) {
      resolveDevices([]);
      return;
    }
    const child = spawn(receiverBinary, ['--list-devices'], { cwd: projectRoot });
    let stdout = '';
    child.stdout.on('data', (chunk) => { stdout += chunk.toString('utf8'); });
    child.on('error', () => resolveDevices([]));
    child.on('exit', () => {
      const devices = [];
      for (const line of stdout.split(/\r?\n/)) {
        const match = line.match(/^\s*\[(\d+)]\s+([^|]+)(?:\|.*serial=([^|]+))?/);
        if (match) devices.push({ index: match[1], name: match[2].trim(), serial: match[3]?.trim() || '' });
      }
      resolveDevices(devices);
    });
  });
}

function serveStatic(pathname, response) {
  const paths = {
    '/': 'index.html',
    '/index.html': 'index.html',
    '/styles.css': 'styles.css',
    '/app.js': 'app.js',
  };
  const name = paths[pathname];
  if (!name) return false;
  const path = join(uiDirectory, name);
  const stats = statSync(path);
  response.writeHead(200, {
    'Content-Type': contentTypes[extname(path)] || 'application/octet-stream',
    'Content-Length': stats.size,
    'Cache-Control': 'no-cache',
  });
  createReadStream(path).pipe(response);
  return true;
}

const server = createServer(async (request, response) => {
  const url = new URL(request.url || '/', `http://${request.headers.host || `${host}:${port}`}`);
  try {
    if (request.method === 'GET' && serveStatic(url.pathname, response)) return;
    if (request.method === 'GET' && url.pathname === '/api/events') {
      response.writeHead(200, {
        'Content-Type': 'text/event-stream',
        'Cache-Control': 'no-cache',
        Connection: 'keep-alive',
      });
      response.write(`data: ${JSON.stringify({ type: 'state', ...currentState })}\n\n`);
      clients.add(response);
      request.on('close', () => clients.delete(response));
      return;
    }
    if (request.method === 'GET' && url.pathname === '/api/status') {
      sendJson(response, 200, { ...currentState, receiverBinary, rtlPowerBinary });
      return;
    }
    if (request.method === 'GET' && url.pathname === '/api/devices') {
      sendJson(response, 200, { devices: await listDevices() });
      return;
    }
    if (request.method === 'GET' && url.pathname.startsWith('/api/artifacts/')) {
      const id = url.pathname.slice('/api/artifacts/'.length);
      const path = artifacts.get(id);
      if (!path || !existsSync(path)) {
        sendJson(response, 404, { error: 'Artifact not found' });
        return;
      }
      const stats = statSync(path);
      response.writeHead(200, {
        'Content-Type': contentTypes[extname(path).toLowerCase()] || 'application/octet-stream',
        'Content-Length': stats.size,
        'Content-Disposition': `inline; filename="${path.split('/').pop()}"`,
        'Cache-Control': 'no-store',
      });
      createReadStream(path).pipe(response);
      return;
    }
    if (request.method === 'POST' && url.pathname === '/api/live/start') {
      sendJson(response, 202, await startLive(await readJsonBody(request)));
      return;
    }
    if (request.method === 'POST' && url.pathname === '/api/live/tune') {
      sendJson(response, 202, tuneLive(await readJsonBody(request)));
      return;
    }
    if (request.method === 'POST' && url.pathname === '/api/live/squelch') {
      sendJson(response, 202, setLiveSquelch(await readJsonBody(request)));
      return;
    }
    if (request.method === 'POST' && url.pathname === '/api/scan/start') {
      sendJson(response, 202, await startScan(await readJsonBody(request)));
      return;
    }
    if (request.method === 'POST' && url.pathname === '/api/stop') {
      await stopActive('user');
      sendJson(response, 200, { ok: true });
      return;
    }
    if (request.method === 'GET' && url.pathname === '/favicon.ico') {
      response.writeHead(204);
      response.end();
      return;
    }
    sendJson(response, 404, { error: 'Not found' });
  } catch (error) {
    sendJson(response, 400, { error: error instanceof Error ? error.message : String(error) });
  }
});

const heartbeat = setInterval(() => {
  for (const response of clients) response.write(': heartbeat\n\n');
}, 15000);

async function shutdown() {
  clearInterval(heartbeat);
  await stopActive('server-shutdown');
  server.close();
}

process.on('SIGINT', shutdown);
process.on('SIGTERM', shutdown);
server.listen(port, host, () => {
  console.log(`RTL Universal Studio: http://${host}:${port}`);
  console.log(`Receiver: ${receiverBinary}`);
});
