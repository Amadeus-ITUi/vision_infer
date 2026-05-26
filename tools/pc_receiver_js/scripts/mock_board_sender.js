#!/usr/bin/env node
'use strict';

const dgram = require('node:dgram');
const net = require('node:net');

const MAGIC = 0x56535544;
const HEADER_SIZE = 20;
const FORMAT_JPEG = 0;
const FORMAT_PNG = 1;
const FORMAT_BMP = 2;
const MODE_HSV_STRIP = 0;
const MODE_GRAY = 1;
const MODE_RGB = 2;
const MODE_ROI64 = 3;

const DEFAULT_HOST = '127.0.0.1';
const DEFAULT_UDP_PORT = 10000;
const DEFAULT_TCP_PORT = 10001;
const DEFAULT_FPS = 10;
const DEFAULT_CHUNK_BYTES = 1200;

const LABELS = ['ambulance', 'armored_car', 'bomb', 'gun', 'medicine', 'telescope'];

function usage() {
  console.log(`Usage:
  node scripts/mock_board_sender.js [options]

Options:
  --host <ip>         Receiver host, default ${DEFAULT_HOST}
  --udp-port <port>   Receiver UDP image port, default ${DEFAULT_UDP_PORT}
  --tcp-port <port>   Receiver TCP status port, default ${DEFAULT_TCP_PORT}
  --fps <n>           Send rate, default ${DEFAULT_FPS}
  --duration <sec>    Stop after N seconds, default 0 (run until Ctrl+C)
  --once              Send one sample and exit
  --help              Show this help
`);
}

function parseArgs(argv) {
  const opts = {
    host: DEFAULT_HOST,
    udpPort: DEFAULT_UDP_PORT,
    tcpPort: DEFAULT_TCP_PORT,
    durationSec: 0,
    fps: DEFAULT_FPS,
    once: false
  };

  for (let i = 0; i < argv.length; i += 1) {
    const arg = argv[i];
    if (arg === '--help' || arg === '-h') { usage(); process.exit(0); }
    if (arg === '--once') { opts.once = true; continue; }
    const next = argv[i + 1];
    if (arg === '--host') { opts.host = String(next || '').trim(); i += 1; }
    else if (arg === '--udp-port') { opts.udpPort = Number(next); i += 1; }
    else if (arg === '--tcp-port') { opts.tcpPort = Number(next); i += 1; }
    else if (arg === '--duration') { opts.durationSec = Number(next); i += 1; }
    else if (arg === '--fps') { opts.fps = Number(next); i += 1; }
    else { throw new Error(`Unknown option: ${arg}`); }
  }
  return opts;
}

function sleep(ms) { return new Promise((resolve) => setTimeout(resolve, ms)); }

function makeBmp(width, height, pixelAt) {
  const rowBytes = Math.ceil((width * 3) / 4) * 4;
  const pixelBytes = rowBytes * height;
  const fileBytes = 54 + pixelBytes;
  const buffer = Buffer.alloc(fileBytes);
  buffer.write('BM', 0, 'ascii');
  buffer.writeUInt32LE(fileBytes, 2);
  buffer.writeUInt32LE(54, 10);
  buffer.writeUInt32LE(40, 14);
  buffer.writeInt32LE(width, 18);
  buffer.writeInt32LE(height, 22);
  buffer.writeUInt16LE(1, 26);
  buffer.writeUInt16LE(24, 28);
  buffer.writeUInt32LE(pixelBytes, 34);

  for (let y = 0; y < height; y += 1) {
    const outY = height - 1 - y;
    for (let x = 0; x < width; x += 1) {
      const [r, g, b] = pixelAt(x, y);
      const idx = 54 + outY * rowBytes + x * 3;
      buffer[idx] = b;
      buffer[idx + 1] = g;
      buffer[idx + 2] = r;
    }
  }
  return buffer;
}

function clampByte(v) { return Math.max(0, Math.min(255, Math.round(v))); }

function syntheticImage(mode, seq) {
  const width = mode === MODE_ROI64 ? 64 : 160;
  const height = mode === MODE_ROI64 ? 64 : 60;
  const format = mode === MODE_HSV_STRIP ? FORMAT_PNG : FORMAT_BMP;

  const image = makeBmp(width, height, (x, y) => {
    const phase = seq % 80;
    if (mode === MODE_GRAY) {
      const v = clampByte(35 + (x / width) * 120 + Math.sin((y + seq) / 7) * 35);
      return [v, v, v];
    }
    if (mode === MODE_RGB || mode === MODE_ROI64) {
      const redBox = Math.abs(x - width / 2) < 12 && Math.abs(y - height / 2) < 8;
      return [
        redBox ? clampByte(200 + Math.sin(seq / 5) * 55) : clampByte(30 + x * 1.2),
        redBox ? clampByte(20 + Math.sin(seq / 3) * 15) : clampByte(50 + y * 2.4),
        redBox ? clampByte(20 + Math.cos(seq / 4) * 15) : clampByte(150 + Math.sin((x + seq) / 9) * 70)
      ];
    }
    if (mode === MODE_HSV_STRIP) {
      const h = clampByte((x / width) * 180);
      return [h, 200, 200];
    }
    return [0, 0, 0];
  });

  // For HSV strip, encode as PNG (but we use BMP for simplicity with different format flag)
  return { image, width, height, format };
}

function makeStatus(seq) {
  const labelIdx = seq % LABELS.length;
  const redFound = seq % 5 !== 0;
  const roiValid = redFound && seq % 7 !== 0;
  const inferValid = roiValid;

  return {
    web_data_profile: 0,
    ts_ms: Date.now(),
    camera_mode: 'camera_red_roi_ncnn',
    capture_thread_fps: 28 + (seq % 5),
    vision_process_fps: 26 + (seq % 4),
    infer_thread_fps: inferValid ? 24 + (seq % 3) : 0,
    udp_tx_fps: 10,
    cpu_usage_percent: 45 + (seq % 15),
    red_detect_us: redFound ? 2500 + (seq % 800) : 0,
    capture_wait_us: 300 + (seq % 200),
    preprocess_us: 1200 + (seq % 400),
    total_us: 6800 + (seq % 1000),
    red_found: redFound,
    red: redFound ? [60, 22, 40, 16, 80, 30] : [0, 0, 0, 0, 0, 0],
    roi_valid: roiValid,
    roi: roiValid ? [56, 18, 48, 24] : [0, 0, 0, 0],
    infer_enabled: true,
    ncnn_enabled: true,
    ncnn_has_result: inferValid,
    ncnn_infer_valid: inferValid,
    ncnn_infer_us: inferValid ? 35000 + (seq % 5000) : 0,
    ncnn_top_class_id: inferValid ? labelIdx : -1,
    ncnn_top_score: inferValid ? +(0.75 + Math.random() * 0.2).toFixed(6) : 0,
    ncnn_top_label: inferValid ? LABELS[labelIdx] : '',
    gray_size: [160, 60],
    roi64_size: [64, 64]
  };
}

function sendUdpFrame(udp, opts, mode, frameId, frame) {
  const chunkTotal = Math.max(1, Math.ceil(frame.image.length / DEFAULT_CHUNK_BYTES));
  for (let idx = 0; idx < chunkTotal; idx += 1) {
    const start = idx * DEFAULT_CHUNK_BYTES;
    const payload = frame.image.subarray(start, start + DEFAULT_CHUNK_BYTES);
    const packet = Buffer.alloc(HEADER_SIZE + payload.length);
    packet.writeUInt32BE(MAGIC, 0);
    packet.writeUInt32BE(frameId >>> 0, 4);
    packet.writeUInt16BE(idx, 8);
    packet.writeUInt16BE(chunkTotal, 10);
    packet.writeUInt16BE(payload.length, 12);
    packet.writeUInt16BE(frame.width, 14);
    packet.writeUInt16BE(frame.height, 16);
    packet.writeUInt8(mode, 18);
    packet.writeUInt8(frame.format, 19);
    payload.copy(packet, HEADER_SIZE);
    udp.send(packet, opts.udpPort, opts.host);
  }
}

function connectTcp(host, port) {
  return new Promise((resolve, reject) => {
    const socket = net.createConnection({ host, port }, () => resolve(socket));
    socket.setNoDelay(true);
    socket.once('error', reject);
  });
}

async function main() {
  const opts = parseArgs(process.argv.slice(2));
  const udp = dgram.createSocket('udp4');
  const tcp = await connectTcp(opts.host, opts.tcpPort);
  const startedAt = Date.now();
  const intervalMs = Math.max(1, Math.round(1000 / opts.fps));
  let seq = 0;
  let frameId = 1;

  const modes = [
    { mode: MODE_HSV_STRIP, name: 'hsv_strip' },
    { mode: MODE_GRAY, name: 'gray' },
    { mode: MODE_RGB, name: 'rgb' },
    { mode: MODE_ROI64, name: 'roi64' }
  ];

  console.log(`[mock_board] tcp=${opts.host}:${opts.tcpPort} udp=${opts.host}:${opts.udpPort}`);
  console.log(`[mock_board] fps=${opts.fps} modes=${modes.map((m) => m.name).join(',')}`);

  const cleanup = () => {
    try { tcp.end(); } catch (_) {}
    try { udp.close(); } catch (_) {}
  };
  process.on('SIGINT', () => { cleanup(); process.exit(0); });

  while (true) {
    const status = makeStatus(seq);
    tcp.write(`${JSON.stringify(status)}\n`);

    for (const { mode, name } of modes) {
      const frame = syntheticImage(mode, seq);
      sendUdpFrame(udp, opts, mode, frameId, frame);
      frameId = (frameId + 1) >>> 0;
    }

    seq += 1;
    process.stdout.write(`\r[mock_board] sent=${seq} frame_id=${frameId}`);
    if (opts.once) break;
    if (opts.durationSec > 0 && Date.now() - startedAt >= opts.durationSec * 1000) break;
    await sleep(intervalMs);
  }

  process.stdout.write('\n');
  await sleep(100);
  cleanup();
}

main().catch((err) => {
  console.error(`[mock_board] ${err && err.stack ? err.stack : err}`);
  process.exit(1);
});
