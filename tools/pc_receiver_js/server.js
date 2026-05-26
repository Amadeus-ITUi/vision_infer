const dgram = require('node:dgram');
const http = require('node:http');
const fs = require('node:fs');
const path = require('node:path');
const net = require('node:net');

const MAGIC = 0x56535544;
const HEADER_SIZE = 20;
const PUBLIC_DIR = path.join(__dirname, 'public');

const BIND_HOST = process.env.BIND_HOST || '0.0.0.0';
const UDP_PORT = Number(process.env.UDP_PORT || 10000);
const TCP_PORT = Number(process.env.TCP_PORT || 10001);
const HTTP_PORT = Number(process.env.HTTP_PORT || 9090);

const WEB_IMAGE_FORMAT_JPEG = 0;
const WEB_IMAGE_FORMAT_PNG = 1;
const WEB_IMAGE_FORMAT_BMP = 2;

const latestByMode = {
  0: { image: null, frameId: -1, updatedAtMs: 0, width: 0, height: 0, mode: 0, format: WEB_IMAGE_FORMAT_JPEG },
  1: { image: null, frameId: -1, updatedAtMs: 0, width: 0, height: 0, mode: 1, format: WEB_IMAGE_FORMAT_JPEG },
  2: { image: null, frameId: -1, updatedAtMs: 0, width: 0, height: 0, mode: 2, format: WEB_IMAGE_FORMAT_JPEG },
  3: { image: null, frameId: -1, updatedAtMs: 0, width: 0, height: 0, mode: 3, format: WEB_IMAGE_FORMAT_JPEG }
};

let latestStatus = { message: 'waiting' };
const inflightFrames = new Map();
const udpByteEvents = [];
const udpFrameEvents = [];

function modeName(mode) {
  if (mode === 0) return 'hsv_strip';
  if (mode === 1) return 'gray';
  if (mode === 2) return 'rgb';
  if (mode === 3) return 'roi64';
  return `mode_${mode}`;
}

function parseHeader(buf) {
  if (buf.length < HEADER_SIZE) return null;
  // Board sender currently writes header in native byte order.
  // Accept both LE and BE so web receiver remains compatible.
  const magicBE = buf.readUInt32BE(0);
  const littleEndian = magicBE !== MAGIC && buf.readUInt32LE(0) === MAGIC;
  const readU32 = littleEndian ? (offset) => buf.readUInt32LE(offset) : (offset) => buf.readUInt32BE(offset);
  const readU16 = littleEndian ? (offset) => buf.readUInt16LE(offset) : (offset) => buf.readUInt16BE(offset);
  return {
    magic: readU32(0),
    frameId: readU32(4),
    chunkIdx: readU16(8),
    chunkTotal: readU16(10),
    payloadLen: readU16(12),
    width: readU16(14),
    height: readU16(16),
    mode: buf.readUInt8(18),
    format: buf.readUInt8(19)
  };
}

function sanitizeImageFormat(format) {
  if (format === WEB_IMAGE_FORMAT_PNG) return WEB_IMAGE_FORMAT_PNG;
  if (format === WEB_IMAGE_FORMAT_BMP) return WEB_IMAGE_FORMAT_BMP;
  return WEB_IMAGE_FORMAT_JPEG;
}

function contentTypeByImageFormat(format) {
  if (format === WEB_IMAGE_FORMAT_PNG) return 'image/png';
  if (format === WEB_IMAGE_FORMAT_BMP) return 'image/bmp';
  return 'image/jpeg';
}

function cleanupInflight() {
  const now = Date.now();
  for (const [frameId, entry] of inflightFrames.entries()) {
    if ((now - entry.ts) > 2000) {
      inflightFrames.delete(frameId);
    }
  }
  while (udpByteEvents.length > 0 && (now - udpByteEvents[0].ts) > 5000) udpByteEvents.shift();
  while (udpFrameEvents.length > 0 && (now - udpFrameEvents[0].ts) > 5000) udpFrameEvents.shift();
}

function isFrameNewer(prevFrameId, nextFrameId) {
  const diff = ((nextFrameId >>> 0) - (prevFrameId >>> 0)) >>> 0;
  return diff !== 0 && diff < 0x80000000;
}

function isLikelyFrameCounterReset(prevFrameId, nextFrameId) {
  const prev = prevFrameId >>> 0;
  const next = nextFrameId >>> 0;
  if (prev < 1024) return false;
  return next < 64;
}

function shouldAcceptFrame(mode, frameId, nowMs) {
  const latest = latestByMode[mode];
  if (!latest) return false;
  if (latest.frameId < 0) return true;
  if (isFrameNewer(latest.frameId, frameId)) return true;
  if (isLikelyFrameCounterReset(latest.frameId, frameId)) return true;
  if ((nowMs - latest.updatedAtMs) > 1500) return true;
  return false;
}

function onUdpMessage(msg) {
  udpByteEvents.push({ ts: Date.now(), bytes: msg.length });
  const hdr = parseHeader(msg);
  if (!hdr) return;
  if (hdr.magic !== MAGIC) return;
  if (!(hdr.mode in latestByMode)) return;
  if (hdr.chunkTotal === 0 || hdr.chunkIdx >= hdr.chunkTotal) return;
  if ((HEADER_SIZE + hdr.payloadLen) > msg.length) return;

  let entry = inflightFrames.get(hdr.frameId);
  if (!entry) {
    entry = {
      chunkTotal: hdr.chunkTotal,
      chunks: new Map(),
      width: hdr.width,
      height: hdr.height,
      mode: hdr.mode,
      format: sanitizeImageFormat(hdr.format),
      ts: Date.now()
    };
    inflightFrames.set(hdr.frameId, entry);
  }

  entry.chunks.set(hdr.chunkIdx, msg.subarray(HEADER_SIZE, HEADER_SIZE + hdr.payloadLen));
  entry.ts = Date.now();

  if (entry.chunks.size !== entry.chunkTotal) {
    return;
  }

  const ordered = [];
  for (let i = 0; i < entry.chunkTotal; i += 1) {
    const chunk = entry.chunks.get(i);
    if (!chunk) {
      return;
    }
    ordered.push(chunk);
  }

  const image = Buffer.concat(ordered);
  const nowMs = Date.now();
  if (shouldAcceptFrame(hdr.mode, hdr.frameId, nowMs)) {
    const wireBytes = ordered.reduce((sum, chunk) => sum + chunk.length, 0) + (entry.chunkTotal * HEADER_SIZE);
    latestByMode[hdr.mode] = {
      image,
      frameId: hdr.frameId >>> 0,
      updatedAtMs: nowMs,
      width: hdr.width,
      height: hdr.height,
      mode: hdr.mode,
      format: sanitizeImageFormat(hdr.format),
      wireBytes
    };
    udpFrameEvents.push({ ts: nowMs, mode: hdr.mode, wireBytes });
  }

  inflightFrames.delete(hdr.frameId);
}

function buildTransportTelemetry(status) {
  const now = Date.now();
  const oneSecAgo = now - 1000;
  const recentBytes = udpByteEvents.filter((item) => item.ts >= oneSecAgo);
  const recentFrames = udpFrameEvents.filter((item) => item.ts >= oneSecAgo);

  const rxBytesPerSec = recentBytes.reduce((sum, item) => sum + item.bytes, 0);
  const rxFramesByMode = { hsv_strip: 0, gray: 0, rgb: 0, roi64: 0 };
  for (const item of recentFrames) {
    const key = modeName(item.mode);
    if (key in rxFramesByMode) {
      rxFramesByMode[key] += 1;
    }
  }

  return {
    rx_udp_bytes_per_sec: rxBytesPerSec,
    rx_udp_frames_per_sec: recentFrames.length,
    rx_udp_gray_fps: rxFramesByMode.gray,
    rx_udp_hsv_strip_fps: rxFramesByMode.hsv_strip,
    rx_udp_rgb_fps: rxFramesByMode.rgb,
    rx_udp_roi64_fps: rxFramesByMode.roi64,
    board_video_host: status && status.board_video_host ? status.board_video_host : null
  };
}

function startUdpReceiver() {
  const udp = dgram.createSocket('udp4');
  udp.on('message', onUdpMessage);
  udp.bind(UDP_PORT, BIND_HOST, () => {
    console.log(`[JS_RECEIVER] UDP video listening on ${BIND_HOST}:${UDP_PORT}`);
  });
  setInterval(cleanupInflight, 500);
}

function startTcpReceiver() {
  const server = net.createServer((socket) => {
    let buffer = '';
    socket.setEncoding('utf8');

    socket.on('data', (chunk) => {
      buffer += chunk;
      let idx = buffer.indexOf('\n');
      while (idx >= 0) {
        const line = buffer.slice(0, idx).trim();
        buffer = buffer.slice(idx + 1);
        if (line) {
          try {
            latestStatus = JSON.parse(line);
          } catch (_) {
            // ignore malformed lines
          }
        }
        idx = buffer.indexOf('\n');
      }
    });
  });

  server.listen(TCP_PORT, BIND_HOST, () => {
    console.log(`[JS_RECEIVER] TCP status listening on ${BIND_HOST}:${TCP_PORT}`);
  });
}

function sendJson(res, code, obj) {
  res.writeHead(code, {
    'Content-Type': 'application/json; charset=utf-8',
    'Cache-Control': 'no-store'
  });
  res.end(JSON.stringify(obj));
}

function serveFile(res, filePath, contentType) {
  fs.readFile(filePath, (err, data) => {
    if (err) {
      res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' });
      res.end('not found');
      return;
    }
    res.writeHead(200, {
      'Content-Type': contentType,
      'Cache-Control': 'no-store'
    });
    res.end(data);
  });
}

function writeImage(res, frame, notReadyMessage) {
  if (!frame || !frame.image) {
    res.writeHead(503, {
      'Content-Type': 'text/plain; charset=utf-8',
      'Cache-Control': 'no-store'
    });
    res.end(notReadyMessage);
    return;
  }

  res.writeHead(200, {
    'Content-Type': contentTypeByImageFormat(frame.format),
    'Cache-Control': 'no-store'
  });
  res.end(frame.image);
}

function startHttpServer() {
  const server = http.createServer((req, res) => {
    const reqUrl = new URL(req.url || '/', `http://${req.headers.host || 'localhost'}`);
    const pathname = reqUrl.pathname;

    if (pathname === '/' || pathname === '/index.html') {
      serveFile(res, path.join(PUBLIC_DIR, 'index.html'), 'text/html; charset=utf-8');
      return;
    }

    if (pathname === '/api/status') {
      sendJson(res, 200, Object.assign({}, latestStatus, buildTransportTelemetry(latestStatus)));
      return;
    }

    if (pathname === '/api/frame_meta') {
      sendJson(res, 200, {
        gray: latestByMode[1],
        hsv_strip: latestByMode[0],
        rgb: latestByMode[2],
        roi64: latestByMode[3]
      });
      return;
    }

    if (pathname === '/api/frame_gray.jpg') {
      writeImage(res, latestByMode[1], 'gray frame not ready');
      return;
    }

    if (pathname === '/api/frame_hsv_strip.png') {
      writeImage(res, latestByMode[0], 'hsv strip frame not ready');
      return;
    }

    if (pathname === '/api/frame_rgb.jpg') {
      writeImage(res, latestByMode[2], 'rgb frame not ready');
      return;
    }

    if (pathname === '/api/frame_roi64.jpg') {
      writeImage(res, latestByMode[3], 'roi64 frame not ready');
      return;
    }

    if (pathname === '/healthz') {
      res.writeHead(200, { 'Content-Type': 'text/plain; charset=utf-8' });
      res.end('ok');
      return;
    }

    res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' });
    res.end('not found');
  });

  server.listen(HTTP_PORT, BIND_HOST, () => {
    console.log(`[JS_RECEIVER] HTTP web listening on http://${BIND_HOST}:${HTTP_PORT}/`);
  });
}

startUdpReceiver();
startTcpReceiver();
startHttpServer();
