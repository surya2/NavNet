export const POINT_CLOUD = 0;
export const OBSTACLES = 1;
export const PATH = 2;
export const STATS = 3;
export const CHUNK_POINTS = 200;
export const MAX_POINTS = 212 * 120;
const HEADER_SIZE = 38;

const CRC_TABLE = new Uint32Array(256).map((_, i) => {
  let c = i;
  for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
  return c;
});

function crc32(bytes) {
  let crc = 0xffffffff;
  for (let i = 0; i < bytes.length; i++) crc = CRC_TABLE[(crc ^ bytes[i]) & 0xff] ^ (crc >>> 8);
  return (crc ^ 0xffffffff) >>> 0;
}

function lz4Decompress(src, dst) {
  let s = 0;
  let d = 0;
  while (s < src.length) {
    const token = src[s++];
    let len = token >> 4;
    if (len === 15) for (let b = 255; b === 255; len += b) b = src[s++];
    dst.set(src.subarray(s, s + len), d);
    s += len;
    d += len;
    if (s >= src.length) break;
    const offset = src[s] | (src[s + 1] << 8);
    s += 2;
    len = token & 15;
    if (len === 15) for (let b = 255; b === 255; len += b) b = src[s++];
    len += 4;
    for (let i = 0; i < len; i++, d++) dst[d] = dst[d - offset];
  }
  return d;
}

export function parsePacket(buffer) {
  if (buffer.byteLength < HEADER_SIZE) return null;
  const bytes = new Uint8Array(buffer);
  const view = new DataView(buffer);
  const pointCount = view.getUint16(6, true);
  if (buffer.byteLength !== HEADER_SIZE + view.getUint16(8, true)) return null;

  const crc = view.getUint32(34, true);
  const zeroed = bytes.slice();
  zeroed.fill(0, 34, 38);
  if (crc32(zeroed) !== crc) return null;

  const type = view.getUint8(0);
  const frameId = view.getUint8(1);
  if (type === STATS) {
    if (buffer.byteLength !== HEADER_SIZE + 16) return null;
    const stats = [0, 1, 2, 3].map((i) => view.getFloat32(HEADER_SIZE + 4 * i, true));
    return { type, frameId, stats };
  }

  const raw = new Uint8Array(pointCount * 6);
  try {
    if (lz4Decompress(bytes.subarray(HEADER_SIZE), raw) !== raw.length) return null;
  } catch {
    return null;
  }

  const min = [0, 1, 2].map((a) => view.getFloat32(10 + 4 * a, true));
  const max = [0, 1, 2].map((a) => view.getFloat32(22 + 4 * a, true));
  const q = new DataView(raw.buffer);
  const values = new Float32Array(pointCount * 3);
  for (let i = 0; i < values.length; i++) {
    const a = i % 3;
    values[i] = min[a] + ((q.getInt16(i * 2, true) + 32767) / 65534) * (max[a] - min[a]);
  }

  return {
    type,
    frameId,
    chunkIndex: view.getUint16(2, true),
    chunkCount: view.getUint16(4, true),
    pointCount,
    min,
    max,
    values,
  };
}

export class FrameAssembler {
  constructor(onFrame) {
    this.onFrame = onFrame;
    this.pending = [null, null, null];
  }

  push(packet) {
    const { type, frameId, chunkIndex, chunkCount, pointCount, min, max, values } = packet;
    if (type > PATH) return;
    let frame = this.pending[type];
    if (frame && frame.frameId !== frameId) {
      this.onFrame(type, frame);
      frame = null;
    }
    if (!frame) {
      frame = { frameId, chunkCount, received: 0, count: 0, min, max, values: new Float32Array(MAX_POINTS * 3).fill(NaN) };
      this.pending[type] = frame;
    }
    const offset = chunkIndex * CHUNK_POINTS;
    if (offset + pointCount > MAX_POINTS) return;
    frame.values.set(values, offset * 3);
    frame.count = Math.max(frame.count, offset + pointCount);
    if (++frame.received === frame.chunkCount) {
      this.onFrame(type, frame);
      this.pending[type] = null;
    }
  }
}
