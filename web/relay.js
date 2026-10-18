import dgram from 'node:dgram';
import fs from 'node:fs';
import { WebSocketServer } from 'ws';

const UDP_PORT = 2003;
const WS_PORT = 8080;
const MAX_BUFFERED = 8 * 1024 * 1024;
const HEADER_SIZE = 38;
const STATS = 3;

const logDir = new URL('./logs/', import.meta.url);
fs.mkdirSync(logDir, { recursive: true });
const csvPath = new URL(`stats-${new Date().toISOString().replace(/[:.]/g, '-')}.csv`, logDir);
const csv = fs.createWriteStream(csvPath);
csv.write('time_ms,frame_id,total_ms,capture_ms,obstacles_ms,path_ms\n');

const wss = new WebSocketServer({ port: WS_PORT });
const udp = dgram.createSocket({ type: 'udp4', reuseAddr: true, recvBufferSize: 4 * 1024 * 1024 });

udp.on('message', (msg) => {
  if (msg.length === HEADER_SIZE + 16 && msg[0] === STATS) {
    const ms = [0, 1, 2, 3].map((i) => msg.readFloatLE(HEADER_SIZE + 4 * i).toFixed(3));
    csv.write(`${Date.now()},${msg[1]},${ms.join(',')}\n`);
  }
  for (const client of wss.clients) {
    if (client.readyState === 1 && client.bufferedAmount < MAX_BUFFERED) client.send(msg);
  }
});

udp.bind(UDP_PORT, () => console.log(`relay: udp :${UDP_PORT} -> ws :${WS_PORT}, stats -> ${csvPath.pathname}`));
