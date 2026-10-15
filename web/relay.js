import dgram from 'node:dgram';
import { WebSocketServer } from 'ws';

const UDP_PORT = 2003;
const WS_PORT = 8080;
const MAX_BUFFERED = 8 * 1024 * 1024;

const wss = new WebSocketServer({ port: WS_PORT });
const udp = dgram.createSocket({ type: 'udp4', reuseAddr: true, recvBufferSize: 4 * 1024 * 1024 });

udp.on('message', (msg) => {
  for (const client of wss.clients) {
    if (client.readyState === 1 && client.bufferedAmount < MAX_BUFFERED) client.send(msg);
  }
});

udp.bind(UDP_PORT, () => console.log(`relay: udp :${UDP_PORT} -> ws :${WS_PORT}`));
