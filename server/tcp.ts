/** Newline-delimited JSON over plain TCP (for MATLAB tcpclient, C++, PLCs). Same request/response objects as the WebSocket API. */
import net from 'node:net';
import { RpcRequest, RpcResponse } from '../src/api/rpc.ts';

export function parseTcpJsonLines(port: number, handler: (req: RpcRequest) => Promise<RpcResponse>): net.Server {
  const server = net.createServer((sock) => {
    let buf = '';
    sock.on('data', (d) => {
      buf += d.toString('utf8');
      let i: number;
      while ((i = buf.indexOf('\n')) >= 0) {
        const line = buf.slice(0, i).trim();
        buf = buf.slice(i + 1);
        if (!line) continue;
        let req: RpcRequest;
        try { req = JSON.parse(line); } catch { sock.write(JSON.stringify({ id: null, error: 'invalid JSON' }) + '\n'); continue; }
        handler(req).then((res) => sock.write(JSON.stringify(res) + '\n')).catch((e) => sock.write(JSON.stringify({ id: req.id, error: String(e?.message ?? e) }) + '\n'));
      }
    });
    sock.on('error', () => {});
  });
  server.listen(port, () => console.log(`[studio-server] TCP JSON-lines API on port ${port}`));
  return server;
}
