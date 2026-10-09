/**
 * Vision inference endpoints for the studio server.
 *
 *  POST /vision/infer   {model, task, image(dataURL), confidence, iou, classes}  → detections JSON
 *                        runs python/vision_infer.py (ultralytics / onnxruntime / VLM) or forwards to
 *                        STUDIO_VISION_URL when set (an external inference server with the same contract).
 *  GET  /vision/models  → {models: [...files in STUDIO_VISION_MODELS], backends: {...}}
 */
import http from 'node:http';
import path from 'node:path';
import { execFile } from 'node:child_process';
import { existsSync, readdirSync } from 'node:fs';

const python = () => process.env.STUDIO_VISION_PYTHON ?? process.env.PYTHON ?? 'python3';
const script = () => path.join(process.cwd(), 'python', 'vision_infer.py');

function runWorker(req: unknown, timeout = 120000): Promise<any> {
  return new Promise((resolve) => {
    const child = execFile(python(), [script()], { timeout, maxBuffer: 64 * 1024 * 1024 }, (err, stdout, stderr) => {
      try { resolve(JSON.parse(String(stdout || '{}'))); }
      catch { resolve({ error: err ? `${err.message}: ${String(stderr).slice(-400)}` : `bad worker output: ${String(stdout).slice(0, 200)}` }); }
    });
    child.stdin?.end(JSON.stringify(req));
  });
}

export async function checkBackends(): Promise<any> {
  if (!existsSync(script())) return { error: 'python/vision_infer.py missing' };
  return new Promise((resolve) => execFile(python(), [script(), '--check'], { timeout: 20000 }, (_e, out) => { try { resolve(JSON.parse(String(out))); } catch { resolve({ error: 'python not available' }); } }));
}

export function listModels(): string[] {
  const dir = process.env.STUDIO_VISION_MODELS;
  if (!dir || !existsSync(dir)) return [];
  return readdirSync(dir).filter((f) => /\.(pt|onnx|engine|tflite|blob)$/i.test(f));
}

/** Returns true when the request was handled. */
export async function handleVisionHttp(req: http.IncomingMessage, res: http.ServerResponse, readJson: () => Promise<any>): Promise<boolean> {
  const url = req.url ?? '';
  if (!url.startsWith('/vision/')) return false;
  const send = (code: number, body: unknown) => { res.writeHead(code, { 'content-type': 'application/json', 'access-control-allow-origin': '*' }); res.end(JSON.stringify(body)); };
  if (req.method === 'OPTIONS') { res.writeHead(204, { 'access-control-allow-origin': '*', 'access-control-allow-headers': '*', 'access-control-allow-methods': 'GET,POST,OPTIONS' }); res.end(); return true; }
  if (url === '/vision/models' && req.method === 'GET') { send(200, { models: listModels(), backends: await checkBackends(), external: process.env.STUDIO_VISION_URL ?? null }); return true; }
  if (url === '/vision/infer' && req.method === 'POST') {
    let body: any;
    try { body = await readJson(); } catch (e) { send(400, { error: (e as Error).message }); return true; }
    if (!body?.image) { send(400, { error: 'image (data URL) required' }); return true; }
    if (process.env.STUDIO_VISION_URL) {
      try {
        const r = await fetch(process.env.STUDIO_VISION_URL, { method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify(body) });
        send(r.status, await r.json());
      } catch (e) { send(502, { error: `external inference server: ${(e as Error).message}` }); }
      return true;
    }
    const out = await runWorker({ confidence: 0.25, iou: 0.5, task: 'detect', model: 'yolov8n.pt', ...body });
    send(out.error ? 500 : 200, out);
    return true;
  }
  return false;
}
