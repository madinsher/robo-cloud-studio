/**
 * Native RoboDK post processors on the studio server: runs RoboDK's own posts (source .py, the compiled libspp / vXY
 * wrappers RoboDK ships, or a user post file) with RoboDK's embedded CPython and its `robodk` package through
 * python/post_shim.py, on a post job produced by the studio (src/posts/job.ts), and returns every file the post
 * writes (ProgSave into a temporary folder). RoboDK.exe is never started (the shim blocks robodk.robolink.Robolink).
 *
 * HTTP (registered in server/index.ts):
 *   GET  /posts/native        -> { available, python, postsDir, posts: string[] }
 *   POST /posts/native/run    { post: "KUKA_KRC4" | "C:/path/MyPost.py", job: PostJob, source?: string }
 *                             -> { ok, files: [{name, content, encoding}], log, error? }
 * Environment: ROBODK_PYTHON (default C:/RoboDK/Python-Embedded/python.exe), ROBODK_POSTS (default C:/RoboDK/Posts).
 */
import type { IncomingMessage, ServerResponse } from 'node:http';
import { execFile } from 'node:child_process';
import { existsSync, mkdtempSync, readdirSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';

export interface NativePostFile { name: string; content: string; encoding: 'utf-8' | 'cp1252' | 'base64' }
export interface NativePostResult { ok: boolean; files: NativePostFile[]; log: string; error?: string }

export function roboDKPython(): string | null {
  const cands = [process.env.ROBODK_PYTHON, 'C:/RoboDK/Python-Embedded/python.exe', '/opt/RoboDK/Python-Embedded/bin/python3'].filter(Boolean) as string[];
  return cands.find((c) => existsSync(c)) ?? null;
}
export function roboDKPostsDir(): string | null {
  const cands = [process.env.ROBODK_POSTS, 'C:/RoboDK/Posts', '/opt/RoboDK/Posts'].filter(Boolean) as string[];
  return cands.find((c) => existsSync(c)) ?? null;
}
/** Post names available in RoboDK's Posts folder (each .py file, wrappers of compiled posts included). */
export function listNativePosts(): string[] {
  const dir = roboDKPostsDir();
  return dir ? readdirSync(dir).filter((f) => f.endsWith('.py') && !f.startsWith('_')).map((f) => f.replace(/\.py$/, '')).sort() : [];
}
const SHIM = () => path.join(process.cwd(), 'python', 'post_shim.py');

/**
 * Run a RoboDK post on a job. `post` is a post name from the Posts folder or a path to a post file; `source` (a user
 * post's text) is written to a temporary file and run instead.
 */
export function runNativePost(post: string, job: unknown, opts: { source?: string; timeoutMs?: number } = {}): Promise<NativePostResult> {
  const py = roboDKPython();
  if (!py) return Promise.resolve({ ok: false, files: [], log: '', error: 'RoboDK embedded Python not found (set ROBODK_PYTHON)' });
  const dir = mkdtempSync(path.join(tmpdir(), 'vbs-post-'));
  const jobPath = path.join(dir, 'job.json'), out = path.join(dir, 'out');
  writeFileSync(jobPath, JSON.stringify(job));
  let postArg = post;
  if (opts.source) {
    const name = (post || 'user_post').replace(/\.py$/i, '').replace(/[^\w]/g, '_');
    postArg = path.join(dir, `${name}.py`);
    writeFileSync(postArg, opts.source);
  } else if (!/[\\/]/.test(post) && !/^[\w.-]+$/.test(post)) {
    rmSync(dir, { recursive: true, force: true });
    return Promise.resolve({ ok: false, files: [], log: '', error: `invalid post name ${post}` });
  }
  const postsDir = roboDKPostsDir();
  const args = [SHIM(), postArg, jobPath, out, ...(postsDir ? ['--posts', postsDir] : [])];
  return new Promise((resolve) => {
    execFile(py, args, { timeout: opts.timeoutMs ?? 300_000, maxBuffer: 512 * 1024 * 1024, windowsHide: true }, (err, stdout, stderr) => {
      let res: NativePostResult;
      try {
        const r = JSON.parse(stdout.slice(stdout.indexOf('{')));
        res = { ok: !!r.ok, files: r.files ?? [], log: r.log ?? '', ...(r.ok ? {} : { error: r.error ?? 'post failed' }) };
      } catch {
        res = { ok: false, files: [], log: '', error: String(err?.message ?? '') + (stderr || stdout).slice(-2000) };
      }
      rmSync(dir, { recursive: true, force: true });
      resolve(res);
    });
  });
}

/** HTTP handler for /posts/native*; returns false when the URL is not handled. */
export async function handlePostsHttp(req: IncomingMessage, res: ServerResponse, readJson: () => Promise<any>): Promise<boolean> {
  const url = req.url ?? '';
  if (!url.startsWith('/posts/native')) return false;
  // origin / host checks and the CORS headers of the studio page are applied by server/index.ts (src/api/security.ts)
  if (req.method === 'OPTIONS') { res.writeHead(204); res.end(); return true; }
  const json = (code: number, body: unknown) => { res.writeHead(code, { 'content-type': 'application/json' }); res.end(JSON.stringify(body)); };
  if (url === '/posts/native' && req.method === 'GET') {
    json(200, { available: !!roboDKPython() && !!roboDKPostsDir(), python: roboDKPython(), postsDir: roboDKPostsDir(), posts: listNativePosts() });
    return true;
  }
  if (url === '/posts/native/run' && req.method === 'POST') {
    try {
      const { post, job, source } = await readJson();
      if (!job || (!post && !source)) { json(400, { ok: false, error: 'post and job required' }); return true; }
      json(200, await runNativePost(String(post ?? ''), job, { source: source ? String(source) : undefined }));
    } catch (e: any) { json(500, { ok: false, files: [], log: '', error: String(e?.message ?? e) }); }
    return true;
  }
  json(404, { ok: false, error: 'unknown posts route' });
  return true;
}
