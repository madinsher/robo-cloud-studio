// Native RoboDK posts (P0d): server/posts_native.ts runs RoboDK's own posts (compiled libspp / vXY wrappers, source
// posts, user post files) with RoboDK's embedded Python through python/post_shim.py and returns every file; the HTTP
// route /posts/native(/run); source posts through post_shim.run_job_source (the Pyodide entry point, here on CPython);
// classification of RoboDK's Posts folder for the browser import (source -> Pyodide, compiled -> server).
import { describe, it, expect, afterAll } from 'vitest';
import http from 'node:http';
import { existsSync, readdirSync, readFileSync, mkdtempSync, rmSync, writeFileSync } from 'node:fs';
import { execFileSync } from 'node:child_process';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { handlePostsHttp, listNativePosts, roboDKPostsDir, roboDKPython, runNativePost } from '../server/posts_native';
import { classifyRoboDKPost } from '../src/posts/user_posts';
import { diffPrograms } from './posts_gt_util';

const PY = roboDKPython();
const POSTS = roboDKPostsDir();
const CASES = path.resolve(__dirname, 'fixtures/posts_gt/cases');
const job = (c: string) => JSON.parse(readFileSync(path.join(CASES, c, 'job.json'), 'utf8'));
const gt = (c: string, post: string, file: string) => readFileSync(path.join(CASES, c, post, file), 'utf8');

describe('native RoboDK posts on the server', () => {
  it('finds RoboDK (embedded Python + Posts folder)', () => {
    expect(PY && POSTS, 'C:/RoboDK not installed: native post tests below are skipped').toBeTruthy();
  });

  it.skipIf(!PY || !POSTS)('lists RoboDK posts and runs the compiled KUKA_KRC2 post: all files equal the RoboDK ground truth', async () => {
    const names = listNativePosts();
    expect(names.length).toBeGreaterThanOrEqual(140);
    for (const n of ['KUKA_KRC4', 'ABB_RAPID_IRC5', 'Fanuc_R30i', 'Universal_Robots', 'Motoman']) expect(names).toContain(n);
    const r = await runNativePost('KUKA_KRC2', job('feature_KUKA_KR_6_R900_sixx'));
    expect(r.error).toBeUndefined();
    expect(r.ok).toBe(true);
    expect(r.files.map((f) => f.name).sort()).toEqual(['FeatureTest.src', 'SubPick.src']);
    for (const f of r.files) expect(diffPrograms(gt('feature_KUKA_KR_6_R900_sixx', 'KUKA_KRC2', f.name), f.content), f.name).toBeNull();
  }, 120_000);

  it.skipIf(!PY || !POSTS)('runs a post given by file path (post_shim.py <path> job out) and a user post source', async () => {
    const r = await runNativePost(path.join(POSTS!, 'KUKA_KRC4.py'), job('feature_KUKA_KR_6_R900_sixx'));
    expect(r.ok).toBe(true);
    expect(diffPrograms(gt('feature_KUKA_KR_6_R900_sixx', 'KUKA_KRC4', 'FeatureTest.src'), r.files.find((f) => f.name === 'FeatureTest.src')!.content)).toBeNull();
    const src = [
      'class RobotPost(object):',
      '    PROG_EXT = "txt"',
      '    def __init__(self, robotpost=None, robotname=None, robot_axes=6, **kwargs):',
      '        self.PROG = []',
      '    def ProgStart(self, progname):',
      '        self.PROG.append("START " + progname)',
      '    def ProgFinish(self, progname):',
      '        self.PROG.append("END " + progname)',
      '    def ProgSave(self, folder, progname, ask_user=False, show_result=False):',
      '        open(folder + "/" + progname + ".txt", "w").write("\\n".join(self.PROG))',
      '    def MoveJ(self, pose, joints, conf_RLF=None):',
      '        self.PROG.append("J " + " ".join("%.3f" % j for j in joints))',
      '    def MoveL(self, pose, joints, conf_RLF=None):',
      '        self.PROG.append("L %.3f %.3f %.3f" % (pose[0, 3], pose[1, 3], pose[2, 3]))',
      '    def RunMessage(self, message, iscomment=False):',
      '        pass',
    ].join('\n');
    const u = await runNativePost('MyPost', job('feature_KUKA_KR_6_R900_sixx'), { source: src });
    expect(u.error).toBeUndefined();
    const text = u.files.find((f) => f.name === 'FeatureTest.txt')!.content.split(String.fromCharCode(13)).join(''); // text mode on Windows
    expect(text.split('\n')[0]).toBe('START FeatureTest');
    expect(text).toMatch(/^J 10\.000 -80\.000 100\.000 20\.000 40\.000 -30\.000$/m);
    expect(text.trim().endsWith('END SubPick')).toBe(true); // RoboDK default: all programs in one post instance
  }, 120_000);

  describe.skipIf(!PY || !POSTS)('HTTP route', () => {
    let server: http.Server;
    let base = '';
    const start = () => new Promise<void>((resolve) => {
      server = http.createServer(async (req, res) => {
        const readJson = () => new Promise<any>((ok) => { let b = ''; req.on('data', (c) => (b += c)); req.on('end', () => ok(JSON.parse(b || '{}'))); });
        if (!(await handlePostsHttp(req, res, readJson))) { res.writeHead(404); res.end(); }
      }).listen(0, () => { base = `http://127.0.0.1:${(server.address() as any).port}`; resolve(); });
    });
    afterAll(() => server?.close());
    it('GET /posts/native and POST /posts/native/run', async () => {
      await start();
      const info = await (await fetch(`${base}/posts/native`)).json();
      expect(info.available).toBe(true);
      expect(info.posts).toContain('ABB_RAPID_IRC5');
      const r = await (await fetch(`${base}/posts/native/run`, { method: 'POST', body: JSON.stringify({ post: 'ABB_RAPID_IRC5', job: job('feature_ABB_IRB_120_3_0_6') }) })).json();
      expect(r.ok).toBe(true);
      expect(diffPrograms(gt('feature_ABB_IRB_120_3_0_6', 'ABB_RAPID_IRC5', 'FeatureTest.mod'), r.files[0].content)).toBeNull();
      const bad = await (await fetch(`${base}/posts/native/run`, { method: 'POST', body: JSON.stringify({ post: '../../etc', job: {} }) })).json();
      expect(bad.ok).toBe(false);
    }, 120_000);
  });
});

describe('RoboDK source posts through post_shim.run_job_source (Pyodide entry point)', () => {
  it.skipIf(!PY || !POSTS || !existsSync(path.join(POSTS ?? '', 'CSV.py')))('RoboDK CSV.py as source text gives the same files as the native import', () => {
    const dir = mkdtempSync(path.join(tmpdir(), 'vbs-shim-'));
    try {
      const jp = path.join(dir, 'job.json');
      writeFileSync(jp, JSON.stringify({ ...job('feature_Denso_Cobotta'), include_sub_programs: false }));
      const script = ['import sys, json', `sys.path.insert(0, ${JSON.stringify(path.resolve('python'))})`, 'import post_shim',
        `src = open(${JSON.stringify(path.join(POSTS!, 'CSV.py'))}, encoding="utf-8").read()`, `print(post_shim.run_job_source(src, open(${JSON.stringify(jp)}).read(), ${JSON.stringify(path.join(dir, 'out'))}, "CSV"))`].join('\n');
      writeFileSync(path.join(dir, 'run.py'), script);
      const out = execFileSync(PY!, [path.join(dir, 'run.py')], { encoding: 'utf8', timeout: 120_000 });
      const r = JSON.parse(out.slice(out.indexOf('{')));
      expect(r.ok, r.error).toBe(true);
      const main = r.files.find((f: any) => f.name === 'FeatureTest.csv');
      expect(diffPrograms(gt('feature_Denso_Cobotta', 'CSV', 'FeatureTest.csv'), main.content)).toBeNull();
    } finally { rmSync(dir, { recursive: true, force: true }); }
  }, 120_000);
});

describe('RoboDK Posts folder classification for the browser import', () => {
  it.skipIf(!POSTS)('every RoboDK post is either a source post (Pyodide) or a compiled wrapper (server)', () => {
    const files = readdirSync(POSTS!).filter((f) => f.endsWith('.py'));
    const kinds = files.map((f) => [f, classifyRoboDKPost(readFileSync(path.join(POSTS!, f), 'utf8'))] as const);
    const none = kinds.filter(([, k]) => k === 'none').map(([f]) => f);
    expect(kinds.filter(([, k]) => k !== 'none').length).toBeGreaterThanOrEqual(140);
    expect(none.length, `unclassified: ${none.join(', ')}`).toBeLessThanOrEqual(files.length - 140);
    expect(kinds.find(([f]) => f === 'KUKA_KRC4.py')![1]).toBe('compiled');
    expect(kinds.find(([f]) => f === 'CSV.py')![1]).toBe('source');
  });
});
