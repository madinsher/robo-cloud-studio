/** Shared helpers of the MURKA retail tests: MURKA paths, a Python + numpy runner for the original MURKA modules. */
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { MURKA, HAS_MURKA } from './webots_helpers';

export { MURKA, HAS_MURKA };
export const V2_SRC = path.join(MURKA, 'robot-retailer-murka-v2/ros2_ws/src');
export const V1_SRC = path.join(MURKA, 'robot-retailer-murka/ros2_ws/src');

/** A Python 3 with numpy (the MURKA modules arm_kinematics.py / recognition.py are pure numpy). */
function findPython(): string | null {
  for (const exe of [process.env.MURKA_PYTHON, 'python3', 'python', 'py'].filter(Boolean) as string[]) {
    try { const r = spawnSync(exe, ['-c', 'import numpy, sys; print(sys.version_info[0])'], { encoding: 'utf8', timeout: 120000 }); if (r.status === 0 && r.stdout.trim() === '3') return exe; } catch { /* next */ }
  }
  return null;
}
export const PYTHON = HAS_MURKA ? findPython() : null;
export const HAS_PY = !!PYTHON;

/**
 * Run a Python script against the original MURKA sources (read-only: -B, no bytecode is written next to them) with a
 * JSON input on stdin; returns the parsed JSON printed by the script. Large binary inputs go through files in a temp dir.
 */
export function runPython(script: string, input: unknown, files: Record<string, Buffer> = {}): any {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'murka-r3-'));
  try {
    for (const [n, b] of Object.entries(files)) fs.writeFileSync(path.join(dir, n), b);
    const file = path.join(dir, 'run.py');
    fs.writeFileSync(file, script);
    const r = spawnSync(PYTHON!, ['-B', file], { input: JSON.stringify({ ...(input as object), dir }), encoding: 'utf8', maxBuffer: 64 * 1024 * 1024, timeout: 300000, env: { ...process.env, PYTHONDONTWRITEBYTECODE: '1', PYTHONIOENCODING: 'utf-8' } });
    if (r.status !== 0) throw new Error(`python failed (${r.status}): ${r.stderr}`);
    return JSON.parse(r.stdout.trim().split('\n').pop()!);
  } finally { fs.rmSync(dir, { recursive: true, force: true }); }
}
