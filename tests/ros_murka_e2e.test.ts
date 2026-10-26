/**
 * R5 / C3 end-to-end (opt-in: ROS_WSL=1): the studio is the simulator backend of the real MURKA ROS 2 stack.
 * WSL Ubuntu (ROS 2 Jazzy) runs rosbridge_server (built from source in ~/r5_rosbridge_ws) and the MURKA workspace
 * built from a COPY of robot-retailer-murka-v2/ros2_ws (~/r5_murom_ws): robot_state_publisher (murom.urdf) and
 * murom_navigation localization_node — what murom_webots/webots.launch.py starts besides Webots. The studio (this
 * process) imports retail_store.wbt + murom_world/config, connects to rosbridge and runs the MURKA interface
 * (src/ros/murom_sim.ts). MURKA's own scripts/smoke_test.py (unmodified copy) must print SMOKE PASS: /clock advances,
 * /odom, /scan (>= 50 finite beams), /joint_states (>= 12 joints), /murom/pose, /murom/sim/objects (50 products), the
 * robot drives backwards 3 s by /cmd_vel (>= 0.3 m) and odometry agrees with ground truth (< 0.10 m).
 * Setup (once, ~5 min): tests/fixtures/ros_gt/tools/murka_sim/setup_ws.sh (run by this test when missing).
 */
import { describe, it, expect } from 'vitest';
import fs from 'node:fs';
import path from 'node:path';
import { spawn, execFileSync, type ChildProcess } from 'node:child_process';
import { startSimBackend, type SimBackendHandle } from '../src/ros/sim_backend';
import type { MuromSimInterface } from '../src/ros/murom_sim';
import { HAS_MURKA } from './webots_helpers';
import { buildRetailStation, MURKA_V2 as V2 } from './ros_murka_helpers';

const WSL = process.env.ROS_WSL === '1';
const TOOLS = path.resolve(__dirname, 'fixtures/ros_gt/tools/murka_sim');
const toWsl = (p: string) => `/mnt/${p[0].toLowerCase()}${p.slice(2).split(path.sep).join('/')}`;
const wsl = (args: string[], timeout = 600000) => execFileSync('wsl.exe', ['-d', 'Ubuntu-22.04', '--', ...args], { encoding: 'utf8', timeout, env: { ...process.env, MSYS_NO_PATHCONV: '1' }, windowsHide: true });
const PORT = Number(process.env.R5_PORT ?? 9097);
/** Async WSL command (the studio backend keeps stepping on this event loop meanwhile). */
function wslAsync(args: string[], timeout: number): Promise<{ code: number | null; out: string }> {
  return new Promise((resolve) => {
    let out = '';
    const p = spawn('wsl.exe', ['-d', 'Ubuntu-22.04', '--', ...args], { env: { ...process.env, MSYS_NO_PATHCONV: '1' }, windowsHide: true });
    const timer = setTimeout(() => { out += ' [timeout]'; p.kill(); }, timeout);
    p.stdout.on('data', (d) => { out += String(d); });
    p.stderr.on('data', (d) => { out += String(d); });
    p.on('close', (code) => { clearTimeout(timer); resolve({ code, out }); });
  });
}

function startStack(mode: string): Promise<{ proc: ChildProcess; out: string[] }> {
  return new Promise((resolve, reject) => {
    const out: string[] = [];
    const proc = spawn('wsl.exe', ['-d', 'Ubuntu-22.04', '--', 'bash', toWsl(path.join(TOOLS, 'run_stack.sh')), mode], { env: { ...process.env, MSYS_NO_PATHCONV: '1', WSLENV: 'R5_PORT/u', R5_PORT: String(PORT) }, windowsHide: true });
    const timer = setTimeout(() => reject(new Error(`stack did not start: ${out.join('')}`)), 150000);
    proc.stdout.on('data', (d) => { out.push(String(d)); if (/STACK_READY/.test(out.join(''))) { clearTimeout(timer); resolve({ proc, out }); } if (/STACK_TIMEOUT/.test(out.join(''))) { clearTimeout(timer); reject(new Error(out.join(''))); } });
    proc.stderr.on('data', (d) => out.push(String(d)));
  });
}

describe.skipIf(!WSL || !HAS_MURKA)('MURKA ROS 2 stack against the studio simulator backend (WSL, ROS_WSL=1)', () => {
  it('scripts/smoke_test.py passes: topics alive, robot drives, odometry agrees with ground truth', async () => {
    const setup = wsl(['bash', toWsl(path.join(TOOLS, 'setup_ws.sh')), toWsl(V2)], 1800000);
    expect(setup).toContain('R5_SETUP_OK');
    try { wsl(['bash', toWsl(path.join(TOOLS, 'stop_stack.sh'))], 60000); } catch { /* nothing running */ }
    const stop = () => { try { wsl(['bash', toWsl(path.join(TOOLS, 'stop_stack.sh'))], 60000); } catch { /* ignore */ } };
    const stack = await startStack('loc').catch((e) => { stop(); throw e; });
    let backend: SimBackendHandle | null = null;
    try {
      const { station, assets } = buildRetailStation();
      backend = await startSimBackend({ url: `ws://127.0.0.1:${PORT}`, station, assets, rtf: 1, murom: { cameraRate: 2 }, log: () => {} });
      expect(backend.kind).toBe('murom');
      const smoke = (await wslAsync(['bash', toWsl(path.join(TOOLS, 'smoke.sh')), toWsl(V2)], 180000)).out;
      fs.mkdirSync(path.resolve(__dirname, '_probe'), { recursive: true });
      fs.writeFileSync(path.resolve(__dirname, '_probe/r5_smoke_out.txt'), `${smoke}\nrtf=${backend.rtf().toFixed(2)} steps=${backend.steps()} sim=${backend.time().toFixed(1)}s\n`);
      expect(smoke).toContain('SMOKE PASS');
      expect(smoke).not.toContain('SMOKE FAIL');
      const sim = backend.sim as MuromSimInterface;
      expect(sim.warnings.filter((w) => /Неизвестные/.test(w))).toEqual([]);
      expect(backend.rtf()).toBeGreaterThan(0.5);
    } finally {
      backend?.stop();
      stop();
      stack.proc.kill();
    }
  }, 2400000);

  it('mission stage: the real MURKA FSM + Nav2 (murom_bringup sim:=none) undock and dock the studio robot (mission_test.py)', async () => {
    const stop = () => { try { wsl(['bash', toWsl(path.join(TOOLS, 'stop_stack.sh'))], 60000); } catch { /* ignore */ } };
    stop();
    const stack = await startStack('full').catch((e) => { stop(); throw e; });
    let backend: SimBackendHandle | null = null;
    const report: string[] = [];
    try {
      const { station, assets } = buildRetailStation();
      backend = await startSimBackend({ url: `ws://127.0.0.1:${PORT}`, station, assets, rtf: 1, murom: { cameraRate: 1 }, log: (m) => report.push(m) });
      const sim = backend.sim as MuromSimInterface;
      const dock = { x: sim.base.state.x / 1000, y: sim.base.state.y / 1000, yaw: sim.base.state.theta };
      const where = () => ({ x: sim.base.state.x / 1000, y: sim.base.state.y / 1000, yaw: sim.base.state.theta });
      const undock = (await wslAsync(['bash', toWsl(path.join(TOOLS, 'mission.sh')), toWsl(V2), 'undock', '400'], 900000)).out;
      const p1 = where();
      report.push(`--- undock ---\n${undock}\nstudio pose after undock: ${JSON.stringify(p1)}`);
      const dockOut = (await wslAsync(['bash', toWsl(path.join(TOOLS, 'mission.sh')), toWsl(V2), 'dock', '600'], 1200000)).out;
      const p2 = where();
      report.push(`--- dock ---\n${dockOut}\nstudio pose after dock: ${JSON.stringify(p2)} rtf=${backend.rtf().toFixed(2)} sim=${backend.time().toFixed(1)} s`);
      fs.writeFileSync(path.resolve(__dirname, '_probe/r5_mission_out.txt'), report.join('\n'));
      expect(undock).toContain('MISSION PASS');
      // undock: reverse glide off the charging station to its standoff, 0.75 m from the dock (locations.yaml)
      expect(Math.hypot(p1.x - dock.x, p1.y - dock.y)).toBeGreaterThan(0.5);
      expect(dockOut).toContain('MISSION PASS');
      // dock: back on the charging station within the glide tolerance (2 cm) and the heading of the dock
      expect(Math.hypot(p2.x - dock.x, p2.y - dock.y)).toBeLessThan(0.03);
      expect(Math.abs(((p2.yaw - dock.yaw + 540) % 360) - 180)).toBeLessThan(3);
    } finally {
      backend?.stop();
      stop();
      stack.proc.kill();
    }
  }, 3600000);
});
