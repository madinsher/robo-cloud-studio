/**
 * T10 acceptance (P0 e), opt-in: every RoboDK Library macro / script (C:/RoboDK/Library/Scripts + Macros) runs
 * headless against the studio — the Example-01.a painting cell served by a headless host, a fresh station per script,
 * the drop-in on PYTHONPATH with the unattended GUI (tkinter presses the main button, dialogs default, matplotlib Agg),
 * 30 s per script. Scripts with network / controller / installer / process side effects are skipped by pattern.
 *
 *   T10_SWEEP=1 npx vitest run tests/api_library_sweep.test.ts      (~40 min)
 *
 * Writes a JSON list and a markdown table to $T10_SWEEP_OUT (default: the temp dir) and fails when a script of the
 * EXPECT_OK list (all scripts that exited 0 in the reference sweep of this round) does not exit 0.
 */
import { describe, it, expect } from 'vitest';
import { readFileSync, readdirSync, writeFileSync, mkdirSync, existsSync } from 'node:fs';
import { spawn } from 'node:child_process';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { importRdkBestEffort } from '../src/io/robodk/rdk_container';
import { AssetStore } from '../src/scene/assets';
import { createNodeHost } from '../src/api/node_host';
import { findPython } from '../src/api/runner_cpython';
import { saveStation, loadStation } from '../src/io/station-file';
import '../src/posts/index';

const LIB = 'C:/RoboDK/Library';
const STATION = 'Example-01.a-Robot Painting - Universal Robots.rdk';
const RISKY = /subprocess|os\.system|startfile|pip |import_install|socket|ftplib|serial|urllib|requests|Connect\(|ConnectSafe|webbrowser|shutil\.rmtree|os\.remove|EmbedWindow|PySide|PyQt/;
let py: { cmd: string; args: string[] } | null = null;
try { py = findPython(); } catch { py = null; }

/** Scripts that exited 0 in the reference sweep (T10 status report). */
const EXPECT_OK: string[] = ['Macros/Add_Curve_by_Points.py', 'Macros/Add_Shape_by_Triangles.py', 'Macros/ArcEnd.py', 'Macros/ArcStart.py', 'Macros/CallNC.py', 'Macros/CameraAdd.py', 'Macros/CameraSnapshot.py', 'Macros/CurvesPointsEditExample.py', 'Macros/Depth_Camera.py', 'Macros/DoPointWeld.py', 'Macros/ExportTargets.py', 'Macros/FilterProgram3.py', 'Macros/FilterTarget.py', 'Macros/Follow_Curve_Auto_Setup.py', 'Macros/ForceProgramsMoveL.py', 'Macros/Gcode_Recalculate.py', 'Macros/LoadMachiningPrograms.py', 'Macros/Machining.py', 'Macros/Make_Cube_Program.py', 'Macros/ModifyProgramTargets.py', 'Macros/MoveRobotRelative.py', 'Macros/ObjectEvents.py', 'Macros/PluginCommand.py', 'Macros/PointFollow_XYZSpeed.py', 'Macros/PointLines_2_Points.py', 'Macros/PrintProgramInstructions.py', 'Macros/ProgramExternalAxes.py', 'Macros/Reorganise_Prog.py', 'Macros/SampleOfflineProgramming.py', 'Macros/SetIO.py', 'Macros/SetToolCutter.py', 'Macros/SetupCamera2D.py', 'Macros/SpindleOff.py', 'Macros/SpindleOn.py', 'Macros/SprayOff.py', 'Macros/SprayOn.py', 'Macros/SyncRobot1.py', 'Macros/UpdateKinematics_Turntable2x.py', 'Macros/WaitDI.py', 'Macros/WeldOff.py', 'Macros/WeldOn.py', 'Macros/calibration_targets_tracker_rail.py', 'Scripts/CalibrationExtractPoseMeasurements.py', 'Scripts/Check_Orientation_Reachability.py', 'Scripts/Comau_LIS_Import.py', 'Scripts/Curve_Change_Normals.py', 'Scripts/Curve_Filter_AverageNormals.py', 'Scripts/Curve_Project_2_Surface.py', 'Scripts/CycleTimeDisplay.py', 'Scripts/CycleTimeDisplay2.py', 'Scripts/CycleTimeDisplayAll.py', 'Scripts/CycleTimeStudy.py', 'Scripts/FilterKukaCamrob.py', 'Scripts/ImportCSV_XYZ.py', 'Scripts/Import_KUKA_SRC_Program.py', 'Scripts/Import_Mecademic_Script.py', 'Scripts/JAKA_LoadControllerKinematics.py', 'Scripts/Mesh_Filter.py', 'Scripts/MoveC_2_MoveL.py', 'Scripts/MovePolygon.py', 'Scripts/ProgramAddInstructions.py', 'Scripts/ProgramCopy.py', 'Scripts/ProgramDisplayRobotTrace.py', 'Scripts/ProgramInvertPoses.py', 'Scripts/ProgramListCSV.py', 'Scripts/ProgramReLinkTool.py', 'Scripts/ProgramSplitJoints.py', 'Scripts/RobotsDisconnect.py', 'Scripts/ScaleXYZ.py', 'Scripts/SetColorCurves.py', 'Scripts/SetColorToolpaths.py', 'Scripts/ShowAsPointCloud.py', 'Scripts/ShowCommands.py', 'Scripts/StopAllPrograms.py', 'Scripts/TakeSnapshot.py', 'Scripts/TakeSnapshotHighRes.py', 'Scripts/UR_LoadControllerKinematics.py'];

describe.skipIf(!process.env.T10_SWEEP || !py || !existsSync(`${LIB}/${STATION}`))('RoboDK Library Scripts/Macros sweep (headless studio)', () => {
  it('runs every script and keeps the reference passes', async () => {
    const b = readFileSync(`${LIB}/${STATION}`);
    const assets = new AssetStore();
    const { station } = await importRdkBestEffort(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength), STATION, assets);
    const snap = JSON.stringify(saveStation(station, assets));
    const host = await createNodeHost({ station, assets, port: 0, tcpPort: -1, speed: 50, log: () => {} });
    const out = process.env.T10_SWEEP_OUT ?? path.join(tmpdir(), 't10_library_sweep');
    const cwd = path.join(out, 'cwd');
    mkdirSync(cwd, { recursive: true });
    const files = [...readdirSync(`${LIB}/Scripts`).map((f) => `Scripts/${f}`), ...readdirSync(`${LIB}/Macros`).map((f) => `Macros/${f}`)].filter((f) => f.endsWith('.py'));
    const rows: Array<{ f: string; skipped?: string; code?: number | null; err?: string; secs?: number }> = [];
    try {
      for (const f of files) {
        const src = readFileSync(`${LIB}/${f}`, 'utf8');
        const risky = src.match(RISKY);
        if (risky) { rows.push({ f, skipped: risky[0] }); continue; }
        host.setStation(loadStation(JSON.parse(snap), assets));
        // macros write next to the open station (CSV, images, programs): the sweep folder, never C:/RoboDK (read-only)
        const scratch = path.join(out, 'station').split(path.sep).join('/');
        mkdirSync(scratch, { recursive: true });
        host.rdk.paths.PATH_OPENSTATION = scratch;
        host.rdk.paths.FILE_OPENSTATION = `${scratch}/${STATION}`;
        host.rdk.paths.PATH_PROGRAMS = `${scratch}/programs`;
        host.rdk.setRunMode(1);
        host.rdk.setSimulationSpeed(50);
        const t0 = Date.now();
        const r = await new Promise<{ code: number | null; text: string }>((resolve) => {
          const env = { ...process.env, STUDIO_URL: host.url, PYTHONPATH: [path.join(process.cwd(), 'python', 'robodk', '_headless_gui'), path.join(process.cwd(), 'python')].join(path.delimiter), PYTHONIOENCODING: 'utf-8', MPLBACKEND: 'Agg' };
          // run a copy: macros that write next to their own file (APT_Split -> apt-out.apt) write into the sweep folder
          const copy = path.join(cwd, path.basename(f));
          writeFileSync(copy, src);
          const c = spawn(py!.cmd, [...py!.args, '-B', '-u', copy], { cwd, env, windowsHide: true, stdio: ['ignore', 'pipe', 'pipe'] });
          let text = '';
          c.stdout.on('data', (d) => (text += d));
          c.stderr.on('data', (d) => (text += d));
          const tm = setTimeout(() => { text += '\nTIMEOUT'; c.kill(); }, 30000);
          c.on('close', (code) => { clearTimeout(tm); resolve({ code, text }); });
        });
        host.rdk.exec.stop();
        const lines = r.text.trim().split(/\r?\n/).filter((l) => l.trim());
        const err = r.code === 0 ? '' : [...lines].reverse().find((l) => /^\w*(Error|Exception)\b|Error:|Exception:|TIMEOUT/.test(l)) ?? lines[lines.length - 1] ?? '';
        rows.push({ f, code: r.code, err: err.slice(0, 200), secs: (Date.now() - t0) / 1000 });
      }
    } finally { await host.close(); }
    writeFileSync(path.join(out, 'results.json'), JSON.stringify(rows, null, 1));
    const md = ['| Script | Result | Detail |', '|---|---|---|', ...rows.map((x) => `| ${x.f} | ${x.skipped ? 'skipped' : x.code === 0 ? 'OK' : /TIMEOUT/.test(x.err ?? '') ? 'timeout' : 'error'} | ${(x.skipped ? `side effect: ${x.skipped}` : x.err ?? '').replace(/\|/g, '/')} |`)];
    writeFileSync(path.join(out, 'results.md'), md.join('\n') + '\n');
    const ok = new Set(rows.filter((x) => x.code === 0).map((x) => x.f));
    expect(EXPECT_OK.filter((f) => !ok.has(f)), 'reference passes that now fail').toEqual([]);
    expect(ok.size).toBeGreaterThanOrEqual(EXPECT_OK.length);
  }, 3 * 3600 * 1000);
});
