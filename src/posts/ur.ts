/**
 * Universal Robots URScript post, output-compatible with RoboDK's Universal_Robots post (tests/posts_gt.test.ts):
 * one .script with global speed/blend variables, sub-programs defined inside the main def, poses relative to
 * ref_frame (pose_trans), movec with RoboDK's radius checks and trace ending, standard / tool / configurable IO by
 * number range, plus the .urp Polyscope wrapper. Studio extensions: loops (while / for counters), if, threads.
 */
import { Mat4 } from '../core/math/pose';
import { PostProcessor, PostProgram, PostFile, registerPost } from './base';
import { RoboPost, RoboPostCtx, runRoboPost, fx, IoName, CtlCondition } from './robopost';
import { pose2UR } from './euler';
import { gzipStore, toBase64, utf8 } from './zip';

const DEG = Math.PI / 180;
export const urPose = (m: Mat4): string => { const [x, y, z, a, b, c] = pose2UR(m); return `p[${[x / 1000, y / 1000, z / 1000, a, b, c].map((v) => fx(v, 6)).join(', ')}]`; };

const UR_HEADER = (name: string) => `def ${name}():
  # Global parameters:
  global speed_ms    = 0.250
  global speed_rads  = 0.750
  global accel_mss   = 1.200
  global accel_radss = 1.200
  global blend_radius_m = 0.001
  global ref_frame = p[0,0,0,0,0,0]

  #
  # TO REMOVE HEADER:
  # Go to "Program" -> "Post-Processor Editor"
  # Select "Universal Robots"
  # Set "INCLUDE_HEADER" to "False"
  #
  #--------------------------
  # Add any default subprograms here
  # For example, to drive a gripper as a program call:
  # def Gripper_Open():
  #   ...
  # end
  #
  # Example to drive a spray gun:
  def SprayOn(value):
    # use the value as an output:
    DO_SPRAY = 5
    if value == 0:
      set_standard_digital_out(DO_SPRAY, False)
    else:
      set_standard_digital_out(DO_SPRAY, True)
    end
  end

  # Example to drive an extruder:
  def Extruder(value):
    # use the value as an output:
    if value < 0:
      # stop extruder
    else:
      # start extruder
    end
  end

  # Example to move an external axis
  def MoveAxis(value):
    # use the value as an output:
    DO_AXIS_1 = 1
    DI_AXIS_1 = 1
    if value <= 0:
      set_standard_digital_out(DO_AXIS_1, False)

      # Wait for digital input to change state
      #while (get_standard_digital_in(DI_AXIS_1) != False):
      #  sync()
      #end
    else:
      set_standard_digital_out(DO_AXIS_1, True)

      # Wait for digital input to change state
      #while (get_standard_digital_in(DI_AXIS_1) != True):
      #  sync()
      #end
    end
  end
  #--------------------------
  `;

/** Radius (mm) of the circle through three points (Infinity when collinear, NaN when degenerate). */
export function circleRadius(p0: number[], p1: number[], p2: number[]): number {
  const a = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]], b = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]];
  const c = [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const n = (v: number[]) => Math.hypot(v[0], v[1], v[2]);
  const amb = [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  return (n(a) * n(b) * n(amb)) / (2 * n(c));
}

interface Prog { name: string; lines: string[] }

export class URPost extends RoboPost {
  override PROG_EXT = 'script';
  protected progs: Prog[] = [];
  protected cur!: Prog;
  protected ind = '  ';
  protected blend = 0.001; // m
  protected lastPos: number[] | null = null;
  protected traceOpen = false;
  protected lastMotion: { idx: number; render: (b: string) => string; prog: Prog } | null = null;
  protected threads: string[] = [];
  protected loopN = 0;
  protected speedMs = 250;
  override addline(s: string): void { this.cur.lines.push(this.ind + s); }
  /** Code that must not run while the robot blends (calls, IO, waits, speed changes) first stops the previous motion. */
  protected codeLine(s: string): void { this.endTrace(); this.addline(s); }
  protected addMotion(render: (b: string) => string): void {
    this.cur.lines.push(this.ind + render(fx(this.blend)));
    this.lastMotion = { idx: this.cur.lines.length - 1, render, prog: this.cur };
    this.traceOpen = this.blend > 0;
  }
  /** Like RoboDK: a blended motion directly followed by code is rewritten with blend 0 and marked "# end trace". */
  protected endTrace(): void {
    const m = this.lastMotion;
    // only when the code line directly follows the blended motion (any other line in between ends the blend already)
    if (!this.traceOpen || !m || m.prog !== this.cur || m.idx !== this.cur.lines.length - 1) { this.traceOpen = false; return; }
    m.prog.lines[m.idx] = `${this.ind}${m.render('0')} # end trace`;
    this.traceOpen = false;
  }
  override ProgStart(name: string): void {
    this.traceOpen = false;
    this.cur = { name, lines: [] };
    this.progs.push(this.cur);
    this.ind = this.progs.length === 1 ? '  ' : '    ';
  }
  override ProgSave(name: string): PostFile[] {
    const [main, ...subs] = this.progs;
    const L: string[] = [UR_HEADER(main.name)];
    for (const s of subs) L.push(`  # Subprogram ${s.name}`, `  def ${s.name}():`, ...s.lines, '  end', '  ');
    for (const t of this.threads) L.push(`  thread Thread_${t}():`, `    ${t}()`, '  end');
    L.push('  ', '  # Main program:', ...main.lines, '  # End of main program', 'end', '', `${main.name}()`, '');
    const script = L.join('\r\n');
    const esc = script.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
    const urp = `<URProgram createdIn="3.0.0" lastSavedIn="3.0.0" name="${main.name}" directory="." installation="default">\n  <children>\n    <MainProgram runOnlyOnce="true" motionType="MoveJ" speed="1.0471975511965976" acceleration="1.3962634015954636" useActiveTCP="false">\n      <children>\n        <Script type="File">\n          <cachedContents>${esc}\n</cachedContents>\n          <file>${main.name}.script</file>\n        </Script>\n      </children>\n    </MainProgram>\n  </children>\n</URProgram>`;
    return [{ name: `${name}.script`, content: script }, { name: `${name}.urp`, content: toBase64(gzipStore(utf8(urp), name)), encoding: 'base64', mime: 'application/gzip' }];
  }
  protected pos(m: Mat4 | null) { return m ? [m[12], m[13], m[14]] : null; }
  MoveJ(pose: Mat4 | null, joints: number[]): void {
    const j = `[${[0, 1, 2, 3, 4, 5].map((i) => fx((joints[i] ?? 0) * DEG, 6)).join(', ')}]`;
    this.addMotion((b) => `movej(${j},accel_radss,speed_rads,0,${b})`);
    this.lastPos = this.pos(pose);
  }
  MoveL(pose: Mat4 | null, joints: number[]): void {
    if (!pose) { this.MoveJ(null, joints); return; }
    const p = urPose(pose);
    this.addMotion((b) => `movel(pose_trans(ref_frame,${p}),accel_mss,speed_ms,0,${b})`);
    this.lastPos = this.pos(pose);
  }
  override MoveC(pose1: Mat4 | null, joints1: number[], pose2: Mat4 | null, joints2: number[]): void {
    if (!pose1 || !pose2) { this.MoveL(pose2, joints2); return; }
    const r = this.lastPos ? circleRadius(this.lastPos, this.pos(pose1)!, this.pos(pose2)!) : NaN;
    if (!(r >= 1 && r <= 10000)) { this.MoveL(pose2, joints2); return; }
    const p1 = urPose(pose1), p2 = urPose(pose2);
    this.addMotion((b) => `movec(pose_trans(ref_frame,${p1}),pose_trans(ref_frame,${p2}),accel_mss,speed_ms,${b})`);
    this.lastPos = this.pos(pose2);
    void joints1;
  }
  override setFrame(pose: Mat4): void { this.addline(`ref_frame = ${urPose(pose)}`); }
  override setTool(pose: Mat4): void { this.addline(`set_tcp(${urPose(pose)})`); }
  override Pause(ms: number): void { this.addline(ms <= 0 ? 'halt() # reimplement this function to force stop' : `sleep(${fx(ms / 1000)})`); }
  override setSpeed(v: number): void { if (v === this.speedMs) return; this.speedMs = v; this.codeLine(`speed_ms    = ${fx(v / 1000)}`); }
  override setSpeedJoints(w: number): void { this.addline(`speed_rads  = ${fx(w * DEG)}`); }
  override setAcceleration(a: number): void { this.addline(`accel_mss   = ${fx(a / 1000)}`); }
  override setAccelerationJoints(a: number): void { this.addline(`accel_radss = ${fx(a * DEG)}`); }
  override setZoneData(z: number): void { this.blend = Math.max(0, z) / 1000; this.addline(`blend_radius_m = ${fx(this.blend)}`); }
  protected bool(v: number | boolean | string) { return (typeof v === 'string' ? ['1', 'true', 'on'].includes(v.toLowerCase()) : Number(v) > 0) ? 'True' : 'False'; }
  /** RoboDK UR IO numbering: 0-9 standard, 10-19 tool (n-10), 20+ configurable (n-20). */
  override setDO(io: IoName, v: number | boolean | string): void {
    if (typeof io !== 'number') { this.codeLine(`${io} = ${typeof v === 'string' ? v : Number(v) > 0 ? 1 : 0}`); return; }
    if (io < 10) this.codeLine(`set_standard_digital_out(${io}, ${this.bool(v)})`);
    else if (io < 20) this.codeLine(`set_tool_digital_out(${io - 10}, ${this.bool(v)})`);
    else this.codeLine(`set_configurable_digital_out(${io - 20}, ${this.bool(v)})`);
  }
  override setAO(io: IoName, v: number | string): void {
    if (typeof io !== 'number') this.codeLine(`${io} = ${v}`);
    else this.codeLine(`set_standard_analog_out(${io}, ${v})`);
  }
  override waitDI(io: IoName, v: number | boolean | string): void {
    const src = typeof io !== 'number' ? io : io < 10 ? `get_standard_digital_in(${io})` : io < 20 ? `${io - 10}` : `get_configurable_digital_in(${io - 20})`;
    this.codeLine(`while (${src} != ${this.bool(v)}):`);
    this.addline('  sync()');
    this.addline('end');
  }
  override RunCode(code: string, isCall = false): void { this.codeLine(isCall && !code.includes('(') ? `${code.replace(/ /g, '')}()` : code); }
  override RunMessage(msg: string, isComment = false): void { this.addline(isComment ? `# ${msg}` : `popup("${msg}","Message",False,False,blocking=False)`); }
  // ---- control flow (URScript) ----
  override ctlLoop(count: number, body: string): void {
    if (count <= 0) { this.codeLine('while (True):'); this.addline(`  ${body}()`); this.addline('end'); return; }
    const v = `loop_i${++this.loopN}`;
    this.codeLine(`${v} = 0`);
    this.addline(`while (${v} < ${Math.trunc(count)}):`);
    this.addline(`  ${body}()`);
    this.addline(`  ${v} = ${v} + 1`);
    this.addline('end');
  }
  override ctlIf(cond: CtlCondition, thenProg: string | null, elseProg: string | null): void {
    const io = cond.io;
    const c = io !== undefined ? `${typeof io !== 'number' ? io : io < 10 ? `get_standard_digital_in(${io})` : io < 20 ? `get_tool_digital_in(${io - 10})` : `get_configurable_digital_in(${io - 20})`} ${cond.op ?? '=='} ${this.bool(cond.value ?? 1)}` : cond.text;
    this.codeLine(`if (${c}):`);
    this.addline(thenProg ? `  ${thenProg}()` : '  sync()');
    if (elseProg) { this.addline('else:'); this.addline(`  ${elseProg}()`); }
    this.addline('end');
  }
  /** Studio extension: a program started in parallel runs in a URScript thread (declared once before the main program). */
  override ctlThread(name: string): void {
    if (!this.threads.includes(name)) this.threads.push(name);
    this.codeLine(`thrd_${name} = run Thread_${name}()`);
  }
}

export const UR_SCRIPT: PostProcessor = {
  id: 'Universal_Robots', name: 'Universal Robots (URScript)', brand: 'UR', extension: 'script',
  aliases: ['Universal_Robots_RobotiQ', 'Universal_Robots_MoveP', 'Universal_Robots_Conveyor', 'Universal_Robots_3D_Printing'],
  description: 'URScript program + .urp wrapper (RoboDK Universal_Robots compatible).',
  generate(p: PostProgram, options: Record<string, any> = {}): PostFile[] { return runRoboPost((ctx: RoboPostCtx) => new URPost(ctx), p, options, true); },
};
registerPost(UR_SCRIPT);
