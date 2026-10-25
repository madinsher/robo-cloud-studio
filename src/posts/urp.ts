/**
 * Universal Robots URP post, output-compatible with RoboDK's Universal_Robots_URP post (tests/posts_gt.test.ts): a
 * URScript with poses converted to the robot base (the reference frame is only documented, RoboDK writes
 * set_reference as a comment), numeric movej / movec parameters, and a Polyscope .urp program tree (Move / Waypoint
 * nodes with joints, TCP and flange poses, Set / Wait / Popup / Halt nodes, CallSubProgram nodes, script lines spelled
 * character by character) so the program opens as a native Polyscope program.
 */
import { Mat4, multiply, invert } from '../core/math/pose';
import { PostFile, PostProcessor, PostProgram, registerPost } from './base';
import { RoboPost, RoboPostCtx, runRoboPost, fx, IoName } from './robopost';
import { pose2UR } from './euler';
import { gzipStore, toBase64, utf8 } from './zip';
import { circleRadius } from './ur';

const DEG = Math.PI / 180;
const HEADER: string[] = ["  # Global parameters:", "  global speed_ms    = 0.250", "  global speed_rads  = 0.750", "  global accel_mss   = 1.200", "  global accel_radss = 1.200", "  global blend_radius_m = 0.000", "  ", "  #--------------------------", "  # Add any default subprograms here", "  # For example, to drive a gripper as a program call:", "  # def Gripper_Open():", "  #   ...", "  # end", "  #", "  # Example to drive a spray gun:", "  def SprayOn(value):", "    # use the value as an output:", "    DO_SPRAY = 5", "    if value == 0:", "      set_standard_digital_out(DO_SPRAY, False)", "    else:", "      set_standard_digital_out(DO_SPRAY, True)", "    end", "  end", "", "  # Example to synchronize 2", "  def Synchronize():", "    # Use the following digital output to signal the state of the robot:", "    DO_SYNC = 1", "    ", "    # Use the following digital input to get the state of another robot:", "    DI_SYNC = 1", "    ", "    if (get_standard_digital_out(DO_SYNC) == get_standard_digital_in(DI_SYNC)):", "      set_standard_digital_out(DO_SYNC, not (get_standard_digital_out(DI_SYNC)))", "      sleep(0.1)", "      thread Thread_wait_1():", "        while (True):", "          sleep(0.01)", "        end", "      end", "      if (get_standard_digital_out(DO_SYNC) != get_standard_digital_in(DI_SYNC)):", "        global thread_handler_1=run Thread_wait_1()", "        while (get_standard_digital_out(DO_SYNC) != get_standard_digital_in(DI_SYNC)):", "          sync()", "        end", "        kill thread_handler_1", "      end", "    else:", "      if (get_standard_digital_out(DO_SYNC) != get_standard_digital_in(DI_SYNC)):", "        set_standard_digital_out(DO_SYNC, not (get_standard_digital_out(DO_SYNC)))", "      end", "    end", "  end", "", "  #", "  # Example to move an external axis", "  def MoveAxis(value):", "    # use the value as an output:", "    DO_AXIS_1 = 1", "    DI_AXIS_1 = 1", "    if value <= 0:", "      set_standard_digital_out(DO_AXIS_1, False)", "      ", "      # Wait for digital input to change state", "      #while (get_standard_digital_in(DI_AXIS_1) != False):", "      #  sync()", "      #end", "    else:", "      set_standard_digital_out(DO_AXIS_1, True)", "      ", "      # Wait for digital input to change state", "      #while (get_standard_digital_in(DI_AXIS_1) != True):", "      #  sync()", "      #end", "    end", "  end", "  #--------------------------"];
const truthy = (v: number | boolean | string) => (typeof v === 'string' ? ['1', 'true', 'on'].includes(v.toLowerCase()) : Number(v) > 0);
const esc = (s: string) => s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
const v6 = (m: Mat4) => { const [x, y, z, a, b, c] = pose2UR(m); return [x / 1000, y / 1000, z / 1000, a, b, c].map((v) => fx(v, 6)).join(', '); };
/** RoboDK filters call names (FilterName): punctuation such as < > & * : % = + @ ! # ^ | ? ; [ ] / \ and blanks is removed. */
const FILTERED = new Set([' ', '<', '>', '&', '*', ':', '%', '=', '+', '@', '!', '#', '^', '|', '?', ';', '[', ']', '/', String.fromCharCode(92)]);

interface Prog { name: string; lines: string[]; xml: string[] }
export class URPPost extends RoboPost {
  override PROG_EXT = 'script';
  protected progs: Prog[] = [];
  protected cur!: Prog;
  protected ind = '  ';
  protected frame: Mat4 | null = null;
  protected tool: Mat4 | null = null;
  protected speedMs = 0.25;
  protected speedRads = 0.75;
  protected accelMss = 1.2;
  protected accelRadss = 1.2;
  protected blend = 0;
  protected lastLinear = false;
  protected lastPos: number[] | null = null;
  protected nWay = 0;
  override addline(s: string): void { this.cur.lines.push(this.ind + s); }
  /** .urp nodes are written for the main program only (sub-programs are separate Polyscope programs). */
  protected node(...xml: string[]): void { if (this.progs[0] === this.cur) { this.cur.xml.push(...xml); this.openMove = null; } }
  /** Last Move node of the tree (consecutive linear moves with the same speed / acceleration share one Move node). */
  protected openMove: string | null = null;
  protected script(expr: string): void { this.node('        <Script type="Line">', '          <expression>', ...[...expr].map((ch) => `            <ExpressionChar character="${esc(ch)}"/>`), '          </expression>', '        </Script>'); }
  override ProgStart(name: string): void {
    this.cur = { name, lines: [], xml: [] };
    this.progs.push(this.cur);
    this.ind = this.progs.length === 1 ? '  ' : '    ';
  }
  override ProgSave(name: string): PostFile[] {
    const [main, ...subs] = this.progs;
    const L: string[] = [`def ${main.name}():`, ...HEADER, '  '];
    for (const s of subs) L.push(`  # Subprogram ${s.name}`, `  def ${s.name}():`, ...s.lines, '  end', '  ');
    // RoboDK drops the closing comment once the program exceeds its line limit
    L.push('  ', '  # Main program:', ...main.lines, ...(main.lines.length > (this.ctx.options.maxLines ?? 1e9) ? [] : ['  # End of main program']), 'end', '', `${main.name}()`, '');
    const script = L.join('\r\n');
    // .urp: main program tree; repeated calls reference the first CallSubProgram node of that name
    const firstCall = new Map<string, number>();
    const X: string[] = [`<URProgram createdIn="3.0.0" lastSavedIn="3.0.0" name="${esc(main.name)}" directory="/programs" installation="default">`, '  <children>',
      '    <MainProgram runOnlyOnce="true" motionType="MoveJ" speed="1.0471975511965976" acceleration="1.3962634015954636" useActiveTCP="false">', '      <children>'];
    let callIdx = 0;
    const idx = (k: number) => (k > 1 ? `[${k}]` : '');
    for (const l of main.xml) {
      if (!l.startsWith('@CALL ')) { X.push(l); continue; }
      callIdx++;
      const nm = l.slice(6);
      X.push('        <CallSubProgram>');
      if (firstCall.has(nm)) X.push(`          <subprogram reference="../../CallSubProgram${idx(firstCall.get(nm)!)}/subprogram"/>`);
      else {
        firstCall.set(nm, callIdx);
        X.push(`          <subprogram name="${esc(nm)}" keepHidden="true" keepSynchronizedWithDisk="true" trackProgramExecution="false">`, `            <programFile resolves-to="file">/programs/${esc(nm)}.urp</programFile>`, '            <children>', '            </children>', '          </subprogram>');
      }
      X.push('        </CallSubProgram>');
    }
    X.push('      </children>', '    </MainProgram>');
    for (const k of firstCall.values()) X.push(`    <SubProgram reference="../MainProgram/children/CallSubProgram${idx(k)}/subprogram"/>`);
    X.push('  </children>', '</URProgram>');
    return [{ name: `${name}.script`, content: script }, { name: `${name}.urp`, content: toBase64(gzipStore(utf8(X.join('\n')), name)), encoding: 'base64', mime: 'application/gzip' }];
  }
  protected base(pose: Mat4): Mat4 { return this.frame ? multiply(this.frame, pose) : pose; }
  protected waypoint(type: 'MoveJ' | 'MoveL', joints: number[], pose: Mat4 | null, blend: number): void {
    const b = pose ? this.base(pose) : null;
    const tcp = b ? v6(b) : '';
    const flange = b ? v6(this.tool ? multiply(b, invert(this.tool)) : b) : '';
    const sp = type === 'MoveJ' ? this.speedRads : this.speedMs, ac = type === 'MoveJ' ? this.accelRadss : this.accelMss;
    const head = `        <Move motionType="${type}" speed="${fx(sp)}" acceleration="${fx(ac)}" useActiveTCP="true">`;
    const wp = [`            <Waypoint type="Fixed" name="Waypoint_${++this.nWay}">`, ...(blend > 0 ? [`              <motionParameters blendRadius="${fx(blend)}"/>`] : []),
      `              <position joints="${joints.slice(0, 6).map((v) => fx(v * DEG, 6)).join(', ')}" pose="${tcp}"/>`, `              <poseInFeatureCoordinates pose="${tcp}"/>`, `              <outputFlangePose pose="${flange}"/>`, '            </Waypoint>'];
    if (this.progs[0] !== this.cur) return;
    if (type === 'MoveL' && this.openMove === head) { this.cur.xml.splice(this.cur.xml.length - 2, 0, ...wp); return; }
    this.node(head, '          <children>', ...wp, '          </children>', '        </Move>');
    this.openMove = head;
  }
  /** RoboDK blends a joint move only when it follows a linear / circular move. */
  MoveJ(pose: Mat4 | null, joints: number[]): void {
    const b = this.lastLinear ? this.blend : 0;
    const line = `movej([${[0, 1, 2, 3, 4, 5].map((i) => fx((joints[i] ?? 0) * DEG, 6)).join(', ')}],${fx(this.accelRadss, 5)},${fx(this.speedRads, 5)},0,${fx(b, 4)})`;
    this.addline(line);
    this.script(line);
    this.waypoint('MoveJ', joints, pose, 0);
    this.lastLinear = false;
    this.lastPos = pose ? Array.from(this.base(pose).subarray(12, 15)) : null;
  }
  MoveL(pose: Mat4 | null, joints: number[]): void {
    if (!pose) { this.MoveJ(null, joints); return; }
    this.addline(`movel(p[${v6(this.base(pose))}],accel_mss,speed_ms,0,${fx(this.blend)})`);
    this.waypoint('MoveL', joints, pose, this.blend);
    this.lastLinear = true;
    this.lastPos = Array.from(this.base(pose).subarray(12, 15));
  }
  override MoveC(pose1: Mat4 | null, joints1: number[], pose2: Mat4 | null, joints2: number[]): void {
    if (!pose1 || !pose2) { this.MoveL(pose2, joints2); return; }
    const p1 = this.base(pose1), p2 = this.base(pose2);
    const r = this.lastPos ? circleRadius(this.lastPos, Array.from(p1.subarray(12, 15)), Array.from(p2.subarray(12, 15))) : NaN;
    if (!(r >= 1 && r <= 10000)) { this.MoveL(pose2, joints2); return; }
    const c = `Circular move (R${fx(r, 1)} mm)`;
    const line = `movec(p[${v6(p1)}],p[${v6(p2)}],${fx(this.accelMss, 5)},${fx(this.speedMs, 5)},${fx(this.blend, 4)})`;
    this.addline(`# ${c}`);
    this.addline(line);
    this.node(`        <Comment comment="${esc(c)}"/>`);
    this.script(line);
    this.lastLinear = true;
    this.lastPos = Array.from(p2.subarray(12, 15));
    void joints1;
  }
  override setFrame(pose: Mat4, _id: number, name: string): void {
    this.frame = pose;
    const c = `Using Ref. ${name}: p[${v6(pose)}]`;
    this.addline(`# ${c}`);
    this.addline(`# set_reference(p[${v6(pose)}])`);
    this.node(`        <Comment comment="${esc(c)}"/>`);
  }
  override setTool(pose: Mat4, _id: number, name: string): void {
    this.tool = pose;
    const c = `Using TCP ${name}: p[${v6(pose)}]`, line = `set_tcp(p[${v6(pose)}])`;
    this.addline(`# ${c}`);
    this.addline(line);
    this.node(`        <Comment comment="${esc(c)}"/>`);
    this.script(line);
  }
  override Pause(ms: number): void {
    if (ms <= 0) { this.addline('halt() # reimplement this function to force stop'); this.node('        <Halt/>'); return; }
    this.addline(`sleep(${fx(ms / 1000)})`);
    this.node('        <Wait type="Sleep">', `          <waitTime>${fx(ms / 1000)}</waitTime>`, '        </Wait>');
  }
  override setSpeed(v: number): void { this.speedMs = v / 1000; this.addline(`speed_ms    = ${fx(this.speedMs, 4)}`); }
  override setSpeedJoints(w: number): void { this.speedRads = w * DEG; this.addline(`speed_rads  = ${fx(this.speedRads, 4)}`); }
  override setAcceleration(a: number): void { this.accelMss = a / 1000; this.addline(`accel_mss   = ${fx(this.accelMss, 4)}`); }
  override setAccelerationJoints(a: number): void { this.accelRadss = a * DEG; this.addline(`accel_radss = ${fx(this.accelRadss)}`); }
  override setZoneData(z: number): void { this.blend = Math.max(0, z) / 1000; this.addline(`blend_radius_m = ${fx(this.blend, 4)}`); }
  override setDO(io: IoName, v: number | boolean | string): void {
    const on = truthy(v);
    this.addline(typeof io === 'number' ? `set_standard_digital_out(${io}, ${on ? 'True' : 'False'})` : `${io} = ${on ? 'True' : 'False'}`);
    this.node('        <Set type="DigitalOutput">', `          <pin referencedName="${esc(typeof io === 'number' ? `digital_out[${io}]` : io)}"/>`, `          <digitalValue>${on ? 1 : 0}</digitalValue>`, '        </Set>');
  }
  override waitDI(io: IoName, v: number | boolean | string): void {
    const on = truthy(v), src = typeof io === 'number' ? `get_standard_digital_in(${io})` : io;
    this.addline(`while (${src} != ${on ? 'True' : 'False'}):`);
    this.addline('  sync()');
    this.addline('end');
    this.node('        <Wait type="DigitalInput">', `          <pin referencedName="${esc(typeof io === 'number' ? `digital_in[${src}]` : io)}"/>`, `          <digitalValue>${on ? 1 : 0}</digitalValue>`, '        </Wait>');
  }
  override RunCode(code: string, isCall = false): void {
    if (!isCall) { this.addline(code); this.script(code); return; }
    const c = [...code.trim()].filter((ch) => !FILTERED.has(ch)).join('');
    this.addline(c.includes('(') ? c : `${c}()`);
    this.node(`@CALL ${c.replace(/\(.*$/, '')}`);
  }
  override RunMessage(msg: string, isComment = false): void {
    if (isComment) { this.addline(`# ${msg}`); this.node(`        <Comment comment="${esc(msg)}"/>`); return; }
    this.addline(`popup("${msg}","Message",False,False,blocking=False)`);
    this.node(`        <Popup type="Message" haltProgram="false" message="${esc(msg)}"/>`);
  }
}

export const UR_URP: PostProcessor = {
  id: 'Universal_Robots_URP', name: 'Universal Robots (URScript + Polyscope URP)', brand: 'UR', extension: 'script', aliases: [],
  description: 'URScript with base coordinates + native Polyscope .urp program tree (RoboDK Universal_Robots_URP compatible).',
  generate(p: PostProgram, options: Record<string, any> = {}): PostFile[] { return runRoboPost((ctx: RoboPostCtx) => new URPPost(ctx), p, options, true); },
};
registerPost(UR_URP);
