/**
 * Stäubli VAL3 post, output-compatible with RoboDK's Staubli_VAL3 post (tests/posts_gt.test.ts): a VAL3 project per
 * program (<name>/<name>.pjx + .dtx, start / stop / main.pgx and the MoveJoint / MoveLine / MoveCirc helper
 * programs), pointRx configuration (shoulder / elbow / wrist from RoboDK's conf_RLF), mdesc speed / blend, Input /
 * Output dio arrays, watch() waits, and program splitting into libraries loaded one after the other by a
 * Main<name> project. Like RoboDK the project holds one frame (fFrame1 = the last frame set) and one tool, and calls
 * to other programs are written as comments (they live in other projects).
 */
import { Mat4 } from '../core/math/pose';
import { PostFile, PostProcessor, PostProgram, registerPost } from './base';
import { RoboPost, RoboPostCtx, runRoboPost, fx, IoName } from './robopost';
import { pose2Staubli } from './euler';

const XML = '<?xml version="1.0" encoding="utf-8"?>';
const PGX_NS = 'xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="http://www.staubli.com/robotics/VAL3/Program/2"';
const truthy = (v: number | boolean | string) => (typeof v === 'string' ? ['1', 'true', 'on'].includes(v.toLowerCase()) : Number(v) > 0);
const pgx = (name: string, code: string, extra = '', attrs = '') => [XML, `<Programs ${PGX_NS}>`, `  <Program name="${name}"${attrs}>`, ...(extra ? [extra] : []), `    <Code><![CDATA[${code}]]></Code>`, '  </Program>', '</Programs>'].join('\r\n') + '\r\n';
const params = (names: string[]) => ['    <Parameters xmlns="http://www.staubli.com/robotics/VAL3/Param/1">', ...names.map((n) => `      <Parameter name="${n}" type="num" xsi:type="element" />`), '    </Parameters>'].join('\r\n');
const AX = ['x_nX', 'x_nY', 'x_nZ', 'x_nRx', 'x_nRy', 'x_nRz'];
const TR = ['x', 'y', 'z', 'rx', 'ry', 'rz'];
const MOVE_JOINT = (n: number) => pgx('MoveJoint', ['begin', ...Array.from({ length: n }, (_, i) => `  l_jJoint.j${i + 1} = x_nJ${i + 1}`), '  movej(l_jJoint,tCurrentTool ,mCurrentSpeed)', 'end'].join('\r\n'),
  [params(Array.from({ length: n }, (_, i) => `x_nJ${i + 1}`)), '    <Locals>', '      <Local name="l_jJoint" type="joint" xsi:type="array" size="1" />', '    </Locals>'].join('\r\n'));
const MOVE_LINE = pgx('MoveLine', ['begin', ...TR.map((t, i) => `  pPointRx.trsf.${t} = ${AX[i]}`), '  movel(pPointRx,tCurrentTool,mCurrentSpeed)', 'end'].join('\r\n'), params(AX));
const MOVE_CIRC = pgx('MoveCirc', ['begin', ...TR.map((t, i) => `  pPointRx.trsf.${t} = ${AX[i]}`), ...TR.map((t, i) => `  pPointRx2.trsf.${t} = ${AX[i]}2`), '  movec(pPointRx,pPointRx2,tCurrentTool,mCurrentSpeed)', 'end'].join('\r\n'), params([...AX, ...AX.map((a) => `${a}2`)]));
const STOP = pgx('stop', 'begin\r\nend');
const INIT = ['close(tCurrentTool)', ...['pPointRx', 'pPointRx2'].flatMap((p) => [`${p}.config.shoulder = lefty`, `${p}.config.elbow = epositive`, `${p}.config.wrist = wpositive`]), 'mCurrentSpeed.tvel = 100.00', 'mCurrentSpeed.blend = joint', 'mCurrentSpeed.reach = 0.010', 'mCurrentSpeed.leave = 0.010'];

function dtx(frame: number[], tool: number[]): string {
  const v = (a: number[]) => TR.map((t, i) => `${t}="${fx(a[i] ?? 0)}"`).join(' ');
  const dio = (name: string, ch: string) => [`    <Data name="${name}" access="private" xsi:type="array" type="dio" size="16">`, ...Array.from({ length: 16 }, (_, i) => `      <Value key="${i}" link="BasicIO-1\\%${ch}${i}" />`), '    </Data>'];
  return [XML, '<Database xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="http://www.staubli.com/robotics/VAL3/Data/2">', '  <Datas>',
    '    <Data name="mNomSpeed" access="private" xsi:type="array" type="mdesc" size="1" />',
    '    <Data name="fFrame1" access="private" xsi:type="array" type="frame" size="1">', `      <Value key="0" ${v(frame)} fatherId="world[0]" />`, '    </Data>',
    ...dio('Input', 'I'), ...dio('Output', 'Q'),
    '    <Data name="tCurrentTool" access="private" xsi:type="array" type="tool" size="1">', `      <Value key="0" ${v(tool)} fatherId="flange[0]" ioLink="ValveIO/valve1" />`, '    </Data>',
    '    <Data name="mCurrentSpeed" access="private" xsi:type="array" type="mdesc" size="1" />',
    '    <Data name="pPointRx" access="private" xsi:type="array" type="pointRx" size="1">', '      <Value key="0" fatherId="fFrame1[0]" />', '    </Data>',
    '    <Data name="pPointRx2" access="private" xsi:type="array" type="pointRx" size="1">', '      <Value key="0" fatherId="fFrame1[0]" />', '    </Data>',
    '  </Datas>', '</Database>'].join('\r\n') + '\r\n';
}
function pjx(name: string, circ: boolean): string {
  return [XML, '<Project xmlns="http://www.staubli.com/robotics/VAL3/Project/3">', '<Parameters version="s7.5.3" stackSize="5000" millimeterUnit="true" />', '<Programs>',
    '<Program file="MoveJoint.pgx" />', '<Program file="MoveLine.pgx" />', ...(circ ? ['<Program file="MoveCirc.pgx" />'] : []), '<Program file="start.pgx" />', '<Program file="stop.pgx" />', '<Program file="main.pgx" />',
    '</Programs>', '<Database>', `<Data file="${name}.dtx" />`, '</Database>', '<Libraries />', '</Project>'].join('\r\n') + '\r\n';
}

type Conf = [string, string, string];
export class StaubliPost extends RoboPost {
  override PROG_EXT = 'pgx';
  protected parts: string[][] = [];
  protected nLines = 0;
  protected maxLines: number;
  protected frame = [0, 0, 0, 0, 0, 0];
  protected tool = [0, 0, 0, 0, 0, 0];
  protected circ = false;
  protected conf: Record<'pPointRx' | 'pPointRx2', Conf> = { pPointRx: ['lefty', 'epositive', 'wpositive'], pPointRx2: ['lefty', 'epositive', 'wpositive'] };
  constructor(ctx: RoboPostCtx) { super(ctx); this.maxLines = ctx.options.maxLines ?? 5000; }
  override addline(s: string): void { this.parts[this.parts.length - 1].push(`  ${s}`); this.nLines++; }
  override ProgStart(name: string): void {
    this.PROG_NAME = name;
    this.parts = [[]];
    this.nLines = 0;
    this.addline(`// Program ${name} start`);
  }
  /** RoboDK counts the move in the old part, then starts the new part with it (parts of max+1 moves after the first). */
  protected moveLine(s: string): void {
    this.nLines++;
    if (this.nLines > this.maxLines) { this.parts.push([]); this.nLines = 0; }
    this.parts[this.parts.length - 1].push(`  ${s}`);
  }
  override ProgFinish(): void { this.parts[this.parts.length - 1].push('  ', '  waitEndMove()', `  // Program ${this.PROG_NAME} end`, ''); }
  protected project(dir: string, name: string, lines: string[]): PostFile[] {
    const main = pgx('main', ['begin', ...INIT.map((l) => `  ${l}`), ...lines, '', 'end'].join('\r\n'));
    const files: PostFile[] = [
      { name: `${dir}MoveJoint.pgx`, content: MOVE_JOINT(Math.max(6, this.nAxes)) }, { name: `${dir}MoveLine.pgx`, content: MOVE_LINE },
      ...(this.circ ? [{ name: `${dir}MoveCirc.pgx`, content: MOVE_CIRC }] : []),
      { name: `${dir}${name}.dtx`, content: dtx(this.frame, this.tool) }, { name: `${dir}${name}.pjx`, content: pjx(name, this.circ) },
      { name: `${dir}main.pgx`, content: main },
    ];
    return files;
  }
  override ProgSave(progName: string): PostFile[] {
    const name = progName.slice(0, 13);
    if (this.parts.length === 1) {
      return [...this.project(`${name}/`, name, this.parts[0]), { name: `${name}/start.pgx`, content: pgx('start', 'begin\r\n  call main()\r\nend') }, { name: `${name}/stop.pgx`, content: STOP }];
    }
    const top = `Main${name}`.slice(0, 13 + 4);
    const partName = (i: number) => (i === 0 ? name : `${name}${i + 1}`);
    const files: PostFile[] = [];
    this.parts.forEach((lines, i) => files.push(...this.project(`${top}/${partName(i)}/`, partName(i), lines), { name: `${top}/${partName(i)}/start.pgx`, content: pgx('start', 'begin\r\n  call main()\r\nend') }, { name: `${top}/${partName(i)}/stop.pgx`, content: STOP }));
    const start: string[] = ['', 'begin'];
    this.parts.forEach((_l, i) => {
      start.push(`  if prog:libLoad("./${partName(i)}")!=0`, '    logMsg("Error Loading RoboDK library")', '    popUpMsg("Error Loading RoboDK library")', '  endIf', '  wait(taskStatus("loading")==-1)');
      if (i + 1 < this.parts.length) start.push(`  taskCreate "loading",10,loadNextOne("./${partName(i + 1)}")`);
      start.push('  call prog:start()', '  ');
    });
    start.push('', 'end', '      ');
    const P2 = `${XML.replace('?>', ' ?>')}`;
    files.push(
      { name: `${top}/${top}.dtx`, content: dtx(this.frame, this.tool) },
      { name: `${top}/${top}.pjx`, content: [P2, '<Project xmlns="http://www.staubli.com/robotics/VAL3/Project/3">', '  <Parameters version="s7.3.1" stackSize="5000" millimeterUnit="true" />', '  <Programs>', '    <Program file="loadNextOne.pgx" />', '    <Program file="start.pgx" />', '    <Program file="stop.pgx" />', '  </Programs>', '  <Database>', `    <Data file="${top}.dtx" />`, '  </Database>', '  <Libraries>', `    <Library alias="prog" path="./${name}/${name}.pjx" />`, `    <Library alias="prog_swap" path="./${name}/${name}.pjx" />`, '    <Library alias="tooldata" path="saveChangeTool" />', '  </Libraries>', '</Project>'].join('\r\n') },
      { name: `${top}/loadNextOne.pgx`, content: [P2, `<Programs ${PGX_NS}>`, '  <Program name="loadNextOne" access="private">', '    <Parameters xmlns="http://www.staubli.com/robotics/VAL3/Param/1">', '      <Parameter name="x_sName" type="string" xsi:type="element" />', '    </Parameters>', '    <Code><![CDATA[', 'begin', '  prog_swap:libLoad(x_sName)', 'end', '      ]]></Code>', '  </Program>', '</Programs>'].join('\r\n') },
      { name: `${top}/start.pgx`, content: [P2, `<Programs ${PGX_NS}>`, '  <Program name="start" access="public">', `    <Code><![CDATA[${start.join('\r\n')}]]></Code>`, '  </Program>', '</Programs>'].join('\r\n') + '\r\n' },
      { name: `${top}/stop.pgx`, content: STOP },
    );
    return files;
  }
  protected setConf(p: 'pPointRx' | 'pPointRx2', c: number[]): void {
    const want: Conf = [c[0] ? 'righty' : 'lefty', c[1] ? 'enegative' : 'epositive', c[2] ? 'wnegative' : 'wpositive'];
    (['shoulder', 'elbow', 'wrist'] as const).forEach((k, i) => { if (this.conf[p][i] !== want[i]) { this.conf[p][i] = want[i]; this.addline(`${p}.config.${k} = ${want[i]}`); } });
  }
  protected xyz(m: Mat4) { const v = pose2Staubli(m); return v.map((x) => fx(x)).join(','); }
  MoveJ(_pose: Mat4 | null, joints: number[]): void { this.moveLine(`call MoveJoint(${joints.slice(0, Math.max(6, this.nAxes)).map((v) => fx(v, 5)).join(',')})`); }
  MoveL(pose: Mat4 | null, joints: number[], conf: number[]): void {
    if (!pose) { this.addline('// Linear movement using joint targets is not supported. Change the target type to cartesian or use a joint movement.'); return; }
    this.setConf('pPointRx', conf);
    this.moveLine(`call MoveLine(${this.xyz(pose)})`);
  }
  override MoveC(pose1: Mat4 | null, joints1: number[], pose2: Mat4 | null, joints2: number[], conf1: number[], conf2: number[]): void {
    if (!pose1 || !pose2) { super.MoveC(pose1, joints1, pose2, joints2, conf1, conf2); return; }
    this.circ = true;
    this.setConf('pPointRx', conf1);
    this.setConf('pPointRx2', conf2);
    this.moveLine(`call MoveCirc(${this.xyz(pose1)},${this.xyz(pose2)})`);
  }
  override setFrame(pose: Mat4, _id: number, name: string): void {
    if (this.frame.some((v) => v !== 0) && this.parts.flat().some((l) => /call Move/.test(l))) this.addlog(`VAL3 project with one frame: moves before "${name}" are written relative to it`);
    this.frame = pose2Staubli(pose);
  }
  override setTool(pose: Mat4): void { this.tool = pose2Staubli(pose); }
  override Pause(ms: number): void { this.addline(ms < 0 ? 'popUpMsg("Paused. Press OK to continue")' : `delay(${fx(ms / 1000)})`); }
  override setSpeed(v: number): void { this.addline(`mCurrentSpeed.tvel = ${fx(v)}`); }
  override setZoneData(z: number): void { this.addline(`mCurrentSpeed.reach = ${fx(z)}`); this.addline(`mCurrentSpeed.leave = ${fx(z)}`); }
  override setDO(io: IoName, v: number | boolean | string): void { this.addline(`${typeof io === 'number' ? `Output[${io}]` : io}=${truthy(v) ? 'true' : 'false'}`); }
  override waitDI(io: IoName, v: number | boolean | string, timeoutMs = -1): void { this.addline(`watch(${typeof io === 'number' ? `Input[${io}]` : io}==${truthy(v) ? 'true' : 'false'}${timeoutMs > 0 ? `,${fx(timeoutMs, 1)}` : ''})`); }
  override RunCode(code: string, isCall = false): void {
    if (!isCall) { this.addline(code); return; }
    const c = code.trim();
    this.addline(`//call prog:${c}${c.includes('(') ? '' : '()'}`);
  }
  override RunMessage(msg: string, isComment = false): void { this.addline(isComment ? `// ${msg}` : `popUpMsg("${msg}")`); }
}

export const STAUBLI_VAL3: PostProcessor = {
  id: 'Staubli_VAL3', name: 'Stäubli VAL3 (project)', brand: 'Staubli', extension: 'pgx', aliases: ['Staubli_VAL3_simplified'],
  description: 'Stäubli VAL3 project (.pjx/.dtx/.pgx, RoboDK Staubli_VAL3 compatible).',
  generate(p: PostProgram, options: Record<string, any> = {}): PostFile[] { return runRoboPost((ctx) => new StaubliPost(ctx), p, options, false); },
};
registerPost(STAUBLI_VAL3);
