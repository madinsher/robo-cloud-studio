/** Additional vendor post processors. */
import { PostProcessor, PostProgram, PostFile, f, safeName, registerPost } from './base';
import { poseToXyzrpw, poseToKuka, poseToFanuc, poseToQuat, getPos, poseToUr, DEG } from '../core/math/pose';
import { poseToZyz } from './euler';

type Ev = Extract<PostProgram['events'][number], { name: string }>;

/** Helper to build simple line-oriented posts. */
function linePost(cfg: {
  id: string; name: string; brand: string; extension: string;
  header: (p: PostProgram) => string[]; footer: (p: PostProgram) => string[];
  moveJ: (q: number[], ev: Ev, s: State) => string[]; moveL: (m: Float64Array, ev: Ev, s: State, q: number[] | null) => string[]; moveC?: (via: Float64Array, m: Float64Array, ev: Ev, s: State) => string[];
  speed?: (s: State) => string[]; pause: (ms: number) => string[]; setDO: (io: string, v: boolean) => string[]; waitDI: (io: string, v: boolean) => string[]; comment: (t: string) => string[]; message?: (t: string) => string[]; call?: (n: string) => string[]; frame?: (m: Float64Array, name: string) => string[]; tool?: (m: Float64Array, name: string) => string[]; gripper?: (close: boolean) => string[];
  indent?: string;
}): PostProcessor {
  return {
    id: cfg.id, name: cfg.name, brand: cfg.brand, extension: cfg.extension,
    generate(p: PostProgram): PostFile[] {
      const s: State = { speedL: 250, speedJ: 50, rounding: 0, n: 0 };
      const L: string[] = [...cfg.header(p)];
      const ind = cfg.indent ?? '';
      const push = (lines: string[]) => { for (const l of lines) L.push(ind + l); };
      for (const ev of p.events) {
        switch (ev.kind) {
          case 'moveJ': s.n++; push(cfg.moveJ(ev.joints, ev as Ev, s)); break;
          case 'moveL': s.n++; push(cfg.moveL(ev.pose, ev as Ev, s, ev.joints)); break;
          case 'moveC': s.n++; push(cfg.moveC ? cfg.moveC(ev.via, ev.pose, ev as Ev, s) : [...cfg.moveL(ev.via, ev as Ev, s, null), ...cfg.moveL(ev.pose, ev as Ev, s, ev.joints)]); break;
          case 'setSpeed': if (ev.speedLinear !== undefined) s.speedL = ev.speedLinear; if (ev.speedJoints !== undefined) s.speedJ = Math.min(100, Math.round(ev.speedJoints / 2)); if (cfg.speed) push(cfg.speed(s)); break;
          case 'setRounding': s.rounding = Math.max(0, ev.radius); break;
          case 'pause': push(cfg.pause(ev.timeMs)); break;
          case 'setDO': push(cfg.setDO(ev.io, !!ev.value)); break;
          case 'waitDI': push(cfg.waitDI(ev.io, !!ev.value)); break;
          case 'runCode': push(ev.isCall && cfg.call ? cfg.call(ev.code) : [ev.code]); break;
          case 'comment': push(cfg.comment(ev.text)); break;
          case 'message': push(cfg.message ? cfg.message(ev.text) : cfg.comment(ev.text)); break;
          case 'callProgram': push(cfg.call ? cfg.call(ev.name) : cfg.comment(`call ${ev.name}`)); break;
          case 'gripper': push(cfg.gripper ? cfg.gripper(ev.close) : cfg.setDO('1', ev.close)); break;
          case 'setFrame': push(cfg.frame ? cfg.frame(ev.pose, ev.name) : cfg.comment(`frame ${ev.name}`)); break;
          case 'setTool': push(cfg.tool ? cfg.tool(ev.pose, ev.name) : cfg.comment(`tool ${ev.name}`)); break;
          case 'navigate': push(cfg.comment(`NAVIGATE ${f(ev.x)} ${f(ev.y)}`)); break;
          case 'task': push(cfg.comment(`TASK ${ev.task}`)); break;
        }
      }
      L.push(...cfg.footer(p));
      const files: PostFile[] = [{ name: `${safeName(p.name)}.${cfg.extension}`, content: L.join('\n') + '\n' }];
      for (const sub of p.subprograms) files.push(...this.generate(sub));
      return files;
    },
  };
}
interface State { speedL: number; speedJ: number; rounding: number; n: number }
const xyzrpw = (m: Float64Array, d = 3) => poseToXyzrpw(m).map((v) => f(v, d));
const kuka = (m: Float64Array) => poseToKuka(m).map((v) => f(v));
const j6 = (q: number[], d = 3) => [0, 1, 2, 3, 4, 5].map((i) => f(q[i] ?? 0, d));

/** Denso RC8 (PacScript). */
registerPost(linePost({
  id: 'Denso_RC8', name: 'Denso RC8 (PacScript)', brand: 'Denso', extension: 'pcs',
  header: (p) => [`'!TITLE "${p.name}"`, `Sub Main`, `  TakeArm Keep = 0`, `  Motor On`, `  ExtSpeed 100`],
  footer: () => [`  GiveArm`, `End Sub`], indent: '  ',
  moveJ: (q, ev) => [`Move P, J(${j6(q).join(', ')})  ' ${ev.name}`],
  moveL: (m, ev, s) => [`Move L, P(${[...xyzrpw(m)].join(', ')}, -1), S=${Math.round(s.speedL / 20)}  ' ${ev.name}`],
  moveC: (via, m) => [`Move C, P(${xyzrpw(via).join(', ')}, -1), P(${xyzrpw(m).join(', ')}, -1)`],
  pause: (ms) => [ms < 0 ? `Suspend` : `Delay ${Math.round(ms)}`], setDO: (io, v) => [`Set IO[${io.replace(/\D/g, '') || 24}] = ${v ? 'On' : 'Off'}`], waitDI: (io, v) => [`Wait IO[${io.replace(/\D/g, '') || 8}] = ${v ? 'On' : 'Off'}`],
  comment: (t) => [`' ${t}`], message: (t) => [`PrintMsg "${t}"`], call: (n) => [`Call ${safeName(n)}`], tool: (m, n) => [`Tool 1, P(${xyzrpw(m).join(', ')})  ' ${n}`], frame: (m, n) => [`Work 1, P(${xyzrpw(m).join(', ')})  ' ${n}`],
}));

/** Kawasaki AS language. */
registerPost(linePost({
  id: 'Kawasaki', name: 'Kawasaki (AS)', brand: 'Kawasaki', extension: 'as',
  header: (p) => [`.PROGRAM ${safeName(p.name).toLowerCase()}()`, `  SPEED 100 ALWAYS`, `  ACCURACY 1 ALWAYS`], footer: () => [`.END`], indent: '  ',
  moveJ: (q, ev) => [`JMOVE #[${j6(q).join(',')}]  ; ${ev.name}`],
  moveL: (m, ev, s) => { const [x, y, z, o, a, t] = poseToZyz(m); return [`SPEED ${f(s.speedL, 0)} MM/S`, `LMOVE TRANS(${f(x)},${f(y)},${f(z)},${f(o)},${f(a)},${f(t)})  ; ${ev.name}`]; },
  moveC: (via, m) => { const p = (mm: Float64Array) => { const [x, y, z, o, a, t] = poseToZyz(mm); return `TRANS(${f(x)},${f(y)},${f(z)},${f(o)},${f(a)},${f(t)})`; }; return [`C1MOVE ${p(via)}`, `C2MOVE ${p(m)}`]; },
  pause: (ms) => [ms < 0 ? `PAUSE` : `TWAIT ${f(ms / 1000, 2)}`], setDO: (io, v) => [`SIGNAL ${v ? '' : '-'}${io.replace(/\D/g, '') || 1}`], waitDI: (io, v) => [`SWAIT ${v ? '' : '-'}${(+io.replace(/\D/g, '') || 1) + 1000}`],
  comment: (t) => [`; ${t}`], message: (t) => [`PRINT "${t}"`], call: (n) => [`CALL ${safeName(n).toLowerCase()}`], tool: (m, n) => { const [x, y, z, o, a, t] = poseToZyz(m); return [`TOOL TRANS(${f(x)},${f(y)},${f(z)},${f(o)},${f(a)},${f(t)})  ; ${n}`]; }, frame: (m, n) => { const [x, y, z, o, a, t] = poseToZyz(m); return [`BASE TRANS(${f(x)},${f(y)},${f(z)},${f(o)},${f(a)},${f(t)})  ; ${n}`]; },
}));

/** Nachi AX/FD (SLIM-like). */
registerPost(linePost({
  id: 'Nachi_AX_FD', name: 'Nachi AX/FD (SLIM)', brand: 'Nachi', extension: 'prg',
  header: (p) => [`'${p.name} generated by VerticalBot Studio`, `PROGRAM ${safeName(p.name).toUpperCase()}`], footer: () => [`END`],
  moveJ: (q, ev, s) => [`MOVEX A=1,AC=0,SM=0,M1J,P,(${j6(q).join(',')}),S=${s.speedJ},H=1,MS  '${ev.name}`],
  moveL: (m, ev, s) => [`MOVEX A=1,AC=0,SM=0,M1X,L,(${kuka(m).join(',')}),R=5.0,H=1,MS,CONF=0000  '${ev.name} S=${f(s.speedL, 0)}`],
  pause: (ms) => [ms < 0 ? `STOP` : `DELAY ${f(ms / 1000, 2)}`], setDO: (io, v) => [`SET ${v ? 'O' : 'RESET O'}${io.replace(/\D/g, '') || 1}`], waitDI: (io, v) => [`WAITI I${io.replace(/\D/g, '') || 1}=${v ? 1 : 0}`],
  comment: (t) => [`'${t}`], call: (n) => [`CALLP ${safeName(n).toUpperCase()}`],
}));

/** Comau C5G (PDL2). */
registerPost(linePost({
  id: 'Comau_C5G', name: 'Comau C5G (PDL2)', brand: 'Comau', extension: 'pdl',
  header: (p) => [`PROGRAM ${safeName(p.name)}`, `VAR`, `  pnt0001j : JOINTPOS FOR ARM[1]`, `BEGIN`, `  $BASE := POS(0, 0, 0, 0, 0, 0, '')`, `  $SPD_OPT := SPD_LIN`, `  $LIN_SPD := 0.25`], footer: () => [`END ${''}`], indent: '  ',
  moveJ: (q, ev) => [`MOVE JOINT TO {${j6(q).join(', ')}}  -- ${ev.name}`],
  moveL: (m, ev, s) => { const [x, y, z, e1, e2, e3] = poseToZyz(m); return [`$LIN_SPD := ${f(s.speedL / 1000, 3)}`, `MOVE LINEAR TO POS(${f(x)}, ${f(y)}, ${f(z)}, ${f(e1)}, ${f(e2)}, ${f(e3)}, '')  -- ${ev.name}`]; },
  moveC: (via, m) => { const p = (mm: Float64Array) => { const [x, y, z, a, b, c] = poseToZyz(mm); return `POS(${f(x)}, ${f(y)}, ${f(z)}, ${f(a)}, ${f(b)}, ${f(c)}, '')`; }; return [`MOVE CIRCULAR TO ${p(m)} VIA ${p(via)}`]; },
  pause: (ms) => [ms < 0 ? `PAUSE` : `DELAY ${Math.round(ms)}`], setDO: (io, v) => [`$DOUT[${io.replace(/\D/g, '') || 1}] := ${v ? 'ON' : 'OFF'}`], waitDI: (io, v) => [`WAIT FOR $DIN[${io.replace(/\D/g, '') || 1}] = ${v ? 'ON' : 'OFF'}`],
  comment: (t) => [`-- ${t}`], message: (t) => [`WRITE LUN_CRT ('${t}', NL)`], call: (n) => [`${safeName(n)}`], tool: (m, n) => { const [x, y, z, a, b, c] = poseToZyz(m); return [`$TOOL := POS(${f(x)}, ${f(y)}, ${f(z)}, ${f(a)}, ${f(b)}, ${f(c)}, '')  -- ${n}`]; }, frame: (m, n) => { const [x, y, z, a, b, c] = poseToZyz(m); return [`$UFRAME := POS(${f(x)}, ${f(y)}, ${f(z)}, ${f(a)}, ${f(b)}, ${f(c)}, '')  -- ${n}`]; },
}));

/** Epson RC+ (SPEL+). */
registerPost(linePost({
  id: 'Epson_RC', name: 'Epson RC+ (SPEL+)', brand: 'Epson', extension: 'prg',
  header: (p) => [`Function ${safeName(p.name)}`, `  Motor On`, `  Power High`, `  Speed 50`, `  Accel 50, 50`, `  SpeedS 250`, `  AccelS 2000`], footer: () => [`Fend`], indent: '  ',
  moveJ: (q, ev) => [`Go JA(${j6(q).join(', ')})  ' ${ev.name}`],
  moveL: (m, ev, s) => { const [x, y, z, u, v, w] = poseToXyzrpw(m); return [`SpeedS ${f(s.speedL, 0)}`, `Move XY(${f(x)}, ${f(y)}, ${f(z)}, ${f(w)}, ${f(v)}, ${f(u)})  ' ${ev.name}`]; },
  moveC: (via, m) => { const p = (mm: Float64Array) => { const [x, y, z, u, v, w] = poseToXyzrpw(mm); return `XY(${f(x)}, ${f(y)}, ${f(z)}, ${f(w)}, ${f(v)}, ${f(u)})`; }; return [`Arc3 ${p(via)}, ${p(m)}`]; },
  pause: (ms) => [ms < 0 ? `Pause` : `Wait ${f(ms / 1000, 2)}`], setDO: (io, v) => [`${v ? 'On' : 'Off'} ${io.replace(/\D/g, '') || 1}`], waitDI: (io, v) => [`Wait Sw(${io.replace(/\D/g, '') || 1}) = ${v ? 'On' : 'Off'}`],
  comment: (t) => [`' ${t}`], message: (t) => [`Print "${t}"`], call: (n) => [`Call ${safeName(n)}`], tool: (m, n) => { const [x, y, z, u, v, w] = poseToXyzrpw(m); return [`TLSet 1, XY(${f(x)}, ${f(y)}, ${f(z)}, ${f(w)}, ${f(v)}, ${f(u)})  ' ${n}`, `Tool 1`]; }, frame: (m, n) => { const [x, y, z, u, v, w] = poseToXyzrpw(m); return [`Local 1, XY(${f(x)}, ${f(y)}, ${f(z)}, ${f(w)}, ${f(v)}, ${f(u)})  ' ${n}`]; },
}));

/** Techman / Omron TM (TMScript / Listen node). */
registerPost(linePost({
  id: 'Techman_TM', name: 'Techman / Omron TM (Script)', brand: 'Techman', extension: 'script',
  header: (p) => [`// ${p.name} — TMScript for the Listen node`, `float speed = 250`], footer: () => [`ScriptExit()`],
  moveJ: (q, ev, s) => [`PTP("JPP", ${j6(q).join(',')}, ${s.speedJ}, 200, 0, false)  // ${ev.name}`],
  moveL: (m, ev, s) => [`Line("CPP", ${xyzrpw(m).join(',')}, ${f(s.speedL, 0)}, 200, ${Math.round(s.rounding)}, false)  // ${ev.name}`],
  moveC: (via, m, _e, s) => [`Circle("CPP", ${xyzrpw(via).join(',')}, ${xyzrpw(m).join(',')}, ${f(s.speedL, 0)}, 200, 0, 0, false)`],
  pause: (ms) => [ms < 0 ? `Pause()` : `Sleep(${Math.round(Math.max(0, ms))})`], setDO: (io, v) => [`IO["ControlBox"].DO[${io.replace(/\D/g, '') || 0}] = ${v ? 1 : 0}`], waitDI: (io, v) => [`WaitFor(IO["ControlBox"].DI[${io.replace(/\D/g, '') || 0}] == ${v ? 1 : 0})`],
  comment: (t) => [`// ${t}`], message: (t) => [`Display("${t}")`], call: (n) => [`// call ${n}`], tool: (m, n) => [`ChangeTCP("${safeName(n)}")  // ${xyzrpw(m).join(',')}`], frame: (m, n) => [`ChangeBase("${safeName(n)}")  // ${xyzrpw(m).join(',')}`],
}));

/** Hanwha HCR (Rodi script, UR-like). */
registerPost(linePost({
  id: 'Hanwha_HCR', name: 'Hanwha HCR (Rodi)', brand: 'Hanwha', extension: 'py',
  header: (p) => [`# ${p.name} — Hanwha HCR`, `def ${safeName(p.name)}():`], footer: (p) => [`${safeName(p.name)}()`], indent: '    ',
  moveJ: (q, ev, s) => [`movej([${j6(q).map((v) => f(+v * DEG, 5)).join(', ')}], v=${f(s.speedJ * 0.03, 3)})  # ${ev.name}`],
  moveL: (m, ev, s) => { const p = poseToUr(m); return [`movel(p[${[p[0] / 1000, p[1] / 1000, p[2] / 1000, p[3], p[4], p[5]].map((v) => f(v, 5)).join(', ')}], v=${f(s.speedL / 1000, 3)})  # ${ev.name}`]; },
  pause: (ms) => [ms < 0 ? `pause()` : `sleep(${f(ms / 1000, 3)})`], setDO: (io, v) => [`set_digital_out(${io.replace(/\D/g, '') || 0}, ${v ? 'True' : 'False'})`], waitDI: (io, v) => [`wait_digital_in(${io.replace(/\D/g, '') || 0}, ${v ? 'True' : 'False'})`],
  comment: (t) => [`# ${t}`], message: (t) => [`print("${t}")`], call: (n) => [`${safeName(n)}()`], tool: (m, n) => { const p = poseToUr(m); return [`set_tcp(p[${[p[0] / 1000, p[1] / 1000, p[2] / 1000, p[3], p[4], p[5]].map((v) => f(v, 5)).join(', ')}])  # ${n}`]; },
}));

/** Kinova Kortex (Python API). */
registerPost(linePost({
  id: 'Kinova_Kortex', name: 'Kinova Kortex (Python)', brand: 'Kinova', extension: 'py',
  header: (p) => [`# ${p.name} — Kinova Kortex API`, `from kortex_api.autogen.messages import Base_pb2`, `def build_sequence(base):`, `    seq = Base_pb2.Sequence(); seq.name = "${p.name}"`], footer: () => [`    return seq`], indent: '    ',
  moveJ: (q, ev) => [`t = seq.tasks.add(); a = t.action; a.name = "${safeName(ev.name)}"; ra = a.reach_joint_angles.joint_angles`, ...j6(q).map((v, i) => `ja = ra.joint_angles.add(); ja.joint_identifier = ${i}; ja.value = ${v}`)],
  moveL: (m, ev, s) => { const [x, y, z, tx, ty, tz] = poseToXyzrpw(m); return [`t = seq.tasks.add(); a = t.action; a.name = "${safeName(ev.name)}"; rp = a.reach_pose; rp.constraint.speed.translation = ${f(s.speedL / 1000, 3)}`, `rp.target_pose.x = ${f(x / 1000, 4)}; rp.target_pose.y = ${f(y / 1000, 4)}; rp.target_pose.z = ${f(z / 1000, 4)}; rp.target_pose.theta_x = ${f(tx)}; rp.target_pose.theta_y = ${f(ty)}; rp.target_pose.theta_z = ${f(tz)}`]; },
  pause: (ms) => [`t = seq.tasks.add(); t.action.delay.duration = ${Math.round(Math.max(0, ms) / 1000)}`], setDO: (io, v) => [`# set DO ${io} = ${v}`], waitDI: (io, v) => [`# wait DI ${io} = ${v}`],
  comment: (t) => [`# ${t}`], gripper: (c) => [`t = seq.tasks.add(); g = t.action.send_gripper_command; g.mode = Base_pb2.GRIPPER_POSITION; fg = g.gripper.finger.add(); fg.finger_identifier = 1; fg.value = ${c ? '1.0' : '0.0'}`],
}));

/** Mitsubishi MELFA BASIC V. */
registerPost(linePost({
  id: 'Mitsubishi_Melfa', name: 'Mitsubishi MELFA BASIC V', brand: 'Mitsubishi', extension: 'prg',
  header: (p) => [`' ${p.name}`, `1 Servo On`, `2 Ovrd 50`, `3 Spd 250`], footer: () => [`999 End`],
  moveJ: (q, ev, s) => [`${100 + s.n * 2} Mov J${s.n}  ' ${ev.name}`, `${101 + s.n * 2} Dly 0`],
  moveL: (m, ev, s) => [`${100 + s.n * 2} Spd ${f(s.speedL, 0)}`, `${101 + s.n * 2} Mvs P${s.n}  ' ${ev.name} ${xyzrpw(m).join(',')}`],
  pause: (ms) => [ms < 0 ? `Hlt` : `Dly ${f(ms / 1000, 2)}`], setDO: (io, v) => [`M_Out(${io.replace(/\D/g, '') || 1})=${v ? 1 : 0}`], waitDI: (io, v) => [`Wait M_In(${io.replace(/\D/g, '') || 1})=${v ? 1 : 0}`],
  comment: (t) => [`' ${t}`], call: (n) => [`CallP "${safeName(n).toUpperCase()}"`], tool: (m, n) => [`Tool (${xyzrpw(m).join(',')})  ' ${n}`], frame: (m, n) => [`Base (${xyzrpw(m).join(',')})  ' ${n}`],
}));

/** KUKA KRC2 (KRL, older syntax with PTP/LIN and $VEL). */
registerPost(linePost({
  id: 'KUKA_KRC2', name: 'KUKA KRC2 (KRL)', brand: 'KUKA', extension: 'src',
  header: (p) => [`&ACCESS RVP`, `&REL 1`, `DEF ${safeName(p.name).toUpperCase()} ( )`, `;FOLD INI`, `  BAS (#INITMOV,0 )`, `;ENDFOLD (INI)`, `$VEL.CP = 0.25`, `$APO.CDIS = 5`, `$ADVANCE = 3`], footer: () => [`END`],
  moveJ: (q, ev) => [`PTP {A1 ${f(q[0])},A2 ${f(q[1])},A3 ${f(q[2])},A4 ${f(q[3])},A5 ${f(q[4])},A6 ${f(q[5])}} ; ${ev.name}`],
  moveL: (m, ev, s) => { const [x, y, z, a, b, c] = poseToKuka(m); return [`$VEL.CP = ${f(s.speedL / 1000, 3)}`, `LIN {X ${f(x)},Y ${f(y)},Z ${f(z)},A ${f(a)},B ${f(b)},C ${f(c)}}${s.rounding > 0 ? ' C_DIS' : ''} ; ${ev.name}`]; },
  moveC: (via, m) => { const p = (mm: Float64Array) => { const [x, y, z, a, b, c] = poseToKuka(mm); return `{X ${f(x)},Y ${f(y)},Z ${f(z)},A ${f(a)},B ${f(b)},C ${f(c)}}`; }; return [`CIRC ${p(via)}, ${p(m)}`]; },
  pause: (ms) => [ms < 0 ? `HALT` : `WAIT SEC ${f(ms / 1000, 2)}`], setDO: (io, v) => [`$OUT[${io.replace(/\D/g, '') || 1}] = ${v ? 'TRUE' : 'FALSE'}`], waitDI: (io, v) => [`WAIT FOR $IN[${io.replace(/\D/g, '') || 1}] == ${v ? 'TRUE' : 'FALSE'}`],
  comment: (t) => [`; ${t}`], call: (n) => [`${safeName(n).toUpperCase()}()`], tool: (m, n) => { const [x, y, z, a, b, c] = poseToKuka(m); return [`$TOOL = {FRAME: X ${f(x)},Y ${f(y)},Z ${f(z)},A ${f(a)},B ${f(b)},C ${f(c)}} ; ${n}`]; }, frame: (m, n) => { const [x, y, z, a, b, c] = poseToKuka(m); return [`$BASE = {FRAME: X ${f(x)},Y ${f(y)},Z ${f(z)},A ${f(a)},B ${f(b)},C ${f(c)}} ; ${n}`]; },
}));

/** AUBO (Lua script). */
registerPost(linePost({
  id: 'AUBO', name: 'AUBO (Lua)', brand: 'AUBO', extension: 'lua',
  header: (p) => [`-- ${p.name} — AUBO script`, `set_joint_maxvelc({1.5,1.5,1.5,1.5,1.5,1.5})`, `set_end_max_line_velc(0.25)`], footer: () => [],
  moveJ: (q, ev) => [`move_joint({${j6(q).map((v) => f(+v * DEG, 5)).join(',')}}, true)  -- ${ev.name}`],
  moveL: (m, ev, s) => { const [x, y, z, rx, ry, rz] = poseToXyzrpw(m); return [`set_end_max_line_velc(${f(s.speedL / 1000, 3)})`, `move_line_pose({${f(x / 1000, 5)},${f(y / 1000, 5)},${f(z / 1000, 5)}}, rpy2quaternion({${f(rx * DEG, 5)},${f(ry * DEG, 5)},${f(rz * DEG, 5)}}), true)  -- ${ev.name}`]; },
  pause: (ms) => [`sleep(${f(Math.max(0, ms) / 1000, 3)})`], setDO: (io, v) => [`set_robot_io_status(RobotIOType.User_DO, "U_DO_0${io.replace(/\D/g, '') || 0}", ${v ? 1 : 0})`], waitDI: (io, v) => [`while get_robot_io_status(RobotIOType.User_DI, "U_DI_0${io.replace(/\D/g, '') || 0}") ~= ${v ? 1 : 0} do sleep(0.01) end`],
  comment: (t) => [`-- ${t}`], message: (t) => [`print("${t}")`], call: (n) => [`${safeName(n)}()`],
}));

/** JAKA (Python SDK). */
registerPost(linePost({
  id: 'JAKA', name: 'JAKA (Python SDK)', brand: 'JAKA', extension: 'py',
  header: (p) => [`# ${p.name} — JAKA SDK`, `import jkrc`, `robot = jkrc.RC("192.168.1.100"); robot.login(); robot.power_on(); robot.enable_robot()`], footer: () => [`robot.logout()`],
  moveJ: (q, ev, s) => [`robot.joint_move([${j6(q).map((v) => f(+v * DEG, 5)).join(', ')}], 0, True, ${f(s.speedJ * 0.03, 3)})  # ${ev.name}`],
  moveL: (m, ev, s) => [`robot.linear_move([${xyzrpw(m).slice(0, 3).join(', ')}, ${poseToXyzrpw(m).slice(3).map((v) => f(v * DEG, 5)).join(', ')}], 0, True, ${f(s.speedL, 1)})  # ${ev.name}`],
  pause: (ms) => [`time.sleep(${f(Math.max(0, ms) / 1000, 3)})`], setDO: (io, v) => [`robot.set_digital_output(0, ${io.replace(/\D/g, '') || 0}, ${v ? 1 : 0})`], waitDI: (io, v) => [`while robot.get_digital_input(0, ${io.replace(/\D/g, '') || 0})[1] != ${v ? 1 : 0}: time.sleep(0.01)`],
  comment: (t) => [`# ${t}`], message: (t) => [`print("${t}")`], call: (n) => [`${safeName(n)}()`], tool: (m, n) => [`robot.set_tool_data(1, [${xyzrpw(m).slice(0, 3).join(', ')}, ${poseToXyzrpw(m).slice(3).map((v) => f(v * DEG, 5)).join(', ')}], "${safeName(n)}"); robot.set_tool_id(1)`],
}));

/** Elite EC (Lua/JBI-like script). */
registerPost(linePost({
  id: 'Elite_EC', name: 'Elite EC (Script)', brand: 'Elite', extension: 'jbi',
  header: (p) => [`NOP`, `'${p.name}`], footer: () => [`END`],
  moveJ: (q, ev, s) => [`MOVJ P[${j6(q).join(',')}] VJ=${s.speedJ}  '${ev.name}`],
  moveL: (m, ev, s) => [`MOVL P[${xyzrpw(m).join(',')}] V=${f(s.speedL, 0)}  '${ev.name}`],
  pause: (ms) => [`TIMER T=${f(Math.max(0, ms) / 1000, 2)}`], setDO: (io, v) => [`DOUT OT#(${io.replace(/\D/g, '') || 1}) ${v ? 'ON' : 'OFF'}`], waitDI: (io, v) => [`WAIT IN#(${io.replace(/\D/g, '') || 1})=${v ? 'ON' : 'OFF'}`],
  comment: (t) => [`'${t}`], call: (n) => [`CALL JOB:${safeName(n).toUpperCase()}`],
}));

/** Dobot (Python API for CR series). */
registerPost(linePost({
  id: 'Dobot_CR', name: 'Dobot CR (Python TCP API)', brand: 'Dobot', extension: 'py',
  header: (p) => [`# ${p.name} — Dobot CR TCP/IP API`, `from dobot_api import DobotApiDashboard, DobotApiMove`, `dash = DobotApiDashboard("192.168.5.1", 29999); move = DobotApiMove("192.168.5.1", 30003)`, `dash.EnableRobot()`], footer: () => [`move.Sync()`, `dash.DisableRobot()`],
  moveJ: (q, ev) => [`move.JointMovJ(${j6(q).join(', ')})  # ${ev.name}`],
  moveL: (m, ev) => [`move.MovL(${xyzrpw(m).join(', ')})  # ${ev.name}`],
  moveC: (via, m) => [`move.Arc(${xyzrpw(via).join(', ')}, ${xyzrpw(m).join(', ')})`],
  speed: (s) => [`dash.SpeedL(${Math.min(100, Math.round(s.speedL / 20))})`, `dash.SpeedJ(${s.speedJ})`],
  pause: (ms) => [`move.Sync(); time.sleep(${f(Math.max(0, ms) / 1000, 3)})`], setDO: (io, v) => [`dash.DO(${io.replace(/\D/g, '') || 1}, ${v ? 1 : 0})`], waitDI: (io, v) => [`while dash.DI(${io.replace(/\D/g, '') || 1}) != ${v ? 1 : 0}: time.sleep(0.01)`],
  comment: (t) => [`# ${t}`], message: (t) => [`print("${t}")`], call: (n) => [`${safeName(n)}()`], tool: (m, n) => [`dash.SetTool(1, ${xyzrpw(m).join(', ')})  # ${n}`, `dash.Tool(1)`], frame: (m, n) => [`dash.SetUser(1, ${xyzrpw(m).join(', ')})  # ${n}`, `dash.User(1)`],
}));

/** Fanuc RJ3 (older TP syntax). */
registerPost(linePost({
  id: 'Fanuc_RJ3', name: 'Fanuc RJ3/RJ3iB (LS)', brand: 'Fanuc', extension: 'ls',
  header: (p) => [`/PROG  ${safeName(p.name).toUpperCase().slice(0, 12)}`, `/ATTR`, `OWNER\t\t= MNEDITOR;`, `COMMENT\t\t= "VerticalBot";`, `/MN`], footer: () => [`/POS`, `/END`],
  moveJ: (q, ev, s) => [`  ${s.n}:J P[${s.n}] ${s.speedJ}% FINE ; ! ${ev.name} ${j6(q).join(' ')}`],
  moveL: (m, ev, s) => [`  ${s.n}:L P[${s.n}] ${f(s.speedL, 0)}mm/sec ${s.rounding > 0 ? `CNT${Math.min(100, Math.round(s.rounding))}` : 'FINE'} ; ! ${ev.name} ${poseToFanuc(m).map((v) => f(v)).join(' ')}`],
  pause: (ms) => [ms < 0 ? `  PAUSE ;` : `  WAIT ${f(ms / 1000, 2)}(sec) ;`], setDO: (io, v) => [`  DO[${io.replace(/\D/g, '') || 1}]=${v ? 'ON' : 'OFF'} ;`], waitDI: (io, v) => [`  WAIT DI[${io.replace(/\D/g, '') || 1}]=${v ? 'ON' : 'OFF'} ;`],
  comment: (t) => [`  ! ${t.slice(0, 32)} ;`], call: (n) => [`  CALL ${safeName(n).toUpperCase()} ;`],
}));

/** ABB S4C (RAPID for legacy controllers). */
registerPost(linePost({
  id: 'ABB_RAPID_S4C', name: 'ABB RAPID (S4C legacy)', brand: 'ABB', extension: 'prg',
  header: (p) => [`%%%`, `  VERSION:1`, `  LANGUAGE:ENGLISH`, `%%%`, `MODULE ${safeName(p.name)}`, `  PERS tooldata tTool:=[TRUE,[[0,0,0],[1,0,0,0]],[1,[0,0,50],[1,0,0,0],0,0,0]];`, `  PROC main()`], footer: () => [`  ENDPROC`, `ENDMODULE`], indent: '    ',
  moveJ: (q, ev) => [`MoveAbsJ [[${j6(q).join(',')}],[9E9,9E9,9E9,9E9,9E9,9E9]],v500,fine,tTool;  ! ${ev.name}`],
  moveL: (m, ev, s) => { const [x, y, z] = getPos(m); const [w, a, b, c] = poseToQuat(m); return [`MoveL [[${f(x)},${f(y)},${f(z)}],[${f(w, 6)},${f(a, 6)},${f(b, 6)},${f(c, 6)}],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],v${Math.round(s.speedL)},${s.rounding > 0 ? `z${Math.min(200, Math.round(s.rounding))}` : 'fine'},tTool;  ! ${ev.name}`]; },
  pause: (ms) => [ms < 0 ? `Stop;` : `WaitTime ${f(ms / 1000, 2)};`], setDO: (io, v) => [`SetDO ${safeName(io)},${v ? 1 : 0};`], waitDI: (io, v) => [`WaitDI ${safeName(io)},${v ? 1 : 0};`],
  comment: (t) => [`! ${t}`], message: (t) => [`TPWrite "${t}";`], call: (n) => [`${safeName(n)};`],
}));

export { registerPost };
