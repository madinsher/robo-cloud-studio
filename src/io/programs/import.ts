/**
 * Import robot programs written in vendor languages into a Program item:
 *  KUKA KRL (.src), ABB RAPID (.mod/.prg), Fanuc (.ls), URScript (.script), Generic CSV (our own export).
 * The parser is tolerant: unknown lines become comments / raw code.
 */
import { Program, MoveType } from '../../core/items/program';
import { Target, Frame, Item, Station } from '../../core/items/item';
import { kukaToPose, quatToPose, fanucToPose, urToPose, xyzrpwToPose, RAD, Mat4 } from '../../core/math/pose';
import { Robot } from '../../core/items/robot';

export type ProgramLanguage = 'krl' | 'rapid' | 'ls' | 'urscript' | 'csv' | 'auto';

export interface ImportResult {
  program: Program;
  targets: Target[];
  warnings: string[];
}

export function detectLanguage(text: string, filename = ''): ProgramLanguage {
  const ext = filename.split('.').pop()?.toLowerCase();
  if (ext === 'src' || /\bDEF\s+\w+\s*\(/.test(text) && /\bPTP\b|\bLIN\b/.test(text)) return 'krl';
  if (ext === 'mod' || ext === 'prg' || /\bMODULE\b/.test(text) && /\bENDMODULE\b/.test(text)) return 'rapid';
  if (ext === 'ls' || /^\/PROG/m.test(text)) return 'ls';
  if (ext === 'script' || /\bmovej\(|\bmovel\(/.test(text)) return 'urscript';
  if (ext === 'csv') return 'csv';
  return 'auto';
}

export function importProgram(text: string, opts: { station: Station; robot?: Robot | null; frame?: Item; name?: string; language?: ProgramLanguage; filename?: string }): ImportResult {
  const lang = !opts.language || opts.language === 'auto' ? detectLanguage(text, opts.filename) : opts.language;
  const frame = opts.frame ?? opts.robot?.activeFrame() ?? opts.station;
  const prog = new Program(opts.name ?? opts.filename?.replace(/\.[^.]+$/, '') ?? 'Imported');
  if (opts.robot) prog.setRobot(opts.robot);
  opts.station.addChild(prog);
  const targets: Target[] = [];
  const warnings: string[] = [];
  let n = 0;
  const addMove = (type: MoveType, pose: Mat4 | null, joints: number[] | null, name?: string) => {
    n++;
    const t = new Target(name ?? `${prog.name}_${n}`);
    if (pose) t.setPose(pose);
    if (joints) { t.joints = joints; if (!pose) t.isJointTarget = true; }
    if (opts.robot) t.robotId = opts.robot.id;
    frame.addChild(t);
    targets.push(t);
    if (type === 'MoveJ') prog.addMoveJ(t);
    else prog.addMoveL(t);
    return t;
  };

  switch (lang) {
    case 'krl': parseKRL(text, prog, addMove, warnings); break;
    case 'rapid': parseRAPID(text, prog, addMove, warnings); break;
    case 'ls': parseLS(text, prog, addMove, warnings); break;
    case 'urscript': parseURScript(text, prog, addMove, warnings); break;
    case 'csv': parseCSV(text, prog, addMove, warnings); break;
    default: warnings.push('Unknown language; imported as raw code'); text.split(/\r?\n/).forEach((l) => l.trim() && prog.runInstruction(l, false));
  }
  return { program: prog, targets, warnings };
}

type AddMove = (type: MoveType, pose: Mat4 | null, joints: number[] | null, name?: string) => Target;

/** Split on top-level commas (ignores commas inside brackets/parentheses/quotes). */
function splitArgs(s: string): string[] {
  const out: string[] = [];
  let depth = 0, cur = '', q = false;
  for (const ch of s) {
    if (ch === '"') q = !q;
    if (!q) {
      if (ch === '[' || ch === '(' || ch === '{') depth++;
      else if (ch === ']' || ch === ')' || ch === '}') depth--;
      else if (ch === ',' && depth === 0) { out.push(cur.trim()); cur = ''; continue; }
    }
    cur += ch;
  }
  if (cur.trim()) out.push(cur.trim());
  return out;
}

const num = (s: string) => parseFloat(s.replace(',', '.'));

function parseKRL(text: string, prog: Program, addMove: AddMove, warnings: string[]) {
  const frameRe = /\{\s*X\s*([-\d.eE+]+)\s*,\s*Y\s*([-\d.eE+]+)\s*,\s*Z\s*([-\d.eE+]+)\s*,\s*A\s*([-\d.eE+]+)\s*,\s*B\s*([-\d.eE+]+)\s*,\s*C\s*([-\d.eE+]+)/i;
  const axisRe = /\{\s*A1\s*([-\d.eE+]+)\s*,\s*A2\s*([-\d.eE+]+)\s*,\s*A3\s*([-\d.eE+]+)\s*,\s*A4\s*([-\d.eE+]+)\s*,\s*A5\s*([-\d.eE+]+)\s*,\s*A6\s*([-\d.eE+]+)/i;
  for (const raw of text.split(/\r?\n/)) {
    const line = raw.trim();
    if (!line || /^&|^DEF\b|^END\b|^;FOLD|^;ENDFOLD|^BAS\b|^DEFDAT|^ENDDAT/i.test(line)) continue;
    if (line.startsWith(';')) { prog.comment(line.slice(1).trim()); continue; }
    const cmt = line.split(';')[1]?.trim();
    const code = line.split(';')[0].trim();
    let m: RegExpMatchArray | null;
    if (/^PTP\b/i.test(code)) {
      if ((m = code.match(axisRe))) addMove('MoveJ', null, m.slice(1, 7).map(num), cmt);
      else if ((m = code.match(frameRe))) addMove('MoveJ', kukaToPose(...(m.slice(1, 7).map(num) as [number, number, number, number, number, number])), null, cmt);
      else prog.runInstruction(code, false);
    } else if (/^LIN\b/i.test(code) && (m = code.match(frameRe))) {
      addMove('MoveL', kukaToPose(...(m.slice(1, 7).map(num) as [number, number, number, number, number, number])), null, cmt);
    } else if (/^CIRC\b/i.test(code)) {
      const all = [...code.matchAll(new RegExp(frameRe.source, 'gi'))];
      if (all.length === 2) { const v0 = all[0].slice(1, 7).map(num), v1 = all[1].slice(1, 7).map(num); addMove('MoveL', kukaToPose(v0[0], v0[1], v0[2], v0[3], v0[4], v0[5]), null, (cmt ?? 'circ') + '_via'); addMove('MoveL', kukaToPose(v1[0], v1[1], v1[2], v1[3], v1[4], v1[5]), null, cmt); warnings.push('CIRC imported as two MoveL'); }
    } else if ((m = code.match(/^\$VEL\.CP\s*=\s*([-\d.]+)/i))) prog.setSpeed(num(m[1]) * 1000);
    else if ((m = code.match(/^WAIT SEC\s+([-\d.]+)/i))) prog.pause(num(m[1]) * 1000);
    else if ((m = code.match(/^\$OUT\[(\d+)\]\s*=\s*(TRUE|FALSE)/i))) prog.setDO(m[1], m[2].toUpperCase() === 'TRUE');
    else if ((m = code.match(/^WAIT FOR\s+\$IN\[(\d+)\]\s*==\s*(TRUE|FALSE)/i))) prog.waitDI(m[1], m[2].toUpperCase() === 'TRUE');
    else if ((m = code.match(/^\$APO\.CDIS\s*=\s*([-\d.]+)/i))) prog.setRounding(num(m[1]));
    else if (/^HALT\b/i.test(code)) prog.pause(-1);
    else if ((m = code.match(/^(\w+)\s*\(\s*\)$/))) prog.runInstruction(m[1], true);
    else if (code) prog.runInstruction(code, false);
  }
}

function parseRAPID(text: string, prog: Program, addMove: AddMove, warnings: string[]) {
  const rt = /\[\[\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*\]\s*,\s*\[\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*\]/;
  const jt = /\[\[\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*\]/;
  const consts = new Map<string, string>();
  for (const m of text.matchAll(/(?:CONST|PERS|VAR)\s+(robtarget|jointtarget)\s+(\w+)\s*:=\s*(\[[^;]+\]);/g)) consts.set(m[2], m[3]);
  const speedOf = (s: string) => { const m = s.match(/v(\d+)/); return m ? +m[1] : null; };
  for (const raw of text.split(/\r?\n/)) {
    const line = raw.trim();
    if (!line || /^MODULE|^ENDMODULE|^PROC|^ENDPROC|^CONST|^PERS|^VAR|^ConfL|^ConfJ/i.test(line)) continue;
    if (line.startsWith('!')) { prog.comment(line.slice(1).trim()); continue; }
    let m: RegExpMatchArray | null;
    if ((m = line.match(/^(MoveL|MoveJ|MoveC|MoveAbsJ)\s+(.+?);/i))) {
      const kind = m[1].toLowerCase();
      const args = m[2];
      const firstArg = splitArgs(args)[0];
      const data = firstArg.startsWith('[') ? firstArg : consts.get(firstArg) ?? '';
      const sp = speedOf(args);
      if (sp) prog.setSpeed(sp);
      const cmt = line.split('!')[1]?.trim() ?? (firstArg.startsWith('[') ? undefined : firstArg);
      if (kind === 'moveabsj') {
        const jm = data.match(jt);
        if (jm) addMove('MoveJ', null, jm.slice(1, 7).map(num), cmt);
      } else {
        if (kind === 'movec') { const parts = splitArgs(args); const viaData = parts[0].trim().startsWith('[') ? parts[0].trim() : consts.get(parts[0].trim()) ?? ''; const vm = viaData.match(rt); if (vm) addMove('MoveL', quatToPose(num(vm[1]), num(vm[2]), num(vm[3]), [num(vm[4]), num(vm[5]), num(vm[6]), num(vm[7])]), null, (cmt ?? 'c') + '_via'); const tData = parts[1].trim().startsWith('[') ? parts[1].trim() : consts.get(parts[1].trim()) ?? ''; const tm = tData.match(rt); if (tm) addMove('MoveL', quatToPose(num(tm[1]), num(tm[2]), num(tm[3]), [num(tm[4]), num(tm[5]), num(tm[6]), num(tm[7])]), null, cmt); warnings.push('MoveC imported as two MoveL'); continue; }
        const pm = data.match(rt);
        if (pm) addMove(kind === 'movej' ? 'MoveJ' : 'MoveL', quatToPose(num(pm[1]), num(pm[2]), num(pm[3]), [num(pm[4]), num(pm[5]), num(pm[6]), num(pm[7])]), null, cmt);
      }
    } else if ((m = line.match(/^WaitTime\s+([-\d.]+)/i))) prog.pause(num(m[1]) * 1000);
    else if ((m = line.match(/^SetDO\s+(\w+)\s*,\s*(\d)/i))) prog.setDO(m[1], m[2] === '1');
    else if ((m = line.match(/^WaitDI\s+(\w+)\s*,\s*(\d)/i))) prog.waitDI(m[1], m[2] === '1');
    else if ((m = line.match(/^TPWrite\s+"([^"]*)"/i))) prog.showMessage(m[1]);
    else if (/^Stop;/i.test(line)) prog.pause(-1);
    else if ((m = line.match(/^(\w+);$/))) prog.runInstruction(m[1], true);
    else prog.runInstruction(line.replace(/;$/, ''), false);
  }
}

function parseLS(text: string, prog: Program, addMove: AddMove, warnings: string[]) {
  const posSection = text.split(/^\/POS/m)[1] ?? '';
  const positions = new Map<number, { pose: Mat4 | null; joints: number[] | null; name?: string }>();
  for (const m of posSection.matchAll(/P\[(\d+)(?::"([^"]*)")?\]\s*\{([\s\S]*?)\};/g)) {
    const body = m[3];
    const g = (k: string) => { const r = body.match(new RegExp(`${k}\\s*=\\s*([-\\d.eE+]+)`)); return r ? num(r[1]) : null; };
    if (/J1\s*=/.test(body)) positions.set(+m[1], { pose: null, joints: ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'].map((k) => g(k) ?? 0), name: m[2] });
    else positions.set(+m[1], { pose: fanucToPose(g('X') ?? 0, g('Y') ?? 0, g('Z') ?? 0, g('W') ?? 0, g('P') ?? 0, g('R') ?? 0), joints: null, name: m[2] });
  }
  const main = text.split(/^\/MN/m)[1]?.split(/^\/POS/m)[0] ?? '';
  for (const raw of main.split(/\r?\n/)) {
    const line = raw.replace(/^\s*\d+:\s*/, '').replace(/;\s*$/, '').trim();
    if (!line) continue;
    let m: RegExpMatchArray | null;
    if ((m = line.match(/^([JLC])\s+P\[(\d+)(?::"[^"]*")?\]\s*([\d.]+)?\s*(%|mm\/sec|cm\/min)?\s*(FINE|CNT\d+)?/i))) {
      const p = positions.get(+m[2]);
      if (!p) { warnings.push(`P[${m[2]}] not found`); continue; }
      if (m[4] === 'mm/sec') prog.setSpeed(num(m[3]));
      if (m[5]) prog.setRounding(m[5].toUpperCase() === 'FINE' ? -1 : +m[5].slice(3));
      addMove(m[1].toUpperCase() === 'J' ? 'MoveJ' : 'MoveL', p.pose, p.joints, p.name);
    } else if ((m = line.match(/^WAIT\s+([\d.]+)\s*\(sec\)/i))) prog.pause(num(m[1]) * 1000);
    else if ((m = line.match(/^DO\[(\d+)\]\s*=\s*(ON|OFF)/i))) prog.setDO(m[1], m[2].toUpperCase() === 'ON');
    else if ((m = line.match(/^WAIT\s+DI\[(\d+)\]\s*=\s*(ON|OFF)/i))) prog.waitDI(m[1], m[2].toUpperCase() === 'ON');
    else if ((m = line.match(/^CALL\s+(\w+)/i))) prog.runInstruction(m[1], true);
    else if (line.startsWith('!')) prog.comment(line.slice(1).trim());
    else if (/^PAUSE/i.test(line)) prog.pause(-1);
    else if (!/^UFRAME_NUM|^UTOOL_NUM/i.test(line)) prog.runInstruction(line, false);
    else prog.comment(line);
  }
}

function parseURScript(text: string, prog: Program, addMove: AddMove, _warnings: string[]) {
  const p6 = /p\[\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*\]/;
  const j6 = /\[\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*\]/;
  for (const raw of text.split(/\r?\n/)) {
    const line = raw.trim();
    if (!line || /^def\b|^end\b/.test(line) || /^\w+\(\)\s*$/.test(line)) continue;
    const cmt = line.split('#')[1]?.trim();
    const code = line.split('#')[0].trim();
    if (!code) { if (cmt) prog.comment(cmt); continue; }
    let m: RegExpMatchArray | null;
    const vel = code.match(/v\s*=\s*([-\d.]+)/);
    if (vel) prog.setSpeed(num(vel[1]) * 1000);
    if ((m = code.match(/^movej\(/))) {
      const pm = code.match(p6);
      if (pm) addMove('MoveJ', urToPose(num(pm[1]) * 1000, num(pm[2]) * 1000, num(pm[3]) * 1000, num(pm[4]), num(pm[5]), num(pm[6])), null, cmt);
      else { const jm = code.match(j6); if (jm) addMove('MoveJ', null, jm.slice(1, 7).map((v) => num(v) * RAD), cmt); }
    } else if ((m = code.match(/^(movel|movep)\(/))) {
      const pm = code.match(p6);
      if (pm) addMove('MoveL', urToPose(num(pm[1]) * 1000, num(pm[2]) * 1000, num(pm[3]) * 1000, num(pm[4]), num(pm[5]), num(pm[6])), null, cmt);
    } else if ((m = code.match(/^movec\(/))) {
      const all = [...code.matchAll(new RegExp(p6.source, 'g'))];
      for (const a of all) addMove('MoveL', urToPose(num(a[1]) * 1000, num(a[2]) * 1000, num(a[3]) * 1000, num(a[4]), num(a[5]), num(a[6])), null, cmt);
    } else if ((m = code.match(/^sleep\(([-\d.]+)\)/))) prog.pause(num(m[1]) * 1000);
    else if ((m = code.match(/^set_(?:standard_)?digital_out\((\d+)\s*,\s*(True|False)\)/))) prog.setDO(m[1], m[2] === 'True');
    else if ((m = code.match(/^set_tcp\(/))) prog.comment(`set_tcp ${cmt ?? ''}`);
    else if ((m = code.match(/^textmsg\("([^"]*)"\)/))) prog.showMessage(m[1]);
    else if ((m = code.match(/^(\w+)\(\)$/))) prog.runInstruction(m[1], true);
    else prog.runInstruction(code, false);
  }
}

function parseCSV(text: string, prog: Program, addMove: AddMove, _warnings: string[]) {
  for (const raw of text.split(/\r?\n/)) {
    const line = raw.trim();
    if (!line || line.startsWith('#') || line.startsWith('type,')) continue;
    const c = line.split(',');
    if (c[0] === 'moveJ' || c[0] === 'moveL' || c[0] === 'moveC') {
      const pose = c.slice(2, 8).map(Number);
      const joints = c.slice(8).map(Number).filter((v) => !isNaN(v));
      addMove(c[0] === 'moveJ' ? 'MoveJ' : 'MoveL', pose.every((v) => !isNaN(v)) ? xyzrpwToPose(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]) : null, joints.length ? joints : null, c[1]);
    }
  }
}

export { Frame };
