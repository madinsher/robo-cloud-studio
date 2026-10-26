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








/** Kinova Kortex (Python API). */
registerPost(linePost({
  id: 'Kinova_Kortex', name: 'Kinova Kortex (Python)', brand: 'Kinova', extension: 'py',
  header: (p) => [`# ${p.name} — Kinova Kortex API`, `from kortex_api.autogen.messages import Base_pb2`, `def build_sequence(base):`, `    seq = Base_pb2.Sequence(); seq.name = "${p.name}"`], footer: () => [`    return seq`], indent: '    ',
  moveJ: (q, ev) => [`t = seq.tasks.add(); a = t.action; a.name = "${safeName(ev.name)}"; ra = a.reach_joint_angles.joint_angles`, ...j6(q).map((v, i) => `ja = ra.joint_angles.add(); ja.joint_identifier = ${i}; ja.value = ${v}`)],
  moveL: (m, ev, s) => { const [x, y, z, tx, ty, tz] = poseToXyzrpw(m); return [`t = seq.tasks.add(); a = t.action; a.name = "${safeName(ev.name)}"; rp = a.reach_pose; rp.constraint.speed.translation = ${f(s.speedL / 1000, 3)}`, `rp.target_pose.x = ${f(x / 1000, 4)}; rp.target_pose.y = ${f(y / 1000, 4)}; rp.target_pose.z = ${f(z / 1000, 4)}; rp.target_pose.theta_x = ${f(tx)}; rp.target_pose.theta_y = ${f(ty)}; rp.target_pose.theta_z = ${f(tz)}`]; },
  pause: (ms) => [`t = seq.tasks.add(); t.action.delay.duration = ${Math.round(Math.max(0, ms) / 1000)}`], setDO: (io, v) => [`# set DO ${io} = ${v}`], waitDI: (io, v) => [`# wait DI ${io} = ${v}`],
  comment: (t) => [`# ${t}`], gripper: (c) => [`t = seq.tasks.add(); g = t.action.send_gripper_command; g.mode = Base_pb2.GRIPPER_POSITION; fg = g.gripper.finger.add(); fg.finger_identifier = 1; fg.value = ${c ? '1.0' : '0.0'}`],
}));






export { registerPost };
