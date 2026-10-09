import { PostProcessor, PostProgram, PostFile, f, safeName, registerPost } from './base';
import { poseToFanuc } from '../core/math/pose';

/** Fanuc R-30iA/iB (.LS) post processor. */
export const FANUC_R30iA: PostProcessor = {
  id: 'Fanuc_R30iA',
  name: 'Fanuc R-30iA/iB (LS)',
  brand: 'Fanuc',
  extension: 'ls',
  generate(p: PostProgram, options: Record<string, any> = {}): PostFile[] {
    const name = safeName(p.name).toUpperCase().slice(0, 36);
    const body: string[] = [];
    const pos: string[] = [];
    let n = 0;
    let speedL = 500;
    let speedJ = 50; // percent
    let term = 'FINE';
    let uframe = options.uframe ?? 1;
    let utool = options.utool ?? 1;
    const addPos = (kind: 'cart' | 'joint', ev: any): number => {
      n++;
      if (kind === 'joint') {
        const q: number[] = ev.joints;
        pos.push(`P[${n}:"${ev.name}"]{`, `   GP1:`, `\tUF : ${uframe}, UT : ${utool},`, `\tJ1=  ${f(q[0] ?? 0)} deg,\tJ2=  ${f(q[1] ?? 0)} deg,\tJ3=  ${f((q[2] ?? 0) + (q[1] ?? 0))} deg,`, `\tJ4=  ${f(q[3] ?? 0)} deg,\tJ5=  ${f(q[4] ?? 0)} deg,\tJ6=  ${f(q[5] ?? 0)} deg`, `};`);
      } else {
        const [x, y, z, w, pp, r] = poseToFanuc(ev.pose);
        const q = ev.joints ?? [0, 0, 0, 0, 0, 0];
        const conf = `'${q[4] >= 0 ? 'N' : 'F'} ${q[2] > -90 ? 'U' : 'D'} ${q[4] >= 0 ? 'T' : 'B'}', ${Math.floor(q[0] / 180)}, ${Math.floor(q[3] / 180)}, ${Math.floor(q[5] / 180)}`;
        pos.push(`P[${n}:"${ev.name}"]{`, `   GP1:`, `\tUF : ${uframe}, UT : ${utool},\t\tCONFIG : ${conf},`, `\tX =  ${f(x)}  mm,\tY =  ${f(y)}  mm,\tZ =  ${f(z)}  mm,`, `\tW =  ${f(w)} deg,\tP =  ${f(pp)} deg,\tR =  ${f(r)} deg`, `};`);
      }
      return n;
    };
    let line = 0;
    const L = (s: string) => body.push(`${String(++line).padStart(4)}:${s} ;`);
    L(`  UFRAME_NUM=${uframe}`);
    L(`  UTOOL_NUM=${utool}`);
    for (const ev of p.events) {
      switch (ev.kind) {
        case 'moveJ': L(`J P[${addPos('joint', ev)}] ${speedJ}% ${term}`); break;
        case 'moveL': L(`L P[${addPos('cart', ev)}] ${Math.round(ev.speed ?? speedL)}mm/sec ${term}`); break;
        case 'moveC': { const v = addPos('cart', { pose: ev.via, joints: ev.joints, name: ev.viaName }); const t = addPos('cart', ev); L(`C P[${v}] `); L(`    P[${t}] ${Math.round(speedL)}mm/sec ${term}`); break; }
        case 'setFrame': uframe = Math.min(9, uframe + 1); L(`  UFRAME_NUM=${uframe}`); L(`  ! frame ${ev.name}`); break;
        case 'setTool': utool = Math.min(10, utool + 1); L(`  UTOOL_NUM=${utool}`); L(`  ! tool ${ev.name}`); break;
        case 'setSpeed': if (ev.speedLinear !== undefined) speedL = ev.speedLinear; if (ev.speedJoints !== undefined) speedJ = Math.max(1, Math.min(100, Math.round(ev.speedJoints / 2))); break;
        case 'setRounding': term = ev.radius <= 0 ? 'FINE' : `CNT${Math.min(100, Math.round(ev.radius))}`; break;
        case 'pause': L(ev.timeMs < 0 ? `  PAUSE` : `  WAIT ${f(ev.timeMs / 1000, 2)}(sec)`); break;
        case 'setDO': L(`  DO[${ev.io.replace(/\D/g, '') || 1}]=${ev.value ? 'ON' : 'OFF'}`); break;
        case 'waitDI': L(`  WAIT DI[${ev.io.replace(/\D/g, '') || 1}]=${ev.value ? 'ON' : 'OFF'}`); break;
        case 'runCode': L(ev.isCall ? `  CALL ${safeName(ev.code).toUpperCase()}` : `  ${ev.code}`); break;
        case 'comment': L(`  ! ${ev.text.slice(0, 32)}`); break;
        case 'message': L(`  MESSAGE[${ev.text.slice(0, 24)}]`); break;
        case 'callProgram': L(`  CALL ${safeName(ev.name).toUpperCase()}`); break;
        case 'gripper': L(`  RO[1]=${ev.close ? 'ON' : 'OFF'}`); break;
        case 'navigate': L(`  ! NAV ${f(ev.x, 0)} ${f(ev.y, 0)}`); break;
        case 'task': L(`  ! TASK ${ev.task}`); break;
      }
    }
    const header = [`/PROG  ${name}`, `/ATTR`, `OWNER\t\t= MNEDITOR;`, `COMMENT\t\t= "VerticalBot Studio";`, `PROG_SIZE\t= 0;`, `CREATE\t\t= DATE ${new Date().toISOString().slice(2, 10).replace(/-/g, '-')}  TIME 00:00:00;`, `MODIFIED\t= DATE ${new Date().toISOString().slice(2, 10)}  TIME 00:00:00;`, `FILE_NAME\t= ;`, `VERSION\t\t= 0;`, `LINE_COUNT\t= ${line};`, `MEMORY_SIZE\t= 0;`, `PROTECT\t\t= READ_WRITE;`, `TCD:  STACK_SIZE\t= 0,`, `      TASK_PRIORITY\t= 50,`, `      TIME_SLICE\t= 0,`, `      BUSY_LAMP_OFF\t= 0,`, `      ABORT_REQUEST\t= 0,`, `      PAUSE_REQUEST\t= 0;`, `DEFAULT_GROUP\t= 1,*,*,*,*;`, `CONTROL_CODE\t= 00000000 00000000;`, `/MN`];
    const files: PostFile[] = [{ name: `${name}.LS`, content: [...header, ...body, `/POS`, ...pos, `/END`].join('\n') + '\n' }];
    for (const s of p.subprograms) files.push(...FANUC_R30iA.generate(s, options));
    return files;
  },
};
registerPost(FANUC_R30iA);
