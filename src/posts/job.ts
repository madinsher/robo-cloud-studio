/**
 * "Post job": the JSON form of a compiled PostProgram exchanged with python/post_shim.py (native RoboDK posts on the
 * server, Python posts in Pyodide) and stored as ground-truth input in tests/fixtures/posts_gt. Poses are 16
 * column-major numbers (studio Mat4 layout).
 */
import { fromArray } from '../core/math/pose';
import { PostEvent, PostProgram } from './base';
import { postProgName, renameEvents } from './robopost';

export interface PostJobProgram { name: string; events: any[] }
export interface PostJob extends PostJobProgram {
  program: string;
  robot: string;
  dof: number;
  axes_type?: string[];
  joint_names?: string[];
  frame: { name: string; pose: number[] };
  tool: { name: string; pose: number[] };
  subprograms: PostJobProgram[];
  header_comments?: string[];
  /** RoboDK post names to run on this job (fixture generator). */
  posts?: string[];
  /** Class attributes overridden on the RoboDK post instance (e.g. MAX_LINES_X_PROG). */
  post_attrs?: Record<string, any>;
  /** Maximum lines per program file (RoboDK option, posts split longer programs). */
  lines_x_prog?: number;
  include_sub_programs?: boolean;
}

const POSE_KEYS = ['pose', 'via'] as const;
export function eventToJSON(e: PostEvent): any {
  const o: any = { ...e };
  for (const k of POSE_KEYS) if (o[k] instanceof Float64Array) o[k] = Array.from(o[k]);
  // RoboDK passes pose None to its posts for a joint target
  if (o.jointTarget && (o.kind === 'moveJ' || o.kind === 'moveL')) o.pose = null;
  return o;
}
export function eventFromJSON(o: any): PostEvent {
  const e: any = { ...o };
  for (const k of POSE_KEYS) if (Array.isArray(e[k])) e[k] = fromArray(e[k].length === 4 ? (e[k] as number[][]).flatMap((_r, c) => [0, 1, 2, 3].map((r) => e[k][r][c])) : e[k]);
  return e as PostEvent;
}

/** Job for a RoboDK post: program names without blanks (as RoboDK passes them), calls renamed accordingly. */
export function toPostJob(p: PostProgram, extra: Partial<PostJob> = {}): PostJob {
  const name = postProgName(p.name);
  return {
    program: name, name, robot: p.robotName, dof: p.dof, axes_type: p.axesType, joint_names: p.jointNames,
    frame: { name: p.frame.name, pose: Array.from(p.frame.pose) }, tool: { name: p.tool.name, pose: Array.from(p.tool.pose) },
    events: renameEvents(p.events).map(eventToJSON), subprograms: p.subprograms.map((s) => ({ name: postProgName(s.name), events: renameEvents(s.events).map(eventToJSON) })), ...extra,
  };
}

/** Rebuild a PostProgram (without the Robot item) from a job, e.g. to replay fixtures through the built-in posts. */
export function fromPostJob(j: PostJob): PostProgram {
  const mk = (name: string, events: any[]): PostProgram => ({
    name, robot: null, robotName: j.robot, dof: j.dof, axesType: j.axes_type, jointNames: j.joint_names ?? [], events: events.map(eventFromJSON), subprograms: [],
    frame: { name: j.frame?.name ?? 'World', pose: fromArray(j.frame?.pose ?? [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]) },
    tool: { name: j.tool?.name ?? 'Tool', pose: fromArray(j.tool?.pose ?? [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]) }, problems: [],
  });
  const main = mk(j.program ?? j.name, j.events);
  main.subprograms = (j.subprograms ?? []).map((s) => mk(s.name, s.events));
  return main;
}
