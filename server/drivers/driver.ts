/**
 * Robot driver interface (RoboDK "robot drivers"): live connection from the studio server to a controller.
 * Drivers translate the generic move/IO commands into vendor protocols and report joint state back.
 */
export interface DriverState {
  connected: boolean;
  joints: number[];
  status: string;
  error?: string;
}

export interface RobotDriver {
  readonly id: string;
  connect(ip: string, port?: number): Promise<void>;
  disconnect(): Promise<void>;
  state(): DriverState;
  /** Joint move (deg), blocking until reached when `wait` is true. */
  moveJ(joints: number[], speedDegS?: number, wait?: boolean): Promise<void>;
  /** Linear move to a pose (mm + rotation as 4x4 column-major), blocking when `wait` is true. */
  moveL(pose: number[], speedMmS?: number, wait?: boolean): Promise<void>;
  setDO(io: string, value: boolean | number): Promise<void>;
  getDI(io: string): Promise<boolean | number>;
  /** Send a raw vendor program/script text. */
  runScript(text: string): Promise<void>;
  stop(): Promise<void>;
}

export const drivers = new Map<string, () => RobotDriver>();
export function registerDriver(id: string, factory: () => RobotDriver): void { drivers.set(id, factory); }
export function createDriver(id: string): RobotDriver {
  const f = drivers.get(id);
  if (!f) throw new Error(`Unknown driver ${id}. Available: ${[...drivers.keys()].join(', ')}`);
  return f();
}
