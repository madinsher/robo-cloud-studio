/**
 * Cooperative tasks on simulated time: the MURKA servers are sequential Python (send a command, sleep in sim time,
 * poll the joints / pose); here each skill is a generator that yields what it waits for and is resumed once per
 * simulation tick by the FSM action that runs it.
 *   yield            -> wait one tick
 *   yield <seconds>  -> wait until sim time has advanced by that much
 *   yield () => bool -> wait until the predicate holds
 */
export type Wait = undefined | number | (() => boolean);
export type Task<T> = Generator<Wait, T, void>;

export interface Clock { readonly time: number }

/** Drives one task: `tick()` resumes it when its wait condition holds; `done` / `value` after it returned. */
export class TaskRunner<T> {
  done = false; value: T | undefined; error: unknown = null;
  private wait: Wait = undefined; private until = -Infinity; private first = true;
  constructor(readonly task: Task<T>, readonly clock: Clock) {}
  tick(): boolean {
    if (this.done) return true;
    for (let guard = 0; guard < 10000; guard++) {
      if (!this.first) {
        if (typeof this.wait === 'number') { if (this.clock.time < this.until - 1e-9) return false; }
        else if (typeof this.wait === 'function') { if (!this.wait()) return false; }
        else if (guard > 0) return false; // plain yield: one tick
      }
      this.first = false;
      let r: IteratorResult<Wait, T>;
      try { r = this.task.next(); } catch (e) { this.error = e; this.done = true; return true; }
      if (r.done) { this.done = true; this.value = r.value; return true; }
      this.wait = r.value;
      if (typeof r.value === 'number') this.until = this.clock.time + r.value;
      if (r.value === undefined) return false;
    }
    return false;
  }
}

/** Sleep in sim time. */
export function* sleep(seconds: number): Task<void> { if (seconds > 0) yield seconds; }
/** Wait until `pred` holds or `timeout` s passed; true when it held. */
export function* waitUntil(clock: Clock, pred: () => boolean, timeout: number): Task<boolean> {
  const t1 = clock.time + timeout;
  while (!pred()) { if (clock.time >= t1) return false; yield; }
  return true;
}
