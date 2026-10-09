/** Event queue for API clients (RoboDK EventsListen / WaitForEvent). */
export type StudioEvent = { type: string; itemId?: string; data?: any; t: number };

export class EventQueue {
  private queue: StudioEvent[] = [];
  private waiters: Array<(e: StudioEvent | null) => void> = [];
  private max = 500;
  push(e: Omit<StudioEvent, 't'>): void {
    const ev = { ...e, t: Date.now() };
    const w = this.waiters.shift();
    if (w) { w(ev); return; }
    this.queue.push(ev);
    if (this.queue.length > this.max) this.queue.shift();
  }
  /** Resolve with the next event, or null after the timeout (ms). */
  wait(timeoutMs = 3600_000): Promise<StudioEvent | null> {
    const next = this.queue.shift();
    if (next) return Promise.resolve(next);
    return new Promise((resolve) => {
      const w = (e: StudioEvent | null) => { clearTimeout(timer); resolve(e); };
      const timer = setTimeout(() => { const i = this.waiters.indexOf(w); if (i >= 0) this.waiters.splice(i, 1); resolve(null); }, timeoutMs);
      this.waiters.push(w);
    });
  }
  drain(): StudioEvent[] { const q = this.queue; this.queue = []; return q; }
  clear(): void { this.queue = []; }
}
