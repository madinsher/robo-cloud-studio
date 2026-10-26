/**
 * Remote Webots PROTOs in the browser. Webots worlds name their standard robots and objects by URL
 * (EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/abb/irb/protos/Irb4600-40.proto")
 * and the Webots install ships no PROTO sources, so the importer needs them fetched. The importer asks for remote files
 * synchronously (ProtoRegistry `remote`), so the import runs in rounds: every round records the URLs it asked for, they
 * are downloaded (raw.githubusercontent.com allows cross-origin reads), and the import runs again until it asks for
 * nothing new. PROTOs, meshes and textures are cached for the session.
 */

export interface RemoteFetchStats { fetched: string[]; failed: string[]; rounds: number }

const cache = new Map<string, Uint8Array | null>();

/** Session cache of remote Webots files (tests / reloading). */
export function webotsRemoteCache(): Map<string, Uint8Array | null> { return cache; }

export interface RemoteRunOptions {
  fetchFn?: (url: string) => Promise<Uint8Array | null>;
  /** Most import rounds (each round resolves one more level of nested PROTOs). */
  maxRounds?: number;
  /** Most files downloaded per import. */
  maxFiles?: number;
  onProgress?: (detail: string) => void;
}

async function defaultFetch(url: string): Promise<Uint8Array | null> {
  if (typeof fetch !== 'function') return null;
  try {
    const ctl = typeof AbortController !== 'undefined' ? new AbortController() : null;
    const timer = ctl ? setTimeout(() => ctl.abort(), 20000) : null;
    const r = await fetch(url, ctl ? { signal: ctl.signal } : {});
    if (timer) clearTimeout(timer);
    return r.ok ? new Uint8Array(await r.arrayBuffer()) : null;
  } catch { return null; }
}

/**
 * Run `importOnce(remote)` until the remote files it asks for are all known. The last call is the one whose result is
 * returned; `final` (when given) is used for that last run instead of `importOnce` (e.g. with the real asset store).
 */
export async function withRemoteWebotsFiles<T>(importOnce: (remote: (url: string) => Uint8Array | undefined) => Promise<T>, opts: RemoteRunOptions = {}, final?: (remote: (url: string) => Uint8Array | undefined) => Promise<T>): Promise<{ result: T; stats: RemoteFetchStats }> {
  const fetchFn = opts.fetchFn ?? defaultFetch;
  const maxRounds = opts.maxRounds ?? 8, maxFiles = opts.maxFiles ?? 600;
  const stats: RemoteFetchStats = { fetched: [], failed: [], rounds: 0 };
  let asked = new Set<string>();
  const remote = (url: string) => {
    if (cache.has(url)) return cache.get(url) ?? undefined;
    asked.add(url);
    return undefined;
  };
  let result = await importOnce(remote);
  stats.rounds = 1;
  let total = 0;
  while (asked.size && stats.rounds < maxRounds && total < maxFiles) {
    const urls = [...asked].filter((u) => !cache.has(u)).slice(0, maxFiles - total);
    asked = new Set();
    if (!urls.length) break;
    opts.onProgress?.(`downloading ${urls.length} Webots file(s) (round ${stats.rounds})`);
    // a few downloads at a time
    for (let i = 0; i < urls.length; i += 8) {
      const part = urls.slice(i, i + 8);
      const got = await Promise.all(part.map((u) => fetchFn(u)));
      part.forEach((u, k) => { cache.set(u, got[k]); (got[k] ? stats.fetched : stats.failed).push(u); });
    }
    total += urls.length;
    stats.rounds++;
    result = await importOnce(remote);
  }
  if (final && (stats.fetched.length || stats.failed.length)) { asked = new Set(); result = await final(remote); }
  return { result, stats };
}
