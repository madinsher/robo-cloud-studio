/**
 * Bit-exact ports of the CPython / NumPy numerics the MURKA agromarca experiments depend on, so the studio's
 * reproduction of `murca_experiment.article_experiments` gives the same numbers as the Python ground truth:
 *
 *  - `PyRandom`: CPython's `random.Random` (MT19937, `init_by_array` seeding of int seeds, `random()` with 53-bit
 *    resolution, `uniform`, `randint` / `randrange` via `_randbelow_with_getrandbits`, `shuffle`, `sample`);
 *  - `pyHypot`: `math.hypot` of CPython ≥ 3.10 (`vector_norm`: power-of-two scaling, exact squaring, compensated sum and
 *    a differential square-root correction) — 3.7's libm `hypot` differs by 1 ulp on some inputs;
 *  - `pyMean`: `statistics.mean` (exact rational sum, correctly rounded division);
 *  - `pyPstdev`: `statistics.pstdev` of CPython ≥ 3.12 (exact variance, correctly rounded square root);
 *  - `npStd`: `numpy.std` (float64 pairwise summation, ddof 0).
 *
 * Python's set iteration order is NOT reproduced here: it depends on PYTHONHASHSEED; callers pass the order explicitly.
 */

// ------------------------------------------------------------------------------------------------------------------
// MT19937 as in CPython Modules/_randommodule.c
// ------------------------------------------------------------------------------------------------------------------

const N = 624, M = 397, MATRIX_A = 0x9908b0df, UPPER = 0x80000000, LOWER = 0x7fffffff;

export class PyRandom {
  private readonly mt = new Uint32Array(N);
  private mti = N + 1;

  constructor(seed: number | bigint = 0) { this.seed(seed); }

  /** `random.seed(int)`: the absolute value split into 32-bit words (little endian) feeds `init_by_array`. */
  seed(seed: number | bigint): void {
    if (typeof seed === 'number' && !Number.isInteger(seed)) throw new Error('PyRandom: only integer seeds are supported');
    let n = BigInt(seed);
    if (n < 0n) n = -n;
    const key: number[] = [];
    if (n === 0n) key.push(0);
    while (n > 0n) { key.push(Number(n & 0xffffffffn)); n >>= 32n; }
    this.initByArray(key);
  }

  private initGenrand(s: number): void {
    const mt = this.mt;
    mt[0] = s >>> 0;
    for (let i = 1; i < N; i++) {
      const prev = mt[i - 1] ^ (mt[i - 1] >>> 30);
      mt[i] = (Math.imul(1812433253, prev) + i) >>> 0;
    }
    this.mti = N;
  }

  private initByArray(key: number[]): void {
    const mt = this.mt;
    this.initGenrand(19650218);
    let i = 1, j = 0;
    for (let k = Math.max(N, key.length); k > 0; k--) {
      const prev = mt[i - 1] ^ (mt[i - 1] >>> 30);
      mt[i] = (((mt[i] ^ Math.imul(prev, 1664525)) >>> 0) + key[j] + j) >>> 0;
      i++; j++;
      if (i >= N) { mt[0] = mt[N - 1]; i = 1; }
      if (j >= key.length) j = 0;
    }
    for (let k = N - 1; k > 0; k--) {
      const prev = mt[i - 1] ^ (mt[i - 1] >>> 30);
      mt[i] = (((mt[i] ^ Math.imul(prev, 1566083941)) >>> 0) - i) >>> 0;
      i++;
      if (i >= N) { mt[0] = mt[N - 1]; i = 1; }
    }
    mt[0] = 0x80000000;
    this.mti = N;
  }

  /** genrand_uint32. */
  uint32(): number {
    const mt = this.mt;
    if (this.mti >= N) {
      let kk = 0, y: number;
      for (; kk < N - M; kk++) { y = (mt[kk] & UPPER) | (mt[kk + 1] & LOWER); mt[kk] = mt[kk + M] ^ (y >>> 1) ^ (y & 1 ? MATRIX_A : 0); }
      for (; kk < N - 1; kk++) { y = (mt[kk] & UPPER) | (mt[kk + 1] & LOWER); mt[kk] = mt[kk + (M - N)] ^ (y >>> 1) ^ (y & 1 ? MATRIX_A : 0); }
      y = (mt[N - 1] & UPPER) | (mt[0] & LOWER);
      mt[N - 1] = mt[M - 1] ^ (y >>> 1) ^ (y & 1 ? MATRIX_A : 0);
      this.mti = 0;
    }
    let y = mt[this.mti++];
    y ^= y >>> 11;
    y ^= (y << 7) & 0x9d2c5680;
    y ^= (y << 15) & 0xefc60000;
    y ^= y >>> 18;
    return y >>> 0;
  }

  /** `random()`: [0, 1) with 53-bit resolution. */
  random(): number {
    const a = this.uint32() >>> 5, b = this.uint32() >>> 6;
    return (a * 67108864.0 + b) * (1.0 / 9007199254740992.0);
  }

  /** `getrandbits(k)` for 0 ≤ k ≤ 32. */
  getrandbits(k: number): number {
    if (k < 0 || k > 32) throw new Error('PyRandom.getrandbits: 0..32 bits only');
    if (k === 0) return 0;
    return this.uint32() >>> (32 - k);
  }

  /** `_randbelow_with_getrandbits(n)`: uniform int in [0, n), rejection sampling on bit_length(n) bits. */
  randbelow(n: number): number {
    if (!(n >= 0) || !Number.isInteger(n) || n > 0xffffffff) throw new Error(`PyRandom.randbelow: bad bound ${n}`);
    if (n === 0) return 0;
    const k = 32 - Math.clz32(n);
    let r = this.getrandbits(k);
    while (r >= n) r = this.getrandbits(k);
    return r;
  }

  /** `uniform(a, b)` = a + (b − a)·random(). */
  uniform(a: number, b: number): number { return a + (b - a) * this.random(); }

  /** `randrange(start, stop)` (step 1). */
  randrange(start: number, stop: number): number {
    const width = stop - start;
    if (width <= 0) throw new Error(`PyRandom.randrange: empty range (${start}, ${stop})`);
    return start + this.randbelow(width);
  }

  /** `randint(a, b)` = randrange(a, b + 1). */
  randint(a: number, b: number): number { return this.randrange(a, b + 1); }

  /** `shuffle(x)` in place (Fisher–Yates from the end). */
  shuffle<T>(x: T[]): T[] {
    for (let i = x.length - 1; i >= 1; i--) {
      const j = this.randbelow(i + 1);
      const t = x[i]; x[i] = x[j]; x[j] = t;
    }
    return x;
  }

  /** `sample(population, k)`; only the small-population branch (n ≤ 21 + table size) used by the experiments. */
  sample<T>(population: readonly T[], k: number): T[] {
    const n = population.length;
    if (!(k >= 0 && k <= n)) throw new Error('Sample larger than population or is negative');
    let setsize = 21;
    if (k > 5) setsize += 4 ** Math.ceil(Math.log(k * 3) / Math.log(4));
    if (n > setsize) throw new Error('PyRandom.sample: the large-population (set) branch is not ported');
    const pool = [...population];
    const result: T[] = new Array(k);
    for (let i = 0; i < k; i++) {
      const j = this.randbelow(n - i);
      result[i] = pool[j];
      pool[j] = pool[n - i - 1];
    }
    return result;
  }

  /** `choice(seq)`. */
  choice<T>(seq: readonly T[]): T {
    if (!seq.length) throw new Error('Cannot choose from an empty sequence');
    return seq[this.randbelow(seq.length)];
  }

  /** `gauss`-free normal deviate as `random.normalvariate` (Kinderman–Monahan), CPython formula. */
  normalvariate(mu: number, sigma: number): number {
    const NV_MAGICCONST = 4 * Math.exp(-0.5) / Math.sqrt(2.0);
    let z: number;
    for (;;) {
      const u1 = this.random();
      const u2 = 1.0 - this.random();
      z = NV_MAGICCONST * (u1 - 0.5) / u2;
      const zz = z * z / 4.0;
      if (zz <= -Math.log(u2)) break;
    }
    return mu + z * sigma;
  }
}

// ------------------------------------------------------------------------------------------------------------------
// math.hypot (CPython >= 3.10 vector_norm, fma variant of dl_mul)
// ------------------------------------------------------------------------------------------------------------------

const SPLIT = 134217729.0; // 2^27 + 1

/** Exact product x·y = hi + lo (Dekker TwoProduct ≡ fma(x, y, −hi)). */
function twoProduct(x: number, y: number): [number, number] {
  const p = x * y;
  let c = SPLIT * x; const xh = c - (c - x), xl = x - xh;
  c = SPLIT * y; const yh = c - (c - y), yl = y - yh;
  const err = xl * yl - (((p - xh * yh) - xl * yh) - xh * yl);
  return [p, err];
}

const f64 = new Float64Array(1);
const u32 = new Uint32Array(f64.buffer);
const LITTLE = new Uint8Array(new Uint16Array([1]).buffer)[0] === 1;
/** frexp exponent: x = m·2^e with 0.5 ≤ |m| < 1 (x finite, non-zero). */
function frexpExp(x: number): number {
  f64[0] = x;
  const hi = u32[LITTLE ? 1 : 0];
  const be = (hi >>> 20) & 0x7ff;
  if (be === 0) return frexpExp(x * 2 ** 64) - 64; // subnormal
  return be - 1022;
}

/** `math.hypot(*coords)` exactly as CPython ≥ 3.10 (verified against 3.12 on Linux). */
export function pyHypot(...coords: number[]): number {
  let max = 0, nan = false;
  const v = coords.map((c) => { const a = Math.abs(c); if (Number.isNaN(a)) nan = true; else if (a > max) max = a; return a; });
  if (max === Infinity) return Infinity;
  if (nan) return NaN;
  if (max === 0 || v.length <= 1) return max;
  const e = frexpExp(max);
  if (e < -1023) {
    const DBL_MIN = 2.2250738585072014e-308;
    return DBL_MIN * pyHypot(...v.map((x) => x / DBL_MIN));
  }
  const scale = 2 ** -e;
  let csum = 1.0, frac1 = 0.0, frac2 = 0.0;
  for (const c of v) {
    const x = c * scale;
    const [hi, lo] = twoProduct(x, x);
    const s = csum + hi; const z = s - csum; const sl = hi - z; // dl_fast_sum(csum, hi)
    csum = s; frac1 += lo; frac2 += sl;
  }
  let h = Math.sqrt(csum - 1.0 + (frac1 + frac2));
  const [phi, plo] = twoProduct(-h, h);
  const s = csum + phi; const z = s - csum; const sl = phi - z;
  csum = s; frac1 += plo; frac2 += sl;
  const x = csum - 1.0 + (frac1 + frac2);
  h += x / (2.0 * h);
  return h / scale;
}

// ------------------------------------------------------------------------------------------------------------------
// Exact rational helpers (BigInt) for statistics.mean / pstdev
// ------------------------------------------------------------------------------------------------------------------

const SHIFT = 1074n;

/** Finite double → integer numerator over 2^1074 (exact). */
function scaled(x: number): bigint {
  if (!Number.isFinite(x)) throw new Error(`statistics: non-finite value ${x}`);
  if (x === 0) return 0n;
  f64[0] = x;
  const hi = u32[LITTLE ? 1 : 0], lo = u32[LITTLE ? 0 : 1];
  const neg = hi >>> 31 === 1;
  const be = (hi >>> 20) & 0x7ff;
  let mant = (BigInt(hi & 0xfffff) << 32n) | BigInt(lo);
  let exp: number;
  if (be === 0) exp = -1074; else { mant |= 1n << 52n; exp = be - 1075; }
  const v = mant << BigInt(exp + 1074);
  return neg ? -v : v;
}

const bitLength = (n: bigint): number => (n === 0n ? 0 : n.toString(2).length);

/** Correctly rounded (half-even) p / q → double, q > 0. */
export function ratioToDouble(p: bigint, q: bigint): number {
  if (q <= 0n) throw new Error('ratioToDouble: q must be positive');
  if (p === 0n) return 0;
  const neg = p < 0n; if (neg) p = -p;
  let k = 53 - (bitLength(p) - bitLength(q));
  const quot = (kk: number) => (kk >= 0 ? [(p << BigInt(kk)) / q, (p << BigInt(kk)) % q, q] : [p / (q << BigInt(-kk)), p % (q << BigInt(-kk)), q << BigInt(-kk)]) as [bigint, bigint, bigint];
  let [Q, R, D] = quot(k);
  if (Q >= 1n << 53n) { k--; [Q, R, D] = quot(k); }
  else if (Q < 1n << 52n) { k++; [Q, R, D] = quot(k); }
  if (k > 1074) { k = 1074; [Q, R, D] = quot(k); } // subnormal result: fixed exponent
  const twice = 2n * R;
  if (twice > D || (twice === D && (Q & 1n) === 1n)) Q += 1n;
  let val: number;
  if (k > 1000) val = Number(Q) * 2 ** -1000 * 2 ** -(k - 1000);
  else if (k < -900) val = Number(Q) * 2 ** 900 * 2 ** (-k - 900);
  else val = Number(Q) * 2 ** -k;
  return neg ? -val : val;
}

/** `statistics.mean(data)` (exact sum, correctly rounded). */
export function pyMean(data: Iterable<number>): number {
  const xs = [...data];
  if (!xs.length) throw new Error('mean requires at least one data point');
  let s = 0n;
  for (const x of xs) s += scaled(x);
  return ratioToDouble(s, BigInt(xs.length) << SHIFT);
}

function isqrt(n: bigint): bigint {
  if (n < 0n) throw new Error('isqrt of a negative number');
  if (n < 2n) return n;
  let x = 1n << BigInt(Math.ceil(bitLength(n) / 2));
  for (;;) { const y = (x + n / x) >> 1n; if (y >= x) return x; x = y; }
}

/** `statistics.pstdev(data)` as CPython ≥ 3.12 (`_float_sqrt_of_frac` of the exact population variance). */
export function pyPstdev(data: Iterable<number>): number {
  const xs = [...data];
  const n = BigInt(xs.length);
  if (!xs.length) throw new Error('pstdev requires at least one data point');
  let sx = 0n, sxx = 0n;
  for (const x of xs) { const v = scaled(x); sx += v; sxx += v * v; }
  // mss = (n·Σx² − (Σx)²) / n² over the common denominator 2^(2·1074)
  const num = n * sxx - sx * sx;
  const den = n * n << (2n * SHIFT);
  if (num === 0n) return 0;
  const q = Math.floor((bitLength(num) - bitLength(den) - 109) / 2);
  const rto = (a: bigint, b: bigint) => { const r = isqrt(a / b); return r * r * b !== a ? r | 1n : r; };
  if (q >= 0) return ratioToDouble(rto(num, den << BigInt(2 * q)) << BigInt(q), 1n);
  return ratioToDouble(rto(num << BigInt(-2 * q), den), 1n << BigInt(-q));
}

// ------------------------------------------------------------------------------------------------------------------
// numpy float64 reductions
// ------------------------------------------------------------------------------------------------------------------

/** numpy's pairwise_sum for float64 (block 128, 8-way unrolled). */
export function npSum(a: readonly number[], off = 0, n = a.length): number {
  if (n < 8) { let res = 0.0; for (let i = 0; i < n; i++) res += a[off + i]; return res; }
  if (n <= 128) {
    const r = [0, 1, 2, 3, 4, 5, 6, 7].map((j) => a[off + j]);
    let i = 8;
    for (; i < n - (n % 8); i += 8) for (let j = 0; j < 8; j++) r[j] += a[off + i + j];
    let res = ((r[0] + r[1]) + (r[2] + r[3])) + ((r[4] + r[5]) + (r[6] + r[7]));
    for (; i < n; i++) res += a[off + i];
    return res;
  }
  let n2 = Math.floor(n / 2); n2 -= n2 % 8;
  return npSum(a, off, n2) + npSum(a, off + n2, n - n2);
}

/** `float(numpy.std(values))` (ddof = 0). */
export function npStd(values: readonly number[]): number {
  const n = values.length;
  if (!n) return NaN;
  const mean = npSum(values) / n;
  const sq = values.map((x) => { const d = x - mean; return d * d; });
  return Math.sqrt(npSum(sq) / n);
}
