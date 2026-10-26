"""Reference vectors for the studio's CPython/NumPy/SciPy ports (src/murka/agro/pycompat.ts, src/fleet/lsap.ts)."""
import json, math, random, statistics, sys
import numpy as np
from scipy.optimize import linear_sum_assignment
out = {'python': sys.version.split()[0], 'numpy': np.__version__}
import scipy; out['scipy'] = scipy.__version__
# PRNG streams
rs = {}
for seed in [0, 1, 7, 97, 12345, 2**32 + 5, 2**70 + 3]:
    r = random.Random(seed)
    rs[str(seed)] = {
        'random': [r.random() for _ in range(5)],
        'uniform': [r.uniform(-2, 3) for _ in range(3)],
        'randint': [r.randint(0, 3) for _ in range(8)],
        'shuffle': (lambda l: (r.shuffle(l), l)[1])(list(range(10))),
        'sample': r.sample(['a', 'b', 'c', 'd', 'e'], k=3),
        'randbelow_big': [r.randrange(0, 1000003) for _ in range(4)],
    }
out['random'] = rs
# hypot
g = random.Random(42)
pairs = []
for i in range(4000):
    kind = i % 4
    if kind == 0: a, b = g.uniform(-30, 30), g.uniform(-30, 30)
    elif kind == 1: a, b = g.uniform(0, 1), g.uniform(0, 1e-3)
    elif kind == 2: a, b = g.uniform(-1e6, 1e6), g.uniform(-1, 1)
    else: a, b = round(g.uniform(-20, 20), 2), round(g.uniform(-20, 20), 2)
    pairs.append([a, b, math.hypot(a, b)])
out['hypot'] = pairs
# statistics
st = []
for i in range(60):
    n = 1 + i % 12
    xs = [g.uniform(-100, 100) * (10 ** g.randint(-3, 3)) for _ in range(n)]
    st.append({'xs': xs, 'mean': statistics.mean(xs), 'pstdev': statistics.pstdev(xs) if n > 0 else 0.0, 'npstd': float(np.std(xs))})
for i in range(10):
    n = 9 + 20 * i
    xs = [g.uniform(0, 10) for _ in range(n)]
    st.append({'xs': xs, 'mean': statistics.mean(xs), 'pstdev': statistics.pstdev(xs), 'npstd': float(np.std(xs))})
out['stats'] = st
# LSAP incl. ties and rectangular
ls = []
for i in range(120):
    nr, nc = g.randint(1, 7), g.randint(1, 7)
    if i % 3 == 0: m = [[float(g.randint(0, 3)) for _ in range(nc)] for _ in range(nr)]
    elif i % 3 == 1: m = [[g.uniform(0, 10) for _ in range(nc)] for _ in range(nr)]
    else:
        m = [[(1e9 if g.random() < 0.4 else g.uniform(0, 40)) for _ in range(nc)] for _ in range(nr)]
    r, c = linear_sum_assignment(np.array(m))
    ls.append({'m': m, 'rows': [int(x) for x in r], 'cols': [int(x) for x in c]})
out['lsap'] = ls
json.dump(out, open(sys.argv[1], 'w'), indent=None)
print('ok', out['python'], out['numpy'], out['scipy'])
