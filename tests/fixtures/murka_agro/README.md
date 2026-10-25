# MURKA agromarca experiment ground truth

Python ground truth for `tests/murka_agro_experiments.test.ts` (studio port: `src/murka/agro/experiments.ts`,
`src/fleet/murka_alloc.ts`, `src/fleet/lsap.ts`, `src/murka/agro/pycompat.ts`).

| Directory | Origin | Interpreter | Set order of the extra capabilities |
|---|---|---|---|
| `article_py310/` | the article's published artifacts, `agromarca/article/replication/artifacts/experiments/*.json` (copied, gzipped) | CPython 3.10 (pycache), NumPy < 2, SciPy < 1.14, random hash seed | `harvest, manipulation, cargo_carry, prune, spray` (`ARTICLE_POPULATION_ORDER`, found by trying all 120 orders: unique) |
| `py312_hashseed0/` | `gt_run.sh` on a copy of `murca_garden_ws/src/{murca_experiment,murca_metrics}`, `all --runs 20` | CPython 3.12.3, NumPy 2.5.3, SciPy 1.18.1 (WSL), `PYTHONHASHSEED=0` | `harvest, cargo_carry, manipulation, prune, spray` (`PY312_HASHSEED0_ORDER`) |
| `pycompat_vectors_py312.json.gz` | `pycompat_vectors.py` under the same CPython 3.12.3 | — | — |

Why the set order matters: `generate_random_mission` calls `rng.sample(list(set(ALL_CAPABILITIES) - caps), k)`;
the list order is CPython's set iteration order, which depends on the string hashes and therefore on PYTHONHASHSEED.
exp2 and exp4 change with it; exp1, exp3 and exp5 do not (both directories agree there).

CPython 3.7 (C:/RoboDK/Python37) gives the same exp1 / exp2 / exp4 / exp5 for the same order but differs by 1 ulp in
`math.hypot` (libm instead of CPython ≥ 3.10's `vector_norm`) — e.g. exp3 `per_robot_distance` — and in
`statistics.pstdev`; the studio follows CPython ≥ 3.10 (the article's interpreter).

Regenerate (WSL, needs numpy + scipy):

```
bash tests/fixtures/murka_agro/gt_run.sh "/mnt/c/.../MURKA_Simulation/simulation_ws/src/agromarca/murca_garden_ws/src" tests/fixtures/murka_agro/py312_hashseed0 0
python3 tests/fixtures/murka_agro/pycompat_vectors.py /tmp/v.json && gzip -9 -c /tmp/v.json > tests/fixtures/murka_agro/pycompat_vectors_py312.json.gz
```
