"""Replay the REAL agromarca ScriptedFleet (default Mission B script) without Webots and dump a reference
pose trace for the studio's future scripted-fleet player (acceptance ground truth).

Seeds = initial poses of orchard_demo.wbt (read by hand from the world file, 2026-10-02 version).
Usage: py -3 -I mission_b_reference.py <controllers/garden_supervisor dir> <out.json>
"""
import json
import math
import sys

CTRL, OUT = sys.argv[1], sys.argv[2]
sys.path.insert(0, CTRL)
import scripted_fleet as sf  # noqa: E402
import garden_supervisor as gs  # noqa: E402

SEED = {
    "UTP-G-01": (-10.8, -12.5, 0.08, 1.5708),
    "UTP-G-02": (-8.0, -12.5, 0.084, 1.5708),
    "UTP-G-03": (-4.5, -12.5, 0.084, 1.5708),
    "UTP-G-04": (-1.0, -12.5, 0.084, 1.5708),
    "UTP-A-01": (0.8, -12.5, 1.2, 1.5708),
    "crate_work": (-10.8, -12.5, 0.0, 1.5708),
    "crate_empty_1": (-8.4, -10.4, 0.0, 0.0),
    "crate_empty_2": (-7.2, -10.4, 0.0, 0.0),
    "module_sprayer": (-18.2, -9.1, 0.83, 0.0),
    "module_harvester_arm": (-18.2, -9.8, 0.93, 0.0),
    "module_scout_mast": (-18.2, -10.5, 0.95, 0.0),
    "depot_slot_1": (-14.0, -9.8, 0.0025, 0.0),
    "depot_slot_2": (-16.0, -9.8, 0.0025, 0.0),
    "collection_point_1": (-8.4, -9.0, 0.025, 0.0),
}
ENTITIES = [k for k in SEED if not k.startswith("depot_slot") and k != "collection_point_1"]
DT = 0.016
T_END = 92.0

script = gs.default_mission_b_script(include_harvester=True)
fleet = sf.ScriptedFleet.from_dict(script)
ad = sf.DictWorldAdapter()
for n, p in SEED.items():
    ad.seed_pose(n, *p)
if hasattr(fleet, "snapshot_depot_modules"):
    fleet.snapshot_depot_modules(ad, gs.DEPOT_MODULES)
fleet.require_world(ad)

trace = []
n = 0
max_step = {e: 0.0 for e in ENTITIES}
prev = {e: ad.poses[e] for e in ENTITIES}
while True:
    n += 1
    t = n * DT
    if t > T_END:
        break
    fleet.tick(ad, t)
    for e in ENTITIES:
        p = ad.poses[e]
        d = math.hypot(p[0] - prev[e][0], p[1] - prev[e][1])
        if d > max_step[e]:
            max_step[e] = d
        prev[e] = p
    if n % 31 == 0 or t + DT > T_END:  # ~0.5 s
        trace.append({"t": round(t, 3), "poses": {e: [round(v, 4) for v in ad.poses[e]] for e in ENTITIES}})

final = {e: [round(v, 4) for v in ad.poses[e]] for e in ENTITIES}
wheel_fields = {"%s.%s" % k: [round(x, 4) for x in v] for k, v in ad.rotations.items()}
report = fleet.plan_report({e: SEED[e] for e in ENTITIES}) if hasattr(fleet, "plan_report") else []
json.dump({"dt": DT, "t_end": T_END, "seed": SEED, "final": final, "max_step_m": max_step,
           "wheel_fields_final": wheel_fields, "errors": fleet.errors, "plan_report": report, "trace": trace,
           "script": script}, open(OUT, "w"), indent=1, default=str)
print("ticks", n - 1, "errors", fleet.errors)
for e in ENTITIES:
    print("%-22s start %-34s final %-36s max-step %.3f m" % (e, SEED[e], final[e], max_step[e]))
