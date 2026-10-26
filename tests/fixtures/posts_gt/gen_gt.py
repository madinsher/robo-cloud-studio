"""Ground-truth generator for tests/posts_gt.test.ts.

Runs RoboDK's ORIGINAL post processors (C:/RoboDK/Posts, compiled libspp / vXY modules) through python/post_shim.py
on every cases/<case>/job.json and stores what each post writes in cases/<case>/<post>/. RoboDK.exe is never
started (robodk.robolink.Robolink is blocked). Use RoboDK's embedded Python (3.10, matching libspp *.cp310*.pyd):

    C:/RoboDK/Python-Embedded/python.exe tests/fixtures/posts_gt/gen_gt.py [case-filter] [post-filter]

Inputs (job.json) come from:  POSTS_GT_GEN=1 npx vitest run tests/posts_gt_gen.test.ts
"""
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SHIM = os.path.normpath(os.path.join(HERE, '..', '..', '..', 'python', 'post_shim.py'))
CASES = os.path.join(HERE, 'cases')
POSTS = 'C:/RoboDK/Posts'
# Posts that do not declare INCLUDE_SUB_PROGRAMS and overwrite the main program when they receive several programs:
# generate every program with its own instance (what RoboDK does when it generates programs one by one).
SEPARATE = {'Staubli_VAL3', 'Techman_TMscript', 'Kawasaki', 'Mecademic', 'Denso_RC8', 'Epson_RC', 'Dobot_V3', 'JAKA', 'Kinova',
            'Mitsubishi', 'Nachi_AX_FD', 'HCR', 'AUBO', 'CSV', 'Doosan_Robotics', 'Omron', 'Techman', 'Elite_Robots_EC_V3'}


# Posts that declare INCLUDE_SUB_PROGRAMS = False but receive the program and its sub-programs in one instance in RoboDK's
# MakeProgram (RoboDK 5.9.4 Library code: KUKA KRC2 writes the main .src only, with EXT for every program called in
# the run): the Library cases are replayed in one instance like RoboDK does; the feature / synthetic cases keep one file
# per program (the importer round trip in tests/posts_import.test.ts reads them).
ONE_INSTANCE = {'KUKA_KRC2', 'KUKA_KRC4'}


def main():
    case_filter = sys.argv[1] if len(sys.argv) > 1 else ''
    post_filter = sys.argv[2] if len(sys.argv) > 2 else ''
    summary = []
    for case in sorted(os.listdir(CASES)):
        jp = os.path.join(CASES, case, 'job.json')
        if not os.path.isfile(jp) or case_filter not in case:
            continue
        job = json.load(open(jp, encoding='utf-8'))
        for post in job.get('posts') or []:
            if post_filter and post != post_filter:
                continue
            out = os.path.join(CASES, case, post)
            shutil.rmtree(out, ignore_errors=True)
            os.makedirs(out)
            jrun = jp
            if post in ONE_INSTANCE and case.startswith('lib_'):
                job2 = dict(job, include_sub_programs=True)
                jrun = out + '.job.json'
                json.dump(job2, open(jrun, 'w', encoding='utf-8'))
            if post in SEPARATE:
                job2 = dict(job, include_sub_programs=False)
                jrun = out + '.job.json'
                json.dump(job2, open(jrun, 'w', encoding='utf-8'))
            r = subprocess.run([sys.executable, SHIM, post, jrun, out, '--posts', POSTS], capture_output=True, text=True, timeout=300)
            if jrun != jp:
                os.remove(jrun)
            try:
                res = json.loads(r.stdout[r.stdout.index('{'):])
            except Exception:
                res = {'ok': False, 'error': (r.stdout + r.stderr)[-2000:]}
            meta = {'ok': res.get('ok'), 'error': res.get('error'), 'log': res.get('log', ''), 'files': [f['name'] for f in res.get('files', [])]}
            json.dump(meta, open(os.path.join(out, '_result.json'), 'w', encoding='utf-8'), indent=1)
            summary.append((case, post, res.get('ok'), meta['files'] if res.get('ok') else res.get('error', '')[:200]))
            print('%-60s %-22s %s %s' % (case, post, 'OK ' if res.get('ok') else 'ERR', summary[-1][3]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
