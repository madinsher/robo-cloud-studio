"""Builds synthetic post GT cases from the feature jobs (cases/feature_*/job.json):
  synth_misc_<robot>   named/numbered frames and tools, IO values, waits with timeouts, long names, rounding values
  synth_split_<robot>  40 linear moves with MAX_LINES_X_PROG lowered so posts split the program
Run with any Python 3, then run gen_gt.py synth to produce RoboDK's outputs.
"""
import copy
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
CASES = os.path.join(HERE, 'cases')
IDENT = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]


def moves(job):
    ev = job['events']
    mj = [e for e in ev if e['kind'] == 'moveJ'][0]
    ml = [e for e in ev if e['kind'] == 'moveL']
    return mj, ml


def misc(job):
    mj, ml = moves(job)
    tool = [e for e in job['events'] if e['kind'] == 'setTool'][0]
    a, b = ml[0], ml[-1]
    ev = [
        {'kind': 'setFrame', 'pose': IDENT, 'name': job['robot'] + ' Base', 'frameId': -1},
        {'kind': 'setTool', 'pose': tool['pose'], 'name': 'Spindle', 'toolId': None, 'toolNum': -1},
        {'kind': 'setSpeed', 'speedLinear': 1000},
        {'kind': 'setRounding', 'radius': 10},
        mj, a,
        {'kind': 'setRounding', 'radius': 0.5},
        b,
        {'kind': 'pause', 'timeMs': 0},
        {'kind': 'pause', 'timeMs': 1500},
        {'kind': 'message', 'text': 'Say "hi" to the operator'},
        {'kind': 'comment', 'text': 'A long comment line that exceeds the usual controller limits for comments by far, really'},
        {'kind': 'setDO', 'io': '12', 'value': False},
        {'kind': 'setDO', 'io': 'OUT_A', 'value': True},
        {'kind': 'waitDI', 'io': '4', 'value': False, 'timeoutMs': 1000},
        {'kind': 'waitDI', 'io': 'IN_B', 'value': True, 'timeoutMs': -1},
        {'kind': 'runCode', 'code': 'Program_With_A_Long_Name_ABCDEFGHIJ', 'isCall': True},
        {'kind': 'setFrame', 'pose': [0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 250, 300, -10, 1], 'name': 'Frame 12', 'frameId': 12},
        {'kind': 'setTool', 'pose': tool['pose'], 'name': 'Tool 7', 'toolId': None, 'toolNum': 7},
        {'kind': 'setSpeed', 'speedJoints': 30, 'accelJoints': 100},
        dict(a, kind='moveL'),
        {'kind': 'setRounding', 'radius': -1},
        dict(mj),
    ]
    j = copy.deepcopy(job)
    j.update(program='Prog_Name_That_Is_Very_Long_123', name='Prog_Name_That_Is_Very_Long_123', events=ev, subprograms=[])
    return j


def split(job):
    mj, ml = moves(job)
    a, b = ml[0], ml[-1]
    ev = [e for e in job['events'][:2]] + [{'kind': 'setSpeed', 'speedLinear': 200}, mj]
    for i in range(40):
        ev.append(dict(a if i % 2 == 0 else b))
    j = copy.deepcopy(job)
    j.update(program='SplitTest', name='SplitTest', events=ev, subprograms=[], lines_x_prog=15)
    return j


def main():
    for case in sorted(os.listdir(CASES)):
        if not case.startswith('feature_'):
            continue
        job = json.load(open(os.path.join(CASES, case, 'job.json'), encoding='utf-8'))
        robot = case[len('feature_'):]
        for name, fn in (('synth_misc_', misc), ('synth_split_', split)):
            d = os.path.join(CASES, name + robot)
            os.makedirs(d, exist_ok=True)
            json.dump(fn(job), open(os.path.join(d, 'job.json'), 'w', encoding='utf-8'))
    return 0


if __name__ == '__main__':
    main()
