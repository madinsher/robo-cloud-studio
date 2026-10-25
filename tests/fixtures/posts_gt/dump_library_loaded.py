"""RoboDK 5.9.4 MakeProgram of every Library program FROM THE STATION AS LOADED (tests/posts_library_code.test.ts).

tests/fixtures/robodk_gt/code/robodk_generated_code.tgz was written by dump_gt.py after it had regenerated every
machining project (Item.Update() of the machining items) and simulated each program (Update / InstructionListJoints
leave the robot at the program's end). This script writes the reference that depends on nothing but the .rdk: the
station is opened, nothing is regenerated or simulated, and MakeProgram is called for each program with the robots at
the joints stored in the file (the station is reopened whenever a robot moved).

RoboDK runs a Python post through an intermediate script (%TEMP%/Post<program>.py: the exact RobotPost calls, with the
poses, joints and configuration RoboDK chose for every move) and deletes it afterwards. RoboDK is started with TEMP
pointing to a private folder, and a watcher thread keeps the last copy of each intermediate script as
<program>/_rdk_job_<name>.py next to the generated files.

    C:/RoboDK/Python37/python -I dump_library_loaded.py <port> <out-dir> [station-filter]

Output: <out-dir>/<station>_/<program>/<files> (same layout as robodk_generated_code.tgz) and <out-dir>/<station>_/
_programs.json (program -> {robot, files, make}).
"""
import sys, os, json, shutil, threading, time
sys.path.insert(0, r'C:\RoboDK\Python')

LIB = r'C:\RoboDK\Library'
port, out = int(sys.argv[1]), sys.argv[2]
flt = sys.argv[3] if len(sys.argv) > 3 else ''
TMP = os.path.join(out, '_tmp_%d' % port)
os.makedirs(TMP, exist_ok=True)
os.environ['TEMP'] = os.environ['TMP'] = TMP          # inherited by RoboDK.exe (Qt temp path)
from robodk.robolink import *

RDK = Robolink(args=['-NOSPLASH', '-HIDDEN', '-NEWINSTANCE', '-NOUI'], port=port)
if os.environ.get('DUMP_TIMEOUT'):
    RDK.COM.settimeout(float(os.environ['DUMP_TIMEOUT']))

capture = {'dir': None, 'stop': False}


POST_LIMIT = float(os.environ.get('DUMP_POST_LIMIT', '600'))


def kill_post(path):
    # a RoboDK post that runs for minutes (or waits for a dialog nobody sees) is stopped: RoboDK then reports the failure
    import subprocess
    ps = ("Get-CimInstance Win32_Process -Filter \"Name='python.exe'\" | Where-Object { $_.CommandLine -like '*%s*' } "
          "| ForEach-Object { Stop-Process -Id $_.ProcessId -Force }" % os.path.basename(path))
    subprocess.run(['powershell', '-NoProfile', '-Command', ps])


def watcher():
    seen = {}
    born = {}
    while not capture['stop']:
        for src, t in list(born.items()):
            if os.path.exists(src) and time.time() - t > POST_LIMIT:
                print('KILL slow post', src, flush=True)
                kill_post(src)
                if capture['dir']:
                    open(os.path.join(capture['dir'], '_post_killed.txt'), 'w').write('RoboDK post ran over %d s' % POST_LIMIT)
                born.pop(src)
            elif not os.path.exists(src):
                born.pop(src)
        try:
            for f in os.listdir(TMP):
                if f.startswith('Post') and f.endswith('.py') and capture['dir']:
                    src = os.path.join(TMP, f)
                    born.setdefault(src, time.time())
                    try:
                        st = os.stat(src)
                        key = (st.st_size, st.st_mtime)
                        if seen.get(src) != key:
                            shutil.copyfile(src, os.path.join(capture['dir'], '_rdk_job_%s' % f[4:]))
                            seen[src] = key
                    except OSError:
                        pass
        except OSError:
            pass
        time.sleep(0.02)


threading.Thread(target=watcher, daemon=True).start()


def safe(name):
    return ''.join(ch if ch.isalnum() or ch in '-_' else '_' for ch in name)


def joints_all():
    return [(r.Name(), [round(v, 6) for v in r.Joints().list()]) for r in RDK.ItemList(ITEM_TYPE_ROBOT)]


def open_station(f):
    RDK.CloseStation()
    RDK.AddFile(os.path.join(LIB, f))
    RDK.setRunMode(RUNMODE_SIMULATE)


try:
    files = sorted((x for x in os.listdir(LIB) if x.lower().endswith('.rdk') and flt in x), reverse=bool(os.environ.get('DUMP_REVERSE')))
    for f in files:
        sdir = os.path.join(out, safe(f) + '_')
        failed_file = os.path.join(sdir, '_failed.txt')
        retry = os.environ.get('DUMP_RETRY_FAILED') and os.path.isfile(failed_file) and not os.path.isfile(os.path.join(sdir, '_programs.json'))
        if not retry and (os.path.isfile(os.path.join(sdir, '_programs.json')) or os.path.isfile(failed_file) or os.path.isdir(sdir)):
            continue   # done, failed, or being dumped by another instance
        # DUMP_RETRY_FAILED: the station again, without the programs that blocked RoboDK before (listed in _failed.txt)
        skip = [l.split(':')[0] for l in open(failed_file, encoding='utf-8').read().splitlines() if l.strip()] if retry else []
        open_station(f)
        names = [p.Name() for p in RDK.ItemList(ITEM_TYPE_PROGRAM)]
        if not names:
            continue
        os.makedirs(sdir, exist_ok=True)
        j0 = joints_all()
        meta = {}
        for name in names:
            if name in skip:
                meta[name] = {'robot': '', 'files': [], 'make': None, 'failed': 'RoboDK did not return from MakeProgram (post blocked or over the time limit)'}
                continue
            if joints_all() != j0:
                open_station(f)
            p = RDK.Item(name, ITEM_TYPE_PROGRAM)
            pdir = os.path.join(sdir, safe(name))
            os.makedirs(pdir, exist_ok=True)
            for old in os.listdir(TMP):            # intermediate files of earlier programs
                try:
                    os.remove(os.path.join(TMP, old))
                except OSError:
                    pass
            capture['dir'] = pdir
            t0 = time.time()
            try:
                mk = p.MakeProgram(pdir)
            except Exception as e:
                # a modal popup blocked the API (e.g. a post that waits for input): mark the station, restart RoboDK
                print('ERROR', f, name, e, flush=True)
                prev = open(failed_file, encoding='utf-8').read() if os.path.isfile(failed_file) else ''
                open(failed_file, 'w', encoding='utf-8').write(prev + ('' if not prev or prev.endswith(chr(10)) else chr(10)) + '%s: %s' % (name, e) + chr(10))
                raise
            time.sleep(0.1)
            capture['dir'] = None
            meta[name] = {'robot': p.getLink(ITEM_TYPE_ROBOT).Name() if p.getLink(ITEM_TYPE_ROBOT).Valid() else '',
                          'files': sorted(x for x in os.listdir(pdir) if not x.startswith('_rdk_job')),
                          'make': list(mk) if mk else None, 'seconds': round(time.time() - t0, 2)}
        json.dump({'joints': j0, 'programs': meta}, open(os.path.join(sdir, '_programs.json'), 'w', encoding='utf-8'), indent=1, default=str)
        print(f, len(names), 'programs', flush=True)
finally:
    capture['stop'] = True
    try:
        RDK.CloseRoboDK()
    except Exception:
        # a modal popup blocked the API: kill only the instance listening on this script's port
        import subprocess
        ps = ("Get-CimInstance Win32_Process -Filter \"Name='RoboDK.exe'\" | Where-Object { $_.CommandLine -like '*-PORT=%d*' } "
              "| ForEach-Object { Stop-Process -Id $_.ProcessId -Force }" % port)
        subprocess.run(['powershell', '-NoProfile', '-Command', ps])
