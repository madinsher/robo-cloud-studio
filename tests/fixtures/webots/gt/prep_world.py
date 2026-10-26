"""Prepare a COPY of a Webots world for the pose dump (never run on the original files):
gravity 0, every controller replaced by <none>, instances of the PROTO robots listed on the command line get
controller "<none>" when they do not set one, and the dump supervisor robot (controller pose_dump) is appended.
usage: python prep_world.py <src.wbt> <dst.wbt> [ProtoRobotName ...]"""
import io
import re
import sys

src, dst = sys.argv[1], sys.argv[2]
t = io.open(src, encoding='utf-8').read()
t = re.sub(r'controller\s+"[^"]*"', 'controller "<none>"', t)
t = re.sub(r'(WorldInfo\s*\{)', r'\1\n  gravity 0', t, count=1)


def block_end(text, start):
    """Index just after the brace closing the block opened at text[start] == '{' (strings and comments skipped)."""
    depth, i, n = 0, start, len(text)
    while i < n:
        c = text[i]
        if c == '"':
            i += 1
            while i < n and text[i] != '"':
                i += 2 if text[i] == chr(92) else 1
        elif c == '#':
            while i < n and text[i] != '\n':
                i += 1
        elif c == '{':
            depth += 1
        elif c == '}':
            depth -= 1
            if depth == 0:
                return i + 1
        i += 1
    return n


for name in sys.argv[3:]:
    out, pos = [], 0
    for m in re.finditer(r'\b%s\s*\{' % re.escape(name), t):
        body = t[m.end() - 1:block_end(t, m.end() - 1)]
        out.append(t[pos:m.end()])
        if not re.search(r'^\s*controller\s', body, re.M):
            out.append('\n  controller "<none>"')
        pos = m.end()
    out.append(t[pos:])
    t = ''.join(out)
t += '\nRobot {\n  name "gt_pose_dump"\n  supervisor TRUE\n  controller "pose_dump"\n}\n'
io.open(dst, 'w', encoding='utf-8').write(t)
