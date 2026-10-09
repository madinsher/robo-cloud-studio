"""RoboDK-API-compatible Python package for VerticalBot Studio.

Drop-in replacement for the `robodk` package: `from robodk.robolink import *` and
`from robodk.robomath import *` work against a running VerticalBot Studio server
(ws://localhost:20500) instead of RoboDK. Scripts written for RoboDK run unchanged
for the common subset of the API (items, frames, targets, programs, moves, IO, posts).
"""

# Same convenience re-exports as the official package: `from robodk import *` gives robomath + dialogs + file IO
from . import robomath, robodialogs, robofileio  # noqa: E402
from .robomath import *  # noqa: F401,F403,E402
from .robodialogs import *  # noqa: F401,F403,E402
from .robofileio import *  # noqa: F401,F403,E402
