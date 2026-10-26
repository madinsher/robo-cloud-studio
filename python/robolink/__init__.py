# Legacy module support (same as RoboDK's own python/robolink/__init__.py):
#
#   from robolink import *      ->  from robodk.robolink import *
#   from robodk import *        ->  robomath + robodialogs + robofileio (python/robodk/__init__.py)
#
# Station programs embedded in RoboDK Library stations and many Library scripts still use the legacy imports.
import sys
from robodk import robolink
sys.modules['robolink'] = robolink
