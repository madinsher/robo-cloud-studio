"""VerticalBot extension calls: drive the world simulation and read fleet KPIs."""
import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from robodk.robolink import *

RDK = Robolink()
RDK.App("startWorld")
time.sleep(10)
RDK.App("pauseWorld")
for fleet in RDK.ItemList(102):
    print(fleet.Name(), RDK.App("fleetManager", fleet))
