#!/usr/bin/env python3
"""Read perception and localization results through the RoboDK-compatible API (no ROS needed).

The studio stores the last pipeline output in camera.params['visionLast'] and the localization estimate in
robot.params['navEstimate']; both are readable with Item.getParam() over the studio server
(WebSocket ws://host:20500 JSON-RPC or TCP JSON-lines on port 20501). Start the server with `npm run server` in
studio/, open the studio with ?server=ws://localhost:20500 (the browser session is relayed), then:

    pip install -e studio/python   # the robodk drop-in package
    python3 robolink_poll_vision_nav.py --camera "Camera 1" --robot "Harvest platform 1"
"""
import argparse
import time

from robodk.robolink import Robolink


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default="Camera 1")
    ap.add_argument("--robot", default="")
    ap.add_argument("--period", type=float, default=1.0)
    a = ap.parse_args()
    RDK = Robolink()  # ROBODK_URL env overrides ws://localhost:20500
    cam = RDK.Item(a.camera)
    robot = RDK.Item(a.robot) if a.robot else None
    while True:
        last = cam.getParam("visionLast") if cam.Valid() else None
        if last:
            dets = last.get("detections", [])
            nearest = min((d for d in dets if d.get("z")), key=lambda d: d["z"], default=None)
            print(f"{last['camera']}: {len(dets)} detections" + (f", nearest {nearest['cls']} at {nearest['z']} mm, world {nearest['p']}" if nearest else ""))
            for t in last.get("targets", [])[:3]:
                print(f"   target {t['name']} p={t['p']} mm σ={t['sigmaMm']} mm")
        if robot is not None and robot.Valid():
            est = robot.getParam("navEstimate")
            if est:
                print(f"{a.robot}: estimate ({est['x']}, {est['y']}) θ={est['theta']}° err={est['error']} mm rmse={est['rmse']} mm lost={est['lost']} gnss={est['gnss']} [{est['method']}]")
        time.sleep(a.period)


if __name__ == "__main__":
    main()
