#!/usr/bin/env python3
"""G3: subscribe to the MURKA head camera / range finder for N wall seconds and report the rates in sim time.

Prints one JSON line: frames per topic, the sim-time span of their stamps, /clock advance, the wall time and the
image sizes / encodings — the studio backend renders the cameras only while they are subscribed (webots_ros2
devices without <alwaysOn>), so this also checks that subscribing turns them on.
"""
import json
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image

SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
rclpy.init()
node = rclpy.create_node("g3_cam_rate")
got = {"/head_camera/image_color": [], "/head_depth/image": []}
info = {}
clock = []


def mk(topic):
    def cb(msg):
        got[topic].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        info[topic] = {"w": msg.width, "h": msg.height, "enc": msg.encoding, "bytes": len(msg.data)}
    return cb


for t in got:
    node.create_subscription(Image, t, mk(t), qos_profile_sensor_data)
node.create_subscription(Clock, "/clock", lambda m: clock.append(m.clock.sec + m.clock.nanosec * 1e-9), 10)
t0 = time.monotonic()
# the first second: the studio notices the subscription (rosapi poll) and starts rendering
while time.monotonic() - t0 < SECS:
    rclpy.spin_once(node, timeout_sec=0.05)
wall = time.monotonic() - t0
out = {"wall": round(wall, 2), "clock_span": round(clock[-1] - clock[0], 3) if len(clock) > 1 else 0, "clock_n": len(clock)}
for t, st in got.items():
    span = st[-1] - st[0] if len(st) > 1 else 0
    out[t] = {"n": len(st), "span": round(span, 3), "rate_sim": round((len(st) - 1) / span, 2) if span > 0 else 0, **info.get(t, {})}
print("CAM_RATE " + json.dumps(out), flush=True)
node.destroy_node()
rclpy.shutdown()
