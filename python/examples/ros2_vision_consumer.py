#!/usr/bin/env python3
"""Custom ROS 2 node consuming the studio's perception output (or the same topics from the exported perception package).

Subscribes to
  /vision/<camera>/detections_3d   vision_msgs/Detection3DArray  — 3D positions in the map frame (metres)
  /vision/<camera>/targets         geometry_msgs/PoseArray        — grasp poses (tool Z along the camera ray)
and republishes the best target as geometry_msgs/PoseStamped on /target_pose for MoveIt / your controller.

Run rosbridge on the ROS side (ros2 launch rosbridge_server rosbridge_websocket_launch.xml — port 9090), connect the
studio with Connect › ROS 2 via rosbridge…, tick "publish to ROS 2" in the Vision tab, then:
    python3 ros2_vision_consumer.py --camera camera_1 --class apple
"""
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from vision_msgs.msg import Detection3DArray


class VisionConsumer(Node):
    def __init__(self, camera: str, wanted: str):
        super().__init__("vision_consumer")
        self.wanted = wanted
        self.create_subscription(Detection3DArray, f"/vision/{camera}/detections_3d", self.on_detections, 10)
        self.create_subscription(PoseArray, f"/vision/{camera}/targets", self.on_targets, 10)
        self.pub = self.create_publisher(PoseStamped, "/target_pose", 10)

    def on_detections(self, msg: Detection3DArray):
        best = None
        for d in msg.detections:
            hyp = d.results[0].hypothesis
            if self.wanted and hyp.class_id != self.wanted:
                continue
            p = d.bbox.center.position
            dist = (p.x ** 2 + p.y ** 2 + p.z ** 2) ** 0.5
            if best is None or dist < best[0]:
                best = (dist, hyp, d)
        if best:
            _, hyp, d = best
            p = d.bbox.center.position
            self.get_logger().info(f"nearest {hyp.class_id} score {hyp.score:.2f} at ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m, track {d.id or '-'}")

    def on_targets(self, msg: PoseArray):
        if not msg.poses:
            return
        out = PoseStamped()
        out.header = msg.header
        out.pose = msg.poses[0]          # first grasp pose (ordered like the detections)
        self.pub.publish(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default="camera_1")
    ap.add_argument("--class", dest="cls", default="")
    a = ap.parse_args()
    rclpy.init()
    rclpy.spin(VisionConsumer(a.camera, a.cls))


if __name__ == "__main__":
    main()
