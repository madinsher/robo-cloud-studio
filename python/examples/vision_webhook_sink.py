#!/usr/bin/env python3
"""Custom node WITHOUT ROS: receive the studio's perception results over HTTP.

In the vision stack wizard (Runtime & endpoints) set "Publish results to URL" to http://<this host>:8765/vision.
Every pipeline run POSTs the JSON summary (the same object as camera.getParam('visionLast')):
  {camera, time, units: "mm", detections: [{cls, score, id, box:[x,y,w,h], z, p:[x,y,z], attr}],
   targets: [{name, cls, p, grasp:[16 column-major], approach:[16], sigmaMm}], cloud: {...} | null, text, vla}
This example prints a line per run and keeps the latest result for GET /latest — plug your PLC / MQTT / OPC UA here.
"""
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

latest = {}
lock = threading.Lock()


class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        n = int(self.headers.get("content-length", 0))
        data = json.loads(self.rfile.read(n) or b"{}")
        with lock:
            latest.clear(); latest.update(data)
        dets = data.get("detections", [])
        by_cls = {}
        for d in dets:
            by_cls[d["cls"]] = by_cls.get(d["cls"], 0) + 1
        first = next((t for t in data.get("targets", [])), None)
        print(f"[{data.get('camera')}] t={data.get('time', 0):.2f}s {by_cls}"
              + (f" first target {first['cls']} at {first['p']} mm (σ {first['sigmaMm']} mm)" if first else ""))
        self.send_response(204); self.end_headers()

    def do_GET(self):
        with lock:
            body = json.dumps(latest).encode()
        self.send_response(200); self.send_header("content-type", "application/json"); self.end_headers(); self.wfile.write(body)

    def log_message(self, *a):  # quiet
        pass


if __name__ == "__main__":
    print("listening on http://0.0.0.0:8765/vision  (GET /latest for the last result)")
    ThreadingHTTPServer(("0.0.0.0", 8765), Handler).serve_forever()
