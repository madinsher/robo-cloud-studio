#!/usr/bin/env python3
"""Minimal inference server implementing the studio contract, so you can plug in ANY model of your own.

POST /vision/infer  {"image": "data:image/jpeg;base64,...", "task": "detect", "classes": [...], "confidence": 0.4, "iou": 0.5, "model": "..."}
→ {"detections": [{"x","y","w","h","score","class", "keypoints"?: [[x,y,s]], "points"?: [[x,y]], "position"?: [x,y,z mm]}],
   "labels"?: [{"class","score"}], "text"?: str}

Use it directly from the browser (Runtime "Studio server" with the studio server's STUDIO_VISION_URL pointing here) or
through the exported ROS 2 package. Replace `detect()` with your model (Ultralytics, TensorRT, a Triton client…).
Here: a red-blob detector with OpenCV when available, otherwise a fixed box — enough to test the wiring end to end.
"""
import base64
import io
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import numpy as np
    from PIL import Image
    import cv2
except Exception:  # noqa: BLE001
    np = cv2 = Image = None


def detect(img_bytes: bytes, classes, conf):
    if cv2 is None:
        return [{"x": 100, "y": 80, "w": 60, "h": 60, "score": 0.5, "class": classes[0] if classes else "object"}]
    img = np.array(Image.open(io.BytesIO(img_bytes)).convert("RGB"))
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255)) | cv2.inRange(hsv, (170, 120, 70), (180, 255, 255))
    out = []
    for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
        x, y, w, h = cv2.boundingRect(c)
        if w * h < 60:
            continue
        out.append({"x": int(x), "y": int(y), "w": int(w), "h": int(h), "score": min(0.99, 0.5 + w * h / 5000), "class": classes[0] if classes else "red_object",
                    "points": [[int(p[0][0]), int(p[0][1])] for p in c[::max(1, len(c) // 32)]]})
    return [d for d in out if d["score"] >= conf]


class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path != "/vision/infer":
            self.send_response(404); self.end_headers(); return
        req = json.loads(self.rfile.read(int(self.headers.get("content-length", 0))) or b"{}")
        b64 = req.get("image", "").split(",", 1)[-1]
        dets = detect(base64.b64decode(b64), req.get("classes") or [], float(req.get("confidence", 0.4)))
        body = json.dumps({"detections": dets, "backend": "custom_inference_server"}).encode()
        self.send_response(200); self.send_header("content-type", "application/json"); self.send_header("access-control-allow-origin", "*"); self.end_headers(); self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        for k, v in {"access-control-allow-origin": "*", "access-control-allow-headers": "*", "access-control-allow-methods": "POST,OPTIONS"}.items():
            self.send_header(k, v)
        self.end_headers()

    def log_message(self, *a):
        pass


if __name__ == "__main__":
    print("inference server on http://0.0.0.0:8500/vision/infer  → export STUDIO_VISION_URL=http://localhost:8500/vision/infer for the studio server")
    ThreadingHTTPServer(("0.0.0.0", 8500), Handler).serve_forever()
