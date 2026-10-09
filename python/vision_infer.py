#!/usr/bin/env python3
"""Studio vision inference worker (used by the studio server's ``POST /vision/infer``).

Reads one JSON request on stdin::

    {"model": "yolov8n.pt" | "path/to/model.onnx", "task": "detect|segment|keypoints|classify",
     "image": "data:image/png;base64,...", "confidence": 0.4, "iou": 0.5, "classes": ["apple"]}

and prints a JSON response the studio's ``parseDetectionsJson`` understands::

    {"detections": [{"x":..,"y":..,"w":..,"h":..,"score":..,"class":"apple","keypoints":[[x,y,s],...],
                     "points":[[x,y],...]}], "labels":[{"class":..,"score":..}], "backend": "ultralytics"}

Backends, tried in order:
  1. ``ultralytics`` (any YOLOv5/8/10/11, RT-DETR, YOLO-World ``.pt``/``.onnx``/``.engine``) — pip install ultralytics
  2. ``onnxruntime`` + a minimal YOLOv8 decoder (detect / classify only) — pip install onnxruntime pillow numpy
  3. an OpenAI-compatible VLM when ``model`` starts with ``vlm:`` (``VLM_URL`` / ``VLM_MODEL`` env, or fields in the request)

``python3 vision_infer.py --check`` prints which backends are available.
"""
from __future__ import annotations

import base64
import io
import json
import os
import sys
import time


def _decode_image(data_url: str):
    from PIL import Image  # type: ignore

    b64 = data_url.split(",", 1)[1] if data_url.startswith("data:") else data_url
    return Image.open(io.BytesIO(base64.b64decode(b64))).convert("RGB")


def _check() -> dict:
    out = {"ultralytics": False, "onnxruntime": False, "pillow": False, "numpy": False}
    for k in list(out):
        try:
            __import__("PIL" if k == "pillow" else k)
            out[k] = True
        except Exception:  # noqa: BLE001
            pass
    out["models_dir"] = os.environ.get("STUDIO_VISION_MODELS", "")
    return out


def run_ultralytics(req: dict, img) -> dict:
    from ultralytics import YOLO  # type: ignore

    model = YOLO(req["model"])
    kw = {"conf": req.get("confidence", 0.25), "iou": req.get("iou", 0.5), "verbose": False}
    if req.get("classes") and hasattr(model, "set_classes") and "world" in str(req["model"]).lower():
        model.set_classes(req["classes"])
    res = model.predict(img, **kw)[0]
    names = res.names if hasattr(res, "names") else {}
    out: dict = {"detections": [], "backend": "ultralytics"}
    if getattr(res, "probs", None) is not None:
        top = res.probs.top5
        out["labels"] = [{"class": names.get(int(i), str(i)), "score": float(res.probs.data[int(i)])} for i in top]
        return out
    boxes = res.boxes
    masks = getattr(res, "masks", None)
    kps = getattr(res, "keypoints", None)
    for i in range(len(boxes)):
        x1, y1, x2, y2 = [float(v) for v in boxes.xyxy[i].tolist()]
        d = {"x": x1, "y": y1, "w": x2 - x1, "h": y2 - y1, "score": float(boxes.conf[i]), "class": names.get(int(boxes.cls[i]), str(int(boxes.cls[i])))}
        if boxes.id is not None:
            d["track_id"] = int(boxes.id[i])
        if masks is not None and masks.xy is not None and i < len(masks.xy):
            d["points"] = [[float(p[0]), float(p[1])] for p in masks.xy[i]]
        if kps is not None and kps.data is not None:
            d["keypoints"] = [[float(k[0]), float(k[1]), float(k[2]) if len(k) > 2 else 1.0] for k in kps.data[i].tolist()]
        out["detections"].append(d)
    return out


def run_onnxruntime(req: dict, img) -> dict:
    import numpy as np  # type: ignore
    import onnxruntime as ort  # type: ignore

    sess = ort.InferenceSession(req["model"], providers=ort.get_available_providers())
    inp = sess.get_inputs()[0]
    size = int(req.get("inputSize") or (inp.shape[2] if isinstance(inp.shape[2], int) else 640))
    w, h = img.size
    scale = min(size / w, size / h)
    nw, nh = int(round(w * scale)), int(round(h * scale))
    px, py = (size - nw) // 2, (size - nh) // 2
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    canvas[py:py + nh, px:px + nw] = np.asarray(img.resize((nw, nh)))
    x = canvas.astype(np.float32).transpose(2, 0, 1)[None] / 255.0
    outs = sess.run(None, {inp.name: x})
    o = outs[0]
    classes = req.get("classes") or []
    conf_t, iou_t = float(req.get("confidence", 0.25)), float(req.get("iou", 0.5))
    task = req.get("task", "detect")
    if task == "classify" or o.ndim == 2:
        probs = o.reshape(-1)
        order = np.argsort(-probs)[:5]
        return {"detections": [], "labels": [{"class": classes[int(i)] if int(i) < len(classes) else str(int(i)), "score": float(probs[i])} for i in order], "backend": "onnxruntime"}
    o = o[0]
    if o.shape[0] < o.shape[1]:
        o = o.T  # [N, 4+nc]
    nc = o.shape[1] - 4
    scores = o[:, 4:4 + nc]
    cls = scores.argmax(1)
    conf = scores.max(1)
    keep = conf >= conf_t
    o, cls, conf = o[keep], cls[keep], conf[keep]
    boxes = []
    for row, c, s in zip(o, cls, conf):
        cx, cy, bw, bh = row[:4]
        boxes.append([(cx - bw / 2 - px) / scale, (cy - bh / 2 - py) / scale, bw / scale, bh / scale, float(s), int(c)])
    boxes.sort(key=lambda b: -b[4])
    kept = []
    for b in boxes:
        ok = True
        for k in kept:
            if k[5] != b[5]:
                continue
            ix = max(0, min(b[0] + b[2], k[0] + k[2]) - max(b[0], k[0]))
            iy = max(0, min(b[1] + b[3], k[1] + k[3]) - max(b[1], k[1]))
            inter = ix * iy
            union = b[2] * b[3] + k[2] * k[3] - inter
            if union > 0 and inter / union > iou_t:
                ok = False
                break
        if ok:
            kept.append(b)
    return {"detections": [{"x": b[0], "y": b[1], "w": b[2], "h": b[3], "score": b[4], "class": classes[b[5]] if b[5] < len(classes) else str(b[5])} for b in kept], "backend": "onnxruntime"}


def run_vlm(req: dict) -> dict:
    import urllib.request

    url = (req.get("vlmUrl") or os.environ.get("VLM_URL", "http://localhost:11434/v1")).rstrip("/") + "/chat/completions"
    model = req.get("vlmModel") or os.environ.get("VLM_MODEL") or req["model"].split(":", 1)[1] or "qwen2.5vl:7b"
    classes = ", ".join(req.get("classes") or ["object"])
    prompt = req.get("prompt") or (
        f"Detect every {classes} in the image. Reply ONLY with JSON: {{\"detections\":[{{\"label\":\"<class>\",\"box_2d\":[x1,y1,x2,y2],\"confidence\":0.0-1.0}}]}} "
        "with coordinates in a 0-1000 normalised frame."
    )
    body = {"model": model, "temperature": 0, "max_tokens": 800, "messages": [{"role": "user", "content": [{"type": "text", "text": prompt}, {"type": "image_url", "image_url": {"url": req["image"]}}]}]}
    headers = {"content-type": "application/json"}
    key = req.get("apiKey") or os.environ.get("VLM_API_KEY")
    if key:
        headers["authorization"] = f"Bearer {key}"
    r = urllib.request.Request(url, data=json.dumps(body).encode(), headers=headers)
    with urllib.request.urlopen(r, timeout=120) as resp:  # noqa: S310
        j = json.loads(resp.read())
    text = j.get("choices", [{}])[0].get("message", {}).get("content", "")
    return {"detections": [], "text": text, "backend": "vlm", "raw": j}


def main() -> int:
    if "--check" in sys.argv:
        print(json.dumps(_check()))
        return 0
    req = json.loads(sys.stdin.read() or "{}")
    t0 = time.time()
    try:
        if str(req.get("model", "")).startswith("vlm:"):
            out = run_vlm(req)
        else:
            img = _decode_image(req["image"])
            models_dir = os.environ.get("STUDIO_VISION_MODELS")
            if models_dir and not os.path.isabs(req["model"]) and os.path.exists(os.path.join(models_dir, req["model"])):
                req["model"] = os.path.join(models_dir, req["model"])
            try:
                out = run_ultralytics(req, img)
            except ImportError:
                out = run_onnxruntime(req, img)
        out["latencyMs"] = round((time.time() - t0) * 1000, 1)
        print(json.dumps(out))
        return 0
    except Exception as e:  # noqa: BLE001
        print(json.dumps({"error": f"{type(e).__name__}: {e}", "check": _check()}))
        return 1


if __name__ == "__main__":
    sys.exit(main())
