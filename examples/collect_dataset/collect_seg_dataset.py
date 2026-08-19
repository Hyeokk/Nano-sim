#!/usr/bin/env python3
"""Nano-sim - RGB + segmentation dataset collector (Python API, SimServer 7720).

Drives the built-in expert (route_follow) around the track and captures paired
frames straight off the unified binary frame (get_frame_bin):

    <out>/rgb/frame_000001.jpg     RGB image (JPEG, lossy but light)
    <out>/mask/frame_000001.png    segmentation label (PNG, LOSSLESS, colour-coded)
    <out>/classes.json             the class legend (name + RGB)
    <out>/manifest.jsonl           one JSON line per frame (paths, shape, pose)

The mask is the class-COLOUR render (flat, non-antialiased), saved losslessly.
Convert colour -> integer class index at training time with seg_labels.py.
The output layout matches collect_dataset_unified.py, so validate_dataset.py works.

Prereqs (Unity): Play mode on IROC_seg; SimServer on 127.0.0.1:7720; the camera's
    Segmentation stream ENABLED in the Sensor Setup panel (so a ".../seg" image is
    in the frame). The camera name must contain --device (default "veye").
Deps: pip install numpy pillow        (opencv-python optional, used if present)

Example:
    python collect_seg_dataset.py --out dataset --frames 3000 --loops 6 \
        --stride 3 --steer-noise 0.05
"""
import argparse
import json
import os
import socket
import struct

import numpy as np

# Class legend: index = position in this list; colour = the sim's flat seg render
# (already float->byte truncated, e.g. tunnel/lidar are 127, not 128). Single
# source for colour <-> class-index; keep in sync with SceneConfig.segCategories.
CLASS_LEGEND = [
    {"name": "road",        "rgb": [128, 64, 128]},
    {"name": "lane",        "rgb": [0, 255, 0]},
    {"name": "stop",        "rgb": [255, 0, 0]},
    {"name": "lidar",       "rgb": [127, 127, 127]},
    {"name": "vehicle",     "rgb": [0, 0, 142]},
    {"name": "obstacle",    "rgb": [135, 206, 250]},
    {"name": "adboard",     "rgb": [69, 69, 69]},
    {"name": "tunnel",      "rgb": [255, 127, 0]},
    {"name": "nondrivable", "rgb": [244, 35, 232]},
    {"name": "background",  "rgb": [0, 0, 0]},
]


# --------------------------------------------------------------- image saving
def make_savers(jpeg_quality):
    """Return (save_rgb, save_mask, backend). Prefer OpenCV, fall back to Pillow.

    RGB is JPEG (light); the mask is ALWAYS a lossless PNG - a label must never
    be resampled or lossily recompressed.
    """
    try:
        import cv2

        def save_rgb(path, rgb):
            cv2.imwrite(path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                        [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])

        def save_mask(path, rgb):
            cv2.imwrite(path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                        [cv2.IMWRITE_PNG_COMPRESSION, 1])

        return save_rgb, save_mask, "opencv"
    except Exception:
        from PIL import Image

        def save_rgb(path, rgb):
            Image.fromarray(rgb, "RGB").save(path, "JPEG", quality=jpeg_quality)

        def save_mask(path, rgb):
            Image.fromarray(rgb, "RGB").save(path, "PNG", compress_level=1)

        return save_rgb, save_mask, "pillow"


# --------------------------------------------------------------- SimServer client
class NanoSimClient:
    def __init__(self, host="127.0.0.1", port=7720, timeout=60.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self._id = 0

    def _recv_exact(self, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("SimServer closed the connection")
            buf.extend(chunk)
        return bytes(buf)

    def _send(self, method, params):
        self._id += 1
        req = {"id": self._id, "method": method,
               "params_json": json.dumps(params or {})}
        payload = json.dumps(req).encode("utf-8")
        self.sock.sendall(struct.pack("<I", len(payload)) + payload)

    def call(self, method, params=None):
        self._send(method, params)
        length = struct.unpack("<I", self._recv_exact(4))[0]
        resp = json.loads(self._recv_exact(length).decode("utf-8"))
        if resp.get("error"):
            raise RuntimeError(f"{method} failed: {resp['error']}")
        result = resp.get("result")
        if isinstance(result, str) and result:
            try:
                return json.loads(result)
            except json.JSONDecodeError:
                return result
        return result

    def read_frame(self):
        """get_frame_bin -> {key: {'kind','array','meta'}} for image sensors only.

        Images arrive bottom-up as raw RGB24; we flip them to top-down. Non-image
        sensors (LiDAR/IMU/GNSS) are skipped here - see collect_dataset_unified.py
        for the full multi-sensor version.
        """
        self._send("get_frame_bin", None)
        total = struct.unpack("<I", self._recv_exact(4))[0]
        body = self._recv_exact(total)
        if total >= 2 and body[0] == 0x7B and body[1] == 0x22:   # '{"' -> JSON error
            raise RuntimeError("get_frame_bin failed: %s" % json.loads(body).get("error"))
        hlen = struct.unpack("<I", body[:4])[0]
        header = json.loads(body[4:4 + hlen].decode("utf-8"))
        pixels = memoryview(body)[4 + hlen:]
        out, off = {}, 0
        for s in header["sensors"]:
            n = int(s["len"])
            raw = pixels[off:off + n]
            off += n
            if s.get("kind") == "image":
                arr = np.frombuffer(raw, np.uint8).reshape(s["shape"])
                arr = np.ascontiguousarray(arr[::-1])        # bottom-up -> top-down
                out[s["key"]] = {"kind": "image", "array": arr, "meta": s}
        return out

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def pick_image_keys(frame, device):
    """Find the (rgb_key, seg_key) pair for the requested device substring."""
    device = device.lower()
    cams = [k for k, v in frame.items() if v["kind"] == "image"]
    dev = [k for k in cams if device in k.lower()] or cams
    seg = next((k for k in dev if "seg" in k.lower()), None)
    if seg is None:
        return None, None
    base = seg.split("/")[0]
    rgb = base if base in frame else next((k for k in dev if "/" not in k), None)
    return rgb, seg


def slim_obs(obs):
    """Keep only the small, useful metadata (pose + route) out of the observation."""
    if not isinstance(obs, dict):
        return None
    return {"ego_state": obs.get("ego_state"), "route": obs.get("route")}


def begin_route(c, args):
    c.call("route_follow", {"action": "begin",
                            "steer_noise": args.steer_noise,
                            "throttle_noise": args.throttle_noise})


def main():
    ap = argparse.ArgumentParser(description="Nano-sim RGB + segmentation dataset collector")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=7720)
    ap.add_argument("--out", default="dataset")
    ap.add_argument("--device", default="veye", help="substring of the camera name to capture")
    ap.add_argument("--frames", type=int, default=2000, help="number of frames to save")
    ap.add_argument("--loops", type=int, default=1, help="stop after this many completed laps")
    ap.add_argument("--stride", type=int, default=1, help="save every Nth frame (decorrelate)")
    ap.add_argument("--warmup-ticks", type=int, default=5)
    ap.add_argument("--steer-noise", type=float, default=0.0, help="expert steering noise (augmentation)")
    ap.add_argument("--throttle-noise", type=float, default=0.0)
    ap.add_argument("--jpeg-quality", type=int, default=95)
    args = ap.parse_args()

    save_rgb, save_mask, backend = make_savers(args.jpeg_quality)
    rgb_dir = os.path.join(args.out, "rgb")
    mask_dir = os.path.join(args.out, "mask")
    os.makedirs(rgb_dir, exist_ok=True)
    os.makedirs(mask_dir, exist_ok=True)
    with open(os.path.join(args.out, "classes.json"), "w") as f:
        json.dump(CLASS_LEGEND, f, indent=2)

    print("image backend:", backend)
    c = NanoSimClient(args.host, args.port)
    manifest = open(os.path.join(args.out, "manifest.jsonl"), "w")
    rgb_key = seg_key = None
    saved = tick_n = seen = finishes = 0
    try:
        print("connected:", c.call("connect"))
        c.call("set_sync_mode", {"enabled": True})
        c.call("reset")
        begin_route(c, args)

        while saved < args.frames:
            res = c.call("tick") or {}
            tick_n += 1
            if tick_n <= args.warmup_ticks:
                continue

            frame = c.read_frame()
            if rgb_key is None:
                rgb_key, seg_key = pick_image_keys(frame, args.device)
                if rgb_key is None or seg_key is None:
                    print("images in frame:", {k: list(v["array"].shape) for k, v in frame.items()})
                    raise SystemExit(
                        "No RGB/seg image pair for device '%s'. Enable the camera's "
                        "Segmentation stream in the Sensor Setup panel." % args.device)
                print("capturing rgb='%s' seg='%s' shape=%s"
                      % (rgb_key, seg_key, list(frame[rgb_key]["array"].shape)))

            if rgb_key in frame and seg_key in frame:
                seen += 1
                if (seen - 1) % args.stride == 0:           # temporal subsampling
                    saved += 1
                    stem = "frame_%06d" % saved
                    rgb_path = os.path.join(rgb_dir, stem + ".jpg")
                    mask_path = os.path.join(mask_dir, stem + ".png")
                    save_rgb(rgb_path, np.ascontiguousarray(frame[rgb_key]["array"]))
                    save_mask(mask_path, np.ascontiguousarray(frame[seg_key]["array"]))
                    manifest.write(json.dumps({
                        "stem": stem,
                        "rgb": os.path.relpath(rgb_path, args.out),
                        "mask": os.path.relpath(mask_path, args.out),
                        "shape": list(frame[rgb_key]["array"].shape),
                        "observation": slim_obs(res.get("observation")),
                    }) + "\n")
                    if saved % 50 == 0:
                        print("  saved %d / %d" % (saved, args.frames))

            if res.get("terminated") or res.get("truncated"):
                finishes += 1
                print("lap %d/%d complete" % (finishes, args.loops))
                if finishes >= args.loops:
                    break
                c.call("reset")
                c.call("set_sync_mode", {"enabled": True})
                begin_route(c, args)
    finally:
        for m, p in (("route_follow", {"action": "halt"}),
                     ("set_sync_mode", {"enabled": False}),
                     ("disconnect", None)):
            try:
                c.call(m, p)
            except Exception:
                pass
        manifest.close()
        c.close()
        print("done: %d paired frames -> %s" % (saved, args.out))


if __name__ == "__main__":
    main()
