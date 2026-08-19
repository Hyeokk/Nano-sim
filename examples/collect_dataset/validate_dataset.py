#!/usr/bin/env python3
"""
Validate a dataset produced by collect_dataset_unified.py.

Checks, without needing the simulator:
  - manifest / rgb / mask counts agree
  - a sample RGB decodes to the expected HxWx3
  - a sample mask decodes and every colour maps to a class in classes.json
  - sensors.npz: lidar_ranges / imu / gnss shapes, plus the point-cloud object
    array (per-frame structured x/y/z/intensity arrays), with basic physical sanity
    (IMU gravity ~9.81, lidar ranges within [0, max], point counts > 0)

Usage:
  python validate_dataset.py --ds test_ds
"""

import argparse
import glob
import json
import os

import numpy as np


def rgb_of(c):
    return tuple(int(v) for v in c["rgb"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ds", default="test_ds")
    ap.add_argument("--sample", type=int, default=-1, help="frame index to inspect (default: middle)")
    args = ap.parse_args()

    ds = args.ds
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok = ok and cond
        print(("  PASS " if cond else "  FAIL ") + name + (("  -> " + detail) if detail else ""))

    print("dataset:", ds)

    # ---- counts ----
    rgbs = sorted(glob.glob(os.path.join(ds, "rgb", "*.jpg")))
    masks = sorted(glob.glob(os.path.join(ds, "mask", "*.png")))
    manifest = []
    with open(os.path.join(ds, "manifest.jsonl")) as f:
        for line in f:
            line = line.strip()
            if line:
                manifest.append(json.loads(line))
    print("counts: rgb=%d mask=%d manifest=%d" % (len(rgbs), len(masks), len(manifest)))
    check("rgb/mask/manifest counts agree", len(rgbs) == len(masks) == len(manifest) and len(rgbs) > 0)

    with open(os.path.join(ds, "classes.json")) as f:
        classes = json.load(f)
    legend = {rgb_of(c): c["name"] for c in classes}

    idx = args.sample if args.sample >= 0 else len(rgbs) // 2
    idx = max(0, min(idx, len(rgbs) - 1))
    print("inspecting frame index", idx)

    # ---- image decode ----
    try:
        import cv2
        rgb = cv2.cvtColor(cv2.imread(rgbs[idx], cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
        mask = cv2.cvtColor(cv2.imread(masks[idx], cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
    except Exception:
        from PIL import Image
        rgb = np.array(Image.open(rgbs[idx]).convert("RGB"))
        mask = np.array(Image.open(masks[idx]).convert("RGB"))

    check("rgb decodes to HxWx3", rgb.ndim == 3 and rgb.shape[2] == 3, str(rgb.shape))
    exp = manifest[idx].get("shape")
    if exp:
        check("rgb shape matches manifest", list(rgb.shape) == list(exp), "%s vs %s" % (list(rgb.shape), exp))

    # ---- mask colours vs legend ----
    cols, counts = np.unique(mask.reshape(-1, 3), axis=0, return_counts=True)
    known, unknown = [], []
    for c, n in zip(cols, counts):
        t = tuple(int(v) for v in c)
        (known if t in legend else unknown).append((t, int(n)))
    print("mask colours: %d known, %d unknown" % (len(known), len(unknown)))
    for t, n in sorted(known, key=lambda x: -x[1])[:6]:
        print("    %-16s %s  (%d px)" % (legend[t], t, n))
    if unknown:
        for t, n in sorted(unknown, key=lambda x: -x[1])[:6]:
            print("    UNKNOWN %s  (%d px)" % (t, n))
    # Mask PNG must be lossless: no stray colours outside the legend.
    check("all mask colours are in classes.json (lossless PNG)", len(unknown) == 0)

    # ---- sensors.npz ----
    npz_path = os.path.join(ds, "sensors.npz")
    if os.path.exists(npz_path):
        d = np.load(npz_path, allow_pickle=True)
        print("sensors.npz keys:", list(d.keys()))
        n = len(manifest)

        if "lidar_ranges" in d:
            lr = d["lidar_ranges"]
            print("  lidar_ranges:", lr.shape, lr.dtype, "min=%.3f max=%.3f" % (float(lr.min()), float(lr.max())))
            check("lidar_ranges rows == frames", lr.shape[0] == n)
            check("lidar_ranges finite & >= 0", np.isfinite(lr).all() and (lr >= 0).all())

        if "imu" in d:
            imu = d["imu"]
            print("  imu:", imu.shape, "mean|accel_y|=%.2f" % float(np.mean(np.abs(imu[:, 8]))))
            check("imu shape (n,10)", imu.shape == (n, 10))
            check("imu gravity present (accel ~9.81)", 8.0 < np.mean(np.abs(imu[:, 8])) < 11.0)

        if "gnss" in d:
            g = d["gnss"]
            print("  gnss:", g.shape, "x[%.2f..%.2f] z[%.2f..%.2f]" %
                  (float(g[:, 0].min()), float(g[:, 0].max()), float(g[:, 2].min()), float(g[:, 2].max())))
            check("gnss shape (n,3)", g.shape == (n, 3))
            check("gnss moves over the run", np.ptp(g[:, 0]) + np.ptp(g[:, 2]) > 0.01)

        pcd_keys = [k for k in d.keys() if k.startswith("pcd_")]
        for k in pcd_keys:
            arr = d[k]                      # object array, one structured array per frame
            counts_pc = [len(a) for a in arr]
            first = arr[0]
            print("  %s: %d frames, dtype fields=%s, count[min=%d max=%d], frame0[0]=%s" %
                  (k, len(arr), first.dtype.names, min(counts_pc), max(counts_pc),
                   tuple(round(float(v), 3) for v in first[0])))
            check(k + " has x/y/z/intensity fields", first.dtype.names == ("x", "y", "z", "intensity"))
            check(k + " every frame has points", min(counts_pc) > 0)
            check(k + " frames == manifest", len(arr) == n)
    else:
        print("  (no sensors.npz)")

    print("\nRESULT:", "ALL PASS ✅" if ok else "FAILURES ABOVE ❌")


if __name__ == "__main__":
    main()