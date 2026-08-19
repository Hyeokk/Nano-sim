#!/usr/bin/env python3
"""Nano-sim segmentation label helpers: class legend + colour <-> index.

The collector saves segmentation masks as flat, colour-coded PNGs (one RGB colour
per class). For training you want integer class indices; convert with rgb_to_index.

Index = position in CLASSES below. Colours are the sim's actual (float->byte
truncated) seg render, so exact matching is lossless on a flat / non-AA mask.
Keep CLASSES in sync with the sim's SceneConfig.segCategories (and classes.json).
"""
import numpy as np

CLASSES = [
    ("road",        (128, 64, 128)),
    ("lane",        (0, 255, 0)),
    ("stop",        (255, 0, 0)),
    ("lidar",       (127, 127, 127)),
    ("vehicle",     (0, 0, 142)),
    ("obstacle",    (135, 206, 250)),
    ("adboard",     (69, 69, 69)),
    ("tunnel",      (255, 127, 0)),
    ("nondrivable", (244, 35, 232)),
    ("background",  (0, 0, 0)),
]

NAMES = [n for n, _ in CLASSES]
NUM_CLASSES = len(CLASSES)
BACKGROUND_INDEX = NAMES.index("background")
INDEX_TO_RGB = np.array([rgb for _, rgb in CLASSES], dtype=np.uint8)   # (K, 3)

# packed 24-bit colour -> class index, for a fast vectorised lookup.
_PACKED = {(r << 16) | (g << 8) | b: i for i, (_, (r, g, b)) in enumerate(CLASSES)}


def rgb_to_index(mask_rgb, unmapped_to=BACKGROUND_INDEX):
    """HxWx3 uint8 colour mask -> (HxW uint8 index mask, n_unmapped_pixels).

    On a correct flat/non-AA render n_unmapped is 0. A non-zero count means the
    legend has drifted from the sim, or the seg camera is antialiasing.
    """
    m = np.ascontiguousarray(mask_rgb[..., :3]).astype(np.uint32)
    packed = (m[..., 0] << 16) | (m[..., 1] << 8) | m[..., 2]
    out = np.full(packed.shape, unmapped_to, dtype=np.uint8)
    seen = np.zeros(packed.shape, dtype=bool)
    for key, idx in _PACKED.items():
        hit = packed == key
        out[hit] = idx
        seen |= hit
    return out, int((~seen).sum())


def index_to_rgb(index_mask):
    """HxW index mask -> HxWx3 uint8 colour mask (for visualisation)."""
    return INDEX_TO_RGB[np.asarray(index_mask)]


if __name__ == "__main__":
    # Round-trip check on a solid tile of each class colour.
    for i, (name, rgb) in enumerate(CLASSES):
        tile = np.tile(np.array(rgb, np.uint8), (4, 4, 1))
        idx, un = rgb_to_index(tile)
        assert un == 0 and (idx == i).all(), (name, idx.tolist(), un)
    print("seg_labels OK: %d classes -> %s" % (NUM_CLASSES, NAMES))
