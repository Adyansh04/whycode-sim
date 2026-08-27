"""Decode every WhyCode texture with the detector's own necklace implementation.

The bit pattern is sampled from the image; decoding is done by the compiled CNecklace from
whycode_vision, so the answer is the id the detector will report.

The sampling phase is swept across all 60 offsets within a half-bit segment and the
consensus taken. A necklace code is rotation-invariant, so a correct read is stable across
phase; only a phase landing on a black/white transition should disagree.
"""

import subprocess
import sys
from collections import Counter
from pathlib import Path

import numpy as np
from PIL import Image

ID_SAMPLES = 720
ID_BITS = 6
HAMMING = 1
SEGMENT_WIDTH = ID_SAMPLES // ID_BITS // 2  # 60
RING_RADIUS_FRAC = 0.23  # centre of the teeth annulus, measured


def ring_bits(path, radius_frac=RING_RADIUS_FRAC):
    gray = np.asarray(Image.open(path).convert("L"), dtype=np.float32)
    h, w = gray.shape
    cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
    radius = radius_frac * w
    angles = np.arange(ID_SAMPLES) * (2.0 * np.pi / ID_SAMPLES)
    xs = np.clip((cx + radius * np.cos(angles)).astype(int), 0, w - 1)
    ys = np.clip((cy + radius * np.sin(angles)).astype(int), 0, h - 1)
    signal = gray[ys, xs]
    level = (signal.min() + signal.max()) / 2.0
    return (signal > level).astype(int)


def codes_for(bits):
    """The 2*ID_BITS sample string, for every phase within a segment."""
    out = []
    for phase in range(SEGMENT_WIDTH):
        idx = [(phase + a * SEGMENT_WIDTH) % ID_SAMPLES for a in range(ID_BITS * 2)]
        out.append("".join(str(bits[i]) for i in idx))
    return out


def decode(codes):
    result = subprocess.run(
        ["./decode_main", str(ID_BITS), str(ID_SAMPLES), str(HAMMING), *codes],
        capture_output=True,
        text=True,
        check=True,
    )
    ids = []
    for line in result.stdout.strip().splitlines():
        ids.append(int(line.split("-> id ")[1].split(" ")[0]))
    return ids


def main(paths):
    print(
        f"{'texture':<16}{'consensus id':>13}{'agree':>8}{'distinct codes':>16}  minority"
    )
    mapping = {}
    for path in paths:
        bits = ring_bits(path)
        codes = codes_for(bits)
        ids = decode(codes)
        counts = Counter(ids)
        winner, votes = counts.most_common(1)[0]
        minority = {k: v for k, v in counts.items() if k != winner}
        name = Path(path).stem
        mapping[name] = winner
        print(
            f"{name:<16}{winner:>13}{votes:>4}/{len(ids):<3}"
            f"{len(set(codes)):>16}  {minority or '-'}"
        )
    print()
    print("texture -> decoded id:", {k: v for k, v in mapping.items()})
    return mapping


if __name__ == "__main__":
    base = Path(__file__).resolve().parents[2] / "textures" / "whycode"
    files = sorted(base.glob("whycode_*.png"), key=lambda p: int(p.stem.split("_")[1]))
    main([str(f) for f in files] if len(sys.argv) == 1 else sys.argv[1:])
