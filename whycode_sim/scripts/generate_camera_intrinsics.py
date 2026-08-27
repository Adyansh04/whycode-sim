#!/usr/bin/env python3
"""Derive whycode_vision's camera intrinsics YAML from the sim's scene config.

The detector reads intrinsics from a hardcoded file in another repository. If resolution
or field of view changes here and that file does not, every reported pose is wrong by a
scale factor and reads as a detector accuracy problem.

Prints what the current scene config implies and diffs it against the detector's file.
Writing across repositories is left to a separate change there.

    python3 generate_camera_intrinsics.py                 # compare against the detector
    python3 generate_camera_intrinsics.py --write out.yaml
"""

import argparse
import sys
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_DIR))

from whycode_sim import scene_config  # noqa: E402
from whycode_sim.geometry import camera_intrinsics  # noqa: E402

DEFAULT_SCENE = PACKAGE_DIR / "config" / "scene.yaml"
DETECTOR_INTRINSICS = (
    PACKAGE_DIR.parent.parent
    / "whycode_vision"
    / "config"
    / "camera_intrinsics_sim.yaml"
)

# Below this is float formatting, not disagreement: fx is derived from the FOV in double
# precision here, while the detector's file was written out once.
TOLERANCE_PX = 1e-3


def build(config):
    camera = config.camera
    fx, fy, cx, cy = camera_intrinsics(camera.width, camera.height, camera.hfov_deg)
    return fx, fy, cx, cy, camera.width, camera.height, camera.frame_id


def render(fx, fy, cx, cy, width, height, frame_id):
    return f"""image_width: {width}
image_height: {height}
camera_name: {frame_id}
camera_matrix:
  rows: 3
  cols: 3
  data: [{fx}, 0.0, {cx}, 0.0, {fy}, {cy}, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0, 0.0, 0.0, 0.0, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [{fx}, 0.0, {cx}, 0.0, 0.0, {fy}, {cy}, 0.0, 0.0, 0.0, 1.0, 0.0]
binning_x: 0
binning_y: 0
roi:
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: false
"""


def compare(fx, cx, cy, width, height, path):
    """Report whether the detector's file matches. Returns True when it does."""
    if not path.exists():
        print(f"detector intrinsics not found at {path}")
        return False

    import yaml

    with path.open() as handle:
        existing = yaml.safe_load(handle)

    matrix = existing["camera_matrix"]["data"]
    checks = [
        ("image_width", existing["image_width"], width, 0),
        ("image_height", existing["image_height"], height, 0),
        ("fx", matrix[0], fx, TOLERANCE_PX),
        ("fy", matrix[4], fx, TOLERANCE_PX),
        ("cx", matrix[2], cx, TOLERANCE_PX),
        ("cy", matrix[5], cy, TOLERANCE_PX),
    ]

    agreed = True
    print(f"{'field':<14}{'detector':>22}{'from scene.yaml':>22}{'':>4}")
    for name, have, want, tolerance in checks:
        ok = abs(have - want) <= tolerance
        agreed &= ok
        print(f"{name:<14}{have:>22}{want:>22}  {'ok' if ok else 'MISMATCH'}")
    return agreed


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", default=str(DEFAULT_SCENE))
    parser.add_argument("--write", metavar="PATH", help="write the derived YAML here")
    parser.add_argument("--detector", default=str(DETECTOR_INTRINSICS))
    args = parser.parse_args(argv)

    config = scene_config.load(args.scene)
    fx, fy, cx, cy, width, height, frame_id = build(config)
    print(f"scene    : {args.scene}")
    print(f"camera   : {width}x{height} at {config.camera.hfov_deg} deg horizontal FOV")
    print(f"derived  : fx=fy={fx:.8f}  cx={cx}  cy={cy}\n")

    if args.write:
        Path(args.write).write_text(render(fx, fy, cx, cy, width, height, frame_id))
        print(f"wrote {args.write}")
        return 0

    agreed = compare(fx, cx, cy, width, height, Path(args.detector))
    print()
    if agreed:
        print("detector intrinsics match the scene config")
        return 0
    print(
        "detector intrinsics do NOT match. Update\n"
        f"  {args.detector}\n"
        "in the whycode_vision repository -- that is a separate change with its own "
        "review, so this script will not write it for you. Use --write to produce the "
        "file, then apply it there."
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
