"""Loader for scene.yaml, the single source of truth for the simulation.

Read by the Isaac scene builder, the ground-truth node and the waypoint follower, so
placement is computed once and cannot drift. Same import rule as geometry: stdlib, yaml
and numpy only.

    python3 -m whycode_sim.scene_config <scene.yaml>
"""

import math
from pathlib import Path
from types import SimpleNamespace

import yaml

from whycode_sim.geometry import marker_normal_yaw, path_segment, place_marker

MARKER_FAMILIES = ("whycode", "apriltag")


def _namespace(value):
    """Recursively turn parsed YAML into attribute-access objects."""
    if isinstance(value, dict):
        return SimpleNamespace(**{key: _namespace(item) for key, item in value.items()})
    if isinstance(value, list):
        return [_namespace(item) for item in value]
    return value


def load(path):
    """Load and validate a scene config."""
    path = Path(path)
    with path.open() as handle:
        raw = yaml.safe_load(handle)

    config = _namespace(raw)
    config.path_on_disk = path
    config.package_root = path.parent.parent
    _validate(config)
    return config


def _validate(config):
    if config.marker.family not in MARKER_FAMILIES:
        raise ValueError(
            f"marker.family must be one of {MARKER_FAMILIES}, got {config.marker.family!r}"
        )
    if len(config.path.waypoints) < 2:
        raise ValueError("path.waypoints needs at least two points")
    if config.camera.width <= 0 or config.camera.height <= 0:
        raise ValueError("camera.width and camera.height must be positive")
    if not 0.0 < config.camera.hfov_deg < 180.0:
        raise ValueError(f"camera.hfov_deg out of range: {config.camera.hfov_deg}")
    if config.camera.fps <= 0:
        raise ValueError("camera.fps must be positive")

    # Ids come from the family table, so uniqueness is a property of that table; these
    # entries describe geometry only.
    for index, marker in enumerate(config.markers):
        if not hasattr(marker, "position") and not hasattr(marker, "at_waypoint"):
            raise ValueError(
                f"marker slot {index} needs either an explicit `position` or an `at_waypoint`"
            )
        if hasattr(marker, "side") and marker.side not in ("left", "right"):
            raise ValueError(f"marker slot {index}: side must be 'left' or 'right'")
        if hasattr(marker, "at_waypoint"):
            _validate_along(config, marker, index)

    ids = [entry[1] for entry in family_table(config)]
    if len(set(ids)) != len(ids):
        raise ValueError(f"family {config.marker.family!r} lists duplicate ids: {ids}")


def _validate_along(config, marker, index):
    """Reject an along_m running past the end of its segment.

    Overshooting places the marker past the corner with a yaw from the wrong segment,
    which reads plausibly in the config and wrong in the scene.
    """
    waypoints = [list(point) for point in config.path.waypoints]
    _, _, length = path_segment(
        waypoints, marker.at_waypoint, getattr(config.path, "loop", True)
    )
    along = getattr(marker, "along_m", 0.0)
    if not -1e-9 <= along <= length + 1e-9:
        raise ValueError(
            f"marker slot {index}: along_m={along} runs off segment {marker.at_waypoint}, "
            f"which is {length:.2f} m long. Use the next waypoint index instead."
        )


def texture_dir(config, family=None):
    """Directory holding the texture set for a marker family."""
    return config.package_root / "textures" / (family or config.marker.family)


def family_table(config):
    """(texture, decoded_id) pairs for the active family, in slot order.

    Geometry is family-independent: a slot says where a marker stands and which way it
    faces, the family says what is printed on it and what a detector decodes. That makes
    `marker.family: apriltag` a one-line swap leaving the run otherwise identical.
    """
    families = getattr(config.marker, "families", None)
    if families is None:
        raise ValueError("scene config needs a marker.families table")
    family = getattr(families, config.marker.family, None)
    if family is None:
        raise ValueError(
            f"marker.families has no entry for {config.marker.family!r}; "
            f"present: {sorted(vars(families))}"
        )
    if len(family.textures) != len(family.ids):
        raise ValueError(
            f"family {config.marker.family!r} lists {len(family.textures)} textures but "
            f"{len(family.ids)} ids; they are parallel and must match"
        )
    return list(zip(family.textures, family.ids))


def texture_path(config, texture, family=None):
    """Absolute path to a texture file."""
    return texture_dir(config, family) / f"{texture}.png"


def resolved_markers(config):
    """Every marker with its world position and face yaw worked out.

    Placement comes from the path unless the entry gives an explicit `position` (and
    optionally `yaw_deg`). Returns entries of id, texture, texture_path, family,
    position, yaw_rad, size_m.
    """
    waypoints = [list(point) for point in config.path.waypoints]
    loop = getattr(config.path, "loop", True)
    default_height = config.marker.height_m
    default_facing = getattr(config.marker, "facing_angle_deg", 45.0)

    table = family_table(config)
    # Reusing an id needs an explicit `slot`; without one an entry takes the table row at
    # its own index, so more entries than rows is an overflow rather than a choice. Two
    # markers sharing an id must never be in frame together, which `run_sim.py
    # --verify-path` checks.
    implicit = sum(1 for marker in config.markers if not hasattr(marker, "slot"))
    if implicit > len(table):
        raise ValueError(
            f"the layout has {implicit} markers without an explicit `slot` but family "
            f"{config.marker.family!r} supplies only {len(table)} distinct markers. "
            "Give the extra entries a `slot` to reuse an id deliberately, or add textures."
        )

    resolved = []
    for slot, marker in enumerate(config.markers):
        row = getattr(marker, "slot", slot)
        if not 0 <= row < len(table):
            raise ValueError(
                f"marker entry {slot}: slot {row} is outside family "
                f"{config.marker.family!r}, which has {len(table)} entries"
            )
        texture, decoded_id = table[row]
        height = getattr(marker, "height_m", default_height)

        if hasattr(marker, "position"):
            position = list(marker.position)
            if len(position) == 2:
                position = [position[0], position[1], height]
            yaw_rad = math.radians(getattr(marker, "yaw_deg", 0.0))
        else:
            facing = getattr(marker, "facing_deg", default_facing)
            position, heading = place_marker(
                waypoints,
                marker.at_waypoint,
                getattr(marker, "along_m", 0.0),
                marker.side,
                marker.offset_m,
                height,
                loop,
            )
            yaw_rad = marker_normal_yaw(heading, marker.side, facing)

        resolved.append(
            SimpleNamespace(
                id=decoded_id,
                texture=texture,
                texture_path=texture_path(config, texture),
                family=config.marker.family,
                position=list(position),
                yaw_rad=yaw_rad,
                size_m=getattr(marker, "size_m", config.marker.size_m),
            )
        )
    return resolved


def main(argv=None):
    import sys

    argv = sys.argv[1:] if argv is None else argv
    if not argv:
        print(f"usage: {Path(__file__).name} <scene.yaml>")
        return 1

    config = load(argv[0])
    fps, camera = config.camera.fps, config.camera
    print(f"world      : {config.world.environment}")
    print(
        f"camera     : {camera.width}x{camera.height} @ {fps} Hz, {camera.hfov_deg} deg hfov"
    )
    print(f"family     : {config.marker.family}  (size {config.marker.size_m} m)")
    print(f"waypoints  : {len(config.path.waypoints)}, loop={config.path.loop}")
    print(f"markers    : {len(config.markers)}")
    for marker in resolved_markers(config):
        x, y, z = marker.position
        exists = "ok" if marker.texture_path.exists() else "MISSING TEXTURE"
        print(
            f"  id={marker.id:<3} {marker.texture:<16} "
            f"pos=({x:6.2f},{y:6.2f},{z:5.2f}) yaw={math.degrees(marker.yaw_rad):7.2f} deg  {exists}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
