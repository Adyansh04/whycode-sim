"""Nova Carter and the benchmark camera prim.

Stock NVIDIA asset. Its built-in Hawk stereo is ignored in favour of our own camera,
because the detector's intrinsics are hardcoded in whycode_vision and must match exactly.
"""

import math

from pxr import Gf, UsdGeom

from whycode_sim.geometry import (
    camera_intrinsics,
    matrix_to_quaternion,
    usd_camera_aperture,
    usd_camera_rotation,
)
from whycode_sim.isaac import usd_utils

ROBOT_PATH = "/World/Robot"
# The prim name becomes the TF child frame, and the image is stamped with the frame from
# scene.yaml. Matching them lets RViz resolve the camera.
CAMERA_NAME = "camera_link"

# The asset moved between Isaac releases; resolve at runtime rather than trust one
# version's docs.
ROBOT_CANDIDATES = (
    "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd",
    "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter_sensors.usd",
    "/Isaac/Robots/Carter/nova_carter.usd",
    "/Isaac/Robots/Carter/nova_carter/nova_carter.usd",
    "/Isaac/Robots/Carter/carter_v2.usd",
)


def resolve_robot_usd(assets_root, configured_path=None):
    """First candidate that exists under the asset root.

    Fails with the candidate list rather than surfacing later as an empty stage.
    """
    import omni.client

    candidates = ([configured_path] if configured_path else []) + list(ROBOT_CANDIDATES)
    tried = []
    for candidate in candidates:
        url = assets_root + candidate
        tried.append(url)
        result, _ = omni.client.stat(url)
        if result == omni.client.Result.OK:
            return url

    raise FileNotFoundError(
        "could not locate the Nova Carter asset. Tried:\n  " + "\n  ".join(tried)
    )


def spawn(stage, assets_root, config):
    """Reference the robot at the world origin.

    The environment is shifted around it instead, so odom and world coincide and no
    offset threads through the follower, the ground truth or the bag.
    """
    from isaacsim.core.utils.stage import add_reference_to_stage

    usd_path = resolve_robot_usd(assets_root, getattr(config.robot, "usd_path", None))
    print(f"[robot] asset: {usd_path}")
    add_reference_to_stage(usd_path, ROBOT_PATH)

    usd_utils.set_translate(stage.GetPrimAtPath(ROBOT_PATH), (0.0, 0.0, 0.0))
    return usd_path


def find_chassis_path(stage, hint):
    """Prim path of the chassis link, searched by name under the robot."""
    from pxr import Usd

    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PATH)):
        if prim.GetName() == hint:
            return str(prim.GetPath())

    # Fall back to the first articulation-bearing prim, so a renamed link is not fatal.
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PATH)):
        if prim.HasAPI("PhysicsArticulationRootAPI"):
            return str(prim.GetPath())
    raise RuntimeError(f"no prim named {hint!r} under {ROBOT_PATH}")


def attach_camera(stage, chassis_path, config):
    """Create the benchmark camera on the chassis and return its prim path.

    Intrinsics derive from width/height/hfov in the scene config, so a resolution change
    propagates rather than leaving a stale focal length.
    """
    camera = config.camera
    camera_path = f"{chassis_path}/{CAMERA_NAME}"

    prim = UsdGeom.Camera.Define(stage, camera_path)
    focal_length, horizontal_aperture, vertical_aperture = usd_camera_aperture(
        camera.width, camera.height, camera.hfov_deg
    )
    prim.CreateFocalLengthAttr(float(focal_length))
    prim.CreateHorizontalApertureAttr(float(horizontal_aperture))
    prim.CreateVerticalApertureAttr(float(vertical_aperture))
    prim.CreateClippingRangeAttr(
        Gf.Vec2f(float(camera.near_clip), float(camera.far_clip))
    )
    prim.CreateProjectionAttr(UsdGeom.Tokens.perspective)

    # The mount is in the body frame (X fwd, Y left, Z up) but a USD camera looks down
    # local -Z. Use the same basis-vector helper as the ground-truth maths: hand-composed
    # Euler angles here point the camera down a different world axis, and every
    # ground-truth pose then comes out permuted.
    roll, pitch, yaw = (math.radians(angle) for angle in camera.mount.rpy_deg)
    basis = usd_camera_rotation(yaw, pitch, roll)
    qx, qy, qz, qw = matrix_to_quaternion(basis)

    usd_utils.set_translate(prim.GetPrim(), camera.mount.position)
    usd_utils.set_orient(
        prim.GetPrim(), Gf.Quatd(float(qw), float(qx), float(qy), float(qz))
    )

    return camera_path


def _measure_drive_geometry(stage, joint_paths):
    """Wheel radius and track width, measured off the stage.

    Both feed the differential controller and differ between asset versions, so read them
    from geometry rather than documentation.
    """
    from pxr import Usd, UsdGeom

    wheels = {}
    for path in joint_paths:
        name = path.rsplit("/", 1)[-1].lower()
        if "wheel" not in name:
            continue
        side = "left" if "left" in name else "right" if "right" in name else None
        if side:
            wheels[side] = path

    if len(wheels) != 2:
        print("  drive geometry: could not identify both wheel joints")
        return

    # Match the wheel bodies, not a name derived from the joint: the two are not named
    # consistently across versions. They are Xforms with mesh children, so bounds must
    # cover the subtree rather than a single gprim.
    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    centres, radii = {}, {}
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PATH)):
        name = prim.GetName().lower()
        if "wheel" not in name or "caster" in name or "swing" in name:
            continue
        if not prim.IsA(UsdGeom.Xformable):
            continue
        side = "left" if "left" in name else "right" if "right" in name else None
        if side is None or side in centres:
            continue

        box = bounds.ComputeWorldBound(prim).ComputeAlignedBox()
        if box.IsEmpty():
            continue
        size = box.GetSize()
        # A wheel is widest in the plane it rolls in; the radius is half of that.
        radii[side] = max(size[0], size[2]) / 2.0
        centres[side] = box.GetMidpoint()

    if len(centres) != 2:
        candidates = [
            f"{prim.GetPath()} ({prim.GetTypeName()})"
            for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PATH))
            if "wheel" in prim.GetName().lower()
        ]
        print(f"  prims matching 'wheel' ({len(candidates)}):")
        for candidate in candidates[:20]:
            print(f"    {candidate}")

    if len(centres) == 2:
        # A wheel on the floor has its centre one radius up, and track width is the
        # lateral gap between centres. Both beat bounding-box extents, which also
        # swallow the suspension.
        radius = (centres["left"][2] + centres["right"][2]) / 2.0
        separation = abs(centres["left"][1] - centres["right"][1])
        extent_radius = sum(radii.values()) / 2.0

        print(
            f"  measured wheel_radius_m   : {radius:.4f}   (centre height above the floor)"
        )
        print(
            f"  measured wheel_distance_m : {separation:.4f}   (lateral centre separation)"
        )
        if abs(extent_radius - radius) > 0.02:
            print(
                f"  note: the wheel bounding box implies r={extent_radius:.4f}, which is wider "
                "than the centre height. The box includes mounting hardware, so the centre "
                "height is the one to trust."
            )
        print(
            "  put these in scene.yaml under robot: if they differ from the configured values"
        )
    else:
        print("  drive geometry: wheel meshes not found; keeping the configured values")


def describe(stage, config):
    """Print the articulation's joints and the derived intrinsics.

    Wheel radius, track width and joint names all differ between asset versions and are
    all values the controller needs.
    """
    from pxr import Usd, UsdPhysics

    print(f"robot root: {ROBOT_PATH}")
    joints = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PATH)):
        if prim.IsA(UsdPhysics.RevoluteJoint):
            joints.append(str(prim.GetPath()))
        if prim.HasAPI("PhysicsArticulationRootAPI"):
            print(f"  articulation root: {prim.GetPath()}")

    print(f"  revolute joints ({len(joints)}):")
    for joint in joints:
        print(f"    {joint}")

    _measure_drive_geometry(stage, joints)

    fx, fy, cx, cy = camera_intrinsics(
        config.camera.width, config.camera.height, config.camera.hfov_deg
    )
    print(
        f"  camera {config.camera.width}x{config.camera.height} @ {config.camera.hfov_deg} deg -> "
        f"fx={fx:.8f} fy={fy:.8f} cx={cx} cy={cy}"
    )
    print("  compare against whycode_vision/config/camera_intrinsics_sim.yaml")
