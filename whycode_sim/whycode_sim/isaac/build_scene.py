"""Composes the stage: environment, lighting, robot, camera, and markers."""

import math

import numpy as np
from pxr import Gf, UsdGeom, UsdLux

from whycode_sim import scene_config
from whycode_sim.isaac import markers as marker_builder
from whycode_sim.isaac import usd_utils
from whycode_sim.isaac import robot as robot_builder

WORLD_PATH = "/World"
ENVIRONMENT_PATH = "/World/Environment"


def _assert_stage_units(stage):
    """Fail early if stage units or up-axis are not what the maths assumes.

    A centimetre-scale asset places markers a hundred times too far away, and the symptom
    reads as "the markers did not spawn".
    """
    meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
    if abs(meters_per_unit - 1.0) > 1e-9:
        raise RuntimeError(
            f"stage metersPerUnit is {meters_per_unit}, expected 1.0. Every distance in "
            "scene.yaml is in metres, so a rescaled stage would misplace everything."
        )

    up_axis = UsdGeom.GetStageUpAxis(stage)
    if up_axis != UsdGeom.Tokens.z:
        raise RuntimeError(f"stage up-axis is {up_axis}, expected Z")


def _add_environment(stage, assets_root, config):
    """Reference the warehouse, or lay down a ground plane when none is configured."""
    from isaacsim.core.utils.stage import add_reference_to_stage

    environment = getattr(config.world, "environment", None)
    if not environment:
        # UsdGeom.Plane is visual only; without a collider the robot falls through and
        # surfaces as a camera pose kilometres below the floor, not as an error.
        from isaacsim.core.api.objects.ground_plane import GroundPlane

        GroundPlane(
            prim_path=f"{ENVIRONMENT_PATH}/GroundPlane",
            size=100.0,
            color=np.array([0.35, 0.35, 0.35]),
        )
        return None

    url = assets_root + environment
    add_reference_to_stage(url, ENVIRONMENT_PATH)

    # The robot stays at the origin, so the environment moves instead. Keeps odom,
    # world and the YAML in one frame.
    environment_prim = stage.GetPrimAtPath(ENVIRONMENT_PATH)
    usd_utils.set_translate(environment_prim, config.world.offset)
    yaw_deg = getattr(config.world, "yaw_deg", 0.0)
    if abs(yaw_deg) > 1e-9:
        usd_utils.set_orient(
            environment_prim, Gf.Rotation(Gf.Vec3d(0, 0, 1), float(yaw_deg)).GetQuat()
        )
    return url


def _add_lighting(stage, config):
    """One distant light, matching the flat lighting the Gazebo worlds used.

    Markers are emissive, so this lights the environment only; marker contrast does not
    depend on it.
    """
    light = UsdLux.DistantLight.Define(stage, f"{WORLD_PATH}/DistantLight")
    light.CreateIntensityAttr(1500.0)
    light.CreateAngleAttr(1.0)
    light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
    UsdGeom.Xformable(light).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 30.0))

    # Shadow control lives on ShadowAPI, not the light schema.
    if not getattr(config.render, "shadows", False):
        UsdLux.ShadowAPI.Apply(light.GetPrim()).CreateShadowEnableAttr(False)
    return light


def apply_render_settings(config):
    """Anti-aliasing, shadows and motion blur.

    Never DLSS: it is a temporal upscaler and ghosts a small high-contrast ring crossing
    the frame. Images still look fine, so the damage shows only in the statistics.
    """
    import carb

    settings = carb.settings.get_settings()
    mode = str(getattr(config.render, "antialiasing", "DLAA")).upper()
    settings.set("/rtx/post/aa/op", {"OFF": 0, "FXAA": 1, "DLAA": 3}.get(mode, 3))
    settings.set(
        "/rtx/post/motionblur/enabled",
        bool(getattr(config.render, "motion_blur", False)),
    )
    settings.set("/rtx/shadows/enabled", bool(getattr(config.render, "shadows", False)))
    # Single-GPU host; leaving this on costs startup time for nothing.
    settings.set("/renderer/multiGpu/enabled", False)


def build(stage, config):
    """Populate the stage. Returns a dict describing what was created."""
    from isaacsim.storage.native import get_assets_root_path

    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError(
            "could not reach the Isaac Sim asset root. The first run streams assets from "
            "NVIDIA and needs network access."
        )

    UsdGeom.Xform.Define(stage, WORLD_PATH)
    _assert_stage_units(stage)

    environment_url = _add_environment(stage, assets_root, config)
    _add_lighting(stage, config)

    robot_url = robot_builder.spawn(stage, assets_root, config)
    chassis_path = robot_builder.find_chassis_path(stage, config.robot.chassis_prim)
    camera_path = robot_builder.attach_camera(stage, chassis_path, config)

    resolved = scene_config.resolved_markers(config)
    prim_paths = marker_builder.build(stage, config, resolved)
    marker_builder.verify_placement(stage, resolved, prim_paths)

    print(f"[scene] environment : {environment_url or 'ground plane'}")
    print(f"[scene] robot       : {robot_url}")
    print(f"[scene] chassis     : {chassis_path}")
    print(f"[scene] camera      : {camera_path}")
    print(
        f"[scene] markers     : {len(resolved)} ({config.marker.family}), placement verified"
    )
    for marker in resolved:
        x, y, z = marker.position
        print(
            f"[scene]   id={marker.id:<3} {marker.texture:<16} "
            f"({x:6.2f},{y:6.2f},{z:5.2f}) yaw={math.degrees(marker.yaw_rad):7.2f} deg"
        )

    return {
        "assets_root": assets_root,
        "chassis_path": chassis_path,
        "camera_path": camera_path,
        "markers": resolved,
        "marker_prim_paths": prim_paths,
    }
