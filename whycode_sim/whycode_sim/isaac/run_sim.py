"""Isaac Sim entry point for the WhyCode benchmark scene.

Launched by scripts/isaac_entrypoint.sh through Isaac's ./python.sh. SimulationApp must
be constructed before anything else in Kit is importable, hence the staged imports.

    ./python.sh run_sim.py                     headless, scene from SCENE_CONFIG
    ./python.sh run_sim.py --no-headless       native GUI, for checking marker placement
    ./python.sh run_sim.py --introspect        print robot joints and intrinsics, then exit
    ./python.sh run_sim.py --scene <path>      use a different layout
"""

import argparse
import os
import sys
from pathlib import Path

# The ament_python package root, which is what has to be on sys.path for
# `whycode_sim.*` to import identically in both containers.
PACKAGE_DIR = Path(__file__).resolve().parents[2]
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

DEFAULT_SCENE = PACKAGE_DIR / "config" / "scene.yaml"


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scene",
        default=os.environ.get("SCENE_CONFIG", str(DEFAULT_SCENE)),
        help="scene config to load (default: $SCENE_CONFIG)",
    )
    headless = parser.add_mutually_exclusive_group()
    headless.add_argument("--headless", dest="headless", action="store_true")
    headless.add_argument("--no-headless", dest="headless", action="store_false")
    parser.set_defaults(headless=None)
    parser.add_argument(
        "--livestream",
        action="store_true",
        help="headless plus a WebRTC viewport, reachable on the host at port 8211",
    )
    parser.add_argument(
        "--introspect",
        action="store_true",
        help="print the robot's joints and the derived intrinsics, then exit",
    )
    parser.add_argument(
        "--survey",
        action="store_true",
        help="print a floor occupancy map of the environment and a suggested loop, then exit",
    )
    parser.add_argument(
        "--survey-bounds",
        nargs=4,
        type=float,
        metavar=("X0", "Y0", "X1", "Y1"),
        help="restrict --survey to this window, e.g. the building interior",
    )
    parser.add_argument(
        "--survey-marker-offset",
        type=float,
        default=0.0,
        help="require this much clearance on BOTH sides of the loop, for side markers",
    )
    parser.add_argument(
        "--suggest-markers",
        type=int,
        default=0,
        help="propose N marker placements that fit along the configured loop, then exit",
    )
    parser.add_argument(
        "--marker-offset",
        type=float,
        default=0.8,
        help="lateral offset to test when suggesting marker placements",
    )
    parser.add_argument(
        "--verify-path",
        action="store_true",
        help="check the configured loop and marker positions for clearance, then exit",
    )
    parser.add_argument(
        "--probe-nodes",
        action="store_true",
        help="print the attribute names of every OmniGraph node type used, then exit",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="stop after this many rendered frames (0 runs until interrupted)",
    )
    return parser.parse_args(argv)


def resolve_headless(args, config):
    """Command line wins, then ISAAC_HEADLESS, then the scene file."""
    if args.livestream:
        return True
    if args.headless is not None:
        return args.headless

    from_env = os.environ.get("ISAAC_HEADLESS")
    if from_env is not None:
        return from_env.strip().lower() not in ("0", "false", "no", "x11")
    return bool(getattr(config, "headless", True))


def main(argv=None):
    args = parse_args(argv)

    # scene_config needs only yaml and numpy, so a bad config fails in a second rather
    # than after a minute of stage loading.
    from whycode_sim import scene_config

    config = scene_config.load(args.scene)
    headless = resolve_headless(args, config)

    print(f"[run_sim] scene    : {args.scene}")
    print(f"[run_sim] headless : {headless}{' (webrtc)' if args.livestream else ''}")

    from isaacsim import SimulationApp

    simulation_app = SimulationApp(
        {
            "headless": headless,
            "width": int(config.camera.width),
            "height": int(config.camera.height),
        }
    )

    # Kit's fast shutdown swallows exceptions raised here, turning a failure into a
    # silent exit 0. Report before teardown.
    status = 0
    try:
        _run(simulation_app, args, config)
    except Exception:
        import traceback

        print("[run_sim] FAILED", file=sys.stderr)
        traceback.print_exc()
        sys.stderr.flush()
        status = 1
    finally:
        simulation_app.close()
    return status


def _run(simulation_app, args, config):
    import omni.usd
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.ros2.bridge")
    if args.livestream:
        enable_extension("omni.services.streamclient.webrtc")
    simulation_app.update()

    from whycode_sim.isaac import build_scene, robot, ros_graph, sim_publishers

    build_scene.apply_render_settings(config)

    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()

    scene = build_scene.build(stage, config)
    simulation_app.update()

    if args.introspect:
        robot.describe(stage, config)
        return

    if args.probe_nodes:
        ros_graph.probe_node_types()
        return

    if args.survey:
        from whycode_sim.isaac import survey

        survey.survey(
            stage,
            bounds=args.survey_bounds,
            marker_offset_m=args.survey_marker_offset,
        )
        return

    if args.suggest_markers:
        from whycode_sim.isaac import survey

        survey.deepest_aisle_point(stage, config)
        survey.suggest_markers(
            stage, config, count=args.suggest_markers, offset_m=args.marker_offset
        )
        return

    if args.verify_path:
        from whycode_sim.isaac import survey

        radius = float(config.robot.wheel_distance_m) / 2.0 + 0.1
        path_ok = survey.verify_path(
            stage,
            [list(point) for point in config.path.waypoints],
            robot_radius=radius,
            loop=getattr(config.path, "loop", True),
            label="loop",
        )
        markers_ok = survey.verify_markers(stage, scene["markers"])
        survey.predict_visibility(config, scene["markers"])
        ids_ok = survey.verify_shared_ids(config, scene["markers"])
        print(
            f"[verify] result: path {'clear' if path_ok else 'BLOCKED'}, "
            f"markers {'clear' if markers_ok else 'OBSTRUCTED'}, "
            f"shared ids {'ok' if ids_ok else 'AMBIGUOUS'}"
        )
        return

    joints = _resolve_drive_joints(stage, scene)
    ros_graph.wire(config, scene, joints)
    simulation_app.update()

    import rclpy

    rclpy.init()
    publishers = sim_publishers.SimPublishers(config, scene)

    from isaacsim.core.api import SimulationContext

    # The render period is quantised to whole physics steps, so a physics rate that does
    # not divide the camera rate shifts the real frame rate: 200 Hz physics with a 30 Hz
    # camera renders at 33.3 Hz. Derive the step from the camera rate instead.
    fps = float(config.camera.fps)
    physics_substeps = 8
    simulation_context = SimulationContext(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / (fps * physics_substeps),
        rendering_dt=1.0 / fps,
    )
    print(f"[run_sim] {fps:g} Hz camera, {fps * physics_substeps:g} Hz physics")
    simulation_context.play()
    simulation_app.update()

    print("[run_sim] simulation running; publishing the ROS 2 topic set")
    frames = 0
    try:
        while simulation_app.is_running():
            simulation_context.step(render=True)
            publishers.publish(stage, simulation_context.current_time)
            publishers.spin_once()

            frames += 1
            if args.max_frames and frames >= args.max_frames:
                print(f"[run_sim] reached --max-frames={args.max_frames}")
                break
    except KeyboardInterrupt:
        print("[run_sim] interrupted")
    finally:
        simulation_context.stop()
        publishers.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _resolve_drive_joints(stage, scene):
    """Wheel joint names for the differential controller.

    Matched by pattern, since names differ between asset versions. None leaves /cmd_vel
    unwired and is reported rather than fatal: the scene is still useful statically.
    """
    from pxr import Usd, UsdPhysics

    left, right = [], []
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Robot")):
        if not prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        name = prim.GetName().lower()
        if "wheel" not in name:
            continue
        if "left" in name:
            left.append(prim.GetName())
        elif "right" in name:
            right.append(prim.GetName())

    if not left or not right:
        print(
            "[run_sim] could not identify left/right wheel joints; /cmd_vel will not be "
            "wired. Run with --introspect to list the articulation's joints."
        )
        return None

    joints = [left[0], right[0]]
    print(f"[run_sim] drive joints: {joints}")
    return joints


if __name__ == "__main__":
    raise SystemExit(main())
