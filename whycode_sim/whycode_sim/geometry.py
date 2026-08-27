"""Frame conversions, marker placement, visibility, and path following.

Imported by both the ROS 2 nodes and the Isaac scripts, which run in different
containers. Imports stdlib and numpy only - no rclpy, pxr, omni or isaacsim - so it stays
loadable from both. If Isaac cannot import it, the scene builder and the ground-truth node
end up with two copies of the placement maths.

Self-checks: python3 -m whycode_sim.geometry
"""

import math

import numpy as np

# USD camera looks down -Z, +Y up, +X right. The detector reports X forward, Y left,
# Z up: whycon_localization.cpp:610-613 permutes the OpenCV optical triple before storing
# it, so /whycon/poses is a body frame, not an optical one. Mismatching this makes every
# comparison an axis permutation.
R_USD_TO_GT = np.array(
    [
        [0.0, 0.0, -1.0],  # X_gt = -Z_usd  (forward)
        [-1.0, 0.0, 0.0],  # Y_gt = -X_usd  (left)
        [0.0, 1.0, 0.0],  # Z_gt = +Y_usd  (up)
    ]
)


def camera_intrinsics(width, height, hfov_deg):
    """Pinhole intrinsics for a centred camera with square pixels.

    Returns (fx, fy, cx, cy). At 1280x720 and 80 degrees this reproduces the values
    hardcoded in whycode_vision/config/camera_intrinsics_sim.yaml.
    """
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    return fx, fx, width / 2.0, height / 2.0


def usd_camera_aperture(width, height, hfov_deg, focal_length=12.486597366034983):
    """USD camera attributes yielding `camera_intrinsics` for the same arguments.

    USD defines fx = width * focalLength / horizontalAperture, so only the ratio matters.
    The default keeps horizontalAperture near Isaac's own 20.955.

    Returns (focal_length, horizontal_aperture, vertical_aperture).
    """
    fx, _, _, _ = camera_intrinsics(width, height, hfov_deg)
    horizontal_aperture = width * focal_length / fx
    return focal_length, horizontal_aperture, horizontal_aperture * height / width


def yaw_to_normal(yaw_rad):
    """Unit horizontal vector a marker with this yaw faces along."""
    return np.array([math.cos(yaw_rad), math.sin(yaw_rad), 0.0])


def marker_normal_yaw(path_heading_rad, side, facing_angle_deg):
    """Yaw of a marker's face normal, from path heading and which side it sits on.

    `facing_angle_deg` is the incidence a robot sees approaching from far up the path:
    45 angles the marker toward oncoming traffic, 90 sets it square across.
    """
    turn = math.radians(180.0 - facing_angle_deg)
    return path_heading_rad - turn if side == "left" else path_heading_rad + turn


def path_segment(waypoints, index, loop=True):
    """Start point, unit direction, and length of the segment leaving `index`."""
    points = np.asarray(waypoints, dtype=float)
    count = len(points)
    end_index = (index + 1) % count if loop else min(index + 1, count - 1)
    start, end = points[index % count], points[end_index]
    delta = end - start
    length = float(np.linalg.norm(delta))
    if length < 1e-9:
        raise ValueError(f"waypoints {index} and {end_index} are coincident")
    return start, delta / length, length


def place_marker(waypoints, at_waypoint, along_m, side, offset_m, height_m, loop=True):
    """World position of a marker placed relative to the path, and the path heading there.

    Returns (position_xyz, path_heading_rad). Position is `along_m` down the segment
    leaving `at_waypoint`, then `offset_m` to the given side of the centreline.
    """
    start, direction, _ = path_segment(waypoints, at_waypoint, loop)
    heading = math.atan2(direction[1], direction[0])
    lateral_sign = 1.0 if side == "left" else -1.0
    lateral = np.array([-direction[1], direction[0]]) * offset_m * lateral_sign
    centre = start + direction * along_m + lateral
    return np.array([centre[0], centre[1], height_m]), heading


def marker_transform(position, yaw_rad):
    """4x4 world transform for a marker quad facing `yaw_rad`.

    Quad spans local XY, face normal along local +Z, texture up along local +Y. Built
    from column vectors rather than composed Euler angles.
    """
    cos_yaw, sin_yaw = math.cos(yaw_rad), math.sin(yaw_rad)
    transform = np.eye(4)
    transform[:3, 0] = [-sin_yaw, cos_yaw, 0.0]  # local +X, across the face
    transform[:3, 1] = [0.0, 0.0, 1.0]  # local +Y, texture up
    transform[:3, 2] = [cos_yaw, sin_yaw, 0.0]  # local +Z, face normal
    transform[:3, 3] = position
    return transform


def marker_corners(position, yaw_rad, size_m):
    """The quad's four corner points in world coordinates."""
    transform = marker_transform(position, yaw_rad)
    half = size_m / 2.0
    offsets = [(-half, -half), (half, -half), (half, half), (-half, half)]
    return [
        transform[:3, 3] + transform[:3, 0] * u + transform[:3, 1] * v
        for u, v in offsets
    ]


def quaternion_to_matrix(x, y, z, w):
    """3x3 rotation matrix from a quaternion in geometry_msgs (x, y, z, w) order."""
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quaternion(matrix):
    """Quaternion in geometry_msgs (x, y, z, w) order from a 3x3 rotation matrix.

    Largest-diagonal branch, not the trace shortcut alone: that loses precision and can
    root a negative when the trace nears -1.
    """
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return (
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        )

    diagonal = [matrix[0, 0], matrix[1, 1], matrix[2, 2]]
    axis = int(np.argmax(diagonal))
    next_axis = (axis + 1) % 3
    prev_axis = (axis + 2) % 3

    scale = (
        math.sqrt(1.0 + diagonal[axis] - diagonal[next_axis] - diagonal[prev_axis])
        * 2.0
    )
    components = [0.0, 0.0, 0.0]
    components[axis] = 0.25 * scale
    components[next_axis] = (matrix[next_axis, axis] + matrix[axis, next_axis]) / scale
    components[prev_axis] = (matrix[axis, prev_axis] + matrix[prev_axis, axis]) / scale
    w = (matrix[prev_axis, next_axis] - matrix[next_axis, prev_axis]) / scale
    return components[0], components[1], components[2], w


def usd_camera_rotation(view_yaw_rad, pitch_rad=0.0, roll_rad=0.0):
    """Rotation for a USD camera prim looking along `view_yaw_rad`, world Z up.

    Columns are [right, up, -forward], since USD cameras look down local -Z. A camera
    aimed along +X therefore equals R_USD_TO_GT, which the frame conversion relies on.
    Built from basis vectors rather than composed Euler angles, which hide sign errors.
    """
    cos_yaw, sin_yaw = math.cos(view_yaw_rad), math.sin(view_yaw_rad)
    cos_pitch, sin_pitch = math.cos(pitch_rad), math.sin(pitch_rad)
    forward = np.array([cos_yaw * cos_pitch, sin_yaw * cos_pitch, sin_pitch])
    right = np.cross(forward, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)

    if abs(roll_rad) > 1e-12:
        cos_roll, sin_roll = math.cos(roll_rad), math.sin(roll_rad)
        right, up = right * cos_roll + up * sin_roll, up * cos_roll - right * sin_roll

    return np.column_stack([right, up, -forward])


def world_to_camera(camera_position, camera_rotation, point_world):
    """Express a world point in the detector's camera frame.

    `camera_rotation` is the prim's 3x3 world rotation in USD convention; R_USD_TO_GT
    rebases into X-forward/Y-left/Z-up.
    """
    return R_USD_TO_GT @ (
        camera_rotation.T @ (np.asarray(point_world) - camera_position)
    )


def project(point_camera, fx, fy, cx, cy):
    """Pinhole projection of a camera-frame point. Returns (u, v, depth).

    Depth is X, this frame being X-forward. Depth <= 0 means behind the camera and the
    returned pixel is meaningless.
    """
    depth = float(point_camera[0])
    if depth <= 1e-9:
        return float("nan"), float("nan"), depth
    # Y is left and Z is up, so both flip to get right-down image axes.
    u = cx - fx * float(point_camera[1]) / depth
    v = cy - fy * float(point_camera[2]) / depth
    return u, v, depth


def incidence_angle_deg(marker_position, marker_yaw_rad, camera_position):
    """Angle between the marker's face normal and the ray to the camera. 0 is head-on."""
    to_camera = np.asarray(camera_position) - np.asarray(marker_position)
    distance = float(np.linalg.norm(to_camera))
    if distance < 1e-9:
        return 0.0
    cosine = float(np.dot(yaw_to_normal(marker_yaw_rad), to_camera / distance))
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def apparent_diameter_px(size_m, fx, range_m, incidence_deg):
    """Foreshortened width of the marker in pixels at this range and viewing angle."""
    if range_m < 1e-9:
        return 0.0
    return size_m * fx * math.cos(math.radians(incidence_deg)) / range_m


def evaluate_marker(
    marker, camera_position, camera_rotation, intrinsics, image_size, limits
):
    """Ground-truth evaluation of one marker against one camera pose.

    `marker` needs .position, .yaw_rad, .size_m; `intrinsics` is (fx, fy, cx, cy);
    `limits` carries max_incidence_deg, min_diameter_px, frustum_margin_px. Returns the
    MarkerGroundTruth fields except occlusion, which only the simulator can determine.
    """
    fx, fy, cx, cy = intrinsics
    width, height = image_size
    margin = getattr(limits, "frustum_margin_px", 0.0)

    centre_camera = world_to_camera(camera_position, camera_rotation, marker.position)
    u, v, depth = project(centre_camera, fx, fy, cx, cy)
    range_m = float(np.linalg.norm(centre_camera))
    incidence = incidence_angle_deg(marker.position, marker.yaw_rad, camera_position)
    diameter = apparent_diameter_px(marker.size_m, fx, range_m, incidence)

    # All four corners, not the centre: a marker centred just inside the edge is still
    # half cut off, and the ring must be substantially in-frame to decode.
    in_frustum = depth > 0.0
    if in_frustum:
        for corner in marker_corners(marker.position, marker.yaw_rad, marker.size_m):
            corner_camera = world_to_camera(camera_position, camera_rotation, corner)
            corner_u, corner_v, corner_depth = project(corner_camera, fx, fy, cx, cy)
            if corner_depth <= 0.0 or not (
                margin <= corner_u <= width - margin
                and margin <= corner_v <= height - margin
            ):
                in_frustum = False
                break

    # Marker face frame in camera coordinates, matching what the detector reports.
    marker_rotation = marker_transform(marker.position, marker.yaw_rad)[:3, :3]
    rotation_camera = R_USD_TO_GT @ camera_rotation.T @ marker_rotation

    facing = incidence < limits.max_incidence_deg
    large_enough = diameter >= limits.min_diameter_px
    return {
        "position_camera": centre_camera,
        "rotation_camera": rotation_camera,
        "range_m": range_m,
        "incidence_angle_deg": incidence,
        "bearing_u_px": u,
        "bearing_v_px": v,
        "apparent_diameter_px": diameter,
        "in_frustum": in_frustum,
        "facing_camera": facing,
        "large_enough": large_enough,
    }


def pure_pursuit_curvature(lookahead_body):
    """Path curvature for a lookahead point in the body frame (X fwd, Y left).

    Positive curvature turns left.
    """
    x, y = float(lookahead_body[0]), float(lookahead_body[1])
    distance_sq = x * x + y * y
    if distance_sq < 1e-9:
        return 0.0
    return 2.0 * y / distance_sq


def resample_path(waypoints, loop=True):
    """Cumulative arclength at each waypoint, and the total path length."""
    points = np.asarray(waypoints, dtype=float)
    closed = np.vstack([points, points[:1]]) if loop else points
    segment_lengths = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(segment_lengths)]), float(
        segment_lengths.sum()
    )


def point_at_arclength(waypoints, arclength, loop=True):
    """Point on the waypoint polyline at the given arclength."""
    points = np.asarray(waypoints, dtype=float)
    cumulative, total = resample_path(waypoints, loop)
    if loop:
        arclength = arclength % total
    else:
        arclength = max(0.0, min(arclength, total))
    index = int(np.searchsorted(cumulative, arclength, side="right") - 1)
    index = max(0, min(index, len(cumulative) - 2))
    span = cumulative[index + 1] - cumulative[index]
    ratio = 0.0 if span < 1e-9 else (arclength - cumulative[index]) / span
    start = points[index % len(points)]
    end = points[(index + 1) % len(points)]
    return start + (end - start) * ratio


def project_onto_path(waypoints, position, loop=True):
    """Arclength of the closest point on the polyline to `position`."""
    points = np.asarray(waypoints, dtype=float)
    cumulative, _ = resample_path(waypoints, loop)
    position = np.asarray(position, dtype=float)[:2]
    best_distance_sq, best_arclength = float("inf"), 0.0
    count = len(points) if loop else len(points) - 1
    for index in range(count):
        start = points[index]
        end = points[(index + 1) % len(points)]
        delta = end - start
        length_sq = float(np.dot(delta, delta))
        if length_sq < 1e-9:
            continue
        ratio = max(0.0, min(1.0, float(np.dot(position - start, delta)) / length_sq))
        closest = start + delta * ratio
        distance_sq = float(np.dot(position - closest, position - closest))
        if distance_sq < best_distance_sq:
            best_distance_sq = distance_sq
            best_arclength = cumulative[index] + ratio * math.sqrt(length_sq)
    return best_arclength


def _check_rotation():
    assert np.allclose(
        R_USD_TO_GT @ R_USD_TO_GT.T, np.eye(3)
    ), "R_USD_TO_GT not orthonormal"
    assert (
        abs(np.linalg.det(R_USD_TO_GT) - 1.0) < 1e-12
    ), "R_USD_TO_GT is not a proper rotation"
    # USD forward, right, and up must land on detector forward, right, and up.
    assert np.allclose(R_USD_TO_GT @ [0, 0, -1], [1, 0, 0]), "USD forward is not GT +X"
    assert np.allclose(R_USD_TO_GT @ [1, 0, 0], [0, -1, 0]), "USD right is not GT -Y"
    assert np.allclose(R_USD_TO_GT @ [0, 1, 0], [0, 0, 1]), "USD up is not GT +Z"


def _check_intrinsics():
    fx, fy, cx, cy = camera_intrinsics(1280, 720, 80.0)
    assert (
        abs(fx - 762.72224426269531) < 1e-3
    ), f"fx drifted from the detector's value: {fx}"
    assert fx == fy and (cx, cy) == (640.0, 360.0)
    focal, horizontal, vertical = usd_camera_aperture(1280, 720, 80.0)
    assert (
        abs(1280 * focal / horizontal - fx) < 1e-6
    ), "USD aperture does not round-trip to fx"
    assert (
        abs(vertical / horizontal - 720 / 1280) < 1e-12
    ), "aperture aspect is not square-pixel"


def _check_marker_yaw():
    # A marker `offset` to the side should read 0 degrees incidence when the robot is
    # `offset` short of abeam, and 90 degrees once it draws level and passes.
    offset = 1.5
    for heading_deg in (0.0, 37.0, 90.0, 180.0, -120.0):
        heading = math.radians(heading_deg)
        direction = np.array([math.cos(heading), math.sin(heading)])
        normal_left = np.array([-direction[1], direction[0]])
        for side, sign in (("left", 1.0), ("right", -1.0)):
            centre = normal_left * offset * sign
            marker = np.array([centre[0], centre[1], 0.0])
            yaw = marker_normal_yaw(heading, side, 45.0)
            for arclength, expected in ((-offset, 0.0), (0.0, 45.0), (offset, 90.0)):
                camera = np.array(
                    [direction[0] * arclength, direction[1] * arclength, 0.0]
                )
                actual = incidence_angle_deg(marker, yaw, camera)
                assert abs(actual - expected) < 1e-6, (
                    f"heading={heading_deg} side={side} s={arclength}: "
                    f"expected {expected} deg, got {actual:.4f} deg"
                )


def _check_placement():
    waypoints = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]
    position, heading = place_marker(waypoints, 0, 3.0, "left", 2.0, 1.2)
    assert np.allclose(position, [3.0, 2.0, 1.2]), position
    assert abs(heading) < 1e-12
    position, _ = place_marker(waypoints, 0, 3.0, "right", 2.0, 1.2)
    assert np.allclose(position, [3.0, -2.0, 1.2]), position


def _check_projection():
    fx, fy, cx, cy = camera_intrinsics(1280, 720, 80.0)
    # Straight ahead lands at the principal point.
    u, v, depth = project(np.array([5.0, 0.0, 0.0]), fx, fy, cx, cy)
    assert abs(u - cx) < 1e-9 and abs(v - cy) < 1e-9 and abs(depth - 5.0) < 1e-9
    # Left of the camera must appear left of centre; up must appear above centre.
    u_left, _, _ = project(np.array([5.0, 1.0, 0.0]), fx, fy, cx, cy)
    _, v_up, _ = project(np.array([5.0, 0.0, 1.0]), fx, fy, cx, cy)
    assert u_left < cx and v_up < cy
    # Behind the camera is reported, not silently projected.
    assert project(np.array([-1.0, 0.0, 0.0]), fx, fy, cx, cy)[2] < 0


def _check_apparent_diameter():
    fx, _, _, _ = camera_intrinsics(1280, 720, 80.0)
    assert abs(apparent_diameter_px(0.2, fx, 1.0, 0.0) - 152.544) < 1e-2
    assert abs(apparent_diameter_px(0.2, fx, 2.0, 0.0) - 76.272) < 1e-2
    assert abs(apparent_diameter_px(0.2, fx, 1.0, 90.0)) < 1e-6


def _check_quaternion():
    # Round-trip a spread of rotations, including the near-180-degree cases where the
    # trace branch is numerically worst.
    for yaw in (0.0, 0.3, math.pi / 2, math.pi - 1e-4, -2.1):
        for pitch in (0.0, 0.7, -1.2):
            rotation = usd_camera_rotation(yaw, pitch)
            x, y, z, w = matrix_to_quaternion(rotation)
            assert (
                abs(math.sqrt(x * x + y * y + z * z + w * w) - 1.0) < 1e-9
            ), "not unit length"
            assert np.allclose(
                quaternion_to_matrix(x, y, z, w), rotation, atol=1e-9
            ), f"round-trip failed at yaw={yaw} pitch={pitch}"
    # A 180-degree rotation about Z has trace -1, the branch the trace shortcut cannot take.
    flip = np.diag([-1.0, -1.0, 1.0])
    x, y, z, w = matrix_to_quaternion(flip)
    assert np.allclose(quaternion_to_matrix(x, y, z, w), flip, atol=1e-9)


def _check_evaluate_marker():
    from types import SimpleNamespace

    intrinsics = camera_intrinsics(1280, 720, 80.0)
    limits = SimpleNamespace(
        max_incidence_deg=70.0, min_diameter_px=15.0, frustum_margin_px=0.0
    )
    # Camera at the origin in USD convention (identity rotation) looks down -Z, which
    # R_USD_TO_GT maps to world +X. So a marker on +X is straight ahead.
    camera_position = np.zeros(3)
    camera_rotation = usd_camera_rotation(0.0)  # looking along world +X
    # A camera facing world +X orients exactly as R_USD_TO_GT; if that ever stops being
    # true, the two derivations have diverged.
    assert np.allclose(camera_rotation, R_USD_TO_GT), camera_rotation

    ahead = SimpleNamespace(position=[3.0, 0.0, 0.0], yaw_rad=math.pi, size_m=0.2)
    result = evaluate_marker(
        ahead, camera_position, camera_rotation, intrinsics, (1280, 720), limits
    )
    assert abs(result["range_m"] - 3.0) < 1e-9
    assert (
        abs(result["incidence_angle_deg"]) < 1e-6
    ), "marker facing back at the camera is head-on"
    assert result["in_frustum"] and result["facing_camera"] and result["large_enough"]
    assert (
        abs(result["bearing_u_px"] - 640.0) < 1e-6
        and abs(result["bearing_v_px"] - 360.0) < 1e-6
    )

    # A marker to the world +Y (the camera's left) must project left of centre.
    left = SimpleNamespace(position=[3.0, 1.0, 0.0], yaw_rad=math.pi, size_m=0.2)
    assert (
        evaluate_marker(
            left, camera_position, camera_rotation, intrinsics, (1280, 720), limits
        )["bearing_u_px"]
        < 640.0
    )

    # Directly behind is out of frustum, not merely off-image.
    behind = SimpleNamespace(position=[-3.0, 0.0, 0.0], yaw_rad=0.0, size_m=0.2)
    assert not evaluate_marker(
        behind, camera_position, camera_rotation, intrinsics, (1280, 720), limits
    )["in_frustum"]

    # Far enough away and the marker is too few pixels to decode.
    far = SimpleNamespace(position=[40.0, 0.0, 0.0], yaw_rad=math.pi, size_m=0.2)
    assert not evaluate_marker(
        far, camera_position, camera_rotation, intrinsics, (1280, 720), limits
    )["large_enough"]


def _check_pure_pursuit():
    # A lookahead point dead ahead is a straight line.
    assert abs(pure_pursuit_curvature([2.0, 0.0])) < 1e-12
    # On a circle of radius R the curvature must come out as 1/R.
    for radius in (1.0, 5.0, 25.0):
        for lookahead in (0.5, 1.5):
            if lookahead >= 2 * radius:
                continue
            angle = 2.0 * math.asin(lookahead / (2.0 * radius))
            body = [radius * math.sin(angle), radius * (1.0 - math.cos(angle))]
            actual = pure_pursuit_curvature(body)
            assert (
                abs(actual - 1.0 / radius) < 1e-9
            ), f"R={radius} L={lookahead}: {actual}"


def _check_path_arclength():
    waypoints = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]
    _, total = resample_path(waypoints)
    assert abs(total - 40.0) < 1e-9, total
    assert np.allclose(point_at_arclength(waypoints, 5.0), [5.0, 0.0])
    assert np.allclose(point_at_arclength(waypoints, 15.0), [10.0, 5.0])
    # Wraps rather than clamping, because the path is a loop.
    assert np.allclose(point_at_arclength(waypoints, 45.0), [5.0, 0.0])
    assert abs(project_onto_path(waypoints, [5.0, 0.3]) - 5.0) < 1e-9


def main():
    for check in (
        _check_rotation,
        _check_intrinsics,
        _check_marker_yaw,
        _check_placement,
        _check_projection,
        _check_apparent_diameter,
        _check_quaternion,
        _check_evaluate_marker,
        _check_pure_pursuit,
        _check_path_arclength,
    ):
        check()
        print(f"  ok  {check.__name__[7:]}")
    print("all geometry self-checks passed")


if __name__ == "__main__":
    main()
