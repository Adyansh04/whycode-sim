"""Occupancy survey of the loaded environment.

Waypoints and markers must sit on free floor inside the building; guessing coordinates
against an unfamiliar asset puts the robot outside the walls or inside a rack. Projects
all geometry onto the floor plane and reports an occupancy map, a drivable circuit, and
where markers fit.

    ./python.sh run_sim.py --scene <scene.yaml> --survey
"""

import math

import numpy as np
from pxr import Usd, UsdGeom

# Below this is floor plate, decal or painted line, not an obstacle; marking those
# occupied reads the whole building as blocked.
OBSTACLE_MIN_HEIGHT_M = 0.15

# Above this is roof truss, beam or light, which does not block a ground vehicle.
OBSTACLE_MAX_FLOOR_CLEARANCE_M = 1.8


def _iter_geometry(stage, root_path):
    """Leaf gprims under `root_path` with their world-aligned bounds."""
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return

    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Gprim):
            continue
        box = bounds.ComputeWorldBound(prim).ComputeAlignedBox()
        if box.IsEmpty():
            continue
        yield prim, box


def occupancy(stage, root_path, resolution=0.5, margin=0.0):
    """Build a floor occupancy grid. Returns (grid, origin_xy, resolution).

    grid[row, col] is True where geometry blocks the robot. Row is the Y axis.
    """
    cells = []
    minimum = np.array([np.inf, np.inf])
    maximum = np.array([-np.inf, -np.inf])

    for _, box in _iter_geometry(stage, root_path):
        low, high = box.GetMin(), box.GetMax()
        minimum = np.minimum(minimum, [low[0], low[1]])
        maximum = np.maximum(maximum, [high[0], high[1]])
        # Skip the floor itself and anything overhead.
        if high[2] < OBSTACLE_MIN_HEIGHT_M or low[2] > OBSTACLE_MAX_FLOOR_CLEARANCE_M:
            continue
        cells.append(
            (low[0] - margin, low[1] - margin, high[0] + margin, high[1] + margin)
        )

    if not np.isfinite(minimum).all():
        raise RuntimeError(f"no geometry found under {root_path}")

    size = np.ceil((maximum - minimum) / resolution).astype(int) + 1
    grid = np.zeros((size[1], size[0]), dtype=bool)

    for min_x, min_y, max_x, max_y in cells:
        col0 = max(0, int((min_x - minimum[0]) / resolution))
        col1 = min(size[0] - 1, int(np.ceil((max_x - minimum[0]) / resolution)))
        row0 = max(0, int((min_y - minimum[1]) / resolution))
        row1 = min(size[1] - 1, int(np.ceil((max_y - minimum[1]) / resolution)))
        grid[row0 : row1 + 1, col0 : col1 + 1] = True

    return grid, minimum, resolution


def largest_free_rectangle(grid):
    """Largest axis-aligned all-free rectangle, as (row0, col0, row1, col1).

    Standard maximal-rectangle-in-histogram sweep; the loop wants the biggest clear
    box it can be inscribed in.
    """
    rows, cols = grid.shape
    heights = np.zeros(cols, dtype=int)
    best = (0, (0, 0, 0, 0))

    for row in range(rows):
        heights = np.where(grid[row], 0, heights + 1)

        stack = []
        for col in range(cols + 1):
            height = heights[col] if col < cols else 0
            start = col
            while stack and stack[-1][1] >= height:
                start, popped_height = stack.pop()
                area = popped_height * (col - start)
                if area > best[0]:
                    best = (area, (row - popped_height + 1, start, row, col - 1))
            stack.append((start, height))

    return best[1]


def aisle_masks(grid, half_width_cells):
    """Cells inside an aisle, i.e. with geometry close on both sides.

    Returns (vertical, horizontal): vertical[r, c] is True where a north-south path is
    flanked left and right; horizontal is the east-west equivalent. This is the
    difference between drivable and between-shelves.
    """
    rows, cols = grid.shape
    near_left = np.zeros_like(grid)
    near_right = np.zeros_like(grid)
    near_below = np.zeros_like(grid)
    near_above = np.zeros_like(grid)

    for shift in range(1, half_width_cells + 1):
        near_left[:, shift:] |= grid[:, :-shift]
        near_right[:, :-shift] |= grid[:, shift:]
        near_below[shift:, :] |= grid[:-shift, :]
        near_above[:-shift, :] |= grid[shift:, :]

    vertical = near_left & near_right & ~grid
    horizontal = near_below & near_above & ~grid
    return vertical, horizontal


def best_loop(
    grid,
    min_extent_cells=8,
    interior_obstacle_fraction=0.04,
    row_stride=2,
    edge_margin_cells=0,
    aisle_half_width_cells=0,
    min_aisle_fraction=0.6,
):
    """Largest rectangular loop whose edges are clear and whose interior is not.

    The largest *free* rectangle is the wrong target: in a warehouse that is the empty
    apron outside. Requiring the enclosed area to contain obstacles keeps the answer
    indoors and makes the loop pass racks.

    `edge_margin_cells` demands free space on both sides of every edge, so markers fit
    left and right rather than only inside. `aisle_half_width_cells` demands most of the
    perimeter run between racks rather than across open floor.

    Returns (row0, col0, row1, col1) or None.
    """
    rows, cols = grid.shape

    if aisle_half_width_cells:
        aisle_vertical, aisle_horizontal = aisle_masks(grid, aisle_half_width_cells)
        vertical_run = np.vstack(
            [
                np.zeros(cols, dtype=np.int32),
                np.cumsum(aisle_vertical.astype(np.int32), axis=0),
            ]
        )
        horizontal_run = np.hstack(
            [
                np.zeros((rows, 1), dtype=np.int32),
                np.cumsum(aisle_horizontal.astype(np.int32), axis=1),
            ]
        )
    else:
        aisle_vertical = aisle_horizontal = None

    if edge_margin_cells > 0:
        # Dilating obstacles by the margin turns "clear" into "clear with margin on
        # every side", so the edge tests carry the requirement unchanged.
        margin = int(edge_margin_cells)
        dilated = grid.copy()
        for shift in range(1, margin + 1):
            dilated[:-shift] |= grid[shift:]
            dilated[shift:] |= grid[:-shift]
            dilated[:, :-shift] |= grid[:, shift:]
            dilated[:, shift:] |= grid[:, :-shift]
        edge_grid = dilated
    else:
        edge_grid = grid

    blocked = edge_grid.astype(np.int32)
    # Column-wise prefix sums, so "is this column clear between two rows" is O(1).
    column_prefix = np.vstack(
        [np.zeros(cols, dtype=np.int32), np.cumsum(blocked, axis=0)]
    )
    true_blocked = grid.astype(np.int32)
    area_prefix = np.cumsum(np.cumsum(true_blocked, axis=0), axis=1)

    def interior_blocked(row0, col0, row1, col1):
        total = area_prefix[row1, col1]
        if row0 > 0:
            total -= area_prefix[row0 - 1, col1]
        if col0 > 0:
            total -= area_prefix[row1, col0 - 1]
        if row0 > 0 and col0 > 0:
            total += area_prefix[row0 - 1, col0 - 1]
        return total

    best_score, best = 0, None
    for row0 in range(0, rows - min_extent_cells, row_stride):
        for row1 in range(row0 + min_extent_cells, rows, row_stride):
            # Columns clear along their whole vertical extent between the two rows.
            vertical_clear = (column_prefix[row1 + 1] - column_prefix[row0]) == 0
            if vertical_clear.sum() < 2:
                continue
            # Columns clear on both horizontal edges.
            horizontal_clear = ~edge_grid[row0] & ~edge_grid[row1]

            candidates = np.flatnonzero(vertical_clear)
            for start_index in range(len(candidates)):
                col0 = candidates[start_index]
                for end_index in range(len(candidates) - 1, start_index, -1):
                    col1 = candidates[end_index]
                    if col1 - col0 < min_extent_cells:
                        break
                    # Both horizontal edges must be walkable across the full span.
                    if not horizontal_clear[col0 : col1 + 1].all():
                        continue
                    span = (row1 - row0) + (col1 - col0)
                    if span <= best_score:
                        break
                    enclosed = (row1 - row0 - 1) * (col1 - col0 - 1)
                    if enclosed <= 0:
                        break
                    fraction = (
                        interior_blocked(row0 + 1, col0 + 1, row1 - 1, col1 - 1)
                        / enclosed
                    )
                    if fraction < interior_obstacle_fraction:
                        break

                    if aisle_vertical is not None:
                        # Long edges must run down aisles; end edges must cross between
                        # them rather than wander the open floor.
                        height = row1 - row0
                        width = col1 - col0
                        flanked = (
                            (vertical_run[row1 + 1, col0] - vertical_run[row0, col0])
                            + (vertical_run[row1 + 1, col1] - vertical_run[row0, col1])
                            + (
                                horizontal_run[row0, col1 + 1]
                                - horizontal_run[row0, col0]
                            )
                            + (
                                horizontal_run[row1, col1 + 1]
                                - horizontal_run[row1, col0]
                            )
                        )
                        if flanked < min_aisle_fraction * 2 * (height + width):
                            continue
                    best_score, best = span, (row0, col0, row1, col1)
                    break
    return best


def _obstacle_footprints(stage, root_path):
    """Floor-plane footprints of everything that could block the robot."""
    boxes = []
    for _, box in _iter_geometry(stage, root_path):
        low, high = box.GetMin(), box.GetMax()
        if high[2] < OBSTACLE_MIN_HEIGHT_M or low[2] > OBSTACLE_MAX_FLOOR_CLEARANCE_M:
            continue
        boxes.append((low[0], low[1], high[0], high[1]))
    return np.array(boxes) if boxes else np.zeros((0, 4))


def clearance_along(stage, points, root_path="/World/Environment", step=0.1, loop=True):
    """Distance from each sampled polyline point to the nearest obstacle.

    Measured against geometry, not the occupancy grid, so it avoids the grid's half-cell
    quantisation. Returns (samples, clearances).
    """
    boxes = _obstacle_footprints(stage, root_path)
    points = np.asarray(points, dtype=float)[:, :2]
    segments = list(zip(points, np.roll(points, -1, axis=0)))
    if not loop:
        segments = segments[:-1]

    samples = []
    for start, end in segments:
        length = float(np.linalg.norm(end - start))
        count = max(2, int(length / step))
        for index in range(count):
            samples.append(start + (end - start) * (index / count))
    samples = np.array(samples)

    if len(boxes) == 0:
        return samples, np.full(len(samples), np.inf)

    # Distance from a point to an axis-aligned box is the length of the componentwise
    # overshoot outside it, and zero when the point is inside.
    dx = np.maximum(
        boxes[None, :, 0] - samples[:, None, 0], samples[:, None, 0] - boxes[None, :, 2]
    )
    dy = np.maximum(
        boxes[None, :, 1] - samples[:, None, 1], samples[:, None, 1] - boxes[None, :, 3]
    )
    dx = np.maximum(dx, 0.0)
    dy = np.maximum(dy, 0.0)
    return samples, np.sqrt(dx * dx + dy * dy).min(axis=1)


def verify_path(
    stage,
    points,
    root_path="/World/Environment",
    robot_radius=0.45,
    loop=True,
    label="path",
):
    """Tightest clearance along a polyline. True when drivable."""
    samples, clearances = clearance_along(stage, points, root_path, loop=loop)
    blocked = clearances < robot_radius
    tightest = float(clearances.min())

    print(
        f"[verify] {label}: {len(samples)} samples, tightest clearance {tightest:.2f} m "
        f"(robot needs {robot_radius:.2f} m)"
    )

    if not blocked.any():
        print(f"[verify] {label}: CLEAR")
        return True

    print(f"[verify] {label}: BLOCKED at {int(blocked.sum())}/{len(samples)} samples")
    for point, clearance in list(zip(samples[blocked], clearances[blocked]))[:10]:
        print(
            f"[verify]   ({point[0]:7.2f}, {point[1]:7.2f})  clearance {clearance:.2f} m"
        )
    return False


def verify_markers(stage, markers, root_path="/World/Environment", clearance_m=0.3):
    """Check each marker post stands on free floor, not inside a rack."""
    boxes = _obstacle_footprints(stage, root_path)
    if len(boxes) == 0:
        return True

    all_clear = True
    for marker in markers:
        x, y = marker.position[0], marker.position[1]
        dx = np.maximum(boxes[:, 0] - x, x - boxes[:, 2])
        dy = np.maximum(boxes[:, 1] - y, y - boxes[:, 3])
        distance = float(np.sqrt(np.maximum(dx, 0) ** 2 + np.maximum(dy, 0) ** 2).min())
        status = "ok" if distance >= clearance_m else "INSIDE GEOMETRY"
        if distance < clearance_m:
            all_clear = False
        print(
            f"[verify] marker {marker.id:<3} ({x:7.2f},{y:7.2f})  clearance {distance:5.2f} m  {status}"
        )
    return all_clear


def verify_shared_ids(config, markers, step=0.1):
    """Markers sharing an id must never be detectable in the same frame.

    Reusing an id is how a layout carries more markers than the family has textures. It
    costs nothing as long as no two of them are ever in frame together -- ground truth
    cannot say which one a detection belongs to, and the analysis would credit or fault
    the wrong placement.

    Swept under deliberately pessimistic conditions, because the robot does not track the
    centreline: a quarter wider field of view, the detector's own 10 px floor rather than
    the config's, and the camera pushed off line and mis-aimed.
    """
    from types import SimpleNamespace

    from whycode_sim.geometry import (
        camera_intrinsics,
        evaluate_marker,
        point_at_arclength,
        resample_path,
        usd_camera_rotation,
    )

    shared = {}
    for index, marker in enumerate(markers):
        shared.setdefault(marker.id, []).append(index)
    shared = {mid: idx for mid, idx in shared.items() if len(idx) > 1}
    if not shared:
        print("[shared-id] every marker carries a distinct id")
        return True

    waypoints = [list(point) for point in config.path.waypoints]
    loop = getattr(config.path, "loop", True)
    _, total = resample_path(waypoints, loop)
    camera = config.camera
    intrinsics = camera_intrinsics(camera.width, camera.height, camera.hfov_deg * 1.25)
    image_size = (camera.width, camera.height)
    height = camera.mount.position[2]
    limits = SimpleNamespace(
        max_incidence_deg=80.0, min_diameter_px=10.0, frustum_margin_px=-40.0
    )

    clashes = {mid: 0 for mid in shared}
    for lateral in (-0.5, -0.25, 0.0, 0.25, 0.5):
        for yaw_error_deg in (-12.0, -6.0, 0.0, 6.0, 12.0):
            distance = 0.0
            while distance < total:
                here = point_at_arclength(waypoints, distance, loop)
                ahead = point_at_arclength(waypoints, distance + 0.1, loop)
                heading = math.atan2(ahead[1] - here[1], ahead[0] - here[0])
                normal = (-math.sin(heading), math.cos(heading))
                position = np.array(
                    [
                        here[0] + normal[0] * lateral,
                        here[1] + normal[1] * lateral,
                        height,
                    ]
                )
                rotation = usd_camera_rotation(heading + math.radians(yaw_error_deg))
                for mid, indices in shared.items():
                    hits = 0
                    for index in indices:
                        result = evaluate_marker(
                            markers[index],
                            position,
                            rotation,
                            intrinsics,
                            image_size,
                            limits,
                        )
                        if (
                            result["in_frustum"]
                            and result["large_enough"]
                            and result["facing_camera"]
                        ):
                            hits += 1
                    if hits > 1:
                        clashes[mid] += 1
                distance += step

    all_clear = True
    for mid, indices in shared.items():
        count = clashes[mid]
        status = "ok" if count == 0 else "AMBIGUOUS"
        if count:
            all_clear = False
        print(
            f"[shared-id] id {mid:<3} at markers {indices}  "
            f"co-detectable on {count} stressed poses  {status}"
        )
    if not all_clear:
        print(
            "[shared-id] two markers with one id can reach the camera together. Give one "
            "of them a different `slot`, or move it."
        )
    return all_clear


def render(grid, origin, resolution, max_width=110):
    """ASCII map of the occupancy grid, '#' occupied and '.' free."""
    step = max(1, int(np.ceil(grid.shape[1] / max_width)))
    lines = []
    for row in range(grid.shape[0] - 1, -1, -step):
        block = grid[max(0, row - step + 1) : row + 1]
        line = "".join(
            "#" if block[:, col : col + step].any() else "."
            for col in range(0, grid.shape[1], step)
        )
        y = origin[1] + row * resolution
        lines.append(f"{y:7.1f} |{line}")

    footer = " " * 8 + "+" + "-" * len(lines[0].split("|")[1])
    x_start = origin[0]
    x_end = origin[0] + grid.shape[1] * resolution
    lines.append(footer)
    lines.append(
        f"{'':8}x from {x_start:.1f} to {x_end:.1f} m, cell {resolution * step:.1f} m"
    )
    return "\n".join(lines)


def survey(
    stage,
    root_path="/World/Environment",
    resolution=0.5,
    robot_radius=0.45,
    bounds=None,
    marker_offset_m=0.0,
):
    """Print an occupancy map and suggest a loop around obstacles.

    `bounds` is an optional (x0, y0, x1, y1) window. Without it the search returns a lap
    around the outside of the building, the building being the enclosed obstacle.
    """
    grid, origin, resolution = occupancy(
        stage, root_path, resolution, margin=robot_radius
    )

    if bounds is not None:
        x0, y0, x1, y1 = bounds
        col0 = max(0, int((x0 - origin[0]) / resolution))
        col1 = min(grid.shape[1] - 1, int((x1 - origin[0]) / resolution))
        row0 = max(0, int((y0 - origin[1]) / resolution))
        row1 = min(grid.shape[0] - 1, int((y1 - origin[1]) / resolution))
        grid = grid[row0 : row1 + 1, col0 : col1 + 1]
        origin = np.array(
            [origin[0] + col0 * resolution, origin[1] + row0 * resolution]
        )
        print(f"[survey] restricted to x {x0}..{x1}  y {y0}..{y1}")

    free = int((~grid).sum())
    total = grid.size
    print(f"[survey] grid {grid.shape[1]}x{grid.shape[0]} cells at {resolution} m")
    print(f"[survey] origin ({origin[0]:.2f}, {origin[1]:.2f}) m")
    print(f"[survey] free {free}/{total} cells ({100.0 * free / total:.0f}%)")
    print(f"[survey] obstacle margin {robot_radius} m (robot half-width)")
    print()
    print(render(grid, origin, resolution))
    print()

    # A warehouse's largest drivable circuit is the open apron. Require the perimeter
    # flanked by racks, and report each aisle width so the choice is visible.
    print("[survey] loop options, requiring the path to run between shelves:")
    options = {}
    for aisle_half_m in (1.5, 2.0, 2.5, 3.0, 4.0):
        cells = max(1, int(round(aisle_half_m / resolution)))
        candidate = best_loop(grid, aisle_half_width_cells=cells)
        if candidate is None:
            print(f"[survey]   flanked within {aisle_half_m:4.1f} m : none")
            continue
        r0, c0, r1, c1 = candidate
        width = (c1 - c0) * resolution
        height = (r1 - r0) * resolution
        print(
            f"[survey]   flanked within {aisle_half_m:4.1f} m : {width:5.1f} x {height:5.1f} m, "
            f"perimeter {2 * (width + height):6.1f} m"
        )
        options[aisle_half_m] = candidate

    if not options:
        print(
            "[survey] no circuit runs between shelves; falling back to any drivable loop"
        )
        loop = best_loop(grid)
        if loop is None:
            print("[survey] no rectangular loop found at all")
            return grid, origin, resolution
    else:
        # Tightest aisle that still yields a circuit is the one most hemmed in by racks.
        chosen = min(options)
        loop = options[chosen]
        print(f"[survey] using the {chosen:.1f} m option")

    row0, col0, row1, col1 = loop
    x0 = origin[0] + col0 * resolution
    x1 = origin[0] + col1 * resolution
    y0 = origin[1] + row0 * resolution
    y1 = origin[1] + row1 * resolution

    print(f"[survey] best drivable loop: x {x0:.2f}..{x1:.2f}  y {y0:.2f}..{y1:.2f}")
    print(
        f"[survey]   {x1 - x0:.1f} x {y1 - y0:.1f} m, perimeter {2 * ((x1 - x0) + (y1 - y0)):.1f} m"
    )

    # The robot spawns at the world origin, so shifting the environment by the negative
    # of the loop's first corner puts the start under the robot and keeps odom == world.
    print()
    print(
        "[survey] put this in scene.yaml (offsets are relative to the current world.offset):"
    )
    print(f"  world.offset shift : [{-x0:.2f}, {-y0:.2f}, 0.0]")
    print("  waypoints (after that shift):")
    for x, y in ((0.0, 0.0), (x1 - x0, 0.0), (x1 - x0, y1 - y0), (0.0, y1 - y0)):
        print(f"    - [{x:.2f}, {y:.2f}]")
    return grid, origin, resolution


def lateral_clearance(
    stage,
    points,
    root_path="/World/Environment",
    step=0.5,
    loop=True,
    max_offset=3.0,
    probe=0.1,
):
    """Free lateral distance on each side of the path, sampled along it.

    Returns dicts of segment, along_m, position, left, right. Segment index is carried
    through rather than recovered afterwards: a point on one leg also projects onto
    another's infinite line, which picks the wrong segment at corners.

    Measured against geometry, which is finer than the grid: the grid inflates obstacles
    by the robot radius and rounds to half-metre cells, understating marker room by about
    a metre.
    """
    boxes = _obstacle_footprints(stage, root_path)
    points = np.asarray(points, dtype=float)[:, :2]
    segments = list(zip(points, np.roll(points, -1, axis=0)))
    if not loop:
        segments = segments[:-1]

    def blocked(point):
        if len(boxes) == 0:
            return False
        inside_x = (boxes[:, 0] <= point[0]) & (point[0] <= boxes[:, 2])
        inside_y = (boxes[:, 1] <= point[1]) & (point[1] <= boxes[:, 3])
        return bool((inside_x & inside_y).any())

    samples = []
    for index, (start_point, end_point) in enumerate(segments):
        length = float(np.linalg.norm(end_point - start_point))
        direction = (end_point - start_point) / length
        normal = np.array([-direction[1], direction[0]])  # left of travel

        for tick in range(max(1, int(length / step))):
            along = tick * step
            position = start_point + direction * along
            free = {}
            for side, sign in (("left", 1.0), ("right", -1.0)):
                distance = 0.0
                while distance < max_offset:
                    if blocked(position + normal * sign * (distance + probe)):
                        break
                    distance += probe
                free[side] = distance
            samples.append(
                {
                    "segment": index,
                    "along_m": along,
                    "position": position,
                    "left": free["left"],
                    "right": free["right"],
                }
            )
    return samples


def suggest_markers(
    stage, config, count=8, offset_m=0.8, root_path="/World/Environment"
):
    """Propose marker placements that fit, spread around the loop, alternating sides.

    Where an aisle is too narrow for one side, that side is skipped rather than pushed
    into a rack.
    """
    waypoints = [list(point) for point in config.path.waypoints]
    loop = getattr(config.path, "loop", True)
    samples = lateral_clearance(stage, waypoints, root_path, loop=loop)

    # Offset plus half the marker's width is what must stand clear.
    needed = offset_m + config.marker.size_m / 2.0
    fits = sum(1 for s in samples for side in ("left", "right") if s[side] >= needed)
    print(
        f"[suggest] {fits} of {2 * len(samples)} path-side positions fit a {offset_m} m "
        f"offset (marker needs {needed:.2f} m)"
    )
    print(
        f"[suggest] narrowest: left {min(s['left'] for s in samples):.2f} m, "
        f"right {min(s['right'] for s in samples):.2f} m"
    )

    chosen, side = [], "left"
    used = set()
    for slot in range(count):
        target = slot * len(samples) / count
        # Nearest free sample to the evenly-spaced target on the wanted side, falling
        # back to the other side before giving up on the slot.
        for want in (side, "right" if side == "left" else "left"):
            options = [
                s
                for s in samples
                if s[want] >= needed and (s["segment"], s["along_m"], want) not in used
            ]
            if not options:
                continue
            pick = min(options, key=lambda s: abs(samples.index(s) - target))
            used.add((pick["segment"], pick["along_m"], want))
            chosen.append((pick, want))
            side = "right" if want == "left" else "left"
            break

    print("[suggest] paste into scene.yaml under `markers:`")
    for pick, want in sorted(chosen, key=lambda c: (c[0]["segment"], c[0]["along_m"])):
        x, y = pick["position"]
        print(
            f"  - {{at_waypoint: {pick['segment']}, along_m: {pick['along_m']:.1f}, "
            f"side: {want}, offset_m: {offset_m}}}   # ({x:.1f}, {y:.1f})"
        )
    return chosen


def deepest_aisle_point(stage, config, root_path="/World/Environment"):
    """Path point most enclosed by racks, i.e. the narrowest part of the aisle.

    Spawning here starts the run between shelves rather than in an open cross-aisle.
    """
    waypoints = [list(point) for point in config.path.waypoints]
    loop = getattr(config.path, "loop", True)
    samples = lateral_clearance(stage, waypoints, root_path, loop=loop)
    best = min(samples, key=lambda s: s["left"] + s["right"])
    position = best["position"]
    print(
        f"[spawn] narrowest point on the loop: ({position[0]:.2f}, {position[1]:.2f}) "
        f"with {best['left']:.2f} m left and {best['right']:.2f} m right"
    )
    print(
        f"[spawn] shift world.offset by [{-position[0]:.2f}, {-position[1]:.2f}, 0.0] "
        "to start the robot there"
    )
    return position


def predict_visibility(config, markers, step=0.25):
    """Sweep the camera along the loop, reporting how often each marker is seen.

    Clearance is not enough: on a short cross-aisle the robot passes within a metre and
    turns away, so the frames where a marker is close are the frames where it is edge-on
    and it never appears at all.
    """
    from whycode_sim.geometry import (
        camera_intrinsics,
        evaluate_marker,
        point_at_arclength,
        resample_path,
        usd_camera_rotation,
    )

    waypoints = [list(point) for point in config.path.waypoints]
    loop = getattr(config.path, "loop", True)
    _, total = resample_path(waypoints, loop)
    camera = config.camera
    intrinsics = camera_intrinsics(camera.width, camera.height, camera.hfov_deg)
    image_size = (camera.width, camera.height)
    height = camera.mount.position[2]

    # Keyed by index, not id: the layout may carry the same id at two placements.
    counts = [{"seen": 0, "best_px": 0.0} for _ in markers]
    samples = 0
    distance = 0.0
    while distance < total:
        here = point_at_arclength(waypoints, distance, loop)
        ahead = point_at_arclength(waypoints, distance + 0.1, loop)
        heading = math.atan2(ahead[1] - here[1], ahead[0] - here[0])
        position = np.array([here[0], here[1], height])
        rotation = usd_camera_rotation(heading)

        for index, marker in enumerate(markers):
            result = evaluate_marker(
                marker, position, rotation, intrinsics, image_size, config.visibility
            )
            if (
                result["in_frustum"]
                and result["facing_camera"]
                and result["large_enough"]
            ):
                counts[index]["seen"] += 1
                counts[index]["best_px"] = max(
                    counts[index]["best_px"], result["apparent_diameter_px"]
                )
        samples += 1
        distance += step

    print(f"[visibility] swept {samples} camera poses over {total:.0f} m of path")
    # The sweep follows the centreline exactly, while the robot lags its lookahead and
    # carries the camera forward of it, so a few percent here comes out as zero in a
    # recording. Flag marginal, not just zero.
    marginal_share = 4.0
    unseen = []
    for index, marker in enumerate(markers):
        entry = counts[index]
        share = 100.0 * entry["seen"] / samples
        if not entry["seen"]:
            status, marginal = "NEVER VISIBLE", True
        elif share < marginal_share:
            status, marginal = "MARGINAL", True
        else:
            status, marginal = "ok", False
        if marginal:
            unseen.append(f"#{index} (id {marker.id})")
        print(
            f"[visibility] #{index:<2} id {marker.id:<3} {entry['seen']:>4} poses "
            f"({share:4.1f}%), largest {entry['best_px']:5.0f} px  {status}"
        )
    if unseen:
        print(
            f"[visibility] markers {unseen} are never, or barely, both facing and close "
            "enough. Short cross-aisle legs cause this: the robot passes within a metre "
            "and turns away before the marker is ever both head-on and large. Move them "
            "to a long leg, widen the offset, or lower facing_angle_deg."
        )
    return counts
