#!/usr/bin/env python3
"""Plots and markdown tables from the benchmark's scores.

Reads <results>/<detector>/<bag>/score.json (evaluate.py) and writes, beside them:
range.png (detection rate against distance, per resolution and marker size), accuracy.png
(range and orientation error against distance), angle.png (detection rate against incidence),
tables.md (range, CPU, accuracy, viewing-angle and loop tables) and summary.svg (an at-a-glance
infographic of range and CPU at 1280x720). Needs only numpy and matplotlib.

    python3 benchmark_report.py <results>
"""

import json
import sys
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

plt.rcParams.update(
    {"axes.spines.top": False, "axes.spines.right": False, "font.size": 10}
)

# Detector, the label it carries, its colour, and the marker family its bags show
DETECTORS = [
    ("whycode", "WhyCode (ID)", "#1a7f37", "whycode"),
    ("whycon", "WhyCon", "#0969da", "whycon"),
    ("apriltag", "AprilTag 3, decimate 2", "#bc4c00", "apriltag"),
    ("apriltag_full", "AprilTag 3, full res", "#cf222e", "apriltag"),
    ("isaac_apriltag", "Isaac ROS AprilTag (GPU)", "#8250df", "apriltag"),
    ("aruco", "OpenCV ArUco 36h11", "#6e7781", "apriltag"),
]
RESOLUTIONS = [("640", "640x480"), ("1280", "1280x720"), ("1920", "1920x1080")]
SIZES = [("0.2", "-", "0.20 m"), ("0.146", "--", "0.146 m")]


def load(results):
    scores = {}
    for path in results.glob("*/*/score.json"):
        scores[(path.parent.parent.name, path.parent.name)] = json.loads(
            path.read_text()
        )
    return scores


def detection_curve(rows_csv, size, width=1.0):
    """Share of frames that found the marker of this size, per `width` m of distance."""
    rows = np.genfromtxt(rows_csv, delimiter=",", names=True)
    rows = rows[np.isclose(rows["size_m"], float(size))]
    bins = np.floor(rows["range_m"] / width).astype(int)
    xs, ys = [], []
    for b in np.unique(bins):
        sel = rows[bins == b]
        if len(sel) >= 5:
            xs.append((b + 0.5) * width)
            ys.append(float(np.mean(sel["found"])))
    return xs, ys


def range_plot(results, scores, out):
    figure, axes = plt.subplots(2, 3, figsize=(16, 7.6), sharex=True, sharey=True)
    for row, (size, _, size_label) in enumerate(SIZES):
        for column, (resolution, label) in enumerate(RESOLUTIONS):
            axis = axes[row][column]
            for name, title, colour, family in DETECTORS:
                bag = f"range_{family}_{resolution}"
                rows_csv = results / name / bag / "rows.csv"
                if (name, bag) not in scores or not rows_csv.exists():
                    continue
                xs, ys = detection_curve(rows_csv, size)
                axis.plot(xs, ys, color=colour, lw=1.8, label=title)
            axis.set_title(f"{size_label} marker, {label}", fontsize=10)
            axis.grid(alpha=0.3)
            axis.set_xlim(0, 40)
        axes[row][0].set_ylabel("frames found (with the right ID)")
    for axis in axes[1]:
        axis.set_xlabel("distance to the marker (m), 1 m bins")
    handles, labels = axes[0][1].get_legend_handles_labels()
    figure.legend(
        handles, labels, loc="lower center", ncol=6, fontsize=9, frameon=False
    )
    figure.suptitle(
        "Driving away from the markers, RealSense D435 colour field of view"
    )
    figure.tight_layout(rect=(0, 0.05, 1, 1))
    figure.savefig(out, dpi=130)
    plt.close(figure)


def binned_errors(rows_csv, size, width=2.0, start=0.0):
    """Median range error (% of the distance) and normal error per `width` m of distance from
    `start`, over the frames that found the marker of this size."""
    rows = np.genfromtxt(rows_csv, delimiter=",", names=True)
    rows = rows[(rows["found"] == 1) & np.isclose(rows["size_m"], float(size))]
    rows = rows[rows["range_m"] >= start]
    bins = np.floor((rows["range_m"] - start) / width).astype(int)
    centres, range_pct, normal_deg = [], [], []
    for b in np.unique(bins):
        sel = rows[bins == b]
        if len(sel) < 5:
            continue
        centres.append(start + (b + 0.5) * width)
        range_pct.append(np.median(100 * np.abs(sel["range_error_m"]) / sel["range_m"]))
        normal_deg.append(np.median(sel["normal_error_deg"]))
    return centres, range_pct, normal_deg


def accuracy_plot(results, scores, out):
    figure, axes = plt.subplots(1, 2, figsize=(13, 4.4))
    for name, title, colour, family in DETECTORS:
        rows_csv = results / name / f"range_{family}_1280" / "rows.csv"
        if (name, f"range_{family}_1280") not in scores or not rows_csv.exists():
            continue
        xs, range_pct, normal_deg = binned_errors(rows_csv, "0.2")
        axes[0].plot(xs, range_pct, "o-", color=colour, lw=1.6, ms=3, label=title)
        axes[1].plot(xs, normal_deg, "o-", color=colour, lw=1.6, ms=3, label=title)
    axes[0].set_ylabel("range error, median (% of distance)")
    axes[1].set_ylabel("marker normal error, median (deg)")
    for axis in axes:
        axis.set_xlabel("distance to the marker (m), 1280x720, 0.20 m marker, 2 m bins")
        axis.set_yscale("log")
        axis.grid(alpha=0.3, which="both")
    axes[0].legend(fontsize=8, frameon=False)
    figure.tight_layout()
    figure.savefig(out, dpi=130)
    plt.close(figure)


# The viewing-angle runs circle the marker at 2, 4 and 6 m; the 6 m circle is past some
# detectors' range, so the angle is judged on the 2 and 4 m circles alone
ANGLE_MAX_RANGE_M = 4.5


def incidence_curve(rows_csv, width=5.0):
    """Detection rate per `width` deg of incidence within ANGLE_MAX_RANGE_M, 0.20 m marker."""
    rows = np.genfromtxt(rows_csv, delimiter=",", names=True)
    rows = rows[
        np.isclose(rows["size_m"], 0.2) & (rows["range_m"] <= ANGLE_MAX_RANGE_M)
    ]
    bins = np.floor(rows["incidence_deg"] / width).astype(int)
    xs, ys = [], []
    for b in np.unique(bins):
        sel = rows[bins == b]
        if len(sel) >= 5:
            xs.append((b + 0.5) * width)
            ys.append(float(np.mean(sel["found"])))
    return xs, ys


def angle_plot(results, scores, out):
    figure, axis = plt.subplots(figsize=(7.5, 4.4))
    for name, title, colour, family in DETECTORS:
        rows_csv = results / name / f"angle_{family}" / "rows.csv"
        if (name, f"angle_{family}") not in scores or not rows_csv.exists():
            continue
        xs, ys = incidence_curve(rows_csv)
        axis.plot(xs, ys, color=colour, lw=1.6, label=title)
    axis.set_xlabel("incidence: angle between the marker's normal and the camera (deg)")
    axis.set_ylabel("frames detected (with the right ID)")
    axis.set_title("Circling a 0.20 m marker at 2 and 4 m, 1280x720")
    axis.grid(alpha=0.3)
    axis.legend(fontsize=8, frameon=False)
    figure.tight_layout()
    figure.savefig(out, dpi=130)
    plt.close(figure)


def cell(value, unit="", digits=1):
    return "-" if value is None else f"{value:.{digits}f}{unit}"


def tables(results, scores):
    lines = ["### Detection range", ""]
    header = (
        "| Resolution | Marker | " + " | ".join(t for _, t, _, _ in DETECTORS) + " |"
    )
    lines += [header, "|---|---|" + "---:|" * len(DETECTORS)]
    for resolution, label in RESOLUTIONS:
        for size, _, size_label in SIZES:
            row = [label, size_label]
            for name, _, _, family in DETECTORS:
                score = scores.get((name, f"range_{family}_{resolution}"))
                entry = score["sizes"].get(size) if score else None
                row.append(
                    f"{cell(entry['reliable_range_m'], ' m')} / {cell(entry['max_range_m'], ' m')}"
                    if entry
                    else "-"
                )
            lines.append("| " + " | ".join(row) + " |")
    lines += ["", "Reliable range (90% of frames) / farthest detection (50%).", ""]

    lines += ["### CPU per frame", ""]
    lines += [header.replace("| Marker ", ""), "|---|" + "---:|" * len(DETECTORS)]
    for resolution, label in RESOLUTIONS:
        row = [label]
        for name, _, _, family in DETECTORS:
            cpu = scores.get((name, f"range_{family}_{resolution}"), {}).get("cpu")
            row.append(
                f"{cpu['cpu_us_per_frame'] / 1000:.2f} ms ({cpu['cpu_percent_at_30hz']:.0f}%)"
                if cpu
                else "-"
            )
        lines.append("| " + " | ".join(row) + " |")
    lines += [
        "",
        "CPU time per frame, every thread, and the share of one core at 30 Hz.",
        "",
    ]

    lines += ["### Accuracy at 2-10 m (1280x720, 0.20 m)", ""]
    lines += [
        "| Detector | Range error, median | Normal error, median |",
        "|---|---:|---:|",
    ]
    for name, title, _, family in DETECTORS:
        rows_csv = results / name / f"range_{family}_1280" / "rows.csv"
        if not rows_csv.exists():
            continue
        _, range_pct, normal_deg = binned_errors(rows_csv, "0.2", width=8.0, start=2.0)
        if range_pct:
            lines.append(f"| {title} | {range_pct[0]:.2f}% | {normal_deg[0]:.1f} deg |")
    lines += [
        "",
        "Every frame that found the marker between 2 and 10 m, which all reach.",
        "",
    ]

    lines += ["### Viewing angle (1280x720, 0.20 m at 2 and 4 m)", ""]
    lines += ["| Detector | Steepest incidence found in 90% of frames |", "|---|---:|"]
    for name, title, _, family in DETECTORS:
        rows_csv = results / name / f"angle_{family}" / "rows.csv"
        if (name, f"angle_{family}") not in scores or not rows_csv.exists():
            continue
        steepest = None
        for x, found in zip(*incidence_curve(rows_csv)):
            if found < 0.9:
                break
            steepest = x + 2.5
        lines.append(f"| {title} | {cell(steepest, ' deg', 0)} |")
    lines += [
        "",
        "Incidence in 5 deg bins from head-on; the last bin before one below 90%.",
        "",
    ]

    lines += ["### Warehouse loop (1280x720)", ""]
    lines += [
        "| Detector | Found | Wrong IDs | False positives (with an ID) | Range error (median) "
        "| CPU per frame |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for name, title, _, family in DETECTORS:
        score = scores.get((name, f"{family}_1lap"))
        if not score:
            continue
        entry = score["sizes"].get("0.2")
        cpu = score.get("cpu")
        lines.append(
            f"| {title} | {100 * entry['found_visible']:.1f}% | {entry['wrong_id']} | "
            f"{score['false_positives']} ({score['false_positives_with_id']}) | "
            f"{cell(entry.get('range_error_pct_median'), '%', 2)} | "
            f"{cpu['cpu_us_per_frame'] / 1000:.2f} ms |"
            if entry and cpu
            else f"| {title} | - | - | - | - | - |"
        )
    return "\n".join(lines) + "\n"


# The summary's colours, light and dark, per detector (GitHub's palette)
SVG_COLOURS = {
    "whycode": ("#1a7f37", "#3fb950"),
    "whycon": ("#0969da", "#58a6ff"),
    "apriltag": ("#bc4c00", "#db6d28"),
    "apriltag_full": ("#cf222e", "#f85149"),
    "isaac_apriltag": ("#8250df", "#a371f7"),
    "aruco": ("#6e7781", "#8b949e"),
}
SVG_NOTES = {
    "whycode": "whycode_vision, IDs decoded",
    "whycon": "whycode_vision, no IDs",
    "apriltag": "apriltag_ros, its default decimate 2",
    "apriltag_full": "apriltag_ros, decimate 1",
    "isaac_apriltag": "NVIDIA cuAprilTags, on the GPU",
    "aruco": "OpenCV 4.6, DICT_APRILTAG_36h11",
}


def glyph(family, x, y):
    """A marker icon centred at x, y: a ring, with its code for WhyCode, or a tag."""
    if family == "apriltag":
        return (
            f'<rect x="{x - 8}" y="{y - 8}" width="16" height="16" class="gly-f"/>'
            f'<rect x="{x - 4}" y="{y - 4}" width="4" height="4" class="gly-b"/>'
            f'<rect x="{x + 1}" y="{y}" width="3" height="4" class="gly-b"/>'
        )
    ring = f'<circle cx="{x}" cy="{y}" r="6" class="gly-s"/>'
    if family == "whycode":
        ring += f'<circle cx="{x}" cy="{y}" r="3.2" class="gly-d"/>'
    return ring


def summary_svg(results, scores, out):
    """One-page summary at 1280x720: range per marker size and CPU per frame per detector, and
    the headline ratios against AprilTag 3, all read from the scores."""
    width, height = 960, 600
    rows = [(n, t, f) for n, t, _, f in DETECTORS]
    entries = {}
    for name, _, family in rows:
        score = scores.get((name, f"range_{family}_1280"))
        if score:
            entries[name] = score
    reach = [
        e["sizes"][size]["max_range_m"] or 0
        for e in entries.values()
        for size in ("0.2", "0.146")
        if size in e["sizes"]
    ]
    axis_m = 5 * int(np.ceil((max(reach, default=20) + 1) / 5))
    cpus = [e["cpu"]["cpu_us_per_frame"] / 1000 for e in entries.values() if "cpu" in e]
    axis_ms = 5 * int(np.ceil(max(cpus, default=10) / 5))

    x_label, x_bar, bar_w = 28, 268, 330
    x_cpu, cpu_w = 664, 150
    y0, row_h = 162, 50
    m_px, ms_px = bar_w / axis_m, cpu_w / axis_ms

    light = " ".join(f"--{n}:{c[0]};" for n, c in SVG_COLOURS.items())
    dark = " ".join(f"--{n}:{c[1]};" for n, c in SVG_COLOURS.items())
    out_lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" '
        f'width="{width}" height="{height}" role="img" '
        'aria-label="Benchmark summary: detection range and CPU per frame per detector">',
        "<title>WhyCode and AprilTag detectors: range and CPU</title>",
        "<style>",
        f":root {{ --bg:#ffffff; --fg:#1f2328; --muted:#656d76; --line:#d0d7de; {light} }}",
        f"@media (prefers-color-scheme: dark) {{ :root {{ --bg:#0d1117; --fg:#e6edf3; "
        f"--muted:#8b949e; --line:#30363d; {dark} }} }}",
        'text { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", "Noto Sans", '
        "Helvetica, Arial, sans-serif; fill: var(--fg); }",
        ".ttl { font-size: 19px; font-weight: 700; }",
        ".sub, .note, .val2, .tick, .cap { fill: var(--muted); }",
        ".sub { font-size: 11.5px; } .note, .tick { font-size: 10px; }",
        ".h { font-size: 13px; font-weight: 650; } .lab { font-size: 12.5px; font-weight: 600; }",
        ".val { font-size: 11.5px; font-weight: 650; } .val2, .cap { font-size: 10.5px; }",
        ".big { font-size: 26px; font-weight: 750; }",
        ".grid { stroke: var(--line); stroke-width: 1; } .axis { stroke: var(--muted); }",
        ".gly-s { fill: none; stroke: var(--fg); stroke-width: 3.6; }",
        ".gly-d { fill: none; stroke: var(--fg); stroke-width: 1.4; stroke-dasharray: 2 1.6; }",
        ".gly-f { fill: var(--fg); } .gly-b { fill: var(--bg); }",
        "</style>",
        f'<rect x="0.5" y="0.5" width="{width - 1}" height="{height - 1}" rx="12" '
        'fill="var(--bg)" stroke="var(--line)"/>',
        f'<text class="ttl" x="{x_label}" y="40">WhyCode against AprilTag, in Isaac Sim</text>',
        f'<text class="sub" x="{x_label}" y="60">RealSense D435 colour camera at 1280x720. '
        "A marker's size is the black ring's diameter or the tag's black square, so a tag of "
        "the same size</text>",
        f'<text class="sub" x="{x_label}" y="75">has 27% more black. Range: found, with the '
        "right ID, in 90% of frames out to there; the dashed line runs on to where half are "
        "still found.</text>",
        f'<text class="h" x="{x_bar}" y="112">How far it finds the marker</text>',
        f'<text class="h" x="{x_cpu}" y="112">CPU per frame</text>',
    ]
    # Distance and CPU grids
    bottom = y0 + row_h * len(rows) - 14
    for metres in range(0, axis_m + 1, 5):
        x = x_bar + metres * m_px
        out_lines.append(
            f'<line class="grid" x1="{x:.1f}" y1="{y0 - 22}" x2="{x:.1f}" y2="{bottom}"/>'
        )
        out_lines.append(
            f'<text class="tick" x="{x:.1f}" y="{bottom + 14}" text-anchor="middle">'
            f"{metres} m</text>"
        )
    # Ticks at least 45 px apart
    step = next(t for t in (5, 10, 20, 25, 50, 100) if t * ms_px >= 45)
    for ms in range(0, axis_ms + 1, step):
        x = x_cpu + ms * ms_px
        out_lines.append(
            f'<line class="grid" x1="{x:.1f}" y1="{y0 - 22}" x2="{x:.1f}" y2="{bottom}"/>'
        )
        out_lines.append(
            f'<text class="tick" x="{x:.1f}" y="{bottom + 14}" text-anchor="middle">{ms} ms</text>'
        )
    legend_y = 128
    out_lines += [
        f'<rect x="{x_bar}" y="{legend_y - 8}" width="14" height="8" rx="1.5" fill="var(--fg)"/>',
        f'<text class="tick" x="{x_bar + 19}" y="{legend_y}">0.20 m marker</text>',
        f'<rect x="{x_bar + 100}" y="{legend_y - 7}" width="14" height="6" rx="1.5" '
        'fill="var(--fg)" fill-opacity="0.5"/>',
        f'<text class="tick" x="{x_bar + 119}" y="{legend_y}">0.146 m marker</text>',
        f'<line x1="{x_bar + 212}" y1="{legend_y - 4}" x2="{x_bar + 232}" y2="{legend_y - 4}" '
        'stroke="var(--fg)" stroke-width="2" stroke-dasharray="3 3"/>',
        f'<text class="tick" x="{x_bar + 237}" y="{legend_y}">half still found</text>',
    ]

    for k, (name, title, family) in enumerate(rows):
        y = y0 + k * row_h
        colour = f"var(--{name})"
        out_lines.append(glyph(family, x_label + 8, y + 4))
        out_lines.append(
            f'<text class="lab" x="{x_label + 24}" y="{y + 2}">{title}</text>'
        )
        out_lines.append(
            f'<text class="note" x="{x_label + 24}" y="{y + 16}">{SVG_NOTES[name]}</text>'
        )
        score = entries.get(name)
        if not score:
            out_lines.append(
                f'<text class="val2" x="{x_bar + 6}" y="{y + 8}">not measured</text>'
            )
            continue
        for size, dy, h, opacity in (("0.2", -6, 12, 1.0), ("0.146", 9, 8, 0.5)):
            entry = score["sizes"].get(size)
            if not entry:
                continue
            reliable = entry["reliable_range_m"] or 0.0
            farthest = entry["max_range_m"] or reliable
            x_end = x_bar + reliable * m_px
            if farthest > reliable:
                out_lines.append(
                    f'<line x1="{x_end:.1f}" y1="{y + dy + h / 2:.1f}" '
                    f'x2="{x_bar + farthest * m_px:.1f}" y2="{y + dy + h / 2:.1f}" '
                    f'stroke="{colour}" '
                    f'stroke-opacity="{opacity}" stroke-width="2" stroke-dasharray="3 3"/>'
                )
            out_lines.append(
                f'<rect x="{x_bar}" y="{y + dy}" width="{max(reliable * m_px, 1):.1f}" '
                f'height="{h}" rx="2" fill="{colour}" fill-opacity="{opacity}"/>'
            )
            label_x = x_bar + max(farthest, reliable) * m_px + 6
            css = "val" if size == "0.2" else "val2"
            out_lines.append(
                f'<text class="{css}" x="{label_x:.1f}" y="{y + dy + h - 1}">'
                f"{reliable:.1f} m</text>"
            )
        cpu = score.get("cpu")
        if cpu:
            ms = cpu["cpu_us_per_frame"] / 1000
            out_lines.append(
                f'<rect x="{x_cpu}" y="{y - 4}" width="{max(ms * ms_px, 1):.1f}" '
                f'height="14" rx="2" fill="{colour}"/>'
            )
            gpu = " + GPU" if name == "isaac_apriltag" else ""
            out_lines.append(
                f'<text class="val" x="{x_cpu + ms * ms_px + 6:.1f}" y="{y + 7}">'
                f"{ms:.1f} ms{gpu}</text>"
            )
            out_lines.append(
                f'<text class="val2" x="{x_cpu + ms * ms_px + 6:.1f}" y="{y + 20}">'
                f'{cpu["cpu_percent_at_30hz"]:.0f}% of a core at 30 Hz</text>'
            )

    # Headline numbers, each against AprilTag 3 at apriltag_ros's defaults
    cards = []
    ours = entries.get("whycode")
    theirs, full = entries.get("apriltag"), entries.get("apriltag_full")
    if ours and theirs:

        def reach(score):
            return score["sizes"]["0.2"]["reliable_range_m"] or 0.0

        def cpu(score):
            return score["cpu"]["cpu_us_per_frame"]

        if reach(theirs):
            cards.append(
                (
                    f"{reach(ours) / reach(theirs):.1f}\u00d7",
                    "the range of AprilTag 3, 0.20 m marker;",
                    f"{reach(ours) / reach(full):.1f}\u00d7 at its full resolution"
                    if full and reach(full)
                    else "",
                )
            )
        if "cpu" in ours and "cpu" in theirs:
            cards.append(
                (
                    f"{cpu(theirs) / cpu(ours):.0f}\u00d7",
                    "less CPU per frame than AprilTag 3;",
                    f"{cpu(full) / cpu(ours):.0f}\u00d7 less than at full resolution"
                    if full and "cpu" in full
                    else "",
                )
            )
    errors, families = {}, {n: f for n, _, f in rows}
    for name in ("whycode", "apriltag"):
        rows_csv = results / name / f"range_{families[name]}_1280" / "rows.csv"
        if rows_csv.exists():
            _, range_pct, _ = binned_errors(rows_csv, "0.2", width=8.0, start=2.0)
            if range_pct:
                errors[name] = range_pct[0]
    if "whycode" in errors:
        against = (
            f"against {errors['apriltag']:.1f}% for AprilTag 3"
            if "apriltag" in errors
            else ""
        )
        cards.append(
            (f"{errors['whycode']:.1f}%", "median range error at 2-10 m", against)
        )
    loop = scores.get(("whycode", "whycode_1lap"))
    if loop and "0.2" in loop["sizes"]:
        entry = loop["sizes"]["0.2"]
        cards.append(
            (
                f"{entry['wrong_id']}",
                "wrong IDs over a warehouse lap",
                f"{100 * entry['found_visible']:.1f}% of visible markers found",
            )
        )
    card_y, card_w = height - 128, (width - 2 * x_label - 3 * 20) / 4
    for k, (big, line1, line2) in enumerate(cards[:4]):
        x = x_label + k * (card_w + 20)
        out_lines.append(
            f'<rect x="{x:.1f}" y="{card_y}" width="{card_w:.1f}" height="96" rx="10" '
            'fill="none" stroke="var(--line)"/>'
        )
        out_lines.append(
            f'<text class="big" x="{x + 16:.1f}" y="{card_y + 40}" fill="var(--whycode)">'
            f"{big}</text>"
        )
        out_lines.append(
            f'<text class="cap" x="{x + 16:.1f}" y="{card_y + 64}">{line1}</text>'
        )
        out_lines.append(
            f'<text class="cap" x="{x + 16:.1f}" y="{card_y + 79}">{line2}</text>'
        )
    out_lines.append("</svg>")
    out.write_text("\n".join(out_lines) + "\n")


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    results = Path(argv[0])
    scores = load(results)
    range_plot(results, scores, results / "range.png")
    accuracy_plot(results, scores, results / "accuracy.png")
    angle_plot(results, scores, results / "angle.png")
    (results / "tables.md").write_text(tables(results, scores))
    summary_svg(results, scores, results / "summary.svg")
    print(
        f"{len(scores)} scores -> {results}: range.png, accuracy.png, angle.png, "
        "tables.md, summary.svg"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
