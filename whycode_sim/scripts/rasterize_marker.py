"""Rasterises a marker SVG from whycode_vision's generator into a texture PNG.

The generator (`whycon-id-gen -t <bits>` for WhyCode, `whycon-id-gen -l -t` for WhyCon)
draws only filled circles, rectangles and one path whose teeth are triangles fanned out from
the centre. That is all this reads; no SVG library is needed. The image is drawn at twice the
size and box-filtered down, so edges are antialiased. The textures in textures/ are 7874 px:
0.2 m at 1000 dpi.

    python3 rasterize_marker.py <marker.svg> <texture.png> [--size 7874]
"""

import argparse
import re
import xml.etree.ElementTree as ElementTree

from PIL import Image, ImageDraw

SUPERSAMPLE = 2
COLOURS = {"black": 0, "white": 255}


def fan_triangles(path_data):
    """The path's points split into triangles at each return to its first point."""
    numbers = [float(value) for value in re.findall(r"-?\d+(?:\.\d+)?", path_data)]
    points = list(zip(numbers[0::2], numbers[1::2]))
    centre, triangles, current = points[0], [], []
    for point in points[1:]:
        if point == centre:
            if len(current) == 2:
                triangles.append([centre, *current])
            current = []
        else:
            current.append(point)
    return triangles


def rasterize(svg_path, size_px):
    root = ElementTree.parse(svg_path).getroot()
    x0, y0, width, height = (float(v) for v in root.get("viewBox").split())
    scale = size_px * SUPERSAMPLE / width

    def to_pixels(x, y):
        return ((x - x0) * scale, (y - y0) * scale)

    canvas = Image.new("L", (size_px * SUPERSAMPLE, round(height * scale)), 255)
    draw = ImageDraw.Draw(canvas)
    for element in root:
        tag = element.tag.split("}")[-1]
        fill = COLOURS.get(element.get("fill", "none"))
        if fill is None:
            continue  # Strokes only: the cut marks outside the tight-fit box
        if tag == "rect":
            x, y = float(element.get("x")), float(element.get("y"))
            corner = to_pixels(
                x + float(element.get("width")), y + float(element.get("height"))
            )
            draw.rectangle([to_pixels(x, y), corner], fill=fill)
        elif tag == "circle":
            cx, cy, r = (float(element.get(key)) for key in ("cx", "cy", "r"))
            draw.ellipse(
                [to_pixels(cx - r, cy - r), to_pixels(cx + r, cy + r)], fill=fill
            )
        elif tag == "path":
            for triangle in fan_triangles(element.get("d")):
                draw.polygon([to_pixels(x, y) for x, y in triangle], fill=fill)
    return canvas.resize(
        (size_px, round(height * scale / SUPERSAMPLE)), Image.Resampling.BOX
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("svg")
    parser.add_argument("png")
    parser.add_argument("--size", type=int, default=7874, help="width in pixels")
    args = parser.parse_args()
    texture = rasterize(args.svg, args.size).convert("RGBA")
    texture.save(args.png, dpi=(1000, 1000))
    print(f"{args.png}: {texture.size[0]}x{texture.size[1]}")


if __name__ == "__main__":
    main()
