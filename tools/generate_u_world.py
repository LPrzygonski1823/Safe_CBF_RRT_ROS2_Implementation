#!/usr/bin/env python3
"""Generate maps/map_u and worlds/custom_world_u.sdf from one obstacle list.

The occupancy image keeps the outer wall of maps/map.pgm and replaces the
interior boxes and cylinders with U-shaped walls (three boxes) and one closed
O (four boxes). The SDF keeps the floor, the sun and the wall_box_* shell from
worlds/custom_world_3.sdf and places the same walls as static boxes.

Map frame and Gazebo world differ by a pure y shift: the robot spawns at
world (0, 2), which is map (0, 0). Each obstacle centre (x, y) on the map is
placed at world (x, y + 2).
"""

from __future__ import annotations

import re
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MAP_PGM = ROOT / "maps" / "map.pgm"
MAP_U_PGM = ROOT / "maps" / "map_u.pgm"
MAP_U_YAML = ROOT / "maps" / "map_u.yaml"
WORLD_3 = ROOT / "worlds" / "custom_world_3.sdf"
WORLD_U = ROOT / "worlds" / "custom_world_u.sdf"

RESOLUTION = 0.05
ORIGIN_X = -11.6
ORIGIN_Y = -13.6
WALL_THICKNESS = 0.3
WALL_HEIGHT = 1.0
# map (x, y) -> world (x, y + MAP_TO_WORLD_Y)
MAP_TO_WORLD_Y = 2.0

# name, centre x, centre y, outer width, outer height, opening (None = closed O)
# Centres are in the map frame, metres.
SHAPES = [
    ("A", -6.0, 4.0, 5.0, 4.0, "S"),
    ("B", 6.0, -7.5, 4.0, 5.0, "W"),
    ("O", 6.0, 4.5, 3.2, 3.2, None),
    ("U1", 2.6, 1.0, 2.2, 2.2, "W"),
    ("U2", -2.4, -2.6, 2.2, 2.2, "E"),
    ("U3", 0.8, -4.4, 2.2, 2.2, "S"),
    ("U4", -7.5, -4.5, 2.2, 2.2, "N"),
    ("U5", -3.5, -9.5, 2.2, 2.2, "E"),
]

MODEL_RE = re.compile(r"(    <model name='([^']+)'>.*?\n    </model>\n)", re.DOTALL)


def read_pgm(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    if not raw.startswith(b"P5"):
        raise SystemExit(f"{path} is not a binary PGM")
    tokens: list[bytes] = []
    i = 2
    while len(tokens) < 3:
        while i < len(raw) and raw[i] in b" \t\r\n":
            i += 1
        if i < len(raw) and raw[i] == ord("#"):
            while i < len(raw) and raw[i] != ord("\n"):
                i += 1
            continue
        start = i
        while i < len(raw) and raw[i] not in b" \t\r\n":
            i += 1
        tokens.append(raw[start:i])
    width, height, maxval = (int(token) for token in tokens)
    if maxval != 255:
        raise SystemExit(f"{path} maxval is {maxval}, expected 255")
    if i < len(raw) and raw[i] in b" \t\r\n":
        i += 1
    pixels = np.frombuffer(raw[i:], dtype=np.uint8)
    if pixels.size != width * height:
        raise SystemExit(f"{path} pixel count {pixels.size} != {width}*{height}")
    return pixels.reshape(height, width).copy()


def write_pgm(path: Path, image: np.ndarray) -> None:
    height, width = image.shape
    header = f"P5\n{width} {height}\n255\n".encode()
    path.write_bytes(header + np.ascontiguousarray(image, dtype=np.uint8).tobytes())


def segments(cx: float, cy: float, width: float, height: float, opening: str | None):
    """Axis-aligned wall boxes in the map frame: (side, x, y, size_x, size_y)."""
    x0, x1 = cx - width / 2, cx + width / 2
    y0, y1 = cy - height / 2, cy + height / 2
    thickness = WALL_THICKNESS
    walls = []
    if opening != "S":
        walls.append(("south", cx, y0 + thickness / 2, width, thickness))
    if opening != "N":
        walls.append(("north", cx, y1 - thickness / 2, width, thickness))
    if opening != "W":
        walls.append(("west", x0 + thickness / 2, cy, thickness, height))
    if opening != "E":
        walls.append(("east", x1 - thickness / 2, cy, thickness, height))
    return walls


def build_map(base: np.ndarray) -> np.ndarray:
    height, width = base.shape
    occupied = base == 0
    labels = -np.ones((height, width), dtype=np.int32)
    sizes: list[int] = []
    for row in range(height):
        for col in range(width):
            if not occupied[row, col] or labels[row, col] >= 0:
                continue
            stack = [(row, col)]
            labels[row, col] = len(sizes)
            count = 0
            while stack:
                r, c = stack.pop()
                count += 1
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nr, nc = r + dr, c + dc
                    if (
                        0 <= nr < height
                        and 0 <= nc < width
                        and occupied[nr, nc]
                        and labels[nr, nc] < 0
                    ):
                        labels[nr, nc] = len(sizes)
                        stack.append((nr, nc))
            sizes.append(count)
    wall_id = int(np.argmax(sizes))
    image = base.copy()
    image[occupied & (labels != wall_id)] = 254

    cols = ORIGIN_X + (np.arange(width) + 0.5) * RESOLUTION
    rows = ORIGIN_Y + (height - 1 - np.arange(height) + 0.5) * RESOLUTION
    grid_x, grid_y = np.meshgrid(cols, rows)

    def fill(x0: float, x1: float, y0: float, y1: float) -> None:
        image[(grid_x >= x0) & (grid_x <= x1) & (grid_y >= y0) & (grid_y <= y1)] = 0

    for _name, cx, cy, width_m, height_m, opening in SHAPES:
        x0, x1 = cx - width_m / 2, cx + width_m / 2
        y0, y1 = cy - height_m / 2, cy + height_m / 2
        if opening != "S":
            fill(x0, x1, y0, y0 + WALL_THICKNESS)
        if opening != "N":
            fill(x0, x1, y1 - WALL_THICKNESS, y1)
        if opening != "W":
            fill(x0, x0 + WALL_THICKNESS, y0, y1)
        if opening != "E":
            fill(x1 - WALL_THICKNESS, x1, y0, y1)
    return image


def box_model(name: str, x: float, y: float, size_x: float, size_y: float) -> str:
    return f"""    <model name='{name}'>
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {WALL_HEIGHT / 2:.4f} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <box>
              <size>{size_x:.4f} {size_y:.4f} {WALL_HEIGHT:.4f}</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>{size_x:.4f} {size_y:.4f} {WALL_HEIGHT:.4f}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>1 1 1 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def obstacle_models() -> str:
    blocks = [
        "    <!-- Interior obstacles from tools/generate_u_world.py.\n"
        "         Map (x, y) is world (x, y + 2): the robot spawns at world (0, 2). -->\n"
    ]
    for name, cx, cy, width, height, opening in SHAPES:
        for side, x, y, size_x, size_y in segments(cx, cy, width, height, opening):
            blocks.append(
                box_model(
                    f"u_{name}_{side}",
                    x,
                    y + MAP_TO_WORLD_Y,
                    size_x,
                    size_y,
                )
            )
    return "\n".join(blocks)


def build_world() -> str:
    text = WORLD_3.read_text()
    models = list(MODEL_RE.finditer(text))
    if not models:
        raise SystemExit(f"no models in {WORLD_3}")
    kept_ground = []
    kept_walls = []
    dropped = []
    for match in models:
        name = match.group(2)
        if name in ("ground_plane", "ground_plane_1"):
            kept_ground.append(match.group(1))
        elif name.startswith("wall_box"):
            kept_walls.append(match.group(1))
        elif name == "box" or name.startswith("box_") or name == "cylinder" or name.startswith("cylinder_"):
            dropped.append(name)
        else:
            raise SystemExit(f"unexpected model {name!r} in {WORLD_3}")
    if not kept_walls:
        raise SystemExit("no wall_box models kept")
    preamble = text[: models[0].start()]
    trailer = text[models[-1].end() :]
    return (
        preamble
        + "".join(kept_ground)
        + "\n"
        + obstacle_models()
        + "\n"
        + "".join(kept_walls)
        + trailer
    )


def write_yaml() -> None:
    MAP_U_YAML.write_text(
        "image: map_u.pgm\n"
        "mode: trinary\n"
        "resolution: 0.05\n"
        "origin: [-11.6, -13.6, 0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
    )


def main() -> None:
    image = build_map(read_pgm(MAP_PGM))
    write_pgm(MAP_U_PGM, image)
    write_yaml()
    world = build_world()
    WORLD_U.write_text(world)
    n_boxes = world.count("<model name='u_")
    n_walls = world.count("<model name='wall_box_")
    print(f"wrote {MAP_U_PGM.relative_to(ROOT)} ({image.shape[1]}x{image.shape[0]})")
    print(f"wrote {MAP_U_YAML.relative_to(ROOT)}")
    print(f"wrote {WORLD_U.relative_to(ROOT)}: {n_boxes} obstacle boxes, {n_walls} outer wall boxes")


if __name__ == "__main__":
    main()
