#!/usr/bin/env python3
"""Generate a blank (all-free) PGM map for turtlebot3_house world.

Usage:
    python3 scripts/generate_blank_map.py [output_dir]

This creates map.pgm and map.yaml in `output_dir` (default: maps/).
The map covers a 20m x 20m area centered at origin with 0.05 m/pixel.

Run this ONLY if you have not yet created a real map via SLAM/Cartographer.
Once you produce a real map, it will overwrite this blank one.
"""

import os
import struct
import sys


def generate_blank_map(output_dir: str = "maps") -> None:
    resolution = 0.05  # m/pixel
    world_size = 20.0   # metres (−10 to +10)
    pixels = int(world_size / resolution)  # 400

    os.makedirs(output_dir, exist_ok=True)

    pgm_path = os.path.join(output_dir, "map.pgm")
    yaml_path = os.path.join(output_dir, "map.yaml")

    # --- PGM (P5 binary) ---
    free_value = 254   # white = free space
    with open(pgm_path, "wb") as f:
        header = f"P5\n{pixels} {pixels}\n255\n"
        f.write(header.encode("ascii"))
        row = bytes([free_value] * pixels)
        for _ in range(pixels):
            f.write(row)

    # --- map.yaml ---
    origin_x = -world_size / 2.0
    origin_y = -world_size / 2.0
    yaml_content = (
        f"image: map.pgm\n"
        f"resolution: {resolution:.6f}\n"
        f"origin: [{origin_x:.6f}, {origin_y:.6f}, 0.000000]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n"
    )
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(yaml_content)

    print(f"[OK] Blank map generated: {pgm_path}  ({pixels}x{pixels}, {resolution} m/px)")
    print(f"[OK] Map YAML: {yaml_path}")


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "maps"
    generate_blank_map(out)
