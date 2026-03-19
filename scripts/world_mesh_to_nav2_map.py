#!/usr/bin/env python3
"""Generate a Nav2 pgm/yaml map directly from a Gazebo world mesh.

This script is intended for "capped" field meshes where sampling a point cloud
and projecting by height makes the whole field appear occupied. Instead of using
the top surface, it samples the mesh and tracks the lowest sampled surface per
grid cell. Cells whose lowest surface is significantly lower than the dominant
walkable floor level are marked occupied.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image
import trimesh

from world_mesh_to_pcd import find_mesh_and_transform, find_target_model, child_with_name
from world_mesh_to_pcd import local_name  # noqa: F401 imported for ET parsing consistency
import xml.etree.ElementTree as ET


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world-file", required=True, help="Path to Gazebo .world file")
    parser.add_argument("--model-name", default="", help="Model name to rasterize")
    parser.add_argument("--output-prefix", required=True, help="Output path without extension")
    parser.add_argument("--resolution", type=float, default=0.05, help="Map resolution in meters")
    parser.add_argument(
        "--samples",
        type=int,
        default=2500000,
        help="Number of mesh surface samples used to estimate the occupancy",
    )
    parser.add_argument(
        "--depth-threshold",
        type=float,
        default=0.05,
        help="Cells with min_z < inferred_floor_z - threshold are marked occupied",
    )
    parser.add_argument(
        "--height-threshold",
        type=float,
        default=0.15,
        help="For protruding-obstacle meshes, cells with max_z > floor_z + threshold are marked occupied",
    )
    parser.add_argument(
        "--mode",
        choices=("auto", "cavity", "protrusion"),
        default="auto",
        help="Occupancy inference mode. Use cavity/protrusion to bypass auto detection.",
    )
    return parser.parse_args()


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    ones = np.ones((points.shape[0], 1), dtype=np.float64)
    homo = np.hstack([points, ones])
    transformed = (transform @ homo.T).T
    return transformed[:, :3]


def save_yaml(yaml_path: Path, image_name: str, resolution: float, origin: tuple[float, float]) -> None:
    yaml_path.write_text(
        "\n".join(
            [
                f"image: {image_name}",
                "mode: trinary",
                f"resolution: {resolution}",
                f"origin: [{origin[0]:.2f}, {origin[1]:.2f}, 0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
                "",
            ]
        )
    )


def main() -> int:
    args = parse_args()

    world_file = Path(args.world_file).expanduser().resolve()
    output_prefix = Path(args.output_prefix).expanduser().resolve()
    output_prefix.parent.mkdir(parents=True, exist_ok=True)

    tree = ET.parse(world_file)
    root = tree.getroot()
    world = child_with_name(root, "world")
    if world is None:
      raise RuntimeError(f"No <world> element found in {world_file}")

    model = find_target_model(world, args.model_name)
    mesh_path, transform = find_mesh_and_transform(model, world_file)

    mesh = trimesh.load(mesh_path, force="mesh")
    if mesh.is_empty:
        raise RuntimeError(f"Failed to load mesh: {mesh_path}")

    points, _ = trimesh.sample.sample_surface(mesh, args.samples)
    points = transform_points(points, transform)

    xy_min = points[:, :2].min(axis=0)
    xy_max = points[:, :2].max(axis=0)
    width = int(np.ceil((xy_max[0] - xy_min[0]) / args.resolution))
    height = int(np.ceil((xy_max[1] - xy_min[1]) / args.resolution))

    min_z = np.full((height, width), np.inf, dtype=np.float64)
    max_z = np.full((height, width), -np.inf, dtype=np.float64)

    ix = np.floor((points[:, 0] - xy_min[0]) / args.resolution).astype(int)
    iy = np.floor((points[:, 1] - xy_min[1]) / args.resolution).astype(int)
    valid = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    ix = ix[valid]
    iy = iy[valid]
    z = points[valid, 2]
    grid_y = height - 1 - iy

    for x, y, zz in zip(ix, grid_y, z):
        if zz < min_z[y, x]:
            min_z[y, x] = zz
        if zz > max_z[y, x]:
            max_z[y, x] = zz

    if not np.isfinite(min_z).any():
        raise RuntimeError("No valid cells were rasterized from the mesh")

    finite_mask = np.isfinite(min_z)
    floor_z = float(np.quantile(min_z[finite_mask], 0.75))
    top_q25 = float(np.quantile(max_z[finite_mask], 0.25))

    # Two common field-mesh styles:
    # 1. "cavity" mesh: the field is a capped solid, obstacles are represented by
    #    deeper cut-outs, so occupied cells have significantly smaller min_z.
    # 2. "protrusion" mesh: obstacles extend upward from the floor, so occupied
    #    cells have significantly larger max_z.
    if args.mode == "cavity":
        cavity_mode = True
    elif args.mode == "protrusion":
        cavity_mode = False
    else:
        cavity_mode = floor_z > args.depth_threshold

    grid = np.full((height, width), 254, dtype=np.uint8)
    if cavity_mode:
        obstacle_limit = floor_z - args.depth_threshold
        grid[min_z < obstacle_limit] = 0
        mode_label = "cavity"
        mode_summary = f"cavity (occupied if min_z < {obstacle_limit:.3f})"
    else:
        obstacle_limit = floor_z + args.height_threshold
        grid[max_z > obstacle_limit] = 0
        mode_label = "protrusion"
        mode_summary = f"protrusion (occupied if max_z > {obstacle_limit:.3f})"

    pgm_path = output_prefix.with_suffix(".pgm")
    yaml_path = output_prefix.with_suffix(".yaml")
    Image.fromarray(grid, mode="L").save(pgm_path)
    save_yaml(yaml_path, pgm_path.name, args.resolution, (float(xy_min[0]), float(xy_min[1])))

    print(f"world_file      : {world_file}")
    print(f"model_name      : {model.attrib.get('name', '')}")
    print(f"mesh_path       : {mesh_path}")
    print(f"output_pgm      : {pgm_path}")
    print(f"output_yaml     : {yaml_path}")
    print(f"resolution      : {args.resolution}")
    print(f"grid_size       : {width} x {height}")
    print(f"origin_xy       : [{xy_min[0]:.3f}, {xy_min[1]:.3f}]")
    print(f"inferred_floor_z: {floor_z:.3f}")
    print(f"top_q25         : {top_q25:.3f}")
    print(f"selected_mode   : {mode_label}")
    print(f"mode            : {mode_summary}")
    print(f"occupied_cells  : {int((grid == 0).sum())}")
    print(f"free_cells      : {int((grid == 254).sum())}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
