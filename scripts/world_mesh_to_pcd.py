#!/usr/bin/env python3
"""Sample a Gazebo world mesh into a PCD point cloud.

This utility is meant for the current sentry workflow:
1. Read a mesh referenced by a Gazebo Classic world/model.
2. Apply the same SDF poses used in Gazebo.
3. Sample a point cloud and save it as .pcd for pcd2pgm / ICP usage.

The important bit is that the resulting PCD stays in the same coordinate
system as the Gazebo world, so later-generated pgm/yaml maps can align with
the simulation scene.
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import open3d as o3d


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world-file", required=True, help="Path to a .world file")
    parser.add_argument(
        "--model-name",
        default="",
        help="Model name to sample. Defaults to the first non-ground/static scene model with a mesh.",
    )
    parser.add_argument("--output", required=True, help="Output .pcd path")
    parser.add_argument(
        "--points",
        type=int,
        default=300000,
        help="Number of surface sample points before optional voxel downsampling",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.02,
        help="Optional voxel downsampling size in meters. Use 0 to disable downsampling.",
    )
    return parser.parse_args()


def pose_text_to_matrix(text: str | None) -> np.ndarray:
    if not text:
        return np.eye(4, dtype=float)

    values = [float(v) for v in text.split()]
    if len(values) != 6:
        raise ValueError(f"Expected 6 pose values, got {len(values)} from '{text}'")

    x, y, z, roll, pitch, yaw = values

    cx, cy, cz = math.cos(roll), math.cos(pitch), math.cos(yaw)
    sx, sy, sz = math.sin(roll), math.sin(pitch), math.sin(yaw)

    rot_x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=float)
    rot_y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=float)
    rot_z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=float)
    rotation = rot_z @ rot_y @ rot_x

    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = [x, y, z]
    return matrix


def local_name(tag: str) -> str:
    if "}" in tag:
        return tag.split("}", 1)[1]
    return tag


def child_with_name(element: ET.Element, tag_name: str, attr_name: str | None = None) -> ET.Element | None:
    for child in element:
        if local_name(child.tag) != tag_name:
            continue
        if attr_name is None or child.attrib.get("name") == attr_name:
            return child
    return None


def iter_children(element: ET.Element, tag_name: str) -> list[ET.Element]:
    return [child for child in element if local_name(child.tag) == tag_name]


def find_target_model(world_root: ET.Element, model_name: str) -> ET.Element:
    models = iter_children(world_root, "model")
    if not models:
        raise RuntimeError("No <model> found in world file")

    if model_name:
        for model in models:
            if model.attrib.get("name") == model_name:
                return model
        raise RuntimeError(f"Model '{model_name}' not found in world file")

    for model in models:
        name = model.attrib.get("name", "")
        if name == "ground_plane":
            continue
        if child_with_name(model, "link") is not None:
            return model

    raise RuntimeError("No suitable model with a mesh-bearing link found in world file")


def resolve_model_uri(uri: str, world_file: Path) -> Path:
    if uri.startswith("file://"):
        return Path(uri.removeprefix("file://"))

    if uri.startswith("model://"):
        relative = uri.removeprefix("model://")
        parts = relative.split("/", 1)
        model_dir = parts[0]
        suffix = parts[1] if len(parts) > 1 else ""

        package_root = world_file.parents[2]

        # Current pb_rm_simulation layout:
        #   package_root/world/<model_dir>/...
        #   package_root/meshes/<model_dir>/...
        candidates = [
            package_root / "meshes" / model_dir / suffix,
            package_root / "world" / model_dir / suffix,
            package_root / model_dir / suffix,
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate

    path = Path(uri)
    if path.exists():
        return path

    raise FileNotFoundError(f"Could not resolve mesh uri '{uri}' from {world_file}")


def scale_text_to_matrix(text: str | None) -> np.ndarray:
    if not text:
        return np.eye(4, dtype=float)

    values = [float(v) for v in text.split()]
    if len(values) != 3:
        raise ValueError(f"Expected 3 scale values, got {len(values)} from '{text}'")

    sx, sy, sz = values
    matrix = np.eye(4, dtype=float)
    matrix[0, 0] = sx
    matrix[1, 1] = sy
    matrix[2, 2] = sz
    return matrix


def find_mesh_and_transform(model: ET.Element, world_file: Path) -> tuple[Path, np.ndarray]:
    model_pose = pose_text_to_matrix(child_with_name(model, "pose").text if child_with_name(model, "pose") is not None else None)

    for link in iter_children(model, "link"):
        link_pose = pose_text_to_matrix(child_with_name(link, "pose").text if child_with_name(link, "pose") is not None else None)

        for visual in iter_children(link, "visual"):
            visual_pose = pose_text_to_matrix(child_with_name(visual, "pose").text if child_with_name(visual, "pose") is not None else None)
            geometry = child_with_name(visual, "geometry")
            if geometry is None:
                continue
            mesh = child_with_name(geometry, "mesh")
            if mesh is None:
                continue
            uri_node = child_with_name(mesh, "uri")
            if uri_node is None or not uri_node.text:
                continue
            scale_node = child_with_name(mesh, "scale")
            mesh_path = resolve_model_uri(uri_node.text.strip(), world_file)
            scale = scale_text_to_matrix(scale_node.text if scale_node is not None else None)
            transform = model_pose @ link_pose @ visual_pose @ scale
            return mesh_path, transform

    raise RuntimeError("No mesh found in the selected model")


def sample_mesh_to_pcd(mesh_path: Path, transform: np.ndarray, point_count: int, voxel_size: float) -> o3d.geometry.PointCloud:
    mesh = o3d.io.read_triangle_mesh(str(mesh_path))
    if mesh.is_empty():
        raise RuntimeError(f"Failed to read mesh: {mesh_path}")
    if not mesh.has_triangles():
        raise RuntimeError(f"Mesh has no triangles: {mesh_path}")

    cloud = mesh.sample_points_uniformly(number_of_points=point_count)
    cloud.transform(transform)

    if voxel_size > 0.0:
        cloud = cloud.voxel_down_sample(voxel_size=voxel_size)

    if cloud.is_empty():
        raise RuntimeError("Sampled point cloud is empty")
    return cloud


def main() -> int:
    args = parse_args()

    world_file = Path(args.world_file).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    tree = ET.parse(world_file)
    root = tree.getroot()
    world = child_with_name(root, "world")
    if world is None:
        raise RuntimeError(f"No <world> element found in {world_file}")

    model = find_target_model(world, args.model_name)
    mesh_path, transform = find_mesh_and_transform(model, world_file)

    cloud = sample_mesh_to_pcd(mesh_path, transform, args.points, args.voxel_size)
    if not o3d.io.write_point_cloud(str(output_path), cloud, write_ascii=False):
        raise RuntimeError(f"Failed to write point cloud: {output_path}")

    pts = np.asarray(cloud.points)
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)

    print(f"world_file : {world_file}")
    print(f"model_name : {model.attrib.get('name', '')}")
    print(f"mesh_path  : {mesh_path}")
    print(f"output_pcd : {output_path}")
    print(f"points     : {len(pts)}")
    print(f"bbox_min   : {mins.tolist()}")
    print(f"bbox_max   : {maxs.tolist()}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # pragma: no cover - CLI error path
        print(f"[ERROR] {exc}", file=sys.stderr)
        raise SystemExit(1)
