#!/usr/bin/env python3
"""Export the STEP-derived triangulation as ROS-scale STL/DAE meshes.

The input JSON is an intermediate tessellation of the STEP assembly.  Its
coordinates are already converted from STEP millimetres to metres.  This
script intentionally preserves the CAD origin and axes; the mounting transform
belongs in URDF, not in the mesh.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
from pathlib import Path
from typing import Iterable, Sequence
from xml.sax.saxutils import escape


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def bounds(points: Sequence[float]) -> tuple[list[float], list[float]]:
    return (
        [min(points[axis::3]) for axis in range(3)],
        [max(points[axis::3]) for axis in range(3)],
    )


def triangle_normal(a: Sequence[float], b: Sequence[float], c: Sequence[float]) -> tuple[float, float, float, float]:
    ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    ac = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
    cross = (
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    )
    twice_area = math.sqrt(sum(value * value for value in cross))
    if twice_area == 0.0:
        return 0.0, 0.0, 0.0, 0.0
    return (
        cross[0] / twice_area,
        cross[1] / twice_area,
        cross[2] / twice_area,
        0.5 * twice_area,
    )


def iter_triangles(meshes: Sequence[dict]) -> Iterable[tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]]:
    for mesh in meshes:
        positions = mesh["positions"]
        indices = mesh["indices"]
        for offset in range(0, len(indices), 3):
            triangle = []
            for index in indices[offset : offset + 3]:
                start = 3 * index
                triangle.append(tuple(positions[start : start + 3]))
            yield triangle[0], triangle[1], triangle[2]


def iter_nondegenerate_triangles(meshes: Sequence[dict]) -> Iterable[tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]]:
    for triangle in iter_triangles(meshes):
        if triangle_normal(*triangle)[3] > 1.0e-18:
            yield triangle


def write_binary_stl(path: Path, meshes: Sequence[dict]) -> dict:
    source_triangle_count = sum(len(mesh["indices"]) // 3 for mesh in meshes)
    triangle_count = sum(1 for _ in iter_nondegenerate_triangles(meshes))
    degenerate_count = source_triangle_count - triangle_count
    surface_area_m2 = 0.0
    header = f"{path.stem} STEP tessellation; units=m; CAD axes preserved".encode("ascii", errors="replace")
    header = header[:80].ljust(80, b"\0")
    with path.open("wb") as stream:
        stream.write(header)
        stream.write(struct.pack("<I", triangle_count))
        for a, b, c in iter_nondegenerate_triangles(meshes):
            nx, ny, nz, area = triangle_normal(a, b, c)
            surface_area_m2 += area
            stream.write(struct.pack("<12fH", nx, ny, nz, *a, *b, *c, 0))
    expected_size = 84 + triangle_count * 50
    if path.stat().st_size != expected_size:
        raise RuntimeError(f"unexpected STL size: {path.stat().st_size} != {expected_size}")
    return {
        "format": "binary STL",
        "units": "metre (STL is unitless; values are metres for ROS)",
        "triangles": triangle_count,
        "degenerate_source_triangles_removed": degenerate_count,
        "surface_area_m2": surface_area_m2,
        "bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def float_text(values: Sequence[float]) -> str:
    return " ".join(format(value, ".9g") for value in values)


def write_collada(path: Path, meshes: Sequence[dict]) -> dict:
    positions: list[float] = []
    normals: list[float] = []
    indices: list[int] = []
    vertex_offset = 0
    for mesh in meshes:
        mesh_positions = mesh["positions"]
        mesh_normals = mesh.get("normals", [])
        if len(mesh_normals) != len(mesh_positions):
            mesh_normals = [0.0] * len(mesh_positions)
        positions.extend(mesh_positions)
        normals.extend(mesh_normals)
        mesh_positions = mesh["positions"]
        for offset in range(0, len(mesh["indices"]), 3):
            triangle_indices = mesh["indices"][offset : offset + 3]
            triangle = []
            for index in triangle_indices:
                start = 3 * index
                triangle.append(tuple(mesh_positions[start : start + 3]))
            if triangle_normal(*triangle)[3] > 1.0e-18:
                indices.extend(index + vertex_offset for index in triangle_indices)
        vertex_offset += len(mesh_positions) // 3

    interleaved_indices = " ".join(f"{index} {index}" for index in indices)
    triangle_count = len(indices) // 3
    vertex_count = len(positions) // 3
    mesh_name = escape(path.stem, {'"': "&quot;"})
    document = f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor><authoring_tool>ur3e_printhead_description export_step_visual_mesh.py</authoring_tool></contributor>
    <unit name="meter" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_effects>
    <effect id="printhead-effect"><profile_COMMON><technique sid="common"><phong>
      <diffuse><color>0.32 0.38 0.44 1</color></diffuse>
      <ambient><color>0.08 0.095 0.11 1</color></ambient>
      <specular><color>0.18 0.18 0.18 1</color></specular>
      <shininess><float>18</float></shininess>
    </phong></technique></profile_COMMON></effect>
  </library_effects>
  <library_materials><material id="printhead-material" name="printhead"><instance_effect url="#printhead-effect"/></material></library_materials>
  <library_geometries>
    <geometry id="printhead-mesh" name="{mesh_name}">
      <mesh>
        <source id="printhead-positions">
          <float_array id="printhead-positions-array" count="{len(positions)}">{float_text(positions)}</float_array>
          <technique_common><accessor source="#printhead-positions-array" count="{vertex_count}" stride="3">
            <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
          </accessor></technique_common>
        </source>
        <source id="printhead-normals">
          <float_array id="printhead-normals-array" count="{len(normals)}">{float_text(normals)}</float_array>
          <technique_common><accessor source="#printhead-normals-array" count="{vertex_count}" stride="3">
            <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
          </accessor></technique_common>
        </source>
        <vertices id="printhead-vertices"><input semantic="POSITION" source="#printhead-positions"/></vertices>
        <triangles material="printhead-material-symbol" count="{triangle_count}">
          <input semantic="VERTEX" source="#printhead-vertices" offset="0"/>
          <input semantic="NORMAL" source="#printhead-normals" offset="1"/>
          <p>{interleaved_indices}</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene"><node id="printhead" name="{mesh_name}">
      <instance_geometry url="#printhead-mesh"><bind_material><technique_common>
        <instance_material symbol="printhead-material-symbol" target="#printhead-material"/>
      </technique_common></bind_material></instance_geometry>
    </node></visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>
'''
    path.write_text(document, encoding="utf-8")
    return {
        "format": "COLLADA 1.4.1",
        "units": "metre (asset unit meter=1)",
        "vertices": vertex_count,
        "triangles": triangle_count,
        "bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def flange_reference(meshes: Sequence[dict], mounting_plane_y_m: float | None = None) -> dict:
    base = next(mesh for mesh in meshes if mesh["name"].casefold() == "base")
    positions = base["positions"]
    # The base's outermost +Y face is the mounting-face candidate. In the new
    # assembly it is y=+5 mm; older exports used y=0. An explicit override is
    # available for a future STEP whose contact face is not the +Y extremum.
    plane_y = max(positions[1::3]) if mounting_plane_y_m is None else mounting_plane_y_m
    if not math.isfinite(plane_y):
        raise RuntimeError("mounting plane must be finite")
    face_points = [
        (positions[offset], positions[offset + 2])
        for offset in range(0, len(positions), 3)
        if abs(positions[offset + 1] - plane_y) <= 1.0e-8
    ]
    if not face_points:
        raise RuntimeError(f"no Base vertices found on CAD y={plane_y:g} mounting plane")
    min_x = min(point[0] for point in face_points)
    max_x = max(point[0] for point in face_points)
    min_z = min(point[1] for point in face_points)
    max_z = max(point[1] for point in face_points)
    center_x = 0.5 * (min_x + max_x)
    center_z = 0.5 * (min_z + max_z)
    radius_x = 0.5 * (max_x - min_x)
    radius_z = 0.5 * (max_z - min_z)
    outer_radius = 0.5 * (radius_x + radius_z)
    outer = [
        math.hypot(x - center_x, z - center_z)
        for x, z in face_points
        if abs(math.hypot(x - center_x, z - center_z) - outer_radius) <= 1.0e-7
    ]
    return {
        "status": "cad_candidate_not_physically_verified",
        "mounting_plane_cad": f"y = {plane_y:g} m",
        "mounting_plane_y_m": plane_y,
        "assembly_side": "geometry extends toward CAD -Y",
        "center_cad_m": [center_x, plane_y, center_z],
        "outer_radius_m": outer_radius,
        "outer_circle_sample_count": len(outer),
        "outer_circle_max_residual_m": max((abs(radius - outer_radius) for radius in outer), default=None),
        "derivation": "extents and outer-circle samples of Base mesh vertices on the candidate mounting plane",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step", type=Path, required=True)
    parser.add_argument("--tessellation", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--mounting-plane-y-m", type=float,
                        help="Candidate CAD mounting plane; default is the Base mesh's maximum Y.")
    args = parser.parse_args()

    data = json.loads(args.tessellation.read_text(encoding="utf-8"))
    if data.get("source_units") != "millimeter" or data.get("output_units") != "meter":
        raise RuntimeError("tessellation must be STEP millimetres converted to output metres")
    step_hash = sha256(args.step)
    if step_hash != data.get("source_sha256"):
        raise RuntimeError("tessellation source SHA-256 does not match the requested STEP file")

    meshes = data["meshes"]
    all_positions = [value for mesh in meshes for value in mesh["positions"]]
    minimum, maximum = bounds(all_positions)
    expected_minimum = data["bounds"]["minimum"]
    expected_maximum = data["bounds"]["maximum"]
    if minimum != expected_minimum or maximum != expected_maximum:
        raise RuntimeError("calculated mesh bounds differ from tessellation metadata")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    stl_path = args.output_dir / f"{args.step.stem}.stl"
    dae_path = args.output_dir / f"{args.step.stem}.dae"
    stl = write_binary_stl(stl_path, meshes)
    dae = write_collada(dae_path, meshes)
    manifest = {
        "purpose": "visual-only ROS/RViz mesh; no collision or TCP is defined here",
        "coordinate_convention": "STEP/CAD origin and XYZ axes preserved; numeric coordinates are metres",
        "source": {
            "step_path": str(args.step),
            "step_sha256": step_hash,
            "step_units": data["source_units"],
            "tessellation_path": str(args.tessellation),
            "tessellation_sha256": sha256(args.tessellation),
            "component_mesh_count": len(meshes),
        },
        "geometry": {
            "vertices_with_per_face_duplication": sum(len(mesh["positions"]) // 3 for mesh in meshes),
            "source_triangles": sum(len(mesh["indices"]) // 3 for mesh in meshes),
            "output_triangles_after_removing_degenerate_faces": stl["triangles"],
            "bounds_minimum_cad_m": minimum,
            "bounds_maximum_cad_m": maximum,
            "size_xyz_m": [maximum[axis] - minimum[axis] for axis in range(3)],
            "flange_reference": flange_reference(meshes, args.mounting_plane_y_m),
        },
        "outputs": {
            stl_path.name: stl,
            dae_path.name: dae,
        },
    }
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
