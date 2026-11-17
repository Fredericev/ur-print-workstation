#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def build_toolpath(box_x_mm, box_y_mm, box_z_mm, layer_height_mm, line_spacing_mm, center_x_m, center_y_m, base_z_m):
    half_x = box_x_mm / 2000.0
    half_y = box_y_mm / 2000.0
    min_x = center_x_m - half_x
    max_x = center_x_m + half_x
    min_y = center_y_m - half_y
    max_y = center_y_m + half_y
    layer_count = max(1, math.ceil(box_z_mm / layer_height_mm))
    rows = []
    for layer in range(layer_count):
        z = base_z_m + (layer * layer_height_mm / 1000.0)
        index = 0
        def push(x, y, segment):
            nonlocal index
            rows.append({
                'layer': layer,
                'index': index,
                'segment': segment,
                'x_m': x,
                'y_m': y,
                'z_m': z,
            })
            index += 1
        push(min_x, min_y, 'perimeter')
        push(max_x, min_y, 'perimeter')
        push(max_x, max_y, 'perimeter')
        push(min_x, max_y, 'perimeter')
        push(min_x, min_y, 'perimeter_close')
        left_to_right = True
        y = min_y + line_spacing_mm / 1000.0
        while y <= max_y - line_spacing_mm / 1000.0 + 1e-9:
            if left_to_right:
                push(min_x, y, 'infill')
                push(max_x, y, 'infill')
            else:
                push(max_x, y, 'infill')
                push(min_x, y, 'infill')
            left_to_right = not left_to_right
            y += line_spacing_mm / 1000.0
    return rows


def write_outputs(rows, outdir: Path, plan_name: str, box_x_mm, box_y_mm, box_z_mm, layer_height_mm, line_spacing_mm, center_x_m, center_y_m, base_z_m, roll_deg, pitch_deg, yaw_deg):
    outdir.mkdir(parents=True, exist_ok=True)
    csv_path = outdir / f"{plan_name}_toolpath.csv"
    with csv_path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['layer', 'index', 'segment', 'x_m', 'y_m', 'z_m'])
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    xs = [r['x_m'] * 1000 for r in rows]
    ys = [r['y_m'] * 1000 for r in rows]
    zs = [r['z_m'] * 1000 for r in rows]
    summary_path = outdir / f"{plan_name}_summary.yaml"
    summary_path.write_text(
        '\n'.join([
            f'plan_name: {plan_name}',
            'mode: cartesian_print_toolpath',
            'robot_tcp_assumption: tool0_as_nozzle',
            f'box_size_mm: [{box_x_mm:.1f}, {box_y_mm:.1f}, {box_z_mm:.1f}]',
            f'layer_height_mm: {layer_height_mm:.1f}',
            f'line_spacing_mm: {line_spacing_mm:.1f}',
            f'layer_count: {len(set(r["layer"] for r in rows))}',
            f'waypoint_count: {len(rows)}',
            f'print_center_base_m: [{center_x_m:.3f}, {center_y_m:.3f}, {base_z_m:.3f}]',
            f'orientation_rpy_deg: [{roll_deg:.1f}, {pitch_deg:.1f}, {yaw_deg:.1f}]',
            f'xy_bounds_mm: [{min(xs):.1f}, {max(xs):.1f}, {min(ys):.1f}, {max(ys):.1f}]',
            f'z_range_mm: [{min(zs):.1f}, {max(zs):.1f}]',
        ]) + '\n'
    )

    scale = 6
    margin = 20
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    width = int((max_x - min_x) * scale + 2 * margin)
    height = int((max_y - min_y) * scale + 2 * margin)
    layer0 = [r for r in rows if r['layer'] == 0]
    def point(row):
        x = (row['x_m'] * 1000 - min_x) * scale + margin
        y = height - ((row['y_m'] * 1000 - min_y) * scale + margin)
        return f"{x:.1f},{y:.1f}"
    polyline = ' '.join(point(row) for row in layer0)
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect x="0" y="0" width="{width}" height="{height}" fill="white"/>
  <rect x="{margin}" y="{margin}" width="{(max_x-min_x)*scale:.1f}" height="{(max_y-min_y)*scale:.1f}" fill="none" stroke="#d0d0d0" stroke-width="1"/>
  <polyline points="{polyline}" fill="none" stroke="#0b57d0" stroke-width="2"/>
  <text x="12" y="16" font-family="Arial, sans-serif" font-size="12">Layer 1 top view · {box_x_mm:.0f}mm x {box_y_mm:.0f}mm toolpath</text>
</svg>'''
    (outdir / f"{plan_name}_layer1_preview.svg").write_text(svg)
    return csv_path, summary_path


def main():
    parser = argparse.ArgumentParser(description='Generate a simple 3D-print-style Cartesian toolpath for UR3e.')
    parser.add_argument('--box-x-mm', type=float, default=50.0)
    parser.add_argument('--box-y-mm', type=float, default=50.0)
    parser.add_argument('--box-z-mm', type=float, default=10.0)
    parser.add_argument('--layer-height-mm', type=float, default=2.0)
    parser.add_argument('--line-spacing-mm', type=float, default=5.0)
    parser.add_argument('--center-x-m', type=float, default=0.20)
    parser.add_argument('--center-y-m', type=float, default=-0.25)
    parser.add_argument('--base-z-m', type=float, default=0.20)
    parser.add_argument('--roll-deg', type=float, default=180.0)
    parser.add_argument('--pitch-deg', type=float, default=0.0)
    parser.add_argument('--yaw-deg', type=float, default=-90.0)
    parser.add_argument('--plan-name', default='print_box_50x50x10')
    parser.add_argument('--output-dir', default=str(Path(__file__).resolve().parents[1] / 'workspaces/ur_ros2_ws/output'))
    args = parser.parse_args()

    rows = build_toolpath(
        args.box_x_mm,
        args.box_y_mm,
        args.box_z_mm,
        args.layer_height_mm,
        args.line_spacing_mm,
        args.center_x_m,
        args.center_y_m,
        args.base_z_m,
    )
    csv_path, summary_path = write_outputs(
        rows,
        Path(args.output_dir),
        args.plan_name,
        args.box_x_mm,
        args.box_y_mm,
        args.box_z_mm,
        args.layer_height_mm,
        args.line_spacing_mm,
        args.center_x_m,
        args.center_y_m,
        args.base_z_m,
        args.roll_deg,
        args.pitch_deg,
        args.yaw_deg,
    )
    print(csv_path)
    print(summary_path)


if __name__ == '__main__':
    main()
