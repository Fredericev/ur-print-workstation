#!/usr/bin/env python3
"""Publish the fixed tabletop and an optional G-code job preview for RViz."""

from __future__ import annotations

import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "workcell_geometry.json"
JOB_PREVIEW_PATH = (
    Path(__file__).resolve().parents[1] / "runtime" / "rviz_job_preview.json"
)


def load_tabletop() -> tuple[str, dict]:
    value = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    if value.get("schema_version") != 1 or value.get("frame_id") != "base_link":
        raise ValueError("workcell geometry must use schema 1 in base_link")
    tabletop = value["tabletop"]
    if float(tabletop["yaw_deg"]) != 0.0:
        raise ValueError("RViz fixed tabletop currently requires yaw 0 degrees")
    if float(tabletop["dimensions_m"]["x"]) <= 0.0 or float(
        tabletop["dimensions_m"]["y"]
    ) <= 0.0:
        raise ValueError("tabletop dimensions must be positive")
    return value["frame_id"], tabletop


def load_job_preview(frame_id: str, tabletop: dict) -> dict | None:
    if not JOB_PREVIEW_PATH.exists():
        return None
    value = json.loads(JOB_PREVIEW_PATH.read_text(encoding="utf-8"))
    if value.get("schema_version") != 1 or value.get("frame_id") != frame_id:
        raise ValueError("RViz job preview must use schema 1 and the tabletop frame")

    preview_table = value.get("table", {})
    for key in ("x", "y"):
        if not math.isclose(
            float(preview_table["center_m"][key]),
            float(tabletop["center_m"][key]),
            abs_tol=1e-9,
        ):
            raise ValueError("RViz job preview does not match the fixed tabletop center")
        if not math.isclose(
            float(preview_table["dimensions_m"][key]),
            float(tabletop["dimensions_m"][key]),
            abs_tol=1e-9,
        ):
            raise ValueError("RViz job preview does not match the fixed tabletop size")
    if not math.isclose(
        float(preview_table["surface_z_m"]),
        float(tabletop["surface_z_m"]),
        abs_tol=1e-9,
    ):
        raise ValueError("RViz job preview does not match the fixed tabletop height")
    return value


def point(x: float, y: float, z: float) -> Point:
    value = Point()
    value.x = float(x)
    value.y = float(y)
    value.z = float(z)
    return value


class WorkcellPreviewPublisher(Node):
    def __init__(self) -> None:
        super().__init__("fixed_workcell_preview")
        frame_id, tabletop = load_tabletop()
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            MarkerArray, "/workcell/table_preview", qos
        )
        job_preview = load_job_preview(frame_id, tabletop)
        self.markers = self._markers(frame_id, tabletop, job_preview)
        if job_preview is None:
            self.get_logger().info("Publishing the fixed tabletop without a G-code job")
        else:
            self.get_logger().info(
                "Publishing fixed tabletop and job preview: "
                f"{job_preview['source']['name']}"
            )
        self.create_timer(1.0, self.publish)
        self.publish()

    def _base_marker(
        self,
        frame_id: str,
        marker_id: int,
        marker_type: int,
        namespace: str = "fixed_workcell",
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 0
        return marker

    def _markers(
        self, frame_id: str, tabletop: dict, job_preview: dict | None
    ) -> MarkerArray:
        center = tabletop["center_m"]
        dimensions = tabletop["dimensions_m"]
        z = float(tabletop["surface_z_m"])
        yaw = math.radians(float(tabletop["yaw_deg"]))

        surface = self._base_marker(frame_id, 0, Marker.CUBE)
        surface.pose.position.x = float(center["x"])
        surface.pose.position.y = float(center["y"])
        surface.pose.position.z = z - 0.001
        surface.pose.orientation.z = math.sin(yaw / 2.0)
        surface.pose.orientation.w = math.cos(yaw / 2.0)
        surface.scale.x = float(dimensions["x"])
        surface.scale.y = float(dimensions["y"])
        # A thin visualization sheet, not a claim about physical thickness.
        surface.scale.z = 0.002
        surface.color.r = 0.20
        surface.color.g = 0.55
        surface.color.b = 0.92
        surface.color.a = 0.42

        center_marker = self._base_marker(frame_id, 1, Marker.SPHERE)
        center_marker.pose.position.x = float(center["x"])
        center_marker.pose.position.y = float(center["y"])
        center_marker.pose.position.z = z + 0.004
        center_marker.scale.x = 0.018
        center_marker.scale.y = 0.018
        center_marker.scale.z = 0.008
        center_marker.color.r = 1.0
        center_marker.color.g = 0.55
        center_marker.color.b = 0.05
        center_marker.color.a = 1.0

        label = self._base_marker(frame_id, 2, Marker.TEXT_VIEW_FACING)
        label.pose.position.x = float(center["x"])
        label.pose.position.y = float(center["y"])
        label.pose.position.z = z + 0.04
        label.scale.z = 0.025
        label.color.r = 0.95
        label.color.g = 0.97
        label.color.b = 1.0
        label.color.a = 1.0
        label.text = (
            "Fixed tabletop (confirmed)\n"
            f"center=({float(center['x']):.4f}, {float(center['y']):.4f}) m  "
            f"Z={z:.4f} m"
        )

        markers = [surface, center_marker, label]
        if job_preview is not None:
            markers.extend(self._job_markers(frame_id, job_preview))
        return MarkerArray(markers=markers)

    def _job_markers(self, frame_id: str, preview: dict) -> list[Marker]:
        segments = preview["segments"]
        print_segments = [segment for segment in segments if segment["kind"] == "print"]
        travel_segments = [segment for segment in segments if segment["kind"] != "print"]
        if not print_segments:
            raise ValueError("RViz job preview contains no printable path")

        print_path = self._base_marker(
            frame_id, 0, Marker.LINE_LIST, namespace="print_path"
        )
        print_path.scale.x = 0.0025
        print_path.color.r = 1.0
        print_path.color.g = 0.30
        print_path.color.b = 0.02
        print_path.color.a = 1.0
        for segment in print_segments:
            print_path.points.extend(
                [
                    point(segment["x0"], segment["y0"], segment["z0"]),
                    point(segment["x1"], segment["y1"], segment["z1"]),
                ]
            )

        travel_path = self._base_marker(
            frame_id, 0, Marker.LINE_LIST, namespace="travel_path"
        )
        travel_path.scale.x = 0.0008
        travel_path.color.r = 0.80
        travel_path.color.g = 0.84
        travel_path.color.b = 0.90
        travel_path.color.a = 0.28
        for segment in travel_segments:
            travel_path.points.extend(
                [
                    point(segment["x0"], segment["y0"], segment["z0"]),
                    point(segment["x1"], segment["y1"], segment["z1"]),
                ]
            )

        coordinates = [
            (float(segment[key_x]), float(segment[key_y]), float(segment[key_z]))
            for segment in print_segments
            for key_x, key_y, key_z in (("x0", "y0", "z0"), ("x1", "y1", "z1"))
        ]
        minimum = tuple(min(value[index] for value in coordinates) for index in range(3))
        maximum = tuple(max(value[index] for value in coordinates) for index in range(3))
        center = tuple((minimum[index] + maximum[index]) / 2.0 for index in range(3))

        model_outline = self._base_marker(
            frame_id, 0, Marker.LINE_LIST, namespace="planned_model"
        )
        model_outline.scale.x = 0.0015
        model_outline.color.r = 0.35
        model_outline.color.g = 1.0
        model_outline.color.b = 0.38
        model_outline.color.a = 0.95
        corners = [
            (x, y, z)
            for z in (minimum[2], maximum[2])
            for y in (minimum[1], maximum[1])
            for x in (minimum[0], maximum[0])
        ]
        for first, second in (
            (0, 1), (0, 2), (1, 3), (2, 3),
            (4, 5), (4, 6), (5, 7), (6, 7),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ):
            model_outline.points.extend([point(*corners[first]), point(*corners[second])])

        model_center = self._base_marker(
            frame_id, 1, Marker.SPHERE, namespace="planned_model"
        )
        model_center.pose.position.x = center[0]
        model_center.pose.position.y = center[1]
        model_center.pose.position.z = maximum[2] + 0.006
        model_center.scale.x = 0.010
        model_center.scale.y = 0.010
        model_center.scale.z = 0.010
        model_center.color.r = 0.35
        model_center.color.g = 1.0
        model_center.color.b = 0.38
        model_center.color.a = 1.0

        model_volume = self._base_marker(
            frame_id, 3, Marker.CUBE, namespace="planned_model"
        )
        model_volume.pose.position.x = center[0]
        model_volume.pose.position.y = center[1]
        model_volume.pose.position.z = center[2]
        model_volume.scale.x = maximum[0] - minimum[0]
        model_volume.scale.y = maximum[1] - minimum[1]
        model_volume.scale.z = max(0.001, maximum[2] - minimum[2])
        model_volume.color.r = 0.20
        model_volume.color.g = 1.0
        model_volume.color.b = 0.28
        model_volume.color.a = 0.12

        model_arrow = self._base_marker(
            frame_id, 4, Marker.ARROW, namespace="planned_model"
        )
        model_arrow.points = [
            point(center[0], center[1], maximum[2] + 0.14),
            point(center[0], center[1], maximum[2] + 0.012),
        ]
        model_arrow.scale.x = 0.008
        model_arrow.scale.y = 0.020
        model_arrow.scale.z = 0.025
        model_arrow.color.r = 0.35
        model_arrow.color.g = 1.0
        model_arrow.color.b = 0.38
        model_arrow.color.a = 0.95

        source_name = preview["source"]["name"]
        size_mm = tuple((maximum[index] - minimum[index]) * 1000.0 for index in range(3))
        model_label = self._base_marker(
            frame_id, 2, Marker.TEXT_VIEW_FACING, namespace="planned_model"
        )
        model_label.pose.position.x = center[0]
        model_label.pose.position.y = center[1]
        model_label.pose.position.z = maximum[2] + 0.165
        model_label.scale.z = 0.024
        model_label.color.r = 0.82
        model_label.color.g = 1.0
        model_label.color.b = 0.82
        model_label.color.a = 1.0
        model_label.text = (
            f"Planned print: {source_name}\n"
            f"size={size_mm[0]:.1f} x {size_mm[1]:.1f} x {size_mm[2]:.1f} mm\n"
            f"center=({center[0]:.4f}, {center[1]:.4f}) m"
        )

        return [
            travel_path,
            print_path,
            model_volume,
            model_outline,
            model_center,
            model_arrow,
            model_label,
        ]

    def publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for marker in self.markers.markers:
            marker.header.stamp = stamp
        self.publisher.publish(self.markers)


def main() -> None:
    rclpy.init()
    node = WorkcellPreviewPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
