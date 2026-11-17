"""Inspect the isolated MoveIt scene after a plan-only tabletop-height probe."""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.node import Node


def main() -> None:
    rclpy.init()
    node = Node("isolated_height_plan_scene_inspector")
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    try:
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("MoveIt planning scene service is unavailable")
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("MoveIt planning scene request failed")
        scene = future.result().scene
        tables = [
            item for item in scene.world.collision_objects
            if item.id == "confirmed_table_surface_guard"
        ]
        heads = [
            item for item in scene.robot_state.attached_collision_objects
            if item.object.id == "measured_print_tool"
        ]
        if len(tables) != 1 or len(heads) != 1:
            raise RuntimeError(f"Expected one table and head; got {len(tables)} and {len(heads)}")
        table = tables[0]
        center_z = table.pose.position.z + table.primitive_poses[0].position.z
        top_z = center_z + table.primitives[0].dimensions[2] / 2.0
        head_size = tuple(heads[0].object.primitives[0].dimensions)
        if abs(top_z - (-0.020)) > 1.0e-6:
            raise RuntimeError(f"Collision table top is {top_z}, expected -0.020")
        config = json.loads(
            (Path(__file__).resolve().parents[4] / "config/printhead_geometry.json").read_text()
        )
        expected_size = config["collision_envelope_tool0_m"]["size"]
        if len(head_size) != 3 or any(abs(actual - expected) > 1.0e-6 for actual, expected in zip(
            head_size, (expected_size[axis] for axis in ("x", "y", "z"))
        )):
            raise RuntimeError(f"Printhead collision envelope is {head_size}")
        print(f"scene_table_top_z_m={top_z:.6f} head_box_m={head_size}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
