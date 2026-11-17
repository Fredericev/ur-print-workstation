#!/usr/bin/env python3
"""Offline ROS-message encoding check; no node, action client or network.

This deliberately injects a curve-validation marker only to reach the goal
builder. It does NOT perform or replace MoveIt collision validation and cannot
authorize or dispatch either goal. Never use its output as motion approval.
"""
import argparse
import json
from pathlib import Path

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from rclpy.serialization import deserialize_message, serialize_message
from ur_klipper_interfaces.action import ManualStepperProgram

from gcode_print_executor import RosPrintExecutor, manifest_execution_id
from smooth_trajectory import validate_bundle
from execution_scope import validate_execution_scope


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True, type=Path)
    args = parser.parse_args()
    package = json.loads(args.manifest.read_text(encoding="utf-8"))
    assert package["execution_id"] == manifest_execution_id(package)
    profiles = validate_bundle(package)
    validate_execution_scope(package)
    profile = profiles["print"]

    builder = RosPrintExecutor.__new__(RosPrintExecutor)
    builder.manifest = package
    builder.FollowJointTrajectory = FollowJointTrajectory
    builder.JointTolerance = JointTolerance
    builder.ManualStepperProgram = ManualStepperProgram
    # Codec-only fixture injection, not a collision or hardware certificate.
    builder._validated_profile_identity = builder._profile_identity(profile)
    builder._validated_start_positions = profile["points"][0]["positions"]
    builder._minimum_validated_hold_s = .05
    ur = builder._trajectory_goal(profile, profile["duration_s"])
    klipper = builder._klipper_program_goal(package["blocks"], package["stepper_limits"]["maximum_speed"])
    ur_bytes = serialize_message(ur)
    klipper_bytes = serialize_message(klipper)
    ur_copy = deserialize_message(ur_bytes, FollowJointTrajectory.Goal)
    klipper_copy = deserialize_message(klipper_bytes, ManualStepperProgram.Goal)
    assert ur_copy == ur
    assert klipper_copy == klipper
    assert len(ur_copy.trajectory.points) == len(profile["points"])
    assert len(klipper_copy.segment_units) == len(package["blocks"])
    assert len(ur_copy.goal_tolerance) == 6
    print(json.dumps({
        "hardware_contacted": False, "executable": False,
        "collision_validation": "not performed; codec fixture only",
        "ur_points": len(ur_copy.trajectory.points), "ur_cdr_bytes": len(ur_bytes),
        "klipper_segments": len(klipper_copy.segment_units), "klipper_cdr_bytes": len(klipper_bytes),
        "native_ros_round_trip": "passed",
    }, indent=2))


if __name__ == "__main__":
    main()
