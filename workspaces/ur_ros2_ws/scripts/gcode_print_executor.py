#!/usr/bin/env python3
"""Execute one immutable, reviewed UR + Klipper print package through ROS 2.

This process deliberately owns only one operation.  It refuses stale files,
checks the live joint state against the reviewed trajectory, publishes status
atomically, and exposes the same coordinated-cancel service used by the
workstation's emergency stop path.
"""

from __future__ import annotations

import argparse
import bisect
import copy
import csv
import hashlib
import json
import itertools
import math
import os
import subprocess
import threading
import time
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Iterable

# This same ROS-free compiler is used by the host's manifest preparation.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from smooth_trajectory import compile_profile, validate_profile, validate_bundle, controls
from smooth_trajectory import POLICY as TRAJECTORY_POLICY, LIMITS as TRAJECTORY_LIMITS
from execution_scope import validate_execution_scope, SYNC_START_LEAD_S
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/ur_klipper_bridge"))
from ur_klipper_bridge.validation import (
    EXTRUSION_PROTOCOL, minimum_extrusion_duration, extrusion_peak_speed,
    validate_extruder, validate_native_execution,
)


UR_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
CONTROLLER_ACTION = "/passthrough_trajectory_controller/follow_joint_trajectory"
KLIPPER_ACTION = "/klipper/manual_stepper_program"
# One source of truth: the executor's dispatch estimates, preparation hold and
# operator diagnostics must describe the exact current curve limit installed by the
# shared compiler, never a retired commissioning cap.
MAX_JOINT_SPEED_RAD_S = TRAJECTORY_LIMITS["velocity"]
MAX_RESAMPLED_STEP_RAD = 0.01
PRINTHEAD_GEOMETRY_PATH = Path(__file__).resolve().parents[1] / "config" / "printhead_geometry.json"


def load_printhead_collision_geometry(path: Path = PRINTHEAD_GEOMETRY_PATH):
    """Read the same represented CAD envelope that CMake compiles into MoveIt."""
    geometry = json.loads(path.read_text(encoding="utf-8"))
    if geometry.get("schema_version") != 1:
        raise ValueError("Unsupported printhead geometry schema")
    source = geometry.get("source_sha256", "")
    if (not isinstance(source, str) or len(source) != 64 or
            any(character not in "0123456789abcdef" for character in source)):
        raise ValueError("Printhead source identity is missing or invalid")
    if not isinstance(geometry.get("geometry_revision"), str) or not geometry["geometry_revision"]:
        raise ValueError("Printhead geometry revision is missing")
    envelope = geometry["collision_envelope_tool0_m"]
    vectors = []
    for name in ("size", "center"):
        values = envelope[name]
        if set(values) != {"x", "y", "z"}:
            raise ValueError(f"Printhead envelope {name} must define X/Y/Z")
        vector = []
        for axis in ("x", "y", "z"):
            value = values[axis]
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError(f"Printhead envelope {name}/{axis} must be finite")
            if name == "size" and not 0.0 < value <= 1.0:
                raise ValueError("Printhead dimensions must be positive and within 1 m")
            if name == "center" and abs(value) > 1.0:
                raise ValueError("Printhead center must be within 1 m of tool0")
            vector.append(float(value))
        vectors.append(tuple(vector))
    canonical = json.dumps(
        geometry, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest(), vectors[0], vectors[1]


PRINTHEAD_GEOMETRY_SHA256, PRINTHEAD_BOX_SIZE_M, PRINTHEAD_BOX_CENTER_TOOL0_M = (
    load_printhead_collision_geometry()
)
PRINTHEAD_TCP_BOUNDS_TOLERANCE_M = 1e-9
START_TOLERANCE_RAD = 0.03
PRINT_START_TOLERANCE_RAD = 0.008
SYNC_ARM_GUARD_S = 0.5
# Retain the existing preparation floor. Smooth execution additionally computes
# a bounded quintic initial alignment and checks it before either goal is sent.
TRAJECTORY_PRELUDE_S = 1.5 * START_TOLERANCE_RAD / MAX_JOINT_SPEED_RAD_S
STATUS_INTERVAL_S = 0.20
CANCEL_CONFIRM_TIMEOUT_S = 5.0
# Emergency stop is the sole idempotent control request that may be retried.
# Its ROS reply can be lost after the bridge already stopped Klipper, so retain
# the safety lock while trying a small, rate-limited number of confirmations.
EMERGENCY_STOP_RETRY_INTERVAL_S = 1.0
EMERGENCY_STOP_MAX_ATTEMPTS = 2
SCALING_READY_TIMEOUT_S = 5.0
SCALING_FRESHNESS_S = 2.0
SCALING_STABLE_DWELL_S = 0.25
TRAJECTORY_SOCKET_FRESHNESS_S = 1.0
STOP_STATIONARY_DWELL_S = 0.25
STOP_JOINT_FRESHNESS_S = 0.10
STOP_JOINT_SPEED_RAD_S = 0.003
STOP_JOINT_DRIFT_RAD = 0.001
MOTION_COMMAND_EVIDENCE_RAD = 0.005
MOTION_OBSERVED_EVIDENCE_RAD = 0.001
MOTION_START_GRACE_S = 5.0
MOTION_PROGRESS_LEAD_S = 1.0
ENDPOINT_TOLERANCE_RAD = 0.005
ENDPOINT_SETTLE_TIMEOUT_S = 1.0
ENDPOINT_ACCELERATION_TOLERANCE_RAD_S2 = 0.5
# Keep concurrent service calls below the installed rmw/Fast DDS request and
# response history depth.  Submitting 16+ at once can leave the surplus calls
# unresolved; eight retains queue headroom without weakening sample coverage.
STATE_VALIDITY_BATCH_SIZE = 8
COLLISION_SAMPLE_SPACING_RAD = 0.002
# The C++ validator intentionally rejects larger CSV inputs.  Long jobs are
# certified in consecutive batches so the limit remains a memory/corruption
# guard instead of silently limiting the length of a valid trajectory.
LOCAL_VALIDATION_BATCH_SAMPLES = 200_000
MAX_LOCAL_VALIDATION_BATCH_FILES = 128
MIN_SMOOTH_VALIDATION_BUDGET_S = 60.0
MAX_SMOOTH_VALIDATION_BUDGET_S = 3600.0
SMOOTH_VALIDATION_SECONDS_PER_SAMPLE = 0.002
TOOL0_FIDELITY_TRANSLATION_LIMIT_M = 0.0005
TOOL0_FIDELITY_ROTATION_LIMIT_RAD = 0.005
BRIDGE_PROTOCOL_VERSION = "native-extruder-program-v6"
CONTROLLER_STOP_PROTOCOL_VERSION = "cancel-deactivate-v2"
# Keep in sync with Software.gcode_path_review and the plan-only C++ producer.
PLANNING_POLICY_VERSION = "continuous-cartesian-approach-v4"


class ExecutionFailure(RuntimeError):
    def __init__(self, message: str, *, diagnostics: dict[str, Any] | None = None):
        super().__init__(message)
        self.diagnostics = diagnostics


class ValidatedExecutionPackage(dict):
    """One process-local certificate, never serialized into the package hash."""

    def __init__(self, payload, profiles):
        super().__init__(payload)
        self.validated_profiles = profiles
        self.profile_digests = {
            id(profile): profile_identity(profile)
            for profile in profiles.values()
            if isinstance(profile, dict)
        }


def profile_identity(profile):
    return hashlib.sha256(
        json.dumps(profile, sort_keys=True, allow_nan=False).encode()
    ).hexdigest()


def synchronized_start_plan(now_nanoseconds: int) -> dict[str, int]:
    """Return a best-effort software target, not a shared hardware clock.

    Passthrough ignores header.stamp. Its actual dispatch uses a relative
    first-point hold; network and controller latency remain unmeasured.
    """

    motion_start = int(now_nanoseconds) + int(SYNC_START_LEAD_S * 1_000_000_000)
    prelude = int(TRAJECTORY_PRELUDE_S * 1_000_000_000)
    return {
        "ur_motion_start_ns": motion_start,
        "ur_header_stamp_ns": motion_start - prelude,
        "klipper_start_ns": motion_start,
    }


def paired_profile_samples(
    profile: dict[str, Any],
    spacing_rad: float = COLLISION_SAMPLE_SPACING_RAD,
    *,
    reference_is_candidate: bool = False,
    reviewed_path: list[dict[str, Any]] | None = None,
    _already_validated: bool = False,
) -> Iterable[tuple[list[float], list[float]]]:
    """Sample every quintic interval with its same-u reviewed q chord.

    The collision candidate is the exact Bernstein curve sent to the
    controller.  Its reference is the straight joint-space segment between
    the two reviewed knots at the same normalized interval coordinate.  The
    derivative control-polygon bound makes each per-joint candidate step no
    larger than ``spacing_rad``; endpoints alone are never treated as proof.
    """
    if not _already_validated:
        validate_profile(profile)
    if not math.isfinite(float(spacing_rad)) or spacing_rad <= 0.0:
        raise ValueError("Smooth-curve sample spacing must be positive and finite")
    progresses = None
    if reviewed_path is not None:
        progresses = [finite(p["path_progress"], "reviewed path progress") for p in reviewed_path]
        if len(progresses) < 2 or any(b < a for a, b in zip(progresses, progresses[1:])):
            raise ExecutionFailure("Reviewed fidelity reference is invalid")

    def reference_at(progress):
        high = min(len(reviewed_path)-1, bisect.bisect_left(progresses, progress))
        if high == 0 or abs(progresses[high]-progress) <= 1.0e-12:
            return list(reviewed_path[high]["positions"])
        low = high-1
        span = progresses[high]-progresses[low]
        fraction = 0.0 if span <= 1.0e-12 else (progress-progresses[low])/span
        return [a+fraction*(b-a) for a, b in zip(
            reviewed_path[low]["positions"], reviewed_path[high]["positions"]
        )]

    first = list(profile["points"][0]["positions"])
    yield first, reference_at(profile["points"][0]["path_progress"]) if progresses is not None else list(first)
    for start, end in zip(profile["points"], profile["points"][1:]):
        curves = [controls(start, end, joint) for joint in range(len(UR_JOINTS))]
        derivative_bound = max(
            5.0 * abs(right - left)
            for curve in curves
            for left, right in zip(curve, curve[1:])
        )
        divisions = max(2, int(math.ceil(derivative_bound / spacing_rad)))
        for index in range(1, divisions + 1):
            u = index / divisions
            candidate = []
            for curve in curves:
                work = list(curve)
                while len(work) > 1:
                    work = [
                        (1.0 - u) * left + u * right
                        for left, right in zip(work, work[1:])
                    ]
                candidate.append(work[0])
            if reference_is_candidate:
                reference = list(candidate)
            elif progresses is not None:
                progress = (1.0-u)*start["path_progress"]+u*end["path_progress"]
                reference = reference_at(progress)
            else:
                reference = [
                    (1.0 - u) * left + u * right
                    for left, right in zip(start["positions"], end["positions"])
                ]
            yield candidate, reference


def profile_collision_sample_count(
    profile: dict[str, Any],
    spacing_rad: float = COLLISION_SAMPLE_SPACING_RAD,
) -> int:
    """Return the exact number of states ``paired_profile_samples`` will emit."""

    if not math.isfinite(float(spacing_rad)) or spacing_rad <= 0.0:
        raise ValueError("Smooth-curve sample spacing must be positive and finite")
    count = 1
    for start, end in zip(profile["points"], profile["points"][1:]):
        curves = [controls(start, end, joint) for joint in range(len(UR_JOINTS))]
        derivative_bound = max(
            5.0 * abs(right - left)
            for curve in curves
            for left, right in zip(curve, curve[1:])
        )
        count += max(2, int(math.ceil(derivative_bound / spacing_rad)))
    return count


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def manifest_execution_id(payload: dict[str, Any]) -> str:
    identity_payload = {
        key: value for key, value in payload.items() if key != "execution_id"
    }
    try:
        canonical = json.dumps(
            identity_payload,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise ExecutionFailure("Execution package cannot be canonicalized") from exc
    return hashlib.sha256(canonical).hexdigest()[:16]


def finite(value: object, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ExecutionFailure(f"{label} is not numeric") from exc
    if not math.isfinite(result):
        raise ExecutionFailure(f"{label} is not finite")
    return result


def require_current_printhead_geometry(payload: dict[str, Any]) -> str:
    try:
        current, _, _ = load_printhead_collision_geometry(PRINTHEAD_GEOMETRY_PATH)
    except (OSError, KeyError, TypeError, ValueError) as exc:
        raise ExecutionFailure(f"Installed printhead geometry cannot be verified: {exc}") from exc
    if current != PRINTHEAD_GEOMETRY_SHA256:
        raise ExecutionFailure("Printhead geometry changed while the executor was running; restart and review again")
    if payload.get("printhead_geometry_sha256") != current:
        raise ExecutionFailure("Execution package uses obsolete printhead geometry; validate and prepare again")
    return current


def load_manifest(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ExecutionFailure(f"Could not read execution package: {exc}") from exc
    if not isinstance(payload, dict) or payload.get("version") != 3:
        raise ExecutionFailure("Execution package version is invalid")
    if payload.get("planning_policy_version") != PLANNING_POLICY_VERSION:
        raise ExecutionFailure("Execution package uses an obsolete planning policy; validate and prepare again")
    if payload.get("extrusion_protocol") != EXTRUSION_PROTOCOL:
        raise ExecutionFailure("Execution package predates native extrusion; validate and prepare again")
    try:
        validate_extruder(payload.get("extruder"))
    except ValueError as exc:
        raise ExecutionFailure("Execution package has no native extruder") from exc
    if payload.get("execution_id") != manifest_execution_id(payload):
        raise ExecutionFailure("Execution package changed after preparation")
    require_current_printhead_geometry(payload)
    try:
        profiles = validate_bundle(payload)
    except (ValueError, KeyError, TypeError, IndexError) as exc:
        raise ExecutionFailure(f"Smooth trajectory package is invalid: {exc}") from exc
    try:
        validate_execution_scope(payload)
    except (ValueError, KeyError, TypeError, IndexError) as exc:
        raise ExecutionFailure(f"Execution scope is invalid: {exc}") from exc
    try:
        validate_native_execution(payload)
    except (ValueError, KeyError, TypeError) as exc:
        raise ExecutionFailure(f"Native extrusion package is invalid: {exc}") from exc
    if payload.get("simulation") is True:
        raise ExecutionFailure("A simulated package cannot command real hardware")
    if payload.get("coordinate_frame") != "base_link":
        raise ExecutionFailure("Execution package is not expressed in base_link")
    if payload.get("end_effector_link") != "tool0":
        raise ExecutionFailure("Execution package does not control tool0")
    if payload.get("operation_mode", "production_print") not in {
        "production_print",
        "tool0_motion_test",
    }:
        raise ExecutionFailure("Execution package operation mode is invalid")
    if payload.get("heating_enabled") is not False:
        raise ExecutionFailure(
            "This motion executor refuses packages that enable heating"
        )
    for key in ("planning_request", "approach_trajectory", "print_trajectory"):
        artifact = payload.get(key)
        if not isinstance(artifact, dict):
            raise ExecutionFailure(f"{key} metadata is missing")
        artifact_path = Path(
            str(artifact.get("container_path", artifact.get("path", "")))
        ).resolve()
        workspace = Path("/home/ubuntu/ur_ros2_ws").resolve()
        try:
            artifact_path.relative_to(workspace)
        except ValueError as exc:
            raise ExecutionFailure(f"{key} escaped the managed workspace") from exc
        if not artifact_path.is_file():
            raise ExecutionFailure(f"{key} is missing")
        if sha256_file(artifact_path) != artifact.get("sha256"):
            raise ExecutionFailure(f"{key} changed after MoveIt review")
    blocks = payload.get("blocks")
    if not isinstance(blocks, list) or not blocks:
        raise ExecutionFailure("Execution package has no synchronized blocks")
    return ValidatedExecutionPackage(payload, profiles)


def load_trajectory(path: Path) -> list[dict[str, Any]]:
    points: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        if tuple(reader.fieldnames or ()) != ("time_s", "path_progress", *UR_JOINTS):
            raise ExecutionFailure(f"Invalid trajectory header in {path.name}")
        previous_time = -1.0
        previous_progress = -1.0
        for row in reader:
            timestamp = finite(row["time_s"], "trajectory timestamp")
            path_progress = finite(row["path_progress"], "trajectory Cartesian progress")
            positions = [finite(row[name], name) for name in UR_JOINTS]
            if timestamp < 0.0 or timestamp <= previous_time:
                raise ExecutionFailure("Trajectory timestamps are not strictly increasing")
            if (
                path_progress < 0.0
                or path_progress > 1.0
                or path_progress < previous_progress
            ):
                raise ExecutionFailure("Trajectory Cartesian progress is invalid")
            points.append(
                {
                    "time_s": timestamp,
                    "path_progress": path_progress,
                    "positions": positions,
                }
            )
            previous_time = timestamp
            previous_progress = path_progress
    if len(points) < 2:
        raise ExecutionFailure("Trajectory needs at least two points")
    if points[0]["path_progress"] > 1.0e-9 or points[-1]["path_progress"] < 1.0 - 1.0e-9:
        raise ExecutionFailure("Trajectory does not cover Cartesian progress 0 through 1")
    return points


def interpolate_progress(points: list[dict[str, Any]], progress: float) -> list[float]:
    if progress <= 0.0:
        return list(points[0]["positions"])
    if progress >= 1.0:
        return list(points[-1]["positions"])
    progresses = [float(point["path_progress"]) for point in points]
    high = bisect.bisect_left(progresses, progress)
    if high < len(points) and abs(progresses[high] - progress) <= 1.0e-12:
        return list(points[high]["positions"])
    high = min(len(points) - 1, high)
    low = max(0, high - 1)
    before = points[low]
    after = points[high]
    progress_span = progresses[high] - progresses[low]
    fraction = 0.0 if progress_span <= 1.0e-12 else (
        progress - progresses[low]
    ) / progress_span
    return [
        before["positions"][index]
        + (after["positions"][index] - before["positions"][index]) * fraction
        for index in range(6)
    ]


def slice_trajectory(
    points: list[dict[str, Any]], start_progress: float, end_progress: float
) -> list[list[float]]:
    if not 0.0 <= start_progress <= end_progress <= 1.0:
        raise ExecutionFailure("Synchronized block progress is invalid")
    positions = [interpolate_progress(points, start_progress)]
    positions.extend(
        list(point["positions"])
        for point in points
        if start_progress < float(point["path_progress"]) < end_progress
    )
    positions.append(interpolate_progress(points, end_progress))
    return resample_positions(positions)


def resample_positions(positions: list[list[float]]) -> list[list[float]]:
    output = [positions[0]]
    for target in positions[1:]:
        start = output[-1]
        maximum = max(abs(target[index] - start[index]) for index in range(6))
        divisions = max(1, int(math.ceil(maximum / MAX_RESAMPLED_STEP_RAD)))
        for subdivision in range(1, divisions + 1):
            fraction = subdivision / divisions
            output.append(
                [
                    start[index] + (target[index] - start[index]) * fraction
                    for index in range(6)
                ]
            )
    return output


def minimum_ur_duration(positions: list[list[float]]) -> float:
    maximum_step = max((
        max(abs(after[index] - before[index]) for index in range(6))
        for before, after in zip(positions, positions[1:])
    ), default=0.0)
    # Legacy position-only diagnostic estimate, NOT used for bounded-quintic
    # execution. New manifests carry final profiles and their complete limits.
    return max(1.5 * maximum_step * (len(positions) - 1) / MAX_JOINT_SPEED_RAD_S, 0.05)


def klipper_minimum_duration(distance: float, speed: float, acceleration: float) -> float:
    return minimum_extrusion_duration(distance, speed, acceleration)


def klipper_speed_for_duration(distance: float, duration: float, limit: float, acceleration: float) -> float:
    return extrusion_peak_speed(distance, duration, limit, acceleration)


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


class RosPrintExecutor:
    def __init__(self, manifest: dict[str, Any], status_path: Path):
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        package_xml = Path(get_package_share_directory("ur_client_library")) / "package.xml"
        if ET.parse(package_xml).getroot().findtext("version") != "2.11.0":
            raise ExecutionFailure("Smooth trajectory wire encoding was verified for URClient 2.11.0; revalidate an upgraded driver before motion")
        import rclpy
        from control_msgs.action import FollowJointTrajectory
        from control_msgs.msg import JointTolerance
        from controller_manager_msgs.srv import SwitchController
        from rcl_interfaces.srv import GetParameters
        from moveit_msgs.srv import GetPlanningScene, GetStateValidity
        from rclpy.action import ActionClient
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Float64
        from std_srvs.srv import Trigger
        from ur_klipper_interfaces.action import ManualStepperProgram

        self.rclpy = rclpy
        self.FollowJointTrajectory = FollowJointTrajectory
        self.JointTolerance = JointTolerance
        self.ManualStepperProgram = ManualStepperProgram
        self.Trigger = Trigger
        self.SwitchController = SwitchController
        self.GetParameters = GetParameters
        self.GetPlanningScene = GetPlanningScene
        self.GetStateValidity = GetStateValidity
        self.manifest = manifest
        self._prepared_profiles = getattr(manifest, "validated_profiles", None)
        self._prepared_profile_digests = getattr(manifest, "profile_digests", {})
        self.status_path = status_path
        self.local_collision_validator_path = (
            Path(get_package_prefix("ur3e_print_planner"))
            / "lib"
            / "ur3e_print_planner"
            / "validate_joint_samples"
        )
        self._collision_sample_validator = self._validate_collision_samples_locally
        self.cancel_requested = threading.Event()
        self.latest_joints: dict[str, float] = {}
        self.latest_joint_stamp = 0.0
        self.program_running = False
        self.program_state_stamp = 0.0
        self._trajectory_socket_state = (False, 0.0)
        self.latest_scaling = None
        self.latest_scaling_stamp = 0.0
        self.scaling_stable_since = 0.0
        self._scaling_invalid = False
        self._coordination_active = False
        self.active_goals: list[Any] = []
        self._initialize_goal_tracking()
        self.controller_switched = False
        self.status: dict[str, Any] = {
            "version": 1,
            "execution_id": manifest["execution_id"],
            "state": "starting",
            "mode": "",
            "progress": 0.0,
            "current_block": 0,
            "block_count": len(manifest["blocks"]),
            "message": "Starting ROS Print Execution",
            "updated_at": time.time(),
        }

        rclpy.init(args=[])
        self.node = Node(f"ur_print_executor_{os.getpid()}")
        group = ReentrantCallbackGroup()
        self.ur_client = ActionClient(
            self.node, FollowJointTrajectory, CONTROLLER_ACTION, callback_group=group
        )
        self.klipper_client = ActionClient(
            self.node, ManualStepperProgram, KLIPPER_ACTION, callback_group=group
        )
        self.emergency_client = self.node.create_client(
            Trigger, "/klipper/emergency_stop", callback_group=group
        )
        self.bridge_protocol_client = self.node.create_client(
            GetParameters, "/klipper_bridge/get_parameters", callback_group=group
        )
        self.controller_protocol_client = self.node.create_client(
            GetParameters, "/passthrough_trajectory_controller/get_parameters", callback_group=group
        )
        self.switch_controller_client = self.node.create_client(
            SwitchController,
            "/controller_manager/switch_controller",
            callback_group=group,
        )
        self.planning_scene_client = self.node.create_client(GetPlanningScene, "/get_planning_scene", callback_group=group)
        self.state_validity_client = self.node.create_client(GetStateValidity, "/check_state_validity", callback_group=group)
        self.model_parameters_client = self.node.create_client(GetParameters, "/move_group/get_parameters", callback_group=group)
        self.cancel_service = self.node.create_service(
            Trigger,
            "/synchronized_motion/cancel",
            self._cancel_callback,
            callback_group=group,
        )
        program_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # Match the driver's actual QoS contracts.  A RELIABLE/VOLATILE
        # subscription can miss both the sensor-data joint stream and the
        # already-published transient robot-program state even though the
        # connection screen correctly reports them as live.
        self.joint_subscription = self.node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_callback,
            qos_profile_sensor_data,
            callback_group=group,
        )
        self.program_subscription = self.node.create_subscription(
            Bool, "/io_and_status_controller/robot_program_running",
            self._program_callback, program_qos, callback_group=group
        )
        self.trajectory_socket_subscription = self.node.create_subscription(
            Bool, "/io_and_status_controller/trajectory_socket_connected",
            self._trajectory_socket_callback, program_qos, callback_group=group
        )
        self.scaling_subscription = self.node.create_subscription(
            Float64, "/speed_scaling_state_broadcaster/speed_scaling",
            self._scaling_callback, qos_profile_sensor_data, callback_group=group
        )
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        self._write_status()

    def _joint_callback(self, message: Any) -> None:
        now = time.monotonic()
        try:
            values = dict(zip(message.name, message.position))
            positions = [float(values[name]) for name in UR_JOINTS]
            positions_valid = all(math.isfinite(value) for value in positions)
        except (AttributeError, KeyError, TypeError, ValueError):
            positions, positions_valid = [], False
        try:
            values = dict(zip(message.name, message.velocity))
            velocities = [float(values[name]) for name in UR_JOINTS]
            velocities_valid = all(math.isfinite(value) for value in velocities)
        except (AttributeError, KeyError, TypeError, ValueError):
            velocities, velocities_valid = [], False
        try:
            stamp = message.header.stamp
            source_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            ros_now_ns = self.node.get_clock().now().nanoseconds
            source_fresh = 0 <= ros_now_ns - source_ns <= int(STOP_JOINT_FRESHNESS_S * 1e9)
        except (AttributeError, TypeError, ValueError):
            source_ns, source_fresh = 0, False
        with self._goal_lock:
            source_fresh = source_fresh and source_ns > self._joint_source_stamp_ns
            if positions_valid:
                self.latest_joints = dict(zip(UR_JOINTS, positions))
                self.latest_joint_stamp = now
            self.latest_joint_velocities = dict(zip(UR_JOINTS, velocities)) if velocities_valid else {}
            if source_fresh:
                self._joint_source_stamp_ns = source_ns
            # A queued pre-stop message, duplicate stamp, missing velocity or
            # stale stream cannot be used as evidence of a physically still UR.
            if (
                not positions_valid or not velocities_valid or not source_fresh
                or self._stop_requested_at is None or now <= self._stop_requested_at
                or self._stop_requested_ros_ns is None or source_ns <= self._stop_requested_ros_ns
                or max(abs(value) for value in velocities) > STOP_JOINT_SPEED_RAD_S
            ):
                self._stationary_observation = None
                return
            observation = self._stationary_observation
            if observation is not None:
                minimum = [min(old, new) for old, new in zip(observation["minimum"], positions)]
                maximum = [max(old, new) for old, new in zip(observation["maximum"], positions)]
                if (
                    now - observation["last"] > STOP_JOINT_FRESHNESS_S
                    or (source_ns - observation["source_last_ns"]) / 1e9 > STOP_JOINT_FRESHNESS_S
                    or any(high - low > STOP_JOINT_DRIFT_RAD for low, high in zip(minimum, maximum))
                ):
                    observation = None
            if observation is None:
                observation = {
                    "since": now, "source_since_ns": source_ns, "count": 0,
                    "minimum": list(positions), "maximum": list(positions),
                }
            else:
                observation.update(minimum=minimum, maximum=maximum)
            observation.update(last=now, source_last_ns=source_ns, count=observation["count"] + 1)
            self._stationary_observation = observation

    def _ur_stationary_after_stop(self, now: float) -> bool:
        """Called under _goal_lock; software observation, not a safety-rated stop."""
        observation = self._stationary_observation
        return bool(
            observation is not None and observation["count"] >= 2
            and now - observation["last"] <= STOP_JOINT_FRESHNESS_S
            and observation["last"] - observation["since"] >= STOP_STATIONARY_DWELL_S
            and (observation["source_last_ns"] - observation["source_since_ns"]) / 1e9 >= STOP_STATIONARY_DWELL_S
        )

    def _program_callback(self, message: Any) -> None:
        self.program_running = bool(message.data)
        self.program_state_stamp = time.monotonic()

    def _trajectory_socket_callback(self, message: Any) -> None:
        self._trajectory_socket_state = (bool(message.data), time.monotonic())

    def _trajectory_socket_snapshot(self) -> dict[str, bool]:
        connected, stamp = getattr(self, "_trajectory_socket_state", (False, 0.0))
        age = time.monotonic() - stamp
        return {
            "trajectory_socket_received": stamp > 0.0,
            "trajectory_socket_fresh": stamp > 0.0 and 0.0 <= age < TRAJECTORY_SOCKET_FRESHNESS_S,
            "trajectory_socket_connected": connected,
        }

    def require_trajectory_socket(self) -> None:
        state = self._trajectory_socket_snapshot()
        if not state["trajectory_socket_received"]:
            reason = "UR trajectory socket status has not been received"
        elif not state["trajectory_socket_fresh"]:
            reason = "UR trajectory socket status is stale"
        elif not state["trajectory_socket_connected"]:
            reason = "UR trajectory socket is disconnected"
        else:
            return
        raise ExecutionFailure(reason, diagnostics={"trajectory_socket": state})

    def _scaling_callback(self, message: Any) -> None:
        try:
            value = float(message.data)  # The installed broadcaster publishes percent.
        except (AttributeError, TypeError, ValueError, OverflowError):
            value = float("nan")
        with self._goal_lock:
            # Timestamp when this callback actually owns the state lock. A
            # delayed older callback must not overwrite a newer sample with
            # an older timestamp.
            now = time.monotonic()
            if not math.isfinite(value) or not 0.0 <= value <= 100.0:
                self.latest_scaling = None
                self.latest_scaling_stamp = now
                self.scaling_stable_since = 0.0
                self._scaling_invalid = True
                return
            if (
                self.latest_scaling is None or abs(value - self.latest_scaling) > 0.01
                or now - self.latest_scaling_stamp > SCALING_FRESHNESS_S
            ):
                self.scaling_stable_since = now
            self.latest_scaling = value
            self.latest_scaling_stamp = now
            self._scaling_invalid = False

    def _scaling_snapshot(self) -> dict[str, Any]:
        with self._goal_lock:
            # Copy one coherent callback state. Read the clock only *after*
            # this copy: a callback that lands while the clock is read may
            # refresh the shared stamp, but it cannot make this snapshot's
            # copied stamp appear to come from the future.
            value = getattr(self, "latest_scaling", None)
            stamp = getattr(self, "latest_scaling_stamp", 0.0)
            stable = getattr(self, "scaling_stable_since", 0.0)
            invalid = getattr(self, "_scaling_invalid", False)
        now = time.monotonic()
        age = now - stamp if stamp > 0.0 else None
        # Evidence spans received samples, not time elapsed since one frame.
        stable_age = max(0.0, stamp - stable) if stable > 0.0 else None
        if invalid:
            reason = "invalid"
        elif value is None or age is None:
            reason = "missing"
        elif not math.isfinite(value) or not 0.0 <= value <= 100.0:
            reason = "invalid"
        elif age < 0.0 or age > SCALING_FRESHNESS_S:
            reason = "stale"
        elif abs(value - 100.0) > 0.01:
            reason = "scaled"
        elif stable_age is None or stable_age < SCALING_STABLE_DWELL_S:
            reason = "warming"
        else:
            reason = "ready"
        return {
            "reason": reason,
            "value_percent": value if value is not None and math.isfinite(value) else None,
            "age_s": age, "stable_age_s": stable_age,
            "required_percent": 100.0, "maximum_sample_age_s": SCALING_FRESHNESS_S,
            "required_stable_age_s": SCALING_STABLE_DWELL_S,
        }

    @staticmethod
    def _scaling_message(snapshot: dict[str, Any]) -> str:
        def shown(value: Any) -> str:
            return "unavailable" if value is None else f"{value:.3f}"
        return (
            "Software coordination requires fresh, stable, unscaled UR timing: "
            f"reason={snapshot['reason']}, scaling_percent={shown(snapshot['value_percent'])}, "
            f"sample_age_s={shown(snapshot['age_s'])}, stable_age_s={shown(snapshot['stable_age_s'])}. "
            "Do not change safety settings to bypass this check. "
            f"Joint speed remains capped at {MAX_JOINT_SPEED_RAD_S:.3g} rad/s."
        )

    def _require_coordination_scaling(self) -> None:
        snapshot = self._scaling_snapshot()
        if snapshot["reason"] != "ready":
            diagnostics = self._diagnostic_snapshot()
            diagnostics["scaling"] = snapshot
            raise ExecutionFailure(self._scaling_message(snapshot), diagnostics=diagnostics)

    def _wait_for_coordination_scaling(self, timeout_s: float = SCALING_READY_TIMEOUT_S) -> None:
        """Only before first dispatch; runtime guards never wait through a fault."""
        deadline = time.monotonic() + timeout_s
        last_write = -float("inf")
        while True:
            snapshot = self._scaling_snapshot()
            if self.cancel_requested.is_set():
                raise ExecutionFailure("Execution canceled while waiting for UR scaling readiness", diagnostics={"scaling": snapshot})
            if snapshot["reason"] == "ready":
                return
            now = time.monotonic()
            if now >= deadline:
                raise ExecutionFailure(
                    f"Timed out waiting {timeout_s:.1f} s for UR scaling readiness. " + self._scaling_message(snapshot),
                    diagnostics={"scaling": snapshot},
                )
            if now - last_write >= STATUS_INTERVAL_S:
                self._set_status("preflight", "Waiting for UR scaling readiness. " + self._scaling_message(snapshot),
                                 diagnostics={"scaling": snapshot})
                last_write = now
            self.cancel_requested.wait(min(0.02, max(0.0, deadline - now)))

    @staticmethod
    def _wait_for_future(future: Any, deadline: float) -> bool:
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return bool(future.done())

    def _initialize_goal_tracking(self) -> None:
        self._goal_lock = threading.RLock()
        self._abort_lock = threading.Lock()
        self._status_write_lock = threading.Lock()
        self._status_sequence = 0
        self._last_written_status_sequence = 0
        self._goal_requests: list[dict[str, Any]] = []
        self._stopping = False
        self._stop_confirmed = False
        self._emergency_stop_future = None
        self._emergency_stop_attempts = 0
        self._next_emergency_stop_attempt_at = 0.0
        self.latest_joint_velocities: dict[str, float] = {}
        self._joint_source_stamp_ns = 0
        self._stop_requested_at: float | None = None
        self._stop_requested_ros_ns: int | None = None
        self._stationary_observation: dict[str, Any] | None = None

    def _goal_snapshots(self) -> list[dict[str, Any]]:
        snapshots = []
        with self._goal_lock:
            for record in self._goal_requests:
                handle, future = record.get("handle"), record.get("result_future")
                result_status, result_error, goal_id = None, None, None
                if future is not None and future.done():
                    try:
                        result_status = getattr(future.result(), "status", None)
                    except Exception as exc:
                        result_error = str(exc)
                try:
                    goal_id = bytes(handle.goal_id.uuid).hex() if handle is not None else None
                except (AttributeError, TypeError, ValueError):
                    pass
                snapshots.append({
                    "label": record["label"], "goal_id": goal_id,
                    "submitted_monotonic": record.get("submitted_at"),
                    "acknowledged": bool(record["acknowledged"]),
                    "ack_monotonic": record.get("ack_time"),
                    "accepted": bool(handle.accepted) if handle is not None else None,
                    "result_received": bool(future is not None and future.done()),
                    "result_status": result_status, "result_error": result_error,
                    "terminal": self._request_is_terminal(record),
                    "cancel_requested": bool(record.get("cancel_requested")),
                    "cancel_error": record.get("cancel_error"), "submission_error": record.get("error"),
                })
        return snapshots

    def _diagnostic_snapshot(self) -> dict[str, Any]:
        with self._goal_lock:
            now = time.monotonic()
            joint_stamp = getattr(self, "latest_joint_stamp", 0.0)
            return {
                "scaling": self._scaling_snapshot(), "goals": self._goal_snapshots(),
                "joint_feedback_age_s": now - joint_stamp if joint_stamp > 0.0 else None,
                "external_control_running": bool(getattr(self, "program_running", False)),
                "captured_monotonic": now,
            }

    def _record_root_error(self, error: Exception | str) -> None:
        """Freeze the first failure before cancellation can change live evidence."""
        with self._goal_lock:
            if self.status.get("root_error"):
                return
            diagnostics = self._diagnostic_snapshot()
            diagnostics.update(copy.deepcopy(getattr(error, "diagnostics", None) or {}))
            self.status["root_error"] = {
                "message": str(error) or type(error).__name__, "type": type(error).__name__,
                "recorded_at": time.time(), "diagnostics": diagnostics,
            }

    def _store_stop_snapshot(
        self,
        reasons: list[str],
        emergency_ok: bool,
        stationary: bool,
        emergency_required: bool,
        stationary_required: bool,
    ) -> None:
        """Called under _goal_lock; reasons describe now, never accumulated history."""
        now = time.monotonic()
        observation = self._stationary_observation
        self.status["stop"] = {
            "confirmed": not reasons, "reasons": list(reasons),
            "requested_monotonic": self._stop_requested_at,
            "observed_monotonic": now, "goals": self._goal_snapshots(),
            "klipper_emergency_required": emergency_required,
            "klipper_emergency_confirmed": emergency_required and emergency_ok,
            "klipper_emergency_attempts": self._emergency_stop_attempts,
            "ur_stationary": {
                "required": stationary_required,
                "confirmed": not stationary_required or stationary,
                "sample_age_s": now - observation["last"] if observation else None,
                "observed_dwell_s": observation["last"] - observation["since"] if observation else 0.0,
                "sample_count": observation["count"] if observation else 0,
                "required_dwell_s": STOP_STATIONARY_DWELL_S,
                "maximum_velocity_rad_s": STOP_JOINT_SPEED_RAD_S,
                "maximum_position_drift_rad": STOP_JOINT_DRIFT_RAD,
            },
        }

    def _ur_stationary_required(self) -> bool:
        """Return whether a submitted UR request might have commanded motion.

        No post-stop arm observation is needed when the only failure was an
        explicitly pre-dispatch Klipper rejection and no UR request ever
        reached its action server.  Any unknown UR acknowledgement, transport
        error, or accepted UR goal remains fail-closed and requires fresh
        stationary joint feedback.
        """
        with self._goal_lock:
            for record in self._goal_requests:
                if record.get("label") != "UR":
                    continue
                if not record.get("acknowledged") or record.get("error"):
                    return True
                handle = record.get("handle")
                if handle is None:
                    return True
                if handle.accepted:
                    return True
        return False

    def _klipper_emergency_required(self) -> bool:
        """Return whether a submitted Klipper request may have started motion.

        ManualStepperProgram results carry a fail-closed ``may_have_moved`` bit.
        A terminal pre-dispatch rejection with an explicit false value is the
        one case where firmware shutdown would create a new fault rather than
        stop motion. Unknown, pending, or post-dispatch outcomes still require
        the emergency request.
        """
        with self._goal_lock:
            for record in self._goal_requests:
                if record.get("label") != "Klipper":
                    continue
                if not record.get("acknowledged") or record.get("error"):
                    return True
                handle = record.get("handle")
                if handle is None:
                    return True
                if not handle.accepted:
                    continue
                future = record.get("result_future")
                if future is None or not future.done():
                    return True
                try:
                    wrapper = future.result()
                    result = None if wrapper is None else wrapper.result
                    if (
                        getattr(wrapper, "status", None) in {4, 5, 6}
                        and result is not None
                        and getattr(result, "may_have_moved", None) is False
                    ):
                        continue
                except Exception:
                    pass
                return True
        return False

    def _publish_failure_state(self, state: str) -> None:
        root_message = str(self.status.get("root_error", {}).get("message", "Execution canceled"))
        reasons = self.status.get("stop", {}).get("reasons", []) if state == "stop-unconfirmed" else []
        message = "; ".join([root_message, *reasons])
        try:
            self._set_status(state, message)
        except Exception as exc:
            # Keep the original failure and active-goal ledger even if status
            # persistence itself fails. Never turn a write error into a retry.
            print(f"Could not persist execution status: {exc}. Root error: {root_message}", flush=True)

    @staticmethod
    def _request_is_terminal(record: dict[str, Any]) -> bool:
        if not record["acknowledged"] or record.get("error"):
            return False
        handle = record.get("handle")
        if handle is not None and not handle.accepted:
            return True
        future = record.get("result_future")
        if future is None or not future.done():
            return False
        try:
            # action_msgs/GoalStatus: SUCCEEDED, CANCELED and ABORTED are
            # terminal. A cancel-service acknowledgement is not a stop.
            return getattr(future.result(), "status", None) in {4, 5, 6}
        except Exception:
            return False

    def _request_goal_cancel(self, record: dict[str, Any]) -> None:
        """Called under _goal_lock; never submit cancellation twice."""
        handle = record.get("handle")
        if (
            handle is None or not handle.accepted
            or record.get("cancel_requested") or self._request_is_terminal(record)
        ):
            return
        record["cancel_requested"] = True
        try:
            record["cancel_future"] = handle.cancel_goal_async()
        except Exception as exc:
            record["cancel_error"] = str(exc)

    def _goal_response(self, record: dict[str, Any], future: Any) -> None:
        """Own late acknowledgements even after the caller's deadline expires."""
        with self._goal_lock:
            if record["acknowledged"]:
                return
            record["acknowledged"] = True
            record["ack_time"] = time.monotonic()
            try:
                handle = future.result()
                if handle is None:
                    raise ExecutionFailure("goal acknowledgement contained no handle")
                record["handle"] = handle
                if handle.accepted:
                    # This is the only defensible baseline for a no-start
                    # watchdog: the pose actually observed when the server
                    # acknowledged this exact goal.  The reviewed first point
                    # may legitimately differ by the bounded initial alignment.
                    if "motion_hold_s" in record:
                        latest_joints = getattr(self, "latest_joints", {})
                        record["ack_positions"] = (
                            [latest_joints[name] for name in UR_JOINTS]
                            if all(name in latest_joints for name in UR_JOINTS)
                            else None
                        )
                        record["ack_joint_stamp"] = getattr(self, "latest_joint_stamp", 0.0)
                    self.active_goals.append(handle)
                    record["result_future"] = handle.get_result_async()
            except Exception as exc:
                # A transport/future error is not evidence that the server
                # rejected the request. Keep the unknown outcome registered.
                record["error"] = str(exc)
            if self._stopping or record["abandoned"]:
                self._request_goal_cancel(record)

    def _has_unresolved_goals(self) -> bool:
        with self._goal_lock:
            return any(not self._request_is_terminal(record) for record in self._goal_requests)

    def _forget_terminal_goal(self, handle: Any) -> None:
        with self._goal_lock:
            matching = [record for record in self._goal_requests if record.get("handle") is handle]
            if any(not self._request_is_terminal(record) for record in matching):
                raise ExecutionFailure("The action result does not confirm a terminal goal state")
            if handle in self.active_goals:
                self.active_goals.remove(handle)
            self._goal_requests = [
                record for record in self._goal_requests if record.get("handle") is not handle
            ]

    def _abort_active_goals(
        self,
        *,
        timeout_s: float = CANCEL_CONFIRM_TIMEOUT_S,
    ) -> tuple[bool, str]:
        """Confirm terminal outcomes, fresh stationary UR feedback and Klipper stop."""
        with self._goal_lock:
            self._stopping = True
            if self._stop_requested_at is None:
                self._stop_requested_at = time.monotonic()
                try:
                    self._stop_requested_ros_ns = self.node.get_clock().now().nanoseconds
                except AttributeError:
                    self._stop_requested_ros_ns = None
                self._stationary_observation = None
            for record in self._goal_requests:
                record["abandoned"] = True
                self._request_goal_cancel(record)
        # Concurrent UI cancellation and main() failure cleanup share one
        # waiter and one emergency request. Callbacks never take _abort_lock.
        with self._abort_lock:
            emergency_required = self._klipper_emergency_required()
            emergency_error = ""
            deadline = time.monotonic() + timeout_s
            while True:
                now = time.monotonic()
                if emergency_required:
                    emergency_error = self._start_or_retry_emergency_stop(now)
                with self._goal_lock:
                    for record in self._goal_requests:
                        future = record.get("future")
                        if future is not None and future.done() and not record["acknowledged"]:
                            self._goal_response(record, future)
                        self._request_goal_cancel(record)
                    unresolved = [
                        record for record in self._goal_requests if not self._request_is_terminal(record)
                    ]
                    emergency_ok = not emergency_required
                    emergency_future = self._emergency_stop_future
                    if emergency_required and emergency_future is not None and emergency_future.done():
                        try:
                            result = emergency_future.result()
                            emergency_ok = result is not None and result.success is True
                            if not emergency_ok:
                                response_message = str(
                                    getattr(result, "message", "")
                                ).strip()
                                if not emergency_error:
                                    emergency_error = (
                                        response_message
                                        or "Klipper did not confirm its emergency stop"
                                    )
                        except Exception as exc:
                            emergency_error = f"Klipper emergency stop failed: {exc}"
                    stationary_required = self._ur_stationary_required()
                    stationary = self._ur_stationary_after_stop(time.monotonic())
                    details = [
                        f"{record['label']} goal acknowledgement/outcome remains unknown"
                        if not record["acknowledged"] or record.get("error")
                        else f"{record['label']} goal terminal stop is unconfirmed"
                        for record in unresolved
                    ]
                    if emergency_required and not emergency_ok:
                        details.append(emergency_error or "Timed out confirming Klipper emergency stop")
                    if stationary_required and not stationary:
                        details.append("Fresh post-stop UR joint feedback has not confirmed a continuously stationary arm")
                    self._store_stop_snapshot(
                        details,
                        emergency_ok,
                        stationary,
                        emergency_required,
                        stationary_required,
                    )
                    if (
                        not unresolved and emergency_ok
                        and (not stationary_required or stationary)
                    ):
                        self.active_goals.clear()
                        self._goal_requests.clear()
                        self._stop_confirmed = True
                        klipper_message = (
                            "Klipper emergency stop confirmed"
                            if emergency_required
                            else "Klipper emergency stop was not required before dispatch"
                        )
                        ur_message = (
                            "fresh UR feedback remained stationary for 0.25 s"
                            if stationary_required
                            else "UR stationary evidence was not required before dispatch"
                        )
                        return True, (
                            "All submitted goals reached terminal states; "
                            f"{ur_message}; {klipper_message}"
                        )
                    self._stop_confirmed = False
                    if time.monotonic() >= deadline or (
                        not unresolved and emergency_error and emergency_future is None
                    ):
                        return False, "; ".join(details)
                time.sleep(0.01)

    def _start_or_retry_emergency_stop(self, now: float) -> str:
        """Submit only bounded, rate-limited retries of the stop-only service.

        Motion requests are never retried: their delivery state could be
        unknown.  The emergency service is different—it can only request a
        Klipper shutdown—and a completed false response must not latch the
        executor forever when the bridge can later prove that shutdown.
        """
        future = self._emergency_stop_future
        if future is not None:
            if not future.done():
                return ""
            try:
                result = future.result()
                if result is not None and result.success is True:
                    return ""
            except Exception:
                pass
            if now < self._next_emergency_stop_attempt_at:
                return ""
            self._emergency_stop_future = None

        if self._emergency_stop_attempts >= EMERGENCY_STOP_MAX_ATTEMPTS:
            return "Klipper emergency-stop confirmation attempts are exhausted"
        if not self.emergency_client.service_is_ready():
            return "Klipper emergency-stop service is not available"
        self._emergency_stop_attempts += 1
        self._next_emergency_stop_attempt_at = (
            now + EMERGENCY_STOP_RETRY_INTERVAL_S
        )
        try:
            self._emergency_stop_future = self.emergency_client.call_async(
                self.Trigger.Request()
            )
        except Exception as exc:
            self._emergency_stop_future = None
            return f"Klipper emergency-stop request failed: {exc}"
        return ""

    def _cancel_callback(self, _request: Any, response: Any) -> Any:
        self.cancel_requested.set()
        self._record_root_error("Execution canceled by the operator")
        confirmed, message = self._abort_active_goals()
        self._publish_failure_state("canceling" if confirmed else "stop-unconfirmed")
        response.success = confirmed
        response.message = message
        return response

    def _write_status(self) -> None:
        with self._goal_lock:
            self.status["updated_at"] = time.time()
            self.status["diagnostics"] = self._diagnostic_snapshot()
            self._status_sequence += 1
            sequence, payload = self._status_sequence, copy.deepcopy(self.status)
        with self._status_write_lock:
            if sequence > self._last_written_status_sequence:
                atomic_json(self.status_path, payload)
                self._last_written_status_sequence = sequence

    def _set_status(self, state: str, message: str, **updates: Any) -> None:
        with self._goal_lock:
            self.status.update(updates)
            self.status["state"] = state
            self.status["message"] = message
        self._write_status()

    def _preflight_details(self) -> dict[str, bool]:
        now = time.monotonic()
        return {
            "klipper_action_server": self.klipper_client.server_is_ready(),
            "joint_state_received": bool(self.latest_joints),
            "joint_state_fresh": bool(
                self.latest_joints and now - self.latest_joint_stamp < 2.0
            ),
            "program_state_received": self.program_state_stamp > 0.0,
            "external_control_running": bool(self.program_running),
            "trajectory_action_server": self.ur_client.server_is_ready(),
            **self._trajectory_socket_snapshot(),
        }

    @staticmethod
    def _preflight_waiting_for(details: dict[str, bool]) -> list[str]:
        missing: list[str] = []
        if not details["program_state_received"]:
            missing.append("live External Control state")
        elif not details["external_control_running"]:
            missing.append("External Control Play")
        if not details["joint_state_fresh"]:
            missing.append("fresh six-joint state")
        if not details["trajectory_socket_received"]:
            missing.append("live UR trajectory socket state")
        elif not details["trajectory_socket_fresh"]:
            missing.append("fresh UR trajectory socket state")
        elif not details["trajectory_socket_connected"]:
            missing.append("UR trajectory socket connection")
        if not details["klipper_action_server"]:
            missing.append("Klipper bridge action server")
        return missing

    def wait_for_services(self) -> None:
        self._require_controller_stop_protocol()
        self._require_bridge_protocol()
        self._set_status(
            "preflight",
            "Checking External Control, joint state, trajectory socket, Klipper bridge, and trajectory controller",
        )
        deadline = time.monotonic() + 15.0
        controller_switched = False
        last_status_write = 0.0
        details = self._preflight_details()
        while time.monotonic() < deadline and not self.cancel_requested.is_set():
            details = self._preflight_details()
            missing = self._preflight_waiting_for(details)
            now = time.monotonic()
            if now - last_status_write >= 0.75:
                message = (
                    "Waiting for " + ", ".join(missing)
                    if missing
                    else "Activating the reviewed trajectory controller"
                )
                self._set_status("preflight", message, preflight=details)
                last_status_write = now
            if not missing:
                if not controller_switched:
                    self._switch_to_passthrough_controller()
                    controller_switched = True
                if self.ur_client.wait_for_server(timeout_sec=5.0):
                    details = self._preflight_details()
                    details["trajectory_action_server"] = True
                    if not self._preflight_waiting_for(details):
                        self._set_status(
                            "preflight",
                            "Live motion stack verified",
                            preflight=details,
                        )
                        return
            self.klipper_client.wait_for_server(timeout_sec=0.1)
            time.sleep(0.05)
        if self.cancel_requested.is_set():
            raise ExecutionFailure("Execution canceled during ROS preflight")
        missing = self._preflight_waiting_for(details)
        if controller_switched and not details["trajectory_action_server"]:
            missing.append("passthrough trajectory action server")
        if not missing:
            missing.append("live ROS motion service")
        raise ExecutionFailure("Print preflight failed: unavailable " + ", ".join(missing))

    def _require_controller_stop_protocol(self) -> None:
        """Verify the running controller, not merely an installed library on disk."""
        client = self.controller_protocol_client
        service_deadline = time.monotonic() + 3.0
        safety = (
            "The running UR passthrough stop lifecycle guard is unconfirmed. "
            "No motion may be submitted until the controller version is confirmed."
        )

        def fail(reason: str, detail: str, *, attempts: int = 0, reported: str | None = None) -> None:
            version = {
                "required": CONTROLLER_STOP_PROTOCOL_VERSION,
                "reported": reported,
                "confirmed": False,
                "reason": reason,
                "attempts": attempts,
            }
            self.status["controller_stop_protocol"] = version
            raise ExecutionFailure(safety + " " + detail, diagnostics={"controller_stop_protocol": version})

        while True:
            if self.cancel_requested.is_set():
                raise ExecutionFailure("Execution canceled while checking the UR controller stop lifecycle version")
            remaining = service_deadline - time.monotonic()
            if remaining <= 0.0:
                fail("service_unavailable", "Controller parameter service is unavailable.")
            if client.wait_for_service(timeout_sec=min(0.1, remaining)):
                break
        if self.cancel_requested.is_set():
            raise ExecutionFailure("Execution canceled while checking the UR controller stop lifecycle version")
        request = self.GetParameters.Request()
        request.names = ["stop_lifecycle_guard_version"]
        query_attempts = 2
        last_failure = "query_timeout"
        last_detail = ""
        for attempt in range(1, query_attempts + 1):
            if self.cancel_requested.is_set():
                raise ExecutionFailure("Execution canceled while checking the UR controller stop lifecycle version")
            try:
                future = client.call_async(request)
                response_deadline = time.monotonic() + 3.0
                while not future.done() and time.monotonic() < response_deadline:
                    remaining = response_deadline - time.monotonic()
                    if self.cancel_requested.wait(min(0.02, max(0.0, remaining))):
                        raise ExecutionFailure("Execution canceled while checking the UR controller stop lifecycle version")
                if self.cancel_requested.is_set():
                    raise ExecutionFailure("Execution canceled while checking the UR controller stop lifecycle version")
                if not future.done():
                    remove_pending = getattr(client, "remove_pending_request", None)
                    if callable(remove_pending):
                        # Humble keeps unanswered call_async futures in the client.
                        # Remove each expired read before a bounded retry.
                        remove_pending(future)
                    last_failure = "query_timeout"
                    last_detail = (
                        f"Parameter query timed out after {attempt} attempt(s). "
                        "Check the controller parameter service while motion is stopped."
                    )
                    continue
                response = future.result()
            except ExecutionFailure:
                raise
            except Exception as exc:
                last_failure = "query_failed"
                last_detail = f"Parameter query failed after {attempt} attempt(s): {exc}"
                continue
            try:
                values = response.values
                actual = values[0].string_value if len(values) == 1 else None
            except (AttributeError, IndexError, TypeError):
                actual = None
            if not isinstance(actual, str) or not actual:
                fail(
                    "version_missing",
                    "Controller parameter response contained no stop lifecycle version. "
                    "Rebuild the patched ur_controllers overlay and restart the stopped motion stack.",
                    attempts=attempt,
                )
            if actual != CONTROLLER_STOP_PROTOCOL_VERSION:
                fail(
                    "version_mismatch",
                    f"Rebuild the patched ur_controllers overlay and restart the stopped motion stack. "
                    f"Reported version: {actual!r}.",
                    attempts=attempt,
                    reported=actual,
                )
            self.status["controller_stop_protocol"] = {
                "required": CONTROLLER_STOP_PROTOCOL_VERSION,
                "reported": actual,
                "confirmed": True,
                "reason": "confirmed",
                "attempts": attempt,
            }
            return
        fail(last_failure, last_detail, attempts=query_attempts)

    def _require_bridge_protocol(self) -> None:
        """Read-only capability check: never mix the fixed executor with an old bridge."""
        client = self.bridge_protocol_client
        if not client.wait_for_service(timeout_sec=3.0):
            raise ExecutionFailure("Updated Klipper bridge is unavailable; restart the bridge while both devices are stopped")
        request = self.GetParameters.Request()
        request.names = ["execution_protocol_version"]
        future = client.call_async(request)
        deadline = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < deadline:
            if self.cancel_requested.wait(0.02):
                raise ExecutionFailure("Execution canceled during bridge version check")
        response = future.result() if future.done() else None
        values = getattr(response, "values", [])
        if len(values) != 1 or values[0].string_value != BRIDGE_PROTOCOL_VERSION:
            raise ExecutionFailure(
                "The running Klipper bridge is outdated or its version is unconfirmed. "
                "Restart the updated bridge with both devices stopped before motion."
            )

    def _switch_controllers(
        self,
        *,
        activate: list[str],
        deactivate: list[str],
        allow_cancel: bool,
    ) -> None:
        if not self.switch_controller_client.wait_for_service(timeout_sec=5.0):
            raise ExecutionFailure("Controller manager switch service is unavailable")
        request = self.SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = request.STRICT
        future = self.switch_controller_client.call_async(request)
        deadline = time.monotonic() + 6.0
        while not future.done() and time.monotonic() < deadline:
            if allow_cancel and self.cancel_requested.wait(0.02):
                raise ExecutionFailure("Execution canceled while switching controller")
            if not allow_cancel:
                time.sleep(0.02)
        if not future.done() or future.result() is None or not future.result().ok:
            raise ExecutionFailure(
                "Controller manager refused the strict controller switch"
            )

    def _switch_to_passthrough_controller(self) -> None:
        self._switch_controllers(
            activate=["passthrough_trajectory_controller"],
            deactivate=["scaled_joint_trajectory_controller"],
            allow_cancel=True,
        )
        self.controller_switched = True

    def _restore_scaled_controller(self) -> None:
        if not self.controller_switched:
            return
        if self.active_goals:
            raise ExecutionFailure(
                "Refusing to restore controllers while active motion is unconfirmed"
            )
        self._switch_controllers(
            activate=["scaled_joint_trajectory_controller"],
            deactivate=["passthrough_trajectory_controller"],
            allow_cancel=False,
        )
        self.controller_switched = False

    def require_live_program(self) -> None:
        if not self.program_running or self.program_state_stamp <= 0.0:
            self._abort_active_goals()
            raise ExecutionFailure("External Control stopped during Print Execution")
        if not self.latest_joints or time.monotonic() - self.latest_joint_stamp > 2.0:
            self._abort_active_goals()
            raise ExecutionFailure("The live UR joint-state stream stopped during Print Execution")
        if getattr(self, "_coordination_active", False):
            self._require_coordination_scaling()

    def current_positions(self) -> list[float]:
        if not self.latest_joints or time.monotonic() - self.latest_joint_stamp > 2.0:
            raise ExecutionFailure("A fresh UR joint state is unavailable")
        return [self.latest_joints[name] for name in UR_JOINTS]

    def require_start_match(self, expected: list[float], tolerance: float, label: str) -> None:
        actual = self.current_positions()
        error = max(abs(actual[index] - expected[index]) for index in range(6))
        if error > tolerance:
            raise ExecutionFailure(
                f"{label} no longer matches the reviewed trajectory (maximum joint error "
                f"{error:.4f} rad > {tolerance:.4f} rad). Revalidate with MoveIt."
            )

    def _trajectory_goal(
        self,
        positions: list[list[float]] | dict[str, Any],
        duration_s: float,
        *,
        header_stamp_ns: int = 0,
    ) -> Any:
        from builtin_interfaces.msg import Duration
        from trajectory_msgs.msg import JointTrajectoryPoint

        goal = self.FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(UR_JOINTS)
        goal.goal_tolerance = []
        for name in UR_JOINTS:
            tolerance = self.JointTolerance()
            tolerance.name = name
            tolerance.position = ENDPOINT_TOLERANCE_RAD
            tolerance.velocity = STOP_JOINT_SPEED_RAD_S
            tolerance.acceleration = ENDPOINT_ACCELERATION_TOLERANCE_RAD_S2
            goal.goal_tolerance.append(tolerance)
        goal.goal_time_tolerance = Duration(sec=2, nanosec=0)
        if header_stamp_ns:
            seconds, nanoseconds = divmod(header_stamp_ns, 1_000_000_000)
            goal.trajectory.header.stamp.sec = seconds
            goal.trajectory.header.stamp.nanosec = nanoseconds
        if isinstance(positions, dict):
            profile = positions
            identity = self._profile_identity(profile)
            if getattr(self, "_validated_profile_identity", None) != identity:
                validate_profile(profile)
            if abs(duration_s-profile["duration_s"]) > 1e-8:
                raise ExecutionFailure("UR profile duration differs from the coordinated block")
        else:
            profile = compile_profile([
                {"time_s": duration_s*i/(len(positions)-1), "positions": q}
                for i,q in enumerate(positions)
            ], duration_s)
            identity = self._profile_identity(profile)
        hold_until = max(TRAJECTORY_PRELUDE_S, getattr(self, "_minimum_validated_hold_s", 0.0))
        for values in profile["points"]:
            point = JointTrajectoryPoint()
            point.positions = list(values["positions"])
            point.velocities = list(values["velocities"])
            point.accelerations = list(values["accelerations"])
            timestamp = hold_until + values["time_s"]
            total_nanoseconds = int(round(timestamp * 1_000_000_000))
            seconds, nanoseconds = divmod(total_nanoseconds, 1_000_000_000)
            point.time_from_start = Duration(
                sec=seconds,
                nanosec=nanoseconds,
            )
            goal.trajectory.points.append(point)
        if self.manifest.get("trajectory_policy_version") == TRAJECTORY_POLICY:
            if getattr(self, "_validated_profile_identity", None) != identity:
                raise ExecutionFailure("The final smooth curve has not passed collision validation")
            if not hasattr(self, "_smooth_goal_checks"):
                self._smooth_goal_checks = {}
            self._smooth_goal_checks[id(goal)] = {
                "start": list(self._validated_start_positions),
                "minimum_hold_s": self._minimum_validated_hold_s,
                "signature": self._goal_signature(goal),
            }
        return goal

    @staticmethod
    def _profile_identity(profile):
        return profile_identity(profile)

    def _prepared_profile(self, key):
        # load_manifest authenticates and certifies the complete bundle once.
        # Inert diagnostic fixtures constructing an executor directly use the
        # same one-time seam.  A changed in-memory profile cannot reuse it.
        if getattr(self, "_prepared_profiles", None) is None:
            self._prepared_profiles = validate_bundle(self.manifest)
            self._prepared_profile_digests = {}
        profile = self._prepared_profiles[key]
        digest = self._profile_identity(profile)
        previous = self._prepared_profile_digests.setdefault(id(profile), digest)
        if digest != previous:
            raise ExecutionFailure("Prepared smooth trajectory changed; prepare again")
        return profile

    @staticmethod
    def _goal_signature(goal):
        first = goal.trajectory.points[0].time_from_start
        offset = first.sec*1_000_000_000 + first.nanosec
        points = tuple((tuple(p.positions), tuple(p.velocities), tuple(p.accelerations),
                        p.time_from_start.sec*1_000_000_000+p.time_from_start.nanosec-offset)
                       for p in goal.trajectory.points)
        tolerances = tuple(
            (item.name, item.position, item.velocity, item.acceleration)
            for item in goal.goal_tolerance
        )
        goal_time = goal.goal_time_tolerance
        return points, tolerances, goal_time.sec*1_000_000_000+goal_time.nanosec

    def _readonly_result(self, future, deadline):
        if self.cancel_requested.is_set():
            raise ExecutionFailure("Execution canceled during smooth-trajectory validation")
        while not future.done():
            if self.cancel_requested.is_set():
                raise ExecutionFailure("Execution canceled during smooth-trajectory validation")
            if time.monotonic() >= deadline:
                raise ExecutionFailure("MoveIt smooth-trajectory validation timed out; no new motion was sent")
            self.cancel_requested.wait(0.005)
        try:
            value = future.result()
        except Exception as exc:
            raise ExecutionFailure(f"MoveIt validation service failed: {exc}") from exc
        if value is None:
            raise ExecutionFailure("MoveIt validation returned no result")
        return value

    def _validate_collision_samples_locally(
        self,
        samples: Iterable[tuple[list[float], list[float]]],
        deadline: float,
    ) -> int:
        """Check every candidate/reference pair in bounded local CSV batches."""

        validator = Path(self.local_collision_validator_path)
        if not validator.is_file() or not os.access(validator, os.X_OK):
            raise ExecutionFailure(
                "The local MoveIt curve validator is unavailable; rebuild the updated "
                "ur3e_print_planner package before motion"
            )
        geometry_sha256 = require_current_printhead_geometry(self.manifest)
        try:
            bed = self.manifest["print_bed_m"]
            expected_parameters = {
                "bed_min_x_m": finite(bed["minimum"]["x"], "bed minimum X"),
                "bed_max_x_m": finite(bed["maximum"]["x"], "bed maximum X"),
                "bed_min_y_m": finite(bed["minimum"]["y"], "bed minimum Y"),
                "bed_max_y_m": finite(bed["maximum"]["y"], "bed maximum Y"),
                "bed_plane_z_m": finite(bed["plane_z"], "bed plane Z"),
            }
            if (
                expected_parameters["bed_min_x_m"]
                >= expected_parameters["bed_max_x_m"]
                or expected_parameters["bed_min_y_m"]
                >= expected_parameters["bed_max_y_m"]
            ):
                raise ExecutionFailure("Prepared table bounds are invalid")
            tool = self.manifest.get("tool_center_point_m", {})
            radius = finite(tool.get("collision_radius_m", 0.0), "tool collision radius")
            if radius < 0.0 or radius > 0.25:
                raise ExecutionFailure("Prepared tool collision radius is invalid")
            offset = tool.get("offset", {}) if radius > 0.0 else {}
            expected_parameters.update(
                {
                    "tool_collision_radius_m": radius,
                    "tcp_offset_x_m": finite(offset.get("x", 0.0), "tool offset X"),
                    "tcp_offset_y_m": finite(offset.get("y", 0.0), "tool offset Y"),
                    "tcp_offset_z_m": finite(offset.get("z", 0.0), "tool offset Z"),
                }
            )
        except (KeyError, TypeError) as exc:
            raise ExecutionFailure(
                "Prepared workcell geometry is incomplete for local MoveIt validation"
            ) from exc

        sample_iterator = iter(samples)
        sample_paths: list[Path] = []
        total_count = 0
        process: subprocess.Popen[str] | None = None
        try:
            self.status_path.parent.mkdir(parents=True, exist_ok=True)
            for batch_index in range(MAX_LOCAL_VALIDATION_BATCH_FILES):
                sample_path = self.status_path.parent / (
                    f".{self.manifest['execution_id']}.{os.getpid()}."
                    f"{batch_index}.joint-samples.csv"
                )
                batch_count = 0
                with sample_path.open("x", encoding="utf-8", newline="") as stream:
                    # Record ownership only after this process has created the
                    # file.  A pre-existing collision must fail closed without
                    # deleting another execution's artifact in ``finally``.
                    sample_paths.append(sample_path)
                    writer = csv.writer(stream, lineterminator="\n")
                    writer.writerow(
                        [f"candidate_{name}" for name in UR_JOINTS]
                        + [f"reference_{name}" for name in UR_JOINTS]
                    )
                    for pair in itertools.islice(
                        sample_iterator, LOCAL_VALIDATION_BATCH_SAMPLES
                    ):
                        batch_count += 1
                        try:
                            candidate, reference = pair
                        except (TypeError, ValueError) as exc:
                            raise ExecutionFailure(
                                "Final smooth-curve validation produced an invalid fidelity pair"
                            ) from exc
                        if (
                            len(candidate) != len(UR_JOINTS)
                            or len(reference) != len(UR_JOINTS)
                            or not all(
                                math.isfinite(float(value))
                                for value in itertools.chain(candidate, reference)
                            )
                        ):
                            raise ExecutionFailure(
                                "Final smooth-curve validation produced an invalid fidelity pair"
                            )
                        writer.writerow(
                            f"{float(value):.17g}"
                            for value in itertools.chain(candidate, reference)
                        )
                        prepared_count = total_count + batch_count
                        if prepared_count % 8192 == 0:
                            self.require_live_program()
                            if self.cancel_requested.is_set():
                                raise ExecutionFailure(
                                    "Execution canceled while preparing local MoveIt validation"
                                )
                            self._set_status(
                                "preflight",
                                f"Prepared {prepared_count} exact smooth-curve states "
                                "for one-scene local MoveIt validation; no motion sent",
                            )
                        if time.monotonic() >= deadline:
                            raise ExecutionFailure(
                                "Final smooth-curve validation timed out before every "
                                "state could be checked; no motion was sent"
                            )
                if batch_count == 0:
                    sample_path.unlink(missing_ok=True)
                    sample_paths.pop()
                    break
                total_count += batch_count
            else:
                # Prove that the iterator is exhausted rather than silently
                # truncating a path at the guarded batch-file ceiling.
                exhausted = object()
                if next(sample_iterator, exhausted) is not exhausted:
                    raise ExecutionFailure(
                        "Final smooth curve exceeds the guarded local validation batch limit"
                    )

            if total_count < 1:
                raise ExecutionFailure("Final smooth curve produced no collision samples")
            self._set_status(
                "preflight",
                f"MoveIt is checking all {total_count} final smooth-curve states "
                f"across {len(sample_paths)} bounded batches in one scene; no motion sent",
            )
            command = [
                str(validator),
                "--ros-args",
                "-p",
                f"sample_csv_count:={len(sample_paths)}",
                "-p",
                f"printhead_geometry_sha256:={geometry_sha256}",
            ]
            for index, sample_path in enumerate(sample_paths):
                command.extend(["-p", f"sample_csv_{index}:={sample_path}"])
            for name, value in expected_parameters.items():
                encoded = f"{value:.17g}"
                # ROS parses a bare zero as an integer even when the declared
                # parameter is double.
                if "." not in encoded and "e" not in encoded.lower():
                    encoded += ".0"
                command.extend(["-p", f"{name}:={encoded}"])
            process = subprocess.Popen(
                command,
                cwd=self.status_path.parent,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            failure: Exception | None = None
            while process.poll() is None:
                try:
                    self.require_live_program()
                    if self.cancel_requested.is_set():
                        raise ExecutionFailure(
                            "Execution canceled during local MoveIt validation"
                        )
                    if time.monotonic() >= deadline:
                        raise ExecutionFailure(
                            "Final smooth-curve validation timed out before every "
                            "state could be checked; no motion was sent"
                        )
                except Exception as exc:
                    failure = exc
                    process.terminate()
                    break
                time.sleep(0.02)
            try:
                output, _unused = process.communicate(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                output, _unused = process.communicate(timeout=2.0)
            if failure is not None:
                raise failure

            certificate_keys = (
                "VALIDATION_PROTOCOL",
                "PRINTHEAD_GEOMETRY_SHA256",
                "VALIDATED_BATCHES",
                "VALIDATED_SAMPLES",
                "FIDELITY_SAMPLES",
                "MAX_TOOL0_TRANSLATION_M",
                "RMS_TOOL0_TRANSLATION_M",
                "MAX_TOOL0_ROTATION_RAD",
                "RMS_TOOL0_ROTATION_RAD",
            )
            parsed: dict[str, list[str]] = {}
            for line in (output or "").splitlines():
                for key in certificate_keys:
                    if line.startswith(key + "="):
                        parsed.setdefault(key, []).append(line.split("=", 1)[1])
            protocol = checked_batches = checked = fidelity_count = None
            metrics = None
            try:
                if any(len(parsed.get(key, [])) != 1 for key in certificate_keys):
                    raise ValueError("missing or duplicate certificate field")
                protocol = int(parsed["VALIDATION_PROTOCOL"][0])
                checked_batches = int(parsed["VALIDATED_BATCHES"][0])
                checked = int(parsed["VALIDATED_SAMPLES"][0])
                fidelity_count = int(parsed["FIDELITY_SAMPLES"][0])
                values = {
                    "maximum_translation_m": float(
                        parsed["MAX_TOOL0_TRANSLATION_M"][0]
                    ),
                    "rms_translation_m": float(parsed["RMS_TOOL0_TRANSLATION_M"][0]),
                    "maximum_rotation_rad": float(parsed["MAX_TOOL0_ROTATION_RAD"][0]),
                    "rms_rotation_rad": float(parsed["RMS_TOOL0_ROTATION_RAD"][0]),
                }
                if (
                    protocol != 3
                    or checked_batches < 0
                    or checked < 0
                    or fidelity_count < 0
                    or any(
                        not math.isfinite(value) or value < 0.0
                        for value in values.values()
                    )
                    or values["rms_translation_m"]
                    > values["maximum_translation_m"] + 1.0e-15
                    or values["rms_rotation_rad"]
                    > values["maximum_rotation_rad"] + 1.0e-15
                ):
                    raise ValueError("invalid certificate values")
                metrics = {
                    "samples": fidelity_count,
                    **values,
                    "translation_limit_m": TOOL0_FIDELITY_TRANSLATION_LIMIT_M,
                    "rotation_limit_rad": TOOL0_FIDELITY_ROTATION_LIMIT_RAD,
                    "passed": (
                        values["maximum_translation_m"]
                        <= TOOL0_FIDELITY_TRANSLATION_LIMIT_M
                        and values["maximum_rotation_rad"]
                        <= TOOL0_FIDELITY_ROTATION_LIMIT_RAD
                    ),
                }
            except (KeyError, TypeError, ValueError, OverflowError):
                metrics = None

            if process.returncode != 0:
                detail = " ".join((output or "").strip().split())[-1000:]
                raise ExecutionFailure(
                    "Final smooth curve is invalid, in collision, or outside TCP "
                    "fidelity limits; no new motion was sent"
                    + (f": {detail}" if detail else "")
                )
            if (
                checked_batches != len(sample_paths)
                or checked != total_count
                or fidelity_count != total_count
            ):
                raise ExecutionFailure(
                    "The local MoveIt validator did not certify every smooth-curve "
                    "state and fidelity pair in its one-scene certificate"
                )
            if metrics is None:
                raise ExecutionFailure(
                    "The local MoveIt validator returned an incomplete TCP fidelity certificate"
                )
            if parsed["PRINTHEAD_GEOMETRY_SHA256"] != [geometry_sha256]:
                raise ExecutionFailure(
                    "The local MoveIt validator uses obsolete printhead geometry; rebuild and review again"
                )
            self._last_fidelity_metrics = metrics
            self.status.setdefault("smooth_validation", {})[
                "tcp_fidelity"
            ] = copy.deepcopy(metrics)
            if not metrics["passed"]:
                raise ExecutionFailure(
                    "Final smooth curve failed TCP fidelity limits: "
                    f"translation {metrics['maximum_translation_m']:.9f} m > "
                    f"{TOOL0_FIDELITY_TRANSLATION_LIMIT_M:.9f} m or rotation "
                    f"{metrics['maximum_rotation_rad']:.9f} rad > "
                    f"{TOOL0_FIDELITY_ROTATION_LIMIT_RAD:.9f} rad; no new motion was sent"
                )
            return total_count
        except FileExistsError as exc:
            raise ExecutionFailure(
                "A stale local MoveIt validation artifact blocks this execution"
            ) from exc
        finally:
            if process is not None and process.poll() is None:
                process.kill()
                process.wait(timeout=2.0)
            for sample_path in sample_paths:
                sample_path.unlink(missing_ok=True)

    def _validate_smooth_motion(self, profile):
        """Read-only gate on the final NOMINAL quintic, before either actuator is armed.

        Collision checks cover the full robot/attachments through MoveIt. Exact
        Bernstein derivative bounds cover v/a/jerk between samples. Collision
        checking remains finite-sample model validation, not physical approval.
        The UR script starts segments from measured joints: tracking error is
        not included in this nominal curve certificate.
        """
        require_current_printhead_geometry(self.manifest)
        self._last_fidelity_metrics = None
        identity = self._profile_identity(profile)
        if getattr(self, "_prepared_profile_digests", {}).get(id(profile)) != identity:
            validate_profile(profile)
        self.require_live_program()
        actual = self.current_positions()
        velocities = self.latest_joint_velocities
        if any(name not in velocities or not math.isfinite(velocities[name]) or
               abs(velocities[name]) > STOP_JOINT_SPEED_RAD_S for name in UR_JOINTS):
            raise ExecutionFailure("Fresh stationary six-joint feedback is required before a smooth trajectory")
        # A complete job goes through this gate once.  Size its wall-clock
        # budget from the exact number of finite collision samples rather than
        # rejecting every otherwise-valid curve at an unrelated global count.
        estimated_samples = profile_collision_sample_count(profile) + 64
        validation_budget_s = min(
            MAX_SMOOTH_VALIDATION_BUDGET_S,
            max(
                MIN_SMOOTH_VALIDATION_BUDGET_S,
                SMOOTH_VALIDATION_SECONDS_PER_SAMPLE * estimated_samples,
            ),
        )
        deadline = time.monotonic() + validation_budget_s
        self._set_status("preflight", "Checking the final smooth UR curve against MoveIt before motion")
        collision_validator = getattr(self, "_collision_sample_validator", None)
        required_clients = [self.planning_scene_client, self.model_parameters_client]
        if collision_validator is None:
            # Hardware-isolated unit fixtures and older diagnostic harnesses use
            # the original service seam. Production installs the local C++
            # validator above, avoiding one DDS round trip per sampled state.
            required_clients.append(self.state_validity_client)
        for client in required_clients:
            if not client.wait_for_service(timeout_sec=3.0):
                raise ExecutionFailure("MoveIt final-curve validation is unavailable; no new motion was sent")
        request = self.GetPlanningScene.Request()
        request.components.components = 1023  # All defined PlanningSceneComponents, including attachments/ACM.
        scene = self._readonly_result(self.planning_scene_client.call_async(request), deadline).scene
        request = self.GetParameters.Request()
        request.names = ["robot_description"]
        values = self._readonly_result(self.model_parameters_client.call_async(request), deadline).values
        try:
            robot = ET.fromstring(values[0].string_value)
            bounds = {}
            for joint in robot.findall("joint"):
                if joint.get("name") not in UR_JOINTS:
                    continue
                limit = joint.find("limit")
                bounds[joint.get("name")] = (-float("inf"), float("inf")) if joint.get("type") == "continuous" else (
                    finite(limit.get("lower"), "joint lower bound"), finite(limit.get("upper"), "joint upper bound"))
            if len(bounds) != 6:
                raise ValueError("six UR joint bounds are required")
        except (ET.ParseError, AttributeError, IndexError, ValueError, TypeError) as exc:
            raise ExecutionFailure(f"MoveIt robot joint limits could not be verified: {exc}") from exc
        guards = [obj for obj in scene.world.collision_objects if obj.id == "confirmed_table_surface_guard"]
        if len(guards) != 1 or len(guards[0].primitives) != 1 or len(guards[0].primitive_poses) != 1:
            raise ExecutionFailure("MoveIt collision scene is missing the confirmed table guard; replan")
        bed = self.manifest["print_bed_m"]
        guard = guards[0]
        dims, pose = guard.primitives[0].dimensions, guard.primitive_poses[0]
        if (len(dims) != 3 or not all(math.isfinite(v) for v in dims) or guard.primitives[0].type != 1 or
            abs(dims[0]-(bed["maximum"]["x"]-bed["minimum"]["x"])) > 1e-6 or
            abs(dims[1]-(bed["maximum"]["y"]-bed["minimum"]["y"])) > 1e-6 or dims[2] < 1.0):
            raise ExecutionFailure("MoveIt collision table dimensions differ from the prepared workcell")
        if guard.header.frame_id == "world":
            # MoveIt serializes collision objects in the model's world frame.
            # Accept it only if URDF proves a fixed identity chain to base_link.
            child, visited = "base_link", set()
            while child != "world":
                links = [j for j in robot.findall("joint")
                         if j.find("child") is not None and j.find("child").get("link") == child]
                if child in visited or len(links) != 1 or links[0].get("type") != "fixed":
                    raise ExecutionFailure("MoveIt world/base_link transform is unverified")
                visited.add(child)
                origin = links[0].find("origin")
                if origin is not None and any(abs(finite(v, "frame origin")) > 1e-9
                        for key in ("xyz", "rpy") for v in origin.get(key, "0 0 0").split()):
                    raise ExecutionFailure("MoveIt world/base_link transform is not identity")
                parent = links[0].find("parent")
                child = parent.get("link") if parent is not None else None
        elif guard.header.frame_id != "base_link":
            raise ExecutionFailure("MoveIt table coordinate frame is unverified; replan")
        # A canonical MoveIt object has a parent pose AND a relative shape pose.
        # The confirmed table is axis-aligned; reject any unverified rotation.
        for item in (guard.pose, pose):
            quat = item.orientation
            if (any(abs(finite(getattr(quat,k), "table orientation")) > 1e-6 for k in ("x","y","z"))
                    or abs(abs(finite(quat.w, "table orientation"))-1.0) > 1e-6):
                raise ExecutionFailure("MoveIt table orientation differs from the confirmed workcell")
        xyz = [finite(getattr(guard.pose.position,k), "table parent position") +
               finite(getattr(pose.position,k), "table shape position") for k in ("x","y","z")]
        expected = [(bed["minimum"]["x"]+bed["maximum"]["x"])/2,
                    (bed["minimum"]["y"]+bed["maximum"]["y"])/2, bed["plane_z"]-dims[2]/2]
        if any(abs(a-b) > 1e-6 for a,b in zip(xyz,expected)):
            raise ExecutionFailure("MoveIt collision table position differs from the prepared workcell")
        acm = scene.allowed_collision_matrix
        for name,row in zip(acm.entry_names, acm.entry_values):
            if name == guard.id and any(row.enabled):
                raise ExecutionFailure("MoveIt allows robot/table collisions; replan the workcell")
            if any(other == guard.id and enabled for other,enabled in zip(acm.entry_names,row.enabled)):
                raise ExecutionFailure("MoveIt allows robot/table collisions; replan the workcell")
        if any(enabled for enabled in acm.default_entry_values):
            raise ExecutionFailure("MoveIt has default collision exemptions; table checking is unverified")
        tool = self.manifest.get("tool_center_point_m", {})
        tool_radius = finite(tool.get("collision_radius_m", 0), "tool collision radius")
        attached = [obj for obj in scene.robot_state.attached_collision_objects
                    if obj.object.id == "measured_print_tool"]
        if tool_radius <= 0 and attached:
            raise ExecutionFailure("Unexpected measured print tool in a tool0-only collision scene")
        if tool_radius > 0:
            try:
                if len(attached) != 1:
                    raise ValueError("one measured print tool is required")
                item, offset = attached[0], tool["offset"]
                obj = item.object
                tcp = [finite(offset[k], "tool offset") for k in ("x","y","z")]
                length = math.sqrt(sum(v*v for v in tcp))
                if (item.link_name != "tool0" or obj.header.frame_id != "tool0" or
                        len(item.touch_links) != 3 or
                        set(item.touch_links) != {"tool0","flange","wrist_3_link"} or
                        len(obj.primitives) != 1 or len(obj.primitive_poses) != 1 or
                        obj.meshes or obj.mesh_poses or obj.planes or obj.plane_poses or length <= 0):
                    raise ValueError("tool frame, touch links or printhead envelope shape is inconsistent")
                def rotate(pose, vector):
                    x,y,z,w = [finite(getattr(pose.orientation,k), "tool quaternion") for k in ("x","y","z","w")]
                    if abs(x*x+y*y+z*z+w*w-1) > 1e-6:
                        raise ValueError("invalid tool orientation")
                    vx,vy,vz = vector
                    cx,cy,cz = y*vz-z*vy, z*vx-x*vz, x*vy-y*vx
                    return [vx+2*(w*cx+y*cz-z*cy), vy+2*(w*cy+z*cx-x*cz), vz+2*(w*cz+x*cy-y*cx)]
                head, head_pose = obj.primitives[0], obj.primitive_poses[0]
                if head.type != 1 or len(head.dimensions) != 3:
                    raise ValueError("printhead envelope is not the expected box")
                if any(abs(finite(actual, "printhead envelope dimension") - expected) > 1e-6
                       for actual, expected in zip(head.dimensions, PRINTHEAD_BOX_SIZE_M)):
                    raise ValueError("printhead envelope dimensions differ from the installed CAD bounds")
                head_center = rotate(obj.pose, [
                    finite(getattr(head_pose.position, k), "printhead envelope center")
                    for k in ("x", "y", "z")
                ])
                head_center = [
                    value + finite(getattr(obj.pose.position, axis), "tool parent position")
                    for value, axis in zip(head_center, ("x", "y", "z"))
                ]
                if any(abs(actual - expected) > 1e-6 for actual, expected in
                       zip(head_center, PRINTHEAD_BOX_CENTER_TOOL0_M)):
                    raise ValueError("printhead envelope center differs from the installed CAD bounds")
                for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1)):
                    rotated = rotate(obj.pose, rotate(head_pose, axis))
                    if any(abs(actual - expected) > 1e-6 for actual, expected in zip(rotated, axis)):
                        raise ValueError("printhead envelope orientation differs from tool0")
                for value, center, size in zip(
                    tcp, PRINTHEAD_BOX_CENTER_TOOL0_M, PRINTHEAD_BOX_SIZE_M
                ):
                    if abs(value - center) > size / 2 + PRINTHEAD_TCP_BOUNDS_TOLERANCE_M:
                        raise ValueError("measured TCP lies outside the printhead CAD envelope")
            except (AttributeError, KeyError, TypeError, ValueError) as exc:
                raise ExecutionFailure(f"Measured tool collision geometry is unverified: {exc}") from exc
        intro = compile_profile([{"time_s": 0.0, "positions": actual},
                                 {"time_s": .05, "positions": profile["points"][0]["positions"]}])
        for trajectory in (intro, profile):
            for a,b in zip(trajectory["points"], trajectory["points"][1:]):
                for j,name in enumerate(UR_JOINTS):
                    if any(not bounds[name][0] <= q <= bounds[name][1] for q in controls(a,b,j)):
                        raise ExecutionFailure("Final smooth curve exceeds the robot's continuous joint bounds")
        self._minimum_validated_hold_s = intro["duration_s"]
        samples = itertools.chain(
            paired_profile_samples(intro, reference_is_candidate=True, _already_validated=True),
            paired_profile_samples(profile, reviewed_path=getattr(self, "_reviewed_reference_path", None),
                                   _already_validated=True),
        )
        if collision_validator is not None:
            def bounded_samples():
                for pair in samples:
                    if time.monotonic() >= deadline:
                        raise ExecutionFailure(
                            "Final smooth-curve validation timed out before every "
                            "state could be checked; no motion was sent"
                        )
                    candidate, reference = pair
                    if any(
                        not bounds[name][0] <= value <= bounds[name][1]
                        for name, value in zip(UR_JOINTS, candidate)
                    ):
                        raise ExecutionFailure(
                            "Final smooth curve exceeds the robot's joint position limits"
                        )
                    yield candidate, reference

            count = collision_validator(bounded_samples(), deadline)
        else:
            samples = (candidate for candidate, _reference in samples)
            count, last_write = 0, time.monotonic()
            while True:
                batch = list(itertools.islice(samples, STATE_VALIDITY_BATCH_SIZE))
                if not batch:
                    break
                pending = []
                for q in batch:
                    if any(not bounds[name][0] <= value <= bounds[name][1] for name,value in zip(UR_JOINTS,q)):
                        raise ExecutionFailure("Final smooth curve exceeds the robot's joint position limits")
                    request = self.GetStateValidity.Request()
                    request.group_name = ""  # Whole robot, not just the manipulator group.
                    request.robot_state = copy.deepcopy(scene.robot_state)
                    state = request.robot_state.joint_state
                    positions = dict(zip(state.name, state.position))
                    positions.update(zip(UR_JOINTS, q))
                    state.name, state.position = list(positions), list(positions.values())
                    state.velocity, state.effort = [], []
                    request.robot_state.is_diff = False
                    pending.append(self.state_validity_client.call_async(request))
                for future in pending:
                    if self._readonly_result(future, deadline).valid is not True:
                        raise ExecutionFailure("Final smooth curve is invalid or in collision; no new motion was sent")
                count += len(batch)
                if time.monotonic() >= deadline:
                    raise ExecutionFailure(
                        "Final smooth-curve validation timed out before every state "
                        "could be checked; no motion was sent"
                    )
                if time.monotonic()-last_write >= .5:
                    self._set_status("preflight", f"MoveIt checked {count} final smooth-curve samples; no motion sent")
                    last_write = time.monotonic()
        self._validated_start_positions = actual
        self._validated_profile_identity = identity
        self.status["smooth_validation"] = {
            "passed": True,
            "samples": count,
            "spacing_rad": COLLISION_SAMPLE_SPACING_RAD,
            "bounds": profile["bounds"],
            "collision_backend": (
                "local_moveit_scene" if collision_validator is not None else "state_validity_service"
            ),
            "physical_validation": False,
            "tcp_fidelity": copy.deepcopy(
                getattr(self, "_last_fidelity_metrics", None)
            ),
        }

    def _check_smooth_dispatch(self, goal):
        check = getattr(self, "_smooth_goal_checks", {}).get(id(goal))
        if check is None or check["signature"] != self._goal_signature(goal):
            raise ExecutionFailure("UR goal changed after final smooth-curve validation")
        if max(abs(a-b) for a,b in zip(self.current_positions(), check["start"])) > .0001:
            raise ExecutionFailure("Robot posture changed during smooth-curve validation; validate again")
        if any(not math.isfinite(self.latest_joint_velocities.get(n, float("nan"))) or
               abs(self.latest_joint_velocities[n]) > STOP_JOINT_SPEED_RAD_S for n in UR_JOINTS):
            raise ExecutionFailure("Robot is no longer stationary before smooth-trajectory dispatch")
        hold = goal.trajectory.points[0].time_from_start
        if hold.sec+hold.nanosec/1e9 + 1e-9 < check["minimum_hold_s"]:
            raise ExecutionFailure("UR preparation hold is too short for the bounded initial alignment")

    def _prepare_goal_for_dispatch(self, goal: Any, label: str, deadline: float | None) -> None:
        schedule = getattr(self, "_scheduled_ur_goals", {}).pop(id(goal), None)
        if label != "UR" or schedule is None:
            return
        now = time.monotonic()
        remaining = schedule["start_monotonic"] - now
        if (deadline is not None and now >= deadline) or remaining < SYNC_ARM_GUARD_S:
            raise ExecutionFailure("Coordination preparation deadline expired; UR goal was not sent")
        # Keep every reviewed position and every inter-point duration unchanged.
        # This controller implements relative point times, not absolute headers.
        points = goal.trajectory.points
        first_ns = points[0].time_from_start.sec * 1_000_000_000 + points[0].time_from_start.nanosec
        hold_ns = int(round(remaining * 1_000_000_000))
        for point in points:
            old_ns = point.time_from_start.sec * 1_000_000_000 + point.time_from_start.nanosec
            sec, nanosec = divmod(old_ns - first_ns + hold_ns, 1_000_000_000)
            point.time_from_start.sec = sec
            point.time_from_start.nanosec = nanosec
        goal.trajectory.header.stamp.sec = 0
        goal.trajectory.header.stamp.nanosec = 0
        self.status["coordination"] = {
            "method": "single_ur_goal_and_virtual_sd_program",
            "physical_sync_verified": False,
            "ur_relative_hold_s": remaining,
            "preparation_remaining_s": deadline - now if deadline is not None else None,
        }
        # Do not do disk I/O between preparing the hold and dispatching the goal.

    def _klipper_program_goal(
        self,
        blocks: list[dict[str, Any]],
        maximum_speed: float,
    ) -> Any:
        """Freeze the complete Klipper timeline into one action request."""
        goal = self.ManualStepperProgram.Goal()
        goal.operation_id = f"print_{self.manifest['execution_id']}"
        goal.stepper = self.manifest["extruder"]
        goal.segment_units = [
            finite(block["klipper_units"], "block Klipper units")
            for block in blocks
        ]
        goal.segment_durations_s = [
            finite(block["duration_s"], "block duration")
            for block in blocks
        ]
        goal.maximum_speed = float(maximum_speed)
        goal.start_delay_s = float(SYNC_START_LEAD_S)
        goal.disable_after = True
        return goal

    def _send(
        self,
        client: Any,
        goal: Any,
        label: str,
        *,
        feedback_callback: Any | None = None,
        deadline: float | None = None,
    ) -> tuple[Any, Any]:
        # The executor may have been waiting for live services while a new
        # plan revoked this package. Recheck only the small request marker
        # immediately before dispatch; cancellation never passes this gate.
        require_current_printhead_geometry(self.manifest)
        request = self.manifest.get("planning_request", {})
        request_path = Path(str(request.get("container_path", request.get("path", ""))))
        if (
            self.manifest.get("planning_policy_version") != PLANNING_POLICY_VERSION
            or not request_path.is_file()
            or sha256_file(request_path) != request.get("sha256")
        ):
            raise ExecutionFailure("The planning request was revoked; validate and prepare again")
        deadline = time.monotonic() + 8.0 if deadline is None else deadline
        # The guard may initiate coordinated cancellation, so run it before
        # acquiring the send/response lock. An immediate ACK must not bypass
        # fresh program/joint-state and active coordination-scaling checks.
        self.require_live_program()
        with self._goal_lock:
            if self._stopping or self.cancel_requested.is_set():
                raise ExecutionFailure("Execution is stopping; no new goal may be sent")
            if time.monotonic() >= deadline:
                raise ExecutionFailure(f"{label} arming deadline expired before dispatch; no goal was sent")
            # Lock contention and the live-state guard can outlast the first
            # identity check. Recheck before retiming the goal's motion hold.
            require_current_printhead_geometry(self.manifest)
            prepare = getattr(self, "_prepare_goal_for_dispatch", None)
            if prepare is not None:
                prepare(goal, label, deadline)
            if label == "UR" and self.manifest.get("trajectory_policy_version") == TRAJECTORY_POLICY:
                self._check_smooth_dispatch(goal)
            if time.monotonic() >= deadline:
                raise ExecutionFailure(f"{label} arming deadline expired before dispatch; no goal was sent")
            if label == "UR":
                self.require_trajectory_socket()
            record: dict[str, Any] = {
                "label": label, "future": None, "acknowledged": False,
                "abandoned": False, "handle": None, "result_future": None,
                "submitted_at": time.monotonic(),
            }
            trajectory = getattr(goal, "trajectory", None)
            if label == "UR" and getattr(trajectory, "points", None):
                first = trajectory.points[0].time_from_start
                record["motion_hold_s"] = first.sec + first.nanosec / 1e9
            # Register before submission; even a synchronous transport error
            # cannot be assumed to mean that no request reached the server.
            self._goal_requests.append(record)
            try:
                future = client.send_goal_async(
                    goal,
                    **({"feedback_callback": feedback_callback} if feedback_callback is not None else {}),
                )
                record["future"] = future
                future.add_done_callback(lambda completed: self._goal_response(record, completed))
            except Exception as exc:
                record["error"] = str(exc)
                record["abandoned"] = True
                self._stopping = True
                raise ExecutionFailure(f"{label} goal submission outcome is unknown: {exc}") from exc
        try:
            while not record["acknowledged"] and time.monotonic() < deadline:
                self.require_live_program()
                if self.cancel_requested.wait(0.02):
                    raise ExecutionFailure("Execution canceled")
            with self._goal_lock:
                if not record["acknowledged"] or record.get("ack_time", float("inf")) > deadline:
                    raise ExecutionFailure(f"Timed out sending {label} goal")
                if record.get("error"):
                    raise ExecutionFailure(f"{label} goal acknowledgement outcome is unknown: {record['error']}")
                handle = record["handle"]
                if not handle.accepted:
                    self._goal_requests.remove(record)
                    raise ExecutionFailure(f"{label} rejected the synchronized goal")
                if self._stopping or self.cancel_requested.is_set():
                    raise ExecutionFailure("Execution canceled")
                return handle, record["result_future"]
        except Exception:
            with self._goal_lock:
                record["abandoned"] = True
                self._stopping = True
                self._request_goal_cancel(record)
            raise

    def _wait_for_klipper_dispatch(
        self,
        dispatched: threading.Event,
        result_future: Any,
        deadline: float,
    ) -> None:
        while not dispatched.is_set() and time.monotonic() < deadline:
            self.require_live_program()
            if self.cancel_requested.wait(0.01):
                raise ExecutionFailure("Execution canceled")
            if result_future.done():
                wrapper = result_future.result()
                result = None if wrapper is None else wrapper.result
                message = getattr(result, "message", "") if result else ""
                raise ExecutionFailure(
                    "Klipper failed before the synchronized dispatch: "
                    + (message or "no successful result")
                )
        if not dispatched.is_set():
            self._abort_active_goals()
            raise ExecutionFailure(
                "Klipper could not arm before the synchronized start deadline"
            )

    @staticmethod
    def _measured_profile_progress(
        profile: dict[str, Any],
        actual: list[float],
        *,
        minimum_time_s: float,
        maximum_time_s: float,
    ) -> tuple[float, float, float]:
        """Project joints only onto the causally reachable part of a profile."""
        points = profile["points"]
        duration = float(profile["duration_s"])
        low_time = min(duration, max(0.0, float(minimum_time_s)))
        high_time = min(duration, max(low_time, float(maximum_time_s)))
        best: tuple[float, float] | None = None

        def consider(profile_time: float, expected: list[float]) -> None:
            nonlocal best
            error = max(abs(value - target) for value, target in zip(actual, expected))
            candidate = (error, profile_time)
            if best is None or candidate < best:
                best = candidate

        # Full jobs contain thousands of points. Binary-search the causal
        # window rather than repeatedly scanning the immutable whole goal.
        time_key = lambda point: float(point["time_s"])
        first = max(0, bisect.bisect_left(points, low_time, key=time_key) - 1)
        last = min(len(points) - 1, bisect.bisect_right(points, high_time, key=time_key))
        for index in range(first, last):
            a, b = points[index], points[index + 1]
            start_time, end_time = float(a["time_s"]), float(b["time_s"])
            if end_time < low_time or start_time > high_time:
                continue
            dt = end_time - start_time
            lower_u = max(0.0, (low_time - start_time) / dt)
            upper_u = min(1.0, (high_time - start_time) / dt)
            curves = [controls(a, b, joint) for joint in range(6)]
            normalized_bound = max(
                5.0 * abs(y - x)
                for curve in curves for x, y in zip(curve, curve[1:])
            )
            divisions = max(2, int(math.ceil(normalized_bound / 0.001)))
            span = upper_u - lower_u
            for sample in range(divisions + 1):
                u = lower_u + span * sample / divisions
                expected = []
                for curve in curves:
                    work = list(curve)
                    while len(work) > 1:
                        work = [(1.0-u)*x + u*y for x, y in zip(work, work[1:])]
                    expected.append(work[0])
                consider(start_time + u*dt, expected)
        if best is None:
            consider(low_time, list(points[0]["positions"]))
        assert best is not None
        error, profile_time = best
        progress = min(1.0, max(0.0, profile_time / float(profile["duration_s"])))
        return progress, profile_time, error

    def _require_ur_endpoint(self, profile: dict[str, Any]) -> None:
        """Require fresh measured endpoint and stopped joints after action success."""
        expected = profile["points"][-1]["positions"]
        deadline = time.monotonic() + ENDPOINT_SETTLE_TIMEOUT_S
        last_error = float("inf")
        while True:
            self.require_live_program()
            actual = self.current_positions()
            last_error = max(abs(value-target) for value, target in zip(actual, expected))
            stopped = all(
                name in self.latest_joint_velocities
                and math.isfinite(self.latest_joint_velocities[name])
                and abs(self.latest_joint_velocities[name]) <= STOP_JOINT_SPEED_RAD_S
                for name in UR_JOINTS
            )
            if last_error <= ENDPOINT_TOLERANCE_RAD and stopped:
                return
            if time.monotonic() >= deadline:
                raise ExecutionFailure(
                    "UR action reported success but fresh measured endpoint confirmation failed "
                    f"(maximum joint error {last_error:.4f} rad)"
                )
            time.sleep(0.02)

    def _await_results(
        self,
        entries: Iterable[tuple[str, Any, Any]],
        timeout_s: float,
        *,
        motion_profile: dict[str, Any] | None = None,
        progress_range: tuple[float, float] = (0.0, 1.0),
        motion_description: str = "reviewed collision-free approach",
        timeline_blocks: list[dict[str, Any]] | None = None,
    ) -> None:
        entries = list(entries)
        timeline_end_times = [float(block["end_time_s"]) for block in (timeline_blocks or [])]
        deadline = time.monotonic() + timeout_s
        motion_profile_time = 0.0
        last_motion_status = -float("inf")
        motion_started_at = time.monotonic()
        motion_baseline = None
        no_start_deadline = None
        hold_s = 0.0
        motion_detected = False
        if motion_profile is not None:
            ur_handles = [handle for label, handle, _future in entries if label == "UR"]
            with self._goal_lock:
                records = [
                    record for record in self._goal_requests
                    if record.get("handle") in ur_handles
                ]
                record = records[0] if len(records) == 1 else None
                if record is not None:
                    motion_baseline = record.get("ack_positions")
                    motion_started_at = float(record.get("ack_time", motion_started_at))
                    hold_s = float(record.get("motion_hold_s", 0.0))
            if motion_baseline is None:
                raise ExecutionFailure("Fresh UR joints were unavailable when the goal was acknowledged")
            evidence_time = next(
                (
                    hold_s + float(point["time_s"])
                    for point in motion_profile["points"]
                    if max(abs(value - start) for value, start in zip(point["positions"], motion_baseline))
                    >= MOTION_COMMAND_EVIDENCE_RAD
                ),
                None,
            )
            if evidence_time is not None:
                no_start_deadline = motion_started_at + evidence_time + MOTION_START_GRACE_S
        while time.monotonic() < deadline:
            self.require_live_program()
            if motion_profile is not None and not motion_detected:
                try:
                    self.require_trajectory_socket()
                except ExecutionFailure:
                    self._abort_active_goals()
                    raise
            if self.cancel_requested.is_set():
                self._abort_active_goals()
                raise ExecutionFailure("Execution canceled")
            for label, _handle, future in entries:
                if not future.done():
                    continue
                wrapper = future.result()
                result = None if wrapper is None else wrapper.result
                failed = (
                    result is None
                    or getattr(wrapper, "status", None) != 4  # SUCCEEDED, not merely terminal.
                    or (label == "UR" and int(result.error_code) != 0)
                    or (label == "Klipper" and result.success is not True)
                )
                if failed:
                    self._abort_active_goals()
                    message = getattr(result, "error_string", "") if label == "UR" else getattr(result, "message", "")
                    raise ExecutionFailure(f"{label} motion failed: {message or 'no successful result'}")
            now = time.monotonic()
            actual = self.current_positions() if motion_profile is not None else None
            if motion_baseline is not None and no_start_deadline is not None:
                measured_delta = max(
                    abs(value - start)
                    for value, start in zip(actual, motion_baseline)
                )
                if measured_delta >= MOTION_OBSERVED_EVIDENCE_RAD:
                    motion_detected = True
                    no_start_deadline = None
                elif now >= no_start_deadline:
                    self._abort_active_goals()
                    raise ExecutionFailure(
                        "UR goal was accepted but produced no measured joint motion "
                        f"(less than {MOTION_OBSERVED_EVIDENCE_RAD:.4f} rad after the reviewed "
                        f"command exceeded {MOTION_COMMAND_EVIDENCE_RAD:.4f} rad)"
                    )
            if motion_profile is not None and now - last_motion_status >= STATUS_INTERVAL_S:
                moved = max(
                    abs(value - expected)
                    for value, expected in zip(actual, motion_baseline)
                )
                motion_detected = motion_detected or moved >= MOTION_OBSERVED_EVIDENCE_RAD
                if motion_detected:
                    causally_reachable_time = max(
                        0.0, now - motion_started_at - hold_s + MOTION_PROGRESS_LEAD_S
                    )
                    local_progress, profile_time, tracking_error = self._measured_profile_progress(
                        motion_profile,
                        actual,
                        minimum_time_s=motion_profile_time,
                        maximum_time_s=causally_reachable_time,
                    )
                    motion_profile_time = profile_time
                else:
                    local_progress, profile_time = 0.0, 0.0
                    tracking_error = max(
                        abs(value - expected)
                        for value, expected in zip(actual, motion_profile["points"][0]["positions"])
                    )
                progress = progress_range[0] + local_progress * (progress_range[1] - progress_range[0])
                remaining = max(0.0, float(motion_profile["duration_s"]) - profile_time)
                message = (
                    f"Moving through the {motion_description} · measured {local_progress * 100:.0f}% · "
                    f"nominally {remaining:.0f} s remaining"
                    if motion_detected
                    else "UR trajectory accepted; waiting for measured joint motion"
                )
                timeline_updates: dict[str, Any] = {}
                if timeline_blocks:
                    marker_index = min(
                        len(timeline_blocks) - 1,
                        bisect.bisect_right(timeline_end_times, profile_time),
                    )
                    marker = timeline_blocks[marker_index]
                    timeline_updates = {
                        "current_block": marker_index + 1,
                        "source_line": int(marker["source_line_start"]),
                        "block_kind": marker["kind"],
                    }
                self._set_status(
                    "running",
                    message,
                    progress=progress,
                    motion={
                        "source": "live_joint_state",
                        "profile_point_count": len(motion_profile["points"]),
                        "profile_time_s": profile_time,
                        "planned_duration_s": float(motion_profile["duration_s"]),
                        "nominal_remaining_s": remaining,
                        "elapsed_since_accept_s": now - motion_started_at,
                        "tracking_error_rad": tracking_error,
                        "measured_joint_delta_rad": moved,
                    },
                    **timeline_updates,
                )
                last_motion_status = now
            if all(future.done() for _label, _handle, future in entries):
                break
            time.sleep(0.02)
        if not all(future.done() for _label, _handle, future in entries):
            self._abort_active_goals()
            raise ExecutionFailure("Synchronized motion timed out")
        for label, handle, future in entries:
            wrapper = future.result()
            result = None if wrapper is None else wrapper.result
            if getattr(wrapper, "status", None) != 4:
                self._abort_active_goals()
                raise ExecutionFailure(f"{label} motion failed: action did not succeed")
            if label == "UR":
                if result is None or int(result.error_code) != 0:
                    message = getattr(result, "error_string", "") if result else "no result"
                    raise ExecutionFailure(f"UR trajectory failed: {message}")
            elif result is None or result.success is not True:
                message = getattr(result, "message", "") if result else "no result"
                raise ExecutionFailure(f"Klipper motion failed: {message}")
            self._forget_terminal_goal(handle)
        if motion_profile is not None and any(label == "UR" for label, _handle, _future in entries):
            self._require_ur_endpoint(motion_profile)

    def move_to_start(self, approach: list[dict[str, Any]]) -> None:
        self.status["mode"] = "move-to-start"
        self._reviewed_reference_path = approach if all("path_progress" in p for p in approach) else None
        self.require_start_match(approach[0]["positions"], START_TOLERANCE_RAD, "Current UR state")
        profile = self._prepared_profile("approach")
        duration = profile["duration_s"]
        self._validate_smooth_motion(profile)
        self._set_status("running", "Moving through the reviewed collision-free approach", progress=0.0)
        handle, future = self._send(
            self.ur_client,
            self._trajectory_goal(profile, duration),
            "UR",
        )
        self._await_results(
            [("UR", handle, future)], duration + 15.0, motion_profile=profile
        )
        self._set_status("at-start", "UR reached the reviewed print start", progress=1.0)

    def execute_print(self, trajectory: list[dict[str, Any]]) -> None:
        if self.manifest.get("operation_mode") not in {"tool0_motion_test", "production_print"}:
            raise ExecutionFailure("Execution package operation mode is invalid")
        self.status["mode"] = "execute"
        self._reviewed_reference_path = trajectory if all("path_progress" in p for p in trajectory) else None
        self._wait_for_coordination_scaling()
        self._require_coordination_scaling()
        self._coordination_active = True
        self.require_start_match(trajectory[0]["positions"], PRINT_START_TOLERANCE_RAD, "UR print start")
        profile = self._prepared_profile("print")
        blocks = self.manifest["blocks"]
        limits = self.manifest["stepper_limits"]
        maximum_speed = finite(limits["maximum_speed"], "Klipper maximum speed")
        for block in blocks:
            units = finite(block["klipper_units"], "block Klipper units")
            segment_duration = finite(block["duration_s"], "block duration")
            if segment_duration <= 0.0:
                raise ExecutionFailure("Prepared Klipper segment duration must be positive")
            acceleration = finite(limits["acceleration"], "extruder acceleration")
            distance_limit = finite(limits["max_extrude_only_distance"], "extruder move limit")
            if segment_duration + 1.0e-9 < minimum_extrusion_duration(
                units, maximum_speed, acceleration, distance_limit,
            ):
                raise ExecutionFailure("Prepared segment is too short for native extrusion acceleration")

        self.require_start_match(
            profile["points"][0]["positions"],
            PRINT_START_TOLERANCE_RAD,
            "Continuous smooth-job start",
        )
        self._validate_smooth_motion(profile)
        duration = finite(profile["duration_s"], "continuous print duration")
        ur_goal = self._trajectory_goal(profile, duration)
        self._check_smooth_dispatch(ur_goal)
        klipper_goal = self._klipper_program_goal(blocks, maximum_speed)
        self._set_status(
            "running",
            "Arming one continuous UR trajectory and one queued Klipper program",
            progress=0.0,
            current_block=1,
            source_line=int(blocks[0]["source_line_start"]),
            block_kind=blocks[0]["kind"],
        )

        dispatched = threading.Event()
        dispatch_info: dict[str, int] = {}

        def klipper_feedback(message: Any) -> None:
            feedback = getattr(message, "feedback", None)
            if (
                feedback is not None
                and int(feedback.phase)
                == int(self.ManualStepperProgram.Feedback.PHASE_DISPATCHED)
            ):
                dispatch_info["program_epoch_ros_ns"] = int(
                    getattr(feedback, "program_epoch_ros_ns", 0)
                )
                dispatched.set()

        setup_deadline = time.monotonic() + 30.0
        self.require_trajectory_socket()
        klipper_handle, klipper_future = self._send(
            self.klipper_client,
            klipper_goal,
            "Klipper",
            feedback_callback=klipper_feedback,
            deadline=setup_deadline,
        )
        self._wait_for_klipper_dispatch(dispatched, klipper_future, setup_deadline)
        program_epoch_ros_ns = dispatch_info.get("program_epoch_ros_ns", 0)
        if program_epoch_ros_ns <= 0:
            self._abort_active_goals()
            raise ExecutionFailure("Klipper did not report a valid continuous-program start epoch")
        now_ros_ns = self.node.get_clock().now().nanoseconds
        remaining = (
            program_epoch_ros_ns
            + int(SYNC_START_LEAD_S * 1_000_000_000)
            - now_ros_ns
        ) / 1_000_000_000
        if remaining < SYNC_ARM_GUARD_S:
            self._abort_active_goals()
            raise ExecutionFailure(
                "Klipper program start acknowledgement left too little time to arm the UR goal"
            )
        start_monotonic = time.monotonic() + remaining
        arm_deadline = start_monotonic - SYNC_ARM_GUARD_S
        self._scheduled_ur_goals = {
            id(ur_goal): {"start_monotonic": start_monotonic}
        }
        ur_handle, ur_future = self._send(
            self.ur_client,
            ur_goal,
            "UR",
            deadline=arm_deadline,
        )
        self._await_results(
            [
                ("UR", ur_handle, ur_future),
                ("Klipper", klipper_handle, klipper_future),
            ],
            duration + remaining + 20.0,
            motion_profile=profile,
            motion_description="continuous reviewed print path",
            timeline_blocks=blocks,
        )
        self.status["progress"] = 1.0
        self.status["current_block"] = len(blocks)
        self._write_status()

        # Real packages have passed load_manifest's shared scope validator.
        # Keep completion telemetry compatible with hardware-isolated direct
        # method fixtures; this does not authorize an unscoped real package.
        scope = self.manifest.get("execution_scope", {})
        extent = self.manifest.get("execution_extent", "sample")
        if self.manifest["operation_mode"] == "tool0_motion_test":
            message = (
                "Full G-code cold Tool0 test completed; all reviewed source blocks finished. "
                "Heating remained disabled; this was not a production print. Physical synchronization remains unverified."
                if extent == "full" else
                "Bounded cold Tool0 sample completed; the full G-code was not executed. "
                "Physical synchronization remains unverified."
            )
        else:
            message = (
                "Reviewed UR path and Klipper stepper program reported successful completion. "
                + ("All reviewed source blocks finished. " if extent == "full" else "Only the bounded sample ran. ")
                + "This executor did not control heating or fans; physical synchronization and material deposition were not independently verified."
            )
        self._set_status(
            "completed",
            message,
            progress=1.0,
            source_progress=float(scope.get("end_progress", blocks[-1]["end_progress"])),
            execution_extent=extent,
            execution_scope=copy.deepcopy(scope),
            current_block=len(blocks),
            source_line=int(blocks[-1]["source_line_end"]),
            block_kind=blocks[-1]["kind"],
        )

    def close(self, *, cleanup_wait: Any | None = None) -> None:
        # Do not destroy the action client while it can still receive a late
        # accepted goal. Keep this job alive (and new jobs locked out) until
        # every submitted request is known terminal. The injected wait is a
        # test watchdog seam; production uses a throttled ordinary sleep.
        cleanup_wait = time.sleep if cleanup_wait is None else cleanup_wait
        while (
            self._has_unresolved_goals()
            or bool(self.active_goals)
            or (self._stopping and not self._stop_confirmed and
                (bool(self._goal_requests) or self.cancel_requested.is_set()))
        ):
            confirmed, message = self._abort_active_goals(timeout_s=1.0)
            if confirmed:
                self._publish_failure_state("canceled" if self.cancel_requested.is_set() else "failed")
                break
            self._publish_failure_state("stop-unconfirmed")
            cleanup_wait(0.25)
        try:
            self._restore_scaled_controller()
        except Exception as exc:
            try:
                self.node.get_logger().error(
                    f"Could not restore scaled_joint_trajectory_controller: {exc}"
                )
            except Exception:
                pass
        # Humble's MultiThreadedExecutor.shutdown() waits for callbacks that
        # have entered its WorkTracker, but not handlers already queued in the
        # underlying ThreadPoolExecutor.  Keep every ROS entity alive until
        # that worker queue is drained; otherwise a late handler can touch a
        # destroyed subscription/action client and emit InvalidHandle from
        # Future.__del__ during process teardown.
        cleanup_errors: list[str] = []

        def log_cleanup_error(message: str) -> None:
            try:
                self.node.get_logger().error(message)
            except Exception:
                pass

        try:
            if self.executor.shutdown() is not True:
                cleanup_errors.append("executor shutdown did not complete")
        except Exception as exc:
            cleanup_errors.append(f"executor shutdown failed: {exc}")
        try:
            if cleanup_errors:
                self.spin_thread.join(timeout=2.0)
            else:
                self.spin_thread.join()
            if self.spin_thread.is_alive():
                cleanup_errors.append("executor spin thread is still alive")
        except Exception as exc:
            cleanup_errors.append(f"executor spin-thread join failed: {exc}")

        worker_pool = getattr(self.executor, "_executor", None)
        if worker_pool is None:
            cleanup_errors.append("executor worker pool is unavailable")
        elif not cleanup_errors:
            try:
                # Do not cancel queued handlers.  They must run while the Node
                # and all of its subscriptions/action clients are still alive.
                worker_pool.shutdown(wait=True)
            except Exception as exc:
                cleanup_errors.append(f"executor worker-pool drain failed: {exc}")

        handler_futures = list(getattr(self.executor, "_futures", ()))
        for future in handler_futures:
            try:
                complete = future.done()
            except Exception as exc:
                cleanup_errors.append(f"could not inspect a ROS handler: {exc}")
                continue
            if not complete:
                cleanup_errors.append("a ROS handler remained incomplete after worker drain")
                continue
            try:
                future.result()
            except Exception as exc:
                # Calling result() is intentional even when it raises: rclpy
                # records that the callback exception was retrieved instead of
                # printing it later from Future.__del__.
                log_cleanup_error(
                    f"ROS callback failed while draining shutdown work: {exc}"
                )

        if cleanup_errors:
            message = (
                "ROS runtime cleanup could not be safely drained: "
                + "; ".join(cleanup_errors)
            )
            print(f"FATAL: {message}", flush=True)
            try:
                # Do not call _set_status here.  A callback that failed to
                # drain may own _goal_lock or _status_write_lock, preventing
                # this process from ever reaching the hard exit below.
                atomic_json(
                    self.status_path,
                    {
                        "version": 1,
                        "execution_id": str(self.manifest.get("execution_id", "")),
                        "state": "failed",
                        "message": message,
                        "updated_at": time.time(),
                        "cleanup": {
                            "confirmed": False,
                            "reasons": list(cleanup_errors),
                        },
                    },
                )
            except Exception as exc:
                print(
                    f"Could not persist ROS cleanup failure: {exc}. {message}",
                    flush=True,
                )
            # This is a one-operation child process.  A normal return would
            # immediately enter Python teardown, whose atexit/GC ordering can
            # recreate the exact Future/Destroyable race we are avoiding.
            # The device stop has already been confirmed above; hard exit only
            # this executor process and let the parent report its nonzero code.
            os._exit(70)
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            self.rclpy.shutdown()
        except Exception:
            pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Execute a reviewed synchronized print")
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--status", required=True, type=Path)
    parser.add_argument("--mode", choices=("move-to-start", "execute"), required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    status: dict[str, Any] = {
        "version": 1,
        "state": "failed",
        "mode": args.mode,
        "message": "Print Execution did not start",
        "updated_at": time.time(),
    }
    executor: RosPrintExecutor | None = None
    try:
        manifest = load_manifest(args.manifest.resolve())
        status["execution_id"] = manifest["execution_id"]
        executor = RosPrintExecutor(manifest, args.status.resolve())
        executor.wait_for_services()
        if args.mode == "move-to-start":
            trajectory = load_trajectory(
                Path(manifest["approach_trajectory"]["container_path"])
            )
            executor.move_to_start(trajectory)
        else:
            trajectory = load_trajectory(
                Path(manifest["print_trajectory"]["container_path"])
            )
            executor.execute_print(trajectory)
        return 0
    except Exception as exc:
        message = str(exc) or type(exc).__name__
        if executor is not None:
            executor._record_root_error(exc)
            stop_required = bool(
                executor.cancel_requested.is_set()
                or getattr(executor, "active_goals", [])
                or getattr(executor, "_has_unresolved_goals", lambda: False)()
            )
            stop_confirmed, stop_message = (
                executor._abort_active_goals()
                if stop_required
                else (True, "")
            )
            if not stop_confirmed:
                state = "stop-unconfirmed"
                message = f"{message}; {stop_message}"
            else:
                state = (
                    "canceled"
                    if executor.cancel_requested.is_set()
                    else "failed"
                )
            executor._publish_failure_state(state)
        else:
            status["message"] = message
            atomic_json(args.status.resolve(), status)
        print(f"ERROR: {message}", flush=True)
        return 2
    finally:
        if executor is not None:
            executor.close()


if __name__ == "__main__":
    raise SystemExit(main())
