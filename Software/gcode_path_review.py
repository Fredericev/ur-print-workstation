"""Execution-locked 3D review data for calibrated G-code paths."""

from __future__ import annotations

import hashlib
import heapq
import json
import math
import re
from pathlib import Path
from typing import Any, Callable

try:
    from gcode_calibration import (
        print_z_candidate_ready_for_planning,
        tool_tip_offset_in_base,
        validate_calibration,
    )
    from gcode_parser import parse_gcode
    from printhead_geometry import printhead_geometry_sha256
except ModuleNotFoundError:
    from Software.gcode_calibration import (
        print_z_candidate_ready_for_planning,
        tool_tip_offset_in_base,
        validate_calibration,
    )
    from Software.gcode_parser import parse_gcode
    from Software.printhead_geometry import printhead_geometry_sha256


MAX_MOVEIT_WAYPOINTS = 300_000
PLANNING_POLICY_VERSION = "continuous-cartesian-approach-v4"
MAX_MOVEIT_RESULT_BYTES = 8 * 1024 * 1024
MAX_MOVEIT_CARTESIAN_SIMPLIFICATION_ERROR_M = 0.001
MAX_OFFLINE_SIMULATION_POINTS = 360
MAX_OFFLINE_VALIDATION_POINTS = 20_000
OFFLINE_SIMULATION_ALGORITHM_VERSION = "offline-ik-v1"
OFFLINE_POSITION_TOLERANCE_M = 0.0015
OFFLINE_ORIENTATION_TOLERANCE_RAD = 0.02
OFFLINE_MAX_JOINT_STEP_RAD = 0.35
OFFLINE_CARTESIAN_CHECK_STEP_M = 0.01
_AXES = ("x", "y", "z")

_UR_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# These robot-specific transforms are the same calibrated UR3e chain used by
# the 3D viewer.  Each revolute joint rotates about its local Z axis.
_UR3E_CALIBRATED_JOINT_ORIGINS = [
    ((0.0, 0.0, 0.15175669655291413), (0.0, 0.0, -3.3891539305726243e-7)),
    ((-0.00006470506157420388, 0.0, 0.0), (1.569666958096674, 0.0, 0.00000699643429615883)),
    ((-0.24351151612770078, 0.0, 0.0), (3.1379339981017003, 3.140776446145916, 3.141590226899748)),
    ((-0.21308022061379375, -0.0004228675518556208, 0.13127163719940804), (0.003221305822064254, 0.001264474842293318, 0.000002294291397259871)),
    ((0.00007433545427084375, -0.08528314119049236, 0.00002025793221502494), (1.5705587894393158, 0.0, 0.000005421801660058894)),
    ((-0.00013442575565489673, 0.09181822218034145, -0.0001463042112754946), (1.5692029166418646, math.pi, -3.141592600047048)),
]


class PathReviewError(ValueError):
    """The requested review cannot be produced without weakening a safety gate."""


def _round(value: float, digits: int = 6) -> float:
    result = round(value, digits)
    return 0.0 if result == -0.0 else result


def _transform_point(
    x_mm: float,
    y_mm: float,
    z_mm: float,
    calibration: dict[str, Any],
) -> dict[str, float]:
    transform = calibration["coordinate_transform"]
    origin = transform["origin_m"]
    scale = transform["scale_m_per_mm"]
    angle = math.radians(transform["rotation_z_deg"])
    cosine = math.cos(angle)
    sine = math.sin(angle)
    scaled_x = x_mm * scale
    scaled_y = y_mm * scale
    return {
        "x": _round(origin["x"] + scaled_x * cosine - scaled_y * sine),
        "y": _round(origin["y"] + scaled_x * sine + scaled_y * cosine),
        "z": _round(origin["z"] + z_mm * scale),
    }


def _layer_catalog(segments: list[dict[str, Any]]) -> tuple[list[dict[str, Any]], dict[float, int]]:
    counts: dict[float, int] = {}
    for segment in segments:
        z_value = _round(float(segment.get("z1", 0.0)), 4)
        counts[z_value] = counts.get(z_value, 0) + 1
    ordered = sorted(counts)
    layer_index = {z_value: index for index, z_value in enumerate(ordered)}
    layers = [
        {
            "index": index,
            "z_mm": z_value,
            "segment_count": counts[z_value],
        }
        for index, z_value in enumerate(ordered)
    ]
    return layers, layer_index


def _initial_table_model(
    calibration_validation: dict[str, Any],
) -> dict[str, Any]:
    """Describe the confirmed table without inventing an unknown thickness."""

    workcell = calibration_validation["workcell_geometry"]
    tabletop = workcell["tabletop"]
    return {
        "frame": workcell["frame_id"],
        "robot_base_lowest_z_m": 0.0,
        "tabletop": {
            "center_m": dict(tabletop["center_m"]),
            "surface_z_m": tabletop["surface_z_m"],
            "minimum": dict(tabletop["bounds_m"]["minimum"]),
            "maximum": dict(tabletop["bounds_m"]["maximum"]),
            "dimensions_m": dict(tabletop["dimensions_m"]),
            "yaw_deg": tabletop["yaw_deg"],
            "thickness_m": tabletop["thickness_m"],
            "safety_margin_m": tabletop["safety_margin_m"],
            "measurement": dict(tabletop["measurement"]),
            "footprint_source": "fixed_workcell_config",
            "visual_only": False,
            "collision_authoritative": True,
        },
    }


def build_offline_review(
    filename: object,
    content: object,
    calibration: object,
    *,
    maximum_preview_segments: int | None = None,
    allow_incomplete_simulation: bool = False,
    use_initial_workcell_model: bool = False,
    workcell_snapshot: dict[str, Any] | None = None,
    tabletop_height_candidate: bool = False,
    print_zero_z_m: float | None = None,
    print_z_candidate: bool = False,
) -> dict[str, Any]:
    """Build a bounded 3D preview without contacting ROS or hardware.

    An incomplete profile is accepted only for an explicitly simulation-only
    preview.  That result stays distinguishable from a physically calibrated
    review and can never be reused as a MoveIt or execution authorization.
    """

    analysis = (
        parse_gcode(filename, content)
        if maximum_preview_segments is None
        else parse_gcode(
            filename,
            content,
            maximum_preview_segments=maximum_preview_segments,
            maximum_timeline_events=maximum_preview_segments,
        )
    )
    if not analysis["safe_to_plan"]:
        raise PathReviewError(
            "Resolve the blocking parser findings before opening 3D path review."
        )
    physical_calibration_validation = validate_calibration(
        calibration,
        analysis["bounds"],
        analysis["file"]["sha256"],
        workcell_snapshot=workcell_snapshot,
        print_zero_z_m=print_zero_z_m,
    )
    if (
        not allow_incomplete_simulation
        and physical_calibration_validation["ready_to_save"] is not True
        and not (
            print_z_candidate
            and print_z_candidate_ready_for_planning(physical_calibration_validation)
        )
    ):
        raise PathReviewError(
            "Save a complete calibration for this G-code file before opening "
            "3D path review."
        )

    physical_calibration_profile = physical_calibration_validation["calibration"]
    calibration_validation = physical_calibration_validation
    calibration_profile = physical_calibration_profile
    operation_mode = physical_calibration_profile["operation_mode"]
    placement = physical_calibration_validation["placement"]
    effective_origin = dict(placement["origin_m"])
    placement_model = {
        "mode": "center_on_fixed_table",
        "assumed": False,
        "execution_authoritative": (
            physical_calibration_validation["ready_to_save"] is True
            and not tabletop_height_candidate
            and not print_z_candidate
        ),
        "tabletop_height_candidate": tabletop_height_candidate,
        "print_z_candidate": print_z_candidate,
        "execution_scope": (
            "cold_tool0_motion_test"
            if operation_mode == "tool0_motion_test"
            else "production_print"
        ),
        "table_surface_z_m": _round(
            float(physical_calibration_profile["print_bed_m"]["plane_z"])
        ),
        "print_zero_z_m": _round(float(placement["print_zero_z_m"])),
        "print_z_offset_m": _round(float(placement["print_z_offset_m"])),
        "bounds_source": placement["bounds_source"],
        "source_center_mm": dict(placement["source_center_mm"]),
        "print_center_m": dict(placement["print_center_m"]),
        "effective_origin_m": effective_origin,
        "applied_translation_m": {"x": 0.0, "y": 0.0, "z": 0.0},
    }
    # Retained as an API compatibility flag; fixed placement applies equally
    # to complete reviews and incomplete, read-only simulations.
    _ = use_initial_workcell_model

    extrusion = physical_calibration_validation["extrusion_mapping"]
    tcp_offset_base = tool_tip_offset_in_base(calibration_profile)
    execution_window = analysis["bounds"]["execution"]
    first_sequence = int(execution_window["first_sequence"])
    last_sequence = int(execution_window["last_sequence"])
    source_segments = [
        segment
        for segment in analysis["preview"]["segments"]
        if first_sequence <= int(segment.get("sequence", 0)) <= last_sequence
    ]
    if not source_segments:
        raise PathReviewError(
            "The parsed file contains no robot motion between its first and last print moves."
        )
    time_offset_s = max(
        0.0,
        float(source_segments[0].get("cumulative_s", 0.0))
        - float(source_segments[0].get("duration_s", 0.0)),
    )
    layers, layer_index = _layer_catalog(source_segments)
    transformed_segments: list[dict[str, Any]] = []
    for segment in source_segments:
        start = _transform_point(
            float(segment["x0"]),
            float(segment["y0"]),
            float(segment.get("z0", 0.0)),
            calibration_profile,
        )
        end = _transform_point(
            float(segment["x1"]),
            float(segment["y1"]),
            float(segment.get("z1", 0.0)),
            calibration_profile,
        )
        tool0_start = {
            axis: _round(start[axis] - tcp_offset_base[axis])
            for axis in _AXES
        }
        tool0_end = {
            axis: _round(end[axis] - tcp_offset_base[axis])
            for axis in _AXES
        }
        source_z = _round(float(segment.get("z1", 0.0)), 4)
        extrusion_delta = float(segment.get("extrusion_delta_mm", 0.0))
        transformed_segments.append(
            {
                "sequence": int(segment.get("sequence", len(transformed_segments) + 1)),
                "line": int(segment.get("line", 0)),
                "kind": "print" if segment.get("kind") == "print" else "travel",
                "layer": layer_index.get(source_z, 0),
                "source_z_mm": source_z,
                "x0": start["x"],
                "y0": start["y"],
                "z0": start["z"],
                "x1": end["x"],
                "y1": end["y"],
                "z1": end["z"],
                "tool0_x0": tool0_start["x"],
                "tool0_y0": tool0_start["y"],
                "tool0_z0": tool0_start["z"],
                "tool0_x1": tool0_end["x"],
                "tool0_y1": tool0_end["y"],
                "tool0_z1": tool0_end["z"],
                "duration_s": _round(float(segment.get("duration_s", 0.0)), 5),
                "cumulative_s": _round(
                    max(
                        0.0,
                        float(segment.get("cumulative_s", 0.0)) - time_offset_s,
                    ),
                    4,
                ),
                "extrusion_delta_mm": _round(extrusion_delta, 5),
                "klipper_units": _round(
                    extrusion_delta
                    * float(extrusion["units_per_e_mm"])
                    * int(extrusion["direction_sign"]),
                    5,
                ),
            }
        )

    timeline_events = []
    source_timeline_events = [
        event
        for event in analysis["motion_timeline"]["events"]
        if first_sequence <= int(event.get("sequence", 0)) <= last_sequence
    ]
    for event in source_timeline_events:
        extrusion_delta = float(event.get("extrusion_delta_mm", 0.0))
        timeline_events.append(
            {
                **event,
                "cumulative_s": _round(
                    max(
                        0.0,
                        float(event.get("cumulative_s", 0.0)) - time_offset_s,
                    ),
                    4,
                ),
                "klipper_units": _round(
                    extrusion_delta
                    * float(extrusion["units_per_e_mm"])
                    * int(extrusion["direction_sign"]),
                    5,
                ),
            }
        )

    canonical = json.dumps(
        {
            "algorithm_version": 3,
            "planning_policy_version": PLANNING_POLICY_VERSION,
            "source_sha256": analysis["file"]["sha256"],
            "profile_sha256": physical_calibration_validation["profile_sha256"],
            "model_profile_sha256": calibration_validation["profile_sha256"],
            "printhead_geometry_sha256": physical_calibration_validation["printhead_geometry_sha256"],
            "simulation_only": allow_incomplete_simulation,
            "placement_model": placement_model,
        },
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")

    calibration_gate_status = {
        gate["code"]: gate["status"]
        for gate in physical_calibration_validation["gates"]
    }
    workcell_model = _initial_table_model(calibration_validation)
    collision_bed = {
        **calibration_profile["print_bed_m"],
        "source": (
            "candidate_workcell_tabletop"
            if tabletop_height_candidate else "fixed_workcell_tabletop"
        ),
    }
    return {
        "status": (
            "offline_simulation"
            if allow_incomplete_simulation
            else "offline_ready"
        ),
        "mode": (
            "offline_simulation"
            if allow_incomplete_simulation
            else "offline"
        ),
        "simulation_only": allow_incomplete_simulation,
        "tabletop_height_candidate": tabletop_height_candidate,
        "print_z_candidate": print_z_candidate,
        "physical_calibration_complete": (
            physical_calibration_validation["production_ready"] is True
        ),
        "motion_test_calibration_complete": (
            physical_calibration_validation["motion_test_ready"] is True
        ),
        "calibration_ready": (
            physical_calibration_validation["ready_to_save"] is True
        ),
        "operation_mode": calibration_profile["operation_mode"],
        "execution_locked": True,
        "source": analysis["file"],
        "source_sha256": analysis["file"]["sha256"],
        "profile_sha256": physical_calibration_validation["profile_sha256"],
        "model_profile_sha256": calibration_validation["profile_sha256"],
        "printhead_geometry_sha256": physical_calibration_validation["printhead_geometry_sha256"],
        "printhead_recalibration_required": not physical_calibration_validation["printhead_geometry_matches"],
        "review_sha256": hashlib.sha256(canonical).hexdigest(),
        "planning_policy_version": PLANNING_POLICY_VERSION,
        "coordinate_frame": "base_link",
        "units": "m",
        "path": {
            "segments": transformed_segments,
            "source_segment_count": (
                len(source_segments)
                if not analysis["preview"]["sampled"]
                else analysis["preview"]["source_segment_count"]
            ),
            "returned_segment_count": len(transformed_segments),
            "sampled": analysis["preview"]["sampled"],
            "continuous": analysis["preview"].get("continuous", False),
            "window_source": execution_window["source"],
            "first_sequence": first_sequence,
            "last_sequence": last_sequence,
            "excluded_lead_in_move_count": execution_window[
                "excluded_lead_in_move_count"
            ],
            "excluded_shutdown_move_count": execution_window[
                "excluded_shutdown_move_count"
            ],
            "arc_chord_tolerance_mm": analysis["preview"].get(
                "arc_chord_tolerance_mm"
            ),
            "maximum_simplification_deviation_mm": analysis["preview"].get(
                "maximum_simplification_deviation_mm", 0.0
            ),
            "print_segment_count": sum(
                segment["kind"] == "print" for segment in transformed_segments
            ),
            "travel_segment_count": sum(
                segment["kind"] == "travel" for segment in transformed_segments
            ),
        },
        "layers": layers,
        "timeline": {
            **analysis["motion_timeline"],
            "events": timeline_events,
            "source_event_count": max(0, last_sequence - first_sequence + 1),
            "returned_event_count": len(timeline_events),
        },
        "calibration": calibration_profile,
        "physical_calibration": physical_calibration_profile,
        "placement_model": placement_model,
        "workcell_model": workcell_model,
        "source_bounds_mm": calibration_validation["source_bounds_mm"],
        "execution_bounds_mm": calibration_validation["execution_bounds_mm"],
        "transformed_bounds_m": calibration_validation["transformed_bounds_m"],
        "deposition_bounds_m": calibration_validation["deposition_bounds_m"],
        "physical_transformed_bounds_m": physical_calibration_validation[
            "transformed_bounds_m"
        ],
        "physical_deposition_bounds_m": physical_calibration_validation[
            "deposition_bounds_m"
        ],
        "tcp_offset_base_m": tcp_offset_base,
        "print_bed_m": calibration_profile["print_bed_m"],
        "collision_bed_m": collision_bed,
        "within_print_bed": calibration_validation["within_print_bed"],
        "print_bed_check": calibration_validation["print_bed_check"],
        "physical_print_bed_check": physical_calibration_validation[
            "print_bed_check"
        ],
        "air_run_check": physical_calibration_validation["air_run_check"],
        "extrusion_mapping": extrusion,
        "moveit_validation": None,
        "gates": [
            {"code": "source_parsed", "status": "passed"},
            {
                "code": "coordinate_transform",
                "status": calibration_gate_status["coordinate_transform"],
            },
            {
                "code": (
                    "tool0_reference"
                    if calibration_profile["operation_mode"] == "tool0_motion_test"
                    else "tcp_geometry"
                ),
                "status": calibration_gate_status[
                    "tool0_reference"
                    if calibration_profile["operation_mode"] == "tool0_motion_test"
                    else "tcp_geometry"
                ],
            },
            *(
                []
                if calibration_profile["operation_mode"] == "tool0_motion_test"
                else [
                    {
                        "code": "print_bed",
                        "status": calibration_gate_status["print_bed"],
                    }
                ]
            ),
            {
                "code": "full_resolution_geometry",
                "status": (
                    "required"
                    if analysis["preview"]["sampled"]
                    else "passed"
                ),
            },
            {"code": "moveit_plan", "status": "required"},
            {"code": "collision_check", "status": "required"},
        ],
    }


def _sample_indexes(length: int, maximum: int) -> list[int]:
    if length <= maximum:
        return list(range(length))
    if maximum < 2:
        return [0]
    indexes = {
        round(position * (length - 1) / (maximum - 1))
        for position in range(maximum)
    }
    return sorted(indexes)


def _mat_mul(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [
        [sum(left[row][inner] * right[inner][column] for inner in range(3)) for column in range(3)]
        for row in range(3)
    ]


def _mat_vec(matrix: list[list[float]], vector: tuple[float, float, float] | list[float]) -> list[float]:
    return [sum(matrix[row][column] * vector[column] for column in range(3)) for row in range(3)]


def _transpose(matrix: list[list[float]]) -> list[list[float]]:
    return [[matrix[column][row] for column in range(3)] for row in range(3)]


def _rotation_x(angle: float) -> list[list[float]]:
    cosine, sine = math.cos(angle), math.sin(angle)
    return [[1.0, 0.0, 0.0], [0.0, cosine, -sine], [0.0, sine, cosine]]


def _rotation_y(angle: float) -> list[list[float]]:
    cosine, sine = math.cos(angle), math.sin(angle)
    return [[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]]


def _rotation_z(angle: float) -> list[list[float]]:
    cosine, sine = math.cos(angle), math.sin(angle)
    return [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]]


def _rpy_rotation(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    return _mat_mul(_mat_mul(_rotation_z(yaw), _rotation_y(pitch)), _rotation_x(roll))


def _compose_pose(
    pose: tuple[list[list[float]], list[float]],
    translation: tuple[float, float, float],
    rotation: list[list[float]],
) -> tuple[list[list[float]], list[float]]:
    parent_rotation, parent_position = pose
    translated = _mat_vec(parent_rotation, translation)
    return (
        _mat_mul(parent_rotation, rotation),
        [parent_position[index] + translated[index] for index in range(3)],
    )


def _offline_forward_kinematics(joints: list[float]) -> tuple[list[list[float]], list[float]]:
    """Return the calibrated base_link -> tool0 pose for six UR joints."""

    pose = (_rotation_z(math.pi), [0.0, 0.0, 0.0])
    for joint, (translation, rpy) in zip(joints, _UR3E_CALIBRATED_JOINT_ORIGINS):
        pose = _compose_pose(pose, translation, _rpy_rotation(*rpy))
        pose = _compose_pose(pose, (0.0, 0.0, 0.0), _rotation_z(joint))
    pose = _compose_pose(pose, (0.0, 0.0, 0.0), _rpy_rotation(0.0, -math.pi / 2.0, -math.pi / 2.0))
    return _compose_pose(pose, (0.0, 0.0, 0.0), _rpy_rotation(math.pi / 2.0, 0.0, math.pi / 2.0))


def _rotation_vector(rotation: list[list[float]]) -> list[float]:
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    cosine = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    angle = math.acos(cosine)
    if angle < 1e-8:
        return [
            (rotation[2][1] - rotation[1][2]) / 2.0,
            (rotation[0][2] - rotation[2][0]) / 2.0,
            (rotation[1][0] - rotation[0][1]) / 2.0,
        ]
    if math.pi - angle < 1e-5:
        axis = [
            math.sqrt(max(0.0, (rotation[0][0] + 1.0) / 2.0)),
            math.sqrt(max(0.0, (rotation[1][1] + 1.0) / 2.0)),
            math.sqrt(max(0.0, (rotation[2][2] + 1.0) / 2.0)),
        ]
        if rotation[2][1] - rotation[1][2] < 0.0:
            axis[0] = -axis[0]
        if rotation[0][2] - rotation[2][0] < 0.0:
            axis[1] = -axis[1]
        if rotation[1][0] - rotation[0][1] < 0.0:
            axis[2] = -axis[2]
        norm = math.sqrt(sum(value * value for value in axis)) or 1.0
        return [angle * value / norm for value in axis]
    scale = angle / (2.0 * math.sin(angle))
    return [
        scale * (rotation[2][1] - rotation[1][2]),
        scale * (rotation[0][2] - rotation[2][0]),
        scale * (rotation[1][0] - rotation[0][1]),
    ]


def _pose_error(
    joints: list[float],
    target_position: list[float],
    target_rotation: list[list[float]],
) -> tuple[list[float], float, float]:
    current_rotation, current_position = _offline_forward_kinematics(joints)
    position_error = [target_position[index] - current_position[index] for index in range(3)]
    orientation_error = _rotation_vector(_mat_mul(target_rotation, _transpose(current_rotation)))
    return (
        position_error + orientation_error,
        math.sqrt(sum(value * value for value in position_error)),
        math.sqrt(sum(value * value for value in orientation_error)),
    )


def _solve_linear(matrix: list[list[float]], vector: list[float]) -> list[float] | None:
    augmented = [row[:] + [vector[index]] for index, row in enumerate(matrix)]
    size = len(vector)
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-12:
            return None
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        augmented[column] = [value / divisor for value in augmented[column]]
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            if factor:
                augmented[row] = [
                    augmented[row][index] - factor * augmented[column][index]
                    for index in range(size + 1)
                ]
    return [augmented[row][-1] for row in range(size)]


def _nearest_joint_equivalent(value: float, reference: float) -> float:
    while value - reference > math.pi:
        value -= math.tau
    while value - reference < -math.pi:
        value += math.tau
    return max(-math.tau, min(math.tau, value))


def _solve_offline_ik(
    target_position: list[float],
    target_rotation: list[list[float]],
    seed: list[float],
) -> tuple[list[float], float, float]:
    joints = seed[:]
    best = (joints[:], float("inf"), float("inf"))
    epsilon = 1e-5
    for _ in range(85):
        error, position_error, orientation_error = _pose_error(
            joints, target_position, target_rotation
        )
        if position_error + orientation_error < best[1] + best[2]:
            best = (joints[:], position_error, orientation_error)
        if position_error <= 0.0008 and orientation_error <= 0.012:
            return joints, position_error, orientation_error

        current_rotation, current_position = _offline_forward_kinematics(joints)
        jacobian = [[0.0 for _ in range(6)] for _ in range(6)]
        for column in range(6):
            shifted = joints[:]
            shifted[column] += epsilon
            shifted_rotation, shifted_position = _offline_forward_kinematics(shifted)
            orientation_delta = _rotation_vector(
                _mat_mul(shifted_rotation, _transpose(current_rotation))
            )
            for row in range(3):
                jacobian[row][column] = (
                    shifted_position[row] - current_position[row]
                ) / epsilon
                jacobian[row + 3][column] = orientation_delta[row] / epsilon

        damping = 0.025 if position_error > 0.01 else 0.008
        normal = [
            [
                sum(jacobian[row][joint] * jacobian[column][joint] for joint in range(6))
                + (damping * damping if row == column else 0.0)
                for column in range(6)
            ]
            for row in range(6)
        ]
        intermediate = _solve_linear(normal, error)
        if intermediate is None:
            break
        delta = [
            sum(jacobian[row][column] * intermediate[row] for row in range(6))
            for column in range(6)
        ]
        maximum_delta = max(abs(value) for value in delta) or 1.0
        scale = min(1.0, 0.22 / maximum_delta)
        joints = [
            _nearest_joint_equivalent(joints[index] + delta[index] * scale, seed[index])
            for index in range(6)
        ]
    return best


def _offline_path_points(
    review: dict[str, Any],
) -> tuple[list[dict[str, float]], int, bool]:
    segments = review.get("path", {}).get("segments", [])
    if not segments:
        return [], 0, False
    segment_data = []
    previous_time = 0.0
    for segment in segments:
        start = [
            float(segment.get(f"tool0_{axis}0", segment[f"{axis}0"]))
            for axis in _AXES
        ]
        end = [
            float(segment.get(f"tool0_{axis}1", segment[f"{axis}1"]))
            for axis in _AXES
        ]
        distance = math.sqrt(
            sum((end[index] - start[index]) ** 2 for index in range(3))
        )
        segment_end_time = float(segment.get("cumulative_s", previous_time))
        desired = max(1, math.ceil(distance / OFFLINE_CARTESIAN_CHECK_STEP_M))
        segment_data.append((start, end, previous_time, segment_end_time, desired))
        previous_time = segment_end_time
    dense_source_count = 1 + sum(item[4] for item in segment_data)

    # Always retain every displayed corner when it fits.  Remaining capacity
    # is assigned to the longest Cartesian segments for interior checks.
    if len(segment_data) + 1 <= MAX_OFFLINE_VALIDATION_POINTS:
        subdivisions = [1 for _ in segment_data]
        extra_budget = MAX_OFFLINE_VALIDATION_POINTS - len(segment_data) - 1
        desired_extras = [item[4] - 1 for item in segment_data]
        heap = [
            (-float(value), index)
            for index, value in enumerate(desired_extras)
            if value > 0
        ]
        heapq.heapify(heap)
        while extra_budget > 0 and heap:
            _priority, index = heapq.heappop(heap)
            subdivisions[index] += 1
            desired_extras[index] -= 1
            extra_budget -= 1
            if desired_extras[index] > 0:
                heapq.heappush(
                    heap,
                    (
                        -desired_extras[index] / subdivisions[index],
                        index,
                    ),
                )
        selected_segment_data = segment_data
    else:
        indexes = _sample_indexes(
            len(segment_data) + 1, MAX_OFFLINE_VALIDATION_POINTS
        )
        endpoint_indexes = indexes[1:]
        selected_segment_data = [segment_data[index - 1] for index in endpoint_indexes]
        subdivisions = [1 for _ in selected_segment_data]

    points = [{
        "x": float(segments[0].get("tool0_x0", segments[0]["x0"])),
        "y": float(segments[0].get("tool0_y0", segments[0]["y0"])),
        "z": float(segments[0].get("tool0_z0", segments[0]["z0"])),
        "time_s": 0.0,
    }]
    for item, subdivision_count in zip(selected_segment_data, subdivisions):
        start, end, segment_start_time, segment_end_time, _desired = item
        for subdivision in range(1, subdivision_count + 1):
            fraction = subdivision / subdivision_count
            points.append(
                {
                    "x": start[0] + (end[0] - start[0]) * fraction,
                    "y": start[1] + (end[1] - start[1]) * fraction,
                    "z": start[2] + (end[2] - start[2]) * fraction,
                    "time_s": segment_start_time
                    + (segment_end_time - segment_start_time) * fraction,
                }
            )
    return points, dense_source_count, len(points) < dense_source_count


def build_offline_robot_simulation(
    review: dict[str, Any],
    *,
    maximum_points: int = MAX_OFFLINE_SIMULATION_POINTS,
    progress_callback: Callable[[int, int], None] | None = None,
) -> dict[str, Any]:
    """Estimate a read-only UR3e joint animation without ROS or hardware.

    This deliberately does not claim MoveIt or collision validation.  It is a
    local numeric inverse-kinematics estimate used only by the 3D review.
    """

    points, dense_source_count, validation_sampled = _offline_path_points(review)
    total_points = len(points)
    if progress_callback is not None:
        progress_callback(0, total_points)
    if len(points) < 2:
        return {
            "status": "failed",
            "simulation": True,
            "offline_estimate": True,
            "execution_locked": True,
            "review_sha256": review.get("review_sha256", ""),
            "joint_names": _UR_JOINT_NAMES,
            "trajectory": [],
            "message": "The calibrated path has too few points for offline robot simulation.",
        }

    orientation = review["calibration"]["coordinate_transform"].get(
        "tool_orientation_rpy_deg", {"roll": 180.0, "pitch": 0.0, "yaw": 0.0}
    )
    target_rotation = _rpy_rotation(
        math.radians(float(orientation["roll"])),
        math.radians(float(orientation["pitch"])),
        math.radians(float(orientation["yaw"])),
    )
    seeds = [
        [math.pi, -1.35, 1.55, -1.75, -math.pi / 2.0, 0.0],
        [math.pi / 2.0, -1.25, 1.65, -1.95, -math.pi / 2.0, 0.0],
        [-math.pi / 2.0, -1.25, 1.65, -1.95, -math.pi / 2.0, 0.0],
        [0.0, -1.35, 1.65, -1.85, -math.pi / 2.0, 0.0],
        [math.pi, -1.35, 1.55, -1.75, math.pi / 2.0, math.pi],
    ]
    first_point = points[0]
    first_target = [first_point[axis] for axis in _AXES]
    candidates = [
        _solve_offline_ik(first_target, target_rotation, seed)
        for seed in seeds
    ]
    joints, first_position_error, first_orientation_error = min(
        candidates, key=lambda candidate: candidate[1] + candidate[2]
    )
    if (
        first_position_error > OFFLINE_POSITION_TOLERANCE_M
        or first_orientation_error > OFFLINE_ORIENTATION_TOLERANCE_RAD
    ):
        return {
            "status": "failed",
            "simulation": True,
            "offline_estimate": True,
            "execution_locked": True,
            "review_sha256": review["review_sha256"],
            "joint_names": _UR_JOINT_NAMES,
            "trajectory": [],
            "maximum_tcp_error_m": _round(first_position_error, 6),
            "message": "Offline IK could not place the UR3e tool at the calibrated path start. Review Calibration before playing the estimate.",
        }

    validated_trajectory = []
    maximum_position_error = first_position_error
    maximum_orientation_error = first_orientation_error
    maximum_joint_step = 0.0
    maximum_nominal_joint_speed = 0.0
    previous = joints[:]
    previous_time_s = float(points[0].get("time_s", 0.0))
    total_time = max(float(points[-1].get("time_s", 0.0)), 1.0)
    progress_interval = max(1, len(points) // 100)
    for source_index, point in enumerate(points):
        target = [point[axis] for axis in _AXES]
        solved, position_error, orientation_error = _solve_offline_ik(
            target, target_rotation, previous
        )
        solved = [
            _nearest_joint_equivalent(solved[index], previous[index])
            for index in range(6)
        ]
        joint_step = max(abs(solved[index] - previous[index]) for index in range(6))
        point_time_s = float(point.get("time_s", 0.0))
        delta_time_s = point_time_s - previous_time_s
        if source_index > 0 and delta_time_s > 1e-9:
            maximum_nominal_joint_speed = max(
                maximum_nominal_joint_speed, joint_step / delta_time_s
            )
        maximum_joint_step = max(maximum_joint_step, joint_step)
        maximum_position_error = max(maximum_position_error, position_error)
        maximum_orientation_error = max(maximum_orientation_error, orientation_error)
        if (
            position_error > OFFLINE_POSITION_TOLERANCE_M
            or orientation_error > OFFLINE_ORIENTATION_TOLERANCE_RAD
        ):
            return {
                "status": "failed",
                "simulation": True,
                "offline_estimate": True,
                "execution_locked": True,
                "review_sha256": review["review_sha256"],
                "joint_names": _UR_JOINT_NAMES,
                "trajectory": [],
                "failed_waypoint_index": source_index,
                "failed_progress": _round(source_index / (len(points) - 1), 7),
                "maximum_tcp_error_m": _round(maximum_position_error, 6),
                "maximum_orientation_error_rad": _round(
                    maximum_orientation_error, 6
                ),
                "message": "Offline IK could not reproduce part of the calibrated path within the 1.5 mm and 0.02 rad estimate tolerances. Review the origin and tool orientation.",
            }
        if source_index > 0 and joint_step > OFFLINE_MAX_JOINT_STEP_RAD:
            return {
                "status": "failed",
                "simulation": True,
                "offline_estimate": True,
                "execution_locked": True,
                "review_sha256": review["review_sha256"],
                "joint_names": _UR_JOINT_NAMES,
                "trajectory": [],
                "failed_waypoint_index": source_index,
                "failed_progress": _round(source_index / (len(points) - 1), 7),
                "max_joint_step_rad": _round(maximum_joint_step, 6),
                "message": "Offline IK found a discontinuous UR3e joint branch. Change the calibrated origin or tool orientation before planning.",
            }
        progress = source_index / (len(points) - 1)
        validated_trajectory.append(
            {
                "progress": _round(progress, 7),
                "time_s": _round(float(point.get("time_s", 0.0)), 4),
                "positions": [_round(value, 7) for value in solved],
            }
        )
        previous = solved
        previous_time_s = point_time_s
        completed_points = source_index + 1
        if (
            progress_callback is not None
            and (
                completed_points == len(points)
                or completed_points % progress_interval == 0
            )
        ):
            progress_callback(completed_points, len(points))

    output_indexes = _sample_indexes(
        len(validated_trajectory), max(2, maximum_points)
    )
    trajectory = [validated_trajectory[index] for index in output_indexes]
    source_simplification_mm = float(
        review.get("path", {}).get("maximum_simplification_deviation_mm", 0.0)
    )

    return {
        "status": "passed",
        "simulation": True,
        "offline_estimate": True,
        "execution_locked": True,
        "source": "offline_numeric_ik",
        "review_sha256": review["review_sha256"],
        "planning_group": "ur_manipulator",
        "end_effector_link": "tool0",
        "reference_frame": "base_link",
        "collision_free": None,
        "joint_limits": "not_validated",
        "planning_scene": "not_loaded",
        "source_waypoint_count": len(review.get("path", {}).get("segments", [])) + 1,
        "dense_check_waypoint_count": dense_source_count,
        "validated_waypoint_count": len(points),
        "validation_sampled": validation_sampled,
        "planned_point_count": len(trajectory),
        "sampled": len(trajectory) < dense_source_count,
        "source_geometry_simplified": bool(
            review.get("path", {}).get("sampled", False)
        ),
        "maximum_source_simplification_deviation_m": _round(
            source_simplification_mm / 1000.0, 6
        ),
        "joint_continuity": (
            "passed_at_checked_resolution" if validation_sampled else "passed"
        ),
        "duration_s": _round(total_time, 4),
        "maximum_tcp_error_m": _round(maximum_position_error, 6),
        "maximum_orientation_error_rad": _round(maximum_orientation_error, 6),
        "max_joint_step_rad": _round(maximum_joint_step, 6),
        "maximum_nominal_joint_speed_rad_s": _round(
            maximum_nominal_joint_speed, 6
        ),
        "time_parameterization": "not_performed",
        "nominal_gcode_timing_only": True,
        "requires_robot_retiming": True,
        "full_source_geometry_validated": not bool(
            review.get("path", {}).get("sampled", False)
        ) and not validation_sampled,
        "geometry_fidelity": (
            "simplified_preview_only"
            if review.get("path", {}).get("sampled", False)
            else "full_displayed_geometry"
        ),
        "joint_names": _UR_JOINT_NAMES,
        "trajectory": trajectory,
        "message": (
            "Offline UR3e inverse-kinematics animation ready. Robot timing has "
            "not been parameterized, and no ROS service, robot, controller, "
            "MoveIt planning scene, or collision checker was contacted."
        ),
    }


def _waypoint_distance_to_chord(
    point: dict[str, Any],
    start: dict[str, Any],
    end: dict[str, Any],
) -> float:
    start_xyz = [float(start[axis]) for axis in _AXES]
    end_xyz = [float(end[axis]) for axis in _AXES]
    point_xyz = [float(point[axis]) for axis in _AXES]
    delta = [end_xyz[index] - start_xyz[index] for index in range(3)]
    length_squared = sum(value * value for value in delta)
    if length_squared <= 1e-20:
        return math.sqrt(
            sum(
                (point_xyz[index] - start_xyz[index]) ** 2
                for index in range(3)
            )
        )
    fraction = max(
        0.0,
        min(
            1.0,
            sum(
                (point_xyz[index] - start_xyz[index]) * delta[index]
                for index in range(3)
            )
            / length_squared,
        ),
    )
    return math.sqrt(
        sum(
            (
                point_xyz[index]
                - (start_xyz[index] + fraction * delta[index])
            )
            ** 2
            for index in range(3)
        )
    )


def _geometric_waypoint_indexes(
    points: list[dict[str, Any]], maximum: int
) -> tuple[list[int], float]:
    """Select important Cartesian corners and report the resulting error."""

    if len(points) <= maximum:
        return list(range(len(points))), 0.0

    def candidate(
        start_index: int, end_index: int
    ) -> tuple[float, int, int, int] | None:
        if end_index - start_index <= 1:
            return None
        maximum_distance = -1.0
        maximum_index = -1
        for index in range(start_index + 1, end_index):
            distance = _waypoint_distance_to_chord(
                points[index], points[start_index], points[end_index]
            )
            if distance > maximum_distance:
                maximum_distance = distance
                maximum_index = index
        if maximum_index < 0:
            return None
        return (-maximum_distance, start_index, end_index, maximum_index)

    keep = {0, len(points) - 1}
    heap: list[tuple[float, int, int, int]] = []
    first = candidate(0, len(points) - 1)
    if first:
        heapq.heappush(heap, first)
    while heap and len(keep) < maximum:
        _negative_distance, start_index, end_index, split_index = heapq.heappop(
            heap
        )
        keep.add(split_index)
        left = candidate(start_index, split_index)
        right = candidate(split_index, end_index)
        if left:
            heapq.heappush(heap, left)
        if right:
            heapq.heappush(heap, right)
    indexes = sorted(keep)
    maximum_deviation = 0.0
    for start_index, end_index in zip(indexes, indexes[1:]):
        for index in range(start_index + 1, end_index):
            maximum_deviation = max(
                maximum_deviation,
                _waypoint_distance_to_chord(
                    points[index], points[start_index], points[end_index]
                ),
            )
    return indexes, maximum_deviation


def write_moveit_path(
    path: Path,
    review: dict[str, Any],
    *,
    maximum_waypoints: int = MAX_MOVEIT_WAYPOINTS,
) -> dict[str, Any]:
    """Write a bounded CSV consumed by the ROS plan-only validator."""

    segments = review.get("path", {}).get("segments", [])
    if not isinstance(segments, list) or not segments:
        raise PathReviewError("The 3D review contains no Cartesian path.")
    points = [
        {
            "sequence": 0,
            "line": int(segments[0].get("line", 0)),
            "x": float(segments[0].get("tool0_x0", segments[0]["x0"])),
            "y": float(segments[0].get("tool0_y0", segments[0]["y0"])),
            "z": float(segments[0].get("tool0_z0", segments[0]["z0"])),
            "kind": "approach",
            "cumulative_s": 0.0,
        }
    ]
    points.extend(
        {
            "sequence": int(segment.get("sequence", index + 1)),
            "line": int(segment.get("line", 0)),
            "x": float(segment.get("tool0_x1", segment["x1"])),
            "y": float(segment.get("tool0_y1", segment["y1"])),
            "z": float(segment.get("tool0_z1", segment["z1"])),
            "kind": "print" if segment.get("kind") == "print" else "travel",
            "cumulative_s": float(segment.get("cumulative_s", 0.0)),
        }
        for index, segment in enumerate(segments)
    )
    source_preview_deviation_m = float(
        review.get("path", {}).get("maximum_simplification_deviation_mm", 0.0)
    ) / 1000.0
    if source_preview_deviation_m > MAX_MOVEIT_CARTESIAN_SIMPLIFICATION_ERROR_M:
        raise PathReviewError(
            "This file is too geometrically detailed for one trustworthy MoveIt "
            "request: the bounded 3D preview differs from the full parsed path by "
            f"up to {source_preview_deviation_m * 1000.0:.3f} mm. "
            "Use full-resolution chunked validation before execution."
        )
    indexes, sampling_deviation_m = _geometric_waypoint_indexes(
        points, maximum_waypoints
    )
    if sampling_deviation_m > MAX_MOVEIT_CARTESIAN_SIMPLIFICATION_ERROR_M:
        raise PathReviewError(
            "This path cannot be reduced to one MoveIt request within the 1 mm "
            "Cartesian fidelity limit. Use full-resolution chunked validation "
            "before execution."
        )
    sampled_points = [points[index] for index in indexes]
    rows = ["sequence,line,x,y,z,kind,cumulative_s"]
    rows.extend(
        (
            f"{point['sequence']},{point['line']},"
            f"{point['x']:.9f},{point['y']:.9f},{point['z']:.9f},"
            f"{point['kind']},{point['cumulative_s']:.6f}"
        )
        for point in sampled_points
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")
    return {
        "source_waypoint_count": len(points),
        "written_waypoint_count": len(sampled_points),
        "sampled": len(sampled_points) < len(points),
        "maximum_simplification_deviation_m": _round(
            max(source_preview_deviation_m, sampling_deviation_m), 7
        ),
    }


def demo_moveit_result(review: dict[str, Any]) -> dict[str, Any]:
    """Return an unmistakably simulated trajectory for demo-mode UI testing."""

    joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    base = [0.0, -1.35, 1.55, -1.75, -1.57, 0.0]
    trajectory = []
    point_count = 120
    for index in range(point_count):
        progress = index / (point_count - 1)
        phase = progress * math.tau
        trajectory.append(
            {
                "progress": _round(progress, 6),
                "time_s": _round(progress * 12.0, 4),
                "positions": [
                    _round(base[0] + 0.22 * math.sin(phase), 6),
                    _round(base[1] + 0.12 * math.sin(phase * 0.8), 6),
                    _round(base[2] + 0.16 * math.sin(phase * 1.1), 6),
                    _round(base[3] + 0.10 * math.cos(phase), 6),
                    _round(base[4] + 0.08 * math.sin(phase * 0.7), 6),
                    _round(base[5] + 0.18 * progress, 6),
                ],
            }
        )
    return {
        "status": "passed",
        "simulation": True,
        "planning_policy_version": PLANNING_POLICY_VERSION,
        "interpolation_validation": "passed",
        "printhead_geometry_sha256": review.get("printhead_geometry_sha256", printhead_geometry_sha256()),
        "execution_locked": True,
        "review_sha256": review["review_sha256"],
        "planning_group": "ur_manipulator",
        "end_effector_link": "tool0",
        "reference_frame": "base_link",
        "failure_stage": "",
        "approach_status": "passed",
        "approach_distance_m": 0.32,
        "cartesian_fraction": 1.0,
        "collision_free": True,
        "joint_limits": "passed",
        "max_joint_step_rad": 0.012,
        "duration_s": 12.0,
        "source_waypoint_count": review["path"]["returned_segment_count"] + 1,
        "planned_point_count": point_count,
        "sampled": review["path"]["sampled"],
        "joint_names": joint_names,
        "trajectory": trajectory,
        "message": (
            "Demo-only MoveIt result. No ROS service, robot, or controller was contacted."
        ),
    }


def read_moveit_result(path: Path, expected_review_sha256: str) -> dict[str, Any]:
    """Read and minimally validate the plan-only result produced inside ROS."""

    if not path.exists():
        raise PathReviewError("MoveIt did not produce a path-review result.")
    if path.stat().st_size > MAX_MOVEIT_RESULT_BYTES:
        raise PathReviewError("The MoveIt path-review result is too large.")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise PathReviewError(f"Could not read the MoveIt result: {exc}") from exc
    if not isinstance(payload, dict):
        raise PathReviewError("The MoveIt result must be a JSON object.")
    if payload.get("review_sha256") != expected_review_sha256:
        raise PathReviewError(
            "The MoveIt result belongs to a different path-review request."
        )
    if payload.get("planning_policy_version") != PLANNING_POLICY_VERSION:
        raise PathReviewError(
            "This MoveIt result uses an obsolete planning policy. Validate the path again."
        )
    if payload.get("printhead_geometry_sha256") != printhead_geometry_sha256():
        raise PathReviewError("The installed printhead geometry changed. Validate the path again.")
    status = payload.get("status")
    if status not in {"passed", "failed"}:
        raise PathReviewError("The MoveIt result has an invalid status.")
    if status == "passed" and payload.get("interpolation_validation") != "passed":
        raise PathReviewError(
            "The MoveIt result has not passed final joint-interpolation validation. Validate again."
        )
    joint_names = payload.get("joint_names")
    trajectory = payload.get("trajectory")
    if not isinstance(joint_names, list):
        raise PathReviewError("The MoveIt result has invalid joint names.")
    if not isinstance(trajectory, list) or len(trajectory) > 1_500:
        raise PathReviewError("The MoveIt trajectory is missing or too large.")
    if status == "passed" and len(joint_names) != 6:
        raise PathReviewError("The passed MoveIt result is missing the six UR joints.")
    if status == "passed" and not trajectory:
        raise PathReviewError("The passed MoveIt result is missing its trajectory.")
    if status == "failed" and len(joint_names) not in {0, 6}:
        raise PathReviewError("The failed MoveIt result has invalid joint names.")
    for point in trajectory:
        if not isinstance(point, dict):
            raise PathReviewError("A MoveIt trajectory point is invalid.")
        positions = point.get("positions")
        if not isinstance(positions, list) or len(positions) != len(joint_names):
            raise PathReviewError("A MoveIt trajectory point has invalid joints.")
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            for value in positions
        ):
            raise PathReviewError("A MoveIt trajectory point is not finite.")
    return payload


def planning_request_path(directory: Path, review_sha256: str) -> Path:
    if not re.fullmatch(r"[a-f0-9]{64}", review_sha256):
        raise PathReviewError("The planning request checksum is invalid.")
    return directory / f"{review_sha256}.request.json"


def begin_planning_request(directory: Path, review_sha256: str, review_id: str) -> Path:
    """Revoke older plans before starting a new attempt, including failed attempts."""

    if not re.fullmatch(r"[a-f0-9]{12}", review_id):
        raise PathReviewError("The planning request identifier is invalid.")
    destination = planning_request_path(directory, review_sha256)
    destination.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "planning_policy_version": PLANNING_POLICY_VERSION,
        "review_sha256": review_sha256,
        "review_id": review_id,
    }
    temporary = destination.with_suffix(f".{review_id}.tmp")
    temporary.write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(destination)
    return destination


def current_planning_request(directory: Path, review_sha256: str) -> dict[str, Any] | None:
    path = planning_request_path(directory, review_sha256)
    if not path.exists():
        return None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise PathReviewError("The current planning request cannot be verified. Validate again.") from exc
    if (
        not isinstance(payload, dict)
        or payload.get("review_sha256") != review_sha256
        or payload.get("planning_policy_version") != PLANNING_POLICY_VERSION
        or not re.fullmatch(r"[a-f0-9]{12}", str(payload.get("review_id", "")))
    ):
        raise PathReviewError("The current planning request is invalid. Validate again.")
    return payload


def find_cached_moveit_result(
    directory: Path,
    expected_review_sha256: str,
) -> dict[str, Any] | None:
    """Return the newest passed six-axis plan for this exact calibrated review."""

    try:
        # Current application requests are selected by identity, not mtime: an
        # older process finishing late must not supersede the latest attempt.
        request = (
            current_planning_request(directory, expected_review_sha256)
            if re.fullmatch(r"[a-f0-9]{64}", expected_review_sha256)
            else None
        )
        if request is not None:
            result = read_moveit_result(
                directory / f"{request['review_id']}.json", expected_review_sha256
            )
            return {**result, "cached": True} if result["status"] == "passed" else None
        candidates = sorted(
            (path for path in directory.glob("*.json") if path.is_file() and not path.name.endswith(".request.json")),
            key=lambda path: path.stat().st_mtime_ns,
            reverse=True,
        )
    except (OSError, PathReviewError):
        return None

    for path in candidates:
        try:
            result = read_moveit_result(path, expected_review_sha256)
        except (OSError, PathReviewError):
            continue
        if result["status"] != "passed":
            return None
        return {
            **result,
            "cached": True,
        }
    return None
