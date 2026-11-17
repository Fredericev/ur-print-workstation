#!/usr/bin/env python3
"""Replay a checksum-locked MoveIt review in RViz without control interfaces.

This process only publishes ``moveit_msgs/DisplayTrajectory``.  It never
creates an action/service client, publishes joint states, contacts a robot IP,
or writes to Moonraker.  The cached approach starts at the posture recorded
when that review was created; it is not a replacement for a fresh online plan.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from typing import NamedTuple

sys.path.insert(0, str(Path(__file__).resolve().parent))
from smooth_trajectory import controls, validate_bundle
from execution_scope import validate_execution_scope


UR_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
MAXIMUM_DISPLAY_SPEED_FACTOR = 100.0


class PlaybackError(ValueError):
    """A cached review cannot be represented as a trusted display-only replay."""


class PlaybackPoint(NamedTuple):
    time_s: float
    path_progress: float
    positions: tuple[float, ...]


class PlaybackTrack(NamedTuple):
    name: str
    points: tuple[PlaybackPoint, ...]
    source_duration_s: float


class Playback(NamedTuple):
    manifest_path: Path
    review_id: str
    source_name: str
    source_path: Path
    speed_factor: float
    joint_names: tuple[str, ...]
    tracks: tuple[PlaybackTrack, ...]
    timing_source: str = "cached_moveit_csv"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _finite(value: object, label: str) -> float:
    if isinstance(value, bool):
        raise PlaybackError(f"{label} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise PlaybackError(f"{label} must be a finite number") from exc
    if not math.isfinite(result):
        raise PlaybackError(f"{label} must be a finite number")
    return result


def _resolve_manifest(manifest: str | Path, workspace_root: Path) -> Path:
    if str(manifest) != "latest":
        path = Path(manifest).expanduser().resolve()
        if not path.is_file():
            raise PlaybackError(f"Execution manifest was not found: {path}")
        return path

    candidates = []
    for path in (workspace_root / "runtime" / "print_execution").glob("*.json"):
        if path.name.endswith(".status.json"):
            continue
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        if value.get("planning_request", {}).get("review_id"):
            candidates.append(path)
    if not candidates:
        raise PlaybackError("No cached execution manifest has a bound MoveIt review")
    return max(candidates, key=lambda path: path.stat().st_mtime_ns)


def _read_track(
    path: Path,
    *,
    name: str,
    joint_names: tuple[str, ...],
    speed_factor: float,
) -> PlaybackTrack:
    try:
        with path.open(encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream))
    except (OSError, csv.Error) as exc:
        raise PlaybackError(f"Could not read cached {name} trajectory: {path}") from exc
    expected_fields = ("time_s", "path_progress", *joint_names)
    if not rows or tuple(rows[0]) != expected_fields:
        raise PlaybackError(f"Cached {name} trajectory has an invalid joint/header layout")

    points = []
    previous_time = -1.0
    previous_progress = -1.0
    for index, row in enumerate(rows):
        source_time = _finite(row["time_s"], f"{name} time")
        progress = _finite(row["path_progress"], f"{name} progress")
        positions = tuple(_finite(row[joint], f"{name} joint") for joint in joint_names)
        if index == 0 and abs(source_time) > 1e-9:
            raise PlaybackError(f"Cached {name} trajectory must begin at time zero")
        if index and source_time <= previous_time:
            raise PlaybackError(f"Cached {name} trajectory times are not strictly increasing")
        if not -1e-9 <= progress <= 1.0 + 1e-9 or progress + 1e-9 < previous_progress:
            raise PlaybackError(f"Cached {name} trajectory progress is invalid")
        points.append(PlaybackPoint(source_time / speed_factor, progress, positions))
        previous_time = source_time
        previous_progress = progress
    if len(points) < 2:
        raise PlaybackError(f"Cached {name} trajectory needs at least two points")
    return PlaybackTrack(name, tuple(points), previous_time)


def _continuous_tracks(package: dict, *, speed_factor: float) -> tuple[PlaybackTrack, ...]:
    """Display the exact frozen v3 curve, not the older position-only CSV."""
    try:
        identity = {key: value for key, value in package.items() if key != "execution_id"}
        digest = hashlib.sha256(json.dumps(
            identity, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
            allow_nan=False,
        ).encode("utf-8")).hexdigest()[:16]
        if package.get("execution_id") != digest:
            raise ValueError("Execution profile identity changed after preparation")
        bundle = validate_bundle(package)
        validate_execution_scope(package)
    except (ValueError, KeyError, TypeError, IndexError) as exc:
        raise PlaybackError(f"Continuous playback package is invalid: {exc}") from exc

    tracks = []
    for name in ("approach", "print"):
        profile = bundle[name]
        first = profile["points"][0]
        points = [PlaybackPoint(0.0, float(first.get("path_progress", 0.0)), tuple(first["positions"]))]
        for a, b in zip(profile["points"], profile["points"][1:]):
            curves = [controls(a, b, joint) for joint in range(6)]
            bound = max(5*abs(y-x) for curve in curves for x, y in zip(curve, curve[1:]))
            divisions = max(2, math.ceil(bound / .002))
            for sample in range(1, divisions + 1):
                u = sample / divisions
                positions = []
                for curve in curves:
                    work = list(curve)
                    while len(work) > 1:
                        work = [(1-u)*x + u*y for x, y in zip(work, work[1:])]
                    positions.append(work[0])
                timestamp = float(a["time_s"]) + u*(float(b["time_s"])-float(a["time_s"]))
                progress = float(a.get("path_progress", 0.0)) + u*(
                    float(b.get("path_progress", 1.0))-float(a.get("path_progress", 0.0))
                )
                points.append(PlaybackPoint(timestamp/speed_factor, progress, tuple(positions)))
        tracks.append(PlaybackTrack(name, tuple(points), float(profile["duration_s"])))
    return tuple(tracks)


def load_playback(
    manifest: str | Path = "latest",
    *,
    workspace_root: Path | None = None,
    project_root: Path | None = None,
    speed_factor: float = 10.0,
) -> Playback:
    """Load and independently verify a cached approach plus complete print path."""
    workspace_root = (
        Path(__file__).resolve().parents[1]
        if workspace_root is None
        else Path(workspace_root).resolve()
    )
    project_root = (
        workspace_root.parents[1]
        if project_root is None
        else Path(project_root).resolve()
    )
    speed_factor = _finite(speed_factor, "Display speed factor")
    if not 0.0 < speed_factor <= MAXIMUM_DISPLAY_SPEED_FACTOR:
        raise PlaybackError(
            f"Display speed factor must be above zero and at most {MAXIMUM_DISPLAY_SPEED_FACTOR:g}"
        )

    manifest_path = _resolve_manifest(manifest, workspace_root)
    try:
        package = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise PlaybackError(f"Execution manifest is unreadable: {manifest_path}") from exc
    request = package.get("planning_request", {})
    review_id = str(request.get("review_id", ""))
    if not review_id or not review_id.replace("-", "").isalnum():
        raise PlaybackError("Execution manifest has no valid MoveIt review ID")

    review_prefix = workspace_root / "runtime" / "path_review" / review_id
    try:
        review = json.loads(review_prefix.with_suffix(".json").read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise PlaybackError(f"MoveIt review is unreadable: {review_id}") from exc
    validated = (
        review.get("status") == "passed"
        and review.get("simulation") is False
        and review.get("approach_status") == "passed"
        and review.get("interpolation_validation") == "passed"
        and review.get("collision_free") is True
        and review.get("joint_limits") == "passed"
        and _finite(review.get("cartesian_fraction", 0.0), "Cartesian fraction")
        >= 1.0 - 1e-6
        and review.get("reference_frame") == "base_link"
    )
    if not validated:
        raise PlaybackError("Cached MoveIt path is not a fully validated approach and print review")
    if (
        review.get("review_sha256") != package.get("review_sha256")
        or review.get("planning_policy_version") != package.get("planning_policy_version")
    ):
        raise PlaybackError("Execution manifest and MoveIt review do not belong together")

    joint_names = tuple(review.get("joint_names", ()))
    if joint_names != UR_JOINTS:
        raise PlaybackError("MoveIt review does not contain the expected six UR3e joints")

    source = package.get("source", {})
    source_name = str(source.get("name", ""))
    source_path = project_root / "Test gcode" / source_name
    expected_source_sha = str(package.get("source_sha256") or source.get("sha256") or "")
    if not source_name or not source_path.is_file() or _sha256(source_path) != expected_source_sha:
        raise PlaybackError("The original G-code source is missing or its checksum changed")

    tracks = []
    for name, suffix, metadata_key in (
        ("approach", ".approach.csv", "approach_trajectory"),
        ("print", ".print.csv", "print_trajectory"),
    ):
        path = Path(str(review_prefix) + suffix)
        metadata = package.get(metadata_key, {})
        if not path.is_file() or _sha256(path) != metadata.get("sha256"):
            raise PlaybackError(f"Cached {name} trajectory checksum does not match its manifest")
        track = _read_track(
            path,
            name=name,
            joint_names=joint_names,
            speed_factor=speed_factor,
        )
        if len(track.points) != int(metadata.get("point_count", -1)):
            raise PlaybackError(f"Cached {name} trajectory point count changed")
        if abs(track.source_duration_s - _finite(metadata.get("duration_s"), f"{name} duration")) > 1e-6:
            raise PlaybackError(f"Cached {name} trajectory duration changed")
        if tuple(metadata.get("joint_names", ())) != joint_names:
            raise PlaybackError(f"Cached {name} trajectory joint order changed")
        tracks.append(track)

    if max(
        abs(left - right)
        for left, right in zip(tracks[0].points[-1].positions, tracks[1].points[0].positions)
    ) > 1e-6:
        raise PlaybackError("Cached approach and complete print trajectory are not continuous")
    interpolation_limit = _finite(
        review.get("interpolation_max_joint_spacing_rad"),
        "Interpolation spacing",
    )
    for track in tracks:
        maximum_step = max(
            max(abs(right - left) for left, right in zip(first.positions, second.positions))
            for first, second in zip(track.points, track.points[1:])
        )
        if maximum_step > interpolation_limit + 1e-9:
            raise PlaybackError(f"Cached {track.name} trajectory exceeds its validated interpolation spacing")
    if len(tracks[1].points) != int(review.get("planned_point_count", -1)):
        raise PlaybackError("Cached print trajectory is not the complete MoveIt path")
    if abs(tracks[1].source_duration_s - _finite(review.get("duration_s"), "Print duration")) > 1e-6:
        raise PlaybackError("Cached complete print duration differs from the MoveIt review")

    timing_source = "cached_moveit_csv"
    if package.get("version") == 3 and package.get("execution_extent") == "full":
        continuous_tracks = _continuous_tracks(package, speed_factor=speed_factor)
        for original, frozen in zip(tracks, continuous_tracks):
            for original_point, frozen_point in (
                (original.points[0], frozen.points[0]),
                (original.points[-1], frozen.points[-1]),
            ):
                if max(abs(a-b) for a, b in zip(original_point.positions, frozen_point.positions)) > 1e-6:
                    raise PlaybackError("Continuous profile endpoints differ from the bound MoveIt path")
        tracks = continuous_tracks
        timing_source = "frozen_continuous_v3_profile"

    return Playback(
        manifest_path=manifest_path,
        review_id=review_id,
        source_name=source_name,
        source_path=source_path,
        speed_factor=speed_factor,
        joint_names=joint_names,
        tracks=tuple(tracks),
        timing_source=timing_source,
    )


def playback_summary(playback: Playback) -> dict[str, object]:
    return {
        "display_only": True,
        "hardware_contacted": False,
        "executable": False,
        "scope": "cached_moveit_approach_plus_complete_print",
        "timing_source": playback.timing_source,
        "source": playback.source_name,
        "source_path": str(playback.source_path),
        "manifest": str(playback.manifest_path),
        "review_id": playback.review_id,
        "speed_factor": playback.speed_factor,
        "source_duration_s": round(sum(track.source_duration_s for track in playback.tracks), 6),
        "display_duration_s": round(
            sum(track.points[-1].time_s for track in playback.tracks), 6
        ),
        "tracks": [
            {
                "name": track.name,
                "points": len(track.points),
                "source_duration_s": round(track.source_duration_s, 6),
                "display_duration_s": round(track.points[-1].time_s, 6),
            }
            for track in playback.tracks
        ],
        "warning": (
            "This is a cached kinematic display. The approach starts at the posture recorded "
            "during the review and must be replanned from fresh joints before real motion."
        ),
    }


def publish_display(playback: Playback, *, subscriber_wait_s: float = 20.0) -> None:
    """Publish only a transient-local RViz DisplayTrajectory message."""
    import rclpy
    from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from trajectory_msgs.msg import JointTrajectoryPoint

    rclpy.init()
    node = rclpy.create_node("offline_rviz_print_playback")
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    publisher = node.create_publisher(DisplayTrajectory, "/display_planned_path", qos)
    message = DisplayTrajectory()
    message.model_id = ""
    message.trajectory_start.is_diff = False
    message.trajectory_start.joint_state.name = list(playback.joint_names)
    message.trajectory_start.joint_state.position = list(playback.tracks[0].points[0].positions)
    for track in playback.tracks:
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.header.frame_id = "base_link"
        trajectory.joint_trajectory.joint_names = list(playback.joint_names)
        for source in track.points:
            point = JointTrajectoryPoint()
            point.positions = list(source.positions)
            total_ns = round(source.time_s * 1_000_000_000)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(total_ns, 1_000_000_000)
            trajectory.joint_trajectory.points.append(point)
        message.trajectory.append(trajectory)

    deadline = time.monotonic() + _finite(subscriber_wait_s, "Subscriber wait")
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() == 0:
        node.destroy_node()
        rclpy.shutdown()
        raise PlaybackError("RViz is not subscribed to /display_planned_path")
    for _ in range(5):
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.2)
    # Keep transient-local durability alive long enough for RViz to take its
    # own looping copy, without turning this display helper into a daemon.
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Display a checksum-locked cached MoveIt print review in RViz; never command hardware."
    )
    parser.add_argument("--manifest", default="latest", help="Execution manifest path, or 'latest'")
    parser.add_argument(
        "--workspace-root",
        help="ROS workspace containing runtime/path_review and runtime/print_execution.",
    )
    parser.add_argument(
        "--project-root",
        help=(
            "Project root containing 'Test gcode'. Use this when the ROS workspace "
            "is mounted at a different depth inside a container."
        ),
    )
    parser.add_argument("--speed-factor", type=float, default=10.0)
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--source-path-only", action="store_true")
    args = parser.parse_args()
    playback = load_playback(
        args.manifest,
        workspace_root=Path(args.workspace_root) if args.workspace_root else None,
        project_root=Path(args.project_root) if args.project_root else None,
        speed_factor=args.speed_factor,
    )
    if args.source_path_only:
        print(playback.source_path)
        return 0
    summary = playback_summary(playback)
    if not args.check_only:
        publish_display(playback)
        summary["published_topic"] = "/display_planned_path"
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
