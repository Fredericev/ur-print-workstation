"""Persistent ROS 2 action server that owns Moonraker writes."""

from __future__ import annotations

import hashlib
import threading
import time
from typing import Any

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_srvs.srv import Trigger
from ur_klipper_interfaces.action import ManualStepperMove, ManualStepperProgram
from ur_klipper_interfaces.msg import KlipperStatus

from .gcode import (
    MAXIMUM_START_DELAY_S,
    build_completion_barrier_script,
    build_dispatch_script,
    build_program_file,
    start_delay_until,
)
from .moonraker import KlipperSnapshot, MoonrakerClient, MoonrakerError
from .validation import (
    ValidationError,
    estimate_move_duration,
    minimum_extrusion_duration,
    EXTRUSION_PROTOCOL,
    validate_finite_number,
    validate_operation_id,
    validate_positive_number,
    validate_extruder,
)


# print_stats is served by Klippy over Moonraker's serialized Unix-socket
# connection.  One sample per second is ample for supervisory state checking
# during a multi-minute pre-queued program and leaves that connection available
# for the active virtual-SD execution itself.
_PROGRAM_PRINT_STATS_POLL_INTERVAL_S = 1.0
# The Pi demonstrably completed valid Moonraker requests just after the former
# five-second client deadline.  Leave a meaningful margin without making a
# true loss of the supervisory channel look healthy indefinitely.
_MOONRAKER_REQUEST_TIMEOUT_S = 8.0
# Reserve part of the existing monitoring read budget for one lost response.
# This is an HTTP observation budget, not a physical-stop timing guarantee.
_MOTION_STATUS_READ_BUDGET_S = 8.0
_MOTION_STATUS_PRIMARY_TIMEOUT_S = 6.0
_MOTION_STATUS_RETRY_DELAY_S = 0.25
# A failed emergency-stop HTTP response has unknown delivery semantics.  The
# action path checks briefly so it can return and let the coordinator cancel
# the UR immediately; the dedicated stop service gets a longer bounded window
# because it is the coordinator's postcondition authority.
_EMERGENCY_ACTION_CONFIRM_TIMEOUT_S = 1.0
_EMERGENCY_SERVICE_CONFIRM_TIMEOUT_S = 8.0
_EMERGENCY_CONFIRM_POLL_INTERVAL_S = 0.25
_EMERGENCY_CONFIRM_READ_TIMEOUT_S = 1.0


class KlipperBridgeNode(Node):
    """Serialize guarded native-extruder actions and publish live readiness."""

    def __init__(self) -> None:
        """Create publishers, services, polling, and the serialized action."""
        super().__init__("klipper_bridge")

        self.declare_parameter(
            "execution_protocol_version",
            "native-extruder-program-v6",
            descriptor=ParameterDescriptor(
                read_only=True,
                description="Compiled bridge execution contract; not a physical-sync guarantee",
            ),
            ignore_override=True,
        )
        self.declare_parameter("moonraker_url", "http://192.168.0.168")
        self.declare_parameter("default_stepper", "extruder")
        self.declare_parameter("request_timeout_s", _MOONRAKER_REQUEST_TIMEOUT_S)
        self.declare_parameter("completion_timeout_margin_s", 10.0)
        self.declare_parameter("poll_interval_s", 2.0)
        self.declare_parameter("max_abs_target", 200.0)
        self.declare_parameter("max_speed", 5.0)
        self.declare_parameter("max_acceleration", 20.0)

        moonraker_url = self.get_parameter("moonraker_url").value
        self._default_stepper = validate_extruder(
            self.get_parameter("default_stepper").value
        )
        self._request_timeout_s = validate_positive_number(
            "request_timeout_s",
            self.get_parameter("request_timeout_s").value,
        )
        self._completion_timeout_margin_s = validate_positive_number(
            "completion_timeout_margin_s",
            self.get_parameter("completion_timeout_margin_s").value,
        )
        poll_interval_s = validate_positive_number(
            "poll_interval_s",
            self.get_parameter("poll_interval_s").value,
        )
        self._max_abs_target = validate_positive_number(
            "max_abs_target",
            self.get_parameter("max_abs_target").value,
        )
        self._max_speed = validate_positive_number(
            "max_speed",
            self.get_parameter("max_speed").value,
        )
        self._max_acceleration = validate_positive_number(
            "max_acceleration",
            self.get_parameter("max_acceleration").value,
        )
        self._client = MoonrakerClient(
            moonraker_url,
            timeout_s=self._request_timeout_s,
        )
        # Injectable monotonic clock seams keep the long-motion completion
        # protocol deterministic in unit tests without weakening production.
        self._monotonic_now = time.monotonic
        self._sleep_for = time.sleep

        self._state_lock = threading.RLock()
        self._dispatch_request_lock = threading.Lock()
        self._refresh_lock = threading.Lock()
        # Status reads are safe to retry, but overlapping object queries can
        # starve a small Moonraker host.  Keep reads serialized while leaving
        # emergency writes independent of a stalled read.
        self._read_request_lock = threading.Lock()
        self._goal_reserved = False
        self._motion_dispatched = False
        self._emergency_sent = False
        self._emergency_error = ""
        self._active_operation_id = ""
        self._last_stepper = self._default_stepper
        self._snapshot: KlipperSnapshot | None = None
        self._last_status_fingerprint: tuple[Any, ...] | None = None

        callback_group = ReentrantCallbackGroup()
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            KlipperStatus,
            "/klipper/status",
            status_qos,
        )
        self._action_server = ActionServer(
            self,
            ManualStepperMove,
            "/klipper/manual_stepper_move",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )
        self._program_action_server = ActionServer(
            self,
            ManualStepperProgram,
            "/klipper/manual_stepper_program",
            execute_callback=self._program_execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )
        self._refresh_service = self.create_service(
            Trigger,
            "/klipper/refresh",
            self._refresh_service_callback,
            callback_group=callback_group,
        )
        self._disable_service = self.create_service(
            Trigger,
            "/klipper/disable_stepper",
            self._disable_service_callback,
            callback_group=callback_group,
        )
        self._emergency_service = self.create_service(
            Trigger,
            "/klipper/emergency_stop",
            self._emergency_service_callback,
            callback_group=callback_group,
        )
        self._poll_timer = self.create_timer(
            poll_interval_s,
            self._poll_status_callback,
            callback_group=callback_group,
        )

        self._publish_status()
        self.get_logger().info(
            "ROS owns Klipper writes through guarded native-extruder actions"
        )

    def _goal_callback(
        self,
        goal_request: ManualStepperMove.Goal,
    ) -> GoalResponse:
        with self._state_lock:
            if self._goal_reserved:
                self.get_logger().warning(
                    "Rejected Klipper action because another operation "
                    "is active"
                )
                return GoalResponse.REJECT
            self._goal_reserved = True
            self._active_operation_id = str(goal_request.operation_id)
            self._motion_dispatched = False
            self._emergency_sent = False
            self._emergency_error = ""
        self._publish_status()
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle: Any) -> CancelResponse:
        with self._state_lock:
            dispatched = self._motion_dispatched
        if dispatched:
            error = self._emergency_stop_once("action cancellation")
            if error:
                self.get_logger().error(error)
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle: Any) -> ManualStepperMove.Result:
        estimated_duration_s = 0.0
        try:
            self._feedback(
                goal_handle,
                ManualStepperMove.Feedback.PHASE_VALIDATING,
                "Validating native-extruder action",
            )
            goal = goal_handle.request
            operation_id = validate_operation_id(goal.operation_id)
            stepper = validate_extruder(goal.stepper)
            target = validate_finite_number(
                "target_position",
                goal.target_position,
            )
            speed = validate_positive_number("speed", goal.speed)
            acceleration = validate_positive_number(
                "acceleration",
                goal.acceleration,
            )
            initial = validate_finite_number(
                "initial_position",
                goal.initial_position,
            )
            start_delay_s = validate_finite_number(
                "start_delay_s",
                goal.start_delay_s,
            )
            if start_delay_s < 0.0 or start_delay_s > MAXIMUM_START_DELAY_S:
                raise ValidationError(
                    f"start_delay_s must be between 0 and {MAXIMUM_START_DELAY_S:g} seconds"
                )
            start_time_nanoseconds = int(goal.start_time_nanoseconds)
            if start_time_nanoseconds < 0:
                raise ValidationError(
                    "start_time_nanoseconds must be zero or a positive ROS timestamp"
                )
            if stepper != self._default_stepper:
                raise ValidationError(
                    f"stepper {stepper} is not the configured ROS bridge "
                    f"allowlist entry {self._default_stepper}"
                )
            if abs(target) > self._max_abs_target:
                raise ValidationError(
                    f"target_position must be between "
                    f"{-self._max_abs_target:g} and {self._max_abs_target:g}"
                )
            if abs(initial) > self._max_abs_target:
                raise ValidationError(
                    f"initial_position must be between "
                    f"{-self._max_abs_target:g} and {self._max_abs_target:g}"
                )
            if speed > self._max_speed:
                raise ValidationError(
                    f"speed must be no greater than {self._max_speed:g}"
                )
            if acceleration > self._max_acceleration:
                raise ValidationError(
                    "acceleration must be no greater than "
                    f"{self._max_acceleration:g}"
                )
            estimated_duration_s = minimum_extrusion_duration(
                target - initial, speed, acceleration,
            )
            with self._state_lock:
                self._active_operation_id = operation_id
                self._last_stepper = stepper
            self._publish_status()

            if goal_handle.is_cancel_requested:
                return self._finish_canceled(
                    goal_handle,
                    estimated_duration_s,
                    "Action canceled before readiness checks",
                )

            self._feedback(
                goal_handle,
                ManualStepperMove.Feedback.PHASE_CHECKING_READY,
                "Checking Klipper readiness and native-extruder configuration",
            )
            snapshot = self._refresh_snapshot()
            if not snapshot.reachable:
                return self._finish_aborted(
                    goal_handle,
                    "MOONRAKER_UNREACHABLE",
                    snapshot.detail or "Moonraker is unreachable",
                    estimated_duration_s,
                )
            if not snapshot.ready:
                return self._finish_aborted(
                    goal_handle,
                    "KLIPPER_NOT_READY",
                    (
                        "Klipper state is "
                        f"{snapshot.printer_state or 'unknown'}: "
                        f"{snapshot.detail}"
                    ).rstrip(": "),
                    estimated_duration_s,
                )
            self._assert_native_motion_ready(snapshot, stepper, speed, acceleration)
            if not goal.set_position_before_move:
                raise ValidationError("native extrusion requires set_position_before_move=true")
            # Exercise the same persistent observation connection that will
            # monitor this action before any G-code can reach the printer.
            # It may have become stale while the bridge was idle; recover it
            # under the read budget while failure still means no dispatch.
            snapshot, _ = self._read_motion_status(goal_handle, snapshot)
            self._assert_native_motion_ready(snapshot, stepper, speed, acceleration)
            if goal_handle.is_cancel_requested:
                return self._finish_canceled(
                    goal_handle,
                    estimated_duration_s,
                    "Action canceled before dispatch",
                )

            if start_time_nanoseconds:
                start_delay_s = start_delay_until(
                    start_time_nanoseconds,
                    self.get_clock().now().nanoseconds,
                )

            dispatch_script = build_dispatch_script(
                stepper=stepper,
                target_position=target,
                speed=speed,
                acceleration=acceleration,
                set_position_before_move=bool(goal.set_position_before_move),
                initial_position=initial,
                start_delay_s=start_delay_s,
            )
            # Mark the write as potentially dispatched before the network
            # call. If Moonraker accepts it but its response is lost, the
            # exception path still requests an emergency stop and reports
            # may_have_moved.
            post_started_ros_ns = self.get_clock().now().nanoseconds
            post_started_monotonic = self._monotonic_now()
            with self._dispatch_request_lock:
                with self._state_lock:
                    self._motion_dispatched = True
                self._client.post_gcode(dispatch_script)
            post_finished_ros_ns = self.get_clock().now().nanoseconds
            post_finished_monotonic = self._monotonic_now()
            self.get_logger().info(
                "Klipper dispatch timing: "
                f"operation_id={operation_id} "
                f"requested_start_ros_ns={start_time_nanoseconds} "
                f"post_started_ros_ns={post_started_ros_ns} "
                f"post_finished_ros_ns={post_finished_ros_ns} "
                f"post_elapsed_s={post_finished_monotonic - post_started_monotonic:.6f}; "
                "HTTP acknowledgement is not physical-start feedback"
            )
            if goal_handle.is_cancel_requested:
                return self._finish_canceled(
                    goal_handle,
                    estimated_duration_s,
                    "Action canceled after dispatch",
                )

            if start_time_nanoseconds:
                try:
                    # The first guard ran before HTTP. A delayed response
                    # must never masquerade as an on-time/armed endpoint.
                    # This guard still cannot prove the physical start time.
                    start_delay_until(start_time_nanoseconds, post_finished_ros_ns)
                except ValidationError as exc:
                    detail = (
                        "Moonraker acknowledgement missed the requested start guard; "
                        f"the move may already be queued or moving: {exc}"
                    )
                    emergency_error = self._emergency_stop_once(
                        "requested start deadline missed after dispatch"
                    )
                    if emergency_error:
                        detail += f"; {emergency_error}"
                    return self._finish_aborted(
                        goal_handle, "START_DEADLINE_MISSED", detail, estimated_duration_s
                    )

            self._feedback(
                goal_handle,
                ManualStepperMove.Feedback.PHASE_DISPATCHED,
                "Moonraker acknowledged the queued move; physical start time is unverified",
            )

            self._feedback(
                goal_handle,
                ManualStepperMove.Feedback.PHASE_WAITING_FOR_COMPLETION,
                "Waiting locally for the planned Klipper motion before "
                "the M400 completion barrier",
            )
            # Native G1 queues motion without M400. Keep the long wait locally
            # cancelable, then prove physical completion with M400.
            self._wait_before_completion_barrier(
                goal_handle,
                start_delay_s + estimated_duration_s,
                post_finished_monotonic,
                snapshot,
            )
            if goal_handle.is_cancel_requested:
                return self._finish_canceled(
                    goal_handle,
                    estimated_duration_s,
                    "Action canceled before completion barrier",
                )
            barrier_script = build_completion_barrier_script(
                stepper=stepper,
                target_position=target,
                speed=speed,
                acceleration=acceleration,
            )
            completion_timeout_s = max(
                self._request_timeout_s,
                minimum_extrusion_duration(
                    2.0 * self._max_abs_target, speed, acceleration,
                )
                + self._completion_timeout_margin_s,
            )
            self._client.post_gcode(
                barrier_script,
                timeout_s=completion_timeout_s,
            )

            if goal_handle.is_cancel_requested:
                return self._finish_canceled(
                    goal_handle,
                    estimated_duration_s,
                    "Action canceled at completion",
                )

            if goal.disable_after:
                self._client.disable_stepper(stepper)

            self._feedback(
                goal_handle,
                ManualStepperMove.Feedback.PHASE_COMPLETED,
                "Klipper native-extruder action completed",
            )
            goal_handle.succeed()
            return self._make_result(
                success=True,
                error_code="",
                message="Klipper native-extruder action completed",
                estimated_duration_s=estimated_duration_s,
                may_have_moved=True,
            )
        except ValidationError as exc:
            return self._finish_aborted(
                goal_handle,
                "INVALID_GOAL",
                str(exc),
                estimated_duration_s,
            )
        except MoonrakerError as exc:
            if goal_handle.is_cancel_requested:
                return self._finish_canceled(
                    goal_handle,
                    estimated_duration_s,
                    f"Action canceled while Moonraker reported {exc.code}",
                )
            emergency_error = self._emergency_after_unexpected_failure()
            detail = str(exc)
            if emergency_error:
                detail += f"; {emergency_error}"
            return self._finish_aborted(
                goal_handle,
                exc.code,
                detail,
                estimated_duration_s,
            )
        # A dispatched unknown failure must always fail closed.
        except Exception as exc:
            emergency_error = self._emergency_after_unexpected_failure()
            detail = f"Unexpected bridge failure: {type(exc).__name__}: {exc}"
            if emergency_error:
                detail += f"; {emergency_error}"
            self.get_logger().error(detail)
            return self._finish_aborted(
                goal_handle,
                "INTERNAL_ERROR",
                detail,
                estimated_duration_s,
            )
        finally:
            with self._state_lock:
                self._goal_reserved = False
                self._motion_dispatched = False
                self._emergency_sent = False
                self._emergency_error = ""
                self._active_operation_id = ""
            self._publish_status()

    def _program_execute_callback(
        self,
        goal_handle: Any,
    ) -> ManualStepperProgram.Result:
        """Upload, start, and observe one complete virtual-SD native extrusion program."""
        estimated_duration_s = 0.0
        try:
            self._program_feedback(
                goal_handle,
                ManualStepperProgram.Feedback.PHASE_VALIDATING,
                "Validating continuous native-extruder program",
            )
            goal = goal_handle.request
            operation_id = validate_operation_id(goal.operation_id)
            stepper = validate_extruder(goal.stepper)
            maximum_speed = validate_positive_number(
                "maximum_speed", goal.maximum_speed
            )
            start_delay_s = validate_finite_number(
                "start_delay_s", goal.start_delay_s
            )
            if maximum_speed > self._max_speed:
                raise ValidationError(
                    f"maximum_speed must be no greater than {self._max_speed:g}"
                )
            if stepper != self._default_stepper:
                raise ValidationError(
                    f"stepper {stepper} is not the configured ROS bridge "
                    f"allowlist entry {self._default_stepper}"
                )
            units = list(goal.segment_units)
            durations = list(goal.segment_durations_s)
            if len(units) != len(durations):
                raise ValidationError(
                    "segment_units and segment_durations_s must have equal length"
                )
            segments: list[dict[str, float]] = []
            for index, (raw_units, raw_duration) in enumerate(zip(units, durations)):
                distance = validate_finite_number(f"segment {index} units", raw_units)
                duration = validate_positive_number(f"segment {index} duration_s", raw_duration)
                segments.append({"units": distance, "duration_s": duration})
            program = build_program_file(
                stepper=stepper,
                segments=segments,
                start_delay_s=start_delay_s,
                maximum_speed=maximum_speed,
                disable_after=bool(goal.disable_after),
                maximum_acceleration=self._max_acceleration,
            )
            estimated_duration_s = start_delay_s + sum(
                segment["duration_s"] for segment in segments
            )
            filename = (
                "ur_extrusion_"
                + hashlib.sha256(operation_id.encode("utf-8")).hexdigest()[:20]
                + ".gcode"
            )
            with self._state_lock:
                self._active_operation_id = operation_id
                self._last_stepper = stepper
            self._publish_status()

            if goal_handle.is_cancel_requested:
                return self._finish_program_canceled(
                    goal_handle,
                    estimated_duration_s,
                    "Continuous program canceled before readiness checks",
                )
            snapshot = self._refresh_snapshot()
            if not snapshot.reachable:
                return self._finish_program_aborted(
                    goal_handle,
                    "MOONRAKER_UNREACHABLE",
                    snapshot.detail or "Moonraker is unreachable",
                    estimated_duration_s,
                )
            if not snapshot.ready:
                return self._finish_program_aborted(
                    goal_handle,
                    "KLIPPER_NOT_READY",
                    (
                        "Klipper state is "
                        f"{snapshot.printer_state or 'unknown'}: {snapshot.detail}"
                    ).rstrip(": "),
                    estimated_duration_s,
                )
            self._assert_native_motion_ready(
                snapshot, stepper, maximum_speed, self._max_acceleration,
            )
            # Establish or recover the runtime GET connection before upload
            # and start. A separate successful full-config request does not
            # prove that an old pooled monitoring socket remains usable.
            snapshot, prior = self._read_motion_status(
                goal_handle, snapshot, include_print_stats=True,
            )
            self._assert_native_motion_ready(
                snapshot, stepper, maximum_speed, self._max_acceleration,
            )
            if prior is None:
                raise MoonrakerError("INVALID_RESPONSE", "Missing combined preflight print status")
            if prior.state in {"printing", "paused"}:
                return self._finish_program_aborted(
                    goal_handle,
                    "VIRTUAL_SD_BUSY",
                    "Klipper already has an active virtual-SD job",
                    estimated_duration_s,
                )
            if goal_handle.is_cancel_requested:
                return self._finish_program_canceled(
                    goal_handle,
                    estimated_duration_s,
                    "Continuous program canceled before upload",
                )

            self._program_feedback(
                goal_handle,
                ManualStepperProgram.Feedback.PHASE_UPLOADING,
                "Uploading immutable virtual-SD program",
            )
            uploaded = self._client.upload_gcode(filename, program)
            if uploaded != filename:
                raise MoonrakerError(
                    "INVALID_RESPONSE",
                    "Moonraker returned a different virtual-SD filename",
                )
            if goal_handle.is_cancel_requested:
                return self._finish_program_canceled(
                    goal_handle,
                    estimated_duration_s,
                    "Continuous program canceled before start",
                )

            post_started_ros_ns = self.get_clock().now().nanoseconds
            with self._dispatch_request_lock:
                with self._state_lock:
                    self._motion_dispatched = True
                self._client.start_print(filename)
            post_finished_ros_ns = self.get_clock().now().nanoseconds
            program_epoch_ros_ns = (
                post_started_ros_ns + post_finished_ros_ns
            ) // 2
            self._program_feedback(
                goal_handle,
                ManualStepperProgram.Feedback.PHASE_DISPATCHED,
                "Moonraker acknowledged one virtual-SD start; physical start is unverified",
                program_epoch_ros_ns=program_epoch_ros_ns,
            )

            observed_running = False
            print_started_monotonic = self._monotonic_now()
            next_print_stats_poll = print_started_monotonic
            deadline = (
                print_started_monotonic
                + estimated_duration_s
                + self._completion_timeout_margin_s
            )
            while self._monotonic_now() < deadline:
                if goal_handle.is_cancel_requested:
                    return self._finish_program_canceled(
                        goal_handle,
                        estimated_duration_s,
                        "Continuous program canceled after start",
                    )
                now = self._monotonic_now()
                if now < next_print_stats_poll:
                    self._sleep_for(
                        min(0.1, next_print_stats_poll - now)
                    )
                    continue
                live, stats = self._read_motion_status(
                    goal_handle, snapshot, include_print_stats=True,
                )
                self._assert_native_motion_ready(
                    live, stepper, maximum_speed, self._max_acceleration,
                )
                if stats is None:
                    raise MoonrakerError("INVALID_RESPONSE", "Missing combined print status")
                next_print_stats_poll = (
                    self._monotonic_now()
                    + _PROGRAM_PRINT_STATS_POLL_INTERVAL_S
                )
                start_age = self._monotonic_now() - print_started_monotonic
                if (
                    not observed_running
                    and stats.state in {"standby", "complete"}
                    and start_age < 2.0
                ):
                    # print_stats can briefly retain the previous completed
                    # run, even when a retry uses the same immutable filename.
                    # Never treat that stale completion as this run's success.
                    continue
                if not observed_running and stats.state != "printing" and start_age >= 2.0:
                    raise MoonrakerError(
                        "PRINT_START_UNCONFIRMED",
                        "Klipper did not confirm the new virtual-SD running state",
                    )
                if stats.filename and stats.filename != filename:
                    if (
                        not observed_running
                        and stats.state not in {"printing", "paused"}
                        and self._monotonic_now() - print_started_monotonic < 2.0
                    ):
                        continue
                    raise MoonrakerError(
                        "PRINT_IDENTITY_CHANGED",
                        "Moonraker print_stats changed to another file",
                    )
                if stats.state == "printing":
                    if stats.filename != filename:
                        raise MoonrakerError(
                            "PRINT_IDENTITY_CHANGED",
                            "Klipper running state did not identify the uploaded file",
                        )
                    observed_running = True
                    progress = min(
                        0.999,
                        max(0.0, stats.print_duration_s / estimated_duration_s),
                    )
                    self._program_feedback(
                        goal_handle,
                        ManualStepperProgram.Feedback.PHASE_RUNNING,
                        "Klipper is executing the queued continuous program",
                        program_epoch_ros_ns=program_epoch_ros_ns,
                        progress=progress,
                    )
                elif stats.state == "complete":
                    if not observed_running:
                        raise MoonrakerError(
                            "PRINT_START_UNCONFIRMED",
                            "Klipper reported completion without an observed running state",
                        )
                    self._program_feedback(
                        goal_handle,
                        ManualStepperProgram.Feedback.PHASE_COMPLETED,
                        "Klipper virtual-SD program completed through its M400 barrier",
                        program_epoch_ros_ns=program_epoch_ros_ns,
                        progress=1.0,
                    )
                    goal_handle.succeed()
                    return self._make_program_result(
                        success=True,
                        error_code="",
                        message="Klipper continuous native-extruder program completed",
                        estimated_duration_s=estimated_duration_s,
                        may_have_moved=True,
                    )
                elif stats.state in {"error", "cancelled", "canceled", "paused"}:
                    raise MoonrakerError(
                        "PRINT_FAILED",
                        f"Klipper virtual-SD job entered {stats.state}: {stats.message}".rstrip(),
                    )
                elif observed_running and stats.state != "printing":
                    raise MoonrakerError(
                        "PRINT_STATE_LOST",
                        f"Klipper virtual-SD job left printing for {stats.state}",
                    )
            raise MoonrakerError(
                "PRINT_TIMEOUT",
                "Timed out waiting for the continuous Klipper program to complete",
            )
        except ValidationError as exc:
            return self._finish_program_aborted(
                goal_handle,
                "INVALID_GOAL",
                str(exc),
                estimated_duration_s,
            )
        except MoonrakerError as exc:
            if goal_handle.is_cancel_requested:
                return self._finish_program_canceled(
                    goal_handle,
                    estimated_duration_s,
                    f"Continuous program canceled while Moonraker reported {exc.code}",
                )
            emergency_error = self._emergency_after_unexpected_failure()
            detail = str(exc)
            if emergency_error:
                detail += f"; {emergency_error}"
            return self._finish_program_aborted(
                goal_handle,
                exc.code,
                detail,
                estimated_duration_s,
            )
        except Exception as exc:
            emergency_error = self._emergency_after_unexpected_failure()
            detail = (
                "Unexpected continuous-program bridge failure: "
                f"{type(exc).__name__}: {exc}"
            )
            if emergency_error:
                detail += f"; {emergency_error}"
            self.get_logger().error(detail)
            return self._finish_program_aborted(
                goal_handle,
                "INTERNAL_ERROR",
                detail,
                estimated_duration_s,
            )
        finally:
            with self._state_lock:
                self._goal_reserved = False
                self._motion_dispatched = False
                self._emergency_sent = False
                self._emergency_error = ""
                self._active_operation_id = ""
            self._publish_status()

    @staticmethod
    def _assert_native_motion_ready(
        snapshot: KlipperSnapshot, extruder: str, speed: float, acceleration: float,
    ) -> None:
        """Fail before dispatch, or stop an active job, on native E guard loss."""
        if not snapshot.reachable or not snapshot.ready:
            raise MoonrakerError("KLIPPER_NOT_READY", snapshot.detail or "Klipper is not ready")
        if extruder not in snapshot.extruders:
            raise MoonrakerError("EXTRUDER_NOT_FOUND", f"Klipper config does not contain [{extruder}]")
        state = snapshot.extruder_states.get(extruder)
        if state is None:
            raise MoonrakerError("EXTRUDER_STATUS_MISSING", f"Missing live {extruder} status")
        if not state.can_extrude:
            raise MoonrakerError(
                "EXTRUDER_TOO_COLD",
                f"{extruder} is {state.temperature:g} C; heat it to the material temperature "
                f"(minimum {state.minimum_temperature:g} C) before extrusion",
            )
        if abs(snapshot.speed_factor - 1.0) > 1.0e-9 or abs(snapshot.extrude_factor - 1.0) > 1.0e-9:
            raise MoonrakerError(
                "EXTRUSION_OVERRIDE_CHANGED",
                "Native synchronized extrusion requires M220 S100 and M221 S100; "
                "restore the speed and flow overrides before preparing the print",
            )
        effective_acceleration = min(state.maximum_acceleration, snapshot.toolhead_max_accel)
        if abs(effective_acceleration - acceleration) > 1.0e-9:
            raise MoonrakerError(
                "EXTRUSION_LIMIT_MISMATCH",
                f"Installed native acceleration is {effective_acceleration:g}, expected {acceleration:g} mm/s^2",
            )
        if speed > min(state.maximum_speed, snapshot.toolhead_max_velocity) + 1.0e-9:
            raise MoonrakerError("EXTRUSION_LIMIT_MISMATCH", "Native extruder speed limit is lower than the prepared program")
        if abs(state.maximum_distance - 50.0) > 1.0e-9:
            raise MoonrakerError(
                "EXTRUSION_LIMIT_MISMATCH",
                "Native extrusion program expects max_extrude_only_distance=50 mm; prepare with matching limits",
            )
        # Stock toolhead minimum-cruise-ratio adds a separate smoothing bound.
        # The installed printer max_accel=1000 leaves it above extruder accel=20.
        smoothing_acceleration = snapshot.toolhead_max_accel * (1.0 - snapshot.minimum_cruise_ratio)
        if smoothing_acceleration + 1.0e-9 < acceleration:
            raise MoonrakerError(
                "EXTRUSION_LIMIT_MISMATCH", "Toolhead minimum_cruise_ratio changes native E segment timing",
            )

    def _finish_program_aborted(
        self,
        goal_handle: Any,
        error_code: str,
        message: str,
        estimated_duration_s: float,
    ) -> ManualStepperProgram.Result:
        with self._state_lock:
            may_have_moved = self._motion_dispatched
        self._program_feedback(
            goal_handle,
            ManualStepperProgram.Feedback.PHASE_FAILED,
            message,
        )
        goal_handle.abort()
        return self._make_program_result(
            success=False,
            error_code=error_code,
            message=message,
            estimated_duration_s=estimated_duration_s,
            may_have_moved=may_have_moved,
        )

    def _finish_program_canceled(
        self,
        goal_handle: Any,
        estimated_duration_s: float,
        message: str,
    ) -> ManualStepperProgram.Result:
        with self._state_lock:
            dispatched = self._motion_dispatched
        self._program_feedback(
            goal_handle,
            ManualStepperProgram.Feedback.PHASE_CANCELING,
            message,
        )
        emergency_error = self._emergency_stop_once(
            "continuous-program cancellation"
        )
        error_code = "CANCELED"
        if emergency_error:
            error_code = "CANCELED_EMERGENCY_STOP_FAILED"
            message += f"; {emergency_error}"
        goal_handle.canceled()
        return self._make_program_result(
            success=False,
            error_code=error_code,
            message=message,
            estimated_duration_s=estimated_duration_s,
            may_have_moved=dispatched,
        )

    @staticmethod
    def _make_program_result(
        *,
        success: bool,
        error_code: str,
        message: str,
        estimated_duration_s: float,
        may_have_moved: bool,
    ) -> ManualStepperProgram.Result:
        result = ManualStepperProgram.Result()
        result.success = success
        result.error_code = error_code
        result.message = message
        result.estimated_duration_s = estimated_duration_s
        result.may_have_moved = may_have_moved
        return result

    @staticmethod
    def _program_feedback(
        goal_handle: Any,
        phase: int,
        detail: str,
        *,
        program_epoch_ros_ns: int = 0,
        progress: float = 0.0,
    ) -> None:
        feedback = ManualStepperProgram.Feedback()
        feedback.phase = phase
        feedback.detail = detail
        feedback.program_epoch_ros_ns = int(program_epoch_ros_ns)
        feedback.progress = float(progress)
        goal_handle.publish_feedback(feedback)

    def _wait_before_completion_barrier(
        self,
        goal_handle: Any,
        estimated_duration_s: float,
        dispatch_finished_monotonic: float,
        reference_snapshot: KlipperSnapshot,
    ) -> None:
        """Wait outside HTTP, while retaining cancel and readiness guards."""
        planned_end = dispatch_finished_monotonic + max(0.0, estimated_duration_s)
        next_readiness_check = dispatch_finished_monotonic + 2.0
        while True:
            now = self._monotonic_now()
            if goal_handle.is_cancel_requested or now >= planned_end:
                return
            if now >= next_readiness_check:
                snapshot, _ = self._read_motion_status(goal_handle, reference_snapshot)
                if not snapshot.reachable or not snapshot.ready:
                    detail = snapshot.detail or snapshot.printer_state or "Klipper is not ready"
                    raise MoonrakerError(
                        "KLIPPER_NOT_READY",
                        f"Klipper became unavailable during motion: {detail}",
                    )
                self._assert_native_motion_ready(
                    snapshot, self._last_stepper, self._max_speed, self._max_acceleration,
                )
                next_readiness_check = self._monotonic_now() + 2.0
            self._sleep_for(min(0.1, max(0.0, planned_end - self._monotonic_now())))

    def _read_motion_status(
        self,
        goal_handle: Any,
        reference: KlipperSnapshot,
        *,
        include_print_stats: bool = False,
    ) -> tuple[KlipperSnapshot, PrintStatsSnapshot | None]:
        """Read fresh dynamic safety data, retrying one transient GET failure.

        Static config is read at every action's preflight. Runtime monitoring
        needs only one compact query, including print_stats for a program.
        The retry shares the observation budget rather than doubling it.
        Neither a cached healthy state nor an uncertain control write is retried.
        """
        started = self._monotonic_now()
        budget = min(self._request_timeout_s, _MOTION_STATUS_READ_BUDGET_S)
        deadline = started + budget

        def unavailable(error: MoonrakerError) -> MoonrakerError:
            # Do not leave an old healthy sample displayed after losing the
            # observation channel or receiving unusable safety data.
            with self._state_lock:
                self._snapshot = KlipperSnapshot(
                    reachable=False, ready=False, printer_state="",
                    detail=f"{error.code}: {error}",
                    manual_steppers=reference.manual_steppers,
                    extruders=reference.extruders,
                )
            self._publish_status()
            self.get_logger().error(
                f"Moonraker motion-status monitor failed after {self._monotonic_now()-started:.3f}s "
                f"(read budget {budget:g}s): {error.code}: {error}"
            )
            return error

        for attempt in range(2):
            if goal_handle.is_cancel_requested:
                raise MoonrakerError("CANCELED", "Canceled before the next motion-status read")
            remaining = deadline - self._monotonic_now()
            if remaining <= 0.0:
                raise unavailable(MoonrakerError("NETWORK_ERROR", "Moonraker motion-status read budget exhausted"))
            acquired = self._read_request_lock.acquire(timeout=remaining)
            if not acquired:
                raise unavailable(MoonrakerError("NETWORK_ERROR", "Moonraker motion-status read lock timed out"))
            try:
                remaining = deadline - self._monotonic_now()
                if remaining <= 0.0:
                    raise MoonrakerError("NETWORK_ERROR", "Moonraker motion-status read budget exhausted")
                timeout = min(remaining, _MOTION_STATUS_PRIMARY_TIMEOUT_S, budget * 0.75) if attempt == 0 else remaining
                snapshot, stats = self._client.read_motion_status(
                    reference, include_print_stats=include_print_stats, timeout_s=timeout,
                )
            except MoonrakerError as exc:
                error = exc
            else:
                error = None
            finally:
                self._read_request_lock.release()
            if goal_handle.is_cancel_requested:
                raise MoonrakerError("CANCELED", "Canceled during the motion-status read")
            now = self._monotonic_now()
            if error is None:
                if now > deadline:
                    raise unavailable(MoonrakerError("NETWORK_ERROR", "Moonraker motion-status response exceeded its read budget"))
                with self._state_lock:
                    self._snapshot = snapshot
                self._publish_status()
                if attempt:
                    self.get_logger().warning(
                        f"Moonraker motion-status GET recovered on retry in {now-started:.3f}s "
                        f"(read budget {budget:g}s); control commands were not repeated"
                    )
                return snapshot, stats
            if error.code != "NETWORK_ERROR" or attempt or now + _MOTION_STATUS_RETRY_DELAY_S >= deadline:
                raise unavailable(error)
            self.get_logger().warning(
                f"Moonraker motion-status GET failed after {now-started:.3f}s; "
                f"retrying once within the remaining {deadline-now:.3f}s read budget: {error}"
            )
            retry_at = now + _MOTION_STATUS_RETRY_DELAY_S
            while self._monotonic_now() < retry_at:
                if goal_handle.is_cancel_requested:
                    raise MoonrakerError("CANCELED", "Canceled during motion-status retry delay")
                self._sleep_for(min(0.05, retry_at - self._monotonic_now()))
        raise AssertionError("unreachable motion-status retry state")

    def _finish_aborted(
        self,
        goal_handle: Any,
        error_code: str,
        message: str,
        estimated_duration_s: float,
    ) -> ManualStepperMove.Result:
        with self._state_lock:
            may_have_moved = self._motion_dispatched
        self._feedback(
            goal_handle,
            ManualStepperMove.Feedback.PHASE_FAILED,
            message,
        )
        goal_handle.abort()
        return self._make_result(
            success=False,
            error_code=error_code,
            message=message,
            estimated_duration_s=estimated_duration_s,
            may_have_moved=may_have_moved,
        )

    def _finish_canceled(
        self,
        goal_handle: Any,
        estimated_duration_s: float,
        message: str,
    ) -> ManualStepperMove.Result:
        with self._state_lock:
            dispatched = self._motion_dispatched
        self._feedback(
            goal_handle,
            ManualStepperMove.Feedback.PHASE_CANCELING,
            message,
        )
        emergency_error = self._emergency_stop_once("action cancellation")
        error_code = "CANCELED"
        if emergency_error:
            error_code = "CANCELED_EMERGENCY_STOP_FAILED"
            message += f"; {emergency_error}"
        goal_handle.canceled()
        return self._make_result(
            success=False,
            error_code=error_code,
            message=message,
            estimated_duration_s=estimated_duration_s,
            may_have_moved=dispatched,
        )

    @staticmethod
    def _make_result(
        *,
        success: bool,
        error_code: str,
        message: str,
        estimated_duration_s: float,
        may_have_moved: bool,
    ) -> ManualStepperMove.Result:
        result = ManualStepperMove.Result()
        result.success = success
        result.error_code = error_code
        result.message = message
        result.estimated_duration_s = estimated_duration_s
        result.may_have_moved = may_have_moved
        return result

    @staticmethod
    def _feedback(goal_handle: Any, phase: int, detail: str) -> None:
        feedback = ManualStepperMove.Feedback()
        feedback.phase = phase
        feedback.detail = detail
        goal_handle.publish_feedback(feedback)

    def _emergency_after_unexpected_failure(self) -> str:
        with self._state_lock:
            dispatched = self._motion_dispatched
        if not dispatched:
            return ""
        return self._emergency_stop_once("unexpected post-dispatch failure")

    def _emergency_stop_once(self, reason: str) -> str:
        with self._state_lock:
            if not self._motion_dispatched:
                return ""
            if self._emergency_sent:
                return self._emergency_error
            self._emergency_sent = True
        # A cancel that races with the first POST waits until that request
        # finishes, so emergency_stop cannot arrive before a delayed move.
        with self._dispatch_request_lock:
            confirmed, message = self._request_emergency_stop(
                confirmation_timeout_s=_EMERGENCY_ACTION_CONFIRM_TIMEOUT_S,
            )
        if confirmed:
            self.get_logger().error(
                f"{message}: {reason}"
            )
            error = ""
        else:
            error = message
        with self._state_lock:
            self._emergency_error = error
        return error

    def _request_emergency_stop(
        self,
        *,
        confirmation_timeout_s: float,
    ) -> tuple[bool, str]:
        """Issue one stop POST and verify a lost acknowledgement safely.

        A timeout is not proof that Moonraker did not receive the POST.  We
        therefore accept it only when a subsequent read-only printer-info
        probe explicitly reports ``shutdown``.  ``ready``, an absent reply,
        or any other state remains unconfirmed and is never converted into a
        success.
        """
        try:
            self._client.emergency_stop()
            return True, "Klipper emergency stop requested"
        except Exception as request_error:
            deadline = self._monotonic_now() + confirmation_timeout_s
            last_observation = "no /printer/info response"
            while True:
                try:
                    snapshot = self._client.read_printer_state(
                        timeout_s=_EMERGENCY_CONFIRM_READ_TIMEOUT_S,
                    )
                    state = snapshot.state.strip().lower()
                    if state == "shutdown":
                        return (
                            True,
                            "Klipper emergency stop confirmed via "
                            "/printer/info state=shutdown after its HTTP "
                            "acknowledgement was lost",
                        )
                    rendered_state = state or "<empty>"
                    last_observation = (
                        "/printer/info reported "
                        f"state={rendered_state}, not shutdown"
                    )
                except Exception as confirmation_error:
                    last_observation = (
                        "/printer/info postcondition probe failed: "
                        f"{type(confirmation_error).__name__}: "
                        f"{confirmation_error}"
                    )
                now = self._monotonic_now()
                if now >= deadline:
                    return (
                        False,
                        "Klipper emergency stop request failed: "
                        f"{type(request_error).__name__}: {request_error}; "
                        f"{last_observation}",
                    )
                self._sleep_for(
                    min(_EMERGENCY_CONFIRM_POLL_INTERVAL_S, deadline - now)
                )

    def _poll_status_callback(self) -> None:
        # The active action owns print-state monitoring.  Do not overlap its
        # object query with the independent readiness timer on the same small
        # Moonraker host.
        with self._state_lock:
            action_active = self._goal_reserved
        if action_active:
            self._publish_status()
            return
        self._refresh_snapshot(idle_only=True)

    def _refresh_snapshot(self, *, idle_only: bool = False) -> KlipperSnapshot:
        with self._refresh_lock:
            with self._read_request_lock:
                # An idle callback may have queued behind action preflight.
                # Recheck after acquiring the locks so it cannot start a full
                # config read after the action has already begun monitoring.
                if idle_only:
                    with self._state_lock:
                        if self._goal_reserved:
                            return self._snapshot or KlipperSnapshot(
                                reachable=False, ready=False, printer_state="",
                                detail="The active action owns live status monitoring",
                                manual_steppers=(),
                            )
                try:
                    snapshot = self._client.read_status()
                except MoonrakerError as exc:
                    snapshot = KlipperSnapshot(
                        reachable=False,
                        ready=False,
                        printer_state="",
                        detail=f"{exc.code}: {exc}",
                        manual_steppers=(),
                    )
        with self._state_lock:
            self._snapshot = snapshot
        self._publish_status()
        return snapshot

    def _publish_status(self) -> None:
        with self._state_lock:
            snapshot = self._snapshot
            action_active = self._goal_reserved
            operation_id = self._active_operation_id

        message = KlipperStatus()
        message.extrusion_protocol = EXTRUSION_PROTOCOL
        message.extruders = list(snapshot.extruders) if snapshot else []
        extruder_state = snapshot.extruder_states.get(self._default_stepper) if snapshot else None
        message.extruder_can_extrude = bool(extruder_state and extruder_state.can_extrude)
        message.extruder_temperature = extruder_state.temperature if extruder_state else 0.0
        message.extruder_target = extruder_state.target if extruder_state else 0.0
        message.stamp = self.get_clock().now().to_msg()
        if snapshot is None:
            message.state = KlipperStatus.STATE_UNKNOWN
            message.moonraker_reachable = False
            message.printer_ready = False
            message.printer_state = ""
            message.detail = "Waiting for the first Moonraker readiness poll"
            message.manual_steppers = []
        else:
            message.moonraker_reachable = snapshot.reachable
            message.printer_ready = snapshot.ready
            message.printer_state = snapshot.printer_state
            message.detail = snapshot.detail
            message.manual_steppers = list(snapshot.manual_steppers)
            if not snapshot.reachable:
                message.state = KlipperStatus.STATE_UNREACHABLE
            elif snapshot.ready:
                message.state = KlipperStatus.STATE_READY
            else:
                message.state = KlipperStatus.STATE_NOT_READY
        message.action_active = action_active
        message.active_operation_id = operation_id
        self._status_publisher.publish(message)

        fingerprint = (
            message.state,
            message.printer_state,
            message.detail,
            tuple(message.manual_steppers),
            tuple(message.extruders),
            message.extruder_can_extrude,
            message.action_active,
            message.active_operation_id,
        )
        if fingerprint != self._last_status_fingerprint:
            self._last_status_fingerprint = fingerprint
            self.get_logger().info(
                "Klipper status: "
                f"state={message.printer_state or 'unknown'}, "
                f"ready={message.printer_ready}, "
                f"action_active={message.action_active}"
            )

    def _refresh_service_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        with self._state_lock:
            action_active = self._goal_reserved
        if action_active:
            response.success = False
            response.message = (
                "The active action owns live status monitoring; "
                "a full configuration refresh is unavailable during motion."
            )
            return response
        snapshot = self._refresh_snapshot(idle_only=True)
        with self._state_lock:
            action_active = self._goal_reserved
        if action_active:
            response.success = False
            response.message = "The active action owns live status monitoring; full refresh deferred."
            return response
        stepper_found = self._default_stepper in snapshot.extruders
        response.success = (
            snapshot.reachable and snapshot.ready and stepper_found
        )
        response.message = (
            f"Klipper state={snapshot.printer_state or 'unknown'}; "
            f"{snapshot.detail}; "
            f"[{self._default_stepper}] "
            f"{'found' if stepper_found else 'missing'}"
        ).rstrip("; ")
        return response

    def _disable_service_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        with self._state_lock:
            stepper = self._last_stepper or self._default_stepper
        try:
            self._client.disable_stepper(stepper)
            response.success = True
            response.message = f"Disabled Klipper native extruder {stepper}"
        except Exception as exc:
            response.success = False
            response.message = (
                f"Failed to disable Klipper native extruder {stepper}: {exc}"
            )
        return response

    def _emergency_service_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        response.success, response.message = self._request_emergency_stop(
            confirmation_timeout_s=_EMERGENCY_SERVICE_CONFIRM_TIMEOUT_S,
        )
        return response

    def emergency_stop_if_dispatched(self, reason: str) -> None:
        """Fail closed when the ROS process exits during an active move."""
        error = self._emergency_stop_once(reason)
        if error:
            self.get_logger().error(error)

    def destroy_bridge(self) -> None:
        """Destroy action resources before the base node is destroyed."""
        self._action_server.destroy()
        self._program_action_server.destroy()
        self._client.close()


def main(args: list[str] | None = None) -> None:
    """Run the persistent ROS bridge until ROS requests shutdown."""
    rclpy.init(args=args)
    node: KlipperBridgeNode | None = None
    executor: MultiThreadedExecutor | None = None
    try:
        node = KlipperBridgeNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.emergency_stop_if_dispatched("bridge shutdown during action")
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_bridge()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
