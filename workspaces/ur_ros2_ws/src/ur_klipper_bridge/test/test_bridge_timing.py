"""Run the real bridge callback with an isolated transport and virtual clock.

Only ROS import types are stubbed. No node, executor, socket or hardware is
created, so transport-deadline failures can be reproduced deterministically.
"""

import importlib.util
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace

import pytest

from ur_klipper_bridge.moonraker import (
    KlipperSnapshot,
    ExtruderSnapshot,
    MoonrakerError,
    PrinterStateSnapshot,
    PrintStatsSnapshot,
)


def native_snapshot(**updates):
    fields = dict(
        reachable=True, ready=True, printer_state="ready", detail="",
        manual_steppers=("lock_motor",), extruders=("extruder",),
        extruder_states={"extruder": ExtruderSnapshot(True, 200.0, 200.0, 5.0, 20.0, 50.0, 170.0)},
        speed_factor=1.0, extrude_factor=1.0, toolhead_max_accel=1000.0,
        toolhead_max_velocity=1000.0, minimum_cruise_ratio=0.5,
    )
    fields.update(updates)
    return KlipperSnapshot(**fields)


@pytest.mark.parametrize("program", [False, True])
@pytest.mark.parametrize("fault,code", [
    ("cold", "EXTRUDER_TOO_COLD"),
    ("missing", "EXTRUDER_NOT_FOUND"),
    ("flow", "EXTRUSION_OVERRIDE_CHANGED"),
    ("acceleration", "EXTRUSION_LIMIT_MISMATCH"),
])
def test_native_preflight_rejects_without_any_motion_or_emergency(bridge_type, program, fault, code):
    bridge, goal, client = (make_program_case(bridge_type) if program else make_case(bridge_type))[:3]
    changes = {}
    if fault == "cold":
        changes["extruder_states"] = {"extruder": ExtruderSnapshot(False, 25., 0., 5., 20., 50., 170.)}
    elif fault == "missing":
        changes["extruders"] = ()
    elif fault == "flow":
        changes["extrude_factor"] = 1.1
    else:
        changes["extruder_states"] = {"extruder": ExtruderSnapshot(True, 200., 200., 5., 19., 50., 170.)}
    client.read_status = lambda: native_snapshot(**changes)
    result = bridge._program_execute_callback(goal) if program else bridge._execute_callback(goal)
    assert result.success is False
    assert result.error_code == code
    assert result.may_have_moved is False
    assert client.emergency_count == 0
    if program:
        assert client.uploads == []
        assert client.starts == []
    else:
        assert client.scripts == []


@pytest.mark.parametrize("program", [False, True])
def test_runtime_channel_unavailable_before_dispatch_never_starts_motion(bridge_type, program):
    bridge, goal, client = (make_program_case(bridge_type) if program else make_case(bridge_type))[:3]
    attempts = []

    def unavailable_channel(_reference, **kwargs):
        attempts.append((bridge._motion_dispatched, kwargs["timeout_s"]))
        bridge._sleep_for(kwargs["timeout_s"])
        raise MoonrakerError("NETWORK_ERROR", "persistent monitor connection unavailable")

    client.read_motion_status = unavailable_channel
    result = bridge._program_execute_callback(goal) if program else bridge._execute_callback(goal)

    assert result.success is False
    assert result.error_code == "NETWORK_ERROR"
    assert result.may_have_moved is False
    assert len(attempts) == 2
    assert all(dispatched is False for dispatched, _timeout in attempts)
    assert client.emergency_count == 0
    if program:
        assert client.uploads == []
        assert client.starts == []
    else:
        assert client.scripts == []


@pytest.mark.parametrize("program", [False, True])
@pytest.mark.parametrize("fault,code", [
    ("cold", "EXTRUDER_TOO_COLD"), ("flow", "EXTRUSION_OVERRIDE_CHANGED"),
])
def test_runtime_channel_preflight_checks_fresh_health_before_dispatch(bridge_type, program, fault, code):
    bridge, goal, client = (make_program_case(bridge_type) if program else make_case(bridge_type))[:3]
    changes = {"extrude_factor": 1.1} if fault == "flow" else {
        "extruder_states": {"extruder": ExtruderSnapshot(False, 25., 0., 5., 20., 50., 170.)},
    }
    client.read_motion_status = lambda _reference, **kwargs: (
        native_snapshot(**changes),
        PrintStatsSnapshot("standby", "", "", 0., 0.) if kwargs.get("include_print_stats") else None,
    )
    result = bridge._program_execute_callback(goal) if program else bridge._execute_callback(goal)

    assert result.success is False
    assert result.error_code == code
    assert result.may_have_moved is False
    assert client.emergency_count == 0
    if program:
        assert client.uploads == []
        assert client.starts == []
    else:
        assert client.scripts == []


@pytest.mark.parametrize("program", [False, True])
def test_runtime_channel_preflight_cancellation_never_writes_or_emergency_stops(bridge_type, program):
    bridge, goal, client = (make_program_case(bridge_type) if program else make_case(bridge_type))[:3]
    attempts = []

    def cancel_read(_reference, **_kwargs):
        attempts.append(True)
        goal.is_cancel_requested = True
        raise MoonrakerError("NETWORK_ERROR", "preflight status request canceled")

    client.read_motion_status = cancel_read
    result = bridge._program_execute_callback(goal) if program else bridge._execute_callback(goal)

    assert result.success is False
    assert result.may_have_moved is False
    assert goal.terminal == "canceled"
    assert attempts == [True]
    assert client.emergency_count == 0
    if program:
        assert client.uploads == []
        assert client.starts == []
    else:
        assert client.scripts == []


def test_program_requires_combined_print_status_before_upload(bridge_type):
    bridge, goal, client, _clock = make_program_case(bridge_type)
    client.read_motion_status = lambda _reference, **_kwargs: (native_snapshot(), None)

    result = bridge._program_execute_callback(goal)

    assert result.success is False
    assert result.error_code == "INVALID_RESPONSE"
    assert result.may_have_moved is False
    assert client.uploads == []
    assert client.starts == []
    assert client.emergency_count == 0


def test_flow_change_during_native_motion_triggers_confirmed_stop(bridge_type):
    bridge, goal, client = make_case(bridge_type, start_ns=0)
    goal.request.target_position = 89.
    client.read_status = lambda: native_snapshot(extrude_factor=1.1 if bridge.wait_clock >= 2. else 1.)
    result = bridge._execute_callback(goal)
    assert result.success is False
    assert result.error_code == "EXTRUSION_OVERRIDE_CHANGED"
    assert result.may_have_moved is True
    assert client.emergency_count == 1
    assert len(client.scripts) == 1


class Feedback:
    PHASE_VALIDATING = 0
    PHASE_CHECKING_READY = 1
    PHASE_DISPATCHED = 2
    PHASE_WAITING_FOR_COMPLETION = 3
    PHASE_COMPLETED = 4
    PHASE_FAILED = 5
    PHASE_CANCELING = 6


class ProgramFeedback:
    PHASE_VALIDATING = 0
    PHASE_UPLOADING = 1
    PHASE_DISPATCHED = 2
    PHASE_RUNNING = 3
    PHASE_COMPLETED = 4
    PHASE_FAILED = 5
    PHASE_CANCELING = 6


@pytest.fixture
def bridge_type(monkeypatch):
    def install(name, **attributes):
        module = ModuleType(name)
        vars(module).update(attributes)
        monkeypatch.setitem(sys.modules, name, module)

    install("rclpy")
    install("rclpy.action", ActionServer=object, CancelResponse=object, GoalResponse=object)
    install("rclpy.callback_groups", ReentrantCallbackGroup=object)
    install("rclpy.executors", ExternalShutdownException=Exception, MultiThreadedExecutor=object)
    install("rclpy.node", Node=type("Node", (), {"__init__": lambda _self, _name: None}))
    install("rcl_interfaces.msg", ParameterDescriptor=SimpleNamespace)
    install("rclpy.qos", DurabilityPolicy=object, HistoryPolicy=object,
            QoSProfile=object, ReliabilityPolicy=object)
    install("std_srvs.srv", Trigger=SimpleNamespace(Request=SimpleNamespace, Response=SimpleNamespace))
    install(
        "ur_klipper_interfaces.action",
        ManualStepperMove=SimpleNamespace(
            Goal=SimpleNamespace,
            Result=SimpleNamespace,
            Feedback=Feedback,
        ),
        ManualStepperProgram=SimpleNamespace(
            Goal=SimpleNamespace,
            Result=SimpleNamespace,
            Feedback=ProgramFeedback,
        ),
    )
    install("ur_klipper_interfaces.msg", KlipperStatus=object)
    path = Path(__file__).resolve().parents[1] / "ur_klipper_bridge" / "bridge_node.py"
    spec = importlib.util.spec_from_file_location("ur_klipper_bridge._isolated_timing_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.KlipperBridgeNode


def test_execution_protocol_marker_is_read_only_and_not_launch_overridable(bridge_type):
    class StopBeforeRuntime(Exception):
        pass

    declared = {}
    bridge = object.__new__(bridge_type)
    bridge.declare_parameter = lambda name, value, **kwargs: declared.update(
        {name: (value, kwargs)}
    )

    def stop(_name):
        raise StopBeforeRuntime()

    bridge.get_parameter = stop
    with pytest.raises(StopBeforeRuntime):
        bridge_type.__init__(bridge)
    value, options = declared["execution_protocol_version"]
    assert value == "native-extruder-program-v6"
    assert options["descriptor"].read_only is True
    assert options["ignore_override"] is True


def test_default_moonraker_url_uses_the_wired_raspberry_pi(bridge_type):
    class StopBeforeRuntime(Exception):
        pass

    declared = {}
    bridge = object.__new__(bridge_type)
    bridge.declare_parameter = lambda name, value, **kwargs: declared.update(
        {name: (value, kwargs)}
    )

    def stop(_name):
        raise StopBeforeRuntime()

    bridge.get_parameter = stop
    with pytest.raises(StopBeforeRuntime):
        bridge_type.__init__(bridge)

    assert declared["moonraker_url"][0] == "http://192.168.0.168"


def make_case(bridge_type, *, post_delay_s=0.0, start_ns=11_250_000_000,
              cancel_during_post=False):
    clock = SimpleNamespace(nanoseconds=10_000_000_000)
    goal = SimpleNamespace(
        request=SimpleNamespace(
            operation_id="test-block-2", stepper="extruder", target_position=0.3,
            speed=0.1, acceleration=20.0, initial_position=0.0, start_delay_s=0.0,
            start_time_nanoseconds=start_ns, set_position_before_move=True,
            disable_after=False,
        ),
        is_cancel_requested=False, phases=[], terminal=None,
    )
    goal.publish_feedback = lambda feedback: goal.phases.append(feedback.phase)
    goal.abort = lambda: setattr(goal, "terminal", "aborted")
    goal.canceled = lambda: setattr(goal, "terminal", "canceled")
    goal.succeed = lambda: setattr(goal, "terminal", "succeeded")

    client = SimpleNamespace(scripts=[], emergency_count=0)

    def post_gcode(script, **_kwargs):
        client.scripts.append(script)
        if len(client.scripts) == 1:
            clock.nanoseconds += round(post_delay_s * 1_000_000_000)
            if cancel_during_post:
                goal.is_cancel_requested = True

    client.post_gcode = post_gcode
    client.emergency_stop = lambda: setattr(client, "emergency_count", client.emergency_count + 1)
    client.read_status = lambda: native_snapshot()
    client.read_motion_status = lambda _reference, **_kwargs: (client.read_status(), None)
    bridge = object.__new__(bridge_type)
    bridge._state_lock = threading.RLock()
    bridge._dispatch_request_lock = threading.Lock()
    bridge._refresh_lock = threading.Lock()
    bridge._read_request_lock = threading.Lock()
    bridge._default_stepper = "extruder"
    bridge._max_abs_target = 200.0
    bridge._max_speed = 5.0
    bridge._max_acceleration = 20.0
    bridge._request_timeout_s = 5.0
    bridge._completion_timeout_margin_s = 10.0
    bridge._goal_reserved = True
    bridge._motion_dispatched = False
    bridge._emergency_sent = False
    bridge._emergency_error = ""
    bridge._client = client
    bridge._publish_status = lambda: None
    bridge.get_clock = lambda: SimpleNamespace(now=lambda: clock)
    bridge.wait_clock = 0.0
    bridge.wait_slices = []
    bridge._monotonic_now = lambda: bridge.wait_clock
    def advance_wait(seconds):
        bridge.wait_slices.append(seconds)
        bridge.wait_clock += seconds
    bridge._sleep_for = advance_wait
    bridge.logs = []
    bridge.get_logger = lambda: SimpleNamespace(info=bridge.logs.append,
                                               warning=bridge.logs.append,
                                               error=bridge.logs.append)
    return bridge, goal, client


@pytest.mark.parametrize("post_delay_s", [1.10, 1.30])
def test_http_ack_after_start_guard_never_reports_dispatched(bridge_type, post_delay_s):
    bridge, goal, client = make_case(bridge_type, post_delay_s=post_delay_s)
    result = bridge._execute_callback(goal)
    assert goal.terminal == "aborted"
    assert result.error_code == "START_DEADLINE_MISSED"
    assert result.may_have_moved is True
    assert Feedback.PHASE_DISPATCHED not in goal.phases
    assert client.emergency_count == 1
    assert len(client.scripts) == 1


def test_relative_delay_has_no_absolute_deadline_claim(bridge_type):
    bridge, goal, client = make_case(bridge_type, post_delay_s=1.30, start_ns=0)
    result = bridge._execute_callback(goal)
    assert result.success is True
    assert "G4" not in client.scripts[0]
    assert client.scripts[-1].startswith("M400\nRESTORE_GCODE_STATE")
    assert client.emergency_count == 0
    assert any("requested_start_ros_ns=0" in line for line in bridge.logs)
    assert any("not physical-start feedback" in line for line in bridge.logs)


@pytest.mark.parametrize("absolute", [False, True])
def test_callback_accepts_four_second_start_window(bridge_type, absolute):
    bridge, goal, client = make_case(bridge_type, start_ns=14_000_000_000 if absolute else 0)
    goal.request.start_delay_s = 0.0 if absolute else 4.0
    result = bridge._execute_callback(goal)
    assert result.success is True
    assert "G4 P4000" in client.scripts[0].splitlines()
    assert client.emergency_count == 0


def test_callback_rejects_unbounded_relative_delay_before_post(bridge_type):
    bridge, goal, client = make_case(bridge_type, start_ns=0)
    goal.request.start_delay_s = 5.001
    result = bridge._execute_callback(goal)
    assert result.success is False
    assert result.error_code == "INVALID_GOAL"
    assert client.scripts == []


def test_timely_ack_then_m400_is_required_before_success(bridge_type):
    bridge, goal, client = make_case(bridge_type, post_delay_s=0.10)
    result = bridge._execute_callback(goal)
    assert result.success is True
    assert goal.terminal == "succeeded"
    assert Feedback.PHASE_DISPATCHED in goal.phases
    assert client.scripts[-1].startswith("M400\nRESTORE_GCODE_STATE")
    assert client.emergency_count == 0


def test_long_move_waits_locally_so_completion_post_does_not_cross_proxy_timeout(bridge_type):
    bridge, goal, client = make_case(bridge_type, start_ns=0)
    goal.request.target_position = 89.0
    goal.request.speed = 1.0
    goal.request.acceleration = 20.0
    barrier_posted_at = []
    original_post = client.post_gcode

    def proxy_limited_post(script, **kwargs):
        if script.startswith("M400\nRESTORE_GCODE_STATE"):
            barrier_posted_at.append(bridge.wait_clock)
            # Model the port-80 reverse proxy: a barrier started at dispatch
            # would span >60 s and fail, while a near-terminal barrier returns.
            if bridge.wait_clock < 88.0:
                raise MoonrakerError("HTTP_ERROR", "Moonraker returned HTTP 504")
        original_post(script, **kwargs)

    client.post_gcode = proxy_limited_post
    result = bridge._execute_callback(goal)

    assert result.success is True
    assert goal.terminal == "succeeded"
    assert barrier_posted_at and barrier_posted_at[0] >= result.estimated_duration_s
    assert max(bridge.wait_slices) <= 0.1
    assert client.scripts[-1].startswith("M400\nRESTORE_GCODE_STATE")


def test_cancel_during_local_completion_wait_never_posts_barrier(bridge_type):
    bridge, goal, client = make_case(bridge_type, start_ns=0)
    goal.request.target_position = 89.0
    goal.request.speed = 1.0
    goal.request.acceleration = 20.0
    original_sleep = bridge._sleep_for

    def cancel_after_dispatch(seconds):
        original_sleep(seconds)
        if bridge.wait_clock >= 1.0:
            goal.is_cancel_requested = True

    bridge._sleep_for = cancel_after_dispatch
    result = bridge._execute_callback(goal)

    assert result.success is False
    assert goal.terminal == "canceled"
    assert client.emergency_count == 1
    assert len(client.scripts) == 1
    assert not any(script.startswith("M400\nRESTORE_GCODE_STATE") for script in client.scripts)


def test_klipper_readiness_loss_during_local_wait_fails_closed(bridge_type):
    bridge, goal, client = make_case(bridge_type, start_ns=0)
    goal.request.target_position = 89.0
    goal.request.speed = 1.0
    goal.request.acceleration = 20.0
    ready = native_snapshot()
    lost = KlipperSnapshot(False, False, "", "connection lost", ())
    client.read_status = lambda: lost if bridge.wait_clock >= 2.0 else ready

    result = bridge._execute_callback(goal)

    assert result.success is False
    assert result.error_code == "KLIPPER_NOT_READY"
    assert goal.terminal == "aborted"
    assert client.emergency_count == 1
    assert len(client.scripts) == 1
    assert not any(script.startswith("M400\nRESTORE_GCODE_STATE") for script in client.scripts)


def test_expired_absolute_deadline_never_posts_a_move(bridge_type):
    bridge, goal, client = make_case(bridge_type, start_ns=10_100_000_000)
    result = bridge._execute_callback(goal)
    assert result.success is False
    assert result.may_have_moved is False
    assert client.scripts == []
    assert client.emergency_count == 0


def test_cancel_during_http_never_reports_dispatched(bridge_type):
    bridge, goal, client = make_case(bridge_type, cancel_during_post=True)
    result = bridge._execute_callback(goal)
    assert goal.terminal == "canceled"
    assert result.may_have_moved is True
    assert Feedback.PHASE_DISPATCHED not in goal.phases
    assert client.emergency_count == 1
    assert len(client.scripts) == 1


def test_completion_barrier_timeout_is_never_reported_as_success(bridge_type):
    bridge, goal, client = make_case(bridge_type)
    original_post = client.post_gcode

    def fail_barrier(script, **kwargs):
        original_post(script, **kwargs)
        if script.startswith("M400\nRESTORE_GCODE_STATE"):
            raise MoonrakerError("NETWORK_ERROR", "completion acknowledgement timed out")

    client.post_gcode = fail_barrier
    result = bridge._execute_callback(goal)
    assert result.success is False
    assert result.may_have_moved is True
    assert result.error_code == "NETWORK_ERROR"
    assert goal.terminal == "aborted"
    assert Feedback.PHASE_COMPLETED not in goal.phases
    assert client.emergency_count == 1


def make_program_case(bridge_type):
    clock = SimpleNamespace(nanoseconds=20_000_000_000)
    goal = SimpleNamespace(
        request=SimpleNamespace(
            operation_id="full-job-1",
            stepper="extruder",
            segment_units=[1.0, 0.5, 0.0, -0.25],
            segment_durations_s=[2.0, 1.0, 1.25, 0.5],
            maximum_speed=5.0,
            start_delay_s=4.0,
            disable_after=True,
        ),
        is_cancel_requested=False,
        feedback=[],
        terminal=None,
    )
    goal.publish_feedback = lambda message: goal.feedback.append(message)
    goal.abort = lambda: setattr(goal, "terminal", "aborted")
    goal.canceled = lambda: setattr(goal, "terminal", "canceled")
    goal.succeed = lambda: setattr(goal, "terminal", "succeeded")

    client = SimpleNamespace(
        uploads=[], starts=[], emergency_count=0, state_reads=0,
    )
    client.read_status = lambda: native_snapshot()
    client.upload_gcode = lambda filename, contents, **_kwargs: (
        client.uploads.append((filename, contents)) or filename
    )
    client.start_print = lambda filename: client.starts.append(filename)

    def print_state():
        client.state_reads += 1
        state = ("standby", "printing", "complete")[
            min(client.state_reads - 1, 2)
        ]
        return PrintStatsSnapshot(
            state, client.starts[0] if client.starts else "", "", 4.0, 4.0
        )

    client.read_print_state = print_state
    client.read_motion_status = lambda _reference, **kwargs: (
        client.read_status(),
        client.read_print_state() if kwargs.get("include_print_stats") else None,
    )
    client.emergency_stop = lambda: setattr(
        client, "emergency_count", client.emergency_count + 1
    )

    bridge = object.__new__(bridge_type)
    bridge._state_lock = threading.RLock()
    bridge._dispatch_request_lock = threading.Lock()
    bridge._refresh_lock = threading.Lock()
    bridge._read_request_lock = threading.Lock()
    bridge._default_stepper = "extruder"
    bridge._max_abs_target = 200.0
    bridge._max_speed = 5.0
    bridge._max_acceleration = 20.0
    bridge._request_timeout_s = 5.0
    bridge._completion_timeout_margin_s = 10.0
    bridge._goal_reserved = True
    bridge._motion_dispatched = False
    bridge._emergency_sent = False
    bridge._emergency_error = ""
    bridge._client = client
    bridge._publish_status = lambda: None
    bridge.get_clock = lambda: SimpleNamespace(now=lambda: clock)
    bridge._monotonic_now = lambda: clock.nanoseconds / 1e9
    bridge._sleep_for = lambda seconds: setattr(
        clock, "nanoseconds", clock.nanoseconds + round(seconds * 1e9)
    )
    bridge.get_logger = lambda: SimpleNamespace(
        info=lambda *_args: None,
        warning=lambda *_args: None,
        error=lambda *_args: None,
    )

    return bridge, goal, client, clock


def test_continuous_program_uploads_and_starts_once_without_block_posts(bridge_type):
    bridge, goal, client, clock = make_program_case(bridge_type)
    epoch = clock.nanoseconds
    result = bridge._program_execute_callback(goal)

    assert result.success is True
    assert goal.terminal == "succeeded"
    assert len(client.uploads) == 1
    assert client.starts == [client.uploads[0][0]]
    program = client.uploads[0][1]
    assert program.count("G4 P4000") == 1
    assert program.count("M400") == 1
    assert "/printer/gcode/script" not in program
    dispatched = [
        item for item in goal.feedback
        if item.phase == ProgramFeedback.PHASE_DISPATCHED
    ]
    assert len(dispatched) == 1
    assert dispatched[0].program_epoch_ros_ns == epoch
    assert client.emergency_count == 0


def test_program_cancel_after_start_uses_emergency_stop(bridge_type):
    bridge, goal, client, _clock = make_program_case(bridge_type)
    def start(filename):
        client.starts.append(filename)
        goal.is_cancel_requested = True
    client.start_print = start
    result = bridge._program_execute_callback(goal)
    assert not result.success
    assert result.may_have_moved
    assert goal.terminal == "canceled"
    assert len(client.starts) == 1
    assert client.emergency_count == 1


def test_program_virtual_sd_busy_does_not_upload_or_start(bridge_type):
    bridge, goal, client, _clock = make_program_case(bridge_type)
    client.read_print_state = lambda: PrintStatsSnapshot(
        "printing", "other.gcode", "", 4.0, 4.0
    )
    result = bridge._program_execute_callback(goal)
    assert not result.success
    assert result.error_code == "VIRTUAL_SD_BUSY"
    assert not result.may_have_moved
    assert not client.uploads and not client.starts
    assert client.emergency_count == 0


def test_program_upload_failure_is_pre_dispatch_and_never_emergency_stops(bridge_type):
    bridge, goal, client, _clock = make_program_case(bridge_type)

    def fail_upload(filename, contents, **_kwargs):
        client.uploads.append((filename, contents))
        raise MoonrakerError("NETWORK_ERROR", "upload acknowledgement failed")

    client.upload_gcode = fail_upload
    result = bridge._program_execute_callback(goal)

    assert not result.success
    assert result.error_code == "NETWORK_ERROR"
    assert result.may_have_moved is False
    assert goal.terminal == "aborted"
    assert len(client.uploads) == 1
    assert client.starts == []
    assert client.emergency_count == 0


def test_program_recovers_one_transient_print_stats_timeout_after_running(bridge_type):
    bridge, goal, client, _clock = make_program_case(bridge_type)
    read_times = []
    states = iter(
        (
            PrintStatsSnapshot("standby", "", "", 0.0, 0.0),
            PrintStatsSnapshot("printing", "", "", 4.0, 4.0),
            MoonrakerError(
                "NETWORK_ERROR",
                "Moonraker request failed for /printer/objects/query?print_stats: "
                "<urlopen error timed out>",
            ),
            PrintStatsSnapshot("printing", "", "", 4.1, 4.1),
            PrintStatsSnapshot("complete", "", "", 4.2, 4.2),
        )
    )

    def print_state():
        read_times.append(bridge._monotonic_now())
        state = next(states)
        if isinstance(state, MoonrakerError):
            raise state
        filename = client.starts[0] if client.starts else ""
        return PrintStatsSnapshot(
            state.state,
            filename,
            state.message,
            state.print_duration_s,
            state.total_duration_s,
        )

    client.read_print_state = print_state
    result = bridge._program_execute_callback(goal)

    assert result.success is True
    assert goal.terminal == "succeeded"
    assert len(client.starts) == 1
    assert client.emergency_count == 0
    # The first runtime check occurs immediately, normal monitoring is 1 Hz,
    # and only the explicit transient-error retry is closer together.
    assert read_times[2] - read_times[1] == pytest.approx(1.0)
    assert read_times[3] - read_times[2] == pytest.approx(0.25)


def test_program_fails_closed_after_print_stats_retry_is_exhausted(bridge_type):
    bridge, goal, client, _clock = make_program_case(bridge_type)
    states = iter(
        (
            PrintStatsSnapshot("standby", "", "", 0.0, 0.0),
            PrintStatsSnapshot("printing", "", "", 4.0, 4.0),
            MoonrakerError("NETWORK_ERROR", "first print_stats timeout"),
            MoonrakerError("NETWORK_ERROR", "second print_stats timeout"),
        )
    )

    def print_state():
        state = next(states)
        if isinstance(state, MoonrakerError):
            raise state
        return PrintStatsSnapshot(
            state.state,
            client.starts[0] if client.starts else "",
            state.message,
            state.print_duration_s,
            state.total_duration_s,
        )

    client.read_print_state = print_state
    result = bridge._program_execute_callback(goal)

    assert result.success is False
    assert result.error_code == "NETWORK_ERROR"
    assert result.may_have_moved is True
    assert goal.terminal == "aborted"
    assert len(client.starts) == 1
    assert client.emergency_count == 1


def test_emergency_stop_timeout_is_confirmed_only_by_shutdown_state(bridge_type):
    bridge, _goal, client, _clock = make_program_case(bridge_type)
    bridge._motion_dispatched = True
    logs = []
    bridge.get_logger = lambda: SimpleNamespace(
        info=lambda *_args: None,
        warning=lambda *_args: None,
        error=logs.append,
    )

    def timeout():
        client.emergency_count += 1
        raise MoonrakerError("NETWORK_ERROR", "Moonraker emergency-stop response timed out")

    client.emergency_stop = timeout
    client.read_printer_state = lambda **_kwargs: PrinterStateSnapshot(
        "shutdown", "Shutdown due to webhooks request"
    )

    error = bridge._emergency_stop_once("test lost acknowledgement")

    assert error == ""
    assert client.emergency_count == 1
    assert any("confirmed via /printer/info state=shutdown" in line for line in logs)


def test_emergency_service_waits_for_delayed_shutdown_after_timeout(bridge_type):
    bridge, _goal, client, _clock = make_program_case(bridge_type)
    states = iter(
        (
            PrinterStateSnapshot("ready", "Printer is ready"),
            PrinterStateSnapshot("shutdown", "Shutdown due to webhooks request"),
        )
    )

    def timeout():
        client.emergency_count += 1
        raise MoonrakerError("NETWORK_ERROR", "Moonraker emergency-stop response timed out")

    client.emergency_stop = timeout
    client.read_printer_state = lambda **_kwargs: next(states)
    response = SimpleNamespace()

    bridge._emergency_service_callback(None, response)

    assert response.success is True
    assert "confirmed via /printer/info state=shutdown" in response.message
    assert client.emergency_count == 1


def test_emergency_service_does_not_treat_ready_as_a_confirmed_stop(bridge_type):
    bridge, _goal, client, _clock = make_program_case(bridge_type)

    def timeout():
        client.emergency_count += 1
        raise MoonrakerError("NETWORK_ERROR", "Moonraker emergency-stop response timed out")

    client.emergency_stop = timeout
    client.read_printer_state = lambda **_kwargs: PrinterStateSnapshot(
        "ready", "Printer is ready"
    )
    response = SimpleNamespace()

    bridge._emergency_service_callback(None, response)

    assert response.success is False
    assert "state=ready" in response.message
    assert client.emergency_count == 1


def test_status_timer_does_not_overlap_an_active_action_read(bridge_type):
    bridge, _goal, client, _clock = make_program_case(bridge_type)
    reads = []
    client.read_status = lambda: reads.append(True) or native_snapshot()

    bridge._poll_status_callback()
    assert reads == []

    bridge._goal_reserved = False
    bridge._poll_status_callback()
    assert reads == [True]


def test_program_lost_identity_fails_closed_after_start(bridge_type):
    bridge, goal, client, _clock = make_program_case(bridge_type)
    client.read_print_state = lambda: PrintStatsSnapshot(
        "printing" if client.starts else "standby", "other.gcode", "", 4.0, 4.0
    )
    result = bridge._program_execute_callback(goal)
    assert not result.success
    assert result.error_code == "PRINT_IDENTITY_CHANGED"
    assert result.may_have_moved
    assert goal.terminal == "aborted"
    assert client.emergency_count == 1


def test_program_stale_same_filename_completion_is_not_a_false_success(bridge_type):
    bridge, goal, client, _clock = make_program_case(bridge_type)
    states = iter(("standby", "complete", "printing", "complete"))
    client.read_print_state = lambda: PrintStatsSnapshot(
        next(states), client.starts[0] if client.starts else "", "", 4.0, 4.0
    )
    result = bridge._program_execute_callback(goal)
    assert result.success
    assert goal.terminal == "succeeded"
    assert len(client.starts) == 1
    assert client.emergency_count == 0


def runtime_monitor_case(bridge_type, program):
    """Keep transport accounting separate from the legacy fixture readers."""
    if program:
        bridge, goal, client, _clock = make_program_case(bridge_type)
    else:
        bridge, goal, client = make_case(bridge_type, start_ns=0)
        goal.request.target_position = 89.0
        goal.request.speed = 1.0
    bridge._request_timeout_s = 8.0
    bridge.get_logger = lambda: SimpleNamespace(
        info=lambda *_args: None, warning=lambda *_args: None,
        error=lambda *_args: None,
    )
    client.full_reads = 0
    client.runtime_reads = []
    client.stats_reads = 0

    def preflight_status():
        client.full_reads += 1
        if client.full_reads > 1:
            raise MoonrakerError(
                "NETWORK_ERROR",
                "Moonraker request failed for /printer/objects/query?"
                "configfile&gcode_move&toolhead: <urlopen error timed out>",
            )
        return native_snapshot()

    def preflight_stats():
        client.stats_reads += 1
        assert not client.starts, "the running program must use one combined query"
        return PrintStatsSnapshot("standby", "", "", 0.0, 0.0)

    client.read_status = preflight_status
    if program:
        client.read_print_state = preflight_stats

    def invoke():
        # Faults in this fixture describe monitoring after a successful
        # compact-channel preflight. Dedicated preflight tests inject faults
        # before dispatch and require zero writes and zero emergency stops.
        runtime_reader = client.read_motion_status

        def read_after_preflight(reference, **kwargs):
            if not bridge._motion_dispatched:
                assert kwargs.get("include_print_stats", False) is program
                return native_snapshot(), preflight_stats() if program else None
            return runtime_reader(reference, **kwargs)

        client.read_motion_status = read_after_preflight
        return (bridge._program_execute_callback(goal) if program
                else bridge._execute_callback(goal))

    def read_motion_status(reference, *, include_print_stats=False, timeout_s=None):
        assert reference.extruders == ("extruder",)
        assert include_print_stats is program
        client.runtime_reads.append((bridge._monotonic_now(), timeout_s))
        stats = None
        if program:
            stats = PrintStatsSnapshot(
                "complete" if len(client.runtime_reads) >= 3 else "printing",
                client.starts[0], "", 4.0, 4.0,
            )
        return native_snapshot(), stats

    client.read_motion_status = read_motion_status
    return bridge, goal, client, invoke


@pytest.mark.parametrize("program", [False, True])
def test_runtime_monitor_uses_light_combined_query_without_configfile_reads(bridge_type, program):
    bridge, goal, client, invoke = runtime_monitor_case(bridge_type, program)

    result = invoke()

    assert result.success is True
    assert goal.terminal == "succeeded"
    assert client.full_reads == 1
    assert len(client.runtime_reads) >= 2
    assert all(0.0 < timeout <= 6.0 for _time, timeout in client.runtime_reads)
    if program:
        assert client.stats_reads == 1
        assert client.runtime_reads[1][0] - client.runtime_reads[0][0] == pytest.approx(1.0)
    assert client.emergency_count == 0


@pytest.mark.parametrize("program", [False, True])
def test_runtime_monitor_recovers_one_timeout_within_one_eight_second_budget(bridge_type, program):
    bridge, goal, client, invoke = runtime_monitor_case(bridge_type, program)
    healthy_read = client.read_motion_status

    def transient_read(reference, **kwargs):
        if not client.runtime_reads:
            client.runtime_reads.append((bridge._monotonic_now(), kwargs["timeout_s"]))
            bridge._sleep_for(kwargs["timeout_s"])
            raise MoonrakerError("NETWORK_ERROR", "one status response timed out")
        return healthy_read(reference, **kwargs)

    client.read_motion_status = transient_read
    result = invoke()

    assert result.success is True
    assert goal.terminal == "succeeded"
    assert client.full_reads == 1
    first, retry = client.runtime_reads[:2]
    assert first[1] == pytest.approx(6.0)
    assert retry[0] - first[0] == pytest.approx(6.25)
    assert retry[1] == pytest.approx(1.75)
    assert client.emergency_count == 0


@pytest.mark.parametrize("program", [False, True])
def test_runtime_monitor_persistent_timeout_aborts_once_at_the_shared_deadline(bridge_type, program):
    bridge, goal, client, invoke = runtime_monitor_case(bridge_type, program)

    def failed_read(reference, **kwargs):
        client.runtime_reads.append((bridge._monotonic_now(), kwargs["timeout_s"]))
        bridge._sleep_for(kwargs["timeout_s"])
        raise MoonrakerError("NETWORK_ERROR", "runtime status remains unavailable")

    client.read_motion_status = failed_read
    result = invoke()

    assert result.success is False
    assert result.error_code == "NETWORK_ERROR"
    assert result.may_have_moved is True
    assert goal.terminal == "aborted"
    assert bridge._snapshot.reachable is False
    assert bridge._snapshot.ready is False
    assert len(client.runtime_reads) == 2
    assert bridge._monotonic_now() - client.runtime_reads[0][0] == pytest.approx(8.0)
    assert client.emergency_count == 1
    assert Feedback.PHASE_COMPLETED not in getattr(goal, "phases", [])
    assert not any(item.phase == ProgramFeedback.PHASE_COMPLETED
                   for item in getattr(goal, "feedback", []))


@pytest.mark.parametrize("program", [False, True])
@pytest.mark.parametrize("configured_timeout", [2.0, 5.0, 20.0])
def test_runtime_monitor_honors_shorter_timeout_and_caps_longer_setting(bridge_type, program, configured_timeout):
    bridge, goal, client, invoke = runtime_monitor_case(bridge_type, program)
    bridge._request_timeout_s = configured_timeout

    def failed_read(reference, **kwargs):
        client.runtime_reads.append((bridge._monotonic_now(), kwargs["timeout_s"]))
        bridge._sleep_for(kwargs["timeout_s"])
        raise MoonrakerError("NETWORK_ERROR", "runtime status remains unavailable")

    client.read_motion_status = failed_read
    result = invoke()

    assert result.success is False
    assert result.error_code == "NETWORK_ERROR"
    assert goal.terminal == "aborted"
    assert bridge._snapshot.reachable is False
    assert bridge._snapshot.ready is False
    assert len(client.runtime_reads) == 2
    assert bridge._monotonic_now() - client.runtime_reads[0][0] == pytest.approx(
        min(8.0, configured_timeout)
    )
    assert client.emergency_count == 1


@pytest.mark.parametrize("program", [False, True])
@pytest.mark.parametrize("fault,code", [
    ("shutdown", "KLIPPER_NOT_READY"), ("cold", "EXTRUDER_TOO_COLD"),
])
def test_runtime_monitor_validates_the_retry_response_instead_of_reusing_preflight(bridge_type, program, fault, code):
    bridge, goal, client, invoke = runtime_monitor_case(bridge_type, program)
    healthy_read = client.read_motion_status

    def transient_then_unsafe(reference, **kwargs):
        if not client.runtime_reads:
            client.runtime_reads.append((bridge._monotonic_now(), kwargs["timeout_s"]))
            raise MoonrakerError("NETWORK_ERROR", "one lost status response")
        _snapshot, stats = healthy_read(reference, **kwargs)
        if fault == "shutdown":
            return native_snapshot(ready=False, printer_state="shutdown"), stats
        return native_snapshot(extruder_states={
            "extruder": ExtruderSnapshot(False, 25.0, 0.0, 5.0, 20.0, 50.0, 170.0),
        }), stats

    client.read_motion_status = transient_then_unsafe
    result = invoke()

    assert result.success is False
    assert result.error_code == code
    assert goal.terminal == "aborted"
    assert len(client.runtime_reads) == 2
    assert client.emergency_count == 1


@pytest.mark.parametrize("program", [False, True])
@pytest.mark.parametrize("fault,code", [
    ("shutdown", "KLIPPER_NOT_READY"),
    ("cold", "EXTRUDER_TOO_COLD"),
    ("flow", "EXTRUSION_OVERRIDE_CHANGED"),
    ("acceleration", "EXTRUSION_LIMIT_MISMATCH"),
    ("malformed", "INVALID_RESPONSE"),
])
def test_runtime_monitor_never_retries_explicit_unsafe_or_malformed_state(bridge_type, program, fault, code):
    bridge, goal, client, invoke = runtime_monitor_case(bridge_type, program)
    healthy_read = client.read_motion_status

    def unsafe_read(reference, **kwargs):
        snapshot, stats = healthy_read(reference, **kwargs)
        if fault == "malformed":
            raise MoonrakerError("INVALID_RESPONSE", "missing heater permission")
        if fault == "shutdown":
            snapshot = native_snapshot(ready=False, printer_state="shutdown", detail="heater fault")
        elif fault == "cold":
            snapshot = native_snapshot(extruder_states={
                "extruder": ExtruderSnapshot(False, 25.0, 0.0, 5.0, 20.0, 50.0, 170.0),
            })
        elif fault == "flow":
            snapshot = native_snapshot(extrude_factor=1.1)
        else:
            snapshot = native_snapshot(extruder_states={
                "extruder": ExtruderSnapshot(True, 200.0, 200.0, 5.0, 19.0, 50.0, 170.0),
            })
        return snapshot, stats

    client.read_motion_status = unsafe_read
    result = invoke()

    assert result.success is False
    assert result.error_code == code
    assert result.may_have_moved is True
    assert goal.terminal == "aborted"
    assert len(client.runtime_reads) == 1
    assert client.emergency_count == 1
    if fault == "malformed":
        assert bridge._snapshot.reachable is False
        assert bridge._snapshot.ready is False


@pytest.mark.parametrize("program", [False, True])
@pytest.mark.parametrize("cancel_during_delay", [False, True])
def test_runtime_monitor_cancel_between_retry_attempts_prevents_another_get(bridge_type, program, cancel_during_delay):
    bridge, goal, client, invoke = runtime_monitor_case(bridge_type, program)
    original_sleep = bridge._sleep_for

    def failed_read(reference, **kwargs):
        client.runtime_reads.append((bridge._monotonic_now(), kwargs["timeout_s"]))
        if not cancel_during_delay:
            goal.is_cancel_requested = True
        raise MoonrakerError("NETWORK_ERROR", "the first runtime GET timed out")

    def cancelable_sleep(seconds):
        original_sleep(seconds)
        if client.runtime_reads:
            goal.is_cancel_requested = True

    bridge._sleep_for = cancelable_sleep
    client.read_motion_status = failed_read
    result = invoke()

    assert result.success is False
    assert result.may_have_moved is True
    assert goal.terminal == "canceled"
    assert len(client.runtime_reads) == 1
    assert client.emergency_count == 1


@pytest.mark.parametrize("program", [False, True])
@pytest.mark.parametrize("late_on_retry", [False, True])
def test_runtime_monitor_discards_healthy_response_that_arrives_after_read_budget(bridge_type, program, late_on_retry):
    bridge, goal, client, invoke = runtime_monitor_case(bridge_type, program)
    healthy_read = client.read_motion_status
    sample_started = []

    def late_read(reference, **kwargs):
        if not sample_started:
            sample_started.append(bridge._monotonic_now())
        if late_on_retry and not client.runtime_reads:
            client.runtime_reads.append((bridge._monotonic_now(), kwargs["timeout_s"]))
            bridge._sleep_for(kwargs["timeout_s"])
            raise MoonrakerError("NETWORK_ERROR", "first query timed out")
        bridge._sleep_for(sample_started[0] + 8.01 - bridge._monotonic_now())
        return healthy_read(reference, **kwargs)

    client.read_motion_status = late_read
    result = invoke()

    assert result.success is False
    assert result.error_code == "NETWORK_ERROR"
    assert goal.terminal == "aborted"
    assert bridge._snapshot.reachable is False
    assert bridge._snapshot.ready is False
    assert len(client.runtime_reads) == (2 if late_on_retry else 1)
    assert client.emergency_count == 1
    assert not any(item.phase == ProgramFeedback.PHASE_COMPLETED
                   for item in getattr(goal, "feedback", []))
    if program:
        assert len(client.uploads) == len(client.starts) == 1
    else:
        assert len(client.scripts) == 1


@pytest.mark.parametrize("unsafe_completion", ["cold", "wrong_file", "error", "none"])
def test_program_checks_fresh_health_and_identity_on_completion_response_after_retry(bridge_type, unsafe_completion):
    bridge, goal, client, invoke = runtime_monitor_case(bridge_type, True)
    healthy_read = client.read_motion_status

    def completion_read(reference, **kwargs):
        if len(client.runtime_reads) == 1:
            client.runtime_reads.append((bridge._monotonic_now(), kwargs["timeout_s"]))
            raise MoonrakerError("NETWORK_ERROR", "completion read timed out once")
        snapshot, stats = healthy_read(reference, **kwargs)
        if len(client.runtime_reads) == 3:
            assert stats.state == "complete"
            if unsafe_completion == "cold":
                snapshot = native_snapshot(extruder_states={
                    "extruder": ExtruderSnapshot(False, 25.0, 0.0, 5.0, 20.0, 50.0, 170.0),
                })
            elif unsafe_completion == "wrong_file":
                stats = PrintStatsSnapshot("complete", "other.gcode", "", 4.0, 4.0)
            elif unsafe_completion == "error":
                stats = PrintStatsSnapshot("error", stats.filename, "heater fault", 4.0, 4.0)
            else:
                stats = None
        return snapshot, stats

    client.read_motion_status = completion_read
    result = invoke()

    assert result.success is False
    assert result.error_code == {
        "cold": "EXTRUDER_TOO_COLD", "wrong_file": "PRINT_IDENTITY_CHANGED",
        "error": "PRINT_FAILED", "none": "INVALID_RESPONSE",
    }[unsafe_completion]
    assert goal.terminal == "aborted"
    assert len(client.runtime_reads) == 3
    assert len(client.uploads) == len(client.starts) == 1
    assert client.emergency_count == 1
    assert not any(item.phase == ProgramFeedback.PHASE_COMPLETED for item in goal.feedback)


def test_full_status_refresh_service_does_not_query_config_during_an_active_action(bridge_type):
    bridge, _goal, client, _clock = make_program_case(bridge_type)
    reads = []
    client.read_status = lambda: reads.append(True) or native_snapshot()
    response = SimpleNamespace()

    returned = bridge._refresh_service_callback(None, response)

    assert returned is response
    assert response.success is False
    assert "active action" in response.message.lower()
    assert reads == []


def test_full_status_refresh_service_reads_fresh_config_when_idle(bridge_type):
    bridge, _goal, client, _clock = make_program_case(bridge_type)
    bridge._goal_reserved = False
    reads = []
    client.read_status = lambda: reads.append(True) or native_snapshot()
    response = SimpleNamespace()

    returned = bridge._refresh_service_callback(None, response)

    assert returned is response
    assert response.success is True
    assert reads == [True]


@pytest.mark.parametrize("service", [False, True])
def test_idle_refresh_queued_before_goal_reservation_does_not_query_during_motion(bridge_type, service):
    bridge, _goal, client, _clock = make_program_case(bridge_type)
    bridge._goal_reserved = False
    bridge._snapshot = native_snapshot()
    reads, errors = [], []
    client.read_status = lambda: reads.append(True) or native_snapshot()
    queued = threading.Event()
    refresh_lock = threading.Lock()
    response = SimpleNamespace()

    class ObservedRefreshLock:
        def __enter__(self):
            queued.set()
            refresh_lock.acquire()

        def __exit__(self, *_args):
            refresh_lock.release()

    bridge._refresh_lock = ObservedRefreshLock()

    def queued_refresh():
        try:
            if service:
                bridge._refresh_service_callback(None, response)
            else:
                bridge._poll_status_callback()
        except BaseException as exc:
            errors.append(exc)

    refresh_lock.acquire()
    worker = threading.Thread(target=queued_refresh, daemon=True)
    worker.start()
    try:
        assert queued.wait(1.0), "the idle refresh did not reach its serialization lock"
        with bridge._state_lock:
            bridge._goal_reserved = True
    finally:
        refresh_lock.release()
        worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert errors == []
    assert reads == []
    if service:
        assert response.success is False
        assert "active action" in response.message.lower()


def test_action_preflight_can_refresh_config_after_reserving_its_goal(bridge_type):
    bridge, _goal, client, _clock = make_program_case(bridge_type)
    reads = []
    client.read_status = lambda: reads.append(True) or native_snapshot()

    snapshot = bridge._refresh_snapshot()

    assert snapshot.ready is True
    assert reads == [True]


@pytest.mark.parametrize("program", [False, True])
def test_runtime_monitor_lock_wait_counts_toward_the_same_read_budget(bridge_type, program):
    bridge, goal, client, invoke = runtime_monitor_case(bridge_type, program)
    lock_waits = []

    class UnavailableReadLock:
        def acquire(self, *, timeout):
            lock_waits.append((bridge._monotonic_now(), timeout))
            bridge._sleep_for(timeout)
            return False

        def release(self):
            pytest.fail("an unacquired read lock cannot be released")

    if program:
        original_start = client.start_print

        def start(filename):
            original_start(filename)
            bridge._read_request_lock = UnavailableReadLock()

        client.start_print = start
    else:
        original_post = client.post_gcode

        def post(script, **kwargs):
            original_post(script, **kwargs)
            bridge._read_request_lock = UnavailableReadLock()

        client.post_gcode = post

    result = invoke()

    assert result.success is False
    assert result.error_code == "NETWORK_ERROR"
    assert goal.terminal == "aborted"
    assert client.runtime_reads == []
    assert len(lock_waits) == 1
    assert bridge._monotonic_now() - lock_waits[0][0] == pytest.approx(8.0)
    assert bridge._snapshot.reachable is False
    assert client.emergency_count == 1
