from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import threading
import unittest
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from unittest import mock


SOFTWARE_DIR = Path(__file__).resolve().parents[1]


def load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


execution = load("print_execution", SOFTWARE_DIR / "print_execution.py")
runner = load(
    "gcode_print_executor",
    SOFTWARE_DIR.parent / "workspaces" / "ur_ros2_ws" / "scripts" / "gcode_print_executor.py",
)
cancel_client = load(
    "synchronized_motion_cancel",
    SOFTWARE_DIR.parent
    / "workspaces"
    / "ur_ros2_ws"
    / "scripts"
    / "synchronized_motion_cancel.py",
)


JOINTS = execution.UR_JOINTS
POLICY = "continuous-cartesian-approach-v4"


def write_request(root: Path) -> Path:
    path = root / "planning.request.json"
    path.write_text(json.dumps({
        "review_sha256": "a" * 64, "review_id": "b" * 12,
        "planning_policy_version": POLICY,
        "printhead_geometry_sha256": execution.printhead_geometry_sha256(),
    }), encoding="utf-8")
    return path


def write_trajectory(path: Path, *, distance: float = 0.03) -> None:
    path.write_text(
        "time_s,path_progress," + ",".join(JOINTS) + "\n"
        + "0.000000000,0.000000000," + ",".join("0" for _ in JOINTS) + "\n"
        + "1.000000000,1.000000000," + ",".join([str(distance), "0", "0", "0", "0", "0"]) + "\n",
        encoding="utf-8",
    )


def review() -> dict:
    return {
        "planning_policy_version": POLICY,
        "printhead_geometry_sha256": execution.printhead_geometry_sha256(),
        "review_sha256": "a" * 64,
        "profile_sha256": "b" * 64,
        "source_sha256": "c" * 64,
        "source": {"name": "test.gcode", "sha256": "c" * 64},
        "print_bed_m": {
            "plane_z": -0.014,
            "minimum": {"x": -0.5, "y": -0.5},
            "maximum": {"x": 0.5, "y": 0.5},
            "edge_margin_m": 0.01,
        },
        "print_bed_check": {"within_xy": True, "surface_aligned": True},
        "calibration": {
            "tool_center_point_m": {
                "offset": {"x": 0.0, "y": 0.0, "z": 0.08},
                "collision_radius_m": 0.01,
            }
        },
        "path": {
            "sampled": False,
            "continuous": True,
            "segments": [
                {
                    "sequence": 1,
                    "line": 1,
                    "layer": 0,
                    "x0": 0.0,
                    "y0": 0.0,
                    "z0": 0.0,
                    "x1": 0.01,
                    "y1": 0.0,
                    "z1": 0.0,
                },
                {
                    "sequence": 2,
                    "line": 2,
                    "layer": 0,
                    "x0": 0.01,
                    "y0": 0.0,
                    "z0": 0.0,
                    "x1": 0.02,
                    "y1": 0.0,
                    "z1": 0.0,
                },
            ],
        },
        "timeline": {
            "sampled": False,
            "events": [
                {
                    "sequence": 1,
                    "line": 1,
                    "duration_s": 1.0,
                    "ur_distance_mm": 10.0,
                    "klipper_units": 0.0,
                    "klipper_action": "idle",
                },
                {
                    "sequence": 2,
                    "line": 2,
                    "duration_s": 1.0,
                    "ur_distance_mm": 10.0,
                    "klipper_units": 4.0,
                    "klipper_action": "extrude",
                },
                {
                    "sequence": 3,
                    "line": 3,
                    "duration_s": 0.4,
                    "ur_distance_mm": 0.0,
                    "klipper_units": -0.5,
                    "klipper_action": "retract",
                },
            ],
        },
    }


def moveit(*, simulation: bool = False) -> dict:
    return {
        "planning_policy_version": POLICY,
        "printhead_geometry_sha256": execution.printhead_geometry_sha256(),
        "planning_request_id": "b" * 12,
        "approach_status": "passed",
        "interpolation_validation": "passed",
        "status": "passed",
        "simulation": simulation,
        "review_sha256": "a" * 64,
        "collision_free": True,
        "cartesian_fraction": 1.0,
        "joint_limits": "passed",
    }


class PrintExecutionTests(unittest.TestCase):
    def test_planning_policy_matches_ros_executor_and_producer(self) -> None:
        self.assertEqual(execution.PLANNING_POLICY_VERSION, runner.PLANNING_POLICY_VERSION)
        for path in (
            SOFTWARE_DIR / "app" / "app.js",
            SOFTWARE_DIR.parent / "workspaces" / "ur_ros2_ws" / "src" / "ur3e_print_planner" / "src" / "gcode_path_review.cpp",
        ):
            self.assertIn(f'"{execution.PLANNING_POLICY_VERSION}"', path.read_text(encoding="utf-8"))

    def test_obsolete_policy_and_discontinuous_branch_are_execution_locked(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            approach, printing = root / "approach.csv", root / "print.csv"
            write_trajectory(approach)
            write_trajectory(printing)
            common = dict(
                review=review(), moveit_result=moveit(),
                approach_trajectory_path=approach, print_trajectory_path=printing,
                destination=root / "manifest.json", manual_stepper="extruder",
                planning_request_path=write_request(root),
            )
            with self.assertRaisesRegex(execution.PrintExecutionError, "start joint"):
                execution.prepare_execution_manifest(**common)
            write_trajectory(approach, distance=0.0)
            for source in ("review", "moveit_result"):
                obsolete = {**common, source: {**common[source]}}
                obsolete[source].pop("planning_policy_version")
                with self.assertRaisesRegex(execution.PrintExecutionError, "planning policy"):
                    execution.prepare_execution_manifest(**obsolete)
            incomplete = {**common, "moveit_result": {**common["moveit_result"]}}
            incomplete["moveit_result"].pop("interpolation_validation")
            with self.assertRaisesRegex(execution.PrintExecutionError, "interpolation"):
                execution.prepare_execution_manifest(**incomplete)

    def test_dispatch_refuses_a_request_revoked_after_loading(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            request = write_request(Path(directory))
            executor = runner.RosPrintExecutor.__new__(runner.RosPrintExecutor)
            executor.manifest = {
                "planning_policy_version": POLICY,
                "printhead_geometry_sha256": execution.printhead_geometry_sha256(),
                "planning_request": {
                    "container_path": str(request), "sha256": runner.sha256_file(request),
                },
            }
            client = SimpleNamespace(send_goal_async=lambda *_args, **_kwargs: self.fail("must not dispatch"))
            request.write_text("new planning request", encoding="utf-8")
            with self.assertRaisesRegex(runner.ExecutionFailure, "planning request"):
                executor._send(client, object(), "UR")

    def test_replanning_revokes_prepared_packages_and_cli_rejects_old_policy(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            approach, printing = root / "approach.csv", root / "print.csv"
            write_trajectory(approach, distance=0.0)
            write_trajectory(printing)
            request = write_request(root)
            manifest_path = root / "manifest.json"
            manifest = execution.prepare_execution_manifest(
                review=review(), moveit_result=moveit(),
                approach_trajectory_path=approach, print_trajectory_path=printing,
                destination=manifest_path, manual_stepper="extruder",
                planning_request_path=request,
            )
            obsolete = {**manifest}
            obsolete.pop("planning_policy_version")
            obsolete["execution_id"] = execution._manifest_execution_id(obsolete)
            manifest_path.write_text(json.dumps(obsolete), encoding="utf-8")
            with self.assertRaisesRegex(execution.PrintExecutionError, "planning policy"):
                execution.read_execution_manifest(manifest_path)
            with self.assertRaisesRegex(runner.ExecutionFailure, "planning policy"):
                runner.load_manifest(manifest_path)
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            request.write_text("new attempt started, no passing result", encoding="utf-8")
            with self.assertRaisesRegex(execution.PrintExecutionError, "planning_request"):
                execution.read_execution_manifest(manifest_path)

    def test_main_preserves_unconfirmed_stop_as_a_terminal_safety_failure(self) -> None:
        fake = runner.RosPrintExecutor.__new__(runner.RosPrintExecutor)
        fake._initialize_goal_tracking()
        fake.status = {}
        fake.cancel_requested = threading.Event()
        fake.wait_for_services = lambda: (_ for _ in ()).throw(runner.ExecutionFailure("Execution canceled"))
        def unconfirmed():
            fake.status["stop"] = {"confirmed": False, "reasons": ["Klipper stop was not confirmed"]}
            return False, "Klipper stop was not confirmed"
        fake._abort_active_goals = unconfirmed
        fake._write_status = lambda: None
        fake.close = lambda: None
        fake.cancel_requested.set()
        args = SimpleNamespace(
            manifest=Path("manifest.json"),
            status=Path("status.json"),
            mode="execute",
        )
        with mock.patch.object(runner, "parse_args", return_value=args), mock.patch.object(
            runner,
            "load_manifest",
            return_value={"execution_id": "test"},
        ), mock.patch.object(runner, "RosPrintExecutor", return_value=fake):
            return_code = runner.main()

        self.assertEqual(return_code, 2)
        self.assertEqual(fake.status["state"], "stop-unconfirmed")
        self.assertIn("Klipper stop was not confirmed", fake.status["message"])

    def test_cancel_client_recognizes_print_executor_nodes(self) -> None:
        self.assertTrue(
            cancel_client.coordinator_is_visible(
                ["unrelated", "ur_print_executor_31842"]
            )
        )
        self.assertTrue(
            cancel_client.coordinator_is_visible(["ur3e_cartesian_xyz_moveit"])
        )
        self.assertFalse(cancel_client.coordinator_is_visible(["unrelated"]))

    def test_cancel_service_fails_when_klipper_stop_cannot_be_confirmed(self) -> None:
        executor = runner.RosPrintExecutor.__new__(runner.RosPrintExecutor)
        executor.cancel_requested = threading.Event()
        executor.active_goals = []
        executor._initialize_goal_tracking()
        terminal = Future()
        terminal.set_result(SimpleNamespace(
            status=6,
            result=SimpleNamespace(may_have_moved=True),
        ))
        executor._goal_requests.append({
            "label": "Klipper",
            "future": Future(),
            "acknowledged": True,
            "ack_time": 1.0,
            "abandoned": False,
            "handle": SimpleNamespace(accepted=True),
            "result_future": terminal,
            "submitted_at": 1.0,
            "cancel_requested": False,
        })
        executor.status = {}
        executor.emergency_client = SimpleNamespace(service_is_ready=lambda: False)
        executor.Trigger = SimpleNamespace(Request=lambda: object())
        executor._set_status = lambda *_args, **_kwargs: None
        response = SimpleNamespace(success=None, message="")

        result = executor._cancel_callback(object(), response)

        self.assertIs(result, response)
        self.assertFalse(response.success)
        self.assertIn("not available", response.message)

    def test_synchronized_start_plan_uses_one_motion_start_for_both_channels(self) -> None:
        plan = runner.synchronized_start_plan(10_000_000_000)

        self.assertEqual(plan["ur_motion_start_ns"], plan["klipper_start_ns"])
        self.assertGreater(plan["ur_motion_start_ns"], 10_000_000_000)
        self.assertLess(plan["ur_header_stamp_ns"], plan["ur_motion_start_ns"])

    def test_klipper_action_protocol_carries_an_absolute_ros_start_time(self) -> None:
        action = (
            SOFTWARE_DIR.parent
            / "workspaces"
            / "ur_ros2_ws"
            / "src"
            / "ur_klipper_interfaces"
            / "action"
            / "ManualStepperMove.action"
        ).read_text(encoding="utf-8")

        self.assertIn("int64 start_time_nanoseconds", action)

    def test_controller_switch_uses_strict_mode(self) -> None:
        class Request:
            BEST_EFFORT = 1
            STRICT = 2

            def __init__(self) -> None:
                self.activate_controllers = []
                self.deactivate_controllers = []
                self.strictness = 0

        future = SimpleNamespace(
            done=lambda: True,
            result=lambda: SimpleNamespace(ok=True),
        )
        client = SimpleNamespace(
            wait_for_service=lambda timeout_sec: True,
            call_async=lambda request: setattr(client, "request", request) or future,
        )
        executor = runner.RosPrintExecutor.__new__(runner.RosPrintExecutor)
        executor.SwitchController = SimpleNamespace(Request=Request)
        executor.switch_controller_client = client
        executor.cancel_requested = threading.Event()
        executor.controller_switched = False

        executor._switch_to_passthrough_controller()

        self.assertEqual(client.request.strictness, Request.STRICT)
        self.assertTrue(executor.controller_switched)

    def test_controller_restore_refuses_while_a_motion_goal_is_unconfirmed(self) -> None:
        executor = runner.RosPrintExecutor.__new__(runner.RosPrintExecutor)
        executor.controller_switched = True
        executor.active_goals = [object()]
        executor._switch_controllers = lambda **_kwargs: self.fail(
            "controller switch must not run with an unconfirmed active goal"
        )

        with self.assertRaisesRegex(runner.ExecutionFailure, "active motion"):
            executor._restore_scaled_controller()

    def test_retired_tabletop_review_cannot_prepare_an_execution_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            approach = root / "approach.csv"
            printing = root / "print.csv"
            write_trajectory(approach, distance=0.0)
            write_trajectory(printing)
            test_review = review()
            test_review["operation_mode"] = "tool0_tabletop_test"
            test_review["within_print_bed"] = True
            test_review["print_bed_check"] = {
                "measured": True,
                "surface_aligned": True,
            }

            with self.assertRaises(execution.PrintExecutionError):
                execution.prepare_execution_manifest(
                    review=test_review,
                    moveit_result=moveit(),
                    approach_trajectory_path=approach,
                    print_trajectory_path=printing,
                    planning_request_path=write_request(root),
                    destination=root / "manifest.json",
                    manual_stepper="extruder",
                )

    def test_tool0_motion_test_manifest_disables_heat_and_requires_operator_click(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            approach = root / "approach.csv"
            printing = root / "print.csv"
            write_trajectory(approach, distance=0.0)
            write_trajectory(printing)
            test_review = review()
            test_review["operation_mode"] = "tool0_motion_test"
            test_review["air_run_check"] = {"passed": True}
            test_review["collision_bed_m"] = {
                "plane_z": -0.014,
                "minimum": {"x": -1.0, "y": -1.0},
                "maximum": {"x": 1.0, "y": 1.0},
                "edge_margin_m": 0.0,
            }

            manifest = execution.prepare_execution_manifest(
                review=test_review,
                moveit_result=moveit(),
                approach_trajectory_path=approach,
                print_trajectory_path=printing,
                planning_request_path=write_request(root),
                destination=root / "manifest.json",
                manual_stepper="extruder",
            )

            self.assertEqual(manifest["operation_mode"], "tool0_motion_test")
            self.assertFalse(manifest["heating_enabled"])
            self.assertEqual(manifest["operator_action_required"], "execute")
            self.assertNotIn("authorization_required", manifest)
            self.assertEqual(manifest["print_bed_m"]["minimum"]["x"], -1.0)
            self.assertEqual(
                manifest["execution_scope"]["type"],
                "bounded_cold_motion_sample",
            )
            self.assertTrue(manifest["execution_scope"]["full_path_validated"])
            self.assertAlmostEqual(manifest["execution_scope"]["motion_duration_s"],
                                   sum(b["duration_s"] for b in manifest["blocks"]), places=3)

    def test_tool0_motion_test_is_bounded_at_a_complete_sync_block(self) -> None:
        blocks = [
            {
                "index": index,
                "kind": "print",
                "source_line_start": index + 1,
                "source_line_end": index + 1,
                "start_progress": index / 20,
                "end_progress": (index + 1) / 20,
                "duration_s": 5.0,
                "klipper_units": 1.0,
            }
            for index in range(20)
        ]

        selected, scope = execution._execution_scope(blocks, "tool0_motion_test")

        self.assertEqual(len(selected), 6)
        self.assertEqual(scope["motion_duration_s"], 30.0)
        self.assertEqual(scope["end_progress"], 0.3)
        self.assertEqual(selected[-1]["index"], 5)

    def test_manifest_preserves_ur_hold_retraction_and_artifact_identity(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            approach = root / "approach.csv"
            printing = root / "print.csv"
            manifest_path = root / "manifest.json"
            write_trajectory(approach, distance=0.0)
            write_trajectory(printing)
            manifest = execution.prepare_execution_manifest(
                review=review(),
                moveit_result=moveit(),
                approach_trajectory_path=approach,
                print_trajectory_path=printing,
                planning_request_path=write_request(root),
                destination=manifest_path,
                manual_stepper="extruder",
            )

            self.assertEqual(manifest["block_count"], 3)
            self.assertEqual(manifest["version"], 3)
            self.assertIn("print", manifest["trajectory_profiles"])
            self.assertNotIn("blocks", manifest["trajectory_profiles"])
            self.assertTrue(manifest["blocks"][-1]["has_ur_motion"] is False)
            self.assertEqual(manifest["blocks"][-1]["kind"], "retract")
            self.assertEqual(manifest["blocks"][-1]["start_progress"], 1.0)
            self.assertEqual(manifest["blocks"][-1]["end_progress"], 1.0)
            self.assertEqual(manifest["blocks"][-1]["klipper_target_position"], -0.5)
            self.assertEqual(manifest["blocks"][-1]["klipper_cumulative_position"], 3.5)
            self.assertEqual(
                execution.read_execution_manifest(manifest_path)["execution_id"],
                manifest["execution_id"],
            )

            tampered = json.loads(manifest_path.read_text(encoding="utf-8"))
            tampered["blocks"][0]["duration_s"] += 0.5
            manifest_path.write_text(json.dumps(tampered), encoding="utf-8")
            with self.assertRaisesRegex(
                execution.PrintExecutionError,
                "changed after preparation",
            ):
                execution.read_execution_manifest(manifest_path)

            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

            printing.write_text(printing.read_text(encoding="utf-8") + "\n", encoding="utf-8")
            with self.assertRaises(execution.PrintExecutionError):
                execution.read_execution_manifest(manifest_path)

    def test_real_manifest_rejects_simulation_and_sampled_timeline(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            approach = root / "approach.csv"
            printing = root / "print.csv"
            write_trajectory(approach, distance=0.0)
            write_trajectory(printing)
            common = {
                "review": review(),
                "approach_trajectory_path": approach,
                "print_trajectory_path": printing,
                "planning_request_path": write_request(root),
                "destination": root / "manifest.json",
                "manual_stepper": "extruder",
            }
            with self.assertRaises(execution.PrintExecutionError):
                execution.prepare_execution_manifest(
                    **common,
                    moveit_result=moveit(simulation=True),
                )
            sampled = review()
            sampled["timeline"]["sampled"] = True
            with self.assertRaises(execution.PrintExecutionError):
                execution.prepare_execution_manifest(
                    **{**common, "review": sampled},
                    moveit_result=moveit(),
                )

    def test_resampling_and_timing_helpers_are_guarded(self) -> None:
        positions = runner.resample_positions(
            [[0.0] * 6, [0.035, 0.0, 0.0, 0.0, 0.0, 0.0]]
        )
        self.assertEqual(len(positions), 5)
        maximum_step = max(
            abs(after[0] - before[0])
            for before, after in zip(positions, positions[1:])
        )
        self.assertLessEqual(maximum_step, runner.MAX_RESAMPLED_STEP_RAD)
        minimum = runner.klipper_minimum_duration(5.0, 5.0, 20.0)
        speed = runner.klipper_speed_for_duration(5.0, minimum + 1.0, 5.0, 20.0)
        self.assertGreater(speed, 0.0)
        self.assertLess(speed, 5.0)

    def test_slicing_uses_cartesian_progress_not_joint_point_index(self) -> None:
        points = [
            {"time_s": 0.0, "path_progress": 0.0, "positions": [0.0] * 6},
            {"time_s": 1.0, "path_progress": 0.1, "positions": [1.0, 0, 0, 0, 0, 0]},
            {"time_s": 2.0, "path_progress": 1.0, "positions": [2.0, 0, 0, 0, 0, 0]},
        ]

        sliced = runner.slice_trajectory(points, 0.05, 0.5)

        self.assertAlmostEqual(sliced[0][0], 0.5)
        self.assertAlmostEqual(sliced[-1][0], 1.0 + 0.4 / 0.9)
        self.assertIn([1.0, 0, 0, 0, 0, 0], sliced)

    def test_execution_blocks_split_when_extrusion_rate_changes(self) -> None:
        changing = review()
        changing["timeline"]["events"] = [
            {
                "sequence": 1,
                "line": 1,
                "duration_s": 1.0,
                "ur_distance_mm": 10.0,
                "klipper_units": 1.0,
                "klipper_action": "extrude",
            },
            {
                "sequence": 2,
                "line": 2,
                "duration_s": 1.0,
                "ur_distance_mm": 10.0,
                "klipper_units": 2.0,
                "klipper_action": "extrude",
            },
        ]

        blocks = execution._execution_blocks(changing, 2.0)

        self.assertEqual(len(blocks), 2)
        self.assertNotEqual(blocks[0]["flow_units_per_m"], blocks[1]["flow_units_per_m"])

    def test_print_preflight_reports_each_missing_live_signal(self) -> None:
        missing = runner.RosPrintExecutor._preflight_waiting_for(
            {
                "program_state_received": False,
                "external_control_running": False,
                "joint_state_fresh": False,
                "trajectory_socket_received": False,
                "trajectory_socket_fresh": False,
                "trajectory_socket_connected": False,
                "klipper_action_server": False,
                "trajectory_action_server": False,
            }
        )

        self.assertEqual(
            missing,
            [
                "live External Control state",
                "fresh six-joint state",
                "live UR trajectory socket state",
                "Klipper bridge action server",
            ],
        )

        self.assertEqual(
            runner.RosPrintExecutor._preflight_waiting_for(
                {
                    "program_state_received": True,
                    "external_control_running": False,
                    "joint_state_fresh": True,
                    "trajectory_socket_received": True,
                    "trajectory_socket_fresh": True,
                    "trajectory_socket_connected": True,
                    "klipper_action_server": True,
                    "trajectory_action_server": False,
                }
            ),
            ["External Control Play"],
        )

        for fresh, connected, expected in (
            (True, False, "UR trajectory socket connection"),
            (False, True, "fresh UR trajectory socket state"),
        ):
            with self.subTest(fresh=fresh, connected=connected):
                self.assertEqual(
                    runner.RosPrintExecutor._preflight_waiting_for(
                        {
                            "program_state_received": True,
                            "external_control_running": True,
                            "joint_state_fresh": True,
                            "trajectory_socket_received": True,
                            "trajectory_socket_fresh": fresh,
                            "trajectory_socket_connected": connected,
                            "klipper_action_server": True,
                        }
                    ),
                    [expected],
                )


if __name__ == "__main__":
    unittest.main()
