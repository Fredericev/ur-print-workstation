"""Command-boundary regressions. All ROS transports are inert test doubles."""
from concurrent.futures import Future
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import sys
import struct
import tempfile
import threading
import time as real_time
from types import SimpleNamespace as NS
import unittest
from unittest import mock

from Software.tests.test_executor_goal_lifecycle import runner, resolved, Clock, Event


class CommandBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.clock = Clock()
        self.addCleanup(mock.patch.stopall)
        mock.patch.object(runner, "time", self.clock).start()
        mock.patch.dict(sys.modules, {
            "builtin_interfaces.msg": NS(Duration=lambda **kw: NS(**kw)),
            "trajectory_msgs.msg": NS(JointTrajectoryPoint=lambda: NS(velocities=[], accelerations=[])),
        }).start()
        self.sent = []
        self.executor = runner.RosPrintExecutor.__new__(runner.RosPrintExecutor)
        e = self.executor
        e._initialize_goal_tracking()
        e.active_goals = []
        e.cancel_requested = Event(self.clock)
        e.status = {}
        e.status_path = self.root / "status.json"
        e.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=int(self.clock.now * 1e9))))
        e.latest_joints = dict.fromkeys(runner.UR_JOINTS, 0.0)
        e.latest_joint_velocities = dict.fromkeys(runner.UR_JOINTS, 0.0)
        e.latest_joint_stamp = e.program_state_stamp = self.clock.now
        e.program_running = True
        e._trajectory_socket_state = (True, self.clock.now)
        original_sleep = self.clock.sleep
        def sleep_with_socket_heartbeat(duration):
            original_sleep(duration)
            if e._trajectory_socket_state[0]:
                e._trajectory_socket_callback(NS(data=True))
        self.clock.sleep = sleep_with_socket_heartbeat
        e.FollowJointTrajectory = NS(Goal=lambda: NS(trajectory=NS(
            joint_names=[], points=[], header=NS(stamp=NS(sec=0, nanosec=0)))))
        e.JointTolerance = lambda: NS(name="", position=0.0, velocity=0.0, acceleration=0.0)
        marker = self.root / "planning.request.json"
        marker.write_text("isolated test request")
        e.manifest = {"planning_policy_version": runner.PLANNING_POLICY_VERSION,
                      "printhead_geometry_sha256": runner.PRINTHEAD_GEOMETRY_SHA256,
                      "planning_request": {"path": str(marker), "sha256": runner.sha256_file(marker)}}
        def send(goal, **_kwargs):
            self.sent.append(goal)
            e.latest_joints = dict(zip(runner.UR_JOINTS, goal.trajectory.points[-1].positions))
            e.latest_joint_stamp = self.clock.now
            return resolved(NS(accepted=True, get_result_async=lambda: resolved(
                NS(status=4, result=NS(error_code=0)))))
        e.ur_client = NS(send_goal_async=send)
        self.validity_requests = []
        e.GetPlanningScene = NS(Request=lambda: NS(components=NS()))
        e.GetStateValidity = NS(Request=lambda: NS())
        e.GetParameters = NS(Request=lambda: NS())
        e.manifest["print_bed_m"] = {"minimum": {"x": -.5175, "y": -.27},
                                     "maximum": {"x": -.0675, "y": .27}, "plane_z": -.014}
        self.scene = scene = NS(world=NS(collision_objects=[NS(id="confirmed_table_surface_guard", header=NS(frame_id="base_link"),
                   primitives=[NS(type=1, dimensions=[.45,.54,2.0])],
                   pose=NS(position=NS(x=0,y=0,z=0), orientation=NS(x=0,y=0,z=0,w=1)),
                   primitive_poses=[NS(position=NS(x=-.2925,y=0,z=-1.014), orientation=NS(x=0,y=0,z=0,w=1))])]),
                   robot_state=NS(joint_state=NS(name=list(runner.UR_JOINTS), position=[0.0]*6),
                                  attached_collision_objects=[]),
                   allowed_collision_matrix=NS(entry_names=[], entry_values=[], default_entry_names=[], default_entry_values=[]))
        e.planning_scene_client = NS(wait_for_service=lambda **kw: True, call_async=lambda r: resolved(NS(scene=scene)))
        def check(request):
            self.validity_requests.append(request)
            return resolved(NS(valid=True))
        e.state_validity_client = NS(wait_for_service=lambda **kw: True, call_async=check)
        urdf = "<robot name='test'><joint name='world_joint' type='fixed'><parent link='world'/><child link='base_link'/></joint>" + "".join(
            f'<joint name="{name}" type="revolute"><limit lower="-6.28" upper="6.28"/></joint>'
            for name in runner.UR_JOINTS) + "</robot>"
        e.model_parameters_client = NS(wait_for_service=lambda **kw: True,
                                      call_async=lambda r: resolved(NS(values=[NS(string_value=urdf)])))

    @staticmethod
    def measured_printhead_attachment():
        pose = lambda x, y, z: NS(
            position=NS(x=x, y=y, z=z),
            orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        tool = NS(
            id="measured_print_tool", header=NS(frame_id="tool0"), pose=pose(0, 0, 0),
            primitives=[NS(type=1, dimensions=list(runner.PRINTHEAD_BOX_SIZE_M))],
            primitive_poses=[pose(*runner.PRINTHEAD_BOX_CENTER_TOOL0_M)],
            meshes=[], mesh_poses=[], planes=[], plane_poses=[],
        )
        return NS(
            link_name="tool0", touch_links=["tool0", "flange", "wrist_3_link"],
            object=tool,
        )

    def test_move_to_start_passes_smooth_internal_waypoints_without_zero_velocity(self):
        path = [{"time_s": t, "path_progress": p, "positions": [q, 0, 0, 0, 0, 0]}
                for t, p, q in [(0, 0, 0), (1, .25, .005), (3, 1, .02)]]
        self.package(path, approach=True)
        self.executor.move_to_start(path)
        self.assertEqual(len(self.sent), 1)
        internal = self.sent[0].trajectory.points[1:-1]
        self.assertTrue(internal)
        self.assertTrue(any(len(p.velocities) == 6 and p.velocities[0] > 0 for p in internal),
                        "A smooth approach must not request zero speed at every intermediate waypoint")

    def test_authenticated_approach_is_not_recursively_recertified_at_each_seam(self):
        path = [{"time_s": t, "positions": [q, 0, 0, 0, 0, 0]}
                for t, q in [(0, 0), (1, .005), (3, .02)]]
        self.package(path, approach=True)
        e = self.executor
        profiles = runner.validate_bundle(e.manifest)
        e.manifest = runner.ValidatedExecutionPackage(e.manifest, profiles)
        e._prepared_profiles = e.manifest.validated_profiles
        e._prepared_profile_digests = e.manifest.profile_digests
        with mock.patch.object(runner, "validate_bundle", wraps=runner.validate_bundle) as bundle_check, \
                mock.patch.object(runner, "validate_profile", wraps=runner.validate_profile) as curve_check:
            e.move_to_start(path)
        self.assertEqual(bundle_check.call_count, 0)
        self.assertEqual(curve_check.call_count, 0,
                         "Dispatch and sampling must reuse the exact authenticated math certificate")
        self.assertTrue(self.validity_requests, "Live MoveIt collision checking must remain")
        self.assertEqual(len(self.sent), 1)

    def test_changed_profile_cannot_reuse_the_prepared_math_certificate(self):
        path = [{"time_s": t, "positions": [q, 0, 0, 0, 0, 0]}
                for t, q in [(0, 0), (1, .005), (3, .02)]]
        self.package(path, approach=True)
        e = self.executor
        profile = e._prepared_profile("approach")
        profile["points"][0]["timeline_markers"] = ["changed-after-authentication"]
        with self.assertRaisesRegex(runner.ExecutionFailure, "changed"):
            e.move_to_start(path)
        self.assertEqual(self.sent, [])

    def test_tiny_clipped_boundary_does_not_stretch_the_whole_move_into_hours(self):
        path = [{"time_s": t, "path_progress": p, "positions": [q,0,0,0,0,0]}
                for t,p,q in [(0,0,0), (1,.25,.005), (3,1-1e-9,.02), (3+1e-8,1,.0200000001)]]
        self.package(path, approach=True)
        self.executor.move_to_start(path)
        points = self.sent[0].trajectory.points
        seconds = lambda p: p.time_from_start.sec + p.time_from_start.nanosec/1e9
        self.assertLess(seconds(points[-1])-seconds(points[0]), 15,
                        "A tiny clipped tail must only retime locally, not stretch the entire approach")

    def test_continuous_print_ur_and_klipper_share_one_final_timeline(self):
        self._run_continuous_print("tool0_motion_test")

    def test_production_package_uses_the_reviewed_ur_and_klipper_timeline(self):
        self._run_continuous_print("production_print")
        self.assertIn("did not control heating", self.executor.status["message"])

    def _run_continuous_print(self, operation_mode):
        e = self.executor
        e.latest_scaling = 100.0
        e.latest_scaling_stamp = self.clock.now
        e.scaling_stable_since = self.clock.now - 1.0
        e.ManualStepperProgram = NS(
            Goal=lambda: NS(), Feedback=NS(PHASE_DISPATCHED=2)
        )
        klipper = []
        def send(goal, **kw):
            klipper.append(goal)
            kw["feedback_callback"](NS(feedback=NS(
                phase=2,
                program_epoch_ros_ns=int(self.clock.now * 1_000_000_000),
            )))
            return resolved(NS(accepted=True, get_result_async=lambda: resolved(
                NS(status=4, result=NS(success=True)))))
        e.klipper_client = NS(send_goal_async=send)
        e.manifest.update({"operation_mode": operation_mode, "execution_id": "test-only",
                           "extruder": "extruder", "manual_stepper": "extruder", "stepper_limits": {"maximum_speed": 5, "acceleration": 20, "max_extrude_only_distance": 50},
                           "blocks": [{"index": 0, "kind": "print", "start_progress": 0, "end_progress": 1,
                                       "duration_s": 1.0, "klipper_units": .1, "klipper_target_position": .1,
                                       "source_line_start": 1, "source_line_end": 1,
                                       "has_ur_motion": True}]})
        if operation_mode == "production_print":
            e.manifest["tool_center_point_m"] = {
                "collision_radius_m": .02, "offset": {"x": 0, "y": 0, "z": .1}}
            self.scene.robot_state.attached_collision_objects = [
                self.measured_printhead_attachment()
            ]
        path = [{"time_s": t, "path_progress": p, "positions": [q, 0, 0, 0, 0, 0]}
                for t,p,q in [(0,0,0), (1,.25,.005), (3,1,.02)]]
        self.package(path, approach=False)
        awaited = []
        original_await = e._await_results
        def capture_await(entries, timeout_s, **kwargs):
            awaited.append(kwargs)
            return original_await(entries, timeout_s, **kwargs)
        e._await_results = capture_await
        e.execute_print(path)
        self.assertIs(
            awaited[0].get("motion_profile"),
            e.manifest["trajectory_profiles"]["print"],
            "The continuous print needs the same live-motion watchdog as the approach",
        )
        points = self.sent[0].trajectory.points
        seconds = lambda p: p.time_from_start.sec + p.time_from_start.nanosec/1e9
        # Driver differences timestamps, narrows each duration to float32,
        # rounds it to integer microseconds, then the UR script decodes it.
        ur_duration = sum(math.floor(struct.unpack("f", struct.pack("f", seconds(b)-seconds(a)))[0]*1e6+.5)/1e6
                          for a,b in zip(points,points[1:]))
        k = klipper[0]
        self.assertEqual(len(k.segment_durations_s), 1)
        self.assertAlmostEqual(ur_duration, k.segment_durations_s[0], places=6,
                               msg="Klipper timing must use the one final UR timeline")
        self.assertEqual(k.start_delay_s, runner.SYNC_START_LEAD_S)

    def package(self, path, *, approach):
        stationary = lambda q: [{"time_s": 0, "positions": q}, {"time_s": .25, "positions": q}]
        profile = runner.compile_profile(path, limit_class="approach" if approach else "print")
        e = self.executor
        if approach:
            block_profile = runner.compile_profile(stationary(path[-1]["positions"]))
            e.manifest["blocks"] = [{"duration_s": block_profile["duration_s"]}]
            e.manifest["trajectory_profiles"] = {"approach": profile, "blocks": [block_profile]}
        else:
            e.manifest["blocks"][0]["duration_s"] = profile["duration_s"]
            e.manifest["blocks"][0]["start_time_s"] = 0.0
            e.manifest["blocks"][0]["end_time_s"] = profile["duration_s"]
            profile["points"][0]["timeline_markers"] = ["block:0:start"]
            profile["points"][-1]["timeline_markers"] = ["block:0:end"]
            e.manifest["trajectory_profiles"] = {"approach": runner.compile_profile(
                                                       stationary(path[0]["positions"]), limit_class="approach"),
                                                   "print": profile}
        e.manifest["trajectory_policy_version"] = runner.TRAJECTORY_POLICY

    def test_collision_on_final_curve_prevents_any_ur_dispatch(self):
        path = [{"time_s": t, "path_progress": p, "positions": [q,0,0,0,0,0]}
                for t,p,q in [(0,0,0), (1,.25,.005), (3,1,.02)]]
        self.package(path, approach=True)
        self.executor.state_validity_client = NS(
            wait_for_service=lambda **kw: True,
            call_async=lambda request: resolved(NS(valid=False)))
        with self.assertRaisesRegex(runner.ExecutionFailure, "collision|invalid"):
            self.executor.move_to_start(path)
        self.assertEqual(self.sent, [], "No actuator goal is allowed before the final curve passes")

    def test_moveit_canonical_parent_pose_is_accepted(self):
        guard = self.scene.world.collision_objects[0]
        guard.header.frame_id = "world"
        guard.pose.position, guard.primitive_poses[0].position = guard.primitive_poses[0].position, guard.pose.position
        path = [{"time_s": t, "positions": [q,0,0,0,0,0]} for t,q in [(0,0),(1,.005),(3,.02)]]
        self.package(path, approach=True)
        self.executor.move_to_start(path)
        self.assertEqual(len(self.sent), 1)

    def test_rotated_table_is_not_accepted_as_the_confirmed_workcell(self):
        guard = self.scene.world.collision_objects[0]
        guard.primitive_poses[0].orientation = NS(x=0,y=0,z=.7071067811865476,w=.7071067811865476)
        path = [{"time_s": t, "positions": [q,0,0,0,0,0]} for t,q in [(0,0),(1,.005),(3,.02)]]
        self.package(path, approach=True)
        with self.assertRaisesRegex(runner.ExecutionFailure, "orientation|rotation"):
            self.executor.move_to_start(path)
        self.assertEqual(self.sent, [])

    def test_measured_tool_id_alone_cannot_authorize_wrong_collision_geometry(self):
        self.executor.manifest["tool_center_point_m"] = {
            "collision_radius_m": .02, "offset": {"x":0,"y":0,"z":.1}}
        self.scene.robot_state.attached_collision_objects = [NS(object=NS(id="measured_print_tool"))]
        path = [{"time_s": t, "positions": [q,0,0,0,0,0]} for t,q in [(0,0),(1,.005),(3,.02)]]
        self.package(path, approach=True)
        with self.assertRaisesRegex(runner.ExecutionFailure, "tool.*geometry|tool.*unverified"):
            self.executor.move_to_start(path)
        self.assertEqual(self.sent, [])

    def test_tool0_only_scene_rejects_unexpected_printhead_attachment(self):
        self.scene.robot_state.attached_collision_objects = [NS(object=NS(id="measured_print_tool"))]
        path = [{"time_s": t, "positions": [q,0,0,0,0,0]} for t,q in [(0,0),(1,.005),(3,.02)]]
        self.package(path, approach=True)
        with self.assertRaisesRegex(runner.ExecutionFailure, "Unexpected measured print tool"):
            self.executor.move_to_start(path)
        self.assertEqual(self.sent, [])

    def test_executor_and_generated_planner_share_geometry_and_identity(self):
        generator = (Path(__file__).resolve().parents[2] / "workspaces" / "ur_ros2_ws" / "src" /
                     "ur3e_print_planner" / "scripts" / "generate_printhead_geometry.py")
        spec = importlib.util.spec_from_file_location("geometry_generator_under_test", generator)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        header = module.generate_header(runner.PRINTHEAD_GEOMETRY_PATH)
        constants = {
            name: float(value)
            for name, value in re.findall(r"constexpr double (kPrintheadBox\w+) = ([^;]+);", header)
        }
        self.assertEqual(runner.PRINTHEAD_BOX_SIZE_M, (
            constants["kPrintheadBoxSizeX"], constants["kPrintheadBoxSizeY"],
            constants["kPrintheadBoxSizeZ"],
        ))
        self.assertEqual(runner.PRINTHEAD_BOX_CENTER_TOOL0_M, (
            constants["kPrintheadBoxCenterX"], constants["kPrintheadBoxCenterY"],
            constants["kPrintheadBoxCenterZ"],
        ))
        self.assertIn(f'kPrintheadGeometrySha256[] = "{runner.PRINTHEAD_GEOMETRY_SHA256}"', header)

    def test_old_geometry_package_is_rejected_before_trajectory_validation(self):
        payload = {"version": 3, "planning_policy_version": runner.PLANNING_POLICY_VERSION,
                   "printhead_geometry_sha256": "0" * 64,
                   "extrusion_protocol": "klipper-native-extruder-v1", "extruder": "extruder"}
        payload["execution_id"] = runner.manifest_execution_id(payload)
        manifest = self.root / "obsolete-geometry.json"
        manifest.write_text(json.dumps(payload), encoding="utf-8")
        with mock.patch.object(runner, "validate_bundle") as validate:
            with self.assertRaisesRegex(runner.ExecutionFailure, "obsolete printhead geometry"):
                runner.load_manifest(manifest)
        validate.assert_not_called()

    def test_invalid_collision_envelope_cannot_be_compiled_or_loaded(self):
        generator = (Path(__file__).resolve().parents[2] / "workspaces" / "ur_ros2_ws" / "src" /
                     "ur3e_print_planner" / "scripts" / "generate_printhead_geometry.py")
        spec = importlib.util.spec_from_file_location("invalid_geometry_generator", generator)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        geometry = json.loads(runner.PRINTHEAD_GEOMETRY_PATH.read_text(encoding="utf-8"))
        geometry["collision_envelope_tool0_m"]["size"]["x"] = -0.01
        invalid = self.root / "invalid-printhead.json"
        invalid.write_text(json.dumps(geometry), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "positive"):
            module.generate_header(invalid)
        with self.assertRaisesRegex(ValueError, "positive"):
            runner.load_printhead_collision_geometry(invalid)

    def test_missing_geometry_identity_cannot_dispatch_an_actuator(self):
        self.executor.manifest.pop("printhead_geometry_sha256")
        with self.assertRaisesRegex(runner.ExecutionFailure, "obsolete printhead geometry"):
            self.executor._send(self.executor.ur_client, NS(), "UR")
        self.assertEqual(self.sent, [])

    def test_geometry_changed_after_review_blocks_dispatch_even_with_same_box(self):
        geometry = json.loads(runner.PRINTHEAD_GEOMETRY_PATH.read_text(encoding="utf-8"))
        geometry["geometry_revision"] += "-revised"
        changed = self.root / "changed-printhead.json"
        changed.write_text(json.dumps(geometry), encoding="utf-8")
        with mock.patch.object(runner, "PRINTHEAD_GEOMETRY_PATH", changed):
            with self.assertRaisesRegex(runner.ExecutionFailure, "geometry changed"):
                self.executor._send(self.executor.ur_client, NS(), "UR")
        self.assertEqual(self.sent, [])

    def test_geometry_changed_while_waiting_for_send_lock_blocks_dispatch(self):
        geometry = json.loads(runner.PRINTHEAD_GEOMETRY_PATH.read_text(encoding="utf-8"))
        config = self.root / "printhead-during-lock-wait.json"
        config.write_text(json.dumps(geometry), encoding="utf-8")
        passed_entry_guard = threading.Event()
        self.executor.require_live_program = passed_entry_guard.set
        errors = []
        prepare = mock.Mock()
        self.executor._prepare_goal_for_dispatch = prepare

        def dispatch():
            try:
                self.executor._send(self.executor.ur_client, NS(), "UR")
            except Exception as exc:
                errors.append(exc)

        # Hold the real send lock until the worker has passed its entry identity
        # check; then replace only the geometry revision while it is blocked.
        with mock.patch.object(runner, "PRINTHEAD_GEOMETRY_PATH", config):
            with self.executor._goal_lock:
                worker = threading.Thread(target=dispatch, daemon=True)
                worker.start()
                self.assertTrue(passed_entry_guard.wait(timeout=2.0))
                geometry["geometry_revision"] += "-changed-during-lock-wait"
                config.write_text(json.dumps(geometry), encoding="utf-8")
            worker.join(timeout=2.0)
            self.assertFalse(worker.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(errors[0], runner.ExecutionFailure)
        self.assertIn("geometry changed", str(errors[0]))
        prepare.assert_not_called()
        self.assertEqual(self.sent, [])

    def test_planner_single_box_printhead_accepts_saved_boundary_tcp(self):
        """The saved nozzle tip may lie on the CAD envelope boundary."""
        self.executor.manifest["tool_center_point_m"] = {
            "collision_radius_m": .004,
            "offset": {"x": .03255, "y": .0344,
                       "z": runner.PRINTHEAD_BOX_CENTER_TOOL0_M[2] + runner.PRINTHEAD_BOX_SIZE_M[2] / 2},
        }
        self.scene.robot_state.attached_collision_objects = [
            self.measured_printhead_attachment()
        ]
        path = [{"time_s": t, "positions": [q, 0, 0, 0, 0, 0]}
                for t, q in [(0, 0), (1, .005), (3, .02)]]
        self.package(path, approach=True)

        self.executor.move_to_start(path)

        self.assertEqual(len(self.sent), 1)
        self.assertTrue(self.validity_requests, "The accepted tool must still receive MoveIt collision checks")

    def test_wrong_printhead_box_blocks_move_to_start_before_ur_dispatch(self):
        self.executor.manifest["tool_center_point_m"] = {
            "collision_radius_m": .02, "offset": {"x": 0, "y": 0, "z": .1}}
        attached = self.measured_printhead_attachment()
        attached.object.primitives[0].dimensions[0] = .2074
        self.scene.robot_state.attached_collision_objects = [attached]
        path = [{"time_s": t, "positions": [q, 0, 0, 0, 0, 0]}
                for t, q in [(0, 0), (1, .005), (3, .02)]]
        self.package(path, approach=True)

        with self.assertRaisesRegex(runner.ExecutionFailure, "printhead.*dimensions|tool.*dimensions"):
            self.executor.move_to_start(path)
        self.assertEqual(self.sent, [], "Unverified printhead geometry cannot reach the UR goal client")

    def test_missing_printhead_envelope_blocks_move_to_start_before_ur_dispatch(self):
        self.executor.manifest["tool_center_point_m"] = {
            "collision_radius_m": .02, "offset": {"x": 0, "y": 0, "z": .1}}
        attached = self.measured_printhead_attachment()
        # A cylinder alone cannot stand in for the installed printhead body.
        attached.object.primitives[0] = NS(type=3, dimensions=[.1, .02])
        self.scene.robot_state.attached_collision_objects = [attached]
        path = [{"time_s": t, "positions": [q, 0, 0, 0, 0, 0]}
                for t, q in [(0, 0), (1, .005), (3, .02)]]
        self.package(path, approach=True)

        with self.assertRaisesRegex(runner.ExecutionFailure, "printhead.*envelope|tool.*shape"):
            self.executor.move_to_start(path)
        self.assertEqual(self.sent, [], "Missing printhead envelope cannot reach the UR goal client")

    def test_legacy_two_shape_tool_blocks_move_to_start_before_ur_dispatch(self):
        self.executor.manifest["tool_center_point_m"] = {
            "collision_radius_m": .004,
            "offset": {"x": .03255, "y": .0344, "z": .145275},
        }
        attached = self.measured_printhead_attachment()
        # The retired planner put a TCP cylinder before the CAD box.
        attached.object.primitives.insert(0, NS(type=3, dimensions=[.1589, .004]))
        attached.object.primitive_poses.insert(0, attached.object.primitive_poses[0])
        self.scene.robot_state.attached_collision_objects = [attached]
        path = [{"time_s": t, "positions": [q, 0, 0, 0, 0, 0]}
                for t, q in [(0, 0), (1, .005), (3, .02)]]
        self.package(path, approach=True)

        with self.assertRaisesRegex(runner.ExecutionFailure, "tool.*shape"):
            self.executor.move_to_start(path)
        self.assertEqual(self.sent, [])

    def test_tcp_outside_cad_box_blocks_move_to_start_before_ur_dispatch(self):
        self.executor.manifest["tool_center_point_m"] = {
            "collision_radius_m": .004,
            "offset": {"x": .03255, "y": .0344,
                       "z": runner.PRINTHEAD_BOX_CENTER_TOOL0_M[2] + runner.PRINTHEAD_BOX_SIZE_M[2] / 2 + .00001},
        }
        self.scene.robot_state.attached_collision_objects = [
            self.measured_printhead_attachment()
        ]
        path = [{"time_s": t, "positions": [q, 0, 0, 0, 0, 0]}
                for t, q in [(0, 0), (1, .005), (3, .02)]]
        self.package(path, approach=True)

        with self.assertRaisesRegex(runner.ExecutionFailure, "TCP.*outside"):
            self.executor.move_to_start(path)
        self.assertEqual(self.sent, [])

    def test_table_collision_exemption_blocks_motion(self):
        self.scene.allowed_collision_matrix = NS(
            entry_names=["confirmed_table_surface_guard","wrist_3_link"],
            entry_values=[NS(enabled=[False,True]),NS(enabled=[True,False])],
            default_entry_values=[])
        path = [{"time_s": t, "positions": [q,0,0,0,0,0]} for t,q in [(0,0),(1,.005),(3,.02)]]
        self.package(path, approach=True)
        with self.assertRaisesRegex(runner.ExecutionFailure, "allows robot/table"):
            self.executor.move_to_start(path)
        self.assertEqual(self.sent, [])

    def test_unavailable_moveit_does_not_fall_back_to_the_old_polyline_review(self):
        self.executor.state_validity_client.wait_for_service = lambda **kw: False
        path = [{"time_s": t, "positions": [q,0,0,0,0,0]} for t,q in [(0,0),(1,.005),(3,.02)]]
        self.package(path, approach=True)
        with self.assertRaisesRegex(runner.ExecutionFailure, "unavailable"):
            self.executor.move_to_start(path)
        self.assertEqual(self.sent, [])

    def test_robot_drift_during_validation_revokes_the_checked_start(self):
        e = self.executor
        def changed(request):
            e.latest_joints[runner.UR_JOINTS[0]] = .002
            return resolved(NS(valid=True))
        e.state_validity_client.call_async = changed
        path = [{"time_s": t, "positions": [q,0,0,0,0,0]} for t,q in [(0,0),(1,.005),(3,.02)]]
        self.package(path, approach=True)
        with self.assertRaisesRegex(runner.ExecutionFailure, "posture changed"):
            e.move_to_start(path)
        self.assertEqual(self.sent, [])

    def test_final_quintic_interior_not_only_original_waypoints_is_collision_checked(self):
        path = [{"time_s":t,"positions":[x,y,0,0,0,0]}
                for t,x,y in [(0,0,0),(1,.005,.001),(3,.01,.01)]]
        self.package(path, approach=True)
        self.executor.move_to_start(path)
        samples = [r.robot_state.joint_state.position for r in self.validity_requests]
        # First polyline segment has y=.2*x. A joined quintic has its own
        # curved interior; those states must reach MoveIt as well.
        self.assertTrue(any(0 < q[0] < .005 and abs(q[1]-.2*q[0]) > 1e-7 for q in samples))
        self.assertTrue(all(r.group_name == "" for r in self.validity_requests))

    def test_full_quintic_is_sent_once_to_the_local_collision_validator(self):
        """Large jobs must not make one ROS service call per sampled state."""
        path = [{"time_s":t,"positions":[x,y,0,0,0,0]}
                for t,x,y in [(0,0,0),(1,.005,.001),(3,.01,.01)]]
        self.package(path, approach=True)
        batches = []

        def validate_locally(samples, _deadline):
            materialized = list(samples)
            batches.append(materialized)
            return len(materialized)

        self.executor._collision_sample_validator = validate_locally
        self.executor.state_validity_client = NS(
            wait_for_service=lambda **_kw: False,
            call_async=lambda _request: self.fail(
                "Local validation must replace per-state GetStateValidity calls"
            ),
        )

        self.executor.move_to_start(path)

        self.assertEqual(len(batches), 1)
        self.assertGreater(len(batches[0]), len(path))
        self.assertTrue(all(len(candidate) == 6 and len(reference) == 6
                            for candidate, reference in batches[0]))
        self.assertEqual(self.validity_requests, [])

    def test_fidelity_pairs_cover_the_interior_of_every_final_interval(self):
        path = [{"time_s": t, "path_progress": p, "positions": [q, 0, 0, 0, 0, 0]}
                for t, p, q in [(0, 0, 0), (1, .25, .005), (3, 1, .02)]]
        profile = runner.compile_profile(path, limit_class="print")

        pairs = list(runner.paired_profile_samples(profile))
        references = [reference[0] for _candidate, reference in pairs]

        self.assertTrue(any(0 < value < .005 for value in references))
        self.assertTrue(any(.005 < value < .02 for value in references))
        self.assertTrue(any(
            abs(candidate[0] - reference[0]) > 1.0e-10
            for candidate, reference in pairs
        ), "The regression must exercise quintic interior points, not knots only")
        self.assertEqual(pairs[0][0], pairs[0][1])
        self.assertEqual(pairs[-1][0], pairs[-1][1])

    def test_intro_fidelity_reference_is_the_exact_candidate(self):
        profile = runner.compile_profile([
            {"time_s": 0.0, "positions": [0.0] * 6},
            {"time_s": 0.25, "positions": [.01] * 6},
        ], limit_class="approach")

        self.assertTrue(all(
            candidate == reference
            for candidate, reference in runner.paired_profile_samples(
                profile, reference_is_candidate=True
            )
        ))

    def test_fidelity_reference_keeps_original_moveit_features_not_only_final_knots(self):
        path = [{"time_s": t, "path_progress": p, "positions": [q, 0, 0, 0, 0, 0]}
                for t, p, q in [(0, 0, 0), (2, 1, .02)]]
        reviewed = [path[0], {"path_progress": .5, "positions": [.015, 0, 0, 0, 0, 0]}, path[-1]]
        profile = runner.compile_profile(path)
        chord_pairs = list(runner.paired_profile_samples(profile))
        reviewed_pairs = list(runner.paired_profile_samples(profile, reviewed_path=reviewed))
        self.assertEqual([p[0] for p in chord_pairs], [p[0] for p in reviewed_pairs])
        self.assertTrue(any(abs(a[1][0]-b[1][0]) > .001
                            for a, b in zip(chord_pairs, reviewed_pairs)),
                        "Removing support knots must not remove the original FK fidelity reference")
        self.assertEqual(reviewed_pairs[0][1], reviewed[0]["positions"])
        self.assertEqual(reviewed_pairs[-1][1], reviewed[-1]["positions"])

    @unittest.skipIf(os.name == "nt", "Executes a POSIX local-validator shebang fixture")
    def test_local_validator_protocol_certifies_every_serialized_sample(self):
        validator = self.root / "validate_joint_samples"
        validator.write_text(
            f"#!{sys.executable}\n"
            "import csv, pathlib, sys\n"
            "params = dict(arg.split(':=', 1) for arg in sys.argv if ':=' in arg)\n"
            f"assert params['printhead_geometry_sha256'] == {runner.PRINTHEAD_GEOMETRY_SHA256!r}\n"
            "expected = {\n"
            " 'bed_min_x_m': -.5175, 'bed_max_x_m': -.0675,\n"
            " 'bed_min_y_m': -.27, 'bed_max_y_m': .27,\n"
            " 'bed_plane_z_m': -.014, 'tool_collision_radius_m': .02,\n"
            " 'tcp_offset_x_m': 0, 'tcp_offset_y_m': 0, 'tcp_offset_z_m': .1}\n"
            "assert all('.' in params[key] or 'e' in params[key].lower() "
            "for key in expected)\n"
            "assert all(abs(float(params[key]) - value) < 1e-12 "
            "for key, value in expected.items())\n"
            "assert int(params['sample_csv_count']) == 1\n"
            "value = params['sample_csv_0']\n"
            "with pathlib.Path(value).open(newline='') as stream:\n"
            "    rows = list(csv.reader(stream))\n"
            "expected_header = ([f'candidate_{name}' for name in "
            f"{list(runner.UR_JOINTS)!r}] + [f'reference_{{name}}' for name in "
            f"{list(runner.UR_JOINTS)!r}])\n"
            "assert rows[0] == expected_header\n"
            "assert all(len(row) == 12 for row in rows[1:])\n"
            "count = len(rows) - 1\n"
            "print('VALIDATION_PROTOCOL=3')\n"
            f"print('PRINTHEAD_GEOMETRY_SHA256={runner.PRINTHEAD_GEOMETRY_SHA256}')\n"
            "print('VALIDATED_BATCHES=1')\n"
            "print(f'VALIDATED_SAMPLES={count}')\n"
            "print(f'FIDELITY_SAMPLES={count}')\n"
            "print('MAX_TOOL0_TRANSLATION_M=0.0002')\n"
            "print('RMS_TOOL0_TRANSLATION_M=0.0001')\n"
            "print('MAX_TOOL0_ROTATION_RAD=0.002')\n"
            "print('RMS_TOOL0_ROTATION_RAD=0.001')\n",
            encoding="utf-8",
        )
        validator.chmod(0o755)
        self.executor.local_collision_validator_path = validator
        self.executor.manifest["execution_id"] = "local-validator-test"
        self.executor.manifest["tool_center_point_m"] = {
            "collision_radius_m": .02,
            "offset": {"x": 0, "y": 0, "z": .1},
        }
        self.executor.require_live_program = lambda: None
        samples = [
            ([0.0] * 6, [0.0] * 6),
            ([0.001] * 6, [0.0009] * 6),
            ([0.002] * 6, [0.0019] * 6),
        ]

        with mock.patch.object(runner, "time", real_time):
            checked = self.executor._validate_collision_samples_locally(
                iter(samples), real_time.monotonic() + 5.0
            )

        self.assertEqual(checked, len(samples))
        self.assertEqual(self.executor._last_fidelity_metrics, {
            "samples": len(samples),
            "maximum_translation_m": .0002,
            "rms_translation_m": .0001,
            "maximum_rotation_rad": .002,
            "rms_rotation_rad": .001,
            "translation_limit_m": runner.TOOL0_FIDELITY_TRANSLATION_LIMIT_M,
            "rotation_limit_rad": runner.TOOL0_FIDELITY_ROTATION_LIMIT_RAD,
            "passed": True,
        })
        self.assertEqual(list(self.root.glob("*.joint-samples.csv")), [])

    @unittest.skipIf(os.name == "nt", "Executes a POSIX local-validator shebang fixture")
    def test_local_validator_chunks_large_curves_without_losing_samples(self):
        invocations = self.root / "validator-invocations.txt"
        validator = self.root / "validate_joint_samples"
        validator.write_text(
            f"#!{sys.executable}\n"
            "import csv, pathlib, sys\n"
            "params = dict(arg.split(':=', 1) for arg in sys.argv if ':=' in arg)\n"
            "assert 'sample_csv' not in params\n"
            "paths = [params[f'sample_csv_{index}'] for index in "
            "range(int(params['sample_csv_count']))]\n"
            "counts = []\n"
            "for path in paths:\n"
            "    with pathlib.Path(path).open(newline='') as stream:\n"
            "        counts.append(sum(1 for _row in csv.reader(stream)) - 1)\n"
            "count = sum(counts)\n"
            f"with pathlib.Path({str(invocations)!r}).open('a') as stream:\n"
            "    stream.write(','.join(str(value) for value in counts) + '\\n')\n"
            "print('VALIDATION_PROTOCOL=3')\n"
            f"print('PRINTHEAD_GEOMETRY_SHA256={runner.PRINTHEAD_GEOMETRY_SHA256}')\n"
            "print(f'VALIDATED_BATCHES={len(paths)}')\n"
            "print(f'VALIDATED_SAMPLES={count}')\n"
            "print(f'FIDELITY_SAMPLES={count}')\n"
            "print('MAX_TOOL0_TRANSLATION_M=0.0002')\n"
            "print('RMS_TOOL0_TRANSLATION_M=0.0001')\n"
            "print('MAX_TOOL0_ROTATION_RAD=0.002')\n"
            "print('RMS_TOOL0_ROTATION_RAD=0.001')\n",
            encoding="utf-8",
        )
        validator.chmod(0o755)
        self.executor.local_collision_validator_path = validator
        self.executor.manifest["execution_id"] = "chunked-validator-test"
        self.executor.require_live_program = lambda: None
        samples = [([index * .001] * 6, [index * .001] * 6) for index in range(5)]

        with mock.patch.object(runner, "time", real_time), mock.patch.object(
            runner, "LOCAL_VALIDATION_BATCH_SAMPLES", 2, create=True
        ):
            checked = self.executor._validate_collision_samples_locally(
                iter(samples), real_time.monotonic() + 10.0
            )

        self.assertEqual(checked, 5)
        self.assertEqual(invocations.read_text(encoding="utf-8").splitlines(), ["2,2,1"])
        self.assertEqual(list(self.root.glob("*.joint-samples.csv")), [])

    @unittest.skipIf(os.name == "nt", "Executes a POSIX local-validator shebang fixture")
    def test_local_validator_never_treats_invalid_none_sample_as_end_of_stream(self):
        validator = self.root / "validate_joint_samples"
        marker = self.root / "validator-ran.txt"
        validator.write_text(
            f"#!{sys.executable}\nfrom pathlib import Path\n"
            f"Path({str(marker)!r}).write_text('ran')\n",
            encoding="utf-8",
        )
        validator.chmod(0o755)
        self.executor.local_collision_validator_path = validator
        self.executor.manifest["execution_id"] = "none-sentinel-test"
        self.executor.require_live_program = lambda: None
        samples = iter([([0.0] * 6, [0.0] * 6), None])

        with mock.patch.object(runner, "time", real_time), mock.patch.object(
            runner, "LOCAL_VALIDATION_BATCH_SAMPLES", 1
        ), mock.patch.object(runner, "MAX_LOCAL_VALIDATION_BATCH_FILES", 1):
            with self.assertRaisesRegex(runner.ExecutionFailure, "batch limit"):
                self.executor._validate_collision_samples_locally(
                    samples, real_time.monotonic() + 5.0
                )

        self.assertFalse(marker.exists())
        self.assertEqual(list(self.root.glob("*.joint-samples.csv")), [])

    @unittest.skipIf(os.name == "nt", "Executes a POSIX local-validator shebang fixture")
    def test_local_validator_cancellation_during_batch_creation_cleans_up(self):
        validator = self.root / "validate_joint_samples"
        marker = self.root / "validator-ran.txt"
        validator.write_text(
            f"#!{sys.executable}\nfrom pathlib import Path\n"
            f"Path({str(marker)!r}).write_text('ran')\n",
            encoding="utf-8",
        )
        validator.chmod(0o755)
        self.executor.local_collision_validator_path = validator
        self.executor.manifest["execution_id"] = "batch-cancel-test"
        self.executor.require_live_program = lambda: None
        self.executor.cancel_requested.set()
        samples = (([0.0] * 6, [0.0] * 6) for _index in range(8192))

        with mock.patch.object(runner, "time", real_time), self.assertRaisesRegex(
            runner.ExecutionFailure, "canceled"
        ):
            self.executor._validate_collision_samples_locally(
                samples, real_time.monotonic() + 5.0
            )

        self.assertFalse(marker.exists())
        self.assertEqual(list(self.root.glob("*.joint-samples.csv")), [])

    @unittest.skipIf(os.name == "nt", "Executes a POSIX local-validator shebang fixture")
    def test_local_validator_preserves_a_preexisting_conflicting_artifact(self):
        validator = self.root / "validate_joint_samples"
        marker = self.root / "validator-ran.txt"
        validator.write_text(
            f"#!{sys.executable}\nfrom pathlib import Path\n"
            f"Path({str(marker)!r}).write_text('ran')\n",
            encoding="utf-8",
        )
        validator.chmod(0o755)
        self.executor.local_collision_validator_path = validator
        execution_id = "preexisting-artifact-test"
        self.executor.manifest["execution_id"] = execution_id
        self.executor.require_live_program = lambda: None
        conflicting_path = self.root / (
            f".{execution_id}.{os.getpid()}.0.joint-samples.csv"
        )
        original = "owned by another execution\n"
        conflicting_path.write_text(original, encoding="utf-8")

        with mock.patch.object(runner, "time", real_time), self.assertRaisesRegex(
            runner.ExecutionFailure, "stale local MoveIt validation artifact"
        ):
            self.executor._validate_collision_samples_locally(
                iter([([0.0] * 6, [0.0] * 6)]), real_time.monotonic() + 5.0
            )

        self.assertFalse(marker.exists())
        self.assertEqual(conflicting_path.read_text(encoding="utf-8"), original)

    @unittest.skipIf(os.name == "nt", "Executes a POSIX local-validator shebang fixture")
    def test_final_curve_validation_chunks_more_than_200000_samples_without_loss(self):
        invocations = self.root / "large-validator-invocations.txt"
        validator = self.root / "validate_joint_samples"
        validator.write_text(
            f"#!{sys.executable}\n"
            "import csv, math, pathlib, sys\n"
            "params = dict(arg.split(':=', 1) for arg in sys.argv if ':=' in arg)\n"
            "assert 'sample_csv' not in params\n"
            "paths = [params[f'sample_csv_{index}'] for index in "
            "range(int(params['sample_csv_count']))]\n"
            "counts = []\n"
            "for path in paths:\n"
            "    with pathlib.Path(path).open(newline='') as stream:\n"
            "        counts.append(sum(1 for _row in csv.reader(stream)) - 1)\n"
            "count = sum(counts)\n"
            f"with pathlib.Path({str(invocations)!r}).open('a') as stream:\n"
            "    stream.write(','.join(str(value) for value in counts) + '\\n')\n"
            "if any(value > 200000 for value in counts):\n"
            "    print('VALIDATION_ERROR=sample_csv exceeds the 200000-state validation limit')\n"
            "    raise SystemExit(1)\n"
            "print('VALIDATION_PROTOCOL=3')\n"
            f"print('PRINTHEAD_GEOMETRY_SHA256={runner.PRINTHEAD_GEOMETRY_SHA256}')\n"
            "print(f'VALIDATED_BATCHES={len(paths)}')\n"
            "print(f'VALIDATED_SAMPLES={count}')\n"
            "print(f'FIDELITY_SAMPLES={count}')\n"
            "print(f'MAX_TOOL0_TRANSLATION_M={max(0.0004 if value == 1 else 0.0002 for value in counts)}')\n"
            "print(f'RMS_TOOL0_TRANSLATION_M={math.sqrt(sum(value * (0.0003 if value == 1 else 0.0001) ** 2 for value in counts) / count)}')\n"
            "print(f'MAX_TOOL0_ROTATION_RAD={max(0.004 if value == 1 else 0.002 for value in counts)}')\n"
            "print(f'RMS_TOOL0_ROTATION_RAD={math.sqrt(sum(value * (0.003 if value == 1 else 0.001) ** 2 for value in counts) / count)}')\n",
            encoding="utf-8",
        )
        validator.chmod(0o755)

        path = [
            {"time_s": 0.0, "positions": [0.0] * 6},
            {"time_s": 1.0, "positions": [0.005, 0.0, 0.0, 0.0, 0.0, 0.0]},
        ]
        self.package(path, approach=True)
        profile = self.executor.manifest["trajectory_profiles"]["approach"]
        self.executor.local_collision_validator_path = validator
        self.executor.manifest["execution_id"] = "oversized-chunked-validator-test"
        self.executor.require_live_program = lambda: None
        self.executor.current_positions = lambda: [0.0] * 6
        self.executor._collision_sample_validator = (
            self.executor._validate_collision_samples_locally
        )

        def oversized_samples(_profile, **kwargs):
            count = 1 if kwargs.get("reference_is_candidate") else 200_000
            for _index in range(count):
                yield [0.0] * 6, [0.0] * 6

        with mock.patch.object(runner, "time", real_time), mock.patch.object(
            runner, "paired_profile_samples", side_effect=oversized_samples
        ):
            self.executor._validate_smooth_motion(profile)

        invocation_lines = invocations.read_text(encoding="utf-8").splitlines()
        self.assertEqual(invocation_lines, ["200000,1"])
        batch_counts = [int(value) for value in invocation_lines[0].split(",")]
        self.assertEqual(batch_counts, [200_000, 1])
        self.assertEqual(sum(batch_counts), 200_001)
        self.assertEqual(self.executor.status["smooth_validation"]["samples"], 200_001)
        metrics = self.executor._last_fidelity_metrics
        self.assertEqual(metrics["samples"], 200_001)
        self.assertEqual(metrics["maximum_translation_m"], 0.0004)
        self.assertEqual(metrics["maximum_rotation_rad"], 0.004)
        self.assertAlmostEqual(
            metrics["rms_translation_m"],
            math.sqrt((200_000 * 0.0001**2 + 0.0003**2) / 200_001),
            places=12,
        )
        self.assertAlmostEqual(
            metrics["rms_rotation_rad"],
            math.sqrt((200_000 * 0.001**2 + 0.003**2) / 200_001),
            places=12,
        )
        self.assertTrue(metrics["passed"])
        self.assertEqual(
            [path for path in self.root.iterdir() if path.name.endswith(".joint-samples.csv")],
            [],
        )

    @unittest.skipIf(os.name == "nt", "Executes a POSIX local-validator shebang fixture")
    def test_local_validator_fails_closed_on_excessive_tcp_fidelity_error(self):
        validator = self.root / "validate_joint_samples"
        validator.write_text(
            f"#!{sys.executable}\n"
            "print('VALIDATION_PROTOCOL=3')\n"
            f"print('PRINTHEAD_GEOMETRY_SHA256={runner.PRINTHEAD_GEOMETRY_SHA256}')\n"
            "print('VALIDATED_BATCHES=1')\n"
            "print('VALIDATED_SAMPLES=1')\n"
            "print('FIDELITY_SAMPLES=1')\n"
            "print('MAX_TOOL0_TRANSLATION_M=0.0005001')\n"
            "print('RMS_TOOL0_TRANSLATION_M=0.0002')\n"
            "print('MAX_TOOL0_ROTATION_RAD=0.004')\n"
            "print('RMS_TOOL0_ROTATION_RAD=0.002')\n",
            encoding="utf-8",
        )
        validator.chmod(0o755)
        self.executor.local_collision_validator_path = validator
        self.executor.manifest["execution_id"] = "local-validator-fidelity-failure"
        self.executor.require_live_program = lambda: None

        with mock.patch.object(runner, "time", real_time), self.assertRaisesRegex(
            runner.ExecutionFailure, "TCP fidelity"
        ):
            self.executor._validate_collision_samples_locally(
                iter([([0.0] * 6, [0.0] * 6)]), real_time.monotonic() + 5.0
            )

        self.assertEqual(
            self.executor.status["smooth_validation"]["tcp_fidelity"]["passed"],
            False,
        )

    @unittest.skipIf(os.name == "nt", "Executes a POSIX local-validator shebang fixture")
    def test_local_validator_rejects_an_incomplete_certificate(self):
        validator = self.root / "validate_joint_samples"
        validator.write_text(
            f"#!{sys.executable}\nprint('VALIDATED_SAMPLES=0')\n",
            encoding="utf-8",
        )
        validator.chmod(0o755)
        self.executor.local_collision_validator_path = validator
        self.executor.manifest["execution_id"] = "local-validator-test"
        self.executor.require_live_program = lambda: None

        with mock.patch.object(runner, "time", real_time), self.assertRaisesRegex(
            runner.ExecutionFailure, "did not certify every"
        ):
            self.executor._validate_collision_samples_locally(
                iter([([0.0] * 6, [0.0] * 6)]), real_time.monotonic() + 5.0
            )

        self.assertEqual(list(self.root.glob("*.joint-samples.csv")), [])

    @unittest.skipIf(os.name == "nt", "Executes a POSIX local-validator shebang fixture")
    def test_local_validator_certificate_cannot_reuse_another_geometry(self):
        validator = self.root / "validate_joint_samples"
        certificate = {
            "VALIDATION_PROTOCOL": 3,
            "PRINTHEAD_GEOMETRY_SHA256": "0" * 64,
            "VALIDATED_BATCHES": 1, "VALIDATED_SAMPLES": 1, "FIDELITY_SAMPLES": 1,
            "MAX_TOOL0_TRANSLATION_M": 0, "RMS_TOOL0_TRANSLATION_M": 0,
            "MAX_TOOL0_ROTATION_RAD": 0, "RMS_TOOL0_ROTATION_RAD": 0,
        }
        validator.write_text(
            f"#!{sys.executable}\n" + "\n".join(
                f"print({(key + '=' + str(value))!r})" for key, value in certificate.items()
            ) + "\n", encoding="utf-8",
        )
        validator.chmod(0o755)
        self.executor.local_collision_validator_path = validator
        self.executor.manifest["execution_id"] = "wrong-geometry-certificate"
        self.executor.require_live_program = lambda: None
        with mock.patch.object(runner, "time", real_time):
            with self.assertRaisesRegex(runner.ExecutionFailure, "obsolete printhead geometry"):
                self.executor._validate_collision_samples_locally(
                    iter([([0.0] * 6, [0.0] * 6)]), real_time.monotonic() + 5.0,
                )
        self.assertEqual(list(self.root.glob("*.joint-samples.csv")), [])

    def test_moveit_validity_requests_respect_the_service_queue_depth(self):
        """Do not overflow the installed service client's ten-sample history.

        Fast DDS can leave surplus concurrent service requests unresolved when
        more than the available request/response history is submitted at once.
        Eight gives both sides headroom while retaining batched validation.
        """
        class QueueDepthFuture:
            def __init__(self, client):
                self.client = client

            def done(self):
                return self.client.outstanding <= 8

            def result(self):
                self.client.outstanding -= 1
                return NS(valid=True)

        class QueueDepthClient:
            def __init__(self):
                self.outstanding = 0
                self.maximum_outstanding = 0

            def wait_for_service(self, **_kwargs):
                return True

            def call_async(self, _request):
                self.outstanding += 1
                self.maximum_outstanding = max(self.maximum_outstanding, self.outstanding)
                return QueueDepthFuture(self)

        client = QueueDepthClient()
        self.executor.state_validity_client = client
        path = [{"time_s": t, "positions": [q, 0, 0, 0, 0, 0]}
                for t, q in [(0, 0), (1, .005), (3, .02)]]
        self.package(path, approach=True)

        self.executor.move_to_start(path)

        self.assertLessEqual(client.maximum_outstanding, 8)
        self.assertEqual(client.outstanding, 0)
        self.assertEqual(len(self.sent), 1)

    def test_move_to_start_reports_measured_progress_before_completion(self):
        path = [{"time_s": t, "positions": [q, 0, 0, 0, 0, 0]}
                for t, q in [(0, 0), (1, .005), (3, .02)]]
        self.package(path, approach=True)
        profile = self.executor.manifest["trajectory_profiles"]["approach"]
        result_future = Future()
        handle = NS(
            accepted=True,
            goal_id=NS(uuid=bytes(range(16))),
            get_result_async=lambda: result_future,
        )

        def send(goal, **_kwargs):
            self.sent.append(goal)
            return resolved(handle)

        self.executor.ur_client = NS(send_goal_async=send)
        snapshots = []
        self.executor._write_status = lambda: snapshots.append(
            json.loads(json.dumps(self.executor.status))
        )

        def observe(point):
            self.executor.latest_joints = dict(zip(runner.UR_JOINTS, point["positions"]))
            self.executor.latest_joint_stamp = self.clock.now

        self.clock.later(2.0, lambda: observe(profile["points"][1]))
        self.clock.later(2.4, lambda: observe(profile["points"][-1]))
        self.clock.later(2.5, lambda: result_future.set_result(
            NS(status=4, result=NS(error_code=0, error_string=""))
        ))

        self.executor.move_to_start(path)

        measured = [item for item in snapshots
                    if item.get("state") == "running" and 0 < item.get("progress", 0) < 1]
        self.assertTrue(measured, "Live joint motion must make approach progress visible before completion")
        self.assertTrue(any(item.get("motion", {}).get("source") == "live_joint_state"
                            for item in measured))

    def test_move_to_start_fails_closed_when_an_accepted_goal_never_moves(self):
        path = [{"time_s": t, "positions": [q, 0, 0, 0, 0, 0]}
                for t, q in [(0, 0), (1, .005), (3, .02)]]
        self.package(path, approach=True)
        start = list(self.executor.manifest["trajectory_profiles"]["approach"]["points"][0]["positions"])
        result_future = Future()
        handle = NS(
            accepted=True,
            goal_id=NS(uuid=bytes(range(16))),
            get_result_async=lambda: result_future,
        )
        self.executor.ur_client = NS(send_goal_async=lambda _goal, **_kwargs: resolved(handle))
        self.executor.require_live_program = lambda: None
        self.executor.current_positions = lambda: list(start)
        stop_calls = []
        self.executor._abort_active_goals = lambda **_kwargs: (stop_calls.append(True) or (True, "stopped"))

        started = self.clock.now
        with self.assertRaisesRegex(runner.ExecutionFailure, "accepted.*no measured joint motion"):
            self.executor.move_to_start(path)

        self.assertTrue(stop_calls, "An accepted but stationary goal must be canceled")
        self.assertLess(
            self.clock.now - started,
            self.executor.manifest["trajectory_profiles"]["approach"]["duration_s"] + 10.0,
            "The no-start watchdog must fire before the whole trajectory timeout",
        )

    def test_socket_drop_after_ur_ack_fails_before_no_motion_timeout(self):
        path = [{"time_s": t, "positions": [q, 0, 0, 0, 0, 0]}
                for t, q in [(0, 0), (1, .005), (3, .02)]]
        self.package(path, approach=True)
        start = list(self.executor.manifest["trajectory_profiles"]["approach"]["points"][0]["positions"])
        result_future = Future()
        handle = NS(accepted=True, goal_id=NS(uuid=bytes(range(16))),
                    get_result_async=lambda: result_future)
        def send(_goal, **_kwargs):
            self.clock.later(0.5, lambda: self.executor._trajectory_socket_callback(NS(data=False)))
            return resolved(handle)
        self.executor.ur_client = NS(send_goal_async=send)
        self.executor.current_positions = lambda: list(start)
        stop_calls = []
        self.executor._abort_active_goals = lambda **_kwargs: (stop_calls.append(True) or (True, "stopped"))

        started = self.clock.now
        with self.assertRaisesRegex(runner.ExecutionFailure, "trajectory socket"):
            self.executor.move_to_start(path)

        self.assertTrue(stop_calls, "A lost trajectory socket must cancel the accepted UR goal")
        self.assertLess(self.clock.now - started, runner.MOTION_START_GRACE_S)

    def test_start_tolerance_offset_is_not_mistaken_for_measured_motion(self):
        path = [{"time_s": t, "positions": [q, 0, 0, 0, 0, 0]}
                for t, q in [(0, 0), (1, .005), (3, .02)]]
        self.package(path, approach=True)
        baseline = [.02, 0, 0, 0, 0, 0]
        self.executor.latest_joints = dict(zip(runner.UR_JOINTS, baseline))
        self.executor.current_positions = lambda: list(baseline)
        result_future = Future()
        handle = NS(accepted=True, goal_id=NS(uuid=bytes(range(16))),
                    get_result_async=lambda: result_future)
        self.executor.ur_client = NS(send_goal_async=lambda _goal, **_kwargs: resolved(handle))
        self.executor.require_live_program = lambda: None
        self.executor._abort_active_goals = lambda **_kwargs: (True, "stopped")

        with self.assertRaisesRegex(runner.ExecutionFailure, "accepted.*no measured joint motion"):
            self.executor.move_to_start(path)

    def test_progress_projection_cannot_jump_to_a_later_returning_pose(self):
        profile = {
            "duration_s": 20.0,
            "points": [
                {"time_s": 0.0, "positions": [0.0] * 6,
                 "velocities": [0.0] * 6, "accelerations": [0.0] * 6},
                {"time_s": 10.0, "positions": [1.0] * 6,
                 "velocities": [0.0] * 6, "accelerations": [0.0] * 6},
                {"time_s": 20.0, "positions": [.0001] * 6,
                 "velocities": [0.0] * 6, "accelerations": [0.0] * 6},
            ],
        }

        progress, profile_time, _error = self.executor._measured_profile_progress(
            profile, [.0001] * 6, minimum_time_s=0.0, maximum_time_s=1.0
        )

        self.assertLessEqual(profile_time, 1.0)
        self.assertLessEqual(progress, 0.05)

    def test_full_profile_progress_reads_only_the_reachable_time_window(self):
        reads = []

        class Point(dict):
            def __getitem__(self, key):
                if key == "time_s":
                    reads.append(True)
                return super().__getitem__(key)

        profile = {
            "duration_s": 10000.0,
            "points": [Point(
                time_s=float(index), positions=[index * 1.0e-5] * 6,
                velocities=[1.0e-5] * 6, accelerations=[0.0] * 6,
            ) for index in range(10001)],
        }
        progress, profile_time, error = self.executor._measured_profile_progress(
            profile, [.05] * 6, minimum_time_s=4999.4, maximum_time_s=5000.6
        )
        self.assertAlmostEqual(progress, .5)
        self.assertAlmostEqual(profile_time, 5000.0)
        self.assertLess(error, 1.0e-12)
        self.assertLess(len(reads), 100, "Telemetry must not scan a whole full-file goal")

    def test_aborted_action_cannot_succeed_even_with_a_zero_result_code(self):
        stopped = []
        self.executor._abort_active_goals = lambda **_kwargs: stopped.append(True)
        for label, result in (
            ("UR", NS(error_code=0, error_string="controller aborted")),
            ("Klipper", NS(success=True, message="controller aborted")),
        ):
            with self.subTest(label=label), self.assertRaisesRegex(
                runner.ExecutionFailure, "motion failed"
            ):
                self.executor._await_results([
                    (label, object(), resolved(NS(status=6, result=result)))
                ], 1.0)
        self.assertEqual(len(stopped), 2)

    def test_trajectory_goal_has_explicit_six_joint_endpoint_tolerances(self):
        path = [{"time_s": t, "positions": [q, 0, 0, 0, 0, 0]}
                for t, q in [(0, 0), (1, .005), (3, .02)]]
        self.package(path, approach=True)

        self.executor.move_to_start(path)

        goal = self.sent[0]
        self.assertEqual([item.name for item in goal.goal_tolerance], list(runner.UR_JOINTS))
        self.assertTrue(all(item.position == runner.ENDPOINT_TOLERANCE_RAD
                            for item in goal.goal_tolerance))

    def test_ur_success_without_fresh_endpoint_match_still_fails(self):
        path = [{"time_s": t, "positions": [q, 0, 0, 0, 0, 0]}
                for t, q in [(0, 0), (1, .005), (3, .02)]]
        self.package(path, approach=True)
        result_future = Future()
        handle = NS(accepted=True, goal_id=NS(uuid=bytes(range(16))),
                    get_result_async=lambda: result_future)
        self.executor.ur_client = NS(send_goal_async=lambda _goal, **_kwargs: resolved(handle))
        self.executor.require_live_program = lambda: None
        self.executor.current_positions = lambda: [0.0] * 6
        self.clock.later(1.0, lambda: result_future.set_result(
            NS(status=4, result=NS(error_code=0, error_string=""))
        ))

        with self.assertRaisesRegex(runner.ExecutionFailure, "endpoint"):
            self.executor.move_to_start(path)


if __name__ == "__main__":
    unittest.main()
