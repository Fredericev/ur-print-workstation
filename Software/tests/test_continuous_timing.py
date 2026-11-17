"""Speed regressions at the exact ROS-free execution compiler seam.

Every candidate must pass the installed limits. This checks command timing,
not physical safety.
"""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "workspaces/ur_ros2_ws/scripts"))
import smooth_trajectory as smooth


def minimum_jerk_path(count):
    return [{"time_s": 5 * (u := i / (count - 1)),
             "positions": [.05 * (10*u**3 - 15*u**4 + 6*u**5)] + [0.] * 5}
            for i in range(count)]


class ContinuousTimingTests(unittest.TestCase):
    def test_timing_repair_finds_a_narrow_non_power_of_two_ratio_window(self):
        q = [
            [0.398787, -2.533877, -2.241448, 0.064517, 1.569515, 1.974064],
            [0.390791, -2.533834, -2.240837, 0.063865, 1.569517, 1.966067],
            [0.393973, -2.534703, -2.252256, 0.076152, 1.569476, 1.969260],
        ]
        intervals = [0.194825, 0.201622]

        smooth._repair_phase_timing(q, intervals, 0.0012)

        self.assertEqual(intervals[0], 0.194825)
        factor = intervals[1] / 0.201622
        self.assertGreaterEqual(factor, 1.1)
        self.assertLessEqual(factor, 1.4)
        target = smooth._corridor_state_target(0.0012, *intervals)
        for joint in range(6):
            self.assertIsNotNone(smooth._phase_state(
                q[1][joint]-q[0][joint],
                q[2][joint]-q[1][joint],
                *intervals,
                target,
                preserve_direction=True,
            ))

    def test_corridor_projection_does_not_accept_opposite_velocity_early(self):
        velocity, _acceleration = smooth._phase_state(
            0.001, 0.001, 1.0, 1.0, 0.01,
            desired_velocity=-0.001,
            preserve_direction=True,
        )
        self.assertGreaterEqual(velocity, 0.0)

    def test_same_direction_projection_does_not_manufacture_a_boundary_stop(self):
        for direction in (-1.0, 1.0):
            velocity, _acceleration = smooth._phase_state(
                direction*0.0007, direction*0.00015, 0.124, 0.099, 0.0012,
                desired_velocity=-direction*0.005,
                desired_acceleration=-direction*0.22,
                preserve_direction=True,
            )
            self.assertGreater(direction*velocity, 0.0)

    def test_dense_already_smooth_motion_is_not_restarted_at_every_waypoint(self):
        profiles = [smooth.compile_profile(minimum_jerk_path(n)) for n in (50, 100, 200)]
        for profile in profiles:
            self.assertLess(profile["duration_s"], 10.,
                            "A bounded 5s smooth move must not become a minute just by densifying it")
            smooth.validate_profile(profile)
        self.assertLess(max(p["duration_s"] for p in profiles)
                        / min(p["duration_s"] for p in profiles), 1.75,
                        "Allow micro-radian wire quantization, not waypoint-density stop inflation")
        self.assertTrue(any(abs(a) > 0 for p in profiles[1]["points"][1:-1]
                            for a in p["accelerations"]),
                        "Interior acceleration must be continuous, not reset to zero at every point")

    def test_source_and_minimum_durations_are_lower_bounds_not_speedup_targets(self):
        path = minimum_jerk_path(100)
        profile = smooth.compile_profile(path, 12.)
        self.assertGreaterEqual(profile["duration_s"], 12.)
        self.assertEqual(profile["points"][0]["positions"], [0.] * 6)
        self.assertEqual(profile["points"][-1]["positions"], [.05] + [0.] * 5)

    def test_shared_derivatives_recover_the_analytic_minimum_jerk_solution(self):
        path = minimum_jerk_path(11)
        velocities, accelerations = smooth._minimum_jerk_states(
            [p["positions"] for p in path], [.5] * 10)
        for i in range(11):
            u = i / 10
            self.assertAlmostEqual(velocities[i][0], .05/5 * (30*u*u - 60*u**3 + 30*u**4), places=10)
            self.assertAlmostEqual(accelerations[i][0], .05/25 * (60*u - 180*u*u + 120*u**3), places=10)

    def test_quantization_merges_no_distinct_wire_position_or_explicit_hold(self):
        path = minimum_jerk_path(200)
        profile = smooth.compile_profile(path)
        def distinct(values):
            return [v for i, v in enumerate(values) if i == 0 or values[i-1] != v]
        self.assertEqual(distinct([p["positions"] for p in profile["points"]]),
                         distinct([[smooth._wire_joint(v) for v in p["positions"]] for p in path]))
        hold = [{"time_s": 0., "positions": [0.] * 6},
                {"time_s": 2., "positions": [0.] * 6},
                {"time_s": 5., "positions": [.01] * 6}]
        result = smooth.compile_profile(hold)
        # Same-progress support points may be added on the following moving
        # chord, but the explicit two-second stationary interval is exact.
        self.assertGreaterEqual(len(result["points"]), 3)
        self.assertEqual(result["points"][0]["positions"], [0.] * 6)
        self.assertEqual(result["points"][1]["positions"], [0.] * 6)
        self.assertEqual(result["points"][1]["time_s"], 2.)
        self.assertEqual(result["points"][-1]["positions"], [.01] * 6)
        self.assertEqual(result["points"][1]["velocities"], [0.] * 6)
        self.assertEqual(result["points"][1]["accelerations"], [0.] * 6)

    def test_explicit_hold_survives_an_adjacent_sub_wire_motion(self):
        path = [{"time_s": 0., "positions": [0.] * 6},
                {"time_s": 2., "positions": [0.] * 6},
                {"time_s": 3., "positions": [.0000001] + [0.] * 5},
                {"time_s": 5., "positions": [.01] + [0.] * 5}]
        profile = smooth.compile_profile(path)
        self.assertEqual(profile["points"][0]["positions"], profile["points"][1]["positions"],
                         "Wire-redundant motion must not erase an explicit source hold boundary")
        self.assertGreaterEqual(profile["points"][1]["time_s"], 2.)
        self.assertEqual(profile["points"][1]["velocities"], [0.] * 6)
        self.assertEqual(profile["points"][1]["accelerations"], [0.] * 6)

    def test_process_marker_next_to_wire_redundant_waypoint_does_not_force_a_stop(self):
        path = [
            {"time_s": 0., "positions": [0.] * 6},
            {"time_s": 1., "positions": [.0050001] + [0.] * 5},
            {"time_s": 1.0001, "positions": [.0050002] + [0.] * 5,
             "timeline_markers": ["block:0:end", "block:1:start"]},
            {"time_s": 2., "positions": [.01] + [0.] * 5},
        ]
        profile = smooth.compile_profile(path)
        marker = next(p for p in profile["points"] if p.get("timeline_markers"))
        self.assertGreater(marker["velocities"][0], 0.)


if __name__ == "__main__":
    unittest.main()
