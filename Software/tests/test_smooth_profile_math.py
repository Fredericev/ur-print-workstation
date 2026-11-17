"""ROS-free nominal quintic speed and spatial-curve regression boundaries.

Independent power-polynomial evaluation deliberately does not use the module's
Bernstein controls/bounds. These tests do not certify robot collisions or motion.
"""
from __future__ import annotations

import copy
import math
from pathlib import Path
import random
import struct
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[2] / "workspaces/ur_ros2_ws/scripts"
sys.path.insert(0, str(SCRIPTS))
import smooth_trajectory as smooth


def source(values, times=None):
    times = list(range(len(values))) if times is None else times
    return [{"time_s": time, "positions": [value] * 6} for time, value in zip(times, values)]


def power_coefficients(a, b, joint):
    """Solve the six q/v/a endpoint constraints in normalized power basis."""
    dt = b["time_s"] - a["time_s"]
    q0, q1 = a["positions"][joint], b["positions"][joint]
    v0, v1 = dt * a["velocities"][joint], dt * b["velocities"][joint]
    a0, a1 = dt * dt * a["accelerations"][joint], dt * dt * b["accelerations"][joint]
    delta = q1 - q0
    return [q0, v0, a0 / 2,
            10 * delta - 6 * v0 - 4 * v1 - 1.5 * a0 + 0.5 * a1,
            -15 * delta + 8 * v0 + 7 * v1 + 1.5 * a0 - a1,
            6 * delta - 3 * v0 - 3 * v1 - 0.5 * a0 + 0.5 * a1]


def power_state(coefficients, u, dt):
    """Evaluate q and three analytic derivatives without Bernstein helpers."""
    values = []
    for order in range(4):
        values.append(math.fsum(
            coefficients[degree] * math.factorial(degree) / math.factorial(degree - order)
            * u ** (degree - order)
            for degree in range(order, 6)
        ) / dt ** order)
    return values


def actual_urcl_wire(profile):
    """Independent installed URCL 2.11.0 encoding: C round, float time API."""
    def c_round(value):
        return math.copysign(math.floor(abs(value) + 0.5), value)
    result = copy.deepcopy(profile)
    elapsed_us = 0
    for index, point in enumerate(result["points"]):
        if index:
            dt = profile["points"][index]["time_s"] - profile["points"][index - 1]["time_s"]
            dt_float = struct.unpack("f", struct.pack("f", dt))[0]
            elapsed_us += int(c_round(dt_float * 1e6))
        point["time_s"] = elapsed_us / 1e6
        for field in ("positions", "velocities", "accelerations"):
            point[field] = [c_round(value * 1e6) / 1e6 for value in point[field]]
    result["duration_s"] = elapsed_us / 1e6
    return result


class DurationValidationTests(unittest.TestCase):
    def test_profile_rejects_nan_duration_instead_of_bypassing_comparison(self):
        profile = smooth.compile_profile(source([0.0, 0.01]))
        profile["duration_s"] = float("nan")
        with self.assertRaises(ValueError):
            smooth.validate_profile(profile)

    def test_bundle_rejects_nan_block_duration_instead_of_bypassing_sync_check(self):
        approach = smooth.compile_profile(source([0.0, 0.01]), limit_class="approach")
        profile = smooth.compile_profile(source([0.01, 0.02]))
        bundle = {
            "trajectory_policy_version": smooth.POLICY,
            "trajectory_profiles": {"approach": approach, "blocks": [profile]},
            "blocks": [{"duration_s": float("nan")}],
        }
        with self.assertRaises(ValueError):
            smooth.validate_bundle(bundle)

    def test_every_point_field_and_duration_reject_nonfinite_values(self):
        valid = smooth.compile_profile(source([0.0, 0.01]))
        for invalid in (float("nan"), float("inf"), -float("inf"), True):
            for field in ("positions", "velocities", "accelerations", "time_s", "duration_s"):
                with self.subTest(field=field, value=invalid):
                    profile = copy.deepcopy(valid)
                    if field == "duration_s":
                        profile[field] = invalid
                    elif field == "time_s":
                        profile["points"][-1][field] = invalid
                    else:
                        profile["points"][-1][field][2] = invalid
                    with self.assertRaises(ValueError):
                        smooth.validate_profile(profile)

    def test_duration_is_positive_and_matches_the_last_point(self):
        for invalid in (-1.0, 0.0, 123.0):
            profile = smooth.compile_profile(source([0.0, 0.01]))
            profile["duration_s"] = invalid
            with self.subTest(duration=invalid), self.assertRaises(ValueError):
                smooth.validate_profile(profile)

    def test_bundle_rejects_invalid_block_durations_and_disconnected_paths(self):
        approach = smooth.compile_profile(source([0.0, 0.01]), limit_class="approach")
        profile = smooth.compile_profile(source([0.01, 0.02]))
        valid = {"trajectory_policy_version": smooth.POLICY,
                 "trajectory_profiles": {"approach": approach, "blocks": [profile]},
                 "blocks": [{"duration_s": profile["duration_s"]}]}
        self.assertIs(smooth.validate_bundle(valid), valid["trajectory_profiles"])
        for invalid in (float("inf"), -1.0, 0.0, True, profile["duration_s"] + 1.0):
            candidate = copy.deepcopy(valid)
            candidate["blocks"][0]["duration_s"] = invalid
            with self.subTest(duration=invalid), self.assertRaises(ValueError):
                smooth.validate_bundle(candidate)
        candidate = copy.deepcopy(valid)
        candidate["trajectory_profiles"]["blocks"][0] = smooth.compile_profile(source([0.5, 0.51]))
        candidate["blocks"][0]["duration_s"] = candidate["trajectory_profiles"]["blocks"][0]["duration_s"]
        with self.assertRaisesRegex(ValueError, "disconnected"):
            smooth.validate_bundle(candidate)


class IndependentQuinticTests(unittest.TestCase):
    def assert_curve(self, profile, *, samples=101):
        points = profile["points"]
        limits = profile["limits"]
        corridor = profile["waypoint_corridor_rad"]
        self.assertTrue(math.isfinite(profile["duration_s"]))
        self.assertGreater(profile["duration_s"], 0.0)
        for point in points:
            self.assertTrue(math.isfinite(point["time_s"]))
            for field in ("positions", "velocities", "accelerations"):
                self.assertEqual(len(point[field]), 6)
                self.assertTrue(all(math.isfinite(value) for value in point[field]))
        for point in (points[0], points[-1]):
            self.assertEqual(point["velocities"], [0.0] * 6)
            self.assertEqual(point["accelerations"], [0.0] * 6)
        for a, b in zip(points, points[1:]):
            dt = b["time_s"] - a["time_s"]
            self.assertGreater(dt, 0.0)
            for joint in range(6):
                coefficients = power_coefficients(a, b, joint)
                for index in range(samples):
                    u = index / (samples - 1)
                    q, v, acceleration, jerk = power_state(coefficients, u, dt)
                    self.assertTrue(all(math.isfinite(value) for value in (q, v, acceleration, jerk)))
                    self.assertLessEqual(abs(v), limits["velocity"] + 1e-9)
                    self.assertLessEqual(abs(acceleration), limits["acceleration"] + 1e-9)
                    self.assertLessEqual(abs(jerk), limits["jerk"] + 1e-8)
                    reference = (
                        a["positions"][joint]
                        + u * (b["positions"][joint] - a["positions"][joint])
                    )
                    self.assertLessEqual(abs(q-reference), corridor + 1e-9)
                for u, point in ((0.0, a), (1.0, b)):
                    endpoint = power_state(coefficients, u, dt)
                    for actual, field in zip(endpoint, ("positions", "velocities", "accelerations")):
                        self.assertAlmostEqual(actual, point[field][joint], delta=1e-10)

    def test_rest_to_rest_polynomial_matches_known_halfway_position_and_derivatives(self):
        # Keep the chord below the endpoint corridor threshold so the public
        # compiler needs no line-preserving support points.
        profile = smooth.compile_profile(source([0.0, 0.0001]))
        a, b = profile["points"]
        dt = profile["duration_s"]
        q, v, acceleration, jerk = power_state(power_coefficients(a, b, 0), 0.5, dt)
        self.assertAlmostEqual(q, 0.00005)
        self.assertAlmostEqual(v, 1.875 * 0.0001 / dt)
        self.assertAlmostEqual(acceleration, 0.0)
        self.assertAlmostEqual(jerk, -30 * 0.0001 / dt ** 3)
        self.assert_curve(profile)

    def test_monotone_path_has_nonzero_interior_velocity_and_no_waypoint_overshoot(self):
        profile = smooth.compile_profile(source([0.0, 0.01, 0.013, 0.025], [0.0, 0.1, 0.7, 1.0]))
        velocities = [point["velocities"][0] for point in profile["points"][1:-1]]
        self.assertTrue(all(value >= 0.0 for value in velocities))
        self.assertTrue(any(value > 0.0 for value in velocities))
        self.assert_curve(profile)

    def test_only_a_real_stationary_interval_stops_the_continuous_curve(self):
        path = source([0.0, 0.01, 0.02, 0.02, 0.03], [0.0, 0.4, 0.8, 1.3, 1.7])
        path[1]["timeline_markers"] = ["ordinary"]
        path[2]["timeline_markers"] = ["hold:start"]
        path[3]["timeline_markers"] = ["hold:end"]

        profile = smooth.compile_profile(path)
        markers = {
            marker: point
            for point in profile["points"]
            for marker in point.get("timeline_markers", [])
        }

        self.assertGreater(markers["ordinary"]["velocities"][0], 0.0)
        self.assertEqual(markers["hold:start"]["velocities"], [0.0] * 6)
        self.assertEqual(markers["hold:end"]["velocities"], [0.0] * 6)
        self.assert_curve(profile)

    def test_fixed_geometry_retimer_absorbs_irregular_marker_timing_without_stopping(self):
        path = source(
            [0.0, 0.01, 0.02, 0.03, 0.04],
            [0.0, 0.01, 1.01, 1.02, 2.02],
        )
        path[2]["timeline_markers"] = ["process-boundary"]

        profile = smooth.compile_profile(path)
        marker = next(
            point for point in profile["points"]
            if point.get("timeline_markers") == ["process-boundary"]
        )

        self.assertLess(profile["duration_s"], 3.5)
        self.assertGreater(marker["velocities"][0], 0.0)
        self.assert_curve(profile)

    def test_seeded_mixed_axis_random_paths_stay_within_independent_derivative_limits(self):
        rng = random.Random(90731)
        for case in range(24):
            positions, elapsed, path = [rng.uniform(-2.5, 2.5) for _ in range(6)], 0.0, []
            for index in range(rng.randint(2, 9)):
                path.append({"time_s": elapsed, "positions": list(positions)})
                elapsed += 10 ** rng.uniform(-5, 0.1)
                positions = [value + rng.uniform(-0.025, 0.025) for value in positions]
            with self.subTest(case=case):
                self.assert_curve(smooth.compile_profile(path), samples=51)

    def test_long_curved_review_path_converges_without_false_rejection(self):
        # A Cartesian MoveIt approach becomes a long, curved sequence in joint
        # space.  Constraint changes can propagate across more than 64 adjacent
        # intervals even though a bounded profile exists.
        point_count = 80
        path = [
            {
                "time_s": index * 0.06,
                "positions": [1.5 * (index / (point_count - 1)) ** 3] + [0.0] * 5,
            }
            for index in range(point_count)
        ]

        profile = smooth.compile_profile(path)

        self.assert_curve(profile, samples=51)

    def test_approach_and_print_use_the_same_reviewed_robot_limits(self):
        point_count = 103
        path = [
            {
                "time_s": index * 0.09,
                "positions": [
                    0.55 * (index / (point_count - 1)) ** 2,
                    -0.35 * (index / (point_count - 1)),
                    0.0, 0.0, 0.0, 0.0,
                ],
            }
            for index in range(point_count)
        ]

        print_profile = smooth.compile_profile(path)
        approach_profile = smooth.compile_profile(path, limit_class="approach")

        self.assertEqual(print_profile["limit_class"], "print")
        self.assertEqual(print_profile["limits"], {
            "velocity": 0.2, "acceleration": 2.5, "jerk": 100.0,
        })
        self.assertEqual(approach_profile["limit_class"], "approach")
        self.assertEqual(approach_profile["limits"], {
            "velocity": 0.2, "acceleration": 2.5, "jerk": 100.0,
        })
        self.assertEqual(approach_profile["duration_s"], print_profile["duration_s"])
        self.assert_curve(print_profile, samples=31)
        self.assert_curve(approach_profile, samples=31)

    def test_bundle_requires_approach_and_print_profiles_in_their_declared_roles(self):
        approach = smooth.compile_profile(source([0.0, 0.01]), limit_class="approach")
        block = smooth.compile_profile(source([0.01, 0.02]))
        valid = {
            "trajectory_policy_version": smooth.POLICY,
            "trajectory_profiles": {"approach": approach, "blocks": [block]},
            "blocks": [{"duration_s": block["duration_s"]}],
        }
        self.assertIs(smooth.validate_bundle(valid), valid["trajectory_profiles"])
        valid["trajectory_profiles"]["blocks"][0] = smooth.compile_profile(
            source([0.01, 0.02]), limit_class="approach"
        )
        valid["blocks"][0]["duration_s"] = valid["trajectory_profiles"]["blocks"][0]["duration_s"]
        with self.assertRaisesRegex(ValueError, "limit class"):
            smooth.validate_bundle(valid)

    def test_tiny_source_intervals_are_finite_without_global_duration_explosion(self):
        profile = smooth.compile_profile(source([0.0, 0.002, 0.003, 0.004], [0.0, 1e-12, 0.2, 0.200000000001]))
        self.assertLess(profile["duration_s"], 60.0)
        self.assert_curve(profile)

    def test_time_stretch_preserves_waypoints_and_limits_after_wire_rounding(self):
        profile = smooth.compile_profile(source([0.0, 0.01, 0.012, -0.003]))
        requested = profile["duration_s"] * 1.71329
        stretched = smooth.stretch_profile(profile, requested)
        self.assert_curve(stretched)
        self.assertGreaterEqual(stretched["duration_s"], requested)
        # Wire-safe restretching may add line-preserving support points, but it
        # must retain every already-certified position in the same order.
        original = [p["positions"] for p in profile["points"]]
        stretched_positions = [p["positions"] for p in stretched["points"]]
        cursor = iter(stretched_positions)
        self.assertTrue(all(any(candidate == required for candidate in cursor)
                            for required in original))
        self.assertEqual(stretched["points"], actual_urcl_wire(stretched)["points"])

    def test_collision_sampler_matches_independent_power_curve_and_bounded_steps(self):
        profile = smooth.compile_profile(source([0.0, 0.0001]))
        samples = list(smooth.sample_profile(profile, spacing_rad=0.001))
        a, b = profile["points"]
        self.assertEqual(samples[0], a["positions"])
        self.assertEqual(samples[-1], b["positions"])
        for index, values in enumerate(samples):
            independent = power_state(power_coefficients(a, b, 0), index / (len(samples) - 1), profile["duration_s"])[0]
            self.assertAlmostEqual(values[0], independent, delta=1e-12)
        self.assertTrue(all(max(abs(y - x) for x, y in zip(a, b)) <= 0.001 + 1e-12
                            for a, b in zip(samples, samples[1:])))


class WireQuantizationTests(unittest.TestCase):
    assert_curve = IndependentQuinticTests.assert_curve

    def test_submicroradian_crossing_does_not_exceed_jerk_after_socket_rounding(self):
        profile = smooth.compile_profile(source([0.000000495, 0.000000505], [0.0, 0.02]))
        encoded = actual_urcl_wire(profile)
        a, b = encoded["points"]
        actual_jerk = abs(power_state(power_coefficients(a, b, 0), 0.0, b["time_s"] - a["time_s"])[3])
        self.assertLessEqual(actual_jerk, 100.0 + 1e-8)
        self.assert_curve(encoded)

    def test_generated_commands_are_unchanged_by_the_actual_wire_conversion(self):
        rng = random.Random(1211)
        for case in range(32):
            positions, elapsed, path = [rng.uniform(-3,3) for _ in range(6)], 0.0, []
            for _ in range(8):
                path.append({"time_s":elapsed,"positions":list(positions)})
                elapsed += 10 ** rng.uniform(-8,1)
                positions = [q+rng.uniform(-.02,.02) for q in positions]
            profile = smooth.compile_profile(path)
            encoded = actual_urcl_wire(profile)
            with self.subTest(case=case):
                self.assertEqual(profile["points"], encoded["points"])
                self.assert_curve(encoded, samples=51)

    def test_nonrepresentable_or_uncertified_profiles_are_rejected(self):
        valid = smooth.compile_profile(source([0,.01,.02]))
        for change in ("positions","velocities","time","metadata"):
            profile = copy.deepcopy(valid)
            if change == "metadata":
                profile.pop("wire_quantization")
            elif change == "time":
                profile["points"][1]["time_s"] += .0000001
            else:
                profile["points"][1][change][0] += .0000001
            with self.subTest(change=change), self.assertRaises(ValueError):
                smooth.validate_profile(profile)


if __name__ == "__main__":
    unittest.main()
