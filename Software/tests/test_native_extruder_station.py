"""Native extrusion readiness is read-only and rejects stale live settings."""

import unittest
from unittest import mock

from Software.tests.test_control_station import station


class NativeExtruderStationTests(unittest.TestCase):
    def setUp(self):
        self.backend = station.StationBackend.__new__(station.StationBackend)
        self.backend.demo = False
        self.manifest = {
            "extrusion_protocol": "klipper-native-extruder-v1", "extruder": "extruder",
            "extruder_config": {"nozzle_diameter": .2, "filament_diameter": 1.75,
                                "rotation_distance": 10., "min_extrude_temp": 170.},
            "stepper_limits": {"maximum_speed": 5., "acceleration": 20.,
                               "max_extrude_only_distance": 50.},
        }
        self.live = {
            **self.manifest["extruder_config"], "name": "extruder", "temperature": 200.,
            "can_extrude": True, "max_extrude_only_velocity": 5.,
            "max_extrude_only_accel": 20., "max_extrude_only_distance": 50.,
            "speed_factor": 1., "extrude_factor": 1.,
        }

    def check(self, live):
        with mock.patch.object(self.backend, "check_klipper", return_value={"ok": True, "extruder": live}):
            self.backend._require_native_extrusion_ready(self.manifest)

    def test_warm_matching_config_passes_without_dispatch(self):
        self.check(self.live)

    def test_cold_extruder_is_blocked_before_execution(self):
        with self.assertRaises(station.ApiError) as raised:
            self.check({**self.live, "can_extrude": False, "temperature": 25.})
        self.assertEqual(raised.exception.code, "extruder_too_cold")

    def test_changed_motor_geometry_limits_and_overrides_are_rejected(self):
        for key, value in (("rotation_distance", 12.), ("nozzle_diameter", .4),
                           ("max_extrude_only_velocity", 4.), ("max_extrude_only_accel", 19.),
                           ("max_extrude_only_distance", 40.), ("speed_factor", .9),
                           ("extrude_factor", 1.1), ("speed_factor", float("nan"))):
            with self.subTest(key=key, value=value), self.assertRaises(station.ApiError):
                self.check({**self.live, key: value})

    def test_legacy_profile_migrates_only_known_extrusion_motor(self):
        old = {**station.DEFAULT_CONFIG, "manual_stepper": "stepper_x", "stepper_speed": 50., "stepper_accel": 200.}
        migrated = station.validate_config(old)
        self.assertEqual((migrated["manual_stepper"], migrated["stepper_speed"], migrated["stepper_accel"]),
                         ("extruder", 5., 20.))
        migrated = station.validate_config({**old, "stepper_accel": 5., "ur_robot_ip": "192.168.0.105"})
        self.assertEqual(migrated["stepper_accel"], 20.)
        self.assertEqual(migrated["ur_robot_ip"], "192.168.0.105")
        with self.assertRaises(station.ApiError):
            station.validate_config({**old, "manual_stepper": "lock_motor"})


if __name__ == "__main__":
    unittest.main()
