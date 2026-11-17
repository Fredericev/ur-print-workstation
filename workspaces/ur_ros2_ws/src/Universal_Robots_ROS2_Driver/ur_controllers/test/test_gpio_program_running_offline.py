"""Offline controller-update regression for the program-running status topic.

The ROS build is unavailable on this workstation. Compile the production update
and publication methods against tiny state/publisher stand-ins so their observable
publication behavior can still be checked without connecting to the robot.
"""

from pathlib import Path
import subprocess
import tempfile
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "src" / "gpio_controller.cpp"


def method(source: str, signature: str) -> str:
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 0
    for position in range(opening, len(source)):
        if source[position] == "{":
            depth += 1
        elif source[position] == "}":
            depth -= 1
            if depth == 0:
                return source[start : position + 1]
    raise AssertionError(f"Unclosed method: {signature}")


class GPIOProgramRunningOfflineTest(unittest.TestCase):
    def test_reactivation_restarts_initial_program_state_publication(self):
        source = SOURCE.read_text()
        activation = method(source, "ur_controllers::GPIOController::on_activate(")
        self.assertIn("program_running_status_published_ = false;", activation)

    def test_update_republishes_stable_state_and_reports_changes_immediately(self):
        source = SOURCE.read_text()
        update = method(source, "controller_interface::return_type ur_controllers::GPIOController::update(")
        publish = method(source, "void GPIOController::publishProgramRunning()")
        harness = r"""
#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

namespace rclcpp { struct Time {}; struct Duration {}; }
namespace controller_interface { enum class return_type { OK }; }
namespace ur_controllers {
struct StateInterfaces { enum { PROGRAM_RUNNING = 0 }; };
struct FakeState { double value = 0.0; double get_value() const { return value; } };
struct BoolMessage { bool data = false; };
struct FakePublisher {
  std::vector<bool> values;
  void publish(const BoolMessage& message) { values.push_back(message.data); }
};
class GPIOController {
public:
  std::array<FakeState, 1> state_interfaces_{};
  BoolMessage program_running_msg_{};
  std::shared_ptr<FakePublisher> program_state_pub_ = std::make_shared<FakePublisher>();
  std::chrono::steady_clock::time_point last_program_running_publish_{};
  bool program_running_status_published_{ false };
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&);
  void publishProgramRunning();
  void publishIO() {}
  void publishToolData() {}
  void publishRobotMode() {}
  void publishSafetyMode() {}
  void publishTrajectorySocketConnected() {}
};
}
""" + update + "\nnamespace ur_controllers {\n" + publish + r"""
}
int main() {
  ur_controllers::GPIOController controller;
  auto check = [&](std::vector<bool> expected) {
    if (controller.program_state_pub_->values == expected) return true;
    std::cerr << "Unexpected program-running publications: expected " << expected.size()
              << ", got " << controller.program_state_pub_->values.size() << '\n';
    return false;
  };
  controller.update({}, {});
  if (!check({false})) return 1;  // Initial stopped state must be observable.
  controller.update({}, {});
  if (!check({false})) return 2;  // No duplicate within the heartbeat interval.
  controller.last_program_running_publish_ = std::chrono::steady_clock::now() - std::chrono::milliseconds(101);
  controller.update({}, {});
  if (!check({false, false})) return 3;  // Late subscriber can get a fresh stopped state.
  controller.state_interfaces_[0].value = 1.0;
  controller.update({}, {});
  if (!check({false, false, true})) return 4;  // Transition bypasses heartbeat interval.
  controller.last_program_running_publish_ = std::chrono::steady_clock::now() - std::chrono::milliseconds(101);
  controller.update({}, {});
  if (!check({false, false, true, true})) return 5;  // Running state also repeats.
  controller.state_interfaces_[0].value = 0.0;
  controller.update({}, {});
  if (!check({false, false, true, true, false})) return 6;
}
"""
        with tempfile.TemporaryDirectory() as temporary:
            cpp = Path(temporary) / "gpio_program_running.cpp"
            executable = Path(temporary) / "gpio_program_running"
            cpp.write_text(harness)
            subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-o", str(executable), str(cpp)], check=True)
            subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    unittest.main()
