// Offline regression for the real hardware interface's trajectory write guard.
// No ROS node, socket, or robot is involved.

#include <atomic>
#include <cstddef>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "ur_robot_driver/passthrough_trajectory_transport.hpp"

namespace
{
void require(bool condition, const char* message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void test_failed_start_never_marks_trajectory_started()
{
  double transfer_state = 1.0;
  double abort = 0.0;
  bool started = false;
  std::size_t sent = 0;
  std::atomic_bool transport_failed{ false };
  int cancel_attempts = 0;
  ur_robot_driver::PassthroughTransportState state{ transfer_state, abort, started, sent, transport_failed };

  const auto result = ur_robot_driver::write_passthrough_start(
      state, [] { return true; }, [] { return false; }, [&] { ++cancel_attempts; return true; });

  require(result == ur_robot_driver::PassthroughStartResult::WRITE_FAILED, "failed START must report write failure");
  require(!started && sent == 0, "failed START must not claim any transfer progress");
  require(abort == ur_robot_driver::PASSTHROUGH_TRANSPORT_FAILURE_ABORT && transfer_state == 5.0,
          "failed START must abort the action as a transport failure");
  require(transport_failed, "failed START must survive a later cancellation callback");
  require(cancel_attempts == 1, "failed START must attempt UR cancellation");
}

void test_disconnected_point_interface_prevents_start()
{
  double transfer_state = 1.0;
  double abort = 0.0;
  bool started = false;
  std::size_t sent = 0;
  std::atomic_bool transport_failed{ false };
  int start_attempts = 0;
  int cancel_attempts = 0;
  ur_robot_driver::PassthroughTransportState state{ transfer_state, abort, started, sent, transport_failed };

  const auto result = ur_robot_driver::write_passthrough_start(
      state, [] { return false; }, [&] { ++start_attempts; return true; },
      [&] { ++cancel_attempts; return true; });

  require(result == ur_robot_driver::PassthroughStartResult::DISCONNECTED,
          "disconnected trajectory point interface must have a distinct result");
  require(start_attempts == 0, "must not send START when point interface is disconnected");
  require(!started && sent == 0 && abort == ur_robot_driver::PASSTHROUGH_TRANSPORT_FAILURE_ABORT &&
              transfer_state == 5.0 && transport_failed,
          "disconnected point interface must fail closed before any transfer progress");
  require(cancel_attempts == 1, "disconnected point interface must attempt cancellation");
}

void test_failed_point_never_advances_count()
{
  double transfer_state = 1.0;
  double abort = 0.0;
  bool started = true;
  std::size_t sent = 7;
  std::atomic_bool transport_failed{ false };
  int cancel_attempts = 0;
  ur_robot_driver::PassthroughTransportState state{ transfer_state, abort, started, sent, transport_failed };

  const bool ok = ur_robot_driver::write_passthrough_point(
      state, [] { return false; }, [&] { ++cancel_attempts; return true; });

  require(!ok, "failed point write must return failure");
  require(sent == 7, "failed point write must not advance sent count");
  require(abort == ur_robot_driver::PASSTHROUGH_TRANSPORT_FAILURE_ABORT && transfer_state == 5.0,
          "failed point write must abort the action as a transport failure");
  require(transport_failed, "failed point write must survive a later cancellation callback");
  require(cancel_attempts == 1, "failed point write must attempt UR cancellation");
}

void test_successful_writes_advance_only_after_send()
{
  double transfer_state = 1.0;
  double abort = 0.0;
  bool started = false;
  std::size_t sent = 0;
  std::atomic_bool transport_failed{ false };
  int cancel_attempts = 0;
  ur_robot_driver::PassthroughTransportState state{ transfer_state, abort, started, sent, transport_failed };
  const auto cancel = [&] { ++cancel_attempts; return true; };

  require(ur_robot_driver::write_passthrough_start(state, [] { return true; }, [] { return true; }, cancel) ==
              ur_robot_driver::PassthroughStartResult::STARTED,
          "successful START must return success");
  require(started, "successful START must mark the trajectory started");
  for (std::size_t index = 0; index < 26279; ++index) {
    require(ur_robot_driver::write_passthrough_point(state, [] { return true; }, cancel),
            "successful point write must return success");
  }
  require(sent == 26279, "all 26279 points must be counted exactly once");
  require(abort == 0.0 && transfer_state == 1.0 && cancel_attempts == 0 && !transport_failed,
          "successful writes must not cancel or alter transfer state");
}

void test_approach_then_large_print_resets_transfer_progress()
{
  double transfer_state = 5.0;
  double abort = 0.0;
  bool started = true;
  std::size_t sent = 88;
  std::atomic_bool transport_failed{ false };
  ur_robot_driver::PassthroughTransportState state{ transfer_state, abort, started, sent, transport_failed };

  ur_robot_driver::begin_new_passthrough_transfer(state);
  require(transfer_state == 1.0 && abort == 0.0 && !started && sent == 0 && !transport_failed,
          "large print must start with fresh counters after an 88-point approach");
  require(ur_robot_driver::write_passthrough_start(state, [] { return true; }, [] { return true; },
                                                   [] { return true; }) ==
              ur_robot_driver::PassthroughStartResult::STARTED,
          "large print START must be accepted");
  for (std::size_t index = 0; index < 26279; ++index) {
    require(ur_robot_driver::write_passthrough_point(state, [] { return true; }, [] { return true; }),
            "large print point must be accepted");
  }
  require(started && sent == 26279, "large print must transfer all 26279 points from a clean state");
}

void test_control_frequency_must_match_the_configured_runtime()
{
  require(ur_robot_driver::valid_passthrough_control_frequency(125, 125.0),
          "configured 125 Hz control frequency must be accepted");
  require(ur_robot_driver::valid_passthrough_control_frequency(500, 0.0),
          "valid auto-negotiated control frequency must be accepted");
  require(!ur_robot_driver::valid_passthrough_control_frequency(0, 125.0),
          "a zero frequency must abort before the START threshold divides by it");
  require(!ur_robot_driver::valid_passthrough_control_frequency(500, 125.0),
          "a frequency that disagrees with the selected RTDE rate must abort before dispatch");
  require(!ur_robot_driver::valid_passthrough_control_frequency(2000, 0.0),
          "a corrupt but nonzero control frequency must not be used for START timing");
  require(!ur_robot_driver::valid_passthrough_control_frequency(
              125, std::numeric_limits<double>::quiet_NaN()),
          "a non-finite configured frequency must not be accepted");
}
}  // namespace

int main()
{
  try {
    test_failed_start_never_marks_trajectory_started();
    test_disconnected_point_interface_prevents_start();
    test_failed_point_never_advances_count();
    test_successful_writes_advance_only_after_send();
    test_approach_then_large_print_resets_transfer_progress();
    test_control_frequency_must_match_the_configured_runtime();
    std::cout << "passthrough trajectory transport: PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "passthrough trajectory transport: FAIL: " << error.what() << '\n';
    return 1;
  }
}
