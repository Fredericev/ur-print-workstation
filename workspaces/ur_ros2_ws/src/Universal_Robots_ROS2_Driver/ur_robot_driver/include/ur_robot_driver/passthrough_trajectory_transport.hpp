#ifndef UR_ROBOT_DRIVER__PASSTHROUGH_TRAJECTORY_TRANSPORT_HPP_
#define UR_ROBOT_DRIVER__PASSTHROUGH_TRAJECTORY_TRANSPORT_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <utility>

namespace ur_robot_driver
{
// Shared command interface convention: 1 means a generic hardware abort,
// while 2 specifically identifies a failed trajectory transport write.
constexpr double PASSTHROUGH_TRANSPORT_FAILURE_ABORT = 2.0;

// A mismatched ur_client_library header/runtime can make the inline frequency
// getter read the wrong object field. Never turn that into an infinite START
// threshold (5 / 0) and a long trajectory that stays at 0% until timeout.
inline bool valid_passthrough_control_frequency(std::uint32_t observed_hz, double configured_hz)
{
  return observed_hz >= 1 && observed_hz <= 1000 && std::isfinite(configured_hz) && configured_hz >= 0.0 &&
         (configured_hz == 0.0 || std::fabs(static_cast<double>(observed_hz) - configured_hz) < 1.0);
}

// References point at the command interfaces and transfer counters used by the
// hardware write loop. A failed socket write is terminal for this goal: the
// peer may have received only part of a message, so retransmission is unsafe.
struct PassthroughTransportState
{
  double& transfer_state;
  double& abort;
  bool& started;
  std::size_t& points_sent;
  std::atomic_bool& failed;
};

inline void begin_new_passthrough_transfer(PassthroughTransportState state)
{
  state.failed.store(false, std::memory_order_release);
  state.abort = 0.0;
  state.started = false;
  state.points_sent = 0;
  state.transfer_state = 1.0;
}

inline void mark_passthrough_transport_failure(PassthroughTransportState state)
{
  state.failed.store(true, std::memory_order_release);
  state.abort = PASSTHROUGH_TRANSPORT_FAILURE_ABORT;
  // The passthrough controller interprets DONE + abort as an aborted action.
  state.transfer_state = 5.0;
}

enum class PassthroughStartResult
{
  STARTED,
  DISCONNECTED,
  WRITE_FAILED,
};

template <typename ConnectionProbe, typename StartWriter, typename CancelWriter>
PassthroughStartResult write_passthrough_start(PassthroughTransportState state, ConnectionProbe&& is_connected,
                                               StartWriter&& write_start, CancelWriter&& write_cancel)
{
  if (!std::forward<ConnectionProbe>(is_connected)()) {
    mark_passthrough_transport_failure(state);
    std::forward<CancelWriter>(write_cancel)();
    return PassthroughStartResult::DISCONNECTED;
  }
  if (!std::forward<StartWriter>(write_start)()) {
    mark_passthrough_transport_failure(state);
    std::forward<CancelWriter>(write_cancel)();
    return PassthroughStartResult::WRITE_FAILED;
  }
  state.started = true;
  return PassthroughStartResult::STARTED;
}

template <typename PointWriter, typename CancelWriter>
bool write_passthrough_point(PassthroughTransportState state, PointWriter&& write_point,
                             CancelWriter&& write_cancel)
{
  if (!std::forward<PointWriter>(write_point)()) {
    mark_passthrough_transport_failure(state);
    std::forward<CancelWriter>(write_cancel)();
    return false;
  }
  ++state.points_sent;
  return true;
}

}  // namespace ur_robot_driver

#endif  // UR_ROBOT_DRIVER__PASSTHROUGH_TRAJECTORY_TRANSPORT_HPP_
