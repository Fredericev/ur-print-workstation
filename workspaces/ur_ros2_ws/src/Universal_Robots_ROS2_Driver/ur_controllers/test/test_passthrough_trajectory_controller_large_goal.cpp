// Copyright 2026, Universal Robots A/S

#include <gmock/gmock.h>

#include <array>
#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>
#include <new>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "ur_controllers/passthrough_trajectory_controller.hpp"

namespace
{
thread_local bool count_update_allocations = false;
thread_local size_t update_allocations = 0;

// Keep the allocator probe out of the compiler's new/delete inlining checks.
__attribute__((noinline)) void release_allocation(void* pointer) noexcept
{
  std::free(pointer);
}
}

void* operator new(std::size_t size)
{
  if (count_update_allocations) {
    ++update_allocations;
  }
  if (void* pointer = std::malloc(size)) {
    return pointer;
  }
  throw std::bad_alloc();
}

void operator delete(void* pointer) noexcept
{
  release_allocation(pointer);
}

void operator delete(void* pointer, std::size_t) noexcept
{
  release_allocation(pointer);
}

TEST(PassthroughTrajectoryControllerLargeGoal, update_does_not_copy_the_whole_goal)
{
  using namespace std::chrono_literals;
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  constexpr size_t point_count = 26279;
  std::array<double, 7> command_values{};
  std::array<double, 3> state_values{ 0.0, 0.0, 1.0 };
  std::vector<hardware_interface::CommandInterface> command_storage;
  std::vector<hardware_interface::StateInterface> state_storage;
  auto controller = std::make_shared<ur_controllers::PassthroughTrajectoryController>();
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("joints", std::vector<std::string>{ "joint1" }),
    rclcpp::Parameter("state_interfaces", std::vector<std::string>{ "position", "velocity" }),
    rclcpp::Parameter("speed_scaling_interface_name", "speed_scaling/speed_scaling_factor"),
  });
  ASSERT_EQ(controller->init("passthrough_large_goal_test", "", options),
            controller_interface::return_type::OK);
  ASSERT_EQ(controller->get_node()->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  command_storage.reserve(command_values.size());
  command_storage.emplace_back("trajectory_passthrough", "setpoint_positions_0", &command_values[0]);
  command_storage.emplace_back("trajectory_passthrough", "setpoint_velocities_0", &command_values[1]);
  command_storage.emplace_back("trajectory_passthrough", "setpoint_accelerations_0", &command_values[2]);
  command_storage.emplace_back("trajectory_passthrough", "abort", &command_values[3]);
  command_storage.emplace_back("trajectory_passthrough", "transfer_state", &command_values[4]);
  command_storage.emplace_back("trajectory_passthrough", "time_from_start", &command_values[5]);
  command_storage.emplace_back("trajectory_passthrough", "trajectory_size", &command_values[6]);
  state_storage.reserve(state_values.size());
  state_storage.emplace_back("joint1", "position", &state_values[0]);
  state_storage.emplace_back("joint1", "velocity", &state_values[1]);
  state_storage.emplace_back("speed_scaling", "speed_scaling_factor", &state_values[2]);
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  for (auto& item : command_storage) {
    commands.emplace_back(item);
  }
  std::vector<hardware_interface::LoanedStateInterface> states;
  for (auto& item : state_storage) {
    states.emplace_back(item);
  }
  controller->assign_interfaces(std::move(commands), std::move(states));
  ASSERT_EQ(controller->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto client_node = std::make_shared<rclcpp::Node>("passthrough_large_goal_client");
  auto client = rclcpp_action::create_client<FollowJointTrajectory>(
      client_node, "/passthrough_large_goal_test/follow_joint_trajectory");
  rclcpp::executors::SingleThreadedExecutor server_executor;
  rclcpp::executors::SingleThreadedExecutor client_executor;
  server_executor.add_node(controller->get_node()->get_node_base_interface());
  client_executor.add_node(client_node);
  ASSERT_TRUE(client->wait_for_action_server(1s));

  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = { "joint1" };
  goal.trajectory.points.resize(point_count);
  for (size_t index = 0; index < point_count; ++index) {
    auto& point = goal.trajectory.points[index];
    point.positions = { static_cast<double>(index) / point_count };
    point.velocities = { 0.0 };
    point.accelerations = { 0.0 };
    point.time_from_start = rclcpp::Duration::from_seconds(3.98 + static_cast<double>(index) * 0.05);
  }
  auto goal_future = client->async_send_goal(goal);
  const auto deadline = std::chrono::steady_clock::now() + 10s;
  while (goal_future.wait_for(0ms) != std::future_status::ready && std::chrono::steady_clock::now() < deadline) {
    server_executor.spin_some();
    client_executor.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_EQ(goal_future.wait_for(0ms), std::future_status::ready);
  ASSERT_NE(goal_future.get(), nullptr);

  update_allocations = 0;
  count_update_allocations = true;
  const auto result = controller->update(rclcpp::Time(0), rclcpp::Duration(8ms));
  count_update_allocations = false;
  EXPECT_EQ(result, controller_interface::return_type::OK);
  EXPECT_EQ(command_values[4], ur_controllers::TRANSFER_STATE_NEW_TRAJECTORY);
  EXPECT_EQ(command_values[6], static_cast<double>(point_count));
  EXPECT_LT(update_allocations, 100u) << "The control cycle copied the large action goal";

  // Emulate the hardware's WAITING_FOR_POINT handshake. A large accepted goal
  // must deliver the initial synchronized hold and then its first moving point.
  command_values[4] = ur_controllers::TRANSFER_STATE_WAITING_FOR_POINT;
  EXPECT_EQ(controller->update(rclcpp::Time(0), rclcpp::Duration(8ms)), controller_interface::return_type::OK);
  EXPECT_EQ(command_values[4], ur_controllers::TRANSFER_STATE_TRANSFERRING);
  EXPECT_DOUBLE_EQ(command_values[5], 3.98);
  EXPECT_DOUBLE_EQ(command_values[0], 0.0);
  command_values[4] = ur_controllers::TRANSFER_STATE_WAITING_FOR_POINT;
  EXPECT_EQ(controller->update(rclcpp::Time(0), rclcpp::Duration(8ms)), controller_interface::return_type::OK);
  EXPECT_EQ(command_values[4], ur_controllers::TRANSFER_STATE_TRANSFERRING);
  EXPECT_NEAR(command_values[5], 4.03, 1e-6);
  EXPECT_GT(command_values[0], 0.0);

  server_executor.remove_node(controller->get_node()->get_node_base_interface());
  ASSERT_EQ(controller->get_node()->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
