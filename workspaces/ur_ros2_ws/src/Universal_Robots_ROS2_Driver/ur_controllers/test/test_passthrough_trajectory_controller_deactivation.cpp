// Copyright 2026, Universal Robots A/S
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.

#include <gmock/gmock.h>

#include <array>
#include <chrono>
#include <future>
#include <memory>
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

namespace ur_controllers_test
{
using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

struct PassthroughHardware
{
  std::array<double, 7> command_values{};
  std::array<double, 4> state_values{ 0.0, 0.0, 1.0, 0.0 };
  std::vector<hardware_interface::CommandInterface> command_storage;
  std::vector<hardware_interface::StateInterface> state_storage;

  void assign(ur_controllers::PassthroughTrajectoryController& controller, bool acceleration = false)
  {
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
    if (acceleration) {
      state_storage.emplace_back("joint1", "acceleration", &state_values[3]);
    }

    std::vector<hardware_interface::LoanedCommandInterface> commands;
    commands.reserve(command_storage.size());
    for (auto& interface : command_storage) {
      commands.emplace_back(interface);
    }
    std::vector<hardware_interface::LoanedStateInterface> states;
    states.reserve(state_storage.size());
    for (auto& interface : state_storage) {
      states.emplace_back(interface);
    }
    controller.assign_interfaces(std::move(commands), std::move(states));
  }
};

template <class FutureT>
bool spin_until_ready(rclcpp::Executor& first, rclcpp::Executor& second, FutureT& future,
                      std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (future.wait_for(0ms) != std::future_status::ready && std::chrono::steady_clock::now() < deadline) {
    first.spin_some();
    second.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  return future.wait_for(0ms) == std::future_status::ready;
}

TEST(PassthroughTrajectoryControllerDeactivation, publishes_terminal_result_without_timer_progress)
{
  PassthroughHardware hardware;
  auto controller = std::make_shared<ur_controllers::PassthroughTrajectoryController>();
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("joints", std::vector<std::string>{ "joint1" }),
    rclcpp::Parameter("state_interfaces", std::vector<std::string>{ "position", "velocity" }),
    rclcpp::Parameter("speed_scaling_interface_name", "speed_scaling/speed_scaling_factor"),
  });
  ASSERT_EQ(controller->init("passthrough_deactivation_test", "", options),
            controller_interface::return_type::OK);
  ASSERT_EQ(controller->get_node()->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  hardware.assign(*controller);
  ASSERT_EQ(controller->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto client_node = std::make_shared<rclcpp::Node>("passthrough_deactivation_client");
  auto client = rclcpp_action::create_client<FollowJointTrajectory>(
      client_node, "/passthrough_deactivation_test/follow_joint_trajectory");
  rclcpp::executors::SingleThreadedExecutor server_executor;
  rclcpp::executors::SingleThreadedExecutor client_executor;
  server_executor.add_node(controller->get_node()->get_node_base_interface());
  client_executor.add_node(client_node);
  ASSERT_TRUE(client->wait_for_action_server(1s));

  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = { "joint1" };
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = { 0.1 };
  point.time_from_start = rclcpp::Duration(1s);
  goal.trajectory.points = { point };

  auto goal_future = client->async_send_goal(goal);
  ASSERT_TRUE(spin_until_ready(server_executor, client_executor, goal_future, 1s));
  auto goal_handle = goal_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = client->async_get_result(goal_handle);
  // Let the server accept the result request and let its monitor timer place
  // the action in EXECUTING before simulating controller-manager removal.
  const auto monitor_deadline = std::chrono::steady_clock::now() + 100ms;
  while (std::chrono::steady_clock::now() < monitor_deadline) {
    server_executor.spin_some();
    client_executor.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_EQ(result_future.wait_for(0ms), std::future_status::timeout);

  server_executor.remove_node(controller->get_node()->get_node_base_interface());
  ASSERT_EQ(controller->get_node()->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  const auto result_deadline = std::chrono::steady_clock::now() + 250ms;
  while (result_future.wait_for(0ms) != std::future_status::ready &&
         std::chrono::steady_clock::now() < result_deadline) {
    client_executor.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_EQ(result_future.wait_for(0ms), std::future_status::ready)
      << "deactivation must not depend on a controller timer that no longer runs";
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::ABORTED);
}

TEST(PassthroughTrajectoryControllerDeactivation, reports_transport_failure_before_first_setpoint)
{
  PassthroughHardware hardware;
  auto controller = std::make_shared<ur_controllers::PassthroughTrajectoryController>();
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("joints", std::vector<std::string>{ "joint1" }),
    rclcpp::Parameter("state_interfaces", std::vector<std::string>{ "position", "velocity" }),
    rclcpp::Parameter("speed_scaling_interface_name", "speed_scaling/speed_scaling_factor"),
  });
  ASSERT_EQ(controller->init("passthrough_transport_failure_test", "", options),
            controller_interface::return_type::OK);
  ASSERT_EQ(controller->get_node()->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  hardware.assign(*controller);
  ASSERT_EQ(controller->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto client_node = std::make_shared<rclcpp::Node>("passthrough_transport_failure_client");
  auto client = rclcpp_action::create_client<FollowJointTrajectory>(
      client_node, "/passthrough_transport_failure_test/follow_joint_trajectory");
  rclcpp::executors::SingleThreadedExecutor server_executor;
  rclcpp::executors::SingleThreadedExecutor client_executor;
  server_executor.add_node(controller->get_node()->get_node_base_interface());
  client_executor.add_node(client_node);
  ASSERT_TRUE(client->wait_for_action_server(1s));

  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = { "joint1" };
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = { 0.1 };
  point.time_from_start = rclcpp::Duration(1s);
  goal.trajectory.points = { point };
  auto goal_future = client->async_send_goal(goal);
  ASSERT_TRUE(spin_until_ready(server_executor, client_executor, goal_future, 1s));
  auto goal_handle = goal_future.get();
  ASSERT_NE(goal_handle, nullptr);
  auto result_future = client->async_get_result(goal_handle);

  // A failed reverse-interface keepalive can occur before the controller has
  // handed over its first point. This must still be terminal, never SUCCESS.
  hardware.command_values[3] = 2.0;
  hardware.command_values[4] = ur_controllers::TRANSFER_STATE_DONE;
  ASSERT_EQ(controller->update(rclcpp::Time(0), rclcpp::Duration(8ms)), controller_interface::return_type::OK);

  ASSERT_TRUE(spin_until_ready(server_executor, client_executor, result_future, 1s));
  const auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_NE(result.result, nullptr);
  EXPECT_NE(result.result->error_string.find("UR trajectory transport failed"), std::string::npos);

  server_executor.remove_node(controller->get_node()->get_node_base_interface());
  ASSERT_EQ(controller->get_node()->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

class PassthroughGoalTolerance : public ::testing::Test
{
protected:
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  void SetUp() override
  {
    const std::string name = "passthrough_tolerance_" +
                             std::string(::testing::UnitTest::GetInstance()->current_test_info()->name());
    controller = std::make_shared<ur_controllers::PassthroughTrajectoryController>();
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("joints", std::vector<std::string>{ "joint1" }),
      rclcpp::Parameter("state_interfaces", std::vector<std::string>{ "position", "velocity", "acceleration" }),
      rclcpp::Parameter("speed_scaling_interface_name", "speed_scaling/speed_scaling_factor"),
    });
    ASSERT_EQ(controller->init(name, "", options), controller_interface::return_type::OK);
    ASSERT_EQ(controller->get_node()->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    hardware.assign(*controller, true);
    ASSERT_EQ(controller->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    client_node = std::make_shared<rclcpp::Node>(name + "_client");
    client = rclcpp_action::create_client<FollowJointTrajectory>(
        client_node, "/" + name + "/follow_joint_trajectory");
    server_executor.add_node(controller->get_node()->get_node_base_interface());
    client_executor.add_node(client_node);
    ASSERT_TRUE(client->wait_for_action_server(1s));
  }

  void TearDown() override
  {
    if (controller && controller->get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      server_executor.remove_node(controller->get_node()->get_node_base_interface());
      controller->get_node()->deactivate();
    }
  }

  void start_goal(double grace_seconds = 2.0, std::chrono::nanoseconds motion_elapsed = 1s)
  {
    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = { "joint1" };
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = { 0.1 };
    point.velocities = { 0.0 };
    point.accelerations = { 0.0 };
    point.time_from_start = rclcpp::Duration(1s);
    goal.trajectory.points = { point };
    control_msgs::msg::JointTolerance tolerance;
    tolerance.name = "joint1";
    tolerance.position = 0.005;
    tolerance.velocity = 0.003;
    tolerance.acceleration = 0.5;
    goal.goal_tolerance = { tolerance };
    goal.goal_time_tolerance = rclcpp::Duration::from_seconds(grace_seconds);
    auto accepted = client->async_send_goal(goal);
    ASSERT_TRUE(spin_until_ready(server_executor, client_executor, accepted, 1s));
    goal_handle = accepted.get();
    ASSERT_NE(goal_handle, nullptr);
    result_future = client->async_get_result(goal_handle);
    update(ur_controllers::TRANSFER_STATE_IDLE, 8ms);
    update(ur_controllers::TRANSFER_STATE_WAITING_FOR_POINT, 8ms);
    update(ur_controllers::TRANSFER_STATE_IN_MOTION, motion_elapsed);
  }

  void update(double transfer_state, std::chrono::nanoseconds period)
  {
    hardware.command_values[4] = transfer_state;
    ASSERT_EQ(controller->update(rclcpp::Time(0), rclcpp::Duration(period)),
              controller_interface::return_type::OK);
  }

  void expect_pending()
  {
    EXPECT_FALSE(spin_until_ready(server_executor, client_executor, result_future, 100ms));
    EXPECT_EQ(hardware.command_values[4], ur_controllers::TRANSFER_STATE_DONE);
  }

  void expect_result(rclcpp_action::ResultCode code, const std::string& message = "")
  {
    ASSERT_TRUE(spin_until_ready(server_executor, client_executor, result_future, 1s));
    const auto result = result_future.get();
    EXPECT_EQ(result.code, code);
    ASSERT_NE(result.result, nullptr);
    EXPECT_EQ(result.result->error_code,
              code == rclcpp_action::ResultCode::SUCCEEDED ? FollowJointTrajectory::Result::SUCCESSFUL :
                                                           FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED);
    if (!message.empty()) {
      EXPECT_NE(result.result->error_string.find(message), std::string::npos);
    }
  }

  PassthroughHardware hardware;
  std::shared_ptr<ur_controllers::PassthroughTrajectoryController> controller;
  rclcpp::Node::SharedPtr client_node;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client;
  rclcpp::executors::SingleThreadedExecutor server_executor;
  rclcpp::executors::SingleThreadedExecutor client_executor;
  GoalHandle::SharedPtr goal_handle;
  std::shared_future<GoalHandle::WrappedResult> result_future;
};

TEST_F(PassthroughGoalTolerance, waits_for_position_and_velocity_then_succeeds)
{
  start_goal();
  hardware.state_values[0] = 0.11;
  hardware.state_values[1] = 0.008;
  update(ur_controllers::TRANSFER_STATE_DONE, 8ms);
  expect_pending();
  hardware.state_values[0] = 0.1;
  hardware.state_values[1] = 0.004;  // Position is ready, velocity must also settle.
  update(ur_controllers::TRANSFER_STATE_DONE, 100ms);
  expect_pending();
  hardware.state_values[1] = 0.0;
  update(ur_controllers::TRANSFER_STATE_DONE, 8ms);
  expect_result(rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(PassthroughGoalTolerance, aborts_after_deadline_even_when_speed_scaling_is_zero)
{
  start_goal();
  hardware.state_values[0] = 0.11;
  hardware.state_values[2] = 0.0;
  update(ur_controllers::TRANSFER_STATE_DONE, 1s);
  expect_pending();
  update(ur_controllers::TRANSFER_STATE_DONE, 1s);  // Exactly at the existing inclusive deadline.
  expect_pending();
  update(ur_controllers::TRANSFER_STATE_DONE, 8ms);
  expect_result(rclcpp_action::ResultCode::ABORTED, "joint1 position");
}

TEST_F(PassthroughGoalTolerance, accepts_settled_state_exactly_at_deadline)
{
  start_goal();
  hardware.state_values[0] = 0.1;
  update(ur_controllers::TRANSFER_STATE_DONE, 2s);
  expect_result(rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(PassthroughGoalTolerance, grace_is_bounded_when_motion_timer_missed_point_transfer)
{
  // Hardware starts moving while the points are still being transferred,
  // whereas the scaled execution timer only runs in IN_MOTION.
  start_goal(2.0, 1ms);
  hardware.state_values[0] = 0.11;
  update(ur_controllers::TRANSFER_STATE_DONE, 2s);
  expect_pending();
  update(ur_controllers::TRANSFER_STATE_DONE, 8ms);
  expect_result(rclcpp_action::ResultCode::ABORTED, "joint1 position");
}

TEST_F(PassthroughGoalTolerance, does_not_accept_state_that_settled_after_deadline)
{
  start_goal();
  hardware.state_values[0] = 0.11;
  update(ur_controllers::TRANSFER_STATE_DONE, 1s);
  expect_pending();
  hardware.state_values[0] = 0.1;
  update(ur_controllers::TRANSFER_STATE_DONE, 1008ms);
  expect_result(rclcpp_action::ResultCode::ABORTED, "time tolerance");
}

TEST_F(PassthroughGoalTolerance, reports_velocity_violation_when_grace_expires)
{
  start_goal();
  hardware.state_values[0] = 0.1;
  hardware.state_values[1] = 0.004;
  update(ur_controllers::TRANSFER_STATE_DONE, 2008ms);
  expect_result(rclcpp_action::ResultCode::ABORTED, "joint1 velocity");
}

TEST_F(PassthroughGoalTolerance, zero_time_tolerance_preserves_immediate_state_failure)
{
  start_goal(0.0);
  hardware.state_values[0] = 0.11;
  update(ur_controllers::TRANSFER_STATE_DONE, 8ms);
  expect_result(rclcpp_action::ResultCode::ABORTED, "joint1 position");
}

TEST_F(PassthroughGoalTolerance, negative_time_tolerance_preserves_immediate_state_failure)
{
  start_goal(-1.0);
  hardware.state_values[0] = 0.11;
  update(ur_controllers::TRANSFER_STATE_DONE, 8ms);
  expect_result(rclcpp_action::ResultCode::ABORTED, "joint1 position");
}

TEST_F(PassthroughGoalTolerance, acceleration_must_settle_without_relaxing_tolerance)
{
  start_goal();
  hardware.state_values[0] = 0.1;
  hardware.state_values[3] = 0.6;
  update(ur_controllers::TRANSFER_STATE_DONE, 8ms);
  expect_pending();
  hardware.state_values[3] = 0.0;
  update(ur_controllers::TRANSFER_STATE_DONE, 8ms);
  expect_result(rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(PassthroughGoalTolerance, reports_acceleration_violation_when_grace_expires)
{
  start_goal();
  hardware.state_values[0] = 0.1;
  hardware.state_values[3] = 0.6;
  update(ur_controllers::TRANSFER_STATE_DONE, 2008ms);
  expect_result(rclcpp_action::ResultCode::ABORTED, "joint1 acceleration");
}

TEST_F(PassthroughGoalTolerance, deactivation_remains_terminal_during_grace)
{
  start_goal();
  hardware.state_values[0] = 0.11;
  update(ur_controllers::TRANSFER_STATE_DONE, 8ms);
  expect_pending();
  server_executor.remove_node(controller->get_node()->get_node_base_interface());
  ASSERT_EQ(controller->get_node()->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_TRUE(spin_until_ready(server_executor, client_executor, result_future, 1s));
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::ABORTED);
}

TEST_F(PassthroughGoalTolerance, each_new_goal_receives_its_own_bounded_grace)
{
  start_goal();
  hardware.state_values[0] = 0.1;
  update(ur_controllers::TRANSFER_STATE_DONE, 1992ms);
  expect_result(rclcpp_action::ResultCode::SUCCEEDED);
  start_goal();
  hardware.state_values[0] = 0.11;
  update(ur_controllers::TRANSFER_STATE_DONE, 16ms);
  expect_pending();
  hardware.state_values[0] = 0.1;
  update(ur_controllers::TRANSFER_STATE_DONE, 8ms);
  expect_result(rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(PassthroughGoalTolerance, hardware_abort_stays_terminal_during_grace)
{
  start_goal();
  hardware.state_values[0] = 0.11;
  update(ur_controllers::TRANSFER_STATE_DONE, 8ms);
  expect_pending();
  hardware.command_values[3] = 1.0;
  update(ur_controllers::TRANSFER_STATE_DONE, 8ms);
  ASSERT_TRUE(spin_until_ready(server_executor, client_executor, result_future, 1s));
  const auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_NE(result.result->error_string.find("UR hardware aborted"), std::string::npos);
}

TEST_F(PassthroughGoalTolerance, cancellation_stays_terminal_during_grace)
{
  start_goal();
  hardware.state_values[0] = 0.11;
  update(ur_controllers::TRANSFER_STATE_DONE, 8ms);
  expect_pending();
  auto cancelled = client->async_cancel_goal(goal_handle);
  ASSERT_TRUE(spin_until_ready(server_executor, client_executor, cancelled, 1s));
  update(ur_controllers::TRANSFER_STATE_DONE, 8ms);
  ASSERT_TRUE(spin_until_ready(server_executor, client_executor, result_future, 1s));
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::CANCELED);
}

}  // namespace ur_controllers_test

int main(int argc, char* argv[])
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
