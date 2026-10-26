#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "eli_cs_controllers/scaled_joint_trajectory_controller.hpp"

class TestController : public ELITE_CS_CONTROLLER::ScaledJointTrajectoryController {
public:
  void target(double position) {
    auto message = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    message->joint_names = {"joint1"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {position};
    point.time_from_start.sec = 2;
    message->points.push_back(point);
    topic_callback(message);
  }
};

class ScaledTrajectoryJazzy : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("joints", std::vector<std::string>{"joint1"}),
      rclcpp::Parameter("command_interfaces", std::vector<std::string>{"position"}),
      rclcpp::Parameter("state_interfaces", std::vector<std::string>{"position", "velocity"}),
    });
    const std::string description = R"(
      <robot name="test"><link name="base"/><link name="tool"/>
      <joint name="joint1" type="revolute"><parent link="base"/><child link="tool"/>
      <limit lower="-3" upper="3" effort="10" velocity="1"/></joint></robot>)";
    controller = std::make_unique<TestController>();
    ASSERT_EQ(controller->init("scaled_trajectory_test", description, 100, "", options),
              controller_interface::return_type::OK);
    ASSERT_EQ(controller->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    commands.emplace_back(command_handle);
    std::vector<hardware_interface::LoanedStateInterface> states;
    states.emplace_back(position_handle);
    states.emplace_back(velocity_handle);
    states.emplace_back(scaling_handle);
    controller->assign_interfaces(std::move(commands), std::move(states));
    ASSERT_EQ(controller->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    time = controller->get_node()->now();
  }

  void TearDown() override {
    if (controller) {
      if (controller->get_node()->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        controller->get_node()->deactivate();
      }
      controller->release_interfaces();
      controller.reset();
    }
  }

  void ticks(int count) {
    auto period = rclcpp::Duration::from_seconds(0.01);
    for (int i = 0; i < count; ++i) {
      time = time + period;
      ASSERT_EQ(controller->update(time, period), controller_interface::return_type::OK);
    }
  }

  double command{0.0}, position{0.0}, velocity{0.0}, scaling{1.0};
  hardware_interface::CommandInterface command_handle{"joint1", "position", &command};
  hardware_interface::StateInterface position_handle{"joint1", "position", &position};
  hardware_interface::StateInterface velocity_handle{"joint1", "velocity", &velocity};
  hardware_interface::StateInterface scaling_handle{"speed_scaling", "speed_scaling_factor", &scaling};
  std::unique_ptr<TestController> controller;
  rclcpp::Time time;
};

TEST_F(ScaledTrajectoryJazzy, PausesAndResumesFromHardwareScaling) {
  controller->target(1.0);
  ticks(30);
  ASSERT_GT(command, 0.05);
  scaling = 0.0;
  ticks(1);
  const double paused = command;
  ticks(50);
  EXPECT_NEAR(command, paused, 1e-12);
  scaling = 0.5;
  ticks(20);
  EXPECT_GT(command, paused);
  EXPECT_LT(command - paused, 0.1);
}

TEST_F(ScaledTrajectoryJazzy, ReceivesReplacementTrajectoryThroughNewBuffer) {
  controller->target(1.0);
  ticks(20);
  ASSERT_GT(command, 0.0);
  controller->target(-1.0);
  ticks(20);
  EXPECT_LT(command, 0.0);
}
