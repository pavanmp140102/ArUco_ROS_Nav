//
// Copyright 2026 Pavan Patil
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// @Author: Pavan Patil
// @Date: 2026-10-23
// @Description: Node tests: goal checks, parameter validation, IDLE state, sim-time ticking.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include "dockbot_docking/docking_controller.hpp"
#include "dockbot_interfaces/msg/aruco_marker.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "kinematic_sim.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

using dockbot_docking::DockingController;
using Dock = dockbot_interfaces::action::Dock;
using std::chrono::milliseconds;

namespace
{

/// Node, executor, action client and state recorder, run in one thread like the real node.
struct Fixture
{
  explicit Fixture(bool sim_time = false)
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("use_sim_time", sim_time);
    node = std::make_shared<DockingController>(options);
    helper = std::make_shared<rclcpp::Node>("docking_test_helper");
    client = rclcpp_action::create_client<Dock>(helper, "dock");
    state_sub = helper->create_subscription<dockbot_interfaces::msg::DockingState>(
      "docking/state", 50,
      [this](dockbot_interfaces::msg::DockingState::ConstSharedPtr m) {states.push_back(*m);});
    executor.add_node(node);
    executor.add_node(helper);
    EXPECT_TRUE(client->wait_for_action_server(std::chrono::seconds(0)) || spin_until([this] {
        return client->action_server_is_ready();
      }));
  }

  /// Spin until the condition holds or two seconds of wall time pass.
  template<typename F>
  bool spin_until(F condition)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!condition() && std::chrono::steady_clock::now() < end) {
      executor.spin_some(milliseconds(10));
    }
    return condition();
  }

  /// Send a goal and return its handle, or null when the server rejected it.
  rclcpp_action::ClientGoalHandle<Dock>::SharedPtr send(const Dock::Goal & goal)
  {
    auto future = client->async_send_goal(goal);
    if (executor.spin_until_future_complete(future, std::chrono::seconds(2)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return nullptr;
    }
    return future.get();
  }

  std::shared_ptr<DockingController> node;
  std::shared_ptr<rclcpp::Node> helper;
  rclcpp_action::Client<Dock>::SharedPtr client;
  rclcpp::Subscription<dockbot_interfaces::msg::DockingState>::SharedPtr state_sub;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::vector<dockbot_interfaces::msg::DockingState> states;
};

/// Build a /dock goal for a marker ID, with optional overrides of the stop distance and timeout.
Dock::Goal make_goal(int id, float dock_distance = 0.0f, float timeout = 0.0f)
{
  Dock::Goal g;
  g.marker_id = id;
  g.dock_distance = dock_distance;
  g.timeout = timeout;
  return g;
}

}  // namespace

class ControllerNode : public ::testing::Test
{
protected:
  /// Start rclcpp once for the suite, so each test can create the controller node it needs.
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  /// Shut rclcpp down after the suite so no node outlives the tests.
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

// Goal fields outside the parameter ranges are rejected; zero means "use the parameter".
TEST_F(ControllerNode, RejectsGoalValuesOutsideParameterRanges)
{
  Fixture f;
  EXPECT_EQ(f.send(make_goal(2, 5.0f)), nullptr);
  EXPECT_EQ(f.send(make_goal(2, 0.0001f)), nullptr);
  EXPECT_EQ(f.send(make_goal(2, -0.1f)), nullptr);
  EXPECT_EQ(f.send(make_goal(2, 0.0f, 0.001f)), nullptr);
  EXPECT_EQ(f.send(make_goal(-1)), nullptr);
  auto handle = f.send(make_goal(2, 0.15f, 30.0f));
  ASSERT_NE(handle, nullptr);
  f.client->async_cancel_goal(handle);
  f.spin_until([&f] {return !f.states.empty() && f.states.back().state == 0;});
}

// A rejected parameter batch changes nothing, including control_rate.
TEST_F(ControllerNode, RejectedParameterBatchChangesNothing)
{
  Fixture f;
  const auto bad = f.node->set_parameters_atomically({
    rclcpp::Parameter("control_rate", 50.0), rclcpp::Parameter("lost_hold_time", 5.0)});
  EXPECT_FALSE(bad.successful);
  EXPECT_DOUBLE_EQ(f.node->get_parameter("control_rate").as_double(), 20.0);
  EXPECT_FALSE(f.node->set_parameter(rclcpp::Parameter("recover_range", 0.4)).successful);
  EXPECT_TRUE(f.node->set_parameter(rclcpp::Parameter("aim_gain", 0.6)).successful);
}

// Inconsistent values in the startup parameters stop the node instead of being accepted.
TEST_F(ControllerNode, InconsistentStartupParametersAreRejected)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", false);
  options.append_parameter_override("lost_hold_time", 5.0);
  EXPECT_THROW(DockingController node(options), std::invalid_argument);
}

// IDLE is published once after a goal ends, and unknown errors are NaN, not zero.
TEST_F(ControllerNode, PublishesIdleAfterGoalAndNanBeforeFirstPose)
{
  Fixture f;
  auto handle = f.send(make_goal(2));
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(f.spin_until([&f] {return !f.states.empty();}));
  EXPECT_EQ(f.states.back().state, 1);  // SEARCH
  EXPECT_TRUE(std::isnan(f.states.back().distance_to_dock));
  EXPECT_TRUE(std::isnan(f.states.back().lateral_error));
  f.client->async_cancel_goal(handle);
  ASSERT_TRUE(f.spin_until([&f] {return f.states.back().state == 0;}));
  EXPECT_EQ(f.states.back().target_id, -1);
}

// With simulation time the control loop only ticks when /clock advances.
TEST_F(ControllerNode, ControlLoopFollowsSimulationTime)
{
  Fixture f(true);
  auto clock_pub = f.helper->create_publisher<rosgraph_msgs::msg::Clock>("clock", 10);
  auto publish_clock = [&](double t) {
      rosgraph_msgs::msg::Clock c;
      c.clock = rclcpp::Time(static_cast<int64_t>(t * 1e9));
      clock_pub->publish(c);
    };
  publish_clock(1.0);
  f.spin_until([] {return false;});
  auto handle = f.send(make_goal(2));
  ASSERT_NE(handle, nullptr);
  f.states.clear();
  f.spin_until([] {return false;});  // two wall seconds with a frozen clock
  EXPECT_LE(f.states.size(), 1u);
  for (int i = 1; i <= 10; ++i) {
    // Give each clock step up to 200 ms of wall time to produce a state, so a loaded machine
    // that delivers /clock late does not look like a loop that ignores simulation time.
    const auto before = f.states.size();
    publish_clock(1.0 + 0.05 * i);
    const auto end = std::chrono::steady_clock::now() + milliseconds(200);
    while (f.states.size() == before && std::chrono::steady_clock::now() < end) {
      f.executor.spin_some(milliseconds(10));
    }
  }
  EXPECT_GE(f.states.size(), 5u);
  f.client->async_cancel_goal(handle);
}

// Goals sent back to back, some cancelled mid-way, each answered with a result, and none left
// hanging. The node runs on a simulated clock driven faster than real time, with a kinematic robot
// and camera closing the loop, so a dozen goals take seconds instead of the minutes of Gazebo.
TEST_F(ControllerNode, BackToBackGoalsAllReturnResults)
{
  using dockbot_docking::testing::CameraModel;
  using dockbot_docking::testing::KinematicSim;
  using dockbot_docking::testing::Pose2D;
  Fixture f(true);
  auto clock_pub = f.helper->create_publisher<rosgraph_msgs::msg::Clock>("clock", 10);
  auto marker_pub = f.helper->create_publisher<dockbot_interfaces::msg::ArucoMarkerArray>(
    "aruco/markers", 10);
  dockbot_docking::VelocityCommand cmd;
  auto cmd_sub = f.helper->create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel", 10, [&cmd](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
      cmd.v = m->twist.linear.x;
      cmd.w = m->twist.angular.z;
    });

  const double dt = 0.05;
  double now = 100.0;
  // Advances the simulated clock one step, drives the robot with the last command and publishes
  // the marker the camera would see, so the controller under test closes the docking loop.
  auto tick = [&](KinematicSim & sim) {
      now += dt;
      rosgraph_msgs::msg::Clock c;
      c.clock = rclcpp::Time(static_cast<int64_t>(now * 1e9));
      clock_pub->publish(c);
      f.executor.spin_some(milliseconds(3));
      sim.step(cmd, dt);
      dockbot_interfaces::msg::ArucoMarkerArray array;
      array.header.stamp = c.clock;
      array.header.frame_id = "base_link";
      if (const auto pose = sim.observe()) {
        dockbot_interfaces::msg::ArucoMarker m;
        m.id = 0;
        m.pose.position.x = pose->translation().x();
        m.pose.position.y = pose->translation().y();
        m.pose.position.z = pose->translation().z();
        const Eigen::Quaterniond q(pose->linear());
        m.pose.orientation.w = q.w();
        m.pose.orientation.x = q.x();
        m.pose.orientation.y = q.y();
        m.pose.orientation.z = q.z();
        array.markers.push_back(m);
      }
      marker_pub->publish(array);
      f.executor.spin_some(milliseconds(3));
    };

  KinematicSim idle_sim(Pose2D{}, Eigen::Vector2d(0, 0), 0.0, CameraModel{}, 1);
  tick(idle_sim);
  const int goals = 12;
  int docked = 0;
  int canceled = 0;
  for (int i = 0; i < goals; ++i) {
    const double y = (i % 2 == 0 ? 0.6 : -0.6);
    KinematicSim sim(
      Pose2D{2.0, y, std::atan2(-y, -2.0)}, Eigen::Vector2d(0, 0), 0.0, CameraModel{}, 10 + i);
    cmd = dockbot_docking::VelocityCommand();
    auto handle = f.send(make_goal(0));
    ASSERT_NE(handle, nullptr) << "goal " << i << " was rejected";
    auto result_future = f.client->async_get_result(handle);
    const bool cancel_this = i % 3 == 2;
    double t_sent = now;
    bool cancel_sent = false;
    while (result_future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      ASSERT_LT(now - t_sent, 200.0) << "goal " << i << " never returned a result";
      tick(sim);
      if (cancel_this && !cancel_sent && now - t_sent > 4.0) {
        f.client->async_cancel_goal(handle);
        cancel_sent = true;
      }
    }
    const auto wrapped = result_future.get();
    if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
      EXPECT_EQ(wrapped.result->outcome, Dock::Result::OUTCOME_DOCKED) << "goal " << i;
      ++docked;
    } else if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
      EXPECT_TRUE(cancel_this) << "goal " << i << " cancelled without a request";
      ++canceled;
    } else {
      ADD_FAILURE() << "goal " << i << " ended with code " << static_cast<int>(wrapped.code);
    }
  }
  EXPECT_EQ(docked + canceled, goals);
  EXPECT_EQ(canceled, goals / 3);
}
