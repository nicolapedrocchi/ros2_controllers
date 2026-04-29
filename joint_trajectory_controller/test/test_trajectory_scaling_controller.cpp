// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#include <gtest/gtest.h>
#endif

#include <cmath>
#include <limits>
#include <string>
#include <vector>


#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/parameter.hpp"

#include "joint_trajectory_controller/trajectory_utils.hpp"

#include "test_assets.hpp"
#include "test_trajectory_controller_utils.hpp"

using lifecycle_msgs::msg::State;
using test_trajectory_controllers::TrajectoryControllerTest;
using test_trajectory_controllers::TrajectoryControllerTestParameterized;

// Floating-point value comparison threshold
const double EPS = 1e-6;

/* *********************************************************************
 * TEST OF THE MATH
 * ********************************************************************* */
TEST(TestTrajectoryUtils, apply_scaling_factor)
{
  auto full_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  full_msg->header.stamp = rclcpp::Time(0);

  std::vector<double> knot_times{1.0, 2.0, 3.0};
  for(const auto & t : knot_times)
  {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions.push_back(t);
    p.time_from_start = rclcpp::Duration::from_seconds(t);
    full_msg->points.push_back(p);
  }

  for(const auto& point : full_msg->points)
  {
    auto pt = point;
    joint_trajectory_controller::trajectory_utils::apply_scaling_factor(0.5, 0.5, pt);
    for(std::size_t i=0; i<point.positions.size(); ++i)
    {
      EXPECT_NEAR(point.positions[i], pt.positions[i], 1e-6);
    }
  }

  full_msg->points.clear();
  for(const auto & t : knot_times)
  {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {1.0 * t, 2.0 * t, 3.0*t};
    p.velocities = {1 * t, 1 * t, 1*t};
    p.accelerations = {1 * t, 1 * t, 1*t};
    p.time_from_start = rclcpp::Duration::from_seconds(t);
    full_msg->points.push_back(p);
  }
  for(const auto& point : full_msg->points)
  {
    auto pt = point;
    auto & p = pt.positions;
    auto & v = pt.velocities;
    auto & a = pt.accelerations;
    double dtau = 0.5;
    double ddtau = 0.5;
    joint_trajectory_controller::trajectory_utils::apply_scaling_factor(dtau, ddtau, pt);
    for(std::size_t i=0; i<point.positions.size(); ++i)
    {
      EXPECT_NEAR(point.positions[i], p[i], 1e-6);
      EXPECT_NEAR(dtau * point.velocities[i], v[i], 1e-6);
      EXPECT_NEAR(ddtau * point.velocities[i] + dtau*dtau * point.accelerations[i], a[i] , 1e-6);
    }
  }  
}

TEST(TestTrajectoryUtils, find_segment)
{
  auto full_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  full_msg->header.stamp = rclcpp::Time(0);

  std::vector<double> knot_times{1.0, 2.0, 3.0};
  for(const auto & t : knot_times)
  {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions.push_back(t);
    p.time_from_start = rclcpp::Duration::from_seconds(t);
    full_msg->points.push_back(p);
  }

  // set current state before trajectory msg was sent
  joint_trajectory_controller::TrajectoryPointConstIter start, end;

  std::vector<std::pair<double, std::pair<joint_trajectory_controller::TrajectoryPointConstIter,joint_trajectory_controller::TrajectoryPointConstIter>>> samples{ 
    {0.000, {full_msg->points.begin()+0,full_msg->points.begin()+0}}, 
    {0.500, {full_msg->points.begin()+0,full_msg->points.begin()+0}},
    {1.000, {full_msg->points.begin()+0,full_msg->points.begin()+1}},
    {1.500, {full_msg->points.begin()+0,full_msg->points.begin()+1}},
    {2.500, {full_msg->points.begin()+1,full_msg->points.begin()+2}},
    {3.000, {full_msg->points.end()  -1,full_msg->points.end()    }},
    {3.125, {full_msg->points.end()  -1,full_msg->points.end()    }},
    {30.000,{full_msg->points.end()  -1,full_msg->points.end()    }}
  };
  // sample at trajectory starting time
  for(const auto& sample : samples)
  {
    std::tie(start, end) = joint_trajectory_controller::trajectory_utils::find_segment(full_msg, rclcpp::Duration::from_seconds(sample.first));
    ASSERT_EQ(sample.second.first, start);
    ASSERT_EQ(sample.second.second, end);
  }
}

TEST(TestTrajectoryUtils, leqt)
{
  std::vector<std::string> joint_names{
  {"joint_1"},
  {"joint_2"},
  {"joint_3"}
 };

 std::map<std::string, double> limits{
  {"joint_1", 1.0},
  {"joint_2", 2.0},
  {"joint_3", 3.0}
 };

 for(const auto& joint_name : joint_names)
 {
    // within limits
    EXPECT_EQ(joint_trajectory_controller::trajectory_utils::leqt(limits, joint_name, limits.at(joint_name), true), 1);
    EXPECT_EQ(joint_trajectory_controller::trajectory_utils::leqt(limits, joint_name, limits.at(joint_name)-0.1, true), 1);
    EXPECT_EQ(joint_trajectory_controller::trajectory_utils::leqt(limits, joint_name, 0.0, true), 1);
    EXPECT_EQ(joint_trajectory_controller::trajectory_utils::leqt(limits, joint_name, -limits.at(joint_name)+0.1, true), 1);
    EXPECT_EQ(joint_trajectory_controller::trajectory_utils::leqt(limits, joint_name, -limits.at(joint_name), true), 1);

    // outside limits
    EXPECT_EQ(joint_trajectory_controller::trajectory_utils::leqt(limits, joint_name, limits.at(joint_name)+0.1, true), 0);
    EXPECT_EQ(joint_trajectory_controller::trajectory_utils::leqt(limits, joint_name, -limits.at(joint_name)-0.1, true), 0);
    EXPECT_EQ(joint_trajectory_controller::trajectory_utils::leqt(limits, joint_name, -limits.at(joint_name)-0.1, false), 1);
 }

 EXPECT_EQ(joint_trajectory_controller::trajectory_utils::leqt(limits, "joint_4", 0.0, true), -1);

 EXPECT_EQ(joint_trajectory_controller::trajectory_utils::leqt(limits, {"joint_1","joint_2"}, {0.5,0.5}, true), 1);
 EXPECT_EQ(joint_trajectory_controller::trajectory_utils::leqt(limits, {"joint_1","joint_2"}, {0.5,1.5}, true), 1);
 EXPECT_EQ(joint_trajectory_controller::trajectory_utils::leqt(limits, {"joint_1","joint_2"}, {1.5,1.5}, true), 0);
 EXPECT_EQ(joint_trajectory_controller::trajectory_utils::leqt(limits, {"joint_1","joint_2"}, {1.5,2.5}, true), 0);
 EXPECT_EQ(joint_trajectory_controller::trajectory_utils::leqt(limits, {"joint_1","joint_4"}, {1.5,2.5}, true), 0);
 EXPECT_EQ(joint_trajectory_controller::trajectory_utils::leqt(limits, {"joint_1","joint_4"}, {1.0,2.5}, true), -1);
}

TEST(TestTrajectoryUtils, multiply)
{
  std::vector<double> a{0.0, 1.0, 2.0, 3.0};
  double factor = 2.0;
  std::vector<double> b{0.0, 2.0, 4.0, 6.0};
  auto c = joint_trajectory_controller::trajectory_utils::multiply(factor, a);
  for(std::size_t i=0; i<a.size(); ++i)
  {
    EXPECT_EQ(c[i], b[i]);
  }
}


TEST(TestTrajectoryUtils, compute_interval_and_scaling_no_input_vel_acc)
{
  std::vector<std::string> joint_names{"joint_1","joint_2","joint_3"};

  std::map<std::string, double> max_velocities{
  {"joint_1", 1.0},
  {"joint_2", 2.0},
  {"joint_3", 3.0}
  };
  std::map<std::string, double> max_accelerations{
  {"joint_1", 1.0},
  {"joint_2", 2.0},
  {"joint_3", 3.0}
  };

  auto full_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  full_msg->header.stamp = rclcpp::Time(0);

  double scaling_factor = 1.0;
  double prev_scaling_factor = 1.0;
  
  std::vector<double> knot_times{1.0, 2.0, 3.0};
  for(const auto & t : knot_times)
  {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions.push_back(t);
    p.time_from_start = rclcpp::Duration::from_seconds(t);
    full_msg->points.push_back(p);
  }

  rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.01);

  double dtau = scaling_factor;
  double ddtau = 0.0;
  
  std::vector<
    std::pair<rclcpp::Duration, 
      std::tuple<double, double, joint_trajectory_controller::TrajectoryPointConstIter,joint_trajectory_controller::TrajectoryPointConstIter>>> samples{ 
    {rclcpp::Duration(0,000e6)   , {dtau,ddtau,full_msg->points.begin()+0,full_msg->points.begin()+0}}, 
    {rclcpp::Duration(0,500e6)   , {dtau,ddtau,full_msg->points.begin()+0,full_msg->points.begin()+0}},
    {rclcpp::Duration(1,000e6)-dt, {dtau,ddtau,full_msg->points.begin()+0,full_msg->points.begin()+1}},
    {rclcpp::Duration(1,000e6)   , {dtau,ddtau,full_msg->points.begin()+0,full_msg->points.begin()+1}},
    {rclcpp::Duration(1,500e6)-dt, {dtau,ddtau,full_msg->points.begin()+0,full_msg->points.begin()+1}},
    {rclcpp::Duration(1,500e6)   , {dtau,ddtau,full_msg->points.begin()+0,full_msg->points.begin()+1}},
    {rclcpp::Duration(2,500e6)-dt, {dtau,ddtau,full_msg->points.begin()+1,full_msg->points.begin()+2}},
    {rclcpp::Duration(3,000e6)-dt, {dtau,ddtau,full_msg->points.end()  -1,full_msg->points.end()    }},
    {rclcpp::Duration(3,125e6)-dt, {dtau,ddtau,full_msg->points.end()  -1,full_msg->points.end()    }},
    {rclcpp::Duration(3,000e6)-dt, {dtau,ddtau,full_msg->points.end()  -1,full_msg->points.end()    }}
  };

  // check if the interval is properly computed with scaling factor 1.0
  for(const auto& sample : samples)
  {
    auto [traj_time, feasible_scaling, feasible_scaling_derivative, start, end] = 
            joint_trajectory_controller::trajectory_utils::compute_interval_and_scaling(
              full_msg, sample.first, dt, scaling_factor, prev_scaling_factor, max_velocities, max_accelerations);

    ASSERT_EQ(std::get<0>(sample.second), dtau);
    ASSERT_EQ(std::get<1>(sample.second), ddtau);
    ASSERT_EQ(std::get<2>(sample.second), start);
    ASSERT_EQ(std::get<3>(sample.second), end);
  }
}


TEST(TestTrajectoryUtils, compute_interval_and_scaling)
{
  std::vector<std::string> joint_names{"joint_1","joint_2","joint_3"};

  std::map<std::string, double> max_velocities{
  {"joint_1", 1.0},
  {"joint_2", 2.0},
  {"joint_3", 3.0}
  };
  std::map<std::string, double> max_accelerations{
  {"joint_1", 1.0},
  {"joint_2", 2.0},
  {"joint_3", 3.0}
  };

  auto full_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  full_msg->header.stamp = rclcpp::Time(0);

  double scaling_factor = 1.0;
  double prev_scaling_factor = 1.0;
  
  std::vector<double> knot_times{1.0, 2.0, 3.0};
  for(const auto & t : knot_times)
  {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions.push_back(t);
    p.time_from_start = rclcpp::Duration::from_seconds(t);
    full_msg->points.push_back(p);
  }

  rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.01);

  double dtau = scaling_factor;
  double ddtau = 0.0;
  
  std::vector<
    std::pair<rclcpp::Duration, 
      std::tuple<double, double, joint_trajectory_controller::TrajectoryPointConstIter,joint_trajectory_controller::TrajectoryPointConstIter>>> samples{ 
    {rclcpp::Duration(0,000e6)   , {dtau,ddtau,full_msg->points.begin()+0,full_msg->points.begin()+0}}, 
    {rclcpp::Duration(0,500e6)   , {dtau,ddtau,full_msg->points.begin()+0,full_msg->points.begin()+0}},
    {rclcpp::Duration(1,000e6)-dt, {dtau,ddtau,full_msg->points.begin()+0,full_msg->points.begin()+1}},
    {rclcpp::Duration(1,000e6)   , {dtau,ddtau,full_msg->points.begin()+0,full_msg->points.begin()+1}},
    {rclcpp::Duration(1,500e6)-dt, {dtau,ddtau,full_msg->points.begin()+0,full_msg->points.begin()+1}},
    {rclcpp::Duration(1,500e6)   , {dtau,ddtau,full_msg->points.begin()+0,full_msg->points.begin()+1}},
    {rclcpp::Duration(2,500e6)-dt, {dtau,ddtau,full_msg->points.begin()+1,full_msg->points.begin()+2}},
    {rclcpp::Duration(3,000e6)-dt, {dtau,ddtau,full_msg->points.end()  -1,full_msg->points.end()    }},
    {rclcpp::Duration(3,125e6)-dt, {dtau,ddtau,full_msg->points.end()  -1,full_msg->points.end()    }},
    {rclcpp::Duration(3,000e6)-dt, {dtau,ddtau,full_msg->points.end()  -1,full_msg->points.end()    }}
  };

  // check if the interval is properly computed with scaling factor 1.0
  for(const auto& sample : samples)
  {
    auto [traj_time, feasible_scaling, feasible_scaling_derivative, start, end] = 
            joint_trajectory_controller::trajectory_utils::compute_interval_and_scaling(
              full_msg, sample.first, dt, scaling_factor, prev_scaling_factor, max_velocities, max_accelerations);

    ASSERT_EQ(std::get<0>(sample.second), dtau);
    ASSERT_EQ(std::get<1>(sample.second), ddtau);
    ASSERT_EQ(std::get<2>(sample.second), start);
    ASSERT_EQ(std::get<3>(sample.second), end);
  }
}

/* *********************************************************************
 * END TEST OF THE MATH
 * ********************************************************************* */





/* *********************************************************************
 * SETTING SCALING FACTOR
 * ********************************************************************* */
TEST_F(TrajectoryControllerTest, setting_scaling_factor_works_correctly)
{
  control_msgs::msg::SpeedScalingFactor msg;
  msg.factor = 0.765;

  rclcpp::executors::MultiThreadedExecutor executor;
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("trajectory_scaling.initial_scaling_factor", msg.factor),
    rclcpp::Parameter("trajectory_scaling.filter_coefficient", 1.0),
    rclcpp::Parameter("trajectory_scaling.subscribed_topics",std::vector<std::string>{
          "~/trajectory_scaling_input" }),
    rclcpp::Parameter("trajectory_scaling.limits.override_urdf", true),
    rclcpp::Parameter("trajectory_scaling.limits.joint1.max_velocity", 3.0),
    rclcpp::Parameter("trajectory_scaling.limits.joint2.max_velocity", 3.0),
    rclcpp::Parameter("trajectory_scaling.limits.joint3.max_velocity", 3.0),
  };
    
  SetUpAndActivateTrajectoryController(executor, params);
  // Create a QoS profile that matches the controller's speed_scaling subscriber
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  auto trajectory_scaling_pub = node_->create_publisher<control_msgs::msg::SpeedScalingFactor>(
    controller_name_ + "/trajectory_scaling_input", qos);
  subscribeToState(executor);

 
  trajectory_scaling_pub->publish(msg);
  traj_controller_->wait_for_trajectory(executor);

  updateController();

  // Spin to receive latest state
  executor.spin_some();
  auto state = getState();
  EXPECT_EQ(state->speed_scaling_factor, msg.factor);

  // 0.0 should work as an edge case
  msg.factor = 0.0;
  trajectory_scaling_pub->publish(msg);
  traj_controller_->wait_for_trajectory(executor);
  updateController();
  executor.spin_some();
  state = getState();
  EXPECT_EQ(state->speed_scaling_factor, 0.0);

  // Sending a negative value will be ignored
  msg.factor = 0.45;
  trajectory_scaling_pub->publish(msg);
  traj_controller_->wait_for_trajectory(executor);
  msg.factor = -0.12;
  trajectory_scaling_pub->publish(msg);
  traj_controller_->wait_for_trajectory(executor);
  updateController();
  executor.spin_some();
  state = getState();
  EXPECT_EQ(state->speed_scaling_factor, 0.45);
}

TEST_F(TrajectoryControllerTest, setting_scaling_factor_with_activate_filter)
{
  control_msgs::msg::SpeedScalingFactor msg;
  msg.factor = 0.765;

  rclcpp::executors::MultiThreadedExecutor executor;
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("trajectory_scaling.initial_scaling_factor", msg.factor),
    rclcpp::Parameter("trajectory_scaling.filter_coefficient", 0.5),
    rclcpp::Parameter("trajectory_scaling.subscribed_topics",std::vector<std::string>{
          "~/trajectory_scaling_input" }),
    rclcpp::Parameter("trajectory_scaling.limits.override_urdf", true),
    rclcpp::Parameter("trajectory_scaling.limits.joint1.max_velocity", 3.0),
    rclcpp::Parameter("trajectory_scaling.limits.joint2.max_velocity", 3.0),
    rclcpp::Parameter("trajectory_scaling.limits.joint3.max_velocity", 3.0),
  };
    
  SetUpAndActivateTrajectoryController(executor, params);
  // Create a QoS profile that matches the controller's speed_scaling subscriber
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  auto trajectory_scaling_pub = node_->create_publisher<control_msgs::msg::SpeedScalingFactor>(
    controller_name_ + "/trajectory_scaling_input", qos);
  subscribeToState(executor);

 
  trajectory_scaling_pub->publish(msg);
  traj_controller_->wait_for_trajectory(executor);

  updateController();

  // Spin to receive latest state
  executor.spin_some();
  auto state = getState();
  EXPECT_EQ(state->speed_scaling_factor, msg.factor);

  // 0.0 should work as an edge case
  msg.factor = 0.0;
  trajectory_scaling_pub->publish(msg);
  traj_controller_->wait_for_trajectory(executor);
  updateController(rclcpp::Duration::from_seconds(0.01));
  executor.spin_some();
  state = getState();
  EXPECT_GE(state->speed_scaling_factor, msg.factor);
  updateController(rclcpp::Duration::from_seconds(1.0));
  executor.spin_some();
  state = getState();
  EXPECT_NEAR(state->speed_scaling_factor, msg.factor, 1e-4);

  // Sending a negative value will be ignored
  msg.factor = 0.45;
  trajectory_scaling_pub->publish(msg);
  traj_controller_->wait_for_trajectory(executor);
  msg.factor = -0.12;
  trajectory_scaling_pub->publish(msg);
  traj_controller_->wait_for_trajectory(executor);
  updateController(rclcpp::Duration::from_seconds(1.0));
  executor.spin_some();
  state = getState();
  EXPECT_NEAR(state->speed_scaling_factor,  0.45, 1e-4);
}

TEST_F(TrajectoryControllerTest, setting_scaling_factor_withbounded_velocities)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("trajectory_scaling.filter_coefficient", 0.5),
    rclcpp::Parameter("trajectory_scaling.subscribed_topics",std::vector<std::string>{
          "~/trajectory_scaling_input" }),
    // rclcpp::Parameter("trajectory_scaling.limits.override_urdf", false),
    // rclcpp::Parameter("trajectory_scaling.limits.joint1.max_velocity", 3.0),
    // rclcpp::Parameter("trajectory_scaling.limits.joint2.max_velocity", 3.0),
    // rclcpp::Parameter("trajectory_scaling.limits.joint3.max_velocity", 3.0),
    rclcpp::Parameter("trajectory_scaling.limits.joint1.max_acceleration", 0.00003),
    rclcpp::Parameter("trajectory_scaling.limits.joint2.max_acceleration", 3.0),
    rclcpp::Parameter("trajectory_scaling.limits.joint3.max_acceleration", 3.0),
  };
  SetUpAndActivateTrajectoryController(executor, params);

  auto trajectory_scaling_pub = node_->create_publisher<control_msgs::msg::SpeedScalingFactor>(
    controller_name_ + "/trajectory_scaling_input", rclcpp::SystemDefaultsQoS().transient_local());
  subscribeToState(executor);
  updateController(rclcpp::Duration::from_seconds(0.01));  // an exponential moving average is used
  // Spin to receive latest state
  executor.spin_some();
  auto state = getState();
  EXPECT_NEAR(state->speed_scaling_factor, speed_scaling_factor_,1e-4);

  control_msgs::msg::SpeedScalingFactor msg;
  msg.factor = 0.765;
  trajectory_scaling_pub->publish(msg);
  traj_controller_->wait_for_trajectory(executor);

  updateController(rclcpp::Duration::from_seconds(.01));  // an exponential moving average is used

  // Spin to receive latest state
  executor.spin_some();
  state = getState();
  // Since we have a speed scaling state interface active, the value set via topic will be
  // overwritten from the state interface. The value should not have changed much bbeacuse of the exponential filter
  EXPECT_GE(std::fabs(state->speed_scaling_factor - msg.factor),1e-3);

  updateController(rclcpp::Duration::from_seconds(1.0));  // an exponential moving average is used

  // Spin to receive latest state
  executor.spin_some();
  state = getState();
  // Since we have a speed scaling state interface active, the value set via topic will be
  // overwritten from the state interface.
  EXPECT_NEAR(state->speed_scaling_factor, msg.factor,1e-4);
}

/**
 * @brief check the managments of the limits
 */
TEST_F(TrajectoryControllerTest, limits_from_urdf)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  std::vector<rclcpp::Parameter> params = {};
  SetUpTrajectoryController(executor, params, test_trajectory_controllers::urdf_rrrbot_continuous);

  auto state = traj_controller_->configure();
  auto max_velocities = traj_controller_->get_max_velocities();
  auto max_accelerations = traj_controller_->get_max_accelerations();
  ASSERT_TRUE(max_velocities.find("joint1")!=max_velocities.end());
  ASSERT_TRUE(max_velocities.find("joint2")!=max_velocities.end());
  ASSERT_TRUE(max_velocities.find("joint3")!=max_velocities.end());
  ASSERT_TRUE(max_accelerations.find("joint1")!=max_accelerations.end());
  ASSERT_TRUE(max_accelerations.find("joint2")!=max_accelerations.end());
  ASSERT_TRUE(max_accelerations.find("joint3")!=max_accelerations.end());
  ASSERT_EQ(max_velocities["joint1"],0.2);
  ASSERT_EQ(max_velocities["joint2"],0.2);
  ASSERT_EQ(max_velocities["joint3"],0.2);
  ASSERT_EQ(max_accelerations["joint1"],std::numeric_limits<double>::infinity());
  ASSERT_EQ(max_accelerations["joint2"],std::numeric_limits<double>::infinity());
  ASSERT_EQ(max_accelerations["joint3"],std::numeric_limits<double>::infinity());
  executor.cancel();
}

/**
 * @brief check the managments of the limits
 */
TEST_F(TrajectoryControllerTest, limits_from_urdf_gt_param)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("trajectory_scaling.limits.override_urdf", true),
    rclcpp::Parameter("trajectory_scaling.limits.joint1.max_velocity", 0.1),
    rclcpp::Parameter("trajectory_scaling.limits.joint2.max_velocity", 0.01),
    rclcpp::Parameter("trajectory_scaling.limits.joint3.max_velocity", 0.05),
    rclcpp::Parameter("trajectory_scaling.limits.joint1.max_acceleration", 1.0),
    rclcpp::Parameter("trajectory_scaling.limits.joint2.max_acceleration", .1),
    rclcpp::Parameter("trajectory_scaling.limits.joint3.max_acceleration", .5),
  };
  SetUpAndActivateTrajectoryController(executor, params);

  auto state = traj_controller_->configure();
  auto max_velocities = traj_controller_->get_max_velocities();
  auto max_accelerations = traj_controller_->get_max_accelerations();
  ASSERT_TRUE(max_velocities.find("joint1")!=max_velocities.end());
  ASSERT_TRUE(max_velocities.find("joint2")!=max_velocities.end());
  ASSERT_TRUE(max_velocities.find("joint3")!=max_velocities.end());
  ASSERT_TRUE(max_accelerations.find("joint1")!=max_accelerations.end());
  ASSERT_TRUE(max_accelerations.find("joint2")!=max_accelerations.end());
  ASSERT_TRUE(max_accelerations.find("joint3")!=max_accelerations.end());
  ASSERT_EQ(max_velocities["joint1"],0.1);
  ASSERT_EQ(max_velocities["joint2"],0.01);
  ASSERT_EQ(max_velocities["joint3"],0.05);
  ASSERT_EQ(max_accelerations["joint1"],1.0);
  ASSERT_EQ(max_accelerations["joint2"],0.1);
  ASSERT_EQ(max_accelerations["joint3"],0.5);
  executor.cancel();
}

/**
 * @brief check the managments of the limits
 */
TEST_F(TrajectoryControllerTest, limits_from_urdf_lt_param)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("trajectory_scaling.limits.override_urdf", false),
    rclcpp::Parameter("trajectory_scaling.limits.joint1.max_velocity", 3.0),
    rclcpp::Parameter("trajectory_scaling.limits.joint2.max_velocity", 3.0),
    rclcpp::Parameter("trajectory_scaling.limits.joint3.max_velocity", 3.0),
    rclcpp::Parameter("trajectory_scaling.limits.joint1.max_acceleration", 1.0),
    rclcpp::Parameter("trajectory_scaling.limits.joint2.max_acceleration", .1),
    rclcpp::Parameter("trajectory_scaling.limits.joint3.max_acceleration", .5),
  };
  SetUpTrajectoryController(executor, params, test_trajectory_controllers::urdf_rrrbot_continuous);

  auto state = traj_controller_->configure();
  auto max_velocities = traj_controller_->get_max_velocities();
  auto max_accelerations = traj_controller_->get_max_accelerations();
  ASSERT_TRUE(max_velocities.find("joint1")!=max_velocities.end());
  ASSERT_TRUE(max_velocities.find("joint2")!=max_velocities.end());
  ASSERT_TRUE(max_velocities.find("joint3")!=max_velocities.end());
  ASSERT_TRUE(max_accelerations.find("joint1")!=max_accelerations.end());
  ASSERT_TRUE(max_accelerations.find("joint2")!=max_accelerations.end());
  ASSERT_TRUE(max_accelerations.find("joint3")!=max_accelerations.end());
  ASSERT_EQ(max_velocities["joint1"],0.2);
  ASSERT_EQ(max_velocities["joint2"],0.2);
  ASSERT_EQ(max_velocities["joint3"],0.2);
  ASSERT_EQ(max_accelerations["joint1"],1.0);
  ASSERT_EQ(max_accelerations["joint2"],0.1);
  ASSERT_EQ(max_accelerations["joint3"],0.5);
  executor.cancel();
}
