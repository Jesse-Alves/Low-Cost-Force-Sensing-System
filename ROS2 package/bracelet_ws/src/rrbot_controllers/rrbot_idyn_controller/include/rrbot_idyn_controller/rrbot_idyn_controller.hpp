// Copyright 2023, ICube Laboratory, University of Strasbourg
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
// Author: Maciej Bednarczyk (macbednarczyk@gmail.com)

#ifndef RRBOT_IDYN_CONTROLLER__RRBOT_IDYN_CONTROLLER_HPP_
#define RRBOT_IDYN_CONTROLLER__RRBOT_IDYN_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/QR>

#include "controller_interface/controller_interface.hpp"
#include "rrbot_idyn_controller/visibility_control.h"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace rrbot_idyn_controller
{
using CmdType = trajectory_msgs::msg::JointTrajectory;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RRbotIDynController : public controller_interface::ControllerInterface
{
public:
  RRBOT_IDYN_CONTROLLER_PUBLIC
  RRbotIDynController();

  RRBOT_IDYN_CONTROLLER_PUBLIC
  CallbackReturn on_init() override;

  RRBOT_IDYN_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  RRBOT_IDYN_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  RRBOT_IDYN_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  RRBOT_IDYN_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  RRBOT_IDYN_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  RRBOT_IDYN_CONTROLLER_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  RRBOT_IDYN_CONTROLLER_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
  
  RRBOT_IDYN_CONTROLLER_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

  RRBOT_IDYN_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

protected:
  std::vector<std::string> joint_names_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;

  std::string logger_name_;
  double kp_, kd_;

  Eigen::Vector2d q_pos_ref_;
  Eigen::Vector2d q_vel_ref_;
  Eigen::Vector2d q_acc_ref_;

  Eigen::Vector2d q_pos_;
  Eigen::Vector2d q_vel_;
  Eigen::Vector2d tau_;
  Eigen::Vector2d u_;

  Eigen::Matrix2d bmat_;
  Eigen::Vector2d cvec_;
  Eigen::Vector2d gvec_;

  double m1_ = 1.0;
  double l1_ = 1.0;
  double lc1_ = l1_/2; 
  double i1_ = m1_/12*l1_*l1_;
  double m2_ = 1.0;
  double l2_ = 1.0;
  double lc2_ = l2_/2; 
  double i2_ = m2_/12*l2_*l2_;
  double g_ = 9.8;  

};

}  // namespace rrbot_idyn_controller

#endif  // RRBOT_IDYN_CONTROLLER__RRBOT_IDYN_CONTROLLER_HPP_