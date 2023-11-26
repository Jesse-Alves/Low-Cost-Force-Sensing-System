// Copyright 2022, ICube Laboratory, University of Strasbourg
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

#include "rrbot_pdg_controller/rrbot_pdg_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace rrbot_pdg_controller
{
using hardware_interface::LoanedCommandInterface;

RRbotPDGController::RRbotPDGController()
: controller_interface::ControllerInterface(),
  rt_command_ptr_(nullptr),
  joints_command_subscriber_(nullptr)
{
}

CallbackReturn RRbotPDGController::on_init()
{
  try {
    // definition of the parameters that need to be queried from the
    // controller configuration file with default values
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<double>("KP", double());
    auto_declare<double>("KD", double());
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn RRbotPDGController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // getting the names of the joints to be controlled
  joint_names_ = get_node()->get_parameter("joints").as_string_array();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return CallbackReturn::FAILURE;
  }

  if(!(kp_>=0 && kd_>=0)){
    RCLCPP_ERROR(get_node()->get_logger(), "Bad 'KP' or 'KD' parameter");
    return CallbackReturn::FAILURE;
  }
 
  // the desired joint position is queried from the commands topic
  // and passed to update via a rt pipe
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) {rt_command_ptr_.writeFromNonRT(msg);});

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}
// As the simulated rrbot hardware interface only supports effort commands, it can be directly
// defined here without the need of getting as parameter. The effort interface is then 
// affected to all controlled joints.
controller_interface::InterfaceConfiguration
RRbotPDGController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size());
  for (const auto & joint_name : joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return conf;
}
// The controller requires the current position and velocity states. For this reason
// it can be directly defined here without the need of getting as parameters.
// The state interface is then deployed to all targeted joints.
controller_interface::InterfaceConfiguration
RRbotPDGController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size());
  for (const auto & joint_name : joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return conf;
}

// Fill ordered_interfaces with references to the matching interfaces
// in the same order as in joint_names
template<typename T>
bool get_ordered_interfaces(
  std::vector<T> & unordered_interfaces, const std::vector<std::string> & joint_names,
  const std::string & interface_type, std::vector<std::reference_wrapper<T>> & ordered_interfaces)
{
  for (const auto & joint_name : joint_names) {
    for (auto & command_interface : unordered_interfaces) {
      if (command_interface.get_name() == joint_name + "/" + interface_type) {
        ordered_interfaces.push_back(std::ref(command_interface));
      }
    }
  }

  return joint_names.size() == ordered_interfaces.size();
}

CallbackReturn RRbotPDGController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::vector<std::reference_wrapper<LoanedCommandInterface>> ordered_interfaces;
  if (
    !get_ordered_interfaces(
      command_interfaces_, joint_names_, hardware_interface::HW_IF_EFFORT, ordered_interfaces) ||
    command_interfaces_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu effort command interfaces, got %zu",
      joint_names_.size(),
      ordered_interfaces.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn RRbotPDGController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn RRbotPDGController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn RRbotPDGController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn RRbotPDGController::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}
// main control loop function getting the state interface and writing to the command interface
controller_interface::return_type RRbotPDGController::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // getting the data from the subscriber using the rt pipe
  auto reference_joint_position = rt_command_ptr_.readFromRT();

  // getting controller KP and KD coefficients
  kp_ = get_node()->get_parameter("KP").as_double();
  kd_ = get_node()->get_parameter("KD").as_double();

  // the states are given in the same order as defines in state_interface_configuration
    q_pos_j1_ = state_interfaces_[0].get_value();
    q_vel_j1_ = state_interfaces_[1].get_value();
    q_pos_j2_ = state_interfaces_[2].get_value();
    q_vel_j2_ = state_interfaces_[3].get_value();

  // no command received yet
  if (!reference_joint_position || !(*reference_joint_position)) {
    q_ref_j1_ = q_pos_j1_;
    q_ref_j2_ = q_pos_j2_;
  } else {
    //checking proxy data validity
    if ((*reference_joint_position)->data.size() != joint_names_.size()){
        RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(), 1000, "command size does not match number of interfaces");
        return controller_interface::return_type::ERROR;
    }
    q_ref_j1_ = (*reference_joint_position)->data[0];
    q_ref_j2_ = (*reference_joint_position)->data[1];
  }

  // ---------------------- START TODO ------------------------------------

    g_j1_ = (g_*lc1_*m1_ + g_*l1_*m2_)*cos(q_pos_j1_) + g_*lc2_*m2_*cos(q_pos_j1_ + q_pos_j2_);
    g_j2_ = g_*lc2_*m2_*cos(q_pos_j1_ + q_pos_j2_);

    tau_j1_ = g_j1_ + kp_*(q_ref_j1_ - q_pos_j1_) - kd_*(q_vel_j1_);
    tau_j2_ = g_j2_ + kp_*(q_ref_j2_ - q_pos_j2_) - kd_*(q_vel_j2_);

  // ------------------ Operation Space ------------------------------------
/*    c1 = cos(q_pos_j1_);
    c2 = cos(q_pos_j2_);
    s1 = sin(q_pos_j1_);
    s2 = sin(q_pos_j2_);

    e = Eigen::Vector2d::Zero();
    e(0)=q_ref_j1_-q_pos_j1_;
    e(1)=q_ref_j2_-q_pos_j2_;

    Ja_1= Eigen::Vector2d::Zero();
    Ja_2= Eigen::Vector2d::Zero();

    Ja_1(0) = -l1_*s1*q_vel_j1_-l2_*q_vel_j1_*(c2*s1+c1*s2);
    Ja_1(1) = l1_*c1*q_vel_j1_+l2_*q_vel_j1_*(c1*c2-s1*s2);
    Ja_2(0) = -l2_*q_vel_j2_*(c1*s2+s1*c2);
    Ja_2(1) = l2_*q_vel_j2_*(c1*c2-s1*s2);

    g_j1_ = m1_*g_*lc1_*cos(q_pos_j1_) + m2_*g_*(l1_*cos(q_pos_j1_)+lc2_*cos(q_pos_j1_+q_pos_j2_));
    g_j2_ = m2_*g_*lc2_*cos(q_pos_j1_+q_pos_j2_);


    tau_j1_ = g_j1_+Ja_1*kp_*e- kd_*q_vel_j1_;
    tau_j2_ = g_j2_+Ja_2*kp_*e - kd_*q_vel_j2_;
*/

  // ---------------------- STOP TODO ------------------------------------
    
    command_interfaces_[0].set_value(tau_j1_);
    command_interfaces_[1].set_value(tau_j2_);

  return controller_interface::return_type::OK;
}

}  // namespace rrbot_pdg_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rrbot_pdg_controller::RRbotPDGController, controller_interface::ControllerInterface)
