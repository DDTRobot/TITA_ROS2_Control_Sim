// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
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

#include "template_ros2_controller/template_ros2_controller.hpp"

#include "pluginlib/class_list_macros.hpp"
namespace tita_locomotion
{
TemplateRos2Controller::TemplateRos2Controller() {}

controller_interface::CallbackReturn TemplateRos2Controller::on_init()
{
  try {
    joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
    command_interface_types_ =
      auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
    state_interface_types_ =
      auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
    sensor_names_ = auto_declare<std::vector<std::string>>("sensors", sensor_names_);
    imu_sensor_ = std::make_unique<semantic_components::IMUSensor>(
      semantic_components::IMUSensor(sensor_names_[0]));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during on_init stage with message: %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  // param list init
  param_listener_ = std::make_shared<template_ros2_controller::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TemplateRos2Controller::on_configure(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "The 'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  for (std::string & joint_name : joint_names_) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Get joint name : %s", joint_name.c_str());
    std::shared_ptr<Joint> joint = std::make_shared<Joint>();
    joint->name = joint_name;
    joints_.emplace_back(joint);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration TemplateRos2Controller::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (std::shared_ptr<Joint> joint : joints_) {
    for (const auto & interface_type : command_interface_types_) {
      conf_names.push_back(joint->name + "/" + interface_type);
    }
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration TemplateRos2Controller::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (std::shared_ptr<Joint> joint : joints_) {
    for (const auto & interface_type : state_interface_types_)
      conf_names.push_back(joint->name + "/" + interface_type);
  }
  for (auto name : imu_sensor_->get_state_interface_names()) conf_names.push_back(name);
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type TemplateRos2Controller::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  // updata states
  std::vector<double> position, velocity, effort; 
  for (std::shared_ptr<Joint> joint : joints_) {
    position.push_back(joint->position_handle->get().get_value());
    velocity.push_back(joint->velocity_handle->get().get_value());
    effort.push_back(joint->effort_handle->get().get_value());
  }
  std::array<double, 3> accl = imu_sensor_->get_linear_acceleration();
  std::array<double, 3> gyro = imu_sensor_->get_angular_velocity();
  std::array<double, 4> quat = imu_sensor_->get_orientation();
  (void)accl;
  (void)gyro;
  (void)quat;
  // update torque
  for (std::shared_ptr<Joint> joint : joints_) {
    joint->effort_command_handle->get().set_value(0);
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn TemplateRos2Controller::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_activate");
  for (std::shared_ptr<Joint> joint : joints_) {
    // Position command
    const auto position_command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&joint](const auto & interface)
      {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
      });
    if (position_command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint command handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->position_command_handle = std::ref(*position_command_handle);

    // Velocity command
    const auto velocity_command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&joint](const auto & interface)
      {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
      });
    if (velocity_command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint command handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->velocity_command_handle = std::ref(*velocity_command_handle);

    // Effort command
    const auto effort_command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
      });
    if (effort_command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain effort command handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->effort_command_handle = std::ref(*effort_command_handle);
    // Position state
    const auto position_handle = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
      });
    if (position_handle == state_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint state handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->position_handle = std::ref(*position_handle);
    // Velocity state
    const auto velocity_handle = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
      });
    if (velocity_handle == state_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint state handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->velocity_handle = std::ref(*velocity_handle);
    // Effort state
    const auto effort_handle = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
      });
    if (effort_handle == state_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint state handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->effort_handle = std::ref(*effort_handle);
  }
  imu_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TemplateRos2Controller::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_deactivate ");
  for (uint id = 0; id < joints_.size(); id++) {
    joints_[id]->effort_command_handle->get().set_value(0);
  }
  imu_sensor_->release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TemplateRos2Controller::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_cleanup ");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TemplateRos2Controller::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_error ");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TemplateRos2Controller::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_shutdown ");
  return controller_interface::CallbackReturn::SUCCESS;
}


TemplateRos2Controller::~TemplateRos2Controller()
{
}

}  // namespace tita_locomotion

#include "class_loader/register_macro.hpp"

PLUGINLIB_EXPORT_CLASS(tita_locomotion::TemplateRos2Controller, controller_interface::ControllerInterface)