// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include "cia406_position_controller/cia406_position_controller.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace cia406_position_controller
{

controller_interface::CallbackReturn Cia406PositionController::on_init()
{
  try {
    auto_declare<std::string>("joint", "encoder_joint");
    auto_declare<std::string>("raw_state_interface", "encoder_joint/rpdo/data");
    auto_declare<std::string>("index_state_interface", "encoder_joint/rpdo/index");
    auto_declare<std::string>("exported_interface", "position");
    auto_declare<double>("counts_per_unit", 1.0);
    auto_declare<double>("offset", 0.0);
    auto_declare<int>("position_object_index", 0x6004);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Cia406PositionController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_ = get_node()->get_parameter("joint").as_string();
  raw_state_interface_ = get_node()->get_parameter("raw_state_interface").as_string();
  index_state_interface_ = get_node()->get_parameter("index_state_interface").as_string();
  exported_interface_ = get_node()->get_parameter("exported_interface").as_string();
  counts_per_unit_ = get_node()->get_parameter("counts_per_unit").as_double();
  offset_ = get_node()->get_parameter("offset").as_double();
  position_object_index_ = get_node()->get_parameter("position_object_index").as_int();

  if (counts_per_unit_ == 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "counts_per_unit must be non-zero");
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

// Claim the raw, generic proxy state interfaces. We read both the data slot and
// the index slot so we can verify the multiplexed value is actually 0x6004.
controller_interface::InterfaceConfiguration
Cia406PositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names = {raw_state_interface_, index_state_interface_};
  return cfg;
}

// Read-only converter: no hardware command interfaces are claimed.
controller_interface::InterfaceConfiguration
Cia406PositionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::NONE;
  return cfg;
}

// Export exactly one converted STATE interface: <controller_name>/<exported>.
std::vector<hardware_interface::StateInterface>
Cia406PositionController::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back(
    hardware_interface::StateInterface(
      get_node()->get_name(), exported_interface_, &exported_position_));
  return interfaces;
}

// No reference (command) interfaces — nothing upstream writes into this one.
std::vector<hardware_interface::CommandInterface>
Cia406PositionController::on_export_reference_interfaces()
{
  return {};
}

bool Cia406PositionController::on_set_chained_mode(bool /*chained_mode*/)
{
  // State-interface exporters are not driven into chained mode by consumers;
  // nothing to toggle here.
  return true;
}

controller_interface::CallbackReturn Cia406PositionController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  raw_data_idx_ = -1;
  raw_index_idx_ = -1;
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    const auto name = state_interfaces_[i].get_name();
    if (name == raw_state_interface_) { raw_data_idx_ = static_cast<int>(i); }
    if (name == index_state_interface_) { raw_index_idx_ = static_cast<int>(i); }
  }
  if (raw_data_idx_ < 0) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "raw state interface '%s' not found among claimed handles",
      raw_state_interface_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type Cia406PositionController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return controller_interface::return_type::OK;
}

controller_interface::return_type Cia406PositionController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // NOTE: newer ros2_control replaces get_value() with get_optional<T>().
  // Defensive demux: only convert when the multiplexed slot carries 0x6004.
  if (raw_index_idx_ >= 0) {
    const double arrived_index = state_interfaces_[raw_index_idx_].get_value();
    if (static_cast<int64_t>(arrived_index) != position_object_index_) {
      // Stale / different object in the mailbox this cycle; keep last value.
      return controller_interface::return_type::OK;
    }
  }

  const double raw_counts = state_interfaces_[raw_data_idx_].get_value();
  if (std::isnan(raw_counts)) {
    return controller_interface::return_type::OK;
  }

  exported_position_ = (raw_counts / counts_per_unit_) + offset_;
  return controller_interface::return_type::OK;
}

}  // namespace cia406_position_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  cia406_position_controller::Cia406PositionController,
  controller_interface::ChainableControllerInterface)
