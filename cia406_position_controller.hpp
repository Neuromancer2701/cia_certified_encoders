// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// Cia406PositionController
// ------------------------
// Chainable controller that reads the proxy CanopenSystem's raw, multiplexed
// `rpdo/data` (a CiA 406 position value at object 0x6004) and re-exports a
// converted position as a STATE interface for downstream controllers.
//
// This is the "Path 1" stopgap described in the README. The clean long-term
// solution is a dedicated Cia406Driver/Cia406System.

#ifndef CIA406_POSITION_CONTROLLER__CIA406_POSITION_CONTROLLER_HPP_
#define CIA406_POSITION_CONTROLLER__CIA406_POSITION_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace cia406_position_controller
{

class Cia406PositionController : public controller_interface::ChainableControllerInterface
{
public:
  Cia406PositionController() = default;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Read-only converter: no upstream commands to ingest.
  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Real-time: read raw counts, demux by index, scale, store in exported value.
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  bool on_set_chained_mode(bool chained_mode) override;

protected:
  // ---- API-churn watch -----------------------------------------------------
  // Across distros these export customization points have appeared as:
  //   export_reference_interfaces() / export_state_interfaces()   (by value)
  //   on_export_reference_interfaces() / on_export_state_interfaces()
  //   on_export_reference_interfaces_list() / on_export_state_interfaces_list()
  // Confirm the exact virtual names + return types in your distro's
  // chainable_controller_interface.hpp and adjust signatures accordingly.
  std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

private:
  // Parameters
  std::string joint_;
  std::string raw_state_interface_;     // e.g. "encoder_joint/rpdo/data"
  std::string index_state_interface_;   // e.g. "encoder_joint/rpdo/index"
  std::string exported_interface_;      // e.g. "position"
  double counts_per_unit_ {1.0};
  double offset_ {0.0};
  int64_t position_object_index_ {0x6004};

  // Storage backing the exported state interface (must outlive the handle).
  double exported_position_ {0.0};

  // Cached indices into state_interfaces_ for the claimed raw handles.
  int raw_data_idx_ {-1};
  int raw_index_idx_ {-1};
};

}  // namespace cia406_position_controller

#endif  // CIA406_POSITION_CONTROLLER__CIA406_POSITION_CONTROLLER_HPP_
