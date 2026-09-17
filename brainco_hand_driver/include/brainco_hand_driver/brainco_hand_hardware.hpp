// Copyright (c) 2025 BrainCo
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

#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "brainco_hand_driver/brainco_hand_api.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace brainco_hand_driver
{

class BraincoHandHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(BraincoHandHardware)

  auto on_init(const hardware_interface::HardwareInfo & info)
  -> hardware_interface::CallbackReturn override;
  auto on_configure(const rclcpp_lifecycle::State & previous_state)
  -> hardware_interface::CallbackReturn override;
  auto on_cleanup(const rclcpp_lifecycle::State & previous_state)
  -> hardware_interface::CallbackReturn override;
  auto on_activate(const rclcpp_lifecycle::State & previous_state)
  -> hardware_interface::CallbackReturn override;
  auto on_deactivate(const rclcpp_lifecycle::State & previous_state)
  -> hardware_interface::CallbackReturn override;
  auto export_state_interfaces() -> std::vector<hardware_interface::StateInterface> override;
  auto export_command_interfaces() -> std::vector<hardware_interface::CommandInterface> override;
  auto read(const rclcpp::Time & time, const rclcpp::Duration & period)
  -> hardware_interface::return_type override;
  auto write(const rclcpp::Time & time, const rclcpp::Duration & period)
  -> hardware_interface::return_type override;

private:
  struct DriverConfig
  {
    BraincoHandApi::DriverConfig transport{};
    std::vector<double> joint_max_positions_rad{};
  };

  auto init_parameters() -> hardware_interface::CallbackReturn;
  auto validate_joints() const -> hardware_interface::CallbackReturn;
  auto open_connection() -> bool;
  void close_connection();
  static auto parse_log_level(const std::string & value) -> BraincoLogLevel;
  static auto parse_bool(const std::string & value, bool default_value) -> bool;
  static auto parse_double_list(const std::string & value) -> std::vector<double>;
  auto get_parameter(const std::string & key, const std::string & default_value) const
  -> std::string;

  DriverConfig config_{};
  BraincoHandApi api_{};
  std::optional<BraincoHandApi::ConnectionInfo> resolved_connection_{};
  bool is_active_{false};
  std::vector<double> hw_positions_{};
  std::vector<double> hw_commands_{};
  std::array<uint16_t, BraincoHandApi::kFingerCount> last_sent_positions_{};
  bool has_last_sent_positions_{false};
};

}  // namespace brainco_hand_driver
