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

#include "brainco_hand_driver/brainco_hand_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

#include "brainco_hand_driver/logger_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace brainco_hand_driver
{

namespace
{
constexpr std::size_t kFingerCount{BraincoHandApi::kFingerCount};
constexpr double kDevicePositionMax{1000.0};
}  // namespace

auto BraincoHandHardware::on_init(const hardware_interface::HardwareInfo & info)
-> hardware_interface::CallbackReturn
{
  const auto result = hardware_interface::SystemInterface::on_init(info);
  if (result != hardware_interface::CallbackReturn::SUCCESS) {
    return result;
  }
  if (
    init_parameters() != hardware_interface::CallbackReturn::SUCCESS ||
    validate_joints() != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.assign(kFingerCount, 0.0);
  hw_commands_.assign(kFingerCount, 0.0);
  BRAINCO_HAND_LOG_INFO(
    "Revo1 right-hand Modbus driver initialized (slave_id=%u, log_level=%s)",
    config_.transport.slave_id,
    brainco_log_level_to_string(config_.transport.log_level).c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto BraincoHandHardware::on_configure(const rclcpp_lifecycle::State & previous_state)
-> hardware_interface::CallbackReturn
{
  (void)previous_state;
  std::fill(hw_positions_.begin(), hw_positions_.end(), 0.0);
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  has_last_sent_positions_ = false;
  return open_connection() ? hardware_interface::CallbackReturn::SUCCESS :
         hardware_interface::CallbackReturn::ERROR;
}

auto BraincoHandHardware::on_cleanup(const rclcpp_lifecycle::State & previous_state)
-> hardware_interface::CallbackReturn
{
  (void)previous_state;
  close_connection();
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto BraincoHandHardware::on_activate(const rclcpp_lifecycle::State & previous_state)
-> hardware_interface::CallbackReturn
{
  (void)previous_state;
  if (!api_.is_open()) {
    BRAINCO_HAND_LOG_ERROR("Cannot activate: Modbus connection is not open");
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_ = hw_positions_;
  has_last_sent_positions_ = false;
  is_active_ = true;
  BRAINCO_HAND_LOG_INFO("Revo1 right hand activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto BraincoHandHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state)
-> hardware_interface::CallbackReturn
{
  (void)previous_state;
  is_active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto BraincoHandHardware::export_state_interfaces()
-> std::vector<hardware_interface::StateInterface>
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(kFingerCount);
  for (std::size_t index = 0; index < kFingerCount; ++index) {
    interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_POSITION, &hw_positions_[index]);
  }
  return interfaces;
}

auto BraincoHandHardware::export_command_interfaces()
-> std::vector<hardware_interface::CommandInterface>
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(kFingerCount);
  for (std::size_t index = 0; index < kFingerCount; ++index) {
    interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_POSITION, &hw_commands_[index]);
  }
  return interfaces;
}

auto BraincoHandHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
-> hardware_interface::return_type
{
  (void)time;
  (void)period;
  if (!api_.is_open()) {
    return hardware_interface::return_type::ERROR;
  }

  const auto status = api_.get_motor_status(config_.transport.slave_id);
  if (!status) {
    BRAINCO_HAND_LOG_WARN("Failed to read Revo1 motor positions");
    return hardware_interface::return_type::ERROR;
  }

  for (std::size_t index = 0; index < kFingerCount; ++index) {
    hw_positions_[index] =
      static_cast<double>(status->positions[index]) * config_.joint_max_positions_rad[index] /
      kDevicePositionMax;
  }
  return hardware_interface::return_type::OK;
}

auto BraincoHandHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
-> hardware_interface::return_type
{
  (void)time;
  (void)period;
  if (!api_.is_open()) {
    return hardware_interface::return_type::ERROR;
  }
  if (!is_active_) {
    return hardware_interface::return_type::OK;
  }

  std::array<uint16_t, kFingerCount> positions{};
  for (std::size_t index = 0; index < kFingerCount; ++index) {
    const double raw =
      hw_commands_[index] * kDevicePositionMax / config_.joint_max_positions_rad[index];
    positions[index] = static_cast<uint16_t>(std::lround(std::clamp(raw, 0.0, kDevicePositionMax)));
  }

  if (has_last_sent_positions_ && positions == last_sent_positions_) {
    return hardware_interface::return_type::OK;
  }
  if (!api_.set_finger_positions(config_.transport.slave_id, positions.data(), positions.size())) {
    BRAINCO_HAND_LOG_WARN("Failed to send Revo1 position command");
    return hardware_interface::return_type::ERROR;
  }

  last_sent_positions_ = positions;
  has_last_sent_positions_ = true;
  return hardware_interface::return_type::OK;
}

auto BraincoHandHardware::init_parameters() -> hardware_interface::CallbackReturn
{
  try {
    const auto slave_id = std::stoul(get_parameter("slave_id", "1"));
    if (slave_id == 0 || slave_id > 247) {
      BRAINCO_HAND_LOG_ERROR("slave_id must be in [1, 247]");
      return hardware_interface::CallbackReturn::ERROR;
    }
    config_.transport.slave_id = static_cast<uint8_t>(slave_id);
    config_.transport.log_level = parse_log_level(get_parameter("log_level", "info"));
    config_.transport.modbus.port = get_parameter("port", "/dev/ttyUSB0");
    config_.transport.modbus.baudrate =
      static_cast<uint32_t>(std::stoul(get_parameter("baudrate", "115200")));
    config_.transport.modbus.auto_detect = parse_bool(get_parameter("auto_detect", "true"), true);
    config_.transport.modbus.auto_detect_quick =
      parse_bool(get_parameter("auto_detect_quick", "true"), true);
    config_.transport.modbus.auto_detect_port = get_parameter("auto_detect_port", "");
    config_.joint_max_positions_rad =
      parse_double_list(get_parameter("joint_max_positions_rad", ""));
  } catch (const std::exception & error) {
    BRAINCO_HAND_LOG_ERROR("Invalid hardware parameter: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (
    config_.joint_max_positions_rad.size() != kFingerCount ||
    std::any_of(
      config_.joint_max_positions_rad.begin(), config_.joint_max_positions_rad.end(),
      [](double value) {return !std::isfinite(value) || value <= 0.0;}))
  {
    BRAINCO_HAND_LOG_ERROR("joint_max_positions_rad must contain six positive values");
    return hardware_interface::CallbackReturn::ERROR;
  }

  BRAINCO_HAND_LOG_INFO(
    "Modbus config: port=%s baudrate=%u auto_detect=%s",
    config_.transport.modbus.port.c_str(), config_.transport.modbus.baudrate,
    config_.transport.modbus.auto_detect ? "true" : "false");
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto BraincoHandHardware::validate_joints() const -> hardware_interface::CallbackReturn
{
  if (info_.joints.size() != kFingerCount) {
    BRAINCO_HAND_LOG_ERROR("Expected six Revo1 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    const bool position_state = std::any_of(
      joint.state_interfaces.begin(), joint.state_interfaces.end(),
      [](const auto & interface) {return interface.name == hardware_interface::HW_IF_POSITION;});
    const bool position_command = std::any_of(
      joint.command_interfaces.begin(), joint.command_interfaces.end(),
      [](const auto & interface) {return interface.name == hardware_interface::HW_IF_POSITION;});
    if (!position_state || !position_command) {
      BRAINCO_HAND_LOG_ERROR("Joint %s requires position state and command", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto BraincoHandHardware::open_connection() -> bool
{
  close_connection();
  api_.configure(config_.transport);
  if (!api_.open()) {
    BRAINCO_HAND_LOG_ERROR("Failed to open Revo1 Modbus connection");
    return false;
  }

  resolved_connection_ = api_.resolved_connection();
  if (resolved_connection_) {
    config_.transport.slave_id = resolved_connection_->slave_id;
    BRAINCO_HAND_LOG_INFO(
      "Connected: port=%s baudrate=%u slave_id=%u", resolved_connection_->port.c_str(),
      resolved_connection_->baudrate, resolved_connection_->slave_id);
  }

  BraincoHandApi::DeviceInfoData device_info{};
  if (api_.fetch_device_info(config_.transport.slave_id, device_info)) {
    BRAINCO_HAND_LOG_INFO(
      "Device: serial=%s firmware=%s",
      device_info.serial_number.empty() ? "<unknown>" : device_info.serial_number.c_str(),
      device_info.firmware_version.empty() ? "<unknown>" : device_info.firmware_version.c_str());
  }
  return true;
}

void BraincoHandHardware::close_connection()
{
  if (api_.is_open()) {
    api_.close();
  }
  resolved_connection_.reset();
  is_active_ = false;
}

auto BraincoHandHardware::parse_log_level(const std::string & value) -> BraincoLogLevel
{
  std::string level = value;
  std::transform(
    level.begin(), level.end(), level.begin(), [](unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });
  if (level == "error") {return BraincoLogLevel::kError;}
  if (level == "warn" || level == "warning") {return BraincoLogLevel::kWarn;}
  if (level == "debug") {return BraincoLogLevel::kDebug;}
  if (level == "trace") {return BraincoLogLevel::kTrace;}
  return BraincoLogLevel::kInfo;
}

auto BraincoHandHardware::parse_bool(const std::string & value, bool default_value) -> bool
{
  std::string normalized = value;
  std::transform(
    normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });
  if (normalized == "true" || normalized == "1" || normalized == "yes" || normalized == "on") {
    return true;
  }
  if (normalized == "false" || normalized == "0" || normalized == "no" || normalized == "off") {
    return false;
  }
  return default_value;
}

auto BraincoHandHardware::parse_double_list(const std::string & value) -> std::vector<double>
{
  std::vector<double> values;
  std::stringstream stream(value);
  std::string token;
  while (std::getline(stream, token, ',')) {
    const auto first = token.find_first_not_of(" \t[]");
    const auto last = token.find_last_not_of(" \t[]");
    if (first != std::string::npos) {
      values.push_back(std::stod(token.substr(first, last - first + 1)));
    }
  }
  return values;
}

auto BraincoHandHardware::get_parameter(
  const std::string & key, const std::string & default_value) const -> std::string
{
  const auto iterator = info_.hardware_parameters.find(key);
  return iterator != info_.hardware_parameters.end() && !iterator->second.empty() ?
         iterator->second :
         default_value;
}

}  // namespace brainco_hand_driver

PLUGINLIB_EXPORT_CLASS(
  brainco_hand_driver::BraincoHandHardware, hardware_interface::SystemInterface)
