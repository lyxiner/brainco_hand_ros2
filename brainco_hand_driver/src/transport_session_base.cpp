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

#include "brainco_hand_driver/transport_session_base.hpp"

#include <chrono>
#include <thread>

#include "brainco_hand_driver/logger_macros.hpp"
#include "brainco_hand_driver/sdk_helpers.hpp"

namespace brainco_hand_driver
{

SessionBase::SessionBase(BraincoHandApi::DriverConfig & config)
: config_(config) {}

bool SessionBase::fetch_device_info(uint8_t slave_id, BraincoHandApi::DeviceInfoData & info) const
{
  if (!handler_) {
    return false;
  }

  DeviceInfoPtr device_info{::stark_get_device_info(handler_, slave_id)};
  if (!device_info) {
    return false;
  }

  info.sku_type = static_cast<uint8_t>(device_info->sku_type);
  info.hardware_type = static_cast<uint8_t>(device_info->hardware_type);
  info.serial_number = device_info->serial_number ? device_info->serial_number : std::string{};
  info.firmware_version =
    device_info->firmware_version ? device_info->firmware_version : std::string{};
  return true;
}

std::optional<BraincoHandApi::MotorStatus> SessionBase::get_motor_status(uint8_t slave_id) const
{
  if (!handler_) {
    return std::nullopt;
  }

  MotorStatusPtr raw_status{::stark_get_motor_status(handler_, slave_id)};
  if (!raw_status) {
    return std::nullopt;
  }

  BraincoHandApi::MotorStatus status{};
  for (std::size_t index = 0; index < BraincoHandApi::kFingerCount; ++index) {
    status.positions[index] = raw_status->positions[index];
  }
  return status;
}

bool SessionBase::set_finger_positions(
  uint8_t slave_id, const uint16_t * positions, std::size_t count)
{
  if (!handler_ || !positions || count != BraincoHandApi::kFingerCount) {
    return false;
  }

  ::stark_set_finger_positions(handler_, slave_id, positions, count);
  return true;
}

bool SessionBase::set_finger_protected_current(
  uint8_t slave_id, std::size_t finger_index, uint16_t current_ma)
{
  if (!handler_ || finger_index >= BraincoHandApi::kFingerCount) {
    return false;
  }

  const auto finger_id = static_cast<StarkFingerId>(finger_index + 1U);
  const auto current_value = ::stark_get_finger_protected_current(handler_, slave_id, finger_id);
  if (current_value == current_ma) {
    return true;
  }

  ::stark_set_finger_protected_current(handler_, slave_id, finger_id, current_ma);
  constexpr std::size_t kReadbackAttempts{3};
  constexpr auto kReadbackDelay = std::chrono::milliseconds{100};
  for (std::size_t attempt = 0; attempt < kReadbackAttempts; ++attempt) {
    std::this_thread::sleep_for(kReadbackDelay);
    if (::stark_get_finger_protected_current(handler_, slave_id, finger_id) == current_ma) {
      return true;
    }
  }
  return false;
}

void SessionBase::set_handler(DeviceHandler * handler) {handler_ = handler;}

void SessionBase::clear_handler() {handler_ = nullptr;}

}  // namespace brainco_hand_driver
