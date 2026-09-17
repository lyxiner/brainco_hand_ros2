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

#include "brainco_hand_driver/modbus_session.hpp"

#include <chrono>
#include <filesystem>
#include <string>
#include <thread>

#include "brainco_hand_driver/logger_macros.hpp"
#include "brainco_hand_driver/sdk_helpers.hpp"

namespace brainco_hand_driver
{

namespace
{
constexpr std::size_t kAutoDetectAttempts{5};
constexpr auto kAutoDetectRetryDelay = std::chrono::milliseconds{500};
}  // namespace

ModbusSession::ModbusSession(BraincoHandApi::DriverConfig & config)
: SessionBase(config), handle_(nullptr, &modbus_close)
{
}

bool ModbusSession::open()
{
  close();
  BraincoHandApi::ConnectionInfo connection{
    config_.modbus.port, config_.modbus.baudrate, config_.slave_id};

  if (config_.modbus.auto_detect) {
    const char * port_hint = config_.modbus.auto_detect_port.empty() ?
      nullptr :
      config_.modbus.auto_detect_port.c_str();
    BRAINCO_HAND_LOG_INFO(
      "Auto-detecting Revo1 Modbus device (quick=%s, port_hint=%s)",
      config_.modbus.auto_detect_quick ? "true" : "false", port_hint ? port_hint : "<any>");

    DeviceConfigPtr detected{nullptr};
    for (std::size_t attempt = 1; attempt <= kAutoDetectAttempts; ++attempt) {
      detected.reset(
        ::auto_detect_modbus_revo1(port_hint, config_.modbus.auto_detect_quick));
      if (detected && detected->port_name) {
        break;
      }
      if (attempt < kAutoDetectAttempts) {
        BRAINCO_HAND_LOG_WARN(
          "Revo1 auto-detection attempt %zu/%zu failed; retrying", attempt,
          kAutoDetectAttempts);
        std::this_thread::sleep_for(kAutoDetectRetryDelay);
      }
    }
    if (!detected || !detected->port_name) {
      BRAINCO_HAND_LOG_ERROR(
        "Revo1 auto-detection failed after %zu attempts; check power, wiring, serial "
        "permissions and USB connection",
        kAutoDetectAttempts);
      return false;
    }
    connection.port = detected->port_name;
    connection.baudrate = detected->baudrate;
    connection.slave_id = detected->slave_id;
    config_.slave_id = detected->slave_id;
  } else if (!std::filesystem::exists(connection.port)) {
    BRAINCO_HAND_LOG_ERROR("Serial port does not exist: %s", connection.port.c_str());
    return false;
  }

  DeviceHandler * raw_handle = ::modbus_open(connection.port.c_str(), connection.baudrate);
  if (!raw_handle) {
    BRAINCO_HAND_LOG_ERROR(
      "Cannot open %s at %u baud; check permissions and ensure no other process uses the port",
      connection.port.c_str(), connection.baudrate);
    return false;
  }

  handle_.reset(raw_handle);
  set_handler(raw_handle);
  resolved_connection_ = connection;
  return true;
}

void ModbusSession::close()
{
  handle_.reset();
  clear_handler();
  resolved_connection_.reset();
}

bool ModbusSession::is_open() const {return static_cast<bool>(handle_);}

std::optional<BraincoHandApi::ConnectionInfo> ModbusSession::connection_info() const
{
  return resolved_connection_;
}

}  // namespace brainco_hand_driver
