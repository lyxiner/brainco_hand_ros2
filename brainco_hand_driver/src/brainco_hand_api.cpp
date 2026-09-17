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

#include "brainco_hand_driver/brainco_hand_api.hpp"

#include <memory>

#include "brainco_hand_driver/modbus_session.hpp"
#include "brainco_hand_driver/sdk_helpers.hpp"
#include "stark-sdk.h"  // NOLINT(build/include_subdir)

namespace brainco_hand_driver
{

auto brainco_log_level_to_string(BraincoLogLevel level) -> std::string
{
  switch (level) {
    case BraincoLogLevel::kError:
      return "error";
    case BraincoLogLevel::kWarn:
      return "warn";
    case BraincoLogLevel::kInfo:
      return "info";
    case BraincoLogLevel::kDebug:
      return "debug";
    case BraincoLogLevel::kTrace:
      return "trace";
  }
  return "info";
}

struct BraincoHandApi::Impl
{
  DriverConfig config{};
  std::unique_ptr<TransportSession> session;
};

BraincoHandApi::BraincoHandApi()
: impl_(std::make_unique<Impl>()) {configure(DriverConfig{});}

BraincoHandApi::BraincoHandApi(const DriverConfig & config)
: impl_(std::make_unique<Impl>())
{
  configure(config);
}

BraincoHandApi::~BraincoHandApi() = default;

auto BraincoHandApi::configure(const DriverConfig & config) -> void
{
  close();
  impl_->config = config;
  ::init_cfg(STARK_PROTOCOL_TYPE_MODBUS, to_sdk_log_level(config.log_level));
  impl_->session = std::make_unique<ModbusSession>(impl_->config);
}

auto BraincoHandApi::open() -> bool
{
  return impl_ && impl_->session && impl_->session->open();
}

auto BraincoHandApi::close() -> void
{
  if (impl_ && impl_->session) {
    impl_->session->close();
  }
}

auto BraincoHandApi::is_open() const -> bool
{
  return impl_ && impl_->session && impl_->session->is_open();
}

auto BraincoHandApi::fetch_device_info(uint8_t slave_id, DeviceInfoData & info) const -> bool
{
  return impl_ && impl_->session && impl_->session->fetch_device_info(slave_id, info);
}

auto BraincoHandApi::get_motor_status(uint8_t slave_id) const -> std::optional<MotorStatus>
{
  if (!impl_ || !impl_->session) {
    return std::nullopt;
  }
  return impl_->session->get_motor_status(slave_id);
}

auto BraincoHandApi::set_finger_positions(
  uint8_t slave_id, const uint16_t * positions, std::size_t count) -> bool
{
  return impl_ && impl_->session &&
         impl_->session->set_finger_positions(slave_id, positions, count);
}

auto BraincoHandApi::resolved_connection() const -> std::optional<ConnectionInfo>
{
  if (!impl_ || !impl_->session) {
    return std::nullopt;
  }
  return impl_->session->connection_info();
}

}  // namespace brainco_hand_driver
