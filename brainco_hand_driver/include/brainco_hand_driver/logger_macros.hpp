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

#include "rclcpp/rclcpp.hpp"

namespace brainco_hand_driver
{

inline auto logger() -> rclcpp::Logger
{
  return rclcpp::get_logger("brainco_hand_driver");
}

}  // namespace brainco_hand_driver

#define BRAINCO_HAND_LOG_DEBUG(...) \
  RCLCPP_DEBUG(::brainco_hand_driver::logger(), __VA_ARGS__)
#define BRAINCO_HAND_LOG_INFO(...) RCLCPP_INFO(::brainco_hand_driver::logger(), __VA_ARGS__)
#define BRAINCO_HAND_LOG_WARN(...) RCLCPP_WARN(::brainco_hand_driver::logger(), __VA_ARGS__)
#define BRAINCO_HAND_LOG_ERROR(...) RCLCPP_ERROR(::brainco_hand_driver::logger(), __VA_ARGS__)
