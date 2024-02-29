// Copyright 2021 ros2_control Development Team
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

#ifndef NP_DIFFBOT__DIFFBOT_SYSTEM_HPP_
#define NP_DIFFBOT__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "visibility_control.h"

#include <Arduino/Arduino.h>
#include <Kangaroo/Kangaroo.h>

namespace np_diffbot
{
class DiffBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware);

  NP_DIFFBOT_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  NP_DIFFBOT_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  NP_DIFFBOT_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  NP_DIFFBOT_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  NP_DIFFBOT_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  NP_DIFFBOT_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  NP_DIFFBOT_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
 
  std::shared_ptr<serial::Serial> serial_port;
  std::shared_ptr<SerialStream> stream;
  std::shared_ptr<KangarooSerial> K;
  std::shared_ptr<KangarooChannel> K1;
  std::shared_ptr<KangarooChannel> K2;

  // serial::Serial* ptr_serial_port;
  // SerialStream* ptr_stream;
  // KangarooSerial* ptr_K;
  // KangarooChannel* ptr_K1;


  unsigned long enc_count_per_rev = 1200;
  std::chrono::time_point<std::chrono::system_clock> time_;
};

}  // namespace NP_DIFFBOT

#endif  // NP_DIFFBOT__DIFFBOT_SYSTEM_HPP_
