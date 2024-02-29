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

#include <np_diffbot/np_diffbot_system.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace np_diffbot
{
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  // hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  // hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"),
      "Hardware interface initiated. Joint '%s' have '%s' as first state interface and '%s' as first command interface. ",
      info_.joints[0].name.c_str(), info_.joints[0].state_interfaces[0].name.c_str(),
      info_.joints[0].command_interfaces[0].name.c_str()
    );

    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"),
      "Hardware interface initiated. Joint '%s' have '%s' as first state interface and '%s' as first command interface. ",
      info_.joints[1].name.c_str(), info_.joints[1].state_interfaces[0].name.c_str(),
      info_.joints[1].command_interfaces[0].name.c_str()
    );

  std::string port = info_.hardware_parameters["port"]; 
  unsigned long baud = std::stol(info_.hardware_parameters["baudrate"]);
  unsigned long timeout = std::stol(info_.hardware_parameters["timeout"]);
  enc_count_per_rev = std::stol(info_.hardware_parameters["enc_count_per_rev"]);
  time_ = std::chrono::system_clock::now();

  serial_port = std::make_shared<serial::Serial>(port, baud, serial::Timeout::simpleTimeout(timeout));
  stream = std::make_shared<SerialStream>(*serial_port);
 
  RCLCPP_INFO(
    rclcpp::get_logger("DiffBotSystemHardware"),
    "Serial port is opened :'%d, %zd'",
    serial_port->isOpen(),
    serial_port->available()
  );

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Activating ...please wait...");

  K = std::make_shared<KangarooSerial>(*stream);
  
  K1 = std::make_shared<KangarooChannel>(*K, '1');//left
  K1->commandTimeout(500);
  K1->commandRetryInterval(50);
  KangarooError err = K1->start();
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), 
    "...........K1->start():%d",
    err
    );

  if(err!=KANGAROO_NO_ERROR)
    return hardware_interface::CallbackReturn::ERROR;

  K2 = std::make_shared<KangarooChannel>(*K, '2');//right
  K2->commandTimeout(500);
  K2->commandRetryInterval(50);
  // K2->start();
  err = K2->start();
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), 
    "...........K2->start():%d",
    err
    );

  RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"), "K1 & K2 started............");

  // hardware_interface::CallbackReturn::FAILURE;
  // for (auto i = 0; i < hw_start_sec_; i++)
  // {
  //   rclcpp::sleep_for(std::chrono::seconds(1));
  //   RCLCPP_INFO(
  //     rclcpp::get_logger("DiffBotSystemHardware"), "%.1f seconds left...", hw_start_sec_ - i);
  // }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

// Independent mode channels on Kangaroo are, by default, '1' and '2'.
 
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Deactivating ...please wait...");
  K1->powerDownAll();
  rclcpp::sleep_for(std::chrono::seconds(3));
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
   //read time delta
  // auto new_time = std::chrono::system_clock::now();
  // std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = period.seconds();//diff.count();
  // time_ = new_time;

  //read encoder readings from kangaroo driver
  double left_wheel_count = (double) K1->getp().value();
  double right_wheel_count = (double) K2->getP().value();
  double pre_pos = hw_positions_[0];
  hw_positions_[0]= left_wheel_count*(2*M_PI)/enc_count_per_rev;
  hw_velocities_[0] = (hw_positions_[0] - pre_pos) / deltaSeconds;
  pre_pos = hw_positions_[1];
  hw_positions_[1]= right_wheel_count*(2*M_PI)/enc_count_per_rev;
  hw_velocities_[1] = (hw_positions_[1] - pre_pos) / deltaSeconds;

  RCLCPP_INFO(rclcpp::get_logger(
    "DiffBotSystemHardware"), 
    "Left_WHEEL_POS: %0.5f, RIGHT_WHEEL_POS: %0.5f",
    left_wheel_count,right_wheel_count
    );

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type np_diffbot::DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Writing...");

  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"), "Got command %.5f for '%s'!", hw_commands_[i],
      info_.joints[i].name.c_str());
    
    hw_velocities_[i] = hw_commands_[i]*enc_count_per_rev/(2*M_PI);

    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"), "Send velocity %.5f for '%s'!", 
      hw_velocities_[i],
      info_.joints[i].name.c_str());

  }
  K1->s((int)hw_velocities_[0]);
  K2->s((int)hw_velocities_[1]);

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Joints successfully written!");

  return hardware_interface::return_type::OK;
}

}  // namespace np_diffbot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  np_diffbot::DiffBotSystemHardware, hardware_interface::SystemInterface)
