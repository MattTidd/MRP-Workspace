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

#include "MRP_Hardware_Interface/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>
#include <rcutils/logging.h>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace MRP_Hardware_Interface
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

  // assign parameters to the config structure:

  cfg_.front_left_wheel_name = info_.hardware_parameters["front_left_wheel_name"];
  cfg_.front_right_wheel_name = info_.hardware_parameters["front_right_wheel_name"];
  cfg_.rear_left_wheel_name = info_.hardware_parameters["rear_left_wheel_name"];
  cfg_.rear_right_wheel_name = info_.hardware_parameters["rear_right_wheel_name"];

  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  // set up the wheels:
  wheel_f_l_.setup(cfg_.front_left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_f_r_.setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_l_.setup(cfg_.rear_left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_r_.setup(cfg_.rear_right_wheel_name, cfg_.enc_counts_per_rev);

  // PID settings:
  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "PID values not supplied, using defaults.");
  }

//----------------------  INTERFACE CHECKING  ----------------------//
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // check that there is exactly one command interface:
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // check that the command interface is indeed a velocity interface:
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // check that there are exactly two state interfaces:
    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // check that there is a position state interface:
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // check that there is a velocity state interface:
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // return successful if checks passed
  return hardware_interface::CallbackReturn::SUCCESS;
}

//---------------------- EXPORT STATE INTERFACES ----------------------//
std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  // declare state interface vector:
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // add front left wheel state interfaces:
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_f_l_.name, hardware_interface::HW_IF_POSITION, &wheel_f_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_f_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_f_l_.vel));

  // add front right wheel state interfaces:
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_f_r_.name, hardware_interface::HW_IF_POSITION, &wheel_f_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_f_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_f_r_.vel));

  // add rear left wheel state interfaces:
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_l_.name, hardware_interface::HW_IF_POSITION, &wheel_r_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_l_.vel));

  // add rear right wheel state interfaces:
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_r_.vel));

  return state_interfaces;
}

//---------------------- EXPORT COMMAND INTERFACES ----------------------//
std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  // declare command interface vector:
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // add front left wheel command interface:
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_f_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_f_l_.cmd));

  // add front right wheel command interface:
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_f_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_f_r_.cmd));

  // add rear left wheel command interface:
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_l_.cmd));

  // add rear right wheel command interface:
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_r_.cmd));

  return command_interfaces;
}

//---------------------- CONFIGURE ----------------------//
hardware_interface::CallbackReturn DiffBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Configuring, please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

//---------------------- CLEANUP ----------------------//
hardware_interface::CallbackReturn DiffBotSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up, please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

//---------------------- ACTIVATION ----------------------//
hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Activating, please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  if (cfg_.pid_p > 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Setting PID values, please wait...");
    comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully set PID values!");
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

//---------------------- DEACTIVATION ----------------------//
hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Deactivating, please wait...");
  // comms_.disconnect();
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

//---------------------- READ FUNCTION ----------------------//
hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  double delta_seconds = period.seconds();
  comms_.read_encoder_values(wheel_f_l_.enc, wheel_f_r_.enc, wheel_r_l_.enc, wheel_r_r_.enc);

  // printf("\rEncoder read: FL: %d, FR: %d, RL: %d, RR: %d   ", 
  //   wheel_f_l_.enc, wheel_f_r_.enc, wheel_r_l_.enc, wheel_r_r_.enc);
  // fflush(stdout);

  // RCUTILS_LOG_INFO_NAMED(
  //   "DiffBotSystemHardware",
  //   "Encoder read: FL: %d, FR: %d, RL: %d, RR: %d \r",
  //   wheel_f_l_.enc,
  //   wheel_f_r_.enc,
  //   wheel_r_l_.enc,
  //   wheel_r_r_.enc
  // );

  RCLCPP_INFO(
    rclcpp::get_logger("DiffBotSystemHardware"),
    "Encoder read: FL: %d, FR: %d, RL: %d, RR: %d",
    wheel_f_l_.enc,
    wheel_f_r_.enc,
    wheel_r_l_.enc,
    wheel_r_r_.enc
  );

  // get the position & velocity values for the front left wheel:
  double pos_prev = wheel_f_l_.pos;
  wheel_f_l_.pos = wheel_f_l_.calc_enc_angle();
  wheel_f_l_.vel = (wheel_f_l_.pos - pos_prev) / delta_seconds;

  // get the position & velocity values for the front right wheel:
  pos_prev = wheel_f_r_.pos;
  wheel_f_r_.pos = wheel_f_r_.calc_enc_angle();
  wheel_f_r_.vel = (wheel_f_r_.pos - pos_prev) / delta_seconds;

  // get the position & velocity values for the rear left wheel:
  pos_prev = wheel_r_l_.pos;
  wheel_r_l_.pos = wheel_r_l_.calc_enc_angle();
  wheel_r_l_.vel = (wheel_r_l_.pos - pos_prev) / delta_seconds;

  // get the position & velocity values for the rear right wheel:
  pos_prev = wheel_r_r_.pos;
  wheel_r_r_.pos = wheel_r_r_.calc_enc_angle();
  wheel_r_r_.vel = (wheel_r_r_.pos - pos_prev) / delta_seconds;

  return hardware_interface::return_type::OK;
}

//---------------------- WRITE FUNCTION ----------------------//
hardware_interface::return_type MRP_Hardware_Interface ::DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  // counts per loop for each motor:
  int motor_f_l_counts_per_loop = wheel_f_l_.cmd / wheel_f_l_.rads_per_count / cfg_.loop_rate;
  int motor_f_r_counts_per_loop = wheel_f_r_.cmd / wheel_f_r_.rads_per_count / cfg_.loop_rate;
  int motor_r_l_counts_per_loop = wheel_r_l_.cmd / wheel_r_l_.rads_per_count / cfg_.loop_rate;
  int motor_r_r_counts_per_loop = wheel_r_r_.cmd / wheel_r_r_.rads_per_count / cfg_.loop_rate;

  // RCLCPP_INFO(
  //   rclcpp::get_logger("DiffBotSystemHardware"),
  //   "cmds are: FL: %f, FR: %f, RL: %f, RR: %f",
  //   wheel_f_l_.cmd,
  //   wheel_f_r_.cmd,
  //   wheel_r_l_.cmd,
  //   wheel_r_r_.cmd
  // );

  // RCUTILS_LOG_INFO_NAMED(
  //   "DiffBotSystemHardware",
  //   "Writing: FL: %d, FR: %d, RL: %d, RR: %d \r",
  //   motor_f_l_counts_per_loop,
  //   motor_f_r_counts_per_loop,
  //   motor_r_l_counts_per_loop,
  //   motor_r_r_counts_per_loop
  // );

  // tell arduino to set the values:
  comms_.set_motor_values(
    motor_f_l_counts_per_loop,
    motor_f_r_counts_per_loop,
    motor_r_l_counts_per_loop,
    motor_r_r_counts_per_loop
  );

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_2

// export to the plugin:

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  MRP_Hardware_Interface::DiffBotSystemHardware, hardware_interface::SystemInterface)