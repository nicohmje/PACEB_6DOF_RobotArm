// Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
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

#ifndef NICK_BOT_HW_INTERFACE_HPP
#define NICK_BOT_HW_INTERFACE_HPP

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <memory>
#include <string>
#include <vector>
#include <map>

#include "arduino_comm.hpp"
#include <serial/serial.h>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "nick_bot_hw_interface/visibility_control.h"
#include <rclcpp_lifecycle/state.hpp>

#include <map>

#include "nick_bot_hw_interface/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;


namespace nick_bot_hw_interface
{
struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct Joint
{
  JointValue state{};
  JointValue command{};
};

struct Parameters
{
  std::string device = "/dev/ttyACM0"; //sometimes ttyUSB1
  int baud_rate = 1000000; //1 MHz
  int timeout = 1000; //ms
  std::vector<double> s_p_a;

};


enum class ControlMode {
  Position,
  Velocity,
  Torque,
  Currrent,
  ExtendedPosition,
  MultiTurn,
  CurrentBasedPosition,
  PWM,
};

class NickBotHwInterface
: public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(NickBotHwInterface)

  NICK_BOT_HW_INTERFACE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  NICK_BOT_HW_INTERFACE_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;


  NICK_BOT_HW_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  NICK_BOT_HW_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  NICK_BOT_HW_INTERFACE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  NICK_BOT_HW_INTERFACE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  NICK_BOT_HW_INTERFACE_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  NICK_BOT_HW_INTERFACE_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  NICK_BOT_HW_INTERFACE_PUBLIC
  double calcSteps(double , int );

  NICK_BOT_HW_INTERFACE_PUBLIC
  double calcAngles(int , int );


private:
  return_type enable_torque(const bool enabled);

  return_type set_control_mode(const ControlMode & mode, const bool force_set = false);

  return_type reset_command();

  Parameters param_; 
  ArduinoComm arduino_; 
  double hw_start_sec_;

  std::vector<double> hw_commands_;
  std::vector<double> hw_states_; 
  std::vector<double> hw_states_vel_;


  std::vector<int> steps_;
  std::vector<double> angles_;
  std::vector<double> pos_;

  DynamixelWorkbench dynamixel_workbench_;
  std::map<const char * const, const ControlItem *> control_items_;
  std::vector<Joint> joints_;
  std::vector<uint8_t> joint_ids_;
  bool torque_enabled_{false};
  ControlMode control_mode_{ControlMode::Position};
  bool use_dummy_{false};
};
}  // namespace dynamixel_hardware

#endif  // NICK_BOT_HW__NICK_BOT_HW_HPP_
