// Copyright 2023 Nicolas Hammje
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

#include "nick_bot_hw_interface/nbhw.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <memory>

#include <typeinfo>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nick_bot_hw_interface
{
constexpr const char * kNickBotHwInterface = "NickBotHwInterface";
constexpr uint8_t kGoalPositionIndex = 0;
constexpr uint8_t kGoalVelocityIndex = 1;
constexpr uint8_t kPresentPositionVelocityCurrentIndex = 0;
constexpr const char * kGoalPositionItem = "Goal_Position";
constexpr const char * kGoalVelocityItem = "Goal_Velocity";
constexpr const char * kMovingSpeedItem = "Moving_Speed";
constexpr const char * kPresentPositionItem = "Present_Position";
constexpr const char * kPresentVelocityItem = "Present_Velocity";
constexpr const char * kPresentSpeedItem = "Present_Speed";
constexpr const char * kPresentCurrentItem = "Present_Current";
constexpr const char * kPresentLoadItem = "Present_Load";
constexpr const char * const kExtraJointParameters[] = {
  "Profile_Velocity",
  "Profile_Acceleration",
  "Position_P_Gain",
  "Position_I_Gain",
  "Position_D_Gain"
  "Velocity_P_Gain",
  "Velocity_I_Gain",
};

hardware_interface::CallbackReturn NickBotHwInterface::on_configure(
    const rclcpp_lifecycle::State & /* previous_state */ )
{
    RCLCPP_INFO(rclcpp::get_logger(kNickBotHwInterface), "Configuring...");

    param_.device = info_.hardware_parameters.at("device_port");

    param_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    param_.timeout = std::stoi(info_.hardware_parameters["timeout"]);

    RCLCPP_INFO(rclcpp::get_logger(kNickBotHwInterface), "Set device parameters... %s", param_.device.c_str());

    RCLCPP_INFO(rclcpp::get_logger(kNickBotHwInterface), "Configured joint info...");

    param_.s_p_a = {int(13200/(2*3.14159)), int(8000/(2*3.14159)), int(13200/(2*3.14159))};

    RCLCPP_INFO(rclcpp::get_logger(kNickBotHwInterface), "Configured steps per angle for 3 joints...");


    arduino_.setup(param_.device, param_.baud_rate, param_.timeout);

    RCLCPP_INFO(rclcpp::get_logger(kNickBotHwInterface), "Finished Configuration");

    return hardware_interface::CallbackReturn::SUCCESS;

}

hardware_interface::CallbackReturn NickBotHwInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kNickBotHwInterface), "configure");
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  steps_.resize(3);
  pos_.resize(3);
  hw_states_.resize(3, std::numeric_limits<double>::quiet_NaN());
  hw_states_vel_.resize(3, std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(3, std::numeric_limits<double>::quiet_NaN());

  joints_.resize(3, Joint());
  joint_ids_.resize(3, 0);

  for (uint i = 0; i < 3; i++) {
    joint_ids_[i] = std::stoi(info_.joints[i+3].parameters.at("id"));
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
    RCLCPP_INFO(rclcpp::get_logger(kNickBotHwInterface), "joint_id %d: %d", i, joint_ids_[i]);
  }


  auto usb_port = info_.hardware_parameters.at("usb_port");
  auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
  const char * log = nullptr;

  RCLCPP_INFO(rclcpp::get_logger(kNickBotHwInterface), "usb_port: %s", usb_port.c_str());
  RCLCPP_INFO(rclcpp::get_logger(kNickBotHwInterface), "baud_rate: %d", baud_rate);

  if (!dynamixel_workbench_.init(usb_port.c_str(), baud_rate, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kNickBotHwInterface), "%s", log);
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (uint i = 0; i < 3; ++i) {
    uint16_t model_number = 0;
    if (!dynamixel_workbench_.ping(joint_ids_[i], &model_number, &log)) {
      RCLCPP_FATAL(rclcpp::get_logger(kNickBotHwInterface), "%s", log);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  for (uint i = 3; i < 6; ++i) {
    for (auto paramName : kExtraJointParameters) {
      if (info_.joints[i].parameters.find(paramName) != info_.joints[i].parameters.end()) {
        auto value = std::stoi(info_.joints[i].parameters.at(paramName));
        if (!dynamixel_workbench_.itemWrite(joint_ids_[i-3], paramName, value, &log)) {
          RCLCPP_FATAL(rclcpp::get_logger(kNickBotHwInterface), "%s", log);
          return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger(kNickBotHwInterface), "%s set to %d for joint %d", paramName, value, i-3);
      }
    }
  }
  enable_torque(true);

  const ControlItem * goal_position =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalPositionItem);
  if (goal_position == nullptr) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const ControlItem * goal_velocity =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalVelocityItem);
  if (goal_velocity == nullptr) {
    goal_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[0], kMovingSpeedItem);
  }
  if (goal_velocity == nullptr) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const ControlItem * present_position =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentPositionItem);
  if (present_position == nullptr) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const ControlItem * present_velocity =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentVelocityItem);
  if (present_velocity == nullptr) {
    present_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentSpeedItem);
  }
  if (present_velocity == nullptr) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const ControlItem * present_current =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentCurrentItem);
  if (present_current == nullptr) {
    present_current = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentLoadItem);
  }
  if (present_current == nullptr) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  control_items_[kGoalPositionItem] = goal_position;
  control_items_[kGoalVelocityItem] = goal_velocity;
  control_items_[kPresentPositionItem] = present_position;
  control_items_[kPresentVelocityItem] = present_velocity;
  control_items_[kPresentCurrentItem] = present_current;

  if (!dynamixel_workbench_.addSyncWriteHandler(
        control_items_[kGoalPositionItem]->address, control_items_[kGoalPositionItem]->data_length,
        &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kNickBotHwInterface), "%s", log);
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!dynamixel_workbench_.addSyncWriteHandler(
        control_items_[kGoalVelocityItem]->address, control_items_[kGoalVelocityItem]->data_length,
        &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kNickBotHwInterface), "%s", log);
    return hardware_interface::CallbackReturn::ERROR;
  }

  uint16_t start_address = std::min(
    control_items_[kPresentPositionItem]->address, control_items_[kPresentCurrentItem]->address);
  uint16_t read_length = control_items_[kPresentPositionItem]->data_length +
                         control_items_[kPresentVelocityItem]->data_length +
                         control_items_[kPresentCurrentItem]->data_length + 2;
  if (!dynamixel_workbench_.addSyncReadHandler(start_address, read_length, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kNickBotHwInterface), "%s", log);
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> NickBotHwInterface::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kNickBotHwInterface), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < 3; i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_vel_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  for (uint i = 3; i < 6; i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i-3].state.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i-3].state.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i-3].state.effort));
  }
  

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> NickBotHwInterface::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kNickBotHwInterface), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < 3; i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }
  for (uint i = 3; i < 6; i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i-3].command.velocity));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn NickBotHwInterface::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kNickBotHwInterface), "start");
  for (uint i = 0; i < joints_.size(); i++) {
    if (use_dummy_ && std::isnan(joints_[i].state.position)) {
      joints_[i].state.position = 0.0;
      joints_[i].state.velocity = 0.0;
      joints_[i].state.effort = 0.0;
    }
  }
  read(rclcpp::Time{}, rclcpp::Duration(0, 0));
  reset_command();
  write(rclcpp::Time{}, rclcpp::Duration(0, 0));

  arduino_.sendEmptyMsg();

  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_commands_[i] = hw_states_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger(kNickBotHwInterface), "Activation succesful");


  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn NickBotHwInterface::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kNickBotHwInterface), "stop");
  RCLCPP_INFO(rclcpp::get_logger(kNickBotHwInterface), "Deactivating ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger(kNickBotHwInterface), "Deactivation succesful");
  return hardware_interface::CallbackReturn::SUCCESS;
}

double NickBotHwInterface::calcSteps(double angle, int number) //DONE
{
    //Converts angles into steps 
    //1 : base, 2:shoulder, 3:elbow
    int val = int(angle * param_.s_p_a[number]);
    if (angle != angle)
    { 
      val = 0;
    }
    return val;
}

double NickBotHwInterface::calcAngles(int steps, int number) //DONE
{
    //Converts steps into angles 
    //1: base, 2: shoulder, 3: elbow

    return int(steps / param_.s_p_a[number]);
}

return_type NickBotHwInterface::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    return return_type::OK;
  }

  if (!arduino_.connected())
    {
      RCLCPP_INFO(rclcpp::get_logger(kNickBotHwInterface), "Arduino es no connectado");
      return hardware_interface::return_type::ERROR;
    }

  std::vector<uint8_t> ids(3, 0);
  std::vector<int32_t> positions(3, 0);
  std::vector<int32_t> velocities(3, 0);
  std::vector<int32_t> currents(3, 0);

  arduino_.getMotorVel(hw_states_[1], hw_states_[2], hw_states_[0],hw_states_vel_[1], hw_states_vel_[2], hw_states_vel_[0]);


  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  const char * log = nullptr;

  if (!dynamixel_workbench_.syncRead(
        kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(), &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kNickBotHwInterface), "%s", log);
  }

  if (!dynamixel_workbench_.getSyncReadData(
        kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
        control_items_[kPresentCurrentItem]->address,
        control_items_[kPresentCurrentItem]->data_length, currents.data(), &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kNickBotHwInterface), "%s", log);
  }

  if (!dynamixel_workbench_.getSyncReadData(
        kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
        control_items_[kPresentVelocityItem]->address,
        control_items_[kPresentVelocityItem]->data_length, velocities.data(), &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kNickBotHwInterface), "%s", log);
  }

  if (!dynamixel_workbench_.getSyncReadData(
        kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
        control_items_[kPresentPositionItem]->address,
        control_items_[kPresentPositionItem]->data_length, positions.data(), &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kNickBotHwInterface), "%s", log);
  }

  joints_[0].state.position = (positions[0]-2048) * ((2*3.14159)/4096);
  joints_[1].state.position = (positions[1]-2048) * ((2*3.14159)/4096);
  joints_[2].state.position = (positions[2]) * ((2*3.14159)/4096);

  for (uint i = 0; i < ids.size(); i++) {
    //RCLCPP_INFO(rclcpp::get_logger(kNickBotHwInterface), "%f", (positions[i]-2048) * ((2*3.14159)/4096));
    joints_[i].state.velocity = dynamixel_workbench_.convertValue2Velocity(ids[i], velocities[i]);
    joints_[i].state.effort = dynamixel_workbench_.convertValue2Current(currents[i]);
  }

  return return_type::OK;
}

return_type NickBotHwInterface::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    for (auto & joint : joints_) {
      joint.state.position = joint.command.position;
    }

    return return_type::OK;
  }
  using std::chrono::high_resolution_clock;
  using std::chrono::duration_cast;
  using std::chrono::duration;
  using std::chrono::milliseconds;
  auto t1 = high_resolution_clock::now();


  if (!arduino_.connected())
  {
    RCLCPP_INFO(rclcpp::get_logger(kNickBotHwInterface), "Arduino es no connectado");
    return hardware_interface::return_type::ERROR;
  }

  std::vector<uint8_t> ids(3, 0);
  std::vector<int32_t> commands(3, 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  const char * log = nullptr;

  if (hw_commands_[1] == hw_commands_[1]) {
      arduino_.setJointAngles(hw_commands_[1], hw_commands_[2], hw_commands_[0]);
  }

  auto t2 = high_resolution_clock::now();
  duration<double, std::milli> ms_double = t2 - t1;

  // Velocity control
  //set_control_mode(ControlMode::Velocity);

  //0.2288 rev/min 
  //rev = 2pi


  for (uint i = 0; i < 3; i++) {
      commands[i] = ((joints_[i].command.velocity*60.) / 0.2288);
      //RCLCPP_INFO(rclcpp::get_logger(kNickBotHwInterface), "Joint %d command is %f", ids[i], ((joints_[i].command.velocity*60.) / 0.2288));
    }


  if (!dynamixel_workbench_.syncWrite(
        kGoalVelocityIndex, ids.data(), ids.size(), commands.data(), 1, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kNickBotHwInterface), "%s", log);
  }

  return return_type::OK;
}

return_type NickBotHwInterface::enable_torque(const bool enabled)
{
  const char * log = nullptr;

  if (enabled && !torque_enabled_) {
    for (uint i = 0; i < 3; ++i) {
      if (!dynamixel_workbench_.torqueOn(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kNickBotHwInterface), "%s", log);
        return return_type::ERROR;
      }
    }
    reset_command();
    RCLCPP_INFO(rclcpp::get_logger(kNickBotHwInterface), "Torque enabled");
  } else if (!enabled && torque_enabled_) {
    for (uint i = 0; i < 3; ++i) {
      if (!dynamixel_workbench_.torqueOff(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kNickBotHwInterface), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kNickBotHwInterface), "Torque disabled");
  }

  torque_enabled_ = enabled;
  return return_type::OK;
}

return_type NickBotHwInterface::set_control_mode(const ControlMode & mode, const bool force_set)
{
  const char * log = nullptr;

  if (mode == ControlMode::Velocity && (force_set || control_mode_ != ControlMode::Velocity)) {
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
      enable_torque(false);
    }

    for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (!dynamixel_workbench_.setVelocityControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kNickBotHwInterface), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kNickBotHwInterface), "Velocity control");
    control_mode_ = ControlMode::Velocity;

    if (torque_enabled) {
      enable_torque(true);
    }
  } 
  else if (
    mode == ControlMode::Position && (force_set || control_mode_ != ControlMode::Position)) {
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
      enable_torque(false);
    }

    for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (!dynamixel_workbench_.setPositionControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kNickBotHwInterface), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kNickBotHwInterface), "Position control");
    control_mode_ = ControlMode::Position;

    if (torque_enabled) {
      enable_torque(true);
    }
  } else if (control_mode_ != ControlMode::Velocity && control_mode_ != ControlMode::Position) {
    RCLCPP_FATAL(
      rclcpp::get_logger(kNickBotHwInterface), "Only position/velocity control are implemented");
    return return_type::ERROR;
  }

  return return_type::OK;
}

return_type NickBotHwInterface::reset_command()
{
  for (uint i = 0; i < joints_.size(); i++) {
    joints_[i].command.position = joints_[i].state.position;
    joints_[i].command.velocity = 0.0;
    joints_[i].command.effort = 0.0;
  }

  return return_type::OK;
}

}  // namespace nick_bot_hw_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(nick_bot_hw_interface::NickBotHwInterface, hardware_interface::SystemInterface)
