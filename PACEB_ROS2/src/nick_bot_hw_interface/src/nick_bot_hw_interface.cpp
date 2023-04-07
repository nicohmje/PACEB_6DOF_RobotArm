#include "nick_bot_hw_interface/nick_bot_hw_interface.hpp"

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

constexpr const char * kDynamixelHardware = "DynamixelHardware";
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
    const rclcpp_lifecycle::State & /* previous_state */ ) {
    RCLCPP_INFO(rclcpp::get_logger("NickBotHwInterface"), "Configuring...");

    

    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn NickBotHwInterface::on_init(
    const hardware_interface::HardwareInfo & info) {

    RCLCPP_INFO(rclcpp::get_logger("NickBotHwInterface"), "Init...");
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    joints_.resize(info_.joints.size(), Joint());
    joint_ids_.resize(info_.joints.size(), 0);
    joint_num_.resize(info_.joints.size(), 0);


    for (uint i = 0; i < info_.joints.size(); i++) {
        joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
        joint_num_[i] = std::stoi(info_.joints[i].parameters.at("joint_num"));
        joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
        joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
        joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
        joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
        RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "i %d, num %d, id %d", i, joint_num_[i], joint_ids_[i]);
    }

    param_.dynamixelport = info_.hardware_parameters.at("u2d2_port");
    param_.stepperport = info_.hardware_parameters.at("arduino_port");
    param_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    param_.timeout = std::stoi(info_.hardware_parameters["timeout"]);


    //Set up arduino commm
    arduino_.setup(param_.stepperport, param_.baud_rate, param_.timeout);
    RCLCPP_INFO(rclcpp::get_logger("NickBotHwInterface"), "Arduino set up at %s", param_.stepperport.c_str());

    const char * log = nullptr;

    //Set up U2D2
    dynamixel_workbench_.init(param_.dynamixelport.c_str(), param_.baud_rate, &log);
    RCLCPP_INFO(rclcpp::get_logger("NickBotHwInterface"), "%s", log);
    RCLCPP_INFO(rclcpp::get_logger("NickBotHwInterface"), "U2D2 set up at %s", param_.dynamixelport.c_str());

    for (uint i = 0; i < 3; ++i) {
      uint16_t model_number = 0;
      if (dynamixel_workbench_.ping(joint_ids_[i+3], &model_number, &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        }
      

      const ControlItem * goal_position =
      dynamixel_workbench_.getItemInfo(joint_ids_[i+3], kGoalPositionItem);
      if (goal_position == nullptr) {
        return CallbackReturn::ERROR;
      }

      const ControlItem * goal_velocity =
        dynamixel_workbench_.getItemInfo(joint_ids_[i+3], kGoalVelocityItem);
      if (goal_velocity == nullptr) {
        goal_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[i+3], kMovingSpeedItem);
      }
      if (goal_velocity == nullptr) {
        return CallbackReturn::ERROR;
      }

      const ControlItem * present_position =
        dynamixel_workbench_.getItemInfo(joint_ids_[i+3], kPresentPositionItem);
      if (present_position == nullptr) {
        return CallbackReturn::ERROR;
      }

      const ControlItem * present_velocity =
        dynamixel_workbench_.getItemInfo(joint_ids_[i+3], kPresentVelocityItem);
      if (present_velocity == nullptr) {
        present_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[i+3], kPresentSpeedItem);
      }
      if (present_velocity == nullptr) {
        return CallbackReturn::ERROR;
      }

      const ControlItem * present_current =
        dynamixel_workbench_.getItemInfo(joint_ids_[i+3], kPresentCurrentItem);
      if (present_current == nullptr) {
        present_current = dynamixel_workbench_.getItemInfo(joint_ids_[i+3], kPresentLoadItem);
      }
      if (present_current == nullptr) {
        return CallbackReturn::ERROR;
      }

      control_items_[kGoalPositionItem] = goal_position;
      control_items_[kGoalVelocityItem] = goal_velocity;
      control_items_[kPresentPositionItem] = present_position;
      control_items_[kPresentVelocityItem] = present_velocity;
      control_items_[kPresentCurrentItem] = present_current;
    }

    


    enable_torque(true);


    dynamixel_workbench_.addSyncWriteHandler(control_items_[kGoalPositionItem]->address, control_items_[kGoalPositionItem]->data_length);
    //RCLCPP_INFO(rclcpp::get_logger("NickBotHwInterface"), "%s", log);
    RCLCPP_INFO(rclcpp::get_logger("NickBotHwInterface"), "Added position SyncWrite.");

    dynamixel_workbench_.addSyncWriteHandler(control_items_[kGoalVelocityItem]->address, control_items_[kGoalVelocityItem]->data_length);
    //RCLCPP_INFO(rclcpp::get_logger("NickBotHwInterface"), "%s", log);
    RCLCPP_INFO(rclcpp::get_logger("NickBotHwInterface"), "Added velocity SyncWrite.");
    uint16_t start_address = std::min(
      control_items_[kPresentPositionItem]->address, control_items_[kPresentCurrentItem]->address);
    uint16_t read_length = control_items_[kPresentPositionItem]->data_length +
                         control_items_[kPresentVelocityItem]->data_length +
                         control_items_[kPresentCurrentItem]->data_length + 2;
    dynamixel_workbench_.addSyncReadHandler(start_address, read_length);
    RCLCPP_INFO(rclcpp::get_logger("NickBotHwInterface"), "Initiated addresses");

    RCLCPP_INFO(rclcpp::get_logger("NickBotHwInterface"), "Finished Dynamixel configuration");

    return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> NickBotHwInterface::export_state_interfaces()
{
    RCLCPP_DEBUG(rclcpp::get_logger("NickBotHwInterface"), "export_state_interfaces");
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {   
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity)); 
    }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> NickBotHwInterface::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger("NickBotHwInterface"), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
    //command_interfaces.emplace_back(hardware_interface::CommandInterface(
    //  info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
  }
  return command_interfaces;
}



hardware_interface::CallbackReturn NickBotHwInterface::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(rclcpp::get_logger("NickBotHwInterface"), "Activation started");

  RCLCPP_DEBUG(rclcpp::get_logger("NickBotHwInterface"), "start");
  read(rclcpp::Time{}, rclcpp::Duration(0, 0));
  reset_command();
  write(rclcpp::Time{}, rclcpp::Duration(0, 0));

  arduino_.sendEmptyMsg();

  RCLCPP_INFO(rclcpp::get_logger("NickBotHwInterface"), "Activation succesful");


  return hardware_interface::CallbackReturn::SUCCESS;

}

hardware_interface::CallbackReturn NickBotHwInterface::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger("NickBotHwInterface"), "stop");
  RCLCPP_INFO(rclcpp::get_logger("NickBotHwInterface"), "Deactivating ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("NickBotHwInterface"), "Deactivation succesful");
  return hardware_interface::CallbackReturn::SUCCESS;
}

return_type NickBotHwInterface::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
    if (!arduino_.connected())
        {
        RCLCPP_INFO(rclcpp::get_logger("NickBotHwInterface"), "Arduino es no connectado");
        return hardware_interface::return_type::ERROR;
        }

    std::vector<uint8_t> fake_ids(3, 0);
    std::vector<uint8_t> ids(3, 0);
    std::vector<int32_t> positions(3, 0);
    std::vector<int32_t> velocities(3, 0);
    std::vector<int32_t> currents(3, 0);


    for (uint i = 0; i < 3; i++) {
        fake_ids[i] = std::stoi(info_.joints[i+3].parameters.at("id"));
    }

    std::copy(fake_ids.begin(), fake_ids.end(), ids.begin());
    
    arduino_.getMotorPos(joints_[2].state.position, joints_[1].state.position, joints_[0].state.position, joints_[6].state.position);
    //arduino_.getMotorVel(joints_[2].state.position, joints_[1].state.position, joints_[0].state.position, joints_[2].state.velocity, joints_[1].state.velocity, joints_[0].state.velocity);

    const char * log = nullptr;

    if (!dynamixel_workbench_.syncRead(
            kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(), &log)) {
        //RCLCPP_ERROR(rclcpp::get_logger("NickBotHwInterface"), "john %s", log);
    }

    if (!dynamixel_workbench_.getSyncReadData(
            kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
            control_items_[kPresentCurrentItem]->address,
            control_items_[kPresentCurrentItem]->data_length, currents.data(), &log)) {
        //RCLCPP_ERROR(rclcpp::get_logger("NickBotHwInterface"), "cena%s", log);
    }

    if (!dynamixel_workbench_.getSyncReadData(
            kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
            control_items_[kPresentVelocityItem]->address,
            control_items_[kPresentVelocityItem]->data_length, velocities.data(), &log)) {
        //RCLCPP_ERROR(rclcpp::get_logger("NickBotHwInterface"), "juan%s", log);
    }

    if (!dynamixel_workbench_.getSyncReadData(
            kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
            control_items_[kPresentPositionItem]->address,
            control_items_[kPresentPositionItem]->data_length, positions.data(), &log)) {
        //RCLCPP_ERROR(rclcpp::get_logger("NickBotHwInterface"), "joooan%s", log);
    }

    joints_[3].state.position = (positions[0]-2048) * ((2*3.14159)/4096);
    joints_[4].state.position = (positions[1]-2048) * ((2*3.14159)/4096);
    joints_[5].state.position = (positions[2]) * ((2*3.14159)/4096);

    for (uint i = 3; i < info_.joints.size(); i++) {
        //RCLCPP_INFO(rclcpp::get_logger("NickBotHwInterface"), "%f", (positions[i]-2048) * ((2*3.14159)/4096));
        joints_[i].state.velocity = dynamixel_workbench_.convertValue2Velocity(ids[i-3], velocities[i-3]);
    }

    return return_type::OK;
}

return_type NickBotHwInterface::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  using std::chrono::high_resolution_clock;
  using std::chrono::duration_cast;
  using std::chrono::duration;
  using std::chrono::milliseconds;
  auto t1 = high_resolution_clock::now();


  if (!arduino_.connected())
  {
    RCLCPP_INFO(rclcpp::get_logger("NickBotHwInterface"), "Arduino es no connectado");
    return hardware_interface::return_type::ERROR;
  }


  std::vector<int32_t> commands(3, 0);
  std::vector<uint8_t> fake_ids(3, 0);
  std::vector<uint8_t> ids(3, 0);


    for (uint i = 0; i < 3; i++) {
        fake_ids[i] = std::stoi(info_.joints[i+3].parameters.at("id"));
    }

    std::copy(fake_ids.begin(), fake_ids.end(), ids.begin());

  if (joints_[1].command.position == joints_[1].command.position) {
      //RCLCPP_INFO(rclcpp::get_logger("NickBotHwInterface"), "i %i command %f",1, joints_[1].command.velocity);
      arduino_.setJointAngles(joints_[2].command.position, joints_[1].command.position, joints_[0].command.position, joints_[6].command.position); //,joints_[2].command.velocity, joints_[1].command.velocity, joints_[0].command.velocity);
  }

  auto t2 = high_resolution_clock::now();
  duration<double, std::milli> ms_double = t2 - t1;

  // Velocity control
  //set_control_mode(ControlMode::Velocity);

  //0.2288 rev/min 
  //rev = 2pi
  const char * log = nullptr;


  commands[0] = int((joints_[3].command.position + 3.14159) * (4096/(2*3.14159)));
  commands[1] = int((joints_[4].command.position + 3.14159) * (4096/(2*3.14159)));
  commands[2] = int(joints_[5].command.position * (4096/(2*3.14159)));


  if (!dynamixel_workbench_.syncWrite(
        kGoalPositionIndex, ids.data(), ids.size(), commands.data(), 1, &log)) {
    //RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
  }
  return return_type::OK;
}

return_type NickBotHwInterface::enable_torque(const bool enabled)
{
  const char * log = nullptr;

  if (enabled && !torque_enabled_) {
    for (uint i = 0; i < 3; ++i) {
      if (!dynamixel_workbench_.torqueOn(joint_ids_[i+3], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger("NickBotHwInterface"), "%s", log);
        return return_type::ERROR;
      }
    }
    reset_command();
    RCLCPP_INFO(rclcpp::get_logger("NickBotHwInterface"), "Torque enabled");
  } else if (!enabled && torque_enabled_) {
    for (uint i = 0; i < 3; ++i) {
      if (!dynamixel_workbench_.torqueOff(joint_ids_[i+3], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger("NickBotHwInterface"), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("NickBotHwInterface"), "Torque disabled");
  }

  torque_enabled_ = enabled;
  return return_type::OK;
}

return_type NickBotHwInterface::reset_command()
{
  for (uint i = 0; i < joints_.size(); i++) {
    joints_[i].command.position = joints_[i].state.position;
    }

  return return_type::OK;
}
} //namespace nbhw

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(nick_bot_hw_interface::NickBotHwInterface, hardware_interface::SystemInterface)
