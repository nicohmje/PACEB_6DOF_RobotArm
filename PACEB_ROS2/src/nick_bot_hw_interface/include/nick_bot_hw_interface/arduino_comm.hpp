#ifndef STEPPER_ARDUINO_COMMUNICATIONS
#define STEPPER_ARDUINO_COMMUNICATIONS

#include <serial/serial.h>
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


class ArduinoComm
{


public:

    ArduinoComm()
    {}


    ArduinoComm(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
      : serial_conn_(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms))
    {  }

    void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
    void sendEmptyMsg();
    void getMotorPos(double &val_1, double &val_2, double &val_3, double &val_4);
    void getMotorVel(double &val_1, double &val_2, double &val_3,double &val_4, double &val_5, double &val_6);
    void setJointAngles(double joint_1, double joint_2, double joint_3, double gripper);//, double joint_1_vel, double joint_2_vel, double joint_3_vel);
    void setPidValues(float kp, float kd, float ki, float ko);

    bool connected() const { return serial_conn_.isOpen(); }

    std::string sendMsg(const std::string &msg_to_send);



private: 

  serial::Serial serial_conn_;  ///< Underlying serial connection 


};


#endif // STEPPER_ARDUINO_COMMUNICATIONS