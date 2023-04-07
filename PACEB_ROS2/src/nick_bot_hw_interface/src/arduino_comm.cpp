#include "nick_bot_hw_interface/arduino_comm.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>

// MESSAGE OBJECTS TO SEND:
// p : get motor angles 1, 2, 3
// j : set joint angles 1, 2, 3
// k : set pid values kp,ki,kd,ko

std::string ArduinoComm::sendMsg(const std::string &msg)
{   
    serial_conn_.write(msg);
    std::string response = serial_conn_.readline();
    return response;
}


void ArduinoComm::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{  
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    serial_conn_.open();
    // serial_conn_.(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms));

}

void ArduinoComm::sendEmptyMsg()
{
    std::string response = sendMsg("\r");
}



void ArduinoComm::getMotorPos(double &val_1, double &val_2, double &val_3,  double &val_4)
{
    std::string response = sendMsg("v\r");

    std::string delimiter = ":";
    size_t first_del = response.find(delimiter);
    std::string token_1 = response.substr(0, first_del);
    std::string token_2 = response.substr(first_del + delimiter.length());
    size_t second_del = response.find(delimiter, first_del + delimiter.length());
    std::string token_3 = response.substr(second_del + delimiter.length());
    size_t third_del = response.find(delimiter, second_del + delimiter.length());
    std::string token_4 = response.substr(third_del + delimiter.length());



    val_1 = std::atof(token_1.c_str());
    val_2 = -1* std::atof(token_2.c_str());
    val_3 = std::atof(token_3.c_str());
    val_4 = std::atof(token_4.c_str());
}

void ArduinoComm::getMotorVel(double &val_1, double &val_2, double &val_3, double &val_4, double &val_5, double &val_6)
{
    std::string response = sendMsg("v\r");

    std::string delimiter = ":";
    size_t first_del = response.find(delimiter);
    std::string token_1 = response.substr(0, first_del);
    std::string token_2 = response.substr(first_del + delimiter.length());
    size_t second_del = response.find(delimiter, first_del + delimiter.length());
    std::string token_3 = response.substr(second_del + delimiter.length());
    size_t third_del = response.find(delimiter, second_del + delimiter.length());
    std::string token_4 = response.substr(third_del + delimiter.length());
    size_t fourth_del = response.find(delimiter, third_del + delimiter.length());
    std::string token_5 = response.substr(fourth_del + delimiter.length());
    size_t fifth_del = response.find(delimiter, fourth_del + delimiter.length());
    std::string token_6 = response.substr(fifth_del + delimiter.length());


    val_1 = std::atof(token_1.c_str());
    val_2 = std::atof(token_2.c_str());
    val_3 = std::atof(token_3.c_str());
    val_4 = std::atof(token_4.c_str());
    val_5 = std::atof(token_5.c_str());
    val_6 = std::atof(token_6.c_str());
}

// void ArduinoComm::getMotorVel(double &val_1, double &val_2, double &val_3)
// {
//     std::string response = sendMsg("v\r");

//     std::string delimiter = ":";
//     size_t first_del = response.find(delimiter);
//     std::string token_1 = response.substr(0, first_del);
//     std::string token_2 = response.substr(first_del + delimiter.length());
//     size_t second_del = response.find(delimiter, first_del + delimiter.length());
//     std::string token_3 = response.substr(second_del + delimiter.length());


//     val_1 = std::atof(token_1.c_str());
//     val_2 = std::atof(token_2.c_str());
//     val_3 = std::atof(token_3.c_str());
// }

void ArduinoComm::setJointAngles(double joint_1, double joint_2, double joint_3, double gripper)//,double joint_1_vel, double joint_2_vel, double joint_3_vel)
{
    std::stringstream ss;
    ss << "j " << joint_1 << " " << -1*joint_2 << " " << joint_3 << " " << gripper << "\r";// << joint_1_vel << " " << joint_2_vel << " " << joint_3_vel << "\r";
    sendMsg(ss.str());

}

void ArduinoComm::setPidValues(float kp, float kd, float ki, float ko)
{
    std::stringstream ss;
    ss << "k " << kp << ":" << kd << ":" << ki << ":" << ko << "\r";
    sendMsg(ss.str());
}












