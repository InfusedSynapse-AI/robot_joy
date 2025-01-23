/*
 * @Author: Msnakes 1327153842@qq.com
 * @Date: 2024-12-24 10:57:04
 * @LastEditors: Msnakes 1327153842@qq.com
 * @Description: ros手柄控制(速度、睿尔曼手柄示教控制)
 *
 */
#pragma once
#include <iostream>
#include <ros/ros.h>
#include <robot_joy/cmd_sender_factory.h>
#include <sensor_msgs/Joy.h>
#include <memory>
namespace robot_joy {
enum ControlType {
  base_vel,
  r_Joint,
  r_Pos,
  r_Ort,
  l_Joint,
  l_Pos,
  l_Ort,
  num_end
};
class TeleopJoy {
public:
  TeleopJoy(ros::NodeHandle &nh);
  ~TeleopJoy() {
    cmd_sender_->StopCmd();
  }
  void run();
private:
  void JoyCallBack(const sensor_msgs::Joy::ConstPtr &joy);
  void ChangeSender(int step);
private:
  ros::NodeHandle nh_;
  bool joy_wakeup_flag_;
  std::shared_ptr<robot_joy_cmd_factory::CmdSenderFactory> cmd_sender_factory_;
  std::shared_ptr<robot_joy_cmd_sender::CmdSenderBase> cmd_sender_;
  int control_type_;
  ros::Subscriber joy_sub_;
  ros::Time last_time_;
};
}