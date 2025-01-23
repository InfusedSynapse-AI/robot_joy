/*
 * @Author: Msnakes 1327153842@qq.com
 * @Date: 2025-01-22 15:32:51
 * @LastEditors: Msnakes 1327153842@qq.com
 * @Description: 手柄指令解析发送基类
 * 
 */
#pragma once
#include <ros/ros.h>
#include <vector>
namespace robot_joy_cmd_sender {
//工厂模式 基类
class CmdSenderBase {
public:
  CmdSenderBase() {}
  virtual ~CmdSenderBase() {}

  virtual void SendCmd(std::vector<double> cmd) = 0;
  virtual void StopCmd() = 0;
  virtual bool Init(ros::NodeHandle &nh) = 0;
};
} // namespace robot_joy_cmd_sender
