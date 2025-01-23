/*
 * @Author: Msnakes 1327153842@qq.com
 * @Date: 2025-01-22 15:28:04
 * @LastEditors: Msnakes 1327153842@qq.com
 * @Description: 适用于底盘速度控制手柄指令解析并发送
 * 
 */
#pragma once
#include "cmd_sender_base.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <robot_joy/VelConfig.h>
#include <dynamic_reconfigure/server.h>
namespace robot_joy_cmd_sender {
class CmdSenderMobile : public CmdSenderBase {
public:
  /**
   * @brief 发送速度
   * @param cmd 速度指令[vx, vy, vth], 单位m/s, rad/s, vy只有当odom_model_type_为omni时才有用
   */
  void SendCmd(std::vector<double> cmd);
  void StopCmd();
  bool Init(ros::NodeHandle &nh);
private:
  void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void ApplyLimit(geometry_msgs::Twist &target_vel, double gain);
  double SmoothControlWithSCurve(double current, double target,
                                         double dt);
  void DynamicCallback(robot_joy::VelConfig &config, uint32_t level);
private:
  bool smooth_control_;
  double smooth_transition_time_;
  double acc_lim_trans_;
  double acc_lim_th_;
  double max_vel_trans_;
  double max_vel_th_;
  double dec_lim_trans_;
  double dec_lim_th_;
  std::string odom_model_type_;
  bool odom_received_;
  ros::Publisher vel_pub_;
  ros::Subscriber odom_sub_;
  ros::NodeHandle nh_;
  geometry_msgs::Twist current_vel_;
  dynamic_reconfigure::Server<robot_joy::VelConfig> cfg_server_;
  ros::Time last_time_;
};
} // namespace robot_joy_cmd_sender
