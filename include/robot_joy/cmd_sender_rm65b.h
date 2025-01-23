
#pragma once
#include "cmd_sender_base.h"
#include <dual_arm_msgs/Joint_Teach.h>
#include <dual_arm_msgs/Ort_Teach.h>
#include <dual_arm_msgs/Pos_Teach.h>
#include <dual_arm_msgs/Stop_Teach.h>
namespace robot_joy_cmd_sender {
class CmdSenderRM65B : public CmdSenderBase {
public:
   /**
   * @brief 发送示教指令
   * @param cmd 示教指令, cmd[0]:機械臂：0右臂，1左臂, cmd[1]:示教模式：0-停止, 1-关节示教，2-位置示教，3-姿态示教
   *            关节示教：cmd[2]:关节号(1-7)，cmd[3]:示教方向，1正轉，-1反轉, 0-停止 
   *            位置示教：cmd[2]:移動軸(1-x,2-y,3-z)，cmd[3]:示教方向，1正轉，-1反轉
   *            姿态示教：cmd[2]:旋轉軸(1-x,2-y,3-z)，cmd[3]:示教方向，1正轉，-1反轉
   */
  void SendCmd(std::vector<double> cmd);
  void StopCmd();
  bool Init(ros::NodeHandle &nh);
private:
  void JointTeach(ros::Publisher &pub, std::vector<double> &cmd);
  void PoseTeach(ros::Publisher &pub, std::vector<double> &cmd);
  void OrtTeach(ros::Publisher &pub, std::vector<double> &cmd);
private:
  ros::Publisher r_arm_pos_teach_pub_;
  ros::Publisher r_arm_rot_teach_pub_;
  ros::Publisher r_arm_joint_teach_pub_;
  ros::Publisher r_arm_stop_teach_pub_;
  ros::Publisher l_arm_pos_teach_pub_;
  ros::Publisher l_arm_rot_teach_pub_;
  ros::Publisher l_arm_joint_teach_pub_;
  ros::Publisher l_arm_stop_teach_pub_;
  dual_arm_msgs::Joint_Teach arm_joint_tech_msg_;
  dual_arm_msgs::Pos_Teach arm_pos_tech_msg_;
  dual_arm_msgs::Ort_Teach arm_ort_tech_msg_;
  dual_arm_msgs::Stop_Teach arm_stop_tech_msg_;
  ros::NodeHandle nh_;
};
} // namespace robot_joy_cmd_sender