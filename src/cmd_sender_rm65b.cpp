#include <robot_joy/cmd_sender_rm65b.h>
#include <ros/ros.h>
namespace robot_joy_cmd_sender {
bool CmdSenderRM65B::Init(ros::NodeHandle &nh) {
  nh_ = nh;
  ros::NodeHandle pnh("~");

  r_arm_pos_teach_pub_ = pnh.advertise<dual_arm_msgs::Pos_Teach>(
      "/r_arm/rm_driver/Arm_PosTeach", 1, true);
  r_arm_rot_teach_pub_ = pnh.advertise<dual_arm_msgs::Ort_Teach>(
      "/r_arm/rm_driver/Arm_OrtTeach", 1, true);
  r_arm_joint_teach_pub_ = pnh.advertise<dual_arm_msgs::Joint_Teach>(
      "/r_arm/rm_driver/Arm_JointTeach", 1, true);
  r_arm_stop_teach_pub_ = pnh.advertise<dual_arm_msgs::Stop_Teach>(
      "/r_arm/rm_driver/Arm_StopTeach", 1, true);
  l_arm_pos_teach_pub_ = pnh.advertise<dual_arm_msgs::Pos_Teach>(
      "/l_arm/rm_driver/Arm_PosTeach", 1, true);
  l_arm_rot_teach_pub_ = pnh.advertise<dual_arm_msgs::Ort_Teach>(
      "/l_arm/rm_driver/Arm_OrtTeach", 1, true);
  l_arm_joint_teach_pub_ = pnh.advertise<dual_arm_msgs::Joint_Teach>(
      "/l_arm/rm_driver/Arm_JointTeach", 1, true);
  l_arm_stop_teach_pub_ = pnh.advertise<dual_arm_msgs::Stop_Teach>(
      "/l_arm/rm_driver/Arm_StopTeach", 1, true);
  arm_joint_tech_msg_.v = 0.1;
  arm_pos_tech_msg_.v = 0.1;
  arm_ort_tech_msg_.v = 0.1;
  return true;
}
void CmdSenderRM65B::JointTeach(ros::Publisher &pub, std::vector<double> &cmd) {
  if (cmd[1] > 0.5) {
    arm_joint_tech_msg_.direction == "pos";
    arm_joint_tech_msg_.teach_joint == int(cmd[0]);
  } else if (cmd[1] < -0.5) {
    arm_joint_tech_msg_.direction == "neg";
    arm_joint_tech_msg_.teach_joint == int(cmd[0]);
  } else {
    StopCmd();
    return;
  }
  pub.publish(arm_joint_tech_msg_);
}
void CmdSenderRM65B::PoseTeach(ros::Publisher &pub, std::vector<double> &cmd) {
  if (cmd[0] == 1.0)
    arm_pos_tech_msg_.teach_type == "x";
  else if (cmd[0] == 2.0)
    arm_pos_tech_msg_.teach_type == "y";
  else if (cmd[0] == 3.0)
    arm_pos_tech_msg_.teach_type == "z";
  else {
    StopCmd();
    return;
  }
  if (cmd[1] == 1.0)
    arm_pos_tech_msg_.direction == "pos";
  else if (cmd[1] == -1.0)
    arm_pos_tech_msg_.direction == "neg";
  else {
    StopCmd();
    return;
  }
  pub.publish(arm_pos_tech_msg_);
}
void CmdSenderRM65B::OrtTeach(ros::Publisher &pub, std::vector<double> &cmd) {
  if (cmd[0] == 1.0)
    arm_ort_tech_msg_.teach_type == "rx";
  else if (cmd[0] == 2.0)
    arm_ort_tech_msg_.teach_type == "ry";
  else if (cmd[0] == 3.0)
    arm_ort_tech_msg_.teach_type == "rz";
  else {
    StopCmd();
    return;
  }
  if (cmd[1] == 1.0)
    arm_pos_tech_msg_.direction == "pos";
  else if (cmd[1] == -1.0)
    arm_pos_tech_msg_.direction == "neg";
  else {
    StopCmd();
    return;
  }
  pub.publish(arm_ort_tech_msg_);
}
void CmdSenderRM65B::StopCmd() {
  r_arm_stop_teach_pub_.publish(arm_stop_tech_msg_);
  l_arm_stop_teach_pub_.publish(arm_stop_tech_msg_);
}
void CmdSenderRM65B::SendCmd(std::vector<double> cmd) {
  int arm = cmd[0];
  int mode = cmd[1];
  std::vector<double> tech_cmd;
  for (size_t i = 2; i < cmd.size(); i++) {
    tech_cmd.push_back(cmd[i]);
  }
  if (arm == 0) {
    switch (mode) {
    case 0:
      StopCmd();
      break;
    case 1:
      JointTeach(r_arm_joint_teach_pub_, tech_cmd);
      break;
    case 2:
      PoseTeach(r_arm_pos_teach_pub_, tech_cmd);
      break;
    case 3:
      OrtTeach(r_arm_rot_teach_pub_, tech_cmd);
      break;
    }
  } else if (arm == 1) {
    switch (mode) {
    case 0:
      StopCmd();
      break;
    case 1:
      JointTeach(l_arm_joint_teach_pub_, tech_cmd);
      break;
    case 2:
      PoseTeach(l_arm_pos_teach_pub_, tech_cmd);
      break;
    case 3:
      OrtTeach(l_arm_rot_teach_pub_, tech_cmd);
      break;
    }
  }
}
} // namespace robot_joy_cmd_sender
