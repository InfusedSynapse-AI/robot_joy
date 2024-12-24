#include <robot_joy/robot_joy.h>

using namespace std;

TeleopJoy::TeleopJoy()
    : n("~"), joy_wakeup_flag_(false), joy_zero_flag_(false),
      control_type_(base_vel), last_button_(0), last_axis_(0), ctrl_joint_(1),
      arm_stop_flag_(false) {
  n.param<int>("axis_linear_x", i_velLinear_x_,
               1); // x方向控制键（前进后退）axis
  n.param<int>("axis_linear_y", i_velLinear_y_,
               0); // y方向控制键（左移右移）axis
  n.param<int>("axis_angular", i_velAngular_, 3); // yaw角控制键（左转右转）axis

  n.param<int>("axis_accelerator", i_accelerator_, 5);   // 加速键 axis
  n.param<int>("axis_deceleration", i_deceleration_, 2); // 减速键 axis

  n.param<double>("linear_vel", f_velLinear_, 0.3);    // 初始最大线速度
  n.param<double>("angular_vel", f_velAngular_, 0.6); // 初始最大角速度

  vel_pub_ = nh.advertise<geometry_msgs::Twist>("joy_vel", 1);
  r_arm_pos_teach_pub_ = nh.advertise<dual_arm_msgs::Pos_Teach>(
      "/r_arm/rm_driver/Arm_PosTeach", 1);
  r_arm_rot_teach_pub_ = nh.advertise<dual_arm_msgs::Ort_Teach>(
      "/r_arm/rm_driver/Arm_OrtTeach", 1);
  r_arm_joint_teach_pub_ = nh.advertise<dual_arm_msgs::Joint_Teach>(
      "/r_arm/rm_driver/Arm_JointTeach", 1);
  r_arm_stop_teach_pub_ = nh.advertise<dual_arm_msgs::Stop_Teach>(
      "/r_arm/rm_driver/Arm_StopTeach", 1);
  l_arm_pos_teach_pub_ = nh.advertise<dual_arm_msgs::Pos_Teach>(
      "/l_arm/rm_driver/Arm_PosTeach", 1);
  l_arm_rot_teach_pub_ = nh.advertise<dual_arm_msgs::Ort_Teach>(
      "/l_arm/rm_driver/Arm_OrtTeach", 1);
  l_arm_joint_teach_pub_ = nh.advertise<dual_arm_msgs::Joint_Teach>(
      "/l_arm/rm_driver/Arm_JointTeach", 1);
  l_arm_stop_teach_pub_ = nh.advertise<dual_arm_msgs::Stop_Teach>(
      "/l_arm/rm_driver/Arm_StopTeach", 1);
  sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopJoy::callBack, this);
  arm_joint_tech_msg_.v = 0.1;
  arm_pos_tech_msg_.v = 0.1;
  arm_ort_tech_msg_.v = 0.1;
}

void TeleopJoy::callBack(const sensor_msgs::Joy::ConstPtr &joy) {
  last_time_ = ros::Time::now();
  joy_vel_ = geometry_msgs::Twist();
  if (joy->axes[6] == -1) {
    control_type_++;
    if (control_type_ >= ControlType::num_end)
      control_type_ = ControlType::base_vel;
    return;
  } else if (joy->axes[6] == 1) {
    control_type_--;
    if (control_type_ < ControlType::base_vel)
      control_type_ = ControlType::num_end - 1;
    return;
  }
  std::vector<float> cmd(7, 0.0);
  switch (control_type_) {
  case ControlType::r_Joint:
    cmd[0] = joy->axes[0];
    cmd[1] = joy->axes[1];
    cmd[2] = joy->axes[3];
    cmd[3] = joy->axes[4];

    if (joy->axes[2] == -1)
      cmd[4] = 1.0;
    else if (joy->axes[5] == -1)
      cmd[4] = -1.0;
    if (joy->buttons[4] == 1)
      cmd[5] = 1.0;
    else if (joy->buttons[5] == 1)
      cmd[5] = -1.0;
    if (joy->buttons[2] == 1)
      cmd[6] = 1.0;
    else if (joy->buttons[1] == 1)
      cmd[6] = -1.0;
    JointTeach(r_arm_joint_teach_pub_, cmd);
    break;
  case ControlType::l_Joint:
    cmd[0] = joy->axes[0];
    cmd[1] = joy->axes[1];
    cmd[2] = joy->axes[3];
    cmd[3] = joy->axes[4];
    if (joy->axes[2] == -1)
      cmd[4] = 1.0;
    else if (joy->axes[5] == -1)
      cmd[4] = -1.0;
    if (joy->buttons[4] == 1)
      cmd[5] = 1.0;
    else if (joy->buttons[5] == 1)
      cmd[5] = -1.0;
    if (joy->buttons[2] == 1)
      cmd[6] = 1.0;
    else if (joy->buttons[1] == 1)
      cmd[6] = -1.0;
    JointTeach(l_arm_joint_teach_pub_, cmd);
    break;
  case ControlType::r_Pos:
    cmd[0] = joy->axes[0];
    cmd[1] = joy->axes[1];
    cmd[2] = joy->axes[3];
    PoseTeach(r_arm_pos_teach_pub_, cmd);
    break;
  case ControlType::l_Pos:
    cmd[0] = joy->axes[0];
    cmd[1] = joy->axes[1];
    cmd[2] = joy->axes[3];
    PoseTeach(l_arm_pos_teach_pub_, cmd);
    break;
  case ControlType::r_Ort:
    cmd[0] = joy->axes[0];
    cmd[1] = joy->axes[1];
    cmd[2] = joy->axes[3];
    OriTeach(r_arm_rot_teach_pub_, cmd);
    break;
  case ControlType::l_Ort:
    cmd[0] = joy->axes[0];
    cmd[1] = joy->axes[1];
    cmd[2] = joy->axes[3];
    OriTeach(l_arm_rot_teach_pub_, cmd);
    break;
  case ControlType::base_vel:
    joy_zero_flag_ = false;
    if (!(joy->axes[0] || joy->axes[1] || joy->axes[3] || joy->axes[4])) {
      joy_zero_flag_ = true;
    }
    float gain = 1.0;
    if (joy->axes[5] == -1) { // RT加速
      gain = 1.5;
      ROS_INFO("accelerate....");
    } else if (joy->axes[2] == -1) { // LT减速
      gain = 0.6;
      ROS_INFO("slow dwon....");
    }
    joy_vel_.angular.z = joy->axes[i_velAngular_] * f_velAngular_ * gain;
    joy_vel_.linear.x = joy->axes[i_velLinear_x_] * f_velLinear_ * gain;
    joy_vel_.linear.y = joy->axes[i_velLinear_y_] * f_velLinear_ * gain;
    break;
  }
  if (joy->buttons[9]) {
    joy_wakeup_flag_ = !joy_wakeup_flag_;
    if (joy_wakeup_flag_)
      ROS_WARN("joy vel controller wakeup");
    else
      ROS_WARN("joy vel controller sleep");
  }
}
