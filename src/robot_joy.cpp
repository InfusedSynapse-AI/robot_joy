#include <robot_joy/robot_joy.h>

using namespace std;
namespace robot_joy {
TeleopJoy::TeleopJoy(ros::NodeHandle &nh)
    : nh_(nh), joy_wakeup_flag_(false), control_type_(base_vel) {
  joy_sub_ =
      nh.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopJoy::JoyCallBack, this);
  cmd_sender_factory_ =
      std::make_shared<robot_joy_cmd_factory::CmdSenderFactory>();
  cmd_sender_ = std::shared_ptr<robot_joy_cmd_sender::CmdSenderBase>(
      cmd_sender_factory_->CreateCmdSender("mobile"));
  if (!cmd_sender_->Init(nh_)) {
    ROS_ERROR("cmd sender init failed");
    return;
  }
  last_time_ = ros::Time::now();
}
void TeleopJoy::ChangeSender(int step) {
  control_type_ = control_type_ + step;
  auto tmp_ptr = cmd_sender_;
  if (control_type_ >= ControlType::num_end)
    control_type_ = ControlType::base_vel;
  if (control_type_ < ControlType::base_vel)
    control_type_ = ControlType::l_Ort;
  cmd_sender_.reset();
  if (control_type_ == ControlType::base_vel)
    cmd_sender_ = std::shared_ptr<robot_joy_cmd_sender::CmdSenderBase>(
        cmd_sender_factory_->CreateCmdSender("mobile"));
  else
    cmd_sender_ = std::shared_ptr<robot_joy_cmd_sender::CmdSenderBase>(
        cmd_sender_factory_->CreateCmdSender("rm65b"));
  if (!cmd_sender_->Init(nh_)) {
    cmd_sender_.reset();
    cmd_sender_ = tmp_ptr;
    control_type_ = control_type_ - step;
    if (control_type_ >= ControlType::num_end)
      control_type_ = ControlType::base_vel;
    if (control_type_ < ControlType::base_vel)
      control_type_ = ControlType::l_Ort;
  }
}
void TeleopJoy::JoyCallBack(const sensor_msgs::Joy::ConstPtr &joy) {
  last_time_ = ros::Time::now();
  if (joy->buttons[9]) {
    joy_wakeup_flag_ = !joy_wakeup_flag_;
    if (joy_wakeup_flag_)
      ROS_WARN("joy vel controller wakeup");
    else
      ROS_WARN("joy vel controller sleep");
    return;
  }
  if (!joy_wakeup_flag_) {
    cmd_sender_->StopCmd();
    return;
  }
  if (joy->axes[6] == -1) {
    ChangeSender(1);
    return;
  } else if (joy->axes[6] == 1) {
    ChangeSender(-1);
    return;
  }
  std::vector<double> cmd;
  switch (control_type_) {
  case ControlType::r_Joint:
    cmd.resize(4, 0);
    cmd[0] = 0;
    cmd[1] = 1;
    if (joy->axes[0] != 0) {
      cmd[2] = 1;
      cmd[3] = joy->axes[0];
    }
    if (joy->axes[1] != 0) {
      cmd[2] = 2;
      cmd[3] = joy->axes[1];
    }
    if (joy->axes[3] != 0) {
      cmd[2] = 3;
      cmd[3] = joy->axes[3];
    }
    if (joy->axes[4] != 0) {
      cmd[2] = 4;
      cmd[3] = joy->axes[4];
    }
    if (joy->axes[2] == -1) {
      cmd[2] = 5;
      cmd[3] = 1.0;
    }
    if (joy->axes[5] == -1) {
      cmd[2] = 5;
      cmd[3] = -1.0;
    }
    if (joy->axes[4] == 1) {
      cmd[2] = 6;
      cmd[3] = 1.0;
    }
    if (joy->axes[5] == 1) {
      cmd[2] = 6;
      cmd[3] = -1.0;
    }
    if (joy->axes[2] == 1) {
      cmd[2] = 7;
      cmd[3] = 1.0;
    }
    if (joy->axes[1] == 1) {
      cmd[2] = 7;
      cmd[3] = -1.0;
    }
    cmd_sender_->SendCmd(cmd);
    break;
  case ControlType::l_Joint:
    cmd.resize(4, 0);
    cmd[0] = 1;
    cmd[1] = 1;
    if (joy->axes[0] != 0) {
      cmd[2] = 1;
      cmd[3] = joy->axes[0];
    }
    if (joy->axes[1] != 0) {
      cmd[2] = 2;
      cmd[3] = joy->axes[1];
    }
    if (joy->axes[3] != 0) {
      cmd[2] = 3;
      cmd[3] = joy->axes[3];
    }
    if (joy->axes[4] != 0) {
      cmd[2] = 4;
      cmd[3] = joy->axes[4];
    }
    if (joy->axes[2] == -1) {
      cmd[2] = 5;
      cmd[3] = 1.0;
    }
    if (joy->axes[5] == -1) {
      cmd[2] = 5;
      cmd[3] = -1.0;
    }
    if (joy->axes[4] == 1) {
      cmd[2] = 6;
      cmd[3] = 1.0;
    }
    if (joy->axes[5] == 1) {
      cmd[2] = 6;
      cmd[3] = -1.0;
    }
    if (joy->axes[2] == 1) {
      cmd[2] = 7;
      cmd[3] = 1.0;
    }
    if (joy->axes[1] == 1) {
      cmd[2] = 7;
      cmd[3] = -1.0;
    }
    cmd_sender_->SendCmd(cmd);
    break;
  case ControlType::r_Pos:
    cmd.resize(4, 0);
    cmd[0] = 0;
    cmd[1] = 2;
    if (joy->axes[0] != 0) {
      cmd[2] = 1;
      cmd[3] = joy->axes[0];
    }
    if (joy->axes[1] != 0) {
      cmd[2] = 2;
      cmd[3] = joy->axes[1];
    }
    if (joy->axes[3] != 0) {
      cmd[2] = 3;
      cmd[3] = joy->axes[3];
    }
    cmd_sender_->SendCmd(cmd);
    break;
  case ControlType::l_Pos:
    cmd.resize(4, 0);
    cmd[0] = 1;
    cmd[1] = 2;
    if (joy->axes[0] != 0) {
      cmd[2] = 1;
      cmd[3] = joy->axes[0];
    }
    if (joy->axes[1] != 0) {
      cmd[2] = 2;
      cmd[3] = joy->axes[1];
    }
    if (joy->axes[3] != 0) {
      cmd[2] = 3;
      cmd[3] = joy->axes[3];
    }
    cmd_sender_->SendCmd(cmd);
    break;
  case ControlType::r_Ort:
    cmd.resize(4, 0);
    cmd[0] = 0;
    cmd[1] = 3;
    if (joy->axes[0] != 0) {
      cmd[2] = 1;
      cmd[3] = joy->axes[0];
    }
    if (joy->axes[1] != 0) {
      cmd[2] = 2;
      cmd[3] = joy->axes[1];
    }
    if (joy->axes[3] != 0) {
      cmd[2] = 3;
      cmd[3] = joy->axes[3];
    }
    cmd_sender_->SendCmd(cmd);
    break;
  case ControlType::l_Ort:
    cmd.resize(4, 0);
    cmd[0] = 1;
    cmd[1] = 3;
    if (joy->axes[0] != 0) {
      cmd[2] = 1;
      cmd[3] = joy->axes[0];
    }
    if (joy->axes[1] != 0) {
      cmd[2] = 2;
      cmd[3] = joy->axes[1];
    }
    if (joy->axes[3] != 0) {
      cmd[2] = 3;
      cmd[3] = joy->axes[3];
    }
    cmd_sender_->SendCmd(cmd);
    break;
  case ControlType::base_vel:
    cmd.resize(4, 0);
    cmd[0] = joy->axes[1];
    cmd[1] = joy->axes[0];
    cmd[2] = joy->axes[3];
    if (joy->axes[5] == -1) { // RT加速
      cmd[3] = 1.0;
      ROS_INFO("accelerate....");
    } else if (joy->axes[2] == -1) { // LT减速
      cmd[3] = -1.0;
      ROS_INFO("slow dwon....");
    }
    cmd_sender_->SendCmd(cmd);
    break;
  }
}
void TeleopJoy::run() {
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    if ((ros::Time::now() - last_time_).toSec() > 10.0) {
      cmd_sender_->StopCmd();
      joy_wakeup_flag_ = false;
    }
    loop_rate.sleep();
  }
}
} // namespace robot_joy
