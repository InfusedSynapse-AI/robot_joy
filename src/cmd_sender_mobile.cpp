#include <robot_joy/cmd_sender_mobile.h>
namespace robot_joy_cmd_sender {
bool CmdSenderMobile::Init(ros::NodeHandle &nh) {
  nh_ = nh;
  nh_.param("smooth_control", smooth_control_, true);
  nh_.param("smooth_transition_time", smooth_transition_time_, 1.0);
  nh_.param("max_vel_trans", max_vel_trans_, 1.0);
  nh_.param("max_vel_th", max_vel_th_, 0.5);
  nh_.param("acc_lim_trans", acc_lim_trans_, 0.3);
  nh_.param("acc_lim_th", acc_lim_th_, 0.2);
  nh_.param("dec_lim_trans", dec_lim_trans_, 0.5);
  nh_.param("dec_lim_th", dec_lim_th_, 0.5);
  nh_.param("odom_model_type", odom_model_type_, std::string("diff"));
  odom_received_ = false;
  if (odom_model_type_ != "diff" && odom_model_type_ != "omni") {
    ROS_ERROR("Invalid odom model type, must be diff or omni");
    return false;
  }
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("joy_vel", 1);
  odom_sub_ = nh_.subscribe("odom", 1, &CmdSenderMobile::OdomCallback, this);
  dynamic_reconfigure::Server<robot_joy::VelConfig>::CallbackType f;
  f = boost::bind(&CmdSenderMobile::DynamicCallback, this, _1, _2);
  cfg_server_.setCallback(f);
  last_time_ = ros::Time::now();
  return true;
}
void CmdSenderMobile::OdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  current_vel_ = msg->twist.twist;
  odom_received_ = true;
  last_time_ = ros::Time::now();
}
void CmdSenderMobile::DynamicCallback(robot_joy::VelConfig &config,
                                      uint32_t level) {
  smooth_control_ = config.smooth_control;
  smooth_transition_time_ = config.smooth_transition_time;
  max_vel_trans_ = config.max_vel_trans;
  max_vel_th_ = config.max_vel_th;
  acc_lim_trans_ = config.acc_lim_trans;
  acc_lim_th_ = config.acc_lim_th;
  dec_lim_trans_ = config.dec_lim_trans;
  dec_lim_th_ = config.dec_lim_th;
  ROS_DEBUG(
      "Reconfigure Request, smooth_control_: %d, smooth_transition_time_: %f, "
      "max_vel_trans_: %f, max_vel_th_: %f, acc_lim_trans_: %f, acc_lim_th_: "
      "%f, dec_lim_trans_: %f, dec_lim_th_: %f",
      config.smooth_control, config.smooth_transition_time,
      config.max_vel_trans, config.max_vel_th, config.acc_lim_trans,
      config.acc_lim_th, config.dec_lim_trans, config.dec_lim_th);
}
double CmdSenderMobile::SmoothControlWithSCurve(double current, double target,
                                                double dt) {
  if (dt >= smooth_transition_time_) {
    return current;
  }
  double t = dt / smooth_transition_time_;
  double s = 3 * t * t - 2 * t * t * t;
  return current + s * (target - current);
}
void CmdSenderMobile::ApplyLimit(geometry_msgs::Twist &target_vel, double gain=1.0) {
  if (smooth_control_) {
    target_vel.linear.x =
        SmoothControlWithSCurve(current_vel_.linear.x, target_vel.linear.x,
                                (ros::Time::now() - last_time_).toSec());
    target_vel.angular.z =
        SmoothControlWithSCurve(current_vel_.angular.z, target_vel.angular.z,
                                (ros::Time::now() - last_time_).toSec());

    double delta_x = target_vel.linear.x - current_vel_.linear.x;
    double delta_th = target_vel.angular.z - current_vel_.angular.z;
    double dt = 0.1;
    double acc_x = (delta_x >= 0) ? acc_lim_trans_ * dt : dec_lim_trans_ * dt;
    double acc_th = (delta_th >= 0) ? acc_lim_th_ * dt : dec_lim_th_ * dt;
    delta_x = std::max(-std::abs(acc_x), std::min(std::abs(acc_x), delta_x));
    delta_th =
        std::max(-std::abs(acc_th), std::min(std::abs(acc_th), delta_th));
    target_vel.linear.x = current_vel_.linear.x + delta_x;

    target_vel.angular.z = current_vel_.angular.z + delta_th;
    if (odom_model_type_ == "omni") {
      double delta_y = target_vel.linear.y - current_vel_.linear.y;
      double acc_y = (delta_y >= 0) ? acc_lim_trans_ * dt : dec_lim_trans_ * dt;
      delta_y = std::max(-std::abs(acc_y), std::min(std::abs(acc_y), delta_y));
      target_vel.linear.y = current_vel_.linear.y + delta_y;
    }
  }
  if (target_vel.linear.x < 0)
    target_vel.linear.x = std::min(target_vel.linear.x, -max_vel_trans_*gain);
  else
    target_vel.linear.x = std::max(target_vel.linear.x, max_vel_trans_*gain);
  if (target_vel.linear.y < 0)
    target_vel.linear.y = std::min(target_vel.linear.y, -max_vel_trans_*gain);
  else
    target_vel.linear.y = std::max(target_vel.linear.y, max_vel_trans_*gain);
  if (target_vel.angular.z < 0)
    target_vel.angular.z = std::min(target_vel.angular.z, -max_vel_th_*gain);
  else
    target_vel.angular.z = std::max(target_vel.angular.z, max_vel_th_*gain);
}
void CmdSenderMobile::SendCmd(std::vector<double> cmd) {
  if(!odom_received_)
    return;
  geometry_msgs::Twist target_vel;
  double gain = 1.0;
  if (cmd[3] == 1) { // RT加速
    gain = 2.0;
  } else if (cmd[3] == -1) { // LT减速
    gain = 0.5;
  }
  target_vel.linear.x = max_vel_trans_ * cmd[0];
  target_vel.angular.z = max_vel_th_ * cmd[2];
  if(odom_model_type_ == "omni")
    target_vel.linear.y = max_vel_trans_ * cmd[1];
  ApplyLimit(target_vel, gain);
  vel_pub_.publish(target_vel);
  odom_received_ = false;
}
void CmdSenderMobile::StopCmd() {
  geometry_msgs::Twist stop;
  vel_pub_.publish(stop);
}
} // namespace robot_joy_cmd_sender