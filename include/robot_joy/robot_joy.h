/*
 * @Author: Msnakes 1327153842@qq.com
 * @Date: 2024-12-24 10:57:04
 * @LastEditors: Msnakes 1327153842@qq.com
 * @Description: ros手柄控制(速度、睿尔曼手柄示教控制)
 *
 */
#include <dual_arm_msgs/Joint_Teach.h>
#include <dual_arm_msgs/Ort_Teach.h>
#include <dual_arm_msgs/Pos_Teach.h>
#include <dual_arm_msgs/Stop_Teach.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
class TeleopJoy {
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

public:
  static TeleopJoy &getInstance() {
    static TeleopJoy instance;
    return instance;
  }
  void run() {
    ros::Rate loop(20);
    while (ros::ok()) {
      ros::spinOnce();
      if (joy_wakeup_flag_) {
        switch (control_type_) {
        case base_vel:
          vel_pub_.publish(joy_vel_);
        }
        if (joy_zero_flag_) {
          if ((ros::Time::now().toSec() - last_time_.toSec()) > 30) {
            joy_wakeup_flag_ = false;
            joy_zero_flag_ = false;
            ROS_WARN("controller has sleeped");
          }
        }
      }
      loop.sleep();
    }
  }

private:
  void callBack(const sensor_msgs::Joy::ConstPtr &joy);
  void StopVel() {
    geometry_msgs::Twist stop;
    vel_pub_.publish(stop);
  }
  void StopArm() {
    r_arm_stop_teach_pub_.publish(arm_stop_tech_msg_);
    l_arm_stop_teach_pub_.publish(arm_stop_tech_msg_);
    arm_stop_flag_ = true;
  }
  void JointTeach(ros::Publisher &pub, std::vector<float> &cmd) {
    if (cmd[0] > 0.5) {
      if ((arm_joint_tech_msg_.direction == "pos") &&
          (arm_joint_tech_msg_.teach_joint == 1))
        return;
      arm_stop_flag_ = false;
      arm_joint_tech_msg_.direction = "pos";
      arm_joint_tech_msg_.teach_joint = 1;
    } else if (cmd[0] < -0.5) {
      if ((arm_joint_tech_msg_.direction == "neg") &&
          (arm_joint_tech_msg_.teach_joint == 1))
        return;
      arm_stop_flag_ = false;
      arm_joint_tech_msg_.direction = "neg";
      arm_joint_tech_msg_.teach_joint = 1;
    } else if (cmd[1] > 0.5) {
      if ((arm_joint_tech_msg_.direction == "pos") &&
          (arm_joint_tech_msg_.teach_joint == 2))
        return;
      arm_stop_flag_ = false;
      arm_joint_tech_msg_.direction = "pos";
      arm_joint_tech_msg_.teach_joint = 2;
    } else if (cmd[1] < -0.5) {
      if ((arm_joint_tech_msg_.direction == "neg") &&
          (arm_joint_tech_msg_.teach_joint == 2))
        return;
      arm_stop_flag_ = false;
      arm_joint_tech_msg_.direction = "neg";
      arm_joint_tech_msg_.teach_joint = 2;
    } else if (cmd[2] > 0.5) {
      if ((arm_joint_tech_msg_.direction == "pos") &&
          (arm_joint_tech_msg_.teach_joint == 3))
        return;
      arm_stop_flag_ = false;
      arm_joint_tech_msg_.direction = "pos";
      arm_joint_tech_msg_.teach_joint = 3;
    } else if (cmd[2] < -0.5) {
      if ((arm_joint_tech_msg_.direction == "neg") &&
          (arm_joint_tech_msg_.teach_joint == 3))
        return;
      arm_stop_flag_ = false;
      arm_joint_tech_msg_.direction = "neg";
      arm_joint_tech_msg_.teach_joint = 3;
    } else if (cmd[3] > 0.5) {
      if ((arm_joint_tech_msg_.direction == "pos") &&
          (arm_joint_tech_msg_.teach_joint == 4))
        return;
      arm_stop_flag_ = false;
      arm_joint_tech_msg_.direction = "pos";
      arm_joint_tech_msg_.teach_joint = 4;
    } else if (cmd[3] < -0.5) {
      if ((arm_joint_tech_msg_.direction == "neg") &&
          (arm_joint_tech_msg_.teach_joint == 4))
        return;
      arm_stop_flag_ = false;
      arm_joint_tech_msg_.direction = "neg";
      arm_joint_tech_msg_.teach_joint = 4;
    } else if (cmd[4] == -1) {
      if ((arm_joint_tech_msg_.direction == "pos") &&
          (arm_joint_tech_msg_.teach_joint == 5))
        return;
      arm_stop_flag_ = false;
      arm_joint_tech_msg_.direction = "pos";
      arm_joint_tech_msg_.teach_joint = 5;
    } else if (cmd[4] == -1) {
      if ((arm_joint_tech_msg_.direction == "neg") &&
          (arm_joint_tech_msg_.teach_joint == 5))
        return;
      arm_stop_flag_ = false;
      arm_joint_tech_msg_.direction = "neg";
      arm_joint_tech_msg_.teach_joint = 5;
    } else if (cmd[5] == 1) {
      if ((arm_joint_tech_msg_.direction == "pos") &&
          (arm_joint_tech_msg_.teach_joint == 6))
        return;
      arm_stop_flag_ = false;
      arm_joint_tech_msg_.direction = "pos";
      arm_joint_tech_msg_.teach_joint = 6;
    } else if (cmd[5] == -1) {
      if ((arm_joint_tech_msg_.direction == "neg") &&
          (arm_joint_tech_msg_.teach_joint == 6))
        return;
      arm_stop_flag_ = false;
      arm_joint_tech_msg_.direction = "neg";
      arm_joint_tech_msg_.teach_joint = 6;
    } else if (cmd[6] == 1) {
      if ((arm_joint_tech_msg_.direction == "pos") &&
          (arm_joint_tech_msg_.teach_joint == 7))
        return;
      arm_stop_flag_ = false;
      arm_joint_tech_msg_.direction = "pos";
      arm_joint_tech_msg_.teach_joint = 7;
    } else if (cmd[6] == -1) {
      if ((arm_joint_tech_msg_.direction == "neg") &&
          (arm_joint_tech_msg_.teach_joint == 7))
        return;
      arm_stop_flag_ = false;
      arm_joint_tech_msg_.direction = "neg";
      arm_joint_tech_msg_.teach_joint = 7;
    } else {
      if (arm_stop_flag_)
        return;
      StopArm();
      return;
    }
    pub.publish(arm_joint_tech_msg_);
  }
  void PoseTeach(ros::Publisher &pub, std::vector<float> &cmd) {
    if (cmd[0] == 1) {
      if ((arm_pos_tech_msg_.direction == "pos") &&
          (arm_pos_tech_msg_.teach_type == "x"))
        return;
      arm_stop_flag_ = false;
      arm_pos_tech_msg_.direction = "pos";
      arm_pos_tech_msg_.teach_type = "x";
    } else if (cmd[0] == -1) {
      if ((arm_pos_tech_msg_.direction == "neg") &&
          (arm_pos_tech_msg_.teach_type == "x"))
        return;
      arm_stop_flag_ = false;
      arm_pos_tech_msg_.direction = "neg";
      arm_pos_tech_msg_.teach_type = "x";
    } else if (cmd[1] == 1) {
      if ((arm_pos_tech_msg_.direction == "pos") &&
          (arm_pos_tech_msg_.teach_type == "y"))
        return;
      arm_stop_flag_ = false;
      arm_pos_tech_msg_.direction = "pos";
      arm_pos_tech_msg_.teach_type = "y";
    } else if (cmd[1] == -1) {
      if ((arm_pos_tech_msg_.direction == "neg") &&
          (arm_pos_tech_msg_.teach_type == "y"))
        return;
      arm_stop_flag_ = false;
      arm_pos_tech_msg_.direction = "neg";
      arm_pos_tech_msg_.teach_type = "y";
    } else if (cmd[2] == 1) {
      if ((arm_pos_tech_msg_.direction == "pos") &&
          (arm_pos_tech_msg_.teach_type == "z"))
        return;
      arm_stop_flag_ = false;
      arm_pos_tech_msg_.direction = "pos";
      arm_pos_tech_msg_.teach_type = "z";
    } else if (cmd[2] == -1) {
      if ((arm_pos_tech_msg_.direction == "neg") &&
          (arm_pos_tech_msg_.teach_type == "z"))
        return;
      arm_stop_flag_ = false;
      arm_pos_tech_msg_.direction = "neg";
      arm_pos_tech_msg_.teach_type = "z";
    } else {
      if (arm_stop_flag_)
        return;
      StopArm();
      return;
    }
    pub.publish(arm_pos_tech_msg_);
  }
  void OriTeach(ros::Publisher &pub, std::vector<float> &cmd) {
    if (cmd[0] == 1) {
      if ((arm_ort_tech_msg_.direction == "pos") &&
          (arm_ort_tech_msg_.teach_type == "x"))
        return;
      arm_stop_flag_ = false;
      arm_ort_tech_msg_.direction = "pos";
      arm_ort_tech_msg_.teach_type = "rx";
    } else if (cmd[0] == -1) {
      if ((arm_ort_tech_msg_.direction == "neg") &&
          (arm_ort_tech_msg_.teach_type == "rx"))
        return;
      arm_stop_flag_ = false;
      arm_ort_tech_msg_.direction = "neg";
      arm_ort_tech_msg_.teach_type = "rx";
    } else if (cmd[1] == 1) {
      if ((arm_ort_tech_msg_.direction == "pos") &&
          (arm_ort_tech_msg_.teach_type == "ry"))
        return;
      arm_stop_flag_ = false;
      arm_ort_tech_msg_.direction = "pos";
      arm_ort_tech_msg_.teach_type = "ry";
    } else if (cmd[1] == -1) {
      if ((arm_ort_tech_msg_.direction == "neg") &&
          (arm_ort_tech_msg_.teach_type == "ry"))
        return;
      arm_stop_flag_ = false;
      arm_ort_tech_msg_.direction = "neg";
      arm_ort_tech_msg_.teach_type = "ry";
    } else if (cmd[2] == 1) {
      if ((arm_ort_tech_msg_.direction == "pos") &&
          (arm_ort_tech_msg_.teach_type == "rz"))
        return;
      arm_stop_flag_ = false;
      arm_ort_tech_msg_.direction = "pos";
      arm_ort_tech_msg_.teach_type = "rz";
    } else if (cmd[2] == -1) {
      if ((arm_ort_tech_msg_.direction == "neg") &&
          (arm_ort_tech_msg_.teach_type == "rz"))
        return;
      arm_stop_flag_ = false;
      arm_ort_tech_msg_.direction = "neg";
      arm_ort_tech_msg_.teach_type = "rz";
    } else {
      if (arm_stop_flag_)
        return;
      StopArm();
      return;
    }
    pub.publish(arm_ort_tech_msg_);
  }
  ros::NodeHandle n;
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher vel_pub_;
  ros::Publisher r_arm_pos_teach_pub_;
  ros::Publisher r_arm_rot_teach_pub_;
  ros::Publisher r_arm_joint_teach_pub_;
  ros::Publisher r_arm_stop_teach_pub_;
  ros::Publisher l_arm_pos_teach_pub_;
  ros::Publisher l_arm_rot_teach_pub_;
  ros::Publisher l_arm_joint_teach_pub_;
  ros::Publisher l_arm_stop_teach_pub_;
  geometry_msgs::Twist joy_vel_;
  dual_arm_msgs::Joint_Teach arm_joint_tech_msg_;
  dual_arm_msgs::Pos_Teach arm_pos_tech_msg_;
  dual_arm_msgs::Ort_Teach arm_ort_tech_msg_;
  dual_arm_msgs::Stop_Teach arm_stop_tech_msg_;
  int i_velLinear_x_, i_velLinear_y_, i_velAngular_; // axes number
  int i_accelerator_, i_deceleration_;
  int i_maxLinearVelIncrease_, i_maxAngularVelIncrease_, i_maxLinearVelReduce_,
      i_maxAngularVelReduce_;
  double f_velLinearMax_, f_velAngularMax_, f_velLinear, f_velAngular_;
  bool joy_wakeup_flag_, joy_zero_flag_, arm_stop_flag_;
  int last_button_;
  int last_axis_;
  int ctrl_joint_;
  int control_type_;

  ros::Time last_time_, now_time_;

  TeleopJoy();
  ~TeleopJoy() {
    geometry_msgs::Twist stop;
    vel_pub_.publish(stop);
  }
  TeleopJoy(const TeleopJoy &other) = delete;
  TeleopJoy &operator=(const TeleopJoy &other) = delete;
};