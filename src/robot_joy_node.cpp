#include <robot_joy/robot_joy.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_joy");
    TeleopJoy& teleop_bobac = TeleopJoy::getInstance();
    ros::spin();    
    geometry_msgs::Twist stop;
    teleop_bobac.pub.publish(stop);
    return 0;
}