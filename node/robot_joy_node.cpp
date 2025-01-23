#include <robot_joy/robot_joy.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_joy");
    ros::NodeHandle nh;
    robot_joy::TeleopJoy teleop(nh);
    teleop.run();
    return 0;
}