#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/Joy.h>
#include<iostream>

class TeleopJoy
{
public:
    
    ros::Publisher pub;
    static TeleopJoy& getInstance(){
        static TeleopJoy instance;
        return instance;
    }
private:
    void callBack(const sensor_msgs::Joy::ConstPtr& joy);
    ros::NodeHandle n;
    ros::NodeHandle nh;
    ros::Subscriber sub;
    int i_velLinear_x, i_velLinear_y, i_velAngular; //axes number
    int i_accelerator, i_deceleration;
    int i_maxLinearVelIncrease, i_maxAngularVelIncrease, i_maxLinearVelReduce, i_maxAngularVelReduce;
    double f_velLinearMax, f_velAngularMax, f_velLinear, f_velAngular;
    bool flag_publish_maker_;
    TeleopJoy();
    TeleopJoy(const TeleopJoy& other) = delete;
    TeleopJoy& operator=(const TeleopJoy& other) = delete;
    
};