#include <ros/ros.h>
#include "GaSlam.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ga_slam_node");
    ros::NodeHandle nh;
    ga_slam::GaSlam gaSlam_(nh);

    ROS_INFO("Hello world!");
    ros::spin();
    return 0;
}
