#include "ros/ros.h"
#include "turtlesim/Pose.h"

void check_turtles(void)
{
    int a = 2;          // placeholder
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "bounds_checker");
    // ros::NodeHandle nh;
    // ros::Subscriber sub1 = nh.subscribe("/turtle1/pose/", 10, check_turtles);
    // ros::Subscriber sub2 = nh.subscribe("/turtle2/pose/", 10, check_turtles);
    ros::spin();
    return 0;
}