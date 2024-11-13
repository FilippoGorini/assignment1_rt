#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "your_node_name");
    ros::NodeHandle n;
   
    ros::spin();
    return 0;
}