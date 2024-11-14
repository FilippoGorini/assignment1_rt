#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include<cmath>

turtlesim::Pose pose1;
turtlesim::Pose pose2;
bool is_pose1_updated = false;
bool is_pose2_updated = false;
ros::Publisher pub_distance;


void pubNewDistance()
{
    float dx = pose1.x - pose2.x;
    float dy = pose1.y - pose2.y;
    float distance = std::sqrt(dx * dx + dy * dy);
    
    ROS_INFO("Distance: %f", distance);
    std_msgs::Float32 distance_msg;
    distance_msg.data = distance;
    pub_distance.publish(distance_msg);
    is_pose1_updated = false;
    is_pose2_updated = false;
}

void checkTurtle1(const turtlesim::Pose::ConstPtr& msg)
{
    pose1 = *msg;
    is_pose1_updated = true;
    if (is_pose2_updated)
        pubNewDistance();
}

void checkTurtle2(const turtlesim::Pose::ConstPtr& msg)
{
    pose2 = *msg;
    is_pose2_updated = true;
    if (is_pose1_updated)
        pubNewDistance();
}

geometry_msgs::Twist stop_msg;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "distance");
    ros::NodeHandle nh;
    ros::Subscriber sub1 = nh.subscribe("/turtle1/pose", 10, checkTurtle1);
    ros::Subscriber sub2 = nh.subscribe("/turtle2/pose", 10, checkTurtle2);
    pub_distance = nh.advertise<std_msgs::Float32>("turtle_distance", 10);
    ros::spin();
    return 0;
}