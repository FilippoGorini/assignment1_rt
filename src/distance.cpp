#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "turtlesim/TeleportAbsolute.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"


const float DISTANCE_THRESHOLD = 1.0;
const float BOUNDARY_MIN = 1.0;
const float BOUNDARY_MAX = 10.0;

bool is_pose1_updated = false;
bool is_pose2_updated = false;
bool distance_flag = false;
int last_turtle_moved;

turtlesim::Pose pose1;
turtlesim::Pose pose2;
geometry_msgs::Twist last_cmd_vel1;
geometry_msgs::Twist last_cmd_vel2;
ros::Publisher pub_distance;
ros::Publisher pub_vel1;
ros::Publisher pub_vel2;
ros::ServiceClient teleport_client1;
ros::ServiceClient teleport_client2;


void stopTurtle(int turtle_select)
{
    geometry_msgs::Twist stop_msg;
    if (turtle_select == 1)
        pub_vel1.publish(stop_msg);
    if (turtle_select == 2)
        pub_vel2.publish(stop_msg);
}

void stepBackTurtle(int turtle_select)
{
    geometry_msgs::Twist rev_vel;
    if (turtle_select == 1)
    {
        rev_vel.linear.x = -last_cmd_vel1.linear.x;
        rev_vel.linear.y = -last_cmd_vel1.linear.y;
        rev_vel.angular.z = -last_cmd_vel1.angular.z;
        pub_vel1.publish(rev_vel);
    }
    if (turtle_select == 2)
    {
        rev_vel.linear.x = -last_cmd_vel2.linear.x;
        rev_vel.linear.y = -last_cmd_vel2.linear.y;
        rev_vel.angular.z = -last_cmd_vel2.angular.z;
        pub_vel2.publish(rev_vel);
    }
}

void pubDistance(float distance)
{
    ROS_INFO("Distance: %f", distance);
    std_msgs::Float32 distance_msg;
    distance_msg.data = distance;
    pub_distance.publish(distance_msg);
}

void checkDistance()
{
    float dx = pose1.x - pose2.x;
    float dy = pose1.y - pose2.y;
    float distance = std::sqrt(dx * dx + dy * dy);

    if (distance <= DISTANCE_THRESHOLD)
    {
        ROS_WARN("The two turtles are too close to eachother!");
        stepBackTurtle(last_turtle_moved); 
        distance_flag = true;
    }

    if(distance > DISTANCE_THRESHOLD && distance_flag)
    {
        stopTurtle(last_turtle_moved);
        distance_flag = false;
    }

    pubDistance(distance);
    is_pose1_updated = false;
    is_pose2_updated = false;
}

void checkMargins()
{
    if (pose1.x < BOUNDARY_MIN || pose1.x > BOUNDARY_MAX || pose1.y < BOUNDARY_MIN || pose1.y > BOUNDARY_MAX)
    {
        stopTurtle(last_turtle_moved);
        ROS_WARN("Turtle1 is out of bounds!");
        turtlesim::TeleportAbsolute teleport1_srv;
        teleport1_srv.request.x = std::max(BOUNDARY_MIN, std::min(BOUNDARY_MAX, pose1.x));
        teleport1_srv.request.y = std::max(BOUNDARY_MIN, std::min(BOUNDARY_MAX, pose1.y));
        teleport1_srv.request.theta = pose1.theta;
        teleport_client1.call(teleport1_srv);
    }

    if (pose2.x < BOUNDARY_MIN || pose2.x > BOUNDARY_MAX || pose2.y < BOUNDARY_MIN || pose2.y > BOUNDARY_MAX)
    {
        stopTurtle(last_turtle_moved);
        ROS_WARN("Turtle2 is out of bounds!");
        turtlesim::TeleportAbsolute teleport2_srv;
        teleport2_srv.request.x = std::max(BOUNDARY_MIN, std::min(BOUNDARY_MAX, pose2.x));
        teleport2_srv.request.y = std::max(BOUNDARY_MIN, std::min(BOUNDARY_MAX, pose2.y));
        teleport2_srv.request.theta = pose2.theta; 
        teleport_client2.call(teleport2_srv);
    }
}

void checkTurtle1(const turtlesim::Pose::ConstPtr& msg)
{
    pose1 = *msg;
    is_pose1_updated = true;
    if (is_pose2_updated)
    {   
        checkDistance();
        checkMargins();
    }
}

void checkTurtle2(const turtlesim::Pose::ConstPtr& msg)
{
    pose2 = *msg;
    is_pose2_updated = true;
    if (is_pose1_updated)
    {
        checkDistance();
        checkMargins();
    }
}

void updateLastCmdVel1(const geometry_msgs::Twist::ConstPtr& msg)
{
    last_cmd_vel1 = *msg;
    last_turtle_moved = 1;
}

void updateLastCmdVel2(const geometry_msgs::Twist::ConstPtr& msg)
{
    last_cmd_vel2 = *msg;
    last_turtle_moved = 2;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "distance");
    ros::NodeHandle nh;
    ros::Subscriber sub_pose1 = nh.subscribe("/turtle1/pose", 10, checkTurtle1);
    ros::Subscriber sub_pose2 = nh.subscribe("/turtle2/pose", 10, checkTurtle2);
    ros::Subscriber sub_vel1 = nh.subscribe("/turtle1/cmd_vel", 10, updateLastCmdVel1);
    ros::Subscriber sub_vel2 = nh.subscribe("/turtle2/cmd_vel", 10, updateLastCmdVel2);
    pub_distance = nh.advertise<std_msgs::Float32>("turtle_distance", 10);
    pub_vel1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    pub_vel2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);
    teleport_client1 = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
    teleport_client2 = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle2/teleport_absolute");
    ros::spin();
    return 0;
}