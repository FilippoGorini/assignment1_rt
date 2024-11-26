#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "turtlesim/TeleportAbsolute.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"


const float DISTANCE_THRESHOLD = 1.0;
const float MARGIN_MIN = 1.0;
const float MARGIN_MAX = 10.0;

bool is_pose1_updated = false;
bool is_pose2_updated = false;
bool is_stepping_back = false;              
int last_turtle_moved = 0;                      // Stores the id of the last turtle that received a cmd_vel message

turtlesim::Pose pose1, pose2;                  
geometry_msgs::Twist last_cmd_vel1, last_cmd_vel2;
ros::Publisher pub_distance, pub_vel1, pub_vel2;
ros::ServiceClient teleport_client1, teleport_client2;


// This function stops the turtle selected with the turtle_select argument
void stopTurtle(int turtle_select)
{
    geometry_msgs::Twist stop_msg;
    if (turtle_select == 1)
        pub_vel1.publish(stop_msg);
    if (turtle_select == 2)
        pub_vel2.publish(stop_msg);
}

// This function publishes a cmd_vel message on the cmd_vel topic of the turtle selected by turtle_select, opposite to the last one ...
// ... that the turtle received.
void stepBackTurtle(int turtle_select)
{
    // During the first call of the function, is_steeping_back is false, so we set rev_vel to the ...
    // ... opposite of the last cmd_vel message. Once this is done, is_stepping_back is set to true.
    static geometry_msgs::Twist rev_vel; 
    
    if (!is_stepping_back)
    {
        if (turtle_select == 1)
        {
            rev_vel.linear.x = -last_cmd_vel1.linear.x;
            rev_vel.linear.y = -last_cmd_vel1.linear.y;
            rev_vel.angular.z = -last_cmd_vel1.angular.z;
        }
        if (turtle_select == 2)
        {
            rev_vel.linear.x = -last_cmd_vel2.linear.x;
            rev_vel.linear.y = -last_cmd_vel2.linear.y;
            rev_vel.angular.z = -last_cmd_vel2.angular.z;
        }
        is_stepping_back = true;
    }

    // Publish the velocity commands to the correct topic
    if (turtle_select == 1)
        pub_vel1.publish(rev_vel);
    if (turtle_select == 2)
        pub_vel2.publish(rev_vel);       
}


// This function simply publishes the distance computed to the corresponding topic created in the main function
void pubDistance(float distance)
{
    ROS_INFO("Distance: %f", distance);
    std_msgs::Float32 distance_msg;
    distance_msg.data = distance;
    pub_distance.publish(distance_msg);
}

// Ths function computes the euclidean distance between the turtles, publishes it, and if it is less than the threshold, it calls the ...
// ... stepBackTurtle function to make the last turtle moved go back on its path until it's no longer too close to to the other one
// Similarly to the checkMargins() function, this is needed to avoid getting the turtle stuck in the illegal area
void checkDistance()
{
    float dx = pose1.x - pose2.x;
    float dy = pose1.y - pose2.y;
    float distance = std::sqrt(dx * dx + dy * dy);
    pubDistance(distance);

    if (distance <= DISTANCE_THRESHOLD)
    {
        ROS_WARN("The two turtles are too close to each other!");
        stepBackTurtle(last_turtle_moved);
    }
    if (distance > DISTANCE_THRESHOLD && is_stepping_back)                              // Stop stepping back once distance is safe
    {
        stopTurtle(last_turtle_moved);
        is_stepping_back = false;                           // Reset the stepping-back state
    }
}

// This function checks both turtles' poses to see if they're out of bounds and, if yes, the turtle is stopped and teleported back to within the ...
// ... set boundary. This is needed because otherwise, if we just stop the turtle, it will remain in an illegal area and it won't be able to exit ...
// ... unless we give it a really large velocity 
void checkMargins()
{
    if (pose1.x < MARGIN_MIN || pose1.x > MARGIN_MAX || pose1.y < MARGIN_MIN || pose1.y > MARGIN_MAX)
    {
        stopTurtle(last_turtle_moved);
        turtlesim::TeleportAbsolute teleport1_srv;
        teleport1_srv.request.x = std::max(MARGIN_MIN, std::min(MARGIN_MAX, pose1.x));
        teleport1_srv.request.y = std::max(MARGIN_MIN, std::min(MARGIN_MAX, pose1.y));
        teleport1_srv.request.theta = pose1.theta;
        teleport_client1.call(teleport1_srv);
        ROS_WARN("Turtle1 is out of bounds!");
    }

    if (pose2.x < MARGIN_MIN || pose2.x > MARGIN_MAX || pose2.y < MARGIN_MIN || pose2.y > MARGIN_MAX)
    {
        stopTurtle(last_turtle_moved);
        turtlesim::TeleportAbsolute teleport2_srv;
        teleport2_srv.request.x = std::max(MARGIN_MIN, std::min(MARGIN_MAX, pose2.x));
        teleport2_srv.request.y = std::max(MARGIN_MIN, std::min(MARGIN_MAX, pose2.y));
        teleport2_srv.request.theta = pose2.theta; 
        teleport_client2.call(teleport2_srv);
        ROS_WARN("Turtle2 is out of bounds!");
    }
}

// updateTurtle1 and updateTurtle2 are the callback functions called by the corresponding pose topic subscribers
// To save some computations, the checkDistance() and checkMargins() routines are called only once both turtles poses have been updated
void updateTurtle1(const turtlesim::Pose::ConstPtr& msg)
{
    pose1 = *msg;
    is_pose1_updated = true;
    if (is_pose2_updated)
    {   
        checkDistance();
        checkMargins();
        is_pose1_updated = false;
        is_pose2_updated = false;
    }
}

void updateTurtle2(const turtlesim::Pose::ConstPtr& msg)
{
    pose2 = *msg;
    is_pose2_updated = true;
    if (is_pose1_updated)
    {
        checkDistance();
        checkMargins();
        is_pose1_updated = false;
        is_pose2_updated = false;
    }
}

// updateLastCmdVel1 and updateLastCmdVel2 are used to store the most recent cmd_vel messages for each turtle ...
// ... and to know which was the last turtle to receive such commands
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

    // Subscribers
    ros::Subscriber sub_pose1 = nh.subscribe("/turtle1/pose", 10, updateTurtle1);
    ros::Subscriber sub_pose2 = nh.subscribe("/turtle2/pose", 10, updateTurtle2);
    ros::Subscriber sub_vel1 = nh.subscribe("/turtle1/cmd_vel", 10, updateLastCmdVel1);
    ros::Subscriber sub_vel2 = nh.subscribe("/turtle2/cmd_vel", 10, updateLastCmdVel2);

    // Publishers
    pub_distance = nh.advertise<std_msgs::Float32>("turtle_distance", 10);
    pub_vel1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    pub_vel2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    // Services
    teleport_client1 = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
    teleport_client2 = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle2/teleport_absolute");

    ros::spin();
    return 0;
}