#include "ros/ros.h"
#include "turtlesim/Spawn.h"                    // Include the Spawn service
#include "turtlesim/TeleportAbsolute.h"         // Include the teleport absolute service to change turtle1 position
#include "turtlesim/SetPen.h"        
#include "geometry_msgs/Twist.h"                // Include Twist message structure


// This function checks if the user input is non valid by checking if the cin function failed
bool inputNonValid()
{
    if (std::cin.fail())
    {
        std::cout << "\nINVALID INPUT!: input velocity must be a number\n";
        std::cin.clear();                                                       // Clear cin error
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');     // Needed to clear all invalid characters in the buffer until \n
        return true;
    }
    return false;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "UI_node");
    ros::NodeHandle nh;

    // Disable pen of turtle1 just to not leave a trail when teleporting turtle1 
    ros::ServiceClient pen1_client = nh.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
    turtlesim::SetPen pen1_srv;
    pen1_srv.request.off = 1;  
    pen1_client.call(pen1_srv);

    // Moving the turtle1 to another starting position instead of the center (Not needed, just for looks)
    ros::ServiceClient teleport1_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
    turtlesim::TeleportAbsolute teleport1_srv;     
    teleport1_srv.request.x = 4.0;  
    teleport1_srv.request.y = 5.5; 
    teleport1_srv.request.theta = 1.5708;  
    teleport1_client.waitForExistence();     
    teleport1_client.call(teleport1_srv);

    // reenable trail with original trail color and size
    pen1_srv.request.r = 0xb3; 
    pen1_srv.request.g = 0xb8;  
    pen1_srv.request.b = 0xff;
    pen1_srv.request.width = 3;
    pen1_srv.request.off = 0;          
    pen1_client.call(pen1_srv);

    // Spawning the new turtle:
    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("/spawn"); // Creates a spawn client
    turtlesim::Spawn spawn_srv;                                                     // Creates a request to spawn a new turtle
    spawn_srv.request.x = 7.0;                                                       
    spawn_srv.request.y = 5.5;         
    spawn_srv.request.theta = 1.5708;            
    spawn_srv.request.name = "turtle2";  
    spawn_client.waitForExistence(); 
    spawn_client.call(spawn_srv);

    // Add publishers on the corresponding cmd_vel topics, using a twist message structure
    ros::Publisher pub_vel1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Publisher pub_vel2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    // Create velocity messages variables for both turtles
    geometry_msgs::Twist vel1;
    geometry_msgs::Twist vel2;

    // Start the loop that checks for user commands and publish on cmd_vel topics
    while (ros::ok())
    {
        ros::spinOnce();  
        std::string turtle_choice;
        std::cout << "\nEnter 1 or 2 to select which turtle to control (turtle1 or turtle2): ";
        std::cin >> turtle_choice;

        if (turtle_choice == "1")
        {
            std::cout << "turtle1 x velocity:";
            std::cin >> vel1.linear.x;
            std::cout << "turtle1 y velocity:";
            std::cin >> vel1.linear.y;
            std::cout << "turtle1 angular velocity:";
            std::cin >> vel1.angular.z;
            if (inputNonValid())                // if the input is invalid, do not publish the command and restart the loop
                continue;
            pub_vel1.publish(vel1);
        }

        if (turtle_choice == "2")
        {
            std::cout << "turtle2 x velocity:";
            std::cin >> vel2.linear.x;
            std::cout << "turtle2 y velocity:";
            std::cin >> vel2.linear.y;
            std::cout << "turtle2 angular velocity:";
            std::cin >> vel2.angular.z;
            if (inputNonValid())                // if the input is invalid, do not publish the command and restart the loop
                continue;
            pub_vel2.publish(vel2);
        }
        
        if (turtle_choice != "1" && turtle_choice != "2")
        {
            std::cout << "\nINVALID INPUT!: you must enter either 1 or 2\n";
        } 
    }
    return 0;
}
