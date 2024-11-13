#include "ros/ros.h"
#include "turtlesim/Spawn.h"                    // Include the Spawn service
#include "turtlesim/TeleportAbsolute.h"         // Include the teleport absolute service to change turtle1 position
#include "turtlesim/SetPen.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "UI_node");
    ros::NodeHandle nh;

    // just to not leave a trail when teleporting turtle1
    ros::ServiceClient pen_client = nh.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
    turtlesim::SetPen pen_srv;
    pen_srv.request.off = 1;  
    pen_client.call(pen_srv);

    // Moving the turtle1 to another starting position instead of the center (Not needed, just for looks)
    ros::ServiceClient teleport_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
    turtlesim::TeleportAbsolute teleport_srv;     
    teleport_srv.request.x = 4.0;  
    teleport_srv.request.y = 5.5; 
    teleport_srv.request.theta = 1.5708; 
    teleport_client.call(teleport_srv);

    pen_srv.request.off = 0;                    // reenables the trail
    pen_client.call(pen_srv);

    // Spawning the new turtle:
    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("/spawn"); // Creates a spawn client
    turtlesim::Spawn spawn_srv;                                                     // Creates a request to spawn a new turtle
    spawn_srv.request.x = 6.5;           
    spawn_srv.request.y = 5.5;         
    spawn_srv.request.theta = 1.5708;      
    spawn_srv.request.name = "turtle2"; 
    spawn_client.call(spawn_srv);

    return 0;
}
