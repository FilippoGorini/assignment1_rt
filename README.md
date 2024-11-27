# RT1 Assignment 1
This repository implements two ROS nodes, `UI` and `distance`, which interact with
the `turtlesim` environment for basic controls and distance/boundary management.



## Running the nodes
Ensure you have **ROS (Noetic or compatible)** and the **turtlesim** package installed.
Then, clone this repository into your ROS workspace's `src` folder:
```
cd ~/<your_workspace>/src
git clone https://github.com/FilippoGorini/assignment1_rt.git
```
Build the package:
```
cd ~/<your_workspace>
catkin_make
```
Source ROS and your workspace in the `.bashrc` file:
```
source /opt/ros/noetic/setup.bash  
source ~/<your_workspace>/devel/setup.bash
```
Once this is done, you can proceed to run the nodes:
- Start the ros master with the command `roscore`
- Start the turtlesim node using `rosrun turtlesim turtlesim_node`
- Launch the `UI` node with `rosrun assignment1_rt UI`
- Launch the `distance` node with `rosrun assignment1_rt distance`

You can now use the `UI` node's interface to command the turtles, while the `distance` node perform checks on the distance between the turtles and on the boundary constraints.



## `UI` node - Command-Line Interface for Turtle Control
This ROS node provides a simple command-line interface to control two turtles (`turtle1` and `turtle2`) in the `turtlesim` environment. The user can select a turtle and set its velocity.
To launch it, once the ROS master and the turtlesim node are running, use the command:
```
rosrun assignment1_rt UI
```

### Main Features
1. **Turtle Selection**: Choose between `turtle1` and `turtle2`.
2. **Velocity Input**: Set the chosen turtle's linear (x, y) and angular velocity.
3. **Input Validation**: Ensures valid numeric input for velocities, with error messages for invalid input or for invalid turtle choice.
4. **Turtle Setup**:
   - `turtle1` is teleported to a specific position without leaving a trail (just for looks).
   - `turtle2` is spawned at a set position.
5. **Velocity Publishing**: Sends velocity commands to the selected turtle's `cmd_vel` topic.

### Code Structure
The code mainly revolves around the `main` function, which consists of the following steps:
- Use the `TeleportAbsolute` service to move `turtle1` to the wanted position (this is just for looks, it isn't actually needed).
- Spawn a new turtle, called `turtle2`, using the `Spawn` service.
- Initialize publishers for the `cmd_vel` topics of each turtle to be able to control their velocities.
- Run the main while loop, which asks the user to choose a turtle by entering either 1 or 2 in the terminal.
- During every iteration, once the turtle has been chosen, the user is asked to input the wanted velocities, which are then published on the corresponding topic.

Additionally, checks are performed to:
- ensure that the turtle choice is valid (either 1 or 2).
- ensure that the velocities given by the user are actual numeric values (it was noticed that entering by mistake a letter instead of a number would cause `std::cin` to fail and break the code).

For this last case, a simple `inputNonValid()` function was made, allowing to clear the error and the invalid inputs from the buffer if a failure of `std::cin` is detected.

### Dependencies
- `turtlesim` package
- `geometry_msgs/Twist` message
- `turtlesim/Spawn`, `turtlesim/TeleportAbsolute`, and  `turtlesim/SetPen` services



## `distance` node - Manage Distances and Boundaries
This ROS node subscribes to the `pose` topic of each turtle, computes the euclidean distance between them, and publishes it to a topic called `turtle_distance`. Additionally, the code checks that the distance
between the turtles never gets under a certain predefined threshold, and the same goes for the distance from the window's boundaries.
To launch the node, once the ROS master and the turtlesim node are running, use the command:
```
rosrun assignment1_rt distance
```

### Main Features
1. **Distance Monitoring**: Calculates the distance between `turtle1` and `turtle2`, and triggers a response if they are too close.
2. **Distance Publishing**: Continuously publishes the distance between the turtles.
3. **Stepping Back**: If the turtles are too close, the last moved turtle will step back along its previous path until the distance gets back to a "legal" value.
4. **Boundary Checking**: Monitors the positions of the turtles to ensure they stay within set boundaries. If a turtle goes out of bounds, it will be stopped and teleported back to a valid location.

### Code structure
In this case, the code relies on a shorter main function, where the various subscribers and publishers are initialized. Once this is done, the main loop is executed, this time, using `ros::spin()` (in the other
node we had a while loop calling the `ros::spinOnce()` function instead).
In this case an approach using callbacks was chosen. The most important functions are:
- `updateTurtle1()` & `updateTurtle2()`: These are the callbacks called by the two pose subscribers whenever the corresponding topic is updated. They store the poses for later computation and call the other functions.
- `checkDistance()`: computes the distance, publishes it using `pubDistance()` and, in the case where the distance gets lower than a set threshold, it forces the last moving turtle to step back until it's no longer in the "illegal" area.
- `checkMargins()`: when called, it checks that the position of the turtles does not exceed predefined margins, and if it does then the turtle is stopped and teleported back into the nearest "legal" position.
- `stopTurtle()`: stops the turtle selected by the argument passed to the function.
- `stepBackTurtle()`: publish a velocity command opposite to the last one given to the selected turtle, until it gets back to a distant enough position.

Notice that the stepping back / teleporting back logic is necessary because if we didn't implement it, the turtles would be stuck in the "illegal" zone unless the user sets a very high velocity value to be able to escape.

### Dependencies
- `turtlesim` package
- `geometry_msgs/Twist` and `std_msgs/Float32` messages
- `turtlesim/TeleportAbsolute` service

