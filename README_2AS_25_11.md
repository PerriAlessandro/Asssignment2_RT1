__Research Track 1  -  Second Assignment__ <img src="https://raw.githubusercontent.com/jmnote/z-icons/master/svg/python.svg" width="30" height="30">
================================

This assignment is based on a simple robot simulator developed using ROS (Robot Operating System), an open-source, meta-operating system designed for controlling a robot.


__Aim of the project__
----------------------
The project aimed to implement a ROS package in which we had to manage the behavior of a robot. Specifically, the robot must be able to lap around the circuit and receive commands from the user via keyboard input. Here's a list of commands (e.g. keyboard keys) the user can use to control the robot:
- `a`: increases the speed of the robot, every time 'a' key is pressed __the velocity is doubled up__;
- `d`: decreases the speed of the robot, every time 'd' key is pressed __the velocity is halved__;
- `r`: __resets the position__ of the robot and sets the velocity to the original one (e.g. equals to 1).
- `q`: __quits__ the nodes in the current package(additional).

The circuit is a simple _.png_ file that represents the real Monza's circuit, here's a screenshot of what the user will see once he runs the `/stageros` node:
<p align="center">
	<img src="https://github.com/PerriAlessandro/second_assignment/blob/master/images%20and%20videos/circuit.jpeg" height=817 width=1035>
</p>




__Running the simulator__
----------------------

The simulator requires _ROS Noetic_ system. Once downloaded, open a terminal and move to the workspace directory, then run this command to build the workspace:

```bash
$ catkin_make #properly build the workspace
```
Now you have to launch all the nodes, here's a list of the needed nodes to properly run the project:
- `Master Node`: provides naming and registration to the rest of the nodes in the ROS system
- `stage_ros`: The stage_ros node wraps the Stage 2-D multi-robot simulator, via libstage. Stage simulates a world as defined in a .world file. This file tells stage everything about the world, from obstacles (usually represented via a bitmap to be used as a kind of background), to robots and other objects.
- `robot_controller`: implements the logic of the navigation inside the circuit and provides a service that changes the speed of the robot.
- `UI_node`: User Interface node, used for receiving commands via keyboard input.


First of all, you need to launch the master node:
```bash
$ roscore #launch the master node
```
Open a new terminal, then run the node __stageros__:
```bash
$ rosrun stage_ros stageros $(rospack find second_assignment)/world/my_world.world #run stageros node
```
At this point a window with the circuit should appear, after opening a new terminal again run the node __robot_controller_node__:
```bash
$ rosrun second_assignment robot_controller_node  #run robot_controller_node node
```
The robot should start moving, open a new terminal again, then run the last node (__UI_node__):
```bash
$ rosrun second_assignment UI_node  #run UI_node node
```

__Nodes Implemented__
----------------------

In the _src_ folder of the package you will find two _.cpp_ files, __controller.cpp__ and __UI.cpp__ whose executables correspond to the two nodes I mentioned before, __robot_controller_node__ and __UI_node__. 
### __robot_controller_node__ ###
This node is the one that allows the robot to lap around the circuit. To do that, it makes a subscription to `/base_scan` topic, published by `stageros` node, whose message type is `sensor_msgs/LaserScan.msg`. In our case, we're interested to the __ranges[]__ Vector that contains, in the i-th position, the distance from the wall of the i-th laser (721 elements) in a range of 180 degrees. As you can imagine, these values are key elements for the robot's movement as they allow it to be aware of the obstacles surrounding it and consequently move in the appropriate direction. For this reason, the logic with which the robot decides to move is implemented directly in the callback function (__driveCallback__) that is invoked every time a new message of this type is published. This function also implements a _Publisher_ for the `/cmd_vel` topic, in which you can publish messages of type `geometry_msgs/Twist.msg` to make the robot move. The fields of this message consist of six _float32_ variables representing _linear_ and _angular_ velocity along the three axis _x_,_y_ and _z_.
This node also implements a server for the service that manages the user inputs designed for changing the velocity (e.g. 'a' and 'd' keys, respectively for accelerating and decelerating). Every time user types one of those two keys from the shell where UI_node is running, it makes a request for the `/changeVel` service. The structure of this service is:
- Request: char input
- Response: float32 multiplier

If the input corresponds to the char ‘a’, the response will be such as to double the speed, if the input corresponds to the char ‘d’, the response will be such as to halve the speed

#### driveCallback ####
As already explained before, this function retrieves information from `/base_scan` topic and publishes to `/cmd_vel` topic to make the robot move. 
The `LaserScan.msg` contains a lot of information, but the most important one is __ranges[]__ Vector from which you are able to get the distances from the wall for each of 721 lasers (i-th laser corresponds to i-th position in the Vector). 
To properly drive the robot around the circuit, the values of that Vector I used are related to the lateral and frontal distances:
- right distances: ca. 120 values (from index 0 to 120 of ranges[]) --> ca. 30 degrees
- frontal distances: ca. 60 values (from index 330 to 390 of ranges[]) --> ca. 15 degrees
- left distances: ca. 120 values (from index 600 to 720 of ranges[]) --> ca. 30 degrees

From each of those sub-arrays I found the lowest value and then I started comparing them to implement the logic that makes the robot move. Here you can find a flowchart concerning the logic I have used:

<p align="center">
	<img src="https://github.com/PerriAlessandro/second_assignment/blob/master/images%20and%20videos/driveCallback_flowchart.png" height=863 width=1597>
</p>

The result seems satisfactory, i created a short video that shows the movement of the robot in the hardest corner of the circuit:

<p align="center">
	<img src="https://github.com/PerriAlessandro/second_assignment/blob/master/images%20and%20videos/corner.mp4" height=698 width=864>
</p>

### __UI_node__ ###

This node is the executable of _UI.cpp_ file, it is used for receiving inputs from the keyboard, you can find a list of them [here](link). The code related to this node is actually pretty simple, the program constantly waits for user input and when a proper key is pressed, clients make a request for the service used to handle that command. More precisely, it has been sufficient to create two clients that make use of these two services, respectively:
- `/changeVel`: changes the velocity of the robot (server is located in __robot_controller_node__), __keys__: `a`, `d`, `r`;
- `/reset_positions`: resets the position of the robot, __keys__: `r`;

As already explained in __robot_controller_node__ section, `a` and `d` keys will make a request such as to double or halve the velocity, `r` key resets the position by sending a request to `/reset_positions` but it also sends another request to `/changeVel` service such as the velocity will be reset to the original value (equals to 1).

The last valid key that user can press is the `q` one and it is just used for closing the node.

__rqt_graph command__
----------------------
The `rqt_graph` command provides a GUI plugin for visualizing the ROS computation graph. It is useful to have an immediate comprehension of how nodes and related topics work together. In this case, the resulting graph is fairly simple because there are only three nodes (represented as elliptical boxes) and two topics (represented as rectangular boxes).
To visualize the graph, you only need to run:
```bash
$ rqt_graph 
```
__NOTE__: this command works properly only if all nodes are running

<p align="center">
 <img src="https://github.com/PerriAlessandro/second_assignment/blob/master/images%20and%20videos/rqt_graph.jpeg" height=555 width=1831>
</p>

As you can see, using this command you are not able to see what `UI_node` node actually does. That is because it doesn't provide the graphic visualization of services. For this reason I modified the resulting graph by adding the services that have been used, here's the result:
<p align="center">
 <img src="https://github.com/PerriAlessandro/second_assignment/blob/master/images%20and%20videos/rqt_graph.jpeg" height=555 width=1831>
</p>














 

