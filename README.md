# pid_tuning

## Description
With this ROS package you can tune automatically your PID ROS-based controllers simultaneously, for both one joint at a time or all the joints of your robot at the same time, in an easy way. This ROS package tackles the problem of pid tuning using as approach a Constrained Numerical Optimization Problem (CNOP) solved by bio-inspired algorithms, that are the baselines of this package (ready to use).
You only need a set of joint trajectories and optionally cartesian space trajectories of the robot for a specific task. 

**NOTE**: This package is intended to be used for Gazebo simulations and has not been adapted for real robots at the moment.

## Robots Morphology
![Screenshot from 2022-05-31 09-26-37](https://user-images.githubusercontent.com/78234785/171197884-c01b5643-e497-4de2-9ef4-20550322f848.png)


## Compatibility with ROS Controllers
This package works for specific [ros_control](http://wiki.ros.org/ros_control). Here is a list of the controllers that you can use with this package:
Markup : 
* [velocity_controllers](https://github.com/ros-controls/ros_controllers/tree/melodic-devel/velocity_controllers/include/velocity_controllers):
  * joint_position_controller.h
* [effort_controllers](https://github.com/ros-controls/ros_controllers/tree/melodic-devel/effort_controllers/include/effort_controllers)
  * joint_group_position_controller.h
  * joint_position_controller.h
  * joint_velocity_controller.h

Markup: ![picture alt]( "Compatibility")

## Installation
Markup:
1. In the terminal, move to the catkin workspace in which you have your robot description package.
 ```
 cd ~/<workspace_name>/src
 ```
2. Clone the github package repository
```
git clone https://github.com/We-R22/pid_tuning.git
```
3. Return to your workspace
```
cd ~/<workspace_name>
```
5. Compile the workspace
```
catkin_make
```
6. Add to the bash
```
source devel/setup.bash
```

## Implementation
You can find tutorials and more information of how you can implement this package for your robots here(link).
