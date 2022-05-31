# pid_tuning

## Description
With this ROS package you can tune automatically your PID ROS-based controllers simultaneously, for both one joint at a time or all the joints of your robot at the same time, in an easy way. This ROS package tackles the problem of pid tuning using as approach a Constrained Numerical Optimization Problem (CNOP) solved by bio-inspired algorithms, that are the baselines of this package (ready to use).
You only need a set of joint trajectories and optionally cartesian space trajectories of the robot for a specific task. 

**NOTE**: This package is intended to be used for Gazebo simulations and has not been adapted for real robots at the moment.

## Robots Morphology


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


## Implementation
You can find tutorials and more information of how you can implement this package for your robots here(link).
