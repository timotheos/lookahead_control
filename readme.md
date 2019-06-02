# Look-ahead control

Look-ahead control is feedback controller for nonholonomic mobile robot systems. The 
mobile robot follows a predefined point a set distance from the center of the robot.
The look-ahead controller is designed by using input-output linearization and nonholonomic 
constraint equations.
[Look-Ahead Control](./diagram-lookahead_control.png)

## Subscribed Topics
* `odometry` ([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html))
* `point_desired` ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))

## Published Topics
* `cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))  

## ROS Parameters
* `lookahead_distance_x` (float, default: 0.5 meters) - define distance with respect to mobile robot
* `lookahead_distance_y` (float, default: 0.5 meters) - define distance with respect to mobile robot
* `robot_rot_vel` (float, default: 140 rad/s) - maximum angular speed of the robot
* `gains_kp_1` (float, default: 0.5)
* `gains_kp_2` (float, default: 0.5)










