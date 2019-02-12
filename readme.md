# Look-ahead control

Look-ahead control is feedback controller for nonholonomic mobile robot systems. The 
mobile robot follows a predefined point a set distance from the center of the robot.
The look-ahead controller is designed by using input-output linearization and nonholonomic 
constraint equations.

## Subscribed Topics
* `odometry`

## Published Topics
* `cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))  
  