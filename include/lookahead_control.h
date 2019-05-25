#ifndef LOOKAHEAD_CONTROL_H_
#define LOOKAHEAD_CONTROL_H_

#include "ros/ros.h"
#include <Eigen/Dense>

#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>

#include <tf/transform_datatypes.h>

class LookAheadControl
{
  private:
    // ROS
    // output (Odometry)

    // input (cmd_vel)

    // 1. State Space Variables (output)
    std_msgs::Float64 pose_x_;
    std_msgs::Float64 pose_y_;
    std_msgs::Float64 pose_theta_;

    // 2. Decoupling matrix
    // 2.a determinant
    std_msgs::Float64 decplg_det;
    // 2.b matrix
    // typedef Eigen::Array<std_msgs::Float64, 2, 2> Eigen::Array22d64;
    Eigen::Array22d decplg_matrix;
    Eigen::Array22d decplg_inverse;

    // 3. Output Equations

    // 4. Linear Feedback


  public:
    // odometry and cmd_vel messages are passed
    LookAheadControl(geometry_msgs::TwistStamped cmd_vel, nav_msgs::Odometry odom);
    // overloaded constructor
    LookAheadControl();
    // destructor
    ~LookAheadControl();

};

#endif