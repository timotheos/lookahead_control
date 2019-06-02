#ifndef LOOKAHEAD_CONTROL_H_
#define LOOKAHEAD_CONTROL_H_

#include <ros/ros.h>
#include <Eigen/Dense>

#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h> // output of pioneer
#include <geometry_msgs/Twist.h> // input to 
#include <geometry_msgs/Pose.h>

#include <tf/transform_datatypes.h>


// outputs are pointers while inputs are references
class LookAheadControl
{
  protected:
    // ROS
    // private node handle
    ros::NodeHandle pnh_; 
    ros::Publisher pub_cmd_vel_;
    ros::Subscriber sub_pos_;

    // output (Odometry)

    // input (cmd_vel)

    // 1. State Space Variables (output)
    geometry_msgs::Pose robot_state;
    double& pose_x_;
    double& pose_y_;
    double& pose_theta_;  
    

    // 2. Decoupling matrix
    // 2.a determinant
    std_msgs::Float64 decplg_det;
    // 2.b matrix
    // typedef Eigen::Array<std_msgs::Float64, 2, 2> Eigen::Array22d64;
    Eigen::Array22d decplg_matrix;
    Eigen::Array22d decplg_inverse;

    // 3. Output Equations

    // 4. Linear Feedback

    // 5. Velocity Input
    geometry_msgs::Twist cmd_vel_;


  public:
    // ROS
    ros::NodeHandle nh_;
    void odomCallback(const nav_msgs::Odometry&);
    void spin();

    // odometry and cmd_vel messages are passed
    LookAheadControl(geometry_msgs::Twist cmd_vel, nav_msgs::Odometry odom);
    // overloaded constructor
    LookAheadControl();
    // destructor
    ~LookAheadControl();

};

#endif