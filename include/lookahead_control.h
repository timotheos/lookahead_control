#ifndef LOOKAHEAD_CONTROL_H_
#define LOOKAHEAD_CONTROL_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h> // output of pioneer
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h> // input to 
#include <geometry_msgs/Pose.h>

#include <tf/LinearMath/Matrix3x3.h>

// outputs are pointers while inputs are references
class LookAheadControl
{
  protected:
    // ROS
    // private node handle
    ros::NodeHandle pnh_; 
    ros::Publisher pub_cmd_vel_;
    ros::Subscriber sub_pos_;

    // parameters
    // required path to follow <y hat> (w_x_rd and w_y_rd)
    nav_msgs::Path path_desired_; // 
    // reference for making this later
    // https://answers.ros.org/question/282094/publish-a-path/

    // maximum rotational speed for the robot <y hat dot>
    // pioneer at is 140 and dx is 300
    double rot_vel_;
    double matrix_here_[];






    // robot output (Odometry)

    // robot input (cmd_vel)

    // 1. State Space Variables (output)
    geometry_msgs::Pose robot_state;
    double pose_x_;
    double pose_y_;
    double pose_theta_;  
    

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
    void odomCallback(const nav_msgs::Odometry& odom);
    void spin();

    // odometry and cmd_vel messages are passed
    LookAheadControl(geometry_msgs::Twist cmd_vel, nav_msgs::Odometry odom);
    // overloaded constructor
    LookAheadControl();
    // destructor
    ~LookAheadControl();

};

#endif