#ifndef LOOKAHEAD_CONTROL_H_
#define LOOKAHEAD_CONTROL_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h> // output of pioneer
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h> // input to 
#include <geometry_msgs/PoseStamped.h>

#include <tf/LinearMath/Matrix3x3.h>

// outputs are pointers while inputs are references
class LookAheadControl
{
  protected:
    // ROS node handles
    ros::NodeHandle pnh_; 
    ros::Publisher pub_cmd_vel_;
    ros::Subscriber sub_current_pose_;
      nav_msgs::Odometry odom_;

    ros::Subscriber sub_target_pose_;
      geometry_msgs::PoseStamped point_desired_;
      // required path to follow <y hat> (w_x_rd and w_y_rd)
      // reference for making this later
      // https://answers.ros.org/question/282094/publish-a-path/
      // TODO: Path planning
      // nav_msgs::Path path_desired_;

    // ROS parameters
      /*
          maximum rotational speed for the robot <y hat dot>
          pioneer AT is 140 and DX is 300 (rad/s)
      */
      double robot_rot_vel_;

      /* look ahead distance */
      double lookahead_distance_x_;
      double lookahead_distance_y_;

      /* Proportional gains */
      double gain_kp_1_;
      double gain_kp_2_;

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
    void targetPoseCallback(const geometry_msgs::PoseStamped& target_pose);
    void setParameters();
    void spin();

    // odometry and cmd_vel messages are passed
    LookAheadControl(geometry_msgs::Twist cmd_vel, nav_msgs::Odometry odom);
    // overloaded constructor
    LookAheadControl();
    // destructor
    ~LookAheadControl();

};

#endif