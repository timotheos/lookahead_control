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
    ros::Subscriber sub_target_pose_;
    ros::Rate loop_rate_;

      // ROS parameters

      double robot_rot_vel_;

      /* look ahead distance */
      double lookahead_distance_x_;
      double lookahead_distance_y_;

      /* Proportional gains */
      double gain_kp_1_;
      double gain_kp_2_;

    // From robot odometry
    // 1. State Space Variables
    // (output of plant/ input of controller)
    double pose_x_;
    double pose_y_;
    double pose_theta_;  
    // NOTE: odom and map frame irl are same

    // 2. Decoupling matrix
    Eigen::Matrix2d decplg_matrix_;
    Eigen::Matrix2d decplg_inverse_;

    // 3. Output Equations
    double reference_pt_y_; double reference_pt_x_;

    // 4. Linear Feedback
    // required path to follow <y hat> (w_x_rd and w_y_rd)
    // reference for making this later
    // https://answers.ros.org/question/282094/publish-a-path/
    // TODO: Path planning
    // nav_msgs::Path path_desired_;
    // TODO: fix members
    double trajectory_pt_x_;
    double trajectory_pt_y_;
    double time_derivative_x_;
    double time_derivative_y_;
    Eigen::Vector2d feedback_eq_;
    double target_pose_x_;
    double target_pose_y_;

    
    // 5. Velocity Input
    Eigen::Vector2d input_cmd_vel_;

    geometry_msgs::Twist cmd_vel_;

    void odomCallback(const nav_msgs::Odometry &odom);
    void targetPoseCallback(const geometry_msgs::PoseStamped &target_pose);

    void parameters();

    void distanceToGoal();
    void findCmdVel();
    void publishCmdVel(const Eigen::Vector2d &input);

  public:
    // ROS
    ros::NodeHandle nh_;

    void spin();

    // constructor
    LookAheadControl();
    // destructor
    ~LookAheadControl();

};

#endif