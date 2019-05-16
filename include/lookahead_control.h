#ifndef LOOKAHEAD_CONTROL_H_
#define LOOKAHEAD_CONTROL_H_
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
// #include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <sensor_msgs/Imu.h>
#include "ros/ros.h"



class LookaheadControl {
  private:
   // ROS 
   ros::Publisher cmd_vel_pub;
   ros::Subscriber odom_sub;
   ros::Subscriber lookahead_point_sub;
   //  ros::Rate rate(10); // double check this later

   // State variables
   double state_variable_x_, state_variable_y_;
   double state_variable_theta_; 

   // Lookahead point
   // translation along the centerline from the center of the robot
   // note that length is in meters
   double lookahead_point_x_, lookahead_point_y_;
   double relative_time_;
   double time_derivative_x_, time_derivative_y_;

   // Reference Point coordinates
   double reference_point_x_, reference_point_y_;

   // decoupling matrix
   double decoupling_matrix_a_, decoupling_matrix_b_;
   double decoupling_matrix_c_, decoupling_matrix_d_;
   double determinant;
   double decoupling_inverse_[2][2];
  //  double decoupling_matrix_[2][2];

   // linear feedback
   double linear_feedback[2];

   // input 
   double distance_to_goal[2];
   double desired_point;

   // nonlinear feedback 
   double feedback_gains[2];

   //
   double nonlinear_feedback[2]; 


  public:
   // Get the odometry message
   void odomCallback(const nav_msgs::OdometryConstPtr& msg);

   // Get desired point
   void poseCallback(const geometry_msgs::TransformStampedConstPtr& msg);
   
   void getReferencePointCoordinates();
   void getDecouplingInverseMatrix();
   // LookaheadControl Functions
   // waypoint minus current position3
  //  void getDistanceToGoal(); 
   void getLinearFeedback();
   void getNonlinearFeedback(); // robot input
   void publishFeedback();


   // Constructor
   LookaheadControl(ros::NodeHandle& node)
   {
     cmd_vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 100);
     ROS_INFO_STREAM("Adding the subscriber: odometry");
     odom_sub = node.subscribe("odometry", 1000,
                             &LookaheadControl::odomCallback, this);
     lookahead_point_sub = node.subscribe("lookahead_point", 1000,
                             &LookaheadControl::poseCallback, this);
     node.param<double>("lookahead_point_x", lookahead_point_x_, 0.8);
     node.param<double>("lookahead_point_y", lookahead_point_y_, 0.0);
   }

    // Clean up
    ~LookaheadControl() {}
};
#endif