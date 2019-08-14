#include <ros/ros.h>
#include <lookahead_control.h>
#include "tf/LinearMath/Matrix3x3.h"
#include <tf/transform_datatypes.h>

void LookAheadControl::parameters()
{
  ROS_INFO("Setting parameters.");

  /* maximum rotational speed for the robot <y hat dot>
     pioneer AT is 140 and DX is 300 (deg/s)              */
  pnh_.param<double>("robot_rot_vel", robot_rot_vel_, 2.443);
  ROS_INFO_STREAM("robot_rot_vel: " << robot_rot_vel_ << "rad/s");

  pnh_.param<double>("lookahead_distance_y", lookahead_distance_y_, 0.0);
  ROS_INFO_STREAM("lookahead_distance_y: " << lookahead_distance_y_ << "meters");
  pnh_.param<double>("lookahead_distance_x", lookahead_distance_x_, 0.5);
  ROS_INFO_STREAM("lookahead_distance_x: " << lookahead_distance_x_ << "meters");

  pnh_.param<double>("gain_kp_1", gain_kp_1_, 0.3);
  ROS_INFO_STREAM("gain_kp_1: " << gain_kp_1_);
  pnh_.param<double>("gain_kp_2", gain_kp_2_, 0.3);
  ROS_INFO_STREAM("gain_kp_2: " << gain_kp_2_);
}


void LookAheadControl::odomCallback(const nav_msgs::Odometry &odometry)
{
  // pass the odometry message to the state estimates
  pose_x_ = odometry.pose.pose.position.x;
  pose_y_ = odometry.pose.pose.position.y;
  pose_theta_ = tf::getYaw(odometry.pose.pose.orientation); // fix the warning later
  ROS_INFO_STREAM("Current Pos (x, y):" << pose_x_ << ", " << pose_y_);
  ROS_INFO_STREAM("Current Yaw (Theta):" << pose_theta_);

  // run through program
}

void LookAheadControl::targetPoseCallback(const geometry_msgs::PoseStamped &target_pose)
{
  target_pose_x_ = target_pose.pose.position.x;
  target_pose_y_ = target_pose.pose.position.y;
  ROS_INFO_STREAM("Target Pos (x, y):" << target_pose_x_ << ", " << target_pose_y_);
}

void LookAheadControl::publishCmdVel(const Eigen::Vector2d &input_cmd_vel)
{
  double diff_x_;
  double diff_y_;

  diff_x_ = abs(target_pose_x_ - trajectory_pt_x_);
  diff_y_ = abs(target_pose_y_ - trajectory_pt_y_);  
  ROS_INFO_STREAM("difference x, y: " << diff_x_ << ", " << diff_y_);


  if (diff_x_ <= 0.10  && diff_y_ <= 0.10)
    {
      cmd_vel_.linear.x = 0;
      cmd_vel_.angular.z = 0;
    }
  else
    {
      cmd_vel_.linear.x = input_cmd_vel(0);
      cmd_vel_.angular.z = input_cmd_vel(1);
    }
  ROS_INFO_STREAM("command_vel:" << cmd_vel_.linear.x << ", " << cmd_vel_.angular.z);
  pub_cmd_vel_.publish(cmd_vel_);
}

void LookAheadControl::distanceToGoal()
{
  // TODO: fix members
  double diff_x_;
  double diff_y_;
  double diff_alpha_;
  
  trajectory_pt_x_ = pose_x_ + lookahead_distance_x_;
  trajectory_pt_y_ = pose_y_ + lookahead_distance_y_;
  
  diff_x_ = target_pose_x_ - trajectory_pt_x_;
  diff_y_ = target_pose_y_ - trajectory_pt_y_;  

  diff_alpha_ = atan2(diff_y_,diff_x_);
  ROS_INFO_STREAM("alpha: " << diff_alpha_);

  time_derivative_x_ = robot_rot_vel_ * (cos(diff_alpha_)); //- pose_theta_);
  time_derivative_y_ = robot_rot_vel_ * (sin(diff_alpha_)); //- pose_theta_);
};

void LookAheadControl::findCmdVel()
{ 
  // decoupling matrix
  decplg_matrix_(0,0) = cos(pose_theta_);
  decplg_matrix_(0,1) = -lookahead_distance_y_ * cos(pose_theta_)
                        - lookahead_distance_x_ * sin(pose_theta_);
  decplg_matrix_(1,0) = sin(pose_theta_);
  decplg_matrix_(1,1) = -lookahead_distance_y_ * sin(pose_theta_)
                        + lookahead_distance_x_ * cos(pose_theta_);
  decplg_inverse_     = decplg_matrix_.inverse();

  // output equations
  reference_pt_x_ = pose_x_ + lookahead_distance_x_ * cos(pose_theta_) - lookahead_distance_y_ * sin(pose_theta_);
  reference_pt_y_ = pose_y_ + lookahead_distance_x_ * sin(pose_theta_) + lookahead_distance_y_ * cos(pose_theta_);
  
  distanceToGoal();

  // linear feedback
  feedback_eq_(0) = time_derivative_x_ + gain_kp_1_ * (trajectory_pt_x_ - reference_pt_x_);
  feedback_eq_(1) = time_derivative_y_ + gain_kp_2_ * (trajectory_pt_y_ - reference_pt_y_);

  ROS_INFO_STREAM("feedback" << feedback_eq_(0) << ", " << feedback_eq_(1));

  input_cmd_vel_ = decplg_inverse_*feedback_eq_;

  publishCmdVel(input_cmd_vel_);
}

void LookAheadControl::spin()
{
  while (ros::ok())
  {
    ros::spinOnce();

    findCmdVel();
    
    loop_rate_.sleep();
  }
}

// Constructor
LookAheadControl::LookAheadControl() : pnh_("~"), loop_rate_(10)
{
  ROS_INFO("Initialized node.");
  
  LookAheadControl::parameters();

  sub_target_pose_  = pnh_.subscribe("target_pose", 1000,
                            &LookAheadControl::targetPoseCallback, this);
  sub_current_pose_ = pnh_.subscribe("odom", 1000,
                            &LookAheadControl::odomCallback, this);
  pub_cmd_vel_ = pnh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  ROS_INFO("Publishing cmd_vel");
}

LookAheadControl::~LookAheadControl() 
{

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lookahead_control");

  LookAheadControl node;

  node.spin();
  
  return 1;
}


// rostopic pub -1 /RosAria/cmd_vel geometry_msgs/Twist '[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
