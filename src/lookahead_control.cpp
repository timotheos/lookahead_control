#include <ros/ros.h>
#include <lookahead_control.h>
#include "tf/LinearMath/Matrix3x3.h"
#include <tf/transform_datatypes.h>

// class LookaheadControl {
//   private:

//    // Lookahead point
//    // translation along the centerline from the center of the robot
//    // note that length is in meters
//    double lookahead_point_x_, lookahead_point_y_;
//    double relative_time_;
//    double time_derivative_x_, time_derivative_y_;

//    // Reference Point coordinates
//    double reference_point_x_, reference_point_y_;

//    // decoupling matrix
//    double decoupling_matrix_a_, decoupling_matrix_b_;
//    double decoupling_matrix_c_, decoupling_matrix_d_;
//    double determinant;
//    double decoupling_inverse_[2][2];
//   //  double decoupling_matrix_[2][2];

//    // linear feedback
//    double linear_feedback[2];

//    // input 
//    double distance_to_goal[2];
//    double desired_point;

//    // nonlinear feedback 
//    double feedback_gains[2];

//    //
//    double nonlinear_feedback[2]; 


//   public:
//    // Get the odometry message
//    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

//    // Get desired point
//    void poseCallback(const geometry_msgs::TransformStampedConstPtr& msg);
   
//    void getReferencePointCoordinates();
//    void getDecouplingInverseMatrix();
//    // LookaheadControl Functions
//    // waypoint minus current position3
//   //  void getDistanceToGoal(); 
//    void getLinearFeedback();
//    void getNonlinearFeedback(); // robot input
//    void publishFeedback();

//     // Clean up
//     ~LookaheadControl() {}
// };

  // void LookaheadControl::odomCallback(const nav_msgs::OdometryConstPtr& msg)
  // {
  //   // Assign x and y state variables 
  //   state_variable_x_ = msg->pose.pose.position.x;
  //   state_variable_y_ = msg->pose.pose.position.y;

  //   // convert quarternion to rotation
  //   // tf::quaternionMsgToTF(msg->pose.pose.orientation, pose);
  //   state_variable_theta_ = tf::getYaw(msg->pose.pose.orientation);
  // }

  // void LookaheadControl::poseCallback(const geometry_msgs::TransformStampedConstPtr& msg)
  // {
  //   // position
  //   lookahead_point_x_ = msg->translation.x;
  //   lookahead_point_y_ = msg->position.y;
  //   // time
  //   relative_time_ = msg->c

  // }


  // void LookaheadControl::getReferencePointCoordinates()
  // {
  //   reference_point_x_ = state_variable_x_
  //                        + lookahead_point_x_*cos(state_variable_theta_)
  //                        - lookahead_point_y_*sin(state_variable_theta_);
  //   reference_point_y_ = state_variable_y_
  //                        + lookahead_point_x_*sin(state_variable_theta_)
  //                        + lookahead_point_y_*cos(state_variable_theta_);
  // }

  // void LookaheadControl::getLinearFeedback()
  // {
  //   (-reference_point_x_);
  //   (-reference_point_x_);
  // }

  // void LookaheadControl::getNonlinearFeedback()
  // {

  // }



void LookAheadControl::odomCallback(const nav_msgs::Odometry::ConstPtr &odometry)
{
  // pass the odometry message to the state estimates
  pose_x_ = odometry->pose.pose.position.x;
  pose_y_ = odometry->pose.pose.position.y;
  pose_theta_ = tf::getYaw(odometry->pose.pose.orientation); // fix the warning later
  ROS_INFO_STREAM("Theta:" << pose_theta_);

  // run through program
}

void LookAheadControl::targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &target_pose)
{
  target_pose_x_ = target_pose->pose.position.x;
  target_pose_y_ = target_pose->pose.position.y;
}

void LookAheadControl::parameters()
{
  ROS_INFO("Setting parameters.");

  /* maximum rotational speed for the robot <y hat dot>
     pioneer AT is 140 and DX is 300 (rad/s)              */
  pnh_.param<double>("robot_rot_vel", robot_rot_vel_, 140.0);
  ROS_INFO_STREAM("robot_rot_vel: " << robot_rot_vel_ << "rad/s");

  pnh_.param<double>("lookahead_distance_y", lookahead_distance_y_, 0.5);
  ROS_INFO_STREAM("lookahead_distance_y: " << lookahead_distance_y_ << "meters");
  pnh_.param<double>("lookahead_distance_x", lookahead_distance_x_, 0.5);
  ROS_INFO_STREAM("lookahead_distance_x: " << lookahead_distance_x_ << "meters");

  pnh_.param<double>("gain_kp_1", gain_kp_1_, 0.3);
  ROS_INFO_STREAM("gain_kp_1: " << gain_kp_1_);
  pnh_.param<double>("gain_kp_2", gain_kp_2_, 0.3);
  ROS_INFO_STREAM("gain_kp_2: " << gain_kp_2_);
}

void LookAheadControl::getReferencePt()
{
  
} 

void LookAheadControl::spin()
{
  while (ros::ok())
  {
    sub_target_pose_  = pnh_.subscribe("target_pose", 1000,
                            &LookAheadControl::targetPoseCallback, this);
    sub_current_pose_ = pnh_.subscribe("odom", 1000,
                            &LookAheadControl::odomCallback, this);

    ros::spinOnce();
    loop_rate_.sleep();
  }
}

// Constructor
LookAheadControl::LookAheadControl() : pnh_("~"), loop_rate_(10)
{
  ROS_INFO("Initialized node.");
  
  LookAheadControl::parameters();

  pub_cmd_vel_ = pnh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);
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
