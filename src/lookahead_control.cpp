#include "ros/ros.h"
// #include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
// #include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <sensor_msgs/Imu.h>

class LookaheadControl {
  private:
   // ROS 
   ros::Publisher cmd_vel_pub;
   ros::Subscriber odom_sub;
   //  ros::Rate rate(10); // double check this later

   // State variables
   double state_variable_x_, state_variable_y_;
   double state_variable_theta_; 
   //  double state_variables_[3];

   // Lookahead point
   // translation along the centerline from the center of the robot
   // note that length is in meters
   double lookahead_point_x_, lookahead_point_y_;

   // Reference Point coordinates
   double reference_point_x_, reference_point_y_;

   // decoupling matrix
   double decoupling_inverse_[2][2];
   double decoupling_matrix_[2][2];
   double& a_ = decoupling_matrix_[0][0];
   double& b_ = decoupling_matrix_[0][1];
   double& c_ = decoupling_matrix_[1][0];
   double& d_ = decoupling_matrix_[1][1];
   double determinant;
   
   // nonlinear feedback 
   double feedback_gains[2];
   double distance_to_goal[2];
   double desired_point;
   double nonlinear_feedback[2]; 


  public:
   // Get the odometry message
   void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
   
   void getReferencePointCoordinates();
   void getDecouplingInverseMatrix();
   // LookaheadControl Functions
   // waypoint minus current position3
   // void getDistanceToGoal(const StateVariables& current, const ); 
   void getNonlinearFeedback(); // robot input


   // Constructor
   LookaheadControl(ros::NodeHandle& node)
   {
     cmd_vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 100);
     ROS_INFO_STREAM("Adding the subscriber: odometry");
     odom_sub = node.subscribe("odometry", 1000,
                             &LookaheadControl::odomCallback, this);
     node.param<double>("lookahead_point_x", lookahead_point_x_, 0.8);
     node.param<double>("lookahead_point_y", lookahead_point_y_, 0.0);
   }

    // Clean up
    ~LookaheadControl() {}
};

  void LookaheadControl::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    // Assign x and y state variables 
    state_variable_x_ = msg->pose.pose.position.x;
    state_variable_y_ = msg->pose.pose.position.y;

    // convert quarternion to rotation
    // tf::quaternionMsgToTF(msg->pose.pose.orientation, pose);
    state_variable_theta_ = tf::getYaw(msg->pose.pose.orientation);
  }

  void LookaheadControl::getReferencePointCoordinates()
  {
    reference_point_x_ = state_variable_x_
                         + lookahead_point_x_*cos(state_variable_theta_)
                         - lookahead_point_y_*sin(state_variable_theta_);
    reference_point_y_ = state_variable_y_
                         + lookahead_point_x_*sin(state_variable_theta_)
                         + lookahead_point_y_*cos(state_variable_theta_);
  }

  void LookaheadControl::getDecouplingInverseMatrix()
  {
    a_ = cos(state_variable_theta_);
    b_ = - state_variable_y_ * cos(state_variable_theta_) 
                              - state_variable_x_ * sin(state_variable_theta_);
    c_ = sin(state_variable_theta_);
    d_ = - state_variable_y_ * sin(state_variable_theta_) 
                              + state_variable_x_ * cos(state_variable_theta_);
    
    determinant = a_*d_- b_*c_;

    decoupling_inverse_[0][0] =  d_ / determinant;
    decoupling_inverse_[0][1] = -b_ / determinant;
    decoupling_inverse_[1][0] = -c_ / determinant;
    decoupling_inverse_[1][1] =  a_ / determinant;
  }

  void LookaheadControl::getNonlinearFeedback()
  { 
    while(ros::ok())
    {
      // obtain current position
      LookaheadControl::getReferencePointCoordinates();

      // get inverse decoupling matrix
      LookaheadControl::getDecouplingInverseMatrix();

      // get linear feedback v

      // calculate for nonlinear feedback 

      ros::spinOnce();
      // rate.sleep;
    }    
  }

int main(int argc, char **argv) {
  ros::init(argc, argv, "lookahead_control");
  ros::NodeHandle nh;
  LookaheadControl lookaheadControl(nh);
  lookaheadControl.getNonlinearFeedback();

  return 0;
}


// rostopic pub -1 /RosAria/cmd_vel geometry_msgs/Twist '[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
