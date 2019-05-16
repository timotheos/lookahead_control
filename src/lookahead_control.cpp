#include "lookahead_control.h"

  void LookaheadControl::odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    // Assign x and y state variables 
    state_variable_x_ = msg->pose.pose.position.x;
    state_variable_y_ = msg->pose.pose.position.y;

    // convert quarternion to rotation
    // tf::quaternionMsgToTF(msg->pose.pose.orientation, pose);
    state_variable_theta_ = tf::getYaw(msg->pose.pose.orientation);
  }

  void LookaheadControl::poseCallback(const geometry_msgs::TransformStampedConstPtr& msg)
  {
    // position
    lookahead_point_x_ = msg->translation.x;
    lookahead_point_y_ = msg->position.y;
    // time
    relative_time_ = msg->c

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
    decoupling_matrix_a_ = cos(state_variable_theta_);
    decoupling_matrix_b_ = - state_variable_y_ * cos(state_variable_theta_) 
                              - state_variable_x_ * sin(state_variable_theta_);
    decoupling_matrix_c_ = sin(state_variable_theta_);
    decoupling_matrix_d_ = - state_variable_y_ * sin(state_variable_theta_) 
                              + state_variable_x_ * cos(state_variable_theta_);
    
    determinant = decoupling_matrix_a_ * decoupling_matrix_d_
                  - decoupling_matrix_b_ * decoupling_matrix_c_;

    decoupling_inverse_[0][0] =  decoupling_matrix_d_ / determinant;
    decoupling_inverse_[0][1] = -decoupling_matrix_b_ / determinant;
    decoupling_inverse_[1][0] = -decoupling_matrix_c_ / determinant;
    decoupling_inverse_[1][1] =  decoupling_matrix_a_ / determinant;
  }

  void LookaheadControl::getLinearFeedback()
  {
    (-reference_point_x_);
    (-reference_point_x_);
  }

  void LookaheadControl::getNonlinearFeedback()
  {

  }

  void LookaheadControl::publishFeedback()
  { 
    // obtain current position
    LookaheadControl::getReferencePointCoordinates();

    // get inverse decoupling matrix
    LookaheadControl::getDecouplingInverseMatrix();

    // get linear feedback v
    LookaheadControl::getLinearFeedback();

    // calculate for nonlinear feedback 
    LookaheadControl::getNonlinearFeedback();

    ros::spinOnce();
    // rate.sleep;
  }

int main(int argc, char **argv) {
  ros::init(argc, argv, "lookahead_control");
  ros::NodeHandle nh;
  LookaheadControl lookaheadControl(nh);
  while(ros::ok()){
    lookaheadControl.getNonlinearFeedback();
  }
  
  

  return 0;
}


// rostopic pub -1 /RosAria/cmd_vel geometry_msgs/Twist '[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
