#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
//#include <tf2_ros/transform_broadcaster.h>
//#include <tf2_ros/transform_broadcaster.h>
//#include <tf2/LinearMath/Quaternion.h>
// #include <sensor_msgs/Imu.h>

struct StateVariables {
  double x_q;
  double y_q;
  double theta_q;
};

class LookaheadControl {
  private:
    StateVariables state_var;

  public:
    double LOOKAHEAD_POINT;

    void odomCallback();
    void getStateVariables();
    void getDecouplingMatrix();
}

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Data holder
    geometry_msgs::Quaternion msg_holder;
    
    // Assign x and y state variables 
    x_q = msg->pose.pose.position.x;
    y_q = msg->pose.pose.position.y;

    // convert quarternion to rotation
    // state_var[0] 
    // tf::getYaw()
    // q.pose.pose.orientation
  }

int main(int argc, char **argv) {
  ros::init(argc, argv, "lookahead_control");
  ros::NodeHandle nh;

  // Obtain the state of the robot
  ros::Subscriber output = nh.subscribe("odometry", 200, odomCallback);
  nh.param<std_msg::Float64>("lookahead_distance", lookahead_distance, 1000);
  
  
  
  // input odometry 

  // output cmd_vel
  ros::Publisher command_feedback =
    nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);


  ros::spin();

  return 0;
}


// rostopic pub -1 /RosAria/cmd_vel geometry_msgs/Twist '[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
