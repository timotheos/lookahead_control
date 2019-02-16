#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
// #include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <sensor_msgs/Imu.h>

struct StateVariables {
  double x_q;
  double y_q;
  double theta_q;

  StateVariables() {
    x_q = 0; y_q = 0; theta_q = 0;
  }
};

class LookaheadControl {
  private:
    // ROS 
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Publisher cmd_vel_pub;
    double output_equation[2];
    // double x_reference
    double decoupling_matrix[2][2];

    // LookaheadControl Functions
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void getCommandVelocity(); // robot input

  public:
    double lookahead_distance;
    double lookahead_point;     // meters to point waypoint
    StateVariables state_var;

    // Constructor
    LookaheadControl() {
      lookahead_distance = 0.9; // in meters
      cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
      odom_sub = nh.subscribe("odometry", 1000, &LookaheadControl::odomCallback, this);
      nh.param<double>("lookahead_distance", lookahead_distance, 0.8);
    }

    // Clean up
    ~LookaheadControl() {}
};

  void LookaheadControl::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Data holder
    geometry_msgs::Quaternion msg_holder;
    
    // Assign x and y state variables 
    state_var.x_q = msg->pose.pose.position.x;
    state_var.y_q = msg->pose.pose.position.y;

    // convert quarternion to rotation
    // tf::quaternionMsgToTF(msg->pose.pose.orientation, pose);
    state_var.theta_q = tf::getYaw(msg->pose.pose.orientation);
  }

  void LookaheadControl::getCommandVelocity() {
    void calculate();
  }

int main(int argc, char **argv) {
  ros::init(argc, argv, "lookahead_control");
  LookaheadControl lookaheadControl;

  // subscribe to odometry and 
  // lookaheadControl.

  ros::spin();

  return 0;
}


// rostopic pub -1 /RosAria/cmd_vel geometry_msgs/Twist '[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
