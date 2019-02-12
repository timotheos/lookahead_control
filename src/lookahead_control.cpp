#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

class lookaheadControl {
  public:
    lookaheadControl(/* args */);
    ~lookaheadControl();

  private:

};

lookaheadControl::lookaheadControl(/* args */)
{
}

lookaheadControl::~lookaheadControl()
{
}

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]",
           msg->pose.pose.position.x, 
           msg->pose.pose.position.y, 
           msg->pose.pose.position.z);
  ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]",
           msg->pose.pose.orientation.x,
           msg->pose.pose.orientation.y,
           msg->pose.pose.orientation.z,
           msg->pose.pose.orientation.w);
  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]",
           msg->twist.twist.linear.x,
           msg->twist.twist.angular.z);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "look_ahead_control");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("odometry", 1000, chatterCallback);

  // input odometry 

  // output cmd_vel

  ros::spin();

  return 0;
}


// rostopic pub -1 /RosAria/cmd_vel geometry_msgs/Twist '[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
