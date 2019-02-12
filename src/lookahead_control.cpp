#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <Aria/Aria.h>

class lookaheadControl
{
private:
  /* data */
public:
  lookaheadControl(/* args */);
  ~lookaheadControl();
};

lookaheadControl::lookaheadControl(/* args */)
{
}

lookaheadControl::~lookaheadControl()
{
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "look_ahead_control");

  ros::NodeHandle nh;

  ros::Subscriber sub
  return 0;
}


// rostopic pub -1 /RosAria/cmd_vel geometry_msgs/Twist '[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
