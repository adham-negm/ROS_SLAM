

#include "ros_arduino_base/ros_arduino_base.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_arduino_base_node");
  ros::NodeHandle nh, pnh("~");
  ROSArduinoBase ros_arduino_base(nh, pnh);

  ros::spin();

  return 0;
}
