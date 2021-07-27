
#include "RGBDExtractorNode.h"

int main (int argc, char ** argv)
{
  ros::init (argc, argv, "ros_cpp_template");
  
  ros::NodeHandle nh;
  RGBDExtractorNode rgbdExtractorNode{nh};

  ros::spin();

  return 0;
}
