#include "SoiamNode.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "boost/thread.hpp"
#include <signal.h>




// githubtest
// this global var is used in getMS(ros::Time t) to convert to a consistent integer timestamp used internally pretty much everywhere.
// kind of an artifact from Windows-Version, where only that was available / used.
unsigned int ros_header_timestamp_base = 0;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "soiam");

  ROS_INFO("Started soiam Node.");

  SoiamNode soiamNode;

  //dynamic_reconfigure::Server<tum_ardrone::AutopilotParamsConfig> srv;
  //dynamic_reconfigure::Server<tum_ardrone::AutopilotParamsConfig>::CallbackType f;
  //f = boost::bind(&ControlNode::dynConfCb, &controlNode, _1, _2);
  //srv.setCallback(f);

  soiamNode.Loop();

  return 0;
}
