#include "ros/ros.h"
#include "leg_control/wrench.h"
// #include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "SetEffort");
  if (argc != 4)
  {
    ROS_INFO("usage: SetEffort fxw fyw fzw");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<leg_control::wrench>("SetEffort");
  
  leg_control::wrench F;
  F.request.fxw = atof(argv[1]);
  F.request.fyw = atof(argv[2]);
  F.request.fzw = atof(argv[3]);

  if (client.call(F)){
    ROS_INFO("New goal is [%f,%f,%f]",F.request.fxw,F.request.fyw, F.request.fzw);
  }else{
    ROS_ERROR("Failed to call service to set new goal");
    return 1;
  }

  return 0;
}