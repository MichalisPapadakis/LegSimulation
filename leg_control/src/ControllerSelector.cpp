#include "ros/ros.h"
#include "leg_control/ControllerSelector.h"
// #include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ControllerSelector");
  if (argc != 2)
  {
    ROS_INFO("usage: ControllerSelector 1 (e.g. for Position Control)");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<leg_control::ControllerSelector>("ControllerSelector");
  
  leg_control::ControllerSelector CS;
  CS.request.ControllerToSelect = atoi(argv[1]);

  std::string s;
  switch (CS.request.ControllerToSelect  ){
      case 1:
        s = "Position Control";
        break;
      case 2:
        s = "Trajectory Control";
        break;
      case 3:
        s = "Effort Control";
        break;
      default:
        s = "No Control";  
        break;
    }


  if (client.call(CS)){
    ROS_INFO("SelectController: %s]",s);
  }else{
    ROS_ERROR("Failed to call service to set new controller");
    return 1;
  }

  return 0;
}