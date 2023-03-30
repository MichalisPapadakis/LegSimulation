#include "ros/ros.h"
#include "leg_control/ellipse.h"
// #include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "SetEllipse");
  if (argc != 7)
  {
    ROS_INFO("usage: SetEllipse a b DX DY dth times");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<leg_control::ellipse>("SetEllipse");
  
  //Populate message:
  leg_control::ellipse ellipse;
  ellipse.request.a     = atof(argv[1]);
  ellipse.request.b     = atof(argv[2]);
  ellipse.request.DX    = atof(argv[3]);
  ellipse.request.DY    = atof(argv[4]);
  ellipse.request.dth   = atof(argv[5]);
  ellipse.request.times = atof(argv[6]);

  if (client.call(ellipse)){
    ROS_INFO("New ellipse!");
  }else{
    ROS_ERROR("Failed to call service to set new ellipse");
    return 1;
  }

  return 0;
}