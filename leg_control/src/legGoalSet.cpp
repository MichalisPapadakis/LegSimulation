#include "ros/ros.h"
#include "leg_control/pos.h"
// #include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "leg_goal_set_client");
  if (argc != 4)
  {
    ROS_INFO("usage: leg_goal_set_client Xwd Ywd Zwd");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<leg_control::pos>("GoalSet");
  
  leg_control::pos goal;
  goal.request.xw = atof(argv[1]);
  goal.request.yw = atof(argv[2]);
  goal.request.zw = atof(argv[3]);

  if (client.call(goal)){
    ROS_INFO("New goal is [%f,%f,%f]",goal.request.xw,goal.request.yw, goal.request.zw);
  }else{
    ROS_ERROR("Failed to call service to set new goal");
    return 1;
  }

  return 0;
}