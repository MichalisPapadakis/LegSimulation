#include "ros/ros.h"
#include <rosbag/bag.h>
#include "leg_control/pos.h"
#include "leg_control/ControllerSelector.h"
#include "leg_control/wrench.h"
#include "leg_control/ellipse.h"
#include "std_srvs/Empty.h"
#include "gazebo_msgs/SetPhysicsProperties.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "LegSim");

  ros::NodeHandle n;
  ros::ServiceClient PosClient    = n.serviceClient<leg_control::pos>("GoalSet");
  ros::ServiceClient EffortClient = n.serviceClient<leg_control::wrench>("SetEffort");
  ros::ServiceClient CSclient     = n.serviceClient<leg_control::ControllerSelector>("ControllerSelector");
//   ros::ServiceClient gzClient     = n.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
//   ros::ServiceClient phClient     = n.serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");

  
  leg_control::pos goal1;
  leg_control::pos goal2;

  leg_control::wrench wrench; 
  leg_control::ControllerSelector Cs; 

 

//task 1 prep
Cs.request.ControllerToSelect = 1;

  if (CSclient.call(Cs)){
    ROS_INFO("[Sim]: Select Position Control");
  }else{
    ROS_ERROR("[Sim]: Failed to select Position Control");
    return 1;
  }

  // Task 1: 
  goal1.request.cartesian = true;
  goal1.request.xw = 0.1667;
  goal1.request.yw = 0.05;
  goal1.request.zw = 0.08;
  

  if (PosClient.call(goal1)){
    ROS_INFO("[Sim]: New position goal is [%f,%f,%f]",goal1.request.xw,goal1.request.yw, goal1.request.zw);
  }else{
    ROS_ERROR("[Sim]: Failed to call service to set new position goal");
    return 1;
  }
  ros::Duration(1).sleep();

    //task 2
goal2.request.cartesian = true;
  goal2.request.xw = 0.1667;
  goal2.request.yw = 0.;
  goal2.request.zw = 0.0225;
  if (PosClient.call(goal2)){
    ROS_INFO("[Sim]: New position goal is [%f,%f,%f]",goal2.request.xw,goal2.request.yw, goal2.request.zw);
  }else{
    ROS_ERROR("[Sim]: Failed to call service to set new  position goal");
    return 1;
  }
  ros::Duration(1).sleep();

 // Task 3 prep

Cs.request.ControllerToSelect = 3;
  if (CSclient.call(Cs)){
    ROS_INFO("[Sim]: Select Effort Control");
  }else{
    ROS_ERROR("[Sim]: Failed to select Effort Control");
    return 1;
  }
  
  // Task 3
  wrench.request.fxw = 0;
  wrench.request.fyw = 0;
  wrench.request.fzw = -100;
   if (EffortClient.call(wrench)){
    ROS_INFO("[Sim]: New effort vertical goal is %f",wrench.request.fzw);
  }else{
    ROS_ERROR("[Sim]: Failed to call service to set new effort goal");
    return 1;
  }





  return 0;
}
