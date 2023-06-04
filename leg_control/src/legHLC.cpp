#include <ros/ros.h>
#include <HLC.hpp>

int main(int argc, char **argv){
  
  ros::init(argc, argv, "controller");

  //initialize the controller
  HLC Controller ; 

  //Set loop rate
  ros::Rate loop_rate(1000);

  while (ros::ok())
  {
    ros::spinOnce();   // for callbacks
    Controller.Loop(); // Controller periodic tasks (if force control is activated)
    loop_rate.sleep(); // sleep for the rest of the time
  }

  return 0;
}


