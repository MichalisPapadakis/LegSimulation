#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include "leg/leg.hpp"
#include "leg_control/pos.h"
#include <eigen3/Eigen/Dense>

//Effort Controllers
#include <leg_control/wrench.h> 

class EffortController{
public:
EffortController(Leg& LegObj): L(LegObj) {
    ros::NodeHandle NH; //Public Nodehanlde

    //Start as Inactive Controller
    CurrentlyActive = false;

    //Initial Goal
    Fd<<0,0,0;

    //Set up goal server
    // ros::NodeHandle GoalServerHandle; ----------------<>
    EffortServer = NH.advertiseService("SetEffort",&EffortController::setEffort, this);

    Joint1_command = NH.advertise<std_msgs::Float64>("/leg/joint1_effort_controller/command",10);
    Joint2_command = NH.advertise<std_msgs::Float64>("/leg/joint2_effort_controller/command",10);
    Joint3_command = NH.advertise<std_msgs::Float64>("/leg/joint3_effort_controller/command",10);

};

void PublishEffort(){
  //required as there is no feedback loop by ros control
  // With gravity compensation
  Eigen::Vector3d t = L.Calculate_Jv().transpose() * Fd + L.CalculateG();
  t1.data = t(0);
  t2.data = t(1);
  t3.data = t(2);

  Joint1_command.publish(t1);
  Joint2_command.publish(t2);
  Joint3_command.publish(t3);


}

bool setEffort(leg_control::wrench::Request & req,leg_control::wrench::Response & res){
  if (!CurrentlyActive){
     ROS_WARN_STREAM("[High Level Controller] Effort Control is not activated");
     res.feasible = true;
     return false;
  }
  // F from EE to world
  Fd << req.fxw, req.fyw, req.fzw ;
  res.feasible = true;
  return true;

}

void  setActive(const bool & state ){ CurrentlyActive = state;}
bool  getActive()                   { return CurrentlyActive;}

private:
 //robot
  Leg& L; //reference to the robot.

  Eigen::Vector3d Fd; 
  std_msgs::Float64 t1,t2,t3;

  //logic flag
  bool isValid;  
  bool CurrentlyActive; 

  // Ros functionality:
  ros::ServiceServer EffortServer;

  ros::Publisher Joint1_command;
  ros::Publisher Joint2_command;
  ros::Publisher Joint3_command;
  
};
