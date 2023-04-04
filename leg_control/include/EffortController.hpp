#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include "leg/leg.hpp"
#include "leg_control/pos.h"
#include <eigen3/Eigen/Dense>

//Effort Controllers
#include <leg_control/wrench.h> 


/** @brief Effort Controller class. 
 * 
 * 
 */
class EffortController{
public:

/** @brief Effort Controller class constructor. 
 * 
 * @param  LegObj reference to an instance of an  object from the class `Leg`. That way, the states are 
 * linked in the different controllers  * 
 */
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

/** @brief Set  `CurrentlyActive` status. Useful for HLC.
 * 
 * 
 * @param state true to activate, false to deactivate
 */
void  setActive(const bool & state ){ CurrentlyActive = state;}

/** @brief Get  `CurrentlyActive` status. Useful for HLC.
 * 
 * @returns `CurrentlyActive`. It is true if  active, otherwise false .
 */
bool  getActive()                   { return CurrentlyActive;}

/** @brief Service callback to set Position Goal. 
 *
 * @param req request part of the srv. 
 * @param res response part of the srv. It is true if there is effort controller is currently active.
 * @returns true if effort controller is currently active.
 */
bool setEffort(leg_control::wrench::Request & req,leg_control::wrench::Response & res){
  if (!CurrentlyActive){
     ROS_WARN_STREAM("[Effort Controller] Effort Control is not activated");
     res.feasible = true;
     return false;
  }
  // F from EE to world
  Fd << req.fxw, req.fyw, req.fzw ;
  res.feasible = true;
  return true;

}


/** @brief Force control with gravity compensation
 *
 */
void PublishEffort(){
  //required as there is no feedback loop by ros control
  // With gravity compensation
  Eigen::Vector3d t = L.Calculate_Jv().transpose() * Fd ;//+ L.CalculateG();
  t1.data = t(0);
  t2.data = t(1);
  t3.data = t(2);

  Joint1_command.publish(t1);
  Joint2_command.publish(t2);
  Joint3_command.publish(t3);


}

private:
 //robot
  Leg& L; //reference to the robot.

  Eigen::Vector3d Fd; 
  std_msgs::Float64 t1,t2,t3; //torque command for each joint

  //logic flag
  bool CurrentlyActive; 

  // Ros functionality:
  ros::ServiceServer EffortServer;

  ros::Publisher Joint1_command;
  ros::Publisher Joint2_command;
  ros::Publisher Joint3_command;
  
};
