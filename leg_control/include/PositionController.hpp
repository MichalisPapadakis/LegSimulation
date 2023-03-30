#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include "leg/leg.hpp"
#include "leg_control/pos.h"
#include <eigen3/Eigen/Dense>

class PositionController{
public:

PositionController(Leg & LegObj): 
L(LegObj)  
{
    ros::NodeHandle NH; //Public Nodehanlde

    //Start as  Inactive Controller
    CurrentlyActive = false;

    // Set zero initial goal
    Qd(0) = 0;
    Qd(1) = 0;
    Qd(2) = 0;

    Xd = L.DK(Qd);

    //Set up goal server
    GoalServer = NH.advertiseService("GoalSet",&PositionController::setTarget, this);

    //Set up command publishers
    Joint1_command = NH.advertise<std_msgs::Float64>("/leg/joint1_position_controller/command", 1000);
    Joint2_command = NH.advertise<std_msgs::Float64>("/leg/joint2_position_controller/command", 1000);
    Joint3_command = NH.advertise<std_msgs::Float64>("/leg/joint3_position_controller/command", 1000);

}

bool setTarget(leg_control::pos::Request &req, leg_control::pos::Response &res){
  if (!CurrentlyActive){
     ROS_WARN_STREAM("[High Level Controller] Position Control is not activated");
     return false;
  }

  // inform user for current state
  L.Querry_state();

  if (req.cartesian){
    Xd(0) = req.xw;
    Xd(1) = req.yw;
    Xd(2) = req.zw;

    ROS_INFO("[High Level Controller] Asking New Goal: xwd = %f, ywd = %f, zwd = %f \n",Xd(0),Xd(1),Xd(2));

    //Call the Inverse Kinematics function to calculate new joint variables.
    if( ! L.IK(Xd) ){
      ROS_WARN_STREAM("[High Level Controller] Goal out of reach");
      res.feasible = false;
      return false;
    }

    // there are solution. Select best solution
    Qd = bestSol();

    ROS_INFO("[High Level Controller] New Goal is: q1 = %f, q2 = %f, q3 = %f \n",Qd(0),Qd(1),Qd(2));


  }else {
    Qd(0) = req.xw;
    Qd(1) = req.yw;
    Qd(2) = req.zw;
    Xd = L.DK(Qd);

    ROS_INFO("[High Level Controller] Asking New Goal in joint state space: q1 = %f, q2 = %f, q3 = %f \n",Qd(0),Qd(1),Qd(2));
    ROS_INFO("[High Level Controller] The new Goal is: xwd = %f, ywd = %f, zwd = %f \n",Xd(0),Xd(1),Xd(2));

  }

  //populate message
  q1m.data = Qd(0);
  q2m.data = Qd(1);
  q3m.data = Qd(2);

  // Publish:
  Joint1_command.publish(q1m);
  Joint2_command.publish(q2m);
  Joint3_command.publish(q3m);

  // Inform user for success!
  res.feasible = true;
  return true; 


};

void  setActive(const bool & state ){ CurrentlyActive = state;}

bool  getActive()                   { return CurrentlyActive;}

private: 

// this is not optimal
Eigen::Vector3d bestSol(){
  // Eigen::VectorXd Dists(nSols);

  double current_ang_dist;
  double min_ang_dist = 1000*M_PI;
  int index = 0;
  int nSols = L.get_nSols();
  Eigen::Vector3d Qs;

  for (int i = 0; i<nSols ; i++){

    Qs << ( L.getSols() ).col(i) ;
    ROS_DEBUG_STREAM(Qs <<  std::endl);

    current_ang_dist = fabs ( L.currentDist( Qs).norm() );
    if (current_ang_dist < min_ang_dist ) {
      index = i;
      min_ang_dist  = current_ang_dist;
    }
  }

  return ( L.getSols() ).col(index) ;
}

// Class variables
#pragma region


  //robot
  Leg& L; //reference to the robot.

  // desired state
  Eigen::Vector3d Qd, Xd;
  
  //msgs to publish desired states
  std_msgs::Float64 q1m,q2m,q3m;

  //logic flag
  bool CurrentlyActive; 
  
  ros::ServiceServer GoalServer;

  //Publish commands:
  ros::Publisher Joint1_command;
  ros::Publisher Joint2_command;
  ros::Publisher Joint3_command;

#pragma endregion

};
