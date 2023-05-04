#pragma once

// functionality
#include <ros/ros.h> 

//data types
#include <eigen3/Eigen/Dense>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include "leg_control/pos.h"
#include <visualization_msgs/Marker.h>




//leg class
#include "leg/leg.hpp"

/** @brief Position Controller class. 
 * 
 * @note It is responsible for setting new position targets (in cartesian and joint angles space) through ros services
 * 
 */
class PositionController{
public:

/** @brief Position Controller class constructor. 
 * 
 * @param  LegObj reference to an instance of an  object from the class `Leg`. That way, the states are 
 * linked in the different controllers 
 */
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

    //set up marker publisher
    GoalMarker = NH.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

/** @brief Service callback to set Position Goal. 
 * 
 *  @note It contains a flag `cartesian`. If it is true, then a cartesian position is specified. Otherwise, 
 * a position in joint state space is specified.
 *
 * @param req request part of the srv. 
 * @param res response part of the srv. It is true if there is a solution to the IK problem or the controller is not activated 
 * @returns true if the goal is reachable and position control is activated 
 */
bool setTarget(leg_control::pos::Request &req, leg_control::pos::Response &res){
  if (!CurrentlyActive){
     ROS_WARN_STREAM("[Position Controller]: Position Control is not activated");
     return false;
  }

  // inform user for current state
  L.Querry_state();

  if (req.cartesian){
    Xd(0) = req.xw;
    Xd(1) = req.yw;
    Xd(2) = req.zw;

    ROS_INFO("[Position Controller] Asking New Goal: xwd = %f, ywd = %f, zwd = %f \n",Xd(0),Xd(1),Xd(2));

    //Call the Inverse Kinematics function to calculate new joint variables.
    if( ! L.IK(Xd) ){
      ROS_WARN_STREAM("[Position Controller] Goal out of reach");
      res.feasible = false;
      return false;
    }

    // there are solution. Select best solution
    Qd = bestSol();

    ROS_INFO("[Position Controller] New Goal is: q1 = %f, q2 = %f, q3 = %f \n",Qd(0),Qd(1),Qd(2));


  }else {
    Qd(0) = req.xw;
    Qd(1) = req.yw;
    Qd(2) = req.zw;
    Xd = L.DK(Qd);

    ROS_INFO("[Position Controller] Asking New Goal in joint state space: q1 = %f, q2 = %f, q3 = %f \n",Qd(0),Qd(1),Qd(2));
    ROS_INFO("[Position Controller] The new Goal is: xwd = %f, ywd = %f, zwd = %f \n",Xd(0),Xd(1),Xd(2));

  }


  //Publish rviz cubic magenta marker at the goal position
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = Xd(0);
  marker.pose.position.y = Xd(1);
  marker.pose.position.z = Xd(2);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.a = 1.0; // Don't forget to set the alpha!

  // Magenta
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  GoalMarker.publish( marker );



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

/** @brief Set  `CurrentlyActive` status. 
 * 
 *  @note In order to not accept new set points and send commands to inactive controllers
 * 
 * @param state true to activate, false to deactivate
 */
void  setActive(const bool & state ){ CurrentlyActive = state;}

/** @brief Get  `CurrentlyActive` status. Usefull for the HLC.
 * 
 * @returns `CurrentlyActive`. It is true if  active, otherwise false .
 */
bool  getActive()                   { return CurrentlyActive;}

private: 

/** @brief Get best solution from the IK solution set
 * 
 * @note Not Optimal: Currently, the best solution is the one with the smallest euclidean distance
 * in the joint state space. 
 * 
 * @returns a vector with the "best" desired states.
 */
Eigen::Vector3d bestSol(){

  double current_ang_dist;
  double min_ang_dist = 1000*M_PI;

  int index = 0;
  int nSols = L.get_nSols();
  
  Eigen::Vector3d Qd;

  //find minumum distance:
  for (int i = 0; i<nSols ; i++){

    Qd << ( L.getSols() ).col(i) ;
    current_ang_dist = fabs ( L.currentDist( Qd).norm() );

    if (current_ang_dist < min_ang_dist ) {
      index = i;
      min_ang_dist  = current_ang_dist;
    }
  }

  //return minum distance
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

  //Marker publisher
  ros::Publisher GoalMarker;

#pragma endregion

};
