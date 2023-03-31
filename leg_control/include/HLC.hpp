#pragma once
#include <ros/ros.h>
// robot
#include "leg/leg.hpp"
// other controllers
#include <PositionController.hpp>
#include <TrajectoryController.hpp>
#include <EffortController.hpp>

//Controller management: Select and (Un)Load controllers
#include <ros/service_client.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/ListControllers.h>
#include "leg_control/ControllerSelector.h"

/** @brief High Level Controller class. 
 * 
 * @note It is responsible for loading and switching Low Level Controllers (ros_control controllers) seemlessly.
 * 
 */
class HLC{

public:

/** @brief Trajectory Controller class constructor. 
 * 
 * @param  LegObj reference to an instance of an  object from the class `Leg`. That way, the states are 
 * linked in the different controllers  
 */
HLC(): PC(this-> L), TC(this->L), EC(this->L) {
    // ros::NodeHandle init("~") ; //Gets the nodename from the launch file. Private Nodehandler
    NH = ros::NodeHandle();     //public NodeHandle

    //Initialize robot
    L.init(); //set up state subscribers

    //Load controllers:
    loadControllers(); //load ros control
    CurrentMode = 0; // no mode selected

    //Switch Controllers:
    CM_switcher = NH.serviceClient<controller_manager_msgs::SwitchController>("/leg/controller_manager/switch_controller");

    //Start Controllers:
    CM_switcher.waitForExistence();
    ROS_INFO("[HLC]: Switch_controller service is available.\n");
    StartPosition();    // Default to position

    //Select Controllers
    Cselector = NH.advertiseService("ControllerSelector",&HLC::SelectController, this);

}

/** @brief Loop function of HLC. 
 * 
 * @note If some functionality of the LLC must be executed in each loop, this functionality is accesed by calling public functions of the LLCs
 * in this loop.  
 * 
 */
void Loop(){
   switch ( CurrentMode ){
    case 1:

        break;
    case 2:

        break;
    case 3:
        EC.PublishEffort();
       break; 
    default:
        break;

   }


}

private: 

/** @brief Load Controllers helper function to automate the load controller service. Informs the user in case of problem.
 * 
 * @param str Controller name
 * @param CMLS  Control Manager Loading Service Server
 */
void loadSpecificController(const std::string & str, ros::ServiceClient & CMLSS ){
    
    controller_manager_msgs::LoadController l_req;
    l_req.request.name  = str;

    if ( CMLSS.call(l_req) ){
        ROS_INFO_STREAM("[HLC] Loaded: "<< l_req.request.name <<"\n");
    }else{
        ROS_ERROR_STREAM("[HLC] Problem Loading" << l_req.request.name << "\n");
    }
}

/** @brief Load Controllers during the initialization of the HLC. The user is inform for the successful loading of all the controllers
 * 
 */
void loadControllers(){
    ros::ServiceClient CM_loader = NH.serviceClient<controller_manager_msgs::LoadController>("/leg/controller_manager/load_controller");

    CM_loader.waitForExistence(); // wait for controller manager to make its services available

    // Position Controllers: 
    loadSpecificController("joint1_position_controller",CM_loader);
    loadSpecificController("joint2_position_controller",CM_loader);
    loadSpecificController("joint3_position_controller",CM_loader);

    //Trajectory Controller:
    loadSpecificController("Trajectory_Controller",CM_loader);

    //Effort Controllers:
    loadSpecificController("joint1_effort_controller",CM_loader);
    loadSpecificController("joint2_effort_controller",CM_loader);
    loadSpecificController("joint3_effort_controller",CM_loader);

    ROS_INFO_STREAM("[HLC] Successfully loaded all controllers! \n");
}

/** @brief Select which controller to start, stopping all the previous started controllers
 * 
 * @param req  request of the ControlSelector srv. Contains the controller mode. 1: Position Control, 2: Trajectory Control, 3: Effort Control
 * @param res  response of the ControlSelector srv. It is set to true if the starting of the controller is successful. 
 */
bool SelectController(leg_control::ControllerSelector::Request &req, leg_control::ControllerSelector::Response &res){
    res.feasible = true;

    // Currently selected controller
    if (  req.ControllerToSelect == CurrentMode) {
      ROS_INFO_STREAM("[HLC]: Already using the selected controller mode!");
      return true;
    }

    // Change
    switch (req.ControllerToSelect  ){
      case 1:
        StartPosition();
        break;
      case 2:
        StartTrajectory();
        break;
      case 3:
        StartEffort();
        break;
      default:
        ROS_WARN_STREAM("[HLC]: No suitable controller. Please select 1: Position Control, 2: Trajectory Control, 3: Effort Control");
        res.feasible = false;
        return false;
        break;
    }

    return true;
}

/** @brief Start Position Controllers.
 * 
 * @note Set the `CurrentlyActive` status of the controllers. 
 * Publish the messages to switch controller and inform the user. 
 * 
 */
void StartPosition(){
PC.setActive(true ); //Activate Position Module
TC.setActive(false); //De-Activate Trajectory Module
EC.setActive(false); //De-Activate Force Module


// Start respective controllers:
controller_manager_msgs::SwitchController Sw;
Sw.request.start_asap = true;

Sw.request.start_controllers.push_back("joint1_position_controller");
Sw.request.start_controllers.push_back("joint2_position_controller");
Sw.request.start_controllers.push_back("joint3_position_controller");

// stop previous running controllers:
switch(CurrentMode) {

    case 2: // Trajectory Control
        Sw.request.stop_controllers.push_back("Trajectory_Controller");
        break;

    case 3: // Force Control
        Sw.request.stop_controllers.push_back("joint1_effort_controller");
        Sw.request.stop_controllers.push_back("joint2_effort_controller");
        Sw.request.stop_controllers.push_back("joint3_effort_controller");
        break;

}
Sw.request.strictness = 2;

// Switch controllers:
CurrentMode = 1;
if ( CM_switcher.call(Sw) ){
    ROS_INFO_STREAM("[HLC]: Started Position Controllers \n");
}else{
    ROS_ERROR_STREAM("[HLC]: Problem starting Position Controllers \n");
}


}

/** @brief Start Trajectory Controller.
 * 
 * @note Set the `CurrentlyActive` status of the controllers. 
 * Publish the messages to switch controller and inform the user. 
 * 
 */
void StartTrajectory(){
PC.setActive(false); //De-Activate Position Module
TC.setActive(true);  //Activate Trajectory Module
EC.setActive(false); //De-Activate Force Module

// Start respective controllers:
controller_manager_msgs::SwitchController Sw;
Sw.request.start_asap = true;

Sw.request.start_controllers.push_back("Trajectory_Controller");

// stop previous running controllers:
switch(CurrentMode) {

    case 1: // Position Control
        Sw.request.stop_controllers.push_back("joint1_position_controller");
        Sw.request.stop_controllers.push_back("joint2_position_controller");
        Sw.request.stop_controllers.push_back("joint3_position_controller");
        break;

    case 3: // Force Control
        Sw.request.stop_controllers.push_back("joint1_effort_controller");
        Sw.request.stop_controllers.push_back("joint2_effort_controller");
        Sw.request.stop_controllers.push_back("joint3_effort_controller");
        break;

}
Sw.request.strictness = 2;

// Switch controllers:
CurrentMode = 2;
if ( CM_switcher.call(Sw) ){
    ROS_INFO_STREAM("[HLC]: Started Trajectory Controller \n");
}else{
    ROS_ERROR_STREAM("[HLC]: Problem starting Trajectory Controller \n");
}


}

/** @brief Start Effort Controllers.
 * 
 * @note Set the `CurrentlyActive` status of the controllers. 
 * Publish the messages to switch controller and inform the user. 
 * 
 */
void StartEffort(){
PC.setActive(false); //De-Activate Position Module
TC.setActive(false); //De-Activate Trajectory Module
EC.setActive(true);  //Activate Force Module

// Start respective controllers:
controller_manager_msgs::SwitchController Sw;
Sw.request.start_asap = true;

Sw.request.start_controllers.push_back("joint1_effort_controller");
Sw.request.start_controllers.push_back("joint2_effort_controller");
Sw.request.start_controllers.push_back("joint3_effort_controller");

// stop previous running controllers:
switch(CurrentMode) {

    case 1: // Position Control
        Sw.request.stop_controllers.push_back("joint1_position_controller");
        Sw.request.stop_controllers.push_back("joint2_position_controller");
        Sw.request.stop_controllers.push_back("joint3_position_controller");
        break;

    case 2: // Trajectory Control
        Sw.request.stop_controllers.push_back("Trajectory_Controller");
        break;

}
Sw.request.strictness = 2;

// Switch controllers:
CurrentMode = 3;
if ( CM_switcher.call(Sw) ){
    ROS_INFO_STREAM("[HLC]: Started Effort Controllers \n");
}else{
    ROS_ERROR_STREAM("[HLC]: Problem starting Effort Controllers \n");
}


}

// Class variables
#pragma region

  //Leg robot
  Leg L; 

  //Controllers:
  PositionController   PC; //Position Controller
  TrajectoryController TC; //Trajectory Controller
  EffortController     EC; //Effort Controller

  //Control Mode: { 1-> Pos, 2-> Traj, 3-> Force }
  uint8_t CurrentMode;  

  //logic flag
  bool isValid;  
  
  // Ros Functionality: 
  ros::NodeHandle NH;             // Public Nodehandle
  ros::ServiceServer Cselector;   // Controller Selector Service Server
  ros::ServiceClient CM_switcher; // Controller Switcher Service Server

#pragma endregion

};