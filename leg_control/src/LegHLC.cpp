#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include "leg/leg.hpp"
#include "leg_control/pos.h"
#include <eigen3/Eigen/Dense>

#include <PositionController.hpp>
#include <TrajectoryController.hpp>
#include <EffortController.hpp>


//Controller management: Select and (Un)Load controllers
#include <ros/service_client.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/ListControllers.h>
#include "leg_control/ControllerSelector.h"

class HLC{

public:

HLC(): PC(this-> L), TC(this->L), EC(this->L) {
    // ros::NodeHandle init("~") ; //Gets the nodename from the launch file. Private Nodehandler
    NH = ros::NodeHandle();     //public NodeHandle

    isValid = true;

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

// Load Controllers:
void loadSpecificController(const std::string & str, ros::ServiceClient & CMNH ){
    
    controller_manager_msgs::LoadController l_req;
    l_req.request.name  = str;

    if ( CMNH.call(l_req) ){
        ROS_INFO_STREAM("[HLC] Loaded: "<< l_req.request.name <<"\n");
    }else{
        ROS_ERROR_STREAM("[HLC] Problem Loading" << l_req.request.name << "\n");
    }
}

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

//Select Controllers:
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

// Start Controllers
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

bool getValid(){
  return isValid;
}

//Looping
void Loop(){
  if (CurrentMode == 3){
    EC.PublishEffort();
  }

}
private: 

// Class variables
#pragma region

  //robot
  Leg L;

  //Controllers:
  PositionController   PC;
  TrajectoryController TC;
  EffortController     EC;

  //Control Mode:
  uint8_t CurrentMode;  //1: Pos, 2: Traj, 3: Force

  //logic flag
  bool isValid;  
  
  // Ros Functionality: 
  ros::NodeHandle NH;
  ros::ServiceServer Cselector;
  ros::ServiceClient CM_switcher;

#pragma endregion

};

int main(int argc, char **argv){
  
  ros::init(argc, argv, "controller");

  //initialize the controller
  HLC Controller ; 

  //if we don't get the parameters we exit; 
  if (! Controller.getValid() ) return -1; 

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    ros::spinOnce(); // for callbacks
    Controller.Loop();
    loop_rate.sleep();
  }
  return 0;
}


