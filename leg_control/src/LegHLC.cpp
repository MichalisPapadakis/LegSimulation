#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include "leg/leg.hpp"
#include "leg_control/pos.h"
#include <eigen3/Eigen/Dense>


 
//Controller management: Select and (Un)Load controllers
#include <ros/service_client.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/ListControllers.h>
#include "leg_control/ControllerSelector.h"

class PositionController{
public:

PositionController(Leg & LegObj): L(LegObj)  {
    // ros::NodeHandle pNH =  ros::NodeHandle("~"); //Private Node Handle
    ros::NodeHandle NH; //Public Nodehanlde

    //Start as  Inactive Controller
    CurrentlyActive = false;

    // Set zero initial goal
    Qd(0) = 0;
    Qd(1) = 0;
    Qd(2) = 0;

    Xd = L.DK(Qd);

    //Set up goal server
    // ros::NodeHandle GoalServerHandle; ----------------<>
    GoalServer = NH.advertiseService("GoalSet",&PositionController::setTarget, this);

    //Set up command publishers
    // ros::NodeHandle n11; ------------------------------------------<>
    Joint1_command = NH.advertise<std_msgs::Float64>("/leg/joint1_position_controller/command", 1000);

    // ros::NodeHandle n21; ------------------------------------------<>
    Joint2_command = NH.advertise<std_msgs::Float64>("/leg/joint2_position_controller/command", 1000);

    // ros::NodeHandle n31; ------------------------------------------<>
    Joint3_command = NH.advertise<std_msgs::Float64>("/leg/joint3_position_controller/command", 1000);

}

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

    current_ang_dist = fabs ( L.currentDist( Qs).sum() );
    if (current_ang_dist < min_ang_dist ) {
      index = i;
      min_ang_dist  = current_ang_dist;
    }
  }

  return ( L.getSols() ).col(index) ;
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

bool getValid(){
  return isValid;
}

void  setActive(const bool & state ){ CurrentlyActive = state;}
bool  getActive()                   { return CurrentlyActive;}

private: 

// Class variables
#pragma region

  //robot
  Leg& L; //reference to the robot.

  // desired state
  Eigen::Vector3d Qd, Xd;
  
  //msgs to publish desired states
  std_msgs::Float64 q1m,q2m,q3m;

  //logic flag
  bool isValid;  
  bool CurrentlyActive; 
  
  ros::ServiceServer GoalServer;

  ros::Publisher Joint1_command;
  ros::Publisher Joint2_command;
  ros::Publisher Joint3_command;

#pragma endregion

};

class TrajectoryController{
public:

TrajectoryController(Leg& LegObj): L(LegObj) {
    //Start as Inactive Controller
    CurrentlyActive = false;
};
void  setActive(const bool & state ){ CurrentlyActive = state;}
bool  getActive()                   { return CurrentlyActive;}

private:
 //robot
  Leg& L; //reference to the robot.

  //logic flag
  bool isValid;  
  bool CurrentlyActive; 
  
};

class EffortController{
public:
EffortController(Leg& LegObj): L(LegObj) {
    //Start as Inactive Controller
    CurrentlyActive = false;
};
void  setActive(const bool & state ){ CurrentlyActive = state;}
bool  getActive()                   { return CurrentlyActive;}

private:
 //robot
  Leg& L; //reference to the robot.

  //logic flag
  bool isValid;  
  bool CurrentlyActive; 
  
};

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
PC.setActive(true); //Activate Module

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
PC.setActive(false); //De-activate Position Module


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
PC.setActive(false); //De-activate Position Module

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
    loop_rate.sleep();
  }
  return 0;
}


