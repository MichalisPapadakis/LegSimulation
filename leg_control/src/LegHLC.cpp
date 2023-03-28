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

class HLC{

public:

HLC()  {
    ros::NodeHandle init =  ros::NodeHandle("~");
    isValid = true;

    // Get parameters: 
    #pragma region 
    // // double len,offset,width; 
    
    // try{
    // //Make sure you get the parameters. Otherwise the controls are meaningless -> throw exception
    //   if ( ! init.getParam( init.getNamespace()+"/2link/width",width) )    throw excP("width") ;
    // } catch (const excP & e ){
    //   ROS_ERROR_STREAM(e.what());
    //   isValid = false; 
    // }

    #pragma endregion 

    isValid = true;

    //Initialize robot
    L.init();

    //Load controllers:
    loadControllers();

    CurrentMode = 0;
    //List controllers:

    ros::NodeHandle NL;
    CM_listing = NL.serviceClient<controller_manager_msgs::ListControllers>("/leg/controller_manager/list_controllers");

    ros::NodeHandle nCM;
    CM_switcher = nCM.serviceClient<controller_manager_msgs::SwitchController>("/leg/controller_manager/switch_controller");

    //Start Controllers:
    CM_switcher.waitForExistence();
    ROS_INFO("[HLC]: Switch_controller service is available.\n");

    ros::Duration a(5);
    StartPosition();
    a.sleep();
    StartTrajectory();
    a.sleep();
    StartEffort();
    a.sleep();
    StartPosition();

    // Set zero initial goal
    Qd(0) = 0;
    Qd(1) = 0;
    Qd(2) = 0;

    Xd = L.DK(Qd);


    //Set up goal server
    ros::NodeHandle GoalServerHandle;
    GoalServer = GoalServerHandle.advertiseService("GoalSet",&HLC::setTarget, this);

    //Set up command publishers
    ros::NodeHandle n11;
    Joint1_command = n11.advertise<std_msgs::Float64>("/leg/joint1_position_controller/command", 1000);

    ros::NodeHandle n21;
    Joint2_command = n21.advertise<std_msgs::Float64>("/leg/joint2_position_controller/command", 1000);

    ros::NodeHandle n31;
    Joint3_command = n31.advertise<std_msgs::Float64>("/leg/joint3_position_controller/command", 1000);

}

// Right Initialization:
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
    ros::NodeHandle nCM1;
    ros::ServiceClient CM_loader = nCM1.serviceClient<controller_manager_msgs::LoadController>("/leg/controller_manager/load_controller");

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

void ListControllers(){
    // controller_manager_msgs::ListControllers LCmsg;
    // if (CM_listing.call(LCmsg)){
    //     LCmsg.response.controller[0].state;
    // }; 
}

void StartPosition(){

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

Eigen::Vector3d bestSol(){
  // Eigen::VectorXd Dists(nSols);

  double current_ang_dist;
  double min_ang_dist = 1000*M_PI;
  int index = 0;
  int nSols = L.get_nSols();

  for (int i = 0; i<nSols ; i++){

    Eigen::Vector3d Qs;
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

    ROS_INFO("[High Level Controller] Asking New Goal in joint state space: q1 = %f, q2 = %f, q3 = %f \n",Qd(0),Qd(1),Qd(2));

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


private: 

// Class variables
#pragma region

  //robot
  Leg L;

  //Control Mode:
  int CurrentMode;  //1: Pos, 2: Traj, 3: Force

  // desired state
  Eigen::Vector3d Qd, Xd;
  
  //msgs to publish desired states
  std_msgs::Float64 q1m,q2m,q3m;

  //logic flag
  bool isValid;  
  
  // Ros Functionality: 
  ros::ServiceServer GoalServer;
  ros::ServiceClient CM_switcher;
  ros::ServiceClient CM_listing;

    // Pos
  ros::Publisher Joint1_command;
  ros::Publisher Joint2_command;
  ros::Publisher Joint3_command;

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


