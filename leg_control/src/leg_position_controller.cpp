#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include "leg/leg.hpp"
#include "leg_control/pos.h"
#include <eigen3/Eigen/Dense>


class PositionController: public Leg {

public:

PositionController()  {
    ros::NodeHandle init =  ros::NodeHandle("~");
    isValid = true;

        // Get parameters: 
    #pragma region 
    // // double len,offset,width; 
    
    // try{
    // //Make sure you get the parameters. Otherwise the controls are meaningless -> throw exception
    //   if ( ! init.getParam( init.getNamespace()+"/2link/length",len   ) )  throw excP("length") ; 
    //   if ( ! init.getParam( init.getNamespace()+"/2link/offset",offset) )  throw excP("offset") ;
    //   if ( ! init.getParam( init.getNamespace()+"/2link/width",width) )    throw excP("width") ;

    // } catch (const excP & e ){
    //   ROS_ERROR_STREAM(e.what());
    //   isValid = false; 
    // }

    #pragma endregion 


    isValid = true;
    // Set zero initial goal
    Qd(0) = 0;
    Qd(1) = 0;
    Qd(2) = 0;

    Xd(0) = LB0+L12;
    Xd(1) = L01+L23+L3E;
    Xd(2) = H;

    //Set up goal server
    ros::NodeHandle GoalServerHandle;
    GoalServer = GoalServerHandle.advertiseService("GoalSet",&PositionController::setTarget, this);

    //Set up command publishers
    ros::NodeHandle n11;
    Joint1_command = n11.advertise<std_msgs::Float64>("/leg/joint1_position_controller/command", 1000);

    ros::NodeHandle n21;
    Joint2_command = n21.advertise<std_msgs::Float64>("/leg/joint2_position_controller/command", 1000);

    ros::NodeHandle n31;
    Joint3_command = n31.advertise<std_msgs::Float64>("/leg/joint3_position_controller/command", 1000);

    //Set up state subsribers
    ros::NodeHandle n12;
    Joint1_pos = n12.subscribe("/leg/joint1_position_controller/state",1000,&PositionController::PoseCallback1,this);

    ros::NodeHandle n22;
    Joint2_pos = n22.subscribe("/leg/joint2_position_controller/state",1000,&PositionController::PoseCallback2,this);

    ros::NodeHandle n32;
    Joint3_pos = n32.subscribe("/leg/joint2_position_controller/state",1000,&PositionController::PoseCallback3,this);


}

Eigen::Vector3d bestSol(){
  // Eigen::VectorXd Dists(nSols);

  double current_ang_dist;
  double min_ang_dist = 1000*M_PI;
  int index = 0;

  for (int i = 0; i<nSols ; i++){
    Eigen::Vector3d Qs( SOLS[0][i],SOLS[1][i],SOLS[2][i] );
    ROS_DEBUG_STREAM(Qs <<  std::endl);
    current_ang_dist = fabs ( gDist( Qs,q).sum() );
    if (current_ang_dist < min_ang_dist ) {
      index = i;
      min_ang_dist  = current_ang_dist;
    }
  }

  return Eigen::Vector3d(SOLS[0][index],SOLS[1][index],SOLS[2][index]) ;
}

bool setTarget(leg_control::pos::Request &req, leg_control::pos::Response &res){

  ROS_INFO_STREAM("[High Level Controller]  Current States: q1 = "<< q(0) <<", q2 = "<< q(1) <<", q3 = "<< q(2) <<"\n" );

  if (req.cartesian){
    Xd(0) = req.xw;
    Xd(1) = req.yw;
    Xd(2) = req.zw;

    ROS_INFO("[High Level Controller] Asking New Goal: xwd = %f, ywd = %f, zwd = %f \n",Xd(0),Xd(1),Xd(2));

    //Call the Inverse Kinematics function to calculate new joint variables.
    if( ! IK(Xd) ){
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

//Callback function to be changed
#pragma region
void PoseCallback1(const control_msgs::JointControllerState::ConstPtr& msg) {
   q(0) = msg->process_value; 
}

void PoseCallback2(const control_msgs::JointControllerState::ConstPtr& msg){
   q(1) = msg->process_value; 
}

void PoseCallback3(const control_msgs::JointControllerState::ConstPtr& msg){
   q(2) = msg->process_value; 
}

// void PoseCallback(const control_msgs::JointControllerState::ConstPtr& msg,double q1){
//    q1 = msg->process_value; 
// }

// funCall = std::bind(RobotController::PoseCallback,q1); 
#pragma endregion
  
// Class variables
#pragma region

  // desired state
  Eigen::Vector3d Qd, Xd;
  
  //msgs to publish desired states
  std_msgs::Float64 q1m,q2m,q3m;

  //logic flag
  bool isValid;  
  
  ros::ServiceServer GoalServer;

  ros::Publisher Joint1_command;
  ros::Publisher Joint2_command;
  ros::Publisher Joint3_command;

  ros::Subscriber Joint1_pos;
  ros::Subscriber Joint2_pos;
  ros::Subscriber Joint3_pos;
#pragma endregion

};

int main(int argc, char **argv){
  
  ros::init(argc, argv, "controller");

  //initialize the controller
  PositionController PC ; 

  //if we don't get the parameters we exit; 
  if (! PC.getValid() ) return -1; 

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    ros::spinOnce(); // for callbacks
    loop_rate.sleep();

  }
  return 0;
}


