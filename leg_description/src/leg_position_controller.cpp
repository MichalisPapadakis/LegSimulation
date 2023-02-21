#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>


class PositionController {

public:

PositionController(): q1d(0), q2d(0), q3d(0) {
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
    xd =0;
    yd =0;
    zd =0;

    //Set up goal server
    // ros::NodeHandle GoalServerHandle = ros::NodeHandle();
    // GoalServer = GoalServerHandle.advertiseService("GoalSet",&PositionController::setTarget, this);

    //Set up command publishers
    ros::NodeHandle n11;
    Joint1_command = n11.advertise<std_msgs::Float64>("/leg/joint1_position_controller/command", 1000);

    ros::NodeHandle n21;
    Joint2_command = n21.advertise<std_msgs::Float64>("/leg/joint2_position_controller/command", 1000);

    ros::NodeHandle n31;
    Joint3_command = n31.advertise<std_msgs::Float64>("/leg/joint3_position_controller/command", 1000);

    ros::NodeHandle n12;
    Joint1_pos = n12.subscribe("/leg/joint1_position_controller/state",1000,&PositionController::PoseCallback1,this);

    ros::NodeHandle n22;
    Joint2_pos = n22.subscribe("/leg/joint2_position_controller/state",1000,&PositionController::PoseCallback2,this);

    ros::NodeHandle n32;
    Joint3_pos = n32.subscribe("/leg/joint2_position_controller/state",1000,&PositionController::PoseCallback3,this);


}

bool getValid(){
  return isValid;
}

void loop(){
  
  //Callbacks (get new destination, get new positions)
  ros::spinOnce();

    // will be changed
    q1m.data = 0;
    q2m.data = 0;
    q3m.data = 0;
   
    ROS_INFO_STREAM("States: q1 = "<< q1 <<", q2 = "<< q2 <<", q3 = "<< q3 <<"\n" );
    //
  
  Joint1_command.publish(q1m);
  Joint2_command.publish(q2m);
  Joint3_command.publish(q3m);

}

private:

//Callback function to be changed
#pragma region
void PoseCallback1(const control_msgs::JointControllerState::ConstPtr& msg) {
   q1 = msg->process_value; 
}

void PoseCallback2(const control_msgs::JointControllerState::ConstPtr& msg){
   q2 = msg->process_value; 
}

void PoseCallback3(const control_msgs::JointControllerState::ConstPtr& msg){
   q3 = msg->process_value; 
}

// void PoseCallback(const control_msgs::JointControllerState::ConstPtr& msg,double q1){
//    q1 = msg->process_value; 
// }

// funCall = std::bind(RobotController::PoseCallback,q1); 
#pragma endregion
  

 // geometry
  double LB0 = 0.05555;
  double L01 = 0.07763;
  double L12 = 0.11208;
  double L23 = 0.2531;
  double L3E = 0.2455;
  double H = 0.4;

  // states
  double q1 ,q2 ,q3;
  double q1d,q2d,q3d;
  double xd, yd, zd;

  //msgs to publish desired states
  std_msgs::Float64 q1m,q2m,q3m;

  //logic flag
  bool isValid;  
  
  ros::Publisher Joint1_command;
  ros::Publisher Joint2_command;
  ros::Publisher Joint3_command;

  ros::Subscriber Joint1_pos;
  ros::Subscriber Joint2_pos;
  ros::Subscriber Joint3_pos;

};

int main(int argc, char **argv){
  
  ros::init(argc, argv, "controller");

  //initialize the controller
  PositionController PC ; 

  //if we don't get the parameters we exit; 
  if (! PC.getValid() ) return -1; 

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    
    PC.loop();
    loop_rate.sleep();

  }
  return 0;
}


