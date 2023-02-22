#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>

class exIK :public ros::Exception {

  public:
  exIK(const std::string& str):
    ros::Exception::Exception(str) {}

};

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
    xw = LB0+L12;
    yw = L01+L23+L3E;
    zw = H;

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

    //Set up state subsribers
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

//For testing
void setXYZ(const double & x,const double & y,const double &z){
    xw = x;
    yw = y;
    zw = z;

    if (IK()){
        for (int i = 0; i<nSols ; i++){
            ROS_INFO_STREAM( "Solution #" << i <<": q1 = "<< SOLS[0][i] <<", q2= "<< SOLS[1][i] <<", q3= "<< SOLS[2][i] << std::endl );
        }
    }else{
        ROS_INFO_STREAM( "Did not find a solution to the IK problem"                    << std::endl);

    }
    ROS_INFO_STREAM( "-----------------------------------------"                    << std::endl);

  

}

private: 

bool IK(){

//reset number of solutions
nSols = 0;
int nTh1 = 0;
double th1;

//set positions as seen in the 0th frame
x0 = zw -H;
y0 = LB0-xw;
z0 = -yw;


// Get th1 ---------------------------:

//Check if solution exists:
try{
  if (x0*x0 + y0*y0 < L12*L12 ) throw( exIK("Solution to IK doesn't exist: x0^2 + y0^2 < L12^2 " ) );
}catch( const exIK& e){
  ROS_ERROR_STREAM(e.what());
  return false;
}

//discriminant
double D = 4*(L12*L12)*(x0*x0) - 4*( (L12*L12)-(y0*y0) )*( (x0*x0)+(y0*y0) ) ;

th1 = asin(( 2*L12*x0+sqrt(D) ) / ( 2*x0*x0+2*y0*y0) ); //+sqrt(D)  // std::cout << "checking : " << abs(L12-x0*sin(th1)+y0*cos(th1))  << std::endl;
//because it may pi-th1 (which is out of bounds)
if ( abs(L12-x0*sin(th1)+y0*cos(th1) ) < 1e-8) {  
    SOLS[0][nTh1] = th1; 
    nTh1++;
}

if (abs(D)>1e-6){ // else identical solutions
    th1 = asin(( 2*L12*x0-sqrt(D) ) / ( 2*x0*x0+2*y0*y0) ); //-sqrt(D)
    if ( abs(L12-x0*sin(th1)+y0*cos(th1) ) < 1e-8) { 
        SOLS[0][2*(nTh1)] = th1; //i want to group the solutions with the same th1 -> [th1 th1 th2 th2;...;...]
        nTh1++;
    }
}

// Get th2 and th3 -------------------:
double th2,th3;
double K1,K2;
double c2,s2,det,A,B;

for (int i = 0; i<nTh1 ;i++){
    th1 = SOLS[0][2*i];    

    K1 = x0*cos(th1)+y0*sin(th1);
    K2 = -z0 - L01;

    // Check if solution exists
    if (K1*K1 + K2*K2 > pow(L23 + L3E,2) ){continue;} //check the second solution
    if (K1*K1 + K2*K2 < L23*L23 + L3E*L3E + 2*L23*L3E*cos(2.229)  ){continue;}


    // system definitions
    th3 = acos( (K1*K1+K2*K2 - L23*L23 - L3E*L3E)/(2*L23*L3E)  );

    det = -( L23*L23+L3E*L3E + 2*L23*L3E*cos(th3)) ;

    A = L3E*sin(th3);
    B = L23+L3E*cos(th3) ; 

    // 1st solution
    c2 = -(  A*K1 + B*K2 ) / det ;
    s2 =  ( -B*K1 + A*K2 ) / det ;

    th2 = atan2(s2,c2);

    SOLS[0][nSols] = th1;
    SOLS[1][nSols] = th2; 
    SOLS[2][nSols] = th3 ;

    nSols++;

    // 2nd solution
    c2 = -( -A*K1 + B*K2 ) / det ;
    s2 =  ( -B*K1 - A*K2 ) / det ;

    th2 = atan2(s2,c2);

    SOLS[0][nSols] = th1;
    SOLS[1][nSols] = th2; 
    SOLS[2][nSols] = -th3 ;

    nSols++;
}

//Inform  if solution doesn't exists:
try{
    if (nSols == 0) throw( exIK("Solution to IK doesn't exist: q3 doesn't exists" ) ) ;
}catch( const exIK& e){
    ROS_ERROR_STREAM(e.what());
    return false; 
    }

return true;

}


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
  
// Class variables
#pragma region
 // geometry
  double LB0 = 0.05555;
  double L01 = 0.07763;
  double L12 = 0.11208;
  double L23 = 0.2531;
  double L3E = 0.2455;
  double H = 0.4;

  // states
  double q1 ,q2 ,q3;

  //desired states:
  double x0, y0, z0; //positions in the 0th frame
  double xw, yw, zw; //positions in the world frame

  double SOLS[3][4]; // 4 solutions maximum;
  int nSols;    

  double q1d,q2d,q3d; // the selected from the solution set solutions
  
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
#pragma endregion
};

int main(int argc, char **argv){
  
  ros::init(argc, argv, "controller");

  //initialize the controller
  PositionController PC ; 

  //if we don't get the parameters we exit; 
  if (! PC.getValid() ) return -1; 

  ros::Rate loop_rate(1);

  while (ros::ok())
  {

    ROS_INFO_STREAM("matlab inputs: q1 = 0,q2 =0 ,q3=0 \n"); 
    PC.setXYZ(0.16763,0.5762294,0.40);

    ROS_INFO_STREAM("matlab inputs: q1 = 1,q2 =1 ,q3=-1 \n"); 
    PC.setXYZ(-0.0631,0.4599,0.6094);

    ROS_INFO_STREAM("matlab inputs: q1 = 0.5,q2 =0.3 ,q3=0.4 \n"); 
    PC.setXYZ(0.0422265,0.5071944,0.658168);

    ROS_INFO_STREAM("matlab inputs: q1 = 1.3,q2 =-0.3 ,q3=1.4 \n"); 
    PC.setXYZ(-0.0532165,0.4307835,0.546514);
    // PC.loop();
    loop_rate.sleep();

  }
  return 0;
}


