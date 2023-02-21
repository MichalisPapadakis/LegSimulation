#include <ros/ros.h>


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

    // Set zero initial goal
    xd =0;
    yd =0;
    zd =0;

}



private:

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

//   std_msgs::Float64 q1m ,  q2m ;

  //logic flag
  bool isValid;  
  
  ros::Publisher Joint1_command;
  ros::Publisher Joint2_command;
  ros::Publisher Joint3_command;

  ros::Subscriber Joint1_pos;
  ros::Subscriber Joint2_pos;
  ros::Subscriber Joint3_pos;



};

int main(){

  return 0;
}


