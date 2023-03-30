#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include "leg/leg.hpp"
#include "leg_control/pos.h"
#include <eigen3/Eigen/Dense>

// Action interface
#include <actionlib/client/simple_action_client.h> 
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <leg_control/ellipse.h> 


class TrajectoryController{
public:

TrajectoryController(Leg& LegObj): 
L(LegObj), 
ac("leg/Trajectory_Controller/follow_joint_trajectory",true)  
// these objects do not have a default constructor.
{
  ros::NodeHandle NH; //public nodehandle

  //Start as Inactive Controller
  CurrentlyActive = false;

  //
  EllipseServer = NH.advertiseService("SetEllipse",&TrajectoryController::SetEllipse, this);

};

bool GenerateVerticalEllipse(){
  // Arrays for element wise multiplication and division:
  #define NpointsE 30
  using Eigen::placeholders::all;

  Eigen::MatrixXd X(3,NpointsE);
  Eigen::ArrayXd  th = Eigen::ArrayXd::LinSpaced( NpointsE, 2*M_PI/(NpointsE-1), 2*M_PI );

  // radius(th)
  Eigen::ArrayXd  r = a*b /                 //r = ab/sqrt( b^2 c(th-dth)^2 +  a^2 s(th-dth)^2 )
  ( 
    (b*b)*(   (th-dth).cos().pow(2)   )  +  // b^2*( cos(th-dth) )^2
    (a*a)*(   (th-dth).sin().pow(2)   )     // a^2*( sin(th-dth) )^2
  ).sqrt();

    ROS_INFO_STREAM(X);

  //Populate X
  double Xo = L.DK(Eigen::Vector3d(0,0,0))  (0); // get first element of vector


  X.row(0) = Eigen::MatrixXd::Constant(1,NpointsE,Xo);
  X.row(1) = ( r*th.cos() + DX ).matrix();
  X.row(2) = ( r*th.sin() + DY ).matrix();

  Eigen::VectorXd t = Eigen::ArrayXd::LinSpaced(NpointsE,5/(NpointsE-1),5).matrix();

    ROS_INFO_STREAM("starting call to generateQ!");
     ROS_INFO_STREAM(X);
ROS_INFO_STREAM(t);

  if (! generateQwp(X,t) ) {
    
    ROS_ERROR_STREAM("Problem With generation trajectory from ellipse");
    return false;
  };

  PublishTrajectory();
  return true;


}


bool SetEllipse(leg_control::ellipse::Request& req,leg_control::ellipse::Response & res){
  a = req.a;   b = req.b; times = req.times;
  DX = req.DX; DY = req.DY; dth = req.dth;

  ROS_INFO_STREAM("starting call to generate!");

  if (! GenerateVerticalEllipse()) {
    res.feasible=false;
  return false;
  }
  res.feasible=true;
  return true;

}

//My traj
void TestTraj(){
  Eigen::Matrix<double,3,5> Xp;
  Xp.col(0) << 0.16763,0.4 ,0.3 ;
  Xp.col(1) << 0.16763,0.2 ,0.2 ;
  Xp.col(2) << 0.16763,0.1 ,0.1 ;
  Xp.col(3) << 0.16763,0.05,(0.045/2)+0.005 ;
  Xp.col(4) << 0.16763,0   ,(0.045/2) ;
  Eigen::Matrix<double,1,5> Tw;
  Tw << 1,2,3,4,5;
  if ( !generateQwp(Xp,Tw)){
    ROS_ERROR_STREAM("Error generating traj");
  };

  ROS_INFO_STREAM(Qw<<std::endl);
  ROS_INFO_STREAM(Qtw<<std::endl);
  ROS_INFO_STREAM(tw<<std::endl);


  PublishTrajectory();

}

//Publish trajectory
bool PublishTrajectory(){
  if (! ac.isServerConnected() ){
    // gain time;
    ROS_ERROR_STREAM("No action server is connected to publish trajectory. Trajectory is aborted!");
    return false;
  }
  trajectory_msgs::JointTrajectoryPoint TrajPoint;
  std::vector<trajectory_msgs::JointTrajectoryPoint> traj; //colection of TrajPoints

  //Populating traj
  for ( int i=0; i< NumberOfWp;i++){
    TrajPoint.positions  = std::vector<double> { Qw(0,i) ,Qw(1,i) ,Qw(2,i)  };
    TrajPoint.velocities = std::vector<double>{ Qtw(0,i),Qtw(1,i),Qtw(2,i) };
    TrajPoint.time_from_start = ros::Duration(tw(i));
    traj.push_back(TrajPoint);
  }

  //Populating traj_goal:
  traj_goal.trajectory.points =traj;
  traj_goal.trajectory.joint_names = std::vector<std::string>{"base_to_link1","link1_to_link2","link2_to_link3"};
  traj_goal.trajectory.header.stamp = ros::Time::now();

  //Send Goal:
  ac.sendGoal(traj_goal); 
  return true;

}

// this is not optimal
Eigen::Vector3d bestSol(Eigen::Vector3d Qs){
  double current_ang_dist;
  double min_ang_dist = 1000*M_PI;

  int index = 0;
  int nSols = L.get_nSols();
  
  Eigen::Vector3d Qd;

  //find minumum distance:
  for (int i = 0; i<nSols ; i++){

    Qd << ( L.getSols() ).col(i) ;
    current_ang_dist = fabs ( L.gDist(Qd,Qs).norm() );

    if (current_ang_dist < min_ang_dist ) {
      index = i;
      min_ang_dist  = current_ang_dist;
    }
  }

  //return minum distance
  return ( L.getSols() ).col(index) ;
}

//Generate Q from Cartesian WP:
bool generateQwp( Eigen::MatrixXd Xwp, Eigen::VectorXd twp){
    ROS_INFO_STREAM("reached qwp!");

  tw = twp; 
  using Eigen::seq ;
  using Eigen::seqN ;
  using Eigen::all; 

  const int N = Xwp.cols();
  NumberOfWp = N;
  bool  feasible_traj = true;

  //In contrast to matlab code, here the starting point is NOT included!
  Qw = Eigen::MatrixXd::Zero(3,N); 

  // Find Qw ----------------------- (Part 1)
  for (int i=0; i< N; i++){
    if ( !L.IK( Xwp.col(i) )){ 
      ROS_ERROR_STREAM("[Trajectory Controller]: The specified trajectory contains unreachable points. Dropping trajectory!");
      feasible_traj = false;
      break;
    }

    //Serially Check -> not optimal
    if (i==0){
        Qw.col(i) = bestSol( L.getState() );
    }else{
        Qw.col(i) = bestSol( Qw.col(i-1) );
    }  
  }

  // Find Velocities:
  Qtw = Eigen::MatrixXd::Zero(3,N); 

  //Find mean Velocity  ----------------------- (Part 2)
  Eigen::MatrixXd Ul = Eigen::MatrixXd::Zero(3,N-1);
  Eigen::MatrixXd Ur = Ul;
  Eigen::Vector3d MeanVel;

  //Create a diagonal matrix with 1/tw_i, in order to create derivatives.
  Eigen::MatrixXd _DT  = ( twp(seqN(1,N-1)) - twp(seqN(0,N-1)) ).asDiagonal() ;
  for (int i=0;i<N-1;i++){
    _DT(i,i) = 1/_DT(i,i);
  }

  // Create the differences: (l: left, r:right)
  Ul(all,0) = Qw(all,1) - L.getState()/tw(0); 
  Ul(all, seqN(1,N-2) ) << ( Qw(all,seqN(1,N-2)) - Qw(all,seqN(0,N-2)) )  * _DT(seqN(0,N-2),seqN(0,N-2));

  Ur = ( Qw(all,seqN(1,N-1)) - Qw(all,seqN(0,N-1)) )  * _DT(seqN(0,N-1),seqN(0,N-1)); 

  //calculate mean velocity;
  for(int c=0;c<N-1;c++){
    for(int r=0;r<3;r++){
        MeanVel(r) = ( Ul(r,c)* Ur(r,c) >0 )? 0.5*( Ul(r,c)+ Ur(r,c)) : 0 ;
    }
    Qtw.col(c) = MeanVel;
  }

  return feasible_traj;
}

void  setActive(const bool & state ){ CurrentlyActive = state;}
bool  getActive()                   { return CurrentlyActive;}

private:
 //robot
  Leg& L; //reference to the robot.

  //Waypoints
  int NumberOfWp; 
  Eigen::Matrix<double, 3,Eigen::Dynamic > Qw;
  Eigen::Matrix<double, 3,Eigen::Dynamic > Qtw;
  Eigen::Matrix<double, 1,Eigen::Dynamic > tw;

  //Ellipse parameters:
  double a,b,DX,DY,dth;
  int16_t times;

  //Action
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac; 
  control_msgs::FollowJointTrajectoryGoal traj_goal;


  // //Action msg
  // control_msgs::FollowJointTrajectoryAction act_msg;
  // control_msgs::FollowJointTrajectoryGoal traj_goal;

  // ROS
  ros::Publisher TrajPub;
  ros::ServiceServer EllipseServer;


  //logic flag
  bool isValid;  
  bool CurrentlyActive; 
  
};
