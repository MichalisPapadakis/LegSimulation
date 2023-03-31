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


/** @brief Trajectory Controller class. 
 * 
 * @note It is responsible for setting new trajectory targets (in cartesian space) through ros services. Also, 
 * 
 */
class TrajectoryController{
public:

/** @brief Trajectory Controller class constructor. 
 * 
 * @param  LegObj reference to an instance of an  object from the class `Leg`. That way, the states are 
 * linked in the different controllers   
 */
TrajectoryController(Leg& LegObj): 
L(LegObj), 
ac("leg/Trajectory_Controller/follow_joint_trajectory",true)  
// these objects do not have a default constructor.
{
  ros::NodeHandle NH; //public nodehandle

  //Start as Inactive Controller
  CurrentlyActive = false;

  //create timer to repeat trajectories
  //default time is 5s
  PublishingTimer = NH.createTimer(ros::Duration(5),&TrajectoryController::PublishingCallback,this);
  PublishingTimer.stop(); //start inactive

  //Set Ellipse Service Server
  EllipseServer = NH.advertiseService("SetEllipse",&TrajectoryController::SetEllipse, this);

};

/** @brief Set  `CurrentlyActive` status. Useful for HLC.
 * 
 * 
 * @param state true to activate, false to deactivate
 */
void  setActive(const bool & state ){ CurrentlyActive = state;}

/** @brief Get  `CurrentlyActive` status. Useful for HLC.
 * 
 * @returns `CurrentlyActive`. It is true if  active, otherwise false .
 */
bool  getActive()                   { return CurrentlyActive;}

/** @brief A test, manual trajectory to test the controller
 */
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

/** @brief Set ellipse parameters through the service call `SetEllipse`. 
 * Also set number of repetitions and duration of the ellipse.
 * 
 * @note The class attributes that are relevant to ellipse parameters are populated. Publish trajectory is invoked to populate the message
 * and send the initial repeatition. If `times`>1, then a timer is started to periodically republish the trajectory. 
 * Failure  may be unsuccesfull due to:
 * 1) failure of the ellipse waypoint generation,
 * 2) due to failure in publishing the trajectory or
 * 3) due to the trajectory controller being inactive
 * 4) due to inappropriate ellipse parameters
 * 
 * @returns true if the waypoint generation is succesful, false otherwise.
 * @throws `ros::InvalidParameterException` if a,b,period or times are not strictly positive
 */
bool SetEllipse(leg_control::ellipse::Request& req,leg_control::ellipse::Response & res){
  if (! CurrentlyActive) {
    ROS_ERROR_STREAM("[Trajectory Controller]: Trajectory controller is not currently active. Dropping ellipse");
    res.feasible=false;
    return false;
  }

  a = req.a;   b = req.b; times = req.times;
  DX = req.DX; DY = req.DY; dth = req.dth;
  period = req.Period;

  try {
    if (! (a>0)     ) throw  ros::InvalidParameterException("[Trajectory Controller]: `a` must be positive") ;
    if (! (b > 0)   ) throw  ros::InvalidParameterException("[Trajectory Controller]: `b` must be positive") ;
    if (!(times > 0)) throw  ros::InvalidParameterException("[Trajectory Controller]: `times` must be positive") ;
    if (!(period > 0))throw  ros::InvalidParameterException("[Trajectory Controller]: `period` must be positive") ;
  } catch (const  ros::InvalidParameterException & e){
    ROS_ERROR_STREAM( e.what() ) ;
    res.feasible=false;
    return false;
  }

  if (! GenerateVerticalEllipse()) {
    ROS_INFO_STREAM("[Trajectory Controller]: Problem with ellipse waypoint generation");
    res.feasible=false;
    return false;
  }

  // initial call to populate the `traj` message and send first (and maybe only) trajectory:
  if (! PublishTrajectory()){
    ROS_INFO_STREAM("[Trajectory Controller]: Problem with ellipse waypoint publishing");
    res.feasible=false;
    return false;

  }
  counter = 1; 
  ROS_INFO("[Trajectory Controller]: Started ellipse with a: %f, b: %f, DX: %f, Dy: %f, dth: %f",a,b,DX,DY,dth);

  //start timer, if we want the trajectory to repeat itself
  if (times > 1){
    PublishingTimer.setPeriod(ros::Duration(period));
    PublishingTimer.start();
  }

  res.feasible=true;
  return true;

}

private:

/** @brief `PublishingTimer` Callback function. For repeating a trajectory. 
 * 
 * @note To repeat a trajectory, after the message `traj` is populated in `PublishTrajectory`. A `ros::timer` callback function is invoked.
 * It checks whether the controller is still active, and the service server is connected. If both conditions are met, it send the trajectory
 * and increments the counter variable.  Finally, if the trajectory is repeated the specified number of times, the timer is stopped. 
 * 
 * @throws `ros::Exception` if the controller is not active (it was externally stopped) or (the action server is down). 
 */
void PublishingCallback(const ros::TimerEvent& ){ //supressing parameters in current implementation
  // checks
  try {
    if ( ! CurrentlyActive)        throw  ros::Exception("[Trajectory Controller]: Currently Trajectory Control is not active. \n[Trajectory Controller]: Trajectory was not executed all the specified times ") ;
    if (! ac.isServerConnected() ) throw  ros::Exception("[Trajectory Controller]: No action server is connected to publish trajectory. Trajectory is aborted!") ;
  } catch (const  ros::Exception & e){
    ROS_ERROR_STREAM( e.what() ) ;
    PublishingTimer.stop();
    return; 
  }

  // publish and increment timer:
  traj_goal.trajectory.header.stamp = ros::Time::now(); //set current time
  ac.sendGoal(traj_goal); 
  counter ++; 

  //stop timer if counter == timers:
  if (counter == times){
    ROS_INFO_STREAM("Stop publishing trajectory");
    PublishingTimer.stop();
  }

}

/** @brief Generate Cartesian waypoint for a vertical ellipse from the ellipse parameter passed by the user.
 * 
 * @note Generation is based on the polar form of an ellipse. Eigen::array is used for element-wise division and multipliation. 
 * Failure  may be unsuccesfull due to failure of the joint angle waypoint generation. 
 * 
 * @returns true if the waypoint generation is succesful, false otherwise.
 */
bool GenerateVerticalEllipse(){
  // Arrays for element wise multiplication and division:
  #define NpointsE 30
  using Eigen::placeholders::all;

  Eigen::MatrixXd X(3,NpointsE);
  Eigen::ArrayXd  th = Eigen::ArrayXd::LinSpaced( NpointsE, 2*M_PI/(NpointsE-1), 2*M_PI );

  Eigen::ArrayXd  r = a*b /                 //r = ab/sqrt( b^2 c(th-dth)^2 +  a^2 s(th-dth)^2 )
  ( 
    (b*b)*(   (th-dth).cos().pow(2)   )  +  // b^2*( cos(th-dth) )^2
    (a*a)*(   (th-dth).sin().pow(2)   )     // a^2*( sin(th-dth) )^2
  ).sqrt();

  //Populate X
  double Xo = L.DK(Eigen::Vector3d(0,0,0))  (0); // get first element of vector


  X.row(0) = Eigen::MatrixXd::Constant(1,NpointsE,Xo);
  X.row(1) = ( r*th.cos() + DX ).matrix();
  X.row(2) = ( r*th.sin() + DY ).matrix();

  Eigen::VectorXd t = Eigen::ArrayXd::LinSpaced(NpointsE,period/(NpointsE-1),period).matrix();

  if (! generateQwp(X,t) ) {
    
    ROS_ERROR_STREAM("[Trajectory Controller]: Problem With generation trajectory from ellipse");
    return false;
  };

  return true;


}

/** @brief Generate angular position waypoints from cartesian waypoints 
 * 
 * @note Checks if a point is reachable (solution to IK exists), then solution with the shortest distance in joint angle
 * space. This is **NOT OPTIMAL**. As for velocities, the mean velocity for each segment is calculated. Then for each setpoint,
 * if the velocity of the previous and next segment are in the same direction, their mean value is given. If the velocities point 
 * to the opposite direction, the desired velocity is set to 0.The generation can fail because: 1) a waypoint is unreachable, 
 * 2) timestamp is unreasonable
 * 
 * @param Xwp A 3xN `Eigen::matrix` with all the cartesian waypoints. Each column is a point in 3D space.
 * @param twp A Nx1 `Eigen::Vector` with the timestamps for each waypoint.
 * 
 * @returns true if the waypoint generation is successful, false otherwise. 
 */
bool generateQwp( Eigen::MatrixXd Xwp, Eigen::VectorXd twp){
  using Eigen::seq ;
  using Eigen::seqN ;
  using Eigen::all; 

  tw = twp; 

  const int N = Xwp.cols(); //number of columns = number of waypoints
  NumberOfWp = N;
  bool  feasible_traj = true;

  // ----------------------- (Part 1) --------------------------
  // ----------------------- Position --------------------------

  //In contrast to matlab code, here the starting point is NOT included!
  Qw = Eigen::MatrixXd::Zero(3,N); 
  
  for (int i=0; i< N; i++){
    if ( !L.IK( Xwp.col(i) )){ 
      ROS_ERROR_STREAM("[Trajectory Controller]: The specified trajectory contains unreachable points. Dropping trajectory!");
      feasible_traj = false;
      break;
    }

    //Serially find best  -> not optimal
    if (i==0){
        Qw.col(i) = bestSol( L.getState() );
    }else{
        Qw.col(i) = bestSol( Qw.col(i-1) );
    }  
  }

  // ----------------------- (Part 2) --------------------------
  // ----------------------- Velocity --------------------------
  Qtw = Eigen::MatrixXd::Zero(3,N); 

  //Create matrices
  Eigen::MatrixXd Ul = Eigen::MatrixXd::Zero(3,N-1);
  Eigen::MatrixXd Ur = Ul;
  Eigen::Vector3d MeanVel;

  //Create a diagonal matrix with 1/tw_i, in order to create derivatives.
  Eigen::MatrixXd _DT  = ( twp(seqN(1,N-1)) - twp(seqN(0,N-1)) ).asDiagonal() ;
  for (int i=0;i<N-1;i++){
    //check division by zero
    if (! _DT(i,i) >0){
      ROS_ERROR_STREAM("Two waypoints in the same timestamp. Infeasible trajectory! Aborting.");
      return false;
    }
    _DT(i,i) = 1/_DT(i,i);
  }

  // Create the derivatives of positions using finite differences: (l: left, r:right)
  Ul.col(0) = (   Qw.col(0) - L.getState()   )/twp(0); 
  Ul(all, seqN(1,N-2) ) << (   Qw(all,seqN(1,N-2)) - Qw(all,seqN(0,N-2))  )  * _DT(seqN(0,N-2),seqN(0,N-2));

  Ur =                     (   Qw(all,seqN(1,N-1)) - Qw(all,seqN(0,N-1))  )  * _DT(seqN(0,N-1),seqN(0,N-1)); 

  //calculate mean velocity. If velocities are in oposite direction, set 0 velocity:
  for(int c=0;c<N-1;c++){
    for(int r=0;r<3;r++){
        MeanVel(r) = ( Ul(r,c)* Ur(r,c) >0 ) ? 0.5*( Ul(r,c)+ Ur(r,c)) : 0 ;
    }
    Qtw.col(c) = MeanVel;
  }

  return feasible_traj;
}

/** @brief Populate the trajectory action message and publish it to the action server.
 * 
 * @note Publishing may be unsuccesfull due to a number of reasons:
 * 1) The action server is not running
 * 2) The trajectory controller in not active.
 * 
 * @returns true if publishing is successful. False if unsuccessful or the trajectory is publisher the require
 */
bool PublishTrajectory(){
  if ( ! CurrentlyActive) {
    ROS_ERROR_STREAM("[Trajectory Controller]: Currently Trajectory Control is not active.");
    return false;
  }
  if (! ac.isServerConnected() ){
    ROS_ERROR_STREAM("[Trajectory Controller]: No action server is connected to publish trajectory. Trajectory is aborted!");
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

/**  @brief Get best solution from the IK solution set.
 * 
 * @note Not Optimal: Currently, the best solution is the one with the smallest euclidean distance
 * in the joint state space. 
 * 
 * @param Qs starting position 
 * @returns a vector with the "best" desired states.
 */
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

 //robot
  Leg& L; //reference to the robot.

  //Waypoints
  int NumberOfWp; 
  Eigen::Matrix3Xd Qw;   //Angular Position Waypoints
  Eigen::Matrix3Xd Qtw;  //Angular Velocities at Waypoints
  Eigen::VectorXd tw;    //timestamps for Waypoints

  //Ellipse parameters:
  double a,b,DX,DY,dth,period;  //ellipse parameters
  int16_t times;         //times to repeat an ellipse
  int16_t counter;       //counter for the trajectories

  //Action
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac;  //action client
  control_msgs::FollowJointTrajectoryGoal traj_goal;                            //message for the trajectory

  // ROS
  ros::ServiceServer EllipseServer;  //Set Ellipse service  server
  ros::Timer PublishingTimer; 

  //logic flag
  bool CurrentlyActive; 
  
};
