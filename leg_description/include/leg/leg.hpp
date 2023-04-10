#pragma once


#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointControllerState.h>
#include <eigen3/Eigen/Dense>
#include <angles/angles.h>

typedef Eigen::Matrix<double, 3, 4 > Matrix3_4d; 

/**
 * Class for exeptions regarding the Inverse Kinematics of the Leg class
 * 
 */
class exIK :public ros::Exception {
  public:
  exIK(const std::string& str);
};


/**
 * Class containing important geometric attributes and geometric-related functions for the robotic leg
 *
 * Leg class subscribes to the relevant topics to read the joint states of the robot and contains all 
 * usefull kinematics functions (`IK`: inverse kinematics, `DK`: Direct Kinematics, Calculate_Jv: Geometric Jacobian) .
 * The solutions from the last IK call are saves as a class attribute.
 * 
 */
class Leg {

public:

// ROS Utilities:
#pragma region
/** @brief Initialize leg in order to set state subscribers
 * 
 */
void init();

private:
/** @brief Callback function to get state of first joint
 * 
 */
void PoseCallback(const sensor_msgs::JointState::ConstPtr& msg) ;


#pragma endregion

public: 
/** @brief Function to inform user of the current state of the robot. Only prints!
 * 
 */
void Querry_state();


// Robot Kinematics:
#pragma region
/** @brief Direct Kinematics for the leg
 *
 * @param Q the vector of joint angular positions
 * @returns the vector containing the end_effector position in the world frame.  
 */
Eigen::Vector3d DK(Eigen::Vector3d Q);

/** @brief True distance between joint positions in joint state space 
 *
 * @param Qd The vector of the desired joint positions
 * @param Qstart The vector of the starting joint positions
 * @returns the vector containing the angular distance of each joint 
 */
Eigen::Vector3d gDist(Eigen::Vector3d Qd, Eigen::Vector3d Qstart);

/** @brief True distance between joint positions in joint state space starting from the current robot position
 *
 * @param Qd The vector of the desired joint positions
 * @returns the vector containing the angular distance of each joint 
 */
Eigen::Vector3d currentDist(Eigen::Vector3d Qd);

/** @brief Inverse Kinematics for the leg
 *
 * @param Xw The vector of the desired end_effector point in the world frame
 * @throws `exIK` If there is no solution to the inverse kinematics problem
 * @returns true if there are solutions to the IK problem, false otherwise.  
 */
bool IK(Eigen::Vector3d Xw);

/** @brief Geometric Jacobian
 *
 * @param Xw The vector of the desired end_effector point in the world frame
 * @throws exIK If there is no solution to the inverse kinematics problem
 * @returns `Jv` 
 */
Eigen::Matrix3d  Calculate_Jv(Eigen::Vector3d Q);

/** @brief Calculate the gravitational matrix for a given state
 *
 * @param Q state of the robot
 * @returns `G` 
 */
Eigen::Vector3d CalculateG(Eigen::Vector3d Q);

/** @brief Calculate the gravitational matrix for current state
 *
 *  @returns `G` 
 */
Eigen::Vector3d CalculateG();

/** @brief Geometric Jacobian using current position
 *
 * @returns `Jv` 
 */
Eigen::Matrix3d  Calculate_Jv();

// Getters
/** @brief get current leg state
 *
 * @returns `q`
 */
Eigen::Vector3d getState(){
  return q; 
}

/** @brief get the number of solutions
 *
 * @returns `nSols`
 */
int  get_nSols(){

  return nSols;
}

/** @brief return the matrix containing all the solutions to the IK problem
 *
 * @returns `SOLS`
 */
Matrix3_4d getSols(){
  return SOLS;
};

#pragma endregion

protected: // they can be inhereted

// geometry
  double LB0 = 0.05555;
  double L01 = 0.07763;
  double L12 = 0.11208;
  double L23 = 0.2531;
  double L3E = 0.223;
  double H = 0.4;

  // states
  Eigen::Vector3d q;
  Eigen::Vector3d qt;

  //end effector cartesian coordinates
  Eigen::Vector3d Xe;

  // Currrent Jacobian
  Eigen::Matrix3d Jv; 

  // Solutions
  Matrix3_4d SOLS; // 4 solutions maximum; // Eigen::fix to set the matrix size at compile time.
  int nSols;    

  // Ros specific:
  ros::Subscriber Joint1_pos;
  ros::Subscriber Joint2_pos;
  ros::Subscriber Joint3_pos;
};