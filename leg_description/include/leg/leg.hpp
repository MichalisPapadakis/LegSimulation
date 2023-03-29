#include <ros/ros.h>
#include <control_msgs/JointControllerState.h>
#include <eigen3/Eigen/Dense>
#include <angles/angles.h>

typedef Eigen::Matrix<double, 3, 4 > Matrix3_4d; 


class exIK :public ros::Exception {
  public:
  exIK(const std::string& str);
};

class Leg {

public:

// ROS Utilities:
#pragma region
void init();

void PoseCallback1(const control_msgs::JointControllerState::ConstPtr& msg) ;
void PoseCallback2(const control_msgs::JointControllerState::ConstPtr& msg) ;
void PoseCallback3(const control_msgs::JointControllerState::ConstPtr& msg) ;

void Querry_state();

#pragma endregion

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

Eigen::Vector3d currentDist(Eigen::Vector3d Qd);

/** @brief Inverse Kinematics for the leg
 *
 * @param Xw The vector of the desired end_effector point in the world frame
 * \throws exIK If there is no solution to the inverse kinematics problem
 * @returns true if there are solutions to the IK problem, false otherwise.  
 */
bool IK(Eigen::Vector3d Xw);

/** @brief Geometric Jacobian
 *
 * @param Xw The vector of the desired end_effector point in the world frame
 * \throws exIK If there is no solution to the inverse kinematics problem
 */
Eigen::Matrix3d  Calculate_Jv(Eigen::Vector3d Q);

Eigen::Vector3d CalculateG(Eigen::Vector3d Q);

Eigen::Vector3d CalculateG();

/** @brief Geometric Jacobian using current position
 *
 * \throws exIK If there is no solution to the inverse kinematics problem
 */
Eigen::Matrix3d  Calculate_Jv();

// Getters
Eigen::Vector3d getState(){
  return q; 
}

int  get_nSols(){

  return nSols;
}

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

  //Dynamics -> advanced features
  // double M[3][3];
  // double C[3][3];
  // double G[3][1];

  // Solutions
  Matrix3_4d SOLS; // 4 solutions maximum; // Eigen::fix to set the matrix size at compile time.
  int nSols;    

  // Ros specific:
  ros::Subscriber Joint1_pos;
  ros::Subscriber Joint2_pos;
  ros::Subscriber Joint3_pos;
};