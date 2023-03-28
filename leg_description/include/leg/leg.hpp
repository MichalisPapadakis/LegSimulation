#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <angles/angles.h>


class exIK :public ros::Exception {
  public:
  exIK(const std::string& str);
};

class Leg {

public:

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

  //desired states:
  Eigen::Vector3d Xw;

  // Solutions
  double SOLS[3][4]; // 4 solutions maximum;
  int nSols;    
};