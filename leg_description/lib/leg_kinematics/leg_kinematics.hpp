#include <ros/ros.h>


class exIK :public ros::Exception {
  public:
  exIK(const std::string& str);
};

class LegKinematics {

public:
bool IK();


protected: // they can be inhereted

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
};