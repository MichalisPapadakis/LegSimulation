#include "leg_kinematics/leg_kinematics.hpp"


exIK::exIK(const std::string& str):
    ros::Exception::Exception(str) {}


bool LegKinematics::IK(){

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
