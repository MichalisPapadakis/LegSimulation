#include "leg/leg.hpp"


exIK::exIK(const std::string& str):
    ros::Exception::Exception(str) {}

void Leg::init(){
    //Set up state subsribers
    ros::NodeHandle n12;
    Joint1_pos = n12.subscribe("/leg/joint_states",1000,&Leg::PoseCallback,this);

    // ros::NodeHandle n22;
    // Joint2_pos = n22.subscribe("/leg/joint2_position_controller/state",1000,&Leg::PoseCallback2,this);

    // ros::NodeHandle n32;
    // Joint3_pos = n32.subscribe("/leg/joint3_position_controller/state",1000,&Leg::PoseCallback3,this);

    q(0) = 0; q(1)=0; q(2)=0;
}

void Leg::Querry_state(){
    ROS_INFO_STREAM("[Leg: ]  Current States: q1 = "<< q(0) <<", q2 = "<< q(1) <<", q3 = "<< q(2) <<"\n" );
};

void Leg::PoseCallback(const sensor_msgs::JointState::ConstPtr& msg) {
   q(0) = msg->position[0]; 
   q(1) = msg->position[1]; 
   q(2) = msg->position[2]; 
}


Eigen::Vector3d Leg::DK(Eigen::Vector3d Q){
    Eigen::Vector3d XE;

        XE(0) = LB0 + L12*cos(Q(0)) - L23*sin(Q(0))*sin(Q(1)) - L3E*sin(Q(0))*sin(Q(1) + Q(2)); 
        XE(1) = L01 + L3E*cos(Q(1) + Q(2)) + L23*cos(Q(1)); 
        XE(2) = H + L12*sin(Q(0)) + L23*cos(Q(0))*sin(Q(1)) + L3E*cos(Q(0))*sin(Q(1) + Q(2));
        

    return XE;
}

Eigen::Vector3d Leg::gDist(Eigen::Vector3d Qd, Eigen::Vector3d Qstart){
        Eigen::Vector3d Dist = Qd - Qstart ; 
        Dist(1) = angles::shortest_angular_distance(Qd(1), Qstart(1));
        
        return Dist;
}

Eigen::Vector3d Leg::currentDist(Eigen::Vector3d Qd){
    return gDist(Qd,q);
}

Eigen::Matrix3d  Leg::Calculate_Jv(Eigen::Vector3d Q){

    Eigen::Matrix3d Jv;

    Jv.row(0) << - L12*sin(Q(0)) - L23*cos(Q(0))*sin(Q(1)) - L3E*sin(Q(1) + Q(2))*cos(Q(0)), -sin(Q(0))*(L3E*cos(Q(1) + Q(2)) + L23*cos(Q(1))), -L3E*cos(Q(1) + Q(2))*sin(Q(0));
    Jv.row(1) <<                                                             0,          - L3E*sin(Q(1) + Q(2)) - L23*sin(Q(1)),         -L3E*sin(Q(1) + Q(2));
    Jv.row(2) <<   L12*cos(Q(0)) - L23*sin(Q(0))*sin(Q(1)) - L3E*sin(Q(1) + Q(2))*sin(Q(0)),  cos(Q(0))*(L3E*cos(Q(1) + Q(2)) + L23*cos(Q(1))),  L3E*cos(Q(1) + Q(2))*cos(Q(0));

    return Jv;

}

Eigen::Vector3d Leg::CalculateG(Eigen::Vector3d Q){
    Eigen::Vector3d G;
    // g = 9.81:
    // G(0) = 1.1898845*cos(Q(0)) + 0.0038749206*sin(Q(0)) + 0.0039136544*cos(Q(1))*sin(Q(0)) - 1.2083284*sin(Q(0))*sin(Q(1)) - 0.38472813*cos(Q(1))*sin(Q(0))*sin(Q(2)) - 0.38472813*cos(Q(2))*sin(Q(0))*sin(Q(1));
    // G(1) = 1.962e-9*cos(Q(0))*(196089770.0*cos(Q(1) + Q(2)) + 615868870.0*cos(Q(1) - 0.0032388883));
    // G(2) = 0.38472813*cos(Q(1) + Q(2))*cos(Q(0));

    // g = 9.8:
    G(0) = 1.188671564*cos(Q(0)) + 0.0038709706*sin(Q(0)) + 0.00390966492*cos(Q(1))*sin(Q(0)) - 1.20709666*sin(Q(0))*sin(Q(1)) - 0.3843359512*cos(Q(1))*sin(Q(0))*sin(Q(2)) - 0.3843359512*cos(Q(2))*sin(Q(0))*sin(Q(1));
    G(1) = 0.00000000196*cos(Q(0))*(196089771.0*cos(Q(1) + Q(2)) + 615868873.4*cos(Q(1) - 0.003238888299));
    G(2) = 0.3843359512*cos(Q(1) + Q(2))*cos(Q(0));

    return G;
};

Eigen::Vector3d Leg::CalculateG(){
    return CalculateG(q);
};

Eigen::Matrix3d  Leg::Calculate_Jv(){
    return Calculate_Jv(q);
}

bool Leg::IK(Eigen::Vector3d Xw){

//reset number of solutions
nSols = 0;
int nTh1 = 0;
double th1;
double x0,y0,z0;

//set positions as seen in the 0th frame 
x0 = Xw(2) -H;
y0 = LB0-Xw(0);
z0 = -Xw(1);


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
    SOLS(0,nTh1) = th1; 
    nTh1++;
}

if (abs(D)>1e-6){ // else identical solutions
    th1 = asin(( 2*L12*x0-sqrt(D) ) / ( 2*x0*x0+2*y0*y0) ); //-sqrt(D)
    if ( abs(L12-x0*sin(th1)+y0*cos(th1) ) < 1e-8) { 
        SOLS(0,2*(nTh1)) = th1; //i want to group the solutions with the same th1 -> [th1 th1 th2 th2;...;...]
        nTh1++;
    }
}
// Get th2 and th3 -------------------:
double th2,th3;
double K1,K2;
double c2,s2,det,A,B;

for (int i = 0; i<nTh1 ;i++){
    th1 = SOLS(0,2*i);    

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

    SOLS(0,nSols) = th1;
    SOLS(1,nSols) = th2; 
    SOLS(2,nSols) = th3 ;

    nSols++;

    // 2nd solution
    c2 = -( -A*K1 + B*K2 ) / det ;
    s2 =  ( -B*K1 - A*K2 ) / det ;

    th2 = atan2(s2,c2);

    SOLS(0,nSols) = th1;
    SOLS(1,nSols) = th2; 
    SOLS(2,nSols) = -th3 ;

    nSols++;
}

//Inform  if solution doesn't exists:
try{
    if (nSols == 0) throw( exIK("Solution to IK doesn't exist: Q(2) doesn't exists" ) ) ;
}catch( const exIK& e){
    ROS_ERROR_STREAM(e.what());
    return false; 
    }

return true;

}
