#include <ros/ros.h>
#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/ContactState.h>
#include <std_msgs/Float64.h>

class ContactMsgHandler {

public:

ContactMsgHandler(){
    ros::NodeHandle n1;
    S = n1.subscribe("/end_effector_collision",10,&ContactMsgHandler::ContactCallback,this);

    ros::NodeHandle n2;
    P = n2.advertise<std_msgs::Float64>("/contact_force",10);
}


void ContactCallback(const gazebo_msgs::ContactsStateConstPtr & msg){
    double Fx, Fy,time;
    time = msg->header.stamp.toSec();

    if  (msg->states.size()==0  ){
        F = 0;
    }else{
        Fx = msg->states[0].total_wrench.force.x ;
        Fy = msg->states[0].total_wrench.force.y; 
        F = sqrt(Fx*Fx + Fy*Fy);

    }


    std_msgs::Float64 Pmsg;
    Pmsg.data = F;
    P.publish(Pmsg);
}

private:
    double F;
    ros::Subscriber S;
    ros::Publisher P;

};




int main(int argc, char **argv){
    ros::init(argc, argv, "ContactHandler");


ContactMsgHandler CH;

ros::Rate r(100);

while(ros::ok())
{

    ros::spinOnce(); //callbacks
    r.sleep(); //sleep

}


return 0;
}