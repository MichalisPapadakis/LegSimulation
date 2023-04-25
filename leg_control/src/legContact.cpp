#include <ros/ros.h>
//read contacts
#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/ContactState.h>
//publish transformed contacts
#include <std_msgs/Float32MultiArray.h>
// Tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // for do transform implementation

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

/** @brief Class responsible for publishing contact messages from gazebo_ros bumper. It transform the forces in the world frame.
 * 
 * 
 */
class ContactMsgHandler {

public:

/** @brief ContactMsgHandler class constructor. It sets the subscriber to contact published by gazebo_ros bumper and the 
 * publisher that publishes the transformed contacts. 
 * 
 */
ContactMsgHandler(): 
tfListener(tfBuffer)
{
    ros::NodeHandle n1;
    S = n1.subscribe("/end_effector_collision",10,&ContactMsgHandler::ContactCallback,this);

    ros::NodeHandle n2;
    P = n2.advertise<std_msgs::Float32MultiArray>("/contact_force",10);

    Fc.transform.translation = geometry_msgs::Vector3() ; //zero vector
}

private:

/** @brief Gazebo_ros bumper subscriber callback function. It gets the contact forces, then the transform information. 
 * And it publishes the transformed contact forces.
 *
 */
void ContactCallback(const gazebo_msgs::ContactsStateConstPtr & msg){
    

    // get contact info and apply transform
    if  (msg->states.size()==0  ){
        Fc.transform.translation = geometry_msgs::Vector3() ; //zero vector
    }else{
        // get transform info
        try{
        transformStamped = tfBuffer.lookupTransform("world", "end_effector",
                                ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        }   

        //get force
        Fc.transform.translation.x = msg->states[0].total_wrench.force.x;
        Fc.transform.translation.y = msg->states[0].total_wrench.force.y;
        Fc.transform.translation.z = msg->states[0].total_wrench.force.z;

        //transform
        tf2::doTransform<geometry_msgs::TransformStamped>(Fc,Fc,transformStamped);
    }

    //populate message
    std_msgs::Float32MultiArray Pmsg;
    Pmsg.data.push_back(Fc.transform.translation.x);
    Pmsg.data.push_back(Fc.transform.translation.y);
    Pmsg.data.push_back(Fc.transform.translation.z);

    //publish
    P.publish(Pmsg);
}

    geometry_msgs::TransformStamped Fc;

    geometry_msgs::TransformStamped transformStamped;

    ros::Subscriber S;
    ros::Publisher P;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

};




int main(int argc, char **argv){
    
ros::init(argc, argv, "ContactHandler");

ContactMsgHandler CH;

ros::Rate r(1000);

while(ros::ok())
{

    ros::spinOnce(); //callbacks
    r.sleep(); //sleep

}

return 0;
}