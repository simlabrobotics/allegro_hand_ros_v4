#ifndef PROJECT_ALLEGRO_NODE_VELSAT_H
#define PROJECT_ALLEGRO_NODE_VELSAT_H

#include "allegro_node.h"
#include "ros/ros.h"

class AllegroNodeVelSat : public AllegroNode {

 public:

    AllegroNodeVelSat();
    ~AllegroNodeVelSat();

    void computeDesiredTorque();

    void publishData();

    void initController();

    void libCmdCallback(const std_msgs::String::ConstPtr& msg);

    void setJointCallback(const sensor_msgs::JointState& msg);

    void doIt(bool polling);

 protected:

    bool controlPD = false;


    ros::Publisher joint_desired_state_pub;
    ros::Publisher joint_current_state_pub;

    ros::Subscriber joint_cmd_sub;
    ros::Subscriber lib_cmd_sub;

    sensor_msgs::JointState msgJoint_desired;	// Desired Position, Desired Velocity, Desired Torque
    sensor_msgs::JointState msgJoint_current;	// Current Position, Current Velocity, NOTE: Current Torque Unavailable


};

#endif //PROJECT_ALLEGRO_NODE_VELSAT_H
