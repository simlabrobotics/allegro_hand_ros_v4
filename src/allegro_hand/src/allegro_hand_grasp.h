#ifndef PROJECT_ALLEGRO_NODE_GRASP_H
#define PROJECT_ALLEGRO_NODE_GRASP_H

#include "allegro_hand_driver/AllegroHandNode.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

// Forward class declaration.
class BHand;

// Grasping controller that uses the BHand library for commanding various
// pre-defined grasp (e.g., three-finger ping, envelop, etc...).
//
// This node is most useful when run with the keyboard node (the keyboard node
// sends the correct String to this node). A map from String command -> Grasp
// type is defined in the implementation (cpp) file.
//
// This node can also save & hold a position, but in constrast to the PD node
// you do not have any control over the controller gains.
//
// Author: Felix Duvallet
//
class AllegroNodeGrasp : public AllegroHandNode {

 public:

    AllegroNodeGrasp();

    ~AllegroNodeGrasp();

    void initController(const std::string &whichHand);

    void computeDesiredTorque();

    void libCmdCallback(const std_msgs::String::ConstPtr &msg);

    void setJointCallback(const sensor_msgs::JointState &msg);

    void envelopTorqueCallback(const std_msgs::Float32 &msg);

    void doIt(bool polling);

 protected:

    // Handles external joint command (sensor_msgs/JointState).
    ros::Subscriber joint_cmd_sub;

    // Handles defined grasp commands (std_msgs/String).
    ros::Subscriber lib_cmd_sub;

    ros::Subscriber envelop_torque_sub;

    // Initialize BHand
    BHand *pBHand = NULL;

  double desired_position[DOF_JOINTS] = {0.0};
};

#endif //PROJECT_ALLEGRO_NODE_GRASP_H
