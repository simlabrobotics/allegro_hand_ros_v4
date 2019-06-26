#ifndef __ALLEGRO_NODE_TORQUE_H__
#define __ALLEGRO_NODE_TORQUE_H__

#include "allegro_hand_driver/AllegroHandNode.h"


// Joint-space torque control of the Allegro hand.
//
// Listens to allegroHand/torque_cmd topic and sets the desired joint torques
// directly.
class AllegroNodeTorque : public AllegroHandNode {

 public:
    AllegroNodeTorque();

    ~AllegroNodeTorque();

    // Main spin code: just waits for messages.
    void doIt(bool polling = false);

    // Sets desired joint torques based on the effort in a JointState message.
    void setTorqueCallback(const sensor_msgs::JointState &msg);

    // Turns torque control on or off.
    void libCmdCallback(const std_msgs::String::ConstPtr &msg);

    // No gains in torque control mode.
    void initController(const std::string &whichHand);

    // Since the desired torque is already stored (received in message
    // callback), just sets the desired torque to zero if torque control is off.
    void computeDesiredTorque();

 protected:
    // Handles external torque command (sensor_msgs/JointState).
    ros::Subscriber torque_cmd_sub;

    // Handles on/off commands (std_msgs/String).
    ros::Subscriber lib_cmd_sub;

    // If true, torque control is active.
    bool controlTorque = false;

};

#endif  // __ALLEGRO_NODE_TORQUE_H__
