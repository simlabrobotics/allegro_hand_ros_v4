#ifndef __ALLEGRO_NODE_PD_H__
#define __ALLEGRO_NODE_PD_H__

#include "allegro_hand_driver/AllegroHandNode.h"


// Joint-space PD control of the Allegro hand.
//
// Allows you to save a position and command it to the hand controller.
// Controller gains are loaded from the ROS parameter server.
class AllegroNodePD : public AllegroHandNode {

 public:
  AllegroNodePD();

  ~AllegroNodePD();

  // Main spin code: just waits for messages.
  void doIt(bool polling = false);

  // Uses the String received command to set the hand into its home
  // position, or saves the grasp in order to go into PD control mode. Also
  // can turn the hand off.
  void libCmdCallback(const std_msgs::String::ConstPtr &msg);

  void setJointCallback(const sensor_msgs::JointState &msg);

  // Loads all gains and initial positions from the parameter server.
  void initController(const std::string &whichHand);

  // PD control happens here.
  void computeDesiredTorque();

 protected:
  // Handles defined grasp commands (std_msgs/String).
  ros::Subscriber lib_cmd_sub;

  // Subscribe to desired joint states, only so we can set control_hand_ to true
  // when we receive a desired command.
  ros::Subscriber joint_cmd_sub;

  // If this flag is true, the hand will be controlled (either in joint position
  // or joint torques). If false, desired torques will all be zero.
  bool control_hand_ = false;
};

#endif  // __ALLEGRO_NODE_PD_H__
