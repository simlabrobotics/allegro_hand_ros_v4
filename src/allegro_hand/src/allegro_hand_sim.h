#ifndef __ALLEGRO_NODE_SIM_H__
#define __ALLEGRO_NODE_SIM_H__

#include "allegro_hand_driver/AllegroHandNode.h"

// Simulated Allegro Hand.
//
// Simulated is probably a generous term, this is simply a pass-through for
// joint states: commanded -> current.
//
// It works by overriding updateWriteReadCAN and setting the current position in
// computeDesiredTorque.
class AllegroNodeSim : public AllegroHandNode {

 public:
  AllegroNodeSim();

  ~AllegroNodeSim();

  // Main spin code: just waits for messages.
  void doIt(bool polling = false);

  // Loads all gains and initial positions from the parameter server.
  void initController(const std::string &whichHand);

  void computeDesiredTorque();

  void updateWriteReadCAN();

 protected:
};

#endif  // __ALLEGRO_NODE_SIM_H__
