#ifndef __ALLEGRO_NODE_SIM_H__
#define __ALLEGRO_NODE_SIM_H__

#include "allegro_node.h"


// Simulated Allegro Hand.
//
// Simulated is probably a generous term, this is simply a pass-through for
// joint states: commanded -> current.
class AllegroNodeSim : public AllegroNode {

 public:
  AllegroNodeSim();

  ~AllegroNodeSim();

  // Main spin code: just waits for messages.
  void doIt(bool polling = false);

  // Loads all gains and initial positions from the parameter server.
  void initController(const std::string &whichHand);

  void computeDesiredTorque();

 protected:
};

#endif  // __ALLEGRO_NODE_SIM_H__
