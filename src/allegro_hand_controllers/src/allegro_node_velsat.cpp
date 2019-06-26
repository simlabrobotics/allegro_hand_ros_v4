// JOINT SPACE VELOCITY SATURATION CONTROL
// Using  timer callback

//////////////////////////////////////////////////////
// WARNING: THIS CONTROL CODE IS UNDER DEVELOPMENT ///
// USE AT YOUR OWN RISK                            ///
//////////////////////////////////////////////////////

#include "ros/ros.h"
#include "allegro_node_velsat.h"

// Topics
const std::string JOINT_DESIRED_TOPIC = "allegroHand/joint_desired_states";
const std::string JOINT_CURRENT_TOPIC = "allegroHand/joint_current_states";

#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI))
#define DEGREES_TO_RADIANS(angle) ((angle) / 180.0 * M_PI)

double desired_position[DOF_JOINTS] = {0.0};
double current_position[DOF_JOINTS] = {0.0};
double previous_position[DOF_JOINTS] = {0.0};
double current_position_filtered[DOF_JOINTS] = {0.0};
double previous_position_filtered[DOF_JOINTS] = {0.0};

double desired_velocity[DOF_JOINTS] = {0.0};
double current_velocity[DOF_JOINTS] = {0.0};
double previous_velocity[DOF_JOINTS] = {0.0};
double current_velocity_filtered[DOF_JOINTS] = {0.0};

double desired_torque[DOF_JOINTS] = {0.0};

double v[DOF_JOINTS] = {0.0};

double k_p[DOF_JOINTS] =
        {
                // Default P Gains for PD Controller, loaded if
                // 'gains_pd.yaml' file is not loaded.
                1200.0, 1200.0, 1200.0, 1200.0, 1200.0, 1200.0, 1200.0, 1200.0,
                1200.0, 1200.0, 1200.0, 1200.0, 1200.0, 1200.0, 1200.0, 1200.0
        };

double k_d[DOF_JOINTS] =
        {
                // Default D Gains for PD Controller, loaded if
                // 'gains_pd.yaml' file is not loaded.
                140.0, 140.0, 140.0, 140.0, 140.0, 140.0, 140.0, 140.0,
                140.0, 140.0, 140.0, 140.0, 140.0, 140.0, 140.0, 140.0
        };

double v_max[DOF_JOINTS] =
        {
                // Velocity limits. With a max of 10rad/s, 6rad/s is achieved
                // within the ~90 degree range of each finger joint.
                // These values are used if the 'gains_velSat.yaml' file is
                // not loaded.
                10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
                10.0, 10.0, 10.0, 10.0, 10.0, 10.0
        };

double home_pose[DOF_JOINTS] =
        {
                // Default (HOME) position (degrees), set at system start if
                // no 'initial_position.yaml' parameter is loaded.
                0.0, -10.0, 45.0, 45.0,  0.0, -10.0, 45.0, 45.0,
                5.0, -5.0, 50.0, 45.0, 60.0, 25.0, 15.0, 45.0
        };

std::string pGainParams[DOF_JOINTS] =
        {
                "~gains_velSat/p/j00", "~gains_velSat/p/j01",
                "~gains_velSat/p/j02", "~gains_velSat/p/j03",
                "~gains_velSat/p/j10", "~gains_velSat/p/j11",
                "~gains_velSat/p/j12", "~gains_velSat/p/j13",
                "~gains_velSat/p/j20", "~gains_velSat/p/j21",
                "~gains_velSat/p/j22", "~gains_velSat/p/j23",
                "~gains_velSat/p/j30", "~gains_velSat/p/j31",
                "~gains_velSat/p/j32", "~gains_velSat/p/j33"
        };

std::string dGainParams[DOF_JOINTS] =
        {
                "~gains_velSat/d/j00", "~gains_velSat/d/j01",
                "~gains_velSat/d/j02", "~gains_velSat/d/j03",
                "~gains_velSat/d/j10", "~gains_velSat/d/j11",
                "~gains_velSat/d/j12", "~gains_velSat/d/j13",
                "~gains_velSat/d/j20", "~gains_velSat/d/j21",
                "~gains_velSat/d/j22", "~gains_velSat/d/j23",
                "~gains_velSat/d/j30", "~gains_velSat/d/j31",
                "~gains_velSat/d/j32", "~gains_velSat/d/j33"
        };

std::string vMaxParams[DOF_JOINTS] =
        {
                "~gains_velSat/v_max/j00", "~gains_velSat/v_max/j01",
                "~gains_velSat/v_max/j02", "~gains_velSat/v_max/j03",
                "~gains_velSat/v_max/j10", "~gains_velSat/v_max/j11",
                "~gains_velSat/v_max/j12", "~gains_velSat/v_max/j13",
                "~gains_velSat/v_max/j20", "~gains_velSat/v_max/j21",
                "~gains_velSat/v_max/j22", "~gains_velSat/v_max/j23",
                "~gains_velSat/v_max/j30", "~gains_velSat/v_max/j31",
                "~gains_velSat/v_max/j32", "~gains_velSat/v_max/j33"
        };

std::string initialPosition[DOF_JOINTS] =
        {
                "~initial_position/j00", "~initial_position/j01",
                "~initial_position/j02", "~initial_position/j03",
                "~initial_position/j10", "~initial_position/j11",
                "~initial_position/j12", "~initial_position/j13",
                "~initial_position/j20", "~initial_position/j21",
                "~initial_position/j22", "~initial_position/j23",
                "~initial_position/j30", "~initial_position/j31",
                "~initial_position/j32", "~initial_position/j33"
        };

std::string jointNames[DOF_JOINTS] =
        {
                "joint_0.0", "joint_1.0", "joint_2.0", "joint_3.0",
                "joint_4.0", "joint_5.0", "joint_6.0", "joint_7.0",
                "joint_8.0", "joint_9.0", "joint_10.0", "joint_11.0",
                "joint_12.0", "joint_13.0", "joint_14.0", "joint_15.0"
        };


AllegroNodeVelSat::AllegroNodeVelSat()
        : AllegroNode() {
  initController();

  msgJoint_desired.position.resize(DOF_JOINTS);
  msgJoint_desired.velocity.resize(DOF_JOINTS);
  msgJoint_desired.effort.resize(DOF_JOINTS);
  msgJoint_desired.name.resize(DOF_JOINTS);

  msgJoint_current.position.resize(DOF_JOINTS);
  msgJoint_current.velocity.resize(DOF_JOINTS);
  msgJoint_current.effort.resize(DOF_JOINTS);
  msgJoint_current.name.resize(DOF_JOINTS);

  joint_cmd_sub = nh.subscribe(
      DESIRED_STATE_TOPIC, 3, &AllegroNodeVelSat::setJointCallback, this);
  lib_cmd_sub = nh.subscribe(
          LIB_CMD_TOPIC, 1, &AllegroNodeVelSat::libCmdCallback, this);

  // Publisher and Subscribers
  joint_state_pub = nh.advertise<sensor_msgs::JointState>(JOINT_STATE_TOPIC, 3);
  joint_desired_state_pub = nh.advertise<sensor_msgs::JointState>(
          JOINT_DESIRED_TOPIC, 3);
  joint_current_state_pub = nh.advertise<sensor_msgs::JointState>(
          JOINT_CURRENT_TOPIC, 3);
}

AllegroNodeVelSat::~AllegroNodeVelSat() {
}

// Called when a desired joint position message is received
void AllegroNodeVelSat::setJointCallback(const sensor_msgs::JointState &msg) {
  //  	printf("frame = %ld: setting desired pos\n", frame);
  mutex->lock();
  for (int i = 0; i < DOF_JOINTS; i++) desired_position[i] = msg.position[i];
  mutex->unlock();
  controlPD = true;
}

void AllegroNodeVelSat::libCmdCallback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("CTRL: Heard: [%s]", msg->data.c_str());

  const std::string lib_cmd = msg->data.c_str();

  // Compare the message received to an expected input
  if (lib_cmd.compare("pdControl") == 0)
    controlPD = true;

  else if (lib_cmd.compare("home") == 0) {
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_position[i] = DEGREES_TO_RADIANS(home_pose[i]);
    controlPD = true;
  }
  else if (lib_cmd.compare("off") == 0)
    controlPD = false;

  else if (lib_cmd.compare("save") == 0)
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_position[i] = current_position[i];
}

// In case of the Allegro Hand, this callback is processed
// every 0.003 seconds
void AllegroNodeVelSat::computeDesiredTorque() {
  /*  =================================
      =       VELOCITY SATURATION     =
      ================================= */
  // maxVelocity = 10 rad/s (dead to home) -> 13 rad/s (envelope) is a high max
  // maxTorque = 0.7 N.m

  if (controlPD) {
    for (int i = 0; i < DOF_JOINTS; i++) {
      desired_velocity[i] = (k_p[i] / k_d[i]) * (desired_position[i] -
                                                 current_position_filtered[i]);
      v[i] = std::min(1.0, v_max[i] /
                           fabs(desired_velocity[i]));  // absolute value for floats
      desired_torque[i] = -k_d[i] * (current_velocity_filtered[i] -
                                     v[i] * desired_velocity[i]);
      desired_torque[i] = desired_torque[i] / canDevice->torqueConversion();
    }
  }
  else {
    for (int i = 0; i < DOF_JOINTS; i++) desired_torque[i] = 0.0;
  }
}

void AllegroNodeVelSat::initController() {
  // set gains via gains_velSat.yaml or to defaul values
  if (ros::param::has("~gains_velSat")) {
    ROS_INFO("CTRL: Velocity Saturation gains loaded from param server.");
    for (int i = 0; i < DOF_JOINTS; i++) {
      ros::param::get(pGainParams[i], k_p[i]);
      ros::param::get(dGainParams[i], k_d[i]);
      ros::param::get(vMaxParams[i], v_max[i]);
      //printf("%f ", k_p[i]);
    }
    //printf("\n");
  }
  else {
    // gains will be loaded every control iteration
    ROS_WARN(
            "CTRL: Velocity Satuartion gains not loaded.\nCheck launch file is loading /parameters/gains_velSat.yaml\nLoading default Vel. Sat. gains...");
  }

  // set initial position via initial_position.yaml or to defaul values
  if (ros::param::has("~initial_position")) {
    ROS_INFO("\n\nCTRL: Initial Pose loaded from param server.\n");
    for (int i = 0; i < DOF_JOINTS; i++) {
      ros::param::get(initialPosition[i], desired_position[i]);
      desired_position[i] = DEGREES_TO_RADIANS(desired_position[i]);
    }
  }
  else {
    ROS_WARN(
            "\n\nCTRL: Initial postion not loaded.\nCheck launch file is loading /parameters/initial_position.yaml\nloading Home position instead...\n");
    // Home position
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_position[i] = DEGREES_TO_RADIANS(home_pose[i]);
  }
  controlPD = false;

  printf("*************************************\n");
  printf("     Velocity Saturation Method      \n");
  printf("-------------------------------------\n");
  printf("  Only 'H', 'O', 'S', 'Space' works. \n");
  printf("*************************************\n");
}

void AllegroNodeVelSat::publishData() {
  // Superclass publishes the current_joint_state data.
  AllegroNode::publishData();

  // Additionally, publish the msgJoint_desired and msgJoint_current data.
  msgJoint_desired.header.stamp = tnow;
  msgJoint_current.header.stamp = tnow;
  for (int i = 0; i < DOF_JOINTS; i++) {
    msgJoint_desired.position[i] = desired_position[i];
    msgJoint_desired.velocity[i] = desired_velocity[i] * v[i];
    msgJoint_desired.effort[i] = desired_torque[i];

    msgJoint_current.position[i] = current_position_filtered[i];
    msgJoint_current.velocity[i] = current_velocity_filtered[i];
    msgJoint_current.effort[i] = desired_torque[i];  // Just for plotting. Not actually current torque,
  }
  joint_desired_state_pub.publish(msgJoint_desired);
  joint_current_state_pub.publish(msgJoint_current);
}

void AllegroNodeVelSat::doIt(bool polling) {
  // Main spin loop, uses the publisher/subscribers.
  if (polling) {
    ROS_INFO("Polling = true.");
    while (ros::ok()) {
      updateController();
      ros::spinOnce();
    }
  } else {
    ROS_INFO("Polling = false.");

    // Timer callback (not recommended).
    ros::Timer timer = startTimerCallback();
    ros::spin();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "allegro_hand_core_velsat");

  AllegroNodeVelSat allegroNode;

  bool polling = false;
  if (argv[1] == std::string("true")) {
    polling = true;
  }
  allegroNode.doIt(polling);
}
