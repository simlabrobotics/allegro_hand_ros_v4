#include <ros/ros.h>

#include "std_msgs/String.h"
//#include "sensor_msgs/JointState.h"

#include <iostream>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>

#include "virtualkey_codes.h"

using namespace std;

#define DOF_JOINTS 16


class AHKeyboard
{
public:
  AHKeyboard();
  void keyLoop();
  void printUsage();

private:
  ros::NodeHandle nh_;
  ros::Publisher cmd_pub_;
};

AHKeyboard::AHKeyboard()
{
  cmd_pub_ = nh_.advertise<std_msgs::String>("allegroHand/lib_cmd", 10);
}


int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "allegro_hand_keyboard");
  AHKeyboard allegro_hand_keyboard_cmd;

  signal(SIGINT,quit);

  allegro_hand_keyboard_cmd.keyLoop();

  return(0);
}

void AHKeyboard::printUsage() {
  std::cout << std::endl;
  std::cout << " -----------------------------------------------------------------------------" << std::endl;
  std::cout << "  Use the keyboard to send Allegro Hand grasp & motion commands" << std::endl;
  std::cout << " -----------------------------------------------------------------------------" << std::endl;

  std::cout << "\tHome Pose:\t\t\t'H'" << std::endl;
  std::cout << "\tReady Pose:\t\t\t'R'" << std::endl;
  std::cout << "\tPinch (index+thumb):\t\t'P'" << std::endl;
  std::cout << "\tPinch (middle+thumb):\t\t'M'" << std::endl;
  std::cout << "\tGrasp (3 fingers):\t\t'G'" << std::endl;
  std::cout << "\tGrasp (4 fingers):\t\t'F'" << std::endl;
  std::cout << "\tGrasp (envelop):\t\t'E'" << std::endl;
  std::cout << "\tGravity compensation:\t\t'Z'" << std::endl;
  std::cout << "\tMotors Off (free motion):\t'O'" << std::endl;
  std::cout << "\tSave Current Pose:\t\t'S'" << std::endl;
  std::cout << "\tPD Control (last saved):\t'Space'" << std::endl;
  std::cout << "\tHelp (this message):\t\t'/ or ?'" << std::endl;
  std::cout << " -----------------------------------------------------------------------------" << std::endl;
  //std::cout << "  Note: Unless elsewhere implemented, these keyboard commands only work with " << std::endl;
  //std::cout << "  the 'allegro_hand_core_grasp' and 'allegro_hand_core_grasp_slp' packages." << std::endl;
  std::cout << "  Subscriber code for reading these messages is included in '~core_template'." << std::endl;
  std::cout << " -----------------------------------------------------------------------------\n" << std::endl;

}

void AHKeyboard::keyLoop()
{
  char c;
  bool dirty=false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  sleep(2);
  printUsage();

  for(;;)
  {
    std_msgs::String msg;
    std::stringstream ss;

    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    ROS_DEBUG("value: 0x%02X\n", c);
    switch(c)
    {
      case VK_SPACE:
        ROS_DEBUG("space bar: PD Control");
        ss << "pdControl";
        dirty = true;
        break;
      case KEYCODE_h:
        ROS_DEBUG("h_key: Home");
        ss << "home";
        dirty = true;
        break;
      case KEYCODE_r:
        ROS_DEBUG("r_key: Ready");
        ss << "ready";
        dirty = true;
        break;
      case KEYCODE_g:
        ROS_DEBUG("g_key: Grasp (3 finger)");
        ss << "grasp_3";
        dirty = true;
        break;
      case KEYCODE_f:
        ROS_DEBUG("f_key: Grasp (4 finger)");
        ss << "grasp_4";
        dirty = true;
        break;
      case KEYCODE_p:
        ROS_DEBUG("p_key: Pinch (index)");
        ss << "pinch_it";
        dirty = true;
        break;
      case KEYCODE_m:
        ROS_DEBUG("m_key: Pinch (middle)");
        ss << "pinch_mt";
        dirty = true;
        break;
      case KEYCODE_e:
        ROS_DEBUG("e_key: Envelop");
        ss << "envelop";
        dirty = true;
        break;
      case KEYCODE_z:
        ROS_DEBUG("z_key: Gravcomp");
        ss << "gravcomp";
        dirty = true;
        break;
      case KEYCODE_o:
        ROS_DEBUG("o_key: Servos Off");
        ss << "off";
        dirty = true;
        break;
      case KEYCODE_s:
        ROS_DEBUG("s_key: Save joint pos. for PD control");
        ss << "save";
        dirty = true;
        break;
      case KEYCODE_slash:
      case KEYCORD_question:
        printUsage();
        break;
    }

    if(dirty ==true)
    {
      msg.data = ss.str();
      ROS_INFO("%s", msg.data.c_str());
      cmd_pub_.publish(msg);
      ros::spinOnce();
      dirty=false;
    }
  }

  return;
}
