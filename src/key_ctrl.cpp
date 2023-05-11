#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <termios.h>

int getch(void) {
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "key_control_node");
  ros::NodeHandle nh;
  ros::Publisher mode_pub = nh.advertise<std_msgs::Int16>("/key_ctrl_mode", 10);
  ros::Rate rate(5.0);

  char key = (' ');

  while (nh.ok()) {

    key = getch();
    std_msgs::Int16 mode;
    switch (key) {
      case 'C':
        return 0;//kill node?
        break;
      case '1':
        mode.data = 1;//homing;
        mode_pub.publish(mode);
        break;
      case '2':
        mode.data = 2;//circling;
        mode_pub.publish(mode);
        break;
      case '3':
        mode.data = 3;//homing;
        mode_pub.publish(mode);
        break;
      case '4':
        mode.data = 4;//circling;
        break;
      case '5':
        mode.data = 5;//homing;
        break;
      case '6':
        mode.data = 6;//circling;
        break;
      case '7':
        mode.data = 7;//circling;
        break;
      case '8':
        mode.data = 8;//circling;
        break;
      case '9':
        mode.data = 9;//circling;
        break;
      case '0':
        mode.data = 0;//circling;
        break;
    }

    std_msgs::String printOut;
    std::stringstream ss;
    ss << "Mode: " << mode.data;
    printOut.data = ss.str();
    ROS_INFO("%s", printOut.data.c_str());

    rate.sleep();
    mode.data = 0;
    mode_pub.publish(mode);
  }
}