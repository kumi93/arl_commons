#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <arl_hw_msgs/MuscleState.h>

#include <vector>
#include <sstream>

#define CONTROLLER_NUM 32

std::vector<ros::Publisher> activation_pubs_;
std::vector<ros::Publisher> pressure_pubs_;

int main(int argc, char **argv) {
  ros::init(argc, argv, "muscle_command_spamming_test_node");
  ros::NodeHandle n;

  activation_pubs_.reserve(CONTROLLER_NUM);
  for (int i = 0; i < CONTROLLER_NUM; i++) {
    std::stringstream name;
    name << "/m" << i << "_controller/activation_command";
    activation_pubs_.push_back(n.advertise<std_msgs::Float64>(name.str().c_str(), 1));
  }

  pressure_pubs_.reserve(CONTROLLER_NUM);
  for (int i = 0; i < CONTROLLER_NUM; i++) {
    std::stringstream name;
    name << "/m" << i << "_controller/pressure_command";
    pressure_pubs_.push_back(n.advertise<std_msgs::Float64>(name.str().c_str(), 1));
  }


  ros::Rate loop_rate(70);
  while (ros::ok()) {

    std_msgs::Float64 msg;

    msg.data = -1;
    for(int i = 0; i < CONTROLLER_NUM; i++){
      activation_pubs_[i].publish(msg);
    }
    ros::spinOnce();
    loop_rate.sleep();

    msg.data = -1;
    for(int i = 0; i < CONTROLLER_NUM; i++){
      pressure_pubs_[i].publish(msg);
    }
    ros::spinOnce();
    loop_rate.sleep();

    msg.data = 1;
    for(int i = 0; i < CONTROLLER_NUM; i++){
      activation_pubs_[i].publish(msg);
    }
    ros::spinOnce();
    loop_rate.sleep();

    msg.data = 1;
    for(int i = 0; i < CONTROLLER_NUM; i++){
      pressure_pubs_[i].publish(msg);
    }
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}