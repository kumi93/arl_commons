#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <arl_hw_msgs/MuscleState.h>
#include <boost/algorithm/string/predicate.hpp>

#include <vector>
#include <sstream>

std::vector<ros::Publisher> activation_pubs_;

bool getROSTopics(ros::master::V_TopicInfo &topics) {
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = ros::this_node::getName();

  if (!ros::master::execute("getTopicTypes", args, result, payload, true)) {
    std::cout << "Failed!" << std::endl;
    return false;
  }

  topics.clear();
  for (int i = 0; i < payload.size(); ++i) {
    std::string name = std::string(payload[i][0]);
    std::string type = std::string(payload[i][1]);
    if (type == "std_msgs/Float64" && boost::starts_with(name, "/muscle_") &&
        boost::ends_with(name, "/activation_command")) {
      topics.push_back(ros::master::TopicInfo(std::string(payload[i][0]), std::string(payload[i][1])));
    }
  }
  return true;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "fast_valve_setting_node");
  ros::NodeHandle n;

  ros::master::V_TopicInfo topics;
  getROSTopics(topics);

  for (ros::master::TopicInfo topic : topics) {
    activation_pubs_.push_back(n.advertise<std_msgs::Float64>(topic.name.c_str(), 1));
  }

  ros::Rate activation_rate(5);
  std_msgs::Float64 msg;

  while (ros::ok()) {
    for (ros::Publisher pub : activation_pubs_) {
      std::cout << "Setting " << pub.getTopic().c_str() << std::endl;
      msg.data = 0.1;
      pub.publish(msg);
      activation_rate.sleep();

      msg.data = -0.3;
      pub.publish(msg);

      if (!ros::ok()) {
        std_msgs::Float64 blow_off_msg;
        blow_off_msg.data = -0.5;
        for (ros::Publisher pub : activation_pubs_) {
          std::cout << "Blowing-Off " << pub.getTopic().c_str() << std::endl;
          pub.publish(blow_off_msg);
        }
        for (ros::Publisher pub : activation_pubs_) {
          std::cout << "Blowing-Off " << pub.getTopic().c_str() << std::endl;
          pub.publish(blow_off_msg);
        }


        break;
      }
      ros::spinOnce();
    }
  }


  return 0;
}
