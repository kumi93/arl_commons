#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <arl_hw_msgs/MuscleState.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "muscle_command_spamming_test_node");
  ros::NodeHandle n;
  ros::Publisher activation_pub = n.advertise<std_msgs::Float64>("chatter", 1);

  while (ros::ok()) {

    std_msgs::Float64 msg;

    msg.data = 0.5;
    activation_pub.publish(msg);
    ros::spinOnce();

    msg.data = 0.5;
    activation_pub.publish(msg);
    ros::spinOnce();

  }


  return 0;
}