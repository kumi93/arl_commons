#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

ros::ServiceClient sc;
void emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg) {
  std_srvs::SetBool req;
  req.request.data = msg->data;
  if (sc.call(req))
  {
    ROS_INFO_STREAM("Emergency Stop changed state to: " << msg->data);
  }
  else
  {
    ROS_ERROR("Failed to call service /emergency_stop");
  }
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "rosjava_wrapper_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/rosjava_wrapper/emergency_stop", 1000, emergencyStopCallback);
  sc =  nh.serviceClient<std_srvs::SetBool>("/emergency_stop");

  ros::spin();
}