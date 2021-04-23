#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ros::NodeHandle nh; 
  ros::Publisher pub = nh.advertise<std_msgs::Float64>("/rscuad/l_sho_pitch_position/command", 1000);

  std_msgs::Float64 move;
  move.data = 2.0;
  while(ros::ok)
  {
    pub.publish(move);
  }
  ROS_INFO("I heard: [%s]", msg->data.c_str()); 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rscuad_gazebo_listener");
  
  ros::NodeHandle nh;
  ROS_INFO("init");

  
  ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

 
  ros::spin();

  return 0;
}
