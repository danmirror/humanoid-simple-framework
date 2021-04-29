/*
 des  : rscuad control gazebo
 year : 2021
 
*/

// author : danu andrean

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ros::Rate loop_rate(10);
  ros::NodeHandle nh; 
  ros::Publisher pub = nh.advertise<std_msgs::String>("rscuad_manager", 1000);

  while(ros::ok)
  {
    ROS_INFO("I heard: [%s]", msg->data.c_str()); 
    pub.publish(msg);
    loop_rate.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rscuad_gazebo_listener");
  ros::NodeHandle nh;
  ROS_INFO("init listener");

  ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}
