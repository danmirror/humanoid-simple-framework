/*
 des  : rscuad control gazebo
 year : 2021
 
*/

// author : danu andrean

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rscuad_gazebo_talker");

 
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  float value = 0.5;
  while (ros::ok())
  {

    std_msgs::String msg;
    std::stringstream joint, value_str;
    value_str << value;
    joint << "tilt";
    msg.data = joint.str()+","+value_str.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
    // break;
  }


  return 0;
}

// #include <ros/ros.h>
// #include <std_msgs/Float64.h>

// int main(int argc, char ** argv)
// {
//      //Initialize and start the node
//      ros::init(argc, argv, "rscuad_gazebo_listener");
//      ros::NodeHandle nh;
 
     
//      ros::Publisher pub = nh.advertise<std_msgs::Float64>("/rscuad/l_sho_pitch_position/command", 1000);
//      //Define and create some messages
//      std_msgs::Float64 mybot_control;
//      mybot_control.data = 2.0;
     
//      ros::Rate(200);
     
//      while(ros::ok)
//      {
//            pub.publish(mybot_control);
//         //    ros::spinOnce();               
//      }

//  }
