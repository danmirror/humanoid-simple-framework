#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rscuad_gazebo_talker");

 
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  float value = 0.5;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
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
