/*
 des  : rscuad control gazebo
 year : 2021
 
*/

// author : danu andrean

#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include "rscuad_manager/rscuad_manager.h"



void Callback(const std_msgs::String::ConstPtr& msg){
    rscuad::rscuad_manager *rscuad =  new rscuad::rscuad_manager;
    char *newdata = (char*)msg->data.c_str();
    // ROS_INFO("newdata");
    // ROS_INFO(newdata);
    rscuad->move(newdata);
}

int main(int argc, char **argv)
{
    // alocation memory
    // rscuad::rscuad_manager *rscuad = new rscuad::rscuad_manager;

    // rscuad->manager_init();


    ros::init(argc, argv, "rscuad_manager");
    ros::NodeHandle nh;
    ROS_INFO("main manager init");
    
    ros::Subscriber sub = nh.subscribe("rscuad_manager", 1000, Callback);
    ros::spin();
}


