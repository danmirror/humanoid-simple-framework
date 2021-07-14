/*
 des  : rscuad manager 
 year : 2021
 
*/

// author : danu andrean




#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include "rscuad_manager/rscuad_manager.h"

/* ROBOTIS Controller Header */
#include "robotis_controller/robotis_controller.h"

/* Sensor Module Header */
#include "open_cr_module/open_cr_module.h"
using namespace robotis_framework;
using namespace robotis_op;

const int SUB_CONTROLLER_ID = 200;
const int POWER_CTRL_TABLE = 24;
const int RGB_LED_CTRL_TABLE = 26;
const int TORQUE_ON_CTRL_TABLE = 64;



void Callback(const std_msgs::String::ConstPtr& msg){
    rscuad::rscuad_manager *rscuad =  new rscuad::rscuad_manager;
    char *newdata = (char*)msg->data.c_str();
    // ROS_INFO("newdata");
    // ROS_INFO(newdata);
    rscuad->move_joint(newdata);
}

void Manager_Robot_Callback(const std_msgs::String::ConstPtr& msg){
    rscuad::rscuad_manager *rscuad =  new rscuad::rscuad_manager;
    char *newdata = (char*)msg->data.c_str();
    // ROS_INFO(" new data %.2f" ,msg->data);
    ROS_INFO(newdata);
    rscuad->move_robot(newdata);
}



int main(int argc, char **argv)
{
    // alocation memory
    rscuad::rscuad_manager *rscuad = new rscuad::rscuad_manager;

    //initial power
    rscuad->manager_init();

   
   

  // //  move servo
    rscuad->dxl_process();

    ros::init(argc, argv, "rscuad_manager");
    ros::NodeHandle nh;
    ROS_INFO("main manager init");
    
    ros::Subscriber joint = nh.subscribe("rscuad_manager", 100, Callback);
    ros::Subscriber robot = nh.subscribe("rscuad_manager/robot", 100, Manager_Robot_Callback);
    ros::spin();
}


