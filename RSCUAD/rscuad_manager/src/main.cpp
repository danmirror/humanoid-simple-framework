/*
 desc : rscuad manager 
 year : 2021
 
*/

// author : danu andrean


#include "rscuad_manager/rscuad_manager.h"


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

    //initial robot
    rscuad->manager_init();
   

  // //  move servo
    rscuad->dxl_process();

    ros::init(argc, argv, "rscuad_manager");
    ros::NodeHandle nh;
    
    ros::Subscriber joint = nh.subscribe("rscuad_manager", 100, Callback);
    ros::Subscriber robot = nh.subscribe("rscuad_manager/robot", 100, Manager_Robot_Callback);
    ros::spin();
}


