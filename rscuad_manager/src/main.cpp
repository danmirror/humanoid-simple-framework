#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include "rscuad_manager/rscuad_manager.h"

#include <sstream>



void Callback(const std_msgs::String::ConstPtr& msg){
    rscuad::rscuad_manager *rscuad =  new rscuad::rscuad_manager;
    rscuad->move("tilt");
}

int main(int argc, char **argv)
{
    // alocation memory
    rscuad::rscuad_manager *rscuad = new rscuad::rscuad_manager;

    rscuad->manager_init();


    ros::init(argc, argv, "rscuad_manager");
    ros::NodeHandle nh;
    ROS_INFO("main manager init");
    
    ros::Subscriber sub = nh.subscribe("rscuad_manager", 1000, Callback);
    ros::spin();
}


