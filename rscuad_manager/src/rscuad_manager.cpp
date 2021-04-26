#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include "rscuad_manager/rscuad_manager.h"

#include <sstream>

void rscuad::rscuad_manager::manager_init()
{
    ROS_INFO("manager init");
    joint rscuad_joint;

    // rscuad_joint.l_hip_yaw_position= "/rscuad/l_hip_yaw_position/command";
    // rscuad_joint.l_hip_roll_position= "/rscuad/l_hip_roll_position/command";
    // rscuad_joint.l_hip_pitch_position= "/rscuad/l_hip_pitch_position/command";
    // rscuad_joint.l_knee_position= "/rscuad/l_knee_position/command";
    // rscuad_joint.l_ank_roll_position= "/rscuad/l_ank_roll_position/command";
    // rscuad_joint.l_ank_pitch_position= "/rscuad/l_ank_pitch_position/command";
    // rscuad_joint.r_hip_yaw_position= "/rscuad/r_hip_yaw_position/command";
    // rscuad_joint.r_hip_roll_position= "/rscuad/r_hip_roll_position/command";
    // rscuad_joint.r_hip_pitch_position= "/rscuad/r_hip_pitch_position/command";
    // rscuad_joint.r_knee_position= "/rscuad/r_knee_position/command";
    // rscuad_joint.r_ank_roll_position= "/rscuad/r_ank_roll_position/command";
    // rscuad_joint.r_ank_pitch_position= "/rscuad/r_ank_pitch_position/command";
    // rscuad_joint.l_sho_pitch_position= "/rscuad/l_sho_pitch_position/command";
    // rscuad_joint.l_sho_roll_position= "/rscuad/l_sho_roll_position/command";
    // rscuad_joint.l_el_position= "/rscuad/l_el_position/command";
    // rscuad_joint.r_sho_pitch_position= "/rscuad/r_sho_pitch_position/command";
    // rscuad_joint.r_sho_roll_position= "/rscuad/r_sho_roll_position/command";
    // rscuad_joint.r_el_position= "/rscuad/r_el_position/command";
    // rscuad_joint.head_pan_position= "/rscuad/head_pan_position/command";
    // rscuad_joint.head_tilt_position= "/rscuad/head_tilt_position/command";

}

void rscuad::rscuad_manager::move(char *str){
    joint rscuad_joint;

    ROS_INFO("data masuk: ,%s", str);
    ros::Rate loop_rate(10);
    ros::NodeHandle nh; 

    ros::Publisher pub = nh.advertise<std_msgs::Float64>(rscuad_joint.l_hip_yaw_position, 1000);

    // ROS_INFO("%s",rscuad_joint.l_hip_yaw_position);

    std_msgs::Float64 move;
    move.data = 2.0;
    while(ros::ok)
    {
        pub.publish(move);
        ros::spin();
        loop_rate.sleep();
    }

}

