/*
 des  : rscuad control gazebo
 year : 2021
 
*/

// author : danu andrean

#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include "rscuad_manager/rscuad_manager.h"
#include <stdlib.h>

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

void rscuad::rscuad_manager::move_joint(char *str){
    joint rscuad_joint;
    char *on_move;

    // ------------------data parsing------------------
    bool lock=false;
    int memory = 0;
    int count = 0;
    char joint[10]="";
    char value[10]="";

    for(int i=0; i <strlen(str);i++){
        if(str[i] == ',')
            lock =true;

        if (lock == false)
            if (count == 0)
                joint[i] = str[i]; 
            if (count == 1)
                value[i-memory] = str[i]; 

        if(lock== true){
            count ++;
            lock = false;
            memory = i+1;
        }
    }
    ROS_WARN("join:  %s", joint);
    ROS_WARN("value: %f",  atof(value));
    ROS_INFO("data masuk: %s", str);

    // -----------------servo selection--------------
    if(atoi(joint) == 1){
        ROS_WARN(">>>>>>>>>>>>>>>");
        ROS_ERROR("masukk");
        on_move = rscuad_joint.r_sho_pitch_position; 
    }
    else if(atoi(joint) == 2)
        on_move = rscuad_joint.l_sho_pitch_position;
    else if(atoi(joint) == 3)
        on_move = rscuad_joint.r_sho_roll_position; 
    else if(atoi(joint) == 4)
        on_move = rscuad_joint.l_sho_roll_position; 
    else if(atoi(joint) == 5)
        on_move = rscuad_joint.r_el_position; 
    else if(atoi(joint) == 6)
        on_move = rscuad_joint.l_el_position; 
    else if(atoi(joint) == 7)
        on_move = rscuad_joint.r_hip_yaw_position; 
    else if(atoi(joint) == 8)
        on_move = rscuad_joint.l_hip_yaw_position; 
    else if(atoi(joint) == 9)
        on_move = rscuad_joint.r_hip_roll_position; 
    else if(atoi(joint) == 10)
        on_move = rscuad_joint.l_hip_roll_position;
    else if(atoi(joint) == 11)
        on_move = rscuad_joint.r_hip_pitch_position; 
    else if(atoi(joint) == 12)
        on_move = rscuad_joint.l_hip_pitch_position; 
    else if(atoi(joint) == 13)
        on_move = rscuad_joint.r_knee_position; 
    else if(atoi(joint) == 14)
        on_move = rscuad_joint.l_knee_position; 
    else if(atoi(joint) == 15)
        on_move = rscuad_joint.r_ank_roll_position; 
    else if(atoi(joint) == 16)
        on_move = rscuad_joint.l_ank_roll_position; 
    else if(atoi(joint) == 17)
        on_move = rscuad_joint.r_ank_pitch_position; 
    else if(atoi(joint) == 18)
        on_move = rscuad_joint.l_ank_pitch_position; 
    else if(atoi(joint) == 19)
        on_move = rscuad_joint.head_pan_position;
    else if(atoi(joint) == 20)
        on_move = rscuad_joint.head_tilt_position;

    ros::Rate loop_rate(10);
    ros::NodeHandle nh; 

    ros::Publisher pub = nh.advertise<std_msgs::Float64>(on_move, 1000);

    // ROS_INFO("%s",rscuad_joint.l_hip_yaw_position);

    std_msgs::Float64 move;
    move.data = atof(value);
    while(ros::ok)
    {
        pub.publish(move);
        ros::spin();
        loop_rate.sleep();
    }

}

