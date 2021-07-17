/*
 des  : rscuad control gazebo
 year :2021
 
*/

// author : danu andrean

#ifndef RSCUAD_MANAGER_H_
#define RSCUAD_MANAGER_H_

#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <sstream>
/* ROBOTIS Controller Header */
#include "robotis_controller/robotis_controller.h"
#include "rscuad_manager/dxl_header.h"

/* Sensor Module Header */
#include "open_cr_module/open_cr_module.h"
using namespace robotis_framework;
using namespace robotis_op;

const int SUB_CONTROLLER_ID = 200;
const int POWER_CTRL_TABLE = 24;
const int RGB_LED_CTRL_TABLE = 26;
const int TORQUE_ON_CTRL_TABLE = 64;


namespace rscuad{

    // declear data joint movement
    typedef struct {
        char *l_hip_yaw_position= "/rscuad/l_hip_yaw_position/command";
        char *l_hip_roll_position= "/rscuad/l_hip_roll_position/command";
        char *l_hip_pitch_position= "/rscuad/l_hip_pitch_position/command";
        char *l_knee_position= "/rscuad/l_knee_position/command";
        char *l_ank_roll_position= "/rscuad/l_ank_roll_position/command";
        char *l_ank_pitch_position= "/rscuad/l_ank_pitch_position/command";
        char *r_hip_yaw_position= "/rscuad/r_hip_yaw_position/command";
        char *r_hip_roll_position= "/rscuad/r_hip_roll_position/command";
        char *r_hip_pitch_position= "/rscuad/r_hip_pitch_position/command";
        char *r_knee_position= "/rscuad/r_knee_position/command";
        char *r_ank_roll_position= "/rscuad/r_ank_roll_position/command";
        char *r_ank_pitch_position= "/rscuad/r_ank_pitch_position/command";
        char *l_sho_pitch_position= "/rscuad/l_sho_pitch_position/command";
        char *l_sho_roll_position= "/rscuad/l_sho_roll_position/command";
        char *l_el_position= "/rscuad/l_el_position/command";
        char *r_sho_pitch_position= "/rscuad/r_sho_pitch_position/command";
        char *r_sho_roll_position= "/rscuad/r_sho_roll_position/command";
        char *r_el_position= "/rscuad/r_el_position/command";
        char *head_pan_position= "/rscuad/head_pan_position/command";
        char *head_tilt_position= "/rscuad/head_tilt_position/command";

        float approach_angle = 0;
        float target_angle_1 = 0;
        float target_angle_2 = 0;
        float target_angle_3 = 0;
        float target_angle_4 = 0;
        float target_angle_5 = 0;
        float target_angle_6 = 0;
        float target_angle_7 = 0;
        float target_angle_8 = 0;
        float target_angle_9 = 0;
        float target_angle_10 = 0;
        float target_angle_11 = 0;
        float target_angle_12 = 0;
        float target_angle_13 = 0;
        float target_angle_14 = 0;
        float target_angle_15 = 0;
        float target_angle_16 = 0;
        float target_angle_17 = 0;
        float target_angle_18 = 0;
        float target_angle_19 = 0;
        float target_angle_20 = 0;

       

    // char *l_hip_yaw_position;
    // char *l_hip_roll_position;
    // char *l_hip_pitch_position;
    // char *l_knee_position;
    // char *l_ank_roll_position;
    // char *l_ank_pitch_position;
    // char *r_hip_yaw_position;
    // char *r_hip_roll_position;
    // char *r_hip_pitch_position;
    // char *r_knee_position;
    // char *r_ank_roll_position;
    // char *r_ank_pitch_position;
    // char *l_sho_pitch_position;
    // char *l_sho_roll_position;
    // char *l_el_position; 
    // char *r_sho_pitch_position;
    // char *r_sho_roll_position;
    // char *r_el_position;
    // char *head_pan_position;
    // char *head_tilt_position;
    } joint;

    class rscuad_manager{
        public:
            int manager_init(); 
            void move_joint(char *str); 
            int move_robot(char *str); 
            int dxl_process();
           

            
        protected:
            #define RADIAN2DEGREE     (180.0 / M_PI)
            int *trigers;
 

           
    };
}
#endif