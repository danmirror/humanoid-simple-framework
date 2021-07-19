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

    } joint;

    class rscuad_manager{
        public:
            int dxl_process();
            int manager_init(); 
            float offset_ID(int id);
            int move_robot(char *str); 
            void move_joint(char *str,int id,float velocity); 

            
        protected:
            #define RADIAN2DEGREE     (180.0 / M_PI)
            int *trigers;

            float approach_angle = 0;


            float offset_1 = 2046;
            float offset_2 = 2046;
            float offset_3 = 2046;
            float offset_4 = 2046;
            float offset_5 = 2046;
            float offset_6 = 2046;
            float offset_7 = -2046;
            float offset_8 = 2046;
            float offset_9 = -2046;
            float offset_10 = -2046;
            float offset_11 = -2046;
            float offset_12 = 2046;
            float offset_13 = 2046;
            float offset_14 = 2046;
            float offset_15 = 2046;
            float offset_16 = -2046;
            float offset_17 = 2046;
            float offset_18 = 2046;
            float offset_19 = 2046;
            float offset_20 = 2046;

            int *ptr_ID[20] {
                &DXL_ID_1,&DXL_ID_2,&DXL_ID_3,&DXL_ID_4,&DXL_ID_5,&DXL_ID_6,&DXL_ID_7,&DXL_ID_8,&DXL_ID_9,&DXL_ID_10,
                &DXL_ID_11,&DXL_ID_12,&DXL_ID_13,&DXL_ID_14,&DXL_ID_15,&DXL_ID_16,&DXL_ID_17,&DXL_ID_18,&DXL_ID_19,&DXL_ID_20};
            
 
    };
}
#endif