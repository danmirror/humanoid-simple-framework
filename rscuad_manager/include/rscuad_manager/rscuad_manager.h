#ifndef RSCUAD_MANAGER_H_
#define RSCUAD_MANAGER_H_

#include <ros/ros.h>
#include <stdio.h>
#include <string.h>


namespace rscuad{
    
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
            void manager_init(); 
            void move(char *str); 
    };
}
#endif