/*
 * desc : rscuad walking
 * year : 2021
 * dev  : danu andrean
 *
 */



#include <walking/walking.h>
#include <iostream>

#include <stdlib.h>     /* srand, rand */
#include <std_msgs/Float64.h>
#include <math.h>

#include <walking/math/Matrix.h>
#define MX28_1024

/*  new paramater temp
*   if you put in header file, walking not working , i have no idea!!
*/
int iter = 0;
double LIMIT_Z = 0;
bool INIT_MODE;
int periode_counter = 0;
float Z_OFFSET_MEM = 60;
float counter_z=0;
    

namespace robotis_op {
using namespace Robot;



Walking::Walking(ros::NodeHandle nh)
    : nh_(nh)
{
    ROS_WARN("Constructor");

    init_pos = false;
    LIMIT = 1;
    INIT_MODE = false;

    m_Ctrl_Running = false;
    m_Real_Running = false;
    m_Time = 0.0;
    rlGyroErr = 0.0;
    fbGyroErr = 0.0;

    X_OFFSET = -10;
    Y_OFFSET = -10;
    Z_OFFSET =5;
    R_OFFSET = 20;
    P_OFFSET = 0;
    A_OFFSET = 0;
    HIP_PITCH_OFFSET = 20.0;
    PERIOD_TIME =600; //600
    DSP_RATIO = 0.1;
    STEP_FB_RATIO = 0.28;
    Z_MOVE_AMPLITUDE = 60; //40
    Y_SWAP_AMPLITUDE = 20.0;
    Z_SWAP_AMPLITUDE = 5;
    PELVIS_OFFSET = 3.0;
    ARM_SWING_GAIN = 1.5;
    BALANCE_KNEE_GAIN = 0.3;
    BALANCE_ANKLE_PITCH_GAIN = 0.9;
    BALANCE_HIP_ROLL_GAIN = 0.5;
    BALANCE_ANKLE_ROLL_GAIN = 1.0;

    P_GAIN = 32.0;
    I_GAIN = 0;
    D_GAIN = 0;

    X_MOVE_AMPLITUDE = 0;
    Y_MOVE_AMPLITUDE = 0;
    A_MOVE_AMPLITUDE = 0;
    A_MOVE_AIM_ON = true;
    BALANCE_ENABLE = true;
    
    /*
     *  JUST CHANGE IN MISSION, DON'T CHANGE IN BELOW
     *   if you want make faster initial used this, but not perfect
     *
     */

    WALK_READY_MODE= false;    

}

Walking::~Walking()
{
}

int Walking::periode_calc()
{
    return periode_counter;
}
int Walking::init_status(){
    return INIT_MODE;
}

void Walking::walk_ready()
{
    WALK_READY_MODE= true;

    /*--------------------------------
     *
     *  COMMENT IF YOU WANT NOT USING INITIAL
     *  just => INIT_MODE = true;
     * 
     *--------------------------------*/
    // INIT_MODE = true;
    ROS_INFO("Z_OFFSET >> %f",Z_OFFSET);

    if(counter_z >= Z_OFFSET_MEM)
    {
        counter_z = Z_OFFSET_MEM;
    }
    else
    {
        counter_z +=0.1;
    }

    Z_OFFSET = counter_z;

    if(Z_OFFSET >= Z_OFFSET_MEM)
    {
            INIT_MODE = true;
        
    }
     /*--------------------------------*/
}
void Walking::Initialize()
{
    init_pos=true;
}
void Walking::InitializeMode()
{

    /*
    *   all of this function, make initial smooth before running 
    *
    *   limit just counter to make smooth
    *
    */

    // 30 is prepare before (running)

    if(iter >=30){
        if(LIMIT_Z <Z_OFFSET/2){
            LIMIT_Z += 0.02;

            //  increment begining
            m_Z_Move_Amplitude = -Z_OFFSET/2 + LIMIT_Z;
            m_Z_Move_Amplitude_Shift = -Z_OFFSET/2 + LIMIT_Z;
            m_Z_Swap_Amplitude = -Z_OFFSET/2 + LIMIT_Z;
            m_Z_Swap_Amplitude_Shift = -Z_OFFSET/2 + LIMIT_Z;
            // HIP_PITCH_OFFSET =-5 +LIMIT_Z ;

        }
        else{
            // start initial
            
            if(LIMIT>0){
                LIMIT -= 0.01;
                ROS_ERROR("one last----------------------------");
            }
            else{
                INIT_MODE = true;
                ROS_ERROR("done init----------------------------");
            }

            // Z Position
            m_Z_Move_Amplitude = Z_MOVE_AMPLITUDE / 2 - ((Z_MOVE_AMPLITUDE / 2) *LIMIT) ;
            m_Z_Move_Amplitude_Shift = m_Z_Move_Amplitude / 2-(( m_Z_Move_Amplitude / 2)*LIMIT);
            m_Z_Swap_Amplitude = Z_SWAP_AMPLITUDE - (Z_SWAP_AMPLITUDE*LIMIT);
            m_Z_Swap_Amplitude_Shift = m_Z_Swap_Amplitude - (m_Z_Swap_Amplitude*LIMIT);

             // Right/Left
            m_Y_Move_Amplitude = Y_MOVE_AMPLITUDE / 2;
            if(m_Y_Move_Amplitude > 0)
                m_Y_Move_Amplitude_Shift = m_Y_Move_Amplitude - (m_Y_Move_Amplitude*LIMIT) ;
            else
                m_Y_Move_Amplitude_Shift = -m_Y_Move_Amplitude + (m_Y_Move_Amplitude *LIMIT);
            m_Y_Swap_Amplitude = Y_SWAP_AMPLITUDE + m_Y_Move_Amplitude_Shift * 0.04 - ((Y_SWAP_AMPLITUDE + m_Y_Move_Amplitude_Shift * 0.04) *LIMIT);

        }
    }
    else{
        // ROS_ERROR("engle not found");
        /* prepare before until robot stable
        */
        m_Z_Move_Amplitude = -Z_OFFSET/2 + LIMIT_Z;
        m_Z_Move_Amplitude_Shift = -Z_OFFSET/2 + LIMIT_Z;
        m_Z_Swap_Amplitude = -Z_OFFSET/2 + LIMIT_Z;
        m_Z_Swap_Amplitude_Shift = -Z_OFFSET/2 + LIMIT_Z;
    }

    ROS_INFO("limit z %f",LIMIT_Z);

    // Forward/Back
    m_X_Move_Amplitude = X_MOVE_AMPLITUDE - (X_MOVE_AMPLITUDE*LIMIT);
    m_X_Swap_Amplitude = X_MOVE_AMPLITUDE * STEP_FB_RATIO - ((X_MOVE_AMPLITUDE * STEP_FB_RATIO) * LIMIT);

    /* check only */
    /*----------------------------------*/
    // m_X_Move_Amplitude = 0;
    // m_X_Swap_Amplitude = 0;

    // m_Z_Move_Amplitude = -8.4;
    // m_Z_Move_Amplitude_Shift = -8.4;
    // m_Z_Swap_Amplitude = -8.4;
    // m_Z_Swap_Amplitude_Shift = -8.5;
    // HIP_PITCH_OFFSET =-8.4;
    /*----------------------------------*/

    // ROS_WARN("m_Z_Move_Amplitude %f",m_Z_Move_Amplitude);
    // ROS_WARN("m_Z_Move_Amplitude ORI %f",Z_MOVE_AMPLITUDE / 2);
    // ROS_WARN("m_Z_Move_Amplitude_Shift %f",m_Z_Move_Amplitude_Shift);
    ROS_WARN("m_X_Move_Amplitude %f",m_X_Move_Amplitude);
    ROS_WARN("X_MOVE_AMPLITUDE %f",X_MOVE_AMPLITUDE);
    ROS_WARN("Z_MOVE_AMPLITUDE %f",Z_MOVE_AMPLITUDE);
    ROS_WARN("Z_SWAP_AMPLITUDE %f",Z_SWAP_AMPLITUDE);


}

void Walking::update(ros::Time time, ros::Duration period)
{
}

double Walking::wsin(double time, double period, double period_shift, double mag, double mag_shift)
{
     if((init_pos == false && WALK_READY_MODE == false)|| INIT_MODE == true ){
     
        return mag * -cos(2 * 3.141592 / period * time - period_shift) + mag_shift;
     }
     else {
        return mag * sin(2 * 3.141592 / period * time - period_shift) + mag_shift;
     }
}

bool Walking::computeIK(double *out, double x, double y, double z, double a, double b, double c)
{
    // ROS_WARN("IK activate");
    Matrix3D Tad, Tda, Tcd, Tdc, Tac;
    Vector3D vec;
    double _Rac, _Acos, _Atan, _k, _l, _m, _n, _s, _c, _theta;
    double LEG_LENGTH = 292.0;
    double THIGH_LENGTH = 125.0;
    double CALF_LENGTH = 122.0;
    double ANKLE_LENGTH = 42.0;

    // double LEG_LENGTH = 219.5;
    // double THIGH_LENGTH = 93.0;
    // double CALF_LENGTH = 93.0;
    // double ANKLE_LENGTH = 33.5;


    Tad.SetTransform(Point3D(x, y, z - LEG_LENGTH), Vector3D(a * 180.0 / M_PI, b * 180.0 / M_PI, c * 180.0 / M_PI));

    vec.X = x + Tad.m[2] * ANKLE_LENGTH;
    vec.Y = y + Tad.m[6] * ANKLE_LENGTH;
    vec.Z = (z - LEG_LENGTH) + Tad.m[10] * ANKLE_LENGTH;

    // Get Knee
    _Rac = vec.Length();
    _Acos = acos((_Rac * _Rac - THIGH_LENGTH * THIGH_LENGTH - CALF_LENGTH * CALF_LENGTH) / (2 * THIGH_LENGTH * CALF_LENGTH));
    if(isnan(_Acos) == 1)
        return false;
    *(out + 3) = _Acos;

    // Get Ankle Roll
    Tda = Tad;
    if(Tda.Inverse() == false)
        return false;
    _k = sqrt(Tda.m[7] * Tda.m[7] + Tda.m[11] * Tda.m[11]);
    _l = sqrt(Tda.m[7] * Tda.m[7] + (Tda.m[11] - ANKLE_LENGTH) * (Tda.m[11] - ANKLE_LENGTH));
    _m = (_k * _k - _l * _l - ANKLE_LENGTH * ANKLE_LENGTH) / (2 * _l * ANKLE_LENGTH);
    if(_m > 1.0)
        _m = 1.0;
    else if(_m < -1.0)
        _m = -1.0;
    _Acos = acos(_m);
    if(isnan(_Acos) == 1)
        return false;
    if(Tda.m[7] < 0.0)
        *(out + 5) = -_Acos;
    else
        *(out + 5) = _Acos;

    // Get Hip Yaw
    Tcd.SetTransform(Point3D(0, 0, -ANKLE_LENGTH), Vector3D(*(out + 5) * 180.0 / M_PI, 0, 0));
    Tdc = Tcd;
    if(Tdc.Inverse() == false)
        return false;
    Tac = Tad * Tdc;
    _Atan = atan2(-Tac.m[1] , Tac.m[5]);
    if(isinf(_Atan) == 1)
        return false;
    *(out) = _Atan;

    // Get Hip Roll
    _Atan = atan2(Tac.m[9], -Tac.m[1] * sin(*(out)) + Tac.m[5] * cos(*(out)));
    if(isinf(_Atan) == 1)
        return false;
    *(out + 1) = _Atan;

    // Get Hip Pitch and Ankle Pitch
    _Atan = atan2(Tac.m[2] * cos(*(out)) + Tac.m[6] * sin(*(out)), Tac.m[0] * cos(*(out)) + Tac.m[4] * sin(*(out)));
    if(isinf(_Atan) == 1)
        return false;
    _theta = _Atan;
    _k = sin(*(out + 3)) * CALF_LENGTH;
    _l = -THIGH_LENGTH - cos(*(out + 3)) * CALF_LENGTH;
    _m = cos(*(out)) * vec.X + sin(*(out)) * vec.Y;
    _n = cos(*(out + 1)) * vec.Z + sin(*(out)) * sin(*(out + 1)) * vec.X - cos(*(out)) * sin(*(out + 1)) * vec.Y;
    _s = (_k * _n + _l * _m) / (_k * _k + _l * _l);
    _c = (_n - _k * _s) / _l;
    _Atan = atan2(_s, _c);
    if(isinf(_Atan) == 1)
        return false;
    *(out + 2) = _Atan;
    *(out + 4) = _theta - *(out + 3) - *(out + 2);

    return true;
}

void Walking::update_param_time()
{
    ROS_ERROR("update time???????????????");
    m_PeriodTime = PERIOD_TIME;
    m_DSP_Ratio = DSP_RATIO;
    m_SSP_Ratio = 1 - DSP_RATIO;

    m_X_Swap_PeriodTime = m_PeriodTime / 2;
    m_X_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;
    m_Y_Swap_PeriodTime = m_PeriodTime;
    m_Y_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;
    m_Z_Swap_PeriodTime = m_PeriodTime / 2;
    m_Z_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio / 2;
    m_A_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;

    m_SSP_Time = m_PeriodTime * m_SSP_Ratio;
    m_SSP_Time_Start_L = (1 - m_SSP_Ratio) * m_PeriodTime / 4;
    m_SSP_Time_End_L = (1 + m_SSP_Ratio) * m_PeriodTime / 4;
    m_SSP_Time_Start_R = (3 - m_SSP_Ratio) * m_PeriodTime / 4;
    m_SSP_Time_End_R = (3 + m_SSP_Ratio) * m_PeriodTime / 4;

    m_Phase_Time1 = (m_SSP_Time_End_L + m_SSP_Time_Start_L) / 2;
    m_Phase_Time2 = (m_SSP_Time_Start_R + m_SSP_Time_End_L) / 2;
    m_Phase_Time3 = (m_SSP_Time_End_R + m_SSP_Time_Start_R) / 2;

    m_Pelvis_Offset = PELVIS_OFFSET*3.413;
    m_Pelvis_Swing = m_Pelvis_Offset * 0.35;
    m_Arm_Swing_Gain = ARM_SWING_GAIN;
}

void Walking::update_param_move()
{
    // X_MOVE_AMPLITUDE = 10; //manuals
    // HIP_PITCH_OFFSET = 0;

    // Forward/Back
    m_X_Move_Amplitude = X_MOVE_AMPLITUDE;
    m_X_Swap_Amplitude = X_MOVE_AMPLITUDE * STEP_FB_RATIO;

    // Right/Left
    m_Y_Move_Amplitude = Y_MOVE_AMPLITUDE / 2;
    if(m_Y_Move_Amplitude > 0)
        m_Y_Move_Amplitude_Shift = m_Y_Move_Amplitude;
    else
        m_Y_Move_Amplitude_Shift = -m_Y_Move_Amplitude;
    m_Y_Swap_Amplitude = Y_SWAP_AMPLITUDE + m_Y_Move_Amplitude_Shift * 0.04;

    m_Z_Move_Amplitude = Z_MOVE_AMPLITUDE / 2;
    m_Z_Move_Amplitude_Shift = m_Z_Move_Amplitude / 2;
    m_Z_Swap_Amplitude = Z_SWAP_AMPLITUDE;
    m_Z_Swap_Amplitude_Shift = m_Z_Swap_Amplitude;

    // Direction
    if(A_MOVE_AIM_ON == false)
    {
        m_A_Move_Amplitude = A_MOVE_AMPLITUDE * M_PI / 180.0 / 2;
        if(m_A_Move_Amplitude > 0)
            m_A_Move_Amplitude_Shift = m_A_Move_Amplitude;
        else
            m_A_Move_Amplitude_Shift = -m_A_Move_Amplitude;
    }
    else
    {
        m_A_Move_Amplitude = -A_MOVE_AMPLITUDE * M_PI / 180.0 / 2;
        if(m_A_Move_Amplitude > 0)
            m_A_Move_Amplitude_Shift = -m_A_Move_Amplitude;
        else
            m_A_Move_Amplitude_Shift = m_A_Move_Amplitude;
    }
}

void Walking::update_param_balance()
{
    m_X_Offset = X_OFFSET;
    m_Y_Offset = Y_OFFSET;
    m_Z_Offset = Z_OFFSET;
    m_R_Offset = R_OFFSET * M_PI / 180.0;
    m_P_Offset = P_OFFSET * M_PI / 180.0;
    m_A_Offset = A_OFFSET * M_PI / 180.0;
    m_Hip_Pitch_Offset = HIP_PITCH_OFFSET*3.413;
}




void Walking::Start()
{
    if(INIT_MODE == true || (WALK_READY_MODE == false && init_pos == false)){
        ROS_INFO("walking starting by main");
        m_Ctrl_Running = true;
        m_Real_Running = true;
    }
}

void Walking::Stop()
{
    m_Ctrl_Running = false;
}

bool Walking::IsRunning()
{
    return m_Real_Running;
}

void Walking::Process(double *outValue)
{

    double x_swap, y_swap, z_swap, a_swap, b_swap, c_swap;
    double x_move_r, y_move_r, z_move_r, a_move_r, b_move_r, c_move_r;
    double x_move_l, y_move_l, z_move_l, a_move_l, b_move_l, c_move_l;
    double pelvis_offset_r, pelvis_offset_l;
    double angle[14], ep[12];
    double offset;
    double TIME_UNIT = 8; //[ms] todo check

    //                     R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL, L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL, R_ARM_SWING, L_ARM_SWING
    int dir[14]          = {   -1,        -1,          -1,         1,         -1,            1,          -1,        -1,         -1,         -1,         -1,            1,           1,           -1      };
    // int dir[14]          = {   -1,        -1,          -1,         1,         -1,            1,          -1,        -1,         -1,         -1,         -1,            1,           1,           -1      };

    double initAngle[14] = {   0.0,       0.0,        0.0,       0.0,        0.0,          0.0,         0.0,       0.0,        0.0,        0.0,       0.0,          0.0,       -48.345,       41.313    };
    // double initAngle[14] = {   4.0,       3.0,        -17.0,       7.0,        -7.0,          3.0,         -2.0,       -4.0,        15.0,        -9.0,      7.0,          -3.0,       -15,       15    };
    // Update walk parameters
    if(m_Time == 0)
    {
        // ROS_ERROR("Update walk parameters 1");
        update_param_time();
        m_Phase = PHASE0;
        if(m_Ctrl_Running == false)
        {
            if(m_X_Move_Amplitude == 0 && m_Y_Move_Amplitude == 0 && m_A_Move_Amplitude == 0)
            {
                // ROS_ERROR(" ->>>> mreal false");
                m_Real_Running = false;
            }
            else
            {
                // ROS_ERROR(" ->>>> mreal true");
                X_MOVE_AMPLITUDE = 0;
                Y_MOVE_AMPLITUDE = 0;
                A_MOVE_AMPLITUDE = 0;
            }
        }
    }
    else if(m_Time >= (m_Phase_Time1 - TIME_UNIT/2) && m_Time < (m_Phase_Time1 + TIME_UNIT/2))
    {
        // ROS_ERROR("Update walk parameters 2");
        update_param_move();
        m_Phase = PHASE1;
    }
    else if(m_Time >= (m_Phase_Time2 - TIME_UNIT/2) && m_Time < (m_Phase_Time2 + TIME_UNIT/2))
    {
        // ROS_ERROR("Update walk parameters 3");
        update_param_time();
        m_Time = m_Phase_Time2;
        m_Phase = PHASE2;
        if(m_Ctrl_Running == false)
        {
            if(m_X_Move_Amplitude == 0 && m_Y_Move_Amplitude == 0 && m_A_Move_Amplitude == 0)
            {
                m_Real_Running = false;
            }
            else
            {
                X_MOVE_AMPLITUDE = 0;
                Y_MOVE_AMPLITUDE = 0;
                A_MOVE_AMPLITUDE = 0;
            }
        }
    }
    else if(m_Time >= (m_Phase_Time3 - TIME_UNIT/2) && m_Time < (m_Phase_Time3 + TIME_UNIT/2))
    {
        // ROS_ERROR("Update walk parameters 4");
        update_param_move();
        m_Phase = PHASE3;
    }
    update_param_balance();

    // Compute endpoints
    x_swap = wsin(m_Time, m_X_Swap_PeriodTime, m_X_Swap_Phase_Shift, m_X_Swap_Amplitude, m_X_Swap_Amplitude_Shift);
    y_swap = wsin(m_Time, m_Y_Swap_PeriodTime, m_Y_Swap_Phase_Shift, m_Y_Swap_Amplitude, m_Y_Swap_Amplitude_Shift);
    z_swap = wsin(m_Time, m_Z_Swap_PeriodTime, m_Z_Swap_Phase_Shift, m_Z_Swap_Amplitude, m_Z_Swap_Amplitude_Shift);

    // ROS_ERROR("x_swap >>>>>>>>>>>>>>>> %f",x_swap);
    //   ROS_ERROR("y_swap >>>>>>>>>>>>>>>>%f",y_swap);
        // ROS_ERROR("z_swap >>>>>>>>>>>>>>>>%f",z_swap);

    a_swap = 0;
    b_swap = 0;
    c_swap = 0;

    if(m_Time <= m_SSP_Time_Start_L)
    {
        // ROS_ERROR("l start >>>>>>>>>>>>>>>>");
        x_move_l = wsin(m_SSP_Time_Start_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 *  M_PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 *  M_PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_SSP_Time_Start_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 *  M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_SSP_Time_Start_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 *  M_PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_SSP_Time_Start_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 *  M_PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 *  M_PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 *  M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_SSP_Time_Start_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 *  M_PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = 0;
        pelvis_offset_r = 0;
    }
    else if(m_Time <= m_SSP_Time_End_L)
    {
        //  ROS_ERROR("l end >>>>>>>>>>>>>>>>");
        x_move_l = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 *  M_PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 *  M_PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 *  M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 *  M_PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 *  M_PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 *  M_PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 *  M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 *  M_PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 *  M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Pelvis_Swing / 2, m_Pelvis_Swing / 2);
        pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 *  M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, -m_Pelvis_Offset / 2, -m_Pelvis_Offset / 2);
    }
    else if(m_Time <= m_SSP_Time_Start_R)
    {
        // ROS_ERROR("R start >>>>>>>>>>>>>>>>");
        x_move_l = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 *  M_PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 *  M_PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 *  M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 *  M_PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 *  M_PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 *  M_PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 *  M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 *  M_PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = 0;
        pelvis_offset_r = 0;
    }
    else if(m_Time <= m_SSP_Time_End_R)
    {
        //  ROS_ERROR("R end >>>>>>>>>>>>>>>>");
        x_move_l = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 *  M_PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R +  M_PI, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 *  M_PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R +  M_PI, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 *  M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 *  M_PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R +  M_PI, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 *  M_PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R +  M_PI, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 *  M_PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R +  M_PI, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 *  M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 *  M_PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R +  M_PI, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 *  M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Pelvis_Offset / 2, m_Pelvis_Offset / 2);
        pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 *  M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, -m_Pelvis_Swing / 2, -m_Pelvis_Swing / 2);
    }
    else
    {
        // ROS_ERROR("---------NORMAL------------------------");
        x_move_l = wsin(m_SSP_Time_End_R, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 *  M_PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R +  M_PI, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_SSP_Time_End_R, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 *  M_PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R +  M_PI, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 *  M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_SSP_Time_End_R, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 *  M_PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R +  M_PI, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_SSP_Time_End_R, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 *  M_PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R +  M_PI, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_SSP_Time_End_R, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 *  M_PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R +  M_PI, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_SSP_Time_End_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 *  M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_SSP_Time_End_R, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 *  M_PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R +  M_PI, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = 0;
        pelvis_offset_r = 0;
    }

    a_move_l = 0;
    b_move_l = 0;
    a_move_r = 0;
    b_move_r = 0;

    // std ::cout<<"x "<<x_swap<<std::endl;
    // std ::cout<<"y "<<y_swap<<std::endl;
    // std ::cout<<"z "<<z_swap<<std::endl;
    

    ep[0] = x_swap + x_move_r + m_X_Offset;
    ep[1] = y_swap + y_move_r - m_Y_Offset / 2;
    ep[2] = z_swap + z_move_r + m_Z_Offset;
    ep[3] = a_swap + a_move_r - m_R_Offset / 2;
    ep[4] = b_swap + b_move_r + m_P_Offset;
    ep[5] = c_swap + c_move_r - m_A_Offset / 2;
    ep[6] = x_swap + x_move_l + m_X_Offset;
    ep[7] = y_swap + y_move_l + m_Y_Offset / 2;
    ep[8] = z_swap + z_move_l + m_Z_Offset;
    ep[9] = a_swap + a_move_l + m_R_Offset / 2;
    ep[10] = b_swap + b_move_l + m_P_Offset;
    ep[11] = c_swap + c_move_l + m_A_Offset / 2;

    // ROS_WARN("init walking  2");
    // Compute body swing
    if(m_Time <= m_SSP_Time_End_L)
    {
        m_Body_Swing_Y = -ep[7];
        m_Body_Swing_Z = ep[8];
    }
    else
    {
        m_Body_Swing_Y = -ep[1];
        m_Body_Swing_Z = ep[2];
    }
    m_Body_Swing_Z -= 219.5;

    // Compute arm swing
    if(m_X_Move_Amplitude == 0)
    {
        angle[12] = 0; // Right
        angle[13] = 0; // Left
    }
    else
    {
        angle[12] = wsin(m_Time, m_PeriodTime,  M_PI * 1.5, -m_X_Move_Amplitude * m_Arm_Swing_Gain, 0);
        angle[13] = wsin(m_Time, m_PeriodTime,  M_PI * 1.5, m_X_Move_Amplitude * m_Arm_Swing_Gain, 0);
    }

    if(m_Real_Running == true)
    {
    //     ROS_INFO("update m_Time %d",m_Time);
    //     ROS_INFO("update periode %d",m_PeriodTime);
        
        // std::cout<<TIME_UNIT<<m_Time<<"-"<< m_PeriodTime<<std::endl;

        m_Time += TIME_UNIT;
        if(m_Time >= m_PeriodTime){
            m_Time = 0;
            periode_counter +=1;
        }
    }

     /* init position*/
    if ( m_Real_Running == false)
    {
        if(init_pos == true)
            InitializeMode();   //VERY SLOW

        if(WALK_READY_MODE ==true)
            walk_ready();       //under construction

    }
    // make iteration global
    iter ++; 

    
    // Compute angles
    if((computeIK(&angle[0], ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]) == 1)
        && (computeIK(&angle[6], ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]) == 1))
    {
        /* 30 just estimate
        */ 
        // if(iter < 30){
            for(int i=0; i<12; i++){
                // if(i <3){  //if(fmod(i , 2) == 0){
                    angle[i] *= 180.0 / M_PI;
                    // ROS_WARN("genap");
                // }
                // else
                    // ROS_WARN("tidak");
            }
            ROS_WARN("angle found");
        // }
        
        // return; // Do not use angle;
    }
    else
    {
        
        //make zero berfore running
        for(int i=0; i<12; i++){
            angle[i] =0;
        }

        ROS_ERROR("angle NOT found");
        // return; // Do not use angle;
    }

    // Compute motor value
    
    for(int i=0; i<14; i++)
    {
        offset = (double)dir[i] * angle[i] * 3.413;
        if(i == 1) // R_HIP_ROLL
            offset += (double)dir[i] * pelvis_offset_r;
        else if(i == 7) // L_HIP_ROLL
            offset += (double)dir[i] * pelvis_offset_l;
        else if(i == 2 || i == 8) // R_HIP_PITCH or L_HIP_PITCH
            offset -= (double)dir[i] * HIP_PITCH_OFFSET * 3.413;

        /* _initial approach using periode
        */ 
        // if(init_pos == true && m_Real_Running == false) {
            // if(i == 1) // R_HIP_ROLL
            //     offset = 0;
            // else if(i == 7) // L_HIP_ROLL
            //     offset = 0;

            // if(i == 5)
            //     // offset = 0;
            //     continue;
            // if(i == 11)
            //     continue;
            //     // offset = 0;
        // }

        /* _shoulder balance off
        */
        // if(i == 12)
        //     offset = 0;
        // if(i == 13)
        //     offset = 0;

        outValue[i] = (offset*0.293)/(180.0/M_PI);//initAngle[i] + (int)offset; //todo check MX28::Angle2Value(initAngle[i]) + (int)offset;
        
    }
    // ROS_ERROR("check----------------------------");
    // }
    ROS_ERROR("X_MOVE_AMPLITUDE = %f", X_MOVE_AMPLITUDE);
    // std::cout<<outValue[1]<<std::endl;


    // adjust balance offset
    if(BALANCE_ENABLE == true)
    {

        outValue[1] += (dir[1] * rlGyroErr * BALANCE_HIP_ROLL_GAIN); // R_HIP_ROLL
        outValue[7] += (dir[7] * rlGyroErr * BALANCE_HIP_ROLL_GAIN); // L_HIP_ROLL

        outValue[3] -= (dir[3] * fbGyroErr * BALANCE_KNEE_GAIN); // R_KNEE
        outValue[9] -= (dir[9] * fbGyroErr * BALANCE_KNEE_GAIN); // L_KNEE

        outValue[4] -= (dir[4] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN); // R_ANKLE_PITCH
        outValue[10] -= (dir[10] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN); // L_ANKLE_PITCH

        outValue[5] -= (dir[5] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN); // R_ANKLE_ROLL
        outValue[11] -= (dir[11] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN); // L_ANKLE_ROLL
        // std::cout<<outValue[1]<<std::endl<<std::endl;
    }

}

}
