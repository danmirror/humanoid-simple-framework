/*
 * desc : rscuad walking node 
 * year : 2021
 * dev  : danu andrean
 *
 */


#include <walking/walking_node.h>

using namespace robotis_op;

#include <iostream>

#include <stdlib.h>     /* srand, rand */
#include <std_msgs/Float64.h>
#include <math.h>

#include <walking/math/Matrix.h>
#define MX28_1024


int index_ = 0;
int dxl_comm_result = COMM_TX_FAIL;             // Communication result
int dxl_goal_position[2] = {MINIMUM_POSITION_LIMIT, MAXIMUM_POSITION_LIMIT};         // Goal position

uint8_t dxl_error = 0;                          // DYNAMIXEL error
#if defined(XL320)
int16_t dxl_present_position = 0;               // XL-320 uses 2 byte Position data
#else
int32_t dxl_present_position = 0;               // Read 4 byte Position data
#endif




namespace robotis_op {
using namespace Robot;

WalkingNode::WalkingNode(ros::NodeHandle nh)
    : nh_(nh)
    , walking_(nh)
{

    // rscuad_robot_publisher = nh_.advertise<std_msgs::String>("rscuad_manager/robot",1);
    rscuad::rscuad_manager *rscuad = new rscuad::rscuad_manager;
    //initial power
    rscuad->manager_init();

    // active all servo
    if (portHandler->setBaudRate(BAUDRATE)) {
        printf("Succeeded to change the baudrate!\n");
    }
    for(int i =0; i<20; i++){

        // Enable DYNAMIXEL Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler,*ptr_ID[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            printf(" ID %d %s\n",i, packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0) {
            printf(" ERROR ID %d %s\n",i, packetHandler->getRxPacketError(dxl_error));
        }
        else {
            printf("Succeeded enabling DYNAMIXEL Torque.\n");
        }
    }


    ROS_WARN("WalkingNode ...");
    j_pelvis_l_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/l_hip_yaw_position/command",1);
    j_thigh1_l_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/l_hip_roll_position/command",1);
    j_thigh2_l_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/l_hip_pitch_position/command",1);
    j_tibia_l_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/l_knee_position/command",1);
    j_ankle1_l_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/l_ank_pitch_position/command",1);
    j_ankle2_l_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/l_ank_roll_position/command",1);
    j_shoulder_l_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/l_sho_pitch_position/command",1);

    j_pelvis_r_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/r_hip_yaw_position/command",1);
    j_thigh1_r_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/r_hip_roll_position/command",1);
    j_thigh2_r_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/r_hip_pitch_position/command",1);
    j_tibia_r_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/r_knee_position/command",1);
    j_ankle1_r_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/r_ank_pitch_position/command",1);
    j_ankle2_r_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/r_ank_roll_position/command",1);
    j_shoulder_r_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/r_sho_pitch_position/command",1);

    cmd_vel_subscriber_ = nh_.subscribe("/rscuad/cmd_vel", 100, &WalkingNode::cmdVelCb, this);
    enable_walking_subscriber_ = nh_.subscribe("/rscuad/enable_walking", 100, &WalkingNode::enableWalkCb, this);
    imu_subscriber_ = nh_.subscribe("/rscuad/imu", 100, &WalkingNode::imuCb, this);
}

WalkingNode::~WalkingNode()
{
}




void WalkingNode::Process()
{

     // alocation memory
    rscuad::rscuad_manager *rscuad = new rscuad::rscuad_manager;
 

    std_msgs::Float64 j_pelvis_l_msg;
    std_msgs::Float64 j_thigh1_l_msg;
    std_msgs::Float64 j_thigh2_l_msg;
    std_msgs::Float64 j_tibia_l_msg;
    std_msgs::Float64 j_ankle1_l_msg;
    std_msgs::Float64 j_ankle2_l_msg;
    std_msgs::Float64 j_shoulder_l_msg;
    std_msgs::Float64 j_pelvis_r_msg;
    std_msgs::Float64 j_thigh1_r_msg;
    std_msgs::Float64 j_thigh2_r_msg;
    std_msgs::Float64 j_tibia_r_msg;
    std_msgs::Float64 j_ankle1_r_msg;
    std_msgs::Float64 j_ankle2_r_msg;
    std_msgs::Float64 j_shoulder_r_msg;

    

    double outValue[14];
    walking_.Process(&outValue[0]);


    j_pelvis_r_msg.data = outValue[0];
    j_thigh1_r_msg.data = outValue[1];
    j_thigh2_r_msg.data = outValue[2];
    j_tibia_r_msg.data = outValue[3];
    j_ankle1_r_msg.data = outValue[4];
    j_ankle2_r_msg.data = outValue[5];

    j_pelvis_l_msg.data = outValue[6];
    j_thigh1_l_msg.data = outValue[7];
    j_thigh2_l_msg.data = outValue[8];
    j_tibia_l_msg.data = outValue[9];
    j_ankle1_l_msg.data = outValue[10];
    j_ankle2_l_msg.data = outValue[11];
    j_shoulder_r_msg.data = outValue[12];
    j_shoulder_l_msg.data = outValue[13];
    
    // ROS_INFO("calc R0 = %f", outValue[0]);
    // ROS_INFO("calc R1 = %f", outValue[1]);
    // ROS_INFO("calc R2 = %f", outValue[2]);
    // ROS_INFO("calc R3 = %f", outValue[3]);
    // ROS_INFO("calc R4 = %f", outValue[4]);
    // ROS_INFO("calc R5 = %f", outValue[5]);
    
    // ROS_INFO("calc L0 = %f", outValue[6]);
    // ROS_INFO("calc L1 = %f", outValue[7]);
    // ROS_INFO("calc L2 = %f", outValue[8]);
    // ROS_INFO("calc L3 = %f", outValue[9]);
    // ROS_INFO("calc L4 = %f", outValue[10]);
    // ROS_INFO("calc L5 = %f", outValue[11]);

    // ROS_INFO("calc L12 = %f", outValue[12]);
    // ROS_INFO("calc R13 = %f", outValue[13]);

    // ROS_WARN("Proses in node ...");
    // ROS_WARN("%f", j_ankle1_r_msg.data);

    //===========================send to Gazebo===============================
    j_pelvis_l_publisher_.publish(j_pelvis_l_msg);
    j_thigh1_l_publisher_.publish(j_thigh1_l_msg);
    j_thigh2_l_publisher_.publish(j_thigh2_l_msg);
    j_tibia_l_publisher_.publish(j_tibia_l_msg);
    j_ankle1_l_publisher_.publish(j_ankle1_l_msg);
    j_ankle2_l_publisher_.publish(j_ankle2_l_msg);
    j_shoulder_l_publisher_.publish(j_shoulder_l_msg);

    j_pelvis_r_publisher_.publish(j_pelvis_r_msg);
    j_thigh1_r_publisher_.publish(j_thigh1_r_msg);
    j_thigh2_r_publisher_.publish(j_thigh2_r_msg);
    j_tibia_r_publisher_.publish(j_tibia_r_msg);
    j_ankle1_r_publisher_.publish(j_ankle1_r_msg);
    j_ankle2_r_publisher_.publish(j_ankle2_r_msg);
    j_shoulder_r_publisher_.publish(j_shoulder_r_msg);


    //====================send to manager, but not use it======================
    //send to robot
    std_msgs::String rscuad_robot;
    std::stringstream  str_0,str_1,str_2,str_3,str_4,str_5,str_6,str_7,str_8,str_9,str_10,str_11,str_12,str_13;
    float float_0,float_1,float_2,float_3,float_4,float_5,float_6,float_7,float_8,float_9,float_10,float_11,float_12,float_13;

    float_0 = outValue[0];
    float_1 = outValue[1];
    float_2 = outValue[2];
    float_3 = outValue[3];
    float_4 = outValue[4];
    float_5 = outValue[5];
    float_6 = outValue[6];
    float_7 = outValue[7];
    float_8 = outValue[8];
    float_9 = outValue[9];
    float_10 = outValue[10];
    float_11 = outValue[11];
    float_12 = outValue[12];
    float_13 = outValue[13];

    str_0 << float_0;
    str_1 << float_1;
    str_2 << float_2;
    str_3 << float_3;
    str_4 << float_4;
    str_5 << float_5;
    str_6 << float_6;
    str_7 << float_7;
    str_8 << float_8;
    str_9 << float_9;
    str_10 << float_10;
    str_11 << float_11;
    str_12 << float_12;
    str_13 << float_13;

    rscuad_robot.data = str_0.str()+","+
                        str_1.str()+","+
                        str_2.str()+","+
                        str_3.str()+","+
                        str_4.str()+","+
                        str_5.str()+","+
                        str_6.str()+","+
                        str_7.str()+","+
                        str_8.str()+","+
                        str_9.str()+","+
                        str_10.str()+","+
                        str_11.str()+","+
                        str_12.str()+","+
                        str_13.str();
                        
    // rscuad->setup_joint();
    //  rscuad_robot_publisher.publish(rscuad_robot);   //send to manager ROS protocol

    // char *newdata = (char*)rscuad_robot.data.c_str();

    // rscuad->move_robot(newdata);                     //send to manager library protocol
    //==============================end to manager===============================


    //=============================== dxl execute================================
    //  j_pelvis_r_msg.data = outValue[0];
    // j_thigh1_r_msg.data = outValue[1];
    // j_thigh2_r_msg.data = outValue[2];
    // j_tibia_r_msg.data = outValue[3];
    // j_ankle1_r_msg.data = outValue[4];
    // j_ankle2_r_msg.data = outValue[5];

    // j_pelvis_l_msg.data = outValue[6];
    // j_thigh1_l_msg.data = outValue[7];
    // j_thigh2_l_msg.data = outValue[8];
    // j_tibia_l_msg.data = outValue[9];
    // j_ankle1_l_msg.data = outValue[10];
    // j_ankle2_l_msg.data = outValue[11];
    // j_shoulder_r_msg.data = outValue[12];
    // j_shoulder_l_msg.data = outValue[13];

    //  j_pelvis_l_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/l_hip_yaw_position/command",1);
    // j_thigh1_l_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/l_hip_roll_position/command",1);
    // j_thigh2_l_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/l_hip_pitch_position/command",1);
    // j_tibia_l_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/l_knee_position/command",1);
    // j_ankle1_l_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/l_ank_pitch_position/command",1);
    // j_ankle2_l_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/l_ank_roll_position/command",1);
    // j_shoulder_l_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/l_sho_pitch_position/command",1);

    // j_pelvis_r_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/r_hip_yaw_position/command",1);
    // j_thigh1_r_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/r_hip_roll_position/command",1);
    // j_thigh2_r_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/r_hip_pitch_position/command",1);
    // j_tibia_r_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/r_knee_position/command",1);
    // j_ankle1_r_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/r_ank_pitch_position/command",1);
    // j_ankle2_r_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/r_ank_roll_position/command",1);
    // j_shoulder_r_publisher_ = nh_.advertise<std_msgs::Float64>("/rscuad/r_sho_pitch_position/command",1);

    //calculation radian to degree
    float target_angle_1 = abs(((RADIAN2DEGREE*float_12) *( MAXIMUM_POSITION_LIMIT/360)) - rscuad->offset_ID(1));
    float target_angle_2 = abs(((RADIAN2DEGREE*float_13) *( MAXIMUM_POSITION_LIMIT/360)) - rscuad->offset_ID(2));
    float target_angle_3 = rscuad->offset_ID(3);
    float target_angle_4 = rscuad->offset_ID(4);
    float target_angle_5 = rscuad->offset_ID(5);
    float target_angle_6 = rscuad->offset_ID(6);

    float target_angle_7 = abs(((RADIAN2DEGREE*float_0) *( MAXIMUM_POSITION_LIMIT/360)) - rscuad->offset_ID(7));
    float target_angle_8 = abs(((RADIAN2DEGREE*float_6) *( MAXIMUM_POSITION_LIMIT/360)) - rscuad->offset_ID(8));
    float target_angle_9 = abs(((RADIAN2DEGREE*float_1) *( MAXIMUM_POSITION_LIMIT/360)) - rscuad->offset_ID(9));
    float target_angle_10 = abs(((RADIAN2DEGREE*float_7) *( MAXIMUM_POSITION_LIMIT/360)) - rscuad->offset_ID(10));
    float target_angle_11 = abs(((RADIAN2DEGREE*float_2) *( MAXIMUM_POSITION_LIMIT/360)) - rscuad->offset_ID(11));

    float target_angle_12 = abs(((RADIAN2DEGREE*float_8) *( MAXIMUM_POSITION_LIMIT/360)) - rscuad->offset_ID(12));
    float target_angle_13 = abs(((RADIAN2DEGREE*float_3) *( MAXIMUM_POSITION_LIMIT/360)) - rscuad->offset_ID(13));
    float target_angle_14 = abs(((RADIAN2DEGREE*float_9) *( MAXIMUM_POSITION_LIMIT/360)) - rscuad->offset_ID(14));
    float target_angle_15 = abs(((RADIAN2DEGREE*float_4) *( MAXIMUM_POSITION_LIMIT/360)) - rscuad->offset_ID(15));
    float target_angle_16 = abs(((RADIAN2DEGREE*float_10) *( MAXIMUM_POSITION_LIMIT/360)) - rscuad->offset_ID(16));
    float target_angle_17 = abs(((RADIAN2DEGREE*float_5) *( MAXIMUM_POSITION_LIMIT/360)) - rscuad->offset_ID(17));
    float target_angle_18 = abs(((RADIAN2DEGREE*float_11) *( MAXIMUM_POSITION_LIMIT/360)) - rscuad->offset_ID(18));
    float target_angle_19 = rscuad->offset_ID(19);
    float target_angle_20 = rscuad->offset_ID(20);
    
    packetHandler->write4ByteTxRx(portHandler, DXL_ID_1, ADDR_GOAL_POSITION, target_angle_1, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, DXL_ID_2, ADDR_GOAL_POSITION, target_angle_2, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, DXL_ID_3, ADDR_GOAL_POSITION, target_angle_3, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, DXL_ID_4, ADDR_GOAL_POSITION, target_angle_4, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, DXL_ID_5, ADDR_GOAL_POSITION, target_angle_5, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, DXL_ID_6, ADDR_GOAL_POSITION, target_angle_6, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, DXL_ID_7, ADDR_GOAL_POSITION, target_angle_7, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, DXL_ID_8, ADDR_GOAL_POSITION, target_angle_8, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, DXL_ID_9, ADDR_GOAL_POSITION, target_angle_9, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, DXL_ID_10, ADDR_GOAL_POSITION, target_angle_10, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, DXL_ID_11, ADDR_GOAL_POSITION, target_angle_11, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, DXL_ID_12, ADDR_GOAL_POSITION, target_angle_12, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, DXL_ID_13, ADDR_GOAL_POSITION, target_angle_13, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, DXL_ID_14, ADDR_GOAL_POSITION, target_angle_14, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, DXL_ID_15, ADDR_GOAL_POSITION, target_angle_15, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, DXL_ID_16, ADDR_GOAL_POSITION, target_angle_16, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, DXL_ID_17, ADDR_GOAL_POSITION, target_angle_17, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, DXL_ID_18, ADDR_GOAL_POSITION, target_angle_18, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, DXL_ID_19, ADDR_GOAL_POSITION, target_angle_19, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, DXL_ID_20, ADDR_GOAL_POSITION, target_angle_20, &dxl_error);
    



    // for(int i=0; i<20;i++){

    //     packetHandler->read4ByteTxRx(portHandler, *ptr_ID[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);

    //     ROS_INFO("join position %d", dxl_present_position);
    // }
    //==============================end dxl execute==============================

}

void WalkingNode::cmdVelCb(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_ERROR("cmdval start");
    double period = walking_.PERIOD_TIME;
    walking_.X_MOVE_AMPLITUDE=(msg->linear.x/period*1000.0*10000.0);
    walking_.Y_MOVE_AMPLITUDE=(msg->linear.y/period*1000.0*10000.0);
    // compute the angular motion parameters to achieve the desired angular speed
    walking_.A_MOVE_AMPLITUDE=(msg->angular.z/period*180.0)/(2.0*3.14159);

}


void WalkingNode::enableWalkCb(std_msgs::BoolConstPtr enable)
{
    if(enable->data)
    {
        if(!walking_.IsRunning())
        {
            walking_.Start();
        }
    }
    else
    {
        if(walking_.IsRunning())
        {
            walking_.Stop();
        }
    }
}



void WalkingNode::imuCb(sensor_msgs::ImuConstPtr msg)
{
    walking_.fbGyroErr = msg->linear_acceleration.x*0.1;
    walking_.rlGyroErr = -msg->linear_acceleration.y*0.02;
}


void WalkingNode::dynamicReconfigureCb(walking::walk_tunnerConfig &config, uint32_t level)
{
    ROS_INFO("%f", config.X_OFFSET);
    ROS_INFO("%f", config.X_OFFSET);
    ROS_INFO("%f", config.Y_OFFSET);
    ROS_INFO("%f", config.Z_OFFSET);
    ROS_INFO("%f", config.R_OFFSET);
    ROS_INFO("%f", config.P_OFFSET);
    ROS_INFO("%f", config.A_OFFSET);
    ROS_INFO("%f", config.PERIOD_TIME);
    ROS_INFO(" dsp %f", config.DSP_RATIO);
    ROS_INFO("%f", config.STEP_FB_RATIO);
    ROS_INFO("%f", config.Z_MOVE_AMPLITUDE);
    ROS_INFO("%f", config.Y_SWAP_AMPLITUDE);
    ROS_INFO("%f", config.PELVIS_OFFSET);
    ROS_INFO("%f", config.ARM_SWING_GAIN);
    ROS_INFO("%f", config.BALANCE_KNEE_GAIN);
    ROS_INFO("%f", config.BALANCE_ANKLE_PITCH_GAIN);
    ROS_INFO("%f", config.BALANCE_HIP_ROLL_GAIN);
    ROS_INFO("%f", config.BALANCE_ANKLE_ROLL_GAIN);
    ROS_INFO("%f", config.HIP_PITCH_OFFSET);

    walking_.X_OFFSET                   =   config.X_OFFSET;
    walking_.Y_OFFSET                   =   config.Y_OFFSET;
    
    walking_.Z_OFFSET                   =   config.Z_OFFSET;
    walking_.R_OFFSET                   =   config.R_OFFSET;
    walking_.P_OFFSET                   =   config.P_OFFSET;
    walking_.A_OFFSET                   =   config.A_OFFSET;
    walking_.PERIOD_TIME                =   config.PERIOD_TIME;
    walking_.DSP_RATIO                  =   config.DSP_RATIO;
    walking_.STEP_FB_RATIO              =   config.STEP_FB_RATIO;
    walking_.Z_MOVE_AMPLITUDE           =   config.Z_MOVE_AMPLITUDE;
    walking_.Y_SWAP_AMPLITUDE           =   config.Y_SWAP_AMPLITUDE;
    walking_.PELVIS_OFFSET              =   config.PELVIS_OFFSET;
    walking_.ARM_SWING_GAIN             =   config.ARM_SWING_GAIN;
    walking_.BALANCE_KNEE_GAIN          =   config.BALANCE_KNEE_GAIN;
    walking_.BALANCE_ANKLE_PITCH_GAIN   =   config.BALANCE_ANKLE_PITCH_GAIN;
    walking_.BALANCE_HIP_ROLL_GAIN      =   config.BALANCE_HIP_ROLL_GAIN;
    walking_.BALANCE_ANKLE_ROLL_GAIN    =   config.BALANCE_ANKLE_ROLL_GAIN;
    walking_.HIP_PITCH_OFFSET           =   config.HIP_PITCH_OFFSET;
}


void WalkingNode::Mission()
{
    if( walking_.init_status()== true)
    {
        int periode = walking_.periode_calc();
        walking_.Start();

        walking_.X_MOVE_AMPLITUDE = 10;
        walking_.HIP_PITCH_OFFSET = 15;
        walking_.X_OFFSET = 0;

        // if(periode > 10 ){
        //     walking_.A_MOVE_AMPLITUDE = 18;
        //     walking_.X_MOVE_AMPLITUDE = 5;
        //     walking_.HIP_PITCH_OFFSET = 6;
            
        //     ROS_INFO("righ");
        // }
        ROS_INFO("periode %d",periode);
        ROS_INFO("mission mode");
    }

}
}



int main(int argc, char **argv)
{
    int i = 0;

    ros::init(argc, argv, ROS_PACKAGE_NAME);

    ros::NodeHandle nh;
    double control_rate;
    nh.param("walk_tunner/control_rate", control_rate, 100.0);
    control_rate = 100.0;

    WalkingNode walking_node(nh);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Time last_time = ros::Time::now();
    ros::Rate rate(control_rate);
    ROS_INFO("Starting walking");
    //walking.Start();

    walking_node.walking_.Initialize();    // make very smooth
    // walking_node.walking_.walk_ready(); // make smooth but not perfect

    dynamic_reconfigure::Server<walking::walk_tunnerConfig> srv;
    dynamic_reconfigure::Server<walking::walk_tunnerConfig>::CallbackType cb;
    cb = boost::bind(&WalkingNode::dynamicReconfigureCb, &walking_node, _1, _2);
    srv.setCallback(cb);


    ROS_INFO("Started walking");

    while (ros::ok())
    {
        
        // make mission
        walking_node.Mission();

        rate.sleep();
        ros::Time current_time = ros::Time::now();
        ros::Duration elapsed_time = current_time - last_time;
        walking_node.Process();

        last_time = current_time;

    }

    return 0;
}

