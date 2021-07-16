/*
 des  : rscuad walking node 
 year : 2021
 
*/

// dev : danu andrean




#include <robotis_op_simulation_walking/gazebo_walking_node.h>

using namespace robotis_op;

#include <iostream>

#include <stdlib.h>     /* srand, rand */
#include <std_msgs/Float64.h>
#include <math.h>

#include <robotis_op_simulation_walking/math/Matrix.h>
#define MX28_1024



namespace robotis_op {
using namespace Robot;

SimulationWalkingNode::SimulationWalkingNode(ros::NodeHandle nh)
    : nh_(nh)
    , walking_(nh)
{

    rscuad_robot_publisher = nh_.advertise<std_msgs::String>("rscuad_manager/robot",1);

    ROS_WARN("SimulationWalkingNode ...");
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

    cmd_vel_subscriber_ = nh_.subscribe("/rscuad/cmd_vel", 100, &SimulationWalkingNode::cmdVelCb, this);
    enable_walking_subscriber_ = nh_.subscribe("/rscuad/enable_walking", 100, &SimulationWalkingNode::enableWalkCb, this);
    imu_subscriber_ = nh_.subscribe("/rscuad/imu", 100, &SimulationWalkingNode::imuCb, this);
}

SimulationWalkingNode::~SimulationWalkingNode()
{
}




void SimulationWalkingNode::Process()
{

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
    
    ROS_INFO("calc R0 = %f", outValue[0]);
    ROS_INFO("calc R1 = %f", outValue[1]);
    ROS_INFO("calc R2 = %f", outValue[2]);
    ROS_INFO("calc R3 = %f", outValue[3]);
    ROS_INFO("calc R4 = %f", outValue[4]);
    ROS_INFO("calc R5 = %f", outValue[5]);
    
    ROS_INFO("calc L0 = %f", outValue[6]);
    ROS_INFO("calc L1 = %f", outValue[7]);
    ROS_INFO("calc L2 = %f", outValue[8]);
    ROS_INFO("calc L3 = %f", outValue[9]);
    ROS_INFO("calc L4 = %f", outValue[10]);
    ROS_INFO("calc L5 = %f", outValue[11]);

    ROS_INFO("calc L12 = %f", outValue[12]);
    ROS_INFO("calc R13 = %f", outValue[13]);

    // ROS_WARN("Proses in node ...");
    // ROS_WARN("%f", j_ankle1_r_msg.data);

    //====================send to robot======================
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
                    
     rscuad_robot_publisher.publish(rscuad_robot);
   
    

    //====================send to Gazebo======================
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

}

void SimulationWalkingNode::cmdVelCb(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_ERROR("cmdval start");
    double period = walking_.PERIOD_TIME;
    walking_.X_MOVE_AMPLITUDE=(msg->linear.x/period*1000.0*10000.0);
    walking_.Y_MOVE_AMPLITUDE=(msg->linear.y/period*1000.0*10000.0);
    // compute the angular motion parameters to achieve the desired angular speed
    walking_.A_MOVE_AMPLITUDE=(msg->angular.z/period*180.0)/(2.0*3.14159);

}


void SimulationWalkingNode::enableWalkCb(std_msgs::BoolConstPtr enable)
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



void SimulationWalkingNode::imuCb(sensor_msgs::ImuConstPtr msg)
{
    walking_.fbGyroErr = msg->linear_acceleration.x*0.1;
    walking_.rlGyroErr = -msg->linear_acceleration.y*0.02;
}


void SimulationWalkingNode::dynamicReconfigureCb(robotis_op_simulation_walking::robotis_op_walkingConfig &config, uint32_t level)
{
    // ROS_INFO("%f", config.X_OFFSET);
    // ROS_INFO("%f", config.X_OFFSET);
    // ROS_INFO("%f", config.Y_OFFSET);
    // ROS_INFO("%f", config.Z_OFFSET);
    // ROS_INFO("%f", config.R_OFFSET);
    // ROS_INFO("%f", config.P_OFFSET);
    // ROS_INFO("%f", config.A_OFFSET);
    // ROS_INFO("%f", config.PERIOD_TIME);
    // ROS_INFO("%f", config.DSP_RATIO);
    // ROS_INFO("%f", config.STEP_FB_RATIO);
    // ROS_INFO("%f", config.Z_MOVE_AMPLITUDE);
    // ROS_INFO("%f", config.Y_SWAP_AMPLITUDE);
    // ROS_INFO("%f", config.PELVIS_OFFSET);
    // ROS_INFO("%f", config.ARM_SWING_GAIN);
    // ROS_INFO("%f", config.BALANCE_KNEE_GAIN);
    // ROS_INFO("%f", config.BALANCE_ANKLE_PITCH_GAIN);
    // ROS_INFO("%f", config.BALANCE_HIP_ROLL_GAIN);
    // ROS_INFO("%f", config.BALANCE_ANKLE_ROLL_GAIN);
    // ROS_INFO("%f", config.HIP_PITCH_OFFSET);

    walking_.X_OFFSET=config.X_OFFSET;
    walking_.Y_OFFSET=config.Y_OFFSET;
    walking_.Z_OFFSET=config.Z_OFFSET;
    walking_.R_OFFSET=config.R_OFFSET;
    walking_.P_OFFSET=config.P_OFFSET;
    walking_.A_OFFSET=config.A_OFFSET;
    walking_.PERIOD_TIME=config.PERIOD_TIME;
    walking_.DSP_RATIO=config.DSP_RATIO;
    walking_.STEP_FB_RATIO=config.STEP_FB_RATIO;
    walking_.Z_MOVE_AMPLITUDE=config.Z_MOVE_AMPLITUDE;
    walking_.Y_SWAP_AMPLITUDE=config.Y_SWAP_AMPLITUDE;
    walking_.PELVIS_OFFSET=config.PELVIS_OFFSET;
    walking_.ARM_SWING_GAIN=config.ARM_SWING_GAIN;
    walking_.BALANCE_KNEE_GAIN=config.BALANCE_KNEE_GAIN;
    walking_.BALANCE_ANKLE_PITCH_GAIN=config.BALANCE_ANKLE_PITCH_GAIN;
    walking_.BALANCE_HIP_ROLL_GAIN=config.BALANCE_HIP_ROLL_GAIN;
    walking_.BALANCE_ANKLE_ROLL_GAIN=config.BALANCE_ANKLE_ROLL_GAIN;
    walking_.HIP_PITCH_OFFSET=config.HIP_PITCH_OFFSET;
}

void SimulationWalkingNode::Mission()
{
    int periode = walking_.periode_calc();
    walking_.Start();

    walking_.X_MOVE_AMPLITUDE = 10;
    walking_.HIP_PITCH_OFFSET = 5;

    if(periode > 10 ){
        walking_.A_MOVE_AMPLITUDE = 18;
        walking_.X_MOVE_AMPLITUDE = 5;
        walking_.HIP_PITCH_OFFSET = 6;
        
        ROS_INFO("righ");
    }
    ROS_INFO("periode %d",periode);
    ROS_INFO("mission mode");

}
}



int main(int argc, char **argv)
{
    int i = 0;

    ros::init(argc, argv, ROS_PACKAGE_NAME);

    ros::NodeHandle nh;
    double control_rate;
    nh.param("robotis_op_walking/control_rate", control_rate, 125.0);
    control_rate = 125.0;

    SimulationWalkingNode gazebo_walking_node(nh);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Time last_time = ros::Time::now();
    ros::Rate rate(control_rate);
    ROS_INFO("Starting walking");
    //gazebo_walking.Start();
    gazebo_walking_node.walking_.Initialize();


    dynamic_reconfigure::Server<robotis_op_simulation_walking::robotis_op_walkingConfig> srv;
    dynamic_reconfigure::Server<robotis_op_simulation_walking::robotis_op_walkingConfig>::CallbackType cb;
    cb = boost::bind(&SimulationWalkingNode::dynamicReconfigureCb, &gazebo_walking_node, _1, _2);
    srv.setCallback(cb);


    ROS_INFO("Started walking");

    while (ros::ok())
    {
        
        // make mission
        gazebo_walking_node.Mission();

        rate.sleep();
        ros::Time current_time = ros::Time::now();
        ros::Duration elapsed_time = current_time - last_time;
        gazebo_walking_node.Process();

        last_time = current_time;

    }

    return 0;
}

