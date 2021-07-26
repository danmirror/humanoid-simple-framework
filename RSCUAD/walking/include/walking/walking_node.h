/*
 * desc : rscuad walking node 
 * year : 2021
 * dev  : danu andrean
 *
 */

#ifndef WALKING_NODE_H
#define WALKING_NODE_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "rscuad_manager/rscuad_manager.h"

#include <sstream>
#include <walking/walking.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <walking/walk_tunnerConfig.h>

namespace robotis_op {

class WalkingNode {
public:
    WalkingNode(ros::NodeHandle nh);
    ~WalkingNode();

    void Mission();
    void Process();
    void walk_tunner();
    void enableWalkCb(std_msgs::BoolConstPtr enable);
    void cmdVelCb(const geometry_msgs::Twist::ConstPtr& msg);
    void imuCb(const sensor_msgs::ImuConstPtr msg);
    void dynamicReconfigureCb(walking::walk_tunnerConfig &config, uint32_t level);

    int counter = 0;
    int *ptr_ID[20] {
        &DXL_ID_1,&DXL_ID_2,&DXL_ID_3,&DXL_ID_4,&DXL_ID_5,&DXL_ID_6,&DXL_ID_7,&DXL_ID_8,&DXL_ID_9,&DXL_ID_10,
        &DXL_ID_11,&DXL_ID_12,&DXL_ID_13,&DXL_ID_14,&DXL_ID_15,&DXL_ID_16,&DXL_ID_17,&DXL_ID_18,&DXL_ID_19,&DXL_ID_20};
        
    Walking walking_;
protected:


private:


    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_subscriber_;
    ros::Subscriber enable_walking_subscriber_;
    ros::Subscriber imu_subscriber_;

    ros::Publisher j_pelvis_l_publisher_;
    ros::Publisher j_thigh1_l_publisher_;
    ros::Publisher j_thigh2_l_publisher_;
    ros::Publisher j_tibia_l_publisher_;
    ros::Publisher j_ankle1_l_publisher_;
    ros::Publisher j_ankle2_l_publisher_;
    ros::Publisher j_shoulder_l_publisher_;

    ros::Publisher j_pelvis_r_publisher_;
    ros::Publisher j_thigh1_r_publisher_;
    ros::Publisher j_thigh2_r_publisher_;
    ros::Publisher j_tibia_r_publisher_;
    ros::Publisher j_ankle1_r_publisher_;
    ros::Publisher j_ankle2_r_publisher_;
    ros::Publisher j_shoulder_r_publisher_;
    ros::Publisher rscuad_robot_publisher;



};

}
#endif //WALKING_NODE_H
