// /*
//  des  : rscuad manager | if using it activate execute in makefiles
//  year : 2021
 
// */

// // author : danu andrean



#include "rscuad_manager/rscuad_manager.h"
int dxl_comm_results = COMM_TX_FAIL;             // Communication result

uint8_t dxl_errors = 0;                          // DYNAMIXEL error


void Callback(const std_msgs::String::ConstPtr& msg){
    rscuad::rscuad_manager *rscuad =  new rscuad::rscuad_manager;
    char *newdata = (char*)msg->data.c_str();

      // ------------------data parsing------------------
    bool lock=false;
    int memory = 0;
    int count = 0;
    char joint[10]="";
    char value[10]="";

    for(int i=0; i <strlen(newdata);i++){
        if(str[i] == ','){
            count ++;
            memory = i+1;
        }

        else {
            if (count == 0)
                joint[i] = newdata[i]; 
            if (count == 1)
                value[i-memory] = newdata[i]; 
        }
 
    }
    ROS_WARN("join:  %s", joint);
    ROS_WARN("value: %f",  atof(value));
    ROS_INFO("data masuk: %s", str);

    // ROS_INFO("newdata");
    // ROS_INFO(newdata);
    rscuad->move_joint(newdata,0,0);
}

void Manager_Robot_Callback(const std_msgs::String::ConstPtr& msg){
    rscuad::rscuad_manager *rscuad =  new rscuad::rscuad_manager;
    char *newdata = (char*)msg->data.c_str();
    // ROS_INFO(" new data %.2f" ,msg->data);
    ROS_INFO(newdata);
    rscuad->move_robot(newdata);
}



int main(int argc, char **argv)
{
    // alocation memory
    rscuad::rscuad_manager *rscuad = new rscuad::rscuad_manager;

    //initial power
    rscuad->manager_init();


    ros::init(argc, argv, "rscuad_manager");
    ros::NodeHandle nh;

    ros::Subscriber joint = nh.subscribe("rscuad_manager", 100, Callback);
    ros::Subscriber robot = nh.subscribe("rscuad_manager/robot", 100, Manager_Robot_Callback);
    ros::spin();
    
}


