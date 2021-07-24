/*
 des  : rscuad manager 
 year : 2021
 
*/

// author : danu andrean


#include "rscuad_manager/rscuad_manager.h"


int index_ = 0;
int dxl_comm_result = COMM_TX_FAIL;             // Communication result
int dxl_goal_position[2] = {MINIMUM_POSITION_LIMIT, MAXIMUM_POSITION_LIMIT};         // Goal position

uint8_t dxl_error = 0;                          // DYNAMIXEL error
#if defined(XL320)
int16_t dxl_present_position = 0;               // XL-320 uses 2 byte Position data
#else
int32_t dxl_present_position = 0;               // Read 4 byte Position data
#endif




int rscuad::rscuad_manager::manager_init()
{

    ROS_INFO("manager init loaded ..");

    RobotisController *controller =  RobotisController::getInstance();

    
    // controller->addSensorModule((SensorModule*) OpenCRModule::getInstance());

    if (portHandler->setBaudRate(BAUDRATE)) {
    printf("Succeeded to change the baudrate!\n");
    }
   

     // power on dxls
    int torque_on_count = 0;

    while (torque_on_count < 5)
    {
      int _return = packetHandler->write1ByteTxRx(portHandler, SUB_CONTROLLER_ID, POWER_CTRL_TABLE, 1);

      if(_return != 0)
        ROS_ERROR("Torque on DXLs! [%s]", packetHandler->getRxPacketError(_return));
      else
        ROS_INFO("Torque on DXLs!");

      if (_return == 0)
        break;
      else
        torque_on_count++;
    }

    // //=================================================setup id==========================================================
    // // Enable DYNAMIXEL Torque
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_13, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // if (dxl_comm_result != COMM_SUCCESS) {
    //     printf(" > %s\n", packetHandler->getTxRxResult(dxl_comm_result));
    // }
    // else if (dxl_error != 0) {
    //     printf(" > %s\n", packetHandler->getRxPacketError(dxl_error));
    // }
    // else {
    //     printf("Succeeded enabling DYNAMIXEL Torque.\n");
    // }
   
    // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID_13, ADDR_GOAL_POSITION,2000, &dxl_error);
    ////=====================================================================================================================

    usleep(100 * 1000);

    // set RGB-LED to GREEN
    int led_full_unit = 0x1F;
    int led_range = 5;
    int led_value = led_full_unit << led_range;
    int _return = packetHandler->write2ByteTxRx(portHandler, SUB_CONTROLLER_ID, RGB_LED_CTRL_TABLE, led_value);

    if(_return != 0)
      ROS_ERROR("Fail to control LED [%s]", packetHandler->getRxPacketError(_return));
}


int rscuad::rscuad_manager::dxl_process(){

  // //=================================================setup id==========================================================
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0) {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else {
    printf("Succeeded enabling DYNAMIXEL Torque.\n");
  }
  ////=====================================================================================================================

    while(1) {

        // Write goal position
        #if defined(XL320)  // XL-320 uses 2 byte Position data
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID_1, ADDR_GOAL_POSITION, dxl_goal_position[index_], &dxl_error);
        #else
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID_1, ADDR_GOAL_POSITION, dxl_goal_position[index_], &dxl_error);
        #endif
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0) {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }

        do {
            // Read the Present Position
            #if defined(XL320)  // XL-320 uses 2 byte Position data
            dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID_1, ADDR_PRESENT_POSITION, (uint16_t*)&dxl_present_position, &dxl_error);
            #else
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID_1, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
            #endif
            if (dxl_comm_result != COMM_SUCCESS) {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0) {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            }

            printf("[ID:%03d] Goal Position:%03d  Present Position:%03d\n", DXL_ID_1, dxl_goal_position[index_], dxl_present_position);

        } while((abs(dxl_goal_position[index_] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

        // Switch the Goal Position
        if (index_ == 0) {
            index_ = 1;
        }
        else {
            index_ = 0;
        }
    }

  // Disable DYNAMIXEL Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0) {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else {
    printf("Succeeded disabling DYNAMIXEL Torque.\n");
  }

  // Close port
  portHandler->closePort();
  return 0;
}

float rscuad::rscuad_manager::offset_ID(int id){
    float *ptr_ID[20] {
        &offset_1,&offset_2,&offset_3,&offset_4,&offset_5,&offset_6,&offset_7,&offset_8,&offset_9,&offset_10,
        &offset_11,&offset_12,&offset_13,&offset_14,&offset_15,&offset_16,&offset_17,&offset_18,&offset_19,&offset_20};

    return *ptr_ID[id-1];
}


int rscuad::rscuad_manager::move_robot(char *str){
    joint rscuad_joint;
    
    // ------------------data parsing------------------
    int count = 0;
    char joint_0 [10]= "";
    char joint_1[10]= "";
    char joint_2[10]= "";
    char joint_3[10]= "";
    char joint_4[10]= "";
    char joint_5[10]= "";
    char joint_6[10]= "";
    char joint_7[10]= "";
    char joint_8[10]= "";
    char joint_9[10]= "";
    char joint_10[10]= "";
    char joint_11[10]= "";
    char joint_12[10]= "";
    char joint_13[10]= "";
    int memory = 0;

    for(int i=0; i <strlen(str);i++){
        if(str[i] == ','){
            count ++;
            memory = i+1;
        }

        else{
            if (count == 0)
                joint_0[i] = str[i];
            if (count == 1)
                joint_1[i-memory] = str[i];
            if (count == 2)
                joint_2[i-memory] = str[i];
            if (count == 3)
                joint_3[i-memory] = str[i];
            if (count == 4)
                joint_4[i-memory] = str[i];
            if (count == 5)
                joint_5[i-memory] = str[i];
            if (count == 6)
                joint_6[i-memory] = str[i];
            if (count == 7)
                joint_7[i-memory] = str[i];
            if (count == 8)
                joint_8[i-memory] = str[i];
            if (count == 9)
                joint_9[i-memory] = str[i];
            if (count == 10)
                joint_10[i-memory] = str[i];
            if (count == 11)
                joint_11[i-memory] = str[i];
            if (count == 2)
                joint_12[i-memory] = str[i];
            if (count == 13)
                joint_13[i-memory] = str[i];
        }
    }

    // --------------Table sampling-------------------
    /*
    j_pelvis_r   =  0
    j_thigh1_r   =  1
    j_thigh2_r   =  2
    j_tibia_r    =  3
    j_ankle1_r   =  4
    j_ankle2_r   =  5

    j_pelvis_l   =  6
    j_thigh1_l   =  7
    j_thigh2_l   =  8
    j_tibia_l_   =  9
    j_ankle1_l   =  10
    j_ankle2_l   =  11
    j_shoulder_r =  12
    j_shoulder_l =  13
    */
               
    //data result
    // ROS_INFO("data masuk: %s", str);
    // ROS_WARN("0 : %.4f",atof(joint_0));
    // ROS_WARN("1 : %.4f",atof(joint_1));
    // ROS_WARN("2 : %.4f",atof(joint_2));
    // ROS_WARN("3 : %.4f",atof(joint_3));
    // ROS_WARN("4 : %.4f",atof(joint_4));
    // ROS_WARN("5 : %.4f",atof(joint_5));
    // ROS_WARN("6 : %.4f",atof(joint_6));
    // ROS_WARN("7 : %.4f",atof(joint_7));
    // ROS_WARN("8 : %.4f",atof(joint_8));
    // ROS_WARN("9 : %.4f",atof(joint_9));
    // ROS_WARN("10 : %.4f",atof(joint_10));
    // ROS_WARN("11 : %.4f",atof(joint_11))
    // ROS_WARN("12 : %.4f",atof(joint_12));
    // ROS_WARN("13 : %.4f",atof(joint_13));

    //offset calculation
    // max 4095

    float offset_13 = -2046;
    // calculation
    approach_angle =  MAXIMUM_POSITION_LIMIT/360;
    
    float target_angle_13 = abs(((RADIAN2DEGREE*atof(joint_3)) *approach_angle) - offset_13);

    ROS_WARN("target> > > %f",target_angle_13);



    //=============================== dxl execute, uncomment if used===============================
    // 

    // // Open port
    // if (portHandler->openPort()) {
    //     printf("Succeeded to open the port!\n" // // Open port
    // if (portHandler->openPort()) {
    //     printf("Succeeded to open the port!\n");
    // }
    // else {
    //     return 0;
    // }
    // if (portHandler->setBaudRate(BAUDRATE)) {
    //     printf("Succeeded to change the baudrate!\n");
    // }
    // else {
    //     return 0;
    // }

    // // Enable DYNAMIXEL Torque
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_13, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // if (dxl_comm_result != COMM_SUCCESS) {
    //     printf(" %s\n", packetHandler->getTxRxResult(dxl_comm_result));
    // }
    // else if (dxl_error != 0) {
    //     printf(" %s\n", packetHandler->getRxPacketError(dxl_error));
    // }
    // else {
    //     printf("Succeeded enabling DYNAMIXEL Torque.\n");
    // }
    
    // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID_13, ADDR_GOAL_POSITION, target_angle_13, &dxl_error);
    // dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID_13, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
    // ROS_INFO("join position %d", dxl_present_position););
    // }
    // else {
    //     return 0;
    // }
    // if (portHandler->setBaudRate(BAUDRATE)) {
    //     printf("Succeeded to change the baudrate!\n");
    // }
    // else {
    //     return 0;
    // }

    // // Enable DYNAMIXEL Torque
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_13, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // if (dxl_comm_result != COMM_SUCCESS) {
    //     printf(" %s\n", packetHandler->getTxRxResult(dxl_comm_result));
    // }
    // else if (dxl_error != 0) {
    //     printf(" %s\n", packetHandler->getRxPacketError(dxl_error));
    // }
    // else {
    //     printf("Succeeded enabling DYNAMIXEL Torque.\n");
    // }
    
    // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID_13, ADDR_GOAL_POSITION, target_angle_13, &dxl_error);
    // dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID_13, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
    // ROS_INFO("join position %d", dxl_present_position);
   
}



void rscuad::rscuad_manager::move_joint(char *str,int id,float velocity){
    //parsing to string
    std::string data;
    data = str;

    //call join in struct
    joint rscuad_joint;
    char *on_move;
    
    if(data == "gazebo" || data == "all"){
        // -----------------servo selection--------------
        if(id == 1)
            on_move = rscuad_joint.r_sho_pitch_position; 
        else if(id == 2)
            on_move = rscuad_joint.l_sho_pitch_position;
        else if(id == 3)
            on_move = rscuad_joint.r_sho_roll_position; 
        else if(id == 4)
            on_move = rscuad_joint.l_sho_roll_position; 
        else if(id == 5)
            on_move = rscuad_joint.r_el_position; 
        else if(id == 6)
            on_move = rscuad_joint.l_el_position; 
        else if(id == 7)
            on_move = rscuad_joint.r_hip_yaw_position; 
        else if(id == 8)
            on_move = rscuad_joint.l_hip_yaw_position; 
        else if(id == 9)
            on_move = rscuad_joint.r_hip_roll_position; 
        else if(id == 10)
            on_move = rscuad_joint.l_hip_roll_position;
        else if(id == 11)
            on_move = rscuad_joint.r_hip_pitch_position; 
        else if(id == 12)
            on_move = rscuad_joint.l_hip_pitch_position; 
        else if(id == 13)
            on_move = rscuad_joint.r_knee_position; 
        else if(id == 14)
            on_move = rscuad_joint.l_knee_position; 
        else if(id == 15)
            on_move = rscuad_joint.r_ank_roll_position; 
        else if(id == 16)
            on_move = rscuad_joint.l_ank_roll_position; 
        else if(id == 17)
            on_move = rscuad_joint.r_ank_pitch_position; 
        else if(id == 18)
            on_move = rscuad_joint.l_ank_pitch_position; 
        else if(id == 19)
            on_move = rscuad_joint.head_pan_position;
        else if(id == 20)
            on_move = rscuad_joint.head_tilt_position;

        ros::Rate loop_rate(10);
        ros::NodeHandle nh; 

        ros::Publisher pub = nh.advertise<std_msgs::Float64>(on_move, 1000);

        // ROS_INFO("%s",rscuad_joint.l_hip_yaw_position);

        std_msgs::Float64 move;
        move.data = velocity;
        while(ros::ok)
        {
            pub.publish(move);
            ros::spin();
            loop_rate.sleep();
        }

    }
    if(data == "robot" || data == "all"){

        // active all servo
        if (portHandler->setBaudRate(BAUDRATE)) {
            printf("Succeeded to change the baudrate!\n");
        }


        // Enable DYNAMIXEL Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler,*ptr_ID[id-1], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            printf(" ID %d %s\n",id, packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0) {
            printf(" ERROR ID %d %s\n",id, packetHandler->getRxPacketError(dxl_error));
        }
        else {
            printf("Succeeded enabling DYNAMIXEL Torque.\n");
        }
        

        float target_angle = velocity *( MAXIMUM_POSITION_LIMIT/360);

        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, *ptr_ID[id-1], ADDR_GOAL_POSITION, target_angle, &dxl_error);
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, *ptr_ID[id-1], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
        ROS_INFO("join position %d", dxl_present_position);

    
    }
   
}

