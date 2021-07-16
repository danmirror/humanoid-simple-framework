/*
 des  : rscuad manager 
 year : 2021
 
*/

// author : danu andrean


#include "rscuad_manager/rscuad_manager.h"
#include "rscuad_manager/dxl_header.h"


int rscuad::rscuad_manager::manager_init()
{
    ROS_INFO("manager init loaded ..");
     RobotisController *controller =  RobotisController::getInstance();

    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    if (portHandler->setBaudRate(BAUDRATE)) {
    printf("Succeeded to change the baudrate!\n");
    }
   
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    // controller->addSensorModule((SensorModule*) OpenCRModule::getInstance());

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

    usleep(100 * 1000);

    // set RGB-LED to GREEN
    int led_full_unit = 0x1F;
    int led_range = 5;
    int led_value = led_full_unit << led_range;
    int _return = packetHandler->write2ByteTxRx(portHandler, SUB_CONTROLLER_ID, RGB_LED_CTRL_TABLE, led_value);

    if(_return != 0)
      ROS_ERROR("Fail to control LED [%s]", packetHandler->getRxPacketError(_return));
    // joint rscuad_joint;

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


int rscuad::rscuad_manager::dxl_process(){

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  int dxl_goal_position[2] = {MINIMUM_POSITION_LIMIT, MAXIMUM_POSITION_LIMIT};         // Goal position

  uint8_t dxl_error = 0;                          // DYNAMIXEL error
  #if defined(XL320)
  int16_t dxl_present_position = 0;  // XL-320 uses 2 byte Position data
  #else
  int32_t dxl_present_position = 0;  // Read 4 byte Position data
  #endif

  // Open port
  if (portHandler->openPort()) {
    printf("Succeeded to open the port!\n");
  }
  else {
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE)) {
    printf("Succeeded to change the baudrate!\n");
  }
  else {
    return 0;
  }

  // Enable DYNAMIXEL Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0) {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else {
    printf("Succeeded enabling DYNAMIXEL Torque.\n");
  }

    while(1) {

        // Write goal position
        #if defined(XL320)  // XL-320 uses 2 byte Position data
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position[index], &dxl_error);
        #else
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position[index], &dxl_error);
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
            dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, (uint16_t*)&dxl_present_position, &dxl_error);
            #else
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
            #endif
            if (dxl_comm_result != COMM_SUCCESS) {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0) {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            }

            printf("[ID:%03d] Goal Position:%03d  Present Position:%03d\n", DXL_ID, dxl_goal_position[index], dxl_present_position);

        } while((abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

        // Switch the Goal Position
        if (index == 0) {
            index = 1;
        }
        else {
            index = 0;
        }
    }

  // Disable DYNAMIXEL Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
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


int rscuad::rscuad_manager::move_robot(char *str){
    
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
               
    //data result
    ROS_INFO("data masuk: %s", str);
    ROS_ERROR("0 : %.4f",atof(joint_0));
    ROS_ERROR("1 : %.4f",atof(joint_1));
    ROS_ERROR("2 : %.4f",atof(joint_2));
    ROS_ERROR("3 : %.4f",atof(joint_3));
    ROS_ERROR("4 : %.4f",atof(joint_4));
    ROS_ERROR("5 : %.4f",atof(joint_5));
    ROS_ERROR("6 : %.4f",atof(joint_6));
    ROS_ERROR("7 : %.4f",atof(joint_7));
    ROS_ERROR("8 : %.4f",atof(joint_8));
    ROS_ERROR("9 : %.4f",atof(joint_9));
    ROS_ERROR("10 : %.4f",atof(joint_10));
    ROS_ERROR("11 : %.4f",atof(joint_11));
    ROS_ERROR("12 : %.4f",atof(joint_12));
    ROS_ERROR("13 : %.4f",atof(joint_13));

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
        if(str[i] == ','){
            count ++;
            memory = i+1;
        }

        else {
            if (count == 0)
                joint[i] = str[i]; 
            if (count == 1)
                value[i-memory] = str[i]; 
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

