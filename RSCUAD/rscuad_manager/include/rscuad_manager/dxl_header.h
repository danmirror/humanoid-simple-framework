/*
 * desc : rscuad dxl 
 * year : 2021
 * dev  : danu andrean
 *
 */

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk/dynamixel_sdk.h"  // Uses DYNAMIXEL SDK library

/********* DYNAMIXEL Model definition *********
***** (Use only one definition at a time) *****/
#define X_SERIES // X330, X430, X540, 2X430
// #define PRO_SERIES // H54, H42, M54, M42, L54, L42
// #define PRO_A_SERIES // PRO series with (A) firmware update.
// #define P_SERIES  // PH54, PH42, PM54
// #define XL320  // [WARNING] Operating Voltage : 7.4V
// #define MX_SERIES // MX series with 2.0 firmware update.

// Control table address
#if defined(X_SERIES) || defined(MX_SERIES)
  #define ADDR_TORQUE_ENABLE          64
  #define ADDR_GOAL_POSITION          116
  #define ADDR_PRESENT_POSITION       132
  #define MINIMUM_POSITION_LIMIT      0  // Refer to the Minimum Position Limit of product eManual
  #define MAXIMUM_POSITION_LIMIT      4095  // Refer to the Maximum Position Limit of product eManual
  #define BAUDRATE                    2000000
#elif defined(PRO_SERIES)
  #define ADDR_TORQUE_ENABLE          562  // Control table address is different in DYNAMIXEL model
  #define ADDR_GOAL_POSITION          596
  #define ADDR_PRESENT_POSITION       611
  #define MINIMUM_POSITION_LIMIT      -150000  // Refer to the Minimum Position Limit of product eManual
  #define MAXIMUM_POSITION_LIMIT      150000  // Refer to the Maximum Position Limit of product eManual
  #define BAUDRATE                    57600
#elif defined(P_SERIES) ||defined(PRO_A_SERIES)
  #define ADDR_TORQUE_ENABLE          512  // Control table address is different in DYNAMIXEL model
  #define ADDR_GOAL_POSITION          564
  #define ADDR_PRESENT_POSITION       580
  #define MINIMUM_POSITION_LIMIT      -150000  // Refer to the Minimum Position Limit of product eManual
  #define MAXIMUM_POSITION_LIMIT      150000  // Refer to the Maximum Position Limit of product eManual
  #define BAUDRATE                    57600
#elif defined(XL320)
  #define ADDR_TORQUE_ENABLE          24
  #define ADDR_GOAL_POSITION          30
  #define ADDR_PRESENT_POSITION       37
  #define MINIMUM_POSITION_LIMIT      0  // Refer to the CW Angle Limit of product eManual
  #define MAXIMUM_POSITION_LIMIT      1023  // Refer to the CCW Angle Limit of product eManual
  #define BAUDRATE                    2000000  // Default Baudrate of XL-320 is 1Mbps
#endif

// DYNAMIXEL Protocol Version (1.0 / 2.0)
// https://emanual.robotis.com/docs/en/dxl/protocol2/
#define PROTOCOL_VERSION  2.0

// Factory default ID of all DYNAMIXEL is 1
int DXL_ID_1  =1;
int DXL_ID_2  =2;
int DXL_ID_3  =3;
int DXL_ID_4  =4;
int DXL_ID_5  =5;
int DXL_ID_6  =6;
int DXL_ID_7  =7;
int DXL_ID_8  =8;
int DXL_ID_9  =9;
int DXL_ID_10  =10;
int DXL_ID_11  =11;
int DXL_ID_12  =12;
int DXL_ID_13  =13;
int DXL_ID_14  =14;
int DXL_ID_15  =15;
int DXL_ID_16  =16;
int DXL_ID_17  =17;
int DXL_ID_18  =18;
int DXL_ID_19  =19;
int DXL_ID_20  =20;




// Use the actual port assigned to the U2D2.
// ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
#define DEVICENAME  "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0
#define DXL_MOVING_STATUS_THRESHOLD     20  // DYNAMIXEL moving status threshold
#define ESC_ASCII_VALUE                 0x1b


//global
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
// end dxl======================================================