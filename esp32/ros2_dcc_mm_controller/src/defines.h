#ifndef RAILTRACk_DEFINES
#define RAILTRACk_DEFINES

#include "ros_messages.h"
#include "micro_ros_includes.h"

#define SPEED_STEP_RESOLUTION_128     ((int)(1000/128))
#define SPEED_STEP_RESOLUTION_28      ((int)(1000/28))
#define SPEED_STEP_RESOLUTION_14      ((int)(1000/14))

//#define MAX_NUMBER_OF_MM_FUNCTIONS     5

// pin defintions for display
#define CS_PIN  16
#define DC_PIN  21//17
#define RST_PIN 17//21

typedef enum{
    ROS = railway_interfaces__msg__TrackProtocolDefine__PROTOCOL_ROS, 
    MM1 = railway_interfaces__msg__TrackProtocolDefine__PROTOCOL_MM1, 
    MM2 = railway_interfaces__msg__TrackProtocolDefine__PROTOCOL_MM2, 
    DCC = railway_interfaces__msg__TrackProtocolDefine__PROTOCOL_DCC, 
    MFX = railway_interfaces__msg__TrackProtocolDefine__PROTOCOL_MFX
}PROTOCOL;

typedef enum{
    SS_128, SS_28, SS_14, SS_INVALID, SS_NOT_USED
}DCC_SPEED_STEPS;

#define MAX_NUMBER_OF_FUNCTION      32

#define LED_RED     0
#define LED_GREEN   2
#define LED_BLUE    4

#ifndef LED_BUILTIN
#define LED_BUILTIN LED_RED
#endif

typedef struct{
    unsigned int address;
    PROTOCOL protocol;
    DCC_SPEED_STEPS speed_steps;
}LOCOMOTIVE;


typedef struct{
    unsigned int address;
    PROTOCOL protocol;
}TRACK_OBJECT;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#endif