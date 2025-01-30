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


#define MAX_NUMBER_OF_FUNCTION      32

#define EEPROM_PROGRAMMED_TAG     0xAA55

#define LED_RED     0
#define LED_GREEN   2
#define LED_BLUE    4

#ifndef LED_BUILTIN
#define LED_BUILTIN LED_RED
#endif

typedef struct{
    uint32_t address;
    uint8_t protocol;
    uint8_t speed_steps;
}LOCOMOTIVE;


typedef struct{
    uint32_t address;
    uint8_t protocol;
    uint8_t dcc_loc_speedsteps;
}TRACK_OBJECT;


#define RCCHECK(fn)                                                                                \
    {                                                                                              \
        rcl_ret_t temp_rc = fn;                                                                    \
        if ((temp_rc != RCL_RET_OK))                                                               \
        {                                                                                          \
            Serial.printf("%s: Failed ros_client status on line %d: %d, Stopping!\n", __FILE__,  __LINE__, (int)temp_rc); \
            error_loop();                                                                        \
        }                                                                                          \
    }

#define RCSOFTCHECK(fn)                                                                             \
    {                                                                                               \
        rcl_ret_t temp_rc = fn;                                                                     \
        if ((temp_rc != RCL_RET_OK))                                                                \
        {                                                                                           \
            Serial.printf("%s: Failed ros_client status on line %d: %d, Continuing\n", __FILE__, __LINE__, (int)temp_rc); \
        }                                                                                           \
    }


#endif