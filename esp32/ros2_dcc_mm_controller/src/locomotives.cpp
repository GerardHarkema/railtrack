#include <Arduino.h>
#include "support.h"
#include "locomotives.h"

//#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(fmt, ...) \
    do { \
        Serial.printf("DEBUG: %s:%d:%s(): " fmt, \
                __FILE__, __LINE__, __func__, ##__VA_ARGS__); \
    } while (0)
#else
#define DEBUG_PRINT(fmt, ...) \
    do {} while (0)
#endif

//extern railway_interfaces__msg__LocomotiveState *locomotive_status;
extern rcl_publisher_t locomoitive_status_publisher;
extern int number_of_active_locomotives;
extern LOCOMOTIVE active_locomotives[];
extern railway_interfaces__msg__LocomotiveState locomotive_status[];

extern TrackPacketScheduler trackScheduler;

int locomotive_state_index = 0;

void locomotive_state_publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if ((timer != NULL) && number_of_active_locomotives) {
    RCSOFTCHECK(rcl_publish(&locomoitive_status_publisher, &locomotive_status[locomotive_state_index], NULL));
    locomotive_state_index++;
    if(locomotive_state_index == number_of_active_locomotives) locomotive_state_index = 0;
  }
}


#define MAX_NUMBER_OF_DCC_FUNCTIONS   14
#define MAX_NUMBER_OF_MM_FUNCTIONS   5
void locomotive_control_callback(const void * msgin)
{  

  const railway_interfaces__msg__LocomotiveControl * control = (const railway_interfaces__msg__LocomotiveControl *)msgin;

  int locomotive_index;
  char* direction_txt;
  char protocol_txt[10];
  //uint address;
  uint16_t functions = 0; 

  switch(control->command){
    case railway_interfaces__msg__LocomotiveControl__SET_SPEED:
      if(lookupLocomotiveIndex(control->address, control->protocol, &locomotive_index)){
        DEBUG_PRINT("Found locomotive for setting speed\n");
        int8_t speed;
        switch(active_locomotives[locomotive_index].protocol){
          case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_DCC:
            DEBUG_PRINT("Protocol DCC\n");         
            switch(control->dcc_speed_step){
              case railway_interfaces__msg__LocomotiveControl__DCC_SPEED_STEP_128:
                speed = (uint8_t)(control->speed / SPEED_STEP_RESOLUTION_128);
                if(locomotive_status[locomotive_index].direction ==
                  railway_interfaces__msg__LocomotiveControl__DIRECTION_REVERSE)
                    speed = speed * -1;
                DEBUG_PRINT("DCC: Set Speed 128: %i\n", speed);
                trackScheduler.dccSetSpeed128(control->address, DCC_SHORT_ADDRESS, speed); //This should be in the call backs of the ROS subscribers
                break;
              case railway_interfaces__msg__LocomotiveControl__DCC_SPEED_STEP_28:
                speed = (uint8_t)(control->speed / SPEED_STEP_RESOLUTION_28);
                speed = locomotive_status[locomotive_index].direction ? speed * -1 : speed;
                DEBUG_PRINT("DCC: Set Speed 28: %i\n", speed);
                trackScheduler.dccSetSpeed28(control->address, DCC_SHORT_ADDRESS,speed); //This should be in the call backs of the ROS subscribers
                break;
              case railway_interfaces__msg__LocomotiveControl__DCC_SPEED_STEP_14:
                speed = (uint8_t)(control->speed / SPEED_STEP_RESOLUTION_14);
                speed = locomotive_status[locomotive_index].direction ? speed * -1 : speed;
                DEBUG_PRINT("DCC: Set Speed 14: %i\n", speed);
                trackScheduler.dccSetSpeed14(control->address, DCC_SHORT_ADDRESS,speed); //This should be in the call backs of the ROS subscribers
                break;
              default:
                break;
            }
            break;
          case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM1:
            speed = (uint8_t)(control->speed / SPEED_STEP_RESOLUTION_14);
            if(locomotive_status[locomotive_index].direction ==
              railway_interfaces__msg__LocomotiveControl__DIRECTION_REVERSE)
                speed = speed * -1;
            DEBUG_PRINT("MM1: Set Speed: %i\n", speed);
            trackScheduler.mm1SetSpeed(control->address, speed); //This should be in the call backs of the ROS subscribers
            break;
          case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM2:
            speed = (uint8_t)(control->speed / SPEED_STEP_RESOLUTION_14);
            if(locomotive_status[locomotive_index].direction ==
              railway_interfaces__msg__LocomotiveControl__DIRECTION_REVERSE)
                speed = speed * -1;
            DEBUG_PRINT("MM2: Set Speed: %i\n", speed);
            trackScheduler.mm2SetSpeed(control->address, speed); //This should be in the call backs of the ROS subscribers
            break;
          default:
            DEBUG_PRINT("Unknomwn protocol\n"); 
            break;
        }
        locomotive_status[locomotive_index].speed = control->speed;
      }
      lookupLocomotiveProtocol(control->protocol, protocol_txt);
      tft_printf(ST77XX_GREEN, "ROS msg\nLocomotive\nAddress(%s): %i\nSet speed: %i\n",
            protocol_txt, control->address, control->speed);
      break;
    case railway_interfaces__msg__LocomotiveControl__SET_DIRECTION:

      if(lookupLocomotiveIndex(control->address, control->protocol, &locomotive_index)){
        DEBUG_PRINT("Found locomotive for setting direction\n");
        switch(active_locomotives[locomotive_index].protocol){
          case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_DCC:
            DEBUG_PRINT("Protocol DCC\n");         
            switch(active_locomotives[locomotive_index].speed_steps){
              case SS_128:
                trackScheduler.dccSetSpeed128(control->address, DCC_SHORT_ADDRESS, 0); //This should be in the call backs of the ROS subscribers
                break;
              case SS_28:
                trackScheduler.dccSetSpeed28(control->address, DCC_SHORT_ADDRESS, 0); //This should be in the call backs of the ROS subscribers
                break;
              case SS_14:
                trackScheduler.dccSetSpeed14(control->address, DCC_SHORT_ADDRESS, 0); //This should be in the call backs of the ROS subscribers
                break;
              default:
                break;
            }
            break;
          case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM1:
            trackScheduler.mm1ChangeDir(control->address);
            trackScheduler.mm1SetSpeed(control->address, 0); //This should be in the call backs of the ROS subscribers
            break;
          case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM2:
            trackScheduler.mm2SetSpeed(control->address, 0); //This should be in the call backs of the ROS subscribers
            break;
          default:
            DEBUG_PRINT("Unknomwn protocol\n"); 
            break;
        }
        locomotive_status[locomotive_index].direction = control->direction;
        locomotive_status[locomotive_index].speed = 0;
      }
      direction_txt = getDirectionTxt(control->direction);
      lookupLocomotiveProtocol(control->protocol, protocol_txt);
      tft_printf(ST77XX_GREEN, "ROS msg\nLocomotive\nAddress(%s): %i\nSet dir: %s\n",
            protocol_txt, control->address, direction_txt);      
      break;
    case railway_interfaces__msg__LocomotiveControl__SET_FUNCTION:
      if(lookupLocomotiveIndex(control->address, control->protocol, &locomotive_index)){
        DEBUG_PRINT("Found locomotive for setting function\n");
        switch(active_locomotives[locomotive_index].protocol){
          case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_DCC:
            DEBUG_PRINT("Protocol DCC: Function switch\n"); 
            if(control->function_index > MAX_NUMBER_OF_DCC_FUNCTIONS){
              DEBUG_PRINT("Invalid function\n");
              return;
            }
            locomotive_status[locomotive_index].function_state.data[control->function_index] = control->function_state;
            for(int i = 0; i <= MAX_NUMBER_OF_DCC_FUNCTIONS; i++){
                functions = functions << 1;
                functions |= locomotive_status[locomotive_index].function_state.data[MAX_NUMBER_OF_DCC_FUNCTIONS - i] ? 0x01 : 0x00;
            }
            trackScheduler.dccSetFunctions(control->address, DCC_SHORT_ADDRESS, functions);
            break;
          case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM1:
            DEBUG_PRINT("Protocol MM1: Function switch\n"); 
            if(control->function_index > MAX_NUMBER_OF_MM_FUNCTIONS){
              DEBUG_PRINT("Invalid function\n");
              return;
            }
            locomotive_status[locomotive_index].function_state.data[control->function_index] = control->function_state;
            for(int i = 0; i <= MAX_NUMBER_OF_MM_FUNCTIONS; i++){
                functions = functions << 1;
                functions |= locomotive_status[locomotive_index].function_state.data[MAX_NUMBER_OF_MM_FUNCTIONS - i] ? 0x01 : 0x00;
            }
            trackScheduler.mm1SetFunctions(control->address, functions);
            DEBUG_PRINT("Protocol MM1\n"); 
            break;
          case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM2:
            DEBUG_PRINT("Protocol MM2: Function switch\n"); 
            if(control->function_index > MAX_NUMBER_OF_MM_FUNCTIONS){
              DEBUG_PRINT("Invalid function\n");
              return;
            }
            locomotive_status[locomotive_index].function_state.data[control->function_index] = control->function_state;
            //??????
            for(int i = 0; i <= MAX_NUMBER_OF_MM_FUNCTIONS; i++){
                functions = functions << 1;
                functions |= locomotive_status[locomotive_index].function_state.data[MAX_NUMBER_OF_MM_FUNCTIONS - i] ? 0x01 : 0x00;
            }
            trackScheduler.mm2SetFunctions(control->address, functions);
            DEBUG_PRINT("Protocol MM2\n"); 
            break;
          default:
            DEBUG_PRINT("Unknomwn protocol\n"); 
            break;
        }
      }
      lookupLocomotiveProtocol(control->protocol, protocol_txt);
      tft_printf(ST77XX_GREEN, "ROS msg\nLocomotive\nAddress(%s): %i\nSet Func. %i: %s\n",
            protocol_txt, control->address, control->function_index, control->function_state ? "True" : "False");
      break;
    default:
      DEBUG_PRINT("Invalid command");
      break;
  }

}
