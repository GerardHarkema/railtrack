#include <Arduino.h>
#include <support.h>
#include "locomotives.h"
#include "TrackController.h"

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

extern TrackController *ctrl;

//extern railway_interfaces__msg__LocomotiveState *locomotive_status;
extern rcl_publisher_t locomoitive_status_publisher;


extern railway_interfaces__msg__LocomotiveState *locomotive_status_msgs;
extern uint16_t *number_of_active_locomotives;
extern TRACK_OBJECT *p_locomtives;


int locomotive_state_index = 0;

void locomotive_state_publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL && *number_of_active_locomotives) {
    RCSOFTCHECK(rcl_publish(&locomoitive_status_publisher, &locomotive_status_msgs[locomotive_state_index], NULL));
    locomotive_state_index++;
    if(locomotive_state_index == *number_of_active_locomotives) locomotive_state_index = 0;
  }
}


bool getProtocolFromAddress(uint address, uint *subaddress, uint8_t *protocol){

  if(address <= 0x3FF){
    *protocol = railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM1;
    *subaddress = address;
    return true;
  }

  if((address >= ADDR_MFX) && (address <= ADDR_MFX_MAX)){
    *protocol = railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MFX;
    *subaddress = address - ADDR_MFX;
    return true;
  }

  if((address >= ADDR_DCC) && (address <= ADDR_DCC_MAX)){
    *protocol = railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_DCC;
    *subaddress = address - ADDR_DCC;
    return true;
  }
  return false;
}

uint getCANAdress(uint8_t protocol, uint address){
  switch(protocol){
    case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_DCC:
      return(address + ADDR_DCC);
//    break;
    case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MFX:
      return(address + ADDR_MFX);
//    break;
  }
  return address;
}

void locomotive_control_callback(const void * msgin)
{  
  const railway_interfaces__msg__LocomotiveControl * control = (const railway_interfaces__msg__LocomotiveControl *)msgin;

  int locomotive_index;
  char* direction_txt;
  char protocol_txt[10];
  uint address;

  switch(control->command){
    case railway_interfaces__msg__LocomotiveControl__SET_SPEED:
      address = getCANAdress(control->protocol, control->address);
      ctrl->setLocoSpeed(address, control->speed);
      //Serial.printf("Address: %i\n", control->address);
      //Serial.printf("Speed: %i\n", control->speed);

      if(lookupLocomotiveIndex(control->address, control->protocol, &locomotive_index)){
        //Serial.printf("Found\n");
        locomotive_status_msgs[locomotive_index].speed = control->speed;

      }
      lookupLocomotiveProtocol(control->protocol, protocol_txt);
      tft_printf(ST77XX_GREEN, "ROS msg\nLocomotive\nAddress(%s): %i\nSet speed: %i\n",
            protocol_txt, control->address, control->speed);
      break;
    case railway_interfaces__msg__LocomotiveControl__SET_DIRECTION:
      address = getCANAdress(control->protocol, control->address);
      ctrl->setLocoDirection(address, control->direction);
      if(lookupLocomotiveIndex(control->address, control->protocol, &locomotive_index)){
        locomotive_status_msgs[locomotive_index].direction = control->direction;
      }
      direction_txt = getDirectionTxt(control->direction);
      lookupLocomotiveProtocol(control->protocol, protocol_txt);
      tft_printf(ST77XX_GREEN, "ROS msg\nLocomotive\nAddress(%s): %i\nSet dir: %s\n",
            protocol_txt, control->address, direction_txt);      
      break;
    case railway_interfaces__msg__LocomotiveControl__SET_FUNCTION:
      address = getCANAdress(control->protocol, control->address);
      //Serial.printf("address = %i, function_index = %i, state = %i/n", address, control->function_index, control->function_state);
      ctrl->setLocoFunction(address, control->function_index, control->function_state ? 31 : 0); // see 3.6 marklin protocol
      if(lookupLocomotiveIndex(control->address, control->protocol, &locomotive_index)){
        locomotive_status_msgs[locomotive_index].function_state.data[control->function_index] = control->function_state;
      }
      lookupLocomotiveProtocol(control->protocol, protocol_txt);
      tft_printf(ST77XX_GREEN, "ROS msg\nLocomotive\nAddress(%s): %i\nSet Func. %i: %s\n",
            protocol_txt, control->address, control->function_index, control->function_state ? "True" : "False");
      break;
    default:
      Serial.println("Invalid command");
  }
}
