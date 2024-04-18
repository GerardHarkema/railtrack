#include <Arduino.h>
#include "support.h"
#include "locomotives.h"

//extern railway_interfaces__msg__LocomotiveState *locomotive_status;
extern rcl_publisher_t locomoitive_status_publisher;
extern int number_of_active_locomotives;
extern LOCOMOTIVE *active_locomotives;
extern railway_interfaces__msg__LocomotiveState *locomotive_status;

extern DCCPacketScheduler DccPacketScheduler;

int locomotive_state_index = 0;

void locomotive_state_publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL && number_of_active_locomotives) {
    RCSOFTCHECK(rcl_publish(&locomoitive_status_publisher, &locomotive_status[locomotive_state_index], NULL));
    locomotive_state_index++;
    if(locomotive_state_index == number_of_active_locomotives) locomotive_state_index = 0;
  }
}


#define MAX_NUMBER_OF_FUNCTIONS   14
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
      if(lookupLocomotiveIndex(control->address, (PROTOCOL)control->protocol, &locomotive_index)){
        //Serial.printf("Found\n");
        int8_t speed;
        switch(active_locomotives[locomotive_index].protocol){
          case DCC:
            Serial.printf("Protocol DCC\n");         
            switch(active_locomotives[locomotive_index].speed_steps){
              case SS_128:
                speed = (uint8_t)(control->speed / SPEED_STEP_RESOLUTION_128);
                if(locomotive_status[locomotive_index].direction ==
                  railway_interfaces__msg__LocomotiveControl__DIRECTION_REVERSE)
                    speed = speed * -1;
                Serial.printf("Set Speed 128: %i\n", speed);
                DccPacketScheduler.setSpeed128(control->address, DCC_SHORT_ADDRESS, speed); //This should be in the call backs of the ROS subscribers
                break;
              case SS_28:
                speed = (uint8_t)(control->speed / SPEED_STEP_RESOLUTION_28);
                speed = locomotive_status[locomotive_index].direction ? speed * -1 : speed;
                Serial.printf("Speed dcc 28: %i\n", speed);
                DccPacketScheduler.setSpeed28(control->address, DCC_SHORT_ADDRESS,speed); //This should be in the call backs of the ROS subscribers
                break;
              case SS_14:
                speed = (uint8_t)(control->speed / SPEED_STEP_RESOLUTION_14);
                speed = locomotive_status[locomotive_index].direction ? speed * -1 : speed;
                Serial.printf("Speed dcc 14: %i\n", speed);
                DccPacketScheduler.setSpeed14(control->address, DCC_SHORT_ADDRESS,speed); //This should be in the call backs of the ROS subscribers
                break;
              default:
                break;
            }
            break;
          case MM1:
          case MM2:
            Serial.printf("Protocol MM\n"); 
            break;
          default:
            Serial.printf("Unknomwn protocol\n"); 
            break;
        }
        locomotive_status[locomotive_index].speed = control->speed;
      }
      lookupLocomotiveProtocol((PROTOCOL)control->protocol, protocol_txt);
      tft_printf(ST77XX_GREEN, "ROS msg\nLocomotive\nAddress(%s): %i\nSet speed: %i\n",
            protocol_txt, control->address, control->speed);
      break;
    case railway_interfaces__msg__LocomotiveControl__SET_DIRECTION:

      if(lookupLocomotiveIndex(control->address, (PROTOCOL)control->protocol, &locomotive_index)){
        //Serial.printf("Found\n");
        switch(active_locomotives[locomotive_index].protocol){
          case DCC:
            Serial.printf("Protocol DCC\n");         
            switch(active_locomotives[locomotive_index].speed_steps){
              case SS_128:
                DccPacketScheduler.setSpeed128(control->address, DCC_SHORT_ADDRESS, 0); //This should be in the call backs of the ROS subscribers
                break;
              case SS_28:
                DccPacketScheduler.setSpeed28(control->address, DCC_SHORT_ADDRESS, 0); //This should be in the call backs of the ROS subscribers
                break;
              case SS_14:
                DccPacketScheduler.setSpeed14(control->address, DCC_SHORT_ADDRESS, 0); //This should be in the call backs of the ROS subscribers
                break;
              default:
                break;
            }
            locomotive_status[locomotive_index].direction = control->direction;
            locomotive_status[locomotive_index].speed = 0;
            break;
          case MM1:
          case MM2:
            Serial.printf("Protocol MM\n"); 
            break;
          default:
            Serial.printf("Unknomwn protocol\n"); 
            break;
        }
      }
      direction_txt = getDirectionTxt(control->direction);
      lookupLocomotiveProtocol((PROTOCOL)control->protocol, protocol_txt);
      tft_printf(ST77XX_GREEN, "ROS msg\nLocomotive\nAddress(%s): %i\nSet dir: %s\n",
            protocol_txt, control->address, direction_txt);      
      break;
    case railway_interfaces__msg__LocomotiveControl__SET_FUNCTION:
          if(lookupLocomotiveIndex(control->address, (PROTOCOL)control->protocol, &locomotive_index)){
        //Serial.printf("Found\n");
        switch(active_locomotives[locomotive_index].protocol){
          case DCC:
            Serial.printf("Protocol DCC\n"); 
            if(control->function_index > MAX_NUMBER_OF_FUNCTIONS){
              Serial.printf("Invalid function\n");
              return;
            }
            locomotive_status[locomotive_index].function_state.data[control->function_index] = control->function_state;

            for(int i = 0; i <= MAX_NUMBER_OF_FUNCTIONS; i++){
                functions = functions << 1;
                functions |= locomotive_status[locomotive_index].function_state.data[MAX_NUMBER_OF_FUNCTIONS - i] ? 0x01 : 0x00;

            }
            Serial.print(functions, BIN);
            DccPacketScheduler.setFunctions(control->address, DCC_SHORT_ADDRESS, functions);
            break;
          case MM1:
          case MM2:
            Serial.printf("Protocol MM\n"); 
            break;
          default:
            Serial.printf("Unknomwn protocol\n"); 
            break;
        }
      }
      lookupLocomotiveProtocol((PROTOCOL)control->protocol, protocol_txt);
      tft_printf(ST77XX_GREEN, "ROS msg\nLocomotive\nAddress(%s): %i\nSet Func. %i: %s\n",
            protocol_txt, control->address, control->function_index, control->function_state ? "True" : "False");
      break;
      break;
#if 0
      //address = getCANAdress((PROTOCOL)control->protocol, control->address);
      if(lookupLocomotiveIndex(address, (PROTOCOL)control->protocol, &locomotive_index)){
        locomotive_status[locomotive_index].function_state.data[control->function_index] = control->function_state;
      }
      lookupLocomotiveProtocol((PROTOCOL)control->protocol, protocol_txt);
      tft_printf(ST77XX_GREEN, "ROS msg\nLocomotive\nAddress(%s): %i\nSet Func. %i: %s\n",
            protocol_txt, address, control->function_index, control->function_state ? "True" : "False");
      break;
#endif
    default:
      Serial.println("Invalid command");
      break;
  }
}
