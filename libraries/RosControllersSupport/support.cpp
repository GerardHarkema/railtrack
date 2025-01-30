#include <Arduino.h>

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

#include "defines.h"
#include "support.h"

#include <Adafruit_GFX.h> // Core graphics library
#include <Fonts/FreeSansBold9pt7b.h>
//#include <Fonts/Tiny3x3a2pt7b.h>
#include <Adafruit_ST7735.h> // Hardware-specific library

#include "tft_printf.h"

extern railway_interfaces__msg__LocomotiveState *locomotive_status_msgs;
extern uint16_t *number_of_active_locomotives;
extern railway_interfaces__msg__TurnoutState *turnout_status_msgs;


void lookupLocomotiveProtocol(uint8_t protocol, char *protocol_txt){
  switch(protocol){
    case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_ROS:
      strcpy(protocol_txt, "ROS");
      break;
    case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM1:
      strcpy(protocol_txt, "MM1");
      break;    
    case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM2:
      strcpy(protocol_txt, "MM2");
      break;    
    case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_DCC:
      strcpy(protocol_txt, "DCC");
      break;
    case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MFX:
      strcpy(protocol_txt, "MFX");
      break;
    default:
      strcpy(protocol_txt, "Invalid");
  }
}

void error_loop(){
  Serial.println("Error: System halted");
  tft_printf(ST77XX_BLUE, "Error\nSystem halted");

  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

char* getDirectionTxt(int direction){
  switch(direction){
    case railway_interfaces__msg__LocomotiveState__DIRECTION_FORWARD:
      return "Forward";
      break;
    case railway_interfaces__msg__LocomotiveState__DIRECTION_REVERSE:
      return "Reverse";
      break;
    default:
      return "Invalid Code";
      break;
  }
}

bool lookupTurnoutIndex(int turnout_number, int *turnout_index){
  int i;
  for(i = 0; i < *number_of_active_locomotives; i++){
    if(turnout_status_msgs[i].number == turnout_number) break;
  }
  if(i >= *number_of_active_locomotives) return false;
  *turnout_index = i;
  return true;
}

bool lookupLocomotiveIndex(int locomotive_address, uint8_t protocol, int *locomotive_index){
  int i;

  for(i = 0; i < *number_of_active_locomotives; i++){
    if((locomotive_status_msgs[i].address == locomotive_address) 
      && (locomotive_status_msgs[i].protocol == protocol)) break;
  }
  if(i >= *number_of_active_locomotives) return false;
  *locomotive_index = i;

  return true;
}
