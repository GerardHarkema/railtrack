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

extern railway_interfaces__msg__LocomotiveState locomotive_status[];
extern LOCOMOTIVE active_locomotives[];
extern unsigned short int active_turnouts_mm[];
extern unsigned short int active_turnouts_ros[];

extern int number_of_active_mm_turnouts;
extern int number_of_active_locomotives;

void lookupLocomotiveProtocol(PROTOCOL protocol, char *protocol_txt){
  switch(protocol){
    case ROS:
      strcpy(protocol_txt, "ROS");
      break;
    case MM1:
      strcpy(protocol_txt, "MM1");
      break;    
    case MM2:
      strcpy(protocol_txt, "MM2");
      break;    
    case DCC:
      strcpy(protocol_txt, "DCC");
      break;
    case MFX:
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
  for(i = 0; i < number_of_active_mm_turnouts; i++){
    if(active_turnouts_mm[i] == turnout_number) break;
  }
  if(i >= number_of_active_mm_turnouts) return false;
  *turnout_index = i;
  return true;
}

bool lookupLocomotiveIndex(int locomotive_address, PROTOCOL protocol, int *locomotive_index){
  int i;

  //Serial.printf("locomotive_address = %i\n", locomotive_address);
  //Serial.printf("number_of_active_locomotives = %i\n", number_of_active_locomotives);

  for(i = 0; i < number_of_active_locomotives; i++){
    if((locomotive_status[i].address == locomotive_address) 
      && (locomotive_status[i].protocol == protocol)) break;
  }
  if(i >= number_of_active_locomotives) return false;
  *locomotive_index = i;

  return true;
}
