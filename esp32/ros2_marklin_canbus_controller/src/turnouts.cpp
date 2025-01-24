#include <Arduino.h>
#include <EEPROM.h>
#include <support.h>
#include "turnouts.h"
#include <TrackController.h>

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

extern rcl_publisher_t turnout_status_publisher;
extern int number_of_active_mm_turnouts;
extern railway_interfaces__msg__PowerState power_status;


extern uint16_t *number_of_active_turnouts;
extern TRACK_OBJECT *p_turnouts;
extern bool *p_turnout_status;

extern railway_interfaces__msg__TurnoutState *turnout_status_msgs;

int turnout_state_index = 0;

void turnout_state_publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL && *number_of_active_turnouts) {
    RCSOFTCHECK(rcl_publish(&turnout_status_publisher, &turnout_status_msgs[turnout_state_index], NULL));
    turnout_state_index++;
    if(turnout_state_index == *number_of_active_turnouts) turnout_state_index = 0;
  }
}

void turnout_control_callback(const void * msgin)
{  
  const railway_interfaces__msg__TurnoutControl * control = (const railway_interfaces__msg__TurnoutControl *)msgin;
  int index;
  boolean straight = control->state ? true : false;
  // update controller always !!!
  if(power_status.state){
    ctrl->setTurnout(TURNOUT_BASE_ADDRESS + control->number - 1, straight);
    if(lookupTurnoutIndex(control->number, &index)){
      p_turnout_status[index] = straight;
      EEPROM.commit();
      turnout_status_msgs[index].state = straight;
    }
    tft_printf(ST77XX_GREEN, "ROS msg\nTurnout\nNumber: %i\nSet: %s\n",
            control->number, straight ? "Green" : "Red");
  }

}
