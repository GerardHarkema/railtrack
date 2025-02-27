#include <Arduino.h>
#include "power.h"
#include "measurements.h"
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

extern railway_interfaces__msg__PowerState power_status;
extern rcl_publisher_t power_status_publisher;


extern railway_interfaces__msg__LocomotiveState locomotive_status[];
extern LOCOMOTIVE active_locomotives[];
extern uint16_t *number_of_active_locomotives;


extern Measurements measurements;
extern bool display_measurents;

void power_state_publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    power_status.current = ctrl->getCurrent();
    power_status.voltage = ctrl->getVoltage();
    power_status.temperature = ctrl->getTemperature();
    //Serial.printf("current = %f A\n", power_status.current);
    RCSOFTCHECK(rcl_publish(&power_status_publisher, &power_status, NULL));
    power_status.current_overload = false;
    power_status.voltage_overload = false;
    power_status.temperature_overload = false;
    char text[100];
    sprintf(text, "U= %0.1fV\nI= %0.1fA\nT= %0.1f°C", 
      power_status.voltage,
      power_status.current,
      power_status.temperature);
    if(display_measurents) tft_printf(ST77XX_GREEN, text);
  }

}

void power_control_callback(const void * msgin)
{  
  const railway_interfaces__msg__PowerControl * control = (const railway_interfaces__msg__PowerControl *)msgin;
  ctrl->setPower(control->enable);
  power_status.state = control->enable;
  tft_printf(ST77XX_GREEN, "ROS msg\nSystem: %s", power_status.state ? "Go" : "Stop");

}
