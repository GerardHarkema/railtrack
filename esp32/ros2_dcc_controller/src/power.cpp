#include <Arduino.h>
#include "power.h"
#include "measurements.h"

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

extern railway_interfaces__msg__PowerState power_status;
extern rcl_publisher_t power_status_publisher;
extern TrackPacketScheduler trackScheduler;

extern railway_interfaces__msg__LocomotiveState locomotive_status[];
extern int number_of_active_locomotives;
extern LOCOMOTIVE active_locomotives[];

extern Measurements measurements;
extern bool display_measurents;

void power_state_publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    power_status.current = measurements.getCurrent();
    power_status.voltage = measurements.getVoltage();
    power_status.temperature = measurements.getTemperature();
    //DEBUG_PRINT("current = %f A\n", power_status.current);
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
  trackScheduler.trackPower(control->enable);
  power_status.state = control->enable;
  if(!power_status.state){
    for(int i = 0 ; i < number_of_active_locomotives; i++){

      switch(active_locomotives[i].protocol){
        case DCC:
          DEBUG_PRINT("Protocol DCC\n");         
          switch(active_locomotives[i].speed_steps){
            case SS_128:
              trackScheduler.dccSetSpeed128(active_locomotives[i].address, DCC_SHORT_ADDRESS, 0); //This should be in the call backs of the ROS subscribers
              break;
            case SS_28:
              trackScheduler.dccSetSpeed28(active_locomotives[i].address, DCC_SHORT_ADDRESS, 0); //This should be in the call backs of the ROS subscribers
              break;
            case SS_14:
              trackScheduler.dccSetSpeed14(active_locomotives[i].address, DCC_SHORT_ADDRESS, 0); //This should be in the call backs of the ROS subscribers
              break;
            default:
              break;
          }
          break;
        case MM1:
          trackScheduler.mm1SetSpeed(active_locomotives[i].address, 0); //This should be in the call backs of the ROS subscribers
          break;
        case MM2:
          trackScheduler.mm2SetSpeed(active_locomotives[i].address, 0); //This should be in the call backs of the ROS subscribers
          break;
        default:
          DEBUG_PRINT("Unknomwn protocol\n"); 
          break;
      }
      locomotive_status[i].speed = 0;
      for(int j = 0; j < MAX_NUMBER_OF_FUNCTION; j++)
        locomotive_status[i].function_state.data[j] = false;
    }
  }
  tft_printf(ST77XX_GREEN, "ROS msg\nSystem: %s", power_status.state ? "Go" : "Stop");
}