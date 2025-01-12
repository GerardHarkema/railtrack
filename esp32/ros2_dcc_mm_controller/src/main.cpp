#include <Arduino.h>
#include <EEPROM.h>
#include <stdio.h>

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

#if !defined(ESP32)
#error This application is only avaible for ESP32 Dev module
#endif

#include "micro_ros_includes.h"

#include <Adafruit_GFX.h> // Core graphics library
#include <Fonts/FreeSansBold9pt7b.h>
//#include <Fonts/Tiny3x3a2pt7b.h>
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>

#include "tft_printf.h"

#include <TrackPacket.h>
#include <TrackPacketQueueList.h>
#include <TrackPacketScheduler.h>
#include <TrackManager.h>

#include "defines.h"
#include "track_config_old.h"
#include "network_config.h"
#include "support.h"

#include "power.h"
#include "locomotives.h"
#include "turnouts.h"
#include "measurements.h"
#include "track_config.h"

int number_of_active_mm_turnouts = NUMBER_OF_ACTIVE_TURNOUTS_MM;
int number_of_active_turnouts_ros = NUMBER_OF_ACTIVE_TURNOUTS_ROS;
int number_of_active_locomotives = NUMBER_OF_ACTIVE_LOCOMOTIVES;


rcl_publisher_t turnout_status_publisher;
rcl_subscription_t turnout_control_subscriber;

rcl_publisher_t locomoitive_status_publisher;
rcl_subscription_t locomotive_control_subscriber;

rcl_publisher_t power_status_publisher;
rcl_subscription_t power_control_subscriber;

rcl_subscription_t track_config_subscriber;

rclc_executor_t executor;

railway_interfaces__msg__TurnoutControl turnout_control;
railway_interfaces__msg__LocomotiveControl locomotive_control;
railway_interfaces__msg__PowerControl power_control;
railway_interfaces__msg__TrackConfig track_config;

Adafruit_ST7735 *tft;

IPAddress agent_ip(ip_address[0], ip_address[1], ip_address[2], ip_address[3]);

#if NUMBER_OF_ACTIVE_TURNOUTS_MM
railway_interfaces__msg__TurnoutState turnout_status[NUMBER_OF_ACTIVE_TURNOUTS_MM] = {0};
#else
// Dummy pointer to turnout_status if no turnouts are defined
railway_interfaces__msg__TurnoutState *turnout_status;
#endif

#if NUMBER_OF_ACTIVE_LOCOMOTIVES
railway_interfaces__msg__LocomotiveState locomotive_status[NUMBER_OF_ACTIVE_LOCOMOTIVES] = {0};
#else
// Dummy pointer to locomotive_status if no locomotives are defined
railway_interfaces__msg__LocomotiveState *locomotive_status;
#endif
railway_interfaces__msg__PowerState power_status;


rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_timer_t turnout_state_publisher_timer;
rcl_timer_t locomotive_state_publisher_timer;
rcl_timer_t power_state_publisher_timer;

TrackPacketScheduler trackScheduler;

Measurements measurements;

#define MEASUREMENT_SWITCH_PIN    27
bool display_measurents = false;


void init_ros(){
    allocator = rcl_get_default_allocator();
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "railtrack_dcc_mm_controller", "", &support));

  // create turnout_status_publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &turnout_status_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(railway_interfaces, msg, TurnoutState),
    "railtrack/turnout/status"));

  // create turnout_status_publisher
  RCCHECK(rclc_subscription_init_default(
    &turnout_control_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(railway_interfaces, msg, TurnoutControl),
    "railtrack/turnout/control"));

  // create locomotive_status_publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &locomoitive_status_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(railway_interfaces, msg, LocomotiveState),
    "railtrack/locomotive/status"));
  // create locomotive_control_subscriber
  RCCHECK(rclc_subscription_init_default(
    &locomotive_control_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(railway_interfaces, msg, LocomotiveControl),
    "railtrack/locomotive/control"));

  // create power_status_publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &power_status_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(railway_interfaces, msg, PowerState),
    "railtrack/power_status"));
  // create power_control_subscriber
  RCCHECK(rclc_subscription_init_default(
    &power_control_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(railway_interfaces, msg, PowerControl),
    "railtrack/power_control"));

  // create track_config_subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &track_config_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(railway_interfaces, msg, TrackConfig),
    "railtrack/track_config"));


  // create timer,
  #define CYCLE_TIME    500
  // prevent division by zero
  unsigned int timer_timeout = CYCLE_TIME / (NUMBER_OF_ACTIVE_TURNOUTS_MM + 1);
  RCCHECK(rclc_timer_init_default(
    &turnout_state_publisher_timer,
    &support,
    RCL_MS_TO_NS((int)timer_timeout),
    turnout_state_publisher_timer_callback));

  // prevent division by zero
  timer_timeout = CYCLE_TIME / (NUMBER_OF_ACTIVE_LOCOMOTIVES + 1);
  RCCHECK(rclc_timer_init_default(
    &locomotive_state_publisher_timer,
    &support,
    RCL_MS_TO_NS((int)timer_timeout),
    locomotive_state_publisher_timer_callback));

  timer_timeout = CYCLE_TIME;
  RCCHECK(rclc_timer_init_default(
    &power_state_publisher_timer,
    &support,
    RCL_MS_TO_NS((int)timer_timeout),
    power_state_publisher_timer_callback));

  if(!trackScheduler.setup())error_loop();

  // create executor

  int number_of_executors = 7;
  
  RCCHECK(rclc_executor_init(&executor, &support.context, number_of_executors, &allocator));

  RCCHECK(rclc_executor_add_timer(&executor, &turnout_state_publisher_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &turnout_control_subscriber, &turnout_control, &turnout_control_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_timer(&executor, &locomotive_state_publisher_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &locomotive_control_subscriber, &locomotive_control, &locomotive_control_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_timer(&executor, &power_state_publisher_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &power_control_subscriber, &power_control, &power_control_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(&executor, &track_config_subscriber, &track_config, &track_config_callback, ON_NEW_DATA));

}

void init_display(){
  tft = new Adafruit_ST7735(CS_PIN, DC_PIN, RST_PIN);
  tft_prinft_begin(tft);

  tft->initR(INITR_GREENTAB);
  tft->fillScreen(ST77XX_BLACK);
  tft->setRotation(3);
  tft->setFont(&FreeSansBold9pt7b);
  //tft->setFont(&Tiny3x3a2pt7b);
  tft->fillScreen(ST77XX_BLACK);
  tft->setTextColor(ST77XX_CYAN);
  tft->setTextSize(1);
  tft->setCursor(1, 22);
  tft->println("DCC/MM Control");
}

uint16_t *p_eeprom_programmed;
uint16_t *p_number_of_active_turnouts;
uint16_t *p_number_of_active_locomotives;
TRACK_OBJECT *p_turnouts;
TRACK_OBJECT *p_locomtives;
bool *p_turnout_status;

uint16_t t_number_of_turnout = 0x10;
uint16_t t_number_of_locomotive = 0x20;

#define EEPROM_PROGRAMMED_TAG     0xAA55

void init_eeprom(){

  int needed_eeprom_size = 0;

  EEPROM.begin(EEPROM_SIZE);
  // asign variables
  p_eeprom_programmed = (uint16_t *)EEPROM.getDataPtr();
  
  p_number_of_active_turnouts = p_eeprom_programmed + 1;
  *p_number_of_active_turnouts = 0;
  p_number_of_active_locomotives = p_number_of_active_turnouts + 1;
  *p_number_of_active_locomotives = 0;


  if(*p_eeprom_programmed != EEPROM_PROGRAMMED_TAG){
    Serial.printf("\nEEPROM not programmed\n");
  }
  else
    Serial.printf("\nEEPROM programmed\n");

  needed_eeprom_size += sizeof(uint16_t); // eeprom programmed
  needed_eeprom_size += sizeof(uint16_t); // number_of_turnouts
  needed_eeprom_size += sizeof(uint16_t); // number_of_locomotive
  needed_eeprom_size += sizeof(TRACK_OBJECT) * t_number_of_turnout; // needed_eeprom_size of turnout objects
  needed_eeprom_size += sizeof(TRACK_OBJECT) * t_number_of_locomotive; // needed_eeprom_size of locomotive objects
  needed_eeprom_size += sizeof(bool) * t_number_of_turnout; // needed_eeprom_size of turnout_status
  Serial.printf("needed_eeprom_size = %i\n", needed_eeprom_size);

  if(needed_eeprom_size > EEPROM_SIZE){
    Serial.printf("needed_eeprom_size = %i\n", needed_eeprom_size);
  }

  // assign array's
  p_turnouts = (TRACK_OBJECT *)(p_number_of_active_locomotives + 1);
  p_locomtives = p_turnouts + t_number_of_turnout;
  p_turnout_status = (bool *)(p_locomtives + t_number_of_turnout);

  Serial.printf("p_eeprom_programmed            = 0x%08x\n", p_eeprom_programmed);
  Serial.printf("p_number_of_active_turnouts    = 0x%08x\n", p_number_of_active_turnouts);
  Serial.printf("p_number_of_active_locomotives = 0x%08x\n", p_number_of_active_locomotives);
  Serial.printf("p_turnouts                     = 0x%08x\n", p_turnouts);
  Serial.printf("p_locomtives                   = 0x%08x\n", p_locomtives);
  Serial.printf("p_turnout_status               = 0x%08x\n", p_turnout_status);

}

void init_power(){
  power_status.state = false;
  power_status.current = 0;
  power_status.operating_mode = railway_interfaces__msg__PowerControl__OPERTING_MODE_NORMAL;
  power_status.controller_type = railway_interfaces__msg__PowerState__CONTROLLER_DCC_MM;

}

void init_turnouts(){
  for(int i = 0; i < NUMBER_OF_ACTIVE_TURNOUTS_MM; i++){
    turnout_status[i].number = active_turnouts_mm[i];
    turnout_status[i].protocol = MM2;
    turnout_status[i].state = EEPROM.readBool(i);
  }
}

void init_locomotives(){
  for(int i = 0; i < NUMBER_OF_ACTIVE_LOCOMOTIVES; i++){
    locomotive_status[i].protocol = active_locomotives[i].protocol;
    locomotive_status[i].address = active_locomotives[i].address;
    locomotive_status[i].direction = railway_interfaces__msg__LocomotiveState__DIRECTION_FORWARD;
    word speed; //?
    //ctrlgetLocoSpeed(locomotive_status[i].address, &speed);
    locomotive_status[i].speed = speed;
    byte direction; //?
    //ctrlgetLocoDirection(locomotive_status[i].address, &direction);
    locomotive_status[i].direction = direction;


    locomotive_status[i].function_state.capacity = MAX_NUMBER_OF_FUNCTION;
    locomotive_status[i].function_state.data = (bool*) malloc(locomotive_status[i].function_state.capacity * sizeof(bool));
    locomotive_status[i].function_state.size = MAX_NUMBER_OF_FUNCTION;

    for(int j = 0; j < MAX_NUMBER_OF_FUNCTION; j++){
      byte power;
      //ctrlgetLocoFunction(locomotive_status[i].address, j, &power);
      locomotive_status[i].function_state.data[j] = power ? true : false;
    }
  }
}

void setup() {
  protect_motor_driver_outputs();
  Serial.begin(115200);
  while (!Serial);
  delay(2000);
  Serial.printf("\nDCC/MM controller started\n");
#if 0
  Serial.print("MOSI: ");Serial.println(MOSI);
  Serial.print("MISO: ");Serial.println(MISO);
  Serial.print("SCK: ");Serial.println(SCK);
  Serial.print("SS: ");Serial.println(SS);  
#endif

init_display();

#ifdef ARDUINO_MOTOR_SHIELD_L298
  tft_printf(ST77XX_MAGENTA, "Controller\nStarted\n\nL298 Version");
#elif IBT_2_MOTOR_DRIVER
  tft_printf(ST77XX_MAGENTA, "Controller\nStarted\n\nIBT_2 Version");
#elif DCC_EX_MOTOR_SHIELD_8874
  tft_printf(ST77XX_MAGENTA, "Controller\nStarted\n\n8874 Version");
#else
  tft_printf(ST77XX_MAGENTA, "Controller\nStarted\n\nUnknown Version");
#endif

  init_eeprom();

  init_power();

  init_turnouts();

  init_locomotives();

  WiFi.setHostname("DccMMController");
  set_microros_wifi_transports(WIFI_SSID, PASSWORD, agent_ip, (size_t)PORT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

#ifdef ARDUINO_MOTOR_SHIELD_L298
  tft_printf(ST77XX_MAGENTA, "WiFi Connected\n\n\nL298 Version");
#elif IBT_2_MOTOR_DRIVER
  tft_printf(ST77XX_MAGENTA, "WiFi Connected\n\n\nIBT_2 Version");
#elif DCC_EX_MOTOR_SHIELD_8874
  tft_printf(ST77XX_MAGENTA, "WiFi Connected\n\n\n8874 Version");
#else
  tft_printf(ST77XX_MAGENTA, "WiFi Connected\n\n\nUnknown Version");
#endif


  Serial.printf("DCC/MM WiFi Connected\n");

  delay(2000);

  init_ros();

  pinMode(MEASUREMENT_SWITCH_PIN, INPUT_PULLUP);

  measurements.begin();

  Serial.printf("!!! Ready for operating !!!\n");
#ifdef ARDUINO_MOTOR_SHIELD_L298
  tft_printf(ST77XX_MAGENTA, "Controller\nReady\n\nL298 Version");
#elif IBT_2_MOTOR_DRIVER
  tft_printf(ST77XX_MAGENTA, "Controller\nReady\n\nIBT_2 Version");
#elif DCC_EX_MOTOR_SHIELD_8874
  tft_printf(ST77XX_MAGENTA, "Controller\nReady\n\n8874 Version");
#else
  tft_printf(ST77XX_MAGENTA, "Controller\nReady\n\nUnknown Version");
#endif
}

int old_display_measurents_switch = HIGH;

int track_config_enable_cnt = 0;
bool track_config_enable_flag = false;

void loop() {

  int new_display_measurents_switch = digitalRead(MEASUREMENT_SWITCH_PIN);
  if(!track_config_enable_flag){
    if(new_display_measurents_switch == LOW){
      track_config_enable_cnt++;
      if(track_config_enable_cnt > 100){
        track_config_enable_flag = true;
        display_measurents = false;
        tft_printf(ST77XX_MAGENTA, "Ready\nto receive\nnew track\nconfiguration");
      }
    }
    else
      track_config_enable_cnt = 0;
  }
  if(!track_config_enable_flag){
    if((old_display_measurents_switch != new_display_measurents_switch) && (new_display_measurents_switch == LOW)){
      display_measurents = display_measurents ? false : true;
      //Serial.println("Toggle");
      //Serial.println(display_measurents);
      tft_printf(ST77XX_GREEN,"");
    }
    old_display_measurents_switch = new_display_measurents_switch;
  }

  vTaskDelay(20);

  //delay(20);

  // Measure current
  // Measure voltage
  // Measure Temperature

#ifndef THREAD_SAFE_QUEUE
  trackScheduler.update();
#endif

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

}