#include <Arduino.h>
#include <EEPROM.h>
#include <stdio.h>

#define DEBUG
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

#include <defines.h>
#include <support.h>

#include "power.h"
#include "locomotives.h"
#include "turnouts.h"
#include "measurements.h"
#include <track_config.h>
#include "network_config_old.h"
#include "network_config.h"


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


railway_interfaces__msg__PowerState power_status;


rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_timer_t turnout_state_publisher_timer;
rcl_timer_t locomotive_state_publisher_timer;
rcl_timer_t power_state_publisher_timer;

TrackPacketScheduler trackScheduler;

Measurements measurements;

uint16_t *eeprom_programmed;
uint16_t *number_of_active_turnouts;
uint16_t *number_of_active_locomotives;
TRACK_OBJECT *p_turnouts;
TRACK_OBJECT *p_locomtives;
bool *p_turnout_status;


#define MEASUREMENT_SWITCH_PIN    27
bool display_measurents = false;

bool track_config_enable_flag = false;

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

  int size = sizeof(TRACK_OBJECT) * EEPROM_SIZE;

  // create track_config_subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &track_config_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(railway_interfaces, msg, TrackConfig),
    "railtrack/track_config"));

  // allocate dynamic msg memory
  static micro_ros_utilities_memory_conf_t conf = {0};
  //conf.max_string_capacity = 20;
  conf.max_ros2_type_sequence_capacity = size;
  //conf.max_basic_type_sequence_capacity = 10;
  bool success = micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(railway_interfaces, msg, TrackConfig),
      &track_config,
      conf
  );

  if(!success){
    DEBUG_PRINT("Unable to allocate memory for configuration message");
  }

  // create timer,
  #define CYCLE_TIME    500
  // prevent division by zero
  unsigned int timer_timeout = CYCLE_TIME / (*number_of_active_turnouts + 1);
  RCCHECK(rclc_timer_init_default(
    &turnout_state_publisher_timer,
    &support,
    RCL_MS_TO_NS((int)timer_timeout),
    turnout_state_publisher_timer_callback));

  // prevent division by zero
  timer_timeout = CYCLE_TIME / (*number_of_active_locomotives + 1);
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

  if(!track_config_enable_flag){
    RCCHECK(rclc_executor_add_timer(&executor, &turnout_state_publisher_timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &turnout_control_subscriber, &turnout_control, &turnout_control_callback, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_timer(&executor, &locomotive_state_publisher_timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &locomotive_control_subscriber, &locomotive_control, &locomotive_control_callback, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_timer(&executor, &power_state_publisher_timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &power_control_subscriber, &power_control, &power_control_callback, ON_NEW_DATA));
  }
  RCCHECK(rclc_executor_add_subscription(&executor, &track_config_subscriber, &track_config, &track_config_callback, ON_NEW_DATA));

}

void stop_track_timers(){

  rclc_executor_remove_timer(&executor, &turnout_state_publisher_timer);
  rclc_executor_remove_subscription(&executor, &turnout_control_subscriber);

  rclc_executor_remove_timer(&executor, &locomotive_state_publisher_timer);
  rclc_executor_remove_subscription(&executor, &locomotive_control_subscriber);

  rclc_executor_remove_timer(&executor, &power_state_publisher_timer);
  rclc_executor_remove_subscription(&executor, &power_control_subscriber);

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

void init_eeprom(){

  int needed_eeprom_size = 0;

  EEPROM.begin(EEPROM_SIZE);
  // asign variables
  eeprom_programmed = (uint16_t *)EEPROM.getDataPtr();
  
  number_of_active_locomotives = eeprom_programmed + 1;
  number_of_active_turnouts = number_of_active_locomotives + 1;
  // assign array's
  p_locomtives = (TRACK_OBJECT *)(number_of_active_turnouts + 1);


  if(*eeprom_programmed == EEPROM_PROGRAMMED_TAG){
    DEBUG_PRINT("\nEEPROM programmed\n");

    if(!enoughNeededEeprom(*number_of_active_locomotives, *number_of_active_locomotives)){
      DEBUG_PRINT("needed_eeprom_size out of range\n");
    }

    p_turnouts = p_locomtives + *number_of_active_locomotives;
    p_turnout_status = (bool *)(p_turnouts + *number_of_active_turnouts);

    dumpConfiguration();
  }
  else{
      track_config_enable_flag = true;
      p_turnouts = p_locomtives;
      p_turnout_status = (bool *)p_locomtives;
      DEBUG_PRINT("\nEEPROM not programmed\n");
  }
}

void init_power(){
  power_status.state = false;
  power_status.current = 0;
  power_status.operating_mode = railway_interfaces__msg__PowerControl__OPERTING_MODE_NORMAL;
  power_status.controller_type = railway_interfaces__msg__PowerState__CONTROLLER_DCC_MM;

}

bool wifiUp;

void setup() {
  protect_motor_driver_outputs();
  Serial.begin(115200);
  while (!Serial);
  delay(2000);

  DEBUG_PRINT("\nDCC/MM controller started\n");
  DEBUG_PRINT("MOSI: %i\n", MOSI);
  DEBUG_PRINT("MISO: %i\n", MISO);
  DEBUG_PRINT("SCK: %i\n", SCK);
  DEBUG_PRINT("SS: %i\n", SS);  

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
  NETWORK_CONFIG networkConfig;
  wifiUp = configureNetwork(true, &networkConfig);
  if(!wifiUp) return;

  init_eeprom();

  if(!track_config_enable_flag){
    init_power();

    init_turnouts();

    init_locomotives();
  }

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


  DEBUG_PRINT("DCC/MM WiFi Connected\n");

  delay(2000);

  init_ros();

  pinMode(MEASUREMENT_SWITCH_PIN, INPUT_PULLUP);

  if(track_config_enable_flag){
    tft_printf(ST77XX_MAGENTA, "Controller\nReady\nto receive\nConfiguration");

  }
  else{
    measurements.begin();

    DEBUG_PRINT("!!! Ready for operating !!!\n");
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
}

int old_display_measurents_switch = HIGH;

int track_config_enable_cnt = 0;


void loop() {

  if(!wifiUp) return;

  int new_display_measurents_switch = digitalRead(MEASUREMENT_SWITCH_PIN);
  if(!track_config_enable_flag){
    if(new_display_measurents_switch == LOW){
      track_config_enable_cnt++;
      //DEBUG_PRINT("%i\n", track_config_enable_cnt);
      if(track_config_enable_cnt > 30){
        track_config_enable_flag = true;
        display_measurents = false;
        trackScheduler.trackPower(false);
        stop_track_timers();
        tft_printf(ST77XX_MAGENTA, "Ready\nto receive\nnew track\nconfiguration");
      }
    }
    else
      track_config_enable_cnt = 0;
  }
  if(!track_config_enable_flag){
    if((old_display_measurents_switch != new_display_measurents_switch) && (new_display_measurents_switch == LOW)){
      display_measurents = display_measurents ? false : true;
      DEBUG_PRINT("Toggle\n");
      DEBUG_PRINT("display_measurents = %i\n", display_measurents);
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