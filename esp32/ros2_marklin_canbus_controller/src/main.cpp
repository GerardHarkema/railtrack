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


#include <defines.h>
#include <support.h>

#include "power.h"
#include "locomotives.h"
#include "turnouts.h"
#include "measurements.h"
#include <track_config.h>
#include <wifi_network_config.h>

#include "TrackController.h"

const bool canbus_debug_flag = true;

TrackController *ctrl;

TrackMessage message;

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

railway_interfaces__msg__PowerState power_status;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_timer_t turnout_state_publisher_timer;
rcl_timer_t locomotive_state_publisher_timer;
rcl_timer_t power_state_publisher_timer;

Measurements measurements;

uint16_t *eeprom_programmed;
uint16_t *number_of_active_turnouts;
uint16_t *number_of_active_locomotives;
TRACK_OBJECT *p_turnouts;
TRACK_OBJECT *p_locomtives;
bool *p_turnout_status;
extern railway_interfaces__msg__TurnoutState *turnout_status_msgs;
extern railway_interfaces__msg__LocomotiveState *locomotive_status_msgs;


#define MEASUREMENT_SWITCH_PIN    27
bool display_measurents = false;

bool track_config_enable_flag = false;


void init_ros(){
  allocator = rcl_get_default_allocator();
  //create init_options
  if(rclc_support_init(&support, 0, NULL, &allocator)){
    tft_printf(ST77XX_BLUE, "microROS agent\nnot found\nCheck network\nsettings\n");
    while(true){};
  }

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
  tft->println("Marklin Control");
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
  power_status.controller_type = railway_interfaces__msg__PowerState__CONTROLLER_CAN;
}

bool wifiUp;

void setup() {

  pinMode(MEASUREMENT_SWITCH_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  while (!Serial);
  delay(2000);
  DEBUG_PRINT("Marklin canbus controller started");
#if 1
  DEBUG_PRINT("MOSI: %i\n", MOSI);
  DEBUG_PRINT("MISO: %i\n", MISO);
  DEBUG_PRINT("SCK: %i\n", SCK);
  DEBUG_PRINT("SS: %i\n", SS);  
#endif

  init_display();

  tft->println("RailTrackControl");
  tft_printf(ST77XX_MAGENTA, "Marklin\ncanbus\ncontroller\nstarted\n");

  ctrl = new TrackController(0xdf24, canbus_debug_flag);

  ctrl->begin();

  bool force_network_configure;
  force_network_configure = !digitalRead(MEASUREMENT_SWITCH_PIN);

  NETWORK_CONFIG networkConfig;
  wifiUp = configureNetwork(force_network_configure, &networkConfig);
  if(!wifiUp) return;

  init_eeprom();

  if(!track_config_enable_flag){
    init_power();

    init_turnouts();

    init_locomotives();
  }


  WiFi.setHostname("RailTrackController");
  set_microros_wifi_transports(const_cast<char*>(networkConfig.ssid.c_str()), 
                               const_cast<char*>(networkConfig.password.c_str()), 
                               networkConfig.microros_agent_ip_address,
                               networkConfig.microros_agent_port);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  tft_printf(ST77XX_MAGENTA, "Marklin\ncanbus\nWiFi\nConnected\n");
  DEBUG_PRINT("Marklin canbus WiFi Connected\n");
  delay(2000);

  init_ros();
  if(track_config_enable_flag){
    tft_printf(ST77XX_MAGENTA, "Controller\nReady\nto receive\nConfiguration");

  }
  else{
    Serial.println("!!! Ready for operating !!!");
    tft_printf(ST77XX_MAGENTA, "Marklin\ncanbus\ncontroller\nReady\n");
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
        //trackScheduler.trackPower(false);
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
      Serial.println("Toggle");
      Serial.println(display_measurents);
      tft_printf(ST77XX_GREEN,"");
    }
    old_display_measurents_switch = new_display_measurents_switch;
  }

  //Serial.print("*");
  if(ctrl->receiveMessage(message)){


    int address = (message.data[2] << 8) 
                 + message.data[3];
    int index = 0;
    bool straight = 0;
    char protocol_txt[10];
    uint sub_address = 0;
    int function_index = 0;
    int function_enable = 0;
    word speed;
    int turnout_number;
    word position;
    char* direction_txt;
    uint8_t protocol;


    switch(message.command){
      case SYSTEM_BEFEHL:
        switch(message.data[4]){
          case SYSTEM_STOP:
              power_status.state = false;
              tft_printf(ST77XX_GREEN, "CANBUS msg\nSystem: Stop");
            break;
          case SYSTEM_GO:
              power_status.state = true;
              tft_printf(ST77XX_GREEN, "CANBUS msg\nSystem: Go");
            break;
          case UBERLAST:
            switch(message.data[5]){
              case STATUS_CHANNEL_CURRENT:
                power_status.current_overload = true;
                tft_printf(ST77XX_BLUE, "Current\nOverload");
                break;
              case STATUS_CHANNEL_VOLTAGE:
                power_status.voltage_overload = true;
                tft_printf(ST77XX_BLUE, "Voltage\nOverload");
                break;
              case STATUS_CHANNEL_TEMPERATURE:
                power_status.temperature_overload = true;
                tft_printf(ST77XX_BLUE, "Temperature\nOverload");
                break;            }
          default:
            break;
        }
        break;
      case ZUBEHOR_SCHALTEN:
          position = message.data[4];

          turnout_number = address - TURNOUT_BASE_ADDRESS + 1;

          straight = position ? true : false;

          if(lookupTurnoutIndex(turnout_number, &index)){
            turnout_status_msgs[index].state = straight;
            p_turnout_status[index] = straight;
            EEPROM.commit();          
          } 
#if 0
          if(message.data[5] && message.response){
          // Testen op M-Track turnout!!!!
            for(int i = 0; i < NUMBER_OF_ACTIVE_TURNOUTS_ROS; i++){
              if(active_turnouts_ros[i] == turnout_number){
                railway_interfaces__msg__TurnoutControl msg;
                msg.number = turnout_number;
                msg.state = straight;
                RCSOFTCHECK(rcl_publish(&turnout_control_publisher, &msg, NULL));
              }
            }
          }
#endif
          tft_printf(ST77XX_GREEN, "CANBUS msg\nTurnout\nNumber: %i\nSet: %s\n",
            turnout_number, straight ? "Green" : "Red");

        break;
      case LOC_GESCHWINDIGHEID:
          speed = (message.data[4] << 8) 
                 + message.data[5];
          if(getProtocolFromAddress(address, &sub_address, &protocol)){
            lookupLocomotiveProtocol(protocol, protocol_txt);
            if(lookupLocomotiveIndex(sub_address, protocol, &index)){
              locomotive_status_msgs[index].speed = speed;
            }
            tft_printf(ST77XX_GREEN, "CANBUS msg\nLocomotive\nAddress(%s): %i\nSet speed: %i\n",
              protocol_txt, sub_address, speed);
          }
          break;
      case LOC_RICHTUNG:
#if 0
          if(lookupLocomotiveIndex(address, &index)){
            locomotive_status[index].direction = message.data[4];
            locomotive_status[index].speed = 0;
          }
#endif
          direction_txt = getDirectionTxt(message.data[4]);
          if(getProtocolFromAddress(address, &sub_address, &protocol)){
            lookupLocomotiveProtocol(protocol, protocol_txt);
            if(lookupLocomotiveIndex(sub_address, protocol, &index)){
              locomotive_status_msgs[index].direction = (message.data[4] == 1) ? true : false;
            }
            tft_printf(ST77XX_GREEN, "CANBUS msg\nLocomotive\nAddress(%s): %i\nSet dir: %s\n",
              protocol_txt, sub_address, direction_txt);
          }
          break;
      case LOC_FUNCTION:
          function_index = message.data[4];
          function_enable = message.data[5];

          if(getProtocolFromAddress(address, &sub_address, &protocol)){
            if(lookupLocomotiveIndex(sub_address, protocol, &index)){
              locomotive_status_msgs[index].function_state.data[function_index] = function_enable ? true : false;
            }
            lookupLocomotiveProtocol(protocol, protocol_txt);
            tft_printf(ST77XX_GREEN, "CANBUS msg\nLocomotive\nAddress(%s): %i\nSet Func. %i: %s\n",
            protocol_txt, sub_address, function_index, function_enable ? "True" : "False");
          }
        break;
      default:
        break;    
    }
  }
  if(ctrl->isCanbusError()){
    tft_printf(ST77XX_BLUE, "CANBUS Error\nCheck\nConnection or\nMarklin Railbox");
    //while(true){};
  }

  vTaskDelay(20);

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}