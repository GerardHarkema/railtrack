#include <Arduino.h>
#include <EEPROM.h>

#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/bool.h>

#include <railway_interfaces/msg/turnout_control.h>
#include <railway_interfaces/msg/turnout_state.h>
#include <railway_interfaces/msg/locomotive_control.h>
#include <railway_interfaces/msg/locomotive_state.h>

#include <Adafruit_GFX.h> // Core graphics library
#include <Fonts/FreeSansBold9pt7b.h>
//#include <Fonts/Tiny3x3a2pt7b.h>
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>

#include "tft_printf.h"

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This application is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

#include "TrackController.h"

const bool DEBUG = true;

TrackController *ctrl;

TrackMessage message;

rcl_publisher_t turnout_status_publisher;
rcl_publisher_t turnout_control_publisher;
rcl_subscription_t turnout_control_subscriber;

rcl_publisher_t locomoitive_status_publisher;
rcl_subscription_t locomotive_control_subscriber;

rcl_publisher_t power_status_publisher;
rcl_subscription_t power_control_subscriber;

rclc_executor_t executor;

railway_interfaces__msg__TurnoutControl turnout_control;
railway_interfaces__msg__LocomotiveControl locomotive_control;
std_msgs__msg__Bool power_control;

// int8_t cs, int8_t dc, int8_t rst
#define CS_PIN  16
#define DC_PIN  17
#define RST_PIN 21
Adafruit_ST7735 *tft;

typedef enum{
    MM1, MM2, DCC, MFX
}PROTOCOL;

typedef struct{
    unsigned int id;
    PROTOCOL protocol;
    unsigned int address;
}LOCOMOTIVE;

#include "track_config.h"

IPAddress agent_ip(ip_address[0], ip_address[1], ip_address[2], ip_address[3]);

railway_interfaces__msg__TurnoutState turnout_status[NUMBER_OF_ACTIVE_TURNOUTS_C] = {0};
railway_interfaces__msg__LocomotiveState locomotive_status[NUMBER_OF_ACTIVE_LOCOMOTIVES] = {0};
std_msgs__msg__Bool power_status = {0};


rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_timer_t turnout_state_publisher_timer;
rcl_timer_t locomotive_state_publisher_timer;
rcl_timer_t power_state_publisher_timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  Serial.println("Error: System halted");
  tft_printf(ST77XX_BLUE, "CANBUS\ncontroller\nError\nSystem halted");

  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}
void lookupLocomotiveProtocolAddress(int address, char *protocol, int *sub_address){
  if(address >= ADDR_MM2 && address < ADDR_SX1){
    strcpy(protocol, "MM");
    
    *sub_address = address;
  }
  else if (address >= ADDR_MFX && address < (ADDR_MFX + 0x0fff))
  {
    strcpy(protocol, "MFX");
    *sub_address = address - ADDR_MFX;
  }
  else if (address >= ADDR_DCC && address < (ADDR_DCC + 0x0fff))
  {
    strcpy(protocol, "DCC");
    *sub_address = address - ADDR_DCC;
  }
  else {
     strcpy(protocol, "???");
    *sub_address = 0;
  }
}

void getDirectionTxt(int direction, char*direction_txt){
  switch(direction){
    case railway_interfaces__msg__LocomotiveState__DIRECTION_FORWARD:
      strcpy(direction_txt, "Forward");
      break;
    case railway_interfaces__msg__LocomotiveState__DIRECTION_REVERSE:
      strcpy(direction_txt, "Reverse");
      break;
    default:
      strcpy(direction_txt, "Invalid Code");
      break;
  }
}

bool lookupTurnoutIndex(int turnout_number, int *turnout_index){
  int i;
  for(i = 0; i < NUMBER_OF_ACTIVE_TURNOUTS_C; i++){
    if(active_turnouts_c[i] == turnout_number) break;
  }
  if(i == NUMBER_OF_ACTIVE_TURNOUTS_C) return false;
  *turnout_index = i;
  return true;
}

bool lookupLocomotiveIndex(int locomotive_address, int *locomotive_index){
  int i;
  for(i = 0; i < NUMBER_OF_ACTIVE_LOCOMOTIVES; i++){
    if(locomotive_status[i].address == locomotive_address) break;
  }
  if(i == NUMBER_OF_ACTIVE_LOCOMOTIVES) return false;
  *locomotive_index = i;
  return true;
}

int turnout_state_index = 0;

void turnout_state_publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&turnout_status_publisher, &turnout_status[turnout_state_index], NULL));
    turnout_state_index++;
    if(turnout_state_index == NUMBER_OF_ACTIVE_TURNOUTS_C) turnout_state_index = 0;
  }
}


int locomotive_state_index = 0;

void locomotive_state_publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&locomoitive_status_publisher, &locomotive_status[locomotive_state_index], NULL));
    locomotive_state_index++;
    if(locomotive_state_index == NUMBER_OF_ACTIVE_LOCOMOTIVES) locomotive_state_index = 0;
  }
}

void power_state_publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&power_status_publisher, &power_status, NULL));
  }

}

void turnout_control_callback(const void * msgin)
{  
  const railway_interfaces__msg__TurnoutControl * control = (const railway_interfaces__msg__TurnoutControl *)msgin;
  int index;
  boolean straight = control->state ? true : false;
  // update controller always !!!
  if(power_status.data){
    ctrl->setTurnout(TURNOUT_BASE_ADDRESS + control->number - 1, straight);
    if(lookupTurnoutIndex(control->number, &index)){
      EEPROM.writeBool(index, straight);
      EEPROM.commit();
      turnout_status[index].state = straight;
    }
    tft_printf(ST77XX_GREEN, "ROS msg\nTurnout\nNumber: %i\nSet: %s\n",
            control->number, straight ? "Green" : "Red");
  }

}

void locomotive_control_callback(const void * msgin)
{  
  const railway_interfaces__msg__LocomotiveControl * control = (const railway_interfaces__msg__LocomotiveControl *)msgin;

  int locomotive_index;
  char direction_txt[10];
  char protocol_txt[10];
  int sub_address;

  switch(control->command){
    case railway_interfaces__msg__LocomotiveControl__SET_SPEED:
      ctrl->setLocoSpeed(control->address, control->speed);
      //Serial.printf("Address: %i\n", control->address);
      //Serial.printf("Speed: %i\n", control->speed);

      if(lookupLocomotiveIndex(control->address, &locomotive_index)){
        //Serial.printf("Found\n");
        locomotive_status[locomotive_index].speed = control->speed;

      }
      lookupLocomotiveProtocolAddress(control->address, protocol_txt, &sub_address);
      tft_printf(ST77XX_GREEN, "ROS msg\nLocomotive\nAddress(%s): %i\nSet speed: %i\n",
            protocol_txt, sub_address, control->speed);
      break;
    case railway_interfaces__msg__LocomotiveControl__SET_DIRECTION:
      ctrl->setLocoDirection(control->address, control->direction);
      if(lookupLocomotiveIndex(control->address, &locomotive_index)){
        locomotive_status[locomotive_index].direction = control->direction;
      }
      getDirectionTxt(control->direction, direction_txt);
      lookupLocomotiveProtocolAddress(control->address, protocol_txt, &sub_address);      
      tft_printf(ST77XX_GREEN, "ROS msg\nLocomotive\nAddress(%s): %i\nSet dir: %s\n",
            protocol_txt, sub_address, direction_txt);      
      break;
    case railway_interfaces__msg__LocomotiveControl__SET_FUNCTION:
      ctrl->setLocoFunction(control->address, control->function_index, control->function_state);
      if(lookupLocomotiveIndex(control->address, &locomotive_index)){
        locomotive_status[locomotive_index].function_state.data[control->function_index] = control->function_state;
      }
      lookupLocomotiveProtocolAddress(control->address, protocol_txt, &sub_address);      
      tft_printf(ST77XX_GREEN, "ROS msg\nLocomotive\nAddress(%s): %i\nSet Func. %i: %s\n",
            protocol_txt, sub_address, control->function_index, control->function_state ? "True" : "False");
      break;
    default:
      Serial.println("Invalid command");
  }
}

void power_control_callback(const void * msgin)
{  
  const std_msgs__msg__Bool * control = (const std_msgs__msg__Bool *)msgin;
  ctrl->setPower(control->data);
  power_status.data = control->data;
  tft_printf(ST77XX_GREEN, "ROS msg\nSystem: %s", power_status.data ? "Go" : "Halt");

}


void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(2000);
  Serial.println("Marklin canbus controller started");
#if 1
  Serial.print("MOSI: ");Serial.println(MOSI);
  Serial.print("MISO: ");Serial.println(MISO);
  Serial.print("SCK: ");Serial.println(SCK);
  Serial.print("SS: ");Serial.println(SS);  
#endif

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
  tft->println("RailTrackControl");
  tft_printf(ST77XX_MAGENTA, "Marklin\ncanbus\ncontroller\nstarted\n");

  ctrl = new TrackController(0xdf24, DEBUG);
  if(DEBUG){  
    Serial.println();
    Serial.println();
    Serial.println("DIR HASH R CMND LNGT DAT0 DAT1 DAT2 DAT3 DAT4 DAT5 DAT6 DAT7");
    Serial.println("--- ---- - ---- ---- ---- ---- ---- ---- ---- ---- ---- ----");
  }
  ctrl->begin();

  EEPROM.begin(NUMBER_OF_ACTIVE_TURNOUTS_C);

  power_status.data = false;

  for(int i = 0; i < NUMBER_OF_ACTIVE_TURNOUTS_C; i++){
    turnout_status[i].number = active_turnouts_c[i];
    turnout_status[i].state = EEPROM.readBool(i);
    //ctrl->getTurnout(turnout_status[i].number, &turnout_status[i].state);
  }

  for(int i = 0; i < NUMBER_OF_ACTIVE_LOCOMOTIVES; i++){
    locomotive_status[i].direction = railway_interfaces__msg__LocomotiveState__DIRECTION_FORWARD;
    switch(active_locomotives[i].protocol){
      case MM1:
      case MM2:
        locomotive_status[i].address = active_locomotives[i].id;     
        break;
      case DCC:
        locomotive_status[i].address = active_locomotives[i].id + ADDR_DCC;     
      break;
      case MFX:
        locomotive_status[i].address = active_locomotives[i].id + ADDR_MFX;    
      break;

    }
    word speed;
    ctrl->getLocoSpeed(locomotive_status[i].address, &speed);
    locomotive_status[i].speed = speed;
    byte direction;
    ctrl->getLocoDirection(locomotive_status[i].address, &direction);
    locomotive_status[i].direction = direction;


    locomotive_status[i].function_state.capacity = 32;
    locomotive_status[i].function_state.data = (bool*) malloc(locomotive_status[i].function_state.capacity * sizeof(bool));
    locomotive_status[i].function_state.size = 32;

    for(int j = 0; j < 32; j++){
      byte power;
      ctrl->getLocoFunction(locomotive_status[i].address, j, &power);
      locomotive_status[i].function_state.data[j] = power ? true : false;
    }
  }
  set_microros_wifi_transports(SSID, PASSWORD, agent_ip, (size_t)PORT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "railtrack_canbus_controller", "", &support));

  char topic_name[40];
  sprintf(topic_name, "railtrack/turnout/status");
  // create turnout_status_publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &turnout_status_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(railway_interfaces, msg, TurnoutState),
    topic_name));


  // create turnout_control_subscriber
  sprintf(topic_name, "railtrack/turnout/control");
  // create turnout_status_publisher

  RCCHECK(rclc_publisher_init_default(
    &turnout_control_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(railway_interfaces, msg, TurnoutControl),
    topic_name));
  RCCHECK(rclc_subscription_init_default(
    &turnout_control_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(railway_interfaces, msg, TurnoutControl),
    topic_name));

  sprintf(topic_name, "railtrack/locomotive/status");
  // create locomotive_status_publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &locomoitive_status_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(railway_interfaces, msg, LocomotiveState),
    topic_name));

  sprintf(topic_name, "railtrack/locomotive/control");
  // create locomotive_control_subscriber
  RCCHECK(rclc_subscription_init_default(
    &locomotive_control_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(railway_interfaces, msg, LocomotiveControl),
    topic_name));

  sprintf(topic_name, "railtrack/power_status");
  // create power_status_publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &power_status_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    topic_name));

  sprintf(topic_name, "railtrack/power_control");
  // create power_control_subscriber
  RCCHECK(rclc_subscription_init_default(
    &power_control_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    topic_name));

  // create timer,
#define CYCLE_TIME    500
  unsigned int timer_timeout = CYCLE_TIME / NUMBER_OF_ACTIVE_TURNOUTS_C;
  RCCHECK(rclc_timer_init_default(
    &turnout_state_publisher_timer,
    &support,
    RCL_MS_TO_NS((int)timer_timeout),
    turnout_state_publisher_timer_callback));

  timer_timeout = CYCLE_TIME / NUMBER_OF_ACTIVE_LOCOMOTIVES;
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
  int number_of_executors = 6;
  
  RCCHECK(rclc_executor_init(&executor, &support.context, number_of_executors, &allocator));

  RCCHECK(rclc_executor_add_timer(&executor, &turnout_state_publisher_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &turnout_control_subscriber, &turnout_control, &turnout_control_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_timer(&executor, &locomotive_state_publisher_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &locomotive_control_subscriber, &locomotive_control, &locomotive_control_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_timer(&executor, &power_state_publisher_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &power_control_subscriber, &power_control, &power_control_callback, ON_NEW_DATA));

  Serial.println("!!! Ready for operating !!!");
  tft_printf(ST77XX_MAGENTA, "Marklin\ncanbus\ncontroller\nReady\n");
}


void loop() {

  //Serial.print("*");
  if(ctrl->receiveMessage(message)){

    int address = (message.data[2] << 8) 
                 + message.data[3];
    int index;
    bool straight;
    char protocol_txt[10];
    int sub_address;
    char direction_txt[10];

    switch(message.command){
      case SYSTEM_BEFEHL:
        switch(message.data[4]){
          case SYSTEM_STOP:
              power_status.data = false;
               tft_printf(ST77XX_GREEN, "CANBUS msg\nSystem: Stop");
            break;
          case SYSTEM_GO:
              power_status.data = true;
               tft_printf(ST77XX_GREEN, "CANBUS msg\nSystem: Go");
            break;
          default:
            break;
        }
        break;
      case ZUBEHOR_SCHALTEN:
          word position;

          position = message.data[4];
          int turnout_number;
          turnout_number = address - TURNOUT_BASE_ADDRESS + 1;

          straight = position ? true : false;

          if(lookupTurnoutIndex(turnout_number, &index)){
            turnout_status[index].state = straight;
            EEPROM.writeBool(index, straight);
            EEPROM.commit();          
          } 
          if(message.data[5] && message.response){
          // Testen op M-Track turnout!!!!
            for(int i = 0; i < NUMBER_OF_ACTIVE_TURNOUTS_M; i++){
              if(active_turnouts_m[i] == turnout_number){
                railway_interfaces__msg__TurnoutControl msg;
                msg.number = turnout_number;
                msg.state = straight;
                RCSOFTCHECK(rcl_publish(&turnout_control_publisher, &msg, NULL));
              }
            }
          }
          tft_printf(ST77XX_GREEN, "CANBUS msg\nTurnout\nNumber: %i\nSet: %s\n",
            turnout_number, straight ? "Green" : "Red");

        break;
      case LOC_GESCHWINDIGHEID:
          word speed;
          speed = (message.data[4] << 8) 
                 + message.data[5];
          if(lookupLocomotiveIndex(address, &index)){
            locomotive_status[index].speed = speed;
          }
          lookupLocomotiveProtocolAddress(address, protocol_txt, &sub_address);
          tft_printf(ST77XX_GREEN, "CANBUS msg\nLocomotive\nAddress(%s): %i\nSet speed: %i\n",
            protocol_txt, sub_address, speed);
          break;
      case LOC_RICHTUNG:
          if(lookupLocomotiveIndex(address, &index)){
            locomotive_status[index].direction = message.data[4];
            locomotive_status[index].speed = 0;
          }
          getDirectionTxt(message.data[4], direction_txt);
          lookupLocomotiveProtocolAddress(address, protocol_txt, &sub_address);
          tft_printf(ST77XX_GREEN, "CANBUS msg\nLocomotive\nAddress(%s): %i\nSet dir: %s\n",
            protocol_txt, sub_address, direction_txt);          
          break;
      case LOC_FUNCTION:
          lookupLocomotiveProtocolAddress(address, protocol_txt, &sub_address);
          int function_index = message.data[4];
          int function_enable = message.data[5];
          tft_printf(ST77XX_GREEN, "CANBUS msg\nLocomotive\nAddress(%s): %i\nSet Func. %i: %s\n",
            protocol_txt, sub_address, function_index, function_enable ? "True" : "False");
          if(lookupLocomotiveIndex(address, &index)){
            locomotive_status[index].function_state.data[function_index] = function_enable ? true : false;
          }  
        break;
///      default:
//        break;    
    }
  }

  vTaskDelay(20);
  //delay(20);

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

}