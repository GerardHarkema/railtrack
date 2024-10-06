#include <Arduino.h>
#include <EEPROM.h>

#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <railway_interfaces/msg/turnout_control.h>
#include <railway_interfaces/msg/turnout_state.h>
#include <railway_interfaces/msg/locomotive_control.h>
#include <railway_interfaces/msg/locomotive_state.h>
#include <railway_interfaces/msg/power_control.h>
#include <railway_interfaces/msg/power_state.h>

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
railway_interfaces__msg__PowerControl power_control;

// int8_t cs, int8_t dc, int8_t rst
#define CS_PIN  16
#define DC_PIN  21//17
#define RST_PIN 17//21
Adafruit_ST7735 *tft;

typedef enum{
    ROS = railway_interfaces__msg__LocomotiveControl__PROTOCOL_ROS, 
    MM1 = railway_interfaces__msg__LocomotiveControl__PROTOCOL_MM1, 
    MM2 = railway_interfaces__msg__LocomotiveControl__PROTOCOL_MM2, 
    DCC = railway_interfaces__msg__LocomotiveControl__PROTOCOL_DCC, 
    MFX = railway_interfaces__msg__LocomotiveControl__PROTOCOL_MFX
}PROTOCOL;

typedef struct{
    unsigned int address;
    PROTOCOL protocol;
}LOCOMOTIVE;

#include "track_config.h"
#include "network_config.h"

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

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define MEASUREMENT_SWITCH_PIN    27
bool display_measurents = false;

void error_loop(){
  Serial.println("Error: System halted");
  tft_printf(ST77XX_BLUE, "CANBUS\ncontroller\nError\nSystem halted");

  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}
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
  for(i = 0; i < NUMBER_OF_ACTIVE_TURNOUTS_MM; i++){
    if(active_turnouts_mm[i] == turnout_number) break;
  }
  if(i >= NUMBER_OF_ACTIVE_TURNOUTS_MM) return false;
  *turnout_index = i;
  return true;
}

bool lookupLocomotiveIndex(int locomotive_address, PROTOCOL protocol, int *locomotive_index){
  int i;
  for(i = 0; i < NUMBER_OF_ACTIVE_LOCOMOTIVES; i++){
    if((locomotive_status[i].address == locomotive_address) 
      && (locomotive_status[i].protocol == protocol)) break;
  }
  if(i >= NUMBER_OF_ACTIVE_LOCOMOTIVES) return false;
  *locomotive_index = i;
  return true;
}

int turnout_state_index = 0;

void turnout_state_publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL && NUMBER_OF_ACTIVE_TURNOUTS_MM) {
    RCSOFTCHECK(rcl_publish(&turnout_status_publisher, &turnout_status[turnout_state_index], NULL));
    turnout_state_index++;
    if(turnout_state_index == NUMBER_OF_ACTIVE_TURNOUTS_MM) turnout_state_index = 0;
  }
}


int locomotive_state_index = 0;

void locomotive_state_publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL && NUMBER_OF_ACTIVE_LOCOMOTIVES) {
    RCSOFTCHECK(rcl_publish(&locomoitive_status_publisher, &locomotive_status[locomotive_state_index], NULL));
    locomotive_state_index++;
    if(locomotive_state_index == NUMBER_OF_ACTIVE_LOCOMOTIVES) locomotive_state_index = 0;
  }
}

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

void turnout_control_callback(const void * msgin)
{  
  const railway_interfaces__msg__TurnoutControl * control = (const railway_interfaces__msg__TurnoutControl *)msgin;
  int index;
  boolean straight = control->state ? true : false;
  // update controller always !!!
  if(power_status.state){
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

uint getCANAdress(PROTOCOL protocol, uint address){
  switch(protocol){
    case DCC:
      return(address + ADDR_DCC);
    break;
    case MFX:
      return(address + ADDR_MFX);
    break;
  }
  return address;
}

void locomotive_control_callback(const void * msgin)
{  
  const railway_interfaces__msg__LocomotiveControl * control = (const railway_interfaces__msg__LocomotiveControl *)msgin;

  int locomotive_index;
  char* direction_txt;
  char protocol_txt[10];
  uint address;

  switch(control->command){
    case railway_interfaces__msg__LocomotiveControl__SET_SPEED:
      address = getCANAdress((PROTOCOL)control->protocol, control->address);
      ctrl->setLocoSpeed(address, control->speed);
      //Serial.printf("Address: %i\n", control->address);
      //Serial.printf("Speed: %i\n", control->speed);

      if(lookupLocomotiveIndex(control->address, (PROTOCOL)control->protocol, &locomotive_index)){
        //Serial.printf("Found\n");
        locomotive_status[locomotive_index].speed = control->speed;

      }
      lookupLocomotiveProtocol((PROTOCOL)control->protocol, protocol_txt);
      tft_printf(ST77XX_GREEN, "ROS msg\nLocomotive\nAddress(%s): %i\nSet speed: %i\n",
            protocol_txt, address, control->speed);
      break;
    case railway_interfaces__msg__LocomotiveControl__SET_DIRECTION:
      address = getCANAdress((PROTOCOL)control->protocol, control->address);
      ctrl->setLocoDirection(address, control->direction);
      if(lookupLocomotiveIndex(control->address, (PROTOCOL)control->protocol, &locomotive_index)){
        locomotive_status[locomotive_index].direction = control->direction;
      }
      direction_txt = getDirectionTxt(control->direction);
      lookupLocomotiveProtocol((PROTOCOL)control->protocol, protocol_txt);
      tft_printf(ST77XX_GREEN, "ROS msg\nLocomotive\nAddress(%s): %i\nSet dir: %s\n",
            protocol_txt, address, direction_txt);      
      break;
    case railway_interfaces__msg__LocomotiveControl__SET_FUNCTION:
      address = getCANAdress((PROTOCOL)control->protocol, control->address);
      if(lookupLocomotiveIndex(address, (PROTOCOL)control->protocol, &locomotive_index)){
        locomotive_status[locomotive_index].function_state.data[control->function_index] = control->function_state;
      }
      lookupLocomotiveProtocol((PROTOCOL)control->protocol, protocol_txt);
      tft_printf(ST77XX_GREEN, "ROS msg\nLocomotive\nAddress(%s): %i\nSet Func. %i: %s\n",
            protocol_txt, address, control->function_index, control->function_state ? "True" : "False");
      break;
    default:
      Serial.println("Invalid command");
  }
}

void power_control_callback(const void * msgin)
{  
  const railway_interfaces__msg__PowerControl * control = (const railway_interfaces__msg__PowerControl *)msgin;
  ctrl->setPower(control->enable);
  power_status.state = control->enable;
  tft_printf(ST77XX_GREEN, "ROS msg\nSystem: %s", power_status.state ? "Go" : "Stop");

}


bool getProtocolFromAddress(uint address, uint *subaddress, PROTOCOL *protocol){

  if(address <= 0x3FF){
    *protocol = MM1;
    *subaddress = address;
    return true;
  }

  if((address >= ADDR_MFX) && (address <= ADDR_MFX_MAX)){
    *protocol = MFX;
    *subaddress = address - ADDR_MFX;
    return true;
  }

  if((address >= ADDR_DCC) && (address <= ADDR_DCC_MAX)){
    *protocol = DCC;
    *subaddress = address - ADDR_DCC;
    return true;
  }
  return false;
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


  EEPROM.begin(NUMBER_OF_ACTIVE_TURNOUTS_MM);

  power_status.state = false;
  power_status.current = 0;
  power_status.operating_mode = railway_interfaces__msg__PowerControl__OPERTING_MODE_NORMAL;
  power_status.controller_type = railway_interfaces__msg__PowerState__CONTROLLER_CAN;

  for(int i = 0; i < NUMBER_OF_ACTIVE_TURNOUTS_MM; i++){
    turnout_status[i].number = active_turnouts_mm[i];
    turnout_status[i].state = EEPROM.readBool(i);
    turnout_status[i].protocol = railway_interfaces__msg__TurnoutControl__PROTOCOL_MM1;
    //ctrl->getTurnout(turnout_status[i].number, &turnout_status[i].state);
  }

  for(int i = 0; i < NUMBER_OF_ACTIVE_LOCOMOTIVES; i++){
    locomotive_status[i].protocol = active_locomotives[i].protocol;
    locomotive_status[i].address = active_locomotives[i].address;
    locomotive_status[i].direction = railway_interfaces__msg__LocomotiveState__DIRECTION_FORWARD;
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
  WiFi.setHostname("RailTrackController");
  set_microros_wifi_transports(WIFI_SSID, PASSWORD, agent_ip, (size_t)PORT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  tft_printf(ST77XX_MAGENTA, "Marklin\ncanbus\nWiFi\nConnected\n");
  Serial.printf("Marklin canbus WiFi Connected\n");
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
    ROSIDL_GET_MSG_TYPE_SUPPORT(railway_interfaces, msg, PowerState),
    topic_name));

  sprintf(topic_name, "railtrack/power_control");
  // create power_control_subscriber
  RCCHECK(rclc_subscription_init_default(
    &power_control_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(railway_interfaces, msg, PowerControl),
    topic_name));

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


  // create executor
  int number_of_executors = 6;
  
  RCCHECK(rclc_executor_init(&executor, &support.context, number_of_executors, &allocator));

  RCCHECK(rclc_executor_add_timer(&executor, &turnout_state_publisher_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &turnout_control_subscriber, &turnout_control, &turnout_control_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_timer(&executor, &locomotive_state_publisher_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &locomotive_control_subscriber, &locomotive_control, &locomotive_control_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_timer(&executor, &power_state_publisher_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &power_control_subscriber, &power_control, &power_control_callback, ON_NEW_DATA));

  pinMode(MEASUREMENT_SWITCH_PIN, INPUT_PULLUP);

  Serial.println("!!! Ready for operating !!!");
  tft_printf(ST77XX_MAGENTA, "Marklin\ncanbus\ncontroller\nReady\n");
}

int old_display_measurents_switch = HIGH;

void loop() {

#if 1
  int new_display_measurents_switch = digitalRead(MEASUREMENT_SWITCH_PIN);
  if((old_display_measurents_switch != new_display_measurents_switch) && (new_display_measurents_switch == LOW)){
    display_measurents = display_measurents ? false : true;
    //Serial.println("Toggle");
    //Serial.println(display_measurents);
    tft_printf(ST77XX_GREEN,"");
  }
  old_display_measurents_switch = new_display_measurents_switch;
#endif

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
    PROTOCOL protocol;


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
            turnout_status[index].state = straight;
            EEPROM.writeBool(index, straight);
            EEPROM.commit();          
          } 
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
          tft_printf(ST77XX_GREEN, "CANBUS msg\nTurnout\nNumber: %i\nSet: %s\n",
            turnout_number, straight ? "Green" : "Red");

        break;
      case LOC_GESCHWINDIGHEID:
          speed = (message.data[4] << 8) 
                 + message.data[5];
          if(getProtocolFromAddress(address, &sub_address, &protocol)){
            lookupLocomotiveProtocol(protocol, protocol_txt);
            if(lookupLocomotiveIndex(sub_address, protocol, &index)){
              locomotive_status[index].speed = speed;
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
        //lookupLocomotiveProtocol(control->protocol, protocol_txt);

#if 0
            Serial.printf("Length %i\n", message.length);
            Serial.printf("CANBUS msg\nLocomotive\nAddress(%s): %i\nSet dir: %s\n",
              protocol_txt, sub_address, direction_txt); 
#endif
            tft_printf(ST77XX_GREEN, "CANBUS msg\nLocomotive\nAddress(%s): %i\nSet dir: %s\n",
              protocol_txt, sub_address, direction_txt);      
          break;
      case LOC_FUNCTION:
          //lookupLocomotiveProtocol(address, protocol_txt, &sub_address);
          //lookupLocomotiveProtocol(control->protocol, protocol_txt);

          function_index = message.data[4];
          function_enable = message.data[5];
          tft_printf(ST77XX_GREEN, "CANBUS msg\nLocomotive\nAddress(%s): %i\nSet Func. %i: %s\n",
            protocol_txt, sub_address, function_index, function_enable ? "True" : "False");
          #if 0
          if(lookupLocomotiveIndex(address, &index)){
            locomotive_status[index].function_state.data[function_index] = function_enable ? true : false;
          } 
          #endif 
        break;
      default:
        break;    
    }
  }

  vTaskDelay(20);
  //delay(20);

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

}