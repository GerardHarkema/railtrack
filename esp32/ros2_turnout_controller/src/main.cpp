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

#include <Adafruit_GFX.h> // Core graphics library
#include <Fonts/FreeSansBold9pt7b.h>
#include <Adafruit_ST7735.h> // Hardware-specific library

#include <tft_printf.h>

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This application is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif


rcl_publisher_t turnout_status_publisher;
rcl_subscription_t turnout_control_subscriber;
rclc_executor_t executor;
railway_interfaces__msg__TurnoutControl turnout_control;


rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;


typedef struct{
  int red_pin;
  int green_pin;
}TURNOUT_CONFIG_MAGNET;

typedef struct{
  int pin;
  int red_value;
  int green_value;
}TURNOUT_CONFIG_SERVO;

typedef struct{
  int pin;
  int red_value;
  int green_value;
}TURNOUT_CONFIG_ANALOG_OUT;

typedef struct{
  int pin;
  bool negative_logic;
}TURNOUT_CONFIG_DIGITAL_OUT;

typedef enum{
  MAGNET,
  SERVO,
  ANALOG_OUT,
  DIGITAL_OUT
}TURNOUT_TYPE;

typedef enum{
    ROS = railway_interfaces__msg__TurnoutControl__PROTOCOL_ROS, 
    MM1 = railway_interfaces__msg__TurnoutControl__PROTOCOL_MM1, 
    MM2 = railway_interfaces__msg__TurnoutControl__PROTOCOL_MM2, 
    DCC = railway_interfaces__msg__TurnoutControl__PROTOCOL_DCC, 
    MFX = railway_interfaces__msg__TurnoutControl__PROTOCOL_MFX
}PROTOCOL;

typedef struct{
  TURNOUT_TYPE type;
  int turnout_number;
  union{
    TURNOUT_CONFIG_MAGNET magnet;
    TURNOUT_CONFIG_SERVO servo;
    TURNOUT_CONFIG_ANALOG_OUT analog_out;
    TURNOUT_CONFIG_DIGITAL_OUT digital_out;    
  };

  //TURNOUT turnout;
}TURNOUT_CONFIG;

#define CS_PIN  17
#define DC_PIN  18
#define RST_PIN 19
#define MOSI_PIN 21
#define SCLK_PIN 22

#define CURSOR_BASE  44
#define CURSOR_OFFSET  22

Adafruit_ST7735 *tft;

#include "turnout_config.h"

IPAddress agent_ip(ip_address[0], ip_address[1], ip_address[2], ip_address[3]);

railway_interfaces__msg__TurnoutState turnout_status[NUMBER_OF_TURNOUTS] = {0};

rcl_timer_t timer;
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


int scan_index = 0;

bool lookupTurnoutIndex(int turnout_number, int *turnout_index){
  //int i;
  while(scan_index < NUMBER_OF_TURNOUTS){
    if(turnout_config[scan_index].turnout_number == turnout_number){
      *turnout_index = scan_index;
      scan_index++; 
      return true;
    }
    scan_index++; 
  }  
  if(scan_index == NUMBER_OF_TURNOUTS) scan_index = 0;
  return false;
}

void error_loop(){
  tft_printf(ST77XX_BLUE, "DCC controller\nError\nSystem halted");
  Serial.printf("DCC controller\nError\nSystem halted");
  while(1){
    digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
    delay(100);
  }
}


void turnout_control_callback(const void * msgin)
{ 
  int turnout_index;
  const railway_interfaces__msg__TurnoutControl * control = (const railway_interfaces__msg__TurnoutControl *)msgin;
  //Serial.println("callback");
  //Serial.println(control->number);
  while(lookupTurnoutIndex(control->number, &turnout_index)){
      //Serial.println("set turnout");
      uint pin;
      bool state;
      int pwm_value, analog_value;
      switch(turnout_config[turnout_index].type){
        case MAGNET:
          pin = control->state ? turnout_config[turnout_index].magnet.green_pin : turnout_config[turnout_index].magnet.red_pin;
          digitalWrite(pin, HIGH);  
          delay(200);
          digitalWrite(pin, LOW);  
          break;
        case SERVO:
          pwm_value = control->state ? 
                      turnout_config[turnout_index].servo.green_value :
                      turnout_config[turnout_index].servo.red_value;
          analogWrite(turnout_config[turnout_index].servo.pin, pwm_value);
          break;
        case ANALOG_OUT:
          analog_value = control->state ? 
                      turnout_config[turnout_index].analog_out.green_value :
                      turnout_config[turnout_index].analog_out.red_value;        
          analogWrite(turnout_config[turnout_index].analog_out.pin, analog_value);
          break;
        case DIGITAL_OUT:
          if(turnout_config[turnout_index].digital_out.negative_logic)
             state = control->state ? false: true;
          else
             state = control->state ? true: false; 
          digitalWrite(turnout_config[turnout_index].digital_out.pin, state); 
          break;
      }


      turnout_status[turnout_index].state = control->state ? true : false;
      EEPROM.writeBool(turnout_index, turnout_status[turnout_index].state);
      EEPROM.commit();
      int cursor = CURSOR_BASE + (turnout_index * CURSOR_OFFSET);
      tft->fillRect(0, cursor- 11, tft->width()-1, CURSOR_OFFSET, ST77XX_BLACK); 
      tft->setCursor(5, cursor);
      if(turnout_status[turnout_index].state){
        tft->setTextColor(ST77XX_GREEN);
        tft->printf("T%i: Green", control->number);
      }
      else{
        tft->setTextColor(ST77XX_BLUE);
        tft->printf("T%i: Red", control->number);
      }
  }
  //else Serial.println("Invalid Turnout");

}

int turnout_state_index = 0;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    //Serial.print("x");
    RCSOFTCHECK(rcl_publish(&turnout_status_publisher, &turnout_status[turnout_state_index], NULL));
    turnout_state_index++;
    if(turnout_state_index == NUMBER_OF_TURNOUTS)turnout_state_index = 0;
  }
}


char* convertToCamelCase(const char *input) {
    int i, j;
    int len = strlen(input);
    char *output = (char *)malloc((len + 1) * sizeof(char));
    
    if(output == NULL) {
        Serial.printf("Error allocating memory\n");
        error_loop();
    }

    // Kopieer de originele string naar de uitvoerstring
    strcpy(output, input);

    // Loop door de uitvoerstring en converteer naar camel case
    for (i = 0; i < len; i++) {
        if (output[i] == '_') {
            // Verwijder de underscore
            for (j = i; j < len; j++) {
                output[j] = output[j + 1];
            }
            // Converteer het volgende teken naar hoofdletter
            output[i] = toupper(output[i]);
            // Verlaag de lengte van de string
            len--;
        }
    }
    return output;
}


void setup() {

  Serial.begin(115200);
  while (!Serial);
  delay(2000);
  Serial.print("Turnout-decoder started, node: ");
  Serial.println(NODE_NAME);

  tft = new Adafruit_ST7735(CS_PIN, DC_PIN, MOSI_PIN, SCLK_PIN, RST_PIN);
  
  tft_prinft_begin(tft);

  tft->initR(INITR_GREENTAB);
  tft->fillScreen(ST77XX_BLACK);
  tft->setRotation(35);
  delay(500);


  tft->setFont(&FreeSansBold9pt7b);
  tft->setTextSize(1);
  tft->fillScreen(ST77XX_BLACK);
  tft->setTextColor(ST77XX_CYAN);
  tft->setCursor(14, 22);
  tft->println("Turnout Control");
  tft->println(NODE_NAME);
  tft->println("Controller Started");

  EEPROM.begin(NUMBER_OF_TURNOUTS);
  const char *host_name = convertToCamelCase(NODE_NAME);
  //Serial.printf("hostname :%s\n", host_name);
  WiFi.setHostname(NODE_NAME);
  set_microros_wifi_transports(SSID, PASSWORD, agent_ip, (size_t)PORT);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);

  tft->fillRect(0, 33, tft->width()-1, tft->height() - 33, ST77XX_BLACK);
  for(int i=0; i < NUMBER_OF_TURNOUTS; i++){
    turnout_status[i].number = turnout_config[i].turnout_number;
    turnout_status[i].state = EEPROM.readBool(i);
    turnout_status[i].protocol = ROS;
    bool state;
    int analog_value;
    int pwm_value;
    int cursor = CURSOR_BASE + (i * CURSOR_OFFSET); 

    tft->setCursor(5, cursor);
    if(turnout_status[i].state){
      tft->setTextColor(ST77XX_GREEN);
      tft->printf("T%i: Green", turnout_status[i].number);
    }
    else{
      tft->setTextColor(ST77XX_BLUE);
      tft->printf("T%i: Red", turnout_status[i].number);
    }


    switch(turnout_config[i].type){
      case MAGNET:
        pinMode(turnout_config[i].magnet.green_pin, OUTPUT);
        digitalWrite(turnout_config[i].magnet.green_pin, LOW);

        pinMode(turnout_config[i].magnet.red_pin, OUTPUT);
        digitalWrite(turnout_config[i].magnet.red_pin, LOW);
        break;
      case SERVO:
        pwm_value = turnout_status[i].state ? 
                    turnout_config[i].servo.green_value :
                    turnout_config[i].servo.red_value;
        analogWrite(turnout_config[i].servo.pin, pwm_value);
        break;
      case ANALOG_OUT:
        analog_value = turnout_status[i].state ? 
                    turnout_config[i].analog_out.green_value :
                    turnout_config[i].analog_out.red_value;        
        analogWrite(turnout_config[i].analog_out.pin, analog_value);
        break;
      case DIGITAL_OUT:
        pinMode(turnout_config[i].digital_out.pin, OUTPUT);
        if(turnout_config[i].digital_out.negative_logic)
            state = turnout_status[i].state ? false: true;
        else
            state = turnout_status[i].state ? true: false; 
        digitalWrite(turnout_config[i].digital_out.pin, state); 
        break;
    }


  }

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  char topic_name[40];
  sprintf(topic_name, "railtrack/turnout/status");
  // create turnout_status_publisher
  RCCHECK(rclc_publisher_init_default(
    &turnout_status_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(railway_interfaces, msg, TurnoutState),
    topic_name));


  // create turnout_control_subscriber
  sprintf(topic_name, "railtrack/turnout/control");
  RCCHECK(rclc_subscription_init_default(
    &turnout_control_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(railway_interfaces, msg, TurnoutControl),
    topic_name));

  // create timer,
  const unsigned int timer_timeout = 500/NUMBER_OF_TURNOUTS;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  int number_of_executors = 2;
  RCCHECK(rclc_executor_init(&executor, &support.context, number_of_executors, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &turnout_control_subscriber, &turnout_control, &turnout_control_callback, ON_NEW_DATA));

  Serial.println("Turnout-decoder ready");
}

void loop() {

  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

}
