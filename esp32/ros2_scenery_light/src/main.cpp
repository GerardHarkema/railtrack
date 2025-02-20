#include <Arduino.h>

#include <EEPROM.h>

#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/bool.h>
#include <railway_interfaces/msg/scenery_control.h>
#include <railway_interfaces/msg/scenery_state.h>

rcl_publisher_t scenery_status_publisher;
rcl_subscription_t scenery_control_subscriber;
railway_interfaces__msg__SceneryControl scenery_light_control;
rclc_executor_t executor;


rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define STATUS_LED 3


typedef struct{
  int pin;
}SCENERY_LIGHT_MONO_CONFIG;

typedef enum{
  LED_ORDER_RGB = 0,
  LED_ORDER_GRB
}LED_ORDER;

typedef struct{
  int pin;
  int number_of_leds;
  LED_ORDER led_order; // to be implemented

}SCENERY_LIGHT_RGB_CONFIG;

typedef enum{
    LT_MONO = railway_interfaces__msg__SceneryState__TYPE_MONO, 
    LT_RGB = railway_interfaces__msg__SceneryState__TYPE_RGB
}LIGHT_TYPES;

typedef struct{
  LIGHT_TYPES type;
  int scenery_light_number;
  union{
    SCENERY_LIGHT_MONO_CONFIG mono;
    SCENERY_LIGHT_RGB_CONFIG rgb;   
  };
}SCENERY_LIGHT_CONFIG;



#include "network_config.h"
#include "scenery_config.h"

IPAddress agent_ip(ip_address[0], ip_address[1], ip_address[2], ip_address[3]);


typedef struct{
  int effect;
  int brightness;
  int effect_speed;
  int r, g, b;
}EEPROM_STORE;

EEPROM_STORE eeprom_store[NUMBER_OF_SCENERY_LIGHTS];

railway_interfaces__msg__SceneryState scenery_light_status[NUMBER_OF_SCENERY_LIGHTS] = {0};

rcl_timer_t timer;
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

int scan_index = 0;

#define RGB_BUILTIN 21
#define RGB_BRIGHTNESS 10 // Change white brightness (max 255)

bool errorLedState = false;


#include <WS2812FX.h>

#define LED_COUNT 24
#define LED_PIN 2

#define STATUS_LED_PIN    8

typedef struct{
  bool active;
  WS2812FX *ws2812fx;
}WS2812_CONFIGURATION;

WS2812_CONFIGURATION WS2812_configurations[NUMBER_OF_SCENERY_LIGHTS];


#if defined(ARDUINO_ESP32C3_DEV)
#elif defined(ARDUINO_ESP32S3_DEV)
    WS2812FX ws2812fxStatus = WS2812FX(1, RGB_BUILTIN, NEO_GRB + NEO_KHZ800);
#else
    "Unknown Platform"
#endif


bool lookupSceneryLightIndex(int scenery_light_number, int *scenery_light_index){

  while(scan_index < NUMBER_OF_SCENERY_LIGHTS){
    if(scenery_lights_config[scan_index].scenery_light_number == scenery_light_number){
      *scenery_light_index = scan_index;
      scan_index++; 
      return true;
    }
    scan_index++; 
  }  
  if(scan_index == NUMBER_OF_SCENERY_LIGHTS) scan_index = 0;
  return false;
}


void error_loop(){
  Serial.printf("Light RGB controller\nError\nSystem halted");
  while(1){
      
#if defined(ARDUINO_ESP32C3_DEV)

#elif defined(ARDUINO_ESP32S3_DEV)
        ws2812fxStatus.service();
#else
    "Unknown Platform"
#endif
        if(errorLedState){
            //neopixelWrite(RGB_BUILTIN,0,0, 0);
#if defined(ARDUINO_ESP32C3_DEV)
              digitalWrite(STATUS_LED_PIN, HIGH);

#elif defined(ARDUINO_ESP32S3_DEV)
            ws2812fxStatus.setColor(0,0,0);
#else
    "Unknown Platform"
#endif
            errorLedState = false;
        }
        else{
            //neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0, 0);
#if defined(ARDUINO_ESP32C3_DEV)
              digitalWrite(STATUS_LED_PIN, LOW);
#elif defined(ARDUINO_ESP32S3_DEV)
            ws2812fxStatus.setColor(RGB_BRIGHTNESS,0,0);
#else
    "Unknown Platform"
#endif
            errorLedState = true;
        }
    delay(100);
  }
}


void scenery_control_callback(const void * msgin)
{  
  int scenery_index;
  const railway_interfaces__msg__SceneryControl * control = (const railway_interfaces__msg__SceneryControl *)msgin;

  while(lookupSceneryLightIndex(control->number, &scenery_index)){
      //Serial.println("set turnout");
      uint pin;
      bool state;
      int pwm_value, analog_value;

      switch(scenery_lights_config[scenery_index].type){
        case LT_MONO:
          scenery_light_status[scenery_index].brightness = control->brightness;
          scenery_light_status[scenery_index].effect_speed = 0;
          scenery_light_status[scenery_index].effect = 0;
          scenery_light_status[scenery_index].color.r = 0;
          scenery_light_status[scenery_index].color.g = 0;
          scenery_light_status[scenery_index].color.b = 0;
          scenery_light_status[scenery_index].light_type = LT_MONO;


          // set led with analog write
          break;
        case LT_RGB:
          scenery_light_status[scenery_index].brightness = control->brightness;
          scenery_light_status[scenery_index].effect_speed = control->effect_speed;
          scenery_light_status[scenery_index].effect = control->effect;
          scenery_light_status[scenery_index].color.r = control->color.r;
          scenery_light_status[scenery_index].color.g = control->color.g;
          scenery_light_status[scenery_index].color.b = control->color.b;
          scenery_light_status[scenery_index].light_type = LT_RGB;

          WS2812_configurations[scenery_index].ws2812fx->setMode(control->effect);
          WS2812_configurations[scenery_index].ws2812fx->setBrightness(control->brightness ? control->brightness : 1); // Patch to set leds off on zero brightspace
          WS2812_configurations[scenery_index].ws2812fx->setSpeed(control->effect_speed);
          WS2812_configurations[scenery_index].ws2812fx->setColor(control->color.r,control->color.g,control->color.b);
          break;
        default:
          break;
      }

      eeprom_store[scenery_index].brightness = scenery_light_status[scenery_index].brightness;
      eeprom_store[scenery_index].effect_speed = scenery_light_status[scenery_index].effect_speed;
      eeprom_store[scenery_index].effect = scenery_light_status[scenery_index].effect;
      eeprom_store[scenery_index].r = scenery_light_status[scenery_index].color.r;
      eeprom_store[scenery_index].g = scenery_light_status[scenery_index].color.g;
      eeprom_store[scenery_index].b = scenery_light_status[scenery_index].color.b;

      int address = scenery_index * sizeof(EEPROM_STORE);
    
      // Cast the current structure as a byte array
      byte *eeprom = (byte*)&eeprom_store[scenery_index];
    
      // Read each byte from EEPROM
      for(int j = 0; j < sizeof(EEPROM_STORE); j++) {
          EEPROM.write(address + j, eeprom[j]);
      }
      EEPROM.commit();
  }
}

int scenery_light_state_index = 0;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&scenery_status_publisher, &scenery_light_status[scenery_light_state_index], NULL));
    scenery_light_state_index++;
    if(scenery_light_state_index == NUMBER_OF_SCENERY_LIGHTS)scenery_light_state_index = 0;
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
  Serial.print("Light-controller started, node: ");
  Serial.println(NODE_NAME);

  EEPROM.begin(sizeof(EEPROM_STORE) * NUMBER_OF_SCENERY_LIGHTS);


  for(int i = 0; i< NUMBER_OF_SCENERY_LIGHTS; i++){
    // Get the starting address for the current EEPROM_STORE
    int address = i * sizeof(EEPROM_STORE);
    
    // Cast the current structure as a byte array
    byte *eeprom = (byte*)&eeprom_store[i];
    
    // Read each byte from EEPROM
    for(int j = 0; j < sizeof(EEPROM_STORE); j++) {
        eeprom[j] = EEPROM.read(address + j);
    }
    scenery_light_status[i].number = scenery_lights_config[i].scenery_light_number;
    scenery_light_status[i].light_type = scenery_lights_config[i].type;
    WS2812_configurations[i].active = false;
    uint8_t led_order_config;
    switch(scenery_lights_config[i].type){
      case LT_MONO:
        // make output pin
        break;
      case LT_RGB:
        switch(scenery_lights_config[i].rgb.led_order){
          case LED_ORDER_RGB:
            led_order_config = NEO_RGB + NEO_KHZ800;
            break;
          case LED_ORDER_GRB:
            led_order_config = NEO_GRB + NEO_KHZ800;
            break;
          // Others need to be implemented
          default:
            led_order_config = NEO_RGB + NEO_KHZ800;
            break;
        }
        WS2812_configurations[i].ws2812fx = new WS2812FX(scenery_lights_config[i].rgb.number_of_leds,
                                                          scenery_lights_config[i].rgb.pin, led_order_config);
        
        WS2812_configurations[i].ws2812fx->init();
        WS2812_configurations[i].ws2812fx->setBrightness(eeprom_store[i].brightness ? eeprom_store[i].brightness : 1); // Patch to set leds off on zero brightspace
        scenery_light_status[i].brightness = eeprom_store[i].brightness;
        WS2812_configurations[i].ws2812fx->setSpeed(eeprom_store[i].effect_speed);
        scenery_light_status[i].effect_speed = eeprom_store[i].effect_speed;
        WS2812_configurations[i].ws2812fx->setMode(eeprom_store[i].effect);
        scenery_light_status[i].effect = eeprom_store[i].effect;
        WS2812_configurations[i].ws2812fx->start();
        WS2812_configurations[i].ws2812fx->setColor(eeprom_store[i].r,
                                                    eeprom_store[i].g,
                                                    eeprom_store[i].b);
        scenery_light_status[i].color.r = eeprom_store[i].r;
        scenery_light_status[i].color.g = eeprom_store[i].g;
        scenery_light_status[i].color.b = eeprom_store[i].b;
      
        WS2812_configurations[i].ws2812fx->service();
        
        WS2812_configurations[i].active = true;
        break;
      default:
        break;
    }
     
  }

#if defined(ARDUINO_ESP32C3_DEV)
  pinMode(STATUS_LED_PIN, OUTPUT); 
  digitalWrite(STATUS_LED_PIN, HIGH);
#elif defined(ARDUINO_ESP32S3_DEV)
  ws2812fxStatus.init();
  ws2812fxStatus.setBrightness(100);
  ws2812fxStatus.setSpeed(200);
  ws2812fxStatus.setMode(FX_MODE_STATIC);
  ws2812fxStatus.start();
  ws2812fxStatus.setColor(RGB_BRIGHTNESS,0,0);
  ws2812fxStatus.service();
#else
    "Unknown Platform"
#endif


  //neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0, 0);

  const char *host_name = convertToCamelCase(NODE_NAME);
  Serial.printf("hostname :%s\n", host_name);
  WiFi.setHostname(NODE_NAME);

    // Set WiFi to station mode and disconnect from an AP if it was previously connected.
    //WiFi.effect(WIFI_STA);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, PASSWORD);
  //WiFi.setTxPower(WIFI_POWER_5dBm);
  delay(100);
#if defined(ARDUINO_ESP32S3_DEV)
  //WiFi.effect(WIFI_STA);
  WiFi.begin(WIFI_SSID, PASSWORD);
  //WiFi.setTxPower(WIFI_POWER_8_5dBm);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println();
  Serial.print("Ip adress: ");
  Serial.println(WiFi.localIP());
  Serial.print("MAC adress: ");
  Serial.println(WiFi.macAddress());
#endif


  set_microros_wifi_transports(WIFI_SSID, PASSWORD, agent_ip, (size_t)PORT);
  //neopixelWrite(RGB_BUILTIN,0,0,RGB_BRIGHTNESS);
#if defined(ARDUINO_ESP32C3_DEV)
#elif defined(ARDUINO_ESP32S3_DEV)
  ws2812fxStatus.setColor(0, 0, RGB_BRIGHTNESS);
  ws2812fxStatus.service();
#else
    "Unknown Platform"
#endif


  Serial.printf(" Scenery Light Control WiFi Connected\n");


  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  char topic_name[40];
  // create scenery_control_subscriber
  sprintf(topic_name, "railtrack/scenery/control");
  // create scenery_status_publisher

  RCCHECK(rclc_subscription_init_default(
    &scenery_control_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(railway_interfaces, msg, SceneryControl),
    topic_name));

  sprintf(topic_name, "railtrack/scenery/status");
  // create scenery_status_publisher
  RCCHECK(rclc_publisher_init_default(
    &scenery_status_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(railway_interfaces, msg, SceneryState),
    topic_name));

  // create timer,
  const unsigned int timer_timeout = 500/NUMBER_OF_SCENERY_LIGHTS;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  int number_of_executors = 2;
  RCCHECK(rclc_executor_init(&executor, &support.context, number_of_executors, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &scenery_control_subscriber, &scenery_light_control, &scenery_control_callback, ON_NEW_DATA));
 
    Serial.println("Light-controller ready");
    // turn led off, running!  
  //neopixelWrite(RGB_BUILTIN,0,RGB_BRIGHTNESS,0);
#if defined(ARDUINO_ESP32C3_DEV)
    digitalWrite(STATUS_LED_PIN, LOW);

#elif defined(ARDUINO_ESP32S3_DEV)
    ws2812fxStatus.setColor(0, RGB_BRIGHTNESS,0);

#else
    "Unknown Platform"
#endif

}

void loop() {

  delay(100);
  for(int i; i < NUMBER_OF_SCENERY_LIGHTS; i++){
    if(WS2812_configurations[i].active) WS2812_configurations[i].ws2812fx->service();
  }

  
#if defined(ARDUINO_ESP32C3_DEV)

#elif defined(ARDUINO_ESP32S3_DEV)
    ws2812fxStatus.service();
#else
    "Unknown Platform"
#endif
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

}
