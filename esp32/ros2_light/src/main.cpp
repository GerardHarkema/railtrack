#include <Arduino.h>

#include <EEPROM.h>

#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/bool.h>
#include <railway_interfaces/msg/light_rgb.h>

rcl_subscription_t light_rgb_subscriber;

railway_interfaces__msg__LightRGB light_rgb_control;

rclc_executor_t executor;


rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define  NODE_NAME  "light_controller"
#define STATUS_LED 3

#include "network_config.h"

IPAddress agent_ip(ip_address[0], ip_address[1], ip_address[2], ip_address[3]);


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

WS2812FX ws2812fx = WS2812FX(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
WS2812FX ws2812fxStatus = WS2812FX(1, RGB_BUILTIN, NEO_GRB + NEO_KHZ800);


void error_loop(){
  Serial.printf("Light RGB controller\nError\nSystem halted");
  while(1){
      ws2812fxStatus.service();
        if(errorLedState){
            //neopixelWrite(RGB_BUILTIN,0,0, 0);
            ws2812fxStatus.setColor(0,0,0);
            errorLedState = false;
        }
        else{
            //neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0, 0);
            ws2812fxStatus.setColor(RGB_BRIGHTNESS,0,0);
            errorLedState = true;
        }
    delay(100);
  }
}


void light_rgb_control_callback(const void * msgin)
{  
  const railway_interfaces__msg__LightRGB * control = (const railway_interfaces__msg__LightRGB *)msgin;

  ws2812fx.setMode(control->mode);
  ws2812fx.setBrightness(control->brightness);
  ws2812fx.setSpeed(control->speed);
  ws2812fx.setColor(control->color.r,control->color.g,control->color.b);

}


void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {

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

  ws2812fx.init();
  ws2812fx.setBrightness(100);
  ws2812fx.setSpeed(200);
  ws2812fx.setMode(FX_MODE_STATIC);
  ws2812fx.start();
  ws2812fx.setColor(0,0,0);
  ws2812fx.service();

  ws2812fxStatus.init();
  ws2812fxStatus.setBrightness(100);
  ws2812fxStatus.setSpeed(200);
  ws2812fxStatus.setMode(FX_MODE_STATIC);
  ws2812fxStatus.start();
  

  ws2812fxStatus.setColor(RGB_BRIGHTNESS,0,0);
  ws2812fxStatus.service();

  //neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0, 0);

  const char *host_name = convertToCamelCase(NODE_NAME);
  Serial.printf("hostname :%s\n", host_name);
  WiFi.setHostname(NODE_NAME);

    // Set WiFi to station mode and disconnect from an AP if it was previously connected.
    //WiFi.mode(WIFI_STA);
#if 1
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, PASSWORD);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
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


#if 0
    WiFi.disconnect();

    Serial.println("Scan start");
 
    // WiFi.scanNetworks will return the number of networks found.
    int n = WiFi.scanNetworks();
    Serial.println("Scan done");
    if (n == 0) {
        Serial.println("no networks found");
    } else {
        Serial.print(n);
        Serial.println(" networks found");
        Serial.println("Nr | SSID                             | RSSI | CH | Encryption");
        for (int i = 0; i < n; ++i) {
            // Print SSID and RSSI for each network found
            Serial.printf("%2d",i + 1);
            Serial.print(" | ");
            Serial.printf("%-32.32s", WiFi.SSID(i).c_str());
            Serial.print(" | ");
            Serial.printf("%4d", WiFi.RSSI(i));
            Serial.print(" | ");
            Serial.printf("%2d", WiFi.channel(i));
            Serial.print(" | ");
            switch (WiFi.encryptionType(i))
            {
            case WIFI_AUTH_OPEN:
                Serial.print("open");
                break;
            case WIFI_AUTH_WEP:
                Serial.print("WEP");
                break;
            case WIFI_AUTH_WPA_PSK:
                Serial.print("WPA");
                break;
            case WIFI_AUTH_WPA2_PSK:
                Serial.print("WPA2");
                break;
            case WIFI_AUTH_WPA_WPA2_PSK:
                Serial.print("WPA+WPA2");
                break;
            case WIFI_AUTH_WPA2_ENTERPRISE:
                Serial.print("WPA2-EAP");
                break;
            case WIFI_AUTH_WPA3_PSK:
                Serial.print("WPA3");
                break;
            case WIFI_AUTH_WPA2_WPA3_PSK:
                Serial.print("WPA2+WPA3");
                break;
            case WIFI_AUTH_WAPI_PSK:
                Serial.print("WAPI");
                break;
            default:
                Serial.print("unknown");
            }
            Serial.println();
            delay(10);
        }
    }
    Serial.println("");
 
    // Delete the scan result to free memory for code below.
    WiFi.scanDelete();
 
    // Wait a bit before scanning again.
    delay(5000);

    WiFi.mode(WIFI_STA);
#endif


  //WiFi.setTxPower(WIFI_POWER_19_5dBm);//WIFI_POWER_8_5dBm);
  //WiFi.setMinSecurity(WIFI_AUTH_WEP); // Lower min security to WEP.
  //WiFi.setMinSecurity(WIFI_AUTH_WPA_PSK); // Lower min security to WPA.
  set_microros_wifi_transports(WIFI_SSID, PASSWORD, agent_ip, (size_t)PORT);
  //neopixelWrite(RGB_BUILTIN,0,0,RGB_BRIGHTNESS);
  ws2812fxStatus.setColor(0, 0, RGB_BRIGHTNESS);
  ws2812fxStatus.service();



  Serial.printf("Light Control WiFi Connected\n");


  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  char topic_name[40];
  // create turnout_control_subscriber
  sprintf(topic_name, "LightRGB");
  // create turnout_status_publisher

  RCCHECK(rclc_subscription_init_default(
    &light_rgb_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(railway_interfaces, msg, LightRGB),
    topic_name));


  // create timer,
  const unsigned int timer_timeout = 500;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  int number_of_executors = 1;
  RCCHECK(rclc_executor_init(&executor, &support.context, number_of_executors, &allocator));
  //RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &light_rgb_subscriber, &light_rgb_control, &light_rgb_control_callback, ON_NEW_DATA));

 
    Serial.println("Light-controller ready");
    // turn led off, running!  
  //neopixelWrite(RGB_BUILTIN,0,RGB_BRIGHTNESS,0);
    ws2812fxStatus.setColor(0, RGB_BRIGHTNESS,0);

}

void loop() {

  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  ws2812fx.service();
  ws2812fxStatus.service();

}
