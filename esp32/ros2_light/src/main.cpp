#include <Arduino.h>

#include <EEPROM.h>

#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/bool.h>

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


void error_loop(){
  Serial.printf("DCC controller\nError\nSystem halted");
  while(1){
    digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
    delay(100);
  }
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

  const char *host_name = convertToCamelCase(NODE_NAME);
  //Serial.printf("hostname :%s\n", host_name);
  WiFi.setHostname(NODE_NAME);
  set_microros_wifi_transports(SSID, PASSWORD, agent_ip, (size_t)PORT);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);
  Serial.printf("Light Control WiFi Connected\n");


  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));


  // create timer,
  const unsigned int timer_timeout = 500;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  int number_of_executors = 2;
  RCCHECK(rclc_executor_init(&executor, &support.context, number_of_executors, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  //RCCHECK(rclc_executor_add_subscription(&executor, &turnout_control_subscriber, &turnout_control, &turnout_control_callback, ON_NEW_DATA));

  Serial.println("Light-controller ready");
}

void loop() {

  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

}
