#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/bool.h>

#include <MaerklinMotorola.h>
//#include <vector>

#define MM_INPUT_PIN 4

MaerklinMotorola mm(MM_INPUT_PIN);

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This example is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

rcl_publisher_t publisher[256];
bool used_publishers[256] = {false};
//rclc_executor_t executor[2];
std_msgs__msg__Bool control[256] = {0};
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

int used_wissels[] = {1, 2, 3, 4, 5, 12, 13};
  

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define WISSEL_ID  8
#define WISSEL_ID_A  WISSEL_ID
#define WISSEL_ID_B  (WISSEL_ID + 1)

#define SSID          "BirdsModelspoor"
#define PASSWORD      "Highway12!"
#define AGENT_IP      "192.168.2.27"

void error_loop(){
  while(1){
    //digitalWrite(LED_PIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    //RCSOFTCHECK(rcl_publish(&publisher[0], &status[0], NULL));
    //RCSOFTCHECK(rcl_publish(&publisher[1], &status[1], NULL));
    //status[0].data != status[0].data;
    //status[1].data != status[1].data;
  }
}


void setup() {

  //pinMode(LED_BUILTIN, OUTPUT);
  pinMode(MM_INPUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MM_INPUT_PIN), mm_isr, CHANGE);

  //Serial.begin(115200);
  set_microros_wifi_transports(SSID, PASSWORD, AGENT_IP, 8888);

  //pinMode(LED_PIN, OUTPUT);
  //digitalWrite(LED_PIN, HIGH);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  Serial.println("wissel_numbercontroller started");


  delay(2000);


  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  char node_name[40];
  sprintf(node_name, "wissel_mm_controller_node%i" , WISSEL_ID_A);
  RCCHECK(rclc_node_init_default(&node, node_name, "", &support));

 // returns length of vector as unsigned int
  unsigned int number_of_used_wissels = sizeof(used_wissels) / sizeof(int);

  // run for loop from 0 to vecSize
  for(unsigned int i = 0; i < number_of_used_wissels; i++)
  {

    char topic_name[40];
    sprintf(topic_name, "wissel%i/control" , used_wissels[i]);
    // create publisher
    RCCHECK(rclc_publisher_init_best_effort(
      &publisher[0],
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      topic_name));
      used_publishers[used_wissels[i]] = true; 
  }
}

int i = 0;

int port_number;
int keyboard;
int wissel_number;
int groen;
int address;

void loop() {

  int input = digitalRead(MM_INPUT_PIN);
  //digitalWrite(LED_BUILTIN, input);  

  mm.Parse();
  MaerklinMotorolaData* Data = mm.GetData();
  if(Data){
    if(Data->IsMagnet){
        address = (int)Data->Address ? (int)Data->Address : 80; // Correct address 0 to 80
        port_number = ((int)address - 1) * 8 + (int)Data->SubAddress;
        keyboard = (port_number/32) + 1;
        wissel_number = (port_number / 2) + 1;
        groen = port_number % 2;
        control[0].data = groen ? true :false;
#if 0
        Serial.print(" - Address: "); Serial.print(Data->Address);
        Serial.print(" - SubAddress: "); Serial.print(Data->SubAddress);
        Serial.print(" - PortAddress: "); Serial.print(Data->PortAddress);
        Serial.print(" - PortNumber: "); Serial.print(port_number);
        Serial.print(" - Keyboard: "); Serial.print(keyboard);
        Serial.print(" - wissel_number: "); Serial.print(wissel_number);
        Serial.print(" - MagnetState: " + String(groen ? "groen" : "rood"));
        Serial.println();
#endif


        digitalWrite(LED_BUILTIN, control[0].data);
        if(used_publishers[wissel_number]){
          char topic_name[40];
          Serial.print("Publishing wissel number "); Serial.print(wissel_number); Serial.print("state: ");Serial.println(String(groen ? " groen" : " rood"));
          control[wissel_number].data = groen ? true : false;
          RCSOFTCHECK(rcl_publish(&publisher[wissel_number], &control[wissel_number], NULL));
    
        }
        else{
          Serial.print("Invalid wissel number: "); Serial.println(wissel_number);
        }


    }
  }

}

void mm_isr() {
  //Serial.print(".");
  mm.PinChange();
}

