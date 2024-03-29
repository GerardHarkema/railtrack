/*

*********************  Do not use Under Construction **********


*/



#include <Arduino.h>
#include <EEPROM.h>


#include <stdio.h>

#include <DCCPacket.h>
#include <DCCPacketQueue.h>
#include <DCCPacketScheduler.h>


#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This application is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

// int8_t cs, int8_t dc, int8_t rst
#define CS_PIN  16
#define DC_PIN  17
#define RST_PIN 21


#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

DCCPacketScheduler DccPacketScheduler;

void error_loop(){
  Serial.println("Error: System halted");

  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  while (!Serial);
  delay(2000);
  Serial.println("DCC controller started");

  if(DccPacketScheduler.setup())error_loop();

#if 0
  Serial.print("MOSI: ");Serial.println(MOSI);
  Serial.print("MISO: ");Serial.println(MISO);
  Serial.print("SCK: ");Serial.println(SCK);
  Serial.print("SS: ");Serial.println(SS);  
#endif

  Serial.println("Controller Started");

  Serial.println("!!! Ready for operating !!!");
  //tft_printf(ST77XX_MAGENTA, "DCC\ncanbus\ncontroller\nReady\n");
}

int once = 0;
void loop() {

  Serial.print("*");

  vTaskDelay(20);
  //delay(20);
  char speed_byte = 0;
  if(!once){
    DccPacketScheduler.setSpeed128(3,DCC_SHORT_ADDRESS,speed_byte); //This should be in the call backs of the ROS subscribers
    once++;
  }
#if 0
  DccPacketScheduler.update(); // This should be a thread started by the DccPacketScheduler.begin()
#endif

}