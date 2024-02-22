/*********************************************************************
 * TrackController - Hacking your Märklin
 *
 * Original version Railuino Copyright (C) 2012 Joerg Pleumann
 * Adapted by Gerard Harkema januari 2024
 * 
 * This example is free software; you can redistribute it and/or
 * modify it under the terms of the Creative Commons Zero License,
 * version 1.0, as published by the Creative Commons Organisation.
 * This effectively puts the file into the public domain.
 *
 * This example is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * LICENSE file for more details.
 */
//#include <Arduino.h>
#include "TrackController.h"

const bool DEBUG = true;

TrackController *ctrl;//(0xdf24, DEBUG);

TrackMessage message;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  ctrl = new TrackController(0xdf24, DEBUG);
  
  Serial.println();
  Serial.println();
  Serial.println("DIR HASH R CMND LNGT DAT0 DAT1 DAT2 DAT3 DAT4 DAT5 DAT6 DAT7");
  Serial.println("--- ---- - ---- ---- ---- ---- ---- ---- ---- ---- ---- ----");
  ctrl->begin();
}

void loop() {
  ctrl->receiveMessage(message);
  delay(20);
}

