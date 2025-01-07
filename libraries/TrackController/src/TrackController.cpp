/*********************************************************************
 * TrackController - Hacking your Märklin
 *
 * Original version Railuino Copyright (C) 2012 Joerg Pleumann
 * Adapted by Gerard Harkema januari 2024
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * LICENSE file for more details.
 */


#include "TrackController.h"
#include "mcp2515.hpp"


size_t printHex(Print &p, unsigned long hex, int digits) {
    size_t size = 0;

    String s = String(hex, HEX);

    for (int i = s.length(); i < digits; i++) {
        size += p.print("0");
    }

    size += p.print(s);

    return size;
}

int parseHex(String &s, int start, int end, boolean *ok) {
    int value = 0;

    for (int i = start; i < end; i++) {
    	char c = s.charAt(i);

        if (c >= '0' && c <= '9') {
            value = 16 * value + c - '0';
        } else if (c >= 'a' && c <= 'f') {
            value = 16 * value + 10 + c - 'a';
        } else if (c >= 'A' && c <= 'F') {
            value = 16 * value + 10 + c - 'A';
        } else {
        	*ok = false;
            return -1;
        }
    }

    return value;
}

//#define SIZE 32
#define SIZE 128

#define ulong unsigned long

can_t _buffer[SIZE];

volatile int posRead = 0;

volatile int posWrite = 0;

volatile boolean lastOpWasWrite = false;

#ifndef IRAM_ATTR
	#define IRAM_ATTR // definition for Arduino UNO platform
#endif 

void IRAM_ATTR enqueue() {
	//Serial.println("!");
	if (posWrite == posRead && lastOpWasWrite) {
		Serial.println("!!! Buffer full");
		return;
	}

	if (can_get_message(&_buffer[posWrite])) {
		posWrite = (posWrite + 1) % SIZE;
	} else {
		Serial.println("!!! No message");
	}

	lastOpWasWrite = true;
}


#if defined(ESP32)
void enqueue_task(void *pvParameter)
{
	while(1)
	{
	    if(digitalRead(MCP1512_INT_PINn) == 0){
			enqueue();
		}
		taskYIELD();
	}
}
#endif

boolean dequeue(can_t *p) {

#ifndef ESP32 
	noInterrupts();
#endif

	if (posWrite == posRead && !lastOpWasWrite) {
#ifndef ESP32 
		interrupts();
#endif
		return false;
	}

	memcpy(p, &_buffer[posRead], sizeof(can_t));

	posRead = (posRead + 1) % SIZE;
	lastOpWasWrite = false;

#ifndef ESP32 
	interrupts();
#endif
	return true;
}

// ===================================================================
// === TrackMessage ==================================================
// ===================================================================

void TrackMessage::clear() {
	command = 0;
	hash = 0;
	response = false;
	length = 0;
	for (int i = 0; i < 8; i++) {
		data[i] = 0;
	}
}

size_t TrackMessage::printTo(Print& p) const {
    size_t size = 0;

    size += printHex(p, hash, 4);
    size += p.print(response ? " R  " : "    ");
    size += printHex(p, command, 2);
    size += p.print("    ");
    size += printHex(p, length, 1);

    for (int i = 0; i < length; i++) {
        size += p.print("   ");
        size += printHex(p, data[i], 2);
    }

    return size;
}
// ===================================================================
// === TrackController ===============================================
// ===================================================================

TrackController::TrackController() {
	if (mDebug) {
		Serial.println(F("### Creating controller"));
	}

	init(0, false, false);
}

TrackController::TrackController(word hash, boolean debug) {
	init(hash, debug, false);
  	if (mDebug) {
		Serial.println(F("### Creating controller"));
	}
}

TrackController::~TrackController() {
	if (mDebug) {
		Serial.println(F("### Destroying controller"));
	}

	end();
}

void TrackController::init(word hash, boolean debug, boolean loopback) {
	mHash = hash;
	mDebug = debug;
	mLoopback = loopback;
}

word TrackController::getHash() {
	return mHash;
}

boolean TrackController::isDebug() {
	return mDebug;
}

boolean TrackController::isLoopback() {
	return mLoopback;
}

void TrackController::begin() {
    // Even if we don't use the real SS pin
    // on all boards, it must be set to out,
    // otherwise SPI might switch to slave
    // and we just hang. Do not delete!
 
   	pinMode(MCP2512_CS_PINn, OUTPUT);
   	pinMode(MCP1512_INT_PINn,  INPUT);

#if defined(ESP32)
    int app_cpu = xPortGetCoreID();
#if 1
    xTaskCreatePinnedToCore(enqueue_task,
                            "enqueue_task", 
                            1024,
                            NULL,
                            1,
                            &enqueue_task_h,
                            app_cpu);

#endif
#else
	attachInterrupt(digitalPinToInterrupt(MCP1512_INT_PINn), enqueue, FALLING);
	interrupts();
#endif

	if (!can_init(5, mLoopback)) {
		Serial.println(F("!?! Init error"));
		Serial.println(F("!?! Emergency stop"));
		for (;;);
	}

	delay(500);

	if (!mLoopback) {
		TrackMessage message;

		message.clear();
		message.command = 0x1b;
		message.length = 0x05;
		message.data[4] = 0x11;

		sendMessage(message);
	}

	if (mHash == 0) {
		generateHash();
	}

}

void TrackController::generateHash() {
	TrackMessage message;

	boolean ok = false;

	while(!ok) {
		mHash = random(0x10000) & 0xff7f | 0x0300;

		if (mDebug) {
			Serial.print(F("### Trying new hash "));
			printHex(Serial, mHash, 4);
			Serial.println();
		}

		message.clear();
		message.command = 0x18;

		sendMessage(message);

		delay(500);

		ok = true;
		while(receiveMessage(message)) {
			if (message.hash == mHash) {
				ok = false;
			}
		}
	}

	if (mDebug) {
        Serial.println(F("### New hash looks good"));
	}
}

// end - no interrupts

void TrackController::end() {
#ifndef ESP32
	detachInterrupt(digitalPinToInterrupt(MCP1512_INT_PINn));
#endif

	can_t t;

	boolean b = dequeue(&t);
	while (b) {
		b = dequeue(&t);
	}
}

boolean TrackController::sendMessage(TrackMessage &message) {
	can_t can;

	message.hash = mHash;
	
	can.id = ((uint32_t)message.command) << 17 | (uint32_t)message.hash;
	can.flags.extended = 1;
	can.flags.rtr = 0;
	can.length = message.length;

	for (int i = 0; i < message.length; i++) {
		can.data[i] = message.data[i];
	}

	if (mDebug) {
	    Serial.print("==> ");
	    Serial.println(message);
	}
	
	return can_send_message(&can);
}

boolean TrackController::receiveMessage(TrackMessage &message) {
	can_t can;

	boolean result = dequeue(&can);

	//Serial.println(result);
	if (result) {

		if(can.length > 8) return false; // Error in protocol?? Prevent unknown segmentation fault

		message.clear();
		message.command = (can.id >> 17) & 0xff;
		message.hash = can.id & 0xffff;
		message.response = bitRead(can.id, 16) || mLoopback;
		message.length = can.length;

		for (int i = 0; i < can.length; i++) {
			message.data[i] = can.data[i];
		}

		if (mDebug) {
		    Serial.print("<== ");
		    Serial.println(message);
		}
	}

	return result;
}

boolean TrackController::exchangeMessage(TrackMessage &out, TrackMessage &in, word timeout) {
	int command = out.command;

	if (!sendMessage(out)) {
		if (mDebug) {
			Serial.println(F("!?! Send error"));
			Serial.println(F("!?! Emergency stop"));
			setPower(false);
			for (;;);
		}
	}

	ulong time = millis();

	// TrackMessage response;

	while (millis() < time + timeout) {
		in.clear();
		boolean result = receiveMessage(in);

		if (result && in.command == command && in.response) {
			return true;
		}
	}

	if (mDebug) {
		Serial.println(F("!?! Receive timeout"));
	}
	
	return false;
}

boolean TrackController::setPower(boolean power) {
	TrackMessage message;

	if (power) {
		message.clear();
		message.command = 0x00;
		message.length = 0x07;
		message.data[4] = 9;
		message.data[6] = 0xD;

		exchangeMessage(message, message, 1000);

		message.clear();
		message.command = 0x00;
		message.length = 0x06;
		message.data[4] = 8;
		message.data[5] = 7;

		exchangeMessage(message, message, 1000);
	}

	message.clear();
	message.command = 0x00;
	message.length = 0x05;
	message.data[4] = power ? 0x01 : 0x00;

	return exchangeMessage(message, message, 1000);
}

boolean TrackController::setLocoDirection(word address, byte direction) {
	TrackMessage message;

    message.clear();
	message.command = 0x00;
	message.length = 0x05;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);
	message.data[4] = 0x03;

	exchangeMessage(message, message, 1000);
	
	message.clear();
	message.command = 0x05;
	message.length = 0x05;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);
	message.data[4] = direction;

	return exchangeMessage(message, message, 1000);
}

boolean TrackController::toggleLocoDirection(word address) {
    return setLocoDirection(address, DIR_CHANGE);	
}

boolean TrackController::setLocoSpeed(word address, word speed) {
	TrackMessage message;

	message.clear();
	message.command = 0x04;
	message.length = 0x06;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);
	message.data[4] = highByte(speed);
	message.data[5] = lowByte(speed);

	return exchangeMessage(message, message, 1000);
}

boolean TrackController::accelerateLoco(word address) {
	word speed;
	
	if (getLocoSpeed(address, &speed)) {
		speed += 77;
		if (speed > 1023) {
			speed = 1023;
		}
		
	    return setLocoSpeed(address, speed);
	}
	
	return false;
}

boolean TrackController::decelerateLoco(word address) {
	word speed;
	
	if (getLocoSpeed(address, &speed)) {
		speed -= 77;
		if (speed > 32767) {
			speed = 0;
		}
		
	    return setLocoSpeed(address, speed);
	}
	
	return false;
}

boolean TrackController::setLocoFunction(word address, byte function, byte power) {
	TrackMessage message;

	message.clear();
	message.command = 0x06;
	message.length = 0x06;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);
	message.data[4] = function;
	message.data[5] = power;

	return exchangeMessage(message, message, 1000);
}

boolean TrackController::toggleLocoFunction(word address, byte function) {
    byte power;
    if (getLocoFunction(address, function, &power)) {
    	return setLocoFunction(address, function, power ? 0 : 1);
    }
    
    return false;
}

boolean TrackController::setAccessory(word address, byte position, byte power,
		word time) {
	TrackMessage message;

	message.clear();
	message.command = 0x0b;
	message.length = 0x06;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);
	message.data[4] = position;
	message.data[5] = power;

	exchangeMessage(message, message, 1000);

	if (time != 0) {
		delay(time);
		
		message.clear();
		message.command = 0x0b;
		message.length = 0x06;
		message.data[2] = highByte(address);
		message.data[3] = lowByte(address);
		message.data[4] = position;

		exchangeMessage(message, message, 1000);
	}
	
	return true;
}

boolean TrackController::setTurnout(word address, boolean straight) {
	return setAccessory(address, straight ? ACC_STRAIGHT : ACC_ROUND, 1, 1000);
}

boolean TrackController::getLocoDirection(word address, byte *direction) {
	TrackMessage message;

	message.clear();
	message.command = 0x05;
	message.length = 0x04;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);

	if (exchangeMessage(message, message, 1000)) {
		direction[0] = message.data[4];
		return true;
	} else {
		return false;
	}
}

boolean TrackController::getLocoSpeed(word address, word *speed) {
	TrackMessage message;

	message.clear();
	message.command = 0x04;
	message.length = 0x04;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);

	if (exchangeMessage(message, message, 1000)) {
		speed[0] = word(message.data[4], message.data[5]);
		return true;
	} else {
		return false;
	}
}

#define CURRENT_RESOLUTION  	(5.0/0x1000)
float TrackController::getCurrent() {
	TrackMessage message;
	float current;

	message.clear();
	message.command = SYSTEM_BEFEHL;
	message.length = 0x06;
	message.data[4] = STATUS;
	message.data[5] = STATUS_CHANNEL_CURRENT;

	if (exchangeMessage(message, message, 1000)) {
		current = ((message.data[6] << 8) + message.data[7]) * CURRENT_RESOLUTION;
		return current;
	} else {
		return 0.0;
	}
	return current;
}
#define VOLTAGE_RESOLUTION  	(50.0/0x1000)

float TrackController::getVoltage() {
	TrackMessage message;
	float voltage;

	message.clear();
	message.command = SYSTEM_BEFEHL;
	message.length = 0x06;
	message.data[4] = STATUS;
	message.data[5] = STATUS_CHANNEL_VOLTAGE;

	if (exchangeMessage(message, message, 1000)) {
		voltage = ((message.data[6] << 8) + message.data[7]) * VOLTAGE_RESOLUTION;
		return voltage;
	} else {
		return 0.0;
	}
	return voltage;
}

#define TEMPERATURE_RESOLUTION  	(1500.0/0x1000)
float TrackController::getTemperature() {
	TrackMessage message;
	float temperature;

	message.clear();
	message.command = SYSTEM_BEFEHL;
	message.length = 0x06;
	message.data[4] = STATUS;
	message.data[5] = STATUS_CHANNEL_TEMPERATURE;

	if (exchangeMessage(message, message, 1000)) {
		temperature = ((message.data[6] << 8) + message.data[7]) * TEMPERATURE_RESOLUTION;
		return temperature;
	} else {
		return 0.0;
	}
	return temperature;
}

boolean TrackController::getLocoFunction(word address, byte function,
		byte *power) {
	TrackMessage message;

	message.clear();
	message.command = 0x06;
	message.length = 0x05;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);
	message.data[4] = function;

	if (exchangeMessage(message, message, 1000)) {
		power[0] = message.data[5];
		return true;
	} else {
		return false;
	}
}

boolean TrackController::getAccessory(word address, byte *position, byte *power) {
	TrackMessage message;

	message.clear();
	message.command = 0x0b;
	message.length = 0x04;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);

	if (exchangeMessage(message, message, 1000)) {
		position[0] = message.data[4];
		power[0] = message.data[5];
		return true;
	} else {
		return false;
	}
}

boolean TrackController::getTurnout(word address, boolean *straight){
	byte *position, *power;
	boolean result = getAccessory(address, position, power);
	straight = (boolean *)position;
	return result;
}

boolean TrackController::writeConfig(word address, word number, byte value) {
	TrackMessage message;

	message.clear();
	message.command = 0x08;
	message.length = 0x08;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);
	message.data[4] = highByte(number);
	message.data[5] = lowByte(number);
	message.data[6] = value;

	return exchangeMessage(message, message, 10000);
}

boolean TrackController::readConfig(word address, word number, byte *value) {
	TrackMessage message;

	message.clear();
	message.command = 0x07;
	message.length = 0x07;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);
	message.data[4] = highByte(number);
	message.data[5] = lowByte(number);
	message.data[6] = 0x01;

	if (exchangeMessage(message, message, 10000)) {
		value[0] = message.data[6];
		return true;
	} else {
		return false;
	}
}

boolean TrackController::getVersion(byte *high, byte *low) {
    boolean result = false;

    TrackMessage message;
    
    message.clear();
    message.command = 0x18;

    sendMessage(message);

    delay(500);

    while(receiveMessage(message)) {
        if (message.command = 0x18 && message.data[6] == 0x00 && message.data[7] == 0x10) {
            (*high) = message.data[4];
            (*low) = message.data[5];
            result = true;
        }
    }
    
    return result;
}

