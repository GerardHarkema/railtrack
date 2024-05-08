/*
 *  © 2021-2022, Harald Barth.
 *  
 *  This file is part of DCC-EX
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */

#if defined(ARDUINO_ARCH_ESP32)
#pragma once
#include <Arduino.h>
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "soc/rmt_struct.h"


#ifdef DCC_EX_MOTOR_SHIELD_8874
#define TRACK_PULSE_PIN_H       (gpio_num_t)32
#define TRACK_PULSE_PIN_L       (gpio_num_t)33
#define TRACK_POWER_ENABLE_PIN  (gpio_num_t)25
#define PROG_POWER_ENABLE_PIN   (gpio_num_t)22
#define PROG_PULSE_PIN          (gpio_num_t)14        
#endif

#ifdef ARDUINO_MOTOR_SHIELD_L298
#define TRACK_PULSE_PIN_H       (gpio_num_t)32
#define TRACK_PULSE_PIN_L       (gpio_num_t)33
#define TRACK_POWER_ENABLE_PIN  (gpio_num_t)25
#define PROG_POWER_ENABLE_PIN   (gpio_num_t)22
#define PROG_PULSE_PIN          (gpio_num_t)14   
#endif

#ifdef IBT_2_MOTOR_DRIVER
#define TRACK_PULSE_PIN_H       (gpio_num_t)32
#define TRACK_PULSE_PIN_L       (gpio_num_t)33
#define TRACK_POWER_ENABLE_PIN  (gpio_num_t)25
#endif

#define TRACK_POWER_ON    HIGH
#define TRACK_POWER_OFF   LOW

// make calculations easy and set up for microseconds
#define RMT_CLOCK_DIVIDER 80
#define DCC_1_HALFPERIOD 58  //4640 // 1 / 80000000 * 4640 = 58us
#define DCC_0_HALFPERIOD 100 //8000


void protect_motor_driver_outputs();

class TrackManager {
 public:
 void begin();

  void IRAM_ATTR RMTinterrupt();
  int RMTfillData(const byte buffer[], byte byteCount);
  bool requestNewPacket();

  bool disableTrackPower();
  bool enableTrackPower();

 private:
  // 2 types of data to send idle or data
#if 1
  rmt_item32_t *idle;
  byte idleLen;
  rmt_item32_t *idle_message;
  byte startDataIndex;
  byte dataLen;
  rmt_item32_t *data_message;
  // flags 
  volatile bool dataReady = false;    // do we have real data available or send idle
  volatile bool request_new_packet = false;
#endif
  bool track_power_enable = false;

};
#endif //ESP32
