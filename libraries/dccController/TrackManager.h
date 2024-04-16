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


// make calculations easy and set up for microseconds
#define RMT_CLOCK_DIVIDER 80
#define DCC_1_HALFPERIOD 58  //4640 // 1 / 80000000 * 4640 = 58us
#define DCC_0_HALFPERIOD 100 //8000

class TrackManager {
 public:
 void begin();

  void IRAM_ATTR RMTinterrupt();
  int RMTfillData(const byte buffer[], byte byteCount);
  inline bool requestNewPacket(){ return request_new_packet;};

  bool disableTrackPower();
  bool enableTrackPower();

 private:
  // 2 types of data to send idle or data
  rmt_item32_t *idle;
  byte idleLen;
  rmt_item32_t *idle_message;
  byte startDataIndex;
  byte dataLen;
  rmt_item32_t *data_message;
  // flags 
  volatile bool dataReady = false;    // do we have real data available or send idle
  volatile bool request_new_packet = false;
  bool track_power_enable = false;

};
#endif //ESP32
