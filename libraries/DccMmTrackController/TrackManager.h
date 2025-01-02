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

#define MM_SHORT_PULSE    26    
#define MM_LONG_PULSE     182

#define MM_SHORT_PULSE_DF    MM_SHORT_PULSE/2  // Double frquency
#define MM_LONG_PULSE_DF     MM_LONG_PULSE/2

#define MM_HALF_IDLE_PULSE   (MM_SHORT_PULSE + MM_LONG_PULSE) * 2 * 4 / 2// according datasheet MC145026 (dead time discriminator. 4 tribits)
#define MM_HALF_IDLE_PULSE_DF MM_HALF_IDLE_PULSE/2

typedef enum{
  NO_TRACK_DATA,
  DCC_TRACK_DATA,
  MM_TRACK_DATA,
}TRACK_DATA_AVAILABLE;

void protect_motor_driver_outputs();

class TrackManager {
 public:
 void begin();

  void IRAM_ATTR RMTinterrupt();
  int RMTfillDataDcc(const byte buffer[], byte byteCount);
  int RMTfillDataMM(const u_int32_t, bool doubleFrequency);
  bool requestNewPacket();

  bool disableTrackPower();
  bool enableTrackPower();

 private:
  // 2 types of data to send idle or data
#if 1
  rmt_item32_t *idle;
  byte idle_len;
  rmt_item32_t *idle_message;
  byte dcc_start_data_index;
  byte dcc_data_len;
  rmt_item32_t *dcc_data_message;
  
  byte mm_data_len;
  rmt_item32_t *mm_data_message;


  // flags 
  volatile TRACK_DATA_AVAILABLE track_data_available = NO_TRACK_DATA;    // do we have real data available or send idle
  volatile bool track_new_track_packet = false;
#endif
  bool track_power_enable = false;

};
#endif //ESP32
