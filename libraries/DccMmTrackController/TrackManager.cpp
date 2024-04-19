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


#include "TrackManager.h"
#include "soc/gpio_sig_map.h"

//#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(fmt, ...) \
    do { \
        Serial.printf("DEBUG: %s:%d:%s(): " fmt, \
                __FILE__, __LINE__, __func__, ##__VA_ARGS__); \
    } while (0)
#else
#define DEBUG_PRINT(fmt, ...) \
    do {} while (0)
#endif

#define LED_RED     0
#define LED_GREEN   2
#define LED_BLUE    4

#ifdef HW_DEBUG
#define TRACK_PULSE_PIN         LED_BLUE
#define TRACK_POWER_ENABLE_PIN  LED_GREEN
#else
#define TRACK_PULSE_PIN         (gpio_num_t)32
#define TRACK_POWER_ENABLE_PIN  (gpio_num_t)25
#endif

#define TRACK_POWER_ON    HIGH
#define TRACK_POWER_OFF   LOW

// Number of bits resulting out of X bytes of DCC payload data
// Each byte has one bit extra and at the end we have one EOF marker
#define DATA_LEN(X) ((X)*9+1)

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4,2,0)
#error wrong IDF version
#endif

void setDCCBit1(rmt_item32_t* item) {
  item->level0    = 1;
  item->duration0 = DCC_1_HALFPERIOD;
  item->level1    = 0;
  item->duration1 = DCC_1_HALFPERIOD;
}

void setDCCBit0(rmt_item32_t* item) {
  item->level0    = 1;
  item->duration0 = DCC_0_HALFPERIOD;
  item->level1    = 0;
  item->duration1 = DCC_0_HALFPERIOD;
}

// special long zero to trigger scope
void setDCCBit0Long(rmt_item32_t* item) {
  item->level0    = 1;
  item->duration0 = DCC_0_HALFPERIOD + DCC_0_HALFPERIOD/10;
  item->level1    = 0;
  item->duration1 = DCC_0_HALFPERIOD + DCC_0_HALFPERIOD/10;
}

void setEOT(rmt_item32_t* item) {
  item->val = 0;
}
#if 0
  static void inline updateMinimumFreeMemoryISR(unsigned char extraBytes=0)
    __attribute__((always_inline)) {
    int spare = freeMemory()-extraBytes;
    if (spare < 0) spare = 0;
    if (spare < minimum_free_memory) minimum_free_memory = spare;
  };
#endif
// This is an array that contains the this pointers
// to all uses channel objects. This is used to determine
// which of the channels was triggering the ISR as there
// is only ONE common ISR routine for all channels.
TrackManager *channelHandle;

void IRAM_ATTR interrupt(rmt_channel_t channel, void *t) {
  //Serial.printf("R");
  TrackManager *tt = channelHandle;
  if (tt) tt->RMTinterrupt();
#if 0
  if (channel == 0)
    updateMinimumFreeMemoryISR(0);
#endif
}

#define MAX_PACKET_LEN  64
#define PREAMBLE_LEN    12
#define RMT_CHANNEL     (rmt_channel_t)0

void TrackManager::begin(){

  pinMode(TRACK_POWER_ENABLE_PIN, OUTPUT);
  digitalWrite(TRACK_POWER_ENABLE_PIN, TRACK_POWER_OFF);

  pinMode(TRACK_PULSE_PIN, OUTPUT);
  digitalWrite(TRACK_PULSE_PIN, LOW);


  // Create idle message
  idleLen = 0;
  idle_message = (rmt_item32_t*)malloc(MAX_PACKET_LEN*sizeof(rmt_item32_t));
  for (byte n=0; n<PREAMBLE_LEN; n++){
    setDCCBit1(idle_message + idleLen++);      // preamble bits
  }

#ifdef SCOPE
  setDCCBit0Long(idle_message + idleLen++); // start of packet 0 bit long version
#else
  setDCCBit0(idle_message + idleLen++);     // start of packet 0 bit normal version
#endif
  //setEOT(idle_message + idleLen++);     // EOT marker

  for (byte n=0; n<8; n++){   // 0 to 7
    setDCCBit1(idle_message + idleLen++);
  }
  for (byte n=8; n<18; n++){  // 8, 9 to 16, 17
    setDCCBit0(idle_message + idleLen++);
  }
  for (byte n=18; n<26; n++){ // 18 to 25
    setDCCBit1(idle_message + idleLen++);
  }
  setDCCBit1(idle_message + idleLen++);     // end bit
  setEOT(idle_message + idleLen++);         // EOT marker

 // Create preamble for data message
  startDataIndex = 0; // calculates th index whare data is stored
  data_message = (rmt_item32_t*)malloc((MAX_PACKET_LEN)*sizeof(rmt_item32_t));
  for (byte n=0; n<PREAMBLE_LEN; n++){
    setDCCBit1(data_message + startDataIndex++);      // preamble bits
  }

#ifdef SCOPE
  setDCCBit0Long(data_message + startDataIndex++); // start of packet 0 bit long version
#else
  setDCCBit0(data_message + startDataIndex++);     // start of packet 0 bit normal version
#endif
  //setEOT(data_message + startDataIndex++);     // EOT marker

  rmt_config_t config;
  // Configure the RMT channel for TX
  bzero(&config, sizeof(rmt_config_t));
  config.rmt_mode = RMT_MODE_TX;
  config.channel = RMT_CHANNEL;
  config.clk_div = RMT_CLOCK_DIVIDER;
  config.gpio_num = TRACK_PULSE_PIN;
  config.mem_block_num = 2; // With longest DCC packet 11 inc checksum (future expansion)
                            // number of bits needed is 22preamble + start +
                            // 11*9 + extrazero + EOT = 124
                            // 2 mem block of 64 RMT items should be enough

  ESP_ERROR_CHECK(rmt_config(&config));

  // NOTE: ESP_INTR_FLAG_IRAM is *NOT* included in this bitmask
  ESP_ERROR_CHECK(rmt_driver_install(RMT_CHANNEL, 0, ESP_INTR_FLAG_LOWMED|ESP_INTR_FLAG_SHARED));

  // //DIAG(F("Register interrupt on core %d"), xPortGetCoreID());

  ESP_ERROR_CHECK(rmt_set_tx_loop_mode(RMT_CHANNEL, true));
  channelHandle = this; // used by interrupt
  rmt_register_tx_end_callback(interrupt, 0);
  rmt_set_tx_intr_en(RMT_CHANNEL, true);

  //DIAG(F("Channel %d DCC signal for %s start"), config.channel, isMain ? "MAIN" : "PROG");

  // send one bit to kickstart the signal, remaining data will come from the
  // packet queue. We intentionally do not wait for the RMT TX complete here.
  //rmt_write_items(channel, preamble, preambleLen, false);
  rmt_fill_tx_items(RMT_CHANNEL, idle_message, idleLen, 0);

  dataReady = false;
  request_new_packet = true;

}


bool TrackManager::enableTrackPower(){
  DEBUG_PRINT("Track power enable");
  digitalWrite(TRACK_POWER_ENABLE_PIN, TRACK_POWER_ON);
  track_power_enable = true;
  return true;
}

bool TrackManager::disableTrackPower(){
  DEBUG_PRINT("Track power disable");
  digitalWrite(TRACK_POWER_ENABLE_PIN, TRACK_POWER_OFF);
  track_power_enable = false;
  return true;
}


bool TrackManager::requestNewPacket(){ 
  return request_new_packet;
}


const byte transmitMask[] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};

int TrackManager::RMTfillData(const byte buffer[], byte byteCount) {

if(!requestNewPacket()) return 0; // no data neded
  request_new_packet = false;

  // convert bytes to RMT stream of "bits"

  dataLen = startDataIndex;
  for(byte n=0; n<byteCount; n++) {
    for(byte bit=0; bit<8; bit++) {
      if (buffer[n] & transmitMask[bit])
	      setDCCBit1(data_message + dataLen++);
      else
	      setDCCBit0(data_message + dataLen++);
    }
    setDCCBit0(data_message + dataLen++); // zero at end of each byte
  }

  setDCCBit1(data_message + dataLen-1);     // overwrite previous zero bit with one bit
  setEOT(data_message + dataLen++);         // EOT marker
  //Serial.printf("dataLen = %i\n", dataLen);
  noInterrupts();                      // keep dataReady and dataRepeat consistnet to each other
  dataReady = true;
  interrupts();

  return 0;
}

void IRAM_ATTR TrackManager::RMTinterrupt() {
    if(dataReady){
      dataReady = false;
      rmt_fill_tx_items(RMT_CHANNEL, data_message, dataLen, 0);
      request_new_packet = true;
    }
    else
      rmt_fill_tx_items(RMT_CHANNEL, idle_message, idleLen, 0);
}


