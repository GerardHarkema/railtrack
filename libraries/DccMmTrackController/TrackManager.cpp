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

void setMMBit1(rmt_item32_t* item, bool double_frequncy){
  item->level0    = 1;
  item->duration0 = double_frequncy ? MM_LONG_PULSE_DF : MM_LONG_PULSE;
  item->level1    = 0;
  item->duration1 = double_frequncy ? MM_SHORT_PULSE_DF : MM_SHORT_PULSE;
}

void setMMBit0(rmt_item32_t* item, bool double_frequncy){
  item->level0    = 1;
  item->duration0 = double_frequncy ? MM_SHORT_PULSE_DF : MM_SHORT_PULSE;
  item->level1    = 0;
  item->duration1 = double_frequncy ? MM_LONG_PULSE_DF : MM_LONG_PULSE;
}

void setMMt1(rmt_item32_t* item, bool double_frequncy){
  item->level0    = 0;
  item->duration0 = double_frequncy ? MM_HALF_T1_PULSE_DF : MM_HALF_T1_PULSE;
  item->level1    = 0;
  item->duration1 = double_frequncy ? MM_HALF_T1_PULSE_DF : MM_HALF_T1_PULSE;
}


void setMMt2(rmt_item32_t* item, bool double_frequncy){
  item->level0    = 0;
  item->duration0 = double_frequncy ? MM_HALF_T2_PULSE_DF : MM_HALF_T2_PULSE;
  item->level1    = 0;
  item->duration1 = double_frequncy ? MM_HALF_T2_PULSE_DF : MM_HALF_T2_PULSE;
}

#if 0
void setMMTribit0(rmt_item32_t* item, bool double_frequncy){
  rmt_item32_t* l_item = item;
  setMMBit0(item, double_frequncy);
  l_item++;
  setMMBit0(item, double_frequncy);
}

void setMMTribit1(rmt_item32_t* item, bool double_frequncy){
  rmt_item32_t* l_item = item;
  setMMBit1(item, double_frequncy);
  l_item++;
  setMMBit1(item, double_frequncy);
}

void setMMTribitOpen(rmt_item32_t* item, bool double_frequncy){
  rmt_item32_t* l_item = item;
  setMMBit1(item, double_frequncy);
  l_item++;
  setMMBit0(item, double_frequncy);
}
#endif

void setDCCeot(rmt_item32_t* item) {
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


void protect_motor_driver_outputs(){
#ifdef DCC_EX_MOTOR_SHIELD_8874 # ARDUINO_MOTOR_SHIELD_L298
  pinMode(TRACK_POWER_ENABLE_PIN, OUTPUT);
  digitalWrite(TRACK_POWER_ENABLE_PIN, TRACK_POWER_OFF);

  pinMode(TRACK_PULSE_PIN_H, OUTPUT);
  digitalWrite(TRACK_PULSE_PIN_H, LOW);

  pinMode(PROG_POWER_ENABLE_PIN, OUTPUT);
  digitalWrite(PROG_POWER_ENABLE_PIN, TRACK_POWER_OFF);

  pinMode(PROG_PULSE_PIN, OUTPUT);
  digitalWrite(PROG_PULSE_PIN, LOW);
#endif

#ifdef IBT_2_MOTOR_DRIVER
  pinMode(TRACK_POWER_ENABLE_PIN, OUTPUT);
  digitalWrite(TRACK_POWER_ENABLE_PIN, TRACK_POWER_OFF);

  pinMode(TRACK_PULSE_PIN_H, OUTPUT);
  digitalWrite(TRACK_PULSE_PIN_H, LOW);

  pinMode(TRACK_PULSE_PIN_L, OUTPUT);
  digitalWrite(TRACK_PULSE_PIN_L, LOW);
#endif

}

#define DCC_MAX_PACKET_LEN  64
#define DCC_PREAMBLE_LEN    12
#define RMT_CHANNEL         (rmt_channel_t)0


#define MM_MAX_PACKET_LEN  80 // 4(twice a double packet) * (18 pulses + 1 idle states(no pulses)) + 4 spare


void TrackManager::begin(){

  pinMode(TRACK_POWER_ENABLE_PIN, OUTPUT);
  digitalWrite(TRACK_POWER_ENABLE_PIN, TRACK_POWER_OFF);

  pinMode(TRACK_PULSE_PIN_H, OUTPUT);
  digitalWrite(TRACK_PULSE_PIN_H, LOW);


  // Create idle message
  idle_len = 0;
  idle_message = (rmt_item32_t*)malloc(DCC_MAX_PACKET_LEN*sizeof(rmt_item32_t));
  for (byte n=0; n<DCC_PREAMBLE_LEN; n++){
    setDCCBit1(idle_message + idle_len++);      // preamble bits
  }

#ifdef SCOPE
  setDCCBit0Long(idle_message + idle_len++); // start of packet 0 bit long version
#else
  setDCCBit0(idle_message + idle_len++);     // start of packet 0 bit normal version
#endif
  //setDCCeot(idle_message + idle_len++);     // EOT marker

  for (byte n=0; n<8; n++){   // 0 to 7
    setDCCBit1(idle_message + idle_len++);
  }
  for (byte n=8; n<18; n++){  // 8, 9 to 16, 17
    setDCCBit0(idle_message + idle_len++);
  }
  for (byte n=18; n<26; n++){ // 18 to 25
    setDCCBit1(idle_message + idle_len++);
  }
  setDCCBit1(idle_message + idle_len++);     // end bit
  setDCCeot(idle_message + idle_len++);         // EOT marker

 // Create preamble for data message
  dcc_start_data_index = 0; // calculates th index whare data is stored
  dcc_data_message = (rmt_item32_t*)malloc((DCC_MAX_PACKET_LEN)*sizeof(rmt_item32_t));
  for (byte n=0; n<DCC_PREAMBLE_LEN; n++){
    setDCCBit1(dcc_data_message + dcc_start_data_index++);      // preamble bits
  }

#ifdef SCOPE
  setDCCBit0Long(dcc_data_message + dcc_start_data_index++); // start of packet 0 bit long version
#else
  setDCCBit0(dcc_data_message + dcc_start_data_index++);     // start of packet 0 bit normal version
#endif
  //setDCCeot(dcc_data_message + dcc_start_data_index++);     // EOT marker

  rmt_config_t config;
  // Configure the RMT channel for TX
  bzero(&config, sizeof(rmt_config_t));
  config.rmt_mode = RMT_MODE_TX;
  config.channel = RMT_CHANNEL;
  config.clk_div = RMT_CLOCK_DIVIDER;
  config.gpio_num = TRACK_PULSE_PIN_H;
  config.mem_block_num = 2; // With longest DCC packet 11 inc checksum (future expansion)
                            // number of bits needed is 22preamble + start +
                            // 11*9 + extrazero + EOT = 124
                            // 2 mem block of 64 RMT items should be enough

  ESP_ERROR_CHECK(rmt_config(&config));
#ifdef DCC_EX_MOTOR_SHIELD_8874
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[TRACK_PULSE_PIN_L], PIN_FUNC_GPIO);
  gpio_set_direction(TRACK_PULSE_PIN_L, GPIO_MODE_OUTPUT);
  gpio_matrix_out(TRACK_PULSE_PIN_L, RMT_SIG_OUT0_IDX+RMT_CHANNEL, true, 0);
#else
  gpio_set_direction(TRACK_PULSE_PIN_L, GPIO_MODE_OUTPUT);
  digitalWrite(TRACK_PULSE_PIN_L, false);
#endif
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
  rmt_fill_tx_items(RMT_CHANNEL, idle_message, idle_len, 0);

  mm_data_len = 0;
  mm_data_message = (rmt_item32_t*)malloc((MM_MAX_PACKET_LEN)*sizeof(rmt_item32_t));;

  track_data_available = NO_TRACK_DATA;
  track_new_track_packet = true;
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
  return track_new_track_packet;
}


const byte transmitMask[] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};

int TrackManager::RMTfillDataDcc(const byte buffer[], byte byteCount) {

  if(!requestNewPacket()) return 0; // no data neded
  track_new_track_packet = false;

  // convert bytes to RMT stream of "bits"

  dcc_data_len = dcc_start_data_index;
  for(byte n=0; n<byteCount; n++) {
    for(byte bit=0; bit<8; bit++) {
      if (buffer[n] & transmitMask[bit])
	      setDCCBit1(dcc_data_message + dcc_data_len++);
      else
	      setDCCBit0(dcc_data_message + dcc_data_len++);
    }
    setDCCBit0(dcc_data_message + dcc_data_len++); // zero at end of each byte
  }

  setDCCBit1(dcc_data_message + dcc_data_len-1);     // overwrite previous zero bit with one bit
  setDCCeot(dcc_data_message + dcc_data_len++);         // EOT marker
  //Serial.printf("dcc_data_len = %i\n", dcc_data_len);
  noInterrupts();                      // keep dataReady and dataRepeat consistnet to each other
  track_data_available = DCC_TRACK_DATA;
  interrupts();

  return 0;
}

int TrackManager::RMTfillDataMM(const u_int32_t data, bool doubleFrequency) {

  if(!requestNewPacket()) return 0; // no data neded
  track_new_track_packet = false;

  mm_data_len = 0;
  for(int i = 0; i < 2; i++){ // Send twice a double packet
    setMMt1(mm_data_message + mm_data_len++, doubleFrequency);
    u_int32_t mask = 1; // controleer dit getal !!!
    for(int j = 0; j < 18; j++){
      if(data & mask)
        setMMBit1(mm_data_message + mm_data_len++, doubleFrequency);
      else
        setMMBit0(mm_data_message + mm_data_len++, doubleFrequency);
      mask = mask << 1;
    }
  }    
  setMMt2(mm_data_message + mm_data_len++, doubleFrequency);
    
  //Serial.printf("mm_data_len = %i\n", mm_data_len);
  noInterrupts();                      // keep dataReady and dataRepeat consistnet to each other
  track_data_available = MM_TRACK_DATA;
  interrupts();

  return 0;
}


void IRAM_ATTR TrackManager::RMTinterrupt() {
    if(track_data_available == DCC_TRACK_DATA){
      track_data_available = NO_TRACK_DATA;
      rmt_fill_tx_items(RMT_CHANNEL, dcc_data_message, dcc_data_len, 0);
      track_new_track_packet = true;
    }
    else if (track_data_available == MM_TRACK_DATA)
    {
      track_data_available = NO_TRACK_DATA;
      rmt_fill_tx_items(RMT_CHANNEL, mm_data_message, mm_data_len, 0);
      track_new_track_packet = true;
    }
    
    else
      rmt_fill_tx_items(RMT_CHANNEL, idle_message, idle_len, 0);
}


