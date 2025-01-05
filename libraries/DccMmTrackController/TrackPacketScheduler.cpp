#include "TrackPacketScheduler.h"
#include "TrackManager.h"

//#define DEBUG_QUEUE
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

/*
 * DCC Waveform Generator
 *
 * 
 * Hardware requirements:
 *     *A DCC booster with Rail A and Rail B wired to pin 9 and 10 respectively.
 *     *A locomotive with a decoder installed, reset to factory defaults.
 *
 * Author: Don Goodman-Wilson dgoodman@artificial-science.org
 *
 * based on software by Wolfgang Kufer, http://opendcc.de
 *
 * Copyright 2010 Don Goodman-Wilson
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
 *  
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *  
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *  
 */


TrackManager track;

///////////////////////////////////////////////
///////////////////////////////////////////////
///////////////////////////////////////////////
 
  
TrackPacketScheduler::TrackPacketScheduler(void) : default_speed_steps(128), last_packet_address(255), packet_counter(1)
{


  packet_buffer.setup();

}
    
//for configuration
void TrackPacketScheduler::setDefaultSpeedSteps(uint8_t new_speed_steps)
{
  
  default_speed_steps = new_speed_steps;
}

#if THREAD_SAFE_QUEUE 
void TrackPacketScheduler::scheduler_task(void *pvParameters)
{
  TrackPacketScheduler *instance = static_cast<TrackPacketScheduler*>(pvParameters);
	while(1)
	{
    vTaskDelay(10);
    instance->update();
		//taskYIELD();
	}
}
#endif

bool TrackPacketScheduler::setup(void) //for any post-constructor initialization
{
  
  //Following RP 9.2.4, begin by putting 20 reset packets and 10 idle packets on the rails.
  //use the e_stop_queue to do this, to ensure these packets go out first!

  //if(!packet_buffer) return false;
  if(!packet_buffer.setup()){
    DEBUG_PRINT("Unable to create packet_buffer\n");
    return false;
  }
  //if(!setup_DCC_waveform_generator()) return false;

  //waveform_generator.setup_DCC_waveform_generator();

  TrackPacket p(TRACK_PROTOCOL_DCC);

  uint8_t data[] = {0x00};
  
  //reset packet: address 0x00, data 0x00, XOR 0x00; S 9.2 line 75
  p.dccAddData(data,1);
  p.dccSetAddress(0x00, DCC_SHORT_ADDRESS);
  p.setRepeatCount(20);
  p.dccSetKind(DCC_RESET_PACKET_KIND);
  p.setPriority(HIGH_PRIORIY);
  packet_buffer.insertPacket(p);
  
  //WHy in the world is it that what gets put on the rails is 4 reset packets, followed by
  //10 god know's what, followed by something else?
  // C0 FF 00 FF
  // 00 FF FF   what are these?
  
  //idle packet: address 0xFF, data 0x00, XOR 0xFF; S 9.2 line 90
  p.dccSetAddress(0xFF, DCC_SHORT_ADDRESS);
  p.setRepeatCount(10);
  p.dccSetKind(DCC_IDLE_PACKET_KIND);
  p.setPriority(HIGH_PRIORIY);

  packet_buffer.insertPacket(p); //e_stop_queue will be empty, so no need to check if insertion was OK.
#ifdef DEBUG
  packet_buffer.printQueue();
#endif

#if THREAD_SAFE_QUEUE
  int app_cpu = xPortGetCoreID();
  xTaskCreatePinnedToCore(scheduler_task,
                        "scheduler_task", 
                        4096,
                        this,
                        1,
                        &scheduler_task_h,
                        app_cpu);
  if(!scheduler_task_h){
    DEBUG_PRINT("Unable to start task\n");
    return false;
  }
#endif  
  track.begin();
  return true;
}

bool TrackPacketScheduler::trackPower(bool enable){
  if(enable) return track.enableTrackPower();
  else return track.disableTrackPower();
}
    
//for enqueueing packets

//setSpeed* functions:
bool TrackPacketScheduler::dccSetSpeed14(uint16_t address, ADDRESS_KIND dcc_address_kind, int8_t new_speed, bool F0)
{
  TrackPacket p(TRACK_PROTOCOL_DCC);
  p.dccSetAddress(address, dcc_address_kind);

  uint8_t dir = 1;
  uint8_t speed_data_uint8_ts[] = {0x40};
  uint16_t abs_speed = new_speed;
  if(new_speed<0)
  {
    dir = 0;
    abs_speed = new_speed * -1;
  }
  if(abs_speed > 14)abs_speed = 14;

  if(!new_speed) //dcc_eStop!
    return dcc_eStop(address, DCC_SHORT_ADDRESS);//speed_data_uint8_ts[0] |= 0x01; //dcc_eStop
    
  else if (abs_speed == 1) //regular stop!
    speed_data_uint8_ts[0] |= 0x00; //stop
  else //movement
    speed_data_uint8_ts[0] |= abs_speed;
  speed_data_uint8_ts[0] |= (0x20*dir); //flip bit 3 to indicate direction;
  //DEBUG_PRINTLN2(speed_data_uint8_ts[0],BIN);
  p.dccAddData(speed_data_uint8_ts,1);

  p.setRepeatCount(REPEAT_COUNT_CONTINOUS);
  
  p.dccSetKind(DCC_SPEED_PACKET_KIND);  

  //speed packets get refreshed indefinitely, and so the repeat doesn't need to be set.
  //speed packets go to the high proirity queue
  return(packet_buffer.insertPacket(p));
}

bool TrackPacketScheduler::dccSetSpeed28(uint16_t address, ADDRESS_KIND dcc_address_kind, int8_t new_speed)
{
  TrackPacket p(TRACK_PROTOCOL_DCC);
  p.dccSetAddress(address, dcc_address_kind);

  uint8_t dir = 1;
  uint8_t speed_data_uint8_ts[] = {0x40};
  uint16_t abs_speed = new_speed;
  if(new_speed<0)
  {
    dir = 0;
    abs_speed = new_speed * -1;
  }
  if(abs_speed > 27)abs_speed = 27;


//  DEBUG_PRINTLN(speed);
//  DEBUG_PRINTLN(dir);
  if(new_speed == 0) //dcc_eStop!
    return dcc_eStop(address, DCC_SHORT_ADDRESS);//speed_data_uint8_ts[0] |= 0x01; //dcc_eStop
  else if (abs_speed == 1) //regular stop!
    speed_data_uint8_ts[0] |= 0x00; //stop
  else //movement
  {
    speed_data_uint8_ts[0] |= abs_speed;
    //most least significant bit has to be shufled around
    speed_data_uint8_ts[0] = (speed_data_uint8_ts[0]&0xE0) | ((speed_data_uint8_ts[0]&0x1F) >> 1) | ((speed_data_uint8_ts[0]&0x01) << 4);
  }
  speed_data_uint8_ts[0] |= (0x20*dir); //flip bit 3 to indicate direction;
//  DEBUG_PRINTLN(speed_data_uint8_ts[0],BIN);
//  DEBUG_PRINTLN("=======");
  p.dccAddData(speed_data_uint8_ts,1);
  
  p.setRepeatCount(REPEAT_COUNT_CONTINOUS);
  
  p.dccSetKind(DCC_SPEED_PACKET_KIND);
    
  //speed packets get refreshed indefinitely, and so the repeat doesn't need to be set.
  //speed packets go to the high proirity queue
  //return(packet_buffer.insertPacket(p));
  return(packet_buffer.insertPacket(p));
}

bool TrackPacketScheduler::dccSetSpeed128(uint16_t address, ADDRESS_KIND dcc_address_kind, int8_t new_speed)
{
  //why do we get things like this?
  // 03 3F 16 15 3F (speed packet addressed to loco 03)
  // 03 3F 11 82 AF  (speed packet addressed to loco 03, speed hex 0x11);

  TrackPacket p(TRACK_PROTOCOL_DCC);
  p.dccSetAddress(address, dcc_address_kind);

  uint8_t dir = 1;
  uint16_t abs_speed = new_speed;
  uint8_t speed_data_uint8_ts[] = {0x3F,0x00};
  if(new_speed<0)
  {
    dir = 0;
    abs_speed = new_speed * -1;
  }

  if(abs_speed > 0x7f)abs_speed = 0x7f;

  if(!new_speed){
    return dcc_eStop(address, DCC_SHORT_ADDRESS);//speed_data_uint8_ts[0] |= 0x01; //dcc_eStop
  }
  else 
  if (abs_speed == 1) //regular stop!
    speed_data_uint8_ts[1] = 0x00; //stop
  else //movement
    speed_data_uint8_ts[1] = abs_speed; //no conversion necessary.

  speed_data_uint8_ts[1] |= (0x80*dir); //flip bit 7 to indicate direction;
  p.dccAddData(speed_data_uint8_ts,2);
  //DEBUG_PRINT2(speed_data_uint8_ts[0],BIN);
  //DEBUG_PRINT(" ");
  //DEBUG_PRINTLN2(speed_data_uint8_ts[1],BIN);
  
  p.setRepeatCount(REPEAT_COUNT_CONTINOUS);
  
  p.dccSetKind(DCC_SPEED_PACKET_KIND);
  
  //speed packets get refreshed indefinitely, and so the repeat doesn't need to be set.
  //speed packets go to the high proirity queue
  bool result = packet_buffer.insertPacket(p);
#ifdef DEBUG
  packet_buffer.printQueue();
#endif
  return result;
}

bool TrackPacketScheduler::dccSetFunctions(uint16_t address, ADDRESS_KIND dcc_address_kind, uint16_t functions)
{
//  DEBUG_PRINTLN(functions,HEX);
  if(dccSetFunctions0to4(address, dcc_address_kind, functions&0x1F))
    if(dccSetFunctions5to8(address, dcc_address_kind, (functions>>5)&0x0F))
      if(dccSetFunctions9to12(address, dcc_address_kind, (functions>>9)&0x0F))
        return true;
  return false;
}

bool TrackPacketScheduler::dccSetFunctions(uint16_t address, ADDRESS_KIND dcc_address_kind, uint8_t F0to4, uint8_t F5to8, uint8_t F9to12)
{
  if(dccSetFunctions0to4(address, dcc_address_kind, F0to4))
    if(dccSetFunctions5to8(address, dcc_address_kind, F5to8))
      if(dccSetFunctions9to12(address, dcc_address_kind, F9to12))
        return true;
  return false;
}

bool TrackPacketScheduler::dccSetFunctions0to4(uint16_t address, ADDRESS_KIND dcc_address_kind, uint8_t functions)
{
//  DEBUG_PRINTLN("dccSetFunctions0to4");
//  DEBUG_PRINTLN(functions,HEX);
  TrackPacket p(TRACK_PROTOCOL_DCC);
  p.dccSetAddress(address, dcc_address_kind);

  uint8_t data[] = {0x80};
  
  //Obnoxiously, the headlights (F0, AKA FL) are not controlled
  //by bit 0, but by bit 4. Really?
  
  //get functions 1,2,3,4
  data[0] |= (functions>>1) & 0x0F;
  //get functions 0
  data[0] |= (functions&0x01) << 4;

  p.dccAddData(data,1);
  p.dccSetKind(DCC_FUNCTION_PACKET_1_KIND);
  p.setRepeatCount(FUNCTION_REPEAT);
  return packet_buffer.insertPacket(p);
}


bool TrackPacketScheduler::dccSetFunctions5to8(uint16_t address, ADDRESS_KIND dcc_address_kind, uint8_t functions)
{
//  DEBUG_PRINTLN("dccSetFunctions5to8");
//  DEBUG_PRINTLN(functions,HEX);
  TrackPacket p(TRACK_PROTOCOL_DCC);
  p.dccSetAddress(address, dcc_address_kind);
  uint8_t data[] = {0xB0};
  
  data[0] |= functions & 0x0F;
  
  p.dccAddData(data,1);
  p.dccSetKind(DCC_FUNCTION_PACKET_2_KIND);
  p.setRepeatCount(FUNCTION_REPEAT);
  return packet_buffer.insertPacket(p);
}

bool TrackPacketScheduler::dccSetFunctions9to12(uint16_t address, ADDRESS_KIND dcc_address_kind, uint8_t functions)
{
//  DEBUG_PRINTLN("dccSetFunctions9to12");
//  DEBUG_PRINTLN(functions,HEX);
  TrackPacket p(TRACK_PROTOCOL_DCC);
  p.dccSetAddress(address, dcc_address_kind);
  uint8_t data[] = {0xA0};
  
  //least significant four functions (F5--F8)
  data[0] |= functions & 0x0F;
  
  p.dccAddData(data,1);
  p.dccSetKind(DCC_FUNCTION_PACKET_3_KIND);
  p.setRepeatCount(FUNCTION_REPEAT);
  return packet_buffer.insertPacket(p);
}


//other cool functions to follow. Just get these working first, I think.

//bool TrackPacketScheduler::setTurnout(uint16_t address)
//bool TrackPacketScheduler::unsetTurnout(uint16_t address)

bool TrackPacketScheduler::dccProgramCV(uint16_t address, ADDRESS_KIND dcc_address_kind, uint16_t CV, uint8_t CV_data)
{
  //format of packet:
  // {preamble} 0 [ AAAAAAAA ] 0 111011VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (write)
  // {preamble} 0 [ AAAAAAAA ] 0 111001VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (verify)
  // {preamble} 0 [ AAAAAAAA ] 0 111010VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (bit manipulation)
  // only concerned with "write" form here.
  
  TrackPacket p(TRACK_PROTOCOL_DCC);
  p.dccSetAddress(address, dcc_address_kind);

  uint8_t data[] = {0xEC, 0x00, 0x00};
  
  // split the CV address up among data uint8_ts 0 and 1
  data[0] |= ((CV-1) & 0x3FF) >> 8;
  data[1] = (CV-1) & 0xFF;
  data[2] = CV_data;
  
  p.dccAddData(data,3);
  p.dccSetKind(DCC_OPS_MODE_PROGRAMMING_KIND);
  p.setRepeatCount(OPS_MODE_PROGRAMMING_REPEAT);
  
  return packet_buffer.insertPacket(p);
}
    
//more specific functions

//broadcast e-stop command
bool TrackPacketScheduler::dcc_eStop(void)
{
    // 111111111111 0 00000000 0 01DC0001 0 EEEEEEEE 1
    TrackPacket e_stop_packet(TRACK_PROTOCOL_DCC); //address 0
    uint8_t data[] = {0x71}; //01110001
    e_stop_packet.dccAddData(data,1);
    e_stop_packet.dccSetKind(DCC_E_STOP_PACKET_KIND);
    e_stop_packet.setRepeatCount(10);
  #if 0
    e_stop_packet_buffer.insertPacket(e_stop_packet);
  #endif
    //now, clear all other queues
    packet_buffer.clear();
  #if 0
    repeat_packet_buffer.clear();
  #endif
    return true;
}
    
bool TrackPacketScheduler::dcc_eStop(uint16_t address, ADDRESS_KIND dcc_address_kind)
{
  // 111111111111 0	0AAAAAAA 0 01001001 0 EEEEEEEE 1
  // or
  // 111111111111 0	0AAAAAAA 0 01000001 0 EEEEEEEE 1
  TrackPacket e_stop_packet(TRACK_PROTOCOL_DCC);
  e_stop_packet.dccSetAddress(address, dcc_address_kind);

  uint8_t data[] = {0x41}; //01000001

  e_stop_packet.dccAddData(data,1);
  e_stop_packet.dccSetKind(DCC_E_STOP_PACKET_KIND);
  e_stop_packet.setRepeatCount(10);
  e_stop_packet.setPriority(HIGH_PRIORIY);
  
  packet_buffer.forget(address, dcc_address_kind);
#ifdef DEBUG
  packet_buffer.printQueue();
#endif
  bool result =  packet_buffer.insertPacket(e_stop_packet);
#ifdef DEBUG
  packet_buffer.printQueue();
#endif
  return result;
}

bool TrackPacketScheduler::dccSetBasicAccessory(uint16_t address, uint8_t function)
{
    TrackPacket p(TRACK_PROTOCOL_DCC);
    p.dccSetAddress(address);

	  uint8_t data[] = { 0x01 | ((function & 0x03) << 1) };
	  p.dccAddData(data, 1);
	  p.dccSetKind(DCC_BASIC_ACCESSORY_PACKET_KIND);
	  p.setRepeatCount(OTHER_REPEAT);

	  return packet_buffer.insertPacket(p);
}

bool TrackPacketScheduler::dccUnsetBasicAccessory(uint16_t address, uint8_t function)
{
  TrackPacket p(TRACK_PROTOCOL_DCC);
  p.dccSetAddress(address);

  uint8_t data[] = { ((function & 0x03) << 1) };
  p.dccAddData(data, 1);
  p.dccSetKind(DCC_BASIC_ACCESSORY_PACKET_KIND);
  p.setRepeatCount(OTHER_REPEAT);

  return packet_buffer.insertPacket(p);
}


bool TrackPacketScheduler::mm1SetSpeed(uint16_t address, int8_t new_speed){
  Serial.printf("MM1 Set speed--> Address = %i, speed = %i\n" , address, new_speed);
  int8_t speed;

  speed = new_speed < 0 ? -new_speed : new_speed; // nevre go for a reverse speed
  TrackPacket p(TRACK_PROTOCOL_MM);
  p.mmSetKind(MM1_LOC_SPEED_TELEGRAM);
  p.mmSetAddress(address);
  p.mmSetSpeed(speed);
  p.setRepeatCount(REPEAT_COUNT_CONTINOUS);
  return packet_buffer.insertPacket(p);
}

bool TrackPacketScheduler::mm1ChangeDir(uint16_t address){
  TrackPacket p(TRACK_PROTOCOL_MM);
  p.mmSetKind(MM1_LOC_CHANGE_DIR_TELEGRAM);
  p.mmSetAddress(address);
  p.setRepeatCount(1);
  return packet_buffer.insertPacket(p);
}


bool TrackPacketScheduler::mm2SetSpeed(uint16_t address, int8_t new_speed){
  Serial.printf("MM2 Set speed--> Address = %i, speed = %i\n" , address, new_speed);
  TrackPacket p(TRACK_PROTOCOL_MM);
  p.mmSetKind(MM2_LOC_SPEED_TELEGRAM);
  p.mmSetAddress(address);
  p.mmSetSpeed(new_speed);
  p.setRepeatCount(REPEAT_COUNT_CONTINOUS);
  return packet_buffer.insertPacket(p);
}

bool TrackPacketScheduler::mmSetSolenoid(uint16_t address, bool state){
  //Serial.printf("set solenoid\n");
  TrackPacket p(TRACK_PROTOCOL_MM);
  p.mmSetKind(MM_SOLENOID_TELEGRAM);
  p.mmSetAddress((address/4) + 1);
  //Serial.printf("State %i\n", state);
  //Serial.printf("set solenoid address = %i\n", (address/4) + 1);
  u_int8_t sub_address = ((address%4) * 2) + (state?1:0); // her gaat nog iets mis
  p.mmSetSolenoidSubaddress(sub_address);
  //Serial.printf("set solenoid sub address = %i\n", sub_address); // begrijp dit nog niet bool schijnt int te zijn
  p.mmSetSolenoidState(state);
  p.setRepeatCount(4);
  return packet_buffer.insertPacket(p);
}


#define MAX_NUMBER_OF_MM_FUNCTIONS     5


bool TrackPacketScheduler::mmSetFunctions(uint16_t address, uint8_t functions){
  uint8_t mask = 1;
  TrackPacket p(TRACK_PROTOCOL_MM);

  for(int i = 0; i <= MAX_NUMBER_OF_MM_FUNCTIONS; i++){
    switch(i){
      case 0:
        p.mmSetKind(MM_LOC_AUXILIARY_TELEGRAM);
        p.mmSetAddress(address);
        p.mmSetAuxiliary(functions & mask);
        break;
      case 1:
        p.mmSetKind(MM2_LOC_F1_TELEGRAM);
        p.mmSetAddress(address);
        p.mmSetFunction(functions & mask);
        break;
      case 2:
        p.mmSetKind(MM2_LOC_F2_TELEGRAM);
        p.mmSetAddress(address);
        p.mmSetFunction(functions & mask);
        break;
      case 3:
        p.mmSetKind(MM2_LOC_F3_TELEGRAM);
        p.mmSetAddress(address);
        p.mmSetFunction(functions & mask);
        break;
      case 4:
        p.mmSetKind(MM2_LOC_F4_TELEGRAM);
        p.mmSetAddress(address);
        p.mmSetFunction(functions & mask);
        break;

    }
    p.setRepeatCount(REPEAT_COUNT_CONTINOUS);
    packet_buffer.insertPacket(p);
    mask = mask << 1;
  }
  return false; 
}

//to be called periodically within loop()
void TrackPacketScheduler::update() //checks queues, puts whatever's pending on the rails via global dcc_bitstream. easy-peasy
{
  if(track.requestNewPacket()){

    TrackPacket p(TRACK_PROTOCOL_UNKNOWN);
    uint8_t dcc_bitstream_size;
    uint8_t dcc_bitstream[6];
    uint32_t mm2_bitstream;
    bool mm2_double_frequency;

#ifdef DEBUG_QUEUE
    if(packet_buffer.isEmpty())
      Serial.printf("Buffer empty\n");
    else
      packet_buffer.printQueue();
#endif
    if(packet_buffer.isEmpty()) return;
    packet_buffer.readPacket(p);

    TRACK_PROTOCOL track_protocol = p.getPacketProtocol();
    switch(track_protocol){
      case TRACK_PROTOCOL_DCC:
        dcc_bitstream_size = p.dccGetBitstream(dcc_bitstream);
        //DEBUG_PRINT("dcc_bitstream_size = %i\n", dcc_bitstream_size);
        //Serial.printf("Fill dcc\n");
        track.RMTfillDataDcc(dcc_bitstream, dcc_bitstream_size);
        break;
      case TRACK_PROTOCOL_MM:
        p.mm2GetBitstream(&mm2_bitstream, &mm2_double_frequency);
        //Serial.printf("Fill mm\n");
        track.RMTfillDataMM(mm2_bitstream, mm2_double_frequency);
        break;  

    }
  }
}
