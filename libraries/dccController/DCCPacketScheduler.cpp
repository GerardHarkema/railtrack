#include "DCCPacketScheduler.h"
#include "TrackManager.h"

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



//#define DEBUG     
#ifdef DEBUG
  #define DEBUG_PRINT(x)      Serial.print(x)
  #define DEBUG_PRINT2(x, y)  Serial.print(x, y)
  #define DEBUG_PRINTLN(x)    Serial.println(x)
  #define DEBUG_PRINTLN2(x, y)    Serial.println(x, y)
#else
  #define DEBUG_PRINT(x)      
  #define DEBUG_PRINT2(x, y)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTLN2(x, y) 
#endif

/// The currently queued packet to be put on the rails. Default is a reset packet.
//extern uint8_t current_packet[6];
/// How many data uint8_ts in the queued packet?
//extern volatile uint8_t current_packet_size;
/// How many uint8_ts remain to be put on the rails?
//extern volatile uint8_t current_uint8_t_counter;
/// How many bits remain in the current data uint8_t/preamble before changing states?
//extern volatile uint8_t current_bit_counter; //init to 14 1's for the preamble
/// A fixed-content packet to send when idle
//uint8_t DCC_Idle_Packet[3] = {255,0,255};
/// A fixed-content packet to send to reset all decoders on layout
//uint8_t DCC_Reset_Packet[3] = {0,0,0};

//waveform_generator_class waveform_generator;

TrackManager track;

///////////////////////////////////////////////
///////////////////////////////////////////////
///////////////////////////////////////////////
  
DCCPacketScheduler::DCCPacketScheduler(void) : default_speed_steps(128), last_packet_address(255), packet_counter(1)
{

  //packet_buffer = new DCCPacketQueue();
  packet_buffer.setup();

}
    
//for configuration
void DCCPacketScheduler::setDefaultSpeedSteps(uint8_t new_speed_steps)
{
  
  default_speed_steps = new_speed_steps;
}


void DCCPacketScheduler::scheduler_task(void *pvParameters)
{
  DCCPacketScheduler *instance = static_cast<DCCPacketScheduler*>(pvParameters);
	while(1)
	{
    vTaskDelay(10);
    instance->update();
		//taskYIELD();
	}
}

bool DCCPacketScheduler::setup(void) //for any post-constructor initialization
{
  
  //Following RP 9.2.4, begin by putting 20 reset packets and 10 idle packets on the rails.
  //use the e_stop_queue to do this, to ensure these packets go out first!

  //if(!packet_buffer) return false;
  if(!packet_buffer.setup()) return false;
  //if(!setup_DCC_waveform_generator()) return false;

  //waveform_generator.setup_DCC_waveform_generator();

  DCCPacket p;
  uint8_t data[] = {0x00};
  
  //reset packet: address 0x00, data 0x00, XOR 0x00; S 9.2 line 75
  p.addData(data,1);
  p.setAddress(0x00, DCC_SHORT_ADDRESS);
  p.setRepeatCount(20);
  p.setKind(reset_packet_kind);
  p.setPriority(HIGH_PRIORIY);
  packet_buffer.insertPacket(p);
  
  //WHy in the world is it that what gets put on the rails is 4 reset packets, followed by
  //10 god know's what, followed by something else?
  // C0 FF 00 FF
  // 00 FF FF   what are these?
  
  //idle packet: address 0xFF, data 0x00, XOR 0xFF; S 9.2 line 90
  p.setAddress(0xFF, DCC_SHORT_ADDRESS);
  p.setRepeatCount(10);
  p.setKind(idle_packet_kind);
  p.setPriority(HIGH_PRIORIY);

  packet_buffer.insertPacket(p); //e_stop_queue will be empty, so no need to check if insertion was OK.

  packet_buffer.printQueue();

#if 1
  int app_cpu = xPortGetCoreID();
  xTaskCreatePinnedToCore(scheduler_task,
                        "scheduler_task", 
                        4096,
                        this,
                        1,
                        &scheduler_task_h,
                        app_cpu);
  if(!scheduler_task_h){
    return false;
  }
#endif  
  track.begin();
  return true;
}

bool DCCPacketScheduler::trackPower(bool enable){
  if(enable) return track.enableTrackPower();
  else return track.disableTrackPower();
}
    
//for enqueueing packets

//setSpeed* functions:
//new_speed contains the speed and direction.
// a value of 0 = estop
// a value of 1/-1 = stop
// a value >1 (or <-1) means go.
// valid non-estop speeds are in the range [1,127] / [-127,-1] with 1 = stop
bool DCCPacketScheduler::setSpeed(uint16_t address, uint8_t address_kind, int8_t new_speed, uint8_t steps)
{
  uint8_t num_steps = steps;
  //steps = 0 means use the default; otherwise use the number of steps specified
  if(!steps)
    num_steps = default_speed_steps;
        
  switch(num_steps)
  {
    case 14:
      return(setSpeed14(address, address_kind, new_speed));
    case 28:
      return(setSpeed28(address, address_kind, new_speed));
    case 128:
      return(setSpeed128(address, address_kind, new_speed));
  }
  return false; //invalid number of steps specified.
}

bool DCCPacketScheduler::setSpeed14(uint16_t address, uint8_t address_kind, int8_t new_speed, bool F0)
{
  DCCPacket p(address, address_kind);
  uint8_t dir = 1;
  uint8_t speed_data_uint8_ts[] = {0x40};
  uint16_t abs_speed = new_speed;
  if(new_speed<0)
  {
    dir = 0;
    abs_speed = new_speed * -1;
  }
  if(!new_speed) //estop!
    return eStop(address, speed_packet_kind);//speed_data_uint8_ts[0] |= 0x01; //estop
    
  else if (abs_speed == 1) //regular stop!
    speed_data_uint8_ts[0] |= 0x00; //stop
  else //movement
    speed_data_uint8_ts[0] |= map(abs_speed, 2, 127, 2, 15); //convert from [2-127] to [1-14]
  speed_data_uint8_ts[0] |= (0x20*dir); //flip bit 3 to indicate direction;
  DEBUG_PRINTLN2(speed_data_uint8_ts[0],BIN);
  p.addData(speed_data_uint8_ts,1);

  p.setRepeatCount(REPEAT_COUNT_CONTINOUS);
  
  p.setKind(speed_packet_kind);  

  //speed packets get refreshed indefinitely, and so the repeat doesn't need to be set.
  //speed packets go to the high proirity queue
  return(packet_buffer.insertPacket(p));
}

bool DCCPacketScheduler::setSpeed28(uint16_t address, uint8_t address_kind, int8_t new_speed)
{
  DCCPacket p(address, address_kind);
  uint8_t dir = 1;
  uint8_t speed_data_uint8_ts[] = {0x40};
  uint16_t abs_speed = new_speed;
  if(new_speed<0)
  {
    dir = 0;
    abs_speed = new_speed * -1;
  }
//  DEBUG_PRINTLN(speed);
//  DEBUG_PRINTLN(dir);
  if(new_speed == 0) //estop!
    return eStop(address, speed_packet_kind);//speed_data_uint8_ts[0] |= 0x01; //estop
  else if (abs_speed == 1) //regular stop!
    speed_data_uint8_ts[0] |= 0x00; //stop
  else //movement
  {
    speed_data_uint8_ts[0] |= map(abs_speed, 2, 127, 2, 0X1F); //convert from [2-127] to [2-31]  
    //most least significant bit has to be shufled around
    speed_data_uint8_ts[0] = (speed_data_uint8_ts[0]&0xE0) | ((speed_data_uint8_ts[0]&0x1F) >> 1) | ((speed_data_uint8_ts[0]&0x01) << 4);
  }
  speed_data_uint8_ts[0] |= (0x20*dir); //flip bit 3 to indicate direction;
//  DEBUG_PRINTLN(speed_data_uint8_ts[0],BIN);
//  DEBUG_PRINTLN("=======");
  p.addData(speed_data_uint8_ts,1);
  
  p.setRepeatCount(REPEAT_COUNT_CONTINOUS);
  
  p.setKind(speed_packet_kind);
    
  //speed packets get refreshed indefinitely, and so the repeat doesn't need to be set.
  //speed packets go to the high proirity queue
  //return(packet_buffer.insertPacket(p));
  return(packet_buffer.insertPacket(p));
}

bool DCCPacketScheduler::setSpeed128(uint16_t address, uint8_t address_kind, int8_t new_speed)
{
  //why do we get things like this?
  // 03 3F 16 15 3F (speed packet addressed to loco 03)
  // 03 3F 11 82 AF  (speed packet addressed to loco 03, speed hex 0x11);

  DCCPacket p(address, address_kind);

  uint8_t dir = 1;
  uint16_t abs_speed = new_speed;
  uint8_t speed_data_uint8_ts[] = {0x3F,0x00};
  if(new_speed<0)
  {
    dir = 0;
    abs_speed = new_speed * -1;
  }

  if(abs_speed > 0x7f)abs_speed = 0x7f;

  if(!new_speed){ //estop!
    //packet_buffer.forget(address, speed_packet_kind);
    //packet_buffer.printQueue();
    //return true;
    return eStop(address, speed_packet_kind);//speed_data_uint8_ts[0] |= 0x01; //estop
  }
  else 
  if (abs_speed == 1) //regular stop!
    speed_data_uint8_ts[1] = 0x00; //stop
  else //movement
    speed_data_uint8_ts[1] = abs_speed; //no conversion necessary.

  speed_data_uint8_ts[1] |= (0x80*dir); //flip bit 7 to indicate direction;
  p.addData(speed_data_uint8_ts,2);
  DEBUG_PRINT2(speed_data_uint8_ts[0],BIN);
  DEBUG_PRINT(" ");
  DEBUG_PRINTLN2(speed_data_uint8_ts[1],BIN);
  
  p.setRepeatCount(REPEAT_COUNT_CONTINOUS);
  
  p.setKind(speed_packet_kind);
  
  //speed packets get refreshed indefinitely, and so the repeat doesn't need to be set.
  //speed packets go to the high proirity queue
  bool result = packet_buffer.insertPacket(p);
        packet_buffer.printQueue();
  return result;
}

bool DCCPacketScheduler::setFunctions(uint16_t address, uint8_t address_kind, uint16_t functions)
{
//  DEBUG_PRINTLN(functions,HEX);
  if(setFunctions0to4(address, address_kind, functions&0x1F))
    if(setFunctions5to8(address, address_kind, (functions>>5)&0x0F))
      if(setFunctions9to12(address, address_kind, (functions>>9)&0x0F))
        return true;
  return false;
}

bool DCCPacketScheduler::setFunctions(uint16_t address, uint8_t address_kind, uint8_t F0to4, uint8_t F5to8, uint8_t F9to12)
{
  if(setFunctions0to4(address, address_kind, F0to4))
    if(setFunctions5to8(address, address_kind, F5to8))
      if(setFunctions9to12(address, address_kind, F9to12))
        return true;
  return false;
}

bool DCCPacketScheduler::setFunctions0to4(uint16_t address, uint8_t address_kind, uint8_t functions)
{
//  DEBUG_PRINTLN("setFunctions0to4");
//  DEBUG_PRINTLN(functions,HEX);
  DCCPacket p(address, address_kind);
  uint8_t data[] = {0x80};
  
  //Obnoxiously, the headlights (F0, AKA FL) are not controlled
  //by bit 0, but by bit 4. Really?
  
  //get functions 1,2,3,4
  data[0] |= (functions>>1) & 0x0F;
  //get functions 0
  data[0] |= (functions&0x01) << 4;

  p.addData(data,1);
  p.setKind(function_packet_1_kind);
  p.setRepeatCount(FUNCTION_REPEAT);
  return packet_buffer.insertPacket(p);
}


bool DCCPacketScheduler::setFunctions5to8(uint16_t address, uint8_t address_kind, uint8_t functions)
{
//  DEBUG_PRINTLN("setFunctions5to8");
//  DEBUG_PRINTLN(functions,HEX);
  DCCPacket p(address, address_kind);
  uint8_t data[] = {0xB0};
  
  data[0] |= functions & 0x0F;
  
  p.addData(data,1);
  p.setKind(function_packet_2_kind);
  p.setRepeatCount(FUNCTION_REPEAT);
  return packet_buffer.insertPacket(p);
}

bool DCCPacketScheduler::setFunctions9to12(uint16_t address, uint8_t address_kind, uint8_t functions)
{
//  DEBUG_PRINTLN("setFunctions9to12");
//  DEBUG_PRINTLN(functions,HEX);
  DCCPacket p(address, address_kind);
  uint8_t data[] = {0xA0};
  
  //least significant four functions (F5--F8)
  data[0] |= functions & 0x0F;
  
  p.addData(data,1);
  p.setKind(function_packet_3_kind);
  p.setRepeatCount(FUNCTION_REPEAT);
  return packet_buffer.insertPacket(p);
}


//other cool functions to follow. Just get these working first, I think.

//bool DCCPacketScheduler::setTurnout(uint16_t address)
//bool DCCPacketScheduler::unsetTurnout(uint16_t address)

bool DCCPacketScheduler::opsProgramCV(uint16_t address, uint8_t address_kind, uint16_t CV, uint8_t CV_data)
{
  //format of packet:
  // {preamble} 0 [ AAAAAAAA ] 0 111011VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (write)
  // {preamble} 0 [ AAAAAAAA ] 0 111001VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (verify)
  // {preamble} 0 [ AAAAAAAA ] 0 111010VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (bit manipulation)
  // only concerned with "write" form here.
  
  DCCPacket p(address, address_kind);
  uint8_t data[] = {0xEC, 0x00, 0x00};
  
  // split the CV address up among data uint8_ts 0 and 1
  data[0] |= ((CV-1) & 0x3FF) >> 8;
  data[1] = (CV-1) & 0xFF;
  data[2] = CV_data;
  
  p.addData(data,3);
  p.setKind(ops_mode_programming_kind);
  p.setRepeatCount(OPS_MODE_PROGRAMMING_REPEAT);
  
  return packet_buffer.insertPacket(p);
}
    
//more specific functions

//broadcast e-stop command
bool DCCPacketScheduler::eStop(void)
{
    // 111111111111 0 00000000 0 01DC0001 0 EEEEEEEE 1
    DCCPacket e_stop_packet(0); //address 0
    uint8_t data[] = {0x71}; //01110001
    e_stop_packet.addData(data,1);
    e_stop_packet.setKind(e_stop_packet_kind);
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
    
bool DCCPacketScheduler::eStop(uint16_t address, uint8_t address_kind)
{
  // 111111111111 0	0AAAAAAA 0 01001001 0 EEEEEEEE 1
  // or
  // 111111111111 0	0AAAAAAA 0 01000001 0 EEEEEEEE 1
  DCCPacket e_stop_packet(address, address_kind);
  uint8_t data[] = {0x41}; //01000001


  e_stop_packet.addData(data,1);
  e_stop_packet.setKind(e_stop_packet_kind);
  e_stop_packet.setRepeatCount(10);
  e_stop_packet.setPriority(HIGH_PRIORIY);
  
  packet_buffer.forget(address, address_kind);
  packet_buffer.printQueue();
  bool result =  packet_buffer.insertPacket(e_stop_packet);
  packet_buffer.printQueue();
  return result;
}

bool DCCPacketScheduler::setBasicAccessory(uint16_t address, uint8_t function)
{
    DCCPacket p(address);

	  uint8_t data[] = { 0x01 | ((function & 0x03) << 1) };
	  p.addData(data, 1);
	  p.setKind(basic_accessory_packet_kind);
	  p.setRepeatCount(OTHER_REPEAT);

	  return packet_buffer.insertPacket(p);
}

bool DCCPacketScheduler::unsetBasicAccessory(uint16_t address, uint8_t function)
{
  DCCPacket p(address);

  uint8_t data[] = { ((function & 0x03) << 1) };
  p.addData(data, 1);
  p.setKind(basic_accessory_packet_kind);
  p.setRepeatCount(OTHER_REPEAT);

  return packet_buffer.insertPacket(p);
}

void DCCPacketScheduler::EnableWaveformGeneration(){
  //waveform_generator.EnableWaveformGenerator();
}

//to be called periodically within loop()
void DCCPacketScheduler::update() //checks queues, puts whatever's pending on the rails via global current_packet. easy-peasy
{
  if(track.requestNewPacket()){

    DCCPacket p;
    uint8_t data_size;
    uint8_t current_packet[6];

    if(packet_buffer.isEmpty()) return;
    //packet_buffer.printQueue();
    packet_buffer.readPacket(p);

    data_size = p.getBitstream(current_packet);
    track.RMTfillData(current_packet, data_size);
  }

#if 0
  //Serial.print("s");
  if(!current_uint8_t_counter) //if the ISR needs a packet:
  {
    DCCPacket p;

    if(packet_buffer.notEmpty()){
      //packet_buffer.printQueue();
      packet_buffer.readPacket(p);
    }
  #if 1
    last_packet_address = p.getAddress(); //remember the address to compare with the next packet
    current_packet_size = p.getBitstream(current_packet); //feed to the starving ISR.
    //output the packet, for checking:
    //if(current_packet[0] != 0xFF) //if not idle
    //{
    //  for(uint8_t i = 0; i < current_packet_size; ++i)
    //  {
    //    DEBUG_PRINT(current_packet[i],BIN);
    //    DEBUG_PRINT(" ");
    //  }
    //  DEBUG_PRINTLN("");
    //}
    current_uint8_t_counter = current_packet_size;
    #endif
  }
#endif
}
