#ifndef __DCCPACKET_H__
#define __DCCPACKET_H__

#include "Arduino.h"

typedef unsigned char uint8_t;

//Packet kinds
// enum packet_kind_t {
//   idle_packet_kind,
//   e_stop_packet_kind,
//   speed_packet_kind,
//   function_packet_kind,
//   accessory_packet_kind,
//   reset_packet_kind,
//   ops_mode_programming_kind,
//   other_packet_kind
// };

#define MULTIFUNCTION_PACKET_KIND_MASK 0x10
#if 1
#define idle_packet_kind            0x10
#define e_stop_packet_kind          0x11
#define speed_packet_kind           0x12
#define function_packet_1_kind      0x13
#define function_packet_2_kind      0x14
#define function_packet_3_kind      0x15
#define accessory_packet_kind       0x16
#define reset_packet_kind           0x17
#define ops_mode_programming_kind   0x18
#else

typedef enum {
  idle_packet_kind  = 0x10,
  e_stop_packet_kind,
  speed_packet_kind,
  function_packet_1_kind,
  function_packet_2_kind,
  function_packet_3_kind,
  accessory_packet_kind,
  reset_packet_kind,
  ops_mode_programming_kind
} kind_type;
#endif

#define ACCESSORY_PACKET_KIND_MASK 0x40
#define basic_accessory_packet_kind 0x40
#define extended_accessory_packet_kind 0x41

#define other_packet_kind           0x00

#if 0
#define DCC_SHORT_ADDRESS           0x00
#define DCC_LONG_ADDRESS            0x01
#endif

typedef enum{
  DCC_SHORT_ADDRESS,
  DCC_LONG_ADDRESS
}address_type;

#define HIGH_PRIORIY    true
#define LOW_PRIORIY     false
#define REPEAT_COUNT_CONTINOUS  -1

class TrackPacket
{
  private:
   //A DCC packet is at most 6 uint8_ts: 2 of address, three of data, one of XOR
    uint16_t address;
    uint8_t address_kind;
    uint8_t data[3];
    int8_t repeat_count; 
    uint8_t size;  
    uint8_t kind;
    bool priority;
    
  public:
    TrackPacket(uint16_t decoder_address=0xFF, uint8_t decoder_address_kind=0x00);
    
    uint8_t getBitstream(uint8_t rawuint8_ts[]); //returns size of array.
    //uint8_t getSize(void);
    inline uint16_t getAddress(void) { return address; }
    inline uint8_t getAddressKind(void) { return address_kind; }
    inline void setAddress(uint16_t new_address) { address = new_address; }
    inline void setAddress(uint16_t new_address, uint8_t new_address_kind) { address = new_address; address_kind = new_address_kind; }
    void addData(uint8_t *new_data, uint8_t new_size); //insert freeform data.
    inline uint8_t getData(uint8_t index){return data[index];}; //insert freeform data.
    inline void setKind(uint8_t new_kind) { kind = new_kind; }
    inline uint8_t getKind(void) { return kind; }
    inline void setRepeatCount(uint8_t new_repeat_count) { repeat_count = new_repeat_count;}
    inline uint8_t getRepeatCount(void) { return repeat_count; }//return repeat; }
    inline uint8_t getSize(void) { return size ; }
    inline bool isHighPriority(void) { return priority; }
    inline void setPriority(bool new_priority) {  priority = new_priority; }
};

#endif //__DCCPACKET_H__
