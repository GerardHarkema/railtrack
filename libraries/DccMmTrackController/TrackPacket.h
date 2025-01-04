#ifndef __DCCPACKET_H__
#define __DCCPACKET_H__

#include "Arduino.h"

typedef unsigned char uint8_t;

//Packet kinds
// enum packet_kind_t {
//   DCC_IDLE_PACKET_KIND,
//   DCC_E_STOP_PACKET_KIND,
//   DCC_SPEED_PACKET_KIND,
//   function_packet_kind,
//   DCC_ACCESSORY_PACKET_KIND,
//   DCC_RESET_PACKET_KIND,
//   DCC_OPS_MODE_PROGRAMMING_KIND,
//   other_packet_kind
// };

#define MULTIFUNCTION_PACKET_KIND_MASK 0x10
#if 0
#define DCC_IDLE_PACKET_KIND            0x10
#define DCC_E_STOP_PACKET_KIND          0x11
#define DCC_SPEED_PACKET_KIND           0x12
#define DCC_FUNCTION_PACKET_1_KIND      0x13
#define DCC_FUNCTION_PACKET_2_KIND      0x14
#define DCC_FUNCTION_PACKET_3_KIND      0x15
#define DCC_ACCESSORY_PACKET_KIND       0x16
#define DCC_RESET_PACKET_KIND           0x17
#define DCC_OPS_MODE_PROGRAMMING_KIND   0x18
#else

typedef enum {
  DCC_IDLE_PACKET_KIND  = 0x10,
  DCC_E_STOP_PACKET_KIND,
  DCC_SPEED_PACKET_KIND,
  DCC_FUNCTION_PACKET_1_KIND,
  DCC_FUNCTION_PACKET_2_KIND,
  DCC_FUNCTION_PACKET_3_KIND,
  //DCC_ACCESSORY_PACKET_KIND,
  DCC_RESET_PACKET_KIND,
  DCC_OPS_MODE_PROGRAMMING_KIND,

  DCC_BASIC_ACCESSORY_PACKET_KIND  = 0x40,
  DCC_EXTEND_ACCESSORY_PACKET_KIND 
} DCC_KIND_TYPE;
#endif

#define DCC_ACCESSORY_PACKET_KIND_MASK 0x40
//#define DCC_BASIC_ACCESSORY_PACKET_KIND 0x40
//#define EXTEND_DCC_ACCESSORY_PACKET_KIND 0x41

#define other_packet_kind           0x00

#if 0
#define DCC_SHORT_ADDRESS           0x00
#define DCC_LONG_ADDRESS            0x01
#endif

typedef enum{
  DCC_SHORT_ADDRESS,
  DCC_LONG_ADDRESS
}ADDRESS_KIND;

typedef enum{
  TRACK_PROTOCOL_DCC,
  TRACK_PROTOCOL_MM,
  TRACK_PROTOCOL_UNKNOWN
}TRACK_PROTOCOL;

typedef enum{
  MM1_LOC_SPEED_TELEGRAM,
  MM1_LOC_CHANGE_DIR_TELEGRAM,
  MM1_LOC_F_TELEGRAM,
  MM2_LOC_SPEED_TELEGRAM,
  MM_LOC_AUXILIARY_TELEGRAM, // Fake telegram
  MM2_LOC_F1_TELEGRAM,
  MM2_LOC_F2_TELEGRAM,
  MM2_LOC_F3_TELEGRAM,
  MM2_LOC_F4_TELEGRAM,
  MM_SOLENOID_TELEGRAM
}MM_KIND_TYPE;


#define HIGH_PRIORIY    true
#define LOW_PRIORIY     false
#define REPEAT_COUNT_CONTINOUS  -1

//A DCC packet is at most 6 uint8_ts: 2 of address, three of dcc_data, one of XOR
typedef struct{
    uint16_t address;
    ADDRESS_KIND address_kind;
    uint8_t data[3];
    uint8_t size;  
    DCC_KIND_TYPE kind;
}DCC_DATA;

typedef struct{
    MM_KIND_TYPE kind;
    int kind_sequence_index;
    uint8_t address;
    uint8_t molenoid_sub_address;
    bool solenoid_state;
    int8_t speed;
    bool function_on;
    bool function_on_[4];
    bool auxiliary;
    uint32_t data;
}MM2_DATA;


class TrackPacket
{
  private:

    TRACK_PROTOCOL track_protocol;
    DCC_DATA dcc_data;
    MM2_DATA mm_data;
    int8_t repeat_count; 
    bool priority;
    
  public:
    TrackPacket(TRACK_PROTOCOL track_protocol);
    
    uint8_t dccGetBitstream(uint8_t rawuint8_ts[]); //returns size of bitstream
    //uint8_t dccGetSize(void);
    inline uint16_t dccGetAddress(void) { return dcc_data.address; }
    inline ADDRESS_KIND dccGetAddressKind(void) { return dcc_data.address_kind; }
    inline void dccSetAddress(uint16_t new_address) { dcc_data.address = new_address; }
    inline void dccSetAddress(uint16_t new_address, ADDRESS_KIND new_address_kind) { dcc_data.address = new_address; dcc_data.address_kind = new_address_kind; }
    void dccAddData(uint8_t *new_data, uint8_t new_size); //insert freeform dcc_data.
    inline uint8_t dccGetData(uint8_t index){return dcc_data.data[index];}; //insert freeform dcc_data.
    inline uint8_t dccGetSize(void) { return dcc_data.size ; }
    inline void dccSetKind(DCC_KIND_TYPE new_kind) { dcc_data.kind = new_kind; }
    inline DCC_KIND_TYPE dccGetKind(void) { return dcc_data.kind; }


    void mm2GetBitstream(uint32_t *bitstream, bool *double_frequency); // fixed size bitstream

    inline void mmSetKind(MM_KIND_TYPE kind) { mm_data.kind = kind; }
    inline MM_KIND_TYPE mmGetKind(void) { return mm_data.kind; }
    void mmSetNextKind(void);
    inline void mmSetAddress(uint8_t new_address) { mm_data.address = new_address; }
    inline uint8_t mmGetAddress(void) { return mm_data.address; }
    inline void mmSetSpeed(uint8_t new_speed) { mm_data.speed = new_speed; }
    inline uint8_t mmGetSpeed(void) { return mm_data.speed; }
    inline void mmSetSolenoidSubaddress(uint8_t molenoid_sub_address) { mm_data.molenoid_sub_address = molenoid_sub_address; }
    inline uint8_t mmgetSolenoidSubaddress(void) { return mm_data.molenoid_sub_address; }
    inline void mmSetSolenoidState(bool solenoid_on) { mm_data.solenoid_state = solenoid_on; }
    inline bool mmGetSolenoidState(void) { return mm_data.solenoid_state; }
    inline void mmSetAuxiliary(bool auxiliary) { mm_data.auxiliary = auxiliary; }
    inline bool mmGetAuxiliary(void) { return mm_data.auxiliary; }
    inline void mmSetFunction(bool function_on) { mm_data.function_on = function_on; }
    inline bool mmGetFunction(void) { return mm_data.function_on; }
    inline void mmSetFunction(u_int8_t index, bool function_on) { mm_data.function_on_[index] = function_on; }
    inline bool mmGetFunction(u_int8_t index) { return mm_data.function_on_[index]; }

    inline void setRepeatCount(uint8_t new_repeat_count) { repeat_count = new_repeat_count;}
    inline uint8_t getRepeatCount(void) { return repeat_count; }//return repeat
    inline bool isHighPriority(void) { return priority; }
    inline void setPriority(bool new_priority) {  priority = new_priority; }
    inline TRACK_PROTOCOL getPacketProtocol(void){return track_protocol;}
};

#endif //__DCCPACKET_H__
