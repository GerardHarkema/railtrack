#ifndef __DCCCOMMANDSTATION_H__
#define __DCCCOMMANDSTATION_H__
#include "TrackPacket.h"
#include "TrackPacketQueueList.h"

#define E_STOP_QUEUE_SIZE           2
#define HIGH_PRIORITY_QUEUE_SIZE    10
#define LOW_PRIORITY_QUEUE_SIZE     10
#define REPEAT_QUEUE_SIZE           10
//#define PERIODIC_REFRESH_QUEUE_SIZE 10

#define LOW_PRIORITY_INTERVAL     5
#define REPEAT_INTERVAL           11
#define PERIODIC_REFRESH_INTERVAL 23

#define SPEED_REPEAT      3
#define FUNCTION_REPEAT   3
#define E_STOP_REPEAT     5
#define OPS_MODE_PROGRAMMING_REPEAT 3
#define OTHER_REPEAT      2


class TrackPacketScheduler
{
  public:
  
    TrackPacketScheduler(void);

    TrackPacketQueue packet_buffer;

    //for configuration
    void setDefaultSpeedSteps(uint8_t new_speed_steps);
    bool setup(void); //for any post-constructor initialization


    bool trackPower(bool enable);
   
    //for enqueueing packets
    bool dccSetSpeed(uint16_t address, ADDRESS_KIND dcc_address_kind, int8_t new_speed, uint8_t steps = 0); //new_speed: [-127,127]
    bool dccSetSpeed14(uint16_t address, ADDRESS_KIND dcc_address_kind, int8_t new_speed, bool F0=true); //new_speed: [-13,13], and optionally F0 settings.
    bool dccSetSpeed28(uint16_t address, ADDRESS_KIND dcc_address_kind, int8_t new_speed); //new_speed: [-28,28]
    bool dccSetSpeed128(uint16_t address, ADDRESS_KIND dcc_address_kind, int8_t new_speed); //new_speed: [-127,127]
    
    //the function methods are NOT stateful; you must specify all functions each time you call one
    //keeping track of function state is the responsibility of the calling program.
    bool dccSetFunctions(uint16_t address, ADDRESS_KIND dcc_address_kind, uint8_t F0to4, uint8_t F5to9=0x00, uint8_t F9to12=0x00);
    bool dccSetFunctions(uint16_t address, ADDRESS_KIND dcc_address_kind, uint16_t functions);
    bool dccSetFunctions0to4(uint16_t address, ADDRESS_KIND dcc_address_kind, uint8_t functions);
    bool dccSetFunctions5to8(uint16_t address, ADDRESS_KIND dcc_address_kind, uint8_t functions);
    bool dccSetFunctions9to12(uint16_t address, ADDRESS_KIND dcc_address_kind, uint8_t functions);
    //other cool functions to follow. Just get these working first, I think.
    
    bool dccSetBasicAccessory(uint16_t address, uint8_t function);
    bool dccUnsetBasicAccessory(uint16_t address, uint8_t function);
    
    bool dccProgramCV(uint16_t address, ADDRESS_KIND dcc_address_kind, uint16_t CV, uint8_t CV_data);

    //more specific functions
    bool dcc_eStop(void); //all locos
    bool dcc_eStop(uint16_t address, ADDRESS_KIND dcc_address_kind); //just one specific loco
#if THREAD_SAFE_QUEUE 
    static void scheduler_task(void *pvParameters);
#endif


    bool mm1SetSpeed(uint16_t address, int8_t new_speed); //new_speed: [0,28]
    bool mm1ChangeDir(uint16_t address);
    bool mm2SetSpeed(uint16_t address, int8_t new_speed); //new_speed: [0, 28]
    bool mmSetBasicAccessory(uint16_t address, uint8_t function);
    bool mmUnsetBasicAccessory(uint16_t address, uint8_t function);
    bool mmSetFunctions(uint16_t address, uint8_t function);

    //to be called periodically within loop()
    void update(void); //checks queues, puts whatever's pending on the rails via global dcc_bitstream. easy-peasy


  //private:
  
  //  void stashAddress(TrackPacket *p); //remember the address to compare with the next packet
    TaskHandle_t scheduler_task_h = NULL;
    uint8_t default_speed_steps;
    uint16_t last_packet_address;
  
    uint8_t packet_counter;

  
//    TrackPacketQueue *queue;
};

//TrackPacketScheduler packet_scheduler;

#endif //__DCC_COMMANDSTATION_H__
