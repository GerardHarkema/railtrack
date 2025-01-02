#ifndef __DCCPACKETQUEUELIST_H__
#define __DCCPACKETQUEUELIST_H__

#include "Arduino.h"

#include "TrackPacket.h"
#include <list>

class TrackPacketQueue
{
  public: //protected:

    std::list<TrackPacket> queue;

    bool queue_full = false; 
    SemaphoreHandle_t semaphore = NULL;

  public:
    TrackPacketQueue(void);
    
    bool setup();
 
    ~TrackPacketQueue(void)
    {

    }

    
    inline bool isFull(void)
    {
      return queue_full;
    }
    inline bool isEmpty(void)
    {
      return (queue.size() == 0);
    }
    inline bool notEmpty(void)
    {
      return (queue.size());
    }
    
    inline bool notRepeat(unsigned int address)
    {
      #if 0
      return (address != read_pos->dccGetAddress());
      #endif
      return false;
    }
    
    //void printQueue(void);
    
    bool insertPacket(TrackPacket &packet); //makes a local copy, does not take over memory management!
    bool readPacket(TrackPacket &packet); //does not hand off memory management of packet. used immediately.
    void printQueue(void);   
    bool forget(uint16_t address, uint8_t dcc_address_kind);
    void clear(void);
};

#endif //__DCCPACKETQUEUELIST_H__
