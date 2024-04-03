#ifndef __DCCPACKETQUEUELIST_H__
#define __DCCPACKETQUEUELIST_H__

#include "Arduino.h"

/**
 * A FIFO queue for holding DCC packets, implemented as a circular buffer.
 * Copyright 2010 D.E. Goodman-Wilson
 * TODO
**/

#include "DCCPacket.h"
#include <list>

class DCCPacketQueue
{
  public: //protected:

    std::list<DCCPacket> queue;

    bool queue_full = false; 
    SemaphoreHandle_t semaphore = NULL;

  public:
    DCCPacketQueue(void);
    
    bool setup();
 
    ~DCCPacketQueue(void)
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
      return (address != read_pos->getAddress());
      #endif
      return false;
    }
    
    //void printQueue(void);
    
    bool insertPacket(DCCPacket &packet); //makes a local copy, does not take over memory management!
    bool readPacket(DCCPacket &packet); //does not hand off memory management of packet. used immediately.
    void printQueue(void);   
    bool forget(uint16_t address, uint8_t address_kind);
    void clear(void);
};

#endif //__DCCPACKETQUEUELIST_H__
