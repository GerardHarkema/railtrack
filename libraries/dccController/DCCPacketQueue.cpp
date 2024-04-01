#ifndef QUEUE_LIST_TYPE
#include "DCCPacketQueue.h"

#define SEMAPHORE_TIMEOUT 10

DCCPacketQueue::DCCPacketQueue(void) : read_pos(0), write_pos(0), size(10), written(0)
{
  return;
}



void DCCPacketQueue::setup(byte length)
{
  semaphore = xSemaphoreCreateMutex();
  if(xSemaphoreTake(semaphore, SEMAPHORE_TIMEOUT) != pdTRUE){
    Serial.println("Error taking Semaphore");
    return;
  };

  size = length;
  queue = (DCCPacket *)malloc(sizeof(DCCPacket) *size);
  for(int i = 0; i<size; ++i)
  {
    queue[i] = DCCPacket();
  }
  xSemaphoreGive(semaphore);
}

bool DCCPacketQueue::insertPacketCtx(DCCPacket *packet)
{
//  Serial.print("Enqueueing a packet of kind: ");
//  Serial.println(packet->getKind(), DEC);
   //First: Overwrite any packet with the same address and kind; if no such packet THEN hitup the packet at write_pos

  bool result = false;

  byte i = read_pos;
  while(i != (read_pos+written)%(size) )//(size+1) ) //size+1 so we can check the last slot, too…
  {
    if( (queue[i].getAddress() == packet->getAddress()) && (queue[i].getKind() == packet->getKind()))
    {
//      Serial.print("Overwriting existing packet at index ");
//      Serial.println(i, DEC);
      memcpy(&queue[i],packet,sizeof(DCCPacket));
      //do not increment written or modify write_pos
      result = true;
      break;
    }
    i = (i+1)%size;
  }
  
  //else, tack it on to the end
  if(!isFull())
  {
    //else, just write it at the end of the queue.
    memcpy(&queue[write_pos],packet,sizeof(DCCPacket));
//    Serial.print("Write packet to index ");
//    Serial.println(write_pos, DEC);
    write_pos = (write_pos + 1) % size;
    ++written;
    result = true;
  }
//  Serial.println("Queue is full!");

  return result;
}



bool DCCPacketQueue::insertPacket(DCCPacket *packet)
{
//  Serial.print("Enqueueing a packet of kind: ");
//  Serial.println(packet->getKind(), DEC);
   //First: Overwrite any packet with the same address and kind; if no such packet THEN hitup the packet at write_pos
  if(xSemaphoreTake(semaphore, SEMAPHORE_TIMEOUT) != pdTRUE){
    Serial.println("Error taking Semaphore");
    return false;
  };
  bool result = insertPacketCtx(packet);

  xSemaphoreGive(semaphore);
  return result;
}

// void DCCPacketQueue::printQueue(void)
// {
//   byte i, j;
//   for(i = 0; i < size; ++i)
//   {
//     for(j = 0; j < (queue[i].size_repeat>>4); ++j)
//     {
//       Serial.print(queue[i].data[j],BIN);
//       Serial.print(" ");
//     }
//     if(i == read_pos) Serial.println("   r");
//     else if(i == write_pos) Serial.println("    w");
//     else Serial.println("");
//   }
// }
bool DCCPacketQueue::readPacketCtx(DCCPacket *packet)
{

  bool result = false;

  if(!isEmpty())
  {
//    Serial.print("Reading a packet from index: ");
//    Serial.println(read_pos, DEC);
    memcpy(packet,&queue[read_pos],sizeof(DCCPacket));
    read_pos = (read_pos + 1) % size;
    --written;
    result = true;
  }

  return result;
}


bool DCCPacketQueue::readPacket(DCCPacket *packet)
{
    if(xSemaphoreTake(semaphore, SEMAPHORE_TIMEOUT) != pdTRUE){
    Serial.println("Error taking Semaphore");
    return false;
  };
  bool result = readPacketCtx(packet);
  xSemaphoreGive(semaphore);
  return result;
}

bool DCCPacketQueue::forget(uint16_t address, uint8_t address_kind)
{
  if(xSemaphoreTake(semaphore, SEMAPHORE_TIMEOUT) != pdTRUE){
    Serial.println("Error taking Semaphore");
    return false;
  };

  bool found = false;
  for(int i = 0; i < size; ++i)
  {
    if( (queue[i].getAddress() == address) && (queue[i].getAddressKind() == address_kind) )
    {
      found = true;
      queue[i] = DCCPacket(); //revert to default value
    }
  }
  --written;
  xSemaphoreGive(semaphore);
  return found;
}

void DCCPacketQueue::clear(void)
{
  if(xSemaphoreTake(semaphore, SEMAPHORE_TIMEOUT) != pdTRUE){
    Serial.println("Error taking Semaphore");
    return;
  };
  read_pos = 0;
  write_pos = 0;
  written = 0;
  for(int i = 0; i<size; ++i)
  {
    queue[i] = DCCPacket();
  }
  xSemaphoreGive(semaphore);
}


/*****************************/

DCCRepeatQueue::DCCRepeatQueue(void) : DCCPacketQueue()
{
}

bool DCCRepeatQueue::insertPacketCtx(DCCPacket *packet)
{

  bool result = false;
  if(packet->getRepeat())
  {
    result = DCCPacketQueue::insertPacketCtx(packet);
  }
  return result;
}

bool DCCRepeatQueue::insertPacket(DCCPacket *packet)
{
  if(xSemaphoreTake(semaphore, SEMAPHORE_TIMEOUT) != pdTRUE){
    Serial.println("Error taking Semaphore");
    return false;
  };
  bool result = insertPacketCtx(packet);
  xSemaphoreGive(semaphore);
  return result;
}

bool DCCRepeatQueue::readPacket(DCCPacket *packet)
{
  if(xSemaphoreTake(semaphore, SEMAPHORE_TIMEOUT) != pdTRUE){
    Serial.println("Error taking Semaphore");
    return false;
  };
  bool result = false;

  if(!isEmpty())
  {
    memcpy(packet,&queue[read_pos],sizeof(DCCPacket));
    read_pos = (read_pos + 1) % size;
    --written;

    if(packet->getRepeat()) //the packet needs to be sent out at least one more time
    {     
      packet->setRepeat(packet->getRepeat()-1);

      insertPacketCtx(packet);
    }
    result = true;
  }
  xSemaphoreGive(semaphore);
  return result;
}


/**************/

DCCEmergencyQueue::DCCEmergencyQueue(void) : DCCPacketQueue()
{
}

/* Goes through each packet in the queue, repeats it getRepeat() times, and discards it */
bool DCCEmergencyQueue::readPacket(DCCPacket *packet)
{
  if(xSemaphoreTake(semaphore, SEMAPHORE_TIMEOUT) != pdTRUE){
    Serial.println("Error taking Semaphore");
    return false;
  };
  bool result = false;
  if(!isEmpty()) //anything in the queue?
  {
    queue[read_pos].setRepeat(queue[read_pos].getRepeat()-1); //decrement the current packet's repeat count
    if(queue[read_pos].getRepeat()) //if the topmost packet needs repeating
    {
      memcpy(packet,&queue[read_pos],sizeof(DCCPacket));
      result = true;
    }
    else //the topmost packet is ready to be discarded; use the DCCPacketQueue mechanism
    {
        result = DCCPacketQueue::readPacketCtx(packet);
    }
  }
  xSemaphoreGive(semaphore);
  return result;
}
#endif