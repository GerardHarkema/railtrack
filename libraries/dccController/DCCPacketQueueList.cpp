
#ifdef QUEUE_LIST_TYPE
#include "DCCPacketQueueList.h"
#include <stdexcept>

DCCPacketQueue::DCCPacketQueue(void) : size(10)
{
  return;
}

void DCCPacketQueue::setup(byte length)
{
  size = length;
  begin_pos = queue.begin();
  read_pos = begin_pos;
  semaphore = xSemaphoreCreateMutex();
}

bool DCCPacketQueue::insertPacket(DCCPacket *packet)
{
  bool result = true;
  if(xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE){
    Serial.println("Error taking Semaphore");
    return false;
  };
  std::list<DCCPacket>::iterator queue_packet;
  for (queue_packet = queue.begin(); queue_packet != queue.end(); ++queue_packet){
    if(queue_packet->getAddress() == packet->getAddress() && queue_packet->getKind() == packet->getKind() ){
      queue.erase(queue_packet);
      written--;
      break;
    }
  }
  try {
    queue.push_back(*packet);
    written++;
  }
  catch (const std::exception& e){
    queue_full = true;
    result = false;
  }
  xSemaphoreGive(semaphore);
  return result;
#if 0
//  Serial.println("Queue is full!");
  return false;
#endif
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

bool DCCPacketQueue::readPacket(DCCPacket *packet)
{
  bool result = false;
  if(xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE){
    Serial.println("Error taking Semaphore");
    return false;
  };  
  if(!isEmpty()){

    *packet = *begin_pos;
    queue.erase(begin_pos);
    written--;
    queue_full = false;
    result = true;
  }
  xSemaphoreGive(semaphore);
  return result;
}

bool DCCPacketQueue::forget(uint16_t address, uint8_t address_kind)
{

  if(xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE){
    Serial.println("Error taking Semaphore");
    return false;
  };
  bool found = false;
  std::list<DCCPacket>::iterator queue_packet;
  for (queue_packet = queue.begin(); queue_packet != queue.end(); ++queue_packet){
    if(queue_packet->getAddress() == address && queue_packet->getKind() == address_kind){
      queue.erase(queue_packet);
      written--;
      queue_full = false;
      found = true;
      break;
    }
  }
  xSemaphoreGive(semaphore);
  return found;
}

void DCCPacketQueue::clear(void)
{
  if(xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE){
    Serial.println("Error taking Semaphore");
  };
  queue.clear();
  read_pos = begin_pos; 
  written = 0;
  xSemaphoreGive(semaphore);
}


/*****************************/

DCCRepeatQueue::DCCRepeatQueue(void) : DCCPacketQueue()
{
}

bool DCCRepeatQueue::insertPacket(DCCPacket *packet)
{
  bool result = false;
    if(xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE){
    Serial.println("Error taking Semaphore");
    return false;
  };
  if(packet->getRepeat())
  {
    result = DCCPacketQueue::insertPacket(packet);
  }
  xSemaphoreGive(semaphore);
  return result;
}

bool DCCRepeatQueue::readPacket(DCCPacket *packet)
{

  bool result = false;
  if(xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE){
    Serial.println("Error taking Semaphore");
    return false;
  };

  if(!isEmpty())
  {

    *packet = *begin_pos;
    queue.erase(begin_pos);
    written--;

    if(packet->getRepeat()) //the packet needs to be sent out at least one more time
    {     
      packet->setRepeat(packet->getRepeat()-1);
      insertPacket(packet); // Hier werkt het nog niet goed!!!!
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

//* deze is nog ongetest !!!!
{
  if(!isEmpty()) //anything in the queue?
  {
    #if 0
    queue[read_pos].setRepeat(queue[read_pos].getRepeat()-1); //decrement the current packet's repeat count
    if(queue[read_pos].getRepeat()) //if the topmost packet needs repeating
    {
      memcpy(packet,&queue[read_pos],sizeof(DCCPacket));
      return true;
    }
    else //the topmost packet is ready to be discarded; use the DCCPacketQueue mechanism
    {
      return(DCCPacketQueue::readPacket(packet));
    }
    #endif
  }
  return false;
}
#endif